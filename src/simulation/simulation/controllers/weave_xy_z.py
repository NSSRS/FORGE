"""Shape 1 dry run on a horizontal seam with a freely rolling FORGE chassis."""
import argparse
import csv
from contextlib import nullcontext
from itertools import cycle
from pathlib import Path
import sys
import time

import mujoco as mj
import numpy as np
from scipy.optimize import least_squares

from simulation.arm_closure import ArmClosure
from simulation.arm_sweep import descendant_bodies
from simulation.controllers.weave_viewer import weave_viewer
from simulation.paths import MODEL_DIR
from simulation.preview_sim import ARM_DRIVES
from simulation.velocity_control import DRIVEN_JOINTS, WHEEL_DIRECTIONS


# Dimensionless phase and transverse displacement, traced from Shape 1.
PHASE = np.array([0, .10, .30, .40, .50, .60, .80, .90, 1])
LEVEL = np.array([0, 1, 1, 0, 0, -1, -1, 0, 0])
WIRE_TIP = (0, -.00054214, -.18834214)  # distal wire face in the D6 CAD frame, m
WIRE_AXIS = (0, -1, -1)  # nozzle-to-tip direction of the APW503 wire in the D6 frame
WIRE_GAP = .002  # wire tip above the actual wheel-contact floor, m
SEAM_XY = (-.440, 0.)  # DOF1-zero centerline, m
SEAM_WIDTH = .020  # edge to edge, m
TARGET_WIRE = np.array([-1., 0., -1.]) / np.sqrt(2)  # away from mount, down 45°
POSE_GAIN = 260.  # resolved-rate pose feedback, per second
WHEEL_RADIUS = .040  # nominal rolling radius, m


def reference(t, origin, hz, amplitude, speed):
    phase = (t * hz) % 1
    i = np.searchsorted(PHASE, phase, side="right") - 1
    slope = (LEVEL[i + 1] - LEVEL[i]) / (PHASE[i + 1] - PHASE[i])
    position = origin + [speed * t, amplitude * np.interp(phase, PHASE, LEVEL), 0]
    return position, np.array([speed, amplitude * hz * slope, 0])


def floor_scene(seam_length=.072, gun_mount="reversed"):
    spec = mj.MjSpec.from_file(str(MODEL_DIR / "scene.xml"))
    if sys.platform == "win32":
        # Python opens the Unicode CAD filenames on NTFS and WSL shares.
        names = {Path(mesh.file).name for mesh in spec.meshes if mesh.file}
        spec.assets.update({name: (MODEL_DIR / spec.meshdir / name).read_bytes() for name in names})
    wall = spec.geom("steel_wall")
    wall_face = wall.pos[1] + wall.size[1]
    spec.delete(wall)
    x, y = SEAM_XY
    spec.worldbody.add_site(name="floor_seam", type=mj.mjtGeom.mjGEOM_BOX,
        pos=[x + seam_length/2, y, 0], size=[seam_length/2, SEAM_WIDTH/2, .00003],
        rgba=[.70, .47, .22, 1])
    tool = spec.body("l6_arm_linkage_d6")
    tip = tool.add_site(
        name="wire_tip", pos=WIRE_TIP, zaxis=WIRE_AXIS, size=[.001, 0, 0], rgba=[0, .9, .6, 1])
    if gun_mount == "reversed":
        # Half-turn about the APW503 barrel centerline at local Y=111 mm.
        frame = tool.add_frame(pos=[0, .222, 0], quat=[0, 0, 0, 1])
        tip.set_frame(frame)
        spec.delete(spec.geom("body_20_l6_arm_linkage_d6"))
        for geom in tool.geoms:
            geom.group = 2  # render the same CAD parts used for contact
            if "apw503" in geom.meshname:
                geom.set_frame(frame)
    elif gun_mount != "cad":
        raise ValueError("gun mount must be cad or reversed")
    # Resolve the rigid rod over two physics steps to limit numerical stretch under gravity.
    rod_constraint = next(eq for eq in spec.equalities if eq.name1 == "closing_b4_2_arm_rod_d3_1")
    spec.option.timestep = .001
    rod_constraint.solref[0] = 2 * spec.option.timestep
    model = spec.compile()
    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(model, data, model.key("terrain_spawn").id)
    # Rotate CAD forward/left/outward (+Z/+X/+Y) onto world +X/+Y/+Z.
    height = data.qpos[1] - wall_face
    data.qpos[:3] = [-.020 if gun_mount == "reversed" else .020, 0, height]
    mj.mju_mulQuat(data.qpos[3:7], np.full(4, .5), data.qpos[3:7].copy())
    closure = ArmClosure(model, data)
    pose = [0, 20, 11, 0, -9, 0] if gun_mount == "reversed" else [0, 20, -18, 0, 78, 0]
    for name, angle in zip(ARM_DRIVES, np.deg2rad(pose)):
        data.joint(name).qpos[0] = data.actuator(name + "_target").ctrl[0] = angle
    if not closure.apply(data):
        raise RuntimeError("Centered approach pose cannot close the D3 linkage")
    mj.mj_forward(model, data)
    bodies = descendant_bodies(model, model.body(ARM_DRIVES[0]).id)
    for _ in range(round(1 / model.opt.timestep)):
        mj.mj_step(model, data)
        check_arm_contacts(model, data, bodies)
    mj.mj_forward(model, data)
    return model, data


def check_arm_contacts(model, data, arm_bodies):
    for contact in data.contact:
        if contact.dist < 0 and any(b in arm_bodies for b in model.geom_bodyid[contact.geom]):
            a, b = (model.geom(int(g)).name for g in contact.geom)
            raise RuntimeError(f"Arm contact between {a} and {b}")


def centered_start(model, data, origin, target_wire=TARGET_WIRE):
    """Fit the CAD wire to the start of the pass with the authored joint limits."""
    fit = mj.MjData(model)
    mj.mj_copyData(fit, model, data)
    closure = ArmClosure(model, fit)
    arm = [model.actuator(j + "_target").id for j in ARM_DRIVES[1:]]
    driven = model.jnt_qposadr[model.actuator_trnid[arm, 0]]
    coordinates = np.r_[driven, model.jnt_qposadr[closure.bearing]]
    bearing_range = model.jnt_range[closure.bearing] if model.jnt_limited[closure.bearing] else [-np.pi, np.pi]
    bounds = np.vstack([model.actuator_ctrlrange[arm], bearing_range]).T
    tip = fit.site("wire_tip")
    wrist = fit.body("l6_arm_linkage_d6")
    rod_length = np.linalg.norm(data.site_xpos[closure.sites[1]] - data.xanchor[closure.rod])
    fit.joint(ARM_DRIVES[0]).qpos[0] = 0

    def residual(q):
        fit.qpos[coordinates] = q
        mj.mj_kinematics(model, fit)
        rod = fit.site_xpos[closure.sites[0]] - fit.xanchor[closure.rod]
        wire = tip.xmat.reshape(3, 3)[:, 2]
        return np.r_[tip.xpos - origin, .12 * (wire - target_wire),
                     .03 * (wrist.xpos[1] - origin[1]), np.linalg.norm(rod) - rod_length]

    fit.qpos[coordinates] = least_squares(residual, fit.qpos[coordinates], bounds=bounds, gtol=1e-13).x
    if not closure.apply(fit):
        return data, "Centered pose did not close the D3 rod"
    fit.ctrl[arm] = fit.qpos[driven]
    mj.mj_forward(model, fit)
    arm_bodies = descendant_bodies(model, model.body(ARM_DRIVES[0]).id)
    try:
        check_arm_contacts(model, fit, arm_bodies)
    except RuntimeError as error:
        return data, str(error)
    wire = tip.xmat.reshape(3, 3)[:, 2]
    error_mm = 1000 * np.linalg.norm(tip.xpos - origin)
    angle_error = np.rad2deg(np.arccos(np.clip(wire @ target_wire, -1., 1.)))
    if error_mm > .6 or angle_error > .2 or abs(wrist.xpos[1] - origin[1]) > .002:
        pitch = np.rad2deg(np.arctan2(-wire[2], np.linalg.norm(wire[:2])))
        d5 = np.rad2deg(fit.joint(ARM_DRIVES[4]).qpos[0])
        return fit, (f"Centered start blocked: wire {pitch:.1f}° to floor "
                     f"(target {np.rad2deg(np.arcsin(-target_wire[2])):g}°), "
                     f"DOF5 {d5:+.1f}°, tip error {error_mm:.2f} mm")
    return fit, None


class ArmTracker:
    def __init__(self, model, target_wire):
        self.target_wire = target_wire
        self.tip_jacobian = np.empty((6, model.nv))
        self.rod_jacobian = np.empty((3, model.nv))
        self.bearing_jacobian = np.empty((3, model.nv))
        self.tip = model.site("wire_tip").id
        self.arm = np.array([model.actuator(j + "_target").id for j in ARM_DRIVES[1:]])
        joints = model.actuator_trnid[self.arm, 0]
        self.qpos = model.jnt_qposadr[joints]
        self.dofs = model.jnt_dofadr[joints]
        self.sites = [model.site(f"closing_b4_2_arm_rod_d3_{i}").id for i in (1, 2)]
        rod = model.joint("b4_1_arm_rod_d3").dofadr[0]
        bearing = model.joint("b4_2_arm_d3").dofadr[0]
        self.passive = [rod, rod + 1, rod + 2, bearing]
        self.mount = model.body(ARM_DRIVES[0]).id
        self.kp = model.actuator_gainprm[self.arm, 0]
        self.rate_lead = -model.actuator_biasprm[self.arm, 2] / self.kp

    def approach(self, model, data, origin, arm_bodies):
        """Acquire the centered wire pose before the cart starts feeding."""
        start_tip = data.site_xpos[self.tip].copy()
        travel = origin - start_tip
        for i in range(round(1 / model.opt.timestep)):
            mj.mj_step1(model, data)
            check_arm_contacts(model, data, arm_bodies)
            u = min(1., i * model.opt.timestep / .6)
            self.command(model, data, start_tip + travel * u**2 * (3 - 2 * u), travel * (6 * u * (1 - u) / .6))
            mj.mj_step2(model, data)
        mj.mj_forward(model, data)
        measured_wire = data.site_xmat[self.tip].reshape(3, 3)[:, 2]
        if (measured_wire @ self.target_wire < np.cos(np.deg2rad(.2))
                or np.linalg.norm(data.site_xpos[self.tip] - origin) > .0006):
            raise RuntimeError("Wire did not reach the requested outward floor pose")

    def command(self, model, data, target, velocity):
        jac = self.tip_jacobian
        mj.mj_jacSite(model, data, jac[:3], jac[3:], self.tip)
        a, b = self.rod_jacobian, self.bearing_jacobian
        mj.mj_jacSite(model, data, a, None, self.sites[0])
        mj.mj_jacSite(model, data, b, None, self.sites[1])
        closure = a - b
        # Passive rod and bearing velocities keep the two D3 closure sites coincident.
        coupling = np.linalg.lstsq(closure[:, self.passive], -closure[:, self.dofs], rcond=1e-6)[0]
        arm_jac = jac[:, self.dofs] + jac[:, self.passive] @ coupling
        wire = data.site_xmat[self.tip].reshape(3, 3)[:, 2]
        angle_error = np.cross(wire, self.target_wire)
        twist = np.r_[velocity + POSE_GAIN * (target - data.site_xpos[self.tip]), POSE_GAIN * angle_error]
        twist -= jac[:, :6] @ data.qvel[:6]  # subtract measured floating-base motion
        # Control pitch and heading. Roll about the wire is free, and D1 holds zero.
        tangent = np.eye(3) - np.outer(wire, wire)
        arm_jac[3:] = tangent @ arm_jac[3:]
        twist[3:] = tangent @ twist[3:]
        arm_jac[3:] *= .1
        twist[3:] *= .1  # metres per radian for the damped pose solve
        rates = arm_jac.T @ np.linalg.solve(arm_jac @ arm_jac.T + .008**2 * np.eye(6), twist)
        rates += 2.8 * (rates - data.qvel[self.dofs])
        # Map the gravity and velocity loads of the passive linkage to its drives.
        load = data.qfrc_bias[self.dofs] + coupling.T @ data.qfrc_bias[self.passive]
        command = data.qpos[self.qpos] + load / self.kp + self.rate_lead * rates
        data.ctrl[self.arm] = np.clip(command, *model.actuator_ctrlrange[self.arm].T)


def line(scene, a, b, color, width):
    if scene.ngeom == scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_LINE, np.zeros(3), a, np.eye(3).ravel(), color)
    mj.mjv_connector(geom, mj.mjtGeom.mjGEOM_LINE, width, a, b)
    scene.ngeom += 1


def draw_traces(scene, rows, hz, dt):
    """Show the last two cycles in world coordinates on top of the robot scene."""
    stride = max(1, round(.008 / dt))
    points = np.asarray(rows[-round(2 / hz / dt) - 1::stride])
    for col, color, width in ((1, [1, .55, .05, 1], 5), (4, [0, .9, .8, 1], 2)):
        for a, b in zip(points[:-1, col:col + 3], points[1:, col:col + 3]):
            line(scene, a, b, color, width)


def preview_plan(hz, amplitude, speed, cycles, show=False, loop=False, gun_mount="reversed", target_wire=TARGET_WIRE):
    """Show the commanded path with the CAD arm held at the centered fit."""
    duration = cycles / hz
    model, data = floor_scene(speed * duration, gun_mount)
    origin = np.array([*SEAM_XY, data.geom("preview_floor").xpos[2] + WIRE_GAP])
    data, blocked = centered_start(model, data, origin, target_wire)
    print(blocked or "Centered start is reachable.", flush=True)
    print(f"Target only: {SEAM_WIDTH*1000:g} mm seam, ±{amplitude*1000:g} mm at {hz:g} Hz. "
          "Arm and wheels held stationary.", flush=True)
    if not (show or loop):
        return
    times = np.r_[((np.arange(cycles)[:, None] + PHASE[:-1]) / hz).ravel(), duration]
    vertices = [reference(t, origin, hz, amplitude, speed)[0] for t in times]
    with weave_viewer(model, [0, SEAM_XY[1], .10], 1.55, 90, -89) as render:
        start = time.monotonic()
        while True:
            elapsed = time.monotonic() - start
            t = elapsed % duration if loop else min(elapsed, duration)
            point, _ = reference(t, origin, hz, amplitude, speed)

            def draw(scene):
                line(scene, point - .040 * target_wire, point, [0., 1., .8, 1.], 4)
                for a, b in zip(vertices, vertices[1:]):
                    line(scene, a, b, [1., .78, .10, 1.], 2)

            if not render(data, draw, "TARGET ONLY", (blocked or "Centered start reachable").replace("°", " deg")):
                break
            if not loop and elapsed >= duration:
                break
            time.sleep(.02)


def run(hz, amplitude, speed, cycles, show=False, loop=False, gun_mount="reversed", target_wire=TARGET_WIRE):
    model, data = floor_scene(speed * cycles / hz, gun_mount)
    base = model.body("body").id
    wheels = [model.actuator(j + "_velocity").id for j in DRIVEN_JOINTS]
    wheel_dofs = model.jnt_dofadr[model.actuator_trnid[wheels, 0]]
    origin = np.array([*SEAM_XY, data.geom("preview_floor").xpos[2] + WIRE_GAP])
    _, blocked = centered_start(model, data, origin, target_wire)
    if blocked:
        raise RuntimeError(blocked + ". Wheel feed stopped.")
    arm_bodies = descendant_bodies(model, model.body(ARM_DRIVES[0]).id)
    tracker = ArmTracker(model, target_wire)
    tracker.approach(model, data, origin, arm_bodies)
    base_x = data.xpos[base, 0]
    lookat = (origin + data.xpos[tracker.mount]) / 2
    window = weave_viewer(model, lookat, 1.45, 105, -35) if show or loop else nullcontext()
    initial = mj.MjData(model) if loop else None
    if loop:
        mj.mj_copyData(initial, model, data)
    completed, rows = [], []
    dt = model.opt.timestep
    with window as render:
        if render:
            print("MuJoCo window open." + (" Repeating until you close it." if loop else ""), flush=True)
        start = time.monotonic()
        steps = round(cycles / hz / dt)
        frames = range(steps + 1)
        for step in cycle(frames) if loop else frames:
            t = step * dt
            mj.mj_step1(model, data)
            check_arm_contacts(model, data, arm_bodies)
            target, velocity = reference(t, origin, hz, amplitude, speed)
            tip = data.site_xpos[tracker.tip]
            cart = data.xpos[base]
            tracker.command(model, data, target, velocity)
            wire = data.site_xmat[tracker.tip].reshape(3, 3)[:, 2]
            cosine = target_wire @ wire
            angle = np.degrees(np.arccos(np.clip(cosine, -1, 1)))
            a, b = data.site_xpos[tracker.sites]
            tilt = np.degrees(np.arctan2(-wire[2], np.linalg.norm(wire[:2])))
            outward = -wire[0]
            rows.append([t, *target, *tip, *cart, *data.qvel[wheel_dofs], angle, np.linalg.norm(a - b),
                         tilt, outward, np.rad2deg(data.joint(ARM_DRIVES[0]).qpos[0]),
                         np.rad2deg(data.qpos[tracker.qpos[-1]])])
            if step == steps:
                if loop:
                    mj.mj_copyData(data, model, initial)
                    completed, rows = rows, []
                    start = time.monotonic()
                    continue
                break
            # The cart carries the longitudinal feed. The arm corrects residual slip.
            feed = speed + 3 * (base_x + speed * t - cart[0])
            data.ctrl[wheels] = np.clip(np.array(WHEEL_DIRECTIONS) * feed / WHEEL_RADIUS,
                                        *model.actuator_ctrlrange[wheels].T)
            mj.mj_step2(model, data)
            if any(w.number for w in data.warning) or not np.isfinite(data.qpos).all():
                raise RuntimeError(f"Unstable simulation at {t:.3f} s")
            if render and step % 20 == 0:
                sample = rows[-1]
                labels = f"Gun mount: {gun_mount}   |   {hz:g} Hz\nCart travel\nWire tip Y\nDOF6\nWire to floor"
                values = (f"{amplitude*2000:g} mm weave\n{(sample[7] - base_x) * 1000:.1f} mm\n"
                          f"{sample[5] * 1000:+.1f} mm\n{sample[19]:+.1f} deg\n{tilt:.2f} deg")
                if not render(data, lambda scene: draw_traces(scene, rows, hz, dt), labels, values):
                    break
                time.sleep(max(0, start + t - time.monotonic()))
    return np.array(completed or rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hz", type=float, default=2)
    parser.add_argument("--amplitude-mm", type=float, default=1000 * SEAM_WIDTH/2, help="center to edge, 0 for straight")
    parser.add_argument("--speed-mm-s", type=float, default=12, help="seam centerline feed")
    parser.add_argument("--cycles", type=int, default=6)
    parser.add_argument("--view", action="store_true", help="show one pass in the MuJoCo window")
    parser.add_argument("--loop", action="store_true", help="open the viewer and repeat until closed")
    parser.add_argument("--plan", action="store_true", help="inspect requested path with the CAD arm stationary")
    parser.add_argument("--gun-mount", choices=("reversed", "cad"), default="reversed",
                        help="clock the gun 180 degrees in its clamp, or use the original CAD assembly")
    parser.add_argument("--work-angle-deg", type=float, default=45, help="outward wire inclination to the floor")
    parser.add_argument("--csv", help="write world-frame targets, measured tip, base and wheel rates")
    args = parser.parse_args()
    if not (1 <= args.hz <= 3 and 0 <= args.amplitude_mm <= 1000 * SEAM_WIDTH/2
            and 0 < args.speed_mm_s < np.inf and args.cycles > 0):
        parser.error("use 1–3 Hz, amplitude 0–10 mm, positive finite feed, and at least one cycle")
    if not 0 < args.work_angle_deg <= 90:
        parser.error("work angle must be greater than zero and at most 90 degrees")
    pitch = np.deg2rad(args.work_angle_deg)
    wire = np.array([-np.cos(pitch), 0., -np.sin(pitch)])
    print(f"Simulated gun mount: {args.gun_mount} | requested wire angle: {args.work_angle_deg:g} deg", flush=True)
    options = (args.hz, args.amplitude_mm / 1000, args.speed_mm_s / 1000, args.cycles,
               args.view, args.loop, args.gun_mount, wire)
    if args.plan:
        if args.csv:
            parser.error("--csv records measured motion and cannot be used with --plan")
        preview_plan(*options)
        return
    try:
        rows = run(*options)
    except RuntimeError as error:
        parser.exit(1, f"{error}\n")
    if args.csv:
        with open(args.csv, "w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", *(f"{p}_{axis}_m" for p in ("ref", "tip", "base") for axis in "xyz"),
                             *(f"{j}_rad_s" for j in DRIVEN_JOINTS), "torch_angle_error_deg", "d3_gap_m",
                             "wire_to_surface_deg", "wire_outward_component", "dof1_deg", "dof6_deg"])
            writer.writerows(rows)
    error = np.linalg.norm(rows[:, 4:7] - rows[:, 1:4], axis=1) * 1000
    print(f"{args.hz:g} Hz | {rows[-1, 0]:.3f} s | cart travel {(rows[-1, 7]-rows[0, 7])*1000:.1f} mm"
          f" | TCP error RMS {np.sqrt(np.mean(error**2)):.3f} mm, peak {max(error):.3f} mm"
          f" | wire to surface {rows[:, 16].min():.2f} to {rows[:, 16].max():.2f} deg")


if __name__ == "__main__":
    main()
