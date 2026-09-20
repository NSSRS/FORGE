"""Paused-by-default MuJoCo preview with magnetic adhesion and limited wheel velocity servos."""
from __future__ import annotations

import argparse
import os
import platform
from pathlib import Path
import time

# Avoid Mesa worker-thread crashes on large merged meshes in WSLg.
# Respect explicit user settings and leave native Linux/Windows unchanged.
if "microsoft" in platform.release().lower() and os.environ.get("GALLIUM_DRIVER") != "d3d12":
    os.environ.setdefault("LP_NUM_THREADS", "1")

import mujoco
import numpy as np
from magnetic import MagneticAttraction
from terrain_menu import TerrainMenu
from spawn_fit import reset_to_spawn
from velocity_control import set_wheel_rpm, keyboard_rpm

MODEL_DIR = Path(__file__).resolve().parent / "model"


def check_scene(scene: Path, steps: int = 500, wheel_rpm=None) -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    reset_to_spawn(model, data)
    magnets = MagneticAttraction(model)
    if wheel_rpm is not None:
        set_wheel_rpm(model, data, wheel_rpm)
    mujoco.mj_forward(model, data)
    for _ in range(steps):
        magnets.step(data)
        if not all(np.isfinite(x).all() for x in (data.qpos, data.qvel, data.qacc)):
            raise RuntimeError("Simulation produced non-finite state")
        if any(w.number for w in data.warning):
            raise RuntimeError("MuJoCo emitted a simulation warning")
    print(f"PASS: {scene.name}; bodies={model.nbody}, joints={model.njnt}, "
          f"actuators={model.nu}, steps={steps}, time={data.time:.3f}s")
    return model


def telemetry(model, data):
    """World-frame base pose and measured joint-space output-shaft telemetry."""
    base = model.body("body").id
    rot = data.xmat[base].reshape(3, 3)
    pitch = np.arcsin(np.clip(-rot[2, 0], -1, 1))
    if abs(np.cos(pitch)) > 1e-7:
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-rot[0, 1], rot[1, 1])
    angles = np.degrees([roll, pitch, yaw])
    xyz = data.xpos[base]
    labels = ["Time (s)", "World XYZ (m)", "World RPY ZYX (deg)", "Wheel: target / actual RPM / motor Nm"]
    values = [f"{data.time:.2f}", " / ".join(f"{v:+.3f}" for v in xyz),
              " / ".join(f"{v:+.1f}" for v in angles), ""]
    for i in range(1, 7):
        joint = model.joint(f"wheel_{i}").id
        rpm = data.qvel[model.jnt_dofadr[joint]] * 30 / np.pi
        if i <= 4:
            aid = model.actuator(f"wheel_{i}_velocity").id
            target = np.clip(data.ctrl[aid], *model.actuator_ctrlrange[aid]) * 30 / np.pi
            torque = data.actuator_force[aid] * model.actuator_gear[aid, 0]
            value = f"{target:+6.1f} / {rpm:+6.1f} / {torque:+6.2f}"
        else:
            value = f"passive / {rpm:+6.1f} / --"
        labels.append(f"Wheel {i}")
        values.append(value)
    return "\n".join(labels), "\n".join(values)


def contact_summary(model, data):
    pairs = {}
    for contact in data.contact:
        if contact.efc_address < 0:
            continue
        names = []
        for geom in contact.geom:
            body = int(model.geom_bodyid[geom])
            names.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
                         if body else mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom)))
        pair = tuple(sorted(str(n) for n in names))
        pairs[pair] = min(pairs.get(pair, 0.), float(contact.dist))
    ordered = sorted(pairs, key=lambda p: (not any("steel_bump" in n for n in p), pairs[p]))
    lines = [f"{a} <> {b}: {-pairs[(a,b)]*1000:.2f} mm" for a,b in ordered[:12]]
    if len(ordered) > 12:
        lines.append(f"... {len(ordered)-12} more pairs")
    return "COLLISION DEBUG | penetration depth\n" + ("\n".join(lines) or "No active contacts")


def set_collision_debug(option, enabled):
    option.geomgroup[2] = not enabled
    option.geomgroup[3] = enabled
    for flag in (mujoco.mjtVisFlag.mjVIS_CONVEXHULL,
                 mujoco.mjtVisFlag.mjVIS_CONTACTPOINT,
                 mujoco.mjtVisFlag.mjVIS_CONTACTFORCE):
        option.flags[flag] = enabled


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=MODEL_DIR / "scene.xml")
    parser.add_argument("--check", action="store_true", help="Headless load and 500-step smoke check")
    parser.add_argument("--run", action="store_true", help="Start with passive physics running")
    parser.add_argument("--seconds", type=float, help="Close viewer after this many seconds")
    parser.add_argument("--wheel-rpm", type=float, nargs=4, metavar=("W1", "W2", "W3", "W4"),
                        help="Initial joint-speed targets in RPM; clamped to +/-75. Reset returns to zero.")
    parser.add_argument("--terrain", choices=["flat", "curved", "bumpy"], default="flat")
    parser.add_argument("--arc-length", type=float, default=10)
    parser.add_argument("--arc-angle", type=float, default=60)
    parser.add_argument("--plate-width", type=float, default=4)
    parser.add_argument("--bump-diameter", type=float, default=.10)
    parser.add_argument("--bump-height", type=float, default=.005)
    parser.add_argument("--bump-spacing", type=float, default=.8)
    parser.add_argument("--bump-track-spacing", type=float, default=.737)
    parser.add_argument("--tilt", type=float, default=0, help="0=vertical, 90=underside")
    parser.add_argument("--inward", action="store_true", help="Use concave instead of convex curvature")
    parser.add_argument("--menu", action="store_true", help="Open terrain settings at startup")
    parser.add_argument("--collision-debug", action="store_true", help="Show collision hulls and contacts")
    args = parser.parse_args()
    if args.terrain != "flat":
        from terrain import generate
        args.scene = generate(args.terrain, args.arc_length, args.arc_angle, args.plate_width,
                              args.bump_diameter, args.bump_height, args.tilt, args.inward,
                              bump_spacing=args.bump_spacing, bump_track_spacing=args.bump_track_spacing)
    if args.wheel_rpm is not None and not np.isfinite(args.wheel_rpm).all():
        parser.error("Wheel RPM values must be finite")
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.check:
        check_scene(args.scene.resolve(), wheel_rpm=args.wheel_rpm)
        return

    import glfw

    model = mujoco.MjModel.from_xml_path(str(args.scene.resolve()))
    data = mujoco.MjData(model)
    reset_to_spawn(model, data)
    magnets = MagneticAttraction(model)
    mujoco.mj_forward(model, data)
    target_rpm = np.zeros(4) if args.wheel_rpm is None else np.array(args.wheel_rpm, dtype=float)
    running = bool(args.run)
    if not glfw.init():
        raise RuntimeError("GLFW initialization failed; check WSLg DISPLAY")
    window = glfw.create_window(1280, 900, "FORGE | WASD drive | X stop | Space pause | R reset", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("Could not create OpenGL preview window")
    glfw.make_context_current(window)
    glfw.swap_interval(1)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.lookat[:] = np.mean(data.xpos[1:], axis=0)
    camera.distance = 1.65
    camera.azimuth = 270
    camera.elevation = -12
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, "terrain_camera")
    if tid >= 0:
        settings = model.numeric("terrain_camera").data
        camera.azimuth, camera.elevation = settings[3:5]
    option = mujoco.MjvOption()
    collision_debug = args.collision_debug
    set_collision_debug(option, collision_debug)
    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    mouse = [0.0, 0.0]
    menu = TerrainMenu(preset=args.terrain, length=args.arc_length, angle=args.arc_angle,
                       width=args.plate_width, bump_diameter=args.bump_diameter,
                       bump_height=args.bump_height, tilt=args.tilt, inward=args.inward,
                       bump_spacing=args.bump_spacing, bump_track_spacing=args.bump_track_spacing)

    menu.visible = args.menu
    if menu.visible:
        running = False
        target_rpm[:] = 0

    def apply_terrain():
        nonlocal model, data, magnets, scene, context, running, accumulator, previous
        from terrain import generate
        try:
            path = MODEL_DIR / "scene.xml" if menu.values["preset"] == "flat" else generate(**menu.values)
            new_model = mujoco.MjModel.from_xml_path(str(path))
            new_data = mujoco.MjData(new_model)
            reset_to_spawn(new_model, new_data)
            new_magnets = MagneticAttraction(new_model)
            mujoco.mj_forward(new_model, new_data)
            new_scene = mujoco.MjvScene(new_model, maxgeom=10000)
            new_context = mujoco.MjrContext(new_model, mujoco.mjtFontScale.mjFONTSCALE_150)
        except Exception as error:
            menu.message = "Failed: " + str(error)
            print(menu.message, flush=True)
            return
        context.free()
        model, data, magnets = new_model, new_data, new_magnets
        scene, context = new_scene, new_context
        target_rpm[:] = 0
        running = False
        accumulator = 0
        previous = time.monotonic()
        mujoco.mjv_defaultFreeCamera(model, camera)
        camera.lookat[:] = np.mean(data.xpos[1:], axis=0)
        camera.distance, camera.azimuth, camera.elevation = 1.65, 270, -12
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, "terrain_camera") >= 0:
            camera.azimuth, camera.elevation = model.numeric("terrain_camera").data[3:5]
        menu.visible = False
        menu.message = "Applied. Edit, then Apply to reset again."
        print("Terrain applied in current window; paused, RPM reset to zero.", flush=True)

    def on_key(window, key, scancode, action, mods):
        nonlocal running, collision_debug
        if action != glfw.PRESS:
            return
        if key == glfw.KEY_M:
            menu.visible = not menu.visible
            running = False
            target_rpm[:] = 0
            set_wheel_rpm(model, data, target_rpm)
            return
        if menu.visible:
            if key == glfw.KEY_ESCAPE: menu.visible = False
            elif key == glfw.KEY_UP: menu.selected = (menu.selected-1)%len(menu.fields)
            elif key == glfw.KEY_DOWN: menu.selected = (menu.selected+1)%len(menu.fields)
            elif key == glfw.KEY_LEFT: menu.adjust(-1)
            elif key == glfw.KEY_RIGHT: menu.adjust(1)
            elif key in (glfw.KEY_ENTER,glfw.KEY_KP_ENTER): menu.pending = True
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)
        elif key == glfw.KEY_C:
            collision_debug = not collision_debug
            set_collision_debug(option, collision_debug)
        elif key == glfw.KEY_SPACE:
            running = not running
            if not running:
                target_rpm[:] = 0
        elif key == glfw.KEY_R:
            running = False
            target_rpm[:] = 0
            reset_to_spawn(model, data)
            mujoco.mj_forward(model, data)
            magnets.compute(data)
        elif 0 <= key < 128:
            command = keyboard_rpm(chr(key))
            if command is not None:
                target_rpm[:] = command
        set_wheel_rpm(model, data, target_rpm)

    def on_button(window, button, action, mods):
        mouse[:] = glfw.get_cursor_pos(window)
        if menu.visible and button == glfw.MOUSE_BUTTON_LEFT and action == glfw.PRESS:
            ww,wh = glfw.get_window_size(window)
            fw,fh = glfw.get_framebuffer_size(window)
            if ww and wh:
                menu.click(mouse[0]*fw/ww, fh-mouse[1]*fh/wh, fw, fh)

    def on_move(window, x, y):
        if menu.visible: return
        dx, dy = x - mouse[0], y - mouse[1]
        mouse[:] = [x, y]
        left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        if not (left or right):
            return
        height = max(1, glfw.get_window_size(window)[1])
        action = mujoco.mjtMouse.mjMOUSE_MOVE_V if right else mujoco.mjtMouse.mjMOUSE_ROTATE_V
        mujoco.mjv_moveCamera(model, action, dx / height, dy / height, camera)

    def on_scroll(window, x, y):
        if menu.visible: return
        mujoco.mjv_moveCamera(model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, -0.05*y, camera)

    glfw.set_key_callback(window, on_key)
    glfw.set_mouse_button_callback(window, on_button)
    glfw.set_cursor_pos_callback(window, on_move)
    glfw.set_scroll_callback(window, on_scroll)
    print("W/S forward/reverse; A/D turn; X stop; Space run/pause; R reset. Commands latch.", flush=True)
    print("Mouse: left drag orbit, right drag pan, scroll zoom. Native viewer shortcuts disabled.", flush=True)
    start = previous = time.monotonic()
    accumulator = 0.0
    try:
        while not glfw.window_should_close(window):
            now = time.monotonic()
            if args.seconds is not None and now - start >= args.seconds:
                break
            elapsed = min(now - previous, 0.1)
            previous = now
            glfw.poll_events()
            if menu.pending:
                menu.pending = False
                apply_terrain()
                elapsed = 0.0
            set_wheel_rpm(model, data, target_rpm)
            if running:
                accumulator += elapsed
                while accumulator >= model.opt.timestep:
                    magnets.step(data)
                    accumulator -= model.opt.timestep
            else:
                accumulator = 0
            mujoco.mj_forward(model, data)
            width, height = glfw.get_framebuffer_size(window)
            if width and height:
                viewport = mujoco.MjrRect(0, 0, width, height)
                mujoco.mjv_updateScene(model, data, option, None, camera,
                                      mujoco.mjtCatBit.mjCAT_ALL, scene)
                mujoco.mjr_render(viewport, scene, context)
                status = "RUNNING" if running else "PAUSED"
                mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                   viewport, "W/S drive | A/D turn | X stop | Space pause | R reset | M terrain | C collision",
                                   f"{status}  RPM: {target_rpm.round(1).tolist()}", context)
                names, values = telemetry(model, data)
                mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                                   viewport, names, values, context)
                if collision_debug:
                    mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                                       viewport, contact_summary(model, data), "", context)
                if menu.visible:
                    menu.draw(width, height, context)
                glfw.swap_buffers(window)
            else:
                time.sleep(.01)
    finally:
        context.free()
        glfw.destroy_window(window)
        glfw.terminate()


if __name__ == "__main__":
    main()
