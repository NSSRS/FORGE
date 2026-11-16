"""Direct measured-pose display, independent of welding IK and collision gates."""

import math


class ArmMirrorScene:
    def __init__(self):
        import mujoco as mj
        import numpy as np
        from simulation.paths import MODEL_DIR
        from simulation.arm_closure import ArmClosure
        from simulation.arm_sweep import descendant_bodies
        from motors.arm_calibration import DRIVES

        self.mj, self.np, self.drives = mj, np, DRIVES
        spec = mj.MjSpec.from_file(str(MODEL_DIR / "scene.xml"))
        wall = spec.geom("steel_wall")
        wall_face = wall.pos[1] + wall.size[1]
        spec.delete(wall)
        self.model = model = spec.compile()
        self.data = data = mj.MjData(model)
        mj.mj_resetDataKeyframe(model, data, model.key("terrain_spawn").id)
        # Display the fixed robot horizontally. No physics or weld target is used.
        data.qpos[:3] = [0, 0, data.qpos[1] - wall_face]
        mj.mju_mulQuat(data.qpos[3:7], np.full(4, .5), data.qpos[3:7].copy())
        self.closure = ArmClosure(model, data)
        self.seed = data.qpos.copy()
        self.addresses = [int(model.joint(name).qposadr[0]) for name in DRIVES]
        self.arm_bodies = descendant_bodies(model, int(model.jnt_bodyid[model.joint(DRIVES[0]).id]))
        self.wrist_bodies = descendant_bodies(model, int(model.jnt_bodyid[model.joint(DRIVES[3]).id]))
        self.rgba = model.geom_rgba.copy()
        model.vis.global_.offwidth = max(800, model.vis.global_.offwidth)
        model.vis.global_.offheight = max(500, model.vis.global_.offheight)
        self.renderer = mj.Renderer(model, width=800, height=500)
        self.option = mj.MjvOption()
        self.option.geomgroup[3:] = 0
        self.option.sitegroup[:] = 0
        self.camera = mj.MjvCamera()

    def render(self, model_deg, *, show_chassis=True, focus="Arm", azimuth=108, elevation=-20, distance=.85):
        mj, np, model, data = self.mj, self.np, self.model, self.data
        if len(model_deg) != 6 or not all(math.isfinite(q) for q in model_deg):
            raise ValueError("Six finite model angles required")
        data.qpos[:] = self.seed
        data.qpos[self.addresses] = np.radians(model_deg)
        if not self.closure.apply(data):
            raise ValueError("D3 closure failed at these mapped angles; pose image withheld")
        mj.mj_forward(model, data)
        model.geom_rgba[:] = self.rgba
        if not show_chassis:
            for i in range(model.ngeom):
                if int(model.geom_bodyid[i]) not in self.arm_bodies and model.geom(i).name != "preview_floor":
                    model.geom_rgba[i, 3] = 0
        bodies = self.arm_bodies if focus == "Arm" else self.wrist_bodies if focus == "Wrist" else range(1, model.nbody)
        positions = data.xpos[list(bodies)]
        self.camera.lookat[:] = (positions.min(axis=0) + positions.max(axis=0)) / 2
        self.camera.azimuth, self.camera.elevation, self.camera.distance = azimuth, elevation, distance
        self.renderer.update_scene(data, camera=self.camera, scene_option=self.option)
        outside = []
        for i, name in enumerate(self.drives):
            jid = model.joint(name).id
            if model.jnt_limited[jid] and not model.jnt_range[jid, 0] <= math.radians(model_deg[i]) <= model.jnt_range[jid, 1]:
                outside.append(f"A{i+1}")
        warning = "Outside authored joint range: " + ", ".join(outside) if outside else None
        return self.renderer.render().tobytes(), warning

    def close(self):
        self.renderer.close()

    def a5_straight_reference(self):
        """Find the extended wrist pose where A4 and A6 shaft axes align.

        This reference is independent of cable travel and collision stop angles.
        The opposite, folded solution (axes antiparallel) is excluded.
        """
        import hashlib
        from scipy.optimize import minimize_scalar
        from simulation.paths import MODEL_DIR
        mj, np, model, data = self.mj, self.np, self.model, self.data
        old, closure_last = data.qpos.copy(), self.closure.last.copy()
        try:
            data.qpos[:] = self.seed
            a4, a6 = [model.joint(self.drives[i]).id for i in (3, 5)]
            def residual(angle):
                data.qpos[self.addresses[4]] = np.radians(angle)
                mj.mj_forward(model, data)
                return float(np.sum((data.xaxis[a4]-data.xaxis[a6])**2))
            fit = minimize_scalar(residual, bounds=(0., 180.), method="bounded",
                                  options={"xatol": 1e-9})
            if not fit.success or residual(fit.x) > 1e-10:
                raise ValueError("Model cannot identify an extended A5 straight pose")
            return dict(kind="a5_straight_axes", model_deg=float(fit.x),
                axis_error=float(fit.fun),
                robot_xml_sha256=hashlib.sha256((MODEL_DIR/"robot.xml").read_bytes()).hexdigest(),
                basis="Extended wrist: A4 forearm roll and A6 tool roll shaft axes point in the same direction",
                physical_alignment_verified=False)
        finally:
            data.qpos[:] = old
            self.closure.last = closure_last
            mj.mj_forward(model, data)
