#!/usr/bin/env python3
"""Mirror measured wheel angles onto the forge_sim CAD model. Never sends commands."""
import argparse
import json
import math
from pathlib import Path
import socket
import sys
import time

WORKSPACE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE / 'src/motor_control'))
from visualization.motor_mirror import MotorMirror


class Projection:
    def __init__(self, scene, config):
        import mujoco
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.mirror = MotorMirror(config)
        self.joints = [self.model.joint(f'wheel_{i}').id for i in range(1, 5)]
        if any(self.model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE for j in self.joints):
            raise ValueError('Expected four hinge wheel joints')
        # Per-wheel colors must override the shared checker material.
        for geom in range(self.model.ngeom):
            if self.model.geom_bodyid[geom] in [self.model.jnt_bodyid[j] for j in self.joints]:
                self.model.geom_matid[geom] = -1
        self.apply(time.monotonic())

    def apply(self, now):
        m, d = self.model, self.data
        # No mj_step: measured joints are authoritative; unmeasured base and
        # suspension remain at their initial pose, with no invented odometry.
        d.qpos[:] = m.qpos0
        d.qvel[:] = 0
        d.ctrl[:] = 0
        for index, (joint, state) in enumerate(zip(self.joints, self.mirror.wheels)):
            d.qpos[m.jnt_qposadr[joint]] += state.angle
            status = self.mirror.status(index, now)
            if status == 'LIVE' and state.velocity is not None:
                d.qvel[m.jnt_dofadr[joint]] = state.velocity
            color = {'LIVE': (0.3, 0.95, 0.5, 1), 'STALE': (1, 0.55, 0.1, 1),
                     'WAITING': (0.65, 0.65, 0.65, 1), 'ABSENT': (0.4, 0.4, 0.4, 1)}[status]
            for geom in range(m.ngeom):
                if m.geom_bodyid[geom] == m.jnt_bodyid[joint]:
                    m.geom_rgba[geom] = color
        self.mj.mj_forward(m, d)

    def markers(self, scene):
        # One oversized dial on the outer face of each driven wheel.
        import numpy as np

        def segment(start, end, width, color, kind=None):
            geom = scene.geoms[scene.ngeom]
            kind = kind or self.mj.mjtGeom.mjGEOM_CAPSULE
            self.mj.mjv_initGeom(geom, kind, np.zeros(3), start,
                                np.eye(3).ravel(), np.array(color))
            self.mj.mjv_connector(geom, kind, width, start, end)
            scene.ngeom += 1

        # Wheel hinge axes are local Z. Each dial lies in local XY, normal to
        # the axle, and inherit measured rotation from xmat exactly once.
        for index, joint in enumerate(self.joints):
            body = self.model.jnt_bodyid[joint]
            rotation = self.data.xmat[body].reshape(3, 3)
            origin = self.data.xpos[body]
            absent = self.mirror.ids[index] == self.mirror.omitted
            bright = (0.5, 0.5, 0.5, 1) if absent else (1., 0.9, 0.05, 1.)
            dark = (0.08, 0.1, 0.15, 1.)
            center = origin + rotation @ np.array([0, 0, -.095])
            for n in range(24):
                angles = (n * math.tau / 24, (n+1) * math.tau / 24)
                ends = [center + rotation @ np.array([.065*math.cos(t), .065*math.sin(t), 0])
                        for t in angles]
                segment(*ends, .004, bright if n % 3 == 0 else dark)
            direction = rotation[:, 0]
            segment(center, center + .058*direction, .01, bright, self.mj.mjtGeom.mjGEOM_ARROW)
            segment(center, center - .025*direction, .007, dark)

    def overlay(self, now, width=960, height=720):
        compact = width < 850 or height < 550
        tiny = width < 650 or height < 400
        labels = ['MIRROR']
        values = ['Read only' if compact else 'Measured feedback | fixed chassis']
        for i, (motor, state) in enumerate(zip(self.mirror.ids, self.mirror.wheels)):
            status = self.mirror.status(i, now)
            rpm = '--' if state.velocity is None or status != 'LIVE' else f'{state.velocity * 30 / math.pi:+.1f}'
            labels.append(f'W{i+1}' if tiny else f'W{i+1} / M{motor}')
            if compact:
                values.append(f'{status} {rpm}' + ('' if tiny else ' rpm'))
            else:
                age = '--' if state.received is None else f'{max(0, now-state.received):.2f}s'
                values.append(f'{status} | {rpm} rpm | {math.degrees(state.angle):+.0f} deg | {age}')
        if not tiny:
            labels.append('Mouse')
            values.append('Orbit / zoom' if compact else 'Orbit / zoom | yellow pointers show rotation')
        return '\n'.join(labels), '\n'.join(values)


def viewport_fit(width, height):
    """Maintain horizontal room in narrow windows without resetting user zoom."""
    return max(1.0, 0.95 * height / max(1, width))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, default=WORKSPACE / 'build/motor_viewer/model/scene.xml')
    parser.add_argument('--config', type=Path, default=WORKSPACE / 'src/bringup/config/chassis_bench.yaml')
    parser.add_argument('--width', type=int, default=960, help='Snapshot width')
    parser.add_argument('--height', type=int, default=720, help='Snapshot height')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--check', action='store_true', help='Load model and verify mapping without a window')
    parser.add_argument('--snapshot', type=Path, help='Render a headless initial-state PNG')
    args = parser.parse_args()
    if args.width < 200 or args.height < 200:
        parser.error('snapshot dimensions must be at least 200')
    if not 1024 <= args.port <= 65535:
        parser.error('port must be 1024-65535')
    import yaml
    import mujoco
    config = yaml.safe_load(args.config.read_text())['motor_driver']['ros__parameters']
    projection = Projection(args.scene, config)
    m, d = projection.model, projection.data
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.lookat[:] = d.xpos[m.body('body').id] + [0, 0, 0.25]
    camera.distance, camera.azimuth, camera.elevation = 1.8, -70, -15
    option = mujoco.MjvOption()
    mujoco.mjv_defaultOption(option)
    option.geomgroup[2], option.geomgroup[3] = 1, 0
    if args.snapshot:
        from PIL import Image
        m.vis.global_.offwidth, m.vis.global_.offheight = args.width, args.height
        camera.distance *= viewport_fit(args.width, args.height)
        with mujoco.Renderer(m, height=args.height, width=args.width) as renderer:
            renderer.update_scene(d, camera=camera, scene_option=option)
            projection.markers(renderer.scene)
            mujoco.mjr_changeFont(mujoco.mjtFontScale.mjFONTSCALE_100, renderer._mjr_context)
            pixels = renderer.render()
            # Render the same responsive telemetry used by the interactive window.
            rect = mujoco.MjrRect(0, 0, args.width, args.height)
            labels, values = projection.overlay(time.monotonic(), args.width, args.height)
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               rect, labels, values, renderer._mjr_context)
            import numpy as np
            mujoco.mjr_readPixels(pixels, None, rect, renderer._mjr_context)
            Image.fromarray(np.flipud(pixels)).save(args.snapshot)
    if args.check or args.snapshot:
        print(f'PASS: loaded {m.nbody} bodies; wheels 1-4; absent motor {projection.mirror.omitted}; no hardware opened')
        return
    import mujoco.viewer
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as listener:
        listener.bind(('127.0.0.1', args.port))
        listener.setblocking(False)
        print(f'Read-only viewer listening on 127.0.0.1:{args.port}; start motor_state_relay', flush=True)
        with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as viewer:
            with viewer.lock():
                viewer.cam.lookat[:] = camera.lookat
                viewer.cam.distance = camera.distance
                viewer.cam.azimuth, viewer.cam.elevation = camera.azimuth, camera.elevation
                viewer.opt.geomgroup[2], viewer.opt.geomgroup[3] = 1, 0
            previous_fit = 1.0
            while viewer.is_running():
                started = time.monotonic()
                for _ in range(128):
                    try:
                        payload, _ = listener.recvfrom(16384)
                    except BlockingIOError:
                        break
                    try:
                        projection.mirror.accept(json.loads(payload), time.monotonic())
                    except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
                        continue
                now = time.monotonic()
                viewport = viewer.viewport
                width, height = viewport.width, viewport.height
                fit = viewport_fit(width, height)
                with viewer.lock():
                    viewer.cam.distance *= fit / previous_fit
                    previous_fit = fit
                    projection.apply(now)
                    viewer.user_scn.ngeom = 0
                    projection.markers(viewer.user_scn)
                labels, values = projection.overlay(now, width, height)
                viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                   labels, values)])
                viewer.sync()
                time.sleep(max(0, 1/60 - (time.monotonic() - started)))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
