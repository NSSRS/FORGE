"""Measured feedback mapping, independent of graphics and motor hardware."""
import importlib.util
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/forge_motor_control'))
from forge_motor_control.motor_mirror import MotorMirror

CONFIG = dict(wheel_motor_ids=[1, 2, 3, 4], wheel_directions=[-1, -1, 1, 1],
              external_ratios=[1, 1, 1, 1], bench_omit_motor_id=4)


def packet(sent=10., stamp=100, positions=None, names=None, session='test'):
    return dict(version=1, session=session, stamp_ns=stamp, sent=sent,
                names=names or ['motor_1_output', 'motor_2_output', 'motor_3_output'],
                positions=positions or [1., 2., 3.], velocities=[-.5, -.5, .5])


class MirrorTests(unittest.TestCase):
    def test_relative_angle_named_mapping_and_absent_motor(self):
        mirror = MotorMirror(CONFIG)
        mirror.accept(packet(), 10.)
        self.assertEqual([s.angle for s in mirror.wheels], [0] * 4)
        mirror.accept(packet(10.1, 101, [4., 0., 1.],
                             ['motor_3_output', 'motor_1_output', 'motor_2_output']), 10.1)
        self.assertEqual([s.angle for s in mirror.wheels], [-1, -1, 1, 0])
        self.assertEqual([mirror.status(i, 10.1) for i in range(4)], ['LIVE'] * 3 + ['ABSENT'])
        self.assertEqual(mirror.status(0, 11.), 'STALE')
        self.assertEqual(mirror.wheels[0].angle, -1)  # No extrapolation after loss.

    def test_protocol_sign_and_external_ratio_without_double_cad_sign(self):
        config = dict(CONFIG, wheel_directions=[1, -1, 1, 1], external_ratios=[2, 1, 1, 1])
        mirror = MotorMirror(config)
        mirror.accept(packet(), 10)
        mirror.accept(packet(10.1, 101, [3., 4., 5.]), 10.1)
        self.assertEqual(mirror.wheels[0].angle, -1)
        self.assertEqual(mirror.wheels[0].velocity, .25)
        self.assertEqual(mirror.wheels[1].angle, 2)

    def test_invalid_stale_and_replayed_samples_do_not_refresh(self):
        mirror = MotorMirror(CONFIG)
        mirror.accept(packet(), 10)
        variants = [dict(sent=9.), dict(sent=12.), dict(stamp_ns=100),
                    dict(positions=[float('nan'), 2, 3]), dict(velocities=[1]),
                    dict(names=['motor_1_output'] * 3)]
        for changes in variants:
            bad = packet(10.1, 101); bad.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                mirror.accept(bad, 10.1)
        self.assertEqual(mirror.wheels[0].received, 10.)

    def test_missing_motor_does_not_get_refreshed_by_other_motors(self):
        mirror = MotorMirror(CONFIG)
        mirror.accept(packet(), 10)
        later = packet(11, 101)
        later.update(names=['motor_3_output'], positions=[4.], velocities=[])
        mirror.accept(later, 11)
        self.assertEqual(mirror.status(0, 11), 'STALE')
        self.assertEqual(mirror.status(2, 11), 'LIVE')
        self.assertIsNone(mirror.wheels[2].velocity)

    def test_relay_restart_rezeros_and_full_profile_restores_motor4(self):
        mirror = MotorMirror(dict(CONFIG, bench_omit_motor_id=0))
        p = packet(); p['names'].append('motor_4_output'); p['positions'].append(4.); p['velocities'].append(.5)
        mirror.accept(p, 10)
        self.assertEqual(mirror.status(3, 10), 'LIVE')
        p.update(session='restart', sent=11., stamp_ns=1, positions=[20., 20., 20., 20.])
        mirror.accept(p, 11)
        self.assertEqual([s.angle for s in mirror.wheels], [0] * 4)


@unittest.skipUnless(importlib.util.find_spec('mujoco') and
                     (ROOT / 'build/motor_viewer/model/scene.xml').exists(), 'requires viewer setup')
class MujocoProjectionTests(unittest.TestCase):
    def test_measured_joint_projection_and_frozen_base(self):
        spec = importlib.util.spec_from_file_location('viewer_test', ROOT / 'scripts/mujoco_motor_viewer.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        projection = module.Projection(ROOT / 'build/motor_viewer/model/scene.xml', CONFIG)
        initial = projection.data.qpos.copy()
        projection.mirror.accept(packet(), 10)
        projection.mirror.accept(packet(10.1, 101, [0., 1., 4.]), 10.1)
        projection.apply(10.1)
        for j, angle in zip(projection.joints, (-1, -1, 1, 0)):
            self.assertAlmostEqual(projection.data.qpos[projection.model.jnt_qposadr[j]], angle)
        base = projection.model.joint('floating_base').qposadr[0]
        self.assertTrue((projection.data.qpos[base:base+7] == initial[base:base+7]).all())
        import mujoco
        import numpy as np
        scene = mujoco.MjvScene(projection.model, maxgeom=256)
        projection.markers(scene)
        self.assertEqual(scene.ngeom, 4 * 26)
        for index, joint in enumerate(projection.joints):
            axle = projection.data.xaxis[joint]
            body = projection.model.jnt_bodyid[joint]
            rotation = projection.data.xmat[body].reshape(3, 3)
            start = index * 26
            for geom in scene.geoms[start:start+26]:
                # Every ring segment and pointer lies perpendicular to axle.
                self.assertAlmostEqual(float(np.dot(geom.mat.reshape(3, 3)[:, 2], axle)), 0, places=6)
            pointer = scene.geoms[start + 24]
            # No second application of the measured wheel angle.
            np.testing.assert_allclose(pointer.mat.reshape(3, 3)[:, 2], rotation[:, 0], atol=1e-6)
        before = projection.data.qpos.copy()
        projection.apply(11)
        self.assertTrue((projection.data.qpos == before).all())
        self.assertTrue((projection.data.qvel == 0).all())
