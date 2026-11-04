"""Full-robot integration with the existing read-only motor status overview."""
import importlib.util
from pathlib import Path
import sys
import unittest

import source_paths  # source-checkout imports; no ROS/hardware connection
from robot_description.paths import model_directory
MODEL_DIR = model_directory()

if any(importlib.util.find_spec(n) is None for n in ('mujoco', 'numpy')):
    raise unittest.SkipTest('requires src/simulation/requirements.txt')
import mujoco
import numpy as np
from visualization.mujoco_motor_viewer import Projection

ROOT = Path(__file__).resolve().parents[1]
SCENES = ('scene.xml', 'scene_curved.xml', 'scene_bumpy.xml')
CONFIG = dict(wheel_motor_ids=[7, 2, 9, 4], wheel_directions=[1, -1, 1, 1],
              external_ratios=[2, 1, 1, 1], bench_omit_motor_id=4)


def sample(sent, stamp, names, positions, velocities):
    return dict(version=1, session='model-integration', sent=sent, stamp_ns=stamp,
                names=names, positions=positions, velocities=velocities)


class MotorOverviewModelTests(unittest.TestCase):
    def test_all_full_robot_scenes_preserve_pose_and_show_mapped_status(self):
        for scene_file in SCENES:
            with self.subTest(scene=scene_file):
                p = Projection(MODEL_DIR / scene_file, CONFIG)
                m, d = p.model, p.data
                self.assertEqual((m.njnt, m.nu, m.neq), (20, 10, 1))
                spawn = m.key('terrain_spawn').id
                np.testing.assert_allclose(d.qpos, m.key_qpos[spawn])
                initial = d.qpos.copy()
                colors = m.geom_rgba.copy()
                materials = m.geom_matid.copy()
                labels, values = p.overlay(10.)
                self.assertIn('W1 / M7', labels)
                self.assertEqual(values.count('WAITING'), 3)
                self.assertIn('ABSENT', values)
                p.mirror.accept(sample(10., 1,
                    ['motor_9_output', 'motor_7_output', 'motor_2_output'],
                    [3., 1., 2.], [0., 0., 0.]), 10.)
                p.mirror.accept(sample(10.1, 2,
                    ['motor_2_output', 'motor_9_output', 'motor_7_output'],
                    [1., 4., 3.], [2., 3., 4.]), 10.1)
                p.apply(10.1)
                expected = initial.copy()
                expected[m.jnt_qposadr[p.joints]] += [-1., -1., 1., 0.]
                np.testing.assert_allclose(d.qpos, expected)
                np.testing.assert_allclose(d.qvel[m.jnt_dofadr[p.joints]], [-2., 2., 3., 0.])
                self.assertEqual(p.overlay(10.1)[1].count('LIVE'), 3)
                self.assertIn('ABSENT', p.overlay(10.1)[1])
                wheel_bodies = m.jnt_bodyid[p.joints]
                others = ~np.isin(m.geom_bodyid, wheel_bodies)
                np.testing.assert_array_equal(m.geom_rgba[others], colors[others])
                np.testing.assert_array_equal(m.geom_matid[others], materials[others])
                for i, body in enumerate(wheel_bodies):
                    for g in np.flatnonzero((m.geom_bodyid == body) & (m.geom_group == 2)):
                        np.testing.assert_allclose(m.geom_rgba[g],
                            [.3, .95, .5, 1] if i < 3 else [.4, .4, .4, 1])
                # Status graphics remain compatible with the full scene.
                graphics = mujoco.MjvScene(m, maxgeom=256)
                p.markers(graphics)
                self.assertEqual(graphics.ngeom, 104)
                p.apply(12.)
                np.testing.assert_allclose(d.qpos, expected)
                np.testing.assert_array_equal(d.qvel, 0)
                self.assertEqual(p.overlay(12.)[1].count('STALE'), 3)
                self.assertEqual(d.time, 0)  # Never integrates invented arm/base motion.
                a=m.site('closing_b4_2_arm_rod_d3_1').id
                b=m.site('closing_b4_2_arm_rod_d3_2').id
                self.assertLess(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b]), 1e-6)

    def test_four_motor_overview_including_compact_layouts(self):
        p = Projection(MODEL_DIR / 'scene.xml', dict(CONFIG, bench_omit_motor_id=0))
        p.mirror.accept(sample(10., 1,
            ['motor_4_output', 'motor_9_output', 'motor_2_output', 'motor_7_output'],
            [4., 3., 2., 1.], [1., 1., 1., 1.]), 10.)
        p.apply(10.)
        for width, height in [(960,720), (800,500), (600,360)]:
            with self.subTest(viewport=(width,height)):
                labels, values = p.overlay(10., width, height)
                self.assertIn('W4', labels)
                self.assertEqual(values.count('LIVE'), 4)
                self.assertNotIn('ABSENT', values)


if __name__ == '__main__':
    unittest.main()
