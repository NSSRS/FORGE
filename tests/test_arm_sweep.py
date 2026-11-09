"""Full-model scan regressions: no hardware transport or physics stepping."""
import source_paths
import importlib.util
import math
import unittest
from unittest.mock import patch

if any(importlib.util.find_spec(name) is None for name in ('mujoco', 'numpy', 'scipy')):
    raise unittest.SkipTest('requires src/simulation/requirements.txt')

import mujoco
import numpy as np
from simulation.arm_sweep import ArmSweepStudy, DRIVES, motion_angle
from simulation.paths import MODEL_DIR


class ArmSweepTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.files = {p: p.read_bytes() for p in MODEL_DIR.glob('*.xml')}
        cls.study = ArmSweepStudy()
        with patch('mujoco.mj_step', side_effect=AssertionError('must stay kinematic')):
            cls.report = cls.study.scan()

    def setUp(self):
        self.study.reset()

    def test_reference_and_audit(self):
        report = self.report
        self.assertTrue(report['baseline']['clear'])
        self.assertGreater(report['baseline']['nearest']['distance_mm'], 3.)
        self.assertFalse(report['hardware_limits'])
        self.assertEqual(len(report['joints']), 6)
        self.assertEqual(report['audit']['explicit_exclusions'], [
            ['b2_arm_d1', 'l3_arm_linkage_d3'], ['b4_1_arm_rod_d3', 'b4_2_arm_d3']])
        self.assertTrue(report['audit']['omitted_surfaces'])
        self.assertEqual(self.study.data.time, 0)
        for path, original in self.files.items():
            self.assertEqual(path.read_bytes(), original, str(path))

    def test_first_boundary_brackets_and_search_end(self):
        s = self.study
        for result in self.report['joints']:
            for side in ('lower', 'upper'):
                with self.subTest(joint=result['joint'], side=side):
                    end = result[side]
                    s.reset()
                    # Walk the same branch before testing the refined endpoint.
                    for angle in np.linspace(result['reference_deg'], end['angle_deg'], 40):
                        self.assertTrue(s.pose(result['joint'], angle)['clear'])
                    if result['joint'] == 4:
                        self.assertEqual(end['reason'], 'search_end')
                        self.assertIsNone(end['boundary'])
                    else:
                        self.assertEqual(end['reason'], 'clearance')
                        self.assertLess(abs(end['first_blocked_deg']-end['angle_deg']), .101)
                        rejected = s.pose(result['joint'], end['first_blocked_deg'])
                        self.assertFalse(rejected['clear'])
                        self.assertLess(rejected['nearest']['distance_mm'], 2.)

    def test_playback_closes_linkage_and_freezes_other_coordinates(self):
        s = self.study
        for result in self.report['joints']:
            s.reset()
            joint = result['joint']
            others = [address for i, address in enumerate(s.addresses, 1) if i != joint]
            for phase in np.linspace(0, 1, 61):
                with self.subTest(joint=joint, phase=phase):
                    sample = s.pose(joint, motion_angle(result, phase))
                    self.assertTrue(sample['clear'])
                    self.assertLess(sample['closure_gap_m'], 1e-6)
                    np.testing.assert_array_equal(s.data.qpos[others], s.home[others])
                    np.testing.assert_array_equal(s.data.qpos[:7], s.home[:7])
                    self.assertEqual(s.data.time, 0)
                    np.testing.assert_array_equal(s.data.qvel, 0)

    def test_detects_environment_and_self_collision(self):
        s = self.study
        sample = s.pose(2, self.report['joints'][1]['reference_deg']+8)
        self.assertFalse(sample['clear'])
        self.assertIn('Steel wall', sample['nearest']['labels'])
        self.assertLess(sample['nearest']['distance_mm'], 0)
        s.reset()
        sample = s.pose(5, self.report['joints'][4]['reference_deg']-2)
        self.assertFalse(sample['clear'])
        self.assertCountEqual(sample['nearest']['labels'], ['Forearm', 'D5 wrist'])

    def test_invalid_reference_and_disabled_contacts_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Contacts disabled'):
            ArmSweepStudy(MODEL_DIR/'arm_preview.xml')
        with self.assertRaisesRegex(ValueError, 'Reference pose'):
            ArmSweepStudy(clearance_mm=5).scan()

    def test_input_validation(self):
        for options in ({'clearance_mm': 0}, {'step_deg': float('nan')},
                        {'extent_deg': 361}, {'step_deg': .01}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                ArmSweepStudy(**options)
        for joint, angle in ((0, 0), (7, 0), (1, math.inf)):
            with self.assertRaises(ValueError):
                self.study.pose(joint, angle)


if __name__ == '__main__':
    unittest.main()
