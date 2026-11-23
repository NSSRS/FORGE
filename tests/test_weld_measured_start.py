"""Measured hardware starts, bounded recovery and strict later paths; no hardware."""

import source_paths
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from motors.arm_calibration import CalibrationMap, RawAngleLimitError
from controls.controller.measured_start import MeasuredStart, RecoveryGeometry
from controls.controller.recovery import recovery_path
from controls.controller.weld_plan import Settings, WeldGeometry, build_plan, move_path
from controls.controller.tracking import TrackingSettings
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from test_weld_hardware import Clock, simple_plan

ROOT = Path(__file__).resolve().parents[1]


class MeasuredStartGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        document = json.loads((ROOT / 'src/robot_description/config/arm_calibration_measured.json').read_text())
        original = CalibrationMap(document)
        cls.goal = np.array(list(original.simulation_pose().values()))
        cls.start = cls.goal.copy(); cls.start[5] += 2
        # A deliberately narrow saved pose range at an otherwise clear pose.
        document['joints']['6']['raw_branch_deg'][1] = cls.goal[5] + 1
        cls.mapping = CalibrationMap(document)
        cls.geometry = WeldGeometry(cls.mapping)
        cls.policy = MeasuredStart(tuple(cls.start), cls.mapping.fingerprint)

    def test_observed_start_recovers_without_changing_saved_mapping(self):
        before = deepcopy(self.mapping.document)
        with self.assertRaises(RawAngleLimitError):
            self.geometry.check(self.start)
        self.assertEqual(self.geometry.observe(self.start), [])
        recovery = RecoveryGeometry(self.geometry, self.policy)
        path, _ = recovery_path(recovery, self.start, self.goal, Settings())
        recovery.validate_path(path, clearance_mm=2)
        self.geometry.check(path.positions[-1])
        self.assertEqual(self.mapping.document, before)
        outward = self.start.copy(); outward[5] += .01
        with self.assertRaisesRegex(ValueError, 'measured-start recovery bounds'):
            recovery.check(outward)
        with self.assertRaisesRegex(ValueError, 'fixed travel envelope'):
            self.geometry.validate_path(path)

    def test_authored_model_range_does_not_reject_measured_start(self):
        joint = self.geometry.model.joint('b7_arm_d6').id
        original_range = self.geometry.model.jnt_range[joint].copy()
        original_limited = self.geometry.model.jnt_limited[joint]
        try:
            row = self.mapping.joints['6']
            angles = np.radians([row['model_per_raw'] * q + row['model_intercept_deg']
                                 for q in (self.goal[5], self.start[5])])
            self.geometry.model.jnt_limited[joint] = True
            self.geometry.model.jnt_range[joint] = [min(angles[0]-.01, angles[0]+.01),
                                                   max(angles[0]-.01, angles[0]+.01)]
            self.geometry.observe(self.start)
            RecoveryGeometry(self.geometry, self.policy).check(self.start)
            outside = self.goal.copy(); outside[5] += .8
            with self.assertRaisesRegex(ValueError, 'authored model joint limit'):
                self.geometry.check(outside)
        finally:
            self.geometry.model.jnt_range[joint] = original_range
            self.geometry.model.jnt_limited[joint] = original_limited

    def test_start_collision_warns_but_moving_samples_still_block(self):
        contact = dict(blocked=True, basis='convex hull', distance_mm=-.1,
                       geom_names=['arm', 'chassis'])
        recovery = RecoveryGeometry(self.geometry, self.policy)
        with patch.object(self.geometry, 'inspect_contacts', return_value=[contact]):
            self.assertIn('Arm/environment clearance', self.geometry.observe(self.start)[0])
            self.assertIn('Arm/environment clearance', recovery.check(self.start)[0])
            following = self.start.copy(); following[5] -= .001
            with self.assertRaisesRegex(ValueError, 'Arm/environment clearance'):
                recovery.check(following)
            with self.assertRaisesRegex(ValueError, 'Arm/environment clearance'):
                recovery.validate_path(move_path(self.start, self.goal, Settings()))

    def test_cable_envelopes_finite_data_and_mapping_identity_remain_required(self):
        for axis in (3, 4):
            raw = self.goal.copy(); raw[axis] = self.mapping.raw_limits(axis+1)[1] + 1
            policy = MeasuredStart(tuple(raw), self.mapping.fingerprint)
            with self.assertRaisesRegex(ValueError, 'fixed cable bounds'):
                policy.check_raw(self.mapping, raw)
        with self.assertRaisesRegex(ValueError, 'different mapping'):
            MeasuredStart(tuple(self.start), 'other').bounds(self.mapping)
        for raw in ([0]*5, [float('nan')]*6):
            with self.assertRaises(ValueError):
                self.geometry.observe(raw)

    def test_feedback_keeps_existing_start_tolerance_without_expanding_commands(self):
        observed = self.start.copy(); observed[5] += .05
        self.geometry.check_recovery(observed, self.policy.document(), measured=True)
        with self.assertRaises(ValueError):
            self.geometry.check_recovery(observed, self.policy.document())
        observed[5] = self.start[5] + .31
        with self.assertRaises(ValueError):
            self.geometry.check_recovery(observed, self.policy.document(), measured=True)

    def test_a6_observed_extent_cannot_extend_fixed_270_degree_limits(self):
        for edge in self.mapping.a6_bounds:
            start = self.goal.copy(); start[5] = edge
            policy = MeasuredStart(tuple(start), self.mapping.fingerprint)
            policy.check_raw(self.mapping, start)
            # Feedback tolerance must not permit crossing a physical cable end.
            outside = start.copy(); outside[5] += -.001 if edge == self.mapping.a6_bounds[0] else .001
            for measured in (False, True):
                with self.assertRaisesRegex(ValueError, 'A6.*fixed cable'):
                    policy.check_raw(self.mapping, outside, measured=measured)
                with self.assertRaisesRegex(ValueError, 'A6.*fixed cable'):
                    MeasuredStart(tuple(outside), self.mapping.fingerprint).check_raw(
                        self.mapping, outside, measured=measured)

    def test_hardware_planner_and_full_controller_recover_then_restore_strict_paths(self):
        request = dict(mapping=self.mapping.document, current_raw_deg=self.start.tolist(),
                       initial_raw_deg=self.goal.tolist(), arm_enabled=True, chassis_enabled=False,
                       settings=asdict(Settings(cycles=1, limit_profile='weld', max_speed_deg_s=30,
                                                optimize_preparation=True)))
        with self.assertRaisesRegex(ValueError, 'saved upper limit'):
            build_plan(request)
        plan = build_plan(dict(request, measured_hardware_start=True))
        self.assertEqual(plan['measured_start'], self.policy.document())
        self.assertFalse(plan['hardware_qualified'])
        clock = Clock(); arm = SimulatedArm(self.start, clock=clock)
        control = WeldController(arm, clock=clock, pose_check=self.geometry.check,
                                 recovery_check=self.geometry.check_recovery,
                                 tracking_settings=TrackingSettings())
        control.select(True, False)
        control.load_plan(plan)
        self.assertFalse(arm.enabled)
        control.go_initial()
        try:
            for _ in range(6000):
                clock.advance(); control.tick()
                if control.state in ('ready', 'fault', 'paused'):
                    break
            self.assertEqual(control.state, 'ready', control.message)
            self.assertIsNone(control.measured_start)
            self.assertIsNone(arm.measured_start)
            self.assertEqual(self.mapping.fingerprint, plan['mapping_fingerprint'])
            np.testing.assert_allclose(arm.commands[-1], plan['approach']['raw_deg'][-1],
                                       atol=1e-10, rtol=0)
        finally:
            control.close()


class MeasuredStartControlTests(unittest.TestCase):
    def make_controller(self):
        clock = Clock(); arm = SimulatedArm([0]*6, clock=clock)
        def normal(raw, **kwargs):
            if raw[0] < .2:
                raise ValueError('saved pose limit')
        def recovery(raw, document, **kwargs):
            if raw[0] < document['raw_deg'][0] - 1e-9:
                raise ValueError('outside observed recovery extent')
        control = WeldController(arm, clock=clock, pose_check=normal,
                                 recovery_check=recovery, tracking_settings=TrackingSettings())
        plan = simple_plan(chassis=False)
        plan.update(mapping_fingerprint='test', measured_start=MeasuredStart((0,)*6, 'test').document())
        control.select(True, False)
        control.load_plan(plan)
        return clock, arm, control, plan

    def test_recovery_enters_strict_mode_before_approach_and_weld(self):
        clock, arm, control, _ = self.make_controller()
        control.go_initial()
        for _ in range(3000):
            clock.advance(); control.tick()
            if control.state in ('ready', 'fault', 'paused'):
                break
        self.assertEqual(control.state, 'ready', control.message)
        self.assertIsNone(control.measured_start)
        self.assertIsNone(arm.measured_start)
        self.assertIn('measured_start_recovery_completed', [e['event'] for e in control.events])
        with self.assertRaisesRegex(ValueError, 'saved pose limit'):
            control.check_pose([0]*6, recovery=True)
        control.start_weld()
        self.assertEqual(control.state, 'welding')

    def test_simulation_without_explicit_guard_rejects_hardware_policy(self):
        _, arm, _, plan = self.make_controller()
        control = WeldController(arm, pose_check=lambda *a, **k: None)
        control.select(True, False)
        with self.assertRaisesRegex(ValueError, 'recovery guard'):
            control.load_plan(plan)
        self.assertFalse(arm.enabled)

    def test_changed_start_and_forged_policy_never_enable(self):
        _, arm, control, plan = self.make_controller()
        arm.q[0] += .4
        with self.assertRaisesRegex(ValueError, 'Start position changed'):
            control.go_initial()
        self.assertFalse(arm.enabled)
        arm.q[0] = 0
        plan['measured_start']['raw_deg'][0] = -10
        with self.assertRaisesRegex(ValueError, 'does not match'):
            control.load_plan(plan)


class RecordedStartTests(unittest.TestCase):
    def test_nov23_start_keeps_a6_winding_and_reaches_ready_on_fake_shafts(self):
        fixture = json.loads((ROOT/'tests/fixtures/weld_start_20261123.json').read_text())
        plan = build_plan(dict(mapping=fixture['mapping'], settings=fixture['settings'],
                               current_raw_deg=fixture['current_raw_deg'],
                               initial_raw_deg=fixture['initial_raw_deg'],
                               arm_enabled=True, chassis_enabled=False, measured_hardware_start=True))
        self.assertLess(plan['recovery']['checked_edges'], fixture['reported_recovery']['checked_edges'])
        mapping = CalibrationMap(fixture['mapping'])
        from controls.controller.weld_plan import JointPath
        for stage in ('home', 'approach', 'weld'):
            low, high = JointPath.from_document(plan[stage]).position_bounds()
            self.assertGreaterEqual(low[5], mapping.a6_bounds[0])
            self.assertLessEqual(high[5], mapping.a6_bounds[1])
        self.assertAlmostEqual(plan['home']['raw_deg'][0][5], -169.92674639837315)
        self.assertGreater(plan['weld']['raw_deg'][0][5], 63)
        # The equivalent -296-degree target would cross the fixed cable end.
        self.assertLess(plan['weld']['raw_deg'][0][5]-360, mapping.a6_bounds[0])
        geometry = WeldGeometry(mapping)
        clock = Clock(); arm = SimulatedArm(fixture['current_raw_deg'], clock=clock)
        control = WeldController(arm, clock=clock, pose_check=geometry.check,
                                 recovery_check=geometry.check_recovery,
                                 allow_optimized_preparation=True,
                                 tracking_settings=TrackingSettings())
        control.select(True, False)
        control.load_plan(plan)
        control.go_initial()
        try:
            for _ in range(6000):
                clock.advance(); control.tick()
                if control.state in ('ready', 'fault', 'paused'):
                    break
            self.assertEqual(control.state, 'ready', control.message)
            self.assertIsNone(control.measured_start)
            self.assertIsNone(arm.measured_start)
        finally:
            control.close()


if __name__ == '__main__':
    unittest.main()
