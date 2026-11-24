"""Fixed hold-target activation checks using the Nov 24 journal; no hardware."""

import source_paths
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from controls.controller.calibration import CalibrationMap
from controls.controller.recovery import START_TOLERANCE_DEG
from controls.controller.tracking import TrackingSettings
from controls.controller.weld_plan import WeldGeometry
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import ArmOwner, SimulatedArm
from test_weld_hardware import Clock, simple_plan


class CapturedHoldArm(SimulatedArm):
    """Keep injected feedback separate from the fixed native preload target."""

    def __init__(self, raw, preload, **kwargs):
        super().__init__(raw, **kwargs)
        self.preload = np.array(preload, dtype=float)

    def poll(self):
        self.last = self.clock()

    def hold(self, expected, pose_check):
        self.q = self.preload.copy()
        super().hold(expected, pose_check)


class HoldActivationTests(unittest.TestCase):
    def controller(self):
        clock = Clock()
        preload = [0, 0, 0, 0.2, 0, 0]
        arm = CapturedHoldArm([0] * 6, preload, clock=clock)
        control = WeldController(arm, clock=clock, pose_check=lambda *a, **k: None,
                                 tracking_settings=TrackingSettings())
        self.addCleanup(control.close)
        control.select(True, False)
        control.load_plan(simple_plan(chassis=False))
        control.go_initial()
        return clock, arm, control

    def test_recorded_enable_transient_settles_before_any_recovery_command(self):
        fixture = json.loads((Path(__file__).parent /
                              "fixtures/weld_hold_activation_20261124.json").read_text())
        plan = fixture["plan"]
        geometry = WeldGeometry(CalibrationMap(fixture["mapping"]), arm_contacts_only=False)
        clock = Clock()
        arm = CapturedHoldArm(plan["current_raw_deg"], fixture["hold_target_raw_deg"], clock=clock)
        control = WeldController(
            arm, clock=clock, pose_check=geometry.check, recovery_check=geometry.check_recovery,
            allow_optimized_preparation=True, tracking_settings=TrackingSettings(),
        )
        self.addCleanup(control.close)
        control.select(True, False)
        control.load_plan(plan)
        control.go_initial()
        self.assertEqual(control.state, "verifying_hold")
        target = arm.target.copy()
        crossed_old_start = False
        eligible_since = None
        for row in fixture["activation_feedback"]:
            clock.value = row["elapsed_s"]
            arm.q = np.array(row["raw_deg"])
            arm.velocity = np.array(row["velocity_deg_s"])
            if max(abs(arm.q - plan["current_raw_deg"])) > 0.3:
                crossed_old_start = True
            eligible = (max(abs(arm.q - plan["current_raw_deg"])) <= START_TOLERANCE_DEG
                        and max(abs(arm.velocity)) < np.degrees(0.03))
            eligible_since = ((clock() if eligible_since is None else eligible_since)
                              if eligible else None)
            control.tick()
            self.assertEqual(arm.commands, [])
            np.testing.assert_array_equal(arm.target, target)
            self.assertNotIn(control.state, ("paused", "fault"), control.message)
            if control.state == "initial":
                self.assertLessEqual(max(abs(arm.q - plan["current_raw_deg"])), START_TOLERANCE_DEG)
                self.assertIsNotNone(eligible_since)
                self.assertGreaterEqual(clock() - eligible_since, 0.2)
                break
            np.testing.assert_array_equal(control.joint_goal()["raw_deg"], target)
        self.assertTrue(crossed_old_start)
        self.assertEqual(control.state, "initial", control.message)
        self.assertLess(clock(), 1)
        reference = next(e for e in control.events if e["event"] == "hold_reference")
        np.testing.assert_array_equal(reference["target_raw_deg"], target)
        self.assertIsNotNone(control.last_observation)

    def test_real_drift_from_fixed_target_still_pauses_without_recapture(self):
        clock, arm, control = self.controller()
        original = arm.target.copy()
        for delta in (0.1, 0.3, 0.5, 0.52):
            arm.q[3] = original[3] + delta
            clock.advance()
            control.tick()
            np.testing.assert_array_equal(arm.target, original)
        self.assertEqual(control.state, "paused")
        self.assertIn("Hold activation drift A4: +0.520 deg", control.message)
        self.assertIn("limit 0.500", control.message)
        self.assertEqual(arm.commands, [])
        self.assertTrue(arm.enabled)

    def test_outside_checked_start_waits_then_times_out_without_motion(self):
        clock, arm, control = self.controller()
        arm.q[3] = 0.6  # 0.4 from hold target, but 0.6 from the checked path.
        for _ in range(105):
            clock.advance()
            control.tick()
        self.assertEqual(control.state, "paused")
        self.assertIn("Hold did not settle at the checked start", control.message)
        self.assertEqual(arm.commands, [])
        self.assertEqual(arm.target[3], 0.2)

    def test_half_degree_start_boundary_is_shared_by_load_preload_and_activation(self):
        for offset in (-0.500001, -0.5, -0.4, 0.4, 0.5, 0.500001):
            for stage in ("load_plan", "go_initial", "preload", "activation"):
                with self.subTest(offset=offset, stage=stage):
                    clock = Clock()
                    arm = CapturedHoldArm([0] * 6, [0] * 6, clock=clock)
                    control = WeldController(arm, clock=clock, pose_check=lambda *a, **k: None)
                    self.addCleanup(control.close)
                    control.select(True, False)
                    outside = abs(offset) > 0.5
                    if stage == "load_plan":
                        arm.q[3] = offset
                        if outside:
                            with self.assertRaisesRegex(ValueError, "moved since calculation"):
                                control.load_plan(simple_plan(chassis=False))
                            self.assertFalse(arm.enabled)
                            continue
                    control.load_plan(simple_plan(chassis=False))
                    if stage == "go_initial":
                        arm.q[3] = offset
                        if outside:
                            with self.assertRaisesRegex(ValueError, "Start position changed"):
                                control.go_initial()
                            self.assertFalse(arm.enabled)
                            continue
                    if stage == "preload":
                        arm.preload[3] = offset
                    control.go_initial()
                    if stage == "preload" and outside:
                        self.assertEqual(control.state, "fault")
                        self.assertFalse(arm.enabled)
                        self.assertEqual(arm.commands, [])
                        continue
                    arm.q[3] = offset
                    clock.advance()
                    control.tick()
                    if outside:
                        self.assertEqual(control.state, "paused")
                    else:
                        self.assertEqual(control.state, "verifying_hold")
                        for _ in range(12):
                            clock.advance()
                            control.tick()
                            if control.state != "verifying_hold":
                                break
                        self.assertEqual(control.state, "initial", control.message)
                    self.assertEqual(arm.commands, [])

    def test_existing_hold_reuses_target_instead_of_recapturing_measurement(self):
        clock, arm, control = self.controller()
        control.pause()
        arm.q[3] = 0.25
        control.load_plan(simple_plan(chassis=False))
        control.go_initial()
        self.assertEqual(control.hold_reference[3], 0.2)
        self.assertEqual(arm.target[3], 0.2)

    def test_owner_hold_target_copy_uses_no_transport_and_requires_all_slots(self):
        owner = ArmOwner.__new__(ArmOwner)
        owner.port = SimpleNamespace(holding=True, hold_targets={i: i / 10 for i in range(6)},
                                     poll=Mock(), cached_positions=Mock())
        expected = np.degrees([i / 10 for i in range(6)])
        copied = owner.held_target()
        np.testing.assert_array_equal(copied, expected)
        copied[:] = 100
        np.testing.assert_array_equal(owner.held_target(), expected)
        owner.port.poll.assert_not_called()
        owner.port.cached_positions.assert_not_called()
        del owner.port.hold_targets[5]
        with self.assertRaisesRegex(ValueError, "All six"):
            owner.held_target()


if __name__ == "__main__":
    unittest.main()
