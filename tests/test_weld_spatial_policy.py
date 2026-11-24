"""Recorded hardware space block, advisory preview and retained motor guards."""

import source_paths
from copy import deepcopy
import json
import os
from pathlib import Path
import unittest
from unittest.mock import patch

os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np

from motors.arm_calibration import CalibrationMap
from controls.controller.measured_start import MeasuredStart, RecoveryGeometry
from controls.controller.recovery import recovery_path
from controls.controller.tracking import TrackingSettings
from controls.controller.weld_plan import Settings, WeldGeometry, build_plan
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from test_weld_hardware import Clock
from test_weld_preparation import PositionLoopArm

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = json.loads((ROOT / "tests/fixtures/weld_spatial_block_20261124.json").read_text())


class HardwareSpatialPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.request = FIXTURE["request"]
        cls.mapping = CalibrationMap(cls.request["mapping"])
        cls.start = np.array(cls.request["current_raw_deg"])
        cls.goal = np.array(cls.request["initial_raw_deg"])
        cls.settings = Settings(**cls.request["settings"])
        cls.policy = MeasuredStart(tuple(cls.start), cls.mapping.fingerprint)
        cls.geometry = WeldGeometry(cls.mapping, enforce_spatial_clearance=False)
        cls.plan = build_plan(deepcopy(cls.request))

    def test_recorded_floor_block_becomes_a_direct_recovery_without_changing_bounds(self):
        strict = WeldGeometry(self.mapping)
        with self.assertRaisesRegex(ValueError, "preview_floor.*collision_795.*0.900 mm"):
            recovery_path(RecoveryGeometry(strict, self.policy), self.start, self.goal, self.settings)
        path, report = recovery_path(RecoveryGeometry(self.geometry, self.policy),
                                     self.start, self.goal, self.settings)
        self.assertEqual(report["checked_edges"], 1)
        np.testing.assert_array_equal(path.positions[0], self.start)
        np.testing.assert_array_equal(path.positions[-1], self.goal)
        path.check_raw_bounds(self.policy.bounds(self.mapping))
        self.assertEqual(self.plan["spatial_clearance_policy"], "advisory")
        self.assertEqual(self.plan["mapping"], self.request["mapping"])
        self.assertFalse(self.plan["hardware_qualified"])

    def test_spatial_warning_covers_floor_self_contact_and_tracker_clearance(self):
        for pair in (("preview_floor", "collision_795"), ("arm", "chassis"), ("wrist", "gun")):
            contact = dict(blocked=False, warning=True, basis="convex hull",
                           distance_mm=-1.0, geom_names=list(pair))
            with patch.object(self.geometry, "inspect_contacts", return_value=[contact]):
                for clearance in (.5, 1.8, 2.0):
                    warnings = self.geometry.check(self.goal, clearance_mm=clearance)
                    self.assertIn("advisory; does not block hardware motion", warnings[0])

    def test_mechanical_stops_cable_bounds_and_linkage_still_reject(self):
        with patch.object(self.geometry.stops, "measure", return_value=iter([
            dict(geom_names=["mechanical_a", "mechanical_b"], distance_mm=-.1)
        ])):
            with self.assertRaisesRegex(ValueError, "Adjacent mechanical stop"):
                self.geometry.check(self.goal)
        for axis in (3, 4, 5):
            raw = self.goal.copy()
            raw[axis] = (self.mapping.a6_bounds[1] if axis == 5 else self.mapping.raw_limits(axis+1)[1]) + .01
            with self.assertRaises(ValueError):
                self.geometry.check_recovery(raw, self.policy.document())
        with patch.object(self.geometry.closure, "apply", return_value=False):
            with self.assertRaisesRegex(ValueError, "D3 linkage"):
                self.geometry.check(self.goal)

    def test_recorded_start_reaches_ready_and_welds_with_actual_runtime_checks(self):
        for plant in (SimulatedArm, PositionLoopArm):
            with self.subTest(plant=plant.__name__):
                clock = Clock()
                arm = plant(self.start, clock=clock)
                control = WeldController(
                    arm, clock=clock, pose_check=self.geometry.check,
                    recovery_check=self.geometry.check_recovery,
                    endpoint_check=self.geometry.check_endpoint,
                    allow_optimized_preparation=True, continuous_weld=True,
                    tracking_settings=TrackingSettings(),
                )
                try:
                    control.select(True, False)
                    control.load_plan(self.plan)
                    self.assertFalse(arm.enabled or arm.commands)
                    control.go_initial()
                    for _ in range(2000):
                        clock.advance(); control.tick()
                        if control.state in ("ready", "paused", "fault"):
                            break
                    self.assertEqual(control.state, "ready", control.message)
                    control.start_weld()
                    for _ in range(200):
                        clock.advance(); control.tick()
                        self.assertEqual(control.state, "welding", control.message)
                finally:
                    control.close()

    def test_hardware_preview_shows_warning_for_a_target_without_blocking(self):
        from simulation.weld_preview import PoseRenderer

        renderer = PoseRenderer(self.mapping.document)
        try:
            result = renderer.frame(dict(
                raw=self.start.tolist(), token=1, view="Overview", clearance_mm=2.0,
                spatial_clearance_policy="advisory",
                recovery_start=self.policy.document(), measured_pose=False,
            ))
            self.assertIn("advisory", result["issue"])
            self.assertTrue(result["contacts"])
            self.assertTrue(all(row["warning"] and not row["blocked"] for row in result["contacts"]))
            renderer.frame(dict(raw=self.goal.tolist(), token=2, view="Overview",
                                clearance_mm=.5))
            self.assertTrue(renderer.geometry.enforce_spatial_clearance)
        finally:
            renderer.close()


if __name__ == "__main__":
    unittest.main()
