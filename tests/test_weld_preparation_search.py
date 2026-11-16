"""Offline staging timing and transport isolation; no motor hardware."""

import source_paths
from dataclasses import asdict
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
from controls.controller.preparation_search import fit_timing
from controls.controller.weld_plan import (
    Settings,
    build_plan,
    move_path,
    JointPath,
    WeldGeometry,
)
from controls.hardware.weld_app import main
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from motors.arm_calibration import CalibrationMap
from test_weld_hardware import Clock, simple_plan

ROOT = Path(__file__).resolve().parents[1]


class TimingTests(unittest.TestCase):
    def test_exact_retiming_keeps_endpoint_and_acceleration_reserve(self):
        path = move_path(np.zeros(6), np.arange(6.0) * 4, Settings(max_speed_deg_s=10))
        speed, acceleration = np.full(6, 10.0), np.full(6, 30.0)
        fitted = fit_timing(path, speed, acceleration)
        self.assertLess(fitted.duration, path.duration)
        np.testing.assert_array_equal(fitted.positions, path.positions)
        self.assertTrue(np.all(fitted.peaks()[0] <= speed))
        self.assertTrue(np.all(fitted.peaks()[1] < 0.9 * acceleration))
        hold = fit_timing(
            move_path(np.zeros(6), np.zeros(6), Settings()), speed, acceleration
        )
        self.assertEqual(hold.duration, 0.5)

    def test_experimental_plan_rejected_for_non_simulated_port(self):
        clock = Clock()
        arm = SimulatedArm([0] * 6, clock=clock)
        arm.offline = False
        control = WeldController(arm, clock=clock, pose_check=lambda *a, **k: None)
        control.select(True, False)
        plan = simple_plan(chassis=False)
        plan["offline_only"] = True
        with self.assertRaisesRegex(ValueError, "offline-only"):
            control.load_plan(plan)
        self.assertFalse(arm.enabled)
        self.assertEqual(arm.commands, [])

    def test_cli_rejects_experimental_hardware_before_opening_owner(self):
        with patch("controls.hardware.weld_ports.ArmOwner") as owner:
            with self.assertRaises(SystemExit):
                main(["--execute", "--arm", "--supported", "--optimize-preparation"])
            owner.assert_not_called()


class ModelSearchTests(unittest.TestCase):
    def test_recorded_start_reaches_same_weld_target_under_five_seconds(self):
        doc = json.loads(
            (
                ROOT / "src/robot_description/config/arm_calibration_measured.json"
            ).read_text()
        )
        candidate = json.loads(
            (ROOT / "docs/measurements/arm-a5-alignment-2026-11-16.json").read_text()
        )["recorded_candidate"]
        doc["joints"]["5"].update(
            model_intercept_deg=candidate["model_intercept_deg"],
            raw_branch_deg=candidate["raw_command_bounds_deg"],
        )
        mapping = CalibrationMap(doc)
        review = json.loads(
            (ROOT / "docs/measurements/arm-recovery-review-2026-11-16.json").read_text()
        )
        request = dict(
            mapping=doc,
            current_raw_deg=review["first_measured_raw_deg"],
            initial_raw_deg=review["standby_goal_raw_deg"],
            arm_enabled=True,
            chassis_enabled=False,
            settings=asdict(
                Settings(
                    max_speed_deg_s=10,
                    limit_profile="weld",
                    cycles=1,
                    optimize_preparation=True,
                )
            ),
        )
        plan = build_plan(request)
        self.assertTrue(plan["offline_only"])
        self.assertLess(plan["preparation_timing"]["nominal_total_s"], 5.0)
        self.assertGreater(plan["preparation_search"]["original_nominal_total_s"], 20.0)
        self.assertTrue(plan["preparation_search"]["changed_standby"])
        self.assertLess(plan["weld_start_check"]["error_mm"], 0.001)
        self.assertAlmostEqual(plan["weld_start_check"]["floor_gap_mm"], 2.0, places=5)
        np.testing.assert_array_equal(
            plan["approach"]["raw_deg"][-1], plan["weld"]["raw_deg"][0]
        )
        clock = Clock()
        arm = SimulatedArm(plan["current_raw_deg"], clock=clock)
        geometry = WeldGeometry(mapping)
        control = WeldController(arm, clock=clock, pose_check=geometry.check)
        control.select(True, False)
        control.load_plan(plan)
        control.go_initial()
        for _ in range(260):
            clock.advance()
            control.tick()
            if control.state in ("ready", "fault", "paused"):
                break
        self.assertEqual(control.state, "ready", control.message)
        self.assertLessEqual(clock(), 5.0)
        np.testing.assert_allclose(
            arm.read().raw_deg, plan["weld"]["raw_deg"][0], atol=0.25
        )
        control.close()
