"""Checked staging timing and hardware opt-in contracts; no motor hardware."""

import source_paths
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
from controls.controller.preparation_search import fit_timing
from controls.controller.calibration import select_a1_encoder_branch
from controls.controller.weld_plan import (
    Settings,
    build_plan,
    move_path,
    JointPath,
    WeldGeometry,
    approach_path,
)
from controls.hardware.weld_app import main
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from motors.arm_calibration import CalibrationMap
from test_weld_hardware import Clock, simple_plan
from test_weld_preparation import WristLagArm, PositionLoopArm

ROOT = Path(__file__).resolve().parents[1]


class TimingTests(unittest.TestCase):
    def test_required_lift_cannot_fall_back_to_direct_route(self):
        geometry = MagicMock()
        geometry.solve.side_effect = ValueError("raised route blocked")
        # The direct route would pass, but an explicit lift must be honored.
        with self.assertRaisesRegex(ValueError, "raised route blocked"):
            approach_path(geometry, [0]*6, [1]*6, Settings(approach_lift_m=0.03))
        geometry.validate_path.assert_not_called()
        for value in (-0.001, 0.151, float("nan"), float("inf"), True):
            with self.assertRaisesRegex(ValueError, "Approach lift"):
                Settings(approach_lift_m=value)

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
        control = WeldController(
            arm, clock=clock, pose_check=lambda *a, **k: None,
            allow_optimized_preparation=True,
        )
        control.select(True, False)
        plan = simple_plan(chassis=False)
        plan["offline_only"] = True
        with self.assertRaisesRegex(ValueError, "offline-only"):
            control.load_plan(plan)
        self.assertFalse(arm.enabled)
        self.assertEqual(arm.commands, [])

    def test_hardware_staging_requires_opt_in_and_load_never_enables(self):
        clock = Clock()
        arm = SimulatedArm([0] * 6, clock=clock)
        arm.offline = False
        plan = simple_plan(chassis=False)
        plan["preparation_search"] = {"changed_standby": True}
        control = WeldController(arm, clock=clock, pose_check=lambda *a, **k: None)
        control.select(True, False)
        with self.assertRaisesRegex(ValueError, "requires --optimize-preparation"):
            control.load_plan(plan)
        self.assertFalse(arm.enabled)
        self.assertEqual(arm.commands, [])
        control.allow_optimized_preparation = True
        control.load_plan(plan)
        self.assertEqual(control.state, "planned")
        self.assertFalse(arm.enabled)
        self.assertEqual(arm.commands, [])

    def test_cli_staging_still_requires_supported_fixture(self):
        with patch("controls.hardware.weld_ports.ArmOwner") as owner:
            with self.assertRaises(SystemExit):
                main(["--execute", "--arm", "--optimize-preparation"])
            owner.assert_not_called()

    def test_cli_passes_hardware_staging_opt_in_to_panel(self):
        window = MagicMock()
        window.worker.report = {}
        window.worker.failed = False
        with tempfile.TemporaryDirectory() as directory, patch(
            "controls.hardware.weld_app.make_window", return_value=window
        ) as panel, patch("PyQt5.QtWidgets.QApplication"), patch(
            "simulation.arm_calibration.scene_fingerprint", return_value="test"
        ), patch("controls.hardware.weld_ports.ArmOwner") as owner:
            result = main([
                "--execute", "--arm", "--supported", "--optimize-preparation",
                "--max-speed-deg-s", "10", "--prep-seconds", "5",
                "--approach-lift-mm", "30",
                "--output", str(Path(directory) / "report.json"),
            ])
            self.assertEqual(result, 0)
            args, _, _, settings, _ = panel.call_args.args
            self.assertTrue(args.execute)
            self.assertTrue(settings.optimize_preparation)
            self.assertEqual(settings.max_speed_deg_s, 10)
            self.assertEqual(settings.preparation_target_s, 5)
            self.assertEqual(settings.approach_lift_m, 0.03)
            owner.assert_not_called()


class ModelSearchTests(unittest.TestCase):
    def test_raised_approach_from_recorded_starts_retains_clearance_with_unequal_lag(self):
        doc = json.loads((ROOT / "src/robot_description/config/arm_calibration_measured.json").read_text())
        candidate = json.loads((ROOT / "docs/measurements/arm-a5-alignment-2026-11-16.json").read_text())["recorded_candidate"]
        doc["joints"]["5"].update(
            model_intercept_deg=candidate["model_intercept_deg"],
            raw_branch_deg=candidate["raw_command_bounds_deg"],
        )
        mapping = select_a1_encoder_branch(CalibrationMap(doc), 1)
        earlier = json.loads((ROOT / "docs/measurements/arm-recovery-review-2026-11-16.json").read_text())
        standby = np.array(earlier["standby_goal_raw_deg"])
        standby[0] += 360
        review = json.loads((ROOT / "docs/measurements/arm-preparation-review-2026-11-17.json").read_text())
        starts = [row["current_raw_deg"] for row in review["attempts"]]
        starts.append(review["disabled_followup"]["raw_deg"])
        for start in starts:
            with self.subTest(start=start):
                plan = build_plan(dict(
                    mapping=mapping.document, current_raw_deg=start,
                    initial_raw_deg=standby.tolist(), arm_enabled=True, chassis_enabled=False,
                    settings=asdict(Settings(max_speed_deg_s=100, limit_profile="weld", cycles=1,
                                             optimize_preparation=True, approach_lift_m=0.03)),
                ))
                self.assertIn("30 mm", plan["approach_route"])
                self.assertFalse(plan["hardware_qualified"])
                self.assertAlmostEqual(plan["weld_start_check"]["floor_gap_mm"], 2, places=5)
                np.testing.assert_array_equal(plan["approach"]["raw_deg"][-1], plan["weld"]["raw_deg"][0])
                for plant in (WristLagArm, PositionLoopArm):
                    clock = Clock()
                    arm = plant(start, clock=clock)
                    geometry = WeldGeometry(mapping)
                    control = WeldController(arm, clock=clock, pose_check=geometry.check,
                                             allow_optimized_preparation=True)
                    control.select(True, False)
                    control.load_plan(plan)
                    control.go_initial()
                    for _ in range(1500):
                        clock.advance(0.04)
                        control.tick()
                        if control.state in ("ready", "fault", "paused"):
                            break
                    self.assertEqual(control.state, "ready", f"{plant.__name__}: {control.message}")
                    self.assertEqual(control.tracking_deg, 3)
                    self.assertEqual(control.arrival_deg, 0.25)
                    np.testing.assert_allclose(arm.q, plan["weld"]["raw_deg"][0], atol=0.25)
                    geometry.check(arm.q)
                    control.close()

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
        self.assertFalse(plan.get("offline_only", False))
        self.assertFalse(plan["hardware_qualified"])
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
        # Exercise the physical-owner policy with ideal fake shafts only.
        arm.offline = False
        geometry = WeldGeometry(mapping)
        control = WeldController(
            arm, clock=clock, pose_check=geometry.check,
            allow_optimized_preparation=True,
        )
        control.select(True, False)
        control.load_plan(plan)
        self.assertFalse(arm.enabled)
        self.assertEqual(arm.commands, [])
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
