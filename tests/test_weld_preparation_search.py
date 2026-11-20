"""Checked staging timing and hardware opt-in contracts; no motor hardware."""

import source_paths
from dataclasses import asdict
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
from controls.controller.preparation_search import fit_timing, fit_approach_timing, apply_lowering_minimum
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
    def lowering_plan(self):
        home = move_path(np.zeros(6), np.ones(6), Settings())
        approach = JointPath(
            [0, 1, 2, 2.2, 2.6, 3.0, 3.4],
            np.array([np.ones(6) * q for q in (1, 2, 3, 3, 2.8, 2.2, 2)]),
        )
        return dict(
            home=home.document(), approach=approach.document(),
            recovery=dict(waypoint_times_s=[home.duration]),
            stage_limits=dict(approach=dict(speed_deg_s=[30]*6, acceleration_deg_s2=[30]*6)),
        )

    def test_lowering_minimum_preserves_transfer_dwell_geometry_and_limits(self):
        plan = self.lowering_plan()
        before = deepcopy(plan)
        changed = apply_lowering_minimum(plan, Settings(approach_lift_m=.03, lowering_min_s=2.3))
        self.assertEqual(plan, before)
        self.assertEqual(changed['home'], before['home'])
        self.assertEqual(changed['stage_limits'], before['stage_limits'])
        self.assertEqual(changed['recovery'], before['recovery'])
        old = JointPath.from_document(plan['approach'])
        new = JointPath.from_document(changed['approach'])
        np.testing.assert_array_equal(new.times[:4], old.times[:4])
        np.testing.assert_array_equal(new.positions, old.positions)
        self.assertAlmostEqual(new.duration-new.times[3], 2.3)
        self.assertAlmostEqual(changed['lowering_timing']['added_s'], 1.1)
        self.assertAlmostEqual(changed['preparation_timing']['stages'][1]['nominal_motion_s'], new.duration)
        for fraction in np.linspace(0, 1, 9):
            np.testing.assert_allclose(
                old.curve(old.times[:-1]+fraction*np.diff(old.times)),
                new.curve(new.times[:-1]+fraction*np.diff(new.times)), atol=1e-10, rtol=0)
        for old_peak, new_peak in zip(old.peaks(), new.peaks()):
            self.assertTrue(np.all(new_peak <= old_peak+1e-10))

    def test_lowering_minimum_is_opt_in_and_never_shortens_a_slower_leg(self):
        plan = self.lowering_plan()
        self.assertIs(apply_lowering_minimum(plan, Settings()), plan)
        changed = apply_lowering_minimum(plan, Settings(approach_lift_m=.03, lowering_min_s=.5))
        self.assertEqual(changed['approach'], plan['approach'])
        self.assertEqual(changed['lowering_timing']['added_s'], 0)

    def test_lowering_requires_a_finite_duration_and_stopped_raised_route(self):
        for duration in (-1, float('inf'), float('nan'), True):
            with self.subTest(duration=duration), self.assertRaisesRegex(ValueError, 'Lowering duration'):
                Settings(approach_lift_m=.03, lowering_min_s=duration)
        with self.assertRaisesRegex(ValueError, 'explicit approach lift'):
            Settings(lowering_min_s=2.3)
        plan = self.lowering_plan()
        plan['approach'] = plan['home']
        with self.assertRaisesRegex(ValueError, 'stationary boundary'):
            apply_lowering_minimum(plan, Settings(approach_lift_m=.03, lowering_min_s=2.3))

    def test_stopped_approach_legs_keep_geometry_without_shared_slowdown(self):
        first = move_path(np.zeros(6), np.arange(6.) * 6, Settings())
        lower = move_path(first.positions[-1], first.positions[-1] + 2, Settings())
        path = JointPath(
            np.r_[first.times, first.duration + .2, first.duration + .2 + lower.times[1:]],
            np.vstack([first.positions, first.positions[-1], lower.positions[1:]]),
        )
        speed, acceleration = np.full(6, 30.), np.full(6, 30.)
        together = fit_timing(path, speed, acceleration)
        separate = fit_approach_timing(path, speed, acceleration)
        self.assertLess(separate.duration, together.duration * .8)
        np.testing.assert_array_equal(separate.positions, path.positions)
        # Check every cubic, including knot boundaries and interior points.
        for fraction in np.linspace(0, 1, 9):
            original = path.times[:-1] + fraction * np.diff(path.times)
            changed = separate.times[:-1] + fraction * np.diff(separate.times)
            np.testing.assert_allclose(path.curve(original), separate.curve(changed), atol=1e-10, rtol=0)
        dwell = np.all(np.diff(path.positions, axis=0) == 0, axis=1)
        np.testing.assert_allclose(np.diff(separate.times)[dwell], np.diff(path.times)[dwell], atol=1e-12)
        self.assertTrue(np.all(separate.peaks()[0] <= speed))
        self.assertTrue(np.all(separate.peaks()[1] < .9 * acceleration))

    def test_approach_without_stationary_interval_keeps_uniform_timing(self):
        path = move_path(np.zeros(6), np.arange(6.) * 6, Settings())
        limits = np.full(6, 10.)
        expected = fit_timing(path, limits, np.full(6, 30.))
        actual = fit_approach_timing(path, limits, np.full(6, 30.))
        np.testing.assert_array_equal(actual.times, expected.times)

    def test_final_leg_floor_preserves_transfer_and_dwell(self):
        path = JointPath.from_document(self.lowering_plan()["approach"])
        speed, acceleration = np.full(6, 30.), np.full(6, 30.)
        fitted = fit_approach_timing(path, speed, acceleration)
        preserved = fit_approach_timing(path, speed, acceleration, minimum_final_s=2.)
        np.testing.assert_array_equal(preserved.times[:4], fitted.times[:4])
        np.testing.assert_array_equal(preserved.positions, fitted.positions)
        self.assertAlmostEqual(preserved.duration - preserved.times[3], 2.)
        for old, new in zip(fitted.peaks(), preserved.peaks()):
            self.assertTrue(np.all(new <= old + 1e-10))

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
                "--lowering-seconds", "2.3",
                "--output", str(Path(directory) / "report.json"),
            ])
            self.assertEqual(result, 0)
            args, _, _, settings, _ = panel.call_args.args
            self.assertTrue(args.execute)
            self.assertTrue(settings.optimize_preparation)
            self.assertEqual(settings.max_speed_deg_s, 10)
            self.assertEqual(settings.preparation_target_s, 5)
            self.assertEqual(settings.approach_lift_m, 0.03)
            self.assertEqual(settings.lowering_min_s, 2.3)
            owner.assert_not_called()


class ModelSearchTests(unittest.TestCase):
    def test_loaded_start_lowering_minimum_preserves_selected_route_with_lag(self):
        from controls.controller.tracking import TrackingSettings

        doc = json.loads((ROOT / "src/robot_description/config/arm_calibration_measured.json").read_text())
        candidate = json.loads((ROOT / "docs/measurements/arm-a5-alignment-2026-11-16.json").read_text())["recorded_candidate"]
        doc["joints"]["5"].update(model_intercept_deg=candidate["model_intercept_deg"],
                                  raw_branch_deg=candidate["raw_command_bounds_deg"])
        start = [-168.63720668307474, -111.56873586557045, 128.83572729492948,
                 175.01944472656137, -25.168263544256504, 157.42460421704172]
        standby = json.loads((ROOT / "docs/measurements/arm-recovery-review-2026-11-16.json").read_text())["standby_goal_raw_deg"]
        selected = []

        def retain_selected(plan, settings):
            selected.append(deepcopy(plan))
            return apply_lowering_minimum(plan, settings)

        with patch("controls.controller.preparation_search.apply_lowering_minimum",
                   side_effect=retain_selected):
            plan = build_plan(dict(
                mapping=doc, current_raw_deg=start, initial_raw_deg=standby,
                arm_enabled=True, chassis_enabled=False,
                settings=asdict(Settings(max_speed_deg_s=30, limit_profile="weld", cycles=1,
                                         optimize_preparation=True, approach_lift_m=.03,
                                         preparation_target_s=10, lowering_min_s=2.3))))
        self.assertEqual(len(selected), 1)
        before = selected[0]
        self.assertIn("preparation_search", before)
        for key in ("home", "recovery", "weld", "initial_raw_deg", "stage_limits", "preparation_search"):
            self.assertEqual(plan[key], before[key])
        self.assertAlmostEqual(plan["lowering_timing"]["duration_s"], 2.3)
        self.assertGreater(plan["lowering_timing"]["added_s"], 0)
        self.assertAlmostEqual(plan["preparation_timing"]["nominal_total_s"],
                               before["preparation_timing"]["nominal_total_s"] +
                               plan["lowering_timing"]["added_s"])
        self.assertFalse(plan["hardware_qualified"])
        clock = Clock()
        arm = PositionLoopArm(start, clock=clock)
        geometry = WeldGeometry(CalibrationMap(doc))
        control = WeldController(arm, clock=clock, pose_check=geometry.check,
                                 allow_optimized_preparation=True, tracking_settings=TrackingSettings())
        control.select(True, False)
        control.load_plan(plan)
        control.go_initial()
        for _ in range(1000):
            clock.advance(); control.tick()
            if control.state in ("ready", "fault", "paused"):
                break
        self.assertEqual(control.state, "ready", control.message)
        self.assertEqual(control.tracking_deg, 3.)
        self.assertEqual(control.arrival_deg, .25)
        np.testing.assert_allclose(arm.q, plan["weld"]["raw_deg"][0], atol=.25)
        control.close()

    def test_nov19_long_wrist_move_reaches_ready_within_ten_seconds_offline(self):
        from controls.controller.tracking import TrackingSettings

        doc = json.loads((ROOT / "src/robot_description/config/arm_calibration_measured.json").read_text())
        candidate = json.loads((ROOT / "docs/measurements/arm-a5-alignment-2026-11-16.json").read_text())["recorded_candidate"]
        doc["joints"]["5"].update(model_intercept_deg=candidate["model_intercept_deg"],
                                  raw_branch_deg=candidate["raw_command_bounds_deg"])
        start = [-141.49092035518254, -106.6509450216479, 140.09204297165059,
                 175.47843343878623, -29.845631729394597, 155.30445885715642]
        standby = json.loads((ROOT / "docs/measurements/arm-recovery-review-2026-11-16.json").read_text())["standby_goal_raw_deg"]
        plan = build_plan(dict(mapping=doc, current_raw_deg=start, initial_raw_deg=standby,
                               arm_enabled=True, chassis_enabled=False,
                               settings=asdict(Settings(max_speed_deg_s=30, limit_profile="weld", cycles=1,
                                                        optimize_preparation=True, approach_lift_m=.03,
                                                        preparation_target_s=10.))))
        self.assertLess(plan["preparation_timing"]["nominal_total_s"], 10.)
        self.assertFalse(plan["hardware_qualified"])
        self.assertIn("30 mm", plan["approach_route"])
        geometry = WeldGeometry(CalibrationMap(doc))
        for plant in (SimulatedArm, PositionLoopArm, WristLagArm):
            with self.subTest(plant=plant.__name__):
                clock = Clock()
                arm = plant(start, clock=clock)
                control = WeldController(arm, clock=clock, pose_check=geometry.check,
                                         allow_optimized_preparation=True, tracking_settings=TrackingSettings())
                control.select(True, False)
                control.load_plan(plan)
                control.go_initial()
                for _ in range(1000):
                    clock.advance(); control.tick()
                    if control.state in ("ready", "fault", "paused"):
                        break
                self.assertEqual(control.state, "ready", control.message)
                if plant is SimulatedArm:
                    self.assertLess(clock.value, 10.)
                np.testing.assert_allclose(arm.q, plan["weld"]["raw_deg"][0], atol=.25)
                self.assertEqual(control.tracking_deg, 3.)
                self.assertEqual(control.arrival_deg, .25)
                control.close()

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
