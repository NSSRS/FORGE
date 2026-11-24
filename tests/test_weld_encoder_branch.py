"""Explicit restart coordinates without wrapping feedback or physical travel."""

import source_paths
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from controls.controller.calibration import (
    select_a1_encoder_branch, check_a1_recovery_branch,
    nearest_a1_recovery_branch, check_a1_branch_translation,
)
from controls.controller.weld_plan import initial_from_model, move_path, Settings, build_plan
from controls.hardware.weld_app import main
from motors.arm_calibration import CalibrationMap

ROOT = Path(__file__).resolve().parents[1]


class EncoderBranchTests(unittest.TestCase):
    def setUp(self):
        self.mapping = CalibrationMap(json.loads(
            (ROOT / "src/robot_description/config/arm_calibration_measured.json").read_text()
        ))
        self.pose = json.loads(
            (ROOT / "src/robot_description/config/arm_standby_compact.json").read_text()
        )

    def test_reported_pose_translates_targets_and_preserves_model_and_travel(self):
        original = deepcopy(self.mapping.document)
        measured = 163.1074
        with self.assertRaisesRegex(ValueError, "A1.*163.1074"):
            self.mapping.model(1, measured)
        session = select_a1_encoder_branch(self.mapping, 1)
        self.assertAlmostEqual(session.model(1, measured), self.mapping.model(1, measured - 360))
        before = initial_from_model(self.pose, self.mapping)
        after = initial_from_model(self.pose, session)
        self.assertAlmostEqual(after[0], 185.30840435359923)
        np.testing.assert_allclose(np.array(after) - before, [360, 0, 0, 0, 0, 0])
        np.testing.assert_allclose(np.array(session.raw_limits(1)) - self.mapping.raw_limits(1), 360)
        start = np.array(after)
        start[0] = measured
        path = move_path(start, after, Settings())
        path.check_raw_bounds([session.raw_limits(i) for i in range(1, 7)])
        self.assertLess(np.ptp(path.positions[:, 0]), 23)
        self.assertTrue(np.all(path.positions[:, 0] > 160))
        for q in path.positions[:, 0]:
            self.assertAlmostEqual(session.raw(1, session.model(1, q)), q)
        for joint in range(2, 7):
            self.assertEqual(session.joints[str(joint)], self.mapping.joints[str(joint)])
            self.assertEqual(session.raw_limits(joint), self.mapping.raw_limits(joint))
        self.assertEqual(self.mapping.document, original)
        self.assertNotEqual(session.fingerprint, self.mapping.fingerprint)
        self.assertEqual(session.document["encoder_branch_selection"]["source_mapping_fingerprint"], self.mapping.fingerprint)
        self.assertFalse(session.document["production_qualified"])

    def test_nearest_target_uses_both_directions_without_wrapping_feedback(self):
        for turns, measured, expected in ((0, 175.01944472656137, 185.30840435359923),
                                           (1, -181.5109657967682, -174.69159564640077),
                                           (0, 4.0, -174.69159564640077),
                                           (0, 6.0, 185.30840435359923)):
            with self.subTest(turns=turns, measured=measured):
                mapping = select_a1_encoder_branch(self.mapping, turns)
                original = deepcopy(mapping.document)
                goal = initial_from_model(self.pose, mapping)
                current = list(goal); current[0] = measured
                selected, target, decision = nearest_a1_recovery_branch(mapping, current, goal)
                self.assertAlmostEqual(target[0], expected)
                self.assertLessEqual(abs(target[0] - measured), 180)
                self.assertEqual(current[0], measured)
                self.assertEqual(mapping.document, original)
                self.assertEqual(target[1:], list(goal)[1:])
                self.assertAlmostEqual(selected.model(1, target[0]), mapping.model(1, goal[0]))
                for joint in range(2, 7):
                    self.assertEqual(selected.joints[str(joint)], mapping.joints[str(joint)])
                    self.assertEqual(selected.raw_limits(joint), mapping.raw_limits(joint))
                if decision:
                    check_a1_branch_translation(mapping, selected)
                    self.assertEqual(decision['direction'], 'CCW' if target[0] > measured else 'CW')
                else:
                    self.assertIs(selected, mapping)

    def test_branch_selection_rejects_nonperiodic_or_unbounded_changes(self):
        goal = initial_from_model(self.pose, self.mapping)
        current = list(goal); current[0] += 360
        for gain, width in ((-.5, 360), (-1, 200)):
            document = deepcopy(self.mapping.document)
            row = document['joints']['1']; row['model_per_raw'] = gain
            row['raw_branch_deg'][1] = row['raw_branch_deg'][0] + width
            with self.assertRaisesRegex(ValueError, 'unit-gain full-turn'):
                nearest_a1_recovery_branch(CalibrationMap(document), current, goal)
        current[0] += 360
        with self.assertRaisesRegex(ValueError, 'more than one adjacent'):
            nearest_a1_recovery_branch(self.mapping, current, goal)

    def test_runtime_branch_update_cannot_change_other_axes_or_move_motor(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from controls.hardware.weld_ports import ArmOwner

        goal = initial_from_model(self.pose, self.mapping)
        current = list(goal); current[0] += 350
        selected, _, _ = nearest_a1_recovery_branch(self.mapping, current, goal)
        owner = ArmOwner.__new__(ArmOwner)
        owner.port = SimpleNamespace(mapping=self.mapping, path_active=False, moving=False,
                                     hold_targets={0: 1.0}, _event=Mock(), poll=Mock(), _refresh_targets=Mock())
        owner.configure_mapping(selected.document)
        self.assertEqual(owner.port.mapping.fingerprint, selected.fingerprint)
        self.assertEqual(owner.port.hold_targets, {0: 1.0})
        owner.port.poll.assert_not_called(); owner.port._refresh_targets.assert_not_called()
        owner.port.path_active = True
        with self.assertRaisesRegex(ValueError, 'during motion'):
            owner.configure_mapping(self.mapping.document)
        altered = deepcopy(selected.document); altered['joints']['4']['raw_branch_deg'][0] += 1
        with self.assertRaisesRegex(ValueError, 'other calibration'):
            check_a1_branch_translation(self.mapping, CalibrationMap(altered))

    def test_ambiguous_measured_return_is_rejected_before_model_search(self):
        goal = initial_from_model(self.pose, self.mapping)
        for measured in (175.01944472656137, 165.0089555544506):
            current = list(goal)
            current[0] = measured
            request = dict(mapping=self.mapping.document, current_raw_deg=current,
                           initial_raw_deg=goal, arm_enabled=True, chassis_enabled=False,
                           measured_hardware_start=True)
            before = deepcopy(request)
            with patch("controls.controller.weld_plan.WeldGeometry") as geometry:
                with self.assertRaisesRegex(ValueError, "A1 recovery would move.*nearest equivalent"):
                    build_plan(request)
                geometry.assert_not_called()
            self.assertEqual(request, before)
            session = select_a1_encoder_branch(self.mapping, 1)
            check_a1_recovery_branch(session, current, initial_from_model(self.pose, session))

    def test_branch_guard_preserves_small_extensions_and_rejects_unsynchronized_long_travel(self):
        low, high = self.mapping.raw_limits(1)
        for current, goal in ((high + 2, high - 2), (low - 2, low + 2)):
            check_a1_recovery_branch(self.mapping, [current]*6, [goal]*6)
        with self.assertRaisesRegex(ValueError, 'nearest equivalent'):
            check_a1_recovery_branch(self.mapping, [low + 1]*6, [high - 1]*6)

    def test_session_branch_stays_fixed_and_a4_never_rebases(self):
        self.assertIs(select_a1_encoder_branch(self.mapping), self.mapping)
        for turns in (-1, 1):
            session = select_a1_encoder_branch(self.mapping, turns)
            for value in (-196.8926, -174.6916):
                shifted = value + 360 * turns
                self.assertAlmostEqual(session.model(1, shifted), self.mapping.model(1, value))
                with self.assertRaises(ValueError):
                    session.model(1, value)
            lo, hi = session.raw_limits(1)
            for value in (lo - .01, hi + .01, float("nan")):
                with self.assertRaises(ValueError):
                    session.model(1, value)
            with self.assertRaises(ValueError):
                session.model(4, self.mapping.joints["4"]["model_intercept_deg"] + 360)
            with self.assertRaisesRegex(ValueError, "already translated"):
                select_a1_encoder_branch(session, turns)
        for turns in (True, 1.0, .5, 2, -2):
            with self.assertRaises(ValueError):
                select_a1_encoder_branch(self.mapping, turns)

    def test_cli_checks_source_pair_then_uses_session_branch_for_saved_feedback(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "mapping.json").write_text(json.dumps(self.mapping.document))
            (folder / "standby.json").write_text(json.dumps(self.pose))
            raw = initial_from_model(self.pose, self.mapping)
            raw[0] = 163.1074
            (folder / "start.json").write_text(json.dumps(dict(
                passed=True, feedback=[dict(joint=i, enabled=False, status=0, raw_position_deg=q)
                                       for i, q in enumerate(raw, 1)]
            )))
            args = ["--plan-only", "--arm", "--a1-encoder-turns", "1",
                    "--mapping", str(folder / "mapping.json"),
                    "--initial-pose", str(folder / "standby.json"),
                    "--start-report", str(folder / "start.json"),
                    "--output", str(folder / "result.json")]
            result = dict(initial_raw_deg=[], preparation_timing={}, weld_start_check={},
                          effective_hz=1.0, feed_m_s=.006, optimization={})
            with patch("controls.hardware.weld_app.build_plan", return_value=result) as build, \
                    patch("controls.hardware.weld_ports.ArmOwner") as owner, \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(main(args), 0)
                request = build.call_args.args[0]
                self.assertEqual(request["current_raw_deg"], raw)
                self.assertAlmostEqual(request["initial_raw_deg"][0], 185.30840435359923)
                session = CalibrationMap(request["mapping"])
                self.assertAlmostEqual(session.model(1, raw[0]), 22.20100435359924)
                saved = json.loads((folder / "result.json").read_text())
                self.assertEqual(saved["encoder_branch_selection"]["turns"], 1)
                owner.assert_not_called()
                # Selecting a branch cannot hide a mismatched source standby.
                (folder / "result.json").unlink()
                self.pose["mapping_fingerprint"] = "wrong-source-map"
                (folder / "standby.json").write_text(json.dumps(self.pose))
                build.reset_mock()
                self.assertEqual(main(args), 2)
                build.assert_not_called()
                owner.assert_not_called()


class RecordedRecoveryBranchTests(unittest.TestCase):
    def test_post_reset_pose_has_short_ccw_route_and_reaches_ready(self):
        from controls.controller.weld_plan import JointPath, WeldGeometry
        from controls.controller.measured_start import MeasuredStart, RecoveryGeometry
        from controls.controller.tracking import TrackingSettings
        from controls.hardware.weld_control import WeldController
        from controls.hardware.weld_ports import SimulatedArm
        from test_weld_hardware import Clock
        from test_weld_preparation import PositionLoopArm

        fixture = json.loads((ROOT / 'tests/fixtures/weld_a1_reset_20261124.json').read_text())
        mapping, initial, decision = nearest_a1_recovery_branch(
            CalibrationMap(fixture['mapping']), fixture['current_raw_deg'], fixture['initial_raw_deg'])
        self.assertEqual(decision['direction'], 'CCW')
        self.assertAlmostEqual(decision['delta_deg'], 6.819370150367433)
        plan = build_plan(dict(mapping=mapping.document, initial_raw_deg=initial,
                               current_raw_deg=fixture['current_raw_deg'], settings=fixture['settings'],
                               arm_enabled=True, chassis_enabled=False, measured_hardware_start=True))
        self.assertEqual(plan['mapping'], mapping.document)
        self.assertEqual(plan['current_raw_deg'], fixture['current_raw_deg'])
        self.assertLess(plan['preparation_timing']['nominal_total_s'], 10)
        geometry = WeldGeometry(mapping, arm_contacts_only=False, enforce_spatial_clearance=False)
        recovery = RecoveryGeometry(geometry, MeasuredStart.from_document(plan['measured_start']))
        samples = []
        for key in ('home', 'approach', 'weld'):
            path = JointPath.from_document(plan[key])
            if key == 'home':
                recovery.validate_path(path, clearance_mm=2)
            else:
                geometry.validate_path(path)
            if key != 'weld':
                samples.extend(path.curve(np.linspace(0, path.duration, 1001))[:, 0])
        self.assertLess(np.sum(abs(np.diff(samples))), 7)
        self.assertTrue(np.all(np.diff(samples) >= -1e-8))
        for plant in (SimulatedArm, PositionLoopArm):
            with self.subTest(plant=plant.__name__):
                clock = Clock(); arm = plant(plan['current_raw_deg'], clock=clock)
                control = WeldController(arm, clock=clock, pose_check=geometry.check,
                    recovery_check=geometry.check_recovery, allow_optimized_preparation=True,
                    tracking_settings=TrackingSettings())
                try:
                    control.select(True, False); control.load_plan(plan); control.go_initial()
                    for _ in range(1000):
                        clock.advance(); control.tick()
                        if control.state in ('ready', 'paused', 'fault'):
                            break
                    self.assertEqual(control.state, 'ready', control.message)
                    self.assertEqual(control.tracking_deg, 3)
                    self.assertLess(max(max(abs(v) for v in row['tracking_error_deg'])
                                        for row in control.trace), 3)
                    self.assertLessEqual(max(abs(arm.q - control.paths['approach'].positions[-1])), .25)
                finally:
                    control.close()

    def test_nov24_starts_use_short_checked_approach_on_verified_branch(self):
        from controls.controller.weld_plan import JointPath, WeldGeometry
        from controls.controller.measured_start import MeasuredStart, RecoveryGeometry
        from controls.controller.tracking import TrackingSettings
        from controls.hardware.weld_control import WeldController
        from controls.hardware.weld_ports import SimulatedArm
        from test_weld_hardware import Clock

        archive = ROOT / "docs/measurements/weld/2026-11-23"
        mapping = select_a1_encoder_branch(CalibrationMap(
            json.loads((archive / "mapping.json").read_text())), 1)
        pose = json.loads((archive / "standby.json").read_text())
        settings = json.loads((archive / "session.json").read_text())["settings"]
        # Saved Nov 24 observations, not new calibration anchors. The +1 branch
        # is an explicit offline hypothesis; it is never inferred by the planner.
        starts = (
            [175.01944472656137, -134.38741433534256, 79.54831726293042,
             184.46166215972707, 13.212536496526106, 112.399286868644],
            [165.0089555544506, -133.7972313495401, 81.7558890441919,
             181.72953185020862, 9.365719314461565, 59.898955851070944],
        )
        for start in starts:
            with self.subTest(start=start):
                plan = build_plan(dict(
                    mapping=mapping.document, initial_raw_deg=initial_from_model(pose, mapping),
                    current_raw_deg=start, settings=settings,
                    arm_enabled=True, chassis_enabled=False, measured_hardware_start=True,
                ))
                self.assertLess(plan["preparation_timing"]["nominal_total_s"], 8.0)
                self.assertLess(abs(plan["weld"]["raw_deg"][0][0] - start[0]), 21)
                np.testing.assert_array_equal(plan["home"]["raw_deg"][0], start)
                np.testing.assert_array_equal(plan["home"]["raw_deg"][-1], start)
                self.assertIn("30 mm", plan["approach_route"])
                self.assertAlmostEqual(plan["lowering_timing"]["duration_s"], 2.3)
                self.assertAlmostEqual(plan["effective_hz"], 1.729907219140131, places=5)
                self.assertFalse(plan["hardware_qualified"])
                geometry = WeldGeometry(mapping, arm_contacts_only=False, enforce_spatial_clearance=False)
                recovery = RecoveryGeometry(geometry, MeasuredStart.from_document(plan["measured_start"]))
                recovery.validate_path(JointPath.from_document(plan["home"]), clearance_mm=2)
                for name in ("approach", "weld"):
                    geometry.validate_path(JointPath.from_document(plan[name]))
                clock = Clock()
                arm = SimulatedArm(start, clock=clock)
                control = WeldController(
                    arm, clock=clock, pose_check=geometry.check,
                    recovery_check=geometry.check_recovery, allow_optimized_preparation=True,
                    tracking_settings=TrackingSettings(),
                )
                try:
                    control.select(True, False)
                    control.load_plan(plan)
                    self.assertFalse(arm.enabled)
                    control.go_initial()
                    for _ in range(1000):
                        clock.advance(); control.tick()
                        if control.state in ("ready", "paused", "fault"):
                            break
                    self.assertEqual(control.state, "ready", control.message)
                    self.assertLess(clock.value, 8.0)
                    self.assertEqual(control.tracking_deg, 3.0)
                    self.assertEqual(control.arrival_deg, .25)
                finally:
                    control.close()


if __name__ == "__main__":
    unittest.main()
