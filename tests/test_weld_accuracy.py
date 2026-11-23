"""Accuracy uses the matching checked pass and never opens a motor owner."""

import source_paths
import csv
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from controls.controller.weld_plan import JointPath
from controls.hardware.weld_accuracy import analyze, TipModel, orientation_error_deg
from motors.arm_calibration import CalibrationMap
from simulation.arm_calibration import scene_fingerprint
from simulation.paths import MODEL_DIR


class WeldAccuracyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = Path(__file__).resolve().parents[1] / "src/robot_description/config"
        cls.mapping = CalibrationMap(json.loads((config / "arm_calibration_measured.json").read_text()))
        cls.identity = scene_fingerprint(MODEL_DIR / "robot.xml")
        raw = list(cls.mapping.simulation_pose().values())
        cls.path = JointPath(
            [0.0, 0.16, 0.32], [raw, np.asarray(raw) + [0.3, 0, 0, 0, 0, 0], raw],
            interpolation="periodic_cubic",
        )

    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.journal = self.directory / "session.jsonl"
        self.plan = dict(
            schema="forge.weld_motion_plan", version=1, arm_enabled=True, chassis_enabled=True,
            mapping_fingerprint=self.mapping.fingerprint, scene_fingerprint=self.identity,
            weld=self.path.document(), duration_s=self.path.duration, feed_m_s=0.012, effective_hz=3.125,
        )

    def write_journal(
        self, *, error=0.0, age_ms=12, complete=True, dropped=0, second=False,
        continuous=False, lag_s=0.0, elapsed_times=(0.02, 0.08, 0.14, 0.20, 0.26, 0.31),
    ):
        rows = [
            dict(kind="session", schema="forge.weld_motion_journal", version=1,
                 mapping=self.mapping.document, simulated=True),
            dict(kind="accepted_plan", plan=self.plan),
        ]
        for started in [100.0, 200.0] if second else [100.0]:
            rows.append(dict(kind="sample", events=[
                dict(event="weld_started", continuous=continuous, time_s=started),
            ]))
            for index, elapsed in enumerate(elapsed_times):
                nominal = self.path.sample(elapsed % self.path.duration if continuous else elapsed)[0]
                actual = nominal + [error, 0, 0, 0, 0, 0]
                if lag_s:
                    actual = self.path.sample((elapsed - lag_s) % self.path.duration)[0]
                command = nominal + [0.05, 0, 0, 0, 0, 0]
                entry = dict(
                    kind="sample",
                    last_command=dict(
                        time_s=started + elapsed, stage="welding", elapsed_s=elapsed,
                        age_ms=[age_ms if index == 2 else 12] * 6, dispatch_state="sent",
                        requested_raw_deg=nominal.tolist(), command_raw_deg=command.tolist(),
                        measured_raw_deg=actual.tolist(),
                    ),
                )
                rows.extend([entry, entry])
            events = [dict(event="stage_arrived", from_stage="weld", state="complete")] if complete else [
                dict(event="pause"),
            ]
            rows.append(dict(kind="sample", events=events))
        rows.append(dict(kind="session_end", dropped_records=dropped, fault=None))
        self.journal.write_text("".join(json.dumps(row) + "\n" for row in rows))

    def write_tip(self, *, deviation_mm=0.0, height_mm=0.0, end_s=0.32, feed_mm_s=12.0, lag_s=0.0):
        model = TipModel(self.mapping, self.identity)
        origin = model.tip_mm(self.path.positions[0])
        csv_path = self.directory / "tip.csv"
        with csv_path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["elapsed_s", "x_mm", "y_mm", "z_mm"])
            for elapsed in np.linspace(0.0, end_s, max(2, round(end_s / 0.04) + 1)):
                phase = (elapsed - lag_s) % self.path.duration if lag_s else elapsed
                tip = model.tip_mm(self.path.sample(phase)[0]) - origin
                tip += [feed_mm_s * elapsed, deviation_mm, height_mm]
                writer.writerow([elapsed, *tip])
        return csv_path

    def test_tracking_dispatch_and_encoder_tip_remain_distinct(self):
        self.write_journal(error=0.10)
        report = analyze(self.journal, peak_joint_deg=0.11)
        self.assertEqual(report["coverage"]["samples"], 6)
        self.assertAlmostEqual(report["joints"]["A1"]["tracking_error_deg"]["rms"], 0.10)
        self.assertAlmostEqual(report["joints"]["A1"]["dispatch_error_deg"]["peak_abs"], 0.05)
        self.assertGreater(report["encoder_tip"]["distance_mm"]["rms"], 0)
        self.assertIsNone(report["independent_tip"])
        self.assertTrue(report["passed"])

    def test_orientation_rotation_angle_is_frame_invariant_and_includes_roll(self):
        # Independent analytic rotations, including the 180-degree singularity.
        common = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        for angle in (0.0, 0.5, 1.0, -30.0, 180.0):
            c, s = np.cos(np.radians(angle)), np.sin(np.radians(angle))
            roll = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
            with self.subTest(angle=angle):
                self.assertAlmostEqual(orientation_error_deg(common, common @ roll), abs(angle))
                self.assertAlmostEqual(orientation_error_deg(common @ roll, common), abs(angle))

    def test_orientation_target_is_separate_from_maximum_and_joint_error(self):
        # A1 rotates the complete tool frame by the same angle in the actual model.
        self.write_journal(error=0.75)
        report = analyze(self.journal, orientation_target_deg=0.5, max_orientation_deg=1.0)
        self.assertAlmostEqual(report["encoder_orientation"]["angle_deg"]["peak_abs"], 0.75)
        self.assertEqual(report["encoder_orientation"]["within_target_percent"], 0.0)
        self.assertEqual(report["encoder_orientation"]["within_maximum_percent"], 100.0)
        self.assertFalse(report["orientation_target_met"])
        self.assertTrue(report["orientation_maximum_met"])
        self.assertIsNone(report["joint_limit_met"])
        self.assertTrue(report["passed"])
        self.write_journal(error=1.2)
        report = analyze(self.journal, orientation_target_deg=0.5, max_orientation_deg=1.0)
        self.assertFalse(report["orientation_maximum_met"])
        self.assertFalse(report["passed"])

    def test_orientation_requires_matching_model_and_complete_evidence(self):
        self.write_journal(age_ms=80, complete=False)
        report = analyze(self.journal, orientation_target_deg=0.5, max_orientation_deg=1.0)
        self.assertTrue(report["orientation_maximum_met"])
        self.assertFalse(report["passed"])
        self.plan["scene_fingerprint"] = "different"
        self.write_journal()
        report = analyze(self.journal, max_orientation_deg=1.0)
        self.assertIsNone(report["encoder_orientation"])
        self.assertFalse(report["orientation_maximum_met"])
        self.assertFalse(report["passed"])
        with self.assertRaisesRegex(ValueError, "must not exceed"):
            analyze(self.journal, orientation_target_deg=2.0, max_orientation_deg=1.0)

    def test_measured_tip_includes_feed_and_applies_operator_limit(self):
        self.write_journal()
        csv_path = self.write_tip(deviation_mm=0.4)
        report = analyze(self.journal, tip_csv=csv_path, peak_tip_mm=0.3)
        self.assertAlmostEqual(report["independent_tip"]["transverse_mm"]["rms"], 0.4)
        self.assertAlmostEqual(report["independent_tip"]["along_mm"]["peak_abs"], 0)
        self.assertFalse(report["passed"])
        csv_path = self.write_tip(deviation_mm=0.4, end_s=0.15)
        report = analyze(self.journal, tip_csv=csv_path, peak_tip_mm=0.5)
        self.assertTrue(report["independent_tip_limit_met"])
        self.assertFalse(report["passed"])

    def test_arm_only_measurement_uses_fixed_workpiece_reference(self):
        self.plan["chassis_enabled"] = False
        self.write_journal()
        report = analyze(self.journal, tip_csv=self.write_tip(feed_mm_s=0.0), peak_tip_mm=0.01)
        self.assertFalse(report["chassis_feed_selected"])
        self.assertEqual(report["nominal_chassis_feed_mm_s"], 0.0)
        self.assertAlmostEqual(report["independent_tip"]["distance_mm"]["peak_abs"], 0.0)
        self.assertTrue(report["passed"])

    def test_fixed_lag_aligns_tip_and_retains_same_time_orientation(self):
        self.plan["chassis_enabled"] = False
        self.write_journal(continuous=True, complete=False, lag_s=0.04,
                           elapsed_times=np.arange(0.02, 0.97, 0.02))
        report = analyze(self.journal, max_orientation_deg=1.0)
        aligned = analyze(self.journal, max_orientation_deg=1.0, fit_tip_lag=True)
        self.assertNotIn("tip_lag", report)
        self.assertEqual(aligned["encoder_orientation"], report["encoder_orientation"])
        self.assertEqual(aligned["passed"], report["passed"])
        lag = aligned["tip_lag"]
        self.assertEqual(lag["source"], "encoder_reconstructed")
        self.assertAlmostEqual(lag["lag_ms"], 40.0, delta=0.05)
        self.assertLess(lag["aligned_error"]["distance_mm"]["peak_abs"], 0.001)
        self.assertAlmostEqual(
            lag["stroke"]["planned_width_mm"], lag["stroke"]["measured_width_mm"], places=3,
        )

    def test_fixed_lag_uses_independent_tip_and_retains_spatial_offset(self):
        self.plan["chassis_enabled"] = False
        self.write_journal(continuous=True, complete=False, lag_s=0.04,
                           elapsed_times=np.arange(0.02, 0.97, 0.02))
        csv_path = self.write_tip(end_s=0.96, feed_mm_s=0, lag_s=0.08, height_mm=0.5)
        lag = analyze(self.journal, tip_csv=csv_path, fit_tip_lag=True)["tip_lag"]
        self.assertEqual(lag["source"], "independent")
        self.assertAlmostEqual(lag["lag_ms"], 80.0, delta=0.05)
        self.assertAlmostEqual(lag["aligned_error"]["distance_mm"]["rms"], 0.5, places=3)
        csv_path = self.write_tip(end_s=0.96, feed_mm_s=0, lag_s=0.08, deviation_mm=0.5)
        lag = analyze(self.journal, tip_csv=csv_path, fit_tip_lag=True)["tip_lag"]
        self.assertAlmostEqual(lag["stroke"]["upper_edge_outward_error_mm"], 0.5, delta=0.02)
        self.assertAlmostEqual(lag["stroke"]["lower_edge_outward_error_mm"], -0.5, delta=0.02)
        csv_path = self.write_tip(end_s=0.15, feed_mm_s=0)
        lag = analyze(self.journal, tip_csv=csv_path, fit_tip_lag=True)["tip_lag"]
        self.assertIn("coverage", lag["status"])

    def test_fixed_lag_is_unavailable_for_chassis_or_finite_runs(self):
        self.write_journal(continuous=True)
        lag = analyze(self.journal, fit_tip_lag=True)["tip_lag"]
        self.assertIn("arm-only", lag["status"])
        self.plan["chassis_enabled"] = False
        self.write_journal()
        lag = analyze(self.journal, fit_tip_lag=True)["tip_lag"]
        self.assertIn("continuous", lag["status"])

    def test_missing_stale_dropped_or_interrupted_data_cannot_pass(self):
        self.write_journal(age_ms=80, complete=False, dropped=1)
        report = analyze(self.journal, peak_joint_deg=0.1)
        self.assertEqual(report["rejected_samples"], 1)
        self.assertEqual(report["coverage"]["samples"], 5)
        self.assertGreater(report["coverage"]["max_gap_ms"], 100)
        self.assertEqual(report["outcome"], "pause")
        self.assertFalse(report["passed"])

    def test_historical_model_can_show_joint_metrics_only(self):
        self.plan["scene_fingerprint"] = "different"
        self.write_journal()
        report = analyze(self.journal)
        self.assertIsNone(report["encoder_tip"])
        self.assertIsNone(report["passed"])
        self.assertEqual(report["joints"]["A1"]["tracking_error_deg"]["peak_abs"], 0)
        csv_path = self.write_tip()
        with self.assertRaisesRegex(ValueError, "exact model"):
            analyze(self.journal, tip_csv=csv_path)

    def test_requires_selection_of_a_specific_run(self):
        self.write_journal(second=True)
        with self.assertRaisesRegex(ValueError, "--run-number"):
            analyze(self.journal)
        self.assertEqual(analyze(self.journal, run_number=2)["coverage"]["samples"], 6)

    def test_rejects_mismatched_mapping_and_invalid_tip_csv(self):
        csv_path = self.directory / "invalid.csv"
        csv_path.write_text("elapsed_s,x_mm,y_mm,z_mm\n0,0,nan,0\n")
        self.write_journal()
        with self.assertRaisesRegex(ValueError, "CSV line 2"):
            analyze(self.journal, tip_csv=csv_path)
        self.plan["mapping_fingerprint"] = "different"
        self.write_journal()
        with self.assertRaisesRegex(ValueError, "different encoder mappings"):
            analyze(self.journal)
