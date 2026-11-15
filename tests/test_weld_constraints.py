"""Measurements, physical units and automatic planning policy; no motor owners."""

import source_paths
import json
import math
from pathlib import Path
import unittest
import tempfile
import numpy as np

from controls.controller.calibration import fit_alignment, derive_measured_configuration
from controls.controller.limits import motor_constraints, trajectory_feasibility, joint_speed_policy
from controls.controller.weld_plan import JointPath, retime, smooth_level, initial_from_model
from motors.arm_calibration import CalibrationMap

ROOT = Path(__file__).resolve().parents[1]


class MeasuredAlignmentTests(unittest.TestCase):
    def test_two_endpoint_scale_is_reported_not_promoted(self):
        fit = fit_alignment([10, 20], [0, -10.2])
        self.assertEqual(fit["gain"], -1)
        self.assertAlmostEqual(fit["affine_comparison"]["gain"], -1.02)
        with self.assertRaisesRegex(ValueError, "three"):
            fit_alignment([10, 20], [0, -10.2], allow_scale=True)

    def test_single_anchor_only_determines_offset(self):
        fit = fit_alignment([173.183435], [0])
        self.assertEqual(fit["gain"], -1)
        self.assertEqual(fit["offset_deg"], 173.183435)
        self.assertIsNone(fit["affine_comparison"])

    def test_known_external_ratio_not_internal_motor_gearing(self):
        fit = fit_alignment([10, 20], [5, 0], known_ratio=2)
        self.assertEqual(fit["gain"], -0.5)
        self.assertEqual(fit["offset_deg"], 10)

    def test_generated_files_match_preserved_measurement_sources(self):
        mapping, pose, report = derive_measured_configuration(ROOT)
        for name, data in (
            ("arm_calibration_measured", mapping),
            ("arm_initial_measured", pose),
            ("arm_alignment_report", report),
        ):
            self.assertEqual(
                json.loads((ROOT / f"src/robot_description/config/{name}.json").read_text()), data
            )
        self.assertFalse(mapping["production_qualified"])
        self.assertEqual([r["model_per_raw"] for r in mapping["joints"].values()], [-1] * 6)
        raw = initial_from_model(pose, CalibrationMap(mapping))
        self.assertAlmostEqual(raw[2], 149.39)
        self.assertFalse(report["initial"][2]["measured_initial"])
        self.assertTrue(report["initial"][2]["adjusted"])
        self.assertLessEqual(max(abs(v) for v in report["fit"]["2"]["residual_deg"]), 0.46)
        self.assertLessEqual(max(abs(v) for v in report["fit"]["3"]["residual_deg"]), 0.11)
        CalibrationMap(mapping).pose(dict(zip("123456", raw)))


class MotorConstraintTests(unittest.TestCase):
    def test_rated_speed_is_output_not_rotor_speed(self):
        limits = motor_constraints(ROOT, 10)
        self.assertEqual(
            [j["rated_output_deg_s"] for j in limits["joints"]], [600, 216, 216, 720, 720, 720]
        )
        self.assertEqual(
            [j["physical_design_deg_s"] for j in limits["joints"]], [300, 108, 108, 360, 360, 360]
        )
        self.assertEqual([j["deployment_deg_s"] for j in limits["joints"]], [10] * 6)
        self.assertEqual(
            [j["torque_budget_nm"] for j in limits["joints"]], [10, 6, 6, 1.75, 1.75, 1.75]
        )
        self.assertFalse(limits["dynamics_qualified"])

    def test_no_rated_speed_can_raise_host_debug_ceiling(self):
        with self.assertRaises(ValueError):
            motor_constraints(ROOT, 11)
        self.assertEqual(motor_constraints(ROOT, 2)["joints"][0]["deployment_deg_s"], 2)

    def test_accepted_weld_profile_uses_native_guards_and_preserves_trips(self):
        limits = motor_constraints(ROOT, 5, profile="weld")
        np.testing.assert_allclose(
            [r["deployment_deg_s"] for r in limits["joints"]], [math.degrees(1.2)] * 6
        )
        self.assertEqual([r["joint_policy_speed_deg_s"] for r in limits["joints"]], [None] * 6)
        self.assertEqual([r["reactive_trip_rad_s"] for r in limits["joints"]], [1.5] * 6)
        experiment = json.loads((ROOT / "src/controls/config/weld_experiment.json").read_text())
        self.assertEqual(experiment["frequency_hz"], 2.0)
        self.assertAlmostEqual(experiment["feed_mm_s"] / experiment["frequency_hz"], 6)

    def test_model_feasibility_preserves_speed_trip_and_separates_debug(self):
        constraints = motor_constraints(ROOT, 5)
        dynamics = dict(raw_model_acceleration_deg_s2=[200] * 6)
        result = trajectory_feasibility(
            [40] * 6, [300] * 6, constraints, dynamics, requested_hz=2, feed_m_s=0.012
        )
        self.assertFalse(result["requested_timing_passes_guarded_model"])
        np.testing.assert_allclose(result["native_guarded_speed_deg_s"], [math.degrees(1.2)] * 6)
        self.assertAlmostEqual(result["guarded_model_hz"], 2 / (math.sqrt(1.5) * 1.02))
        self.assertEqual(result["limiting_speed_joint"], 1)
        self.assertFalse(result["changes_debug_command_limits"])
        self.assertFalse(result["hardware_qualified"])
        self.assertFalse(constraints["secondary_jog_smoothing"])

    def test_joint_policy_rejects_missing_or_invalid_axis_without_fallback(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "src/controls/config/arm_motion_policy.json"
            target.parent.mkdir(parents=True)
            for value in (0, -1, float("nan"), float("inf"), True):
                policy = joint_speed_policy(ROOT)
                policy["max_speed_deg_s"]["A1"] = value
                target.write_text(json.dumps(policy))
                with self.assertRaises(ValueError):
                    joint_speed_policy(root)
            del policy["max_speed_deg_s"]["A1"]
            target.write_text(json.dumps(policy))
            with self.assertRaises(ValueError):
                joint_speed_policy(root)

    def test_automatic_retime_preserves_geometry_and_derivatives(self):
        path = JointPath(
            [0, 0.25, 0.5, 0.75, 1], [[v] * 6 for v in (0, 5, 0, -5, 0)], "periodic_cubic"
        )
        timed, factor = retime(path, np.array([3, 4, 5, 6, 7, 8]), np.full(6, 12.0))
        self.assertGreater(factor, 1)
        np.testing.assert_allclose(timed.positions, path.positions)
        self.assertTrue(np.all(timed.peaks()[0] <= np.array([3, 4, 5, 6, 7, 8])))
        self.assertTrue(np.all(timed.peaks()[1] <= 12.0))
        np.testing.assert_allclose(timed.sample(0.3 * factor)[0], path.sample(0.3)[0])
        with self.assertRaisesRegex(ValueError, "infeasible"):
            retime(path, np.ones(6), np.ones(6), automatic=False)

    def test_smooth_shape_preserves_dwell_stroke_and_zero_corner_speed(self):
        np.testing.assert_allclose(
            smooth_level([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]),
            [0, 1, 1, 1, 0, 0, -1, -1, -1, 0, 0],
            atol=1e-12,
        )
        for p in (0, 0.1, 0.3, 0.4, 0.5, 0.6, 0.8, 0.9):
            epsilon = 1e-6
            speed = (smooth_level(p + epsilon) - smooth_level(p - epsilon)) / (2 * epsilon)
            self.assertLess(abs(speed), 1e-5)


if __name__ == "__main__":
    unittest.main()
