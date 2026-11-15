"""Optimizer semantics and fail-closed reuse; no hardware owners."""

import source_paths
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np

from controls.controller.weld_plan import (
    Settings,
    JointPath,
    candidate_weights,
    validate_search_solution,
)
from controls.controller.weld_search import decode, encode, objective, solution_document
from controls.controller.limits import motor_constraints
from motors.arm_calibration import CalibrationMap

ROOT = Path(__file__).resolve().parents[1]


class SearchObjectiveTests(unittest.TestCase):
    def path(self, amplitude):
        times = np.linspace(0, 1, 101)
        values = np.sin(2 * np.pi * times)[:, None] * np.asarray(amplitude)[None, :]
        values[-1] = values[0]
        return JointPath(times, values, "periodic_cubic")

    def test_larger_mobility_means_less_penalty(self):
        candidate = decode([-0.5, -0.5, 0.5, 0.5, 0.5, 0, 0])
        weights = candidate_weights(candidate)
        self.assertGreater(weights[0], weights[1])
        self.assertGreater(weights[1], weights[3])
        self.assertEqual(candidate["center_yaw_deg"], 0)
        for bad in ([0] * 6, [0] * 6 + [float("nan")], [0] * 6 + [1.1]):
            with self.assertRaises(ValueError):
                decode(bad)

    def test_warm_start_preserves_candidate_and_rejects_outside_search(self):
        x = [-0.5, -0.3, 0.1, 0.3, 0.8, -0.6, 0.2]
        np.testing.assert_allclose(encode(decode(x)), x)
        candidate = decode(x)
        candidate["center_yaw_deg"] = 15
        with self.assertRaises(ValueError):
            encode(candidate)

    def test_lower_speed_at_same_clock_has_lower_cost(self):
        settings = Settings(hz=1, cycles=1)
        low = objective(self.path([1] * 6), settings, [20] * 6, [500] * 6)
        high = objective(self.path([2] * 6), settings, [20] * 6, [500] * 6)
        self.assertEqual(low["time_scale"], high["time_scale"])
        self.assertLess(low["score"], high["score"])

    def test_a1_cap_affects_required_timing_not_just_a_reward(self):
        settings = Settings(hz=1, cycles=1)
        path = self.path([4, 0, 0, 0, 0, 0])
        metrics = objective(path, settings, [20, 60, 60, 60, 60, 60], [1000] * 6)
        self.assertGreater(metrics["time_scale"], 1)
        self.assertLessEqual(metrics["peak_executable_deg_s"][0], 20)
        self.assertLess(metrics["effective_hz"], settings.hz)

    def test_extra_reversals_are_penalized(self):
        path = self.path([1] * 6)
        settings = Settings(hz=1, cycles=1)
        t = path.times
        positions = np.sin(6 * np.pi * t)[:, None] * np.ones((1, 6))
        positions[-1] = positions[0]
        extra = objective(
            JointPath(t, positions, "periodic_cubic"), settings, [100] * 6, [1000] * 6
        )
        simple = objective(path, settings, [100] * 6, [1000] * 6)
        self.assertEqual(simple["extra_reversals"], 0)
        self.assertGreater(extra["extra_reversals"], 0)

    def test_reuse_rejects_changed_model_mapping_constraints_and_amplitude(self):
        mapping = CalibrationMap(
            json.loads(
                (ROOT / "src/robot_description/config/arm_calibration_measured.json").read_text()
            )
        )
        geometry = SimpleNamespace(mapping=mapping)
        settings = Settings(limit_profile="weld")
        constraints = motor_constraints(ROOT, profile="weld")
        candidate = decode([0] * 7)
        document = solution_document(candidate, geometry, settings, constraints)
        self.assertEqual(
            validate_search_solution(document, geometry, settings, constraints), candidate
        )
        for key in (
            "scene_fingerprint",
            "mapping_fingerprint",
            "constraints_fingerprint",
            "amplitude_m",
        ):
            invalid = {**document, key: None}
            with self.assertRaises(ValueError):
                validate_search_solution(invalid, geometry, settings, constraints)
        with self.assertRaises(ValueError):
            replace(settings, search_solution=document, lock_base_yaw=True)


if __name__ == "__main__":
    unittest.main()
