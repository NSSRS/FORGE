"""Planner optimization correctness; wall-clock speed belongs in benchmarks."""

import source_paths
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from controls.controller.path_sampling import geometry_times, interval_peaks
from controls.controller.weld_plan import JointPath, Settings, move_path
from controls.controller.collision_filter import retain_arm_pairs
from controls.controller.planning_cache import cached_weave, clear_planning_caches
from controls.controller.preparation_search import stopped_move_lower_bound, fit_timing


class MotionBoundTests(unittest.TestCase):
    def test_stopped_bound_is_optimistic_for_checked_cubic_moves(self):
        rng = np.random.default_rng(19)
        for _ in range(40):
            start, goal = rng.uniform(-270, 270, (2, 6))
            speed = rng.uniform(5, 68, 6)
            acceleration = rng.uniform(5, 30, 6)
            path = fit_timing(move_path(start, goal, Settings(max_speed_deg_s=100)),
                              speed, acceleration)
            self.assertLessEqual(stopped_move_lower_bound(start, goal, speed, acceleration),
                                 path.duration + 1e-10)

    def test_triangular_trapezoidal_and_stationary_bounds(self):
        self.assertEqual(stopped_move_lower_bound(np.zeros(6), np.zeros(6), 10, 30), 0)
        self.assertAlmostEqual(stopped_move_lower_bound(np.zeros(6), np.ones(6)*30, 100, 30), 2)
        self.assertAlmostEqual(stopped_move_lower_bound(np.zeros(6), np.ones(6)*100, 10, 10), 11)


class GeometrySamplingTests(unittest.TestCase):
    def test_move_keeps_exact_joined_endpoint_after_large_raw_angle_changes(self):
        start = np.array([-140.71234567, -65.491, 138.6, 166.21234567, 31.1, 49.3])
        goal = np.array([-174.6915956464, -61.357358, 149.39, 173.183435236, 41.5140578775, 64.161])
        path = move_path(start, goal, Settings())
        np.testing.assert_array_equal(path.positions[0], start)
        np.testing.assert_array_equal(path.positions[-1], goal)

    def test_includes_knots_and_bounds_travel_at_internal_velocity_extrema(self):
        values = np.array([0, 2, -1, 0.0])
        path = JointPath([0, .15, .65, 1], np.tile(values[:, None], (1, 6)), "periodic_cubic")
        times = geometry_times(path)
        self.assertTrue(set(path.times).issubset(times))
        speed, acceleration = interval_peaks(path)
        for i in range(len(path.times) - 1):
            dense = np.linspace(path.times[i], path.times[i + 1], 401, endpoint=False)
            self.assertTrue(np.all(abs(path.curve(dense, 1)) <= speed[i] + 1e-10))
            self.assertTrue(np.all(abs(path.curve(dense, 2)) <= acceleration[i] + 1e-10))
        for lo, hi in zip(times[:-1], times[1:]):
            q = path.curve(np.linspace(lo, hi, 17))
            self.assertTrue(np.all(np.sum(abs(np.diff(q, axis=0)), axis=0) <= .1 + 1e-10))

    def test_slow_retiming_preserves_geometric_resolution_without_time_oversampling(self):
        path = move_path(np.zeros(6), np.arange(6) * 3.17, Settings(max_speed_deg_s=30))
        slow = JointPath(path.times * 37, path.positions)
        np.testing.assert_allclose(geometry_times(slow) / 37, geometry_times(path), atol=1e-12)
        self.assertLess(len(geometry_times(slow)), slow.duration / .02 / 10)

    def test_stationary_interval_and_invalid_budget(self):
        path = JointPath([0, 600], np.zeros((2, 6)))
        np.testing.assert_array_equal(geometry_times(path), [0, 600])
        for step in (0, -1, float("nan"), float("inf")):
            with self.subTest(step=step), self.assertRaises(ValueError):
                geometry_times(path, max_step_deg=step)
        moving = JointPath([0, 1], np.array([[0] * 6, [100] * 6]))
        with self.assertRaisesRegex(ValueError, "budget"):
            geometry_times(moving, budget=10)


class ArmCollisionFilterTests(unittest.TestCase):
    def test_pair_equivalence_including_asymmetric_types_and_body_aggregates(self):
        rng = np.random.default_rng(34)
        types, affinity = rng.integers(0, 32, (2, 100))
        arm = rng.random(100) < .5
        model = SimpleNamespace(geom_contype=types.copy(), geom_conaffinity=affinity.copy(),
            geom_bodyid=np.arange(100) // 5, body_contype=np.zeros(20, dtype=int),
            body_conaffinity=np.zeros(20, dtype=int))
        self.assertTrue(retain_arm_pairs(model, arm))
        before = ((types[:, None] & affinity[None, :]) | (affinity[:, None] & types[None, :])) != 0
        after = ((model.geom_contype[:, None] & model.geom_conaffinity[None, :]) |
                 (model.geom_conaffinity[:, None] & model.geom_contype[None, :])) != 0
        np.testing.assert_array_equal(after, before & (arm[:, None] | arm[None, :]))
        for body in range(20):
            geoms = model.geom_bodyid == body
            self.assertEqual(model.body_contype[body], np.bitwise_or.reduce(model.geom_contype[geoms]))
            self.assertEqual(model.body_conaffinity[body], np.bitwise_or.reduce(model.geom_conaffinity[geoms]))

    def test_unsupported_bit_masks_keep_full_collision_model(self):
        for mask in (-1, 1 << 20):
            model = SimpleNamespace(geom_contype=np.array([mask]), geom_conaffinity=np.array([1]))
            self.assertFalse(retain_arm_pairs(model, np.array([True])))
            self.assertEqual(model.geom_contype[0], mask)

    def test_actual_arm_contacts_match_unfiltered_model(self):
        from controls.controller.weld_plan import WeldGeometry
        from motors.arm_calibration import CalibrationMap
        root = Path(__file__).resolve().parents[1]
        fixture = json.loads((root / "tests/fixtures/weld_tracking_20261117.json").read_text())
        geometry = WeldGeometry(CalibrationMap(fixture["mapping"]), arm_contacts_only=False)
        model, data = geometry.model, geometry.data
        types, affinity = model.geom_contype.copy(), model.geom_conaffinity.copy()
        body_types, body_affinity = model.body_contype.copy(), model.body_conaffinity.copy()
        def arm_contacts():
            return [(tuple(c.geom), float(c.dist)) for c in data.contact
                    if np.any(geometry.arm_geoms[c.geom])]
        for name in ("home", "approach", "weld"):
            path = JointPath.from_document(fixture["plan"][name])
            for raw in path.curve(np.linspace(0, path.duration, 15)):
                model.geom_contype[:], model.geom_conaffinity[:] = types, affinity
                model.body_contype[:], model.body_conaffinity[:] = body_types, body_affinity
                geometry.check(raw)
                expected = arm_contacts()
                retain_arm_pairs(model, geometry.arm_geoms)
                geometry.mj.mj_collision(model, data)
                actual = arm_contacts()
                self.assertEqual([r[0] for r in actual], [r[0] for r in expected])
                np.testing.assert_allclose([r[1] for r in actual], [r[1] for r in expected], atol=1e-12, rtol=0)


class PlanningCacheTests(unittest.TestCase):
    def test_scene_restores_complete_integration_state_and_transforms(self):
        import mujoco
        from controls.controller.planning_cache import scene_copy
        model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint name="j"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor joint="j"/></actuator></mujoco>')
        data = mujoco.MjData(model)
        data.time, data.qpos[0], data.qvel[0], data.ctrl[0] = 3.2, .4, .6, .8
        mujoco.mj_forward(model, data)
        signature = mujoco.mjtState.mjSTATE_INTEGRATION
        expected = np.empty(mujoco.mj_stateSize(model, signature))
        mujoco.mj_getState(model, data, expected, signature)
        clear_planning_caches()
        with patch('controls.simulation.weave_xy_z.floor_scene', return_value=(model, data)):
            copied_model, copied = scene_copy('state-test')
        actual = np.empty_like(expected)
        mujoco.mj_getState(copied_model, copied, actual, signature)
        np.testing.assert_array_equal(actual, expected)
        np.testing.assert_allclose(copied.geom_xpos, data.geom_xpos, atol=1e-12)
        clear_planning_caches()

    def test_weave_cache_invalidates_inputs_and_returns_independent_data(self):
        clear_planning_caches()
        geometry = SimpleNamespace(mapping=SimpleNamespace(document={"calibration": 1}),
                                   enforce_spatial_clearance=True)
        path = move_path(np.zeros(6), np.ones(6), Settings())
        constraints = {"version": 1}
        with patch("controls.controller.weld_plan.solve_weave", return_value=(path, {"detail": [1]})) as solve, \
             patch("controls.controller.limits.motor_constraints", side_effect=lambda *a, **k: constraints):
            first, report, hit = cached_weave(geometry, Settings(), Path("."), "scene-a")
            self.assertFalse(hit)
            report["detail"][0] = 99
            first.curve.c[:] = 0
            second, report, hit = cached_weave(geometry, Settings(), Path("."), "scene-a")
            self.assertTrue(hit)
            self.assertEqual(report["detail"], [1])
            np.testing.assert_allclose(second.sample(second.duration)[0], np.ones(6))
            self.assertEqual(solve.call_count, 1)
            cached_weave(geometry, Settings(hz=1), Path("."), "scene-a")
            cached_weave(geometry, Settings(), Path("."), "scene-b")
            geometry.mapping.document["calibration"] = 2
            cached_weave(geometry, Settings(), Path("."), "scene-a")
            constraints["version"] = 2
            cached_weave(geometry, Settings(), Path("."), "scene-a")
            self.assertEqual(solve.call_count, 5)
            geometry.enforce_spatial_clearance = False
            _, _, hit = cached_weave(geometry, Settings(), Path("."), "scene-a")
            self.assertFalse(hit)
            self.assertEqual(solve.call_count, 6)
        clear_planning_caches()

    def test_scene_copies_do_not_share_mutable_state(self):
        from controls.controller.planning_cache import scene_copy
        # Reuse the current model identity to keep the test about ownership.
        from simulation.arm_calibration import scene_fingerprint
        from simulation.paths import MODEL_DIR
        identity = scene_fingerprint(MODEL_DIR / "robot.xml")
        first, data_a = scene_copy(identity)
        second, data_b = scene_copy(identity)
        original = second.geom_contype.copy()
        first.geom_contype[:] = 0
        data_a.qpos[:] = 0
        np.testing.assert_array_equal(second.geom_contype, original)
        self.assertTrue(np.any(data_b.qpos != 0))


if __name__ == "__main__":
    unittest.main()
