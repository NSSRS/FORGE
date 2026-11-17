"""Nonideal fake shafts and reference pacing; no physical friction claim."""

import source_paths
from copy import deepcopy
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from controls.controller.preparation import PreparationClock, preparation_timing
from controls.controller.weld_plan import Settings, move_path, JointPath
from controls.controller.preparation_search import fit_timing
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_app import main
from controls.hardware.weld_ports import SimulatedArm
from test_weld_hardware import Clock, FakeChassis, simple_plan


def preparation_plan():
    settings = Settings(max_speed_deg_s=10)
    goal = np.array([5.0, 20.0, 15.0, 0.3, 7.0, -0.3])
    plan = simple_plan()
    home = move_path(np.zeros(6), goal, settings)
    plan.update(
        home=home.document(),
        initial_raw_deg=goal.tolist(),
        max_speed_deg_s=10.0,
        recovery=dict(version=1, clearance_mm=2.0, waypoint_times_s=[home.duration]),
    )
    plan["approach"] = move_path(goal, goal + 0.6, settings).document()
    plan["weld"]["raw_deg"] = (np.array(plan["weld"]["raw_deg"]) + goal).tolist()
    return plan


class LimitedArm(SimulatedArm):
    """A test plant with reduced speed and a minimum effective speed allowance."""

    capacity = 3.0
    minimum_allowance = 2.0

    def command(self, raw, speed):
        available = np.where(
            np.asarray(speed) >= self.minimum_allowance,
            np.minimum(speed, self.capacity),
            0.0,
        )
        super().command(raw, available)


class PositionLoopArm(SimulatedArm):
    """First-order shaft lag, not a gravity or motor model."""

    def poll(self):
        now = self.clock()
        dt = now - self.last
        self.last = now
        gain = np.array([10.0, 1.3, 1.3, 10.0, 10.0, 10.0])
        self.velocity = (
            np.clip(gain * (self.target - self.q), -self.speed, self.speed)
            if self.enabled
            else np.zeros(6)
        )
        self.q += self.velocity * dt


class WristLagArm(PositionLoopArm):
    """Include A5 response lag in a synthetic plant, not a fitted motor model."""

    def poll(self):
        now = self.clock()
        dt = now - self.last
        self.last = now
        gain = np.array([10.0, 1.3, 1.3, 10.0, 2.5, 10.0])
        self.velocity = (
            np.clip(gain * (self.target - self.q), -self.speed, self.speed)
            if self.enabled else np.zeros(6)
        )
        self.q += self.velocity * dt


class PreparationTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.arm = LimitedArm([0] * 6, clock=self.clock)
        self.control = WeldController(
            self.arm, FakeChassis(), clock=self.clock, pose_check=lambda raw, **kw: None
        )
        self.plan = preparation_plan()
        self.control.select(True, True)
        self.control.load_plan(self.plan)

    def tick(self):
        self.clock.advance()
        self.control.tick()

    def test_limited_shafts_reach_weld_start_without_reference_running_away(self):
        self.control.go_initial()
        for _ in range(1200):
            self.tick()
            if self.control.state in ("ready", "paused", "fault"):
                break
        self.assertEqual(self.control.state, "ready", self.control.message)
        trace = list(self.control.trace)
        self.assertTrue(any(row["path_clock_scale"] < 0.5 for row in trace))
        self.assertLess(
            max(max(abs(np.array(r["tracking_error_deg"]))) for r in trace), 3.0
        )
        self.assertTrue(all(r["speed_limit_deg_s"] == [10.0] * 6 for r in trace))
        self.assertEqual(self.control.chassis.last_feed, 0.0)
        # A1's target remains on the original monotone route, never a backlash kick.
        home = [r for r in trace if r["stage"] == "initial"]
        self.assertTrue(np.all(np.diff([r["requested_raw_deg"][0] for r in home]) >= 0))
        path = self.control.paths["home"]
        for row in home:
            np.testing.assert_allclose(
                row["requested_raw_deg"], path.sample(row["path_elapsed_s"])[0]
            )

    def test_fast_preparation_brakes_for_wrist_lag_without_relaxing_trips(self):
        settings = Settings(max_speed_deg_s=100)
        speed, acceleration = np.full(6, np.degrees(1.2)), np.full(6, 30.0)
        goal = np.array([0.15, 12.4, -1.0, -3.0, 45.0, -0.25])
        staging = 0.75 * goal
        home = fit_timing(move_path(np.zeros(6), staging, settings), speed, acceleration)
        approach = fit_timing(move_path(staging, goal, settings), speed, acceleration)
        plan = simple_plan(chassis=False)
        plan.update(
            home=home.document(), approach=approach.document(),
            initial_raw_deg=staging.tolist(), max_speed_deg_s=100.0,
            recovery=dict(version=1, clearance_mm=2.0, waypoint_times_s=[home.duration]),
            stage_limits={name: dict(speed_deg_s=speed.tolist(), acceleration_deg_s2=acceleration.tolist())
                          for name in ("home", "approach", "weld")},
        )
        plan["weld"]["raw_deg"] = (np.array(plan["weld"]["raw_deg"]) + goal - 0.6).tolist()
        for dt in (0.02, 0.04, 0.08, 0.1):
            with self.subTest(dt=dt):
                clock = Clock()
                arm = WristLagArm([0] * 6, clock=clock)
                control = WeldController(arm, clock=clock, pose_check=lambda *a, **k: None)
                control.select(True, False)
                control.load_plan(plan)
                control.go_initial()
                for _ in range(2000):
                    clock.advance(dt)
                    control.tick()
                    if control.state in ("ready", "paused", "fault"):
                        break
                self.assertEqual(control.state, "ready", control.message)
                self.assertLess(max(max(abs(np.array(r["tracking_error_deg"]))) for r in control.trace), 2.7)
                np.testing.assert_allclose(arm.q, goal, atol=0.25)
                self.assertEqual(control.tracking_deg, 3.0)
                self.assertEqual(control.arrival_deg, 0.25)

    def test_local_cubic_bounds_include_internal_extrema_and_knots(self):
        positions = np.array([[0, 2, 1, 0, -2, 3], [4, -1, 0, 2, 3, -1],
                              [-2, 3, -1, 0, 1, 4], [0, 2, 1, 0, -2, 3]])
        for interpolation in ("pchip", "periodic_cubic"):
            path = JointPath([0, 2, 4, 6], positions, interpolation)
            clock = PreparationClock(path, [100] * 6)
            for start, end in ((0, 0), (0, 2), (1.2, 4.8), (2, 4), (5.9, 6), (6, 6)):
                bounds = clock.derivative_bounds(start, end)
                times = np.linspace(start, end, 1001)
                for order, (low, high) in enumerate((bounds[:2], bounds[2:]), 1):
                    actual = path.curve.derivative(order)(times)
                    self.assertTrue(np.all(actual >= low - 1e-10))
                    self.assertTrue(np.all(actual <= high + 1e-10))

    def test_fast_clock_bounds_acceleration_through_each_step_and_zero_interval(self):
        path = fit_timing(
            move_path([0] * 6, [40, -20, 0.6, 3, 45, -0.2], Settings(max_speed_deg_s=100)),
            np.full(6, np.degrees(1.2)), np.full(6, 30.0),
        )
        clock = PreparationClock(path, [30] * 6)
        previous = np.zeros(6)
        for index in range(300):
            measured = previous.copy()
            if 10 <= index % 50 < 35:
                measured[4] -= 2.0
            start, scale = clock.elapsed, clock.scale
            dt = (0.01, 0.04, 0.08)[index % 3]
            previous, _ = clock.advance(dt, measured, 0, path.duration)
            change = (clock.scale - scale) / dt
            t = np.linspace(0, dt, 31)
            s = start + scale * t + 0.5 * change * t**2
            valid = s <= path.duration
            actual = (path.curve.derivative(2)(s[valid]) * (scale + change * t[valid])[:, None]**2
                      + path.curve.derivative()(s[valid]) * change)
            self.assertLessEqual(np.max(abs(actual)), 30.0 + 1e-8)
            self.assertGreaterEqual(clock.elapsed, start)
            self.assertGreaterEqual(clock.scale, 0)
            self.assertLessEqual(clock.scale, 1)
        before = clock.elapsed, clock.scale
        clock.advance(0, previous, 0, path.duration)
        self.assertEqual((clock.elapsed, clock.scale), before)

    def test_stuck_joint_times_out_without_skipping_waypoint(self):
        self.arm.capacity = 0.0
        self.control.go_initial()
        for _ in range(1000):
            self.tick()
            if self.control.state in ("paused", "fault"):
                break
        self.assertEqual(self.control.state, "paused")
        self.assertIn("Preparation stalled A", self.control.message)
        self.assertEqual(self.control.recovery_index, 0)
        self.assertTrue(self.arm.enabled)
        self.assertEqual(self.control.events[-1]["reason"], self.control.message)
        self.assertFalse(
            any(e["event"] == "recovery_waypoint_arrived" for e in self.control.events)
        )

    def test_large_error_still_trips_and_identifies_axis(self):
        self.control.go_initial()
        for _ in range(15):
            self.tick()
        self.arm.q[0] -= 4.0
        self.tick()
        self.assertEqual(self.control.state, "paused")
        self.assertIn("Tracking error A1", self.control.message)
        self.assertEqual(self.control.tracking_deg, 3.0)
        self.assertEqual(self.control.arrival_deg, 0.25)

    def test_pause_preserves_hold_and_requires_fresh_plan(self):
        self.control.go_initial()
        for _ in range(70):
            self.tick()
        self.control.pause()
        before = len(self.arm.commands)
        for _ in range(30):
            self.tick()
        self.assertEqual(len(self.arm.commands), before)
        self.assertTrue(self.arm.enabled)
        with self.assertRaisesRegex(ValueError, "fresh plan"):
            self.control.go_initial()
        self.assertEqual(self.control.chassis.last_feed, 0.0)

    def test_slow_position_loop_can_finish_after_old_nominal_deadline(self):
        # Scale of the failed Nov 16 home move, reproduced without its hardware.
        plan = simple_plan()
        goal = np.array([11.56, 58.69, 42.45, -13.03, 32.80, -1.31])
        home = move_path(np.zeros(6), goal, Settings(max_speed_deg_s=10))
        plan.update(
            home=home.document(),
            initial_raw_deg=goal.tolist(),
            max_speed_deg_s=10.0,
            recovery=dict(
                version=1, clearance_mm=2.0, waypoint_times_s=[home.duration]
            ),
        )
        plan["approach"] = move_path(goal, goal + 0.6, Settings()).document()
        plan["weld"]["raw_deg"] = (np.array(plan["weld"]["raw_deg"]) + goal).tolist()
        arm = PositionLoopArm([0] * 6, clock=self.clock)
        control = WeldController(
            arm, clock=self.clock, pose_check=lambda *a, **kw: None
        )
        control.select(True, False)
        plan["chassis_enabled"] = False
        control.load_plan(plan)
        control.go_initial()
        passed_old_deadline = False
        for _ in range(5000):
            self.clock.advance()
            control.tick()
            if self.clock() > home.duration + 10.5 and control.state == "initial":
                passed_old_deadline = True
            if control.state in ("ready", "paused", "fault"):
                break
        self.assertTrue(passed_old_deadline)
        self.assertEqual(control.state, "ready", control.message)
        self.assertLessEqual(max(abs(arm.q - (goal + 0.6))), 0.25)

    def test_pacing_still_detects_one_stalled_axis_and_ignores_encoder_noise(self):
        clock = PreparationClock(self.control.paths["home"], np.full(6, 30.0))
        reason = None
        for n in range(502):
            actual = np.full(6, n * 0.01)
            actual[4] = 0.04 if n % 2 else -0.04
            desired = actual + 0.8
            desired[4] = 1.0
            reason = clock.timeout_reason(n * 0.02, actual, desired, at_end=False)
        self.assertIsNotNone(reason)
        self.assertIn("A5", reason)

    def test_arrival_deadline_starts_at_reference_end_and_resets_for_next_waypoint(
        self,
    ):
        clock = PreparationClock(self.control.paths["home"], np.full(6, 30.0))
        actual, desired = np.zeros(6), np.full(6, 0.3)
        self.assertIsNone(clock.timeout_reason(100, actual, desired, at_end=True))
        self.assertIsNone(clock.timeout_reason(109, actual, desired, at_end=True))
        self.assertIn(
            "Arrival timeout", clock.timeout_reason(110.1, actual, desired, at_end=True)
        )
        clock.reset()
        self.assertIsNone(clock.timeout_reason(200, actual, desired, at_end=True))

    def test_five_second_goal_cannot_override_stage_limits_or_hide_settling(self):
        settings = Settings(max_speed_deg_s=10)
        home = move_path(np.zeros(6), np.array([0, 58.69, 0, 0, 0, 0]), settings)
        approach = move_path(
            home.positions[-1], np.array([0, 20.59, 0, 0, 0, 0]), settings
        )
        report = preparation_timing(
            home, approach, [home.duration], np.full(6, 10), np.full(6, 30), 5.0
        )
        self.assertTrue(report["ruled_out_by_limits"])
        self.assertFalse(report["nominal_meets_target"])
        self.assertAlmostEqual(report["minimum_settling_s"], 0.6)
        self.assertGreater(report["acceleration_only_lower_bound_s"], 5.0)
        self.assertFalse(report["hardware_qualified"])
        self.assertAlmostEqual(home.positions[-1, 1], 58.69)

    def test_welding_keeps_clock_and_full_allowance_through_endpoint_hold(self):
        self.arm.capacity = 10.0
        # This plant cannot move with the old sub-degree endpoint allowance.
        self.arm.minimum_allowance = 2.0
        self.control.go_initial()
        for _ in range(500):
            self.tick()
            if self.control.state == "ready":
                break
        self.assertEqual(self.control.state, "ready")
        self.control.start_weld()
        self.tick()
        row = self.control.trace[-1]
        self.assertEqual(row["stage"], "welding")
        self.assertEqual(row["path_clock_scale"], 1.0)
        self.assertAlmostEqual(row["path_elapsed_s"], row["elapsed_s"])
        np.testing.assert_allclose(row["speed_limit_deg_s"], 10.0)
        for _ in range(100):
            self.tick()
            if self.control.state == "complete":
                break
        self.assertEqual(self.control.state, "complete", self.control.message)
        target = self.arm.target.copy()
        allowance = self.arm.speed.copy()
        self.arm.q -= 0.2
        for _ in range(500):
            self.tick()
        np.testing.assert_array_equal(self.arm.target, target)
        np.testing.assert_array_equal(self.arm.speed, allowance)
        np.testing.assert_allclose(self.arm.q, target, atol=0.01)
        self.assertTrue(self.arm.enabled)

    def test_clock_respects_acceleration_envelope_and_never_reverses(self):
        path = move_path([0] * 6, [20, 8, 2, 0, -5, 3], Settings(max_speed_deg_s=10))
        clock = PreparationClock(path, [30] * 6)
        previous = np.zeros(6)
        positions, rates, scales = [], [], []
        for i in range(500):
            # Alternate abrupt lag and catch-up to exercise both clock slews.
            measured = previous.copy()
            if 30 <= i % 100 < 70:
                measured[0] -= 2.0
            position, rate = clock.advance(0.02, measured, 0, path.duration)
            positions.append(position)
            rates.append(rate)
            scales.append(clock.scale)
            previous = position
        self.assertLess(min(scales), 0.5)
        self.assertLessEqual(max(scales), 1.0)
        self.assertGreaterEqual(min(scales), 0.0)
        self.assertTrue(np.all(np.diff(np.array(positions)[:, 0]) >= 0))
        self.assertLessEqual(np.max(abs(np.diff(rates, axis=0))) / 0.02, 30.0 + 1e-6)
        np.testing.assert_allclose(positions[-1], path.positions[-1])

    def test_weld_allowance_uses_its_stage_limits_independently_of_preparation(self):
        plan = deepcopy(self.plan)
        plan["max_speed_deg_s"] = 5.0
        plan["home"] = move_path(np.zeros(6), plan["initial_raw_deg"], Settings()).document()
        plan["recovery"]["waypoint_times_s"] = [plan["home"]["times_s"][-1]]
        plan["limit_profile"] = "weld"
        plan["optimization"] = {"dynamics": {"raw_acceleration_deg_s2": [30] * 6}}
        plan["stage_limits"] = {
            "weld": dict(speed_deg_s=[7, 8, 9, 10, 9, 8], acceleration_deg_s2=[30] * 6)
        }
        self.control.load_plan(plan)
        self.control.go_initial()
        for _ in range(1500):
            self.tick()
            if self.control.state == "ready":
                break
        self.assertEqual(self.control.state, "ready", self.control.message)
        self.control.start_weld()
        self.tick()
        np.testing.assert_allclose(self.control.trace[-1]["speed_limit_deg_s"], [7, 8, 9, 10, 9, 8])

    def test_stage_specific_allowance_never_exceeds_checked_limit(self):
        plan = deepcopy(self.plan)
        # A1's small move fits a tighter independent limit.
        plan["stage_limits"] = {
            "home": dict(
                speed_deg_s=[3, 10, 10, 10, 10, 10], acceleration_deg_s2=[30] * 6
            )
        }
        self.control.load_plan(plan)
        self.control.go_initial()
        for _ in range(20):
            self.tick()
        self.assertEqual(
            self.control.trace[-1]["speed_limit_deg_s"],
            [3.0, 10.0, 10.0, 10.0, 10.0, 10.0],
        )

    def test_cli_withdraws_automatic_speed_increase_and_preserves_explicit_speed(self):
        with tempfile.TemporaryDirectory() as directory:
            for i, (profile, extra, expected) in enumerate(
                (
                    ("weld", [], 5.0),
                    ("debug", [], 5.0),
                    ("weld", ["--max-speed-deg-s", "7.5"], 7.5),
                    ("weld", ["--max-speed-deg-s", "100"], 100.0),
                    ("debug", ["--max-speed-deg-s", "100"], 100.0),
                )
            ):
                with (
                    self.subTest(profile=profile, extra=extra),
                    patch(
                        "controls.hardware.weld_app.build_plan",
                        return_value=dict(
                            effective_hz=1.0,
                            feed_m_s=0.006,
                            optimization={},
                            initial_raw_deg=[0.0] * 6,
                            preparation_timing={},
                            weld_start_check={},
                        ),
                    ) as build,
                    patch("controls.hardware.weld_ports.ArmOwner") as owner,
                    redirect_stdout(StringIO()),
                ):
                    code = main(
                        [
                            "--plan-only",
                            "--arm",
                            "--demo-initial",
                            "--profile",
                            profile,
                            "--output",
                            str(Path(directory) / f"{i}.json"),
                            *extra,
                        ]
                    )
                    self.assertEqual(code, 0)
                    self.assertEqual(
                        build.call_args.args[0]["settings"]["max_speed_deg_s"], expected
                    )
                    owner.assert_not_called()


if __name__ == "__main__":
    unittest.main()
