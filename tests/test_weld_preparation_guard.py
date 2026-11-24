"""Nov 24 preparation guard regression: recorded feedback and synthetic shafts.

Recorded feedback cannot predict the response to changed commands. Closed-loop
cases therefore use explicitly synthetic plants and the unfiltered model; none
of these checks establish physical qualification.
"""

import source_paths
import json
from pathlib import Path
import unittest

import numpy as np

from controls.controller.calibration import CalibrationMap
from controls.controller.preparation import PreparationClock
from controls.controller.tracking import TrackingSettings
from controls.controller.weld_plan import JointPath, WeldGeometry
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from test_weld_hardware import Clock
from test_weld_preparation import PositionLoopArm, WristLagArm
from test_weld_tracking import LaggedArm


FIXTURE = Path(__file__).parent / "fixtures/weld_preparation_guard_20261124.json"


class DelayedSlowArm(LaggedArm):
    lag = np.array([0.16, 1.0, 1.0, 0.14, 0.4, 0.25])
    delayed = True


class QuantizedDelayedArm(DelayedSlowArm):
    """Synthetic delay plus encoder/velocity quantization, not identified gains."""

    def __init__(self, raw, **kwargs):
        super().__init__(raw, **kwargs)
        self.origin = self.q.copy()

    def read(self):
        feedback = super().read()
        position_step = np.array([0.0437, 0.02185, 0.02185, 0.0437, 0.02185, 0.0437])
        speed_step = np.array([0.21, 0.14, 0.14, 0.42, 0.42, 0.42])
        feedback.raw_deg = self.origin + position_step * np.round(
            (feedback.raw_deg - self.origin) / position_step)
        feedback.velocity_deg_s = speed_step * np.round(feedback.velocity_deg_s / speed_step)
        feedback.age_ms[:] = 5
        return feedback


class PreparationGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads(FIXTURE.read_text())
        cls.plan = cls.fixture["plan"]

    def test_recorded_a3_guard_input_slows_before_three_degree_trip(self):
        path = JointPath.from_document(self.plan["approach"])
        clock = PreparationClock(path, [30] * 6)
        rows = self.fixture["approach_feedback"]
        origin = rows[0]["time_s"] - rows[0]["path_elapsed_s"]
        previous = origin
        errors, scales = [], []
        for row in [*rows, self.fixture["guard_observation"]]:
            dt = row["time_s"] - previous
            measured = np.array(row["measured_raw_deg"])
            reference, _ = clock.advance(dt, measured, 0, path.duration)
            errors.append(float(max(abs(reference - measured))))
            scales.append(clock.scale)
            previous = row["time_s"]
        old_reference, _ = path.sample(previous - origin)
        old_error = old_reference - measured
        self.assertAlmostEqual(old_error[2], 3.093, places=3)
        self.assertLess(max(errors), 2.5)
        self.assertLess(min(scales), 0.8)
        self.assertLess(clock.elapsed, previous - origin)
        self.assertTrue(all(0 <= value <= 1 for value in scales))

    def test_healthy_reference_keeps_nominal_clock(self):
        path = JointPath.from_document(self.plan["approach"])
        clock = PreparationClock(path, [30] * 6)
        for _ in range(100):
            measured, _ = path.sample(clock.elapsed)
            clock.advance(0.02, measured, 0, path.duration)
            self.assertEqual(clock.scale, 1)
        self.assertAlmostEqual(clock.elapsed, 2)

    def test_long_tick_does_not_hide_current_lag(self):
        path = JointPath.from_document(self.plan["approach"])
        clock = PreparationClock(path, [30] * 6)
        clock.elapsed = 1.4
        predicted, rate = path.sample(clock.elapsed + 0.08)
        measured = predicted - 1.5 * np.sign(rate)
        old_reference, _ = path.sample(clock.elapsed)
        # The old comparison saw less than 1 deg on the fast A3 axis even
        # though feedback trails this tick's reference by 1.5 deg.
        self.assertLess(abs(old_reference[2] - measured[2]), 1)
        clock.advance(0.08, measured, 0, path.duration)
        self.assertLess(clock.scale, 1)

    def test_ahead_axes_do_not_slow_the_reference_away_from_them(self):
        path = JointPath.from_document(self.plan["approach"])
        clock = PreparationClock(path, [30] * 6)
        clock.elapsed = 1.4
        predicted, rate = path.sample(clock.elapsed + 0.08)
        measured = predicted + 1.8 * np.sign(rate)
        clock.advance(0.08, measured, 0, path.duration)
        self.assertEqual(clock.scale, 1)

    def test_failed_plan_reaches_ready_with_lag_and_variable_ticks(self):
        geometry = WeldGeometry(CalibrationMap(self.fixture["mapping"]), arm_contacts_only=False)
        rows = self.fixture["approach_feedback"]
        recorded_intervals = tuple(np.diff([
            rows[0]["time_s"] - rows[0]["path_elapsed_s"],
            *[row["time_s"] for row in rows],
        ]))
        cases = (
            (SimulatedArm, (0.02,), 8.0),
            (PositionLoopArm, (0.02,), 11.0),
            (WristLagArm, (0.04,), 12.0),
            (WristLagArm, (0.08,), 13.0),
            (DelayedSlowArm, (0.01, 0.02, 0.04, 0.08, 0.099), 16.0),
            (QuantizedDelayedArm, recorded_intervals, 16.0),
        )
        for plant, intervals, deadline in cases:
            with self.subTest(plant=plant.__name__, intervals=intervals):
                clock = Clock()
                arm = plant(self.plan["current_raw_deg"], clock=clock)
                control = WeldController(
                    arm, clock=clock, pose_check=geometry.check,
                    recovery_check=geometry.check_recovery, allow_optimized_preparation=True,
                    tracking_settings=TrackingSettings(),
                )
                try:
                    control.select(True, False)
                    control.load_plan(self.plan)
                    self.assertFalse(arm.enabled)
                    control.go_initial()
                    for tick in range(1500):
                        clock.advance(intervals[tick % len(intervals)])
                        control.tick()
                        if control.state in ("ready", "paused", "fault"):
                            break
                    self.assertEqual(control.state, "ready", control.message)
                    self.assertLess(clock(), deadline)
                    self.assertEqual(control.tracking_deg, 3)
                    self.assertEqual(control.arrival_deg, 0.25)
                    self.assertEqual(control.tracker.settings.max_correction_deg, 12)
                    self.assertFalse(self.plan["hardware_qualified"])
                    self.check_dispatched_trace(control)
                    final = control.paths["approach"].positions[-1]
                    np.testing.assert_allclose(arm.target, final, atol=1e-9)
                    self.assertLessEqual(max(abs(arm.q - final)), 0.25)
                    # Ready retains the exact endpoint; no new offset is held.
                    for _ in range(20):
                        clock.advance(0.02)
                        control.tick()
                    np.testing.assert_allclose(arm.target, final, atol=1e-9)
                    self.assertTrue(arm.enabled)
                finally:
                    control.close()

    def check_dispatched_trace(self, control):
        previous = None
        for row in control.trace:
            self.assertEqual(row["dispatch_state"], "sent")
            self.assertLess(max(abs(np.array(row["tracking_error_deg"]))), 3)
            self.assertLessEqual(max(abs(np.array(row["tracker"]["correction_deg"]))), 12 + 1e-9)
            velocity = np.array(row["tracker"]["command_rate_deg_s"])
            self.assertLessEqual(max(abs(velocity)), 30 + 1e-9)
            key = "home" if row["stage"] == "initial" else "approach"
            low, high = control.paths[key].position_bounds()
            command = np.array(row["command_raw_deg"])
            self.assertTrue(np.all(command >= low - 1e-9))
            self.assertTrue(np.all(command <= high + 1e-9))
            if previous is not None and previous["stage"] == row["stage"]:
                dt = row["time_s"] - previous["time_s"]
                old_velocity = np.array(previous["tracker"]["command_rate_deg_s"])
                self.assertLessEqual(max(abs(velocity - old_velocity)) / dt, 30 + 1e-7)
                delta = np.array(row["command_raw_deg"]) - previous["command_raw_deg"]
                np.testing.assert_allclose(delta / dt, velocity, atol=1e-8)
                self.assertGreaterEqual(row["path_elapsed_s"], previous["path_elapsed_s"])
            previous = row


if __name__ == "__main__":
    unittest.main()
