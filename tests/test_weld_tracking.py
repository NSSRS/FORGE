"""Closed-loop tracking and measured Hz with synthetic plants; no hardware."""

import source_paths
import json
from pathlib import Path
import unittest

import numpy as np

from controls.controller.motion_feedback import MotionFrequency
from controls.controller.tracking import PositionTracker, TrackingSettings
from controls.controller.weld_plan import JointPath
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from test_weld_hardware import Clock, FakeChassis
from test_weld_preparation import preparation_plan


class LaggedArm(SimulatedArm):
    """Unequal first-order responses with optional one-tick transport delay."""

    lag = np.array([0.16, 0.76, 0.27, 0.14, 0.22, 0.25])
    delayed = False

    def poll(self):
        dt = self.clock() - self.last
        if dt <= 0:
            return
        self.last = self.clock()
        target = getattr(self, "previous_target", self.target) if self.delayed else self.target
        self.previous_target = self.target.copy()
        self.velocity = (np.clip((target - self.q) / self.lag, -self.speed, self.speed)
                         if self.enabled else np.zeros(6))
        self.q += self.velocity * dt


class TrackingTests(unittest.TestCase):
    def controller(self, pid=True, plant=LaggedArm, *, a1_trial=None):
        clock = Clock()
        arm = plant([0] * 6, clock=clock)
        control = WeldController(
            arm, FakeChassis(), clock=clock, pose_check=lambda *a, **k: None,
            continuous_weld=True,
            tracking_settings=TrackingSettings(weld_a1_shaper_only=bool(a1_trial)) if pid else None,
        )
        plan = preparation_plan()
        if a1_trial is not None:
            plan["chassis_enabled"] = False
            plan["weld"]["raw_deg"][1][0] += 0.10
            plan["weld"]["raw_deg"][1][5] += 0.10
        control.select(True, plan["chassis_enabled"])
        control.load_plan(plan)
        control.go_initial()
        return clock, arm, control

    def run_to_ready(self, clock, control, dt=0.02):
        for _ in range(2000):
            clock.advance(dt)
            control.tick()
            if control.state in ("ready", "paused", "fault"):
                break
        self.assertEqual(control.state, "ready", control.message)

    def test_pid_with_bounded_clock_tracks_unequal_lag_faster_than_pacing(self):
        times = []
        for pid in (False, True):
            clock, arm, control = self.controller(pid)
            self.run_to_ready(clock, control)
            times.append(clock())
            if pid:
                scales = [r["path_clock_scale"] for r in control.trace]
                self.assertTrue(all(0 <= scale <= 1 for scale in scales))
                self.assertLess(min(scales), 1)
                self.assertTrue(any(max(abs(np.array(r["tracker"]["correction_deg"]))) > 1
                                    for r in control.trace))
                self.assertLess(max(max(abs(np.array(r["tracking_error_deg"])))
                                    for r in control.trace), 3)
                np.testing.assert_allclose(arm.target, control.paths["weld"].positions[0], atol=1e-12)
                self.assertEqual(control.tracking_deg, 3)
                self.assertEqual(control.arrival_deg, 0.25)
            control.close()
        self.assertLess(times[1], times[0] * 0.8, times)

    def test_delayed_drive_hold_disturbance_and_continuous_pause(self):
        class Delayed(LaggedArm):
            delayed = True
        clock, arm, control = self.controller(plant=Delayed)
        self.run_to_ready(clock, control)
        target, allowance = arm.target.copy(), arm.speed.copy()
        arm.q += 0.15
        for _ in range(500):
            clock.advance()
            control.tick()
        np.testing.assert_array_equal(arm.target, target)
        np.testing.assert_array_equal(arm.speed, allowance)
        self.assertTrue(arm.enabled)
        np.testing.assert_allclose(arm.q, target, atol=0.01)
        control.start_weld()
        lead = control.tracker.lead_time.copy()
        for _ in range(500):
            clock.advance()
            control.tick()
        self.assertEqual(control.state, "welding", control.message)
        np.testing.assert_array_equal(control.tracker.lead_time, lead)
        control.pause()
        target = arm.target.copy()
        for _ in range(500):
            clock.advance()
            control.tick()
        control.pause()
        self.assertTrue(arm.enabled)
        np.testing.assert_array_equal(arm.target, target)
        self.assertEqual(control.chassis.last_feed, 0)

    def test_a1_shaper_only_retains_preparation_and_delayed_weld_guards(self):
        class Delayed(LaggedArm):
            delayed = True

        preparation = []
        for trial in (False, True):
            with self.subTest(trial=trial):
                clock, arm, control = self.controller(plant=Delayed, a1_trial=trial)
                self.addCleanup(control.close)
                self.run_to_ready(clock, control)
                preparation.append([row["command_raw_deg"] for row in control.trace])
                control.start_weld()
                for _ in range(150):
                    clock.advance()
                    control.tick()
                self.assertEqual(control.state, "welding", control.message)
                weld = [row for row in control.trace if row["stage"] == "welding"]
                self.assertTrue(weld)
                corrections = np.array([row["tracker"]["target_correction_deg"] for row in weld])
                self.assertEqual(control.weld_correction_mask[0], not trial)
                self.assertTrue(control.weld_correction_mask[5])
                if trial:
                    np.testing.assert_array_equal(corrections[:, 0], 0)
                    commands = np.array([row["command_raw_deg"] for row in weld])
                    reference = np.array([row["requested_raw_deg"] for row in weld])
                    np.testing.assert_allclose(commands[:, 0], reference[:, 0], atol=1e-12)
                    self.assertTrue(all(row["tracker"]["shaping_policy"] ==
                                        "sampled_reference_increment" for row in weld))
                else:
                    self.assertGreater(np.max(abs(corrections[:, 0])), 0)
                self.assertEqual(control.tracking_deg, 3.0)
                self.assertEqual(control.arrival_deg, 0.25)
        np.testing.assert_array_equal(preparation[0], preparation[1])

    def test_mismatched_response_reaches_ready_with_unchanged_tracking_trip(self):
        # The controller's initial estimates must not have to match the plant.
        for factor in (0.5, 1.2, 1.5):
            with self.subTest(response_time_factor=factor):
                class Mismatched(LaggedArm):
                    lag = LaggedArm.lag * factor
                clock, arm, control = self.controller(plant=Mismatched)
                try:
                    for _ in range(2000):
                        clock.advance()
                        control.tick()
                        if control.state in ("ready", "paused", "fault"):
                            break
                    self.assertEqual(control.state, "ready", control.message)
                    self.assertLess(max(max(abs(np.array(row["tracking_error_deg"])))
                                        for row in control.trace), 3)
                    control.start_weld()
                    for _ in range(500):
                        clock.advance()
                        control.tick()
                    self.assertEqual(control.state, "welding", control.message)
                    self.assertTrue(all(row["path_clock_scale"] == 1
                                        for row in control.trace if row["stage"] == "welding"))
                    self.assertTrue(arm.enabled)
                    self.assertEqual(control.tracking_deg, 3)
                finally:
                    control.close()

    def test_no_motion_catchup_and_stale_feedback_fault(self):
        clock, arm, control = self.controller()
        self.run_to_ready(clock, control)
        control.start_weld()
        clock.advance(0.11)
        control.tick()
        self.assertEqual(control.state, "paused")
        self.assertTrue(arm.enabled)
        clock, arm, control = self.controller()
        read = arm.read
        def stale():
            result = read()
            result.age_ms[:] = 80
            return result
        arm.read = stale
        clock.advance()
        control.tick()
        self.assertEqual(control.state, "fault")
        self.assertFalse(arm.enabled)

    def test_jammed_shaft_pauses_at_existing_progress_deadline(self):
        class Jammed(LaggedArm):
            def poll(self):
                self.last = self.clock()
                self.velocity[:] = 0
        clock, arm, control = self.controller(plant=Jammed)
        for _ in range(800):
            clock.advance()
            control.tick()
            if control.state in ("paused", "fault"):
                break
        self.assertEqual(control.state, "paused", control.message)
        self.assertIn("Preparation stalled A2: no 0.1 deg net progress for 10 s", control.message)
        self.assertLess(clock(), 12)
        self.assertTrue(arm.enabled)
        self.assertEqual(control.tracking_deg, 3)
        np.testing.assert_array_equal(arm.target, arm.q)

    def test_displaced_shaft_still_pauses_at_three_degree_tracking_guard(self):
        clock, arm, control = self.controller()
        self.addCleanup(control.close)
        for _ in range(25):
            clock.advance()
            control.tick()
        self.assertEqual(control.state, "initial")
        # A disturbance cannot be hidden by slowing the reference clock.
        arm.q[0] -= 5
        clock.advance()
        control.tick()
        self.assertEqual(control.state, "paused", control.message)
        self.assertIn("Tracking error A1", control.message)
        self.assertEqual(control.tracking_deg, 3)
        np.testing.assert_array_equal(arm.target, arm.q)

    def test_command_speed_acceleration_saturation_and_integral_reset(self):
        for dt in (0.01, 0.02, 0.04, 0.08, 0.1):
            tracker = PositionTracker()
            tracker.reset([0] * 6)
            previous, velocity = np.zeros(6), np.zeros(6)
            for _ in range(60):
                command = tracker.step([20]*6, [10]*6, [0]*6, [0]*6,
                                       dt, [8]*6, [30]*6)
                rate = (command - previous) / dt
                self.assertLessEqual(max(abs(rate)), 8 + 1e-9)
                self.assertLessEqual(max(abs(rate-velocity)) / dt, 30 + 1e-8)
                self.assertLessEqual(max(abs(tracker.integral)), 0.5)
                previous, velocity = command, rate
            # Endpoint settling must remove every residual correction.
            for _ in range(2000):
                tracker.step([20]*6, [0]*6, [20]*6, [0]*6,
                             dt, [8]*6, [30]*6, endpoint=True)
                if tracker.at_endpoint(np.full(6, 20)):
                    break
            self.assertTrue(tracker.at_endpoint(np.full(6, 20)))
            np.testing.assert_array_equal(tracker.integral, np.zeros(6))

    def test_invalid_settings_and_no_zero_interval_integral(self):
        for kwargs in (dict(kp=float("nan")), dict(ki_per_s=-1), dict(lead_time_s=(1,)),
                       dict(kd_s=1), dict(velocity_filter_s=0)):
            with self.assertRaises(ValueError):
                TrackingSettings(**kwargs)
        tracker = PositionTracker()
        tracker.reset([1]*6)
        for dt in (0, 0):
            np.testing.assert_array_equal(
                tracker.step([2]*6, [3]*6, [1]*6, [0]*6, dt, [5]*6, [30]*6), [1]*6)
        np.testing.assert_array_equal(tracker.integral, [0]*6)

    def test_small_weld_axes_keep_reference_without_lead_or_integral(self):
        tracker = PositionTracker()
        tracker.reset([0]*6)
        mask = np.array([True, False, False, False, False, True])
        for _ in range(50):
            tracker.step([0.1]*6, [0.2]*6, [-0.1]*6, [0]*6,
                         0.02, [5]*6, [30]*6, adapt=False, correction_mask=mask)
        np.testing.assert_array_equal(np.array(tracker.last["target_correction_deg"])[~mask], 0)
        np.testing.assert_array_equal(tracker.integral[~mask], 0)
        self.assertGreater(tracker.last["target_correction_deg"][5], 0)
        self.assertEqual(tracker.last["correction_axes"], [1, 6])
        with self.assertRaisesRegex(ValueError, "six booleans"):
            tracker.step([0]*6, [0]*6, [0]*6, [0]*6, .02, [5]*6, [30]*6,
                         correction_mask=[1]*6)

    def test_finite_extrema_braking_with_variable_ticks_and_exact_hold(self):
        tracker = PositionTracker()
        direction = np.array([1, -1, 1, -1, 0, 1])
        start, end = np.zeros(6), 12 * direction
        bounds = np.minimum(start, end), np.maximum(start, end)
        tracker.reset(start)
        previous, previous_rate = start.copy(), start.copy()
        # Alternate 10–100 ms intervals while requesting excessive lead in
        # both directions. A constant joint must remain exactly stationary.
        for tick in range(1200):
            dt = (0.01, 0.02, 0.08, 0.1, 0.04)[tick % 5]
            endpoint = tick >= 100
            command = tracker.step(
                end, np.zeros(6) if endpoint else 25 * direction,
                previous, previous_rate, dt, [40]*6, [30]*6,
                endpoint=endpoint, position_bounds=bounds,
            )
            rate = (command - previous) / dt
            self.assertTrue(np.all(command >= bounds[0] - 1e-9))
            self.assertTrue(np.all(command <= bounds[1] + 1e-9))
            self.assertLessEqual(max(abs(rate)), 40 + 1e-9)
            self.assertLessEqual(max(abs(rate - previous_rate)) / dt, 30 + 1e-7)
            previous, previous_rate = command, rate
            if endpoint and tracker.at_endpoint(end):
                break
        self.assertTrue(tracker.at_endpoint(end))
        np.testing.assert_array_equal(tracker.integral, np.zeros(6))

    def test_reference_braking_keeps_lead_inside_correction_envelope(self):
        tracker = PositionTracker(TrackingSettings(lead_time_s=(1,) * 6))
        tracker.reset([0] * 6)
        elapsed, previous_velocity = 0.0, np.zeros(6)
        for tick in range(300):
            dt = (0.01, 0.02, 0.08, 0.099)[tick % 4]
            elapsed += dt
            if elapsed < 1:
                reference, rate = 15 * elapsed**2, 30 * elapsed
            elif elapsed < 2:
                u = elapsed - 1
                reference, rate = 15 + 30 * u - 15 * u**2, 30 - 30 * u
            else:
                reference, rate = 30, 0
            # A valid 30 deg/s^2 reference brakes while substantial command
            # lead is still moving. Finite path extrema alone are too far
            # away to stop the old shaper exceeding 20 degrees of correction.
            command = tracker.step(
                [reference] * 6, [rate] * 6, [reference - 1] * 6, [rate] * 6,
                dt, [30] * 6, [30] * 6, adapt=False,
                position_bounds=([0] * 6, [80] * 6),
            )
            self.assertLessEqual(max(abs(command - reference)), 12 + 1e-9)
            self.assertLessEqual(max(abs(tracker.velocity - previous_velocity)) / dt, 30 + 1e-8)
            previous_velocity = tracker.velocity.copy()

    def test_recorded_approach_commands_brake_inside_checked_extrema(self):
        fixture = json.loads((Path(__file__).parent /
                              "fixtures/weld_tracking_overshoot_20261117.json").read_text())
        path = JointPath.from_document(fixture["plan"]["approach"])
        bounds = path.position_bounds()
        tracker = PositionTracker()
        tracker.reset(path.positions[0])
        tracker.lead_time[:] = fixture["approach_initial_lead_time_s"]
        old_commands = np.array([row["command_raw_deg"] for row in fixture["approach_feedback"]])
        # This real input caught the old A6 target going beyond the weld start
        # before the reference had finished approaching it.
        self.assertLess(old_commands[:, 5].min(), bounds[0][5] - 1)
        elapsed, previous, previous_rate = 0.0, tracker.command.copy(), np.zeros(6)
        for row in fixture["approach_feedback"]:
            dt = row["path_elapsed_s"] - elapsed
            elapsed = row["path_elapsed_s"]
            reference, rate = path.sample(elapsed)
            command = tracker.step(
                reference, rate, row["measured_raw_deg"], row["measured_velocity_deg_s"],
                dt, fixture["plan"]["raw_speed_limits_deg_s"], [30]*6,
                remaining_s=path.duration - elapsed, position_bounds=bounds,
            )
            command_rate = (command - previous) / dt
            self.assertTrue(np.all(command >= bounds[0] - 1e-9))
            self.assertTrue(np.all(command <= bounds[1] + 1e-9))
            self.assertLessEqual(max(abs(command_rate - previous_rate)) / dt, 30 + 1e-7)
            previous, previous_rate = command, command_rate

    def test_ordered_recovery_lead_stays_within_current_leg(self):
        from controls.controller.recovery import recovery_path
        from controls.controller.weld_plan import Settings, move_path
        from test_weld_hardware import simple_plan
        from test_weld_recovery import Obstacle
        home, recovery = recovery_path(Obstacle(), [0]*6, [1, 1, 0, 0, 0, 0], Settings())
        plan = simple_plan()
        plan.update(home=home.document(), recovery=recovery,
                    initial_raw_deg=home.positions[-1].tolist())
        plan["approach"] = move_path(home.positions[-1], [0.6]*6, Settings()).document()
        clock = Clock()
        arm = LaggedArm([0]*6, clock=clock)
        control = WeldController(arm, FakeChassis(), clock=clock,
                                 pose_check=lambda *a, **k: None,
                                 tracking_settings=TrackingSettings())
        control.select(True, True)
        control.load_plan(plan)
        control.go_initial()
        read = arm.read
        def shifted():
            feedback = read()
            # Small measured error would invite a PID correction on the axis
            # that is supposed to hold until A2 finishes its clearance leg.
            feedback.raw_deg[0] -= 0.15
            return feedback
        arm.read = shifted
        try:
            for _ in range(500):
                clock.advance()
                control.tick()
                if control.recovery_index or control.state in ("paused", "fault"):
                    break
            self.assertEqual(control.state, "initial", control.message)
            self.assertEqual(control.recovery_index, 1)
            self.assertTrue(control.trace)
            self.assertTrue(all(row["command_raw_deg"][0] == 0 for row in control.trace))
            clock.advance()
            control.tick()
            low, high = control.preparation["home"].position_bounds
            self.assertEqual(low[1], 1)
            self.assertEqual(high[1], 1)
        finally:
            control.close()



class FrequencyTests(unittest.TestCase):
    def test_recorded_a6_two_cycle_response_is_not_half_travel(self):
        fixture = json.loads((Path(__file__).parent /
                              "fixtures/weld_small_motion_20261117.json").read_text())
        measured, commands = MotionFrequency(), MotionFrequency()
        for row in fixture["steady_weld_feedback"]:
            measured.update(row["time_s"], row["measured_raw_deg"], row["age_ms"])
            commands.update(row["time_s"], row["command_raw_deg"], [0]*6)
        self.assertAlmostEqual(measured.result[5]["hz"], fixture["planned_hz"] / 2, delta=0.01)
        self.assertAlmostEqual(commands.result[5]["hz"], fixture["planned_hz"], delta=0.01)
        self.assertGreater(measured.result[5]["peak_to_peak_deg"], 0.4)
        for axis in range(1, 5):
            self.assertIsNone(measured.result[axis]["hz"])

    def test_encoder_frequency_not_command_frequency_and_small_motion(self):
        meter = MotionFrequency()
        for t in np.arange(0, 8, 0.02):
            wave = np.sin(2*np.pi*1.73*t)
            # A6 repeats every other command cycle, with a stronger harmonic.
            double = 0.2*np.sin(2*np.pi*0.865*t) + 0.4*wave
            meter.update(t, [wave, 0, 0.02*wave, 0.03*wave, wave, double], [4]*6)
        self.assertAlmostEqual(meter.result[0]["hz"], 1.73, delta=0.02)
        self.assertAlmostEqual(meter.result[5]["hz"], 0.865, delta=0.02)
        for axis in (1, 2, 3):
            self.assertIsNone(meter.result[axis]["hz"])
            self.assertEqual(meter.result[axis]["status"], "insufficient motion")
        self.assertAlmostEqual(meter.result[0]["peak_to_peak_deg"], 2, delta=0.01)

    def test_stale_gap_reset_no_old_estimate_or_unbounded_history(self):
        meter = MotionFrequency()
        for t in np.arange(0, 12, 0.02):
            meter.update(t, [np.sin(t*2*np.pi*2)]*6, [0]*6)
        self.assertIsNotNone(meter.result[0]["hz"])
        self.assertLessEqual(len(meter.samples[0]), 401)
        meter.update(12.2, [0]*6, [100]*6)
        self.assertTrue(all(r["hz"] is None for r in meter.result))
        meter.reset()
        meter.update(13, [0]*6, [0]*6)
        self.assertTrue(all(r["hz"] is None for r in meter.result))


class ModelTrackingTests(unittest.TestCase):
    def test_hardware_space_warnings_cannot_throttle_a6_tracking_projection(self):
        from controls.controller.weld_plan import WeldGeometry
        from motors.arm_calibration import CalibrationMap
        root = Path(__file__).parent / "fixtures"
        samples = json.loads((root / "weld_small_motion_20261117.json").read_text())
        fixture = json.loads((root / samples["mapping_fixture"]).read_text())
        mapping = CalibrationMap(fixture["mapping"])
        geometry = WeldGeometry(mapping, enforce_spatial_clearance=False)
        strict = WeldGeometry(mapping)
        clock = Clock()
        arm = LaggedArm(fixture["plan"]["current_raw_deg"], clock=clock)
        control = WeldController(arm, clock=clock, pose_check=geometry.check,
                                 allow_optimized_preparation=True, continuous_weld=True,
                                 tracking_settings=TrackingSettings())
        control.select(True, False)
        control.load_plan(fixture["plan"])
        np.testing.assert_array_equal(control.weld_correction_mask, [True, False, False, False, False, True])
        self.assertTrue(samples["blocked_correction_samples"])
        for row in samples["blocked_correction_samples"]:
            reference = np.array(row["requested_raw_deg"])
            correction = np.array(row["tracker"]["target_correction_deg"])
            # Preserve the historical block as evidence, then exercise the
            # hardware policy: spatial warnings cannot throttle any correction.
            with self.assertRaises(ValueError):
                strict.check(reference + correction, clearance_mm=1.8)
            self.assertTrue(geometry.check(reference + correction, clearance_mm=1.8))
            control.tracking_projection_scale = 1.0
            wanted = reference + correction * control.weld_correction_mask
            projected = control.project_tracking_target(reference, wanted, recovery=False, dt=.02)
            np.testing.assert_array_equal(projected, wanted)
            self.assertEqual(control.tracking_projection_scale, 1)
        control.close()

    def test_overshoot_route_reaches_ready_holds_and_can_start_welding(self):
        from controls.controller.weld_plan import WeldGeometry
        from motors.arm_calibration import CalibrationMap
        fixture = json.loads((Path(__file__).parent /
                              "fixtures/weld_tracking_overshoot_20261117.json").read_text())
        plan = fixture["plan"]
        geometry = WeldGeometry(CalibrationMap(fixture["mapping"]))
        clock = Clock()
        arm = LaggedArm(plan["current_raw_deg"], clock=clock)
        control = WeldController(arm, clock=clock, pose_check=geometry.check,
                                 allow_optimized_preparation=True, continuous_weld=True,
                                 tracking_settings=TrackingSettings())
        try:
            control.select(True, False)
            control.load_plan(plan)
            control.go_initial()
            for _ in range(1500):
                clock.advance()
                control.tick()
                if control.state in ("ready", "paused", "fault"):
                    break
            self.assertEqual(control.state, "ready", control.message)
            for row in control.trace:
                key = "home" if row["stage"] == "initial" else "approach"
                low, high = control.paths[key].position_bounds()
                self.assertTrue(np.all(np.array(row["command_raw_deg"]) >= low - 1e-9))
                self.assertTrue(np.all(np.array(row["command_raw_deg"]) <= high + 1e-9))
            target, allowance = arm.target.copy(), arm.speed.copy()
            for _ in range(500):
                clock.advance()
                control.tick()
            self.assertEqual(control.state, "ready", control.message)
            self.assertTrue(arm.enabled)
            np.testing.assert_allclose(target, control.paths["weld"].positions[0], atol=1e-9)
            np.testing.assert_array_equal(arm.target, target)
            np.testing.assert_array_equal(arm.speed, allowance)
            control.start_weld()
            for _ in range(150):
                clock.advance()
                control.tick()
            self.assertEqual(control.state, "welding", control.message)
        finally:
            control.close()

    def test_recorded_route_with_unequal_lag_full_clearance_and_repeated_weld(self):
        from controls.controller.weld_plan import WeldGeometry
        from motors.arm_calibration import CalibrationMap
        fixture = json.loads((Path(__file__).parent / "fixtures/weld_tracking_20261117.json").read_text())
        plan = fixture["plan"]
        geometry = WeldGeometry(CalibrationMap(fixture["mapping"]))
        times = []
        for pid in (False, True):
            clock = Clock()
            arm = LaggedArm(plan["current_raw_deg"], clock=clock)
            control = WeldController(arm, clock=clock, pose_check=geometry.check,
                                     allow_optimized_preparation=True, continuous_weld=True,
                                     tracking_settings=TrackingSettings() if pid else None)
            control.select(True, False)
            control.load_plan(plan)
            control.go_initial()
            for _ in range(1500):
                clock.advance()
                control.tick()
                if control.state in ("ready", "paused", "fault"):
                    break
            self.assertEqual(control.state, "ready", control.message)
            times.append(clock())
            if pid:
                target = arm.target.copy()
                for _ in range(50):
                    clock.advance()
                    control.tick()
                np.testing.assert_array_equal(arm.target, target)
                control.start_weld()
                for _ in range(1000):
                    clock.advance()
                    control.tick()
                    if control.state != "welding":
                        break
                self.assertEqual(control.state, "welding", control.message)
                self.assertGreater(control.trace[-1]["weld_repeat_index"], 30)
                self.assertTrue(any(r["tracker"]["projected"] for r in control.trace))
                control.pause()
                target = arm.target.copy()
                for _ in range(500):
                    clock.advance()
                    control.tick()
                self.assertTrue(arm.enabled)
                np.testing.assert_array_equal(arm.target, target)
            control.close()
        self.assertLess(times[1], times[0] * 0.75, times)


if __name__ == "__main__":
    unittest.main()
