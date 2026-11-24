"""Weld command sampling under recorded timing; no transport or plant claims."""

import source_paths
import json
from pathlib import Path
import unittest

import numpy as np

from controls.controller.tracking import PositionTracker, TrackingSettings
from controls.controller.weld_plan import JointPath
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from test_weld_hardware import Clock, FakeChassis, simple_plan


FIXTURE = Path(__file__).parent / "fixtures/weld_command_sampling_20261124.json"


class CommandSamplingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads(FIXTURE.read_text())
        cls.path = JointPath.from_document(cls.fixture["weld"])

    def replay(self, times):
        tracker = PositionTracker()
        tracker.reset(self.path.positions[0])
        previous_time, previous = 0.0, tracker.command.copy()
        previous_rate = np.zeros(6)
        previous_dt = None
        speed = np.array(self.fixture["limits"]["speed_deg_s"])
        acceleration = np.array(self.fixture["limits"]["acceleration_deg_s2"])
        references, commands = [], []
        for elapsed in times:
            dt = elapsed - previous_time
            reference, rate = self.path.sample(elapsed % self.path.duration)
            # Deliberately offset feedback: disabled corrections must not alter
            # the planned small motions or the A1 shaper-only reference.
            command = tracker.step(
                reference, rate, reference - 0.1, np.zeros(6), dt,
                speed, acceleration, adapt=False,
                correction_mask=np.zeros(6, dtype=bool), sampled_reference=True,
            )
            command_rate = (command - previous) / dt
            self.assertTrue(np.all(abs(command_rate) <= speed + 1e-8))
            acceleration_dt = dt if previous_dt is None else (previous_dt + dt) / 2
            self.assertTrue(np.all(abs(command_rate - previous_rate) <= acceleration * acceleration_dt + 1e-8))
            np.testing.assert_array_equal(tracker.integral, np.zeros(6))
            references.append(reference)
            commands.append(command)
            previous_time, previous, previous_rate = elapsed, command, command_rate
            previous_dt = dt
        return np.array(references), np.array(commands)

    def test_recorded_ticks_remove_command_inflation_on_all_uncompensated_axes(self):
        times = np.array(self.fixture["elapsed_s"])
        window = (times >= 2 * self.path.duration) & (times <= 22 * self.path.duration)
        old = np.array(self.fixture["old_a1_command_deg"])[window]
        self.assertAlmostEqual(np.ptp(old), 3.4429419596804394, places=9)
        reference, command = self.replay(times)
        np.testing.assert_allclose(command, reference, atol=1e-10, rtol=0)
        self.assertAlmostEqual(np.ptp(command[window, 0]), 2.260654781825963, places=9)
        low, high = self.path.position_bounds()
        self.assertTrue(np.all(command >= low - 1e-10))
        self.assertTrue(np.all(command <= high + 1e-10))

    def test_nominal_path_and_cycle_seams_at_fixed_and_irregular_cadence(self):
        duration = 12 * self.path.duration
        for period in (0.005, 0.01, 0.02, 0.04, 0.065, 0.1):
            with self.subTest(period=period):
                # Stay inside the existing 100 ms input interval contract,
                # including representational noise at its upper boundary.
                times = np.arange(period, duration, period * (1 - 1e-12))
                reference, command = self.replay(times)
                np.testing.assert_allclose(command, reference, atol=1e-9, rtol=0)
        times = np.cumsum(np.resize([0.01, 0.065, 0.02, 0.04], 240))
        reference, command = self.replay(times)
        np.testing.assert_allclose(command, reference, atol=1e-9, rtol=0)

    def test_correction_still_uses_feedback_and_respects_derivative_limits(self):
        tracker = PositionTracker(TrackingSettings(lead_time_s=(0.5,) * 6))
        tracker.reset(np.zeros(6))
        previous, previous_rate = np.zeros(6), np.zeros(6)
        elapsed = 0.0
        previous_dt = None
        projected = []

        def project(reference, target):
            projected.append(target.copy())
            return reference + np.clip(target - reference, -0.2, 0.2)

        for tick in range(200):
            dt = (0.01, 0.065, 0.02, 0.04)[tick % 4]
            elapsed += dt
            reference = np.full(6, 4 * np.sin(elapsed * 3))
            rate = np.full(6, 12 * np.cos(elapsed * 3))
            command = tracker.step(
                reference, rate, previous - 0.3, previous_rate,
                dt, [5] * 6, [20] * 6, adapt=False,
                correction_mask=np.ones(6, dtype=bool),
                sampled_reference=True, project=project,
            )
            command_rate = (command - previous) / dt
            self.assertLessEqual(max(abs(command_rate)), 5 + 1e-9)
            acceleration_dt = dt if previous_dt is None else (previous_dt + dt) / 2
            self.assertLessEqual(max(abs(command_rate - previous_rate)), 20 * acceleration_dt + 1e-8)
            self.assertLessEqual(max(abs(tracker.integral)), 0.5)
            previous, previous_rate = command, command_rate
            previous_dt = dt
        self.assertEqual(len(projected), 200)
        self.assertTrue(tracker.last["limited"])
        self.assertTrue(tracker.last["projected"])
        self.assertGreater(max(abs(np.array(tracker.last["target_correction_deg"]))), 0.2)
        for _ in range(1000):
            tracker.step([0] * 6, [0] * 6, [0] * 6, [0] * 6,
                         0.02, [5] * 6, [20] * 6, endpoint=True,
                         sampled_reference=True)
            if tracker.at_endpoint(np.zeros(6)):
                break
        self.assertTrue(tracker.at_endpoint(np.zeros(6)))
        np.testing.assert_array_equal(tracker.integral, np.zeros(6))

    def test_reset_and_zero_interval_do_not_carry_an_old_reference(self):
        tracker = PositionTracker()
        tracker.reset([100] * 6)
        mask = np.zeros(6, dtype=bool)
        tracker.step([101] * 6, [10] * 6, [100] * 6, [0] * 6,
                     0.05, [100] * 6, [2000] * 6,
                     sampled_reference=True, correction_mask=mask)
        tracker.reset([-20] * 6)
        result = tracker.step([50] * 6, [1] * 6, [-20] * 6, [0] * 6,
                              0, [100] * 6, [2000] * 6,
                              sampled_reference=True, correction_mask=mask)
        np.testing.assert_array_equal(result, [-20] * 6)
        result = tracker.step([-19.9] * 6, [2] * 6, [-20] * 6, [0] * 6,
                              0.05, [100] * 6, [2000] * 6,
                              sampled_reference=True, correction_mask=mask)
        np.testing.assert_allclose(result, [-19.9] * 6, atol=1e-12)

    def test_controller_continuous_and_finite_runs_keep_guards_and_fixed_holds(self):
        for continuous in (False, True):
            with self.subTest(continuous=continuous):
                clock = Clock()
                arm = SimulatedArm([0] * 6, clock=clock)
                chassis = FakeChassis()
                controller = WeldController(
                    arm, chassis, clock=clock, pose_check=lambda *a, **k: None,
                    continuous_weld=continuous,
                    tracking_settings=TrackingSettings(minimum_weld_travel_deg=1),
                )
                self.addCleanup(controller.close)
                controller.select(True, True)
                controller.load_plan(simple_plan())
                controller.go_initial()
                for _ in range(500):
                    clock.advance()
                    controller.tick()
                    if controller.state in ("ready", "paused", "fault"):
                        break
                self.assertEqual(controller.state, "ready", controller.message)
                self.assertTrue(all(row["tracker"]["shaping_policy"] ==
                                    "velocity_reference_pursuit" for row in controller.trace))
                controller.start_weld()
                for tick in range(150):
                    clock.advance((0.02, 0.04, 0.065)[tick % 3])
                    controller.tick()
                    if controller.state != "welding":
                        break
                self.assertEqual(controller.state, "welding" if continuous else "complete",
                                 controller.message)
                weld = [row for row in controller.trace if row["stage"] == "welding"]
                self.assertTrue(weld)
                for row in weld:
                    np.testing.assert_allclose(row["command_raw_deg"], row["requested_raw_deg"], atol=1e-10)
                self.assertEqual(controller.tracking_deg, 3)
                self.assertEqual(controller.arrival_deg, 0.25)
                if continuous:
                    self.assertGreater(weld[-1]["weld_repeat_index"], 3)
                    clock.advance(0.11)
                    controller.tick()
                    self.assertEqual(controller.state, "paused")
                    self.assertIn("Control timing gap", controller.message)
                held = arm.target.copy()
                for _ in range(10):
                    clock.advance()
                    controller.tick()
                np.testing.assert_array_equal(arm.target, held)
                self.assertTrue(arm.enabled)
                self.assertEqual(chassis.last_feed, 0)


if __name__ == "__main__":
    unittest.main()
