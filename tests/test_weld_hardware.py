"""Supported weld-motion contracts; fake ports only, never motor hardware."""

import source_paths
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
from controls.hardware.weld_plan import (
    JointPath,
    Settings,
    move_path,
    initial_from_model,
    speed_report,
)
from controls.hardware.weld_control import WeldController, chassis_only_plan
from controls.hardware.weld_ports import SimulatedArm, ChassisOwner
from motors.arm_calibration import CalibrationMap

ROOT = Path(__file__).resolve().parents[1]


class Clock:
    value = 0.0

    def __call__(self):
        return self.value

    def advance(self, dt=0.02):
        self.value += dt


class FakeChassis:
    def __init__(self):
        self.feeds = []
        self.last_feed = 0.0
        self.closed = False
        self.error = False

    def velocity(self, value):
        self.last_feed = value
        self.feeds.append(value)

    def stop(self):
        self.last_feed = 0.0
        self.feeds.append(0.0)

    def check(self):
        if self.error:
            raise RuntimeError("link fault")

    def close(self):
        self.stop()
        self.closed = True


def simple_plan(arm=True, chassis=True):
    home = move_path(np.zeros(6), np.full(6, 0.3), Settings())
    approach = move_path(np.full(6, 0.3), np.full(6, 0.6), Settings())
    weld = JointPath([0.0, 0.5, 1.0], [[0.6] * 6, [0.9] * 6, [0.6] * 6])
    return dict(
        schema="forge.weld_motion_plan",
        version=1,
        arm_enabled=arm,
        chassis_enabled=chassis,
        home=home.document(),
        approach=approach.document(),
        weld=weld.document(),
        feed_m_s=0.001,
        max_speed_deg_s=5.0,
        max_accel_deg_s2=30.0,
    )


class WeldControlTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.arm = SimulatedArm([0] * 6, clock=self.clock)
        self.chassis = FakeChassis()
        self.control = WeldController(self.arm, self.chassis, clock=self.clock)

    def tick(self, n=1):
        for _ in range(n):
            self.clock.advance()
            self.control.tick()

    def ready(self, chassis=True):
        self.control.select(True, chassis)
        self.control.load_plan(simple_plan(chassis=chassis))
        self.control.go_initial()
        for _ in range(500):
            self.tick()
            if self.control.state == "ready":
                return
        self.fail(self.control.message)

    def test_two_arrival_stages_then_explicit_weld(self):
        self.ready()
        self.assertTrue(all(v == 0 for v in self.chassis.feeds))
        self.tick(20)
        self.assertEqual(self.control.state, "ready")
        self.control.start_weld()
        self.tick(100)
        self.assertEqual(self.control.state, "complete", self.control.message)
        self.assertTrue(any(v > 0 for v in self.chassis.feeds))
        self.assertEqual(self.chassis.last_feed, 0)
        self.assertTrue(self.arm.enabled)

    def test_cannot_skip_initial_or_approach(self):
        self.control.select(True, True)
        with self.assertRaises(ValueError):
            self.control.start_weld()
        self.control.load_plan(simple_plan())
        self.control.go_initial()
        with self.assertRaises(ValueError):
            self.control.start_weld()

    def test_arm_only_does_not_feed(self):
        self.ready(chassis=False)
        self.control.start_weld()
        self.tick(100)
        self.assertEqual(self.control.state, "complete")
        self.assertTrue(all(v == 0 for v in self.chassis.feeds))

    def test_chassis_only_never_commands_arm(self):
        self.control.select(False, True)
        self.control.load_plan(chassis_only_plan(Settings(cycles=1)))
        self.control.go_initial()
        self.control.start_weld()
        self.tick(40)
        self.assertEqual(self.control.state, "complete")
        self.assertEqual(self.arm.commands, [])
        self.assertFalse(self.arm.enabled)

    def test_switch_off_invalidates_plan_and_pauses_both(self):
        self.ready()
        self.control.start_weld()
        self.tick(5)
        self.control.select(False, True)
        self.assertEqual(self.chassis.last_feed, 0)
        self.assertTrue(self.arm.enabled)
        self.assertIsNone(self.control.plan)
        before = self.arm.q.copy()
        self.tick(20)
        np.testing.assert_allclose(before, self.arm.q)
        with self.assertRaises(ValueError):
            self.control.start_weld()

    def test_control_gap_never_catches_up(self):
        self.ready()
        self.control.start_weld()
        self.tick(3)
        self.clock.advance(0.11)
        self.control.tick()
        self.assertEqual(self.control.state, "paused")
        self.assertIsNone(self.control.plan)
        self.assertEqual(self.chassis.last_feed, 0)

    def test_stale_feedback_faults_both_and_no_auto_resume(self):
        self.ready()
        self.control.start_weld()
        original = self.arm.read

        def stale():
            value = original()
            value.age_ms[:] = 100
            return value

        self.arm.read = stale
        self.tick()
        self.assertEqual(self.control.state, "fault")
        self.assertFalse(self.arm.enabled)
        self.assertEqual(self.chassis.last_feed, 0)
        self.arm.read = original
        self.tick()
        self.assertEqual(self.control.state, "fault")
        with self.assertRaises(RuntimeError):
            self.control.select(True, True)

    def test_link_fault_stops_arm(self):
        self.ready()
        self.chassis.error = True
        self.tick()
        self.assertEqual(self.control.state, "fault")
        self.assertFalse(self.arm.enabled)

    def test_tracking_error_pauses_feed(self):
        self.ready()
        self.control.start_weld()
        self.arm.q += 4
        self.arm.target += 4
        self.tick()
        self.assertEqual(self.control.state, "paused")
        self.assertEqual(self.chassis.last_feed, 0)

    def test_partial_or_fast_path_rejected_before_motion(self):
        self.control.select(True, True)
        plan = simple_plan()
        plan["weld"]["raw_deg"][0] = [0] * 5
        with self.assertRaises(ValueError):
            self.control.load_plan(plan)
        plan = simple_plan()
        plan["weld"]["times_s"] = [0, 0.001, 0.002]
        with self.assertRaises(ValueError):
            self.control.load_plan(plan)
        self.assertEqual(self.arm.commands, [])

    def test_changed_measured_start_requires_replan(self):
        self.control.select(True, True)
        self.control.load_plan(simple_plan())
        self.arm.q += 1
        self.arm.target += 1
        with self.assertRaises(ValueError):
            self.control.go_initial()

    def test_motor_specific_speed_limit_enforced_before_motion(self):
        self.control.select(True, True)
        plan = simple_plan()
        plan["raw_speed_limits_deg_s"] = [0.01, 5, 5, 5, 5, 5]
        with self.assertRaisesRegex(ValueError, "motor-specific"):
            self.control.load_plan(plan)
        self.assertEqual(self.arm.commands, [])

    def test_weld_profile_limits_are_stage_specific(self):
        self.control.select(True, True)
        plan = simple_plan()
        plan["limit_profile"] = "weld"
        plan["weld"] = JointPath(
            [0, 0.1, 0.2], [[0.6] * 6, [0.9, 1.6, 1.6, 1.6, 1.6, 1.6], [0.6] * 6]
        ).document()
        plan["optimization"] = {"dynamics": {"raw_acceleration_deg_s2": [1000] * 6}}
        plan["stage_limits"] = {
            key: dict(
                speed_deg_s=[5] * 6 if key != "weld" else [math.degrees(1.2)] * 6,
                acceleration_deg_s2=[30 if key != "weld" else 1000] * 6,
            )
            for key in ("home", "approach", "weld")
        }
        self.control.load_plan(plan)
        plan["stage_limits"]["weld"]["speed_deg_s"][0] = 69
        with self.assertRaises(ValueError):
            self.control.load_plan(plan)
        plan["stage_limits"]["weld"]["speed_deg_s"][0] = math.degrees(1.2)
        plan["stage_limits"]["home"]["speed_deg_s"] = [60] * 6
        with self.assertRaises(ValueError):
            self.control.load_plan(plan)
        plan["stage_limits"]["home"]["speed_deg_s"] = [5] * 6
        plan["stage_limits"]["weld"]["speed_deg_s"] = [86] * 6
        with self.assertRaises(ValueError):
            self.control.load_plan(plan)

    def test_switches_do_not_compensate_arm_targets(self):
        traces = []
        for enabled in (True, False):
            clock = Clock()
            arm = SimulatedArm([0] * 6, clock=clock)
            chassis = FakeChassis()
            control = WeldController(arm, chassis, clock=clock)
            control.select(True, enabled)
            control.load_plan(simple_plan(chassis=enabled))
            control.go_initial()
            for _ in range(400):
                clock.advance()
                control.tick()
                if control.state == "ready":
                    control.start_weld()
                if control.state == "complete":
                    break
            traces.append([row["requested_raw_deg"] for row in control.trace])
            self.assertEqual(control.state, "complete")
            control.close()
        np.testing.assert_allclose(traces[0], traces[1])

    def test_close_closes_both_even_on_stop_failure(self):
        self.ready()
        self.arm.pause = lambda: (_ for _ in ()).throw(RuntimeError("pause failed"))
        self.control.close()
        self.assertTrue(self.chassis.closed)
        self.assertEqual(self.control.state, "fault")

    def test_failed_stop_cannot_enable_other_component(self):
        self.chassis.stop = lambda: (_ for _ in ()).throw(RuntimeError("stop failed"))
        with self.assertRaises(RuntimeError):
            self.control.select(True, True)
        self.assertFalse(self.arm.enabled)
        self.assertEqual(self.control.state, "fault")


class WeldPlanTests(unittest.TestCase):
    def test_periodic_path_has_exact_interior_extrema_and_stable_samples(self):
        amplitudes = np.arange(1.0, 7.0)
        times = np.array([0.0, 1.0, 2.0])
        positions = np.array([np.zeros(6), amplitudes, np.zeros(6)])
        path = JointPath(times, positions, "periodic_cubic")
        # Each half-cycle is a cubic smoothstep: q=A*(3*t^2-2*t^3).
        expected_speed, expected_acceleration = 1.5 * amplitudes, 6.0 * amplitudes
        np.testing.assert_allclose(path.peaks(), [expected_speed, expected_acceleration])
        speed, acceleration = path.peaks()
        speed[:] = 0.0
        acceleration[:] = 0.0
        times[1] = 9.0
        positions[:] = 999.0
        np.testing.assert_allclose(path.peaks(), [expected_speed, expected_acceleration])
        q, velocity = path.sample(0.5)
        np.testing.assert_allclose(q, 0.5 * amplitudes)
        np.testing.assert_allclose(velocity, expected_speed)

    def test_initial_model_uses_existing_measured_map_and_preserves_conflict(self):
        mapping = CalibrationMap(
            json.loads(
                (ROOT / "src/robot_description/config/arm_calibration_candidate.json").read_text()
            )
        )
        initial = initial_from_model(
            json.loads((ROOT / "src/robot_description/mujoco/initial_pose.json").read_text()),
            mapping,
        )
        self.assertAlmostEqual(initial[1], -61.357358, places=5)
        self.assertAlmostEqual(initial[2], 152.045323, places=5)
        with self.assertRaisesRegex(ValueError, "A3"):
            mapping.pose(dict(zip("123456", initial)))

    def test_move_has_speed_bound_and_stationary_ends(self):
        settings = Settings()
        path = move_path([0] * 6, [90, -20, 0, 1, -55, 3], settings)
        self.assertTrue(speed_report(path, settings)["within_limits"])
        np.testing.assert_allclose(path.sample(0)[1], 0, atol=0.001)
        np.testing.assert_allclose(path.sample(path.duration)[1], 0, atol=0.001)

    def test_common_time_scale_changes_both_feed_and_period(self):
        a = chassis_only_plan(Settings())
        b = chassis_only_plan(Settings(time_scale=10))
        self.assertAlmostEqual(a["feed_m_s"] * a["duration_s"], b["feed_m_s"] * b["duration_s"])
        self.assertAlmostEqual(a["effective_hz"] / b["effective_hz"], 10)

    def test_invalid_requests(self):
        for args in (
            {"hz": 0},
            {"max_speed_deg_s": 11},
            {"cycles": 1.5},
            {"feed_m_s": float("nan")},
            {"time_scale": 0.5},
            {"amplitude_m": -0.1},
        ):
            with self.subTest(args=args), self.assertRaises(ValueError):
                Settings(**args)
        with self.assertRaises(ValueError):
            JointPath([0, 0], [[0] * 6] * 2)

    def test_omitted_fourth_motor_rejected_before_owner(self):
        with patch("motors.MotorBus") as bus:
            with self.assertRaisesRegex(ValueError, "four-wheel"):
                ChassisOwner(
                    {"motor_driver": {"ros__parameters": {"bench_omit_motor_id": 4}}},
                    simulate=False,
                    execute=True,
                    mapping_confirmed=True,
                )
            bus.assert_not_called()

    def test_real_python_chassis_simulator_feed_and_pause(self):
        import yaml

        config = yaml.safe_load((ROOT / "src/bringup/config/chassis.yaml").read_text())
        chassis = ChassisOwner(config, simulate=True, execute=False, mapping_confirmed=False)
        try:
            chassis.velocity(0.012)
            targets = chassis.drive.mix(0.012, 0)
            np.testing.assert_array_equal(np.sign(targets), [-1, -1, 1, 1])
            self.assertAlmostEqual(max(abs(v) for v in targets), 2.86478898)
            chassis.stop()
            self.assertEqual(chassis.last_feed, 0)
        finally:
            chassis.close()


class WeldNativeArmTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import test_damiao_bench as native

        native.RoutedRoundTrip.setUpClass.__func__(cls)

    @classmethod
    def tearDownClass(cls):
        import test_damiao_bench as native

        native.RoutedRoundTrip.tearDownClass.__func__(cls)

    def setUp(self):
        import test_damiao_bench as native

        native.RoutedRoundTrip.setUp(self)

    def tearDown(self):
        import test_damiao_bench as native

        native.RoutedRoundTrip.tearDown(self)

    def port(self, max_speed_deg_s=5.0):
        from controls.hardware.weld_ports import TrajectoryArmPort
        from motors.arm_calibration import DRIVES

        profile = json.loads((ROOT / "src/robot_description/config/arm_rig_a1_a6.json").read_text())
        profile.update(fixture_confirmed=True, supported_confirmed=True)
        mapping = dict(
            version=1,
            equipment_revision="native-test",
            joints={
                str(i): dict(
                    model_joint=n,
                    model_per_raw=1.0,
                    model_intercept_deg=0.0,
                    raw_branch_deg=[-360.0, 360.0],
                )
                for i, n in enumerate(DRIVES, 1)
            },
        )
        return TrajectoryArmPort(
            profile, bus=self.bus, mapping=mapping, max_speed_deg_s=max_speed_deg_s
        )

    def test_native_stream_complete_group_and_pause(self):
        from motors import arm_position as position, arm_bench as bench

        with patch.object(position, "time", bench.time), self.port() as port:
            port.hold_all(math.radians(5))
            writes = self.lib.fake_parameter_writes()
            for i in range(1, 61):
                self.lib.fake_advance(20)
                port.set_targets(np.array([1, -1, 1, -1, 1, -1]) * i * 0.02, np.full(6, 5.0))
            self.assertEqual([self.lib.fake_status(i) for i in range(6)], [1] * 6)
            self.assertEqual(self.lib.fake_parameter_writes(), writes)
            port.pause_trajectory()
            target = dict(port.hold_targets)
            for _ in range(20):
                self.lib.fake_advance(20)
                port.poll()
            self.assertEqual(port.hold_targets, target)
            self.assertFalse(port.path_active)
            self.assertTrue(port.holding)

    def test_invalid_sixth_target_does_not_send_first_five(self):
        from motors import arm_position as position, arm_bench as bench

        with patch.object(position, "time", bench.time), self.port() as port:
            port.hold_all(math.radians(5))
            before = [self.lib.fake_commands(i) for i in range(6)]
            with self.assertRaises(ValueError):
                port.set_targets([0, 0, 0, 0, 0, float("nan")], [5] * 6)
            with self.assertRaises(ValueError):
                port.set_targets([0] * 6, [5, 5, 5, 5, 5, 11])
            self.assertEqual(before, [self.lib.fake_commands(i) for i in range(6)])

    def test_native_trajectory_needs_no_custom_smoothing_capability(self):
        from motors import arm_position as position, arm_bench as bench
        from motors.arm import CAP_SMOOTH_JOG, CAP_SMOOTH_JOG_10

        self.bus.capabilities &= ~(CAP_SMOOTH_JOG | CAP_SMOOTH_JOG_10)
        with patch.object(position, "time", bench.time), self.port() as port:
            port.hold_all(math.radians(5))
            with patch.object(self.bus, "_request", wraps=self.bus._request) as requests:
                self.lib.fake_advance(20)
                port.set_targets([0.2] * 6, [5] * 6)
                kinds = [call.args[0] for call in requests.call_args_list]
                self.assertGreaterEqual(kinds.count(4), 6)
                self.assertNotIn(25, kinds)
            self.assertFalse(port.smooth_jog)
            self.assertFalse(port.report["secondary_jog_smoothing"])
            self.assertEqual(port.report["usb_command_type"], 4)

    def test_native_weld_can_exceed_jog_cap_without_raising_speed_trip(self):
        from motors import arm_position as position, arm_bench as bench

        with patch.object(position, "time", bench.time), self.port(math.degrees(1.2)) as port:
            port.hold_all(math.radians(5))
            self.lib.fake_advance(20)
            port.set_targets([0.2] * 6, [65] * 6)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)], [1] * 6)
            self.assertTrue(np.all(port.path_speeds[1:] > math.radians(10)))
            np.testing.assert_allclose(port.axis_ceilings, [math.degrees(1.2)] * 6)
            before = [self.lib.fake_commands(i) for i in range(6)]
            for speeds in (
                [69, 65, 65, 65, 65, 65],
                [65, 69, 65, 65, 65, 65],
                [65, 65, 65, 69, 65, 65],
            ):
                with self.assertRaises(ValueError):
                    port.set_targets([0.2] * 6, speeds)
            self.assertEqual(before, [self.lib.fake_commands(i) for i in range(6)])
            with self.assertRaises(ValueError):
                port.set_targets([0.2] * 6, [80] * 6)
            port.pause_trajectory()
            self.assertTrue(np.all(port.path_speeds <= math.radians(5)))

    def test_native_feedback_loss_stops_all_joints(self):
        from motors import arm_position as position, arm_bench as bench

        with patch.object(position, "time", bench.time), self.port() as port:
            port.hold_all(math.radians(5))
            self.lib.fake_route(4, 2, 5, 31)
            try:
                with self.assertRaises(RuntimeError):
                    for _ in range(10):
                        self.lib.fake_advance(20)
                        port.set_targets([0.1] * 6, [5] * 6)
                self.assertTrue(port.recovery_required)
                self.assertEqual([self.lib.fake_status(i) for i in range(6)], [0] * 6)
            finally:
                self.lib.fake_route(4, 2, 5, 21)


if __name__ == "__main__":
    unittest.main()
