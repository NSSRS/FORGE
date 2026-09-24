"""Hardware-free acceptance tests for simulation-to-ENCOS chassis mapping."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/forge_motor_control"))
from forge_motor_control.differential import CommandLease, DifferentialDrive, KeyboardCommand


class DifferentialTests(unittest.TestCase):
    def setUp(self):
        self.drive = DifferentialDrive()
        self.speed = 10 * math.tau * 0.040 / 60
        self.yaw = 2 * 6 * math.tau * 0.040 / (60 * 0.737)

    def assertTargets(self, actual, expected):
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e)

    def test_sim_wasd_mapping(self):
        # forge_sim keyboard_rpm convention at 10 forward / 6 turn RPM.
        self.assertTargets(self.drive.mix(self.speed, 0), [-10, -10, 10, 10])
        self.assertTargets(self.drive.mix(-self.speed, 0), [10, 10, -10, -10])
        self.assertTargets(self.drive.mix(0, self.yaw), [6, 6, 6, 6])
        self.assertTargets(self.drive.mix(0, -self.yaw), [-6, -6, -6, -6])
        self.assertTargets(self.drive.mix(0, 0), [0, 0, 0, 0])

    def test_arc_and_curvature_preserving_limit(self):
        expected = [-4, -4, 16, 16]
        self.assertTargets(self.drive.mix(self.speed, self.yaw), expected)
        self.assertTargets(self.drive.mix(self.speed * 10, self.yaw * 10),
                           [v * 30 / 16 for v in expected])

    def test_driver_order_gearing_and_hardware_signs(self):
        d = replace(self.drive, motor_ids=(9, 4, 6, 2), wheel_motor_ids=(2, 9, 4, 6),
                    external_ratios=(2.0, 1.0, 1.0, 1.0),
                    wheel_rpm_scales=(1.0, 0.5, 1.0, 1.0),
                    wheel_directions=(-1, -1, -1, 1))
        self.assertTargets(d.mix(self.speed, 0), [-5, -10, 10, -20])

    def test_invalid_config_and_input(self):
        for change in ({"motor_ids": (1, 2, 3)}, {"motor_ids": (1, 2, 3, 3)},
                       {"wheel_motor_ids": (1, 2, 3, 5)}, {"wheel_radius_m": 0},
                       {"track_width_m": float("nan")}, {"max_velocity_rpm": 76},
                       {"wheel_directions": (0, -1, 1, 1)},
                       {"wheel_sides": ("left",) * 4},
                       {"external_ratios": (1, 1, 1, -1)}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                replace(self.drive, **change)
        for v in (float("nan"), float("inf"), 1e308):
            with self.assertRaises(ValueError):
                self.drive.mix(v, v)


class LeaseTests(unittest.TestCase):
    def setUp(self):
        self.lease = CommandLease(DifferentialDrive())
        self.values = (0.02, 0, 0, 0, 0, 0.07)

    def receive(self, stamp=10_000_000_000, now=10_000_000_000, values=None, frame="base_link", mono=1):
        self.lease.receive(values or self.values, stamp, frame, now, mono)

    def test_query_only_then_command_then_stop(self):
        self.assertIsNone(self.lease.output(1, 10_000_000_000))
        self.receive()
        self.assertNotEqual(self.lease.output(1.1, 10_100_000_000), [0] * 4)
        self.assertEqual(self.lease.output(1.151, 10_151_000_000), [0] * 4)
        self.assertEqual(self.lease.output(3, 12_000_000_000), [0] * 4)

    def test_transport_age_subtracts_from_lease(self):
        self.receive(stamp=9_900_000_000)
        self.assertEqual(self.lease.output(1.051, 10_051_000_000), [0] * 4)

    def test_invalid_messages_stop_current_motion(self):
        for changes in ({"stamp": 0}, {"stamp": 9_000_000_000},
                        {"stamp": 11_000_000_000}, {"frame": "odom"},
                        {"values": (0, 1, 0, 0, 0, 0)},
                        {"values": (float("nan"), 0, 0, 0, 0, 0)},
                        {"values": (0, 0, 0, 0, 0, float("inf"))}):
            self.lease = CommandLease(DifferentialDrive())
            self.receive(stamp=9_990_000_000)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.receive(**changes)
            self.assertEqual(self.lease.output(1.01, 10_010_000_000), [0] * 4)

    def test_replay_does_not_renew_and_clock_rollback_stops(self):
        self.receive()
        with self.assertRaises(ValueError):
            self.receive(now=10_020_000_000, mono=1.02)
        self.assertEqual(self.lease.output(1.03, 10_030_000_000), [0] * 4)
        self.receive(stamp=10_040_000_000, now=10_040_000_000, mono=1.04)
        self.assertEqual(self.lease.output(1.05, 9_000_000_000), [0] * 4)
        self.receive(stamp=9_010_000_000, now=9_010_000_000, mono=1.06)
        self.assertNotEqual(self.lease.output(1.07, 9_020_000_000), [0] * 4)

    def test_frozen_ros_clock_still_expires(self):
        self.receive()
        self.assertEqual(self.lease.output(1.16, 10_000_000_000), [0] * 4)


class KeyboardTests(unittest.TestCase):
    def test_tap_repeat_release_stop_and_turn(self):
        keys = KeyboardCommand()
        self.assertEqual(keys.update("w", 1), (0, 0))
        self.assertEqual(keys.update("w", 1.5), (0.02, 0))
        self.assertEqual(keys.update(None, 1.66), (0, 0))
        self.assertEqual(keys.update("a", 2), (0, 0))
        self.assertEqual(keys.update("a", 2.1), (0, 0.07))
        self.assertEqual(keys.update("x", 2.11), (0, 0))
        self.assertEqual(keys.update("d", 3), (0, 0))
        self.assertEqual(keys.update("d", 3.1), (0, -0.07))
        self.assertEqual(keys.update(" ", 3.11), (0, 0))


if __name__ == "__main__":
    unittest.main()
