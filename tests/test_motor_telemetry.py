"""Passive dual-family telemetry, stale data and ROS publication semantics."""

import source_paths
import math
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from motor_control.telemetry import DamiaoTelemetry, TelemetryPublisher, damiao_frame, encos_frame


NAMES = [f"arm_joint_{i+1}" for i in range(6)]


def arm_sample(now=10.0):
    return dict(
        generation=42,
        at_ms=1234,
        fault=False,
        fault_reason=0,
        fault_slot=0,
        host_received_monotonic_s=now,
        acquisition_duration_ms=4.0,
        motors=[
            dict(
                slot=i,
                configured=True,
                enabled=False,
                feedback_valid=True,
                stopping=False,
                enable_confirmed=False,
                age_ms=10,
                position=0.1 * i,
                velocity=0.2 * i,
                torque=0.3 * i,
                status=0,
                mos_c=25,
                rotor_c=26,
            )
            for i in range(6)
        ],
    )


def chassis_sample(now=10.0, ids=(1, 2)):
    return dict(
        api_version=1,
        simulated=False,
        fault=0,
        captured_monotonic_s=now,
        timestamp_basis="host_pdo_observation",
        can_freshness_verified=False,
        exchanges=100,
        failed_exchanges=0,
        last_wkc=3,
        expected_wkc=3,
        max_cycle_gap_s=0.01,
        motors=[
            dict(
                motor_id=i,
                slot=n,
                position_deg=90.0,
                velocity_rpm=30.0,
                current_a=0.5,
                temperature_c=25.0,
                error=0,
                position_observed_s=now - 0.01,
                velocity_observed_s=now - 0.02,
                electrical_observed_s=now - 0.02,
            )
            for n, i in enumerate(ids)
        ],
    )


class FrameTests(unittest.TestCase):
    def test_damiao_ages_include_usb_duration_and_time_in_host(self):
        joint, diagnostics = damiao_frame(arm_sample(), NAMES, 10.02)
        self.assertAlmostEqual(joint["age_s"], 0.034)
        self.assertEqual(joint["names"], NAMES)
        self.assertEqual(joint["position"], [0.1 * i for i in range(6)])
        self.assertEqual(diagnostics[3]["values"]["generation"], 42)
        self.assertAlmostEqual(diagnostics[3]["values"]["feedback_age_ms"], 34.0)

    def test_damiao_missing_stale_invalid_and_faulted_never_publish_complete_state(self):
        cases = [
            ("age_ms", 100),
            ("position", math.nan),
            ("velocity", math.inf),
            ("torque", None),
            ("configured", False),
            ("feedback_valid", False),
            ("age_ms", None),
            ("age_ms", -1),
            ("status", 9),
        ]
        for key, value in cases:
            sample = arm_sample()
            sample["motors"][3][key] = value
            with self.subTest(key=key, value=value):
                joint, diagnostics = damiao_frame(sample, NAMES, 10.0)
                self.assertIsNone(joint)
                self.assertIn(diagnostics[3]["level"], (2, 3))
        sample = arm_sample()
        sample["fault"] = True
        self.assertIsNone(damiao_frame(sample, NAMES, 10.0)[0])
        self.assertIsNone(damiao_frame(arm_sample(), NAMES, 9.0)[0])

    def test_damiao_rejects_partial_and_reordered_snapshots(self):
        for rows in (arm_sample()["motors"][:-1], list(reversed(arm_sample()["motors"]))):
            sample = arm_sample()
            sample["motors"] = rows
            with self.assertRaises(ValueError):
                damiao_frame(sample, NAMES, 10.0)

    def test_encos_keeps_units_and_explicit_freshness_boundary(self):
        joint, diagnostics = encos_frame(chassis_sample(), (1, 2), 10.0)
        self.assertAlmostEqual(joint["position"][0], math.pi / 2)
        self.assertAlmostEqual(joint["velocity"][0], math.pi)
        self.assertEqual(joint["effort"], [])  # Current is not shaft torque.
        self.assertEqual(diagnostics[1]["level"], 1)
        self.assertFalse(diagnostics[1]["values"]["can_freshness_verified"])
        self.assertEqual(diagnostics[1]["values"]["current_a"], 0.5)

    def test_encos_velocity_update_cannot_refresh_old_position_or_electrical_data(self):
        sample = chassis_sample()
        sample["motors"][0]["position_observed_s"] = 9.0
        sample["motors"][0]["electrical_observed_s"] = 9.0
        joint, diagnostics = encos_frame(sample, (1, 2), 10.0)
        self.assertEqual(joint["position"], [])
        self.assertEqual(len(joint["velocity"]), 2)
        self.assertAlmostEqual(joint["age_s"], 0.02)
        self.assertFalse(diagnostics[1]["values"]["position_valid"])
        self.assertFalse(diagnostics[1]["values"]["electrical_valid"])

    def test_encos_missing_nonfinite_fault_and_stale_snapshot(self):
        sample = chassis_sample()
        sample["motors"][0].update(position_deg=None, velocity_rpm=math.nan)
        self.assertIsNone(encos_frame(sample, (1, 2), 10.0)[0])
        for field in ("fault",):
            sample = chassis_sample()
            sample[field] = 2
            joint, diagnostics = encos_frame(sample, (1, 2), 10.0)
            self.assertIsNone(joint)
            self.assertEqual(diagnostics[0]["level"], 2)
        self.assertIsNone(encos_frame(chassis_sample(), (1, 2), 11.0)[0])
        sample = chassis_sample()
        sample["motors"][0]["error"] = 1
        self.assertIsNone(encos_frame(sample, (1, 2), 10.0)[0])
        with self.assertRaises(ValueError):
            encos_frame(chassis_sample(), (2, 1), 10.0)

    def test_simulation_snapshot_preserves_physical_slots_and_does_not_invent_bus_counters(self):
        from motors import MotorBus, MotorConfig

        with MotorBus(
            [MotorConfig(i, -180, 180, 5, 1) for i in (3, 1)], simulate=True, motor_slots=[3, 0]
        ) as bus:
            sample = bus.snapshot()
            self.assertEqual([m["motor_id"] for m in sample["motors"]], [3, 1])
            self.assertEqual([m["slot"] for m in sample["motors"]], [3, 0])
            self.assertTrue(sample["simulated"])
            self.assertIsNone(sample["last_wkc"])
            self.assertEqual(bus._active, set())


class Message:
    def __init__(self):
        self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id="")
        self.status, self.values = [], []


class PublicationTests(unittest.TestCase):
    def setUp(self):
        self.ros = patch.dict(
            sys.modules,
            {
                "sensor_msgs.msg": NS(JointState=Message),
                "diagnostic_msgs.msg": NS(
                    DiagnosticArray=Message, DiagnosticStatus=Message, KeyValue=Message
                ),
            },
        )
        self.ros.start()
        self.addCleanup(self.ros.stop)
        self.node = Mock()
        self.node.get_clock.return_value.now.return_value = NS(
            nanoseconds=10_000_000_000, to_msg=lambda: NS(sec=10, nanosec=0)
        )
        self.joints = Mock()
        self.publisher = TelemetryPublisher(self.node, self.joints, "test")

    def test_joint_stamp_is_oldest_sample_not_publication_time(self):
        self.publisher.publish(*damiao_frame(arm_sample(), NAMES, 10.0))
        message = self.joints.publish.call_args.args[0]
        self.assertEqual(message.header.stamp.sec, 9)
        self.assertEqual(message.header.stamp.nanosec, 986_000_000)
        self.assertEqual(message.header.frame_id, "arm_motor_shafts")

    def test_diagnostics_continue_when_joint_feedback_missing(self):
        sample = arm_sample()
        sample["motors"][2]["feedback_valid"] = False
        self.publisher.publish(*damiao_frame(sample, NAMES, 10.0))
        self.joints.publish.assert_not_called()
        self.assertEqual(len(self.publisher.publisher.publish.call_args.args[0].status), 6)

    def test_diagnostics_throttle_and_immediate_fault_transition(self):
        with patch("motor_control.telemetry.time.monotonic", return_value=10.0):
            self.publisher.publish(*damiao_frame(arm_sample(), NAMES, 10.0))
            self.publisher.publish(*damiao_frame(arm_sample(), NAMES, 10.0))
            self.assertEqual(self.publisher.publisher.publish.call_count, 1)
            sample = arm_sample()
            sample["fault"] = True
            self.publisher.publish(*damiao_frame(sample, NAMES, 10.0))
            self.assertEqual(self.publisher.publisher.publish.call_count, 2)

    def test_damiao_consumer_has_one_owner_and_only_reads_cached_apis(self):
        bus = Mock(spec=["base_info", "bus_health", "snapshot"])
        bus.base_info.return_value = {"api_version": 1, "build_id": 123}
        bus.bus_health.return_value = dict(status_flags=0, tx_failed=0, bus_off_events=0)
        with patch("motor_control.telemetry.time.monotonic", return_value=10.0) as clock:
            consumer = DamiaoTelemetry(bus, self.publisher, NAMES, [1, 2, 2, 2, 2, 2])
            for i in range(26):
                clock.return_value = 10.0 + i * 0.04
                bus.snapshot.return_value = arm_sample(clock.return_value)
                bus.snapshot.return_value["generation"] += i
                consumer.poll()
        bus.base_info.assert_called_once()
        self.assertEqual(bus.snapshot.call_count, 26)
        self.assertEqual(bus.bus_health.call_count, 4)
        self.assertEqual({c.args[0] for c in bus.bus_health.call_args_list}, {1, 2})
        self.assertEqual(len(self.joints.publish.call_args_list), 26)

    def test_unsupported_damiao_api_fails_before_polling(self):
        bus = Mock()
        bus.base_info.return_value = {"api_version": 2}
        with self.assertRaises(RuntimeError):
            DamiaoTelemetry(bus, self.publisher, NAMES, [1])
        bus.snapshot.assert_not_called()
        bus.bus_health.assert_not_called()

    def test_replayed_snapshot_is_rejected_and_generation_wrap_is_valid(self):
        bus = Mock()
        bus.base_info.return_value = {"api_version": 1}
        bus.bus_health.return_value = dict(status_flags=0, tx_failed=0, bus_off_events=0)
        with patch("motor_control.telemetry.time.monotonic", return_value=10.0):
            consumer = DamiaoTelemetry(bus, self.publisher, NAMES, [1])
            sample = arm_sample()
            sample["generation"] = 0xFFFFFFFF
            bus.snapshot.return_value = sample
            consumer.poll()
            sample["generation"] = 1
            consumer.poll()
            with self.assertRaisesRegex(RuntimeError, "generation"):
                consumer.poll()
