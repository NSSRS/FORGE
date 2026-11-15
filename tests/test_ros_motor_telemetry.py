"""Both ROS drivers on an isolated domain, simulated/fake owners only."""

import source_paths
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
from test_motor_telemetry import arm_sample, NAMES


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "requires ROS 2 Jazzy")
class RosMotorTelemetryTests(unittest.TestCase):
    def test_both_families_publish_and_arm_stale_feedback_is_explicit(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from diagnostic_msgs.msg import DiagnosticArray
        from motor_control.arm_driver import driver_class
        from motor_control.motor_driver import ForgeMotorDriver

        bus = Mock(
            spec=[
                "configure",
                "probe",
                "base_info",
                "bus_health",
                "snapshot",
                "close",
                "stop",
                "command",
                "enable",
                "reset_fault",
            ]
        )
        bus.base_info.return_value = dict(
            api_version=1, capabilities=4096, build_id=123, command_lease_ms=150
        )
        bus.bus_health.return_value = dict(status_flags=0, tx_failed=0, bus_off_events=0)
        stale = False
        generation = 0

        def snapshot():
            nonlocal generation
            generation += 1
            sample = arm_sample(time.monotonic())
            sample["generation"] = generation
            if stale:
                sample["motors"][4]["age_ms"] = 200
            return sample

        bus.snapshot.side_effect = snapshot
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / "fake-profile.json"
            profile.write_text(
                json.dumps(
                    {
                        "joints": [
                            dict(name=n, bus=1 if i == 0 else 2, limits=dict(kp=1.0, kd=0.1))
                            for i, n in enumerate(NAMES)
                        ]
                    }
                )
            )
            rclpy.init(
                args=[
                    "--ros-args",
                    "-p",
                    "motor_driver:simulate:=true",
                    "-p",
                    "motor_driver:execute:=false",
                    "-p",
                    "arm_driver:execute:=true",
                    "-p",
                    f"arm_driver:profile:={profile}",
                    "-p",
                    "arm_driver:port:=fake-only",
                ]
            )
            executor = SingleThreadedExecutor()
            nodes = []
            try:
                with patch("motors.arm.ArmBus", return_value=bus) as factory:
                    arm = driver_class()()
                    nodes.append(arm)
                factory.assert_called_once_with("fake-only")
                chassis = ForgeMotorDriver()
                nodes.append(chassis)
                probe = Node("dual_telemetry_probe")
                nodes.append(probe)
                arms, drives, diagnostics = [], [], []
                probe.create_subscription(JointState, "/arm/joint_states", arms.append, 10)
                probe.create_subscription(JointState, "/joint_states", drives.append, 10)
                probe.create_subscription(DiagnosticArray, "/diagnostics", diagnostics.append, 10)
                commands = probe.create_publisher(JointState, "/arm/command", 1)
                for node in nodes:
                    executor.add_node(node)

                def spin(seconds):
                    until = time.monotonic() + seconds
                    while time.monotonic() < until:
                        executor.spin_once(timeout_sec=0.005)

                spin(0.8)
                self.assertTrue(arms)
                self.assertTrue(drives)
                self.assertEqual(arms[-1].name, NAMES)
                self.assertEqual(len(drives[-1].position), 4)
                names = {s.name for d in diagnostics for s in d.status}
                self.assertTrue({"encos/bridge", "damiao/controller", "damiao/can_2"} <= names)
                bus.command.assert_not_called()
                bus.enable.assert_not_called()
                bus.reset_fault.assert_not_called()
                bus.stop.assert_not_called()
                self.assertLess(bus.bus_health.call_count, bus.snapshot.call_count)

                # Exercise command reception while both telemetry timers are running.
                latencies = []
                for _ in range(5):
                    message = JointState()
                    message.header.stamp = probe.get_clock().now().to_msg()
                    message.name = NAMES
                    message.position, message.velocity, message.effort = (
                        [0.0] * 6,
                        [0.0] * 6,
                        [0.0] * 6,
                    )
                    before = bus.command.call_count
                    sent = time.monotonic()
                    commands.publish(message)
                    until = sent + 0.1
                    while bus.command.call_count == before and time.monotonic() < until:
                        executor.spin_once(timeout_sec=0.002)
                    self.assertEqual(bus.command.call_count - before, 6)
                    latencies.append(time.monotonic() - sent)
                print(
                    f"Fake-owner dual ROS telemetry: maximum command callback latency {max(latencies)*1000:.2f} ms"
                )

                stale = True
                spin(0.15)  # Drain already-published messages.
                count = len(arms)
                spin(0.6)
                self.assertEqual(len(arms), count)
                a5 = [s for d in diagnostics for s in d.status if s.name == "damiao/arm_joint_5"]
                self.assertEqual(a5[-1].level, bytes([3]))
                bus.stop.assert_not_called()  # Telemetry alone never requests motion or recovery.
            finally:
                for node in reversed(nodes):
                    executor.remove_node(node)
                    node.destroy_node()
                executor.shutdown()
                if rclpy.ok():
                    rclpy.shutdown()
