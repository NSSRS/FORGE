"""Real ROS subscription to loopback UDP, using synthetic measured feedback only."""
import importlib.util
import json
from pathlib import Path
import socket
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/forge_motor_control'))


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'requires ROS 2')
class RelayTests(unittest.TestCase):
    def test_joint_state_relay_and_stale_rejection(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState
        from forge_motor_control.motor_state_relay import MotorStateRelay
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0)); receiver.setblocking(False)
            port = receiver.getsockname()[1]
            rclpy.init(args=['--ros-args', '-p', f'port:={port}'])
            relay = probe = executor = None
            try:
                relay, probe = MotorStateRelay(), Node('mirror_test_probe')
                publisher = probe.create_publisher(JointState, '/joint_states', 10)
                executor = SingleThreadedExecutor()
                executor.add_node(relay); executor.add_node(probe)
                message = JointState()
                message.name = ['motor_1_output', 'motor_2_output', 'motor_3_output']
                message.position = [1., 2., 3.]; message.velocity = [-.5, -.5, .5]
                received = None
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    message.header.stamp = probe.get_clock().now().to_msg()
                    publisher.publish(message)
                    executor.spin_once(timeout_sec=.03)
                    try:
                        received = json.loads(receiver.recv(16384)); break
                    except BlockingIOError:
                        pass
                self.assertIsNotNone(received)
                self.assertEqual(received['positions'], [1., 2., 3.])
                self.assertEqual(received['names'], message.name)
                # Direct stale callback must not emit a UDP packet.
                message.header.stamp.sec = 1
                relay.receive(message)
                with self.assertRaises(BlockingIOError):
                    receiver.recv(16384)
                topics = {name for name, _ in relay.get_publisher_names_and_types_by_node(
                    relay.get_name(), relay.get_namespace())}
                self.assertNotIn('/cmd_vel', topics)
                self.assertNotIn('/drive/motor_rpm_commands', topics)
            finally:
                if executor: executor.shutdown()
                if relay: relay.destroy_node()
                if probe: probe.destroy_node()
                rclpy.shutdown()
