"""Subscribe to measured JointState and relay to a same-host viewer. No commands."""
import json
import socket
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .motor_mirror import validate_sample


class MotorStateRelay(Node):
    def __init__(self):
        super().__init__('forge_motor_state_relay')
        self.declare_parameter('port', 8766)
        port = self.get_parameter('port').value
        if type(port) is not int or not 1024 <= port <= 65535:
            raise ValueError('port must be 1024-65535')
        self.destination = ('127.0.0.1', port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.session = uuid.uuid4().hex
        self.last_stamp = 0
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.receive, qos_profile_sensor_data)
        self.get_logger().info(f'Read-only /joint_states relay -> 127.0.0.1:{port}')

    def receive(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= self.last_stamp or not 0 <= age <= 0.5:
            return
        try:
            validate_sample(msg.name, msg.position, msg.velocity)
            packet = dict(version=1, session=self.session, stamp_ns=stamp,
                          sent=time.monotonic() - age, names=list(msg.name),
                          positions=list(msg.position), velocities=list(msg.velocity))
            payload = json.dumps(packet, allow_nan=False).encode()
            self.socket.sendto(payload, self.destination)
            self.last_stamp = stamp
        except (ValueError, TypeError, OSError):
            # Viewer absence or malformed telemetry cannot affect the driver.
            return

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorStateRelay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
