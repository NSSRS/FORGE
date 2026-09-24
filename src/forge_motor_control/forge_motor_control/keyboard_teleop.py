"""Unified WASD chassis teleop and explicit direct-RPM commissioning mode."""
import math
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray

from .differential import KeyboardCommand


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("forge_keyboard_teleop")
        for name, value in (("command_mode", "chassis"), ("speed_m_s", 0.02),
                            ("yaw_rate_rad_s", 0.07), ("deadman_timeout_s", 0.15),
                            ("command_frame", "base_link"), ("motor_count", 3),
                            ("speed_rpm", 10.0)):
            self.declare_parameter(name, value)
        self._mode = self.get_parameter("command_mode").value
        if self._mode not in ("chassis", "direct_rpm"):
            raise ValueError("command_mode must be chassis or direct_rpm")
        self._motor_count = self.get_parameter("motor_count").value
        speed = self.get_parameter("speed_m_s" if self._mode == "chassis" else "speed_rpm").value
        if self._mode == "direct_rpm" and (
                not 1 <= self._motor_count <= 4 or not math.isfinite(speed) or not 0 < speed <= 30):
            raise ValueError("Commissioning requires 1-4 motors and speed_rpm in (0, 30]")
        self._keys = KeyboardCommand(speed, self.get_parameter("yaw_rate_rad_s").value,
                                     self.get_parameter("deadman_timeout_s").value)
        if self._mode == "direct_rpm":
            self._keys.commands = {k: v for k, v in self._keys.commands.items() if k in ("w", "s")}
        self._frame = self.get_parameter("command_frame").value
        self._publisher = self.create_publisher(
            TwistStamped if self._mode == "chassis" else Float64MultiArray,
            "/cmd_vel" if self._mode == "chassis" else "/drive/motor_rpm_commands", 1)

    def publish(self, target):
        if self._mode == "chassis":
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame
            msg.twist.linear.x, msg.twist.angular.z = target
        else:
            msg = Float64MultiArray()
            msg.data = [target[0]] * self._motor_count
        self._publisher.publish(msg)

    def run(self):
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard teleop requires an interactive terminal")
        original = termios.tcgetattr(sys.stdin)
        directions = "W/S: forward/reverse; A/D: left/right" if self._mode == "chassis" else "W/S: +/-RPM"
        print(f"Hold {directions}; Space/X: stop; Q/Esc: quit ({self._mode})")
        last_target = (0.0, 0.0)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                key = sys.stdin.read(1) if ready else None
                target = self._keys.update(key, time.monotonic())
                if target != (0.0, 0.0) or target != last_target or key in (" ", "x", "X"):
                    self.publish(target)
                last_target = target
                if key and key.lower() in ("q", "\x1b"):
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
            if rclpy.ok():
                for _ in range(5):
                    self.publish((0.0, 0.0))
                    time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleop()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
