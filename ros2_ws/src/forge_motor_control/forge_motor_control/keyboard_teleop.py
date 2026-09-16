"""Dead-man keyboard publisher for direct motor-RPM commissioning."""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


HELP = "W: +RPM  S: -RPM  Space/X: stop  Q/Esc: quit (hold/repeat W or S)"


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("forge_keyboard_teleop")
        self.declare_parameter("motor_count", 3)
        self.declare_parameter("speed_rpm", 10.0)
        self.declare_parameter("deadman_timeout_s", 0.15)
        self._motor_count = int(self.get_parameter("motor_count").value)
        self._speed = float(self.get_parameter("speed_rpm").value)
        self._deadman_timeout = float(self.get_parameter("deadman_timeout_s").value)
        if self._motor_count < 1 or not 0 < self._speed <= 30:
            raise ValueError("motor_count must be positive and speed_rpm must be in (0, 30]")
        self._publisher = self.create_publisher(
            Float64MultiArray, "/drive/motor_rpm_commands", 1
        )

    def publish(self, target: float) -> None:
        message = Float64MultiArray()
        message.data = [target] * self._motor_count
        self._publisher.publish(message)

    def run(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard teleop requires an interactive terminal")
        original = termios.tcgetattr(sys.stdin)
        target = 0.0
        last_motion_key = 0.0
        last_published = 0.0
        pending_key = None
        pending_until = 0.0
        print(HELP)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                readable, _, _ = select.select([sys.stdin], [], [], 0.05)
                if readable:
                    key = sys.stdin.read(1)
                    if key in ("q", "Q", "\x1b"):
                        break
                    if key in ("w", "W"):
                        direction = 1.0
                    elif key in ("s", "S"):
                        direction = -1.0
                    elif key in (" ", "x", "X"):
                        target = 0.0
                        pending_key = None
                        continue
                    else:
                        continue
                    now = time.monotonic()
                    if target == 0.0 and (key.lower() != pending_key or
                                          now > pending_until):
                        pending_key = key.lower()
                        pending_until = now + 1.0
                        continue
                    target = direction * self._speed
                    last_motion_key = now
                    pending_key = None
                if (target != 0.0 and
                        time.monotonic() - last_motion_key > self._deadman_timeout):
                    target = 0.0
                    pending_key = None
                if target != 0.0 or target != last_published:
                    self.publish(target)
                    last_published = target
        finally:
            for _ in range(5):
                self.publish(0.0)
                time.sleep(0.02)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except (KeyboardInterrupt, RuntimeError, ValueError) as error:
        node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
