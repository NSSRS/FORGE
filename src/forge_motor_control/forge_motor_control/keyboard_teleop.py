"""WASD chassis teleop with keyboard or proportional single-stick radio input."""
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from .differential import KeyboardCommand
from .radio_input import Axis, RadioCommand, parse_channels


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("forge_keyboard_teleop")
        for name, value in (("command_mode", "chassis"), ("input_source", "keyboard"),
                            ("speed_m_s", 0.02), ("yaw_rate_rad_s", 0.07),
                            ("deadman_timeout_s", 0.15), ("command_frame", "base_link"),
                            ("serial_port", ""), ("radio_timeout_s", 0.15),
                            ("radio_deadzone", 0.05)):
            self.declare_parameter(name, value)
        for axis in ("forward", "turn"):
            for field, default in (("channel", 0), ("min", -1), ("center", -1),
                                   ("max", -1), ("sign", 1)):
                self.declare_parameter(f"{axis}_{field}", default)
        value = lambda name: self.get_parameter(name).value
        if value("command_mode") != "chassis":
            raise ValueError("Teleop now supports chassis/WASD only; use chassis_bench.yaml")
        self._source = value("input_source")
        if self._source not in ("keyboard", "radio", "radio_monitor"):
            raise ValueError("input_source must be keyboard, radio, or radio_monitor")
        self._keys = KeyboardCommand(value("speed_m_s"), value("yaw_rate_rad_s"),
                                     value("deadman_timeout_s"))
        self._frame = value("command_frame")
        if not self._frame:
            raise ValueError("command_frame must not be empty")
        self._radio = None
        if self._source == "radio":
            def axis(prefix):
                return Axis(*(value(f"{prefix}_{field}") for field in
                              ("channel", "min", "center", "max", "sign")),
                            deadzone=value("radio_deadzone"))
            self._radio = RadioCommand(axis("forward"), axis("turn"),
                                       value("speed_m_s"), value("yaw_rate_rad_s"),
                                       value("radio_timeout_s"))
        self._publisher = (None if self._source == "radio_monitor" else
                           self.create_publisher(TwistStamped, "/cmd_vel", 1))

    def publish(self, target):
        if self._publisher is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.twist.linear.x, msg.twist.angular.z = target
        self._publisher.publish(msg)

    def run_radio(self):
        import serial
        port = self.get_parameter("serial_port").value
        if not port:
            raise ValueError("Set serial_port to the rediscovered ESP32 USB device")
        # Nonblocking reads keep USB silence from blocking the command watchdog.
        pending = bytearray()
        last_poll = time.monotonic()
        try:
            with serial.Serial(port, 115200, timeout=0, exclusive=True) as stream:
                stream.reset_input_buffer()
                self.get_logger().info(
                    "Radio monitor: no commands published" if self._radio is None else
                    "Radio input: center the selected stick for 0.3 s to enable motion")
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.0)
                    now = time.monotonic()
                    waiting = stream.in_waiting
                    # Discard queued samples after a stalled host; no stale replay.
                    if now - last_poll >= 0.15 or waiting + len(pending) > 256:
                        stream.reset_input_buffer()
                        pending.clear()
                        if self._radio:
                            self._radio.reset()
                    else:
                        pending.extend(stream.read(waiting))
                        while b"\n" in pending:
                            line, _, tail = pending.partition(b"\n")
                            pending = bytearray(tail)
                            text = line.decode("ascii", errors="replace").strip()
                            if self._radio:
                                self._radio.receive(text, now)
                            else:
                                try:
                                    age, channels = parse_channels(text)
                                    print(f"age_ms={age * 1000:.0f} " + " ".join(
                                        f"CH{i}={v}" for i, v in enumerate(channels, 1)), flush=True)
                                except ValueError:
                                    print(text, flush=True)
                    last_poll = now
                    if self._radio:
                        self.publish(self._radio.output(time.monotonic()))
                    time.sleep(0.02)
        finally:
            if self._radio:
                self._radio.reset()
            if rclpy.ok():
                for _ in range(5):
                    self.publish((0.0, 0.0))
                    time.sleep(0.02)

    def run(self):
        if self._source != "keyboard":
            return self.run_radio()
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard teleop requires an interactive terminal")
        original = termios.tcgetattr(sys.stdin)
        directions = "W/S: forward/reverse; A/D: left/right"
        print(f"Hold {directions}; Space/X: stop; Q/Esc: quit (chassis)")
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
