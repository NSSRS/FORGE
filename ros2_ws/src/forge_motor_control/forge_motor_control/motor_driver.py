"""ROS 2 wrapper for the native FORGE ENCOS motor driver."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from forge_motors import MotorBus, MotorConfig, MotorError


class ForgeMotorDriver(Node):
    """Own one bridge and expose bounded direct-motor RPM commissioning I/O."""

    def __init__(self) -> None:
        super().__init__("forge_motor_driver")
        self.declare_parameter("simulate", True)
        self.declare_parameter("execute", False)
        self.declare_parameter("interface", "enp86s0")
        self.declare_parameter("motor_ids", [1, 2, 3])
        self.declare_parameter("max_velocity_rpm", 30.0)
        self.declare_parameter("max_current_a", 2.0)
        self.declare_parameter("command_timeout_s", 0.20)

        simulate = bool(self.get_parameter("simulate").value)
        interface = str(self.get_parameter("interface").value)
        self._motor_ids = tuple(int(value) for value in self.get_parameter("motor_ids").value)
        self._max_velocity = float(self.get_parameter("max_velocity_rpm").value)
        current = float(self.get_parameter("max_current_a").value)
        self._command_timeout = float(self.get_parameter("command_timeout_s").value)

        if not simulate and not bool(self.get_parameter("execute").value):
            raise RuntimeError("Hardware requires simulate:=false and execute:=true")
        if not 0.02 <= self._command_timeout <= 0.20:
            raise ValueError("command_timeout_s must be between 0.02 and 0.20")

        configs = [
            MotorConfig(motor_id, -720.0, 720.0, self._max_velocity, current)
            for motor_id in self._motor_ids
        ]
        kwargs = {"simulate": True} if simulate else {
            "interface": interface,
            "execute": True,
        }
        self._bus = MotorBus(configs, **kwargs)
        self._bus.wait_ready(timeout=3.0)
        self._targets = [0.0] * len(self._motor_ids)
        self._last_command: float | None = None
        self._closed = False

        self._state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self._command_subscription = self.create_subscription(
            Float64MultiArray,
            "/drive/motor_rpm_commands",
            self._receive_command,
            1,
        )
        self._timer = self.create_timer(0.02, self._update)
        mode = "simulation" if simulate else f"hardware on {interface}"
        self.get_logger().info(
            f"Ready in {mode}; motor IDs={self._motor_ids}; command topic="
            "/drive/motor_rpm_commands"
        )

    def _receive_command(self, message: Float64MultiArray) -> None:
        values = list(message.data)
        valid = (
            len(values) == len(self._motor_ids)
            and all(math.isfinite(value) and abs(value) <= self._max_velocity
                    for value in values)
        )
        if not valid:
            self.get_logger().error(
                f"Rejected command: require {len(self._motor_ids)} finite RPM values "
                f"within +/-{self._max_velocity}"
            )
            return
        self._targets = values
        self._last_command = time.monotonic()

    def _publish_state(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f"motor_{motor_id}_output" for motor_id in self._motor_ids]
        states = [self._bus.state(motor_id) for motor_id in self._motor_ids]
        if all(state.position_deg is not None for state in states):
            message.position = [math.radians(state.position_deg) for state in states]
        if all(state.velocity_rpm is not None for state in states):
            message.velocity = [state.velocity_rpm * math.tau / 60.0 for state in states]
        self._state_publisher.publish(message)

    def _update(self) -> None:
        try:
            self._bus.check()
            if self._last_command is not None:
                stale = (
                    time.monotonic() - self._last_command > self._command_timeout
                )
                targets = [0.0] * len(self._motor_ids) if stale else self._targets
                for motor_id, target in zip(self._motor_ids, targets):
                    self._bus.velocity(motor_id, target)
            self._publish_state()
        except (MotorError, ValueError) as error:
            self.get_logger().fatal(f"Motor session fault: {error}")
            self.close()
            if rclpy.ok():
                rclpy.shutdown()

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._bus.close()

    def destroy_node(self) -> bool:
        self.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ForgeMotorDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, MotorError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"forge_motor_driver: {error}")
        raise SystemExit(1) from error
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
