#!/usr/bin/env python3
"""Bypass keyboard teleop and publish one bounded ROS motor command."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


COMMAND_TOPIC = "/drive/motor_rpm_commands"
STATE_TOPIC = "/joint_states"
MOTOR_COUNT = 3
COMMAND_DURATION_S = 2.0
COMMAND_PERIOD_S = 0.02
DISCOVERY_TIMEOUT_S = 5.0
FEEDBACK_TIMEOUT_S = 0.50
STOP_DURATION_S = 1.0
MAX_RPM = 30.0


class AutomaticCommandTest(Node):
    def __init__(self) -> None:
        super().__init__("forge_automatic_command_test")
        self._last_state_time: float | None = None
        self._last_state: JointState | None = None
        self._publisher = self.create_publisher(
            Float64MultiArray, COMMAND_TOPIC, 1
        )
        self._state_subscription = self.create_subscription(
            JointState, STATE_TOPIC, self._receive_state, 10
        )

    def _receive_state(self, message: JointState) -> None:
        self._last_state = message
        self._last_state_time = time.monotonic()

    def _spin_until(self, predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=COMMAND_PERIOD_S)
            if predicate():
                return True
        return False

    def wait_ready(self) -> None:
        if not self._spin_until(
            lambda: self._publisher.get_subscription_count() > 0,
            DISCOVERY_TIMEOUT_S,
        ):
            raise RuntimeError(
                f"no subscriber discovered on {COMMAND_TOPIC}; "
                "start forge_motor_driver first"
            )
        if not self._spin_until(
            lambda: self._last_state_time is not None,
            DISCOVERY_TIMEOUT_S,
        ):
            raise RuntimeError(
                f"driver discovered, but no data arrived on {STATE_TOPIC}; "
                "restart both processes with scripts/ros2_env.sh sourced"
            )

    def _feedback_is_fresh(self) -> bool:
        return (
            self._last_state_time is not None
            and time.monotonic() - self._last_state_time <= FEEDBACK_TIMEOUT_S
        )

    def publish(self, rpm: float) -> None:
        message = Float64MultiArray()
        message.data = [rpm] * MOTOR_COUNT
        self._publisher.publish(message)

    def run(self, rpm: float) -> None:
        self.wait_ready()
        print(
            f"Driver and feedback ready. Commanding motors 1,2,3 at "
            f"{rpm:+.3f} RPM for {COMMAND_DURATION_S:.1f} seconds.",
            flush=True,
        )
        for remaining in (3, 2, 1):
            print(f"Starting in {remaining}...", flush=True)
            countdown_deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < countdown_deadline:
                rclpy.spin_once(self, timeout_sec=0.0)
                if not self._feedback_is_fresh():
                    raise RuntimeError("joint-state feedback became stale")
                self.publish(0.0)
                time.sleep(COMMAND_PERIOD_S)

        started = time.monotonic()
        while rclpy.ok() and time.monotonic() - started < COMMAND_DURATION_S:
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self._feedback_is_fresh():
                raise RuntimeError("joint-state feedback became stale")
            self.publish(rpm)
            time.sleep(COMMAND_PERIOD_S)

    def stop(self) -> None:
        deadline = time.monotonic() + STOP_DURATION_S
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish(0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(COMMAND_PERIOD_S)

    def stopped(self) -> bool:
        message = self._last_state
        return bool(
            self._feedback_is_fresh()
            and message is not None
            and len(message.velocity) == MOTOR_COUNT
            and all(abs(value) < math.tau * 0.5 / 60.0 for value in message.velocity)
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish equal RPM targets for three motors for exactly two seconds, "
            "bypassing keyboard teleop."
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="required acknowledgement that this publishes a live motion command",
    )
    parser.add_argument(
        "--rpm",
        type=float,
        default=5.0,
        help="signed target RPM for all three motors (default: 5.0; limit: +/-30)",
    )
    args = parser.parse_args()
    if not args.execute:
        parser.error("motion requires the literal --execute flag")
    if not math.isfinite(args.rpm) or not 0.0 < abs(args.rpm) <= MAX_RPM:
        parser.error("--rpm must be finite, nonzero, and within +/-30")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = AutomaticCommandTest()
    result = 0
    try:
        node.run(args.rpm)
    except (KeyboardInterrupt, RuntimeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        result = 1
    finally:
        node.stop()
        if result == 0 and not node.stopped():
            print(
                "FAIL: motors did not report below 0.5 RPM after the stop window",
                file=sys.stderr,
            )
            result = 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if result == 0:
        print("PASS: two-second ROS command completed and stopped.", flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
