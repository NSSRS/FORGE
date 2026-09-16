#!/usr/bin/env python3
"""Bounded hardware velocity test for the three-motor FORGE bench.

The default hardware profile is intentionally fixed to the values recorded in
config/bench-equipment.md. Hardware motion requires the literal --execute flag.
"""

from __future__ import annotations

import argparse
import sys
import time

from forge_motors import MotorBus, MotorConfig, MotorError


INTERFACE = "enp86s0"
MOTOR_IDS = (1, 2, 3)
TARGET_RPM = 30.0
COMMAND_DURATION_S = 2.0
CURRENT_CEILING_A = 2.0
COMMAND_PERIOD_S = 0.02
STOP_TIMEOUT_S = 1.5


def configs() -> list[MotorConfig]:
    # Query-only preflight positions were within -120..120 degrees. These bounds
    # allow the approved single positive revolution while still rejecting a
    # multi-turn runaway or implausible feedback.
    return [
        MotorConfig(
            motor_id=motor_id,
            min_position_deg=-720.0,
            max_position_deg=720.0,
            max_velocity_rpm=TARGET_RPM,
            max_current_a=CURRENT_CEILING_A,
        )
        for motor_id in MOTOR_IDS
    ]


def print_states(bus: MotorBus, label: str) -> None:
    print(label)
    for motor_id in MOTOR_IDS:
        state = bus.state(motor_id)
        print(
            f"  motor {motor_id}: position={state.position_deg!r} deg, "
            f"velocity={state.velocity_rpm!r} rpm, current={state.current_a!r} A, "
            f"temperature={state.temperature_c!r} C, error={state.error}"
        )


def run(*, simulate: bool) -> None:
    kwargs = {"simulate": True} if simulate else {
        "interface": INTERFACE,
        "execute": True,
    }
    with MotorBus(configs(), **kwargs) as bus:
        bus.wait_ready(timeout=3.0)
        print_states(bus, "Initial feedback:")

        if not simulate:
            print(
                "HARDWARE MOTION STARTING: motors 1,2,3; +30 RPM target; "
                "2.0 seconds; 2.0 A phase-current ceiling each."
            )
            for remaining in (3, 2, 1):
                print(f"Starting in {remaining}...", flush=True)
                time.sleep(1.0)

        started = time.monotonic()
        while time.monotonic() - started < COMMAND_DURATION_S:
            for motor_id in MOTOR_IDS:
                bus.velocity(motor_id, TARGET_RPM)
            bus.check()
            time.sleep(COMMAND_PERIOD_S)

        stop_started = time.monotonic()
        while time.monotonic() - stop_started < STOP_TIMEOUT_S:
            bus.stop()
            bus.check()
            states = [bus.state(motor_id) for motor_id in MOTOR_IDS]
            if all(state.velocity_rpm is not None and abs(state.velocity_rpm) < 0.5
                   for state in states):
                break
            time.sleep(COMMAND_PERIOD_S)
        else:
            raise MotorError("Motors did not report below 0.5 RPM before stop timeout")

        print_states(bus, "Final feedback:")
        print("PASS: bounded velocity command completed and all motors reported stopped.")


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulate", action="store_true")
    mode.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    try:
        run(simulate=args.simulate)
    except (MotorError, ValueError, KeyboardInterrupt) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
