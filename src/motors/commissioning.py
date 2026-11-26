#!/usr/bin/env python3
"""Bounded ENCOS bench motion through the existing single MotorBus owner.

The compatibility launcher retains its historical three-motor CAN1 defaults.
The fourth-motor launcher uses the commissioned split bus and a 5 RPM trial.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

from motors import MotorBus, MotorConfig, MotorError

INTERFACE = 'enp86s0'
MOTOR_IDS = (1, 2, 3)
TARGET_RPM = 30.0
COMMAND_DURATION_S = 2.0
CURRENT_CEILING_A = 2.0
COMMAND_PERIOD_S = 0.02
STOP_THRESHOLD_RPM = 0.5
STOP_TIMEOUT_S = 1.5
ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class BenchPlan:
    motor_ids: tuple[int, ...] = MOTOR_IDS
    motor_slots: tuple[int, ...] = (0, 1, 2)
    rpm: float = TARGET_RPM
    seconds: float = COMMAND_DURATION_S
    bidirectional: bool = False
    interface: str = INTERFACE

    @classmethod
    def fourth(cls, *, only=False, interface=INTERFACE):
        return cls(
            motor_ids=(4,) if only else (1, 2, 3, 4),
            motor_slots=(4,) if only else (0, 1, 3, 4),
            rpm=5.0,
            seconds=1.0,
            bidirectional=True,
            interface=interface,
        )


def configs(plan=None) -> list[MotorConfig]:
    plan = plan or BenchPlan()
    return [MotorConfig(i, -720.0, 720.0, plan.rpm, CURRENT_CEILING_A)
            for i in plan.motor_ids]


def print_states(bus, label, plan=None):
    plan = plan or BenchPlan()
    print(label, flush=True)
    for motor_id in plan.motor_ids:
        state = bus.state(motor_id)
        print(f'  motor {motor_id}: position={state.position_deg!r} deg, '
              f'velocity={state.velocity_rpm!r} rpm, current={state.current_a!r} A, '
              f'temperature={state.temperature_c!r} C, error={state.error}', flush=True)


def run(*, simulate, plan=None, report_path=None, bus_factory=MotorBus, query_only=False,
        clock=time.monotonic, sleep=time.sleep):
    plan = plan or BenchPlan()
    # Reserve output before acquiring any motor owner; never overwrite evidence.
    output = None
    if report_path is not None:
        report_path = Path(report_path)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        output = report_path.open('x')
    report = dict(schema='forge.encos_bench', version=1, simulated=simulate,
                  hardware_qualified=False, interface=plan.interface,
                  motor_ids=list(plan.motor_ids), motor_slots=list(plan.motor_slots),
                  target_rpm=plan.rpm, command_seconds=plan.seconds,
                  phase_current_ceiling_a=CURRENT_CEILING_A,
                  bidirectional=plan.bidirectional, stages=[], samples=[],
                  outcome='failed', error=None, owner_closed=False,
                  can_freshness_verified=False)
    if query_only:
        report.update(schema='forge.encos_idle_feedback', target_rpm=None, command_seconds=0,
                      bidirectional=False, observation_seconds=2.0)
    bus = None
    started = clock()

    def sample(stage, target):
        telemetry = bus.snapshot()
        report['samples'].append(dict(elapsed_s=clock()-started, stage=stage,
                                      target_rpm=target, telemetry=telemetry))
        return telemetry

    def stop_and_check():
        stop_start = clock()
        while clock() - stop_start < STOP_TIMEOUT_S:
            bus.stop()
            bus.check()
            sample('stop', 0.0)
            states = [bus.state(i) for i in plan.motor_ids]
            if all(s.velocity_rpm is not None and abs(s.velocity_rpm) < STOP_THRESHOLD_RPM
                   for s in states):
                return
            sleep(COMMAND_PERIOD_S)
        raise MotorError('Motors did not report below 0.5 RPM before stop timeout')

    def observe_idle():
        previous = {}
        changes = {i: 0 for i in plan.motor_ids}
        peaks = {i: 0.0 for i in plan.motor_ids}
        until = clock() + report['observation_seconds']
        while clock() < until:
            bus.check()
            snapshot = sample('query_only', None)
            if snapshot['fault'] or {row['motor_id'] for row in snapshot['motors']} != set(plan.motor_ids):
                raise MotorError('Incomplete or faulted idle feedback snapshot')
            for row in snapshot['motors']:
                motor = row['motor_id']
                stamp = row['position_observed_s']
                if stamp is None:
                    raise MotorError(f'No accepted idle position for motor {motor}')
                age = snapshot['captured_monotonic_s'] - stamp
                peaks[motor] = max(peaks[motor], age * 1000)
                if not math.isfinite(age) or not 0 <= age <= 0.25:
                    raise MotorError(f'Motor {motor} idle position is {age:.3f} s old')
                if motor in previous and stamp < previous[motor]:
                    raise MotorError(f'Motor {motor} idle timestamp moved backwards')
                if motor in previous and stamp > previous[motor]:
                    changes[motor] += 1
                previous[motor] = stamp
            sleep(COMMAND_PERIOD_S)
        if not all(changes.values()):
            raise MotorError(f'Idle position timestamps did not advance for every motor: {changes}')
        report['stages'].append(dict(observed_updates=changes, peak_observed_age_ms=peaks))
        for motor in plan.motor_ids:
            print(f'Motor {motor}: {changes[motor]} observed updates, '
                  f'peak sampled age {peaks[motor]:.1f} ms', flush=True)

    def run_motion():
        if not simulate:
            print(f'HARDWARE MOTION: IDs {plan.motor_ids}, slots {plan.motor_slots}; '
                  f'{plan.rpm:g} RPM, {plan.seconds:g} s per direction, '
                  f'{CURRENT_CEILING_A:g} A phase-current ceiling. Secure all outputs.', flush=True)
            for remaining in (3, 2, 1):
                print(f'Starting in {remaining}...', flush=True)
                bus.check()
                sleep(1.)
        for direction in ((1, -1) if plan.bidirectional else (1,)):
            target = direction * plan.rpm
            positions = {i: bus.state(i).position_deg for i in plan.motor_ids}
            observed = {i: False for i in plan.motor_ids}
            phase_start = clock()
            while clock() - phase_start < plan.seconds:
                for i in plan.motor_ids:
                    bus.velocity(i, target)
                bus.check()
                sample('positive' if direction > 0 else 'negative', target)
                for i in plan.motor_ids:
                    velocity = bus.state(i).velocity_rpm
                    observed[i] |= velocity is not None and velocity * direction >= STOP_THRESHOLD_RPM
                sleep(COMMAND_PERIOD_S)
            stop_and_check()
            missing = [i for i, seen in observed.items() if not seen]
            if missing:
                raise MotorError(f'No signed motion feedback for motor IDs: {missing}')
            stage = dict(
                target_rpm=target,
                below_half_rpm=True,
                travel_deg={str(i): bus.state(i).position_deg - positions[i] for i in plan.motor_ids},
            )
            report['stages'].append(stage)
            print(f'Stopped after {target:+g} RPM: travel_deg={stage["travel_deg"]}', flush=True)

    try:
        kwargs = dict(simulate=True) if simulate else dict(interface=plan.interface, execute=True)
        bus = bus_factory(configs(plan), motor_slots=plan.motor_slots, **kwargs)
        bus.wait_ready(timeout=3.0)  # Every selected ID is required, without omission.
        sample('initial', None)
        print_states(bus, 'Initial feedback:', plan)
        if query_only:
            observe_idle()
        else:
            run_motion()
        print_states(bus, 'Final feedback:', plan)
        report['outcome'] = 'passed'
    except BaseException as error:
        report['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        try:
            if bus is not None:
                try:
                    report['last_telemetry'] = bus.snapshot()
                finally:
                    bus.close()
                    report['owner_closed'] = True
        except Exception as error:
            report['outcome'] = 'failed'
            report['close_error'] = f'{type(error).__name__}: {error}'
            raise
        finally:
            if output is not None:
                with output:
                    json.dump(report, output, indent=2, allow_nan=False)
                    output.write('\n')
                print(f'Report: {report_path}', flush=True)
    if query_only:
        print('PASS: all selected motors supplied recurring idle positions.', flush=True)
    else:
        print('PASS: every selected motor reported both commanded motion and stop.', flush=True)
    return report


def main(*, default_four_motors=False):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--simulate', action='store_true')
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--query-only', action='store_true',
                      help='observe idle position updates on hardware for two seconds')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--motor4-only', action='store_true', help='Only CAN ID 4, PDO slot 4 on CAN2')
    selection.add_argument('--four-motors', action='store_true', help='Require IDs 1,2,3,4 in slots 0,1,3,4')
    parser.add_argument('--interface', default=INTERFACE)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.motor4_only or args.four_motors or default_four_motors:
        plan = BenchPlan.fourth(only=args.motor4_only, interface=args.interface)
    else:
        plan = BenchPlan(interface=args.interface)
    report_name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '.json'
    report_path = args.report or ROOT / 'build/encos/sessions' / report_name
    try:
        run(simulate=args.simulate, plan=plan, report_path=report_path, query_only=args.query_only)
    except (MotorError, ValueError, OSError, KeyboardInterrupt) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
