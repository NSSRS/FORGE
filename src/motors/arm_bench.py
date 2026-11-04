"""P-series position/velocity commissioning, separate from the production MIT API.

All coordinates are raw output-shaft radians. No calibration or ID writes.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import struct
import sys
import time

from .arm import ArmBus


def number(value, label):
    if type(value) not in (float, int) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite measured number')
    return value


def validate_plan(plan, *, execute=False, all_motors=False):
    motors = plan.get('motors', [])
    if not isinstance(motors, list) or len(motors) != (6 if all_motors else 1):
        raise ValueError('Select exactly six motors with --all, otherwise exactly one')
    if execute and plan.get('fixture_confirmed') is not True:
        raise ValueError('Confirm the secured, unloaded fixture and measured limits in the plan')
    ids, feedback, units = set(), set(), set()
    for m in motors:
        if m.get('model') not in ('8009P', '4340P', '4310P'):
            raise ValueError('Use the exact P-series model from the FORGE inventory')
        bus, mid, rx = (m.get(k) for k in ('bus', 'can_id', 'feedback_id'))
        if (type(bus) is not int or bus not in (1, 2, 3) or type(mid) is not int or not 1 <= mid <= 15 or
                type(rx) is not int or not (rx == 0 or 16 <= rx < 0x7ff)):
            raise ValueError('Supply discovered bus, command ID (1..15), and feedback ID (0 or 16..2046)')
        if (m['model'] == '8009P') != (bus == 1):
            raise ValueError('8009P must be alone on CAN1; 4340P/4310P use CAN2 or CAN3')
        unit = m.get('unit')
        if not isinstance(unit, str) or not unit.strip() or unit in units:
            raise ValueError('Every physical motor needs a distinct unit label')
        units.add(unit)
        if (bus, mid) in ids or (bus, rx) in feedback:
            raise ValueError('Duplicate command/feedback ID on a shared CAN bus')
        ids.add((bus, mid)); feedback.add((bus, rx))
        ranges = m.get('ranges', {})
        p, v, t = (number(ranges.get(k), k) for k in ('pmax', 'vmax', 'tmax'))
        if not (0 < p <= 1000 and 0 < v <= 10000 and 0 < t <= 10000):
            raise ValueError('Invalid discovered protocol ranges')
        lo, hi, speed, trip, torque, degrees = (number(m.get(k), k) for k in
            ('lower_rad', 'upper_rad', 'speed_rad_s', 'speed_trip_rad_s', 'torque_trip_nm', 'delta_degrees'))
        if not (-min(p, 12.5) <= lo < hi <= min(p, 12.5) and .001 <= speed <= .1 and
                .03 <= trip <= min(.3, v) and speed <= trip and .01 <= torque <= min(3, t) and
                2 <= abs(degrees) <= 45):
            raise ValueError('Invalid fixture limits or test envelope (2..45 degrees, speed <=0.1 rad/s)')
        if math.radians(abs(degrees)) / speed + 1.0 > 12:
            raise ValueError('Requested travel/speed cannot complete within the 12-second test window')
        if execute and m.get('identity_confirmed') is not True:
            raise ValueError(f'{unit}: confirm physical model, bus and IDs after isolated discovery')
        if execute and all_motors and m.get('individual_test_passed') is not True:
            raise ValueError(f'{unit}: complete the individual test before --all')
    if all_motors:
        if Counter(m['model'] for m in motors) != {'8009P': 1, '4340P': 2, '4310P': 3}:
            raise ValueError('Full arm requires one 8009P, two 4340P and three 4310P')
        if sum(m['bus'] == 1 for m in motors) != 1:
            raise ValueError('CAN1 must contain only the 8009P')
    return motors


def preflight(bus, motors):
    if bus.capabilities & 399 != 399:
        raise RuntimeError('Rebuild/flash the routed arm-test firmware (capability 0x100 required)')
    observed = []
    for m in motors:
        values = {}
        for reg in (7, 8, 9, 10, 21, 22, 23, 31):
            reply = bus.read_register(m['bus'], m['can_id'], reg)
            if reply is None or reply[0] != m['feedback_id']:
                raise RuntimeError(f"{m['unit']}: missing/wrong feedback ID for register {reg}")
            values[reg] = reply[1] if reg < 21 else struct.unpack('<f', struct.pack('<I', reply[1]))[0]
        if values[7] != m['feedback_id'] or values[8] != m['can_id'] or values[10] not in (1, 2):
            raise RuntimeError(f"{m['unit']}: identity or mode mismatch")
        for reg, key in ((21, 'pmax'), (22, 'vmax'), (23, 'tmax')):
            if not math.isclose(values[reg], m['ranges'][key], rel_tol=1e-5):
                raise RuntimeError(f"{m['unit']}: {key} changed; repeat discovery")
        if not math.isfinite(values[31]) or values[31] <= 0:
            raise RuntimeError(f"{m['unit']}: positive velocity-loop damping required")
        observed.append(values)
    return observed


class BenchSession:
    def __init__(self, bus, motors, observed, report):
        self.bus, self.motors, self.observed, self.report = bus, motors, observed, report
        self.configured = []
        self.restore = set()
        self.origins = {}
        self.bounds = {}

    def poll(self, slot, predicate, description, *, faults=True, timeout=1):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            s = self.bus.state(slot)
            if faults and (s.fault or s.status >= 8):
                raise RuntimeError(f'{description}: {s.fault_detail()}, motor status={s.status}')
            if predicate(s):
                return s
            time.sleep(.005)
        raise TimeoutError(f'slot {slot}: {description}')

    def stop(self):
        self.bus.stop()
        result = {}
        for i in self.configured:
            result[i] = self.poll(i, lambda s: not s.enabled and not s.stopping and s.status == 0
                                  and s.age_ms < 80 and abs(s.velocity) <= .03,
                                  'fresh stationary disable acknowledgement', faults=False)
        return result

    def prepare(self):
        for i, m in enumerate(self.motors):
            self.configured.append(i)  # An accepted configuration can lose its USB ACK.
            p = min(12.5, m['ranges']['pmax'])
            self.bus.prepare_routed_bench(i, m, -p, p, self.observed[i][10])
        for i in self.configured:
            self.bus.probe(i)
            self.poll(i, lambda s: s.registers & 15 == 15, 'initial range/mode probe')
        initial = self.stop()
        for i, m in enumerate(self.motors):
            origin = initial[i].position
            target = origin + math.radians(m['delta_degrees'])
            lo, hi = min(origin, target) - .05, max(origin, target) + .05
            if not (m['lower_rad'] <= lo < hi <= m['upper_rad']):
                raise RuntimeError(f"{m['unit']}: measured start/target corridor exceeds fixture limits")
            self.origins[i], self.bounds[i] = origin, (lo, hi)
        for i in self.configured:
            self.stop()
            self.bus.write_routed_bench(i, 9, 2000)
            if self.observed[i][10] != 2:
                self.stop()
                self.restore.add(i)  # Before write: ACK/readback may fail after motor changes mode.
                self.bus.write_routed_bench(i, 10, 2)
        for i, m in enumerate(self.motors):
            self.bus.prepare_routed_bench(i, m, *self.bounds[i], 2)
        for i in self.configured:
            self.bus.probe(i)
            s = self.poll(i, lambda s: s.registers == 31, 'verified mode/ranges/timeout')
            if any(not math.isclose(getattr(s, key), mvalue, rel_tol=1e-5)
                   for key, mvalue in self.motors[i]['ranges'].items()):
                raise RuntimeError('Firmware range readback differs from preflight')
        self.stop()

    def sample(self, i):
        s = self.bus.state(i)
        m = self.motors[i]
        lo, hi = self.bounds[i]
        if (s.fault or not s.enabled or s.age_ms >= 80 or s.status not in (0, 1) or
                not all(math.isfinite(v) for v in (s.position, s.velocity, s.torque)) or
                not lo < s.position < hi or abs(s.velocity) > m['speed_trip_rad_s'] or
                abs(s.torque) > m['torque_trip_nm'] or max(s.mos_c, s.rotor_c) >= 80):
            raise RuntimeError(f"{m['unit']}: invalid motion feedback; {s.fault_detail()}")
        return s

    def command(self, i, target):
        self.bus.command(i, target, velocity=self.motors[i]['speed_rad_s'])

    def move(self, slots, targets, label):
        states = self.stop()
        holds = {i: states[i].position for i in slots}
        if any(abs(holds[i] - self.origins[i]) > .02 for i in slots):
            raise RuntimeError('Shaft moved away from its initial position before test')
        for i in slots:
            self.command(i, holds[i])
        self.bus.enable_bench_group(slots)
        deadline = time.monotonic() + .4
        while True:
            for i in slots:
                self.command(i, holds[i])
            states = {i: self.sample(i) for i in slots}
            if all(s.enable_confirmed and s.status == 1 for s in states.values()):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError('Enable not confirmed for every selected motor; no movement target sent')
            time.sleep(.005)
        start = time.monotonic()
        settled = {i: None for i in slots}
        peaks = {i: {'sampled_speed_rad_s': 0., 'sampled_torque_nm': 0.} for i in slots}
        next_print = start
        while time.monotonic() - start < 12:
            # Refresh every lease before polling sequential USB state requests.
            for i in slots:
                self.command(i, targets[i])
            states = {i: self.sample(i) for i in slots}
            now = time.monotonic()
            for i, s in states.items():
                if s.status != 1 or not s.enable_confirmed:
                    raise RuntimeError('Motor lost enabled confirmation')
                peaks[i]['sampled_speed_rad_s'] = max(peaks[i]['sampled_speed_rad_s'], abs(s.velocity))
                peaks[i]['sampled_torque_nm'] = max(peaks[i]['sampled_torque_nm'], abs(s.torque))
                reached = abs(s.position-targets[i]) <= math.radians(1) and abs(s.velocity) <= .03
                settled[i] = (settled[i] if settled[i] is not None else now) if reached else None
            if now >= next_print:
                print(label, ', '.join(f"{self.motors[i]['unit']}: error={math.degrees(targets[i]-s.position):+.2f}deg"
                                      for i, s in states.items()), flush=True)
                next_print = now + .25
            if all(t is not None and now-t >= .3 for t in settled.values()):
                self.report['stages'].append({'stage': label, 'passed': True, 'elapsed_s': now-start,
                    'motors': [{**peaks[i], 'unit': self.motors[i]['unit'], 'position_rad': s.position,
                                'target_rad': targets[i], 'error_rad': targets[i]-s.position}
                               for i, s in states.items()]})
                self.stop()
                return
            time.sleep(.005)
        raise RuntimeError(f'{label}: target not reached and settled within 12 seconds')

    def run(self, simultaneous=False):
        self.prepare()
        # Excursion then return are separate bounded stages, with disable between stages.
        for i, m in enumerate(self.motors):
            origin = self.origins[i]
            target = origin + math.radians(m['delta_degrees'])
            self.move([i], {i: target}, f"{m['unit']} outward")
            self.origins[i] = self.stop()[i].position
            self.move([i], {i: origin}, f"{m['unit']} return")
            self.origins[i] = origin
        if simultaneous:
            origins = dict(self.origins)
            targets = {i: origins[i] + math.radians(m['delta_degrees']) for i, m in enumerate(self.motors)}
            self.move(self.configured, targets, 'all motors outward')
            self.origins = {i: s.position for i, s in self.stop().items()}
            self.move(self.configured, origins, 'all motors return')
            self.origins = origins

    def cleanup(self):
        if not self.configured:
            return
        errors = []
        try:
            self.stop()
            self.report['disable_acknowledged'] = True
        except (OSError, RuntimeError, ValueError) as exc:
            raise RuntimeError(f'Disable not confirmed: turn OFF motor PSU; {exc}') from exc
        for i in sorted(self.restore):
            try:
                self.stop()
                self.bus.write_routed_bench(i, 10, self.observed[i][10])
            except (OSError, RuntimeError, ValueError) as exc:
                errors.append(f"{self.motors[i]['unit']}: {exc}")
        self.report['mode_restoration_verified'] = not errors
        if errors:
            raise RuntimeError('Mode restoration failed; power-cycle before reuse: ' + '; '.join(errors))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--all', action='store_true', help='six individual tests followed by concurrent outward/return')
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--check', action='store_true', help='read registers only; no configuration, disable or writes')
    action.add_argument('--execute', action='store_true', help='run the confirmed unloaded test plan')
    parser.add_argument('--report', type=Path, help='JSON result, including failures and cleanup')
    args = parser.parse_args(argv)
    report = {'passed': False, 'hardware_test': bool(args.execute), 'stages': [],
              'disable_acknowledged': False, 'mode_restoration_verified': False}
    bus = session = None
    result = 2
    try:
        plan = json.loads(args.plan.read_text())
        motors = validate_plan(plan, execute=args.execute, all_motors=args.all)
        report['plan'] = plan
        if not args.check and not args.execute:
            print(json.dumps(plan, indent=2))
            print('Plan validated offline. No serial device opened.')
            result = 0
            return result
        bus = ArmBus(args.port)
        observed = preflight(bus, motors)
        report['registers'] = observed
        if args.check:
            print(json.dumps(observed, indent=2)); result = 0
        else:
            session = BenchSession(bus, motors, observed, report)
            session.run(simultaneous=args.all)
            result = 0
    except KeyboardInterrupt:
        report['error'] = 'Interrupted'; result = 130
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        report['error'] = str(exc)
    finally:
        if session is not None:
            try:
                session.cleanup()
            except (OSError, RuntimeError, ValueError) as exc:
                report['cleanup_error'] = str(exc); result = 2
        if bus is not None:
            try:
                bus.close()
            except (OSError, RuntimeError) as exc:
                report['close_error'] = str(exc); result = 2
        report['passed'] = result == 0
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
    if result:
        print(json.dumps(report, indent=2), file=sys.stderr)
    else:
        print('PASS: ' + ('motion, disable and mode restoration verified.' if args.execute else 'read-only checks.'))
    return result


if __name__ == '__main__':
    raise SystemExit(main())
