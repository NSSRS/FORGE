"""P-series position/velocity commissioning, separate from the production MIT API.

All coordinates are raw output-shaft radians. No calibration or ID writes.
"""
import argparse
from dataclasses import asdict
from collections import Counter
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

from .arm import (ArmBus, BENCH_MODEL_LIMITS, CAP_CONTROL_READBACK,
                  CAP_MANUAL_TRAVEL, CAP_POSITION_HOLD, CAP_GROUP_HOLD, CONTROL_REGISTERS, DISCOVERY_REGISTERS, decode_register)


def number(value, label):
    if type(value) not in (float, int) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite measured number')
    return value


def active_joints(plan):
    """Physical DOF numbers; a subset never renumbers motor IDs or MCU slots."""
    count = len(plan.get('motors', []))
    joints = plan.get('active_joints', list(range(1, count+1)))
    if (not isinstance(joints, list) or not joints or
            any(type(j) is not int or not 1 <= j <= count for j in joints) or
            len(set(joints)) != len(joints)):
        raise ValueError('active_joints must be distinct physical joint numbers in the plan')
    if 'active_joints' in plan:
        if count != 6 or any(m.get('can_id') != i+1 or m.get('feedback_id') != 17+i
                             for i, m in enumerate(plan['motors'])):
            raise ValueError('Subset plans require the complete verified A1..A6 inventory and IDs')
    return tuple(sorted(joints))


def validate_plan(plan, *, execute=False, all_motors=False, group_first=False):
    if group_first and not all_motors:
        raise ValueError("--group-first requires --all")
    motors = plan.get('motors', [])
    if not isinstance(motors, list) or len(motors) != (6 if all_motors else 1):
        raise ValueError('Select exactly six motors with --all, otherwise exactly one')
    active_joints(plan)
    if execute and plan.get('fixture_confirmed') is not True:
        raise ValueError('Confirm the secured fixture and measured limits in the plan')
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
        max_speed, max_torque = BENCH_MODEL_LIMITS[m['model']]
        if not (-min(p, 12.5) <= lo < hi <= min(p, 12.5) and .001 <= speed and
                .03 <= trip <= min(max_speed, v) and speed <= trip and .01 <= torque <= min(max_torque, t) and
                2 <= abs(degrees) <= 180):
            raise ValueError('Invalid fixture limits or test envelope (2..180 degrees, speed must not exceed feedback trip)')
        if math.radians(abs(degrees)) / speed + 1.0 > 12:
            raise ValueError('Requested travel/speed cannot complete within the 12-second test window')
        if execute and m.get('identity_confirmed') is not True:
            raise ValueError(f'{unit}: confirm physical model, bus and IDs after isolated discovery')
        if execute and all_motors and not group_first and m.get('individual_test_passed') is not True:
            raise ValueError(f'{unit}: complete the individual test before --all')
    if all_motors:
        if Counter(m['model'] for m in motors) != {'8009P': 1, '4340P': 2, '4310P': 3}:
            raise ValueError('Full arm requires one 8009P, two 4340P and three 4310P')
        if sum(m['bus'] == 1 for m in motors) != 1:
            raise ValueError('CAN1 must contain only the 8009P')
    return motors


def read_arm_plan(path):
    """Read a six-axis commissioning plan and its active physical joint numbers."""
    if not path:
        return None, tuple(range(1, 7))
    plan = json.loads(Path(path).read_text())
    validate_plan(plan, all_motors=True)
    return plan, active_joints(plan)


class LatchedControllerFault(RuntimeError):
    """Retained controller fault; an idle UI may offer explicit recovery."""


def stationary_disabled(state):
    return (not state.enabled and not state.stopping and state.status == 0 and
            state.age_ms < 80 and math.isfinite(state.velocity) and abs(state.velocity) <= .03)


class DisableAcknowledgementTimeout(TimeoutError):
    """Keep the final per-joint feedback when a disabled check cannot pass."""
    def __init__(self, states):
        self.states = dict(states)
        details = []
        for i, state in states.items():
            if stationary_disabled(state):
                continue
            reasons = []
            if state.status != 0:
                label = ' (undervoltage)' if state.status == 9 else ''
                reasons.append(f'drive status {state.status}{label}')
            if state.enabled:reasons.append('still enabled')
            if state.stopping:reasons.append('disable not acknowledged')
            if state.age_ms >= 80:
                reasons.append('no valid feedback' if state.age_ms == 0xffffffff else f'feedback age {state.age_ms} ms')
            if not math.isfinite(state.velocity) or abs(state.velocity) > .03:
                reasons.append(f'speed {state.velocity:.4f} rad/s; waiting for stationary shaft')
            details.append(f'A{i+1}: '+', '.join(reasons))
        super().__init__('fresh stationary disable acknowledgement failed; '+'; '.join(details))


def read_settings(bus, channel, mid, rx, stop=None):
    values = {}
    for register in DISCOVERY_REGISTERS:
        if stop is not None:
            stop()
        reply = bus.read_register(channel, mid, register)
        if reply is None or reply[0] != rx:
            raise RuntimeError(f'CAN{channel} ID {mid}: missing/wrong feedback ID for register {register}')
        values[register] = decode_register(register, reply[1])
    if values[7] != rx or values[8] != mid or values[10] not in (1, 2):
        raise RuntimeError(f'CAN{channel} ID {mid}: identity or mode mismatch')
    return values


def preflight(bus, motors):
    if bus.capabilities & 399 != 399:
        raise RuntimeError('Rebuild/flash the routed arm-test firmware (capability 0x100 required)')
    # A completed run leaves configured slots in MCU RAM. Refresh disabled
    # feedback before each read instead of resetting the controller or faults.
    states = {i: bus.state(i) for i in range(6)}
    if any(s.fault or s.enabled for s in states.values()):
        error = RuntimeError if any(s.enabled for s in states.values()) else LatchedControllerFault
        raise error('MC02 is active or faulted; explicit recovery required. Stop the other owner and address the cause, then use '
                    'test_dm_arm.py --check --feedback --supported --recover --report PATH '
                    'with the same plan/port; no automatic reset')
    retained = [i for i, s in states.items() if s.registers or s.stopping]
    previous = None
    if retained:
        print('Reusing prior session: confirming stationary disabled feedback before register reads.', flush=True)
        previous = BenchSession(bus, [], [], {})
        previous.configured = retained
        previous.stop()
        bus.configured = True
        bus.bench = True
    observed = []
    for m in motors:
        values = read_settings(bus, m['bus'], m['can_id'], m['feedback_id'],
                               previous.stop if previous else None)
        for reg, key in ((21, 'pmax'), (22, 'vmax'), (23, 'tmax')):
            if not math.isclose(values[reg], m['ranges'][key], rel_tol=1e-5):
                raise RuntimeError(f"{m['unit']}: {key} changed; repeat discovery")
        if not math.isfinite(values[31]) or values[31] <= 0:
            raise RuntimeError(f"{m['unit']}: positive velocity-loop damping required")
        observed.append(values)
    return observed


class BenchSession:
    def __init__(self, bus, motors, observed, report, *, slots=None):
        self.bus, self.motors, self.observed, self.report = bus, motors, observed, report
        self.slots = tuple(range(len(motors))) if slots is None else tuple(slots)
        if (len(set(self.slots)) != len(self.slots) or
                any(type(i) is not int or not 0 <= i < len(motors) for i in self.slots)):
            raise ValueError('Invalid commissioning slots')
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
        deadline = time.monotonic() + 1
        next_disable = time.monotonic() + .02
        states = {}
        while time.monotonic() < deadline:
            states = {i: self.bus.state(i) for i in self.configured}
            if all(stationary_disabled(s) for s in states.values()):
                return states
            # A disabled reply can arrive before the shaft stops coasting. The
            # motor only replies to commands; request fresh disable feedback.
            if time.monotonic() >= next_disable:
                self.bus.stop()
                next_disable = time.monotonic() + .02
            time.sleep(.005)
        self.report['disable_timeout_states'] = {i: asdict(s) for i, s in states.items()}
        raise DisableAcknowledgementTimeout(states)

    def capture_failure(self):
        # Disable first; diagnostics must never delay the stop request.
        self.bus.stop()
        self.report['fault_snapshot'] = self.bus.fault_snapshot()
        self.report['failure_states'] = {i: asdict(self.bus.state(i)) for i in self.configured}

    def probe_disabled(self):
        """Configure MCU routes and capture disabled feedback; no motor parameter writes."""
        for i in self.slots:
            m = self.motors[i]
            self.configured.append(i)  # An accepted configuration can lose its USB ACK.
            p = min(12.5, m['ranges']['pmax'])
            self.bus.prepare_routed_bench(i, m, -p, p, self.observed[i][10])
        for i in self.configured:
            self.bus.probe(i)
            self.poll(i, lambda s: s.registers & 15 == 15, 'initial range/mode probe')
        return self.stop()

    def prepare(self, *, jog_window_rad=None, manual_travel=False, position_hold=False, group_hold=False):
        if jog_window_rad is not None and not 0 < jog_window_rad <= math.pi/2:
            raise ValueError("Jog window must be within +/-90 degrees of session start")
        if manual_travel:
            if jog_window_rad is not None:
                raise ValueError("Choose manual travel or a startup window")
            if not self.bus.capabilities & CAP_MANUAL_TRAVEL:
                raise RuntimeError("Manual travel requires MC02 capability 0x8000; flash the updated firmware")
        if position_hold and (not manual_travel or not self.bus.capabilities & CAP_POSITION_HOLD):
            raise RuntimeError('Position hold requires manual travel and MC02 capability 0x10000')
        if group_hold and (not position_hold or not self.bus.capabilities & CAP_GROUP_HOLD):
            raise RuntimeError('Group hold requires position hold and MC02 capability 0x20000')
        initial = self.probe_disabled()
        for i in self.slots:
            m = self.motors[i]
            origin = initial[i].position
            target = origin + math.radians(m['delta_degrees'])
            if manual_travel:
                p = min(12.5, m["ranges"]["pmax"])
                lo, hi = -p, p  # Protocol representation, not mechanical travel limits.
            elif jog_window_rad is None:
                lo, hi = min(origin, target) - .05, max(origin, target) + .05
            else:
                lo, hi = origin-jog_window_rad-.05, origin+jog_window_rad+.05
            if not manual_travel and not (m['lower_rad'] <= lo < hi <= m['upper_rad']):
                raise RuntimeError(f"{m['unit']}: measured start/target corridor exceeds fixture limits")
            self.origins[i], self.bounds[i] = origin, (lo, hi)
        self.prepare_motion(manual_travel=manual_travel,position_hold=position_hold,group_hold=group_hold)

    def prepare_motion(self, *, manual_travel=False, observed=None, position_hold=False, group_hold=False):
        """Reverify mode/timeout/routes without changing origins or bounds."""
        if position_hold and (not manual_travel or not self.bus.capabilities & CAP_POSITION_HOLD):
            raise RuntimeError('Position hold requires manual travel and MC02 capability 0x10000')
        if group_hold and (not position_hold or not self.bus.capabilities & CAP_GROUP_HOLD):
            raise RuntimeError('Group hold requires position hold and MC02 capability 0x20000')
        current = self.observed if observed is None else observed
        for i in self.configured:
            self.stop()
            self.bus.write_routed_bench(i, 9, 2000)
            if current[i][10] != 2:
                self.stop()
                self.restore.add(i)  # Before write: ACK/readback may fail after motor changes mode.
                self.bus.write_routed_bench(i, 10, 2)
        for i in self.slots:
            m = self.motors[i]
            self.bus.prepare_routed_bench(i, m, *self.bounds[i], 2, manual_travel=manual_travel,
                                          position_hold=position_hold,group_hold=group_hold)
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

    def inspect_control(self):
        """Explicit disabled-only readback; preserve partial evidence and raw bits.

        No gain/current/limit writes, motion targets, enables or fault resets.
        A missing/nonfinite register is unavailable evidence, never a zero.
        """
        if not self.bus.capabilities & CAP_CONTROL_READBACK:
            raise RuntimeError('Control/protection readback requires reviewed MC02 firmware with capability 0x4000')
        if not self.configured or set(self.configured) != set(self.slots):
            raise RuntimeError('Prepare all selected disabled routes before control inspection')
        rows = self.report['control_registers'] = []
        self.report['control_readback_complete'] = False
        complete = True
        for i in self.configured:
            m = self.motors[i]
            row = dict(joint=i+1, unit=m['unit'], model=m['model'], bus=m['bus'],
                       can_id=m['can_id'], feedback_id=m['feedback_id'], registers={})
            rows.append(row)
            for reg, (name, kind) in CONTROL_REGISTERS.items():
                entry = row['registers'][name] = dict(address=reg, type=kind)
                if reg in (59, 60) and m['model']=='8009P':
                    entry.update(status='not_documented_for_model', value=None)
                    continue
                self.stop()  # Fresh stationary disabled ACK on every retained route.
                reply = self.bus.read_register(m['bus'], m['can_id'], reg)
                if reply is None:
                    entry.update(status='no_reply', value=None)
                    complete = False
                    continue
                feedback_id, raw = reply
                entry.update(reply_can_id=feedback_id, raw_uint32=raw, raw_hex=f'{raw:08x}')
                if feedback_id != m['feedback_id']:
                    entry.update(status='wrong_feedback_id', value=None)
                    raise RuntimeError(f"{m['unit']}: wrong feedback ID for {name}")
                value = decode_register(reg, raw)
                valid = math.isfinite(value)
                entry.update(status='ok' if valid else 'nonfinite', value=value if valid else None)
                complete &= valid
            print(f"{m['unit']}: disabled control/protection readback recorded", flush=True)
        self.report['control_readback_complete'] = bool(complete)
        if not complete:
            raise RuntimeError('Control/protection readback incomplete; inspect unavailable registers in the report')

    def command(self, i, target):
        self.bus.command(i, target, velocity=self.motors[i]['speed_rad_s'])

    def move(self, slots, targets, label):
        states = self.stop()
        holds = {i: states[i].position for i in slots}
        if any(abs(holds[i] - self.origins[i]) > .02 for i in slots):
            raise RuntimeError('Shaft moved away from its initial position before test')
        for i in slots:
            self.command(i, holds[i])
        enable_started = time.monotonic()
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
        enable_elapsed = start - enable_started
        print(f'{label}: CTRL_MODE=2 position-velocity; enable confirmed in {enable_elapsed:.3f}s; '
              'sending final target directly.', flush=True)
        first_motion = {i: None for i in slots}
        first_within_target = {i: None for i in slots}
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
                if first_motion[i] is None and abs(s.position-holds[i]) >= math.radians(.1):
                    first_motion[i] = now-start
                if first_within_target[i] is None and abs(s.position-targets[i]) <= math.radians(1):
                    first_within_target[i] = now-start
                reached = abs(s.position-targets[i]) <= math.radians(1) and abs(s.velocity) <= .03
                settled[i] = (settled[i] if settled[i] is not None else now) if reached else None
            if now >= next_print:
                print(f"{label} t={now-start:.3f}s", ', '.join(f"{self.motors[i]['unit']}: error={math.degrees(targets[i]-s.position):+.2f}deg "
                                      f"speed={s.velocity:+.3f}rad/s torque={s.torque:+.3f}Nm"
                                      for i, s in states.items()), flush=True)
                next_print = now + .25
            if all(t is not None and now-t >= .3 for t in settled.values()):
                self.report['stages'].append({'stage': label, 'passed': True, 'elapsed_s': now-start,
                    'enable_confirmation_s': enable_elapsed,
                    'motors': [{**peaks[i], 'unit': self.motors[i]['unit'], 'position_rad': s.position,
                                'target_rad': targets[i], 'error_rad': targets[i]-s.position,
                                'first_motion_observed_s': first_motion[i],
                                'first_within_one_degree_s': first_within_target[i]}
                               for i, s in states.items()]})
                self.stop()
                return
            time.sleep(.005)
        raise RuntimeError(f'{label}: target not reached and settled within 12 seconds')

    def run(self, simultaneous=False, group_first=False):
        if group_first and not simultaneous:
            raise ValueError("Group-first requires simultaneous testing")
        started = time.monotonic()
        print('Preparing position-velocity mode, timeout, stationary feedback and travel bounds...', flush=True)
        self.prepare()
        self.report['preparation_s'] = time.monotonic()-started
        print(f"Preparation complete in {self.report['preparation_s']:.3f}s; CTRL_MODE=2 verified.", flush=True)
        if simultaneous and group_first:
            self.move_group()
        # Excursion then return are separate bounded stages, with disable between stages.
        for i in self.slots:
            m = self.motors[i]
            origin = self.origins[i]
            target = origin + math.radians(m['delta_degrees'])
            self.move([i], {i: target}, f"{m['unit']} outward")
            self.origins[i] = self.stop()[i].position
            self.move([i], {i: origin}, f"{m['unit']} return")
            self.origins[i] = origin
        if simultaneous and not group_first:
            self.move_group()

    def move_group(self):
        origins = dict(self.origins)
        targets = {i: origins[i] + math.radians(self.motors[i]['delta_degrees']) for i in self.slots}
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


def recover_fault(bus, report, report_path):
    """Explicit disabled-only latch recovery; never enable or write motor parameters.

    Retained routes must all acknowledge disable, including slots omitted by the
    next plan. Older deployed firmware exposes ranges instead of a configured
    mask. Reset clears range-valid bits, so restore them before decoding feedback
    or allowing preflight to reuse those routes.
    """
    recovery = report['recovery'] = dict(reset_attempted=False, reset_acknowledged=False,
                                         reset_verified=False)
    if bus.capabilities & 911 != 911:  # Routed bench + retained first-fault sample.
        raise RuntimeError('Recovery requires routed bench and first-fault diagnostics firmware')
    states = {i: bus.state(i) for i in range(6)}
    recovery['states_before'] = {i: asdict(s) for i, s in states.items()}
    fault = recovery['fault_before'] = bus.fault_snapshot()
    if fault is None or bool(fault['fault']) != any(s.fault for s in states.values()):
        raise RuntimeError('Cannot establish a consistent retained fault; no reset sent')
    if any(s.enabled for s in states.values()):
        raise RuntimeError('Stop the active motor owner before recovery; no reset sent')
    retained = [i for i, s in states.items() if s.registers or s.stopping
                or any(v != 0 for v in (s.pmax, s.vmax, s.tmax))]
    if bus.capabilities & 4096:
        info = bus.base_info()
        recovery['base_info'] = info
        if info['id_session_locked']:
            raise RuntimeError('An ID-commissioning session requires an MCU restart, not fault recovery')
        retained = [i for i in range(6) if info['configured_mask'] & (1 << i)]
    if fault['fault'] and not retained:
        raise RuntimeError('Cannot verify retained motor routes for this fault; no reset sent')
    recovery['retained_slots'] = retained
    previous = BenchSession(bus, [], [], {})
    previous.configured = retained
    if retained:
        bus.configured = True
        bus.bench = True

    def disabled_feedback():
        # Stop ACK is available even when reset invalidated the range cache.
        bus.stop()
        for i in retained:
            previous.poll(i, lambda s: not s.enabled and not s.stopping and s.status == 0,
                          'recovery disable acknowledgement', faults=False)
        for i in retained:
            if bus.state(i).registers & 7 != 7:
                bus.probe(i)  # Read-only range/mode/timeout probe of existing route.
                previous.poll(i, lambda s: s.registers & 7 == 7,
                              'recovery range probe', faults=False)
        # Cancels any remaining mode probe. Cleanup may have restored mode 1
        # while MCU still expects 2; only range bits are needed to decode disable.
        disabled = previous.stop()
        if any(not all(math.isfinite(v) for v in (s.position, s.velocity, s.torque))
               or max(s.mos_c, s.rotor_c) >= 80 for s in disabled.values()):
            raise RuntimeError('Invalid/hot disabled feedback; no recovery verified')
        return disabled

    try:
        print('Recovery: confirming every retained motor is disabled and stationary...', flush=True)
        disabled = disabled_feedback()
        recovery['disabled_before'] = {i: asdict(s) for i, s in disabled.items()}
        if fault['fault']:
            # This unique companion file survives later report overwrites. A
            # write/fsync failure aborts before erasing the MCU first-fault latch.
            report_path = Path(report_path)
            report_path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', delete=False,
                    dir=report_path.parent, prefix=report_path.stem + '.fault-', suffix='.json') as evidence:
                recovery['evidence_path'] = str(Path(evidence.name).resolve())
                json.dump(report, evidence, indent=2)
                evidence.write('\n')
                evidence.flush()
                os.fsync(evidence.fileno())
            print(f"Saved pre-reset fault evidence: {recovery['evidence_path']}", flush=True)
            # Disk I/O can stall: renew the stationary disable check immediately
            # before reset instead of relying on feedback from before the write.
            disabled_feedback()
            recovery['reset_attempted'] = True
            bus.reset_fault()
            recovery['reset_acknowledged'] = True
            disabled = disabled_feedback()
        after = {i: bus.state(i) for i in range(6)}
        recovery['states_after'] = {i: asdict(s) for i, s in after.items()}
        recovery['fault_after'] = bus.fault_snapshot()
        if any(s.fault or s.enabled or s.stopping for s in after.values()) or recovery['fault_after']['fault']:
            raise RuntimeError('MC02 is still/newly faulted or not disabled after recovery')
        recovery['reset_verified'] = bool(fault['fault'])
        recovery['outcome'] = 'recovered' if fault['fault'] else 'no_latched_fault'
        print('Recovery complete; motors remain disabled. Running the normal connection check.'
              if fault['fault'] else 'No latched fault; no reset sent. Running the normal connection check.',
              flush=True)
    except BaseException as exc:
        recovery['error'] = str(exc)
        try:
            bus.stop()
        except (OSError, RuntimeError) as stop_error:
            recovery['stop_error'] = str(stop_error)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--all', action='store_true', help='six individual tests followed by concurrent outward/return')
    parser.add_argument('--group-first', action='store_true',
                        help='explicitly run the group before individual tests; requires --all')
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--check', action='store_true', help='check registers; refresh disable feedback if reusing a prior session, no enable or parameter writes')
    action.add_argument('--execute', action='store_true', help='run the confirmed unloaded test plan')
    parser.add_argument('--feedback', action='store_true',
                        help='with --check, capture fresh disabled shaft feedback; no enable/parameter writes')
    parser.add_argument('--supported', action='store_true',
                        help='confirm all attached links are supported when torque is disabled')
    parser.add_argument('--recover', action='store_true',
                        help='explicit fault recovery after addressing the cause; requires '
                             '--check --feedback --supported --report; never enables motors')
    parser.add_argument('--inspect-control', action='store_true',
                        help='read documented current/protection/gain registers while disabled; '
                             'requires --check --feedback --supported --report and MC02 capability 0x4000')
    parser.add_argument('--report', type=Path, help='JSON result, including failures and cleanup')
    args = parser.parse_args(argv)
    if args.feedback and (not args.check or not args.supported):
        parser.error('--feedback requires --check --supported because it sends torque-disable frames')
    if args.recover and (not args.check or not args.feedback or not args.supported or not args.report):
        parser.error('--recover requires --check --feedback --supported --report PATH')
    if args.inspect_control and (not args.check or not args.feedback or not args.supported or not args.report):
        parser.error('--inspect-control requires --check --feedback --supported --report PATH')
    report = {'passed': False, 'hardware_test': bool(args.execute), 'stages': [],
              'disable_acknowledged': False, 'mode_restoration_verified': False}
    bus = session = None
    result = 2
    try:
        plan = json.loads(args.plan.read_text())
        motors = validate_plan(plan, execute=args.execute, all_motors=args.all, group_first=args.group_first)
        joints = active_joints(plan)
        slots = [j-1 for j in joints]
        if args.execute and (len(joints) != len(motors) or plan.get('fixture_mode', 'unloaded') != 'unloaded'):
            raise ValueError('Subset/supported plans are for --check and manual jogging, not automatic sweeps')
        report['plan'] = plan
        report['active_joints'] = joints
        report['motor_parameter_writes'] = bool(args.execute)
        if not args.check and not args.execute:
            print(json.dumps(plan, indent=2))
            print('Plan validated offline. No serial device opened.')
            result = 0
            return result
        preflight_started = time.monotonic()
        print('Opening MC02 and checking motor IDs, ranges and original mode...', flush=True)
        bus = ArmBus(args.port)
        if args.inspect_control:
            if not bus.capabilities & CAP_CONTROL_READBACK:
                raise RuntimeError('Control/protection readback requires reviewed MC02 firmware with capability 0x4000')
            report['base_info'] = bus.base_info()
        if args.recover:
            recover_fault(bus, report, args.report)
        observed = dict(zip(slots, preflight(bus, [motors[i] for i in slots])))
        report['connection_and_preflight_s'] = time.monotonic()-preflight_started
        print(f"Connection/preflight complete in {report['connection_and_preflight_s']:.3f}s.", flush=True)
        report['registers'] = [observed[i] for i in slots]
        report['register_slots'] = slots
        if args.check:
            if args.feedback:
                session = BenchSession(bus, motors, observed, report, slots=slots)
                states = session.probe_disabled()
                report['feedback'] = [dict(joint=i+1, unit=motors[i]['unit'], bus=motors[i]['bus'],
                    can_id=motors[i]['can_id'], feedback_id=motors[i]['feedback_id'],
                    raw_position_rad=s.position, raw_position_deg=math.degrees(s.position),
                    velocity_rad_s=s.velocity, torque_nm=s.torque, age_ms=s.age_ms,
                    enabled=bool(s.enabled), status=s.status, mos_c=s.mos_c, rotor_c=s.rotor_c)
                    for i, s in states.items()]
                for row in report['feedback']:
                    print(f"A{row['joint']}: {row['raw_position_deg']:+.3f} deg raw; "
                          f"age={row['age_ms']} ms; disabled; CAN{row['bus']}/ID{row['can_id']}")
                omitted = sorted(set(range(1, len(motors)+1))-set(joints))
                if omitted: print('Unavailable/omitted:', ', '.join(f'A{j}' for j in omitted))
                if args.inspect_control:
                    session.inspect_control()
            else:
                print(json.dumps(observed, indent=2))
            result = 0
        else:
            session = BenchSession(bus, motors, observed, report)
            session.run(simultaneous=args.all, group_first=args.group_first)
            result = 0
    except KeyboardInterrupt:
        report['error'] = 'Interrupted'; result = 130
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        report['error'] = str(exc)
    finally:
        if session is not None:
            if result != 0:
                try:
                    session.capture_failure()
                except (OSError, RuntimeError, ValueError) as exc:
                    report['diagnostic_error'] = str(exc)
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
        print('PASS: ' + ('motion, disable and mode restoration verified.' if args.execute else 'ID/register checks; no enable or parameter writes.'))
    return result


if __name__ == '__main__':
    raise SystemExit(main())
