#!/usr/bin/env python3
"""Bounded first movement of the secured, unloaded CAN2/ID1 J4340P bench motor."""
import argparse
import math
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from motors.arm import ArmBus, decode_register


def poll_until(bus, predicate, description, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        state = bus.state(0)
        if state.fault:
            raise RuntimeError(f'MC02 fault during {description}: {state.fault_detail()}')
        if state.status >= 8:
            raise RuntimeError(f'Motor status {state.status} during {description}; no automatic fault clear')
        if predicate(state):
            return state
        time.sleep(0.01)
    raise TimeoutError(f'Timed out waiting for {description}')


def probe_and_stop(bus):
    bus.probe(0)
    poll_until(bus, lambda s: s.registers == 31, 'range/mode/timeout checks')
    bus.stop()
    return poll_until(bus, lambda s: not s.enabled and not s.stopping and s.status == 0 and s.age_ms < 80,
                      'disabled position feedback')


def run(bus):
    delta, duration = math.radians(45), 12.0
    if bus.capabilities & 143 != 143:
        raise RuntimeError('Flash the position-velocity bench ELF first')
    # Fixed identity and mode, using an independent read for each observed setting.
    for reg, expected in ((7, 0), (8, 1)):
        if bus.read_register(2, 1, reg) != (0, expected):
            raise RuntimeError(f'CAN2/ID1 register {reg} does not match the discovered bench motor')
    mode_reply = bus.read_register(2, 1, 10)
    if mode_reply is None or mode_reply[0] != 0 or mode_reply[1] not in (1, 2):
        raise RuntimeError('Unexpected initial control mode; no mode change or enable performed')
    original_mode = mode_reply[1]
    damping_reply = bus.read_register(2, 1, 31)
    if damping_reply is None or damping_reply[0] != 0:
        raise RuntimeError('No velocity-loop damping readback')
    damping = decode_register(31, damping_reply[1])
    if not math.isfinite(damping) or damping <= 0:
        raise RuntimeError('Motor velocity-loop damping must be positive; configure with Damiao tool')
    print(f'Position-velocity precheck: existing damping factor={damping:g}; no internal gains changed.', flush=True)
    for reg, expected in ((21, 12.5), (22, 10.0), (23, 28.0)):
        reply = bus.read_register(2, 1, reg)
        if reply is None or reply[0] != 0:
            raise RuntimeError(f'No matching range reply for register {reg}')
        actual = decode_register(reg, reply[1])
        if not math.isclose(actual, expected, rel_tol=1e-5):
            raise RuntimeError(f'Range register {reg} changed: {actual}; review before motion')
    bus.set_bench_timeout()
    print('Motor timeout set and independently verified: 100 ms (not saved to flash).', flush=True)
    bus.prepare_bench(-12.5, 12.5, posvel=original_mode == 2)  # Firmware explicitly forbids enable with this wide interval.
    initial = probe_and_stop(bus)
    origin = initial.position
    if not math.isfinite(origin) or abs(origin) > 12.3:
        raise RuntimeError('Position too close to protocol range boundary for this test')
    upper = origin + delta + 0.15
    if original_mode != 2:
        # Record restoration responsibility before transmission (ACK can be lost).
        bus.bench_original_mode = original_mode
        bus.set_bench_mode(2)
        print('CTRL_MODE=2 acknowledged and independently verified; not saved to flash.', flush=True)
    bus.prepare_bench(origin - 0.15, upper, posvel=True)
    initial = probe_and_stop(bus)
    if abs(initial.position - origin) > 0.02 or abs(initial.velocity) > 0.03:
        raise RuntimeError('Shaft moved during preparation; stop and check the bench')
    print(f'Start {origin:.4f} rad; target offset +{delta:.4f} rad ({math.degrees(delta):.1f} degrees), duration {duration:g} seconds.', flush=True)
    print('Position-velocity mode: CAN ID 0x101, speed limit 0.1 rad/s; measured torque stop=3 Nm.', flush=True)
    command_args = dict(velocity=0.1)
    bus.command(0, initial.position, **command_args)
    bus.enable(0)
    enable_started = time.monotonic()
    while True:
        state = bus.state(0)
        if state.fault or not state.enabled or state.age_ms >= 80:
            raise RuntimeError(f'Enable confirmation failed: {state.fault_detail()}')
        if state.enable_confirmed and state.status == 1:
            print(f'Motor enabled confirmed after {(time.monotonic()-enable_started)*1000:.0f} ms.', flush=True)
            break
        if time.monotonic()-enable_started > 0.4:
            raise TimeoutError('No enabled feedback; no movement target sent')
        bus.command(0, initial.position, **command_args)
        time.sleep(0.01)
    started = time.monotonic()
    report_at = started
    max_travel = 0.0
    settled_since = None
    displacement = 0.0
    while True:
        now = time.monotonic()
        elapsed = now - started
        if elapsed >= duration:
            break
        state = bus.state(0)
        if (state.fault or not state.enabled or state.age_ms >= 80 or
                state.status >= 8 or state.status != 1):
            raise RuntimeError(f'Motion stopped: fault={state.fault}, enabled={state.enabled}, '
                               f'status={state.status}, feedback age={state.age_ms} ms; {state.fault_detail()}; '
                               f'latest position={state.position:+.4f}, speed={state.velocity:+.4f}, torque={state.torque:+.4f}')
        if not all(math.isfinite(v) for v in (state.position, state.velocity, state.torque)):
            raise RuntimeError('Non-finite motor feedback')
        displacement = state.position - origin
        max_travel = max(max_travel, abs(displacement))
        upper_travel = delta + 0.14
        torque_limit = 3.0
        violations = []
        if displacement <= -0.14 or displacement >= upper_travel:
            violations.append(f'travel {displacement:+.4f} rad outside (-0.14, {upper_travel:.4f})')
        if abs(state.velocity) > 0.3:
            violations.append(f'speed {state.velocity:+.4f} rad/s exceeds 0.3')
        if abs(state.torque) > torque_limit:
            violations.append(f'measured torque {state.torque:+.4f} Nm exceeds {torque_limit}')
        if violations:
            raise RuntimeError('Bench limit: ' + '; '.join(violations) +
                               f'; position={state.position:+.4f}, speed={state.velocity:+.4f}, torque={state.torque:+.4f}')
        target = origin + delta
        bus.command(0, target, **command_args)
        if abs(delta-displacement) <= math.radians(1) and abs(state.velocity) <= 0.03:
            if settled_since is None:
                settled_since = now
            elif now-settled_since >= 0.3:
                print('Target reached within 1 degree and settled for 0.3 seconds.', flush=True)
                break
        else:
            settled_since = None
        if now >= report_at:
            print(f'  t={elapsed:.2f}s, offset={displacement:+.4f} rad, speed={state.velocity:+.3f} rad/s, '
                  f'torque={state.torque:+.3f} Nm, target_offset={target-origin:+.4f} rad, '
                  f'error={target-state.position:+.4f} rad', flush=True)
            report_at = now + 0.25
        time.sleep(0.02)
    print(f"Final measured offset: {math.degrees(displacement):+.2f} degrees; requested +45 degrees.", flush=True)
    return max_travel, delta-displacement, (settled_since is not None and now-settled_since >= 0.3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--degrees', type=int, choices=[45], default=45, help='slow +45 degree test (requires updated firmware)')
    parser.add_argument('--mode', choices=['pos-vel'], default='pos-vel', help='default: vendor position-velocity mode')
    parser.add_argument('--execute', action='store_true', help='authorize the secured +45 degree test')
    args = parser.parse_args()
    if not args.execute:
        parser.error('requires --execute; use only the secured, unloaded single 4340P on CAN2')
    bus = None
    try:
        bus = ArmBus(args.port)
        try:
            travel, final_error, settled = run(bus)
        finally:
            if bus.configured:
                bus.stop()
                # Faults may be latched; still require a motor disable acknowledgement.
                end = time.monotonic() + 1.0
                while time.monotonic() < end:
                    state = bus.state(0)
                    if not state.enabled and not state.stopping and state.status == 0:
                        print('Motor disable acknowledged.', flush=True)
                        if hasattr(bus, 'bench_original_mode'):
                            try:
                                bus.set_bench_mode(bus.bench_original_mode)
                                print(f'Original CTRL_MODE={bus.bench_original_mode} restored and verified.', flush=True)
                            except (OSError, RuntimeError, ValueError) as exc:
                                raise RuntimeError(f'Motor disabled, but original mode restoration failed: {exc}; '
                                                   'power-cycle motor before another test') from exc
                        break
                    time.sleep(0.01)
                else:
                    raise RuntimeError('Disable not acknowledged: turn OFF motor PSU')
        if travel < 0.01:
            print('Test ended; little/no motion detected. Do not raise gains automatically; send the output.')
            return 1
        if abs(final_error) > math.radians(1) or not settled:
            print(f'Target NOT reached/settled: remaining error {math.degrees(final_error):+.2f} degrees. Motor disabled.')
            return 1
        print(f'Test ended; maximum measured travel {travel:.4f} rad. Motor disabled.')
        return 0
    except KeyboardInterrupt:
        print('Interrupted. Check disable acknowledgement above; turn off motor PSU if uncertain.', file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'BENCH TEST STOPPED: {exc}', file=sys.stderr)
        return 2
    finally:
        if bus is not None:
            try:
                bus.close()
            except (OSError, RuntimeError):
                print('USB close/stop failed: turn OFF motor PSU.', file=sys.stderr)


if __name__ == '__main__':
    raise SystemExit(main())
