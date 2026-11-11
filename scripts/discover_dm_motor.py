#!/usr/bin/env python3
"""Read Damiao CAN IDs/settings without enable, disable or parameter writes."""
import argparse
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from motors.arm import ArmBus, DISCOVERY_REGISTERS, decode_register


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--bus', type=int, choices=(1, 2, 3), default=2)
    parser.add_argument('--first-id', type=int, default=0)
    parser.add_argument('--last-id', type=int, default=15,
                        help='inclusive; use 2046 to search all non-broadcast standard IDs')
    args = parser.parse_args()
    if not 0 <= args.first_id <= args.last_id <= 2046:
        parser.error('require 0 <= first-id <= last-id <= 2046')
    print(f'CAN{args.bus}: classic 1 Mbps; scanning IDs {args.first_id}..{args.last_id}.', flush=True)
    print('Register reads only. No enable, disable, zeroing or parameter writes.', flush=True)
    try:
        with ArmBus(args.port) as bus:
            if not bus.capabilities & 1:
                raise RuntimeError('Firmware lacks read-only CAN discovery; flash the updated forge_arm.elf first')
            for slot in range(6):
                state = bus.state(slot)
                if state.enabled or state.stopping or state.fault:
                    raise RuntimeError('MC02 is active/stopping/faulted; use a fresh controller reset before discovery')
            found = 0
            for motor_id in range(args.first_id, args.last_id + 1):
                result = None
                for _ in range(2):
                    result = bus.read_register(args.bus, motor_id, 8)
                    time.sleep(0.025)
                    if result is not None:
                        break
                if result is None:
                    if (motor_id - args.first_id + 1) % 16 == 0:
                        print(f'Scanned through ID {motor_id}...', flush=True)
                    continue
                feedback, esc_id = result
                if esc_id != motor_id:
                    raise RuntimeError(f'ID {motor_id} returned inconsistent ESC_ID={esc_id}; stop and inspect wiring/settings')
                found += 1
                print(f'FOUND command ID={motor_id} (0x{motor_id:03X}), reply CAN ID={feedback} (0x{feedback:03X})', flush=True)
                for reg, (name, _) in DISCOVERY_REGISTERS.items():
                    reply = bus.read_register(args.bus, motor_id, reg)
                    time.sleep(0.025)
                    if reply is None:
                        print(f'  {name}: no reply', flush=True)
                        continue
                    rx_id, raw = reply
                    if rx_id != feedback:
                        raise RuntimeError('Reply CAN ID changed during discovery; possible duplicate motor IDs')
                    value = decode_register(reg, raw)
                    extra = ''
                    if reg == 9:
                        extra = f' ({raw * 0.05:g} ms; zero does not meet FORGE timeout requirements)'
                    if reg == 10:
                        extra = {1: ' (MIT)', 2: ' (position-velocity)', 3: ' (velocity)', 4: ' (force-position)'}.get(raw, ' (unknown mode)')
                    print(f'  {name}: {value}{extra}', flush=True)
            if not found:
                print('No matching register replies. This does not prove the motor is faulty or needs an ID reset.')
                print('Check motor power, H/L, ground reference, termination and bitrate. Default scan covers only IDs 0..15.')
                return 1
            print(f'Found {found} responding ID(s). No motor was enabled. Send this output before motion commissioning.')
            return 0
    except KeyboardInterrupt:
        print('Discovery interrupted; no enable command was sent.', file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'DISCOVERY FAILED: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
