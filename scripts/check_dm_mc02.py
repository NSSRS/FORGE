#!/usr/bin/env python3
"""Check FORGE MC02 USB firmware identity without configuring/enabling motors."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from motors.arm import ArmBus


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0')
    args = parser.parse_args()
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print('Missing pyserial. Install with: sudo apt install python3-serial', file=sys.stderr)
        return 2
    print('Visible serial devices:')
    for device in list_ports.comports():
        print(f'  {device.device}: {device.description} (serial {device.serial_number})')
    print(f'Checking {args.port}; USB identity/status only. No enable, configuration or CAN motion commands.')
    try:
        with ArmBus(args.port) as bus:
            print(f'FORGE MC02 handshake OK; firmware reports {bus.update_hz} Hz.')
            for slot in range(6):
                state = bus.state(slot)
                age = 'none' if state.age_ms == 0xffffffff else f'{state.age_ms} ms'
                print(f'  slot {slot}: registers={state.registers}/31, enabled={state.enabled}, '
                      f'fault={state.fault}, status={state.status}, feedback_age={age}')
            print('USB connection verified. This does not discover or verify the CAN2 motor ID.')
        return 0
    except (OSError, RuntimeError, serial.SerialException) as exc:
        print(f'CHECK FAILED: {exc}', file=sys.stderr)
        print('A timeout can mean different/unflashed firmware, an incorrect port, or a connection problem.', file=sys.stderr)
        print('No motor was enabled by this script. Send the complete output back.', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
