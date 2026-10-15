"""Terminal reader for the native Linux IMU; no ROS imports required."""
from .protocol import read_sample, decode

def main():
    import argparse
    import math
    import shutil
    import subprocess
    import sys
    import time
    from pathlib import Path
    import serial

    parser = argparse.ArgumentParser(description='Read HWT9052 high-precision RS485 IMU directly in Ubuntu.')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=9600)
    parser.add_argument('--address', type=lambda x: int(x, 0), default=0x50)
    parser.add_argument('--samples', type=int, default=0, help='Stop after N samples; 0 means continuously')
    args = parser.parse_args()
    if not 1 <= args.address <= 247 or args.samples < 0:
        parser.error('Address must be 1..247 and samples must be nonnegative')
    if not Path(args.port).exists():
        print(f'{args.port} is missing. Check the Linux serial device path and USB connection.', file=sys.stderr)
        return 1
    processes = subprocess.run(['ps', '-eo', 'args'], capture_output=True, text=True, check=True).stdout
    busy = any(('/lib/imu/imu' in line or 'ros2 run imu imu' in line) for line in processes.splitlines())
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            for fd in (process / 'fd').iterdir():
                try:
                    if fd.resolve() == Path(args.port).resolve():
                        busy = True
                except OSError:
                    pass
        except (OSError, PermissionError):
            pass
    if busy or (shutil.which('fuser') and subprocess.run(['fuser', '-s', args.port]).returncode == 0):
        print(f'{args.port} is in use. Stop the ROS IMU reader with Ctrl+C first.', file=sys.stderr)
        return 1
    try:
        with serial.Serial(args.port, args.baud, timeout=0.3, write_timeout=0.3, exclusive=True) as port:
            print(f'Reading {args.port}, {args.baud} baud, address 0x{args.address:02X}. Ctrl+C stops.')
            print('Angles are vendor roll/pitch/yaw, not a calibrated robot orientation.')
            count = 0
            failures = 0
            while not args.samples or count < args.samples:
                try:
                    acc, gyro, mag, angles = decode(read_sample(port, args.address))
                except ValueError as error:
                    failures += 1
                    print(f'No valid reading: {error}', file=sys.stderr, flush=True)
                    if failures >= 5:
                        return 1
                    time.sleep(0.2)
                    continue
                failures = 0
                count += 1
                angle_text = ', '.join(f'{math.degrees(x):8.2f}' for x in angles)
                acc_text = ', '.join(f'{x:7.3f}' for x in acc)
                gyro_text = ', '.join(f'{math.degrees(x):7.3f}' for x in gyro)
                mag_text = ', '.join(f'{x*1e6:7.2f}' for x in mag)
                print(f'{count:6d} | RPY deg [{angle_text}] | accel m/s² [{acc_text}] | gyro deg/s [{gyro_text}] | mag µT [{mag_text}]', flush=True)
                time.sleep(0.2)
    except KeyboardInterrupt:
        print('\nStopped.')
    except (OSError, serial.SerialException) as error:
        print(f'Cannot read IMU: {error}\nFor permission errors, run: newgrp dialout', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
