"""Identify the existing CH340 ESP32 by its CSV stream before starting nodes.

USB identity is shared with the IMU; a by-id link is not enough. Discovery is
bounded, receives only, and never probes the STM32 motor controller.
"""
import argparse
from pathlib import Path
import time

from teleop.radio_input import parse_channels


# Observed radio USB bridge. Do not expand to arbitrary serial/motor adapters.
RADIO_USB_ID = (0x1A86, 0x7523)


def probe_radio(port, *, timeout=1.5, serial_factory=None):
    """Require five consecutive fresh radio rows; discard them after discovery."""
    if serial_factory is None:
        from serial import Serial
        serial_factory = Serial
    stream = serial_factory(port=None, baudrate=115200, timeout=0, exclusive=True)
    try:
        # Set line state before opening; do not intentionally reset an ESP32.
        stream.dtr = stream.rts = False
        stream.port = port
        stream.open()
        stream.reset_input_buffer()
        pending = bytearray()
        fresh = 0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pending.extend(stream.read(min(stream.in_waiting, 512)))
            while b'\n' in pending:
                line, _, pending = pending.partition(b'\n')
                try:
                    age, _ = parse_channels(line.decode('ascii', errors='replace'))
                    fresh = fresh + 1 if age < .15 else 0
                except ValueError:
                    fresh = 0
                if fresh >= 5:
                    return True
            if len(pending) > 512:
                pending.clear()
                fresh = 0
            time.sleep(.01)
        return False
    finally:
        stream.close()


def resolve_radio_port(requested='auto', *, exclude=(), ports=None, probe=None):
    """Return an explicit path unchanged, or require exactly one verified radio.

    Errors/busy ports make discovery incomplete; do not pick another device as
    a fallback. An explicit path remains available for different bridge types.
    """
    if requested != 'auto':
        if not requested:
            raise ValueError('Specify radio_port:=auto or an explicit serial path')
        return requested
    if ports is None:
        from serial.tools.list_ports import comports
        ports = comports()
    if probe is None:
        probe = probe_radio
    excluded = {Path(p).resolve() for p in exclude if p and p != 'auto'}
    candidates = sorted({str(Path(p.device).resolve()) for p in ports
                         if (p.vid, p.pid) == RADIO_USB_ID
                         and Path(p.device).resolve() not in excluded})
    if not candidates:
        raise ValueError('No eligible CH340 radio adapter found. Check USB and excluded IMU/arm '
                         'paths, or set radio_port:=/dev/serial/by-path/ACTUAL_RADIO')
    matches, errors = [], []
    for port in candidates:
        try:
            if probe(port):
                matches.append(port)
        except OSError as exc:
            errors.append(f'{port}: {exc}')
    if errors:
        raise ValueError('Radio discovery incomplete; close other serial readers/check permissions '
                         'or specify radio_port explicitly. ' + '; '.join(errors))
    if len(matches) > 1:
        raise ValueError('Multiple radio CSV streams found: ' + ', '.join(matches) +
                         '; specify radio_port:=/dev/serial/by-path/ACTUAL_RADIO')
    if not matches:
        raise ValueError('No fresh ESP32 radio CSV found on ' + ', '.join(candidates) +
                         '. Check receiver/transmitter and close other readers; '
                         'or specify radio_port explicitly.')
    return matches[0]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exclude', action='append', default=[], metavar='PORT',
                        help='Known IMU/other serial device to leave unopened; repeat as needed')
    args = parser.parse_args(argv)
    try:
        print(resolve_radio_port(exclude=args.exclude))
    except (ValueError, OSError) as exc:
        parser.exit(1, f'{exc}\n')


if __name__ == '__main__':
    main()
