"""Read one documented Modbus address block from a welding machine."""

import argparse
import json
import sys

from .client import READ_TABLES, WelderClient, WelderError


def _number(value):
    """Accept decimal and 0x-prefixed hexadecimal addresses and unit IDs."""
    return int(value, 16 if value.lower().startswith("0x") else 10)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="transport", required=True)
    rtu = modes.add_parser("rtu", help="Local Modbus RTU serial port")
    rtu.add_argument("--port", required=True, help="Serial device, for example /dev/ttyUSB0")
    rtu.add_argument("--baudrate", required=True, type=int)
    rtu.add_argument("--parity", required=True, choices=("N", "E", "O"))
    rtu.add_argument("--stopbits", required=True, choices=(1, 2), type=int)
    tcp = modes.add_parser("tcp", help="Native Modbus TCP or a Modbus TCP-to-RTU gateway")
    tcp.add_argument("--host", required=True)
    tcp.add_argument("--tcp-port", type=int, default=502)

    for mode in (rtu, tcp):
        mode.add_argument("--unit", required=True, type=_number, help="Vendor-documented unit ID")
        mode.add_argument("--table", required=True, choices=READ_TABLES)
        mode.add_argument("--address", required=True, type=_number, help="Zero-based protocol address")
        mode.add_argument("--count", type=int, default=1)
        mode.add_argument("--timeout", type=float, default=1.0, help="Connection and response timeout in seconds")

    args = parser.parse_args(argv)
    try:
        if args.transport == "rtu":
            client = WelderClient.rtu(
                args.port, unit=args.unit, baudrate=args.baudrate, parity=args.parity,
                stopbits=args.stopbits, timeout=args.timeout,
            )
        else:
            client = WelderClient.tcp(
                args.host, unit=args.unit, port=args.tcp_port, timeout=args.timeout,
            )
        with client:
            values = client.read(args.table, args.address, count=args.count)
    except (ValueError, WelderError) as exc:
        print(f"Welder read failed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(dict(
        transport=args.transport, unit=args.unit, table=args.table, address=args.address,
        count=args.count, values=values,
    )))
    return 0
