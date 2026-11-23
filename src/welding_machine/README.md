# Welding machine Modbus bring-up

FORGE has a standalone, read-only status probe for Modbus RTU and Modbus TCP.
Use the welder's model-specific manual to select the connection and the first
status address. Commission this connection independently of the arm and chassis.
The arm weld panel continues to run motion-only tests.

## What the supplied archive establishes

`modbus src.zip` contains libmodbus **3.1.4** transport and protocol source. Its
standard interfaces are `modbus_new_rtu` for serial and `modbus_new_tcp` for
native Modbus TCP. The modified `modbus-rtu.c` can route its UART operations
through `rtvm_client_uart.c` when `rtvm_client.h` detects an ACRN user VM.
`sos_vm.h` defines a Service-VM UART relay over UDP ports **12300-12303** for
guest device names `/dev/ttyS0` through `/dev/ttyS3`. The relay forwards
serial operations and requires the matching Service-VM server and its physical
UART. Its UDP ports are specific to that relay.

The archive supplies generic Modbus operations. It leaves the welder model,
connector pinout, serial settings, unit ID, register addresses and weld
command meanings unverified. FORGE uses PyModbus for direct RTU/TCP, with the
vendor archive retained as a local reference. Confirm the Service-VM deployment
with the supplier if their ACRN appliance is in the connection path.

## Connection options

| Confirmed welder interface | Connection from FORGE | Information to obtain |
| --- | --- | --- |
| Modbus RTU over two-wire RS-485 | Ubuntu/WSL USB port to an isolated, automatic-direction USB-to-RS-485 adapter, then the welder's RS-485 pair and specified signal common | Welder pinout, D0/D1 or +/- mapping, signal common, termination, bias and shielding instructions, baud, parity, stop bits, unit ID |
| Native Modbus TCP | Ethernet from the FORGE host to the welder on the machine network | Welder IP/hostname, port (standard Modbus TCP is 502), unit ID if required, and network settings |
| RTU welder via a Modbus TCP-to-RTU gateway | FORGE to gateway over Ethernet, then gateway to welder over the documented serial wiring | Gateway IP and TCP port, downstream unit ID and serial settings, and confirmation that the gateway converts Modbus TCP to RTU |
| Vendor ACRN UART relay | ACRN guest to the supplier's Service VM over the relay network, then the Service VM's UART to the welder | Vendor image/server availability, relay address, UART mapping, and physical port wiring |

For RS-485, match the documented signal names at both ends. A/B and +/- labels
vary between vendors, and an RJ45 connector can carry either serial or
Ethernet. Verify the electrical interface and pinout before connecting. Follow
the manufacturer's instructions for grounding, common, shielding and
termination. Select an adapter for the specified electrical interface if the
manual calls for four-wire RS-485 or RS-232. Use one client on a serial bus. For TCP, verify reachability from
Ubuntu or WSL on the intended machine network. A raw serial-over-IP socket and
the ACRN UDP relay use different framing from Modbus TCP.

For a USB adapter in WSL 2, keep Ubuntu running and list devices with
`usbipd list` in Windows. Use `usbipd bind --busid <BUSID>` from an elevated
PowerShell the first time, then `usbipd attach --wsl --busid <BUSID>`. In WSL,
verify the adapter with `lsusb` and check `/dev/serial/by-id/` or the assigned
`/dev/ttyUSB*` node. Use the port whose identity matches the adapter. On native
Ubuntu, connect the adapter directly and check the same Linux device paths.

## First status read

1. Record the welder make/model, firmware, connector label, and the manual or
   register-map revision. Keep the arc disabled under the machine's documented
   commissioning procedure.
2. Confirm the transport settings and unit ID. Select one documented status
   point whose value can be compared with the machine panel.
3. Identify its table, zero-based protocol offset, width and data type. For
   example, if the manual uses conventional one-based holding-register labels,
   `40001` corresponds to PDU offset `0`. Use the manual's addressing convention.
4. Install the dependency from the FORGE root and run the matching command,
   setting the variables from the machine documentation.

```bash
.venv/bin/python -m pip install -e '.[welder]'

# Direct RTU serial
.venv/bin/python scripts/read_welder.py rtu \
  --port "$WELDER_SERIAL_PORT" --baudrate "$WELDER_BAUD" \
  --parity "$WELDER_PARITY" --stopbits "$WELDER_STOP_BITS" \
  --unit "$WELDER_UNIT" --table "$WELDER_TABLE" --address "$WELDER_OFFSET"

# Native Modbus TCP or a Modbus TCP-to-RTU gateway
.venv/bin/python scripts/read_welder.py tcp \
  --host "$WELDER_HOST" --tcp-port "$WELDER_TCP_PORT" \
  --unit "$WELDER_UNIT" --table "$WELDER_TABLE" --address "$WELDER_OFFSET"
```

The probe returns one JSON result with the table, PDU address and raw values.
Use `--count` for a documented contiguous block. A failed read exits 2. A
Modbus exception identifies the unit, function and exception code. A timeout
or malformed response is reported as a failed read. Requests have no automatic
retries. Check the unit, port, framing and table against the manual before
repeating. Confirm signed values, multiword order, units and scaling against
the documented signal and machine panel before turning a raw value into a
named status field.

The Python API exposes the same raw reads through `WelderClient.rtu(...)` and
`WelderClient.tcp(...)`. Use the client in a context manager and serialize calls.
The supported tables are `coils` (01), `discrete-inputs` (02),
`holding-registers` (03) and `input-registers` (04). Arc enable, configuration
writes and arm-welder interlocks require a verified machine map and separate
commissioning.

## Offline verification and references

```bash
PYTHONPATH=tests .venv/bin/python -m unittest test_welding_machine -v
```

Tests use a loopback TCP device and a Linux pseudo-terminal for RTU.
They verify addresses, read functions, replies, CRC handling, device exceptions
and disconnects. Record the first physical result separately in the
[welder TODO](TODO.md).

- [Modbus application protocol and register functions](https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf)
- [Modbus serial physical-layer and cabling guide](https://www.modbus.org/file/secure/modbusoverserial.pdf)
- [Modbus TCP default port and API](https://libmodbus.org/reference/modbus_new_tcp/)
- [Microsoft USB attachment instructions for WSL](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
