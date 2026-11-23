"""Explicit-address Modbus reads for welder commissioning."""

import math


# Function code, PyModbus method, and protocol maximum per request.
READ_TABLES = {
    "coils": (1, "read_coils", 2000),
    "discrete-inputs": (2, "read_discrete_inputs", 2000),
    "holding-registers": (3, "read_holding_registers", 125),
    "input-registers": (4, "read_input_registers", 125),
}


class WelderError(RuntimeError):
    """The connection or response could not be used."""


class WelderDeviceError(WelderError):
    """The addressed unit returned a Modbus exception."""

    def __init__(self, unit, function, code):
        self.unit, self.function, self.code = unit, function, code
        super().__init__(f"Unit {unit} rejected function 0x{function:02x} with exception 0x{code:02x}")


def _require_int(value, name, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer from {minimum} to {maximum}")


def _client_module(endpoint, unit, unit_min, unit_max, timeout):
    if not isinstance(endpoint, str) or not endpoint.strip():
        raise ValueError("A serial port or TCP host is required")
    _require_int(unit, "Unit ID", unit_min, unit_max)
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("Timeout must be a finite positive number of seconds")
    try:
        from pymodbus import client
    except ModuleNotFoundError as exc:
        if exc.name != "pymodbus":
            raise
        raise WelderError("Install the welder extra with python -m pip install -e '.[welder]'") from exc
    return client


class WelderClient:
    """One synchronous connection. Use a context manager and serialize calls."""

    def __init__(self, transport, unit, endpoint):
        self._transport = transport
        self._unit = unit
        self._endpoint = endpoint

    @classmethod
    def rtu(cls, port, *, unit, baudrate, parity, stopbits, timeout=1.0):
        """Configure an 8-bit Modbus RTU serial endpoint with explicit framing."""
        if type(baudrate) is not int or baudrate < 1:
            raise ValueError("Baud rate must be a positive integer")
        if parity not in ("N", "E", "O"):
            raise ValueError("Parity must be N, E or O")
        _require_int(stopbits, "Stop bits", 1, 2)
        client = _client_module(port, unit, 1, 247, timeout)
        try:
            transport = client.ModbusSerialClient(
                port, baudrate=baudrate, bytesize=8, parity=parity, stopbits=stopbits,
                timeout=timeout, retries=0,
            )
        except RuntimeError as exc:
            raise WelderError(f"Cannot set up serial port {port}: {exc}") from exc
        return cls(transport, unit, port)

    @classmethod
    def tcp(cls, host, *, unit, port=502, timeout=1.0):
        """Configure a native Modbus TCP endpoint, including TCP-to-RTU gateways."""
        _require_int(port, "TCP port", 1, 65535)
        client = _client_module(host, unit, 0, 255, timeout)
        return cls(client.ModbusTcpClient(host, port=port, timeout=timeout, retries=0),
                   unit, f"{host}:{port}")

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()

    def close(self):
        self._transport.close()

    def read(self, table, address, *, count=1):
        """Read raw bools or unsigned 16-bit words at a zero-based PDU address."""
        if table not in READ_TABLES:
            raise ValueError(f"Unknown Modbus table: {table}")
        function, method, maximum = READ_TABLES[table]
        _require_int(address, "Address", 0, 65535)
        _require_int(count, "Count", 1, maximum)
        if address + count > 65536:
            raise ValueError("Address range extends beyond the Modbus table")

        from pymodbus.exceptions import ModbusException
        from pymodbus.pdu import ExceptionResponse

        try:
            response = getattr(self._transport, method)(address, count=count, device_id=self._unit)
            if response.dev_id == self._unit and response.function_code == (function | 0x80):
                if isinstance(response, ExceptionResponse):
                    raise WelderDeviceError(self._unit, function, response.exception_code)
            if response.isError() or response.dev_id != self._unit or response.function_code != function:
                raise WelderError("Unexpected Modbus response function or unit")
            if function <= 2:
                values = response.bits
                expected = (count + 7) // 8 * 8
            else:
                values = response.registers
                expected = count
            if len(values) != expected:
                raise WelderError(f"Expected {expected} decoded values, received {len(values)}")
            return tuple(values[:count])
        except WelderDeviceError:
            raise
        except (ModbusException, OSError, WelderError) as exc:
            self.close()
            raise WelderError(f"Read from {self._endpoint}, unit {self._unit}, failed: {exc}") from exc
