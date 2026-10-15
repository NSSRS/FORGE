"""Read-only WIT high-precision Modbus telemetry, following the vendor SDK.

https://github.com/WITMOTION/WitHighModbus_HWT9073485
"""
import struct


def crc(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xa001 if value & 1 else 0)
    return value


def read_sample(port, address=0x50):
    request = struct.pack('>BBHH', address, 3, 0x34, 15)
    port.reset_input_buffer()
    port.write(request + struct.pack('<H', crc(request)))
    response = port.read(35)
    if len(response) != 35 or response[:3] != bytes([address, 3, 30]):
        raise ValueError('Missing or unexpected Modbus response')
    if crc(response[:-2]) != struct.unpack('<H', response[-2:])[0]:
        raise ValueError('Modbus CRC mismatch')
    return list(struct.unpack('>15H', response[3:-2]))


def decode(regs):
    if len(regs) != 15 or any(type(x) is not int or not 0 <= x <= 65535 for x in regs):
        raise ValueError('Invalid register block')
    def signed(value, bits):
        return value - (1 << bits) if value & (1 << (bits - 1)) else value
    import math
    acceleration = [signed(x, 16) * 16 / 32768 * 9.80665 for x in regs[:3]]
    gyro = [math.radians(signed(x, 16) * 2000 / 32768) for x in regs[3:6]]
    magnetic = [signed(x, 16) * 13e-9 for x in regs[6:9]]
    angles = [math.radians(signed(regs[i] | regs[i+1] << 16, 32) / 1000) for i in (9,11,13)]
    return acceleration, gyro, magnetic, angles
