"""Damiao arm over FORGE DM-MC02 USB firmware (not the ENCOS bus).

No background refresh: application must supply fresh commands within 150 ms.
Opening a port or reading status never enables motors. SI output-shaft units.
"""
from dataclasses import dataclass
import json
import math
import struct
import threading
import time
import zlib

MAGIC = b'\xa5\x5a'


def frame(kind, sequence, payload=b''):
    if len(payload) > 52:
        raise ValueError('payload too long')
    data = struct.pack('<2sBBI', MAGIC, 1, kind, sequence) + payload.ljust(52, b'\0')
    return data + struct.pack('<I', zlib.crc32(data))


def decode(data):
    if len(data) != 64 or data[:3] != MAGIC + b'\x01' or zlib.crc32(data[:60]) != struct.unpack_from('<I', data, 60)[0]:
        raise ValueError('invalid MC02 frame')
    return data[3], struct.unpack_from('<I', data, 4)[0], data[8:60]


@dataclass(frozen=True)
class ArmState:
    fault: bool
    enabled: bool
    registers: int
    status: int
    mos_c: int
    rotor_c: int
    stopping: bool
    age_ms: int
    position: float
    velocity: float
    torque: float
    pmax: float
    vmax: float
    tmax: float


class ArmBus:
    def __init__(self, port=None, *, transport=None):
        if transport is None:
            import serial
            transport = serial.Serial(port, 115200, timeout=0.02, write_timeout=0.1, exclusive=True)
        self.serial = transport
        self.sequence = 0
        self.lock = threading.Lock()
        self.buffer = bytearray()
        self.configured = False
        self.closed = False
        try:
            reply = self._request(1)
            if not reply[1:].startswith(b'FORGE-MC02/1'):
                raise RuntimeError('The USB device is not running FORGE MC02 firmware')
            self.update_hz = struct.unpack_from('<I', reply, 13)[0]
            if self.update_hz != 500:
                raise RuntimeError('MC02 firmware does not report the required 500 Hz update rate; rebuild/flash the matching image')
        except Exception:
            self.serial.close()
            raise

    def _request(self, kind, payload=b''):
        with self.lock:
            if self.closed:
                raise RuntimeError('arm bus is closed')
            self.sequence += 1
            packet = frame(kind, self.sequence, payload)
            if self.serial.write(packet) != len(packet):
                raise OSError('short USB write')
            deadline = time.monotonic() + 0.1
            while time.monotonic() < deadline:
                self.buffer.extend(self.serial.read(64))
                while len(self.buffer) >= 64:
                    if self.buffer[:2] != MAGIC:
                        del self.buffer[0]
                        continue
                    try:
                        rkind, seq, result = decode(self.buffer[:64])
                    except ValueError:
                        del self.buffer[0]
                        continue
                    del self.buffer[:64]
                    if seq != self.sequence or rkind != kind | 0x80:
                        continue
                    if result[0]:
                        raise RuntimeError(f'MC02 rejected command {kind}; check configuration, state and watchdogs')
                    return result
            raise TimeoutError('MC02 response timeout; firmware command leases are not refreshed')

    def configure(self, profile):
        """Load a measured hardware profile; deliberately rejects the shipped template."""
        if isinstance(profile, (str, bytes)) or hasattr(profile, '__fspath__'):
            with open(profile) as stream:
                profile = json.load(stream)
        if profile.get('hardware_confirmed') is not True or profile.get('joint_order_confirmed') is not True:
            raise ValueError('Confirm wiring, joint order and measured limits in the arm profile first')
        if (profile.get('baud_rate', 1000000) != 1000000 or
                profile.get('can_frame_format', 'classic_11bit') != 'classic_11bit' or
                profile.get('can_update_hz', 500) != 500):
            raise ValueError('firmware requires classic 11-bit CAN at 1 Mbps and 500 Hz')
        joints = profile['joints']
        if len(joints) != 6:
            raise ValueError('exactly six arm joints are required')
        packets=[]; ids=set(); feedback=set()
        for i, joint in enumerate(joints):
            bus, motor_id, rx_id = (joint[k] for k in ('bus', 'can_id', 'feedback_id'))
            if any(type(v) is not int for v in (bus, motor_id, rx_id)) or not 1 <= bus <= 3 or not 1 <= motor_id <= 15 or not 16 <= rx_id < 0x7ff:
                raise ValueError('invalid CAN routing')
            if (bus, motor_id) in ids or (bus, rx_id) in feedback:
                raise ValueError('duplicate CAN address')
            ids.add((bus,motor_id)); feedback.add((bus,rx_id))
            limits = joint['limits']
            vals = [limits[k] for k in ('lower_rad','upper_rad','speed_rad_s','torque_nm','kp','kd')]
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in vals):
                raise ValueError('all measured limits must be finite numbers')
            lo,hi,speed,torque,kp,kd=vals
            if not (lo < hi and speed > 0 and torque > 0 and 0 <= kp <= 500 and 0 <= kd <= 5):
                raise ValueError('invalid joint limits')
            packets.append(struct.pack('<BBHH6f',i,bus,motor_id,rx_id,*vals))
        # Validate the entire profile before any configuration writes.
        for packet in packets:
            self._request(2,packet)
        self.configured=True

    @staticmethod
    def _slot(slot):
        if type(slot) is not int or not 0 <= slot < 6:
            raise ValueError('joint slot must be 0..5')
        return bytes([slot])

    def probe(self, slot):
        """Read range, mode and timeout registers. No enable or register writes."""
        self._request(3,self._slot(slot))

    def command(self, slot, position, velocity=0., kp=0., kd=0., torque=0.):
        values=(position,velocity,kp,kd,torque)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite command')
        self._request(4,self._slot(slot)+struct.pack('<5f',*values))

    def enable(self, slot):
        """Requires fresh disabled feedback and a fresh command near measured position."""
        self._request(5,self._slot(slot))

    def stop(self):
        """Request torque disable on all configured arm motors; this is not a brake."""
        self._request(6)

    def state(self, slot):
        p=self._request(7,self._slot(slot))
        return ArmState(*struct.unpack_from('<7BI6f',p,1))

    def reset_fault(self):
        """Requires acknowledged disabled motors; re-probing is needed afterwards."""
        self._request(8)

    def close(self):
        if not self.closed:
            try:
                if self.configured:
                    self.stop()
            finally:
                self.closed=True
                self.serial.close()

    def __enter__(self): return self
    def __exit__(self,*_): self.close()
