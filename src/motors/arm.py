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
FAULT_REASONS = {0: 'not recorded', 1: 'host command lease expired', 2: 'CAN feedback expired',
                 3: 'motor did not stay enabled', 4: 'command encoding rejected',
                 5: 'CAN transmit queue failure', 6: 'motor reported an error',
                 7: 'position limit', 8: 'speed limit', 9: 'measured torque limit',
                 10: 'bench duration cutoff', 11: 'temperature limit',
                 12: 'USB receive overflow', 13: 'CAN bus-off/status failure', 14: 'USB disconnected'}


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
    fault_reason: int = 0
    fault_status: int = 0
    fault_slot: int = 0
    enable_flags: int = 0
    fault_command_age_ms: int = 0
    fault_feedback_age_ms: int = 0
    fault_at_ms: int = 0

    @property
    def enable_confirmed(self):
        return bool(self.enable_flags & 1)

    def fault_detail(self):
        return (f'{FAULT_REASONS.get(self.fault_reason, "unknown fault")} (code={self.fault_reason}, '
                f'slot={self.fault_slot}, motor status at fault={self.fault_status}, '
                f'command age={self.fault_command_age_ms} ms, feedback age={self.fault_feedback_age_ms} ms)')


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
        self.bench = False
        self.closed = False
        try:
            reply = self._request(1)
            if not reply[1:].startswith(b'FORGE-MC02/1'):
                raise RuntimeError('The USB device is not running FORGE MC02 firmware')
            self.capabilities = struct.unpack_from("<I", reply, 17)[0]
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

    def read_register(self, bus, motor_id, register):
        """Read one whitelisted register without configuring, enabling or stopping motors.

        Return (reply_CAN_ID, raw_uint32), or None if no matching reply arrived.
        Requires no configured joints, or new firmware and a freshly disabled bench motor.
        """
        if not self.capabilities & 1:
            raise RuntimeError('Firmware lacks read-only CAN discovery; flash the updated ELF first')
        if (type(bus) is not int or bus not in (1, 2, 3) or
                type(motor_id) is not int or not 0 <= motor_id < 0x7ff or
                type(register) is not int or register not in (7, 8, 9, 10, 21, 22, 23, 31)):
            raise ValueError('invalid diagnostic bus, ID or register')
        if self.configured and not (self.bench and self.capabilities & 128):
            raise RuntimeError('Discovery requires a fresh MCU reset with no configured joints')
        self._request(9, struct.pack('<BHB', bus, motor_id, register))
        return self._diagnostic_result(bus, motor_id, register)

    def _diagnostic_result(self, bus, motor_id, register):
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            p = self._request(10)
            status, rx_bus, rx_motor, rx_reg, feedback_id, value = struct.unpack_from('<BBHBHI', p, 1)
            if (rx_bus, rx_motor, rx_reg) != (bus, motor_id, register):
                raise RuntimeError('Diagnostic response does not match the requested motor/register')
            if status == 2:
                return feedback_id, value
            if status == 3:
                return None
            if status == 4:
                raise RuntimeError('MC02 could not queue the CAN read request; reset and check the bus')
            if status != 1:
                raise RuntimeError('Unexpected diagnostic state')
            time.sleep(0.01)
        raise TimeoutError('MC02 diagnostic did not finish')

    def set_bench_timeout(self):
        """Set CAN2/ID1 TIMEOUT to 100 ms in RAM; verify ACK and independent readback."""
        if not self.capabilities & 2 or self.configured:
            raise RuntimeError('Bench firmware and a fresh controller reset are required')
        self._request(11)
        if self._diagnostic_result(2, 1, 9) != (0, 2000):
            raise RuntimeError('Bench timeout write was not acknowledged on feedback ID 0')
        if self.read_register(2, 1, 9) != (0, 2000):
            raise RuntimeError('Bench timeout readback failed; no motion permitted')

    def prepare_bench(self, lower, upper, *, posvel=False):
        """Fixed CAN2/ID1/feedback0 bench configuration; wide limits cannot enable."""
        if not self.capabilities & 2:
            raise RuntimeError('Flash the updated bench-capable firmware first')
        if not all(math.isfinite(v) for v in (lower, upper)) or not -12.5 <= lower < upper <= 12.5:
            raise ValueError('Invalid bench bounds')
        if posvel and not self.capabilities & 128:
            raise RuntimeError('Flash the position-velocity firmware first')
        self._request(14 if posvel else 12, struct.pack('<2f', lower, upper))
        self.configured = True
        self.bench = True

    def set_bench_mode(self, mode):
        """Volatile CAN2/ID1 mode write; firmware requires fresh disabled feedback."""
        if not self.capabilities & 128 or mode not in (1, 2) or not self.bench:
            raise RuntimeError('Position-velocity bench firmware and disabled bench motor required')
        self._request(15, bytes([mode]))
        if self._diagnostic_result(2, 1, 10) != (0, mode):
            raise RuntimeError('Mode write was not acknowledged')
        if self.read_register(2, 1, 10) != (0, mode):
            raise RuntimeError('Mode readback mismatch; no enable permitted')

    def prepare_routed_bench(self, slot, motor, lower, upper, mode):
        """Configure one explicit P-series test slot; requires firmware capability 8."""
        if not self.capabilities & 256:
            raise RuntimeError('Flash the routed arm-test firmware first (capability 0x100)')
        self._slot(slot)
        models = {'8009P': 1, '4340P': 2, '4310P': 3}
        model = models[motor['model']]
        bus, can_id, feedback_id = (motor[k] for k in ('bus', 'can_id', 'feedback_id'))
        if (type(bus) is not int or bus not in (1, 2, 3) or
                type(can_id) is not int or not 1 <= can_id <= 15 or
                type(feedback_id) is not int or not (feedback_id == 0 or 16 <= feedback_id < 0x7ff) or
                (model == 1) != (bus == 1) or mode not in (1, 2)):
            raise ValueError('Invalid routed bench identity/mode; 8009P alone belongs on CAN1')
        speed, torque = motor['speed_trip_rad_s'], motor['torque_trip_nm']
        if (not all(math.isfinite(v) for v in (lower, upper, speed, torque)) or
                not -12.5 <= lower < upper <= 12.5 or not .03 <= speed <= .3 or not .01 <= torque <= 3):
            raise ValueError('Invalid routed bench limits')
        # Mark cleanup responsibility BEFORE transmission: the ACK may be lost.
        self.configured = self.bench = True
        if not hasattr(self, 'bench_routes'):
            self.bench_routes = {}
        self.bench_routes[slot] = (bus, can_id, feedback_id)
        self._request(16, struct.pack('<BBHHBB4f', slot, bus, can_id, feedback_id,
                                     model, mode, lower, upper, speed, torque))

    def enable_bench_group(self, slots):
        """MCU validates every slot before enabling any; feedback confirmation is separate."""
        mask = 0
        if not self.capabilities & 256:
            raise RuntimeError('Routed bench firmware required')
        for slot in slots:
            self._slot(slot)
            mask |= 1 << slot
        if not mask:
            raise ValueError('No slots selected')
        self._request(18, bytes([mask]))

    def write_routed_bench(self, slot, register, value):
        """Volatile TIMEOUT=100ms or CTRL_MODE=1/2; requires fresh disabled feedback."""
        self._slot(slot)
        if not self.capabilities & 256 or slot not in getattr(self, 'bench_routes', {}):
            raise RuntimeError('No routed bench slot configured')
        if not ((register == 9 and value == 2000) or (register == 10 and value in (1, 2))):
            raise ValueError('Only bench timeout and mode writes are permitted')
        bus, can_id, feedback_id = self.bench_routes[slot]
        self._request(17, struct.pack('<BBI', slot, register, value))
        if self._diagnostic_result(bus, can_id, register) != (feedback_id, value):
            raise RuntimeError('Bench register write was not acknowledged by the selected motor')
        if self.read_register(bus, can_id, register) != (feedback_id, value):
            raise RuntimeError('Bench register readback mismatch')


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
        return ArmState(*struct.unpack_from('<7BI6f',p,1), *struct.unpack_from('<4B3I',p,36))

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
