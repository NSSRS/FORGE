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

# Manufacturer rated torque ceilings (manual specification page 7).
# 8009P uses its 24 V speed rating until supply voltage is verified.
# 4340P keeps its 1.5 rad/s commissioning speed cap; torque ceiling is the
# V1.1 manual rated 12 Nm. Profiles may choose a lower reactive trip threshold.
CAP_RATED_4340_TORQUE = 1 << 13
CAP_CONTROL_READBACK = 1 << 14
CAP_MANUAL_TRAVEL = 1 << 15
CAP_POSITION_HOLD = 1 << 16
CAP_GROUP_HOLD = 1 << 17
CAP_SMOOTH_JOG = 1 << 18
CAP_SMOOTH_JOG_10 = 1 << 19
# P-series manual register maps. These are reads only; write whitelist stays
# TIMEOUT/CTRL_MODE. Imax/VBus are documented for 4340P/4310P, not 8009P V1.1.
CONTROL_REGISTERS = {
    0: ('UV_Value', 'float32'), 1: ('KT_Value', 'float32'),
    2: ('OT_Value', 'float32'), 3: ('OC_Value', 'float32'),
    4: ('ACC', 'float32'), 5: ('DEC', 'float32'), 6: ('MAX_SPD', 'float32'),
    14: ('sw_ver', 'uint32'), 24: ('I_BW', 'float32'),
    25: ('KP_ASR', 'float32'), 26: ('KI_ASR', 'float32'),
    27: ('KP_APR', 'float32'), 28: ('KI_APR', 'float32'),
    30: ('GREF', 'float32'), 31: ('Deta', 'float32'),
    32: ('V_BW', 'float32'), 33: ('IQ_c1', 'float32'),
    34: ('VL_c1', 'float32'), 36: ('sub_ver', 'uint32'),
    59: ('Imax', 'float32'), 60: ('VBus', 'float32'),
}
DISCOVERY_REGISTERS = {
    7: ('MST_ID', 'uint32'), 8: ('ESC_ID', 'uint32'), 9: ('TIMEOUT', 'uint32'),
    10: ('CTRL_MODE', 'uint32'), 21: ('PMAX', 'float32'), 22: ('VMAX', 'float32'),
    23: ('TMAX', 'float32'), 31: CONTROL_REGISTERS[31],
}
REGISTERS = DISCOVERY_REGISTERS | CONTROL_REGISTERS


def decode_register(register, raw):
    if REGISTERS[register][1] == 'float32':
        return struct.unpack('<f', struct.pack('<I', raw))[0]
    return raw


BENCH_MODEL_LIMITS = {
    '8009P': (100 * 2 * math.pi / 60, 20.0),
    '4340P': (1.5, 12.0),
    '4310P': (120 * 2 * math.pi / 60, 3.5),
}

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
        self.snapshot_lock = threading.Lock()
        self.buffer = bytearray()
        self.configured = False
        self.bench = False
        self.bench_routes = {}
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

    def _require_base_telemetry(self):
        if not self.capabilities & 4096:
            raise RuntimeError('Firmware lacks cached base telemetry (capability 0x1000); no fallback CAN polling')

    def base_info(self):
        """Cached API/timing metadata. No CAN traffic, lease refresh or reset."""
        self._require_base_telemetry()
        p=self._request(24)
        keys=('api_version','capabilities','control_period_ms','command_lease_ms',
              'feedback_timeout_ms','enable_timeout_ms','can_bitrate','configured_mask','id_session_locked','build_id')
        return dict(zip(keys,struct.unpack_from('<10I',p,1)))

    def bus_health(self, bus):
        """Cumulative uint32 counters since MCU boot; reading never clears them."""
        self._require_base_telemetry()
        if type(bus) is not int or bus not in (1,2,3):
            raise ValueError('invalid CAN bus')
        p=self._request(23,bytes([bus]))
        keys=('at_ms','tx_frames','tx_failed','rx_frames','feedback_frames',
              'max_feedback_gap_ms','max_tick_gap_ms','late_ticks','bus_off_events',
              'status_flags','tx_error_count','rx_error_count')
        return dict(zip(keys,struct.unpack_from('<12I',p,1)))

    def snapshot(self):
        """Frozen six-slot MCU cache, three USB round trips and zero CAN reads.

        A coherent cache is not simultaneous physical sampling. Use each age
        and validity flag. Ages saturate at 65535 ms; missing data is not zero.
        This never refreshes command leases or enables motors.
        """
        self._require_base_telemetry()
        with self.snapshot_lock:
            acquisition_started=time.monotonic()
            result=None
            for page in range(3):
                generation=0 if result is None else result['generation']
                p=self._request(22,struct.pack('<BI',page,generation))
                gen,at,fault,reason,slot=struct.unpack_from('<IIBBB',p,1)
                if result is None:
                    result=dict(generation=gen,at_ms=at,fault=bool(fault),fault_reason=reason,
                                fault_slot=slot,motors=[])
                elif (gen,at)!=(result['generation'],result['at_ms']):
                    raise RuntimeError('Incoherent snapshot pages; discard entire sample')
                for k in range(2):
                    pos,vel,torque,age,flags,status,mos,rotor=struct.unpack_from('<fffHBBBB',p,12+k*18)
                    valid=bool(flags & 4)
                    result['motors'].append(dict(slot=page*2+k,configured=bool(flags & 1),
                        enabled=bool(flags & 2),feedback_valid=valid,stopping=bool(flags & 8),
                        enable_confirmed=bool(flags & 16),age_ms=age if valid else None,
                        position=pos if valid else None,velocity=vel if valid else None,
                        torque=torque if valid else None,status=status,mos_c=mos,rotor_c=rotor))
            received=time.monotonic()
            result['host_received_monotonic_s']=received
            result['acquisition_duration_ms']=(received-acquisition_started)*1000
            return result

    def can_rx_trace(self, bus):
        """Passive last CAN RX frame; does not transmit on CAN or change state."""
        if type(bus) is not int or bus not in (1, 2, 3):
            raise ValueError('invalid CAN bus')
        if not self.capabilities & 2048:
            raise RuntimeError('Flash firmware with passive CAN RX diagnostics first')
        p = self._request(21, bytes([bus]))
        count, at_ms, can_id, length = struct.unpack_from('<IIHB', p, 1)
        return dict(count=count, at_ms=at_ms, can_id=can_id, length=length,
                    data=p[12:12+length].hex())

    def read_register(self, bus, motor_id, register):
        """Read one whitelisted register without configuring, enabling or stopping motors.

        Return (reply_CAN_ID, raw_uint32), or None if no matching reply arrived.
        Legacy registers allow unconfigured discovery. Expanded control reads
        require capability 0x4000 and verified disabled routed motors; the MCU
        checks every retained route's freshness, status and speed.
        """
        if not self.capabilities & 1:
            raise RuntimeError('Firmware lacks read-only CAN discovery; flash the updated ELF first')
        if (type(bus) is not int or bus not in (1, 2, 3) or
                type(motor_id) is not int or not 0 <= motor_id < 0x7ff or
                type(register) is not int or register not in REGISTERS):
            raise ValueError('invalid diagnostic bus, ID or register')
        if register not in DISCOVERY_REGISTERS:
            if not self.capabilities & CAP_CONTROL_READBACK:
                raise RuntimeError('Control/protection readback requires reviewed MC02 firmware with capability 0x4000')
            if not self.configured or not self.bench or (bus, motor_id) not in {
                    route[:2] for route in self.bench_routes.values()}:
                raise RuntimeError('Control/protection reads require a configured disabled routed motor')
        if self.configured and not (self.bench and self.capabilities & 128):
            raise RuntimeError('Discovery requires a fresh MCU reset with no configured joints')
        # DOF1 hardware returned replies two requests behind after ID save.
        # Retry only read-only discovery, before any joint is configured. Never
        # retry writes or bypass configured-motor freshness/stop requirements.
        for _ in range(1 if self.configured else 3):
            self._request(9, struct.pack('<BHB', bus, motor_id, register))
            reply = self._diagnostic_result(bus, motor_id, register)
            if reply is not None:
                return reply
        return None

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
        self._verify_write(2, 1, 9, (0, 2000))

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
        self._verify_write(2, 1, 10, (0, mode))

    def prepare_routed_bench(self, slot, motor, lower, upper, mode, *, manual_travel=False, position_hold=False, group_hold=False):
        """Configure one explicit P-series test slot; requires firmware capability 0x100."""
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
        max_speed, max_torque = BENCH_MODEL_LIMITS[motor['model']]
        if (not all(math.isfinite(v) for v in (lower, upper, speed, torque)) or
                not -12.5 <= lower < upper <= 12.5 or not .03 <= speed <= max_speed or not .01 <= torque <= max_torque):
            raise ValueError('Invalid routed bench limits')
        if model == 2 and torque > 3.0 and not self.capabilities & CAP_RATED_4340_TORQUE:
            raise RuntimeError('MC02 firmware supports only the old 3 Nm 4340P ceiling; '
                               'a reviewed firmware update with capability 0x2000 is required '
                               'before requesting a higher trip threshold')
        if manual_travel:
            if mode != 2:
                raise ValueError("Manual travel requires position-velocity mode")
            if not self.capabilities & CAP_MANUAL_TRAVEL:
                raise RuntimeError("Manual travel requires MC02 capability 0x8000; flash the updated firmware")
        if position_hold:
            if not manual_travel or mode != 2:
                raise ValueError('Position hold requires manual travel and position-velocity mode')
            if not self.capabilities & CAP_POSITION_HOLD:
                raise RuntimeError('Position hold requires MC02 capability 0x10000; flash the updated firmware')
        if group_hold:
            if not position_hold:raise ValueError('Group hold requires position hold')
            if not self.capabilities & CAP_GROUP_HOLD:
                raise RuntimeError('Group hold requires MC02 capability 0x20000; flash the updated firmware')
        # Mark cleanup responsibility BEFORE transmission: the ACK may be lost.
        self.configured = self.bench = True
        self.bench_routes[slot] = (bus, can_id, feedback_id)
        self._request(16, struct.pack('<BBHHBB4f', slot, bus, can_id, feedback_id,
                                     model, mode | (0x80 if manual_travel else 0) | (0x40 if position_hold else 0) | (0x20 if group_hold else 0),
                                     lower, upper, speed, torque))

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
        if not self.capabilities & 256 or slot not in self.bench_routes:
            raise RuntimeError('No routed bench slot configured')
        if not ((register == 9 and value == 2000) or (register == 10 and value in (1, 2))):
            raise ValueError('Only bench timeout and mode writes are permitted')
        bus, can_id, feedback_id = self.bench_routes[slot]
        self._request(17, struct.pack('<BBI', slot, register, value))
        self._verify_write(bus, can_id, register, (feedback_id, value))

    def _verify_write(self, bus, can_id, register, expected):
        if self._diagnostic_result(bus, can_id, register) != expected:
            raise RuntimeError(f'CAN{bus} ID {can_id}: register {register} write was not acknowledged')
        if self.read_register(bus, can_id, register) != expected:
            raise RuntimeError(f'CAN{bus} ID {can_id}: register {register} readback mismatch')

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

    def check_smooth_jog_speed(self, velocity):
        """Reject an incompatible MCU before preloading/enabling at the new rate."""
        if not self.capabilities & CAP_SMOOTH_JOG:
            raise RuntimeError('Smooth jog requires MC02 capability 0x40000; flash the updated firmware')
        if velocity>math.radians(5) and not self.capabilities & CAP_SMOOTH_JOG_10:
            raise RuntimeError('10 deg/s smooth jog requires MC02 capability 0x80000; flash the updated firmware')

    def command(self, slot, position, velocity=0., kp=0., kd=0., torque=0., *, smooth=False):
        values=(position,velocity,kp,kd,torque)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite command')
        if smooth:self.check_smooth_jog_speed(velocity)
        self._request(25 if smooth else 4,self._slot(slot)+struct.pack('<5f',*values))

    def enable(self, slot):
        """Requires fresh disabled feedback and a fresh command near measured position."""
        self._request(5,self._slot(slot))

    def stop(self):
        """Request torque disable on all configured arm motors; this is not a brake."""
        self._request(6)

    def state(self, slot):
        p=self._request(7,self._slot(slot))
        return ArmState(*struct.unpack_from('<7BI6f',p,1), *struct.unpack_from('<4B3I',p,36))

    def id_operation(self, operation, value=0):
        """Explicit disabled-only ID session. Requires a fresh MCU reset to move again."""
        if not self.capabilities & 1024:
            raise RuntimeError('Flash ID-commissioning firmware (capability 0x400) first')
        if type(value) is not int or not ((operation==0 and value==0) or
                (operation==7 and 17<=value<=22) or (operation==8 and 1<=value<=6) or
                (operation==0xaa and value==0)):
            raise ValueError('ID-only operation accepts begin, IDs 1..6/17..22, or save')
        self._request(20, struct.pack('<BI', operation, value))

    def reset_fault(self):
        """Requires acknowledged disabled motors; re-probing is needed afterwards."""
        self._request(8)

    def fault_snapshot(self):
        """Read the first-fault sample, without resetting or sending CAN commands."""
        if not self.capabilities & 512:
            return None
        p = self._request(19)
        values = struct.unpack_from('<4B3I7fB', p, 1)
        keys = ('fault', 'reason', 'slot', 'motor_status', 'at_ms', 'command_age_ms',
                'feedback_age_ms', 'position_rad', 'velocity_rad_s', 'torque_nm',
                'target_rad', 'command_velocity_rad_s', 'speed_limit_rad_s',
                'torque_limit_nm', 'sample_flags')
        return dict(zip(keys, values))

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
