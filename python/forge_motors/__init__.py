"""Per-motor ENCOS commands; units are output-shaft degrees, RPM and phase A.

No implicit heartbeat: callers must renew EVERY active motor within 250 ms.
One MotorBus owns one bridge with 1-3 explicitly identified motors on CAN1.
"""
from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass
import math
import os
from pathlib import Path
import threading
import time

__all__ = ["MotorBus", "MotorConfig", "MotorState", "MotorError"]


class MotorError(RuntimeError):
    pass


@dataclass(frozen=True)
class MotorConfig:
    motor_id: int
    min_position_deg: float
    max_position_deg: float
    max_velocity_rpm: float
    max_current_a: float
    max_acceleration_rpm_s: float

    def __post_init__(self):
        if type(self.motor_id) is not int or not 1 <= self.motor_id < 0x7FF:
            raise ValueError("motor_id must be an integer from 1 through 2046")
        values = (self.min_position_deg, self.max_position_deg, self.max_velocity_rpm,
                  self.max_current_a, self.max_acceleration_rpm_s)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("limits must be finite")
        if not self.min_position_deg < self.max_position_deg:
            raise ValueError("minimum position must be below maximum")
        if not 0 < self.max_velocity_rpm <= 3276.7 or not 0.1 <= self.max_current_a <= 409.5:
            raise ValueError("speed/current limits exceed the shared position/velocity codec range")
        if not 0 < self.max_acceleration_rpm_s <= 1e6:
            raise ValueError("acceleration must be positive and at most 1e6 RPM/s")
        # Prevent finite Python doubles overflowing the C float ABI.
        if any(not math.isfinite(ct.c_float(v).value) for v in values):
            raise ValueError("limits must fit float32")


@dataclass(frozen=True)
class MotorState:
    position_deg: float | None
    velocity_rpm: float | None
    current_a: float | None
    temperature_c: float | None
    observed_monotonic_s: float
    error: int


class _Config(ct.Structure):
    _fields_ = [("motor_id", ct.c_uint16)] + [(name, ct.c_float) for name in (
        "min_position_deg", "max_position_deg", "max_velocity_rpm", "max_current_a",
        "max_acceleration_rpm_s")]


class _State(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in (
        "position_deg", "velocity_rpm", "current_a", "temperature_c")]
    _fields_ += [("observed_monotonic_s", ct.c_double), ("flags", ct.c_uint32), ("error", ct.c_uint32)]


class _Native:
    def __init__(self, interface, configs, library):
        path = library or os.environ.get("FORGE_MOTORS_LIBRARY") or (
            Path(__file__).resolve().parents[2] / "build/encos_query/libencos_driver.so")
        try:
            self.lib = ct.CDLL(str(path))
        except OSError as e:
            raise MotorError("Build with ./scripts/build.sh on Ubuntu, then set FORGE_MOTORS_LIBRARY "
                             "to the absolute libencos_driver.so path if needed") from e
        signatures = {
            "open": ([ct.c_char_p, ct.POINTER(_Config), ct.c_size_t], ct.c_void_p),
            "command": ([ct.c_void_p, ct.c_uint16, ct.c_int, ct.c_float], ct.c_int),
            "read": ([ct.c_void_p, ct.c_uint16, ct.POINTER(_State)], ct.c_int),
            "status": ([ct.c_void_p], ct.c_int),
            "close": ([ct.c_void_p], None),
        }
        for name, (args, result) in signatures.items():
            f = getattr(self.lib, "encos_driver_" + name)
            f.argtypes, f.restype = args, result
        array = (_Config * len(configs))(*(_Config(**vars(c)) for c in configs))
        self.handle = self.lib.encos_driver_open(interface.encode(), array, len(array))
        if not self.handle:
            raise MotorError("Cannot open bridge: check interface/permissions, exclusive ownership, "
                             "one-slave topology and 86-byte OUT / 92-byte IN PDO layout")

    def status(self):
        return self.lib.encos_driver_status(self.handle)

    def command(self, motor_id, mode, target):
        if self.lib.encos_driver_command(self.handle, motor_id, mode, target):
            raise MotorError("Command rejected: wait for initial position, check limits and fault status")

    def state(self, motor_id):
        state = _State()
        if self.lib.encos_driver_read(self.handle, motor_id, ct.byref(state)):
            raise MotorError("Unknown motor")
        return MotorState(state.position_deg if state.flags & 1 else None,
                          state.velocity_rpm if state.flags & 2 else None,
                          state.current_a if state.flags & 4 else None,
                          state.temperature_c if state.flags & 4 else None,
                          state.observed_monotonic_s, state.error)

    def close(self):
        self.lib.encos_driver_close(self.handle)
        self.handle = None


class _Sim:
    """Ideal kinematic model, not a model of torque, traction or ENCOS firmware."""
    def __init__(self, configs, clock=time.monotonic):
        self.clock, self.last, self.fault = clock, clock(), 0
        self.configs = {c.motor_id: c for c in configs}
        self.positions = {c.motor_id: min(max(0., c.min_position_deg), c.max_position_deg) for c in configs}
        self.velocities = dict.fromkeys(self.configs, 0.)
        self.commands = {}

    def _advance(self):
        now = self.clock()
        # Integrate in small steps, stopping at a missed command deadline.
        while self.last < now and not self.fault:
            dt = min(0.01, now - self.last)
            self.last += dt
            if any(self.last - stamp > 0.25 for _, _, stamp in self.commands.values()):
                self.fault = 1
                break
            for motor_id, (mode, target, _) in self.commands.items():
                c, p = self.configs[motor_id], self.positions[motor_id]
                if mode == 2:
                    step = c.max_acceleration_rpm_s * dt
                    velocity = self.velocities[motor_id]
                    velocity += max(-step, min(step, target - velocity))
                else:
                    velocity = max(-c.max_velocity_rpm, min(c.max_velocity_rpm, (target-p)/(6*dt)))
                p += velocity * 6 * dt
                self.positions[motor_id], self.velocities[motor_id] = p, velocity
                if not c.min_position_deg <= p <= c.max_position_deg:
                    self.fault = 3
                    break
        self.last = now
        if self.fault:
            self.velocities = dict.fromkeys(self.configs, 0.)

    def status(self):
        self._advance()
        return self.fault

    def command(self, motor_id, mode, target):
        self._advance()
        if self.fault:
            raise MotorError("Simulation fault is latched")
        self.commands[motor_id] = (mode, target, self.clock())

    def state(self, motor_id):
        self._advance()
        return MotorState(self.positions[motor_id], self.velocities[motor_id], 0., 25., self.last, 0)

    def close(self):
        self.fault = 4
        self.velocities = dict.fromkeys(self.configs, 0.)


class MotorBus:
    """Explicitly owned bridge session. Use as a context manager.

    Hardware requires execute=True. Opening is query-only; each first command
    activates its motor. Commands for one motor never refresh another's lease.
    Public methods are serialized so close cannot race a ctypes call.
    """
    def __init__(self, configs, *, interface=None, simulate=False, execute=False, library=None):
        configs = tuple(configs)
        if not 1 <= len(configs) <= 3 or not all(isinstance(c, MotorConfig) for c in configs):
            raise ValueError("Supply 1-3 MotorConfig objects for CAN1")
        self.configs = {c.motor_id: c for c in configs}
        if len(self.configs) != len(configs):
            raise ValueError("Motor IDs must be unique")
        if not simulate and (not execute or not isinstance(interface, str) or not interface):
            raise ValueError("Hardware requires interface='...' and execute=True")
        self._lock = threading.RLock()
        self._closed = False
        self._active = set()
        self._backend = _Sim(configs) if simulate else _Native(interface, configs, library)

    def _check(self):
        if self._closed:
            raise MotorError("MotorBus is closed")
        fault = self._backend.status()
        if fault:
            reason = {1: "command deadline missed", 2: "EtherCAT/timing failure",
                      3: "feedback/limit failure", 4: "closing"}.get(fault, "unknown failure")
            raise MotorError(f"Latched motor fault: {reason}; close and investigate before reopening")

    def _command(self, motor_id, mode, target):
        with self._lock:
            self._check()
            if type(motor_id) is not int or motor_id not in self.configs:
                raise ValueError("Unknown motor ID")
            c = self.configs[motor_id]
            if not math.isfinite(target):
                raise ValueError("Target must be finite")
            if mode == 1 and not c.min_position_deg <= target <= c.max_position_deg:
                raise ValueError("Position outside configured limits")
            if mode == 2 and abs(target) > c.max_velocity_rpm:
                raise ValueError("Velocity outside configured limits")
            self._backend.command(motor_id, mode, target)
            self._active.add(motor_id)

    def velocity(self, motor_id: int, rpm: float):
        """Mode 2, signed output RPM; native loop applies acceleration limiting."""
        self._command(motor_id, 2, rpm)

    def position(self, motor_id: int, degrees: float):
        """Mode 1, absolute output angle; speed/current use MotorConfig ceilings.

        Acceleration here is firmware-controlled, not the Python velocity ramp.
        """
        self._command(motor_id, 1, degrees)

    def state(self, motor_id: int) -> MotorState:
        """Read even after a fault. Timestamp is observation time, not CAN freshness."""
        with self._lock:
            if self._closed:
                raise MotorError("MotorBus is closed")
            if type(motor_id) is not int or motor_id not in self.configs:
                raise ValueError("Unknown motor ID")
            return self._backend.state(motor_id)

    def check(self):
        """Raise on a latched fault, including during otherwise idle application work."""
        with self._lock:
            self._check()

    def wait_ready(self, timeout=2.0):
        """Wait for initial positions; does not activate or refresh motors."""
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check()
            if all(self.state(i).position_deg is not None for i in self.configs):
                return
            time.sleep(0.01)
        raise MotorError("Initial position feedback timed out")

    def stop(self):
        """Request ramped zero speed on all active motors. Keep refreshing until stopped.

        This does not disarm, engage a brake, or hold chassis position.
        """
        with self._lock:
            self._check()
            for motor_id in self._active:
                self.velocity(motor_id, 0.)

    def close(self):
        """Best-effort immediate zero speed for 0.5 s then clear outputs/release bridge."""
        with self._lock:
            if not self._closed:
                self._backend.close()
                self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
