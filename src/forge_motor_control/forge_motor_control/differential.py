"""ROS-independent four-wheel mixing and command lease handling.

Wheel order follows forge_sim 7aa258d: left 1/2, right 3/4; 5/6 passive.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DifferentialDrive:
    motor_ids: tuple = (1, 2, 3, 4)  # Driver/PDO order, NOT wheel order.
    wheel_motor_ids: tuple = (1, 2, 3, 4)
    wheel_sides: tuple = ("left", "left", "right", "right")
    wheel_directions: tuple = (-1.0, -1.0, 1.0, 1.0)
    external_ratios: tuple = (1.0, 1.0, 1.0, 1.0)
    wheel_rpm_scales: tuple = (1.0, 1.0, 1.0, 1.0)
    wheel_radius_m: float = 0.040
    track_width_m: float = 0.737
    max_velocity_rpm: float = 30.0

    def __post_init__(self):
        arrays = (self.motor_ids, self.wheel_motor_ids, self.wheel_sides,
                  self.wheel_directions, self.external_ratios, self.wheel_rpm_scales)
        if any(len(a) != 4 for a in arrays):
            raise ValueError("Differential chassis requires exactly four driven wheels/motors")
        if (any(type(i) is not int or not 1 <= i < 0x7FF for i in self.motor_ids)
                or len(set(self.motor_ids)) != 4
                or any(type(i) is not int for i in self.wheel_motor_ids)
                or set(self.motor_ids) != set(self.wheel_motor_ids)):
            raise ValueError("wheel_motor_ids must be a permutation of four unique motor_ids")
        if sorted(self.wheel_sides) != ["left", "left", "right", "right"]:
            raise ValueError("Two left and two right wheels are required")
        if any(d not in (-1.0, 1.0) for d in self.wheel_directions):
            raise ValueError("wheel_directions must be +/-1")
        positives = (self.wheel_radius_m, self.track_width_m, self.max_velocity_rpm,
                     *self.external_ratios, *self.wheel_rpm_scales)
        if any(not math.isfinite(v) or v <= 0 for v in positives):
            raise ValueError("Geometry, ratios, scales and speed limit must be finite and positive")
        if self.max_velocity_rpm > 75:
            raise ValueError("Speed limit must not exceed the simulation's rated 75 output RPM")

    def mix(self, forward_m_s, yaw_rad_s):
        """Positive yaw turns left in the local surface-tangent base_link frame."""
        if not all(math.isfinite(v) for v in (forward_m_s, yaw_rad_s)):
            raise ValueError("Chassis command must be finite")
        wheel_targets = [
            (forward_m_s + (-1 if side == "left" else 1) * yaw_rad_s * self.track_width_m / 2)
            * 60 / (math.tau * self.wheel_radius_m) * direction * ratio * scale
            for side, direction, ratio, scale in zip(
                self.wheel_sides, self.wheel_directions, self.external_ratios, self.wheel_rpm_scales)
        ]
        if not all(math.isfinite(v) for v in wheel_targets):
            raise ValueError("Chassis conversion overflow")
        # Scale together to retain curvature instead of clipping wheels independently.
        factor = max(1.0, max(abs(v) for v in wheel_targets) / self.max_velocity_rpm)
        by_id = dict(zip(self.wheel_motor_ids, (v / factor for v in wheel_targets)))
        return [by_id[i] for i in self.motor_ids]


class CommandLease:
    """Receipt-time watchdog plus source timestamp/frame validation; no ROS imports."""
    def __init__(self, drive, timeout_s=0.15, frame_id="base_link"):
        if not math.isfinite(timeout_s) or not 0.02 <= timeout_s <= 0.20:
            raise ValueError("command_timeout_s must be in [0.02, 0.20]")
        if not frame_id:
            raise ValueError("command_frame must not be empty")
        self.drive, self.timeout_s, self.frame_id = drive, timeout_s, frame_id
        self.targets = None  # Stay query-only until the first valid command.
        self.deadline = 0.0
        self.last_stamp_ns = None
        self.last_clock_ns = None

    def stop(self):
        if self.targets is not None:
            self.targets = [0.0] * 4
        self.deadline = 0.0

    def check_clock(self, now_ns):
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.stop()
            self.last_stamp_ns = None
        self.last_clock_ns = now_ns

    def receive(self, values, stamp_ns, frame_id, now_ns, monotonic_s):
        self.check_clock(now_ns)
        try:
            age = (now_ns - stamp_ns) / 1e9
            if (stamp_ns <= 0 or age < 0 or age >= self.timeout_s
                    or (self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns)):
                raise ValueError("Command timestamp is stale, future, zero, or replayed")
            if frame_id != self.frame_id:
                raise ValueError(f"Command frame must be {self.frame_id}")
            if len(values) != 6 or not all(math.isfinite(v) for v in values):
                raise ValueError("Twist must have six finite components")
            if any(values[i] != 0 for i in (1, 2, 3, 4)):
                raise ValueError("Only linear.x and angular.z are supported")
            targets = self.drive.mix(values[0], values[5])
        except ValueError:
            self.stop()
            raise
        self.targets = targets
        self.last_stamp_ns = stamp_ns
        self.deadline = monotonic_s + self.timeout_s - age

    def output(self, monotonic_s, now_ns):
        self.check_clock(now_ns)
        if monotonic_s >= self.deadline:
            self.stop()
        return None if self.targets is None else list(self.targets)


class KeyboardCommand:
    """A second matching key arms motion; repeat renews a short dead-man lease."""
    def __init__(self, speed=0.02, yaw_rate=0.07, timeout=0.15):
        if any(not math.isfinite(v) or v <= 0 for v in (speed, yaw_rate, timeout)):
            raise ValueError("Keyboard speeds and timeout must be finite and positive")
        if timeout > 0.20:
            raise ValueError("Keyboard timeout must not exceed 0.20 s")
        self.commands = {"w": (speed, 0.0), "s": (-speed, 0.0),
                         "a": (0.0, yaw_rate), "d": (0.0, -yaw_rate)}
        self.timeout = timeout
        self.target = (0.0, 0.0)
        self.pending = None
        self.pending_until = self.deadline = 0.0

    def update(self, key, now):
        if now >= self.deadline:
            self.target = (0.0, 0.0)
        key = key.lower() if key else None
        if key in (" ", "x", "q", "\x1b"):
            self.target, self.pending = (0.0, 0.0), None
        elif key in self.commands:
            if self.target == (0.0, 0.0) and (self.pending != key or now > self.pending_until):
                self.pending, self.pending_until = key, now + 1.0
            else:
                self.target = self.commands[key]
                self.deadline = now + self.timeout
                self.pending = None
        return self.target
