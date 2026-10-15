"""Pure parsing, calibration and freshness rules for the ESP32 CRSF CSV stream."""
from dataclasses import dataclass
import math


def parse_channels(line):
    """Return (source age in seconds, 16 raw ticks); diagnostics are not samples."""
    parts = line.strip().split(',')
    if len(parts) != 17 or any(not p.isascii() or not p.isdecimal() for p in parts):
        raise ValueError('Expected age_ms and sixteen unsigned raw channels')
    values = tuple(map(int, parts))
    if values[0] > 0xFFFFFFFF or any(v > 2047 for v in values[1:]):
        raise ValueError('Channel/age outside wire range')
    return values[0] / 1000.0, values[1:]


@dataclass(frozen=True)
class Axis:
    channel: int  # User-facing CH1..CH16, never inferred from stick position.
    minimum: int
    center: int
    maximum: int
    sign: int = 1
    deadzone: float = 0.05

    def __post_init__(self):
        if not 1 <= self.channel <= 16:
            raise ValueError('Set measured channel numbers (1..16) before radio driving')
        if not 0 <= self.minimum < self.center < self.maximum <= 2047:
            raise ValueError('Set measured raw minimum < center < maximum for each axis')
        if self.sign not in (-1, 1) or not math.isfinite(self.deadzone) or not 0 <= self.deadzone < 1:
            raise ValueError('Axis sign must be +/-1 and deadzone must be in [0, 1)')

    def value(self, channels):
        raw = channels[self.channel - 1]
        span = self.maximum - self.center if raw >= self.center else self.center - self.minimum
        value = max(-1.0, min(1.0, (raw - self.center) / span))
        magnitude = max(0.0, (abs(value) - self.deadzone) / (1 - self.deadzone))
        return self.sign * math.copysign(magnitude, value)


class RadioCommand:
    """One stick drives the chassis. Loss requires neutral before motion resumes.

    Fresh channel packets do NOT establish RF health: receiver hold-last failsafe
    cannot be detected by this CSV protocol. Verify that behavior separately.
    """
    def __init__(self, forward, turn, speed, yaw_rate, timeout=0.15):
        if forward.channel == turn.channel:
            raise ValueError('Forward and turn must be different axes of the selected stick')
        if any(not math.isfinite(v) or v <= 0 for v in (speed, yaw_rate, timeout)) or timeout > 0.20:
            raise ValueError('Positive finite speed/yaw/timeout required; timeout <= 0.20 s')
        self.forward, self.turn = forward, turn
        self.speed, self.yaw_rate, self.timeout = speed, yaw_rate, timeout
        self.reset()

    def reset(self):
        self.target = (0.0, 0.0)
        self.deadline = 0.0
        self.armed = False
        self.neutral_since = None

    def receive(self, line, now):
        self.output(now)  # Expiry between samples also resets neutral acquisition.
        try:
            age, channels = parse_channels(line)
            if age >= self.timeout:
                raise ValueError('Stale channel sample')
        except ValueError:
            self.reset()
            return
        forward, turn = self.forward.value(channels), self.turn.value(channels)
        self.deadline = now + self.timeout - age
        if not self.armed:
            if (forward, turn) == (0.0, 0.0):
                if self.neutral_since is None:
                    self.neutral_since = now
                self.armed = now - self.neutral_since >= 0.30
            else:
                self.neutral_since = None
        self.target = (forward * self.speed, turn * self.yaw_rate) if self.armed else (0.0, 0.0)

    def output(self, now):
        if now >= self.deadline:
            self.reset()
        return self.target
