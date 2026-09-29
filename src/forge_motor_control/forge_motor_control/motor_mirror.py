"""Read-only motor feedback mapping; no ROS, MuJoCo or hardware dependencies."""
from dataclasses import dataclass
import math

CAD_FORWARD_SIGNS = (-1, -1, 1, 1)


def validate_sample(names, positions, velocities):
    if (not names or len(names) > 32 or len(set(names)) != len(names)
            or any(not isinstance(n, str) for n in names)
            or len(positions) != len(names) or len(velocities) not in (0, len(names))
            or any(type(v) not in (int, float) or not math.isfinite(v)
                   for v in (*positions, *velocities))):
        raise ValueError("Invalid joint-state names/positions/velocities")


@dataclass
class WheelState:
    angle: float = 0.0
    velocity: float | None = None
    received: float | None = None
    baseline: float | None = None


class MotorMirror:
    def __init__(self, config, timeout=0.5):
        self.ids = tuple(config['wheel_motor_ids'])
        directions = tuple(config['wheel_directions'])
        ratios = tuple(config['external_ratios'])
        if (len(self.ids) != 4 or len(set(self.ids)) != 4
                or any(type(i) is not int or not 1 <= i < 2047 for i in self.ids)
                or len(directions) != 4 or any(d not in (-1, 1) for d in directions)
                or len(ratios) != 4 or any(not math.isfinite(r) or r <= 0 for r in ratios)
                or not math.isfinite(timeout) or timeout <= 0):
            raise ValueError("Invalid mirror wheel mapping")
        omitted = config.get('bench_omit_motor_id', 0)
        if omitted and omitted not in self.ids:
            raise ValueError("Omitted motor is not in wheel mapping")
        self.omitted = omitted
        self.factors = tuple(cad * direction / ratio for cad, direction, ratio in
                             zip(CAD_FORWARD_SIGNS, directions, ratios))
        self.timeout = timeout
        self.wheels = [WheelState() for _ in self.ids]
        self.session = None
        self.last_stamp = -1
        self.last_sent = -1.0

    def accept(self, packet, now):
        if packet.get('version') != 1:
            raise ValueError("Unknown mirror packet version")
        sent, stamp, session = packet['sent'], packet['stamp_ns'], packet['session']
        if (type(sent) not in (float, int) or not math.isfinite(sent)
                or not 0 <= now - sent <= self.timeout
                or type(stamp) is not int or stamp <= 0
                or not isinstance(session, str) or not session or len(session) > 64):
            raise ValueError("Stale or invalid mirror packet")
        names, positions, velocities = packet['names'], packet['positions'], packet['velocities']
        validate_sample(names, positions, velocities)
        if sent <= self.last_sent or (session == self.session and stamp <= self.last_stamp):
            raise ValueError("Out-of-order mirror packet")
        if session != self.session:
            self.wheels = [WheelState() for _ in self.ids]
        self.session, self.last_stamp, self.last_sent = session, stamp, sent
        by_name = {name: index for index, name in enumerate(names)}
        for motor_id, factor, state in zip(self.ids, self.factors, self.wheels):
            if motor_id == self.omitted:
                continue
            index = by_name.get(f'motor_{motor_id}_output')
            if index is None:
                continue
            value = positions[index]
            if state.baseline is None:
                state.baseline = value
            state.angle = (value - state.baseline) * factor
            state.velocity = velocities[index] * factor if velocities else None
            state.received = sent

    def status(self, index, now):
        if self.ids[index] == self.omitted:
            return 'ABSENT'
        received = self.wheels[index].received
        if received is None:
            return 'WAITING'
        return 'LIVE' if now - received <= self.timeout else 'STALE'
