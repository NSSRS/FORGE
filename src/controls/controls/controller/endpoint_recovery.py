"""Reviewed A3 upper-endpoint retreat; never a wider operating envelope."""

from dataclasses import dataclass
from copy import deepcopy
import math

import numpy as np


A3_UPPER_DEG = 151.39
A3_BACKOFF_DEG = 149.39
ALLOWANCE_DEG = 0.25
SPEED_DEG_S = 1.0
ACCEL_DEG_S2 = 5.0
STOP_PAIR = ("collision_469", "collision_519")
# The reviewed model reaches about 0.117 mm overlap at upper + 0.25 deg.
# Reject deeper overlap, including a changed/misaligned model.
MAX_OVERLAP_MM = 0.15


@dataclass(frozen=True)
class A3Retreat:
    start: tuple
    mapping_fingerprint: str

    def __post_init__(self):
        q = np.asarray(self.start, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("A3 retreat requires six finite start readings")
        if not A3_BACKOFF_DEG < q[2] <= A3_UPPER_DEG + ALLOWANCE_DEG:
            raise ValueError("A3 start exceeds the reviewed 0.25 degree endpoint allowance")
        if not isinstance(self.mapping_fingerprint, str) or not self.mapping_fingerprint:
            raise ValueError("A3 retreat requires a mapping fingerprint")
        object.__setattr__(self, "start", tuple(float(x) for x in q))

    @property
    def goal(self):
        q = np.array(self.start)
        q[2] = A3_BACKOFF_DEG
        return q

    def document(self):
        return dict(version=1, start_raw_deg=list(self.start),
                    mapping_fingerprint=self.mapping_fingerprint)

    @classmethod
    def from_document(cls, value):
        if value.get("version") != 1:
            raise ValueError("Unsupported A3 endpoint recovery policy")
        return cls(tuple(value["start_raw_deg"]), value["mapping_fingerprint"])

    def check_mapping(self, mapping):
        row = mapping.joints["3"]
        if (mapping.fingerprint != self.mapping_fingerprint
                or not np.allclose(mapping.raw_limits(3), [72.77, A3_UPPER_DEG], rtol=0, atol=1e-9)
                or row["model_per_raw"] != -1.0
                or abs(row["model_intercept_deg"] - 126.72764404296873) > 1e-9
                or list(STOP_PAIR) not in mapping.document.get("adjacent_stop_pairs", [])):
            raise ValueError("A3 retreat belongs to a different or unreviewed calibration")

    def check_raw(self, raw, *, command=False):
        q = np.asarray(raw, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("A3 retreat requires six finite positions")
        other = [0, 1, 3, 4, 5]
        tolerance = 1e-8 if command else ALLOWANCE_DEG
        if np.max(abs(q[other] - np.asarray(self.start)[other])) > tolerance:
            raise ValueError("Other joints must hold their checked positions during A3 retreat")
        low = A3_BACKOFF_DEG - (0 if command else ALLOWANCE_DEG)
        high = min(A3_UPPER_DEG + ALLOWANCE_DEG,
                   self.start[2] + (0 if command else ALLOWANCE_DEG))
        if not low - 1e-9 <= q[2] <= high + 1e-9:
            raise ValueError("A3 exceeds the fixed inward-recovery envelope")
        return q

    def check_path(self, path):
        if path.interpolation != "pchip":
            raise ValueError("A3 retreat requires a monotone path")
        if (np.max(abs(path.positions[0] - self.start)) > 1e-8
                or np.max(abs(path.positions[-1] - self.goal)) > 1e-8
                or np.any(np.diff(path.positions[:, 2]) > 1e-10)):
            raise ValueError("A3 retreat must move inward to the fixed backoff")
        for q in path.positions:
            self.check_raw(q, command=True)
        speed, acceleration = path.peaks()
        if np.max(speed) > SPEED_DEG_S + 1e-8 or np.max(acceleration) >= ACCEL_DEG_S2:
            raise ValueError("A3 retreat exceeds 1 deg/s or the 5 deg/s² acceleration budget")


def retreat_path(policy, *, speed=SPEED_DEG_S, acceleration=ACCEL_DEG_S2):
    from .weld_plan import Settings, move_path

    path = move_path(policy.start, policy.goal, Settings(
        max_speed_deg_s=min(speed, SPEED_DEG_S),
        max_accel_deg_s2=0.9 * min(acceleration, ACCEL_DEG_S2)))
    policy.check_path(path)
    return path


def add_retreat_timing(plan, endpoint):
    """Include the common startup cost once in every staging comparison."""
    from .weld_plan import JointPath

    policy = A3Retreat.from_document(endpoint['policy'])
    duration = JointPath.from_document(endpoint['path']).duration
    extra = duration + .2
    # Keep timing comparisons JSON-native; goal is a NumPy array.
    distance = float(policy.start[2] - policy.goal[2])
    speed, accel = endpoint['speed_deg_s'], endpoint['acceleration_deg_s2']
    triangular = 2 * math.sqrt(distance / accel)
    lower = triangular if distance <= speed**2 / accel else distance / speed + speed / accel

    def include(original):
        # Search candidate timing can alias the selected plan's timing.
        timing = deepcopy(original)
        timing['endpoint_recovery_s'] = extra
        timing['nominal_total_s'] += extra
        timing['minimum_settling_s'] += .2
        timing['lower_bound_total_s'] += lower + .2
        timing['acceleration_only_lower_bound_s'] += triangular + .2
        timing['stages'].insert(0, dict(stage='endpoint_recovery', nominal_motion_s=duration,
                                      lower_bound_s=lower))
        timing['nominal_meets_target'] = timing['nominal_total_s'] <= timing['target_s']
        timing['ruled_out_by_limits'] = timing['lower_bound_total_s'] > timing['target_s']
        return timing

    plan['preparation_timing'] = include(plan['preparation_timing'])
    if 'preparation_search' in plan:
        search = plan['preparation_search']
        search['original_nominal_total_s'] += extra
        for candidate in search['candidates']:
            if candidate.get('checked'):
                candidate['timing'] = include(candidate['timing'])


def prepare_retreat(geometry, raw, settings):
    """A normal start stays strict. Only this reviewed A3 stop may be escaped."""
    try:
        geometry.check(raw, clearance_mm=2.0)
        return None
    except ValueError as original:
        try:
            policy = A3Retreat(tuple(raw), geometry.mapping.fingerprint)
            policy.check_mapping(geometry.mapping)
            path = retreat_path(policy, speed=settings.max_speed_deg_s,
                                acceleration=settings.max_accel_deg_s2)
            previous = -float("inf")
            samples = []
            for q3 in np.linspace(raw[2], A3_BACKOFF_DEG,
                                  max(2, int(np.ceil((raw[2] - A3_BACKOFF_DEG) / .02)) + 1)):
                q = np.array(raw, dtype=float); q[2] = q3
                clearance = geometry.check_endpoint(q, policy.document())
                if clearance < previous - 1e-6:
                    raise ValueError("A3 stop contact deepens on the inward path")
                previous = clearance
                samples.append(dict(a3_raw_deg=float(q3), signed_clearance_mm=clearance))
            geometry.check(policy.goal, clearance_mm=2.0)
            return dict(policy=policy.document(), path=path.document(),
                        speed_deg_s=min(SPEED_DEG_S, settings.max_speed_deg_s),
                        acceleration_deg_s2=min(ACCEL_DEG_S2, settings.max_accel_deg_s2),
                        separation_samples=samples, hardware_qualified=False)
        except ValueError as exc:
            raise ValueError(f"{original} A3 inward recovery unavailable: {exc}") from exc
