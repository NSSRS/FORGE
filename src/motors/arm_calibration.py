"""Repeatable, model-bounded commissioning policy; no implicit hardware owner.

Angles in files are raw output-shaft degrees. Transport stays in radians.
Only the dedicated calibration application uses the bounded waypoint adapter.
"""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

from .arm_capture import _write
from .arm_group_hold import ArmGroupHoldPort, SimulatedGroupHoldPort

DRIVES = ('b2_arm_d1', 'b3_arm_d2', 'b4_arm_d3', 'b5_arm_d4', 'b6_arm_d5', 'b7_arm_d6')


def number(value, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f'{label} must be finite')
    return float(value)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


class CalibrationMap:
    def __init__(self, document):
        self.document = deepcopy(document)
        if document.get('version') != 1 or not document.get('equipment_revision'):
            raise ValueError('Calibration requires version 1 and an equipment_revision')
        self.joints = document['joints']
        if set(self.joints) != set('123456'):
            raise ValueError('A mapping for every joint A1–A6 is required')
        for key, row in self.joints.items():
            gain = number(row['model_per_raw'], 'model_per_raw')
            if abs(gain) < 1e-6:
                raise ValueError('model_per_raw must be nonzero')
            number(row['model_intercept_deg'], 'model_intercept_deg')
            limits = row['raw_branch_deg']
            if len(limits) != 2 or not number(limits[0], 'branch lower') < number(limits[1], 'branch upper'):
                raise ValueError('Ordered raw_branch_deg required; no automatic angle wrapping')
            if row['model_joint'] != DRIVES[int(key)-1]:
                raise ValueError(f'A{key}: unexpected model joint')
        self.fingerprint = digest(self.document)

    def model(self, joint, raw):
        row = self.joints[str(joint)]
        raw = number(raw, 'raw angle')
        if not row['raw_branch_deg'][0] <= raw <= row['raw_branch_deg'][1]:
            raise ValueError(f'A{joint}: raw angle outside configured encoder branch; do not wrap it automatically')
        return row['model_per_raw'] * raw + row['model_intercept_deg']

    def raw(self, joint, model):
        row = self.joints[str(joint)]
        return (number(model, 'model angle') - row['model_intercept_deg']) / row['model_per_raw']

    def pose(self, raw):
        if set(raw) != set('123456'):
            raise ValueError('Fresh positions for all six joints are required')
        return {key: self.model(int(key), value) for key, value in raw.items()}

    def simulation_pose(self):
        return {key: self.raw(int(key), value) for key, value in self.document['simulation_model_deg'].items()}


def snapshot(rows, *, enabled=None, stationary=False):
    rows = list(rows)
    if len(rows) != 6 or {r.joint for r in rows} != set(range(1, 7)):
        raise ValueError('Complete six-joint feedback required')
    result = {}
    for row in rows:
        if not 0 <= row.age_ms < 80:
            raise ValueError(f'A{row.joint}: stale feedback; refresh before planning or recording')
        if enabled is not None and bool(row.enabled) != enabled:
            raise ValueError('All six joints must be holding' if enabled else 'Disable all torque before hand-position recording')
        if stationary and row.velocity_rad_s is not None:
            if abs(number(row.velocity_rad_s, 'velocity')) > .03:
                raise ValueError(f'A{row.joint}: wait for stationary feedback')
        result[str(row.joint)] = math.degrees(number(row.position_rad, 'position'))
    return result


def validate_start(plan, current, tolerance_deg=.3):
    if set(current) != set(plan['start_raw_deg']):
        raise ValueError('Incomplete planned pose')
    for joint, angle in current.items():
        if abs(angle-plan['start_raw_deg'][joint]) > tolerance_deg:
            raise ValueError(f'A{joint} changed since calculation; calculate a fresh path')


def constraint_limits(mapping, joint, constraints, scene_fingerprint):
    lower, upper = mapping.joints[str(joint)]['raw_branch_deg']
    for item in constraints:
        if item['mapping_fingerprint'] != mapping.fingerprint or item['scene_fingerprint'] != scene_fingerprint:
            raise ValueError('Saved constraints belong to a different model/mapping; use a new calibration run')
        if item['joint'] != joint:
            continue
        angle = number(item['raw_deg'], 'constraint angle')
        margin = number(item['backoff_deg'], 'constraint backoff')
        if margin <= 0:
            raise ValueError('Constraint backoff must be positive')
        if item['side'] == 'lower': lower = max(lower, angle+margin)
        elif item['side'] == 'upper': upper = min(upper, angle-margin)
        else: raise ValueError('Constraint side must be lower or upper raw angle')
    if lower >= upper:
        raise ValueError('Constraints leave no usable interval')
    return lower, upper


class CalibrationRecord:
    """Versioned observation file; fsync runs off the control thread."""
    def __init__(self, path, mapping, *, simulated, constraints=()):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.data = dict(schema='forge.model_guided_calibration', version=1,
                         created_utc=datetime.now(timezone.utc).isoformat(),
                         equipment_revision=mapping.document['equipment_revision'],
                         mapping=deepcopy(mapping.document), mapping_fingerprint=mapping.fingerprint,
                         simulated=simulated, production_qualified=False,
                         calibration_anchors_changed=False, constraints=deepcopy(list(constraints)),
                         plans=[], events=[])
        with self.path.open('x') as stream:
            json.dump(self.data, stream, indent=2, allow_nan=False)
        self.writer = ThreadPoolExecutor(max_workers=1, thread_name_prefix='calibration-save')
        self.pending = []

    def save(self):
        self.check()
        self.pending.append(self.writer.submit(_write, self.path, deepcopy(self.data)))

    def check(self):
        remaining = []
        for future in self.pending:
            if future.done(): future.result()
            else: remaining.append(future)
        self.pending = remaining

    def event(self, action, **values):
        self.data['events'].append(dict(action=action, at_monotonic_s=time.monotonic(), **values))
        self.save()

    def add_constraint(self, *, joint, side, reason, raw, backoff_deg, scene_fingerprint):
        if side not in ('lower', 'upper') or not reason.strip():
            raise ValueError('Choose lower/upper raw limit and describe the obstacle')
        if not 0 < number(backoff_deg, 'backoff') <= 20:
            raise ValueError('Backoff must be >0 and <=20 degrees')
        self.data['constraints'].append(dict(joint=joint, side=side, reason=reason.strip(),
            raw_deg=raw[str(joint)], group_raw_deg=dict(raw), backoff_deg=backoff_deg,
            mapping_fingerprint=self.data['mapping_fingerprint'], scene_fingerprint=scene_fingerprint,
            scope='operator obstacle in recorded arm configuration; restricts future paths but does not certify all configurations'))
        self.save()

    def close(self):
        self.writer.shutdown(wait=True)
        self.check()


class CalibrationGroupHoldPort(ArmGroupHoldPort):
    def start_waypoint(self, target_rad, speed_rad_s):
        """At most 1 degree per planned segment, keeping the existing owner/leases."""
        current = snapshot(self.cached_positions(), enabled=True)[str(self.selected)]
        if abs(math.degrees(target_rad)-current) > 1.001:
            raise ValueError('Calibration waypoints must be within one raw degree')
        if not 0 < speed_rad_s <= math.radians(5):
            raise ValueError('Calibration speed must be >0 and <=5 degrees/s')
        self._start_move(target_rad, speed_rad_s, 0)


class SimulatedCalibrationPort(SimulatedGroupHoldPort):
    def start_waypoint(self, target_rad, speed_rad_s):
        current = self.values[self.selected-1]
        if abs(target_rad-current) > math.radians(1.001):
            raise ValueError('Calibration waypoints must be within one raw degree')
        if not 0 < speed_rad_s <= math.radians(5):
            raise ValueError('Calibration speed must be >0 and <=5 degrees/s')
        if not self.armed or not self.hold_targets or self.moving:
            raise ValueError('Hold all and arm before moving')
        self.target=target_rad;self.speed=speed_rad_s;self.jog_direction=0
        self.moving=True;self.holding=True;self.last=time.monotonic()
        self.hold_targets[self.selected-1]=target_rad
