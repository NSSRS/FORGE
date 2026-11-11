"""Single-joint fixture teaching in raw shaft coordinates. No transport ownership."""
import json
import math
import os
from pathlib import Path
import tempfile

from .arm_bench import number as finite


REFERENCES = {
    1: 'align base to model initial pose',
    2: 'hand-place at CW stop',
    3: 'hand-place at CCW stop',
    4: 'hand-align wrist to model initial pose',
    5: 'hand-place at CW stop',
    6: 'hand-align tool to model initial pose',
}
STOP_JOINTS = {2, 3, 5}
MIN_CLEARANCE = math.radians(0.5)


class PoseTeach:
    """Persist relative travel. Absolute references and inward checks expire on restart."""

    def __init__(self, path, pose_path, plan, simulated, mappings):
        self.path = Path(path)
        angles = json.loads(Path(pose_path).read_text())['joint_angles_rad']
        self.goals = {}
        for j in REFERENCES:
            name = f'b{j+1}_arm_d{j}'
            # D4 and D6 use the MJCF hinge reference of zero unless overridden.
            angle = angles.get(name, 0.) if j in (4, 6) else angles[name]
            self.goals[j] = finite(angle, name)
        inventory = [[m[k] for k in ('unit', 'model', 'bus', 'can_id', 'feedback_id')]
                     for m in plan['motors']]
        header = dict(version=1, simulated=simulated, inventory=inventory,
                      model_goal_rad={str(k): v for k, v in self.goals.items()})
        self.records = json.loads(self.path.read_text()) if self.path.exists() else dict(header, joints={})
        if any(self.records.get(k) != v for k, v in header.items()):
            raise ValueError('Teach file belongs to a different model pose, inventory or simulation mode')
        self.mappings = {}
        for number, angle, ratio in mappings:
            j = int(number)
            scale = finite(float(ratio), 'signed raw-shaft radians per model radian')
            if j not in STOP_JOINTS or j in self.mappings or scale == 0:
                raise ValueError('Each stop map needs a distinct A2/A3/A5 number and nonzero signed ratio')
            self.mappings[j] = (math.radians(finite(float(angle), 'measured stop angle')), scale)
        self.reset_session()

    def reset_session(self):
        self.captured = set()

    def _current(self, joint):
        if joint not in self.captured:
            raise ValueError('Capture the reference with C in this session')
        return self.records['joints'][str(joint)]

    def _save(self, joint, record):
        updated = dict(self.records, joints={**self.records['joints'], str(joint): record})
        self.path.parent.mkdir(parents=True, exist_ok=True)
        stream = tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent, delete=False)
        try:
            with stream:
                json.dump(updated, stream, indent=2, allow_nan=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(stream.name, self.path)
        finally:
            Path(stream.name).unlink(missing_ok=True)
        self.records = updated

    def capture(self, joint, action, row):
        raw = finite(row.position_rad, 'raw encoder position')
        if row.joint != joint or row.enabled or not 0 <= row.age_ms < 80:
            raise ValueError('Capture requires fresh disabled feedback for the selected joint')
        record = dict(self.records['joints'].get(str(joint), {}))
        if action == 'c':
            if joint in self.mappings:
                stop, scale = self.mappings[joint]
                if (record.get('model_reference_rad'), record.get('raw_per_model')) != (stop, scale):
                    record = dict(delta_rad=(self.goals[joint]-stop)*scale, source='measured map',
                                  model_reference_rad=stop, raw_per_model=scale)
            elif joint not in STOP_JOINTS and 'delta_rad' not in record:
                record = dict(delta_rad=0., source='manual model alignment')
            record['reference_raw_rad'] = raw
            if 'delta_rad' in record:
                record['goal_raw_rad'] = raw + finite(record['delta_rad'], 'goal offset')
            record.pop('inward_sample_raw_rad', None)
            message = 'Reference recorded.'
        else:
            reference = self._current(joint)['reference_raw_rad']
            delta = raw - reference
            if abs(delta) >= (row.upper_rad - row.lower_rad) / 2:
                raise ValueError('Ambiguous encoder wrap. Check the raw branch and recapture.')
            if action == 'b':
                if joint not in STOP_JOINTS or abs(delta) < MIN_CLEARANCE:
                    raise ValueError('For a stop joint, hand back away at least 0.5 raw degrees before B')
                record['inward_sample_raw_rad'] = raw
                message = 'Inward direction recorded.'
            elif action == 't':
                if joint in STOP_JOINTS and delta * self._inward_sign(record) <= 0:
                    raise ValueError('Teach a goal inward from the recorded stop')
                record.update(delta_rad=delta, source='hand-taught model pose', goal_raw_rad=raw,
                              taught_reference_raw_rad=reference, taught_raw_rad=raw)
                message = 'Adjusted goal saved.'
            else:
                raise ValueError('Unknown teach action')
        self._save(joint, record)
        self.captured.add(joint)
        return message

    @staticmethod
    def _inward_sign(record):
        if 'inward_sample_raw_rad' not in record:
            raise ValueError('Record the inward direction with B')
        return math.copysign(1, record['inward_sample_raw_rad'] - record['reference_raw_rad'])

    def target(self, joint):
        record = self._current(joint)
        if 'delta_rad' not in record:
            raise ValueError('Hand-align to the model pose and press T, or supply a measured stop map')
        delta = finite(record['delta_rad'], 'goal offset')
        if joint in STOP_JOINTS and delta * self._inward_sign(record) <= 0:
            raise ValueError('Check the stop map: the goal must be inward from the stop')
        return record['reference_raw_rad'] + delta

    def _check_travel(self, joint, row, target):
        record = self._current(joint)
        if not row.lower_rad <= target <= row.upper_rad:
            raise ValueError('Target exceeds raw shaft bounds')
        start = row.position_rad - record['reference_raw_rad']
        end = target - record['reference_raw_rad']
        if abs(start) >= (row.upper_rad - row.lower_rad) / 2:
            raise ValueError('Current position is on an ambiguous encoder branch')
        if joint in STOP_JOINTS:
            inward = self._inward_sign(record)
            if min(start * inward, end * inward) < MIN_CLEARANCE:
                raise ValueError('Leave 0.5 raw degrees of clearance from the captured stop')

    def waypoint(self, joint, row, speed):
        target = self.target(joint)
        self._check_travel(joint, row, target)
        if abs(self._current(joint)['delta_rad']) >= (row.upper_rad - row.lower_rad) / 2:
            raise ValueError('Goal crosses an ambiguous encoder branch')
        delta = target-row.position_rad
        if abs(delta) <= math.radians(.5):
            return None
        # One press authorizes one short move, including at low requested speeds.
        step = min(math.radians(5), speed*8, abs(delta))
        return row.position_rad + math.copysign(step, delta)

    def describe(self, joint):
        try:
            raw = f'raw goal {math.degrees(self.target(joint)):+.2f}°'
        except ValueError as exc:
            raw = str(exc)
        return (f'A{joint} • {REFERENCES[joint]} • model goal '
                f'{math.degrees(self.goals[joint]):+.2f}° • {raw}')

    def trim_target(self, joint, row, direction):
        target = row.position_rad + direction * math.radians(1)
        self._check_travel(joint, row, target)
        return target
