"""Guided, observation-only raw bounds/initial-pose recording for group hold.

Consumes cached feedback from the existing owner. It never sends a motor
command, sets encoder zero, or installs the observations as operating limits.
Disk writes run separately so fsync cannot block the command-refresh loop.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import time


STAGES = ('endpoint_1', 'endpoint_2', 'initial')
REFERENCES = {
    1: ('initial', 'point the arm forward in its initial position'),
    4: ('initial', 'align the wrist to its initial position'),
    5: ('initial', 'align the wrist to its initial position'),
    6: ('initial', 'align the tool to its initial position'),
}
EXTREMES = {
    'cw_extreme': 'move to the most clockwise travel end; stop short of hard contact',
    'ccw_extreme': 'move to the most counterclockwise travel end; stop short of hard contact',
}
SETTLE_S = .5
POSITION_TOLERANCE = math.radians(.2)


def _write(path, records):
    stream = tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False)
    try:
        with stream:
            json.dump(records, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(stream.name, path)
    finally:
        Path(stream.name).unlink(missing_ok=True)


class ArmCapture:
    def __init__(self, path, pose_path, plan, simulated, note='', *, mode='reference'):
        if mode not in ('reference', 'bounds'):
            raise ValueError('Capture mode must be reference or bounds')
        self.mode = mode
        self.path = Path(path)
        self.joints = tuple(plan['active_joints'])
        angles = json.loads(Path(pose_path).read_text())['joint_angles_rad']
        self.goals = {j: angles.get(f'b{j+1}_arm_d{j}', 0.) for j in self.joints}
        self.records = dict(
            version=2, capture_mode=mode, simulated=simulated, created_utc=datetime.now(timezone.utc).isoformat(),
            capture_steps={str(j): list(self.stages_for(j)) for j in self.joints},
            coordinate_system='raw output-shaft radians; current encoder branch',
            bounds_basis='operator-chosen clearance endpoints in recorded arm configurations'
                if mode == 'bounds' or any(j in (2, 3) for j in self.joints) else None,
            viewing_convention='CW/CCW viewed from the output-shaft side, looking toward the motor body',
            fixture_note=note, plan=deepcopy(plan), model_initial_rad=self.goals,
            production_qualified=False, power_cycle_repeatability_verified=False,
            model_sign=None, model_offset=None, external_ratio=None,
            complete=False, joints={})
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Refuse to mix encoder branches or overwrite an earlier measurement run.
        with self.path.open('x') as stream:
            json.dump(self.records, stream, indent=2, allow_nan=False)
            stream.write('\n')
        self.history = deque()
        self.wait_reason = 'Waiting for fresh stationary holding feedback.'
        self.pending = None
        self.writer = ThreadPoolExecutor(max_workers=1, thread_name_prefix='arm-capture-save')

    def stages_for(self, joint):
        if self.mode == 'bounds':return STAGES
        return ('cw_extreme', 'ccw_extreme') if joint in (2, 3) else ('reference',)

    def next_stage(self, joint):
        record = self.records['joints'].get(str(joint), {})
        return next((s for s in self.stages_for(joint) if s not in record), None)

    @staticmethod
    def stage_label(stage):
        return {'cw_extreme': 'CW extreme', 'ccw_extreme': 'CCW extreme'}.get(stage, stage)

    def observe(self, rows, eligible, now=None):
        now = time.monotonic() if now is None else now
        rows = {row.joint: row for row in rows}
        problem = None
        if not eligible:
            problem = 'Activate H, release arrows, and press Space to pause/hold.'
        elif set(rows) != set(self.joints):
            problem = 'Waiting for feedback from every configured joint.'
        else:
            for joint, row in rows.items():
                if not row.enabled:
                    problem = f'A{joint} is not enabled for holding.'
                elif not 0 <= row.age_ms < 80:
                    problem = f'A{joint} feedback is stale ({row.age_ms} ms; must be below 80 ms).'
                elif not math.isfinite(row.position_rad):
                    problem = f'A{joint} position is invalid.'
                elif row.velocity_rad_s is None and not self.records['simulated']:
                    problem = f'A{joint} has no speed feedback.'
                elif row.velocity_rad_s is not None:
                    if not math.isfinite(row.velocity_rad_s):
                        problem = f'A{joint} speed feedback is invalid.'
                    elif abs(row.velocity_rad_s) > .03:
                        problem = f'A{joint} is not stationary: {row.velocity_rad_s:+.3f} rad/s (maximum 0.030).'
                if problem:break
        if problem:
            self.wait_reason = problem
            self.history.clear()
            return
        if self.history and (now <= self.history[-1][0] or now-self.history[-1][0] > .1):
            self.history.clear()
        self.history.append((now, rows))
        while len(self.history) > 1 and now-self.history[1][0] >= SETTLE_S:
            self.history.popleft()

    def readiness_reason(self, now=None):
        """Explain a rejected sample without fetching feedback or changing holds."""
        now = time.monotonic() if now is None else now
        if not self.history:
            return f'Waiting for stationary holding: {self.wait_reason}'
        if now-self.history[-1][0] > .08:
            return 'Waiting for fresh stationary holding feedback; latest sample is over 80 ms old.'
        span = self.history[-1][0]-self.history[0][0]
        if span < SETTLE_S or len(self.history) < 5:
            return f'Waiting for stationary holding: {span:.2f} / {SETTLE_S:.2f} s collected.'
        for joint in self.joints:
            values = [rows[joint].position_rad for _, rows in self.history]
            if max(values)-min(values) > POSITION_TOLERANCE:
                return f'A{joint} has not settled within 0.2°. Wait before capturing.'
        elapsed_ms = max(0., (now-self.history[-1][0])*1000)
        for joint, row in self.history[-1][1].items():
            if row.age_ms+elapsed_ms >= 80:
                return f'A{joint} feedback aged before capture. Wait for the next owner sample.'
        return None

    def remaining_joints(self):
        return [j for j in self.joints if self.next_stage(j) is not None]

    def progress(self):
        saved = len(self.joints)-len(self.remaining_joints())
        details = []
        for joint in self.joints:
            record = self.records['joints'].get(str(joint), {})
            stages = self.stages_for(joint)
            if self.mode == 'reference' and joint in (2, 3):
                values = [f'{direction} {record[s]["raw_deg"]:+.3f}°' if s in record else f'{direction} pending'
                          for s, direction in zip(stages, ('CW', 'CCW'))]
                details.append(f'A{joint}: ' + ', '.join(values))
            else:
                stage = next((s for s in reversed(stages) if s in record), None)
                details.append(f'A{joint}: {record[stage]["raw_deg"]:+.3f}° saved' if stage else f'A{joint}: pending')
        lines = ['  •  '.join(details[start:start+3]) for start in range(0, len(details), 3)]
        return f'{saved}/{len(self.joints)} motors recorded\n' + '\n'.join(lines)

    def describe(self, joint):
        stage = self.next_stage(joint)
        if stage is None:
            if self.mode == 'reference':
                remaining = self.remaining_joints()
                return (f'A{joint} saved. Select A{remaining[0]} with {remaining[0]}.' if remaining else
                        'All requested positions saved. Space keeps holding; closing removes torque.')
            return f'A{joint}: all three captures saved. Select the next joint (1–6).'
        if self.mode == 'reference':
            if stage in EXTREMES:
                step = self.stages_for(joint).index(stage)+1
                return (f'A{joint} — {step}/2 ({self.stage_label(stage)}): {EXTREMES[stage]} '
                        '(viewed from the output-shaft side). Release → Space → wait for Ready → C to save.')
            return f'A{joint}: {REFERENCES[joint][1]}. Release → Space → wait for Ready → C to save.'
        instruction = {
            'endpoint_1': 'jog to one usable travel endpoint, leaving physical/cable clearance',
            'endpoint_2': 'jog to the opposite usable travel endpoint, leaving clearance',
            'initial': f'align to the model initial pose ({math.degrees(self.goals[joint]):+.2f}° model angle)',
        }[stage]
        return f'A{joint} — {STAGES.index(stage)+1}/3: {instruction}. Release → Space → wait 0.5 s → C.'

    def capture(self, joint, base_info=None, now=None):
        now = time.monotonic() if now is None else now
        if self.pending:
            raise ValueError('Still saving the previous capture. Wait for Saved before capturing again.')
        problem = self.readiness_reason(now)
        if problem:raise ValueError(problem)
        record = deepcopy(self.records['joints'].get(str(joint), {}))
        stage = self.next_stage(joint)
        if stage is None:
            raise ValueError('This joint is saved. Select the next joint; use a new file to redo.')
        rows = self.history[-1][1]
        row = rows[joint]
        raw = row.position_rad
        if not row.lower_rad <= raw <= row.upper_rad:
            raise ValueError('Raw reading is outside the protocol range.')
        if stage == 'reference':
            kind, instruction = REFERENCES[joint]
            record.update(reference_raw_rad=raw, reference_kind=kind, physical_instruction=instruction)
            if kind == 'initial':
                record.update(initial_raw_rad=raw, model_initial_rad=self.goals[joint])
            else:
                record.update(model_reference_rad=None, endstop_contact_verified=False)
        elif stage in EXTREMES:
            record.update(model_reference_rad=None, endstop_contact_verified=False)
            record['raw_cw_rad' if stage == 'cw_extreme' else 'raw_ccw_rad'] = raw
        if stage in ('endpoint_2', 'ccw_extreme'):
            other = record['endpoint_1' if stage == 'endpoint_2' else 'cw_extreme']['raw_rad']
            if abs(raw-other) < math.radians(.5):
                raise ValueError('Endpoints must be at least 0.5 raw degrees apart.')
            if abs(raw-other) >= (row.upper_rad-row.lower_rad)/2:
                raise ValueError('Ambiguous encoder branch: do not combine endpoints across a wrap.')
            record.update(raw_lower_rad=min(other, raw), raw_upper_rad=max(other, raw))
        elif stage == 'initial':
            if not record['raw_lower_rad'] <= raw <= record['raw_upper_rad']:
                raise ValueError('Initial position must lie inside the recorded clearance endpoints.')
            record.update(initial_raw_rad=raw, model_initial_rad=self.goals[joint])
        record[stage] = dict(
            raw_rad=raw, raw_deg=math.degrees(raw), feedback_age_ms=row.age_ms,
            at_monotonic_s=now, captured_utc=datetime.now(timezone.utc).isoformat(),
            settled_span_s=self.history[-1][0]-self.history[0][0],
            settled_samples=len(self.history),
            group_raw_rad={str(j): r.position_rad for j, r in rows.items()},
            group_feedback_age_ms={str(j): r.age_ms for j, r in rows.items()})
        updated = deepcopy(self.records)
        updated['joints'][str(joint)] = record
        updated['base_info'] = deepcopy(base_info)
        updated['complete'] = all(all(s in updated['joints'].get(str(j), {}) for s in self.stages_for(j))
                                  for j in self.joints)
        self.pending = (self.writer.submit(_write, self.path, updated), updated, joint, stage)
        return f'Saving A{joint} {self.stage_label(stage)}: raw {math.degrees(raw):+.3f}°…'

    def finish_save(self):
        if not self.pending or not self.pending[0].done():
            return None
        future, updated, joint, stage = self.pending
        self.pending = None
        try:
            future.result()
        except Exception as exc:
            # A file error is not a motor fault; keep holding and allow retry.
            return f'Capture NOT saved: {exc}. Press C to retry after fixing the file error.'
        self.records = updated
        raw_deg = updated['joints'][str(joint)][stage]['raw_deg']
        return f'Saved A{joint} {self.stage_label(stage)}: raw {raw_deg:+.3f}°.' + (' All joints recorded.' if updated['complete'] else '')

    def close(self):
        self.writer.shutdown(wait=True)
        return self.finish_save()
