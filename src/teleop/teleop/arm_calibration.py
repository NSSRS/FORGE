"""Standalone model-guided calibration; manual jogging has its own entry point."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
import multiprocessing
from pathlib import Path
import sys
import time
import uuid

from motors.arm_bench import read_arm_plan
from motors.arm_calibration import (CalibrationMap, CalibrationRecord, CalibrationGroupHoldPort,
    SimulatedCalibrationPort, snapshot, validate_start, constraint_limits)
from .arm_keyboard import ArmJogKeys, make_window


class CalibrationKeys(ArmJogKeys):
    def __init__(self, port, speed_deg_s, *, mapping, record, scene, scene_fingerprint,
                 executor, options, **kwargs):
        super().__init__(port, speed_deg_s, **kwargs)
        self.mapping, self.record = mapping, record
        self.scene, self.scene_fingerprint = scene, scene_fingerprint
        self.executor, self.options = executor, options
        self.future = None;self.path = None;self.run = None
        self.constraint_side = 'lower';self.constraint_reason = ''
        if not self.failed:
            self.message = 'P calculates the selected axis from measured feedback. H holds all. [ / ] start a calculated path. Space pauses.'

    def is_motion_key(self, key):
        return key in ('[', ']')

    def stop(self):
        active = self.run
        self.run = None
        if self.future:
            self.future.cancel();self.future = None
        super().stop()
        if active:
            self.path = None
            self.record.event('paused', joint=active['joint'], target_raw_deg=active['goal'])
            self.message = 'Paused and holding. P recalculates; a fresh [ / ] starts movement. D disables for supported hand positioning.'

    def disable(self):
        self.run = None;self.path = None
        if self.future:
            self.future.cancel();self.future = None
        super().disable()

    def begin_recovery(self):
        self.run = None;self.path = None
        if self.future:
            self.future.cancel();self.future = None
        return super().begin_recovery()

    def fail(self, error):
        self.run = None;self.path = None
        super().fail(error)

    def press(self, key, repeat=False):
        if repeat:
            return
        if key in (' ', 'stop', 'x', 'd', 'q', 'r'):
            return super().press(key)
        if self.failed or self.recovering:
            return
        if key in self.directions or key == 'e':
            self.stop()
            self.message = 'This is model-guided calibration. P calculates; [ / ] starts. Use control_arm.sh for manual jogging.'
        elif key in tuple('123456') or key == 'h':
            self.stop();self.path = None
            super().press(key)
            self.message = f'A{self.port.selected} selected. P calculates a path; H holds all before [ / ] starts.'
        elif key == 'p':
            self.calculate()
        elif key in ('[', ']'):
            try:self.start_path('lower' if key == '[' else 'upper')
            except ValueError as exc:self.message = str(exc)
        elif key == 'c':
            try:self.record_constraint()
            except ValueError as exc:self.message = str(exc)

    def release(self, key):
        # A path is an explicit bounded operation, not a held-arrow manual jog.
        self.down.discard(key)

    def fresh_pose(self, *, disabled=False):
        if self.port.moving:
            raise ValueError('Pause movement first')
        if disabled and self.port.holding:
            raise ValueError('Support all links and press D before hand-position recording')
        if self.port.holding:self.port.poll()
        else:self.port.stationary_position()  # Acknowledged disabled feedback, never enable.
        return snapshot(self.port.cached_positions(), enabled=False if disabled else None, stationary=True)

    def calculate(self):
        self.stop();self.path = None
        try:
            raw = self.fresh_pose()
            self.mapping.pose(raw)
        except ValueError as exc:
            self.message = str(exc);return
        from simulation.arm_calibration import calculate_path
        self.future = self.executor.submit(calculate_path, dict(scene=str(self.scene),
            mapping=self.mapping.document, raw=raw, joint=self.port.selected,
            constraints=list(self.record.data['constraints']), options=self.options))
        self.message = 'Calculating both directions from this six-axis pose. No motion starts when calculation finishes.'

    def start_path(self, side):
        if self.run or self.future or self.path is None:
            raise ValueError('Press P and wait for a calculated path first')
        if not self.port.holding:
            raise ValueError('Press H to hold all joints, then P to calculate')
        if self.path['joint'] != self.port.selected:
            raise ValueError('Selection changed; calculate again')
        if self.path['scene_fingerprint'] != self.scene_fingerprint or self.path['mapping_fingerprint'] != self.mapping.fingerprint:
            raise ValueError('Model/mapping changed; restart calibration')
        raw = self.fresh_pose()
        validate_start(self.path, raw)
        goal = self.path['ends'][side]['raw_deg']
        current = raw[str(self.port.selected)]
        if abs(goal-current) < .1:
            self.message = 'No travel available in this direction after model clearance/backoff.';return
        self.run = dict(joint=self.port.selected, goal=goal, start=current,
                        direction=1 if goal > current else -1, pose=raw,
                        deadline=time.monotonic()+abs(goal-current)/math.degrees(self.speed)*4+10)
        self.record.event('path_started', joint=self.port.selected, side=side,
                          start_raw_deg=raw, goal_raw_deg=goal)
        try:self.next_waypoint(raw)
        except Exception:
            self.run = None
            raise
        self.message = f'A{self.port.selected} moving toward calculated raw {goal:+.2f}°. Space pauses and holds.'

    def next_waypoint(self, raw):
        joint = self.run['joint']
        current = raw[str(joint)]
        delta = self.run['goal']-current
        self.run['waypoint'] = current+max(-1., min(1., delta))
        self.run['segment_deadline'] = time.monotonic()+4/math.degrees(self.speed)+3
        self.port.arm()
        self.port.start_waypoint(math.radians(self.run['waypoint']), self.speed)

    def tick(self):
        super().tick()
        self.record.check()
        if self.failed or self.recovering:
            return
        if self.future and self.future.done():
            future, self.future = self.future, None
            try:
                self.path = future.result()
                self.record.data['plans'].append(self.path);self.record.save()
                lower, upper = (self.path['ends'][s]['raw_deg'] for s in ('lower', 'upper'))
                self.message = f'Calculated A{self.path["joint"]}: raw {lower:+.2f}° to {upper:+.2f}°. H holds; [ / ] starts one direction. Space pauses.'
            except Exception as exc:
                self.path = None;self.message = f'No path authorized: {exc}'
        if not self.run:
            return
        rows = self.port.cached_positions()
        raw = snapshot(rows, enabled=True)
        joint = self.run['joint'];current = raw[str(joint)]
        if time.monotonic() > min(self.run['deadline'], self.run['segment_deadline']):
            self.stop();self.message = 'Progress timeout; paused. Inspect the obstruction before recalculating.';return
        for other, angle in raw.items():
            if int(other) != joint and abs(angle-self.run['pose'][other]) > .3:
                self.stop();self.message = f'A{other} moved from the planned pose; paused. P recalculates.';return
        lo, hi = sorted((self.run['start'], self.run['goal']))
        if not lo-.3 <= current <= hi+.3:
            self.stop();self.message = 'Measured angle left the planned corridor; paused. Check the encoder branch and pose.';return
        row = next(r for r in rows if r.joint == joint)
        stationary = row.velocity_rad_s is None or abs(row.velocity_rad_s) < .03
        if abs(current-self.run['waypoint']) <= .05 and stationary:
            self.port.pause()
            if abs(current-self.run['goal']) <= .1:
                goal = self.run['goal'];self.run = None;self.path = None
                self.record.event('path_finished', joint=joint, target_raw_deg=goal, measured_raw_deg=raw)
                self.message = 'Calculated boundary reached; holding and disarmed. P calculates the next path.'
            else:self.next_waypoint(snapshot(self.port.cached_positions(), enabled=True))

    def record_constraint(self):
        if self.run or self.future:
            raise ValueError('Pause the path/calculation before recording')
        raw = self.fresh_pose(disabled=True)
        self.mapping.pose(raw)
        self.record.add_constraint(joint=self.port.selected, side=self.constraint_side,
            reason=self.constraint_reason, raw=raw, backoff_deg=self.options['backoff_deg'],
            scene_fingerprint=self.scene_fingerprint)
        self.path = None
        self.message = f'Saved A{self.port.selected} {self.constraint_side} raw constraint at {raw[str(self.port.selected)]:+.2f}°. Back off by hand, then H and P. No movement started.'


def model_position_table(mapping):
    from .arm_widgets import JointPositionTable
    class ModelTable(JointPositionTable):
        def update_positions(self, rows, selected=None, stale=False):
            rows=list(rows)
            super().update_positions(rows,selected,stale)
            for row in rows:
                try:value=f'{mapping.model(row.joint,math.degrees(row.position_rad)):+.2f}'
                except ValueError:value='BRANCH?'
                self.item(row.joint-1,1).setText(value)
    table=ModelTable()
    table.horizontalHeaderItem(1).setText('Model °')
    table.horizontalHeaderItem(1).setToolTip('Candidate raw-to-model mapping; not the session-relative angle.')
    return table


def calibration_panel(window, layout):
    from PyQt5 import QtCore, QtWidgets
    class StopFilter(QtCore.QObject):
        def eventFilter(self, obj, event):
            if (event.type() == QtCore.QEvent.KeyPress and not event.isAutoRepeat() and
                    event.key() in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape) and
                    isinstance(obj, QtWidgets.QWidget) and
                    (obj is window or window.isAncestorOf(obj)) and
                    (window.starting or window.keys.port.moving or window.keys.port.armed or
                     event.key() == QtCore.Qt.Key_Escape)):
                window.invoke(window.keys.stop)
                return True
            return False
    window.stop_filter = StopFilter(window)
    QtWidgets.QApplication.instance().installEventFilter(window.stop_filter)
    window.shortcuts.setText('1–6 select • H hold all • P calculate • [ / ] start a calculated direction • Space pause/hold • D disable • R recover')
    window.arm_button.hide()
    group = QtWidgets.QGroupBox('Model-guided calibration')
    box = QtWidgets.QVBoxLayout(group)
    row = QtWidgets.QHBoxLayout();box.addLayout(row)
    for label, key in (('Calculate both directions (P)', 'p'), ('Run toward lower raw ([)', '['), ('Run toward upper raw (])', ']')):
        button = QtWidgets.QPushButton(label);button.setFocusPolicy(QtCore.Qt.NoFocus)
        button.clicked.connect(lambda checked=False, key=key:window.invoke(lambda:window.start_motion(key)))
        row.addWidget(button)
    inputs = QtWidgets.QHBoxLayout();box.addLayout(inputs)
    side = QtWidgets.QComboBox();side.addItems(['Lower raw limit', 'Upper raw limit'])
    side.currentIndexChanged.connect(lambda i:setattr(window.keys, 'constraint_side', 'lower' if i == 0 else 'upper'))
    reason = QtWidgets.QLineEdit();reason.setPlaceholderText('Constraint reason, e.g. new tool / cable / bracket')
    window.constraint_reason_input = reason
    reason.textChanged.connect(lambda text:setattr(window.keys, 'constraint_reason', text))
    save = QtWidgets.QPushButton('Record hand-positioned constraint (C)');save.setFocusPolicy(QtCore.Qt.NoFocus)
    save.clicked.connect(lambda:window.invoke(lambda:window.keys.press('c')))
    inputs.addWidget(side);inputs.addWidget(reason);inputs.addWidget(save)
    hint = QtWidgets.QLabel('Space pauses/holds. Support → D → hand-position → Record.\nSaved obstacles narrow future paths with backoff. No zero/homing changes.')
    hint.setWordWrap(True);box.addWidget(hint)
    plan_label = QtWidgets.QLabel('No path calculated yet.');plan_label.setWordWrap(True);box.addWidget(plan_label)
    def show_plan():
        path=window.keys.path
        if path:
            plan_label.setText('\n'.join(f'{side}: raw {path["ends"][side]["raw_deg"]:+.2f}°, model {path["ends"][side]["model_deg"]:+.2f}° — {path["ends"][side]["reason"]}' for side in ('lower','upper')))
        else:plan_label.setText('No current path. P calculates from fresh measured coordinates.')
        for label in (hint,plan_label):label.setMinimumHeight(label.heightForWidth(max(1,label.width())))
    window.plan_display_timer=QtCore.QTimer(window);window.plan_display_timer.timeout.connect(show_plan)
    window.plan_display_timer.start(200)
    layout.addWidget(group)
    window.setWindowTitle('FORGE • Model-guided arm calibration')
    window.resize(1050,760)


def main(argv=None):
    from robot_description.paths import model_directory
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--simulate', action='store_true', help='offline panel; default')
    mode.add_argument('--execute', action='store_true', help='hardware panel; H and explicit path start still required')
    mode.add_argument('--plan-only', action='store_true', help='calculate selected-axis paths without opening a window or hardware')
    parser.add_argument('--supported', action='store_true')
    parser.add_argument('--plan', type=Path, default=root/'src/robot_description/config/arm_rig_a1_a6.json')
    parser.add_argument('--mapping', type=Path, default=root/'src/robot_description/config/arm_calibration_candidate.json')
    parser.add_argument('--scene', type=Path, default=model_directory()/'robot.xml', help='default is robot-only bench geometry; use an appropriate environment scene when available')
    parser.add_argument('--equipment-revision', help='new revision for a changed mounting; previous constraints will not silently transfer')
    parser.add_argument('--load-constraints', type=Path, help='reuse constraints from a previous run with exactly matching model/mapping')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--joint', type=int, choices=range(1,7), default=1)
    parser.add_argument('--raw-deg', nargs=6, type=float, help='offline starting pose only; never a hardware target')
    parser.add_argument('--speed-deg-s', type=float, default=2.)
    parser.add_argument('--clearance-mm', type=float, default=2.)
    parser.add_argument('--stop-clearance-mm', type=float, default=.1)
    parser.add_argument('--step-deg', type=float, default=.25)
    parser.add_argument('--extent-deg', type=float, default=90.)
    parser.add_argument('--backoff-deg', type=float, default=2.)
    args = parser.parse_args(argv)
    if args.execute and (not args.supported or args.raw_deg is not None):
        parser.error('Hardware requires --supported and uses measured coordinates, never --raw-deg')
    if not math.isfinite(args.speed_deg_s) or not 0 < args.speed_deg_s <= 5:
        parser.error('Calibration speed must be >0 and <=5 deg/s')
    record = port = executor = window = None
    result = 2
    try:
        from simulation.arm_calibration import ModelCalibrationPlanner
        # Load dependencies and validate model/options before acquiring hardware.
        import fcl  # noqa: F401
        document = json.loads(args.mapping.read_text())
        if args.equipment_revision:document['equipment_revision'] = args.equipment_revision
        mapping = CalibrationMap(document)
        options = {key:getattr(args,key) for key in ('clearance_mm','stop_clearance_mm','step_deg','extent_deg','backoff_deg')}
        model = ModelCalibrationPlanner(args.scene, mapping.document, **options)
        fingerprint = model.fingerprint
        constraints = []
        if args.load_constraints:
            previous = json.loads(args.load_constraints.read_text())
            if previous.get('simulated') and args.execute:
                raise ValueError('Simulation constraints cannot authorize hardware movement')
            if previous.get('mapping_fingerprint') != mapping.fingerprint or previous.get('scene_fingerprint') != fingerprint:
                raise ValueError('Saved run belongs to a different model or mounting revision; start a new run')
            constraints = previous['constraints']
            for joint in range(1,7):constraint_limits(mapping,joint,constraints,fingerprint)
        output = args.output or root/'build/arm_tests'/f'calibration-{time.time_ns()}-{uuid.uuid4().hex[:6]}.json'
        record = CalibrationRecord(output, mapping, simulated=not args.execute, constraints=constraints)
        record.data['scene_fingerprint'] = fingerprint
        raw = {str(j):v for j,v in enumerate(args.raw_deg,1)} if args.raw_deg else mapping.simulation_pose()
        if args.plan_only:
            path = model.plan(raw,args.joint,constraints)
            record.data['plans'].append(path);record.save()
            print(json.dumps(path,indent=2));result=0
        else:
            from PyQt5 import QtWidgets
            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])
            plan, joints = read_arm_plan(args.plan)
            if set(joints) != set(range(1,7)):
                raise ValueError('Model-guided calibration requires all six configured joints')
            executor = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))
            if args.execute:
                plan.update(fixture_confirmed=True, fixture_mode='supported', supported_confirmed=True)
                port = CalibrationGroupHoldPort(plan, port=args.port, window_degrees=None,
                    allow_faulted_start=True, smooth_jog=True)
            else:
                port = SimulatedCalibrationPort(window_degrees=None)
                port.values = [math.radians(raw[str(j)]) for j in range(1,7)]
            if not getattr(port, 'recovery_required', False):port.select(args.joint)
            report_path = output.with_name(output.stem+'-session.json')
            def keys_factory(p, speed, **kwargs):
                return CalibrationKeys(p,speed,mapping=mapping,record=record,scene=args.scene,
                    scene_fingerprint=fingerprint,executor=executor,options=options,**kwargs)
            window = make_window(port,'HARDWARE (SUPPORTED)' if args.execute else 'SIMULATION',
                speed_deg_s=args.speed_deg_s,recovery_report_path=report_path,
                keys_factory=keys_factory,panel_factory=calibration_panel,
                table_factory=lambda:model_position_table(mapping))
            window.show();print(f'Calibration record: {output}',flush=True)
            app.exec_();result = 1 if window.keys.failed else 0
    except (ImportError, ModuleNotFoundError) as exc:
        print(f'{exc}. Install src/simulation/calibration-requirements.txt in the Python environment.',file=sys.stderr)
    except Exception as exc:
        print(str(exc),file=sys.stderr)
    finally:
        if window is not None and window.recovery_thread is not None:window.recovery_thread.wait()
        if port:
            try:port.close()
            except Exception as exc:print(f'Cleanup failed: {exc}',file=sys.stderr);result=2
            if record:
                record.data['session'] = port.report
                try:record.save()
                except Exception as exc:print(f'Record save failed: {exc}',file=sys.stderr);result=2
        if executor:executor.shutdown(wait=False,cancel_futures=True)
        if record:
            try:
                record.data['exit_code']=result;record.save();record.close()
            except Exception as exc:print(f'Record save failed: {exc}',file=sys.stderr);result=2
    return result
