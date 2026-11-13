"""Desktop, hold-to-jog arm control; independent of ENCOS WASD."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
from motors.arm_position import (ArmPositionPort, MotionCancelled, SimulatedPositionPort,
                                 SUPPORTED_JOG_DEG_S, TRIAL_JOG_DEG_S, HOLD_JOG_DEG_S)
from motors.arm_group_hold import ArmGroupHoldPort, SimulatedGroupHoldPort
from motors.arm_pose_teach import PoseTeach
from motors.arm_capture import ArmCapture
from motors.arm_bench import read_arm_plan


class ArmJogKeys:
    """Press/release policy, independent of Qt and hardware transport."""
    directions={'right':1,'up':1,'left':-1,'down':-1}
    def __init__(self,port,speed_deg_s=45,*,recovery_report_path=None,teach=None,capture=None):
        if not math.isfinite(speed_deg_s) or not 0<speed_deg_s<=45:
            raise ValueError('Jog speed must be >0 and <=45 degrees/s')
        self.speed=math.radians(speed_deg_s)
        self.teach=teach
        self.capture=capture
        self.capture_message='No capture attempted yet.'
        self.position_hold=bool(getattr(port,'position_hold',False))
        self.group_hold=bool(getattr(port,'group_hold',False))
        self.port=port;self.held=set();self.down=set();self.quit=False
        self.recovering=False;self.recovery_report_path=recovery_report_path
        self.failed=bool(getattr(port,'recovery_required',False))
        self.message=(f"FAULT: {port.report.get('error','Recovery required')}. Check the cause, then press R."
                      if self.failed else 'Press H to hold all connected joints, then select a joint and press E.'
                      if self.group_hold else 'Select A1–A6, press E once, then hold an arrow.')
        self.last_tick=time.monotonic()
    def stop(self):
        self.held.clear()
        if self.position_hold:self.port.pause()
        else:self.port.stop()
        if not self.failed:
            self.message=('Holding position; motion disarmed. E arms again. D disables torque.'
                          if getattr(self.port,'holding',False) else 'Stopped and disarmed. E arms again.')
    def disable(self):
        self.held.clear();self.port.stop()
        if not self.failed:self.message='Torque disabled. Support the arm. E arms again.'
    def is_motion_key(self, key):
        return key in self.directions or (self.teach is not None and key == 'h')

    def _capture_pose(self, key):
        if self.port.armed or self.down or self.port.moving:
            self.message = 'Release all motion keys and press Space to disarm before capturing.'
            return
        row = self.port.stationary_position()
        try:
            self.message = self.teach.capture(self.port.selected, key, row)
        except ValueError as exc:
            self.message = str(exc)
            return
        self.port.report.setdefault('pose_captures', []).append(dict(
            joint=row.joint, action=key, raw_rad=row.position_rad, feedback_age_ms=row.age_ms,
            model_goal_rad=self.teach.goals[row.joint], at_monotonic_s=time.monotonic(),
            record=dict(self.teach.records['joints'][str(row.joint)])))

    def capture_block_reason(self):
        if self.failed or self.recovering:return 'Resolve the displayed fault and recover before capturing.'
        if not self.port.holding:return 'Press H to activate holding before capturing.'
        if self.down or self.port.moving:return 'Release all arrows, then press Space before capturing.'
        if self.port.armed:return 'Press Space to pause/hold before capturing.'
        return self.capture.readiness_reason()

    def _capture_notice(self,message):
        self.capture_message=message
        self.message=message
        self.port.report.setdefault('capture_events',[]).append(dict(
            at_monotonic_s=time.monotonic(),selected_joint=self.port.selected,message=message))

    def _capture_bounds(self):
        reason=self.capture_block_reason()
        if reason:
            self._capture_notice(f'NOT SAVED — A{self.port.selected}: {reason}')
            return
        try:
            self._capture_notice(self.capture.capture(self.port.selected,self.port.report.get('base_info')))
        except ValueError as exc:
            self._capture_notice(f'NOT SAVED — A{self.port.selected}: {exc}')

    def press(self,key,repeat=False):
        if repeat:return
        motion_key = self.is_motion_key(key)
        if motion_key:self.down.add(key)
        if self.recovering:return
        if key=='r':self.recover();return
        if key=='q':self.disable();self.quit=True;return
        if key=='d' and self.position_hold:self.disable();return
        if key in ('stop',' ','x'):self.stop();return
        if self.failed:return
        if self.capture and key=='c':
            self._capture_bounds();return
        if key=='h' and self.group_hold:
            if self.down:
                self.message='Release arrows before activating Hold all.';return
            self.port.hold_all(self.speed);self.last_tick=time.monotonic()
            self.message='All connected joints holding. Select 1–6, then E and an arrow. D disables all torque.'
            return
        if self.teach and key in ('c','b','t'):
            self._capture_pose(key)
            return
        if key in tuple('123456'):
            if self.position_hold and not self.group_hold and (self.port.moving or self.port.holding):
                self.message='Selection locked while torque is active. Support the arm and press D before switching.'
                return
            if int(key) not in self.port.available_joints:
                self.stop();self.message=f'A{key} unavailable. Select a connected motor.';return
            if int(key)!=self.port.selected:
                self.held.clear();self.port.select(int(key))
                self.message=f'A{key} selected. Press E to arm.'+(' Other joints keep holding.' if self.group_hold and self.port.holding else '')
            return
        if key=='e':
            if self.group_hold and not self.port.hold_targets:
                self.message='Press H to hold all connected joints first.';return
            if not self.down and not self.port.armed:
                self.port.arm();self.message='Armed. Hold an arrow; release to stop.'
            return
        if not motion_key or key in self.held:return
        self.held.add(key)
        if len(self.down)>1:
            self.port.release();self.message='Release all arrows before changing direction.';return
        if not self.port.armed:
            self.message='Release arrow, then press E to arm.';return
        self._start_move(key)

    def _start_move(self, key):
        if self.teach:
            row = self.port.stationary_position()
            j = self.port.selected
            try:
                if key == 'h':
                    target = self.teach.waypoint(j, row, self.speed)
                else:
                    target = self.teach.trim_target(j, row, self.directions[key])
            except ValueError as exc:
                self.message = str(exc)
                return
            if target is None:
                self.message = 'Within 0.5° of the raw goal. Release H and compare the physical pose.'
                return
            self.port.move_to(target,self.speed)
            self.message=f'A{j} moving toward raw {math.degrees(target):+.2f}°. Release to stop.'
        elif self.port.window is None:
            self.port.jog(self.directions[key],self.speed)
            self.message=('Manual travel — release holds position; Space pauses; D disables torque.'
                          if self.position_hold else 'Manual travel — release to stop; 12 s maximum per hold.')
        else:
            row=next(row for row in self.port.positions() if row.joint==self.port.selected)
            target=row.upper_rad if self.directions[key]>0 else row.lower_rad
            self.port.move_to(target,self.speed)
            self.message=f'Moving while held — release to stop; ±{math.degrees(self.port.window):g}° session window.'
        self.last_tick=time.monotonic()
    def release(self,key):
        self.down.discard(key)
        if self.recovering:return
        if key in self.held:
            self.held.remove(key);self.port.release()
            self.message=('Holding position; still armed. Hold an arrow to move again; D disables torque.'
                          if getattr(self.port,'holding',False) else 'Stopped; still armed. Hold an arrow to move again.')
    def begin_recovery(self):
        if self.recovering:return False
        if getattr(self.port,'holding',False):
            self.message='Support the arm and press D to disable torque before recovery.'
            return False
        if self.down or self.port.moving:
            self.message='Release all arrows before recovery.'
            return False
        self.held.clear();self.failed=True;self.recovering=True
        if self.teach:self.teach.reset_session()
        if self.capture:self.capture.history.clear()
        self.message='Recovering; verifying disabled motors. Motion stays disarmed.'
        return True
    def finish_recovery(self,error=None):
        self.recovering=False;self.failed=error is not None
        self.last_tick=time.monotonic()
        self.message=(f'Recovery failed: {error}. Address the cause, then press R again.' if error else
                      'Recovered and disarmed. Release arrows, press E, then a fresh arrow to move.')
    def recover(self):
        if not self.begin_recovery():return
        try:self.port.recover(self.recovery_report_path)
        except Exception as exc:self.finish_recovery(str(exc))
        else:self.finish_recovery()
    def fail(self,error):
        self.failed=True;self.held.clear()
        detail=str(error)
        try:
            if hasattr(self.port,'capture_failure'):detail=self.port.capture_failure(error)
            else:self.port.stop();self.port.report['error']=detail
        except Exception as exc:self.port.report['stop_error']=str(exc)
        self.message=f'FAULT: {detail}. Release arrows, check the cause, then press R to recover.'
    def tick(self):
        if self.recovering:return
        now=time.monotonic()
        active=self.port.moving or getattr(self.port,'holding',False)
        if active and now-self.last_tick>.15:
            self.disable();raise RuntimeError('UI stalled; torque disabled. Inspect controller before recovery.')
        self.last_tick=now
        if not self.failed and active:
            self.port.poll()
            if not self.port.moving and not getattr(self.port,'holding',False):
                self.message=('Step complete. Release the key and compare the pose.' if self.teach else
                              'Travel boundary reached. Release and reverse direction.')
        if self.capture:
            self.capture.observe(self.port.cached_positions(),not self.failed and
                self.port.holding and not self.port.moving and not self.port.armed and not self.down)
            message=self.capture.finish_save()
            if message:self._capture_notice(message)
            self.port.report['position_capture']=self.capture.records


def make_window(port,mode,*,speed_deg_s=45,recovery_report_path=None,teach=None,capture=None,
                keys_factory=ArmJogKeys,panel_factory=None,table_factory=None):
    from PyQt5 import QtCore,QtWidgets
    from .arm_widgets import JointPositionTable
    class RecoveryThread(QtCore.QThread):
        def __init__(self,parent):
            super().__init__(parent);self.error=None
        def run(self):
            try:port.recover(recovery_report_path)
            except Exception as exc:self.error=str(exc)

    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__();self.keys=keys_factory(port,speed_deg_s,recovery_report_path=recovery_report_path,teach=teach,capture=capture);self.next_display=0
            self.recovery_thread=None;self.close_after_recovery=False
            self.starting=False;self.cancel_start=False;self.close_after_start=False
            self.setWindowTitle('FORGE • Arm joint jog • '+mode);self.resize(880,420)
            layout=QtWidgets.QVBoxLayout(self);layout.setContentsMargins(8,8,8,8);layout.setSpacing(4)
            self.title=QtWidgets.QLabel();layout.addWidget(self.title)
            shortcuts=QtWidgets.QLabel('1–6 select   •   E arm   •   Hold ↑/→ positive, ↓/← negative   •   '+
                ('Space pause/hold   •   D disable   •   R recover' if self.keys.position_hold else 'Space stop   •   R recover'))
            shortcuts.setWordWrap(True);layout.addWidget(shortcuts);self.shortcuts=shortcuts
            if capture:
                from .arm_recording import CapturePanel
                self.recording_panel=CapturePanel(self,port,capture)
                layout.addWidget(self.recording_panel)
            if teach:
                layout.addWidget(QtWidgets.QLabel('C reference   •   B inward sample   •   T save aligned goal   •   Hold H toward goal   •   Arrows trim 1°'))
                self.pose_label=QtWidgets.QLabel();self.pose_label.setWordWrap(True);layout.addWidget(self.pose_label)
                if isinstance(port,SimulatedPositionPort):
                    hand=QtWidgets.QPushButton('Place simulated shaft by hand…')
                    hand.setFocusPolicy(QtCore.Qt.NoFocus)
                    hand.clicked.connect(self.place_simulated);layout.addWidget(hand)
            travel='Manual travel • 12 s maximum per hold' if port.window is None else f'±{math.degrees(port.window):g}° window'
            if self.keys.position_hold:
                travel='Position hold • no bench duration cutoff • Release holds • Focus loss pauses/holds'
            if self.keys.group_hold:
                travel='Hold all • H activates • 1–6 selects without releasing torque • Release / Space holds'
            elif not self.keys.position_hold:travel+=' • Release stops • Focus loss disarms'
            if getattr(port,'smooth_jog',False):travel+=' • Smooth jog (2 ms)'
            layout.addWidget(QtWidgets.QLabel(f'{speed_deg_s:g}°/s • {travel}'))
            buttons=QtWidgets.QHBoxLayout();layout.addLayout(buttons)
            for n in range(1,7):
                b=QtWidgets.QPushButton(f'A{n}');b.setFocusPolicy(QtCore.Qt.NoFocus)
                if n not in port.available_joints:
                    b.setText(f'A{n} unavailable');b.setEnabled(False)
                b.clicked.connect(lambda checked=False,n=n:self.invoke(lambda:self.keys.press(str(n))))
                buttons.addWidget(b)
            arm=QtWidgets.QPushButton('Arm selected (E)');arm.setFocusPolicy(QtCore.Qt.NoFocus)
            arm.clicked.connect(lambda:self.invoke(lambda:self.keys.press('e')));buttons.addWidget(arm);self.arm_button=arm
            self.recover_button=QtWidgets.QPushButton('Recover fault (R)')
            self.recover_button.setFocusPolicy(QtCore.Qt.NoFocus)
            self.recover_button.clicked.connect(self.start_recovery);buttons.addWidget(self.recover_button)
            self.table=table_factory() if table_factory else JointPositionTable()
            table_font=self.table.font();table_font.setPointSize(10);self.table.setFont(table_font)
            self.table.verticalHeader().setMinimumSectionSize(20)
            self.table.verticalHeader().setDefaultSectionSize(22)
            self.table.setMinimumHeight(self.table.horizontalHeader().sizeHint().height()+6*22+2*self.table.frameWidth())
            layout.addWidget(self.table)
            self.status=QtWidgets.QLabel();self.status.setWordWrap(True);layout.addWidget(self.status)
            if panel_factory:panel_factory(self,layout)
            if self.keys.group_hold:
                self.hold_all_button=QtWidgets.QPushButton('HOLD ALL CONNECTED JOINTS (H)')
                self.hold_all_button.setFocusPolicy(QtCore.Qt.NoFocus)
                self.hold_all_button.clicked.connect(lambda:self.invoke(lambda:self.start_motion('h')))
                layout.addWidget(self.hold_all_button)
            stop=QtWidgets.QPushButton('PAUSE / HOLD (Space)' if self.keys.position_hold else 'STOP / DISARM (Space)');stop.setFocusPolicy(QtCore.Qt.NoFocus)
            stop.clicked.connect(lambda:self.invoke(self.keys.stop));layout.addWidget(stop)
            if self.keys.position_hold:
                self.disable_button=QtWidgets.QPushButton('DISABLE TORQUE (D) — support the arm first')
                self.disable_button.setFocusPolicy(QtCore.Qt.NoFocus)
                self.disable_button.clicked.connect(lambda:self.invoke(self.keys.disable));layout.addWidget(self.disable_button)
            support=QtWidgets.QLabel('Powered hold, not a brake. D / close / fault / power loss removes torque from all joints.'
                if self.keys.group_hold else 'Selected-joint hold only. Support other links. D / close / fault / power loss removes torque.'
                if self.keys.position_hold else 'Fixture commissioning; release disables torque. Support all attached links. No gravity holding.')
            support.setWordWrap(True);layout.addWidget(support)
            self.timer=QtCore.QTimer(self);self.timer.setTimerType(QtCore.Qt.PreciseTimer)
            self.timer.timeout.connect(self.update_state);self.timer.start(20)
        def select_next_capture(self):
            remaining=capture.remaining_joints()
            if remaining and port.selected not in remaining and not capture.pending:
                self.invoke(lambda:self.keys.press(str(remaining[0])))
        def place_simulated(self):
            if self.keys.failed or self.keys.recovering or self.keys.down or port.armed or port.moving:return
            row=port.stationary_position()
            degrees,ok=QtWidgets.QInputDialog.getDouble(self,'Simulated hand placement',
                f'A{port.selected} raw shaft angle (degrees)',math.degrees(row.position_rad),
                math.degrees(row.lower_rad),math.degrees(row.upper_rad),3)
            if ok:port.values[port.selected-1]=math.radians(degrees)
        def invoke(self,fn):
            if self.starting:
                self.cancel_start=True
                return
            if self.keys.recovering:return
            try:fn()
            except Exception as exc:self.keys.fail(exc)
            if self.keys.quit:
                self.keys.quit=False;self.close()
        def start_recovery(self):
            if self.starting:
                self.cancel_start=True
                return
            if not self.keys.begin_recovery():return
            # Recovery owns the port; show existing samples as stale until it ends.
            self.table.update_positions(port.cached_positions(),port.selected,stale=True)
            self.recover_button.setEnabled(False)
            self.recovery_thread=RecoveryThread(self)
            self.recovery_thread.finished.connect(self.finish_recovery)
            self.recovery_thread.start()
        def finish_recovery(self):
            self.keys.finish_recovery(self.recovery_thread.error)
            self.recover_button.setEnabled(True);self.next_display=0
            if self.close_after_recovery:self.close()
        def update_state(self):
            if self.starting:return
            self.invoke(self.keys.tick)
            self.title.setText(f'{mode} — A{port.selected} — '+('RECOVERING' if self.keys.recovering else 'FAULT' if self.keys.failed else 'MOVING' if port.moving else
                ('HOLDING — ARMED' if port.armed else 'HOLDING — MOTION DISARMED') if getattr(port,'holding',False) else 'ARMED' if port.armed else 'DISARMED'))
            self.status.setText(self.keys.message)
            if teach:self.pose_label.setText(teach.describe(port.selected))
            if capture:self.recording_panel.refresh()
            self.status.setMinimumHeight(self.status.heightForWidth(max(1,self.status.width())))
            if time.monotonic()>=self.next_display and not self.keys.recovering:
                def refresh():
                    # Fault lockout must still age the table. A frozen enabled
                    # row is not evidence that the motor remains enabled.
                    rows=port.cached_positions() if self.keys.failed or port.moving or getattr(port,'holding',False) else port.positions()
                    self.table.update_positions(rows,port.selected,stale=self.keys.failed)
                self.invoke(refresh);self.next_display=time.monotonic()+.1
        def key_name(self,event):
            return {QtCore.Qt.Key_Left:'left',QtCore.Qt.Key_Right:'right',QtCore.Qt.Key_Up:'up',QtCore.Qt.Key_Down:'down',QtCore.Qt.Key_Escape:'stop',QtCore.Qt.Key_C:'c'}.get(event.key(),event.text().lower())
        def start_motion(self, key):
            # The serial enable handshake is synchronous. Process releases while
            # waiting, without allowing a nested timer or input to use the port.
            previous=getattr(port,'motion_allowed',None)
            def still_held():
                QtWidgets.QApplication.processEvents()
                return not self.cancel_start and (previous is None or previous())
            self.starting=True;self.cancel_start=False
            port.motion_allowed=still_held
            try:
                self.keys.press(key)
            except MotionCancelled:
                self.keys.held.clear()
                self.keys.message='Motion cancelled during enable. Press E to arm again.'
            finally:
                port.motion_allowed=previous;self.starting=False
                if self.close_after_start:QtCore.QTimer.singleShot(0,self.close)
        def keyPressEvent(self,event):
            key=self.key_name(event)
            if self.starting:
                if not event.isAutoRepeat():self.cancel_start=True
            elif key=='r' and not event.isAutoRepeat():self.start_recovery()
            elif self.keys.recovering:self.keys.press(key,event.isAutoRepeat())
            elif (self.keys.is_motion_key(key) or (key=='h' and self.keys.group_hold)) and not event.isAutoRepeat() and isinstance(port,ArmPositionPort):
                self.invoke(lambda:self.start_motion(key))
            else:self.invoke(lambda:self.keys.press(key,event.isAutoRepeat()))
            event.accept()
        def keyReleaseEvent(self,event):
            if not event.isAutoRepeat():
                if self.starting:
                    if self.key_name(event) in self.keys.down:
                        self.keys.down.discard(self.key_name(event));self.cancel_start=True
                elif self.keys.recovering:self.keys.release(self.key_name(event))
                else:self.invoke(lambda:self.keys.release(self.key_name(event)))
            event.accept()
        def changeEvent(self,event):
            if event.type()==QtCore.QEvent.ActivationChange and not self.isActiveWindow():
                self.invoke(self.keys.stop)
            super().changeEvent(event)
        def closeEvent(self,event):
            if self.starting:
                self.cancel_start=True;self.close_after_start=True;event.ignore();return
            if self.keys.recovering:
                self.close_after_recovery=True;event.ignore();return
            self.timer.stop();self.invoke(self.keys.disable);event.accept()
    return Window()


def main(argv=None, *, recording=False):
    parser=argparse.ArgumentParser(description=__doc__)
    action=parser.add_mutually_exclusive_group()
    action.add_argument('--simulate',action='store_true',help='offline window; default')
    action.add_argument('--execute',action='store_true',help='prepare hardware; motion requires keys')
    parser.add_argument('--plan',type=Path)
    parser.add_argument('--port',default='/dev/ttyACM0')
    fixture=parser.add_mutually_exclusive_group()
    fixture.add_argument('--unloaded',action='store_true')
    fixture.add_argument('--supported',action='store_true',help='all links mechanically supported throughout motion and torque disable')
    travel=parser.add_mutually_exclusive_group()
    travel.add_argument('--window-deg',type=float,default=90)
    travel.add_argument('--manual-travel',action='store_true',help='no startup angle window; maximum 12 seconds unless --hold-position is selected')
    parser.add_argument('--speed-deg-s',type=float)
    parser.add_argument('--speed-trial', action='store_true', help='supported manual jog trial up to 10 deg/s')
    parser.add_argument('--hold-position', action='store_true',
                        help='selected-joint hold on release/pause; D disables; requires updated firmware')
    parser.add_argument('--hold-all',action='store_true',
                        help='H activates holding of all connected joints; selection preserves torque; requires group-hold firmware')
    parser.add_argument('--smooth-jog',action='store_true',
                        help='hardware trial: ramp jog targets at 2 ms; requires holding mode and updated firmware')
    parser.add_argument('--teach-file', type=Path, help='save fixture references and tuned raw goal offsets')
    if recording:
        parser.add_argument('--capture-file',type=Path,required=True,
                            help='new JSON output for the separate position recorder')
        parser.add_argument('--capture-mode',choices=('reference','bounds'),default='reference')
        parser.add_argument('--fixture-note',default='')
    else:
        parser.set_defaults(capture_file=None,capture_mode='reference',fixture_note='')
    parser.add_argument('--stop-map', nargs=3, action='append', default=[],
                        metavar=('DOF','MODEL_STOP_DEG','SIGNED_RAW_PER_MODEL'),
                        help='measured stop mapping for A2/A3/A5; used on C')
    parser.add_argument('--report',type=Path)
    args=parser.parse_args(argv)
    if args.hold_all:args.hold_position=True
    if args.capture_file and (not args.hold_all or not args.plan or args.teach_file):
        parser.error('--capture-file requires --hold-all --plan and cannot use --teach-file')
    if args.capture_file and args.report and args.capture_file.resolve()==args.report.resolve():
        parser.error('Use different paths for --capture-file and --report')
    if args.smooth_jog and (not args.execute or not args.hold_position):
        parser.error('--smooth-jog requires --execute and --hold-position or --hold-all')
    if args.hold_position and (not args.supported or not args.manual_travel or args.teach_file or args.speed_trial):
        parser.error('--hold-position requires --supported --manual-travel without teaching or speed trials')
    if args.speed_deg_s is None:
        args.speed_deg_s=HOLD_JOG_DEG_S if args.hold_position else SUPPORTED_JOG_DEG_S if args.supported or args.teach_file else 45
    if args.speed_trial and (not args.supported or not args.manual_travel or args.teach_file):
        parser.error('--speed-trial requires --supported --manual-travel in a separate jog session')
    if args.teach_file and (not args.plan or not args.manual_travel or
                            (args.execute and not args.supported) or args.speed_deg_s>SUPPORTED_JOG_DEG_S):
        parser.error('Teaching requires --plan --manual-travel, speed <=5 deg/s, and supported hardware')
    if args.teach_file and args.report and args.teach_file.resolve() == args.report.resolve():
        parser.error('Use different paths for --teach-file and --report')
    if args.stop_map and not args.teach_file:parser.error('--stop-map requires --teach-file')
    if args.execute and (not (args.unloaded or args.supported) or args.plan is None):
        parser.error('Hardware requires --plan and either --unloaded or --supported')
    if not math.isfinite(args.speed_deg_s) or not 0<args.speed_deg_s<=45:
        parser.error('--speed-deg-s must be >0 and <=45')
    cap=HOLD_JOG_DEG_S if args.hold_position else TRIAL_JOG_DEG_S if args.speed_trial else SUPPORTED_JOG_DEG_S
    if args.supported and ((not args.manual_travel and not 0<args.window_deg<=3) or args.speed_deg_s>cap):
        parser.error(f'--supported requires manual travel or a window <=3°, and speed <={cap} deg/s')
    from PyQt5 import QtWidgets
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])
    if args.execute and args.report is None:
        args.report=Path('build/arm_tests')/f'keyboard-{time.time_ns()}.json'
    port=None;window=None;capture=None;error=None;result=2
    try:
        plan, joints = read_arm_plan(args.plan)
        teach=None
        if args.capture_file:
            from robot_description.paths import model_directory
            capture=ArmCapture(args.capture_file,model_directory()/'initial_pose.json',plan,
                               not args.execute,args.fixture_note,mode=args.capture_mode)
        if args.teach_file:
            from robot_description.paths import model_directory
            model = model_directory() / 'initial_pose.json'
            teach = PoseTeach(args.teach_file, model, plan, not args.execute, args.stop_map)
        if args.execute:
            plan['fixture_confirmed']=True
            plan['fixture_mode']='supported' if args.supported else 'unloaded'
            plan['supported_confirmed']=bool(args.supported)
            port_type=ArmGroupHoldPort if args.hold_all else ArmPositionPort
            hold_options={} if args.hold_all else dict(position_hold=args.hold_position)
            port=port_type(plan,port=args.port,window_degrees=None if args.manual_travel else args.window_deg,
                           allow_faulted_start=True,speed_trial=args.speed_trial,smooth_jog=args.smooth_jog,**hold_options)
        else:
            port_type=SimulatedGroupHoldPort if args.hold_all else SimulatedPositionPort
            hold_options={} if args.hold_all else dict(position_hold=args.hold_position)
            port=port_type(window_degrees=None if args.manual_travel else args.window_deg,
                           available_joints=joints,**hold_options)
        window=make_window(port,'HARDWARE (SUPPORTED)' if args.execute and args.supported else
                           'HARDWARE (UNLOADED)' if args.execute else 'SIMULATION',speed_deg_s=args.speed_deg_s,
                           recovery_report_path=args.report,teach=teach,capture=capture)
        if recording:window.setWindowTitle('FORGE • Position recorder')
        window.show();app.exec_();result=1 if window.keys.failed else 0
    except KeyboardInterrupt:error='Interrupted';result=130
    except Exception as exc:error=str(exc)
    finally:
        # Recovery owns the serial port exclusively until its bounded I/O ends.
        if window is not None and window.recovery_thread is not None:
            window.recovery_thread.wait()
        if port:
            try:port.close()
            except Exception as exc:error=f'Cleanup failed: {exc}';result=2
        if capture:
            message=capture.close()  # Wait for disk only after the controller cleanup.
            if message and 'NOT saved' in message:
                error=error or message;result=2
            if port:port.report['position_capture']=capture.records
        if args.report:
            report=dict(port.report if port else {});report.update(error=error or report.get('error'),exit_code=result)
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(report,indent=2)+'\n')
    if error:print(error,file=sys.stderr)
    return result
