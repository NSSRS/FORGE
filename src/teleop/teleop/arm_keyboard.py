"""Desktop, hold-to-jog arm control; independent of ENCOS WASD."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
from motors.arm_position import ArmPositionPort, SimulatedPositionPort
from motors.arm_bench import active_joints, validate_plan


class ArmJogKeys:
    """Press/release policy, independent of Qt and hardware transport."""
    directions={'right':1,'up':1,'left':-1,'down':-1}
    def __init__(self,port,speed_deg_s=45,*,recovery_report_path=None):
        if not math.isfinite(speed_deg_s) or not 0<speed_deg_s<=45:
            raise ValueError('Jog speed must be >0 and <=45 degrees/s')
        self.speed=math.radians(speed_deg_s)
        self.port=port;self.held=set();self.down=set();self.quit=False
        self.recovering=False;self.recovery_report_path=recovery_report_path
        self.failed=bool(getattr(port,'recovery_required',False))
        self.message=(f"FAULT: {port.report.get('error','Recovery required')}. Check the cause, then press R."
                      if self.failed else 'Select A1–A6, press E once, then hold an arrow.')
        self.last_tick=time.monotonic()
    def stop(self):
        self.held.clear();self.port.stop()
        if not self.failed:self.message='Stopped and disarmed. E arms again.'
    def press(self,key,repeat=False):
        if repeat:return
        if key in self.directions:self.down.add(key)
        if self.recovering:return
        if key=='r':self.recover();return
        if key=='q':self.stop();self.quit=True;return
        if key in ('stop',' ','x'):self.stop();return
        if self.failed:return
        if key in tuple('123456'):
            if int(key) not in self.port.available_joints:
                self.stop();self.message=f'A{key} unavailable. Select a connected motor.';return
            if int(key)!=self.port.selected:
                self.held.clear();self.port.select(int(key))
                self.message=f'A{key} selected. Press E to arm.'
            return
        if key=='e':
            if not self.down and not self.port.armed:
                self.port.arm();self.message='Armed. Hold an arrow; release to stop.'
            return
        if key not in self.directions or key in self.held:return
        self.held.add(key)
        if len(self.held)>1:
            self.port.release();self.message='Release all arrows before changing direction.';return
        if not self.port.armed:
            self.message='Release arrow, then press E to arm.';return
        row=next(row for row in self.port.positions() if row.joint==self.port.selected)
        target=row.upper_rad if self.directions[key]>0 else row.lower_rad
        if self.port.window is None:
            self.port.jog(self.directions[key],self.speed)
        else:
            self.port.move_to(target,self.speed)
        self.last_tick=time.monotonic()
        self.message=('Manual travel — release to stop; 12 s maximum per hold.' if self.port.window is None
                      else f'Moving while held — release to stop; ±{math.degrees(self.port.window):g}° session window.')
    def release(self,key):
        self.down.discard(key)
        if self.recovering:return
        if key in self.held:
            self.held.remove(key);self.port.release()
            self.message='Stopped; still armed. Hold an arrow to move again.'
    def begin_recovery(self):
        if self.recovering:return False
        if self.down or self.port.moving:
            self.message='Release all arrows before recovery.'
            return False
        self.held.clear();self.failed=True;self.recovering=True
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
        if self.port.moving and now-self.last_tick>.15:
            self.stop();raise RuntimeError('UI stalled; motion stopped. Inspect controller before recovery.')
        self.last_tick=now
        if not self.failed and self.port.moving:
            self.port.poll()
            if not self.port.moving:self.message='Travel boundary reached. Release and reverse direction.'


def make_window(port,mode,*,speed_deg_s=45,recovery_report_path=None):
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
            super().__init__();self.keys=ArmJogKeys(port,speed_deg_s,recovery_report_path=recovery_report_path);self.next_display=0
            self.recovery_thread=None;self.close_after_recovery=False
            self.setWindowTitle('FORGE • Arm joint jog • '+mode);self.resize(880,420)
            layout=QtWidgets.QVBoxLayout(self);layout.setContentsMargins(8,8,8,8);layout.setSpacing(4)
            self.title=QtWidgets.QLabel();layout.addWidget(self.title)
            shortcuts=QtWidgets.QLabel('1–6 select   •   E arm   •   Hold ↑/→ positive, ↓/← negative   •   Space stop   •   R recover')
            shortcuts.setWordWrap(True);layout.addWidget(shortcuts)
            travel='Manual travel • 12 s maximum per hold' if port.window is None else f'±{math.degrees(port.window):g}° window'
            layout.addWidget(QtWidgets.QLabel(f'{speed_deg_s:g}°/s • {travel} • Release stops • Focus loss disarms'))
            buttons=QtWidgets.QHBoxLayout();layout.addLayout(buttons)
            for n in range(1,7):
                b=QtWidgets.QPushButton(f'A{n}');b.setFocusPolicy(QtCore.Qt.NoFocus)
                if n not in port.available_joints:
                    b.setText(f'A{n} unavailable');b.setEnabled(False)
                b.clicked.connect(lambda checked=False,n=n:self.invoke(lambda:self.keys.press(str(n))))
                buttons.addWidget(b)
            arm=QtWidgets.QPushButton('Arm selected (E)');arm.setFocusPolicy(QtCore.Qt.NoFocus)
            arm.clicked.connect(lambda:self.invoke(lambda:self.keys.press('e')));buttons.addWidget(arm)
            self.recover_button=QtWidgets.QPushButton('Recover fault (R)')
            self.recover_button.setFocusPolicy(QtCore.Qt.NoFocus)
            self.recover_button.clicked.connect(self.start_recovery);buttons.addWidget(self.recover_button)
            self.table=JointPositionTable()
            table_font=self.table.font();table_font.setPointSize(10);self.table.setFont(table_font)
            self.table.verticalHeader().setMinimumSectionSize(20)
            self.table.verticalHeader().setDefaultSectionSize(22)
            self.table.setMinimumHeight(self.table.horizontalHeader().sizeHint().height()+6*22+2*self.table.frameWidth())
            layout.addWidget(self.table)
            self.status=QtWidgets.QLabel();self.status.setWordWrap(True);layout.addWidget(self.status)
            stop=QtWidgets.QPushButton('STOP / DISARM (Space)');stop.setFocusPolicy(QtCore.Qt.NoFocus)
            stop.clicked.connect(lambda:self.invoke(self.keys.stop));layout.addWidget(stop)
            support=QtWidgets.QLabel('Fixture commissioning; release disables torque. Support all attached links. No gravity holding.')
            support.setWordWrap(True);layout.addWidget(support)
            self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.update_state);self.timer.start(20)
        def invoke(self,fn):
            if self.keys.recovering:return
            try:fn()
            except Exception as exc:self.keys.fail(exc)
            if self.keys.quit:
                self.keys.quit=False;self.close()
        def start_recovery(self):
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
            self.invoke(self.keys.tick)
            self.title.setText(f'{mode} — A{port.selected} — '+('RECOVERING' if self.keys.recovering else 'FAULT' if self.keys.failed else 'MOVING' if port.moving else 'ARMED' if port.armed else 'DISARMED'))
            self.status.setText(self.keys.message)
            self.status.setMinimumHeight(self.status.heightForWidth(max(1,self.status.width())))
            if time.monotonic()>=self.next_display and not self.keys.recovering:
                def refresh():
                    # Fault lockout must still age the table. A frozen enabled
                    # row is not evidence that the motor remains enabled.
                    rows=port.cached_positions() if self.keys.failed else port.positions()
                    self.table.update_positions(rows,port.selected,stale=self.keys.failed)
                self.invoke(refresh);self.next_display=time.monotonic()+.1
        def key_name(self,event):
            return {QtCore.Qt.Key_Left:'left',QtCore.Qt.Key_Right:'right',QtCore.Qt.Key_Up:'up',QtCore.Qt.Key_Down:'down',QtCore.Qt.Key_Escape:'stop'}.get(event.key(),event.text().lower())
        def keyPressEvent(self,event):
            key=self.key_name(event)
            if key=='r' and not event.isAutoRepeat():self.start_recovery()
            elif self.keys.recovering:self.keys.press(key,event.isAutoRepeat())
            else:self.invoke(lambda:self.keys.press(key,event.isAutoRepeat()))
            event.accept()
        def keyReleaseEvent(self,event):
            if not event.isAutoRepeat():
                if self.keys.recovering:self.keys.release(self.key_name(event))
                else:self.invoke(lambda:self.keys.release(self.key_name(event)))
            event.accept()
        def changeEvent(self,event):
            if event.type()==QtCore.QEvent.ActivationChange and not self.isActiveWindow():
                self.invoke(self.keys.stop)
            super().changeEvent(event)
        def closeEvent(self,event):
            if self.keys.recovering:
                self.close_after_recovery=True;event.ignore();return
            self.timer.stop();self.invoke(self.keys.stop);event.accept()
    return Window()


def main(argv=None):
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
    travel.add_argument('--manual-travel',action='store_true',help='no startup angle window; held input advances target, maximum 12 seconds')
    parser.add_argument('--speed-deg-s',type=float,default=45)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args(argv)
    if args.execute and (not (args.unloaded or args.supported) or args.plan is None):
        parser.error('Hardware requires --plan and either --unloaded or --supported')
    if not math.isfinite(args.speed_deg_s) or not 0<args.speed_deg_s<=45:
        parser.error('--speed-deg-s must be >0 and <=45')
    if args.supported and ((not args.manual_travel and not 0<args.window_deg<=3) or args.speed_deg_s>5):
        parser.error('--supported requires --manual-travel or --window-deg <=3, and --speed-deg-s <=5')
    from PyQt5 import QtWidgets
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])
    if args.execute and args.report is None:
        args.report=Path('build/arm_tests')/f'keyboard-{time.time_ns()}.json'
    port=None;window=None;error=None;result=2
    try:
        plan=json.loads(args.plan.read_text()) if args.plan else None
        if plan:validate_plan(plan,all_motors=True)
        joints=active_joints(plan) if plan else tuple(range(1,7))
        if args.execute:
            plan['fixture_confirmed']=True
            plan['fixture_mode']='supported' if args.supported else 'unloaded'
            plan['supported_confirmed']=bool(args.supported)
            port=ArmPositionPort(plan,port=args.port,window_degrees=None if args.manual_travel else args.window_deg,
                                 allow_faulted_start=True)
        else:port=SimulatedPositionPort(window_degrees=None if args.manual_travel else args.window_deg,available_joints=joints)
        window=make_window(port,'HARDWARE (SUPPORTED)' if args.execute and args.supported else
                           'HARDWARE (UNLOADED)' if args.execute else 'SIMULATION',speed_deg_s=args.speed_deg_s,
                           recovery_report_path=args.report)
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
        if args.report:
            report=dict(port.report if port else {});report.update(error=error or report.get('error'),exit_code=result)
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(report,indent=2)+'\n')
    if error:print(error,file=sys.stderr)
    return result
