"""Desktop, hold-to-jog arm control; independent of ENCOS WASD."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
from motors.arm_position import ArmPositionPort, SimulatedPositionPort


class ArmJogKeys:
    """Press/release policy, independent of Qt and hardware transport."""
    directions={'right':1,'up':1,'left':-1,'down':-1}
    def __init__(self,port):
        self.port=port;self.held=set();self.failed=False;self.quit=False
        self.message='Select A1–A6, press E once, then hold an arrow.'
        self.last_tick=time.monotonic()
    def stop(self):
        self.held.clear();self.port.stop();self.message='Stopped and disarmed. E arms again.'
    def press(self,key,repeat=False):
        if repeat:return
        if key=='q':self.stop();self.quit=True;return
        if key in ('stop',' ','x'):self.stop();return
        if self.failed:return
        if key in tuple('123456'):
            if int(key)!=self.port.selected:
                self.held.clear();self.port.select(int(key))
                self.message=f'A{key} selected. Press E to arm.'
            return
        if key=='e':
            if not self.held and not self.port.armed:
                self.port.arm();self.message='Armed. Hold an arrow; release to stop.'
            return
        if key not in self.directions or key in self.held:return
        self.held.add(key)
        if len(self.held)>1:
            self.port.release();self.message='Release all arrows before changing direction.';return
        if not self.port.armed:
            self.message='Release arrow, then press E to arm.';return
        row=self.port.positions()[self.port.selected-1]
        target=row.upper_rad if self.directions[key]>0 else row.lower_rad
        self.port.move_to(target,math.radians(45))
        self.last_tick=time.monotonic()
        self.message='Moving while held — release to stop; ±90° maximum session window.'
    def release(self,key):
        if key in self.held:
            self.held.remove(key);self.port.release()
            self.message='Stopped; still armed. Hold an arrow to move again.'
    def tick(self):
        now=time.monotonic()
        if self.port.moving and now-self.last_tick>.15:
            self.stop();raise RuntimeError('UI stalled; motion stopped. Inspect controller before recovery.')
        self.last_tick=now
        if not self.failed and self.port.moving:
            self.port.poll()
            if not self.port.moving:self.message='Travel boundary reached. Release and reverse direction.'


def make_window(port,mode):
    from PyQt5 import QtCore,QtWidgets
    from .arm_widgets import JointPositionTable
    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__();self.keys=ArmJogKeys(port);self.next_display=0
            self.setWindowTitle('FORGE • Arm joint jog • '+mode);self.resize(760,440)
            layout=QtWidgets.QVBoxLayout(self)
            self.title=QtWidgets.QLabel();layout.addWidget(self.title)
            layout.addWidget(QtWidgets.QLabel('1–6 select   •   E arm once   •   Hold ↑/→ positive, ↓/← negative   •   Space stop'))
            layout.addWidget(QtWidgets.QLabel('45°/s • Release stops • Focus loss disarms • Angles relative to session start'))
            buttons=QtWidgets.QHBoxLayout();layout.addLayout(buttons)
            for n in range(1,7):
                b=QtWidgets.QPushButton(f'A{n}');b.setFocusPolicy(QtCore.Qt.NoFocus)
                b.clicked.connect(lambda checked=False,n=n:self.invoke(lambda:self.keys.press(str(n))))
                buttons.addWidget(b)
            arm=QtWidgets.QPushButton('Arm selected (E)');arm.setFocusPolicy(QtCore.Qt.NoFocus)
            arm.clicked.connect(lambda:self.invoke(lambda:self.keys.press('e')));buttons.addWidget(arm)
            self.table=JointPositionTable()
            layout.addWidget(self.table)
            self.status=QtWidgets.QLabel();self.status.setWordWrap(True);layout.addWidget(self.status)
            stop=QtWidgets.QPushButton('STOP / DISARM (Space)');stop.setFocusPolicy(QtCore.Qt.NoFocus)
            stop.clicked.connect(lambda:self.invoke(self.keys.stop));layout.addWidget(stop)
            layout.addWidget(QtWidgets.QLabel('Unloaded commissioning only; disable removes torque. No gravity holding.'))
            self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.update_state);self.timer.start(20)
        def invoke(self,fn):
            try:fn()
            except Exception as exc:
                self.keys.failed=True;port.report['error']=str(exc)
                try:self.keys.stop()
                except Exception as stop_exc:port.report['stop_error']=str(stop_exc)
                self.keys.message=f'FAULT: {exc}. Close and recover; no automatic re-enable.'
            if self.keys.quit:
                self.keys.quit=False;self.close()
        def update_state(self):
            self.invoke(self.keys.tick)
            self.title.setText(f'{mode} — A{port.selected} — '+('MOVING' if port.moving else 'ARMED' if port.armed else 'DISARMED'))
            self.status.setText(self.keys.message)
            if time.monotonic()>=self.next_display and not self.keys.failed:
                def refresh():
                    self.table.update_positions(port.positions(), port.selected)
                self.invoke(refresh);self.next_display=time.monotonic()+.1
        def key_name(self,event):
            return {QtCore.Qt.Key_Left:'left',QtCore.Qt.Key_Right:'right',QtCore.Qt.Key_Up:'up',QtCore.Qt.Key_Down:'down',QtCore.Qt.Key_Escape:'stop'}.get(event.key(),event.text().lower())
        def keyPressEvent(self,event):
            self.invoke(lambda:self.keys.press(self.key_name(event),event.isAutoRepeat()));event.accept()
        def keyReleaseEvent(self,event):
            if not event.isAutoRepeat():self.invoke(lambda:self.keys.release(self.key_name(event)))
            event.accept()
        def changeEvent(self,event):
            if event.type()==QtCore.QEvent.ActivationChange and not self.isActiveWindow():
                self.invoke(self.keys.stop)
            super().changeEvent(event)
        def closeEvent(self,event):
            self.timer.stop();self.invoke(self.keys.stop);event.accept()
    return Window()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    action=parser.add_mutually_exclusive_group()
    action.add_argument('--simulate',action='store_true',help='offline window; default')
    action.add_argument('--execute',action='store_true',help='prepare hardware; motion requires keys')
    parser.add_argument('--plan',type=Path)
    parser.add_argument('--port',default='/dev/ttyACM0')
    parser.add_argument('--unloaded',action='store_true')
    parser.add_argument('--window-deg',type=float,default=90)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args(argv)
    if args.execute and (not args.unloaded or args.plan is None):parser.error('Hardware requires --plan and --unloaded')
    from PyQt5 import QtWidgets
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])
    port=None;error=None;result=2
    try:
        if args.execute:
            plan=json.loads(args.plan.read_text());plan['fixture_confirmed']=True
            port=ArmPositionPort(plan,port=args.port,window_degrees=args.window_deg)
        else:port=SimulatedPositionPort(window_degrees=args.window_deg)
        window=make_window(port,'HARDWARE' if args.execute else 'SIMULATION')
        window.show();app.exec_();result=1 if window.keys.failed else 0
    except KeyboardInterrupt:error='Interrupted';result=130
    except Exception as exc:error=str(exc)
    finally:
        if port:
            try:port.close()
            except Exception as exc:error=f'Cleanup failed: {exc}';result=2
        if args.report:
            report=dict(port.report if port else {});report.update(error=error or report.get('error'),exit_code=result)
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(report,indent=2)+'\n')
    if error:print(error,file=sys.stderr)
    return result
