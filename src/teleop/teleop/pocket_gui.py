"""Pocket desktop dashboard; observes one controller and can request STOP."""
import json
import math
import signal
import sys
import time

from PyQt5 import QtCore, QtGui, QtWidgets

from motors.arm_position import JointPosition
from .arm_widgets import JointPositionTable


INK = '#162c3b'
GREEN = '#087e70'
AMBER = '#a65b08'
RED = '#ba3544'


def label(text='', size=11, bold=False):
    item = QtWidgets.QLabel(text)
    item.setTextFormat(QtCore.Qt.PlainText)
    item.setWordWrap(True)
    item.setFont(QtGui.QFont('DejaVu Sans', size, QtGui.QFont.Bold if bold else QtGui.QFont.Normal))
    return item


class StickPlot(QtWidgets.QWidget):
    def __init__(self, arm=False):
        super().__init__()
        self.arm = arm
        self.x = self.y = 0.
        self.active = False
        self.diagonal = False
        self.deadzone = .05
        self.setMinimumSize(200, 180)
        self.setFixedHeight(180)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        center = QtCore.QPointF(self.width()/2, self.height()/2)
        radius = min(self.width(), self.height())/2-28
        box = QtCore.QRectF(center.x()-radius, center.y()-radius, 2*radius, 2*radius)
        painter.setPen(QtGui.QPen(QtGui.QColor('#c7d5db'), 1))
        painter.setBrush(QtGui.QColor('#f2f6f7'))
        painter.drawRoundedRect(box, 9, 9)
        painter.drawLine(QtCore.QPointF(box.left(),center.y()), QtCore.QPointF(box.right(),center.y()))
        painter.drawLine(QtCore.QPointF(center.x(),box.top()), QtCore.QPointF(center.x(),box.bottom()))
        painter.setBrush(QtGui.QColor('#bddbd3'))
        size = max(3., self.deadzone*radius)
        painter.drawRect(QtCore.QRectF(center.x()-size,center.y()-size,2*size,2*size))
        painter.setPen(QtGui.QColor(INK))
        painter.setFont(QtGui.QFont('DejaVu Sans', 9))
        for text, rect in (
            ('CW' if self.arm else 'Forward', QtCore.QRectF(0,0,self.width(),22)),
            ('CCW' if self.arm else 'Reverse', QtCore.QRectF(0,self.height()-22,self.width(),22)),
            ('CW' if self.arm else 'L', QtCore.QRectF(0,center.y()-12,30,24)),
            ('CCW' if self.arm else 'R', QtCore.QRectF(self.width()-33,center.y()-12,33,24))):
            painter.drawText(rect, QtCore.Qt.AlignCenter, text)
        dot = QtCore.QPointF(center.x()+self.x*radius, center.y()-self.y*radius)
        color = AMBER if self.diagonal else GREEN if self.active else '#87969f'
        painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
        painter.drawLine(center, dot)
        painter.setBrush(QtGui.QColor(color))
        painter.drawEllipse(dot, 7, 7)


class MotorDial(QtWidgets.QWidget):
    def __init__(self, joint):
        super().__init__()
        self.joint = joint
        self.angle = None
        self.selected = self.moving = self.stale = False
        self.setMinimumSize(112, 132)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        color = '#87969f' if self.stale else GREEN if self.selected else '#8da0aa'
        center = QtCore.QPointF(self.width()/2, 65)
        painter.setPen(QtGui.QColor(INK))
        painter.setFont(QtGui.QFont('DejaVu Sans', 11, QtGui.QFont.Bold))
        painter.drawText(QtCore.QRectF(0,2,self.width(),24), QtCore.Qt.AlignCenter,
                         f'A{self.joint}' + (' • selected' if self.selected else ''))
        painter.setPen(QtGui.QPen(QtGui.QColor(color), 3 if self.selected else 1))
        painter.setBrush(QtGui.QColor('#e4f3ef' if self.selected else '#f2f6f7'))
        painter.drawEllipse(center, 31, 31)
        if self.angle is not None:
            theta = math.radians(self.angle)
            end = QtCore.QPointF(center.x()+25*math.sin(theta), center.y()-25*math.cos(theta))
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 4))
            painter.drawLine(center,end)
        painter.setPen(QtGui.QColor(INK))
        painter.drawText(QtCore.QRectF(0,101,self.width(),22), QtCore.Qt.AlignCenter,
                         '—' if self.angle is None else f'{self.angle:+.1f}°')


class PocketWindow(QtWidgets.QWidget):
    def __init__(self, stop_callback=lambda: None, clock=time.monotonic):
        super().__init__()
        self.stop_callback, self.clock = stop_callback, clock
        self.snapshot = None
        self.received_at = None
        self.setWindowTitle('FORGE • Pocket controls')
        self.resize(1040, 850)
        self.setStyleSheet('''
            QWidget { background: #f4f7f8; color: #162c3b; }
            QLabel { background: transparent; }
            QGroupBox { background: white; border: 1px solid #d5e0e5;
                        border-radius: 9px; margin-top: 12px; padding: 12px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 5px; }
            QTableWidget { background: white; gridline-color: #e4ebef; border: none; }
            QHeaderView::section { background: #edf3f5; padding: 6px; border: none; }
            QPushButton { background: #ba3544; color: white; border: none;
                          padding: 12px; border-radius: 7px; font-weight: bold; }
            QPushButton:hover { background: #9b2937; }
        ''')
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(20,16,20,16)
        heading = QtWidgets.QHBoxLayout()
        heading.addWidget(label('Pocket control desk', 22, True))
        self.connection = label('Waiting for controller…', 11, True)
        self.connection.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        heading.addWidget(self.connection)
        outer.addLayout(heading)
        self.status = label('Start the Pocket launch to connect.', 13, True)
        self.status.setMinimumHeight(46)
        outer.addWidget(self.status)

        controls = QtWidgets.QHBoxLayout()
        self.plots, self.axis_labels = {}, {}
        for key, title, arm in (('left','Left stick • chassis',False),('right','Right stick • arm',True)):
            box = QtWidgets.QGroupBox(title)
            layout = QtWidgets.QVBoxLayout(box)
            self.plots[key] = StickPlot(arm)
            layout.addWidget(self.plots[key])
            self.axis_labels[key] = label('Waiting for raw channels…', 10)
            self.axis_labels[key].setMinimumHeight(38)
            layout.addWidget(self.axis_labels[key])
            controls.addWidget(box, 1)
        box = QtWidgets.QGroupBox('Selection & primer')
        panel = QtWidgets.QVBoxLayout(box)
        self.switch_labels = {}
        for name in ('sb','sc','se'):
            self.switch_labels[name] = label(name.upper()+' —', 12, True)
            panel.addWidget(self.switch_labels[name])
        self.chassis = label('Chassis command: —', 11)
        panel.addWidget(self.chassis)
        self.arm_state = label('Arm feedback: —', 10)
        self.chassis.setMinimumHeight(66)
        self.arm_state.setMinimumHeight(36)
        panel.addWidget(self.arm_state)
        controls.addWidget(box, 1)
        outer.addLayout(controls)

        self.joint_caption = label('Arm shaft positions • relative to session start', 12, True)
        outer.addWidget(self.joint_caption)
        dials = QtWidgets.QHBoxLayout()
        self.dials = [MotorDial(joint) for joint in range(1,7)]
        for dial in self.dials: dials.addWidget(dial)
        outer.addLayout(dials)
        self.table = JointPositionTable()
        self.table.setMinimumHeight(212)
        outer.addWidget(self.table, 1)
        self.raw_channels = label('Raw receiver channels: —', 9)
        outer.addWidget(self.raw_channels)
        stop = QtWidgets.QPushButton('STOP CHASSIS + ARM  •  Space')
        stop.setFocusPolicy(QtCore.Qt.NoFocus)
        stop.clicked.connect(self.request_stop)
        outer.addWidget(stop)
        outer.addWidget(label('Select with SB / SC → center right stick with SE released → hold SE → move right stick. '
                              'Closing this window ends the launched session.', 10))
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)

    def accept_snapshot(self, snapshot):
        if snapshot.get('version') != 1:
            return
        self.snapshot = snapshot
        self.received_at = self.clock()
        self.refresh()

    def refresh(self):
        if self.snapshot is None:
            return
        data = self.snapshot
        elapsed = self.clock()-self.received_at
        stale = elapsed > .35
        fresh = data['radio_fresh'] and not stale
        mode = 'HARDWARE ARM' if data['hardware_arm'] else 'SIMULATED ARM'
        self.connection.setText(mode+' • '+('LIVE' if fresh else 'INPUT STALE'))
        self.connection.setStyleSheet('color: '+(GREEN if fresh else RED))
        self.status.setText('Controller telemetry stopped — showing last readings.' if stale else
                            data['status'] if fresh else 'Radio input expired — motion commands stopped.')
        diagonal = data.get('diagonal', False) and fresh
        self.status.setStyleSheet('color: '+(RED if not fresh else AMBER if diagonal else
                                            GREEN if data['authorized'] else AMBER))
        axes = data['axes']
        for key, names in (('left',('chassis_turn','chassis_forward')),
                           ('right',('arm_cw_horizontal','arm_cw_vertical'))):
            horizontal,vertical = (axes.get(name) for name in names)
            plot = self.plots[key]
            plot.active, plot.diagonal = fresh, diagonal and key == 'right'
            if horizontal and vertical:
                plot.x,plot.y = -horizontal['offset'],vertical['offset']
                plot.deadzone = max(horizontal['deadzone'],vertical['deadzone'])
                self.axis_labels[key].setText('\n'.join(
                    f'CH{axis["channel"]}: {axis["raw"]}   center {axis["center"]}   '
                    f'{"NEUTRAL" if axis["value"] == 0 else "ACTIVE"}'
                    for axis in (horizontal,vertical)))
            else:
                plot.x=plot.y=0.
                self.axis_labels[key].setText('No fresh receiver sample')
            plot.update()
        for name,item in self.switch_labels.items():
            switch = data['switches'][name]
            position = switch['position']
            value = '—' if switch['raw'] is None else str(switch['raw'])
            text = ('RELEASED','HELD')[position] if name == 'se' and position is not None else (
                ('TOP','MIDDLE','BOTTOM')[position] if position is not None else 'UNKNOWN')
            item.setText(f'{name.upper()}  {text}\nCH{switch["channel"]}: {value}')
            item.setStyleSheet('color: '+(GREEN if fresh and name == 'se' and position == 1 else INK))
        forward,yaw = data['chassis'] if fresh else (0.,0.)
        self.chassis.setText(f'Chassis command\nForward {forward:+.3f} m/s\nTurn {yaw:+.3f} rad/s')
        arm = data['arm']
        age = arm.get('sample_age_ms')
        arm_stale = stale or age is None or age+elapsed*1000 > 350
        extra_age = max(0, round((age or 0)+elapsed*1000))
        rows = [JointPosition(**{**row, 'age_ms': row['age_ms']+extra_age})
                for row in arm.get('rows',[])]
        self.table.update_positions(rows, data['selected'], arm_stale)
        by_joint = {row.joint: row for row in rows}
        for dial in self.dials:
            row = by_joint.get(dial.joint)
            dial.angle = row.relative_degrees if row else None
            dial.selected = dial.joint == data['selected']
            dial.stale = arm_stale
            dial.update()
        motion = 'MOVING' if arm.get('moving') and not arm_stale else 'STALE' if arm_stale else 'STOPPED'
        self.arm_state.setText('Arm feedback: '+motion+ ('\n'+str(arm['error']) if arm.get('error') else ''))
        self.joint_caption.setText('Arm shaft positions • '+('HARDWARE / cached' if data['hardware_arm'] else
            'SIMULATED / CW sign not yet measured')+' • relative to session start')
        channels = data.get('channels')
        self.raw_channels.setText('   '.join(f'CH{i} {raw}' for i,raw in enumerate(channels,1))
                                  if channels else 'Raw receiver channels: no fresh sample')

    def request_stop(self):
        self.stop_callback()
        self.status.setText('STOP requested — center both sticks, release SE and reselect a motor.')

    def keyPressEvent(self, event):
        if event.key() in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape) and not event.isAutoRepeat():
            self.request_stop()
        event.accept()

    def closeEvent(self, event):
        self.timer.stop()
        self.stop_callback()
        event.accept()


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String, Empty
    rclpy.init(args=args)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])
    node = rclpy.create_node('pocket_gui')
    stop = node.create_publisher(Empty, '/pocket/stop', 1)
    window = PocketWindow(lambda: stop.publish(Empty()) if rclpy.ok() else None)

    def receive(message):
        try:
            window.accept_snapshot(json.loads(message.data))
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            node.get_logger().warning(f'Invalid Pocket dashboard sample: {exc}')

    node.create_subscription(String, '/pocket/telemetry', receive, 1)
    timer = QtCore.QTimer()
    def spin():
        if not rclpy.ok():
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.)
        except ExternalShutdownException:
            app.quit()
    timer.timeout.connect(spin)
    timer.start(20)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    try:
        app.exec_()
    finally:
        timer.stop()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
