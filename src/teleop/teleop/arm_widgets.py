"""Joint readout shared by keyboard jogging and the Pocket dashboard."""
import math

from PyQt5 import QtCore, QtGui, QtWidgets


class JointPositionTable(QtWidgets.QTableWidget):
    def __init__(self):
        super().__init__(6, 8)
        self.setHorizontalHeaderLabels(['Joint', 'Relative °', 'Raw °', 'Enabled', 'Age ms',
                                        'Speed rad/s', 'Feedback Nm', 'Trip Nm'])
        self.horizontalHeaderItem(6).setToolTip('Motor-reported torque estimate; not a torque command or PSU current.')
        self.horizontalHeaderItem(7).setToolTip('Reactive shutdown threshold; not a commanded torque or current clamp.')
        self.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.setFocusPolicy(QtCore.Qt.NoFocus)
        self.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.verticalHeader().hide()
        self.verticalHeader().setDefaultSectionSize(26)
        self.update_positions([])

    def update_positions(self, rows, selected=None, stale=False):
        by_joint = {row.joint: row for row in rows}
        for joint in range(1, 7):
            row = by_joint.get(joint)
            fresh = bool(row and not stale and row.age_ms < 80)
            def feedback(value):
                return f'{value:+.3f}' if fresh and value is not None else '—'
            values = ([f'A{joint}', f'{row.relative_degrees:.2f}',
                       f'{math.degrees(row.position_rad):.2f}',
                       'STALE' if not fresh else 'Yes' if row.enabled else 'No', str(row.age_ms),
                       feedback(row.velocity_rad_s), feedback(row.torque_feedback_nm),
                       f'{row.torque_trip_nm:.2f}' if row.torque_trip_nm is not None else '—']
                      if row else [f'A{joint}'] + ['—']*7)
            for column, value in enumerate(values):
                item = self.item(joint-1, column)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    item.setTextAlignment(QtCore.Qt.AlignCenter)
                    self.setItem(joint-1, column, item)
                item.setText(value)
                item.setBackground(QtGui.QColor('#dceeea' if joint == selected else '#ffffff'))
                item.setForeground(QtGui.QColor('#6b7785' if stale else '#162c3b'))
