"""Pocket dashboard diagnostics, cached readouts and actual Qt interactions."""
import os
import unittest
from dataclasses import asdict
from types import SimpleNamespace
from unittest.mock import Mock, patch

import source_paths
from motors.arm_position import ArmPositionPort, JointPosition, SimulatedPositionPort
from teleop.pocket_display import dashboard_snapshot, position_snapshot
from teleop.pocket_input import PocketCommand, PocketMapping
from test_pocket_controls import config, row


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.policy = PocketCommand(PocketMapping(config()))
        self.now = 1.

    def send(self, **values):
        self.now += .1
        self.policy.receive(row(**values),self.now)

    def test_conflict_exposes_both_axes_and_freshness(self):
        self.send()
        self.send(sb=992, x=1811, y=1811)
        data = dashboard_snapshot(self.policy,{},self.now)
        self.assertIn('conflicting',data['status'])
        self.assertEqual(data['axes']['arm_cw_horizontal']['raw'],1811)
        self.assertEqual(data['axes']['arm_cw_horizontal']['value'],-1)
        self.assertEqual(data['axes']['arm_cw_vertical']['value'],1)
        self.assertEqual(data['direction'],0)
        expired = dashboard_snapshot(self.policy,{},self.now+.2)
        self.assertFalse(expired['radio_fresh'])
        self.assertIsNone(expired['channels'])
        self.assertEqual(expired['chassis'],[0.,0.])

    def test_center_and_se_messages_distinguish_blockers(self):
        self.send();self.send(sb=992,x=1200)
        self.assertIn('CH1=1200',self.policy.status)
        for _ in range(6):self.send(sb=992)
        self.assertIn('ready to prime',self.policy.status)
        self.send(sb=992,se=1811)
        self.assertIn('ready, SE held',self.policy.status)
        self.send(sb=992,se=1811,y=1811)
        self.assertTrue(self.policy.status.endswith(': CW'))

    def test_hardware_display_ages_existing_samples_without_transport(self):
        port = object.__new__(ArmPositionPort)
        port.bus = Mock()
        port.session = SimpleNamespace(origins={0:1.})
        port.window = .5
        port._display_rows = {}
        port.selected=1;port.armed=True;port.moving=True
        sample = SimpleNamespace(position=1.2,age_ms=4,enabled=True)
        with patch('motors.arm_position.time.monotonic',return_value=10.):
            port._remember_sample(0,sample)
        with patch('motors.arm_position.time.monotonic',return_value=10.1):
            result = position_snapshot(port,10.1)
        self.assertEqual(result['rows'][0]['age_ms'],104)
        self.assertEqual(result['rows'][0]['position_rad'],1.2)
        self.assertEqual(port.bus.mock_calls,[])

    def test_qt_live_angles_raw_channels_staleness_and_stop(self):
        os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
        from PyQt5 import QtCore, QtWidgets, QtTest
        from teleop.pocket_gui import PocketWindow
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        now=[10.]
        stops=[]
        window=PocketWindow(lambda:stops.append(True),clock=lambda:now[0])
        self.send();self.send(sb=992,x=1811,y=1811)
        position=JointPosition(2,.5,0.,-1.5,1.5,0,True)
        worker=dict(rows=[asdict(position)],sample_age_ms=0,moving=False)
        data=dashboard_snapshot(self.policy,worker,self.now)
        window.accept_snapshot(data)
        window.show();app.processEvents()
        self.assertIn('conflicting',window.status.text())
        self.assertIn('CH1: 1811',window.axis_labels['right'].text())
        self.assertIn('ACTIVE',window.axis_labels['right'].text())
        self.assertTrue(window.dials[1].selected)
        self.assertAlmostEqual(window.dials[1].angle,28.6478898)
        self.assertEqual(window.table.item(1,1).text(),'28.65')
        QtTest.QTest.keyClick(window,QtCore.Qt.Key_Space)
        self.assertEqual(len(stops),1)
        now[0]+=.5;window.refresh()
        self.assertIn('telemetry stopped',window.status.text())
        self.assertIn('STALE',window.connection.text())
        self.assertIn('STALE',window.arm_state.text())
        self.assertEqual(window.table.item(1,3).text(),'STALE')
        self.assertEqual(window.table.item(1,4).text(),'500')
        window.close()
        self.assertEqual(len(stops),2)


if __name__ == '__main__':unittest.main()
