import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from dataclasses import replace
from unittest.mock import patch

import source_paths
from motors.arm_pose_teach import PoseTeach
from motors.arm_position import SimulatedPositionPort
from teleop.arm_keyboard import ArmJogKeys, main


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'src/robot_description/config'
MODEL = ROOT/'src/robot_description/mujoco/initial_pose.json'


class TeachTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name)/'test.json'
        self.plan = json.loads((CONFIG/'arm_rig_a2_a6.json').read_text())
        self.port = SimulatedPositionPort(window_degrees=None, available_joints=(2,3,4,5,6))
        self.teach = PoseTeach(self.path, MODEL, self.plan, True, [])
        self.keys = ArmJogKeys(self.port, 5, teach=self.teach)

    def test_profiles_match_physical_slots(self):
        full = json.loads((CONFIG/'arm_rig_a1_a6.json').read_text())
        self.assertEqual(full['motors'], self.plan['motors'])
        self.assertEqual(full['active_joints'], list(range(1,7)))
        self.assertFalse(full['fixture_confirmed'] or full['supported_confirmed'])

    def test_stop_rehearsal_and_trim_save_then_rebase(self):
        k,p=self.keys,self.port
        k.press('c')
        self.assertFalse(p.moving or p.armed)
        p.values[1]=.05
        k.press('b')
        k.press('e');k.press('h')
        self.assertFalse(p.moving)
        k.release('h');k.stop()
        p.values[1]=.4
        k.press('t')
        p.values[1]=.06
        k.press('e');k.press('h')
        self.assertTrue(p.moving)
        self.assertAlmostEqual(p.target,.06+math.radians(5))
        k.press('t')  # motion cannot capture
        self.assertAlmostEqual(self.teach.target(2),.4)
        k.release('h')
        self.assertFalse(p.moving)
        k.press('up')
        self.assertAlmostEqual(p.target,.06+math.radians(1))
        k.release('up');k.stop()
        p.values[1]=.42;k.press('t')
        other=PoseTeach(self.path,MODEL,self.plan,True,[])
        with self.assertRaisesRegex(ValueError,'in this session'):other.target(2)
        row=replace(p.stationary_position(),position_rad=1.0)
        other.capture(2,'c',row)
        with self.assertRaisesRegex(ValueError,'inward'):other.target(2)
        other.capture(2,'b',replace(row,position_rad=1.02))
        self.assertAlmostEqual(other.target(2),1.42)

    def test_measured_stop_map_sign_and_manual_alignment(self):
        teach=PoseTeach(self.path,MODEL,self.plan,True,[['3','30','-2']])
        self.port.select(3);row=self.port.stationary_position()
        teach.capture(3,'c',row)
        teach.capture(3,'b',replace(row,position_rad=-.02))
        with self.assertRaisesRegex(ValueError,'inward'):teach.waypoint(3,row,.05)
        teach.capture(3,'b',replace(row,position_rad=.02))
        self.assertAlmostEqual(teach.target(3),(-.44-math.radians(30))*-2)
        self.port.select(6);row=replace(self.port.stationary_position(),position_rad=1.3)
        teach.capture(6,'c',row)
        self.assertAlmostEqual(teach.target(6),1.3)
        self.assertIsNone(teach.waypoint(6,row,.05))
        with self.assertRaisesRegex(ValueError,'different'):PoseTeach(self.path,MODEL,self.plan,False,[])
        self.keys.press('4');self.keys.press('c')
        self.keys.recover()
        with self.assertRaisesRegex(ValueError,'in this session'):self.teach.target(4)

    def test_invalid_capture_and_disk_error_do_not_commit(self):
        row=self.port.stationary_position()
        for bad in [replace(row,age_ms=80),replace(row,enabled=True),replace(row,position_rad=math.nan)]:
            with self.assertRaises(ValueError):self.teach.capture(2,'c',bad)
        self.assertFalse(self.path.exists())
        self.teach.capture(2,'c',row)
        self.teach.capture(2,'b',replace(row,position_rad=.1))
        old=self.path.read_bytes()
        with patch('motors.arm_pose_teach.os.replace',side_effect=OSError('disk full')), self.assertRaises(OSError):
            self.teach.capture(2,'t',replace(row,position_rad=.5))
        self.assertEqual(self.path.read_bytes(),old)
        with self.assertRaisesRegex(ValueError,'Hand-align'):self.teach.target(2)
        with self.assertRaisesRegex(ValueError,'clearance'):
            self.teach.trim_target(2,replace(row,position_rad=.01),-1)

    def test_trial_cli_envelope_before_hardware_open(self):
        args=['--execute','--plan',str(CONFIG/'arm_rig_a2_a6.json'),'--manual-travel','--supported']
        for extra in [['--speed-deg-s','7.5'], ['--speed-trial','--speed-deg-s','10.1'],
                      ['--speed-trial','--teach-file',str(self.path)]]:
            with self.subTest(extra=extra), patch('teleop.arm_keyboard.ArmPositionPort') as ctor:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):main(args+extra)
                ctor.assert_not_called()

    def test_simulator_entry_point_writes_separate_teach_and_session_files(self):
        from PyQt5 import QtCore,QtTest,QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        report=self.path.with_name('report.json')
        def capture_and_close():
            window=next(w for w in app.topLevelWidgets() if w.isVisible() and hasattr(w,'keys'))
            QtTest.QTest.keyClick(window,QtCore.Qt.Key_6)
            QtTest.QTest.keyClick(window,QtCore.Qt.Key_C)
            window.close()
        QtCore.QTimer.singleShot(50,capture_and_close)
        with patch('teleop.arm_keyboard.ArmPositionPort') as ctor:
            result=main(['--simulate','--manual-travel','--plan',str(CONFIG/'arm_rig_a2_a6.json'),
                         '--teach-file',str(self.path),'--report',str(report)])
            ctor.assert_not_called()
        self.assertEqual(result,0)
        self.assertEqual(json.loads(report.read_text())['pose_captures'][0]['joint'],6)
        self.assertEqual(json.loads(self.path.read_text())['joints']['6']['delta_rad'],0)
