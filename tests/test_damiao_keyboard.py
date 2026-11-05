import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import source_paths
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/teleop'))
from motors.arm_position import SimulatedPositionPort
from teleop.arm_keyboard import ArmJogKeys,make_window

class KeyboardTests(unittest.TestCase):
    def test_hold_release_repeat_and_selection(self):
        with patch('motors.arm_position.time.monotonic',return_value=1.) as clock:
            port=SimulatedPositionPort();keys=ArmJogKeys(port)
            keys.press('3');keys.press('e');keys.press('right')
            for i in range(1,51):
                clock.return_value=1+i*.02;keys.press('right',repeat=True);keys.tick()
            self.assertAlmostEqual(port.values[2],math.radians(45))
            keys.release('right');self.assertFalse(port.moving);self.assertTrue(port.armed)
            keys.press('left');self.assertTrue(port.moving)
            keys.press('3');self.assertTrue(port.armed)
            keys.press('4');self.assertFalse(port.armed);self.assertFalse(port.moving)
            self.assertEqual(port.selected,4)
    def test_boundaries_stop_and_stall(self):
        with patch('motors.arm_position.time.monotonic',return_value=1.) as clock:
            port=SimulatedPositionPort(window_degrees=1);keys=ArmJogKeys(port)
            keys.press('e');keys.press('up')
            clock.return_value=1.1;keys.tick()
            self.assertAlmostEqual(port.values[0],math.radians(1));self.assertFalse(port.moving)
            self.assertTrue(port.armed);keys.release('up');keys.press('down')
            clock.return_value=1.4
            with self.assertRaisesRegex(RuntimeError,'stalled'):keys.tick()
            self.assertFalse(port.armed);self.assertFalse(port.moving)
    def test_conflicting_keys_and_fault_lockout(self):
        port=SimulatedPositionPort();keys=ArmJogKeys(port)
        keys.press('e');keys.press('right');keys.press('left')
        self.assertFalse(port.moving);keys.release('left');self.assertFalse(port.moving)
        keys.release('right');keys.stop();keys.failed=True
        keys.press('e');keys.press('up');self.assertFalse(port.armed)
    def test_qt_real_key_events_and_focus_loss(self):
        os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
        from PyQt5 import QtCore,QtWidgets,QtTest,QtGui
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        port=SimulatedPositionPort();window=make_window(port,'SIMULATION');window.show();app.processEvents()
        QtTest.QTest.keyClick(window,QtCore.Qt.Key_6)
        QtTest.QTest.keyClick(window,QtCore.Qt.Key_E)
        QtTest.QTest.keyPress(window,QtCore.Qt.Key_Right)
        QtTest.QTest.qWait(100)
        self.assertTrue(port.moving);self.assertGreater(port.values[5],0)
        event=QtGui.QKeyEvent(QtCore.QEvent.KeyRelease,QtCore.Qt.Key_Right,QtCore.Qt.NoModifier,'',True)
        app.sendEvent(window,event);self.assertTrue(port.moving)
        QtTest.QTest.keyRelease(window,QtCore.Qt.Key_Right)
        self.assertFalse(port.moving);self.assertTrue(port.armed)
        QtTest.QTest.keyPress(window,QtCore.Qt.Key_Left);self.assertTrue(port.moving)
        with patch.object(window,'isActiveWindow',return_value=False):
            app.sendEvent(window,QtCore.QEvent(QtCore.QEvent.ActivationChange))
        self.assertFalse(port.moving);self.assertFalse(port.armed)
        window.close();port.close()
