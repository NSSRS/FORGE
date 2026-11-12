import math
import os
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch
import source_paths
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/teleop'))
from motors.arm_position import ArmPositionPort, JointPosition, SimulatedPositionPort
from teleop.arm_keyboard import ArmJogKeys,make_window

class KeyboardTests(unittest.TestCase):
    def test_position_hold_release_pause_selection_lock_and_explicit_disable(self):
        with patch('motors.arm_position.time.monotonic',return_value=1.) as clock:
            port=SimulatedPositionPort(window_degrees=None,position_hold=True)
            keys=ArmJogKeys(port,speed_deg_s=5)
            keys.press('2');keys.press('e');keys.press('up')
            clock.return_value=1.1;keys.tick();keys.release('up')
            self.assertTrue(port.holding);target=port.target
            for step in range(1,220):
                clock.return_value=1.1+step*.1;keys.tick()
            self.assertEqual(port.values[1],target)
            keys.press('3');self.assertEqual(port.selected,2);self.assertTrue(port.holding)
            keys.press('r');self.assertFalse(keys.recovering);self.assertTrue(port.holding)
            keys.press(' ');self.assertFalse(port.armed);self.assertTrue(port.holding)
            keys.press('left');self.assertFalse(port.moving);keys.release('left')
            keys.press('e');keys.press('left');self.assertTrue(port.moving)
            keys.release('left');keys.press('d')
            self.assertFalse(port.holding or port.moving or port.armed)
            keys.press('3');self.assertEqual(port.selected,3)
            port.close()

    def test_position_hold_qt_focus_loss_holds_and_close_disables(self):
        from PyQt5 import QtCore,QtWidgets,QtTest
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        port=SimulatedPositionPort(window_degrees=None,position_hold=True)
        window=make_window(port,'SIMULATION',speed_deg_s=5);window.show();app.processEvents()
        try:
            QtTest.QTest.keyClick(window,QtCore.Qt.Key_E)
            QtTest.QTest.keyPress(window,QtCore.Qt.Key_Up)
            with patch.object(window,'isActiveWindow',return_value=False):
                app.sendEvent(window,QtCore.QEvent(QtCore.QEvent.ActivationChange))
            self.assertTrue(port.holding);self.assertFalse(port.armed or port.moving)
            window.update_state();self.assertIn('HOLDING',window.title.text())
            QtTest.QTest.keyRelease(window,QtCore.Qt.Key_Up)
            QtTest.QTest.keyClick(window,QtCore.Qt.Key_E)
            QtTest.QTest.keyPress(window,QtCore.Qt.Key_Down)
            QtTest.QTest.keyRelease(window,QtCore.Qt.Key_Down)
            self.assertTrue(port.holding)
            window.close();self.assertFalse(port.holding or port.armed or port.moving)
        finally:window.close();port.close()

    def test_position_hold_ui_stall_disables_instead_of_renewing_hold(self):
        with patch('motors.arm_position.time.monotonic',return_value=1.) as clock:
            port=SimulatedPositionPort(window_degrees=None,position_hold=True)
            keys=ArmJogKeys(port,speed_deg_s=5);keys.press('e');keys.press('up');keys.release('up')
            clock.return_value=1.16
            with self.assertRaisesRegex(RuntimeError,'UI stalled'):keys.tick()
            self.assertFalse(port.holding or port.armed or port.moving)
            port.close()

    def test_fault_table_ages_cached_samples_and_hides_old_enabled_values(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        port=SimulatedPositionPort();window=make_window(port,'SIMULATION')
        row=JointPosition(6,.28,.3,-12.5,12.5,2,True,.051,.105,3.5)
        cache=object.__new__(ArmPositionPort);cache._display_rows={5:(row,10.)}
        window.table.update_positions([row],6)
        self.assertEqual(window.table.item(5,3).text(),'Yes')
        self.assertEqual(window.table.item(5,5).text(),'+0.051')
        window.keys.fail(RuntimeError('A6 speed trip'))
        try:
            with patch.object(port,'positions',side_effect=AssertionError('fault UI must not query transport')), \
                 patch.object(port,'cached_positions',side_effect=cache.cached_positions), \
                 patch('motors.arm_position.time.monotonic',return_value=10.2):
                window.next_display=0;window.update_state()
                self.assertIn('FAULT',window.title.text())
                self.assertEqual(window.table.item(5,3).text(),'STALE')
                self.assertEqual(window.table.item(5,4).text(),'202')
                self.assertEqual(window.table.item(5,5).text(),'—')
                self.assertEqual(window.table.item(5,6).text(),'—')
            with patch.object(port,'cached_positions',side_effect=cache.cached_positions), \
                 patch('motors.arm_position.time.monotonic',return_value=11.):
                window.next_display=0;window.update_state()
                self.assertEqual(window.table.item(5,4).text(),'1002')
        finally:window.close();port.close()

    def test_recovery_requires_released_arrows_and_fresh_arm_and_direction(self):
        port=SimulatedPositionPort();keys=ArmJogKeys(port)
        keys.press('e');keys.press('up');keys.fail(RuntimeError('torque trip'))
        with patch.object(port,'recover',wraps=port.recover) as recover:
            keys.press('e');self.assertFalse(port.armed)
            keys.press('r');recover.assert_not_called()
            keys.release('up');keys.press('r',repeat=True);recover.assert_not_called()
            keys.press('r');recover.assert_called_once()
        self.assertFalse(keys.failed or port.armed or port.moving)
        keys.press('up');keys.press('e');self.assertFalse(port.armed)
        keys.release('up');keys.press('e');self.assertTrue(port.armed);self.assertFalse(port.moving)
        keys.press('down');self.assertTrue(port.moving);keys.release('down');port.close()

    def test_recovery_failure_stays_locked_and_startup_fault_is_visible(self):
        port=SimulatedPositionPort();port.recovery_required=True;port.report['error']='retained speed fault'
        keys=ArmJogKeys(port)
        self.assertTrue(keys.failed);self.assertIn('retained speed fault',keys.message)
        with patch.object(port,'recover',side_effect=RuntimeError('motor still coasting')):
            keys.press('r')
        self.assertTrue(keys.failed);self.assertFalse(keys.recovering)
        self.assertIn('motor still coasting',keys.message)
        keys.press('e');self.assertFalse(port.armed)
        keys.stop();self.assertIn('motor still coasting',keys.message)
        keys.press('r');self.assertFalse(keys.failed or port.armed or port.moving)
        port.close()

    def test_qt_recovery_worker_exclusive_transport_and_fresh_input(self):
        from PyQt5 import QtCore,QtWidgets,QtTest
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        port=SimulatedPositionPort();window=make_window(port,'SIMULATION');window.show();app.processEvents()
        entered=threading.Event();finish=threading.Event()
        original=port.recover
        def slow_recovery(path):
            entered.set()
            if not finish.wait(2):raise RuntimeError('test worker timeout')
            original(path)
        window.keys.fail(RuntimeError('torque trip'))
        try:
            with patch.object(port,'recover',side_effect=slow_recovery), \
                 patch.object(port,'positions',wraps=port.positions) as positions, \
                 patch.object(port,'stop',wraps=port.stop) as stop, \
                 patch.object(port,'poll',wraps=port.poll) as poll:
                QtTest.QTest.keyClick(window,QtCore.Qt.Key_R)
                self.assertTrue(entered.wait(1));self.assertTrue(window.keys.recovering)
                positions.reset_mock()  # Offline cache delegates to positions before worker start.
                QtTest.QTest.keyClick(window,QtCore.Qt.Key_E)
                QtTest.QTest.keyPress(window,QtCore.Qt.Key_Up)
                window.recover_button.click();window.update_state()
                with patch.object(window,'isActiveWindow',return_value=False):
                    app.sendEvent(window,QtCore.QEvent(QtCore.QEvent.ActivationChange))
                QtTest.QTest.qWait(60)
                positions.assert_not_called();stop.assert_not_called();poll.assert_not_called()
                self.assertIn('RECOVERING',window.title.text())
                self.assertFalse(port.armed or port.moving)
                finish.set();self.assertTrue(window.recovery_thread.wait(1000));app.processEvents()
            self.assertFalse(window.keys.failed or window.keys.recovering or port.armed or port.moving)
            QtTest.QTest.keyClick(window,QtCore.Qt.Key_E);self.assertFalse(port.armed)
            QtTest.QTest.keyRelease(window,QtCore.Qt.Key_Up)
            QtTest.QTest.keyClick(window,QtCore.Qt.Key_E);self.assertTrue(port.armed)
            QtTest.QTest.keyPress(window,QtCore.Qt.Key_Right);self.assertTrue(port.moving)
            QtTest.QTest.keyRelease(window,QtCore.Qt.Key_Right)
        finally:
            finish.set()
            if window.recovery_thread:window.recovery_thread.wait(1000)
            app.processEvents();window.close();port.close()

    def test_qt_close_during_recovery_waits_for_worker(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        port=SimulatedPositionPort();window=make_window(port,'SIMULATION');window.show();app.processEvents()
        entered=threading.Event();finish=threading.Event()
        def slow_recovery(path):
            entered.set()
            if not finish.wait(2):raise RuntimeError('test worker timeout')
        try:
            with patch.object(port,'recover',side_effect=slow_recovery), patch.object(port,'stop',wraps=port.stop) as stop:
                window.recover_button.click();self.assertTrue(entered.wait(1))
                window.close();app.processEvents()
                self.assertTrue(window.isVisible());self.assertTrue(window.close_after_recovery)
                stop.assert_not_called()
                finish.set();self.assertTrue(window.recovery_thread.wait(1000));app.processEvents()
                self.assertFalse(window.isVisible());stop.assert_called()
        finally:
            finish.set()
            if window.recovery_thread:window.recovery_thread.wait(1000)
            app.processEvents();window.close();port.close()

    def test_manual_keys_cross_old_window_release_reverse_and_render(self):
        with patch('motors.arm_position.time.monotonic',return_value=1.) as clock:
            port=SimulatedPositionPort(window_degrees=None,available_joints=(2,3,4,5,6))
            keys=ArmJogKeys(port,speed_deg_s=5);keys.press('e');keys.press('up')
            for i in range(1,101):
                clock.return_value=1+i*.02;keys.tick()
            self.assertAlmostEqual(port.values[1],math.radians(10));self.assertTrue(port.moving)
            self.assertIn('Manual travel',keys.message)
            keys.release('up');self.assertFalse(port.moving)
            keys.press('down');clock.return_value=3.02;keys.tick()
            self.assertLess(port.values[1],math.radians(10))
            keys.stop();self.assertFalse(port.armed)
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window=make_window(port,'SIMULATION',speed_deg_s=5)
        self.assertTrue(any('Manual travel' in label.text() for label in window.findChildren(QtWidgets.QLabel)))
        window.close();port.close()

    def test_subset_keys_keep_motor_numbers_and_slow_speed(self):
        with patch('motors.arm_position.time.monotonic',return_value=1.) as clock:
            port=SimulatedPositionPort(window_degrees=3,available_joints=(2,3,4,5,6))
            keys=ArmJogKeys(port,speed_deg_s=5)
            keys.press('6');keys.press('e');keys.press('up')
            for i in range(1,11):
                clock.return_value=1+i*.02;keys.tick()
            self.assertAlmostEqual(port.values[5],math.radians(1))
            self.assertEqual(port.values[:5],[0.]*5)
            keys.press('1')
            self.assertFalse(port.moving);self.assertFalse(port.armed)
            self.assertIn('unavailable',keys.message)
            self.assertEqual(port.selected,6)
            self.assertEqual([r.joint for r in port.positions()],[2,3,4,5,6])

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
