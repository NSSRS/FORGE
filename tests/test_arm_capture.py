"""Offline evidence for guided capture without changing motor ownership/holds."""
import contextlib
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import source_paths
from motors.arm_capture import ArmCapture
from motors.arm_group_hold import SimulatedGroupHoldPort
from teleop.arm_keyboard import ArmJogKeys, main, make_window
import test_arm_group_hold as group_tests
from motors import arm_position as pos, arm_bench as bench

ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT/'src/robot_description/config/arm_rig_a1_a6.json'
MODEL = ROOT/'src/robot_description/mujoco/initial_pose.json'


class CaptureTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name)/'capture.json'
        self.capture = ArmCapture(self.path, MODEL, json.loads(PLAN.read_text()), True, 'test fixture', mode='bounds')
        self.addCleanup(self.capture.close)
        self.port = SimulatedGroupHoldPort(window_degrees=None)
        self.port.hold_all()
        self.keys = ArmJogKeys(self.port, 10, capture=self.capture)
        self.addCleanup(self.port.close)

    def settle(self, raw, joint=1):
        self.capture.history.clear()
        self.port.values[joint-1] = raw
        for n in range(31):
            self.capture.observe(self.port.cached_positions(), True, now=10+n*.02)

    def save(self, joint=1):
        self.capture.capture(joint, dict(build_id=123), now=10.6)
        self.capture.pending[0].result(timeout=2)
        return self.capture.finish_save()

    def test_both_endpoint_orders_and_initial_raw_not_session_zero(self):
        for joint, ends in ((1, (2., 1.)), (2, (-1., 1.))):
            for raw in (*ends, sum(ends)/2):
                self.settle(raw, joint);self.assertIn('Saved', self.save(joint))
            record = self.capture.records['joints'][str(joint)]
            self.assertEqual(record['raw_lower_rad'], min(ends))
            self.assertEqual(record['raw_upper_rad'], max(ends))
            self.assertEqual(record['initial_raw_rad'], sum(ends)/2)
            self.assertEqual(set(record['initial']['group_raw_rad']), set('123456'))
            self.assertEqual(record['model_initial_rad'], self.capture.goals[joint])
        saved = json.loads(self.path.read_text())
        self.assertFalse(saved['complete'] or saved['production_qualified'])
        self.assertIsNone(saved['model_sign'])
        self.assertEqual(saved['base_info']['build_id'], 123)
        with self.assertRaises(FileExistsError):
            ArmCapture(self.path, MODEL, json.loads(PLAN.read_text()), True, 'new run')

    def test_complete_only_after_all_configured_joints_have_three_points(self):
        for joint in range(1, 7):
            for raw in (-1., 1., .25):
                self.settle(raw, joint);self.save(joint)
            self.assertEqual(self.capture.records['complete'], joint == 6)

    def test_reference_mode_requires_both_a2_a3_ends_and_one_initial_for_other_joints(self):
        self.capture = ArmCapture(self.path.with_name('initial.json'), MODEL, json.loads(PLAN.read_text()), True)
        self.addCleanup(self.capture.close)
        self.assertIn('forward', self.capture.describe(1))
        self.assertIn('most clockwise', self.capture.describe(2))
        self.assertIn('most clockwise', self.capture.describe(3))
        for joint in (4, 5, 6):self.assertIn('initial position', self.capture.describe(joint))
        for joint in range(1, 7):
            raw = joint*.1
            self.settle(raw, joint);self.save(joint)
            record = self.capture.records['joints'][str(joint)]
            self.assertNotIn('raw_lower_rad', record)
            self.assertNotIn('raw_upper_rad', record)
            if joint in (2, 3):
                self.assertEqual(record['cw_extreme']['raw_rad'], raw)
                self.assertEqual(record['raw_cw_rad'], raw)
                self.assertNotIn('ccw_extreme', record)
                self.assertNotIn('initial_raw_rad', record)
                self.assertIsNone(record['model_reference_rad'])
                self.assertFalse(record['endstop_contact_verified'])
                self.assertIn('2/2 (CCW extreme)', self.capture.describe(joint))
                self.assertIn('most counterclockwise', self.capture.describe(joint))
            else:
                self.assertEqual(record['reference_raw_rad'], raw)
                self.assertEqual(record['initial_raw_rad'], raw)
                self.assertEqual(record['model_initial_rad'], self.capture.goals[joint])
            self.assertFalse(self.capture.records['complete'])
        self.assertEqual(self.capture.remaining_joints(), [2, 3])
        self.assertIn('4/6 motors recorded', self.capture.progress())
        # CW/CCW labels do not assume the raw encoder's sign.
        for joint, other in ((2, -.2), (3, .5)):
            self.settle(other, joint);self.save(joint)
            record = self.capture.records['joints'][str(joint)]
            self.assertEqual(record['raw_ccw_rad'], other)
            self.assertEqual(record['raw_lower_rad'], min(joint*.1, other))
            self.assertEqual(record['raw_upper_rad'], max(joint*.1, other))
            self.assertEqual(record['ccw_extreme']['group_raw_rad'].keys(), set('123456'))
            self.assertNotIn('initial_raw_rad', record)
            self.assertEqual(self.capture.records['complete'], joint == 3)
        saved = json.loads(self.capture.path.read_text())
        self.assertEqual(saved['version'], 2)
        self.assertEqual(saved['capture_steps']['2'], ['cw_extreme', 'ccw_extreme'])
        self.assertEqual(saved['capture_steps']['3'], ['cw_extreme', 'ccw_extreme'])
        self.assertEqual(saved['capture_steps']['5'], ['reference'])
        self.assertEqual(saved['capture_mode'], 'reference')
        self.assertEqual(saved['fixture_note'], '')
        self.assertFalse(saved['production_qualified'] or saved['power_cycle_repeatability_verified'])

    def test_opposite_end_rejects_duplicate_wrap_and_failed_write_without_losing_first_end(self):
        self.capture = ArmCapture(self.path.with_name('ends.json'), MODEL, json.loads(PLAN.read_text()), True)
        self.addCleanup(self.capture.close)
        self.settle(1., 2);self.save(2)
        previous = self.capture.path.read_bytes()
        for raw, message in ((1.001, 'apart'), (-12., 'branch')):
            self.settle(raw, 2)
            with self.assertRaisesRegex(ValueError, message):self.capture.capture(2, now=10.6)
            self.assertEqual(self.capture.path.read_bytes(), previous)
            self.assertEqual(self.capture.next_stage(2), 'ccw_extreme')
        self.settle(2., 2)
        with patch('motors.arm_capture.os.replace', side_effect=OSError('disk full')):
            self.capture.capture(2, now=10.6)
            with self.assertRaises(OSError):self.capture.pending[0].result(timeout=2)
            self.assertIn('NOT saved', self.capture.finish_save())
        self.assertEqual(self.capture.path.read_bytes(), previous)
        self.assertEqual(self.capture.next_stage(2), 'ccw_extreme')
        self.assertTrue(self.port.holding)
        self.assertIn('Saved A2 CCW extreme', self.save(2))
        self.assertIsNone(self.capture.next_stage(2))

    def test_qt_a2_keeps_capture_active_after_cw_and_offers_next_only_after_ccw(self):
        from PyQt5 import QtWidgets, QtTest, QtCore
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.capture = ArmCapture(self.path.with_name('a2-ui.json'), MODEL, json.loads(PLAN.read_text()), True)
        self.addCleanup(self.capture.close)
        self.settle(.1);self.save()  # A1 completed before A2.
        self.port.select(2)
        window = make_window(self.port, 'SIMULATION', speed_deg_s=10, capture=self.capture)
        try:
            window.show();app.processEvents();window.timer.stop()
            targets = dict(self.port.hold_targets)
            for raw, step in ((1., 'CW'), (-1., 'CCW')):
                self.settle(raw, 2)
                with patch('motors.arm_capture.time', SimpleNamespace(monotonic=lambda: 10.6)):
                    QtTest.QTest.mouseClick(window.capture_button, QtCore.Qt.LeftButton)
                    self.capture.pending[0].result(timeout=2);window.update_state()
                self.assertIn(f'Saved A2 {step} extreme', window.capture_result.text())
                self.assertEqual(self.port.selected, 2)
                self.assertEqual(self.port.hold_targets, targets)
                self.assertTrue(self.port.holding)
                self.assertFalse(self.port.armed or self.port.moving)
                self.assertEqual(window.capture_button.isEnabled(), step == 'CW')
                self.assertEqual(window.capture_next.isEnabled(), step == 'CCW')
                if step == 'CW':self.assertIn('2/2 (CCW extreme)', window.capture_label.text())
            self.assertIn('NEXT: select A3', window.capture_next.text())
            QtTest.QTest.mouseClick(window.capture_next, QtCore.Qt.LeftButton);window.update_state()
            self.assertEqual(self.port.selected, 3)
            self.assertIn('1/2 (CW extreme)', window.capture_label.text())
        finally:window.close()

    def test_reject_stale_disabled_moving_missing_nonfinite_or_unsettled_feedback(self):
        rows = self.port.cached_positions()
        bads = [rows[:-1]]
        for changes in (dict(age_ms=80), dict(enabled=False), dict(position_rad=float('nan')),
                        dict(velocity_rad_s=.04), dict(velocity_rad_s=float('inf'))):
            bads.append([replace(rows[0], **changes), *rows[1:]])
        for bad in bads:
            self.settle(1.)
            self.capture.observe(bad, True, now=10.62)
            with self.assertRaisesRegex(ValueError, 'stationary'):
                self.capture.capture(1, now=10.62)
        self.settle(1.)
        with self.assertRaisesRegex(ValueError, 'stationary'):
            self.capture.capture(1, now=10.7)
        self.capture.history.clear()
        for n in range(31):
            self.port.values[0] = n*.001
            self.capture.observe(self.port.cached_positions(), True, now=10+n*.02)
        with self.assertRaisesRegex(ValueError, 'settled'):
            self.capture.capture(1, now=10.6)

    def test_reject_wrap_duplicate_endpoint_and_initial_outside_bounds(self):
        self.settle(1.);self.save()
        for raw, message in ((1.001, 'apart'), (-12., 'branch')):
            self.settle(raw)
            with self.assertRaisesRegex(ValueError, message):self.capture.capture(1, now=10.6)
        self.settle(2.);self.save()
        self.settle(3.)
        with self.assertRaisesRegex(ValueError, 'inside'):self.capture.capture(1, now=10.6)
        self.assertNotIn('initial', self.capture.records['joints']['1'])

    def test_capture_gate_and_disk_failure_never_disable_holds(self):
        self.keys.press('e');self.keys.press('c')
        self.assertIsNone(self.capture.pending)
        self.keys.press(' ');self.settle(1.)
        old = self.path.read_bytes()
        with patch('motors.arm_capture.os.replace', side_effect=OSError('disk full')):
            self.capture.capture(1, now=10.6)
            with self.assertRaises(OSError):self.capture.pending[0].result(timeout=2)
            self.assertIn('NOT saved', self.capture.finish_save())
        self.assertEqual(self.path.read_bytes(), old)
        self.assertEqual(self.capture.records['joints'], {})
        self.assertTrue(self.port.holding)
        self.assertIn('Saved', self.save())

    def test_slow_disk_is_background_and_does_not_prevent_owner_ticks(self):
        entered, release = threading.Event(), threading.Event()
        def slow_save(*args):
            entered.set();release.wait(2)
        self.settle(1.)
        with patch('motors.arm_capture._write', side_effect=slow_save):
            try:
                self.capture.capture(1, now=10.6)
                self.assertTrue(entered.wait(1))
                with patch.object(self.port, 'poll', wraps=self.port.poll) as poll:
                    self.keys.tick()
                    poll.assert_called_once()
                self.assertFalse(self.capture.pending[0].done())
                self.assertTrue(self.port.holding)
            finally:release.set()

    def test_qt_capture_button_uses_existing_owner_and_no_disarm(self):
        from PyQt5 import QtWidgets, QtTest, QtCore
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window = make_window(self.port, 'SIMULATION', speed_deg_s=10, capture=self.capture)
        try:
            window.show();app.processEvents();self.settle(1.)
            with patch('motors.arm_capture.time.monotonic', return_value=10.6):
                QtTest.QTest.mouseClick(window.capture_button, QtCore.Qt.LeftButton)
            self.assertIsNotNone(self.capture.pending)
            self.assertTrue(self.port.holding)
            self.capture.pending[0].result(timeout=2)
            self.capture.finish_save()
            window.update_state()
            self.assertIn('2/3', window.capture_label.text())
        finally:window.close()

    def test_qt_c_key_without_text_keeps_result_after_focus_loss_and_offers_next_motor(self):
        from PyQt5 import QtWidgets, QtTest, QtCore, QtGui
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.capture = ArmCapture(self.path.with_name('reference.json'), MODEL, json.loads(PLAN.read_text()), True)
        self.addCleanup(self.capture.close)
        window = make_window(self.port, 'SIMULATION', speed_deg_s=10, capture=self.capture)
        try:
            window.show();app.processEvents();window.timer.stop()
            fixed = dict(self.port.hold_targets)
            window.keys.press('e')
            def press_c():
                app.sendEvent(window, QtGui.QKeyEvent(QtCore.QEvent.KeyPress,
                    QtCore.Qt.Key_C, QtCore.Qt.NoModifier, ''))
            press_c()
            self.assertIn('NOT SAVED', window.keys.capture_message)
            self.assertIn('Space', window.keys.capture_message)
            with patch.object(window, 'isActiveWindow', return_value=False):
                app.sendEvent(window, QtCore.QEvent(QtCore.QEvent.ActivationChange))
            window.update_state()
            self.assertIn('NOT SAVED', window.capture_result.text())
            self.assertEqual(json.loads(self.capture.path.read_text())['joints'], {})
            self.assertTrue(self.port.holding)
            self.settle(1.)
            with patch('motors.arm_capture.time', SimpleNamespace(monotonic=lambda: 10.6)):
                press_c()
                self.assertIsNotNone(self.capture.pending, window.keys.capture_message)
                self.capture.pending[0].result(timeout=2)
                window.update_state()
            self.assertIn('Saved A1 reference: raw +57.296°', window.capture_result.text())
            self.assertIn('1/6 motors recorded', window.capture_progress.text())
            self.assertIn('Select A2', window.capture_label.text())
            self.assertTrue(window.capture_next.isEnabled())
            self.assertFalse(window.capture_button.isEnabled())
            self.assertEqual(self.port.selected, 1)  # No automatic selection/movement.
            window.keys.stop();window.update_state()
            self.assertIn('Saved A1', window.capture_result.text())
            QtTest.QTest.mouseClick(window.capture_next, QtCore.Qt.LeftButton)
            window.update_state()
            self.assertEqual(self.port.selected, 2)
            self.assertIn('most clockwise', window.capture_label.text())
            self.assertIn('Saved A1', window.capture_result.text())
            self.assertTrue(self.port.holding)
            self.assertFalse(self.port.armed or self.port.moving)
            self.assertEqual(self.port.hold_targets, fixed)
            self.assertEqual(json.loads(self.capture.path.read_text())['joints']['1']['reference_raw_rad'], 1.)
            self.assertEqual(len(self.port.report['capture_events']), 3)
        finally:window.close()

    def test_readiness_names_joint_and_recovers_after_full_stationary_window(self):
        rows = self.port.cached_positions()
        for changes, detail in ((dict(age_ms=81), 'stale'), (dict(enabled=False), 'not enabled'),
                                (dict(velocity_rad_s=.04), 'not stationary')):
            self.settle(1.)
            bad = [*rows[:3], replace(rows[3], **changes), *rows[4:]]
            self.capture.observe(bad, True, now=10.62)
            reason = self.capture.readiness_reason(now=10.62)
            self.assertIn('A4', reason);self.assertIn(detail, reason)
            self.assertIsNone(self.capture.pending)
        self.capture.observe(rows, True, now=10.64)
        self.assertIn('0.00 / 0.50 s', self.capture.readiness_reason(now=10.64))
        self.settle(1.)
        self.assertIsNone(self.capture.readiness_reason(now=10.6))

    def test_cli_rejects_invalid_capture_combinations_before_motor_open(self):
        args = ['--execute', '--supported', '--manual-travel', '--plan', str(PLAN),
                '--capture-file', str(self.path.with_name('new.json'))]
        for extra in ([], ['--hold-all', '--fixture-note', 'rig', '--report', str(self.path.with_name('new.json'))],
                      ['--hold-all', '--fixture-note', 'rig', '--teach-file', 'pose.json']):
            with patch('teleop.arm_keyboard.ArmGroupHoldPort') as ctor:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):main(args+extra)
                ctor.assert_not_called()

    def test_simulated_entry_point_finishes_pending_save_on_close(self):
        from PyQt5 import QtWidgets, QtCore
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        target = self.path.with_name('entry.json')
        report = self.path.with_name('session.json')
        state = {}
        def start():
            window = next(w for w in app.topLevelWidgets() if w.isVisible() and hasattr(w, 'keys'))
            state['window'] = window
            window.keys.press('h')
        def capture_and_close():
            window = state['window']
            window.keys.press('c')
            state['message'] = window.keys.message
            window.close()
        QtCore.QTimer.singleShot(50, start)
        QtCore.QTimer.singleShot(750, capture_and_close)
        with patch('teleop.arm_keyboard.ArmGroupHoldPort') as ctor:
            result = main(['--simulate', '--supported', '--manual-travel', '--hold-all',
                           '--plan', str(PLAN), '--capture-file', str(target),
                           '--report', str(report)])
            ctor.assert_not_called()
        self.assertEqual(result, 0)
        self.assertIn('Saving', state['message'])
        saved = json.loads(target.read_text())
        self.assertEqual(saved, json.loads(report.read_text())['position_capture'])
        self.assertEqual(saved['capture_mode'], 'reference')
        self.assertIn('reference', saved['joints']['1'])
        self.assertFalse(state['window'].keys.port.holding)


class NativeCaptureTests(unittest.TestCase):
    setUpClass = classmethod(group_tests.GroupHoldTests.setUpClass.__func__)
    tearDownClass = classmethod(group_tests.GroupHoldTests.tearDownClass.__func__)
    setUp = group_tests.GroupHoldTests.setUp
    tearDown = group_tests.GroupHoldTests.tearDown
    port = group_tests.GroupHoldTests.port

    def test_startup_undervoltage_opens_fault_window_and_recovery_keeps_motors_disabled(self):
        from PyQt5 import QtWidgets
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with tempfile.TemporaryDirectory() as directory, patch.object(pos, 'time', bench.time):
            previous = self.port()
            previous.session.cleanup()  # Leave the configured routes, like the real previous run.
            state = self.bus.state
            def undervoltage(slot):
                sample = state(slot)
                return replace(sample, status=9, stopping=1, age_ms=0xffffffff) if slot == 0 else sample
            with patch.object(self.bus, 'state', side_effect=undervoltage), \
                 patch.object(self.bus, 'enable_bench_group') as enable, \
                 patch.object(self.bus, 'write_routed_bench') as write, \
                 patch.object(self.bus, 'reset_fault') as reset:
                port = self.port(allow_faulted_start=True)
                self.assertTrue(port.recovery_required)
                self.assertFalse(port.closed or port.holding or port.armed or port.moving)
                self.assertIn('A1: drive status 9 (undervoltage)', port.report['error'])
                self.assertEqual(port.report['startup_disable_states'][0]['status'], 9)
                window = make_window(port, 'HARDWARE (TEST)', speed_deg_s=10,
                                     recovery_report_path=Path(directory)/'session.json')
                try:
                    window.show();window.update_state();app.processEvents()
                    self.assertTrue(window.isVisible())
                    self.assertIn('undervoltage', window.status.text())
                    for key in ('h','e','up','c'):window.keys.press(key)
                    self.assertFalse(port.holding or port.armed or port.moving)
                    window.keys.release('up')
                finally:window.close()
                enable.assert_not_called();write.assert_not_called();reset.assert_not_called()
            try:
                with patch.object(self.bus, 'enable_bench_group') as enable, \
                     patch.object(self.bus, 'reset_fault') as reset:
                    port.recover(Path(directory)/'session.json')
                    enable.assert_not_called();reset.assert_not_called()
                self.assertFalse(port.recovery_required or port.holding or port.armed or port.moving)
                self.assertTrue(all(row.age_ms < 80 and not row.enabled for row in port.cached_positions()))
            finally:port.close()

    def test_disabled_timeout_identifies_missing_motor_without_accepting_it(self):
        with patch.object(pos, 'time', bench.time):
            port = self.port()
            try:
                self.lib.fake_route(3, 2, 4, 31)  # No feedback on the configured A4 route.
                with self.assertRaises(bench.DisableAcknowledgementTimeout) as caught:
                    port.session.stop()
                self.assertIn('A4:', str(caught.exception))
                self.assertIn(3, caught.exception.states)
                self.assertIn(3, port.report['disable_timeout_states'])
                self.assertFalse(port.holding or port.armed)
            finally:
                self.lib.fake_route(3, 2, 4, 20)
                port.close()

    def test_capture_through_real_owner_keeps_group_enabled_without_register_io(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(pos, 'time', bench.time), self.port() as port:
            capture = ArmCapture(Path(directory)/'capture.json', MODEL, json.loads(PLAN.read_text()), False, 'native fake')
            try:
                keys = ArmJogKeys(port, 10, capture=capture);keys.press('h')
                fixed = dict(port.hold_targets)
                disables = [self.lib.fake_disables(i) for i in range(6)]
                writes = self.lib.fake_parameter_writes()
                with patch('motors.arm_capture.time', bench.time), patch('teleop.arm_keyboard.time', bench.time):
                    for _ in range(35):self.lib.fake_advance(20);keys.tick()
                    with patch.object(port.bus, '_request', wraps=port.bus._request) as request:
                        keys.press('c')
                        request.assert_not_called()
                    self.assertIsNotNone(capture.pending, keys.message)
                    capture.pending[0].result(timeout=2);keys.tick()
                self.assertEqual(capture.records['joints']['1']['reference']['group_raw_rad'].keys(), set('123456'))
                self.assertEqual(port.hold_targets, fixed)
                self.assertEqual([self.lib.fake_status(i) for i in range(6)], [1]*6)
                self.assertEqual([self.lib.fake_disables(i) for i in range(6)], disables)
                self.assertEqual(self.lib.fake_parameter_writes(), writes)
            finally:capture.close()
