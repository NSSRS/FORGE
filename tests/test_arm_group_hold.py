import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from motors import arm_position as pos
from motors import arm_bench as bench
from motors.arm import CAP_GROUP_HOLD
from motors.arm_group_hold import ArmGroupHoldPort, SimulatedGroupHoldPort
import test_damiao_bench as routed_tests
from teleop.arm_keyboard import ArmJogKeys, make_window


class GroupHoldTests(unittest.TestCase):
    setUpClass=classmethod(routed_tests.RoutedRoundTrip.setUpClass.__func__)
    tearDownClass=classmethod(routed_tests.RoutedRoundTrip.tearDownClass.__func__)
    setUp=routed_tests.RoutedRoundTrip.setUp
    tearDown=routed_tests.RoutedRoundTrip.tearDown

    def port(self,subset=False,**kwargs):
        p=json.loads((routed_tests.ROOT/'src/robot_description/config'/('arm_rig_a2_a6.json' if subset else 'arm_rig_a1_a6.json')).read_text())
        p.update(fixture_confirmed=True,supported_confirmed=True)
        if subset:self.lib.fake_missing(0)
        return ArmGroupHoldPort(p,bus=self.bus,window_degrees=None,**kwargs)

    def test_old_firmware_rejected_before_motor_operations(self):
        self.bus.capabilities &= ~CAP_GROUP_HOLD
        with patch.object(self.bus,'_request',wraps=self.bus._request) as request:
            with self.assertRaisesRegex(RuntimeError,'0x20000'):self.port()
            request.assert_not_called()

    def test_all_hold_current_positions_select_and_jog_without_disable(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
            with self.assertRaisesRegex(RuntimeError,'Press H'):port.arm()
            for i in range(6):self.lib.fake_position(i,.02*(i+1))
            port.hold_all();targets=dict(port.hold_targets)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[1]*6)
            for i in range(6):self.assertAlmostEqual(targets[i],.02*(i+1),delta=.001)
            disables=[self.lib.fake_disables(i) for i in range(6)]
            writes=self.lib.fake_parameter_writes()
            self.lib.fake_position(5,targets[5]-.02)
            for _ in range(300):self.lib.fake_advance(20);port.poll()
            self.assertEqual(port.hold_targets,targets)
            self.assertAlmostEqual(self.bus.state(5).position,targets[5],delta=.001)
            for joint,direction in ((2,1),(3,-1)):
                port.select(joint);self.assertFalse(port.armed);port.arm();port.jog(direction,math.radians(5))
                for _ in range(30):self.lib.fake_advance(20);port.poll()
                port.release();targets[joint-1]=port.target
                self.assertEqual(port.hold_targets,targets)
                self.assertTrue(port.holding)
                self.assertEqual([self.lib.fake_disables(i) for i in range(6)],disables)
            port.arm();port.jog(1,math.radians(5));self.lib.fake_advance(20);port.poll()
            port.select(4);self.assertFalse(port.moving or port.armed);self.assertTrue(port.holding)
            targets=dict(port.hold_targets)
            self.lib.fake_position(5,targets[5]-.01);port.hold_all()
            self.assertEqual(port.hold_targets,targets)  # H cannot adopt passive drift.
            self.assertEqual(self.lib.fake_parameter_writes(),writes)
            self.assertEqual([self.lib.fake_disables(i) for i in range(6)],disables)
            self.assertTrue(all(r.enabled and r.age_ms<80 for r in port.cached_positions()))
            self.assertEqual(set(port.report['motion_summary']),set('123456'))
            port.stop();self.assertFalse(port.hold_targets)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)

    def test_sparse_group_keeps_physical_slots(self):
        with patch.object(pos,'time',bench.time),self.port(subset=True) as port:
            port.hold_all()
            for _ in range(300):self.lib.fake_advance(20);port.poll()
            self.assertEqual(set(port.hold_targets),{1,2,3,4,5})
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0,1,1,1,1,1])
            with self.assertRaises(ValueError):port.select(1)
            self.assertTrue(port.holding)

    def test_firmware_rejects_partial_group_individual_and_bad_preload_before_enable(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            states=port._stop_session()
            for i,s in states.items():self.bus.command(i,s.position,velocity=math.radians(5))
            with self.assertRaises(RuntimeError):self.bus.enable_bench_group([1])
            with self.assertRaises(RuntimeError):self.bus.enable(1)
            self.bus.command(5,states[5].position+.1,velocity=math.radians(5))
            with self.assertRaises(RuntimeError):self.bus.enable_bench_group(port.slots)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)

    def test_firmware_rejects_mixed_group_policy_before_any_enable(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            self.bus.prepare_routed_bench(5,port.motors[5],*port.session.bounds[5],2,
                                         manual_travel=True,position_hold=True)
            self.bus.probe(5)
            port.session.poll(5,lambda s:s.registers==31,'register readback')
            states=port._stop_session()
            for i,s in states.items():self.bus.command(i,s.position,velocity=math.radians(5))
            with self.assertRaises(RuntimeError):self.bus.enable_bench_group(port.slots)
            with self.assertRaises(RuntimeError):self.bus.enable(5)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)

    def test_group_jog_keeps_requested_speed_with_lagging_selected_joint(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            self.lib.fake_position_gain(1,1.3)
            port.hold_all();port.select(2);port.arm()
            for direction in (1,-1):
                port.jog(direction,math.radians(5));samples=[]
                for step in range(200):
                    self.lib.fake_advance(20);port.poll()
                    if step>=150:samples.append(port.report['motion_summary']['2']['last_sample'])
                first,last=samples[0],samples[-1]
                speed=direction*(last['position_rad']-first['position_rad'])/(last['at_monotonic_s']-first['at_monotonic_s'])
                self.assertAlmostEqual(math.degrees(speed),5,delta=.15)
                self.assertFalse(any(s['lead_limited'] for s in samples))
                port.release()
                self.assertTrue(all(abs(port.hold_targets[i])<.001 for i in (0,2,3,4,5)))

    def test_unselected_joint_missing_feedback_stops_group_with_fresh_commands(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            port.hold_all();self.lib.fake_route(4,2,5,31)
            try:
                with self.assertRaises(RuntimeError):
                    for _ in range(8):self.lib.fake_advance(20);port._refresh_targets()
                self.assertEqual(self.bus.fault_snapshot()['reason'],2)
                with self.assertRaises(RuntimeError):port.poll()
                self.assertFalse(port.hold_targets)
                self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
            finally:self.lib.fake_route(4,2,5,21)

    def test_unselected_joint_torque_trip_disables_whole_group(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            port.hold_all();self.lib.fake_torque(5,4);self.lib.fake_advance(10)
            with self.assertRaisesRegex(RuntimeError,'torque'):port.poll()
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
            self.assertFalse(port.hold_targets or port.armed)
            self.assertTrue(port.recovery_required)
            self.lib.fake_torque(5,0)

    def test_one_missing_command_lease_stops_all_and_recovery_does_not_hold(self):
        with patch.object(pos,'time',bench.time),self.port() as port,tempfile.TemporaryDirectory() as d:
            port.hold_all()
            with self.assertRaises(RuntimeError):
                for _ in range(10):
                    self.lib.fake_advance(20)
                    for i in (0,1,2,3,5):self.bus.command(i,port.hold_targets[i],velocity=port.speed)
            fault=self.bus.fault_snapshot();self.assertEqual(fault['reason'],1)
            with self.assertRaises(RuntimeError):port.poll()
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
            port.recover(Path(d)/'fault.json')
            self.assertFalse(port.hold_targets or port.holding or port.armed or port.moving)
            with self.assertRaisesRegex(RuntimeError,'Press H'):port.arm()
            port.hold_all();self.assertTrue(port.holding)

    def test_display_reads_do_not_extend_group_leases(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            port.hold_all()
            with self.assertRaisesRegex(RuntimeError,'lease expired'):
                for _ in range(8):self.lib.fake_advance(20);port.positions()
            with self.assertRaises(RuntimeError):port.poll()
            self.assertEqual(self.bus.fault_snapshot()['reason'],1)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)

    def test_activation_cancellation_after_enable_disables_all(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            allowed=[True];port.motion_allowed=lambda:allowed[0]
            enable=self.bus.enable_bench_group
            def cancel(slots):enable(slots);allowed[0]=False
            with patch.object(self.bus,'enable_bench_group',side_effect=cancel):
                with self.assertRaises(pos.MotionCancelled):port.hold_all()
            self.assertFalse(port.hold_targets or port.holding or port.armed)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)


class GroupKeyboardTests(unittest.TestCase):
    def test_ui_stall_disables_all_and_does_not_reactivate(self):
        with patch('motors.arm_position.time.monotonic',return_value=1.) as clock:
            port=SimulatedGroupHoldPort(window_degrees=None);keys=ArmJogKeys(port,speed_deg_s=5)
            keys.press('h');clock.return_value=1.16
            with self.assertRaisesRegex(RuntimeError,'UI stalled'):keys.tick()
            self.assertFalse(port.hold_targets or port.holding or port.armed)
            keys.press('e');self.assertFalse(port.armed);port.close()

    def test_selection_preserves_other_holds_and_requires_fresh_arm(self):
        with patch('motors.arm_position.time.monotonic',return_value=1.) as clock:
            port=SimulatedGroupHoldPort(window_degrees=None);keys=ArmJogKeys(port,speed_deg_s=5)
            keys.press('e');self.assertFalse(port.armed)
            keys.press('h');self.assertTrue(port.holding);self.assertFalse(port.armed)
            keys.press('2');keys.press('e');keys.press('up');clock.return_value=1.1;keys.tick()
            keys.press('3');self.assertFalse(port.moving or port.armed)
            self.assertEqual(port.selected,3);self.assertTrue(all(r.enabled for r in port.positions()))
            old=port.values[1]
            keys.press('e');self.assertFalse(port.armed)  # Arrow is still down.
            keys.release('up');keys.press('e');keys.press('down');clock.return_value=1.2;keys.tick();keys.release('down')
            self.assertEqual(port.values[1],old);self.assertLess(port.values[2],0)
            keys.press(' ');self.assertTrue(port.holding);self.assertFalse(port.armed)
            keys.press('d');self.assertFalse(any(r.enabled for r in port.positions()));port.close()

    def test_qt_hold_button_focus_selection_close(self):
        from PyQt5 import QtCore,QtWidgets,QtTest
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        port=SimulatedGroupHoldPort(window_degrees=None)
        window=make_window(port,'SIMULATION',speed_deg_s=5);window.show();app.processEvents()
        try:
            QtTest.QTest.mouseClick(window.hold_all_button,QtCore.Qt.LeftButton)
            self.assertTrue(port.holding)
            QtTest.QTest.keyClick(window,QtCore.Qt.Key_2);QtTest.QTest.keyClick(window,QtCore.Qt.Key_E)
            QtTest.QTest.keyPress(window,QtCore.Qt.Key_Up)
            with patch.object(window,'isActiveWindow',return_value=False):
                app.sendEvent(window,QtCore.QEvent(QtCore.QEvent.ActivationChange))
            self.assertTrue(port.holding);self.assertFalse(port.moving or port.armed)
            QtTest.QTest.keyRelease(window,QtCore.Qt.Key_Up)
            QtTest.QTest.keyClick(window,QtCore.Qt.Key_3)
            self.assertEqual(port.selected,3);self.assertTrue(all(r.enabled for r in port.positions()))
            window.close();self.assertFalse(port.hold_targets)
        finally:window.close();port.close()
