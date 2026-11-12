"""Offline end-to-end smooth-jog commands through the native MCU and CAN plant."""
import contextlib
import ctypes
import io
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from motors import arm_position as pos, arm_bench as bench
from motors.arm import CAP_SMOOTH_JOG, CAP_SMOOTH_JOG_10
from teleop.arm_keyboard import ArmJogKeys, main
import test_arm_group_hold as group_tests


class SmoothJogTests(unittest.TestCase):
    setUpClass=classmethod(group_tests.GroupHoldTests.setUpClass.__func__)
    tearDownClass=classmethod(group_tests.GroupHoldTests.tearDownClass.__func__)
    setUp=group_tests.GroupHoldTests.setUp
    tearDown=group_tests.GroupHoldTests.tearDown

    def port(self):
        self.lib.fake_target.restype=ctypes.c_float
        return group_tests.GroupHoldTests.port(self,smooth_jog=True)

    def test_capability_checked_before_any_motor_operations(self):
        self.bus.capabilities &= ~CAP_SMOOTH_JOG
        with patch.object(self.bus,'_request',wraps=self.bus._request) as request:
            with self.assertRaisesRegex(RuntimeError,'0x40000'):self.port()
            with self.assertRaisesRegex(RuntimeError,'0x40000'):
                self.bus.command(0,0,velocity=math.radians(5),smooth=True)
            request.assert_not_called()

    def test_selected_smoothing_and_immediate_release_pause_selection(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            self.assertEqual(port.report['base_info']['capabilities'],self.bus.capabilities)
            self.assertEqual(port.report['target_policy'],'mcu_slew_2ms')
            keys=ArmJogKeys(port,10);keys.press('h')
            fixed=dict(port.hold_targets)
            disables=[self.lib.fake_disables(i) for i in range(6)]
            self.lib.fake_position_gain(0,1.3)
            for action in ('release','pause','selection'):
                keys.press('1');keys.press('e');keys.press('right')
                for _ in range(30):self.lib.fake_advance(20);keys.tick()
                self.assertEqual([self.lib.fake_smoothing(i) for i in range(6)],[1,0,0,0,0,0])
                old_target=self.lib.fake_target(0)
                if action=='release':keys.release('right')
                elif action=='pause':keys.stop()  # Focus loss uses the same pause path.
                else:keys.press('4')
                captured=port.hold_targets[0]
                self.assertGreater(old_target-captured,.01)  # Would take >100 ms if slewed.
                self.assertAlmostEqual(self.lib.fake_target(0),captured,delta=2e-7)
                self.assertEqual([self.lib.fake_smoothing(i) for i in range(6)],[0]*6)
                self.assertEqual([self.lib.fake_disables(i) for i in range(6)],disables)
                for i in range(1,6):self.assertEqual(port.hold_targets[i],fixed[i])
                keys.release('right')
            port.stop()
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)

    def test_lagging_a2_keeps_requested_speed_both_directions(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            self.lib.fake_position_gain(1,1.3)
            port.hold_all();port.select(2);port.arm()
            for rate,direction in ((5,1),(5,-1),(10,1),(10,-1)):
                port.jog(direction,math.radians(rate));samples=[]
                for step in range(200):
                    self.lib.fake_advance(20);port.poll()
                    if step>=150:samples.append(port.report['motion_summary']['2']['last_sample'])
                first,last=samples[0],samples[-1]
                speed=direction*(last['position_rad']-first['position_rad'])/(last['at_monotonic_s']-first['at_monotonic_s'])
                self.assertAlmostEqual(math.degrees(speed),rate,delta=.15)
                self.assertFalse(any(s['lead_limited'] for s in samples))
                port.release()

    def test_old_five_degree_firmware_rejects_ten_before_any_enable(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            self.bus.capabilities &= ~CAP_SMOOTH_JOG_10
            with patch.object(self.bus,'_request',wraps=self.bus._request) as request:
                with self.assertRaisesRegex(RuntimeError,'0x80000'):port.hold_all(math.radians(10))
                with self.assertRaisesRegex(RuntimeError,'0x80000'):
                    self.bus.command(0,0,velocity=math.radians(10),smooth=True)
                request.assert_not_called()
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
            port.hold_all(math.radians(5));port.arm();port.jog(1,math.radians(5));port.release()

    def test_all_six_joints_reach_ten_and_nonselected_targets_stay_fixed(self):
        with patch.object(pos,'time',bench.time),self.port() as port:
            port.hold_all()
            self.assertEqual(port.speed,math.radians(10))
            for joint in range(1,7):
                port.select(joint);port.arm();fixed=dict(port.hold_targets)
                for direction in (1,-1):
                    port.jog(direction,math.radians(10));samples=[]
                    for step in range(45):
                        self.lib.fake_advance(20);port.poll()
                        if step>=30:samples.append(port.report['motion_summary'][str(joint)]['last_sample'])
                    first,last=samples[0],samples[-1]
                    speed=direction*(last['position_rad']-first['position_rad'])/(last['at_monotonic_s']-first['at_monotonic_s'])
                    self.assertAlmostEqual(math.degrees(speed),10,delta=.15)
                    self.assertAlmostEqual(math.degrees(port.report['hold_jog_max_lead_rad']),10)
                    self.assertTrue(all(s['owner_poll_ms']>0 for s in samples))
                    port.release()
                    for i in port.slots:
                        if i!=joint-1:self.assertEqual(port.hold_targets[i],fixed[i])
            with self.assertRaisesRegex(RuntimeError,'must not exceed 10'):port.jog(1,math.radians(10.1))
        self.assertEqual(set(port.report['bus_health_after_close']),{'1','2'})

    def test_smoothing_does_not_renew_host_lease_and_recovery_stays_disarmed(self):
        with patch.object(pos,'time',bench.time),self.port() as port,tempfile.TemporaryDirectory() as directory:
            # Every existing owner sample is retained for holding-mode A/B runs.
            self.assertEqual(port.report['motion_trace_interval_s'],0.)
            port.hold_all();port.arm();port.jog(1,math.radians(5))
            self.lib.fake_advance(170)
            with self.assertRaises(RuntimeError):port.poll()
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
            port.recover(Path(directory)/'fault.json')
            self.assertFalse(port.armed or port.moving or port.holding or port.hold_targets)
            self.assertEqual([self.lib.fake_smoothing(i) for i in range(6)],[0]*6)

    def test_cli_rejects_smoothing_without_hardware_hold_mode(self):
        for args in (['--smooth-jog'],['--execute','--smooth-jog'],
                     ['--simulate','--hold-all','--supported','--manual-travel','--smooth-jog']):
            with self.subTest(args=args),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                main(args)

    def test_cli_defaults_all_holding_joints_to_ten_and_accepts_explicit_five(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        for mode,extra,expected in (('--hold-all',[],10),('--hold-position',[],10),
                                     ('--hold-all',['--speed-deg-s','5'],5)):
            window=SimpleNamespace(show=lambda:None,keys=SimpleNamespace(failed=False),recovery_thread=None)
            with self.subTest(mode=mode,extra=extra),patch.object(app,'exec_',return_value=0), \
                 patch('teleop.arm_keyboard.make_window',return_value=window) as make:
                self.assertEqual(main(['--simulate','--supported','--manual-travel',mode]+extra),0)
                self.assertEqual(make.call_args.kwargs['speed_deg_s'],expected)


if __name__=='__main__':unittest.main()
