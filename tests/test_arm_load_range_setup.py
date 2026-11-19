"""Multi-drive disabled TMAX commissioning, persistence and uncertain-save handling."""
import source_paths
from copy import deepcopy
import ctypes
import struct
import unittest
from unittest.mock import patch

from motors import arm_bench as bench, arm_load_range_setup as setup
from motors.arm import CAP_LOAD_RANGE_SETUP
import test_damiao_bench as routed
from test_arm_a2_range_setup import baseline
from test_arm_gun_load import gun_plan


class LoadRangeSetupTests(unittest.TestCase):
    setUpClass=classmethod(routed.RoutedRoundTrip.setUpClass.__func__)
    tearDownClass=classmethod(routed.RoutedRoundTrip.tearDownClass.__func__)
    setUp=routed.RoutedRoundTrip.setUp
    tearDown=routed.RoutedRoundTrip.tearDown

    def fresh(self):
        self.bus.close()
        self.bus=routed.ArmBus(transport=routed.NativeTransport(self.lib))

    def run_setup(self, checkpoint=None):
        report={}
        evidence=[]
        with patch('motors.arm.time',bench.time):
            setup.configure_load_ranges(self.bus,baseline(),report,
                checkpoint or (lambda:evidence.append(deepcopy(report))))
        return report,evidence

    def session(self, slots=None):
        motors=baseline()['motors']
        observed=bench.preflight(self.bus,motors)
        session=bench.BenchSession(self.bus,motors,observed,{},slots=slots)
        session.probe_disabled()
        return session

    def test_all_ranges_persist_after_single_power_cycle_and_old_profile_rejected(self):
        report,evidence=self.run_setup()
        self.assertTrue(report['passed'] and report['disable_acknowledged'])
        self.assertFalse(report['persistence_verified'])
        self.assertIn('settings_before',evidence[0])
        self.assertEqual(self.lib.fake_parameter_writes(),5)
        self.assertEqual([self.lib.fake_saves(i) for i in range(6)],[0,1,1,1,1,1])
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)],[0]*6)
        self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
        self.assertEqual(self.lib.fake_invalid(),0)
        info=self.bus.base_info()
        for key in ('range_written_mask','range_save_attempted_mask','range_saved_mask'):
            self.assertEqual(info[key],62)
        self.assertEqual(info['range_setup_joint'],6)
        self.bus.reset_fault()
        for slot in range(6):
            with self.assertRaises(RuntimeError): self.bus.enable(slot)
            with self.assertRaises(RuntimeError): self.bus.command(slot,0,velocity=.05)
            with self.assertRaises(RuntimeError):
                self.bus.prepare_routed_bench(slot,baseline()['motors'][slot],-1,1,2)
        with self.assertRaisesRegex(RuntimeError,'power-cycle'):
            bench.preflight(self.bus,gun_plan()['motors'])
        transport=self.bus.serial
        self.bus.close()
        self.lib.fake_power_cycle()
        transport.closed=False
        self.bus=routed.ArmBus(transport=transport)
        self.assertEqual(self.bus.base_info()['range_saved_mask'],0)
        bench.preflight(self.bus,gun_plan()['motors'])
        with self.assertRaisesRegex(RuntimeError,'tmax changed'):
            bench.preflight(self.bus,baseline()['motors'])

    def test_existing_a2_40_is_skipped_then_all_already_set_never_write(self):
        self.lib.fake_torque_range.argtypes=[ctypes.c_uint,ctypes.c_float]
        self.lib.fake_torque_range(1,40)
        report,_=self.run_setup()
        self.assertTrue(report['steps'][0]['already_at_target'])
        self.assertEqual(self.lib.fake_parameter_writes(),4)
        self.assertEqual([self.lib.fake_saves(i) for i in range(6)],[0,0,1,1,1,1])
        self.fresh()
        for joint,target in setup.TARGETS.items(): self.lib.fake_torque_range(joint-1,target)
        report,_=self.run_setup()
        self.assertTrue(all(s['already_at_target'] for s in report['steps']))
        self.assertEqual(self.lib.fake_parameter_writes(),0)

    def test_capability_and_unknown_range_rejected_without_writes(self):
        self.bus.capabilities &= ~CAP_LOAD_RANGE_SETUP
        with patch.object(self.bus,'_request',wraps=self.bus._request) as request:
            with self.assertRaisesRegex(RuntimeError,'0x1000000'): self.run_setup()
            request.assert_not_called()
        self.fresh()
        self.lib.fake_torque_range.argtypes=[ctypes.c_uint,ctypes.c_float]
        self.lib.fake_torque_range(4,11)
        with self.assertRaisesRegex(RuntimeError,'A5 reports unexpected'): self.run_setup()
        self.assertEqual(self.lib.fake_parameter_writes(),0)

    def test_durable_evidence_failure_prevents_write(self):
        with self.assertRaisesRegex(OSError,'disk full'):
            self.run_setup(lambda:(_ for _ in ()).throw(OSError('disk full')))
        self.assertEqual(self.lib.fake_parameter_writes(),0)

    def test_wrong_joint_op_subset_stale_and_moving_feedback_rejected_by_mcu(self):
        self.session(slots=[2])
        with self.assertRaises(RuntimeError): self.bus._request(28,bytes([3,0]))
        self.fresh()
        session=self.session()
        for joint,op in ((0,0),(1,0),(2,0),(7,0),(3,2),(3,1)):
            with self.assertRaises(RuntimeError): self.bus._request(28,bytes([joint,op]))
        self.lib.fake_advance(101)
        with self.assertRaises(RuntimeError): self.bus._request(28,bytes([3,0]))
        session.stop()
        self.lib.fake_coast(5,100000)
        with self.assertRaises(TimeoutError): session.stop()
        with self.assertRaises(RuntimeError): self.bus._request(28,bytes([3,0]))
        self.assertEqual(self.lib.fake_parameter_writes(),0)

    def test_write_requires_readback_save_then_next_joint_and_cannot_repeat(self):
        session=self.session()
        self.bus._request(28,bytes([3,0]))
        with self.assertRaises(RuntimeError): self.bus._request(28,bytes([3,1]))
        with patch('motors.arm.time',bench.time):
            self.bus._verify_write(2,3,23,(19,0x42200000))
            session.stop()
            for payload in (bytes([4,0]),bytes([4,1]),bytes([3,0])):
                with self.assertRaises(RuntimeError): self.bus._request(28,payload)
            self.bus.setup_load_torque_range(3,save=True)
            session.stop()
            with self.assertRaises(RuntimeError): self.bus.setup_load_torque_range(3,save=True)
            with self.assertRaises(RuntimeError): self.bus.setup_load_torque_range(3)
            self.bus.setup_load_torque_range(4)
            session.stop()
            self.bus.setup_load_torque_range(4,save=True)
        self.assertEqual(self.bus.base_info()['range_saved_mask'],12)
        self.assertEqual(self.lib.fake_parameter_writes(),2)
        self.assertEqual(self.lib.fake_saves(2),1)
        self.assertEqual(self.lib.fake_saves(3),1)

    def test_ignored_write_and_lost_ack_stop_without_retry_or_later_joint_writes(self):
        for mode,error,saves in (('fake_ignore_tmax','readback mismatch',0),
                                 ('fake_drop_tmax_ack','not acknowledged',0),
                                 ('fake_drop_save','save unconfirmed',1)):
            self.fresh()
            getattr(self.lib,mode)(3)  # A4, after successful A2/A3 setup.
            with self.subTest(mode=mode), self.assertRaisesRegex(RuntimeError,error): self.run_setup()
            self.assertEqual(self.lib.fake_parameter_writes(),3)
            self.assertEqual([self.lib.fake_saves(i) for i in range(6)],[0,1,1,saves,0,0])
            self.assertEqual(self.bus.base_info()['range_saved_mask'],6)
            with self.assertRaises(RuntimeError): self.bus.setup_load_torque_range(5)
            self.assertEqual(self.lib.fake_invalid(),0)

    def test_other_motor_parameter_change_blocks_save_but_live_voltage_is_recorded(self):
        def change_parameter():
            # Before the first A2 write, change an A5 gain: full inventory comparison catches it.
            if self.lib.fake_parameter_writes()==0:
                change_parameter.calls+=1
                if change_parameter.calls==2:
                    self.lib.fake_register(4,27,struct.unpack('<I',struct.pack('<f',54))[0])
        change_parameter.calls=0
        with self.assertRaisesRegex(RuntimeError,'other than A2 TMAX changed'): self.run_setup(change_parameter)
        self.assertEqual(self.lib.fake_saves(1),0)
        self.fresh()
        counter=0
        def change_voltage():
            nonlocal counter
            counter+=1
            for slot in range(1,6):
                self.lib.fake_register(slot,60,struct.unpack('<I',struct.pack('<f',24+counter/100))[0])
        report,_=self.run_setup(change_voltage)
        self.assertTrue(report['passed'])
        self.assertNotEqual(report['settings_before']['control_raw'][5][60],
                            report['steps'][-1]['settings_after_save']['control_raw'][5][60])


if __name__=='__main__': unittest.main()
