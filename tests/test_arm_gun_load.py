"""Expanded gun-load profile against the real native MC02 code and fake CAN rig."""
import source_paths
import ctypes
import json
import struct
import unittest
from unittest.mock import patch

from motors import arm_bench as bench, arm_position as position
from motors.arm import CAP_GUN_LOAD
from motors.arm_group_hold import ArmGroupHoldPort
import test_damiao_bench as routed


def gun_plan():
    return json.loads((routed.ROOT / 'src/robot_description/config/arm_rig_gun_a2_a6_load.json').read_text())


class GunLoadTests(unittest.TestCase):
    setUpClass = classmethod(routed.RoutedRoundTrip.setUpClass.__func__)
    tearDownClass = classmethod(routed.RoutedRoundTrip.tearDownClass.__func__)
    setUp = routed.RoutedRoundTrip.setUp
    tearDown = routed.RoutedRoundTrip.tearDown

    def fresh(self):
        self.bus.close()
        self.bus = routed.ArmBus(transport=routed.NativeTransport(self.lib))

    def port(self):
        self.lib.fake_torque_range.argtypes = [ctypes.c_uint, ctypes.c_float]
        plan = gun_plan()
        for i, motor in enumerate(plan['motors']):
            self.lib.fake_torque_range(i, motor['ranges']['tmax'])
        plan.update(fixture_confirmed=True, supported_confirmed=True)
        return ArmGroupHoldPort(plan, bus=self.bus, window_degrees=None)

    def test_explicit_profile_and_route_specific_ceilings(self):
        plan = gun_plan()
        bench.validate_plan(plan, all_motors=True)
        self.assertEqual([m['torque_trip_nm'] for m in plan['motors']], [20,30,30,10,10,10])
        self.assertFalse(plan['current_protection_verified'])
        self.assertIsNone(plan['torque_policy']['overload_duration_cutoff_s'])
        self.assertTrue(self.bus.capabilities & CAP_GUN_LOAD)
        for slot in range(2,6):
            good = plan['motors'][slot]
            for changes in ({'torque_trip_nm':good['torque_trip_nm']+.1},
                            {'bus':3}, {'can_id':1}, {'feedback_id':31}):
                self.fresh()
                motor = dict(good, **changes)
                with self.subTest(slot=slot, changes=changes):
                    with self.assertRaises(ValueError):
                        self.bus.prepare_routed_bench(slot, motor, -1, 1, 2)
                    with self.assertRaises(RuntimeError):
                        self.bus._request(16, struct.pack('<BBHHBB4f',slot,motor['bus'],
                            motor['can_id'],motor['feedback_id'],2 if slot==2 else 3,2,
                            -1,1,1.5,motor['torque_trip_nm']))
            self.bus.prepare_routed_bench(slot, good, -1, 1, 2)
            self.lib.fake_torque_limit.restype = ctypes.c_float
            self.assertEqual(self.lib.fake_torque_limit(slot),good['torque_trip_nm'])
        self.assertEqual(self.lib.fake_parameter_writes(),0)

    def test_old_a2_firmware_rejected_before_any_preflight_operations(self):
        self.bus.capabilities &= ~CAP_GUN_LOAD
        with patch.object(self.bus, '_request', wraps=self.bus._request) as request:
            with self.assertRaisesRegex(RuntimeError,'0x800000'):
                bench.preflight(self.bus,gun_plan()['motors'])
            for slot in range(2,6):
                with self.assertRaisesRegex(RuntimeError,'0x800000'):
                    self.bus.prepare_routed_bench(slot,gun_plan()['motors'][slot],-1,1,2)
            request.assert_not_called()

    def test_insufficient_actual_range_blocks_even_raw_firmware_enable(self):
        self.lib.fake_torque_range.argtypes = [ctypes.c_uint,ctypes.c_float]
        for slot in range(2,6):
            for tmax in ((28,30) if slot==2 else (10,)):
                with self.subTest(slot=slot,tmax=tmax):
                    self.fresh()
                    self.lib.fake_torque_range(slot,tmax)
                    plan=gun_plan()
                    plan['motors'][slot]['ranges']['tmax']=tmax
                    with self.assertRaisesRegex(ValueError,'TMAX must exceed'):
                        bench.validate_plan(plan,all_motors=True)
                    with self.assertRaisesRegex(ValueError,'TMAX must exceed'):
                        self.bus.prepare_routed_bench(slot,plan['motors'][slot],-1,1,2)
                    self.bus._request(16,struct.pack('<BBHHBB4f',slot,2,slot+1,slot+17,
                        2 if slot==2 else 3,1,-1,1,1.5,30 if slot==2 else 10))
                    self.bus.probe(slot)
                    self.lib.fake_advance(60)
                    self.assertFalse(self.bus.state(slot).registers & 4)
                    with self.assertRaises(RuntimeError): self.bus.enable(slot)
                    self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)

    def test_positive_and_negative_trips_latch_and_disable_all_six(self):
        for slot in range(2,6):
            limit=30 if slot==2 else 10
            for sign in (-1,1):
                with self.subTest(slot=slot,sign=sign):
                    self.fresh()
                    with patch.object(position,'time',bench.time), self.port() as port:
                        port.hold_all()
                        self.lib.fake_torque(slot,sign*(limit+.1))
                        self.lib.fake_advance(10)
                        with self.assertRaisesRegex(RuntimeError,'torque'): port.poll()
                        fault=self.bus.fault_snapshot()
                        self.assertEqual((fault['reason'],fault['slot'],fault['torque_limit_nm']),(9,slot,limit))
                        self.assertGreater(abs(fault['torque_nm']),limit)
                        self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)
                        self.assertTrue(port.recovery_required)

    def test_no_overload_timer_and_each_axis_records_torque_and_temperatures(self):
        with patch.object(position,'time',bench.time), self.port() as port:
            port.hold_all()
            writes=self.lib.fake_parameter_writes()
            for slot in range(1,6):
                self.lib.fake_torque(slot,29.5 if slot<3 else 9.5)
                self.lib.fake_temperature(slot,35+slot,40+slot)
            started=self.lib.fake_now()
            while self.lib.fake_now()-started<30000:
                self.lib.fake_advance(20)
                port.poll()
            for slot in range(1,6):
                sample=port.report['motion_summary'][str(slot+1)]['last_sample']
                self.assertAlmostEqual(sample['torque_feedback_nm'],29.5 if slot<3 else 9.5,delta=.02)
                self.assertEqual((sample['mos_c'],sample['rotor_c']),(35+slot,40+slot))
            self.assertEqual(self.lib.fake_parameter_writes(),writes)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)],[1]*6)

    def test_motor_and_mos_80c_trips_retained_on_all_new_axes(self):
        for slot in range(2,6):
            for mos,rotor in ((80,35),(35,80)):
                with self.subTest(slot=slot,mos=mos,rotor=rotor):
                    self.fresh()
                    with patch.object(position,'time',bench.time), self.port() as port:
                        port.hold_all()
                        self.lib.fake_temperature(slot,mos,rotor)
                        self.lib.fake_advance(10)
                        with self.assertRaises(RuntimeError): port.poll()
                        self.assertEqual(self.bus.fault_snapshot()['reason'],11)
                        self.assertEqual([self.lib.fake_status(i) for i in range(6)],[0]*6)


if __name__=='__main__': unittest.main()
