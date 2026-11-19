"""A2 gun-load trip: native firmware/host agreement, group shutdown and cached evidence."""
import source_paths
import ctypes
import json
import struct
import unittest
from unittest.mock import patch

from motors import arm_bench as bench, arm_position as position
from motors.arm import CAP_A2_LOAD_30
from motors.arm_group_hold import ArmGroupHoldPort
import test_damiao_bench as routed


def gun_plan():
    return json.loads((routed.ROOT / 'src/robot_description/config/arm_rig_gun_a2_30nm.json').read_text())


class A2LoadTests(unittest.TestCase):
    setUpClass = classmethod(routed.RoutedRoundTrip.setUpClass.__func__)
    tearDownClass = classmethod(routed.RoutedRoundTrip.tearDownClass.__func__)
    setUp = routed.RoutedRoundTrip.setUp
    tearDown = routed.RoutedRoundTrip.tearDown

    def port(self):
        self.lib.fake_torque_range.argtypes = [ctypes.c_uint, ctypes.c_float]
        self.lib.fake_torque_range(1, 40)
        plan = gun_plan()
        plan.update(fixture_confirmed=True, supported_confirmed=True)
        return ArmGroupHoldPort(plan, bus=self.bus, window_degrees=None)

    def test_a2_30_requires_correct_route_and_slot_in_host_and_raw_firmware(self):
        bench.validate_plan(gun_plan(), all_motors=True)
        self.assertTrue(self.bus.capabilities & CAP_A2_LOAD_30)
        good = gun_plan()['motors'][1]
        for slot, updates in ((1, {'torque_trip_nm': 30.01}), (0, {}),
                              (2, {}),
                              (1, {'bus': 3}), (1, {'can_id': 3}), (1, {'feedback_id': 19})):
            motor = dict(good, **updates)
            with self.subTest(slot=slot, updates=updates):
                with self.assertRaises(ValueError):
                    self.bus.prepare_routed_bench(slot, motor, -1, 1, 2)
                # Bypass host validation: MCU independently enforces the same ceiling/route.
                with self.assertRaises(RuntimeError):
                    self.bus._request(16, struct.pack('<BBHHBB4f', slot, motor['bus'],
                        motor['can_id'], motor['feedback_id'], 2, 2, -1, 1, 1.5, motor['torque_trip_nm']))
        self.bus.prepare_routed_bench(1, good, -1, 1, 2)
        self.lib.fake_torque_limit.restype = ctypes.c_float
        self.assertEqual(self.lib.fake_torque_limit(1), 30)
        self.assertEqual(self.lib.fake_parameter_writes(), 0)

    def test_default_profile_and_other_joints_keep_existing_trips(self):
        original = json.loads((routed.ROOT / 'src/robot_description/config/arm_rig_a1_a6.json').read_text())
        self.assertEqual([m['torque_trip_nm'] for m in original['motors']], [20, 12, 12, 3.5, 3.5, 3.5])
        changed = gun_plan()
        changed['motors'][1]['torque_trip_nm'] = 12
        changed['motors'][1]['ranges']['tmax'] = 28
        self.assertEqual(changed['motors'], original['motors'])
        self.assertFalse(gun_plan()['current_protection_verified'])
        bad = gun_plan()
        bad['motors'][0]['torque_trip_nm'] += .1
        with self.assertRaises(ValueError):
            bench.validate_plan(bad, all_motors=True)

    def test_old_firmware_rejected_before_preflight_operations(self):
        self.bus.capabilities &= ~CAP_A2_LOAD_30
        with patch.object(self.bus, '_request', wraps=self.bus._request) as request:
            with self.assertRaisesRegex(RuntimeError, '0x200000'):
                bench.preflight(self.bus, gun_plan()['motors'])
            with self.assertRaisesRegex(RuntimeError, '0x200000'):
                self.bus.prepare_routed_bench(1, gun_plan()['motors'][1], -1, 1, 2)
            request.assert_not_called()
        self.assertFalse(self.bus.configured)

    def test_sustained_above_rated_feedback_has_no_new_timer_and_is_recorded(self):
        with patch.object(position, 'time', bench.time), self.port() as port:
            port.hold_all()
            writes = self.lib.fake_parameter_writes()
            self.lib.fake_torque(1, 29.5)
            self.lib.fake_temperature(1, 36, 42)
            started = self.lib.fake_now()
            while self.lib.fake_now()-started < 30000:
                self.lib.fake_advance(20)
                port.poll()
            sample = port.report['motion_summary']['2']['last_sample']
            self.assertAlmostEqual(sample['torque_feedback_nm'], 29.5, delta=.02)
            self.assertEqual((sample['mos_c'], sample['rotor_c']), (36, 42))
            self.assertEqual(sample['torque_trip_nm'], 30)
            self.assertEqual(self.lib.fake_parameter_writes(), writes)
            self.assertEqual([self.lib.fake_status(i) for i in range(6)], [1]*6)
            self.assertFalse(port.recovery_required)

    def test_saved_28_range_cannot_use_30_profile_before_any_motor_write(self):
        with self.assertRaisesRegex(RuntimeError, 'tmax changed'):
            bench.preflight(self.bus, gun_plan()['motors'])
        self.assertEqual(self.lib.fake_parameter_writes(), 0)
        self.assertEqual([self.lib.fake_status(i) for i in range(6)], [0]*6)

    def test_trip_at_or_above_range_rejected_by_host_and_firmware_probe(self):
        self.lib.fake_torque_range.argtypes = [ctypes.c_uint, ctypes.c_float]
        for tmax in (28, 30):
            with self.subTest(tmax=tmax):
                self.bus.close()
                self.bus = routed.ArmBus(transport=routed.NativeTransport(self.lib))
                self.lib.fake_torque_range(1, tmax)
                plan = gun_plan()
                plan['motors'][1]['ranges']['tmax'] = tmax
                with self.assertRaisesRegex(ValueError, 'TMAX must exceed'):
                    bench.validate_plan(plan, all_motors=True)
                with self.assertRaisesRegex(ValueError, 'TMAX must exceed'):
                    self.bus.prepare_routed_bench(1, plan['motors'][1], -1, 1, 2)
                # Raw MCU setup has no claimed host range: it must discover and
                # reject the actual insufficient range, even if the host lies.
                self.bus._request(16, struct.pack('<BBHHBB4f', 1, 2, 2, 18, 2, 1,
                                                 -1, 1, 1.5, 30))
                self.bus.probe(1)
                self.lib.fake_advance(60)
                self.assertFalse(self.bus.state(1).registers & 4)
                with self.assertRaises(RuntimeError):
                    self.bus.enable(1)
                self.assertEqual([self.lib.fake_status(i) for i in range(6)], [0]*6)

    def test_positive_and_negative_over_trip_disable_group_and_latch_snapshot(self):
        for torque in (30.1, -30.1):
            with self.subTest(torque=torque):
                # NativeTransport creates a fresh controller/plant for each case.
                self.bus.close()
                self.bus = routed.ArmBus(transport=routed.NativeTransport(self.lib))
                with patch.object(position, 'time', bench.time), self.port() as port:
                    port.hold_all()
                    self.lib.fake_torque(1, torque)
                    self.lib.fake_advance(10)
                    with self.assertRaisesRegex(RuntimeError, 'torque'):
                        port.poll()
                    fault = self.bus.fault_snapshot()
                    self.assertEqual((fault['reason'], fault['slot'], fault['torque_limit_nm']), (9, 1, 30))
                    self.assertGreater(abs(fault['torque_nm']), 30)
                    self.assertEqual([self.lib.fake_status(i) for i in range(6)], [0]*6)
                    self.assertTrue(port.recovery_required)

    def test_both_80_degree_temperature_trips_remain_independent(self):
        for mos, rotor in ((80, 35), (35, 80)):
            with self.subTest(mos=mos, rotor=rotor):
                self.bus.close()
                self.bus = routed.ArmBus(transport=routed.NativeTransport(self.lib))
                with patch.object(position, 'time', bench.time), self.port() as port:
                    port.hold_all()
                    self.lib.fake_torque(1, 29.5)
                    self.lib.fake_temperature(1, mos, rotor)
                    self.lib.fake_advance(10)
                    with self.assertRaises(RuntimeError):
                        port.poll()
                    self.assertEqual(self.bus.fault_snapshot()['reason'], 11)
                    self.assertEqual([self.lib.fake_status(i) for i in range(6)], [0]*6)


if __name__ == '__main__':
    unittest.main()
