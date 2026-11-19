"""MC02/host commissioning contract with a native firmware and independent fake rig."""
import source_paths
from copy import deepcopy
import ctypes
import contextlib
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from motors import arm_bench as bench, arm_a2_range_setup as setup
from motors.arm import CAP_A2_RANGE_SETUP, decode_register
import test_damiao_bench as routed


def baseline():
    return json.loads((routed.ROOT / 'src/robot_description/config/arm_rig_a1_a6.json').read_text())


class A2RangeSetupTests(unittest.TestCase):
    setUpClass = classmethod(routed.RoutedRoundTrip.setUpClass.__func__)
    tearDownClass = classmethod(routed.RoutedRoundTrip.tearDownClass.__func__)
    setUp = routed.RoutedRoundTrip.setUp
    tearDown = routed.RoutedRoundTrip.tearDown

    def run_setup(self, checkpoint=None):
        report = {}
        evidence = []
        with patch('motors.arm.time', bench.time):
            setup.configure_a2_range(self.bus, baseline(), report,
                                     checkpoint or (lambda: evidence.append(deepcopy(report))))
        return report, evidence

    def disabled_session(self, slots=None):
        motors = baseline()['motors']
        observed = bench.preflight(self.bus, motors)
        session = bench.BenchSession(self.bus, motors, observed, {}, slots=slots)
        session.probe_disabled()
        return session

    def test_write_verify_save_persist_and_keep_motion_locked_until_power_cycle(self):
        report, evidence = self.run_setup()
        self.assertTrue(report['passed'] and report['save_acknowledged'])
        self.assertFalse(report['persistence_verified'])
        self.assertEqual(report['a2_tmax_before_nm'], 28)
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)], [0]*6)
        self.assertEqual([self.lib.fake_status(i) for i in range(6)], [0]*6)
        self.assertEqual([self.lib.fake_saves(i) for i in range(6)], [0,1,0,0,0,0])
        self.assertEqual(self.lib.fake_parameter_writes(), 1)
        self.assertEqual(self.lib.fake_invalid(), 0)  # No CAN traffic during save quiet time.
        self.assertIn('settings_before', evidence[0])
        self.assertEqual(report['settings_before']['a2_control_raw'], report['settings_after_save']['a2_control_raw'])
        self.assertEqual(self.bus.base_info()['a2_range_setup_flags'], 7)
        with self.assertRaisesRegex(RuntimeError, 'power-cycle'):
            bench.preflight(self.bus, baseline()['motors'])
        with self.assertRaises(RuntimeError):
            self.bus.setup_a2_torque_range(save=True)
        self.bus.reset_fault()  # Fault reset cannot unlock motion or permit reconfiguration.
        with self.assertRaises(RuntimeError):
            self.bus.command(1, 0, velocity=.05)
        with self.assertRaises(RuntimeError):
            self.bus.enable(1)
        with self.assertRaises(RuntimeError):
            self.bus.prepare_routed_bench(1, baseline()['motors'][1], -1, 1, 2)
        transport = self.bus.serial
        self.bus.close()
        self.lib.fake_power_cycle()
        transport.closed = False
        self.bus = routed.ArmBus(transport=transport)
        self.assertEqual(decode_register(23, self.bus.read_register(2, 2, 23)[1]), 40)
        self.assertEqual(self.bus.base_info()['a2_range_setup_flags'], 0)
        plan = baseline()
        plan['motors'][1]['ranges']['tmax'] = 40
        plan['motors'][1]['torque_trip_nm'] = 30
        bench.preflight(self.bus, plan['motors'])

    def test_old_firmware_rejected_before_any_operations(self):
        self.bus.capabilities &= ~CAP_A2_RANGE_SETUP
        with patch.object(self.bus, '_request', wraps=self.bus._request) as request:
            with self.assertRaisesRegex(RuntimeError, '0x400000'):
                self.run_setup()
            request.assert_not_called()

    def test_evidence_failure_prevents_motor_write(self):
        with self.assertRaisesRegex(OSError, 'disk full'):
            self.run_setup(lambda: (_ for _ in ()).throw(OSError('disk full')))
        self.assertEqual(self.lib.fake_parameter_writes(), 0)
        self.assertEqual(self.lib.fake_saves(1), 0)

    def test_subset_and_stale_or_moving_other_joint_block_raw_mcu_write(self):
        session = self.disabled_session(slots=[1])
        with self.assertRaises(RuntimeError):
            self.bus._request(27, b'\0')
        self.assertEqual(self.lib.fake_parameter_writes(), 0)
        self.bus.close()
        self.bus = routed.ArmBus(transport=routed.NativeTransport(self.lib))
        session = self.disabled_session()
        self.lib.fake_advance(101)
        with self.assertRaises(RuntimeError):
            self.bus._request(27, b'\0')
        session.stop()
        self.lib.fake_coast(4, 100000)
        with self.assertRaises(TimeoutError):
            session.stop()
        with self.assertRaises(RuntimeError):
            self.bus._request(27, b'\0')
        self.assertEqual(self.lib.fake_parameter_writes(), 0)

    def test_save_requires_independent_readback_and_write_cannot_repeat(self):
        session = self.disabled_session()
        self.bus._request(27, b'\0')
        with self.assertRaises(RuntimeError):
            self.bus._request(27, b'\1')
        self.assertEqual(self.lib.fake_saves(1), 0)
        with patch('motors.arm.time', bench.time):
            self.bus._verify_write(2, 2, 23, (18, 0x42200000))
        session.stop()
        with self.assertRaises(RuntimeError):
            self.bus._request(27, b'\0')
        self.assertEqual(self.lib.fake_parameter_writes(), 1)

    def test_ignored_write_never_saves_or_unlocks(self):
        self.lib.fake_ignore_tmax(1)
        with self.assertRaisesRegex(RuntimeError, 'readback mismatch'):
            self.run_setup()
        self.assertEqual(self.lib.fake_saves(1), 0)
        self.assertEqual(self.bus.base_info()['a2_range_setup_flags'], 1)

    def test_missing_write_ack_never_retries_or_saves(self):
        self.lib.fake_drop_tmax_ack(1)
        with self.assertRaisesRegex(RuntimeError, 'not acknowledged'):
            self.run_setup()
        self.assertEqual(self.lib.fake_parameter_writes(), 1)
        self.assertEqual(self.lib.fake_saves(1), 0)

    def test_missing_save_ack_never_retries_flash_write(self):
        self.lib.fake_drop_save(1)
        with self.assertRaisesRegex(RuntimeError, 'save unconfirmed'):
            self.run_setup()
        self.assertEqual(self.lib.fake_saves(1), 1)
        self.assertEqual(self.lib.fake_invalid(), 0)

    def test_already_40_does_not_write_or_save(self):
        self.lib.fake_torque_range.argtypes = [ctypes.c_uint, ctypes.c_float]
        self.lib.fake_torque_range(1, 40)
        report, _ = self.run_setup()
        self.assertTrue(report['already_at_40'])
        self.assertEqual(self.lib.fake_parameter_writes(), 0)
        self.assertEqual(self.lib.fake_saves(1), 0)

    def test_unexpected_range_stops_without_writing(self):
        self.lib.fake_torque_range.argtypes = [ctypes.c_uint, ctypes.c_float]
        self.lib.fake_torque_range(1, 32)
        with self.assertRaisesRegex(RuntimeError, 'unexpected TMAX=32'):
            self.run_setup()
        self.assertEqual(self.lib.fake_parameter_writes(), 0)

    def test_other_parameter_change_prevents_save(self):
        calls = 0
        def checkpoint():
            nonlocal calls
            calls += 1
            if calls == 2:
                self.lib.fake_register(1, 27, struct.unpack('<I', struct.pack('<f', 54))[0])
        with self.assertRaisesRegex(RuntimeError, 'other than A2 TMAX changed'):
            self.run_setup(checkpoint)
        self.assertEqual(self.lib.fake_saves(1), 0)

    def test_live_bus_voltage_can_change_during_write_and_save_and_remains_recorded(self):
        # First two readings reproduce the physical report's normal VBus change.
        voltages = (1103250176, 1103236237, struct.unpack('<I', struct.pack('<f', 24.3))[0])
        self.lib.fake_register(1, 60, voltages[0])
        calls = 0
        def checkpoint():
            nonlocal calls
            calls += 1
            if calls in (2, 3):
                self.lib.fake_register(1, 60, voltages[calls - 1])
        report, _ = self.run_setup(checkpoint)
        self.assertTrue(report['passed'] and report['save_acknowledged'])
        self.assertEqual(self.lib.fake_saves(1), 1)
        self.assertEqual(self.lib.fake_parameter_writes(), 1)
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)], [0]*6)
        for phase, voltage in zip(('before', 'after_write', 'after_save'), voltages):
            self.assertEqual(report[f'settings_{phase}']['a2_control_raw'][60], voltage)

    def test_other_parameter_change_after_save_still_fails(self):
        calls = 0
        def checkpoint():
            nonlocal calls
            calls += 1
            if calls == 3:
                self.lib.fake_register(1, 27, struct.unpack('<I', struct.pack('<f', 54))[0])
        with self.assertRaisesRegex(RuntimeError, 'Post-save settings mismatch'):
            self.run_setup(checkpoint)
        self.assertEqual(self.lib.fake_saves(1), 1)

    def test_cli_requires_both_explicit_flags_before_opening_port(self):
        for flag in ('--write', '--supported'):
            with self.subTest(flag=flag), patch.object(setup, 'ArmBus') as factory:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    setup.main([flag, '--report', '/tmp/not-created-a2-setup.json'])
                factory.assert_not_called()

    def test_cli_saves_evidence_and_never_overwrites_an_existing_report(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'setup.json'
            with patch.object(setup, 'ArmBus', return_value=self.bus), patch('motors.arm.time', bench.time):
                self.assertEqual(setup.main(['--supported', '--write', '--report', str(path)]), 0)
            report = json.loads(path.read_text())
            self.assertTrue(report['passed'] and report['save_acknowledged'])
            self.assertFalse(report['persistence_verified'] or report['motor_enabled_by_setup'])
            with patch.object(setup, 'ArmBus') as factory, self.assertRaises(FileExistsError):
                setup.main(['--supported', '--write', '--report', str(path)])
            factory.assert_not_called()


if __name__ == '__main__':
    unittest.main()
