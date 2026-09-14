"""Exercise the real C thread and ctypes ABI against a test-only fake bridge."""
import ctypes
import os
import time
import unittest
from forge_motors import MotorBus, MotorConfig, MotorError


@unittest.skipUnless(os.environ.get("ENCOS_TEST_LIBRARY"), "requires C fake-bridge test library")
class NativeTests(unittest.TestCase):
    def setUp(self):
        self.path = os.environ["ENCOS_TEST_LIBRARY"]
        self.fake = ctypes.CDLL(self.path)
        self.fake.fake_fault.argtypes = [ctypes.c_int]
        self.configs = [MotorConfig(i, -180, 180, 5, 1, 10) for i in (1, 2)]
        self.bus = MotorBus(self.configs, interface="forge-test", execute=True, library=self.path)
        self.addCleanup(self.bus.close)
        self.bus.wait_ready()

    def test_query_only_and_exclusive_owner(self):
        self.assertEqual(self.fake.fake_controls(), 0)
        self.assertIsNone(self.bus.state(1).current_a)
        with self.assertRaises(MotorError):
            MotorBus(self.configs, interface="forge-test", execute=True, library=self.path)

    def test_bad_layout_rejected_and_owner_released(self):
        self.bus.close()
        with self.assertRaises(MotorError):
            MotorBus(self.configs, interface="forge-test-bad-layout", execute=True, library=self.path)
        with MotorBus(self.configs, interface="forge-test", execute=True, library=self.path) as b:
            b.wait_ready()

    def test_native_validation_cannot_be_bypassed_by_ctypes(self):
        native = self.bus._backend
        for motor_id, mode, target in [(1, 2, 6), (1, 1, 181), (1, 0, 0),
                                      (1, 2, float("nan")), (9, 2, 0)]:
            result = native.lib.encos_driver_command(native.handle, motor_id, mode, target)
            self.assertEqual(result, -1)
        self.assertEqual(self.fake.fake_controls(), 0)

    def test_independent_commands_and_feedback(self):
        for _ in range(8):
            self.bus.velocity(1, -2)
            self.bus.position(2, 15)
            time.sleep(.03)
        self.assertLess(self.bus.state(1).velocity_rpm, 0)
        self.assertEqual(self.bus.state(2).position_deg, 15)
        self.assertEqual(self.bus.state(1).temperature_c, 25)

    def test_watchdog_runs_without_python_heartbeat(self):
        self.bus.velocity(1, 2)
        time.sleep(.35)
        self.assertEqual(self.bus._backend.status(), 1)
        self.assertGreater(self.fake.fake_zeros(), 0)
        with self.assertRaises(MotorError): self.bus.velocity(1, 2)

    def test_renewing_one_does_not_renew_other(self):
        self.bus.velocity(2, 1)
        for _ in range(4):
            self.bus.velocity(1, 1)
            time.sleep(.06)
        time.sleep(.06)
        self.assertEqual(self.bus._backend.status(), 1)

    def test_transport_failure_latches(self):
        self.bus.velocity(1, 1)
        self.fake.fake_fault(1)
        time.sleep(.04)
        self.assertEqual(self.bus._backend.status(), 2)
        self.fake.fake_fault(0)
        with self.assertRaises(MotorError): self.bus.velocity(1, 1)

    def test_missing_feedback_latches(self):
        self.bus.velocity(1, 1)
        self.fake.fake_fault(2)
        for _ in range(4):
            self.bus.velocity(1, 1)
            time.sleep(.06)
        time.sleep(.06)
        self.assertEqual(self.bus._backend.status(), 3)

    def test_motor_fault_stops_all(self):
        self.bus.velocity(1, 1); self.bus.velocity(2, -1)
        self.fake.fake_fault(3)
        time.sleep(.05)
        self.assertEqual(self.bus._backend.status(), 3)
        self.assertGreater(self.fake.fake_zeros(), 0)

    def test_close_sends_zero_then_clears_and_releases(self):
        self.bus.velocity(1, 1)
        self.bus.close()
        self.assertGreater(self.fake.fake_zeros(), 0)
        self.assertGreaterEqual(self.fake.fake_clears(), 5)
        with MotorBus(self.configs, interface="forge-test", execute=True, library=self.path) as next_bus:
            next_bus.wait_ready()


if __name__ == "__main__": unittest.main()
