import math
import unittest
from unittest.mock import patch
from forge_motors import MotorBus, MotorConfig, MotorError, _Sim


class Clock:
    def __init__(self): self.now = 10.
    def __call__(self): return self.now
    def advance(self, seconds): self.now += seconds


def config(motor_id=1):
    return MotorConfig(motor_id, -180, 180, 5, 1)


class MotorTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.bus = MotorBus([config(1), config(2)], simulate=True)
        self.bus._backend = _Sim([config(1), config(2)], self.clock)
        self.addCleanup(self.bus.close)

    def test_independent_position_velocity_and_sign(self):
        for _ in range(10):
            self.bus.velocity(1, -2)
            self.bus.position(2, 15)
            self.clock.advance(.05)
        one, two = self.bus.state(1), self.bus.state(2)
        self.assertLess(one.position_deg, 0)
        self.assertAlmostEqual(one.velocity_rpm, -2)
        self.assertGreater(two.position_deg, 0)

    def test_each_motor_has_its_own_deadline(self):
        self.bus.velocity(1, 2)
        self.bus.velocity(2, 1)
        self.clock.advance(.2)
        self.bus.velocity(1, 2)
        self.clock.advance(.06)
        with self.assertRaisesRegex(MotorError, "deadline"):
            self.bus.check()
        with self.assertRaises(MotorError):
            self.bus.velocity(2, 0)
        self.assertEqual(self.bus.state(1).velocity_rpm, 0)

    def test_late_command_cannot_revive_session(self):
        self.bus.position(1, 10)
        self.clock.advance(.3)
        with self.assertRaises(MotorError): self.bus.position(1, 10)

    def test_invalid_commands_do_not_activate(self):
        for target in [math.nan, math.inf, -math.inf, 6, -6]:
            with self.assertRaises(ValueError): self.bus.velocity(1, target)
        with self.assertRaises(ValueError): self.bus.position(1, 181)
        with self.assertRaises(ValueError): self.bus.velocity(3, 0)
        with self.assertRaises(ValueError): self.bus.velocity(True, 0)
        self.assertFalse(self.bus._active)

    def test_query_only_open_and_stop_does_not_activate(self):
        self.bus.wait_ready()
        self.bus.stop()
        self.clock.advance(1)
        self.bus.check()
        self.assertFalse(self.bus._backend.commands)

    def test_velocity_and_stop_use_protective_slew(self):
        self.bus.velocity(1, 5)
        self.clock.advance(.05)
        self.assertAlmostEqual(self.bus.state(1).velocity_rpm, 3, places=5)
        self.bus.stop()
        self.clock.advance(.1)
        self.assertAlmostEqual(self.bus.state(1).velocity_rpm, 0, places=5)

    def test_velocity_allows_continuous_position(self):
        self.bus._backend.positions[1] = 179.99
        self.bus.velocity(1, 5)
        self.clock.advance(.1)
        self.bus.check()
        self.assertGreater(self.bus.state(1).position_deg, 180)

    def test_position_mode_retains_position_limit(self):
        self.bus.position(1, 179)
        self.bus._backend.positions[1] = 181
        self.clock.advance(.01)
        with self.assertRaisesRegex(MotorError, "limit"): self.bus.check()

    def test_close_is_idempotent_and_final(self):
        self.bus.close(); self.bus.close()
        with self.assertRaises(MotorError): self.bus.velocity(1, 0)
        with self.assertRaises(MotorError): self.bus.state(1)

    def test_context_cleanup_on_exception(self):
        b = MotorBus([config()], simulate=True)
        with self.assertRaises(RuntimeError):
            with b: raise RuntimeError("application failure")
        self.assertTrue(b._closed)

    def test_hardware_requires_explicit_execution(self):
        with patch("forge_motors._Native") as native:
            with self.assertRaises(ValueError): MotorBus([config()], interface="eth0")
            with self.assertRaises(ValueError): MotorBus([config()], execute=True)
            native.assert_not_called()

    def test_configuration_validation(self):
        with self.assertRaises(ValueError): MotorBus([config(), config()], simulate=True)
        with self.assertRaises(ValueError): MotorBus([], simulate=True)
        with self.assertRaises(ValueError): MotorConfig(1, 2, 1, 5, 1)
        with self.assertRaises(ValueError): MotorConfig(1, -180, 180, 0, 1)
        with self.assertRaises(ValueError): MotorConfig(1, -180, 180, 5, math.nan)
        with self.assertRaises(ValueError): MotorConfig(1, -1e100, 180, 5, 1)


if __name__ == "__main__": unittest.main()
