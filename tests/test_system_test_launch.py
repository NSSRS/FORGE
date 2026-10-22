"""Full-test configuration guards; these tests never open motor hardware."""
import importlib.util
import unittest
import source_paths  # noqa: F401

ROS_AVAILABLE = importlib.util.find_spec('launch_ros') is not None


@unittest.skipUnless(ROS_AVAILABLE, 'requires ROS 2 launch')
class SystemTestOptions(unittest.TestCase):
    def options(self):
        return dict(simulate=True, execute=False, mapping_confirmed=False,
                    radio_failsafe_confirmed=False, use_imu=True, use_radio=True,
                    camera_interface='enx000ec8a22028', motor_interface='enp86s0',
                    imu_port='/dev/ttyUSB0', radio_port='/dev/ttyUSB1')

    def validate(self, **overrides):
        from bringup.system_test import validate_options
        options = self.options()
        options.update(overrides)
        validate_options(options)

    def test_simulated_motors_allowed(self):
        self.validate()

    def test_each_hardware_confirmation_required(self):
        confirmations = dict(simulate=False, execute=True, mapping_confirmed=True,
                             radio_failsafe_confirmed=True)
        self.validate(**confirmations)
        for flag in ('execute', 'mapping_confirmed', 'radio_failsafe_confirmed'):
            with self.assertRaises(ValueError):
                self.validate(**{**confirmations, flag: False})

    def test_shared_network_interface_rejected(self):
        with self.assertRaises(ValueError):
            self.validate(camera_interface='enp86s0')

    def test_missing_or_shared_serial_ports_rejected(self):
        for overrides in ({'imu_port': ''}, {'radio_port': ''},
                          {'radio_port': '/dev/ttyUSB0'}):
            with self.assertRaises(ValueError):
                self.validate(**overrides)

    def test_disabled_devices_do_not_require_ports(self):
        self.validate(use_imu=False, use_radio=False, imu_port='', radio_port='')
