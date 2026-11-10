"""Full-test configuration guards; these tests never open motor hardware."""
import importlib.util
import unittest
from unittest.mock import patch
import source_paths  # noqa: F401

ROS_AVAILABLE = importlib.util.find_spec('launch_ros') is not None


@unittest.skipUnless(ROS_AVAILABLE, 'requires ROS 2 launch')
class SystemTestOptions(unittest.TestCase):
    def test_auto_radio_excludes_configured_imu_and_arm_even_when_disabled(self):
        from bringup.system_test import resolve_serial_options
        options = dict(self.options(), radio_port='auto', use_imu=False,
                       arm_execute=False, arm_port='/dev/ttyACM0')
        with patch('bringup.system_test.resolve_radio_port', return_value='/dev/ttyUSB7') as resolve:
            resolve_serial_options(options)
        resolve.assert_called_once_with('auto', exclude=('/dev/ttyUSB0', '/dev/ttyACM0'))
        self.assertEqual(options['radio_port'], '/dev/ttyUSB7')

    def test_discovery_cannot_bypass_device_separation(self):
        from bringup.system_test import resolve_serial_options
        options = dict(self.options(), radio_port='auto')
        with patch('bringup.system_test.resolve_radio_port', return_value='/dev/ttyUSB0'):
            with self.assertRaisesRegex(ValueError, 'different serial'):
                resolve_serial_options(options)

    def test_invalid_hardware_authorization_fails_before_discovery(self):
        from bringup.system_test import resolve_serial_options
        with patch('bringup.system_test.resolve_radio_port') as resolve:
            with self.assertRaises(ValueError):
                resolve_serial_options(dict(self.options(), simulate=False, radio_port='auto'))
        resolve.assert_not_called()

    def test_disabled_radio_does_not_discover(self):
        from bringup.system_test import resolve_serial_options
        with patch('bringup.system_test.resolve_radio_port') as resolve:
            resolve_serial_options(dict(self.options(), use_radio=False, radio_port='auto'))
        resolve.assert_not_called()

    def test_supported_arm_is_explicit_and_distinct_from_unloaded(self):
        flags=dict(use_arm=True,arm_execute=True,arm_supported=True,
                   arm_port='/dev/ttyACM0',arm_plan='arm_rig_a2_a6.json')
        self.validate(**flags)
        with self.assertRaises(ValueError):self.validate(**flags,arm_unloaded=True)

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

    def test_arm_requires_explicit_unloaded_and_distinct_port(self):
        flags=dict(use_arm=True,arm_execute=True,arm_unloaded=True,
                   arm_port='/dev/ttyACM0',arm_plan='arm_jog.json')
        self.validate(**flags)
        for override in ({'arm_unloaded':False},{'use_arm':False},{'use_radio':False},
                         {'arm_port':'/dev/ttyUSB0'},{'arm_port':'/dev/ttyUSB1'},
                         {'arm_plan':''}):
            with self.assertRaises(ValueError):self.validate(**{**flags,**override})
