"""Camera launch contract checks; no physical camera is opened."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import source_paths  # noqa: F401

try:
    from launch import LaunchContext
    import ament_index_python
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(ROS_AVAILABLE, 'requires ROS 2 launch environment')
class CameraLaunchTests(unittest.TestCase):
    def test_profile_and_command_line_forwarding(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument, OpaqueFunction
        from stereo_cam.launch import camera_launch_description
        description = camera_launch_description(ROOT / 'src/bringup/config/stereo_cam.yaml')
        context = LaunchContext()
        context.launch_configurations.update(net_device_ip='192.0.2.2',
                                             enable_temporal_filter='true', camera_name='inspection')
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        include = next(entity for entity in description.entities if isinstance(entity, OpaqueFunction))
        with patch('stereo_cam.launch.get_package_share_directory', return_value='/tmp/orbbec_camera'), \
                patch('stereo_cam.launch.prepare_network') as prepare:
            actions = include.execute(context)
        prepare.assert_called_once_with(interface='auto', host_address='192.168.1.100/24',
                                        camera_ip='192.0.2.2')
        forwarded = dict(actions[0].launch_arguments)
        self.assertEqual(forwarded['camera_name'], 'inspection')
        self.assertEqual(forwarded['enumerate_net_device'], 'true')
        self.assertEqual(forwarded['net_device_ip'], '192.0.2.2')
        self.assertEqual(forwarded['enable_temporal_filter'], 'true')
        self.assertEqual(forwarded['enable_left_ir'], 'false')
        self.assertEqual(forwarded['depth_registration'], 'false')

    def test_scalar_values(self):
        from stereo_cam.launch import argument_value
        self.assertEqual(argument_value(False), 'false')
        self.assertEqual(argument_value('001234'), '001234')
        self.assertEqual(argument_value(30), '30')
        with self.assertRaises(ValueError):
            argument_value({'unsupported': 'nested config'})


if __name__ == '__main__':
    unittest.main()
