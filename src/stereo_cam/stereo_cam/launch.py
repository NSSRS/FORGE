"""Include the vendor driver; FORGE owns only the profile and orchestration."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from stereo_cam.network import prepare_network


def argument_value(value):
    if isinstance(value, bool):
        return str(value).lower()
    if not isinstance(value, (str, int, float)):
        raise ValueError('Camera profile values must be scalar launch arguments')
    return str(value)


def camera_launch_description(profile):
    defaults = yaml.safe_load(Path(profile).read_text())
    if not isinstance(defaults, dict) or not defaults:
        raise ValueError('Camera profile must be a nonempty mapping')
    arguments = [DeclareLaunchArgument(key, default_value=argument_value(value))
                 for key, value in defaults.items()]

    arguments.extend([
        DeclareLaunchArgument('prepare_camera_network', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('camera_interface', default_value='auto'),
        DeclareLaunchArgument('camera_ip', default_value='192.168.1.10'),
        DeclareLaunchArgument('camera_host_address', default_value='192.168.1.100/24'),
    ])

    def include_driver(context):
        try:
            share = get_package_share_directory('orbbec_camera')
        except LookupError as exc:
            raise RuntimeError('Install the Orbbec ROS 2 v2 driver; see docs/STEREO_CAM_SETUP.md') from exc
        values = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        if LaunchConfiguration('prepare_camera_network').perform(context) == 'true':
            prepare_network(
                interface=LaunchConfiguration('camera_interface').perform(context),
                host_address=LaunchConfiguration('camera_host_address').perform(context),
                camera_ip=values['net_device_ip'] or LaunchConfiguration('camera_ip').perform(context),
            )
        # Keep network discovery enabled for Gemini Ethernet devices. A direct
        # legacy TCP open can misidentify a Gemini as a Femto Mega.
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(share) / 'launch' / 'gemini_330_series.launch.py')),
            launch_arguments=values.items(),
        )]

    return LaunchDescription(arguments + [OpaqueFunction(function=include_driver)])
