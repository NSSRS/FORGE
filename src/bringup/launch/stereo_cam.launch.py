"""Camera-only bringup with an optional RGB video window."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from stereo_cam.launch import camera_launch_description


def generate_launch_description():
    description = camera_launch_description(
        Path(get_package_share_directory('bringup')) / 'config' / 'stereo_cam.yaml')
    description.add_action(DeclareLaunchArgument(
        'show_video', default_value='false',
        description='Open the RGB viewer on the desktop'))
    description.add_action(Node(
        package='stereo_cam', executable='view_camera',
        name='stereo_cam_viewer', output='screen',
        arguments=['--topic', ['/', LaunchConfiguration('camera_name'), '/color/image_raw']],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        condition=IfCondition(LaunchConfiguration('show_video')),
    ))
    return description
