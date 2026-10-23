"""Observe an existing RGB ROS stream; never starts camera acquisition or motors."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    profile = str(Path(get_package_share_directory('bringup'))/'config'/'seam_edges.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=profile),
        DeclareLaunchArgument('show_video', default_value='true'),
        DeclareLaunchArgument('input_topic', default_value='/stereo_cam/color/image_raw'),
        Node(package='seam_detection', executable='seam_edges', name='seam_edges', output='screen',
             parameters=[LaunchConfiguration('config'), {
                 'show_video': ParameterValue(LaunchConfiguration('show_video'), value_type=bool),
                 'input_topic': ParameterValue(LaunchConfiguration('input_topic'), value_type=str)}],
             additional_env={'QT_QPA_PLATFORM': 'xcb'}),
    ])
