from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('execute',default_value='false'),
        DeclareLaunchArgument('port',default_value=''),
        DeclareLaunchArgument('profile',default_value=''),
        Node(package='motor_control',executable='arm_driver',output='screen',parameters=[{
            'execute':ParameterValue(LaunchConfiguration('execute'),value_type=bool),
            'port':LaunchConfiguration('port'), 'profile':LaunchConfiguration('profile')}])
    ])
