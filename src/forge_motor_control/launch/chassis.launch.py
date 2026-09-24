"""Unified motor driver with chassis mixing; simulation by default."""
from dataclasses import fields
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from forge_motor_control.differential import CommandLease, DifferentialDrive


def setup(context):
    def flag(name):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ("true", "false"):
            raise ValueError(f"{name} must be true or false")
        return value == "true"

    simulate, execute = flag("simulate"), flag("execute")
    if not simulate and not (execute and flag("mapping_confirmed")):
        raise ValueError("Hardware requires execute:=true and mapping_confirmed:=true after calibration")
    config_path = LaunchConfiguration("config").perform(context)
    config = yaml.safe_load(Path(config_path).read_text())
    driver = config["forge_motor_driver"]["ros__parameters"]
    if driver["command_mode"] != "chassis":
        raise ValueError("Chassis launch requires command_mode: chassis")
    drive = DifferentialDrive(**{f.name: driver[f.name] for f in fields(DifferentialDrive)})
    CommandLease(drive, driver["command_timeout_s"], driver["command_frame"])
    nodes = [
        Node(package="forge_motor_control", executable="forge_motor_driver",
             parameters=[config_path, {"simulate": simulate, "execute": execute,
                                       "mapping_confirmed": flag("mapping_confirmed")}], output="screen"),
    ]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(reason="Chassis node exited"))]))
        for node in nodes]
    return handlers + nodes


def generate_launch_description():
    config = str(Path(get_package_share_directory("forge_motor_control")) / "config/chassis.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        DeclareLaunchArgument("simulate", default_value="true"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("mapping_confirmed", default_value="false"),
        OpaqueFunction(function=setup),
    ])
