"""Pocket plus chassis and arm adapters, simulated motors by default."""
from bringup.system_test import generate_launch_description as system_launch


def generate_launch_description():
    return system_launch({'use_imu': 'false', 'use_camera': 'false', 'show_video': 'false',
                          'show_imu': 'false', 'show_motor_model': 'false',
                          'show_pocket': 'true', 'log_motor_targets': 'false'})
