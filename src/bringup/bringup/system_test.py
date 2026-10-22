"""Combined bench launch, with explicit physical-motor authorization."""
from dataclasses import fields
import os
from pathlib import Path
import stat

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from motor_control.differential import DifferentialDrive, CommandLease
from teleop.radio_input import Axis, RadioCommand


def validate_options(options):
    if options['camera_interface'] == options['motor_interface']:
        raise ValueError('Camera and EtherCAT must use different Ethernet interfaces')
    if not options['simulate'] and not all(options[key] for key in
            ('execute', 'mapping_confirmed', 'radio_failsafe_confirmed')):
        raise ValueError('Physical motors require execute:=true, mapping_confirmed:=true and '
                         'radio_failsafe_confirmed:=true after bench verification')
    for key, enabled in [('imu_port', options['use_imu']), ('radio_port', options['use_radio'])]:
        if enabled and not options[key]:
            raise ValueError(f'Specify {key}:=/dev/serial/by-path/... for the correct device')
    if options['use_imu'] and options['use_radio']:
        if Path(options['imu_port']).resolve() == Path(options['radio_port']).resolve():
            raise ValueError('IMU and radio must use different serial devices')


def setup(context):
    names = ('simulate', 'execute', 'mapping_confirmed', 'radio_failsafe_confirmed',
             'use_imu', 'use_camera', 'use_radio', 'show_video', 'show_imu', 'show_motor_model')
    options = {key: LaunchConfiguration(key).perform(context) == 'true' for key in names}
    for key in ('imu_port', 'radio_port', 'motor_interface', 'camera_interface',
                'camera_ip', 'camera_serial', 'config', 'radio_config', 'motor_viewer_port'):
        options[key] = LaunchConfiguration(key).perform(context)
    validate_options(options)
    config = yaml.safe_load(Path(options['config']).read_text())
    driver = config['motor_driver']['ros__parameters']
    if driver['command_mode'] != 'chassis':
        raise ValueError('System test requires a chassis-mode motor profile')
    drive = DifferentialDrive(**{field.name: driver[field.name] for field in fields(DifferentialDrive)})
    CommandLease(drive, driver['command_timeout_s'], driver['command_frame'])
    radio = yaml.safe_load(Path(options['radio_config']).read_text())['keyboard_teleop']['ros__parameters']
    if options['use_radio']:
        def axis(prefix):
            return Axis(*(radio[f'{prefix}_{field}'] for field in
                          ('channel', 'min', 'center', 'max', 'sign')), deadzone=radio['radio_deadzone'])
        teleop = config['keyboard_teleop']['ros__parameters']
        RadioCommand(axis('forward'), axis('turn'), teleop['speed_m_s'],
                     teleop['yaw_rate_rad_s'], radio['radio_timeout_s'])
    for key, enabled in [('imu_port', options['use_imu']), ('radio_port', options['use_radio'])]:
        if enabled:
            path = Path(options[key])
            if not path.exists() or not stat.S_ISCHR(path.stat().st_mode):
                raise ValueError(f'{key} is not an attached serial device: {path}')
            if not os.access(path, os.R_OK | os.W_OK):
                raise ValueError(f'No read/write access to {path}; check serial permissions')
    if not options['simulate']:
        if os.geteuid() != 0:
            raise ValueError('Hardware test must run under sudo so all ROS nodes share the motor account')
        # Loading the shared library does not open EtherCAT or command motors.
        # The driver performs its own transport validation when it starts.
        import ctypes
        library = os.environ.get('FORGE_MOTORS_LIBRARY') or (
            Path(__file__).resolve().parents[3] / 'build/encos_driver/libencos_driver.so')
        ctypes.CDLL(str(library))
        import json
        import subprocess
        ports = json.loads(subprocess.check_output(['ip', '-j', 'address', 'show'], text=True))
        port = next((item for item in ports if item['ifname'] == options['motor_interface']), None)
        if port is None or 'LOWER_UP' not in port.get('flags', []):
            raise ValueError('EtherCAT port has no carrier; check bridge connection and power')
        if any(address.get('family') == 'inet' for address in port.get('addr_info', [])):
            raise ValueError('Dedicated EtherCAT port still has IPv4 configuration; remove the old camera address first')
    share = Path(get_package_share_directory('bringup'))
    workspace = Path(__file__).resolve().parents[3]
    viewer_port = int(options['motor_viewer_port'])
    if not 1024 <= viewer_port <= 65535:
        raise ValueError('motor_viewer_port must be 1024-65535')
    if options['show_motor_model']:
        if not (workspace / 'build/motor_viewer/venv/bin/python').is_file() or not (
                workspace / 'build/motor_viewer/model/scene.xml').is_file():
            raise ValueError('Run scripts/setup_motor_viewer.sh before enabling show_motor_model')
    actions = []
    if options['use_camera']:
        # Run network preparation now, before any motor process starts. The
        # camera-only include skips its second preparation pass.
        from stereo_cam.network import prepare_network
        prepare_network(interface=options['camera_interface'], camera_ip=options['camera_ip'])
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/stereo_cam.launch.py')),
            launch_arguments={
                'prepare_camera_network': 'false', 'net_device_ip': '',
                'enumerate_net_device': 'true', 'serial_number': options['camera_serial'],
                'show_video': str(options['show_video']).lower(),
            }.items()))
    if options['use_imu']:
        actions.append(Node(package='imu', executable='imu', name='imu', output='screen',
                            parameters=[{'port': options['imu_port'], 'baud': 9600, 'address': 80}]))
    actions.append(Node(package='motor_control', executable='motor_driver', name='motor_driver',
                        output='screen', parameters=[options['config'], {
                            'simulate': options['simulate'], 'execute': options['execute'],
                            'mapping_confirmed': options['mapping_confirmed'],
                            'interface': options['motor_interface'], 'log_motor_targets': True}]))
    if options['use_radio']:
        actions.append(Node(package='teleop', executable='keyboard_teleop', name='keyboard_teleop',
                            output='screen', parameters=[options['config'], options['radio_config'],
                            {'input_source': 'radio', 'serial_port': options['radio_port']}]))
    if options['use_imu'] and options['show_imu']:
        actions.append(Node(package='visualization', executable='imu_viewer',
                            name='imu_viewer', output='screen',
                            additional_env={'QT_QPA_PLATFORM': 'xcb'}))
    if options['show_motor_model']:
        actions.append(Node(package='visualization', executable='motor_state_relay',
                            name='motor_state_relay', output='screen',
                            parameters=[{'port': viewer_port}]))
        command = ['bash', str(workspace / 'scripts/run_motor_viewer.sh'),
                   '--config', options['config'], '--port', str(viewer_port)]
        # DDS relay runs with the driver; MuJoCo stays on the desktop account.
        if os.geteuid() == 0 and os.environ.get('SUDO_USER') not in (None, 'root'):
            command = ['sudo', '-u', os.environ['SUDO_USER'],
                       '--preserve-env=DISPLAY,XAUTHORITY', '--', *command]
        actions.append(ExecuteProcess(cmd=command, cwd=str(workspace), output='screen',
                                      sigterm_timeout='2', sigkill_timeout='2'))
    # Any launched process ending (including the camera container and viewer)
    # stops the entire session. Command leases remain the motor protection layer.
    return [RegisterEventHandler(OnProcessExit(on_exit=[
        EmitEvent(event=Shutdown(reason='System-test process exited; stopping all nodes'))]))] + actions


def generate_launch_description():
    share = Path(get_package_share_directory('bringup'))
    defaults = {
        'simulate': 'true', 'execute': 'false', 'mapping_confirmed': 'false',
        'radio_failsafe_confirmed': 'false', 'use_imu': 'true', 'use_camera': 'true',
        'use_radio': 'true', 'show_video': 'true',
        'show_imu': 'true', 'show_motor_model': 'true', 'motor_viewer_port': '8766',
        # Operator identified IMU=ttyUSB1 and ESP32=ttyUSB0 on these sockets.
        'imu_port': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0',
        'radio_port': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0',
        'motor_interface': 'enp86s0',
        'camera_interface': 'enx000ec8a22028', 'camera_ip': '192.168.1.10',
        'camera_serial': 'CHL5663000HZ',
        'config': str(share / 'config/chassis_bench.yaml'),
        'radio_config': str(share / 'config/radio_pocket.yaml'),
    }
    flags = {'simulate', 'execute', 'mapping_confirmed', 'radio_failsafe_confirmed',
             'use_imu', 'use_camera', 'use_radio', 'show_video', 'show_imu', 'show_motor_model'}
    return LaunchDescription([
        DeclareLaunchArgument(key, default_value=value,
                              **({'choices': ['true', 'false']} if key in flags else {}))
        for key, value in defaults.items()
    ] + [OpaqueFunction(function=setup)])
