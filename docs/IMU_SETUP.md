# HWT9052 IMU setup and ROS interface

## Source organization and deployment

Native Linux is the deployment target. WSL is a temporary development host.

```text
src/imu/
  package.xml, setup.py, setup.cfg
  imu/
    protocol.py  # Modbus and decoding
    node.py      # ROS publisher
    cli.py       # Terminal reader
scripts/
  read_imu.py    # Thin wrapper
  run_imu.sh     # ROS launch helper
tests/test_imu_transport.py
```

The installed executable remains `ros2 run imu imu`.
The IMU has its own ROS package, independent of motor control.
Neither production code nor launch helpers require Windows or usbipd.

On native Ubuntu, connect the USB-RS485 adapter and grant the operator serial
access via the dialout group. Use `/dev/serial/by-id/` when a unique device ID
is available, or pass the appropriate `/dev/ttyUSB*` path to either reader.
The default workspace path below is an example, not hard-coded in the launchers.

## Current status

Verified on 2026-10-15 with Ubuntu 24.04 / ROS 2 Jazzy in WSL 2.
The CH340 USB adapter is attached directly to Ubuntu as `/dev/ttyUSB0`.
Sensor acquisition and ROS publication run in Linux; no Windows Python bridge
or shared telemetry file is needed. Windows provides the usbipd USB transport.
The sensor responses match the model-linked high-precision Modbus SDK.

Connection: 9600 baud, 8N1, Modbus address `0x50` (80 decimal).
Readers issue only function 03 requests for 15 registers starting at `0x34`.
They never write calibration, configuration, save, or motor commands.

## Choose one reader

Run only one reader at a time; stop it with Ctrl+C before switching.

### Standalone terminal reader

```bash
python3 ~/forge2_ws/scripts/read_imu.py
```

A small launch wrapper using the shared IMU module, requiring system Python and pyserial. No ROS setup
is needed. It prints sample number, RPY in degrees, acceleration in m/s²,
angular velocity in degrees/s, and magnetic field in µT. It does not publish
ROS topics. Use `--samples 10` for a bounded read, or `--port /dev/ttyUSB1`
to select a different device. Reported sensor acceleration includes gravity.

### ROS sensor node

```bash
cd ~/forge2_ws
./scripts/run_imu.sh
```

Node `/imu`, executable `imu` in package `imu`.
The launcher sources the workspace and defaults to ROS domain 42, respecting
an existing ROS_DOMAIN_ID. An optional first argument selects another port.
Startup and live status messages appear in the terminal; readings are logged
approximately every two seconds. Polling is scheduled at 10 Hz; actual delivery
depends on serial response time and valid packets.

| Topic | ROS message type | Fields and units |
| --- | --- | --- |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Acceleration XYZ in m/s²; angular velocity XYZ in rad/s |
| `/imu/data_rpy` | `geometry_msgs/msg/Vector3Stamped` | x = roll, y = pitch, z = yaw, all radians |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | Magnetic field XYZ in tesla |

All use sensor-data QoS and `imu_link`. ROS calculations use radians; terminal
display uses degrees for readability. `/imu/data_rpy` replaces the earlier
RPY topic name. Message timestamps represent host receipt/publication time,
not a synchronized sensor acquisition clock.

To view ROS angles from another Ubuntu terminal:

```bash
cd ~/forge2_ws
source scripts/ros2_env.sh
export ROS_DOMAIN_ID=42
ros2 topic echo /imu/data_rpy --qos-reliability best_effort
```

The standard Imu orientation is marked unavailable with
`orientation_covariance[0] = -1`. Other covariances are unknown (zero arrays).
Sensor-provided RPY is separate pending mounting and navigation-frame validation.

## USB attachment and permissions

usbipd-win is installed on Windows. The first installation failed because it
lacked administrator privileges; elevated installation succeeded. The Linux
ch341 driver loaded and the Ubuntu user was added to `dialout`.

After unplugging or restarting WSL, inspect and reattach from Ubuntu:

```bash
"/mnt/c/Program Files/usbipd-win/usbipd.exe" list
"/mnt/c/Program Files/usbipd-win/usbipd.exe" attach --wsl --busid 2-7
```

`2-7` was the verified CH340 IMU bus ID; confirm it in the list, particularly
after moving USB sockets. Binding persists; attachment does not. Windows COM4
cannot be used while the device is attached to WSL. Do not install Ubuntu's
unrelated usbipd packages in response to a command-not-found suggestion.

If the device exists but this terminal lacks permission:

```bash
newgrp dialout
cd ~/forge2_ws
./scripts/run_imu.sh
```

The launcher distinguishes a missing device from insufficient permissions.
The temporary Windows file bridge has been removed from FORGE. Both readers
use Linux serial directly. WSL instructions here are development-host notes only.

## Verified register mapping

The manufacturer protocol and official Python SDK both specify R, P, Y order:

| Registers | Angle | ROS field |
| --- | --- | --- |
| 0x3D / 0x3E | Roll, sensor X | vector.x |
| 0x3F / 0x40 | Pitch, sensor Y | vector.y |
| 0x41 / 0x42 | Yaw, sensor Z | vector.z |

Angles are signed 32-bit values, low register word first, divided by 1000 for
degrees and converted to radians internally. SDK AngX/AngY/AngZ read response
bytes 21–24 / 25–28 / 29–32 respectively. Register scaling for acceleration is
16/32768 g, gyroscope 2000/32768 degrees/s, magnetic field 13 nT/LSB.
The physical measurement range is not the register encoding scale.

## Nine-axis operation, calibration, and future estimation

Nine-axis operation is the chosen configuration for now. The manufacturer's
AXIS6 register (0x24) documents 0 = nine-axis and 1 = six-axis, with a default
of zero. The current unit's setting has not been read back or changed; do not
describe the default as a verified configuration.

Starting either reader does not recalibrate the sensor. Firmware supports
stationary gyroscope correction controlled by threshold/time settings; those
settings have not been checked on this unit. Accelerometer and magnetometer
calibration are separate procedures, not performed by these scripts. Mounting,
magnetic interference from the robot, and calibration remain to be assessed.

This is a sensor node, not a pose estimator. No robot mounting TF, ROS ENU
orientation conversion, wheel odometry fusion, or localization is implemented.
Future work: verify axes and mounting, characterize uncertainty and bias, then
combine IMU and wheel odometry (and later visual/LiDAR references) in a separate
state estimator. Keep quaternion orientation for estimator calculations and RPY
for display. IMU-only integration does not provide stable robot position.

## Validation and sources

Jazzy package build passed. A separate ROS subscriber received 10 physical
samples directly through Linux serial as the normal user, with resting Z
acceleration about 9.80 m/s². Captured-frame tests cover CRC, signed decoding,
scaling, and distinct roll/pitch/yaw ordering. These checks do not establish
calibration accuracy or robot-frame orientation correctness.

- [Product specification](https://wit-motion.yuque.com/wumwnr/docs/gs7anz)
- [Model-linked SDK page](https://wit-motion.yuque.com/wumwnr/ltst03/ymzkk34re95pk4f8)
- [Official SDK](https://github.com/WITMOTION/WitHighModbus_HWT9073485)
- [Protocol and register table](https://wit-motion.gitbook.io/witmotion-sdk/high-precision-modbus-protocol/high-precision-sensor-modbus-protocol)
- [Python decoder](https://github.com/WITMOTION/WitHighModbus_HWT9073485/blob/main/Python/9073-485_python_sdk/device_model.py)
