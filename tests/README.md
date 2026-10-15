# Controls and sensor tests

Follow the [Ubuntu build guide](../docs/ENCOS_WORKSPACE.md#build-on-ubuntu);
`./scripts/build.sh` builds the native driver and runs CTest, including Python
test discovery. Cases requiring ROS, MuJoCo, or an explicit fake-native library
skip when those dependencies are unavailable; a passing CTest run alone does
not establish that those optional cases ran.

- `encos_protocol_test.c`: query/position/velocity codec vectors and malformed frames.
- `test_imu_transport.py`: Modbus frames, checksums, signed readings and IMU unit conversions.
- `test_motor_test_session.py`: pane/session lifecycle behavior.
- `source_paths.py`: adds the reorganized source packages for test imports.
- `test_python_motors.py`: simulation, independent commands, deadlines, validation and cleanup.
- `test_differential_drive.py`: simulation-derived wheel signs, chassis arcs, RPM
  saturation, motor ordering, timestamp validation and keyboard dead-man stopping.
- `test_radio_input.py`: raw CSV validation, measured calibration, one-stick mixing,
  proportional speed, neutral acquisition, source-age expiry and recovery.
- `test_unified_controls.py`: actual node callbacks with a ROS facade, including
  chassis-only teleop, calibration rejection and publisher-free radio monitoring.
- `test_ros_radio.py`: real pseudo-terminal serial input through the radio teleop
  and ROS chassis simulator, including silence, neutral recovery and USB disconnect.
  Requires ROS 2 and pyserial; never connects to motor hardware.
- `test_ros_chassis.py`: real ROS topics through the existing simulated motor
  driver, including upstream publisher loss; skips without ROS 2. Source the ROS
  environment and run on an isolated ROS domain with no hardware or other publishers.
- `test_native_driver.py`: real C pthread and ctypes ABI against a fake SOEM bridge;
  transport/feedback failures, motor faults, per-motor leases, ownership, shutdown
  and differential forward/left/right/stop commands through all four PDO slots.
  Split-bus tests require ID 3 on CAN2, verify slots `[0,1,3]`, reject invalid
  slot maps, and check that the vacant passage carries no control commands.

The fake library has no raw-socket transport and cannot access motor hardware.
Native integration cases skip unless `ENCOS_TEST_LIBRARY` names that library;
CTest sets it automatically. The normal loader uses `libencos_driver.so`, not
`libencos_driver_test.so`.

The [motor mirror](../docs/MOTOR_MIRROR.md) adds `test_motor_mirror.py` for
feedback mapping and optional MuJoCo model checks, and `test_ros_motor_mirror.py`
for the read-only ROS-to-UDP relay. The latter requires ROS and loopback sockets;
run it on an isolated domain with no hardware publishers.

Portable Python-only tests:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -v
```

On PowerShell:

```powershell
$env:PYTHONPATH = 'src'
python -m unittest discover -s tests -p 'test_*.py' -v
```

These checks do not validate physical stopping or bridge feedback freshness.

Run the ROS radio integration test on an isolated domain (no hardware processes):

```bash
source scripts/ros2_env.sh
ROS_DOMAIN_ID=84 python3 -m unittest discover -s tests -p test_ros_radio.py -v
```

For the combined native and ROS suite after building both layers:

```bash
source scripts/ros2_env.sh
ROS_DOMAIN_ID=91 ENCOS_TEST_LIBRARY="$PWD/build/encos_driver/libencos_driver_test.so" python3 -m unittest discover -s tests -p 'test_*.py'
```

Keep this ROS domain free of hardware publishers. Optional MuJoCo checks can be
run with the viewer environment after `./scripts/setup_motor_viewer.sh`:

```bash
build/motor_viewer/venv/bin/python -m unittest discover -s tests -p test_motor_mirror.py
./scripts/run_motor_viewer.sh --check
```
