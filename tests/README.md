# Controls, sensors, and robot-model tests

Follow the [Ubuntu build guide](../docs/ENCOS_WORKSPACE.md#build-on-ubuntu);
`./scripts/build.sh` builds the native driver and runs CTest, including Python
test discovery. Cases requiring ROS, MuJoCo, or an explicit fake-native library
skip when those dependencies are unavailable; a passing CTest run alone does
not establish that those optional cases ran.

- `encos_protocol_test.c`: query/position/velocity codec vectors and malformed frames.
- `test_imu_viewer.py`: orientation rotation, invalid/replayed/stale sample rejection.
- `test_system_test_wrapper.py`: hardware-default elevation, explicit simulation, and CLI override checks using stub commands (no motor access).
- `test_system_test_launch.py`: physical-motor confirmation, separate Ethernet and serial-port guards.
- `test_stereo_cam_network.py`: camera interface selection and non-destructive subnet setup.
- `test_stereo_cam.py`: camera launch argument forwarding and CLI override checks (requires ROS launch).
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

`test_camera_frame_buffer.py` checks latest-image replacement and bounded telemetry
when the GUI falls behind. Live ROS callback and display rates are shown in the
camera window; see `docs/STEREO_CAM_SETUP.md` for the measurement boundaries.

Seam edge detection uses synthetic geometry tests and an isolated ROS image test:

```bash
source scripts/ros2_env.sh
source scripts/camera_env.sh
ROS_DOMAIN_ID=91 python3 -m unittest discover -s tests -p 'test_seam_edges*.py' -v
```

The ROS test requires the installed `seam_detection` package. It starts only the
detector and synthetic publisher, never camera acquisition or motor control.
These tests do not establish real-weld recognition or millimeter accuracy.

## Damiao arm

`damiao_arm_test.c` verifies the portable firmware codec, leases, feedback loss,
fault reset, rollover, CAN failure and framed USB protocol. `test_damiao_arm.py`
checks the host profile/protocol and drives the real C firmware through fake CAN
for six joints. `bash scripts/build.sh` includes both. No test opens motor hardware.
See [arm implementation](../docs/DAMIAO_ARM.md) for the separate STM32 build.

The native Damiao round-trip test covers both selected bus maps and checks 50
control frames per joint over 100 ms of simulated MCU time (500 Hz). This does
not measure wire utilization, CAN arbitration latency or physical motor response.


## Current chassis and arm MJCF

Use the root .venv with src/simulation/requirements.txt for simulation tests. The dedicated
model tests skip when optional simulation dependencies are unavailable.
Generate terrain scenes before the collision broad-phase tests:

```bash
.venv/bin/python scripts/preview_sim.py --terrain curved --check
.venv/bin/python scripts/preview_sim.py --terrain bumpy --check
.venv/bin/python -m unittest discover -s tests -p test_model_compatibility.py -v
.venv/bin/python -m unittest discover -s tests -p test_magnetic.py -v
.venv/bin/python -m unittest discover -s tests -p test_velocity_control.py -v
.venv/bin/python -m unittest discover -s tests -p test_terrain.py -v
```

The full model has 20 joints, ten actuators and one D3 connect closure.
Model compatibility tests cover default joint angles/targets, the rebased D1,
static D3 solving, and wheel-feedback projection without changing arm/base pose.
See [model guide](../docs/ONSHAPE_ARM_PREVIEW.md) for remaining physical limitations.


### Current Damiao bench regressions

Position–velocity is the only bench motion mode. The14 Python tests cover
host/firmware round trips, wrong mode readback, speed/gain rejection, simulated
target completion and stalls, delayed enable, restoration, watchdogs and both
six-joint bus maps. Two CTest targets cover codec/state machine/USB framing
and CAN completion/abort. The general MIT codec remains for the separate
production interface; obsolete MIT bench variants are removed.

```bash
cmake -S src/damiao_firmware -B build/damiao_tests
cmake --build build/damiao_tests
ctest --test-dir build/damiao_tests --output-on-failure
PYTHONPATH=src python3 -m unittest discover -s tests -p test_damiao_arm.py -v
```

Hardware evidence and remaining model/unit tests are in
[arm commissioning](../docs/DAMIAO_ARM.md) and the
[description handoff](../src/robot_description/ARM_COMMISSIONING.md).

### Routed P-series and six-motor bench

`test_damiao_bench.py` adds an independent six-motor CAN plant, exercising the
real C firmware with the shared host runner. Run both Damiao suites with
`PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_damiao*.py' -v`.
Coverage includes both bus topologies, signed excursions/returns, factory reply
ID 0, model ranges, group enable, wrong identity/mode, stalls, missing feedback,
speed/torque/lease trips, restoration and immutable unconfirmed templates.
See [the test guide](../docs/DAMIAO_MOTOR_TESTS.md) for physical commissioning.

## Full-robot modeling compatibility

With src/simulation/requirements.txt installed, run model_compatibility, motor_overview_models,
body_colors, magnetic, terrain, velocity_control and motor_mirror test modules.
The overview test uses synthetic named feedback across flat, curved and bumpy
full-robot scenes; it verifies status overlays and frozen arm/base coordinates.
No motor hardware or ROS runtime is needed for those tests.
