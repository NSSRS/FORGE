# Controls, sensors, and robot-model tests

Follow the [Ubuntu build guide](../docs/ENCOS_WORKSPACE.md#build-on-ubuntu);
`./scripts/build.sh` builds the native driver and runs CTest, including Python
test discovery. Cases requiring ROS, MuJoCo, or an explicit fake-native library
skip when those dependencies are unavailable; a passing CTest run alone does
not establish that those optional cases ran.

- `encos_protocol_test.c`: query/position/velocity codec vectors and malformed frames.
- `test_weave_reference.py` checks the shared trajectory with NumPy. `test_weave_xy_z.py` checks the fixed CAD gun, 1–3 Hz wheel and wrist motion, and repeat playback. Run both with `.venv/bin/python -m unittest discover -s tests -p 'test_weave*.py'`.
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

Position–velocity is the only bench motion mode. Python tests cover
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

## Arm clearance study

`test_arm_geometry.py` checks exact 150 mm D3 connection-center spacing across
all active scenes, preserved CAD meshes, geometry-review rejection and closure
repair when no independent joint command changed.

`test_arm_sweep.py` checks the full-scene clearance scan, first-boundary brackets,
D3 closure, fixed independent coordinates/chassis, contact filters, and rejection
of invalid reference poses. It never commands hardware or advances physics. Run:

```bash
.venv/bin/python -m unittest discover -s tests -p test_arm_sweep.py -v
```

## Full-robot modeling compatibility

With src/simulation/requirements.txt installed, run model_compatibility, motor_overview_models,
body_colors, magnetic, terrain, velocity_control and motor_mirror test modules.
The overview test uses synthetic named feedback across flat, curved and bumpy
full-robot scenes; it verifies status overlays and frozen arm/base coordinates.
No motor hardware or ROS runtime is needed for those tests.

### A2–A6 supported rig (A1 disconnected)

The suites also cover the six-axis profile and optional keyboard position hold:
fixed release targets, >15-second hold/jog execution, requested speed with a
lagging position-loop plant, bounded target lead during a stall in both directions,
uninterrupted resume, selection locking, focus-loss pause, explicit
disable, capability/mode gating, single-axis firmware authorization, retained
command/feedback watchdogs, torque trips and disarmed recovery. These use the
native firmware and fake motors; they are not physical load/thermal acceptance.

`test_arm_group_hold.py` adds complete/sparse group activation at current positions,
selection without disabling, fresh jog authorization, fixed nonselected targets,
position-loop lag, whole-group shutdown on an unselected joint's torque/feedback
fault or missing command lease, old-firmware/partial-mask/mixed-policy rejection,
activation cancellation, display reads without lease refresh, disarmed recovery,
Qt focus/close and UI-stall handling. All motor access uses the fake native plant.

`test_arm_capture.py` checks eight-position recording without mandatory notes:
A1/A4/A5/A6 initial and both CW/CCW ends for each of A2/A3, with no invented model
mapping for the ends. Completion and Qt Next require both endpoint saves;
failed second saves preserve the first. It also checks optional endpoint/initial recording in both raw-angle
orders, complete-group context, stationary/freshness/wrap validation, partial
progress, file reuse rejection, atomic save failures and slow background writes.
Native-backed capture adds no bus requests or disables; Qt/CLI tests cover the
capture button and finishing an outstanding save on normal shutdown. These are
software checks, not physical calibration evidence.
Startup tests inject A1 drive status 9 and missing A4 feedback: diagnostics
identify the joint, the GUI stays visible and locked, and recovery never enables
motors or clears a nonexistent controller fault. Existing active-owner rejection
and stationary/feedback interlocks remain covered.

`test_arm_smooth_jog.py` exercises the new command through the native firmware
and fake CAN plant: capability rejection before motor operations, selected-only
smoothing, immediate release/pause/selection targets, all-six 10 deg/s travel,
lagging A2 at 5 and 10 deg/s in both directions, old-firmware speed gating,
holding-mode CLI defaults, expired command leases and disarmed recovery.
`damiao_smooth_jog_test.c` checks individual 2 ms CAN targets, endpoint saturation,
immediate hold bypass, direction changes, delayed ticks, rollover, invalid
commands, feedback loss and transmit failure at 5 and 10 deg/s. CTest runs it as `damiao_smooth_jog`.
No test result establishes physical jitter improvement.

The [operator guide](../docs/ARM_RIG_TEST.md) and
[current milestone](../docs/ARM_SESSION_HANDOFF.md) distinguish physical
observations from these hardware-free regressions. Install `python3-pyqt5` for
Qt checks; source the ROS environment for launch/PTY tests.

```bash
source scripts/ros2_env.sh
QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH="tests:src:$PYTHONPATH" python3 -m unittest -v \
  test_arm_capture test_arm_smooth_jog test_arm_group_hold test_arm_pose_teach test_damiao_arm test_damiao_bench test_damiao_keyboard \
  test_pocket_controls test_pocket_gui test_radio_port test_system_test_launch
ROS_DOMAIN_ID=94 PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH="tests:src:$PYTHONPATH" python3 -m unittest -v test_ros_pocket
```

Keep domain 94 free of physical drivers. The tests use native C firmware/fake
CAN motors, Qt offscreen events and virtual serial devices; none opens physical
motor hardware. Do not count skipped optional ROS/Qt cases as passed coverage.

| Boundary | Coverage |
| --- | --- |
| Routing / feedback | Physical A2–A6 slots with absent A1, another missing motor, retained A1 routes, disabled capture without enables or parameter writes |
| Manual travel | Beyond former startup window, current-position preload, release/intent expiry, duration/protocol guards, old-firmware rejection |
| Profile / limits | 4340P 12 Nm ceiling and legacy rejection; independent lower torque trips; all active joints accept the old speed-trip region and fault above 1.5 rad/s in both directions |
| Recovery | Evidence saved before reset, retained disabled/stationary checks, range-cache repair, lost ACK, disk/readback/reset failures, no automatic enable; session bounds/original modes survive keyboard R |
| Readback / diagnostics | Disabled-only model register whitelist, wrong identities, missing/nonfinite values, original fault, enabled sample/peak retention, Feedback Nm versus Trip Nm |
| Keyboard / display | Selection, E/held arrows/release, conflicts, focus loss, UI stall, startup fault lockout, threaded recovery, fresh input after recovery, aging stale rows |
| Pocket | SE priming, diagonals, deadlines, input withdrawal during enable, unavailable A1, worker startup/motion/cleanup faults and saved errors |
| Radio / launch | USB relocation/ambiguity/exclusions, receive-only probe, stale/malformed CSV, busy ports, virtual-serial cleanup, authorization before discovery |
| ROS / PTY | Real node topics with offline motors, telemetry/STOP, supported sparse selection/manual travel, silence/shutdown and fault-report exit |

The existing native firmware CTest targets are listed above. Build the target
STM32 firmware separately using `scripts/build_dm_mc02.py` and the toolchain/SDK
instructions in [DAMIAO_ARM.md](../docs/DAMIAO_ARM.md); this build never flashes.
Physical bus timing, current/torque accuracy, position calibration, higher speeds,
loaded holding and fault-stop latency remain acceptance work in
[the open registry](../docs/ARM_LAYER_TODOS.md).

### Model-guided arm calibration

Install `src/simulation/calibration-requirements.txt` in the model environment.
`test_arm_calibration.py` covers path authorization/cancellation, one-degree
waypoints through the fake C firmware, complete fresh-pose validation, constraint
identity/backoff, disk failures, Space with text-field focus, and actual model
closure/adjacent-stop checks. No hardware is opened.

```bash
QT_QPA_PLATFORM=offscreen PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -p test_arm_calibration.py -v
QT_QPA_PLATFORM=offscreen PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -p test_arm_capture.py -v
```
