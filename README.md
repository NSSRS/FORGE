# FORGE robot controls and simulation

Motor control, IMU acquisition, teleoperation, and visualization for the magnetic wall-climbing welding robot.

Pocket combined controls now use **left stick for chassis** and **right stick
for the selected arm motor**: left/up=CW, right/down=CCW, with **SE held**.
SB selects motors 1/2/3; SC selects 4/5/6. All right-stick diagonals stop motion
without breaking an existing SE prime. See [mapping capture and launch](docs/POCKET_ARM_CONTROL.md).

The [assembled-arm handoff](docs/ASSEMBLED_ARM_BRINGUP.md) records the control
changes, encoder/startup behavior, model constraints and next steps for physical
calibration and gravity holding. The current jog adapters remain unloaded tools.

ENCOS motors are the **chassis** actuators. The six Damiao motors are **arm**
actuators controlled by a DM-MC02 over USB. See [Damiao arm implementation and
commissioning](docs/DAMIAO_ARM.md) for firmware builds, the separate host/ROS
interface, motor descriptions, manufacturer manuals and outstanding hardware mapping.

- **[`nathan`](https://github.com/NSSRS/FORGE/tree/nathan)**: Damiao arm firmware, descriptions, host interface, and [six-motor unloaded commissioning](docs/DAMIAO_ARM.md#current-status--2026-11-05) and desktop joint jogging; calibration and loaded-arm validation pending. Also includes the integrated Onshape chassis/arm MJCF, static posing and full MuJoCo simulation under `src/simulation/`.
- **`main`**: ENCOS C11 driver/utilities, Python motor API, and ROS 2 motor, IMU, teleoperation, visualization, and bringup packages.
- **[`physics-analysis`](https://github.com/NSSRS/FORGE/tree/physics-analysis)**:
  thermal/magnetic adhesion and suspension/wheel/terrain simulators, preserved
  from the former `main` at `324d21e`.
- **`encos-motor-workspace`**: preserved earlier combined workspace.

## Arm base-layer APIs and cross-layer work

See [base-layer contract, CAN budget and cached APIs](docs/ARM_BASE_LAYER.md)
and [cross-layer TODO registry](docs/ARM_LAYER_TODOS.md). Each affected package
has a local `TODO.md` linking its responsibilities. New telemetry is software-tested;
its deployment/hardware acceptance is pending. Historical bench notes below
must not be used to infer current IDs or loaded-arm readiness.

## Damiao arm bench status

As of 2026-11-05, all six motors have distinct programmed/read-back IDs and
passed the saved 14-stage unloaded group/individual sequence, with disable
and mode restoration verified. A1 is on CAN1; A2..A6 are on CAN2.
Successful repeat sessions reuse configuration without routine resets or flashes.
See [current evidence and historical bench notes](docs/DAMIAO_ARM.md),
[ID inventory](docs/ARM_MOTOR_IDS.md) and [test commands](docs/DAMIAO_MOTOR_TESTS.md).
The old fixed CAN2/ID1 4340P script does not match the current programmed IDs.
Production ROS, calibrated joint limits and loaded-arm operation remain unvalidated.

## Manual arm joint control

[Position API and keyboard joint jog](docs/ARM_POSITION_JOG.md): run
`python3 scripts/jog_dm_arm.py --simulate`, select 1..6, press E once and hold an arrow in the desktop window; release stops.
Hardware execution is explicit and limited to the documented unloaded fixture.
ENCOS WASD driving remains separate.

## Robot model and preview

See [model setup, coordinates, collisions and compatibility](docs/ONSHAPE_ARM_PREVIEW.md).
From this checkout, create .venv and install src/simulation/requirements.txt, then:

```bash
.venv/bin/python scripts/preview_native.py --renderer gpu
```

This is static posing: click Simulation > Pause to enable Joint sliders.
D3's passive linkage follows kinematically; no physics steps run.
For full physics, magnets, WASD driving, terrain and telemetry:

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA .venv/bin/python scripts/preview_sim.py
```

The default arm angles are [0, 0.0942, -0.44, 0, -0.188, 0] radians.
Only DOF1's zero was rebased. These are not calibrated hardware encoder coordinates.

For the six-joint collision-clearance study, run
`.venv/bin/python scripts/preview_arm_sweep.py`. See
[candidate intervals and limitations](docs/ARM_COLLISION_STUDY.md). The D3 rod
assembly uses exactly 150 mm between connection centers; the 140 mm bare-rod mesh
is unchanged. These sampled intervals are not hardware operating limits.
MoveIt/Mink integration remains future work.

## Python motion interface

Write application logic in Python; the **C11** shared library owns the SOEM
EtherCAT loop. This is C, not C++. The first interface supports one bridge and
1-4 unique motor IDs, with independent commands for each motor. By default, configuration
entries 1-3 use CAN1; the fourth uses CAN2. Explicit `motor_slots` can override
this routing. The current [three-motor chassis bench](docs/CAN2_BENCH.md) uses
IDs 1/2 on CAN1 and ID 3 on CAN2:

| API | Meaning |
|---|---|
| `bus.velocity(id, rpm)` | Mode 2: signed output-shaft speed |
| `bus.position(id, degrees)` | Mode 1: absolute output-shaft position |
| `bus.state(id)` | Position, velocity, phase current, temperature, motor error |
| `bus.stop()` | Request zero speed for all active motors |
| `bus.check()` | Raise on a latched fault |

Both manual driving and future automatic chassis trajectory tracking can use
this velocity interface. The [differential chassis controller](docs/DIFFERENTIAL_DRIVE.md)
maps timestamped chassis velocity and WASD teleop to four motor RPM targets using
the preserved four-driven-wheel simulation layout. Hardware mapping still needs commissioning.
State estimation and trajectory tracking remain future work. Teleoperation now
uses chassis W/S/A/D only, with keyboard or proportional
single-stick radio input in the same executable. See
[radio calibration and operation](docs/RADIO_CONTROL.md) and
[ROS 2 motor-control status](docs/ROS2_MOTOR_CONTROL_STATUS.md).

Run the independent-motor example without hardware (Python 3.10+):

```bash
PYTHONPATH=src python3 scripts/independent_motors.py
```

Or on PowerShell:

```powershell
$env:PYTHONPATH = 'src'
python scripts/independent_motors.py
```

For native Ubuntu 24.04 builds, follow the
[workspace build guide](docs/ENCOS_WORKSPACE.md#build-on-ubuntu). It builds the
C utilities and shared library and runs hardware-free protocol/Python tests.
See **[Python setup, examples, deadlines, and limitations](docs/PYTHON_CONTROL.md)**
before hardware use. Supplier files and the local equipment record stay outside Git.
The **[robot platform roadmap](docs/ROBOT_PLATFORM_ROADMAP.md)** defines the planned
ROS 2 nodes, teleoperation, localization, trajectory tracking, and ESTOP architecture.

## Repository layout

```text
src/
  encos_driver/    ENCOS chassis C11 EtherCAT driver and bench utilities
  damiao_firmware/ DM-MC02 arm firmware, USB protocol and watchdogs
  robot_description/ Shared MJCF/meshes, chassis/arm inventories and motor Xacro macros
  simulation/     MuJoCo previews, CAD integration, terrain and simulation controls
  motors/         Python motor API, simulator and native commissioning
  motor_control/  ROS motor driver, chassis mixing and ROS commissioning
  imu/            ROS IMU node, serial protocol and terminal reader
  stereo_cam/     Orbbec RGB/depth/stereo IR integration and stream checks
  seam_detection/ Observation-only RGB joint edge detection
  teleop/         Keyboard and radio command input
  visualization/  Measured-state relay, mirror model and MuJoCo viewer
  bringup/        System launch files, shared profiles and pane/session manager
scripts/          Thin launch wrappers and build/setup helpers
tests/            Protocol, simulator, ROS and fake-hardware verification
docs/             Architecture, setup and commissioning documentation
```

Each ROS package has an inner Python module of the same name; its outer
directory contains `package.xml`, setuptools metadata, and resources.
Build output, installed packages, and logs are generated locally and untracked.

## IMU sensor

The HWT9052 high-precision Modbus IMU has been read through native Ubuntu serial
in WSL 2. Use `./scripts/run_imu.sh` for the ROS node or
`python3 scripts/read_imu.py` for standalone terminal output; run one at a time.
ROS topics are `/imu/data_raw`, `/imu/data_rpy` (roll/pitch/yaw in radians), and
`/imu/mag`. See [IMU setup and interface](docs/IMU_SETUP.md) for USB attachment,
permissions, message fields, calibration limits, and validation. Robot-frame
orientation and pose-estimator integration remain future work.

## Stereo camera

The Orbbec Gemini 338Le uses the official ROS 2 driver through the `stereo_cam`
package. Run `./scripts/setup_stereo_cam.sh`, then `./scripts/run_stereo_cam.sh`.
See [camera setup](docs/STEREO_CAM_SETUP.md) for networking, RGB/depth/IR topics,
filters, calibration, and the RGB-only 30 FPS default and optional 60 FPS streams.

For observation-only joint edge detection on the RGB stream, run
`bash scripts/run_seam_edges.sh`. See [seam edge detection](docs/SEAM_EDGE_DETECTION.md)
for the live controls, ROS output images, and pixel-only measurement limits.

## Combined system test

Use [the full-system bench guide](docs/FULL_SYSTEM_TEST.md) to launch the IMU,
camera, calibrated Pocket controller and three-motor driver together.
`bash scripts/run_system_test.sh` starts the **actual three-motor bench**, asking
for sudo when needed, with the saved IMU/ESP32 ports and Pocket left stick.
Use `bash scripts/run_system_test.sh simulate:=true` for simulated motors.
The camera window shows RGB at a requested 30 FPS with received-FPS and CPU/RAM
diagnostics. Camera-only wrappers use shared-memory DDS for smooth local video;
the combined motor launch retains its existing transport settings.

## Documentation

| Guide | Contents |
|---|---|
| [Source organization](docs/SOURCE_LAYOUT.md) | Package ownership, script placement, and generated build folders |
| [Workspace](docs/ENCOS_WORKSPACE.md) | Repository layout, native build, and C utility commands |
| [Python control](docs/PYTHON_CONTROL.md) | Installation, API, command deadlines, and driver limitations |
| [IMU setup](docs/IMU_SETUP.md) | Native serial, standalone/ROS readers, topics, units, calibration and fusion status |
| [ROS 2 commissioning](docs/ROS2_MOTOR_CONTROL_STATUS.md) | Topics, keyboard behavior, build/run, and DDS setup |
| [Radio control](docs/RADIO_CONTROL.md) | ESP32 serial input, single-stick calibration, and proportional WASD control |
| [Differential chassis control](docs/DIFFERENTIAL_DRIVE.md) | Simulation geometry, four-wheel mapping, chassis launch and WASD control |
| [Bench procedure and record](docs/ENCOS_TEST_BENCH.md) | Wiring, acceptance checks, dated measurements and outcomes |
| [Protocol reference](docs/ENCOS_BRINGUP.md) | Packet units/encoding, vendor findings, and original bring-up design |
| [Platform roadmap](docs/ROBOT_PLATFORM_ROADMAP.md) | Planned chassis control, localization, and safety architecture |
| [Tests](tests/README.md) | Coverage and native/portable test commands |

## Live motor visualization

[Mirror measured motor movement in MuJoCo](docs/MOTOR_MIRROR.md) using the
full robot in `src/robot_description/mujoco/` and a read-only `/joint_states`
relay. Run `scripts/setup_motor_viewer.sh` once for its environment. Only wheel
feedback is displayed, and arm joints remain at their saved pose. The current
three-motor profile marks motor 4 absent and preserves its model for later.

## Existing bench utilities

- `encos_query`: discovery and read-only telemetry.
- `encos_assign_id`: guarded ID assignment with one isolated motor.
- `encos_motion`: bounded positive position excursion and return.

The original utilities remain available. See the [workspace guide](docs/ENCOS_WORKSPACE.md),
[protocol reference](docs/ENCOS_BRINGUP.md), and [bench procedure](docs/ENCOS_TEST_BENCH.md).
Do not run more than one EtherCAT master on the interface.

## Verification and limits

The new library compiles on Ubuntu 24.04. Protocol vectors, Python simulation,
and native-thread tests with a fake SOEM bridge cover independent motion,
validation, command expiry, feedback/transport faults, and cleanup.
The native Python path and temporary ROS 2 commissioning path have completed
bounded tests with three secured, unloaded physical motors. This does not
validate loaded robot motion, traction, braking, or host-loss behavior.

Every active motor needs an explicit command refresh within 250 ms. The C worker
attempts zero-speed commands on timeout/fault/close, then clears its outputs.
That is not a verified mechanical stop: cached bridge feedback and bridge behavior
after host loss still require bench verification. Motor/fixture limits must
be supplied explicitly; no model-specific safe settings are assumed.

## Package migration

Package and executable names now omit `forge_`. Examples:
`ros2 run motor_control motor_driver`, `ros2 run teleop keyboard_teleop`,
`ros2 run visualization motor_state_relay`, `ros2 run imu imu`, and
`ros2 launch bringup chassis.launch.py`. Node names and YAML keys were updated
together; ROS topic names are unchanged. Start a fresh terminal and rebuild
before using the renamed packages. Source `scripts/ros2_env.sh` for the shared
configuration and source-tree `motors` API. Native library output is now under
`build/encos_driver/`. See [source organization](docs/SOURCE_LAYOUT.md).

## Damiao arm motor tests

[8009P, 4310P and full-arm test preparation](docs/DAMIAO_MOTOR_TESTS.md) provides
individual outward/return and six-motor sequential/concurrent tests through
`scripts/test_dm_arm.py`. CAN1 is reserved for the 8009P alone. Rebuild/flash
matching MC02 firmware and fill the deliberately unconfirmed bench plans from
measured hardware before execution. The previous 4340P result remains the only
physical arm-motor validation; these new paths have software coverage.
