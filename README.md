# FORGE robot controls

Motor control, IMU acquisition, teleoperation, and visualization for the magnetic wall-climbing welding robot.

- **`main`**: ENCOS C11 driver/utilities, Python motor API, and ROS 2 motor, IMU, teleoperation, visualization, and bringup packages.
- **[`physics-analysis`](https://github.com/NSSRS/FORGE/tree/physics-analysis)**:
  thermal/magnetic adhesion and suspension/wheel/terrain simulators, preserved
  from the former `main` at `324d21e`.
- **`encos-motor-workspace`**: preserved earlier combined workspace.

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
the `forge_sim` wheel layout. Hardware mapping still needs commissioning.
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
  encos_driver/    C11 EtherCAT driver, protocol and bench utilities
  motors/          Python motor API, simulator and native commissioning
  motor_control/   ROS motor driver, chassis mixing and ROS commissioning
  imu/             ROS IMU node, serial protocol and terminal reader
  teleop/          Keyboard and radio command input
  visualization/   Measured-state relay, mirror model and MuJoCo viewer
  bringup/         System launch files, shared profiles and pane/session manager
scripts/           Thin launch wrappers and build/setup helpers
tests/             Protocol, simulator, ROS and fake-hardware verification
docs/              Architecture, setup and commissioning documentation
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
`forge_sim` CAD model and a read-only `/joint_states` relay. The current
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
