# FORGE motor controls

Motor-control development for the magnetic wall-climbing welding robot.

- **`main`**: ENCOS C11 driver/utilities, Python interface, and ROS 2 commissioning package.
- **[`physics-analysis`](https://github.com/NSSRS/FORGE/tree/physics-analysis)**:
  thermal/magnetic adhesion and suspension/wheel/terrain simulators, preserved
  from the former `main` at `324d21e`.
- **`encos-motor-workspace`**: preserved earlier combined workspace.

## Python motion interface

Write application logic in Python; the **C11** shared library owns the SOEM
EtherCAT loop. This is C, not C++. The first interface supports one bridge and
1-4 unique motor IDs, with independent commands for each motor. Configuration
entries 1-3 use CAN1; the fourth uses CAN2:

| API | Meaning |
|---|---|
| `bus.velocity(id, rpm)` | Mode 2: signed output-shaft speed |
| `bus.position(id, degrees)` | Mode 1: absolute output-shaft position |
| `bus.state(id)` | Position, velocity, phase current, temperature, motor error |
| `bus.stop()` | Request zero speed for all active motors |
| `bus.check()` | Raise on a latched fault |

Both manual driving and future automatic chassis trajectory tracking can use
this velocity interface. Chassis kinematics, encoder/IMU/camera state estimation,
steering-aware keyboard mapping, and trajectory tracking are future work. A
temporary direct-RPM ROS 2 keyboard publisher is available for commissioning;
see [ROS 2 motor-control status](docs/ROS2_MOTOR_CONTROL_STATUS.md).

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
  encos_query/          C11 driver, protocol codec, and bench utilities
  forge_motors/         Standalone Python API and simulator
  forge_motor_control/  ROS 2 package metadata, configuration, and Python module
docs/                   Guides, equipment template, and bench results
scripts/                Build/run helpers and simulation example
tests/                  Protocol, Python, and fake-bridge tests
```

The ROS package's inner `forge_motor_control/` is its importable Python module;
the outer directory contains `package.xml`, setuptools metadata, and resources.
Build output, installed packages, and logs are generated locally and untracked.

## Documentation

| Guide | Contents |
|---|---|
| [Workspace](docs/ENCOS_WORKSPACE.md) | Repository layout, native build, and C utility commands |
| [Python control](docs/PYTHON_CONTROL.md) | Installation, API, command deadlines, and driver limitations |
| [ROS 2 commissioning](docs/ROS2_MOTOR_CONTROL_STATUS.md) | Topics, keyboard behavior, build/run, and DDS setup |
| [Bench procedure and record](docs/ENCOS_TEST_BENCH.md) | Wiring, acceptance checks, dated measurements and outcomes |
| [Protocol reference](docs/ENCOS_BRINGUP.md) | Packet units/encoding, vendor findings, and original bring-up design |
| [Platform roadmap](docs/ROBOT_PLATFORM_ROADMAP.md) | Planned chassis control, localization, and safety architecture |
| [Tests](tests/README.md) | Coverage and native/portable test commands |

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
