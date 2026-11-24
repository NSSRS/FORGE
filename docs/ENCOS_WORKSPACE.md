# ENCOS motor software workspace

Current development is on `nathan`, with the shared C11 driver and Python
position/velocity API in [PYTHON_CONTROL.md](PYTHON_CONTROL.md). MotorBus uses
a 100 Hz owner loop, separate from the older bench CLI. It supports 1-4 motors,
with contiguous slots 0-3 by default or explicit `motor_slots`.

The operator-confirmed Nov 24 bench uses four IDs in slots 0/1/3/4: IDs 1/2 on
CAN1, IDs 3/4 on CAN2. Use [CAN2_BENCH.md](CAN2_BENCH.md) for current commands.
Query and isolated ID-assignment utilities accept `--can-bus 1|2`; normal
successful reuse does not require resetting IDs. Native Ubuntu 24.04 on the
NUC uses dedicated `enp86s0` -> EtherCAT IN -> ENCOS bridge -> CAN motors.

## Layout

| Path | Purpose |
|---|---|
| `src/encos_driver/` | C11 protocol codec, shared driver, and bench utilities |
| `src/motors/` | Python API, ctypes binding, and hardware-free simulator |
| `src/motor_control/` | ROS motor driver and chassis mixing |
| `src/imu/`, `src/stereo_cam/`, `src/teleop/`, `src/visualization/` | Independent sensor, command input and display packages |
| `src/bringup/` | Shared profiles, launch files and session manager |
| `docs/` | Protocol, bench procedure/results, Python/ROS usage, and platform roadmap |
| `docs/bench-equipment.example.md` | Redacted equipment-record template |
| `tests/` | Host-side protocol, simulation, and fake-native-driver tests |
| `scripts/` | Build/ROS helpers, Python simulation example, and guarded commissioning commands |
| `build/` | Generated native (`encos_driver/`), ROS (`ros2/`), and viewer (`motor_viewer/`) assets |
| `logs/` | Generated hardware recordings and colcon logs (untracked; create when needed) |
| `install/ros2/` | Generated ROS package installation (untracked) |
| `tmp/` | Temporary files, including existing PDF extracts |

The local `vendor/` folder holds the supplier material used during bench work. It is excluded from Git along with the extracted supplier demo. The build fetches the pinned, public upstream SOEM v1.4.0 source into the build directory.

## Updating from the previous layout

The current source ownership is documented in [source organization](SOURCE_LAYOUT.md).
`src/encos_query` is now `src/encos_driver`; the native binary `encos_query`
keeps its name. `forge_motors` is now the Python module `motors`.
The former `forge_motor_control` package is split into `motor_control`,
`teleop`, `visualization`, and `bringup`; IMU acquisition lives in `imu`.
ROS executable and node names omit `forge_`; topic names are unchanged.
Shared launch files and YAML/DDS profiles live in `src/bringup/`.

Start a fresh Ubuntu terminal, run `./scripts/build.sh`, rebuild the six ROS
packages using the [ROS build command](ROS2_MOTOR_CONTROL_STATUS.md#build-and-run),
and source `scripts/ros2_env.sh`. Native output is `build/encos_driver/`;
ROS output is `build/ros2/` and `install/ros2/`. Do not source old installations.
For an existing editable Python installation, rerun `python -m pip install -e .`
with that environment's interpreter. For source-tree API use, set `PYTHONPATH=src`.

Use the current output paths above. Older build folders, when present, are
historical artifacts and may contain a reusable pinned SOEM checkout.
The ignored source backup remains `tmp/before-package-reorganization.tar.gz`.
If a local `config/bench-equipment.md` exists, move it to
`docs/bench-equipment.md`; both paths remain ignored to protect local records.
Create `logs/` locally when saving runs.

## Build on Ubuntu

Install the development tools if needed:

```bash
sudo apt update
sudo apt install build-essential cmake git python3 python3-venv unzip ethtool iproute2 ripgrep
```

From this workspace:

```bash
./scripts/build.sh
```

The helper fetches pinned public upstream SOEM v1.4.0 on the first run and builds
`encos_query`, `encos_assign_id`, `encos_motion`, and `libencos_driver.so` under
`build/encos_driver/`. It then runs C protocol and Python/native integration tests
against a fake bridge; no motor hardware is accessed. See [test coverage and
portable checks](../tests/README.md), [Python installation](PYTHON_CONTROL.md#build-and-install),
and the separate [ROS 2 build](ROS2_MOTOR_CONTROL_STATUS.md#build-and-run).

## Current software status

The project-owned utilities use a pinned public release of SOEM. The private local supplier distribution remains outside the repository.

Use `sudo ./build/encos_driver/encos_query enp86s0` for automatic discovery when exactly one motor is powered on the CAN bus. It validates the 86-byte output and 92-byte input PDOs, then reads motor ID, position, versions, and CAN timeout. It sends no control, configuration, zeroing, brake, or movement commands.

For multiple motors on CAN1, use known, unique CAN IDs and avoid broadcast discovery:

```bash
sudo ./build/encos_driver/encos_query enp86s0 --motor-ids 1,2,3
```

`encos_query` defaults to slot 0/CAN1; add `--can-bus 2` for slot 3/CAN2.
Every ID in one invocation must be on the selected bus. Discovery requires a
single isolated powered motor. Query/assignment CLIs share the native driver's
interface lock. See the [motor-4 procedure](CAN2_BENCH.md#motor-4-identification-and-four-motor-test)
for CAN2 identification, isolated assignment and four-motor testing.

### Assigning duplicate CAN IDs

The ID-change command is addressed through CAN ID `0x7FF` but matches the **old motor ID in its payload**. If two motors are both ID 1, both would accept a `1 -> 2` command. Therefore power off and disconnect every other motor before changing an ID. The guarded utility sends the documented command once, waits for the success reply, and then clears bridge output:

```bash
# With only the second motor connected and powered:
sudo ./build/encos_driver/encos_assign_id enp86s0 1 2 --execute

# Power-cycle it, then verify only that motor:
sudo ./build/encos_driver/encos_query enp86s0
```

Both discovery and assignment accept `--can-bus 2`; assignment verifies that
isolated discovery matches OLD_ID before sending the configuration frame.
Repeat with one isolated motor at a time. Do not use `encos_assign_id` while duplicate IDs share the powered CAN bus.

The bounded motion utility accepts the same explicit ID list. It commands each listed motor through PDO slots 0–3 with a target 1 ms loop, monitors type-2 feedback for every motor, and stops all outputs if any motor exceeds a safety limit. Slot 3 routes to CAN2, but this legacy utility sends initial position queries
through slot 0/CAN1. It is therefore not a working four-motor split-bus
commissioning path. Use it for the original one-to-three-motor CAN1 bench;
use mapped MotorBus/ROS for CAN2:

```bash
sudo ./build/encos_driver/encos_motion enp86s0 2.0 1 --motor-ids 1,2 --execute
```

The initial profile targets a 1 kHz loop, ramps outward over 2 seconds at a 5 rpm ceiling, holds 0.5 seconds, and returns over 2 seconds. Standard Linux scheduling can add jitter, so treat 1 kHz as a target and inspect the resulting motion/feedback log before relying on it. Start with one motor and 2.0 A only if that phase-current ceiling is approved for the actual motor and fixture. Do not run a multi-motor motion until IDs, termination, direction, clearance, and individual feedback are verified.

For the original all-CAN1 three-motor verification sequence, after IDs 1, 2, and 3 have individually passed telemetry, run the three motors together, then ID 1, ID 2, and ID 3 in turn. Every stage uses the bounded out-and-return profile and aborts the remaining sequence if a stage fails:

```bash
sudo ./scripts/run_three_motor_sequence.sh enp86s0 2.0 1 --execute
```

Read [the protocol and architecture guide](ENCOS_BRINGUP.md) for command units and known example defects. Copy [the equipment template](../docs/bench-equipment.example.md) to `docs/bench-equipment.md` to record actual hardware; that local record is ignored by Git.

Dated hardware outcomes and remaining limits are recorded in the
[bench commissioning record](ENCOS_TEST_BENCH.md#commissioning-record-2026-09-16).
