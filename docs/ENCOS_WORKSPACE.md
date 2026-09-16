# ENCOS motor software workspace

Current development: `main` now contains controls only. The physics simulators
are on `physics-analysis`. The new C11 shared driver and Python per-motor position
and velocity API are documented in [PYTHON_CONTROL.md](PYTHON_CONTROL.md). Its
100 Hz loop is separate from the older bench CLI below. Both accept up to four
motors in configuration order: slots 0-2 use CAN1 and slot 3 uses CAN2.

Ubuntu 24.04 on the Intel NUC. Hardware route: dedicated Ethernet → ENCOS EtherCAT-CAN bridge → classic CAN motors. Begin with one bridge and one motor on CAN1.

## Layout

| Path | Purpose |
|---|---|
| `src/encos_query/` | C11 protocol codec, shared driver, and bench utilities |
| `src/forge_motors/` | Python API, ctypes binding, and hardware-free simulator |
| `src/forge_motor_control/` | ROS 2 driver, keyboard publisher, and commissioning config |
| `docs/` | Protocol, bench procedure/results, Python/ROS usage, and platform roadmap |
| `docs/bench-equipment.example.md` | Redacted equipment-record template |
| `tests/` | Host-side protocol, simulation, and fake-native-driver tests |
| `scripts/` | Build/ROS helpers, Python simulation example, and guarded commissioning commands |
| `build/` | Generated native (`encos_query/`) and ROS (`ros2/`) build output |
| `logs/` | Generated hardware recordings and colcon logs (untracked; create when needed) |
| `install/ros2/` | Generated ROS package installation (untracked) |
| `tmp/` | Temporary files, including existing PDF extracts |

The local `vendor/` folder holds the supplier material used during bench work. It is excluded from Git along with the extracted supplier demo. The build fetches the pinned, public upstream SOEM v1.4.0 source into the build directory.

## Updating from the previous layout

The source folders formerly named `python/` and `ros2_ws/src/` are now under
`src/`. The simulation example is `scripts/independent_motors.py`, and the DDS
profile belongs to the ROS package's `config/` directory. Public Python imports
and ROS executable names are unchanged.

After pulling this reorganization, rebuild the ROS package using the
[updated command](ROS2_MOTOR_CONTROL_STATUS.md#build-and-run), then start a fresh
terminal and source `scripts/ros2_env.sh`. The new build/install paths avoid
reusing colcon caches or symlinks that point to the former source location.
For an existing editable Python installation, rerun `python -m pip install -e .`
with that environment's interpreter. For source-tree use, set `PYTHONPATH=src`.

If a local `config/bench-equipment.md` exists, move it to
`docs/bench-equipment.md`; both paths remain ignored to protect local records.
Existing `ros2_ws/` build artifacts remain ignored and are no longer used.
The native C build path is unchanged. Create `logs/` locally when saving runs.

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
`build/encos_query/`. It then runs C protocol and Python/native integration tests
against a fake bridge; no motor hardware is accessed. See [test coverage and
portable checks](../tests/README.md), [Python installation](PYTHON_CONTROL.md#build-and-install),
and the separate [ROS 2 build](ROS2_MOTOR_CONTROL_STATUS.md#build-and-run).

## Current software status

The project-owned utilities use a pinned public release of SOEM. The private local supplier distribution remains outside the repository.

Use `sudo ./build/encos_query/encos_query enp86s0` for automatic discovery when exactly one motor is powered on the CAN bus. It validates the 86-byte output and 92-byte input PDOs, then reads motor ID, position, versions, and CAN timeout. It sends no control, configuration, zeroing, brake, or movement commands.

For two to four powered motors, use their known, unique CAN IDs and avoid broadcast discovery:

```bash
sudo ./build/encos_query/encos_query enp86s0 --motor-ids 1,2,3,4
```

### Assigning duplicate CAN IDs

The ID-change command is addressed through CAN ID `0x7FF` but matches the **old motor ID in its payload**. If two motors are both ID 1, both would accept a `1 -> 2` command. Therefore power off and disconnect every other motor before changing an ID. The guarded utility sends the documented command once, waits for the success reply, and then clears bridge output:

```bash
# With only the second motor connected and powered:
sudo ./build/encos_query/encos_assign_id enp86s0 1 2 --execute

# Power-cycle it, then verify only that motor:
sudo ./build/encos_query/encos_query enp86s0
```

Repeat with one isolated motor at a time. Do not use `encos_assign_id` while duplicate IDs share the powered CAN bus.

The bounded motion utility accepts the same explicit ID list. It commands each listed motor through PDO slots 0–3 with a target 1 ms loop, monitors type-2 feedback for every motor, and stops all outputs if any motor exceeds a safety limit. Slot 3 routes to CAN2 according to the bridge guide; four IDs do not mean four motors on CAN1:

```bash
sudo ./build/encos_query/encos_motion enp86s0 2.0 1 --motor-ids 1,2 --execute
```

The initial profile targets a 1 kHz loop, ramps outward over 2 seconds at a 5 rpm ceiling, holds 0.5 seconds, and returns over 2 seconds. Standard Linux scheduling can add jitter, so treat 1 kHz as a target and inspect the resulting motion/feedback log before relying on it. Start with one motor and 2.0 A only if that phase-current ceiling is approved for the actual motor and fixture. Do not run a multi-motor motion until IDs, termination, direction, clearance, and individual feedback are verified.

For a three-motor verification sequence, after IDs 1, 2, and 3 have individually passed telemetry, run the three motors together, then ID 1, ID 2, and ID 3 in turn. Every stage uses the bounded out-and-return profile and aborts the remaining sequence if a stage fails:

```bash
sudo ./scripts/run_three_motor_sequence.sh enp86s0 2.0 1 --execute
```

Read [the protocol and architecture guide](ENCOS_BRINGUP.md) for command units and known example defects. Copy [the equipment template](../docs/bench-equipment.example.md) to `docs/bench-equipment.md` to record actual hardware; that local record is ignored by Git.

Dated hardware outcomes and remaining limits are recorded in the
[bench commissioning record](ENCOS_TEST_BENCH.md#commissioning-record-2026-09-16).
