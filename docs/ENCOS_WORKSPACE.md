# ENCOS motor software workspace

Ubuntu 24.04 on the Intel NUC. Hardware route: dedicated Ethernet → ENCOS EtherCAT-CAN bridge → classic CAN motors. Begin with one bridge and one motor on CAN1.

## Layout

| Path | Purpose |
|---|---|
| `src/encos_query/` | Project-owned query-only bridge and motor diagnostic |
| `docs/` | Bring-up guide, bench procedure, and file migration record |
| `config/bench-equipment.example.md` | Redacted equipment-record template |
| `tests/` | Future host-side protocol and controller tests |
| `scripts/` | Build helpers |
| `build/` | Generated build output |
| `logs/` | Hardware test recordings |
| `tmp/` | Temporary files, including existing PDF extracts |

The local `vendor/` folder holds the supplier material used during bench work. It is excluded from Git along with the extracted supplier demo. The build fetches the pinned, public upstream SOEM v1.4.0 source into the build directory.

## Build on Ubuntu

Install the development tools if needed:

```bash
sudo apt update
sudo apt install build-essential cmake git unzip ethtool iproute2 ripgrep
```

From this workspace:

```bash
./scripts/build.sh
```

The helper fetches public upstream SOEM on the first run, builds `build/encos_query/encos_query` and the bounded `build/encos_query/encos_motion` test, then runs host-side protocol tests. It does not access hardware.

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

The bounded motion utility accepts the same explicit ID list. It commands each listed motor through PDO slots 0–3 every 10 ms, monitors type-2 feedback for every motor, and stops all outputs if any motor exceeds a safety limit:

```bash
sudo ./build/encos_query/encos_motion enp86s0 2.0 45 --motor-ids 1,2 --execute
```

The initial profile ramps outward over 10 seconds at a 1 rpm ceiling, holds 0.5 seconds, and returns over 10 seconds. Start with one motor and 2.0 A only if that phase-current ceiling is approved for the actual motor and fixture. Do not run a multi-motor motion until IDs, termination, direction, clearance, and individual feedback are verified.

Read [the protocol and architecture guide](docs/ENCOS_BRINGUP.md) for command units and known example defects. Copy [the equipment template](config/bench-equipment.example.md) to `config/bench-equipment.md` to record actual hardware; that local record is ignored by Git.

Both programs compile on this Ubuntu 24.04 NUC, and the query protocol tests pass. The query utility has been run successfully with a powered motor.
