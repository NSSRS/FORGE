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

Use `sudo ./build/encos_query/encos_query enp86s0` for the one-bridge, one-motor query test after the powered bench is prepared. It validates the 86-byte output and 92-byte input PDOs, then reads motor ID, position, versions, and CAN timeout. It sends no control, configuration, zeroing, brake, or movement commands.

The separate motion test requires a phase-current limit, positive travel in degrees, and `--execute`. For the initial 45-degree test, it ramps outward over 10 seconds at a 1 rpm ceiling, holds 0.5 seconds, returns over 10 seconds, and logs type-2 feedback. Start at 2.0 A only if that phase-current ceiling is approved for the actual motor and fixture.

Read [the protocol and architecture guide](docs/ENCOS_BRINGUP.md) for command units and known example defects. Copy [the equipment template](config/bench-equipment.example.md) to `config/bench-equipment.md` to record actual hardware; that local record is ignored by Git.

Both programs compile on this Ubuntu 24.04 NUC, and the query protocol tests pass. The query utility has been run successfully with a powered motor.
