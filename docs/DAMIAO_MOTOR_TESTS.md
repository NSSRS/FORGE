# Individual and six-motor arm tests

Prepared 2026-11-04 from `nathan` commit `50b0ffb`. This extends the successful
4340P position-velocity workflow to the inventory's **8009P and 4310P**, plus all
six motors. ENCOS chassis control and calibrated robot motion are separate.
Only the earlier CAN2/ID1 4340P run has physical test evidence. The new paths
have software verification; they have not been flashed or run on motors.

## Wiring and test plans

| Template in `src/robot_description/config/` | Scope | CAN assignment |
| --- | --- | --- |
| `bench_8009p.json` | One 8009P | CAN1 alone |
| `bench_4310p.json` | One 4310P | CAN2; change to CAN3 if wired there |
| `bench_4340p.json` | One 4340P | CAN2 |
| `bench_arm.json` | All six | CAN1: 8009P; CAN2: two 4340P and three 4310P |
| `bench_arm_final.json` | All six | CAN1: 8009P; CAN2: two 4340P; CAN3: three 4310P |

CAN1 isolation is enforced in both the host plan and firmware. All buses use
classic 11-bit CAN at 1 Mbps. The MCU repeats commands at 500 Hz per enabled
motor. USB state reads are sequential; this is not synchronized trajectory
control or a measurement of actual CAN timing.

The templates deliberately leave IDs, protocol ranges, absolute fixture travel
and torque trip thresholds null. Copy one into ignored `build/arm_tests/`, then
fill it from isolated discovery and the actual fixture. The original production
`arm_motors*.json` calibration/confirmation fields remain unchanged.

## Build and discovery

Build matching firmware and flash it using the established ST-LINK procedure
in [DAMIAO_ARM.md](DAMIAO_ARM.md). Rebuild even if the old 4340 test worked:
the new runner requires capability `0x100` (HELLO capability mask `0x18f`).

```bash
python3 scripts/fetch_mc02_sdk.py
python3 scripts/build_dm_mc02.py
python3 scripts/check_dm_mc02.py --port /dev/ttyACM0
```

After flashing, reset the MC02 before a new test or discovery session. Reset
with the mechanism supported and motors disabled: no test automatically resets
a latched fault or clears motor faults. No software command flashes hardware.

Discover one physical unit at a time, using its actual bus:

```bash
python3 scripts/discover_dm_motor.py --port /dev/ttyACM0 --bus 1
python3 scripts/discover_dm_motor.py --port /dev/ttyACM0 --bus 2
```

Record the physical label/model/revision, command ID, feedback ID, PMAX, VMAX,
TMAX, CTRL_MODE, TIMEOUT and Deta. Proposed inventory IDs are not discoveries.
Readback does not identify the physical model: `identity_confirmed` records
the operator's label/wiring verification. The P-series manuals and hardware
labels must agree; do not substitute generic non-P model limits.

Before sharing CAN2/CAN3, assign distinct command and feedback IDs using the
Damiao configuration tool with one motor connected at a time. This runner never
changes IDs. Command IDs must be 1..15; feedback IDs may be 0 for an isolated
unit or 16..2046. Duplicate feedback IDs on one bus are rejected, including 0.
The same command ID can be used on different buses.

## Fill and run a single-motor plan

```bash
mkdir -p build/arm_tests
cp src/robot_description/config/bench_8009p.json build/arm_tests/8009p.json
# Edit build/arm_tests/8009p.json with the measured values below.
python3 scripts/test_dm_arm.py --plan build/arm_tests/8009p.json
python3 scripts/test_dm_arm.py --plan build/arm_tests/8009p.json --check
python3 scripts/test_dm_arm.py --plan build/arm_tests/8009p.json --execute \
  --report build/arm_tests/8009p-result.json
```

Use `bench_4310p.json` in the same workflow for each of the three 4310Ps, and
`bench_4340p.json` for the remaining 4340P. Give every physical unit a distinct
`unit` label; the plan list order is a temporary firmware slot order, not a
calibrated arm joint map. `--port` defaults to `/dev/ttyACM0`.

Plan fields:

- `ranges.pmax/vmax/tmax`: actual register readbacks; the host compares these
  again before configuration and the MCU probes them for feedback decoding.
- `lower_rad/upper_rad`: measured absolute raw output-shaft fixture boundaries,
  including clearance. No gearbox multiplication or simulated joint offsets.
- `torque_trip_nm`: explicit measured-feedback trip threshold, at most 3 Nm for
  this unloaded test tool. This is **not** a torque/current command clamp.
  Set appropriate motor-side current limits with the vendor tool first; do not
  infer a safe threshold from the motor's rated or peak torque.
- `speed_rad_s`: positive position-mode velocity limit, at most 0.1 rad/s.
  `speed_trip_rad_s`: measured overspeed threshold, 0.03..0.3 rad/s, no lower
  than the command speed. Template values are test requests to review, not
  model ratings or claims of safety for an assembled mechanism.
- `delta_degrees`: signed 2..45 degree excursion; default +5 degrees. Travel
  divided by speed plus one second must fit the 12-second host window.
- `fixture_confirmed` and per-unit `identity_confirmed`: set true only after
  securing an unloaded motor, verifying wiring/model and filling real limits.

No action flag validates/prints a complete plan offline and opens no port.
`--check` reads registers only, including positive existing Deta, without mode,
timeout, disable or enable writes. `--execute` performs the following:

1. Read/compare every selected motor's identity, ranges and initial mode (1/2).
2. Configure probing, obtain fresh stationary disabled feedback, and calculate
   a narrow origin-to-target corridor with 0.05 rad clearance on each side.
   Reject any corridor outside the supplied fixture limits.
3. Set and independently read back runtime TIMEOUT=2000 (100 ms), then mode 2
   where needed. No flash save, zeroing, gain tuning or calibration is performed.
4. Probe mode/ranges/timeout again, preload the measured position, and wait for
   enabled feedback before sending a movement target.
5. Move outward, require error within 1 degree and speed <=0.03 rad/s for 0.3 s,
   disable, then return to the measured origin and verify again. Each leg has a
   12-second host deadline and a 15-second independent MCU cutoff.
6. Require fresh stationary disable acknowledgement and restore/read back each
   changed original mode. Runtime timeout remains 100 ms; it is not saved.

Close, error, stall and Ctrl+C all enter cleanup. No automatic re-enable occurs.
A missing disable acknowledgement or failed restoration is a failed run, with
instructions to turn off motor power or power-cycle before reuse. Disabling
torque is not a brake; keep the mechanism supported throughout.

## Full-arm motor bench test

After all **six physical units** pass individually, copy the appropriate arm
template and fill their measured records. Set `individual_test_passed` for each
only from those results. This is an unloaded, supported motor bench test, not
validation of a loaded assembled arm, gravity hold or simulation coordinates.

```bash
cp src/robot_description/config/bench_arm.json build/arm_tests/all-arm.json
# Fill actual IDs/ranges/fixture limits and individual test records.
python3 scripts/test_dm_arm.py --plan build/arm_tests/all-arm.json --all --check
python3 scripts/test_dm_arm.py --plan build/arm_tests/all-arm.json --all --execute \
  --report build/arm_tests/all-arm-result.json
```

The full sequence tests each motor outward/return individually on the shared
harness, then all six outward together and all six back together: 14 legs.
The MCU validates all selected slots before group enable. All motors must
acknowledge enable before targets change. Firmware stops the whole configured
arm on one motor's fault, lease expiry, feedback loss, excess travel, speed,
torque or temperature. Existing board CAN/USB/watchdog faults remain active.
No register diagnostics run while motors are enabled.

Reports include the submitted plan, raw register readings, completed legs,
target errors, elapsed time, sampled peak speed/torque, errors and cleanup
results. Sampled peaks are USB polling observations, not high-rate extrema.
Physical bus utilization, jitter, electrical fault recovery and loaded stopping
still require instrumented hardware tests. Slow USB exchanges fail closed under
the existing leases; this code does not extend watchdogs to hide transport delay.

## Manufacturer code and description checked

Reviewed 2026-11-04:

- [Official DM SDK, `control_Pos_Vel`, parameter writes and enable/disable](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py).
  It sends position/velocity to `0x100 + motor ID`, and base-ID special commands.
  Its generic non-P range table is not used as a P-series hardware calibration.
- [8009P V1.1 manual V1.0](https://github.com/dmBots/DM-J8009P-2EC/blob/42eaaf7f05cccba18d182b51504ee0349e16cff6/manual/DM-J8009P-2EC%20V1.1%20Gear%20Motor%20User%20Manual%20V1.0%202026-09-10.pdf),
  printed page 17: two little-endian float32 fields, position then velocity limit.
- [4310P manual V1.3](https://github.com/dmBots/DM-J4310P-2EC/blob/9e728c9a42b498380ae43964636b3d66d01f42c7/manual/DM-J4310P-2EC%20Geared%20Motor%20User%20Manual%20V1.3%202026-09-14.pdf),
  printed page 16: the same payload; velocity sets the trapezoidal motion limit.
  Both register tables define PMAX/VMAX/TMAX as configurable mapping ranges,
  CTRL_MODE at 10, TIMEOUT at 9 (50 microseconds/count), and Deta at 31.

No motor firmware, internal gains, model geometry or production ROS mode changed.
Older DM firmware can differ in special-command routing; there is no blind
fallback. Missing enable confirmation must be investigated, not bypassed.

## Software verification

```bash
cmake -S src/damiao_firmware -B build/damiao_tests
cmake --build build/damiao_tests
ctest --test-dir build/damiao_tests --output-on-failure
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_damiao*.py' -v
```

The new independent fake-CAN plant runs the real C state machine/USB codec for
all model ranges and both bus layouts, including negative travel, factory reply
ID 0, stalls, mode mismatch, missing feedback, speed/torque/lease faults, group
enable, firmware routing guards and cleanup. Simulated 500 Hz scheduling is not
physical bus timing. Existing 4340P and production MIT tests remain covered.
