# Individual and six-motor arm tests

## Current status — 2026-11-05

All six IDs were written and read back: A1..A6 command IDs 1..6, feedback
IDs 17..22; A1 is on CAN1, A2..A6 on CAN2. A1 power-cycle persistence is
verified; A2..A6 post-power-cycle readback remains pending.
The saved `build/arm_tests/all-six-motion-result.json` passes all 14 stages
(group outward/return, then each motor outward/return), with disable and mode
restoration verified. The operator reported the later 180° test worked;
the currently saved 180° report is interrupted, so it is not full-run evidence.
These are unloaded shaft checks, not joint calibration or loaded-arm validation.

The [position API and PyQt window](ARM_POSITION_JOG.md) support A1..A6 selection,
E once per selection, held arrows at 45°/s and release-to-stop; default window
is ±90° from startup. Focus loss, selection changes and explicit stop disarm.
59 Python tests, including native-backed firmware and Qt events, pass; physical
window jogging remains unverified. Existing 180° firmware supports this window
without a new flash. The separate cached-telemetry firmware is built and tested
but still awaits deployment and bus qualification. See [cross-layer TODOs](ARM_LAYER_TODOS.md).

## Repeat runs without MCU reset (2026-11-05)

The routed `test_dm_arm.py` runner now refreshes stationary disabled feedback
before each preflight register read when it detects retained slots from a
previous session. Repeat the same wiring/slot layout without resetting or
flashing MC02. `--check` may send disable commands to refresh feedback; it
never enables motors or writes motor parameters. Active/faulted sessions are
rejected without clearing faults. Changed wiring/slot layouts, ID-session
locks and CAN failures still need explicit recovery. This supersedes routine
reset-between-tests guidance for successful routed sessions.

Validated with 48 Python tests, including repeated group-first runs without
reset and preserved fault latches. Hardware check reproduced retained
six-slot configuration and expired feedback using disabled-only probes;
subsequent all-six preflight passed without reset in 0.464 s. No motors were
enabled during this validation. Report:
`build/arm_tests/session-reuse-hardware-check.json`.

## Historical implementation notes

Prepared 2026-11-04 from `nathan` commit `50b0ffb`. This extends the successful
4340P position-velocity workflow to the inventory's **8009P and 4310P**, plus all
six motors. ENCOS chassis control and calibrated robot motion are separate.
At preparation time, only the earlier CAN2/ID1 4340P run had successful physical test evidence.
The routed paths were initially software-verified only; the failed 2026-11-05
8009P attempt and subsequent diagnostic changes are recorded below.

## 2026-11-05 bench attempt and diagnostic update

The routed firmware was flashed and discovery succeeded with one 8009P on
CAN1 and one 4310P on CAN2; the 4340P was disconnected. Both units returned
command ID 1, feedback ID 0, mode 1, timeout 0 and Deta 4. Ranges were
12.5/45/54 for the 8009P and 12.5/30/10 for the 4310P (rad, rad/s, Nm).

The secured, unloaded, unobstructed 8009P attempted +5 degrees at 0.05 rad/s,
with a 0.15 rad/s speed trip and 1 Nm measured-torque trip. Firmware latched
speed fault 8 before any leg completed. The old firmware did not retain the
triggering velocity; the cause is not established. Cleanup failed to confirm
stationary disable and did not verify mode restoration. Operator confirmed
motor power off. The 4310P has not been motion-tested. Neither unit passed.

The host now retries disable requests at 20 ms intervals within a one-second
deadline to obtain fresh feedback while an already disabled shaft coasts.
Missing feedback or continued movement still fails cleanup. Motion limits,
watchdogs and the fault latch are unchanged.

Firmware capability 0x200 (HELLO mask 0x38f) adds read-only USB type 19 for the
first-fault sample: reason/slot/status, timestamps/ages, measured position,
velocity/torque, commanded position/velocity and speed/torque thresholds.
Sample flags identify whether feedback and a command existed at the fault.
Later disable feedback does not overwrite this record. Hosts support older
firmware by reporting a null snapshot. Failure reports request disable before
reading diagnostics; they never clear the fault or re-enable a motor.

The later 45-degree attempt at 0.2617994 rad/s also stopped on speed fault 8:
latched velocity 0.3186798 rad/s exceeded the 0.3 rad/s feedback threshold;
latched torque was 0.6461525 Nm, below the 1 Nm torque threshold. No leg
completed. Both stationary disable acknowledgement and original-mode
restoration succeeded with the updated cleanup logic. This verifies the
cleanup recovery on that attempt, not motor speed tracking or a passing test.

The user explicitly approved raising the feedback trip from 0.3 to 0.5 rad/s
for the next secured, unloaded function tests. Plans retain the 0.2617994 rad/s
command, leaving margin above the captured 0.3186798 rad/s sample. The routed
host/MCU guard now allows configuring up to 0.5 rad/s. This is not a motor
rating, diagnosis of the transient, or loaded-operation validation.
Torque/travel/temperature stops, watchdogs and fault latching remain unchanged.

The next attempt with the approved 0.5 rad/s speed trip stopped on measured
torque fault 9: 1.0681305 Nm exceeded the selected 1 Nm threshold, with
feedback velocity 0.0988998 rad/s. No stage completed. Disable and original
mode restoration were verified. The 1 Nm value is an assistant-selected
unloaded test threshold, not a manufacturer motor rating. No torque-limit
increase has been authorized or applied; the 4310P remains motion-untested.

Current-protection verification was explicitly deferred by the user for these
basic free-shaft checks; see [all-motor protection TODO](MOTOR_PROTECTION_TODO.md).
The temporary bench plan's outer bounds describe an unobstructed shaft and
protocol range, not measured/calibrated joint travel. Production profiles
remain unconfirmed. The diagnostic update still requires hardware validation.

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
- `torque_trip_nm`: explicit measured-feedback trip threshold, bounded by
  the model-specific envelope below and discovered TMAX. This is **not** a torque/current command clamp.
  Set appropriate motor-side current limits with the vendor tool first; do not
  infer a safe threshold from the motor's rated or peak torque.
- `speed_rad_s`: positive position-mode velocity limit, at least 0.001 rad/s
  and no greater than the configured `speed_trip_rad_s`. The fixed 0.1 rad/s
  command cap was removed at user request on 2026-11-05 in host and firmware;
  rebuild/reflash before using faster commands. 45 degrees / 3 seconds is
  0.2617994 rad/s; acceleration and settling mean this is a nominal travel
  speed, not a guaranteed three-second completion time.
  `speed_trip_rad_s`: measured overspeed threshold, at least 0.03 rad/s and no greater than the model-specific
  envelope below or discovered VMAX, no lower
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

## Successful 8009P function test and timing

The 8009P completed the 45-degree outward/return test with manufacturer-rated
feedback thresholds. Outward elapsed time including settling was 3.583802 s,
final error 0.08397 degrees; return elapsed time was 3.551556 s, final error
0.02186 degrees. Disable and original-mode restoration both passed. Sampled
peak speeds were 0.472527/0.494507 rad/s; sampled peak torques were
1.279121/1.252747 Nm. These are USB samples, not high-rate extrema. The 4310P
remains untested; full-arm calibration/protection verification remain pending.

Comparison with `scripts/test_dm_4340_bench.py` confirms both use CTRL_MODE=2,
CAN ID 0x100+command ID and two little-endian float32 values: final position
and speed limit. Both use FORGE's MC02 transport, not a direct vendor SDK
import. Neither motion path uses MIT gains or a host position ramp. Initial
mode 1 readback is the motor's original mode, which is restored after cleanup.

The routed runner now prints connection/preflight, preparation, enable and
per-leg elapsed timings, velocity and torque. Reports include the first USB
sample displaced at least 0.1 degrees from the initial hold and the first sample
within one degree of target. Those observations do not measure exact CAN or
mechanical onset. This logging change requires no firmware flash and does not
alter the movement commands or settling checks.

## Manufacturer-based limits (2026-11-05)

At the user's explicit request, the active 8009P/4310P free-shaft plans now
use the manufacturer's continuous rated torque and speed as feedback stops.
The command remains 0.2617994 rad/s for nominal 45-degree/3-second travel.
These are reactive feedback stops, not motor-side current clamps or evidence
of loaded-arm safety. Narrow travel bounds, disabled preloading, temperature
checks, watchdogs, fault latching and cleanup remain active.

| Model | Rated torque / selected torque trip | Peak torque (not selected) | Selected speed trip |
| --- | --- | --- | --- |
| 8009P V1.1 | 20 Nm | 40 Nm | 100 rpm = 10.4719755 rad/s (24 V rating) |
| 4310P | 3.5 Nm | 12.5 Nm | 120 rpm = 12.5663706 rad/s (both variants) |
| 4340P | Existing 3 Nm bench ceiling retained | Outside this change | Existing 0.5 rad/s bench ceiling retained |

Sources: specification page 7 of the official [8009P V1.1 manual V1.0](https://github.com/dmBots/DM-J8009P-2EC/blob/42eaaf7f05cccba18d182b51504ee0349e16cff6/manual/DM-J8009P-2EC%20V1.1%20Gear%20Motor%20User%20Manual%20V1.0%202026-09-10.pdf)
and [4310P manual V1.3](https://github.com/dmBots/DM-J4310P-2EC/blob/9e728c9a42b498380ae43964636b3d66d01f42c7/manual/DM-J4310P-2EC%20Geared%20Motor%20User%20Manual%20V1.3%202026-09-14.pdf).

The 8009P manual also rates 200 rpm at 48 V; supply voltage is not recorded,
so the selected bound uses the 24 V rating. The 4310P's measured TMAX is 10 Nm,
which cannot represent its published 12.5 Nm peak; TMAX is left unchanged and
the selected 3.5 Nm continuous threshold remains representable. Peak/no-load
ratings are not treated as continuous capability. Motor internal current,
acceleration, gains and persistent protection registers are unchanged.
Reflash the rebuilt MC02 firmware before using these updated plan thresholds.

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

## A2–A6 rig check and slow manual test (2026-11-10)

Use the [rig procedure](ARM_RIG_TEST.md) when A1 is temporarily disconnected.
It includes a disabled connection/encoder capture and explicitly supported
keyboard or optional Pocket jogging at 5 degrees/s with explicit manual travel. No automatic sweep
or gravity holding is authorized by this profile.
