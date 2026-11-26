# Arm and chassis session handoff

Updated 2026-11-26 on `nathan`. This file describes the current baseline;
[dated measurements](#calibration-and-evidence) retain earlier findings.
Open work belongs in [ARM_LAYER_TODOS.md](ARM_LAYER_TODOS.md) and
[CHASSIS_MODEL_COMMS_TODOS.md](CHASSIS_MODEL_COMMS_TODOS.md).

## Current status and next session

The operator deferred ROS-02 integration on Nov 26 until another day. No next
physical motor test is selected. Preserve the existing ROS implementation; the
integration resumes only when explicitly selected.

BASE-06 firmware **cd93f9b3 / 0x7FFFF8F** was programmed and verified by the
operator on Nov 26, replacing **f78f6b0d / 0x1FFFF8F**. The image is preserved
under `build/dm_mc02/atomic-targets/`. Agent USB readback matched the new identity:
all six slots unconfigured/disabled, no fault and zero CAN transmissions since
reset. The invalid type-29 slot check returned `INVALID_ARGUMENT` with no slot.
See [deployment evidence](measurements/arm-base06-deployment-2026-11-26.json).
**BASE-06 is complete**, including merged BASE-05/DEPLOY-01/COMMS-04. The operator
held all six joints through `atomic_mode2` for about 7.12 seconds, with 2,142
samples (357 per joint), no dropped samples or recorded fault, at most 2 ms
feedback age and 0.02186 deg peak hold error. Both CAN buses recorded zero TX
failures and bus-off events. Final disable and original motor-mode restoration
passed. After owner close, all 147 cached configuration comparisons passed
against the gun-load profile/session bounds, with all six slots disabled and
fault-free. MCU command mode remains 2 in this cached configuration; the owner
verified drive mode 2 before holding and restored drive mode 1 on close. The
mode-valid cache bit is consequently clear (register mask 23), while range bits
remain valid. This supported stationary check does not establish coordinated
travel accuracy, unsupported load or thermal qualification; it is not a request
for another automatic test. The host retains older-firmware compatibility.
No motor was enabled by the agent.
See [the batch contract](ARM_BASE_LAYER.md#base-06-atomic-group-targets).

Verification: four native CTest cases and 219 Python tests passed, covering
Damiao protocol/bench, group holding, smooth jog, weld hardware adapters and v7
response replay. Interrupted/invalid batches leave all targets and leases
unchanged; an uncertain ACK stops without retry. STM32 compilation succeeded
with existing vendor SDK warnings. The ELF SHA-256 is
`2c9d993753715b7cb8eac618e6c4ecbafa34a93d83a9477912f13025b3654368`;
the BIN SHA-256 is
`3af088b745878d6b302b784cebd8f23e947d17a9795133b977885f4c6b23c7df`.
This is software evidence, not new physical qualification or a v7 tuning change.

The operator selected **v7 for welding** and deferred further weld tuning.
The operator removed the gun-off/gun-on paired hold comparison from the test
plan on Nov 26; do not request gun removal for that comparison.
PROTECT-01 is also deferred by the operator. The
[consolidated work list](ARM_LAYER_TODOS.md) merges duplicate support, fault,
deployment and communications tasks. It separates unfinished implementation
from deferred tests and future features; no extra hardware session is selected
by this cleanup. Existing guards and evidence/qualification status are unchanged.
ENCOS motor 4 is identified as ID 4 on CAN2; all four chassis motors passed
signed motion and stop checks. Routine use needs no ID assignment, firmware
flash or repeated commissioning. The agent's post-flash access was limited to
USB identity/cached state and the invalid readback request described above.

| Subsystem | Current entry point | Status |
| --- | --- | --- |
| Arm welding | [V7 operator command](ARM_RIG_TEST.md#weld-motion-accuracy-test) | ACCURACY-01 measurement/review complete Nov 26; further tuning remains deferred under SPEED-01. V7 remains selected |
| Four ENCOS motors | `bash scripts/run_motor_test.sh --four-motors` | IDs 1/2 CAN1, 3/4 CAN2; slots 0/1/3/4; 5 RPM, 2 A trial limits |
| Combined sensors/Pocket | [Full-system guide](FULL_SYSTEM_TEST.md) | Deferred by operator on Nov 26; chassis not assembled. When resumed, explicitly select `chassis_four_motor_bench.yaml` for four |

The operator reports that the chassis is not assembled and deferred the
combined four-motor Pocket/IMU/camera check on Nov 26. No chassis hardware
session is currently selected; resume when the operator chooses. Retain the
secured, unloaded fixture and explicit 5 RPM / 2 A four-motor profile for that
check, with physical arm output disabled. COMMS-02 is also deferred, and
USB-debug/CAN-logger investigation is removed from the immediate plan.
Host/link-loss stopping and restart behavior remain unverified.

The four-motor report `20261124T110254.496409Z.json` records both +5/-5 RPM
one-second legs and both below-0.5-RPM stop checks for all IDs. Across 115
cached samples: zero motor faults, WKC 3/3, 532 exchanges without failure and
normal owner close. The operator subsequently reported normal visual behavior
in the four-motor keyboard/viewer session. The Terminator window-deregistration
message came from the terminal UI; its exact trigger was not captured.
See the [physical record](measurements/encos-four-motor-physical-review-2026-11-24.json).
On Nov 25, the operator confirmed the [wheel locations, forward signs and
1:1 gearing](DIFFERENTIAL_DRIVE.md#model-source-and-hardware-mapping) in the
four-motor profile, closing CHASSIS-02. Loaded stopping and host-loss
qualification remain open.

### COMMS-01 closure

On Nov 26 the operator verified the updated ENCOS driver's stationary feedback
on all four motors. Each supplied 12 observed position updates in two seconds,
with a 160.006 ms median interval and 158.9 ms peak sampled age. The driver
reported zero failed EtherCAT exchanges. Individual reply-loss tests passed
for ID 1 on CAN1 and ID 3 on CAN2 with all four continuously polled. In
each run the target was absent in all 181 loss-window samples, the other
three motors continued answering, and the target resumed answering both
query types. The operator accepted these two port tests and closed COMMS-01
on Nov 26. The journals measure host-side EtherCAT observations. The
[stationary bench record](measurements/encos-inventory-2026-11-26.json)
contains the test scope, counts and journal checksums.

## BASE-07 command-loss check

The [Nov 26 operator run](measurements/arm-command-loss-review-2026-11-26.json)
completed the selected stationary command-timeout/recovery check. With the arm
and gun supported, the operator suspended the existing manual owner for one
second and resumed it. MC02 recorded a latched command-lease fault at 151 ms
command age with 1 ms CAN feedback age. All six motors were confirmed disabled
before reset; R acknowledged and verified recovery, leaving all six disabled
and jogging disarmed. No hold or movement was activated afterward. Final exit
was 0, with disable acknowledged and mode restoration verified.

The pre-reset fault file intentionally predates reset and records success=false;
the final session contains the successful recovery result. This BASE-07 bench
task is complete. The 151 ms value is fault-detection command age, not physical
stop latency. USB remained connected; moving stop distance, physical stop timing
and other failure cases remain unmeasured. They are not new automatic test tasks.

## V7 baseline and reusable control scheme

The active `src/controls/config/weld_response_gun_20261124.json` exactly matches
physical session `20261124T101206.334308Z-lz462pcc`: tracker version **7**,
response profile version **1**, `periodic_blend` startup and 1.96 kg EE.
[Parameters and future comparison rules](measurements/arm-weld-v7-baseline-2026-11-24.json)
and the [raw archive](measurements/weld/2026-11-24/README.md) preserve this baseline.

Preserve these mechanisms when developing a later controller:

1. Advance commands by exact sampled-reference increments; do not integrate
   instantaneous reference velocity a second time.
2. Apply bounded three-harmonic amplitude/phase compensation to A1/A6 only.
   Inverse gain is capped at 2; command spans including residuals stay within
   4/1 degrees. Keep tiny A2-A5 nominal targets and preparation independent.
3. Disable the old additive weld lead/I/D in response mode. Residual P gain is
   0.15, capped at A1 0.20 degrees and A6 0.05 degrees.
4. Share the original approximately 0.578 s quintic startup reference across
   execution, preview and accuracy scoring. Subsequent nominal cycles remain
   20 mm at the checked 1.730 Hz, from the requested 2 Hz / 12 mm/s path.
5. Keep one cached feedback batch and one command batch per moving tick;
   preserve profile identity, command bounds and runtime protection.

| Physical v7 measurement | Result |
| --- | ---: |
| Recovery request to Ready | 7.428 s |
| Sent weld commands | 561, with zero rejected samples |
| Matched cycles 3-18: fitted tip lag | 3.19 ms |
| Matched cycles: same-time 3D tip RMS | 2.168 mm |
| Matched cycles: same-time orientation peak | 0.759 deg |
| Whole-pass orientation peak, including startup | 1.095 deg |
| Per-cycle stroke median / range | 18.350 / 15.723-19.611 mm |

V7 recorded no fault/guard and verified final disable/mode restoration.
Its 20.050 mm aggregate envelope is not every-cycle stroke. Encoder estimates
are not independent tip measurements; the 1-degree whole-pass maximum was
missed at startup and formal accuracy remains failed. Operator preference does
not waive that criterion or promote calibration.

V8's first physical startup peaked at 1.130 degrees despite an optimistic
model prediction. Its profile and finite-startup implementation remain available
for explicit offline regression, not selected for normal operation. See the
[v8 review](measurements/arm-weld-response-v8-physical-review-2026-11-24.json).
ILC remains a later future TODO; no learning is active.

## Current execution contract

- Continuous welding repeats until manual **Pause / Hold**. `--cycles` sizes
  preview only; omit `--single-pass` in normal runs. The application never
  energizes the welder. Arm/chassis selections independently gate output.
- Hardware Calculate chooses the nearest adjacent equivalent A1 target from
  fresh stationary feedback, synchronizing mapping, bounds and preview before
  planning. Raw feedback is never wrapped. The branch stays fixed during motion;
  simulation uses explicit `--a1-encoder-turns` selection.
- Hardware general floor/self/environment clearance is advisory by operator
  request. Model warnings remain visible. Explicit mechanical stops, D3 closure,
  cable bounds and motor protections remain enforced; simulation stays strict.
- Startup tolerance is **0.5 deg** around the fixed owner preload and checked
  start. Require 0.2 s stationary dwell, with a 2 s activation deadline.
  Arrival remains 0.25 deg; the moving tracking trip remains 3 deg.
- Preparation uses bounded lead/PID, a lag-aware common clock and braking inside
  finite path/correction bounds. The current trial requests 30 deg/s, a 10 s
  goal, optimized staging, 30 mm lift and 2.3 s final lowering. Default speed
  remains 5 deg/s; acceleration <=30 deg/s². Timing goals never override guards.
- Settled endpoint pauses retain the original fixed target/speed allowance.
  Travel interruption captures a measured hold once and requires replanning.
  Ready focus/heartbeat pauses retain the plan and require explicit Start welding.
  Disable/fault/close remove torque; explicit recovery leaves outputs disabled.
- One owner per controller. Telemetry consumes cached feedback and cannot renew
  a motion lease. Do not open another owner or interrupt a loaded powered arm.

See [architecture](WELD_MOTION.md) and [operator procedures](ARM_RIG_TEST.md).

## Deployed profile and completed work — November 19

The Nov 19 deployment used MC02 **f78f6b0d / 0x1FFFF8F** with the profile
below. Nov 26 firmware **cd93f9b3 / 0x7FFFF8F** retains these settings and
passed active-configuration readback. The selected profile remains
`src/robot_description/config/arm_rig_gun_a2_a6_load.json`:

| Joint | Reactive torque trip | Feedback TMAX |
| --- | ---: | ---: |
| A1 | 20 Nm | 54 Nm |
| A2/A3 | 30 Nm | 40 Nm |
| A4/A5/A6 | 10 Nm | 12.5 Nm |

Both 80 C trips and all 1.5 rad/s speed trips remain. These are trial thresholds,
not continuous ratings; drive gains/current settings and standard profiles are
unchanged. Range persistence/readback passed; do not repeat setup for normal use.
Manual `control_arm.sh` also needs this explicit profile. See the
[deployment evidence](measurements/arm-gun-load-review-2026-11-19.json).

## ACCURACY-01 and SPEED-01 review

ACCURACY-01 is complete: the operator ran the v7 motion check and its stroke,
startup and same-time orientation were reviewed. Completion records the measured
performance; it does not claim that every accuracy target passed. Remaining
startup/stroke improvements belong to the existing deferred SPEED-01 work.

The [Nov 23 baseline](measurements/arm-accuracy-speed-review-2026-11-23.json)
preceded the command-sampling correction and v7. Its 2.633-degree orientation
peak and A1/A6 overswing are historical comparisons, not current v7 performance.
The [sampling review](measurements/arm-weld-command-sampling-review-2026-11-24.json)
and [v7 physical comparison](measurements/arm-weld-response-physical-review-2026-11-24.json)
record the progression. The [Nov 26 operator check](measurements/arm-weld-accuracy-review-2026-11-26.json)
reached Ready in 8.562 s on its successful attempt and welded for 14.049 s until
manual pause, with final disable and mode restoration verified. Its 521 confirmed
weld dispatches reconstruct a 1.080-degree startup orientation peak, above the
1-degree maximum; steady orientation peaked at 0.891 deg. The 23 complete steady
cycles measured 17.066-18.812 mm transverse stroke (median 18.034 mm) against
20 mm planned. Matched cycles 3-18 had 2.053 mm same-time tip RMS, compared with
2.168 mm on Nov 24. This is encoder/model evidence, not independent tip measurement
or physical qualification; the 0.5-degree orientation target still applies.

Earlier attempts in that same session recorded one finite-path-envelope fault
after A3 retreat and two stale-feedback faults during preparation, followed by
explicit resets. The successful welding interval recorded no fault. The live
journal stopped on a NumPy boolean in A3 retreat timing and reported 1,589 dropped
records; the final JSON retained all controller traces and cached owner samples.
The review uses that final JSON explicitly, excluding one final incomplete
dispatch. The timing metadata conversion is corrected offline, with a JSON
serialization regression. Confirm live recording on the next operator session;
another run is not needed solely to recover this session's accuracy measurements.
The selected response profile and all motion guards remain unchanged.

## PLAN-01 and BASE-04 resolution

Closed by operator confirmation on Nov 23. Do not reopen historical planning
latency or CAN incidents. The later A1 shortest-target and advisory spatial-policy
fixes are recorded in the Nov 24 evidence below.

## HOLD-01 resolution

Closed by operator confirmation on Nov 23. That session reached Ready in 8.22 s,
held fixed targets for 25.19 s, recorded A5 peak hold error 0.053 deg and verified
final disable/mode restoration without a fault. The [dated review](measurements/arm-accuracy-speed-review-2026-11-23.json)
and [raw archive](measurements/weld/2026-11-23/README.md) preserve the evidence.
Historical untouched A5 movement on Nov 19 remains in its gun-load review;
it is not an open HOLD-01 blocker. General load/fault qualification remains open.

## Calibration and evidence

All six arm motors are connected: A1 CAN1/1/17; A2-A6 CAN2, command IDs 2-6,
feedback IDs 18-22. Feedback is already geared; candidate gain magnitude is one.
Corrected A3 endpoints are CW 72.77 deg / CCW 151.39 deg, standby 149.39 deg.
Its bounded contact retreat permits at most 0.25 deg startup excess, then moves
inward at <=1 deg/s and <=5 deg/s². Physical loaded retreat remains unqualified.

A4 retains +/-270 deg from raw +173.183435236 deg; A6 retains +/-270 deg from
+64.161048535 deg. Never rebase or wrap those cable envelopes. The independent
A5 candidate is `model_deg = -raw_deg + 101.514057878`, raw command bounds
[-96.607604972, +114.836287037] deg. Use its matching mapping and standby.
D3 connection centers remain 150 mm apart. Production calibration stays false.

On Nov 26, the operator confirmed that the physical wrist and model matched
in both saved off-straight poses and in movement direction. The
[A5 visual review](measurements/arm-a5-pose-review-2026-11-26.json) records fresh,
disabled six-axis feedback, unchanged mapping and A5 raw change +14.316 deg
(model change -14.316 deg), without branch/render warnings. Other joints also
moved, so this is full-pose visual evidence rather than an isolated angular
accuracy measurement. This visual subcheck is complete; independent angle
measurements, broader branch coverage, cable margins and CAL-01 qualification
remain open. No calibration anchors or production flags changed.

| Evidence | Scope |
| --- | --- |
| [Nov 26 command-loss/recovery](measurements/arm-command-loss-review-2026-11-26.json) | Stationary host pause, 151 ms lease-fault detection, all-six disabled recovery and normal close |
| [Nov 26 A5 visual comparison](measurements/arm-a5-pose-review-2026-11-26.json) | Two off-straight poses and direction matched by operator observation; no quantitative angle qualification |
| [Nov 24 V7 and ENCOS archive](measurements/weld/2026-11-24/README.md) | Original source bytes, checksums, baseline parameters and four-motor result |
| [A1 shortest recovery](measurements/arm-a1-shortest-recovery-review-2026-11-24.json) | Raw -181.511 -> -174.692 deg, CCW 6.819 deg; synchronized branch |
| [Spatial policy](measurements/arm-spatial-policy-review-2026-11-24.json) | Advisory hardware model space; strict mechanical/cable checks |
| [Hold activation](measurements/arm-hold-activation-review-2026-11-24.json) | Fixed preload reference and 0.5-degree start tolerance |
| [Nov 23 archive](measurements/weld/2026-11-23/README.md) | Earlier baseline, original journals and load context |
| [Nov 19 gun load](measurements/arm-gun-load-review-2026-11-19.json) | Firmware/ranges, separate hold attempts and corrected gun-attached context |
| [Nov 17 preparation](measurements/arm-preparation-review-2026-11-17.json) | Earlier tracking and shutdown evidence |
| [Nov 17 six-motor control readback](measurements/arm-control-readback-review-2026-11-17.json) | Documented disabled register reads complete, including A1; independent protection qualification remains open |
| [Nov 16 A5 alignment](measurements/arm-a5-alignment-2026-11-16.json) | Cable ends and straight candidate; broader alignment remains open |
| [Nov 13 returns](measurements/arm-return-checks-2026-11-13.json) | Stationary cycle and off-power observations; not new anchors |
| [Nov 12 shutdown](measurements/arm-reference-2026-11-12.session-summary.json) | Earlier failed shutdown; later successful runs do not qualify all loads |

## Entry points and ownership

The checkout is `/home/forge2/forge2_ws`. Shared planning belongs in
`src/controls/controls/controller/`; hardware and simulation are siblings.
[Source ownership](SOURCE_LAYOUT.md), [operator commands](ARM_RIG_TEST.md),
[ENCOS procedure](CAN2_BENCH.md) and [verification commands](../tests/README.md)
are the maintained references. Generated reports/builds remain ignored;
selected dated archives are explicitly versioned. Normal host updates need no flash.

## Publication validation — November 24

See the [publication record](measurements/publication-review-2026-11-24.json)
for final offline checks and the archived source hashes. Physical v7 and ENCOS
results above are operator evidence, separate from fake-motor/model tests.
