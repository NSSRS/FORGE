# Arm and chassis session handoff

Updated 2026-11-24 on `nathan`. This file describes the current baseline;
[dated measurements](#calibration-and-evidence) retain earlier findings.
Open work belongs in [ARM_LAYER_TODOS.md](ARM_LAYER_TODOS.md) and
[CHASSIS_MODEL_COMMS_TODOS.md](CHASSIS_MODEL_COMMS_TODOS.md).

## Current status and next session

The operator selected **v7 for welding** and deferred further weld tuning.
ENCOS motor 4 is identified as ID 4 on CAN2; all four chassis motors passed
signed motion and stop checks. Routine use needs no ID assignment, firmware
flash or repeated commissioning. The agent performed offline work only.

| Subsystem | Current entry point | Status |
| --- | --- | --- |
| Arm welding | [V7 operator command](ARM_RIG_TEST.md#weld-motion-accuracy-test) | Selected baseline; startup and independent tip accuracy remain unqualified |
| Four ENCOS motors | `bash scripts/run_motor_test.sh --four-motors` | IDs 1/2 CAN1, 3/4 CAN2; slots 0/1/3/4; 5 RPM, 2 A trial limits |
| Combined sensors/Pocket | [Full-system guide](FULL_SYSTEM_TEST.md) | Default remains three motors; explicitly select `chassis_four_motor_bench.yaml` for four |

The four-motor report `20261124T110254.496409Z.json` records both +5/-5 RPM
one-second legs and both below-0.5-RPM stop checks for all IDs. Across 115
cached samples: zero motor faults, WKC 3/3, 532 exchanges without failure and
normal owner close. The operator subsequently reported normal visual behavior
in the four-motor keyboard/viewer session. The Terminator window-deregistration
message came from the terminal UI; its exact trigger was not captured.
See the [physical record](measurements/encos-four-motor-physical-review-2026-11-24.json).
Explicit wheel locations, forward signs and external gearing remain to be
recorded; loaded stopping, CAN freshness and host-loss behavior are separate.

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

MC02 firmware **f78f6b0d**, capabilities **0x1FFFF8F**, is deployed with
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

The [Nov 23 baseline](measurements/arm-accuracy-speed-review-2026-11-23.json)
preceded the command-sampling correction and v7. Its 2.633-degree orientation
peak and A1/A6 overswing are historical comparisons, not current v7 performance.
The [sampling review](measurements/arm-weld-command-sampling-review-2026-11-24.json)
and [v7 physical comparison](measurements/arm-weld-response-physical-review-2026-11-24.json)
record the progression. Future work prioritizes wire-tip position and stroke,
with one fixed lag allowed for arm-only position reporting. Preserve same-time
orientation targets of 0.5 deg / 1 deg maximum and report startup separately.
Independent workpiece-relative tip measurements and a tip-mm tolerance are missing.

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

| Evidence | Scope |
| --- | --- |
| [Nov 24 V7 and ENCOS archive](measurements/weld/2026-11-24/README.md) | Original source bytes, checksums, baseline parameters and four-motor result |
| [A1 shortest recovery](measurements/arm-a1-shortest-recovery-review-2026-11-24.json) | Raw -181.511 -> -174.692 deg, CCW 6.819 deg; synchronized branch |
| [Spatial policy](measurements/arm-spatial-policy-review-2026-11-24.json) | Advisory hardware model space; strict mechanical/cable checks |
| [Hold activation](measurements/arm-hold-activation-review-2026-11-24.json) | Fixed preload reference and 0.5-degree start tolerance |
| [Nov 23 archive](measurements/weld/2026-11-23/README.md) | Earlier baseline, original journals and load context |
| [Nov 19 gun load](measurements/arm-gun-load-review-2026-11-19.json) | Firmware/ranges, separate hold attempts and corrected gun-attached context |
| [Nov 17 preparation](measurements/arm-preparation-review-2026-11-17.json) | Earlier tracking and shutdown evidence |
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
