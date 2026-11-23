# Arm session handoff

Updated 2026-11-23 for `NSSRS/FORGE`, branch `nathan`.
Open work belongs in [ARM_LAYER_TODOS.md](ARM_LAYER_TODOS.md) and package TODOs;
operator commands belong in [ARM_RIG_TEST.md](ARM_RIG_TEST.md).

## Current status and next session

**HOLD-01 is resolved** by operator confirmation on Nov 23; see the
[closure record](#hold-01-resolution).
**PLAN-01 and BASE-04 are resolved** by operator confirmation on Nov 23; see
[their closure](#plan-01-and-base-04-resolution).
**Current development:** ACCURACY-01 and SPEED-01. The operator accepted
**8.22 s preparation** and retired the previous five-second goal. Current
comparisons prioritize **wire-tip position and stroke** with one fixed lag
allowed for arm-only position scoring. Retain **0.5 deg target / 1 deg maximum**
for same-time tool orientation as a secondary criterion. See the
[matched A1 trial](ARM_RIG_TEST.md#weld-motion-accuracy-test) and the
[complete offline archive](measurements/weld/2026-11-23/README.md).

The fixed-lag report on the hardware journal finds 101.8 ms lag, 4.58 mm
RMS / 11.09 mm peak residual tip-position error and a 34.56 mm reconstructed
stroke for the 20 mm plan. The A1 shaper-only command replay reduced dispatched
A1 travel from 7.69 to 3.83 deg, with zero A1 rate or acceleration limit hits.
Use the matched operator dry passes to measure the physical tip response. The
64-candidate offline search reached 1.739 Hz and failed strict 2 Hz timing.

The Nov 23 19:05 launch (`build/weld/sessions/20261123T110514.433378Z-tu8yc8s8/`)
failed the startup stationary-disable check: A1-A4 had valid feedback, but
A5/A6 retained `stopping=1` and feedback age `UINT32_MAX`. Two explicit recovery
attempts also failed at A5's disable acknowledgement; no plan or motion target
was recorded. The historical missing-acknowledgement cause was not established;
the operator has since closed BASE-04.
The UI now labels this sentinel as unavailable and suppresses the current-pose
render instead of showing billions of milliseconds or validating retained angles.
The subsequent 19:09 launch (`20261123T110914.651777Z-0avafxc0`) received valid
six-axis feedback and entered recovery/approach, then latched A1/CAN1 reason 5
(transmission failure) with feedback only 1 ms old. This was not a stale-feedback
trip. The historical physical cause was not established; this incident is no
longer an open BASE-04 blocker. Idle/fault cards now
say Cached, selection alone no longer triggers a live-feedback warning, and
new fault records retain cached MCU bus diagnostics after stop. Reason 5's label
now covers both enqueue and completion failure instead of asserting queue failure.
No firmware timing/guards changed and the agent did not operate hardware.

Hardware now accepts fresh measured starting coordinates outside saved pose
windows or simulator joint ranges. A session-local recovery allowance includes
only that observed extra extent; A4/A5/A6 cable bounds and motor protections remain.
Starting self/environment contact remains visible as a warning; subsequent path
samples must still pass collision checks. Normal bounds resume before approach
and welding. Simulation remains strict. This is offline implementation evidence,
not physical qualification; use the existing launch command and inspect the
[current-pose/recovery procedure](ARM_RIG_TEST.md#weld-panel-with-embedded-model-preview).

Offline verification: all eleven measured-start tests passed, including the
recorded Nov 23 start through Ready on fake shafts. The publication check passed
257/258 weld tests; 191 motor/cable/calibration/keyboard/GUI tests and all ten
accuracy-analyzer tests also passed.
The sole weld failure,
`test_tiny_joint_corrections_cannot_throttle_a6_clearance_projection`, also fails
with the pre-change `2dd6caa` planner/controller/port sources: its masked target
has 1.538 mm floor clearance against the required 1.8 mm. That existing tracking
fixture discrepancy remains unresolved; clearance thresholds are unchanged.
No hardware was accessed.

The Nov 23 `hold-01-20261123-181305` session took 292.48 s to plan, including
218.70 s for the original recovery search. It paused on A2 tracking error
+3.003 degrees before reaching A6's movement leg; final disable and mode
restoration are reported verified. A6 began at raw -169.927 degrees and the
plan retained the +63.621-degree weld target. The operator confirmed A6's fixed
+/-270-degree travel limit. It is now enforced independently of model start
allowances, using the preserved +64.161-degree reference. A -296-degree target
would violate that cable limit, so no shorter wrapped turn is substituted.
Depth-first search reduced an offline replay to 77.39 s (same route,
9 versus 56 checked edges for the selected recovery). This is calculation
latency, not physical motion time; physical A2 tracking remains unresolved.

The Nov 20 paired 14-start comparison reduced median planning from **73.10 to
24.80 s**. The complete random corpus produced 47/48 plans, with one 180 s
planning timeout; none met the five-second motion goal. A separate 16-start
work-area sample produced 16/16 plans and reached Ready in **4.58–4.94 s** on
ideal shafts. All 63 accepted paths passed dense unfiltered-model checks and
final-controller replay. This identifies a candidate starting region, not
physical five-second qualification. Process workers improved batch throughput;
threads were slower. The GUI keeps one persistent isolated planner process.
Bounded caches, geometric sampling and optimistic staging pruning reduce its
work. Recovery stops resolve to exact validated knots to prevent endpoint
roundoff faults. This benchmark predates the operator's Nov 23 PLAN-01 closure.
Release regressions passed **223 weld tests plus 30 shared arm/GUI/model tests**;
no motor hardware was operated.

The weld panel has six cached torque/temperature cards, trends and stage/run peaks,
separate targets/response, clear state/next action and expandable diagnostics.
The manual panel shares the telemetry view. Planning and physical preparation
clocks are separate; operator/focus/heartbeat pauses have distinct journal
causes. No protection settings or motion behavior were intentionally changed.
Offline verification passed 166 controller/Qt/cached-data/shared-panel tests;
state renders were checked at 1366×768. No hardware was operated.
See the [interface guide](WELD_MOTION.md#telemetry-dashboard).

## ACCURACY-01 and SPEED-01 review

The latest operator-supplied session is
`build/weld/sessions/20261123T114316.007261Z-cnyzuo1w/session.jsonl`, with the
operator-confirmed 1.96 kg EE. The offline
[accuracy/speed review](measurements/arm-accuracy-speed-review-2026-11-23.json)
retains source hashes, all six axes and the existing analyzer's report.

The arm-only weld interval contains **202 usable command ticks over 5.95 s**,
with a maximum gap of **63.7 ms**, no rejected samples and no dropped records.
The checked rate is **1.730 Hz** (retimed from the requested 2 Hz). Cached
encoder autocorrelation estimates **A1 1.734 Hz / A6 1.728 Hz**.

| Axis | Nominal error RMS / peak | Planned / measured peak-to-peak travel |
| --- | --- | --- |
| A1 | 1.138 / 2.098 deg | 2.261 / 3.891 deg |
| A6 | 0.379 / 0.750 deg | 0.428 / 1.333 deg |

A2-A5 nominal peak errors are **0.043 / 0.107 / 0.193 / 0.078 deg**. Their small
excursions do not establish a reliable oscillation frequency. Fixed-base encoder
reconstruction gives tip error **10.059 mm RMS / 18.575 mm peak**, predominantly
transverse. This is model-dependent; independent tip measurements are absent.

The operator confirmed that acceptance is based on end-effector orientation,
not separate 0.5-degree joint limits. Encoder reconstruction of the full tool
frame gives **1.408 deg RMS / 2.633 deg peak**; **21.8%** of usable ticks are
within the 0.5-degree target and **40.1%** within the 1-degree maximum. This
baseline exceeds the maximum. The estimate includes all three orientation
components and compares against the same-time planned frame without phase
alignment. Independent tool-pose measurements and a separate tip-mm tolerance
remain missing. See the [orientation report](measurements/weld/2026-11-23/accuracy-end-effector.json).

ACCURACY-01 has both joint diagnostics and a model-based end-effector baseline.
Measure synchronized tool pose in the workpiece frame and apply the confirmed
orientation criteria to corrected motion. SPEED-01
should first investigate A1/A6 host lead and command shaping: dispatched travel
was **7.693 / 2.203 deg**, while actual travel was 1.72 / 3.12 times nominal.
Approximate fundamental fits after the first cycle show **101 / 112 ms** phase
lag. These observations identify a tuning target, not a proven motor/drive defect.
Compare a bounded change against this baseline at the same load, start and plan;
retain nominal geometry and existing protection thresholds.

Recovery request to Ready was **8.22 s**, meeting the selected 10 s trial goal
and accepted by the operator; the earlier five-second goal is retired. During approach,
A2/A5/A6 maximum nominal errors were **1.838 / 1.348 / 1.552 deg**. The larger
8.834 / 6.131 / 2.971 deg values in the hold analyzer compare feedback with
lead-adjusted dispatched targets; they are not the nominal tracking-guard error.

The run ended via **operator_input Pause / Hold**, with no fault and verified
disable/mode restoration. The accuracy analyzer currently labels every pause
as an evidence issue and exits 2; here that is a reporting limitation, not a
failed motion or a missing journal. Keep continuous welding and manual pause.
The original unthresholded report has `passed: null`. The new orientation report
uses the confirmed 0.5-degree target / 1-degree maximum and fails the maximum;
this result is separate from the analyzer's manual-pause reporting limitation.

## PLAN-01 and BASE-04 resolution

**Resolved on 2026-11-23 by operator confirmation:** "Plan 1 is resolved.
Base 4 resolved". Both IDs are removed from active registries and package TODOs.
Historical benchmarks and fault evidence are retained; closure does not invent
an implementation change or a newly measured physical cause. Do not reopen
these tasks from historical planning/CAN records. Current work is ACCURACY-01
and SPEED-01 as described above.

## HOLD-01 resolution

**Resolved on 2026-11-23 by operator confirmation:** "not an issue anymore,
mark it as resolved". This closes HOLD-01 and the historical untouched A5
fixed-hold fault. It is removed from the open-work registry and package TODOs;
do not carry that incident forward as an unresolved blocker.

Supporting physical evidence is the operator's session
`build/weld/sessions/20261123T114316.007261Z-cnyzuo1w/session.jsonl`, analyzed in
the adjacent `hold-summary.json`. The operator confirmed **EE mass: 1.96 kg**;
`session.load-context.json` records that confirmation separately from the raw journal.

- Recovery request to Ready: **8.22 s**.
- Fresh six-axis Ready evidence: **25.19 s**, with unchanged targets and speed
  allowances. Maximum error across all axes: **0.175 deg**, within 0.25 deg.
- A5 maximum hold error: **0.053 deg**; mean reported torque **-1.16 Nm**,
  peak absolute torque **1.24 Nm**; motor temperature **28 -> 29 C**.
- No recorded faults or dropped journal records. Final disable acknowledgement
  and original-mode restoration both passed.

The fixed-reference target and speed-allowance software defects were corrected
earlier. The historical physical fault's root cause was not established; its
original evidence remains unchanged. Closure records the operator's acceptance
and this run's observations, without changing production flags or thermal ratings.
ACCURACY-01/SPEED-01 own the remaining motion-tracking work; LOAD-01/PROTECT-01
retain the separate payload and thermal/protection work.

### Historical November 19 evidence

The earlier completed physical session is
`session-load-30-10-20261119-190025.json`, reviewed in
[the Nov 19 gun-load record](measurements/arm-gun-load-review-2026-11-19.json).
The operator reports that it ran well and separately confirms the first
attempt was **untouched**. Record the attempts separately:

- **Attempt 1:** Ready in 11.86 s, then A5 moved during fixed-target holding
  and tripped at **1.51648 rad/s** against 1.5 rad/s. Its target stayed fixed;
  the fault snapshot shows approximately **4.96 deg** displacement. A5 averaged
  **6.22 Nm** during Ready, peaked at **6.65 Nm**, and reported motor temperature
  **26 -> 40 C during preparation, then 46 C during Ready**. This was a speed
  fault, not a torque/temperature trip. The cause remains unknown; operator
  contact is explicitly excluded. Reset completed through the existing owner.
- **Attempt 2:** Ready in **12.53 s** versus 9.85 s nominal / 10 s goal; fixed
  Ready hold **8.69 s**, all six within **0.177 deg**. Continuous welding ran
  **45.71 s** before an operator-intent pause, with no further motor fault.
  A2 peaked at **13.57 Nm preparation / 9.25 Nm welding**, motor **35 C**.
  A3/A4/A5/A6 welding peaks were **6.63 / 0.51 / 0.81 / 0.57 Nm**.
  Ready targets were identical across attempts, but torque signatures differed
  substantially; the mechanical/load cause has not been established.
- **Tracking and close:** A1/A6 final-window rates were **1.729/1.730 Hz** versus
  planned 1.731 Hz, but travel was **3.869/1.180 deg** versus **2.260/0.427 deg**.
  Amplitude remains open. Final disable and original-mode restoration both
  passed; CAN health reports no TX failures or bus-off. The JSONL has no dropped
  records; the bounded embedded motor-sample list dropped 6000 earlier rows,
  so analysis uses the full JSONL. That run did not reach the requested
  15-second Ready hold or 5–10 second physical preparation goal. The Nov 23
  session above subsequently recorded 25.19 s of holding and 8.22 s to Ready.

## Deployed profile and completed work — November 19

The operator deployed firmware **f78f6b0d**, capability mask **0x1FFFF8F**.
The current explicit profile is
`src/robot_description/config/arm_rig_gun_a2_a6_load.json`:

| Joint | Reactive torque trip | Feedback TMAX |
| --- | ---: | ---: |
| A1 | 20 Nm, unchanged | 54 Nm |
| A2/A3 | 30 Nm | 40 Nm |
| A4/A5/A6 | 10 Nm | 12.5 Nm |

These are operator-requested trial thresholds, not continuous-load ratings.
Rated references remain 12 Nm for A2/A3 and 3.5 Nm for A4-A6. No overload timer
was added. Both 80 C trips, all 1.5 rad/s speed trips, the 3-degree tracking trip,
0.25-degree arrival tolerance, freshness/lease guards and latched whole-arm
fault disable remain. Standard profiles and drive current/gain settings are
unchanged; see [torque units and limits](DAMIAO_TORQUE_LIMITS.md).

`load-range-setup-20261119-185854.json` records A3 TMAX 28 -> 40 and A4-A6
10 -> 12.5, independent readbacks, one acknowledged save each and disabled
cleanup. A2 already read 40. The fresh-MCU check at 18:59:39 finds all ranges
retained with zero setup locks/masks; `load-readback-20261119-190024.json` passes
with all six disabled. Setup need not be repeated for each run. The older
A2-only profile is retained for provenance and intentionally fails range matching
after A3-A6 commissioning. The agent did not operate hardware.

Today's implementation:

- **A3 startup allowance:** up to 0.25 deg above raw 151.39 deg, followed by a
  separate inward retreat to 149.39 deg at <=1 deg/s and <=5 deg/s². It checks
  shallow, separating contact at the reviewed stop pair; other limits stay
  strict. Raw readings remain visible without wrapping/clamping. The recorded
  151.5014 deg start has about 0.076 mm modeled overlap; the backoff has about
  0.378 mm clearance. Physical loaded retreat remains unqualified.
- **Preparation timing:** independently time moving legs separated by existing
  stops; retain the exact path and acceleration reserve. Avoid repeating mesh
  checks after timing-only transformations. Calculation still takes about 40 s
  in the offline comparison and is separate from physical movement time.
- **Final lowering:** `--lowering-seconds 2.3` stretches only the final lowering
  leg after staging selection. The recorded A2 3.048-degree lag pause motivated
  this change; subsequent loaded runs reached Ready. Gains and tracking trips
  remain unchanged. The Nov 19 12.53 s arrival exceeded the 5–10 s trial goal;
  the Nov 23 session reached Ready in 8.22 s, which the operator accepted.
- **Load commissioning:** fixed, disabled-only TMAX write/readback/save helpers
  and matching host/MC02 capabilities; setup locks motion until MCU restart and
  never retries uncertain writes/saves. The operator performed deployment.
- **Evidence:** cached JSONL records torque, motor/MOS temperatures, motion and
  fixed holds without another owner or enabled register polling. The offline
  `scripts/analyze_weld_hold.py` separates attempts, stages and endpoint-retreat
  timing, excludes stale/faulted samples and retains shutdown uncertainty.

The current launch uses PID tracking, explicit 30 deg/s preparation, a 10 s goal,
optimized staging, a 30 mm raised approach and 2.3 s final lowering. Default
preparation remains 5 deg/s. Use the [current commands](ARM_RIG_TEST.md#expanded-gun-load-profile-a2a3-30-nm-a4-a6-10-nm).
Report paths are automatic: omit `--output` and use `build/weld/latest/` for the
latest launch's live `session.jsonl` and final `session.json` after close.
Every launch is retained under `build/weld/sessions/`, including incomplete journals.

## Today's earlier observations

These local source journals are under ignored
`build/arm_tests/hold-01-20261119/`; preserve them as dated evidence, not current
operating instructions. The final session and commissioning sources are indexed
with hashes in the [gun-load review](measurements/arm-gun-load-review-2026-11-19.json).

| Local source | Finding |
| --- | --- |
| `no-ee/session-branch0-20261119-161036.jsonl` | 42.73 s to Ready at 5 deg/s; A6 travels 91.61 deg. The speed cap rules out 5–10 s for that path. Offline 30 deg/s replay gives 8.71 s nominal / 8.88 s ideal / 11.12 s synthetic lag; these are not hardware timings. |
| `no-ee/session-branch0-20261119-164725.jsonl` | A2 12.0137 Nm trip. The operator confirmed the gun was already attached despite the directory name; a load-context sidecar preserves that correction. |
| `with-ee/session-a2-30nm-20261119-175404.json` | A2 nominal-reference tracking pause at 3.048 deg during lowering. The operator cut power afterward; the later communication fault is separate. Cleanup was not verified in this run. |
| `with-ee/session-a2-30nm-lower23-20261119-182504.jsonl` | Ready 12.87 s, hold about 8.13 s, weld 2.33 s, then operator-intent pause; no motor fault. A2 weld peak 8.53 Nm / 28 C. The log does not identify focus/input/heartbeat as the individual pause trigger. |
| `with-ee/session-a2-30nm-lower23-20261119-183911.jsonl` | Two A5 torque trips at 3.5092 / 3.5140 Nm against 3.5 Nm; neither reached Ready. This prompted the expanded trial profile. |

A matched no-EE versus approximately 1.96 kg EE comparison remains unfinished.
The loaded runs are not a qualified constant-load or thermal endurance test.

## Current execution contract

- Flow: measured pose -> stationary hold verification -> optional A3 retreat ->
  checked recovery/staging -> raised approach/lowering -> Ready -> explicit
  continuous welding -> manual Pause / Hold. The operator requires no automatic
  stop after one or two swings. `--cycles` sizes only the checked preview block,
  including at 1. Normal launch commands must omit `--single-pass`; use it only
  for an explicitly requested finite run. The panel must show **continuous until
  Pause / Hold**. Existing fault/focus/heartbeat stops remain. No mode energizes
  the welder.
- One owner per transport. Native mode 2 / USB type 4 uses raw output-shaft SI
  units; the planner/panel use explicitly named degree vectors. One feedback
  batch and one target batch per moving tick; idle monitoring refreshes fixed
  holds. Telemetry cannot renew motion authorization.
- Bounded PID/velocity lead follows nominal preparation time, brakes within each
  finite path leg and sheds correction at its fixed endpoint. Welding omits host
  correction below 0.3 deg planned travel while retaining nominal targets.
  `--tracker paced` remains available. No gravity feedforward, Cartesian outer
  correction, ESO or drive-gain tuning is active.
- Settled endpoint pauses preserve the fixed target and checked speed allowance.
  Interrupted travel captures one measured hold and requires replanning. Ready
  focus/heartbeat pauses retain the checked plan and explicit Start welding
  action. Disable/fault/close remove torque; recovery is explicit through the
  existing owner and leaves motors disabled with the old plan discarded.
- Preparation acceleration remains <=30 deg/s²; command speed is capped by the
  existing 68.7549 deg/s motor guards. Arrival requires <=0.25 deg error and
  stationary feedback for 0.2 s. A requested duration is a goal, not a timeout.
- Requested welding remains 2 Hz / 12 mm/s / +/-10 mm. Model clearance, cable
  winding, calibration and motor limits constrain the planned rate. Kinematic
  previews and fake shafts do not establish physical load/stopping behavior.

## Calibration and evidence

| Evidence | Interpretation |
| --- | --- |
| [Nov 23 full offline archive](measurements/weld/2026-11-23/README.md) | Original JSON/JSONL, mapping/standby/profile/plan, checksums, end-effector orientation criteria and offline reproduction commands |
| [Nov 23 accuracy/speed review](measurements/arm-accuracy-speed-review-2026-11-23.json) | Tool orientation exceeds the confirmed 1-degree maximum; A1/A6 amplitude/phase need work; independent tool-pose data and tip-mm tolerance remain missing |
| [Nov 23 HOLD-01 resolution](#hold-01-resolution) | Operator closure; 1.96 kg EE, 25.19 s fixed Ready hold, A5 maximum error 0.053 deg, no faults and verified final shutdown |
| [Nov 19 gun-load review](measurements/arm-gun-load-review-2026-11-19.json) | Operator-deployed expanded ranges; untouched A5 hold fault, successful second attempt, torque/temperature/tracking statistics and verified final shutdown |
| [Nov 12 reference capture](measurements/arm-reference-2026-11-12.capture.json) | Eight recorded anchors; deployed firmware `d61f6d16`, capabilities `0xFFF8F` |
| [Nov 12 shutdown summary](measurements/arm-reference-2026-11-12.session-summary.json) | Capture succeeded, cleanup did not: A2 reported -0.168498 rad/s; stationary-disable/mode restoration unconfirmed |
| [Nov 13 return observations](measurements/arm-return-checks-2026-11-13.json) | Stationary cycle and off-power returns; not new calibration anchors |
| [Nov 16 A5 observations](measurements/arm-a5-alignment-2026-11-16.json) | Complete cable-end/straight-reference candidate; independent alignment away from straight remains open |
| [Nov 16 recovery review](measurements/arm-recovery-review-2026-11-16.json) | Old timeout stopped a paced home path at 54.7%; corrected offline. A5 physical drop cause remains unknown |
| [Nov 17 preparation review](measurements/arm-preparation-review-2026-11-17.json) | Tracking/shutdown failures, passed disabled readbacks and offline controller sweep |
| [Nov 17 PID review](measurements/arm-pid-tracking-review-2026-11-17.json) | Synthetic lag/geometry replay; timing, fixed hold and command limits |
| [Nov 17 small-motion review](measurements/arm-weld-small-motion-review-2026-11-17.json) | Earlier A6 two-cycle response, correction-mask diagnosis and recorded-feedback regression |
| [Five saved Nov 17 reports](measurements/weld/2026-11-17/README.md) | Original physical reports, including tracker version 3 frequency improvement, amplitude overshoot and unverified final shutdown |

All six motors are connected by operator report. A1 is CAN1/1/17; A2-A6 are
CAN2, command IDs 2-6, feedback IDs 18-22. A2/A3 manual motion is reported smooth;
A1/A4/A5/A6 travel jitter remains open. The operator confirmed the earlier A1
undervoltage problem was resolved.

The candidate mapping uses unit-magnitude gain because feedback is already
output-shaft angle. Corrected A3 endpoints are CW 72.77 deg / CCW 151.39 deg;
derived standby is raw 149.39 deg, a 2-degree inward backoff. The original CAD
initial A3 target overlaps its stop and is not an automatic hardware target.

A4 cable travel is fixed at +/-270 deg from raw +173.183435236 deg, giving
[-96.816564764, +443.183435236] deg. Startup must not rebase or wrap this interval.
A6 uses +/-270 deg from raw +64.161048535 deg, giving
[-205.838951465, +334.161048535] deg. Startup, measured recovery and manual jogging
retain this fixed envelope; its physical qualification remains open.
The complete A5 candidate uses `model_deg = -raw_deg + 101.514057878`, with raw
command bounds [-96.607604972, +114.836287037] deg. It is separate from the default
mapping. Use the matching reviewed mapping and standby together; their fingerprints
must match. Production qualification remains false.

The weld launcher supports explicit `--a1-encoder-turns -1|0|1` for an
independently verified A1 restart branch. This translates session calibration,
raw bounds and targets together while leaving the saved mapping/standby and A4
cable reference unchanged. It is recorded in the journal/report; no runtime
wrapping or automatic branch selection is used. Physical encoder-branch
qualification remains under CAL-01. See the [panel procedure](ARM_RIG_TEST.md#weld-panel-with-embedded-model-preview).

The D3 linkage uses 150 mm connection-center spacing. Its branch-continuation fix
and the reviewed `collision_139` / `collision_463` triangle refinement preserve
clearance thresholds. Original triangle meshes reject actual intersections;
convex overlap alone is not used to reject that reviewed concave pair. The gun
remains modeled even when physically absent.

## Entry points and ownership

| Purpose | Entry point |
| --- | --- |
| Manual control | `bash scripts/control_arm.sh` |
| Reference recording | `scripts/record_arm_positions.sh` |
| Independent raw/model pose inspection | `scripts/mirror_arm.py` |
| Model-guided calibration | `scripts/calibrate_arm.py` |
| Plan / fake execution / operator hardware panel | `scripts/control_weld.py --plan-only`, `--simulate`, or explicit `--execute` |
| Offline hold/torque/temperature analysis | `scripts/analyze_weld_hold.py` |
| Disabled load-range commissioning | `scripts/setup_arm_load_ranges.py` |
| Offline branch search / exported animation | `scripts/optimize_weld.py`, `scripts/render_weld.py` |

The NUC checkout is `/home/forge2/forge2_ws`. Generated logs, plans, firmware and
captures remain ignored under `build/` and `logs/`; dated derived reviews and
explicitly archived reports are the documented exceptions. A fresh checkout does
not include the local candidate mapping/standby or unarchived source journals.
Shared planning belongs in `controller/`; hardware and simulation are siblings.
Normal host changes need no flash. The operator performs hardware work; do not
open a second owner or interrupt a powered owner while its load depends on torque.

Next hardware work is ACCURACY-01/SPEED-01 with the applicable CAL-01/SUPPORT-01
checks. Use the [weld motion accuracy procedure](ARM_RIG_TEST.md#weld-motion-accuracy-test)
and end welding with manual Pause / Hold. LOAD-01 and FEEDBACK-01 follow the first qualified
arm welding run. Reproduction checks are in [tests/README.md](../tests/README.md).

## Publication validation — November 19

Validated 200 weld tests, 183 motor/commissioning tests and 49 shared-model/
calibration tests, plus three native firmware checks and the offline weld audit.
The first full weld run had two model-fingerprint rejections because concurrent
terrain tests regenerated scene files; after restoring those generated files,
both affected tests passed sequentially. The 64 endpoint/hardware tests also
passed after consolidating duplicate A3 startup validation. Run terrain/model
and weld suites sequentially as documented in [tests/README.md](../tests/README.md).

The ARM firmware rebuild matches deployed identity **f78f6b0d**; no hardware was
flashed or operated by the agent. All nine measurement-source hashes, local
Markdown links/anchors and whitespace checks passed. Generated raw logs/builds
stay local and ignored. These checks verify software/model behavior, not the
remaining physical acceptance items.
