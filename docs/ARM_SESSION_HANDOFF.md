# Arm session handoff

Updated 2026-11-17 for `NSSRS/FORGE`, branch `nathan`.
Open acceptance work is owned by [ARM_LAYER_TODOS.md](ARM_LAYER_TODOS.md).
Operator procedures belong in [ARM_RIG_TEST.md](ARM_RIG_TEST.md); controller
architecture and planning are described in [WELD_MOTION.md](WELD_MOTION.md).

## Current status and next session

The latest saved physical report is
[`weld-1794913974470083980.json`](measurements/weld/2026-11-17/weld-1794913974470083980.json),
using tracker version 3. Recovery reaches ready in **10.54 s**, then welding runs
for **15.58 s** before an operator-intent pause. The final measured window gives
A1 **1.730 Hz / 3.694 deg** peak-to-peak and A6 **1.731 Hz / 0.634 deg**; the
planned weave is **1.731 Hz**, with about **2.260 / 0.427 deg** planned travel on
A1/A6. A6's earlier two-cycle response is absent in this window, but amplitude
overshoot remains. A2–A5 have tiny nominal moves that the operator accepts; they
remain commanded and show no numeric frequency when motion is insufficient.
All 739 recorded welding samples admit the full host correction through the
existing clearance check. This is one physical run, not loaded qualification.

The same report later ends with missing feedback on all six joints during close:
`disable_acknowledged=false` and `mode_restoration_verified=false`. A disable
event appears earlier, but the final report does not verify shutdown. The cause
of missing feedback is not established. Keep that result separate from successful
recovery and frequency tracking. The [five archived reports](measurements/weld/2026-11-17/README.md)
are byte-for-byte copies with checksums; detailed source JSONL journals remain local.

Next acceptance work:

1. **SPEED-01 / WEAVE-01:** confirm repeatable A1/A6 frequency and amplitude under
   the supported load, including reversals, command timing and clearance. Tracker
   gains and lag estimates remain provisional. Five-second physical preparation
   is still unfinished; the latest recovery takes 10.54 s.
2. **HOLD-01:** qualify settled and post-weld fixed holding, stiffness and thermal
   behavior. Earlier evidence records 12.76 s within 0.177 deg and recovery from
   operator pushes, but a later paused journal also shows about 0.46 deg A1
   oscillation with fixed targets. Follow the [hold procedure](ARM_RIG_TEST.md#post-preparation-hold-acceptance).
3. **SUPPORT-01:** establish supported torque loss and verified stationary disable
   and mode restoration. Do not infer these from a completed motion.
4. **PLAN-01:** the requested encoder-drift check between Calculate and Start
   recovery remains a TODO; the proposed 2 or 5 degree threshold is undecided.

## Tracking changes and evidence boundaries

The default `--tracker pid` follows nominal preparation time using bounded
velocity lead, filtered derivative feedback, anti-windup and command speed/
acceleration shaping. `--tracker paced` retains the older shared-clock policy.
No drive gain/current/mode writes, weaker trips or wider arrival tolerances were
introduced. Moving ticks read one feedback batch and dispatch one target batch.
Finite preparation legs bound virtual targets to exact path extrema and brake
before those extrema; endpoints shed correction and latch the original fixed
hold. This fixes the earlier A6 approach command that passed its endpoint.

Welding repeats until Pause / Hold by default. Small-axis nominal trajectories
are retained, but host lead/PID is omitted below 0.3 deg planned peak-to-peak
travel. Previously those corrections consumed shared clearance margin and
limited A1/A6: a replay of 1,019 recorded samples changes median admitted
correction from 0.059 to 1.0 without a shaped-command clearance failure. The newer
physical run above also records full correction throughout welding. Larger
measured travel still needs tracking acceptance; matching Hz alone is insufficient.

The panel displays cached encoder-derived frequency and measured/planned travel,
separately from requested/planned weave Hz. It labels a repeating half-rate
pattern as a two-cycle response, clears stale/paused values, and withholds Hz for
small or irregular motion. These observations add no motor/register polling.

Historical model replay improves preparation from 15.32 s with legacy pacing to
10.08 s with PID on the same synthetic lagging shafts; it covers 20 s of welding
and 10 s of fixed pause hold. Those results and the earlier ideal 4.62 s staging
case are software/model evidence, not five-second hardware acceptance. Detailed
measurements and limitations are linked below; operator commands are in
[ARM_RIG_TEST.md](ARM_RIG_TEST.md).

## Current implementation

- Flow: measured pose -> verify hold -> checked recovery/standby -> approach ->
  ready -> explicit continuous welding -> Pause / Hold. Single-pass execution
  finishes with a fixed endpoint hold. Output switches gate
  arm/chassis commands independently; no inactive-subsystem compensation exists.
- One owner per transport. Damiao commands use native position/velocity mode 2,
  USB type 4, raw output-shaft SI units at the bus boundary. The planner and panel
  use explicitly named degree-valued vectors. No type-25 jog smoothing is used.
  Moving ticks read all six shafts once and dispatch all six new targets once;
  idle ticks continue fixed-target refresh and monitoring.
- Preparation defaults to 5 deg/s and <=30 deg/s². PID tracking uses nominal path
  time; the explicit legacy paced controller slows for lagging shafts.
  Per-axis progress and final-settling watchdogs are separate;
  the five-second goal is not a timeout or permission to truncate a move.
  Explicit `--max-speed-deg-s 100` is now accepted at the operator's request;
  actual command allowances remain capped by the existing 68.7549 deg/s motor
  guard. The 1.5 rad/s measured-speed trip remains unchanged.
- Arrival requires <=0.25 deg error and stationary feedback for 0.2 s. The 3 deg
  motion tracking trip and all six 1.5 rad/s measured-speed trips are unchanged.
- Requested welding is 2 Hz / 12 mm/s / +/-10 mm. Weld command ceilings are
  68.7549 deg/s on all six axes. A historical, complete-route checked CMA-ES
  candidate reached 1.815288 Hz under provisional CAD dynamics; regenerate any
  saved solution after mapping, model or constraint changes.
- `--optimize-preparation` explicitly enables checked staging comparison,
  including hardware execution. The panel labels this route; planning still
  leaves motors disabled until the operator starts recovery. It retains fresh
  feedback, clearance, cable, speed, acceleration and tracking checks.
  For the recorded Nov 16 start and A5 candidate, it reduces nominal preparation
  from 20.9272 s to 4.5526 s; ideal fake-shaft arrival is 4.62 s including settling.
  That comparison uses explicit `--max-speed-deg-s 10` and changes the
  intermediate pose. Other starts can exceed five seconds. A synthetic lagging
  drive takes 14.44 s, so physical five-second execution is not established.
  Historical plans marked `offline_only` remain rejected by physical owners.
- Preview is kinematic; fake shafts do not model gravity, backlash or drive
  stiffness. No load feedforward, Cartesian outer correction or ESO is installed.

## Calibration and evidence

| Evidence | Interpretation |
| --- | --- |
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
| Plan / fake execution / physical panel | `scripts/control_weld.py --plan-only`, `--simulate`, or explicit `--execute` |
| Offline branch search / exported animation | `scripts/optimize_weld.py`, `scripts/render_weld.py` |

The current NUC checkout is `/home/forge2/forge2_ws`. Generated plans, captures
and logs remain under ignored `build/` and `logs/`; the five explicitly archived
JSON reports are the documented exception. A new checkout does not include other
local artifacts or the candidate files referenced by their original paths.
Normal host changes do not require a firmware flash. The operator performs physical testing; do not open a
second owner or interrupt a powered owner while the load depends on its torque.

Priority order: HOLD-01 and required CAL-01/SUPPORT-01 checks, then WEAVE-01.
LOAD-01 and FEEDBACK-01 remain deferred until the first qualified arm welding run.
Software checks are listed in [tests/README.md](../tests/README.md).

## Publication validation — November 17

After cleanup, all **166 weld tests**, **six arm-geometry tests** and **43 model/
calibration tests** passed, with no skips. The offline `control_weld.py --audit`
also passed. Coverage includes native fake-CAN execution, Qt ownership/readouts,
recorded tracking regressions, complete model paths, D3 closure, overview/colors,
magnetics, terrain, actuators and calibration. These are software/model checks;
physical findings and unfinished acceptance remain separate above. All five
archived report blobs match their original bytes and recorded SHA-256 hashes.
Reproduction commands are in [tests/README.md](../tests/README.md).
