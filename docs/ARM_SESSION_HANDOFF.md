# Arm session handoff

Updated 2026-11-16 for `NSSRS/FORGE`, branch `nathan`.
Open acceptance work is owned by [ARM_LAYER_TODOS.md](ARM_LAYER_TODOS.md).
Operator procedures belong in [ARM_RIG_TEST.md](ARM_RIG_TEST.md); controller
architecture and planning are described in [WELD_MOTION.md](WELD_MOTION.md).

## Next hardware session: November 17

The operator reports that most/all joints reach the preparation target, then
sag slightly before welding starts. Prioritize **HOLD-01**: verify the fixed
endpoint hold, identify the physical holding response, and tune joint stiffness
with recorded per-axis drive settings if needed. No physical cause or stiffness
improvement is claimed yet. Follow the [hold acceptance procedure](ARM_RIG_TEST.md#post-preparation-hold-acceptance).

The host now explicitly latches the settled endpoint without changing its raw
position, speed allowance or motor mode. Normal idle handling and pause preserve
that fixed target; pausing an unfinished trajectory still captures measured
positions. Disable, faults and closing remove torque. Native command refresh,
freshness checks and protection trips remain active. No gain/current writes or
feedforward were added. Mode-2 drive stiffness requires hardware tuning; the
existing host/firmware write whitelist does not expose those gains.

The panel displays the current stage goal and post-arrival error. Live JSONL
records include endpoint-hold state, target, speed allowance, cached enabled/
status/torque samples, encoder feedback, and drift/recovery events. Use this
boundary evidence to distinguish command changes from drive/load response.
An earlier A5 alignment/drop report remains separately unresolved.

## Current implementation

- Flow: measured pose -> verify hold -> checked recovery/standby -> approach ->
  ready -> explicit welding motion -> fixed endpoint hold. Output switches gate
  arm/chassis commands independently; no inactive-subsystem compensation exists.
- One owner per transport. Damiao commands use native position/velocity mode 2,
  USB type 4, raw output-shaft SI units at the bus boundary. The planner and panel
  use explicitly named degree-valued vectors. No type-25 jog smoothing is used.
- Preparation defaults to 5 deg/s and <=30 deg/s². A common path clock slows for
  lagging shafts. Per-axis progress and final-settling watchdogs are separate;
  the five-second goal is not a timeout or permission to truncate a move.
- Arrival requires <=0.25 deg error and stationary feedback for 0.2 s. The 3 deg
  motion tracking trip and all six 1.5 rad/s measured-speed trips are unchanged.
- Requested welding is 2 Hz / 12 mm/s / +/-10 mm. Weld command ceilings are
  68.7549 deg/s on all six axes. A historical, complete-route checked CMA-ES
  candidate reached 1.815288 Hz under provisional CAD dynamics; regenerate any
  saved solution after mapping, model or constraint changes.
- `--optimize-preparation` is an explicit **offline-only** staging comparison.
  For the recorded Nov 16 start and A5 candidate, it reduces nominal preparation
  from 20.9272 s to 4.5526 s; ideal fake-shaft arrival is 4.62 s including settling.
  It changes the intermediate pose and is rejected by physical owners. A synthetic
  lagging drive takes 14.44 s, so physical five-second execution is not established.
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

The WSL development checkout is `/home/ns_wsl/forge_ws/FORGE-nathan`.
Generated plans, captures and logs remain under ignored `build/` and `logs/`.
A new checkout does not contain those local files. Normal host changes do not
require a firmware flash. The operator performs physical testing; do not open a
second owner or interrupt a powered owner while the load depends on its torque.

Priority order: HOLD-01 and required CAL-01/SUPPORT-01 checks, then WEAVE-01.
LOAD-01 and FEEDBACK-01 remain deferred until the first qualified arm welding run.
Software checks are listed in [tests/README.md](../tests/README.md).
