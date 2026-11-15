# Arm session handoff

Updated 2026-11-15 for `NSSRS/FORGE`, branch `nathan`. This document describes
current software and the limits of the saved physical evidence. Open work is
owned by [the cross-layer registry](ARM_LAYER_TODOS.md); procedures belong in
[the rig guide](ARM_RIG_TEST.md) and [weld guide](WELD_MOTION.md).

## Current implementation

- Both motor families expose cached telemetry through their existing owners.
  ROS publishes Damiao `/arm/joint_states` at 25 Hz and ENCOS `/joint_states`
  at 50 Hz, plus diagnostics. [Telemetry details](MOTOR_TELEMETRY.md) distinguish
  genuine MCU feedback ages from ENCOS PDO observation ages; the latter do not
  prove fresh per-motor CAN reception. A second transport owner is not permitted.
- `src/controls/controls/controller/` owns measured alignment, motor constraints,
  MuJoCo closed-chain IK, spline retiming and multi-start CMA-ES.
  `hardware/` contains actual Damiao/ENCOS adapters and the execution state
  machine, beside `simulation/`. Thin launchers live in `scripts/`.
- Welding runs current pose -> derived standby -> weld start -> ready -> explicit
  weld -> hold. Arm and chassis switches independently gate outputs, assuming
  the inactive subsystem follows nominal motion; there is no switch compensation.
  The application controls motion, not welder energization.
- Hardware commands use Damiao position/velocity mode 2 and immediate USB type 4
  (`smooth=False`). The optional custom firmware jog-smoothing experiment is
  not used by welding. It did not resolve the reported manual-travel jitter.
- The original target remains **2 Hz, 12 mm/s feed, +/-10 mm weave**. All six
  weld command caps are **68.7549 deg/s (1.2 rad/s)**; temporary A1=20 deg/s
  limiting has been withdrawn. Measured-speed trips remain 1.5 rad/s.
  Initial/approach stages use 5 deg/s and at most 30 deg/s^2.
- The selected CMA-ES candidate reaches **1.815288 Hz / 10.891728 mm/s** under
  provisional model acceleration constraints, versus 1.730607 Hz before search.
  This bounded search found no checked 2 Hz candidate; that is not proof of
  impossibility. The candidate is `build/weld/cmaes-2hz-independent/solution.json`;
  its 12-cycle plan is `build/weld/cmaes-animation-plan.json`. Generated artifacts
  are local and ignored by Git; reproduction is in [WELD_MOTION.md](WELD_MOTION.md).
- The animation replays that saved plan kinematically. A separate fake-motor
  replay exercises the actual controller state machine. Neither establishes
  physical tracking, loaded dynamics, real welding or production readiness.

## Calibration and recorded evidence

All six motors are connected. The operator confirmed A1 rotation/cable clearance
and mechanical support; six-axis disabled identity/feedback checks passed on
Nov 12. The A2-A6 profile remains available only when A1 is deliberately absent.

The [Nov 12 reference file](../src/robot_description/config/arm_reference_positions_2026-11-12.json)
preserves the original eight readings and the
[full-precision capture](measurements/arm-reference-2026-11-12.capture.json).
A1/A4/A5/A6 initial anchors come from that capture. A2/A3 model alignment comes
from recorded endpoints and prior CAD geometry; their initial raw angles were
not separately captured in that eight-position sequence.

The [Nov 13 return observations](measurements/arm-return-checks-2026-11-13.json)
record one stationary cycle and one off-power return for each joint. Return
differences A1-A6 were -0.33, +0.31, +0.22, -0.48, +0.76 and 0.00 degrees,
including manual placement uncertainty. Do not repeat that same round solely
because these values are nonzero or use those arbitrary poses as new anchors.
Broader wrap/branch and independent pose validation remain open.

Nov 13 corrected A3 endpoints are **CW 72.77 deg, CCW 151.39 deg**. The original
CAD initial A3 pose overlaps its modeled stop by about 0.65 deg and is retained
as source evidence, not commanded automatically. The candidate controller
standby uses **raw A3 149.39 deg**, a derived 2-degree backoff from CCW. The
other standby coordinates use existing initial-pose intent and measurement
alignment. This is not a newly measured six-axis home.

`arm_calibration_measured.json`, `arm_initial_measured.json` and
`arm_alignment_report.json` in `src/robot_description/config/` hold the candidate
sign/offset mapping and provenance. The gain is one: feedback is already
output-shaft angle, so internal gearbox ratios must not be applied again.
`arm_motors.json` and `arm_motors_final.json` retain null/false production fields.

The Nov 12 capture passed stationary/freshness gates, but cleanup exited 2:
all six final rows reported disabled, A2 still reported -0.168498 rad/s against
a 0.03 rad/s stationary threshold, and disable acknowledgement/mode restoration
were unconfirmed. Preserve [the session summary](measurements/arm-reference-2026-11-12.session-summary.json).
Successful recording is not evidence of a qualified loaded shutdown.

## Configuration and entry points

| Item | Current state |
| --- | --- |
| Working checkout | `/home/ns_wsl/forge_ws/FORGE-nathan` in Ubuntu-24.04 WSL |
| Controller / profile | DM-MC02, normally `/dev/ttyACM0`; `arm_rig_a1_a6.json` |
| Routing | A1 CAN1/1/17; A2-A6 CAN2, command IDs 2-6, feedback IDs 18-22 |
| Manual controls | `bash scripts/control_arm.sh`; holding jog 10 deg/s, non-holding/teaching 5 deg/s |
| Reference recorder | `bash scripts/record_arm_positions.sh`; eight-step reference workflow |
| Model-guided calibration | `.venv/bin/python scripts/calibrate_arm.py` |
| Offline welding / real path | `scripts/control_weld.py --plan-only` or `--simulate`; explicit `--execute` selects hardware |
| Search / animation | `scripts/optimize_weld.py`; `scripts/render_weld.py` |
| Reactive torque trips | A1 20 Nm; A2/A3 12 Nm; A4-A6 3.5 Nm |
| Saved deployed firmware evidence | Nov 12 capture: `d61f6d16`, capabilities `0xFFF8F` |
| Production readiness | Not confirmed; no physical weld acceptance yet |

Use the detailed guides for dependencies, profiles and complete commands.
The firmware image is pinned at `build/dm_mc02/jog-10/forge_arm.elf`; the generic
root build image may be overwritten. Normal host workflow updates require no
new flash. Do not use earlier build notes to infer which image is deployed.

One owner must retain the live transport. Default jog release disables torque;
explicit hold modes and welding maintain authorized position holds until a
supported disable or fault. Hold is not a mechanical brake. No dependable
rest/restraint on torque loss has been qualified. The operator performs hardware
work; do not stop, restart, flash or open another owner while load relies on it.

## Remaining work and findings

1. **WEAVE-01:** qualify the measured initial transition, approach and arm weld
   on the supported fixture, recording tracking, timing, stops and the actual
   attained frequency. Physical acceptance remains separate from model replay.
2. **SUPPORT-01, CAL-01, PROTECT-01, DEPLOY-01:** retain unresolved restraint,
   independent alignment, current/protection and load/fault acceptance gates.
3. **LOAD-01, after the first arm weld:** identify actual mass/COM/inertia and
   introduce qualified weight/gravity compensation. The operator estimate is
   6.5 kg, 0.30 m COM extension, 24 V, with rough A2/A3 30-45 degree poses; it is
   not the current CAD dynamics. No load compensation is active in hardware.
4. **FEEDBACK-01, after the first arm weld:** timestamp baseline tracking, add
   tool/seam sensing and bounded outer position correction. Assess a discrete
   ESO first in observation mode with identified mode-2 dynamics. Encoders
   alone cannot detect all base motion, flex or seam-position errors. No ESO or
   external Cartesian feedback correction is active.
5. **SPEED-01, deferred:** A2/A3 move smoothly by operator report; A1/A4/A5/A6
   jitter during travel but hold steady. The 5 deg/s smoothing trial did not
   resolve it. Higher manual jog defaults are a comparison setting, not a fix.

Retain the following unresolved evidence when planning physical tests:

- An earlier A4 feedback spike reported 2.4542 rad/s despite the 1.5 rad/s trip;
  it is not independently verified physical speed. Raw timing and fault
  attribution remain open; see local `logs/arm_tests/` and SPEED-01.
- The retained smoothing trace covered only an A2 jog after 44,600 samples were
  evicted. Whole-run summaries showed about 124 ms maximum host gap. Existing
  timing fields and MCU counters support a new attributable test; they do not
  clear timing or jitter on all axes.
- Mode 2 sends position/speed, not torque. `Feedback Nm` is a drive estimate and
  `Trip Nm` a reactive threshold; no hidden 2 Nm command clamp was found.
  A6 control readbacks matched A4/A5; no gains/current settings were changed.
  See [the current/torque audit](DAMIAO_TORQUE_LIMITS.md).
- The operator confirmed the earlier A1 undervoltage issue was fixed. Broader
  bus, host-loss and fault-injection qualification remains open.

Run the relevant [software tests](../tests/README.md) after source changes.
Fake-motor, ROS/PTY, model and physical evidence must remain clearly separated.
