# Arm session handoff

Updated 2026-11-16 for `NSSRS/FORGE`, branch `nathan`. This document describes
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
- Selection/planning now leave an unpowered arm disabled. Explicit recovery checks
  the fresh preload pose, verifies current-position holding, then follows a checked
  direct or joint-order route with measured settling at each waypoint. Collision
  or missing routes block startup; faults never resume automatically. See the
  [recovery sequence](WELD_MOTION.md#supported-startup-and-recovery).
- The requested normal startup is automatic pickup from a settled, hanging arm
  on a secured base, without hand-positioning each joint. This remains unqualified:
  the fake shafts have no gravity/load model, physical torque pickup is untested,
  and faults/shutdown disable torque. Qualify both powered recovery and the passive
  drop/swing path; supported-fixture replay does not complete this requirement.
- `arm_standby_compact.json` is an optional reviewed model candidate: original A2/A3,
  A5 model +60 degrees, about 113 mm wire-tip floor clearance. Calibration/default
  files are preserved and qualification flags remain false. The earlier Nov 16
  disabled diagnostic was rejected for modeled wrist self-contact and gun/chassis
  contact using the old A5 mapping. The later recorded straight pose passes with
  the new candidate below; the actual mount and fresh wrist alignment still need
  checking before physical recovery. The gun is absent, but the operator requires
  planning with its geometry included. Startup/hold/stop physical qualification
  remains open under CAL-01, SUPPORT-01 and DEPLOY-01.
- The operator confirmed a secured base and protection catch on Nov 16. The
  authorized disabled diagnostic passed on all six axes with 6-8 ms feedback,
  disable acknowledgement and mode restoration verified. It did not enable or
  move motors and does not qualify loaded pickup or shutdown. Local evidence:
  `build/arm_tests/protected-recovery-disabled-20261116-170946.json`; paired
  `protected-recovery-geometry-20261116-170946.json` records the rejected start
  and triangle-mesh cross-check. Collision guards remain unchanged.
- Hardware commands use Damiao position/velocity mode 2 and immediate USB type 4
  (`smooth=False`). The optional custom firmware jog-smoothing experiment is
  not used by welding. It did not resolve the reported manual-travel jitter.
- The original target remains **2 Hz, 12 mm/s feed, +/-10 mm weave**. All six
  weld command caps are **68.7549 deg/s (1.2 rad/s)**; temporary A1=20 deg/s
  limiting has been withdrawn. Measured-speed trips remain 1.5 rad/s.
  Initial/approach default to 5 deg/s in both CLI profiles, with
  at most 30 deg/s^2. Preparation uses a steady checked speed allowance and a
  common feedback-paced clock; no position kick, torque/gain change or widened
  tracking/arrival tolerance is applied. Physical jitter improvement is unverified.
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

The operator ran recovery and reported A1 oscillating within about one degree.
The saved physical session `build/weld/weld-1794827690288570596.json` contains
379 recovery samples: A1 requested angles and sampled encoder positions are
monotone, its speed allowance falls as low as 0.1 deg/s, and A2 lag reaches
2.996 deg near the unchanged 3 deg trip. This does not identify physical backlash
or exclude movement between samples or downstream of the encoder. The pause
event did not record its reason, so its cause is not asserted. Shutdown ended
with mode restoration false: A1 acknowledgement failure and A4 reported motion
after disable; a clean loaded shutdown remains unqualified. Preserve that reuse
fault and do not automatically reset/restart hardware.

The later physical session `build/weld/weld-1794828901283333587.json` used
10 deg/s and stopped on `Arrival timeout` after 22.35 seconds, with only
6.75/12.33 seconds of its paced home path completed. It never reached standby
or approach. Remaining raw angles to standby were A1 +4.87, A2 +25.56 and
A3 +18.50 degrees. A5 was commanded from about +8.7 toward +28.0 degrees and
sampled feedback reached +27.86; this does not resolve the operator's physical
drop/alignment report. No fault/disable is recorded during recovery; explicit
disable occurs later. This session's shutdown acknowledgement and mode restore
passed. The native bounded sample ring evicted recovery torque/enable samples
during the subsequent hold, so a complete time-correlated load trace is absent.

The software timeout defect is fixed: paced travel now checks for 0.1 degree
of net forward progress within 10 seconds on each lagging axis, followed by a
separate 10-second endpoint-settling timeout. The 3-degree tracking trip and
0.25-degree stationary arrival check remain. Mid-dispatch intent cancellation
now retains an established measured hold, matching between-tick pause behavior;
cancellation during activation and actual faults still disable. Tests are fake
plants/native fake wire, not physical qualification. The automatic 10 deg/s CLI
default is withdrawn. Live JSONL evidence records cached target/feedback/state
while running, and shutdown failures cannot replace the primary fault.

The operator now requests total preparation within **5 seconds**, interpreted as
current pose -> standby -> weld start including minimum stationary checks.
`--prep-seconds 5` records this goal; it does not override guards or stop early.
The saved route takes 20.93 seconds nominally at its 10 deg/s cap. Its optimistic
limits-only lower bound is 10.95 seconds, or 5.65 seconds even with unlimited
speed at the existing 30 deg/s² acceleration envelope. A2 raises 58.69 degrees
to standby then reverses 38.10 degrees toward weld start. Review unnecessary
standby travel and actual mode-2 response before promising five-second hardware
execution. Offline review artifacts: `build/arm_tests/recovery-review-20261116/`.
A bounded offline staging-pose search found a different route at 4.851 seconds
nominal, including 0.6 seconds minimum stationary checks, and an ideal fake-shaft
replay reached ready in 4.92 seconds. Its staging wire-tip height is 37.65 mm;
full geometry, fixed cable/raw bounds and the existing model acceleration check
pass with speed <=10 deg/s. This changes the intermediate pose; it has **not**
replaced the reviewed hardware standby or established physical tracking. Candidate
and preview: `staging-candidate.json`, `prep-candidate-4.85s.gif` in that directory.
Tracked evidence: [recovery review](measurements/arm-recovery-review-2026-11-16.json).
Next: reconcile the reported A5 physical motion with commanded motion and its
independent pose mapping. Do not use another complete recovery as the diagnostic.

The independent `scripts/mirror_arm.py` window now exposes raw shaft radians and
degrees, model angles, per-axis candidate gain/offset and feedback age beside a
direct kinematic rendering. Its explicit disabled-refresh mode obtains new
encoder replies with disable frames only; passive cache mode may remain stale.
View edits and snapshots do not modify calibration or motor settings. Follow
[the mirror procedure](ARM_RIG_TEST.md#independent-encoder-to-model-mirror) to
resolve the operator's suspected mapping mismatch before physical recovery.
The current all-negative directions and measured offsets remain candidates;
software/fake-wire/rendering checks do not independently establish their physical
correctness. No hardware was opened to implement or test this viewer.

The operator now reports that only A5 appears misaligned: the physical CW end
looks straight in the mirror, and the mismatch persists away from the end. The
[A5 observation](measurements/arm-a5-alignment-2026-11-16.json) preserves rounded
screenshot values (raw -92.20351 degrees, mapped +90.18558 degrees). Its exact
physical angle and corrected offset are unknown. The operator rejected manual
view-offset matching. The latest operator update identifies the A5 ends as cable
restrictions, with CW a few degrees shorter, and requests fresh CW and CCW captures.
Do not align either end to a CAD mechanical stop. The mirror now records both ends
and one independent extended-straight pose with dedicated buttons: ends define
raw bounds with a provisional 2-degree margin, straight fits the A5 offset, and
all other coordinate mappings remain unchanged. The operator has now saved a complete
round: CW raw -98.607605, CCW +116.836287, straight +8.341527 degrees. The candidate
is model A5 = -raw A5 +101.514058 degrees, with command bounds
[-96.607605, +114.836287] degrees. Evidence is in
`build/arm_mirror/a5-capture-1794824963125893820/`; the later CW-only recapture is
an incomplete round and is not merged into it. A matching compact standby now
requires raw A5 +41.514058 degrees, replacing the old -62.017927 target. Other
standby model angles and other axes' mappings are preserved. The local candidate
bundle is `build/arm_tests/a5-recovery-20261116/{mapping,standby}.json`; default
production/calibration files have not adopted the A5 change. A large-edit D3
closure branch jump was reproduced and corrected with bounded kinematic
continuation; collision geometry and margins are unchanged. The saved straight
pose to compact standby passes the 2 mm model path check in 28.63 seconds of
planned motion (`recovery-check.json` in that bundle). This is historical offline
evidence, not proof of the current physical pose, intermediate alignment or loaded
recovery. The complete recovery/approach/one-cycle weld model plan also passes at
1.730609 Hz; fake-shaft replay with the live model guard reaches `complete` with
no dropped trace (`plan.json` and `plan-check.json`). Ninety-three relevant model,
mapping, recovery, controller and GUI tests passed. No hardware owner was opened.
See the [capture procedure](ARM_RIG_TEST.md#independent-encoder-to-model-mirror).
Do not infer a 90-degree correction or relabel the old initial capture as a CW stop.

The later lowered-pose screenshot's `collision_139` / `collision_463` alarm was
reproduced from its rounded angles: convex overlap 0.096 mm, original triangle
mesh gap 14.883 mm, no triangle intersection. Welding checks now refine this
specific concave platform/upper-arm pair without changing clearance thresholds
or excluding it. Normal preview cameras show only failed clearance markers;
passed mounting-gap labels/rings are limited to explicit Contact / Contact parts
diagnostic views. Switching windows preserves calculation and unused preview plans;
active motion still pauses, queued starts/enable are cancelled, and no motion resumes
automatically. Space/Escape and heartbeat loss still invalidate plans.
The screenshot pose's full model plan
passes, with a 29.65-second recovery to standby. Local evidence is in
`build/arm_tests/contact-display-20261116/`; it is not fresh motor feedback or a
physical clearance measurement. Recalculate in the restarted panel before motion.

A4's operator-specified cable travel is **CW and CCW 270 degrees each from the
preserved initial pose**, not 360 degrees total. The fixed reference is raw
+173.183435236 degrees; bounds are [-96.816564764, +443.183435236] degrees.
`arm_cable_limits.json` records that policy. The existing A4 coordinate gain/offset
are unchanged; its candidate raw branch now covers this explicit nonperiodic
interval. Manual sessions apply the bounds to the MCU configuration, and
planning checks complete cubic extrema. Startup, recovery and model-angle aliases
must not reset the winding allowance. Physical cable/stop behavior remains unqualified.
The mapping fingerprint changed with the A4 travel policy; the checked-in initial
and compact standby fingerprints were refreshed without changing their pose angles.
Previously generated search solutions/plans need regeneration against this mapping.

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
sign/offset mapping and provenance. The gain magnitude is one (current sign `-1`
on all six axes): feedback is already
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
| Working checkout | `/home/forge2/forge2_ws` on native Ubuntu |
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
5. **SPEED-01:** current A1 recovery tracking work is active; broader manual-jog
   jitter research remains deferred. A2/A3 move smoothly by operator report; A1/A4/A5/A6
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
