# Measurement-driven weld controller

The reusable solver is in `src/controls/controls/controller/`. It uses MuJoCo
closed-chain kinematics and inertia, SciPy bounded least squares and spline
retiming. No motor transport is imported by this layer. `hardware/` contains the
single-owner Damiao/ENCOS adapters, execution state machine and Qt panel.
`controls/hardware/` is a sibling of `controls/simulation/`, not a placeholder:
it sends real motor commands when the operator explicitly selects `--execute`.
See the [hardware directory guide](../src/controls/controls/hardware/README.md)
for the command chain and the distinction between trajectory animation,
fake-motor execution and physical execution.
Mink is not required: the existing MuJoCo closure, meshes and joint model are
reused directly. This is candidate search with measured constraints, not a claim
of a global optimum or physically qualified dynamics.

## Run without hand-calculating timing

From the workspace root, with the existing simulation/calibration environment:

```bash
.venv/bin/python scripts/control_weld.py --plan-only --profile weld --cycles 1
.venv/bin/python scripts/control_weld.py --simulate --profile weld --cycles 1
```

The first command solves and saves the entire route without any motor owners.
The second opens the offline panel. Both motion switches start OFF. Select the
outputs, calculate, then choose **Start recovery** to verify hold and reach the weld start.
Selection and calculation do not enable an unpowered arm. Measured arrival and
stationary confirmation separate recovery waypoints and the approach. **Start welding
motion** is enabled after arrival at the weld start; it never energizes a welder.

While solving, the panel shows **Planning**, elapsed time and an activity bar.
An existing saved solution does not skip checks from the current measured pose.
Switching windows preserves pending calculation, an unused accepted plan and
preview playback. Focus loss still pauses active recovery/welding and cancels
queued starts or an enable in progress; returning never resumes motion. Space/Escape
or loss of the UI heartbeat cancels the pending plan and pauses motion.
Cancellation reasons are shown explicitly; discarded results never enable recovery.
**Calculated** frequency and **Start recovery** become available only after the plan
is accepted.

The panel now embeds a model view: **Current pose**, **Standby target**, and
**Checked path**. **Play preview** and the time slider replay the accepted
recovery/approach/weld paths without sending motor commands. Overview, side and
wrist cameras help compare modeled contacts with the assembly. Normal views show
only failed clearance markers; passed mesh-gap diagnostics appear only in the
explicit **Contact** / **Contact parts** views. Blocked plans have no playable path. The renderer runs
in its own process and receives only poses, never a transport owner. The gun is
always included and the chassis stays fixed in this view. See the
[operator commands and controls](ARM_RIG_TEST.md#weld-panel-with-embedded-model-preview).
`--start-report PATH` reproduces a saved disabled six-axis pose with fake motors;
it is rejected in hardware mode. Passive display shows sample age, while explicit
startup actions refresh disabled feedback through the existing owner before
stationary checks. An existing powered hold is retained.

### Supported startup and recovery

The requested operating behavior is a secured base with the unpowered arm
settled in its natural hanging pose, followed by measured-pose capture, motor
weight pickup and automatic travel to standby. The operator should not need to
hand-place each joint at standby. This unsupported hanging-start behavior is an
open physical acceptance requirement, not established by the existing tests.
`SimulatedArm` models rate-limited shafts without gravity, load or torque pickup;
its passing replay cannot qualify lifting the real arm. The current fault and
shutdown policy disables torque, so the resulting passive drop/swing path also
needs qualification. Software holding cannot persist through loss of power.

Until that behavior is qualified, the existing physical commissioning procedure
requires a fixture that carries the links on torque loss. This test requirement
must not be presented as completion of automatic hanging-start recovery.

Keep the unpowered links mechanically supported. The sequence is fresh measured
pose -> checked recovery route -> current-position preload/enable -> stationary
hold verification -> checked waypoints -> standby -> weld start -> explicit weld.
Selecting the arm only selects participation; an already active hold is retained.

`controller/recovery.py` first checks the current pose and standby, then the direct
joint transition. If obstructed, it searches orders of individual joint moves
between combinations of the measured and standby angles (at most 64 poses and
192 graph edges). The order depends on collision results, not a fixed wrist-first
rule. Each edge and the final interpolated route are checked. No route, an already
colliding start, an invalid encoder branch or failed D3 closure blocks startup.
This bounded search does not find every possible detour and never automatically
escapes contact with a chassis, stop or support fixture.

Pressing Start recovery rechecks the start and modeled clearance. The transport
owner checks its fresh disabled preload snapshot immediately before enable;
movement over 0.3 degrees from the planned start rejects activation. Current-angle
holding must remain stationary for 0.2 seconds, within that same start tolerance,
before the first recovery command. Failure to settle within 2 seconds pauses and
requires recalculation. Each intermediate waypoint then requires measured arrival
within 0.25 degrees and stationary feedback for 0.2 seconds before the next segment.
The model checks measured poses during holding verification and active motion.
Faults stay latched and never automatically re-enable or resume.

Recovery uses 2 mm modeled arm/environment clearance; approach/welding retains
0.5 mm. These are geometric screening margins, not qualified tracking/stopping
envelopes. Existing collision masks, neighboring-body exclusions and explicit
A2/A3 triangle stops remain. Cables, support fixtures, calibration uncertainty,
torque pickup and torque-loss behavior still require physical qualification.
Old plans without recovery metadata must be recalculated before execution.

The reviewed compact candidate preserves the original A2/A3 standby and changes
A5 to model +60 degrees, giving approximately 113 mm wire-tip floor clearance.
It is saved separately from the measurement-derived calibration/defaults:

```bash
.venv/bin/python scripts/control_weld.py --plan-only --profile weld --cycles 1 \
  --initial-pose src/robot_description/config/arm_standby_compact.json
```

Plan-only starts at that candidate, not a measured unpowered pose. The latest
Nov 16 disabled measurement fails the modeled wrist self-contact and gun/chassis
checks; the earlier measurement failed floor clearance. Reconcile the actual
mount and wrist alignment before physical recovery. The operator requires the
gun geometry to remain in planning even while the physical gun is absent.
No hardware qualification or production calibration is implied.

The solver automatically slows an infeasible request and displays the resulting
frequency and nominal feed before motion. `--strict-timing` instead rejects
infeasible timing. `--time-scale` remains an optional explicit minimum slowdown;
it is no longer necessary to guess a factor. `--max-speed-deg-s` permits a debug
ceiling up to 10 deg/s for initial/approach and the debug profile. The weld CLI
profile and debug both default to 5 deg/s for preparation; the automatic 10 deg/s
increase was withdrawn after the Nov 16 recovery failure. An explicit value
overrides the CLI default but does not qualify physical tracking.
`--prep-seconds 5` sets the requested total preparation goal, including initial
hold verification, standby stops and approach settling. The plan reports nominal
time and a limits-only lower bound. A missed goal is shown explicitly, not used
as a motion timeout, an authorization to raise limits, or a preview speed factor.

The operator reaffirmed the original 2 Hz target; 1.73 Hz is an acceptable
fallback. `--profile weld` loads 2 Hz and 12 mm/s
feed (6 mm per cycle) from `src/controls/config/weld_experiment.json`. Its weld
stage applies `src/controls/config/arm_motion_policy.json`: all six temporary
per-axis caps have been withdrawn (null). The operator confirmed restoration
to the original project configuration: physical/protocol limits and the 80%
measured-speed-trip guard, currently 68.7549 deg/s on A1-A6. The measured-speed
trip remains 1.5 rad/s; these are project protection values, not factory motor
register settings. VMAX is only a protocol encoding range. The model-derived
coupled acceleration box also applies. The solver may reduce the requested 2 Hz
and feed together to honor the caps. Initial and approach stages default to
5 deg/s in this profile, with the existing <=30 deg/s^2 model envelope.
The separate `--profile debug` is still the default
when no profile is supplied; it retains low-speed limits throughout. Profile
selection never changes measured-speed/torque trips or starts a motor.

### Offline CMA-ES weight and branch search

The reusable search is `controls/controller/weld_search.py`, with a thin entry
point in `scripts/optimize_weld.py`. Install the optional search environment:

```bash
.venv/bin/python -m pip install -r src/controls/search-requirements.txt
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/optimize_weld.py \
  --output-dir build/weld/my-search --seeds 11 23 47 \
  --generations 12 --population 12 --workers 6
.venv/bin/python scripts/control_weld.py --simulate --profile weld --cycles 1 \
  --solution build/weld/my-search/solution.json
# Optional additional search around an already checked candidate:
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/optimize_weld.py \
  --output-dir build/weld/my-refined-search --warm-start build/weld/my-search/solution.json \
  --seeds 101 223 347 --generations 40 --population 12 --workers 6
```

Use a new output directory for each run. Search opens no hardware owners and
never modifies motor limits. It uses [pycma](https://github.com/CMA-ES/pycma),
independent restarts, bounded log mobility weights and different center-yaw /
wrist IK seeds. Greater mobility means MORE willingness to move that joint;
the corresponding IK penalty is its normalized inverse. Initial populations
prefer A4-A6, then A2/A3, over A1. The five mobility ratios are optimized, not
hard-coded as additional speed caps. All current configured caps remain hard limits.
`--warm-start` verifies the saved candidate's identities before using it and
retains that candidate among the alternatives; different restart spreads keep
the refinement from depending on a single initial population.

The objective is evaluated at the unchanged requested clock and stroke: large
penalty for required slowdown, then normalized peak/RMS speed, acceleration,
and unnecessary direction changes. A1 motion costs more than shoulder motion,
which costs more than wrist motion. No reward can shrink the seam amplitude,
remove dwell intervals, loosen clearance, or waive speed/torque checks.

Coarse evaluation only ranks candidates. Diverse finalists are regenerated
with 41 spatial knots, 401 task/clearance checks and the sampled reduced-inertia
envelope. Complete-route acceptance additionally checks the initial transition,
approach, and interpolated cycle using the normal travel-bounded collision
sampling. A new center branch is approached via a checked reverse lift when
the original D1-zero approach cannot reach that branch continuously.

`search.json` retains all evaluations, seeds, failure reasons, scores and dense
finalists. Only an improving candidate with a checked complete route produces
`solution.json` and `plan.json`. `--solution` verifies model, mapping, constraints
and amplitude identity and solves/checks the entire route again. It never
replays an unchecked search sample. Changing participation switches still only
gates outputs. These are model candidates, not globally optimal or physically
qualified trajectories; hardware timing/tracking remains an operator task.

The earlier A1=20 deg/s search is historical. Its saved constraint fingerprint
is intentionally stale after withdrawal of the temporary cap; do not edit
fingerprints to bypass checks. Current results are described below. Without
`--solution`, the default planner compares its standard candidate branches.
Search reports and selected plans remain in ignored `build/weld/`; reproducible
entry points and the controller implementation are source files.

The operator subsequently estimated 6.5 kg moving load and 0.30 m COM extension
and confirmed 24 V supply. This estimate is NOT incorporated into the CAD
dynamics: the model A1 descendant mass is about 0.898 kg. Inertia is recomputed
for the candidate poses, but still comes from provisional CAD masses. The corresponding point-mass
term alone would be 0.585 kg m^2 if 0.30 m is the perpendicular axis distance.
Mass distribution, COM direction, angle convention and chassis orientation must
be reconciled before applying the reported frequency to that load. Current
search timings remain evidence for the provisional model only. No motor limit
or physical owner was changed by the load estimate.

The operator explicitly deferred weight/gravity compensation until after the
first arm welding run. This follow-up is tracked as
[LOAD-01](ARM_LAYER_TODOS.md#load-01--weight-compensation-after-the-first-arm-welding-run).
The current milestone is the complete arm sequence and its trajectory preview;
mass/inertia identification, feedforward and loaded retuning follow it. Chassis
and arm switches remain independent output gates, without switch compensation.

### Render the actual optimized plan

With the original 2 Hz target restored, two 48-generation searches (population
12; seeds 307/419/563 and 601/733/887) evaluated 3,456 new candidates plus one
warm-start reevaluation. The best dense and complete-route checked candidate
reaches about **1.815288 Hz / 10.891728 mm/s**, versus **1.730607 Hz** for the
default candidate comparison (about 4.89% faster). All six command speed limits
remain 68.7549 deg/s. Executable peaks A1-A6 are approximately 34.796 / 2.762 /
5.833 / 27.599 / 3.264 / 41.677 deg/s. The requested 2 Hz trajectory exceeds the
provisional acceleration box; speed limits are not the active restriction.
This bounded search did not find a checked 2 Hz solution. It is not a proof
that no such solution exists, nor a measured-load result.

The selected local candidate is
`build/weld/cmaes-2hz-independent/solution.json`; its 12-cycle playback plan is
`build/weld/cmaes-animation-plan.json`. Standby/approach retain 5 deg/s limits.
The target stays at 2 Hz in project configuration; automatic retiming reports
the achieved value instead of silently replacing that target with 1.73 Hz.

The renderer lives in `src/simulation/simulation/weld_animation.py`; the thin
launcher has no motor-owner or hardware options. First regenerate a complete
plan from the selected solution, then render that exact saved path:

```bash
.venv/bin/python -m pip install -r src/simulation/requirements-animation.txt
.venv/bin/python scripts/control_weld.py --plan-only --profile weld --cycles 12 \
  --solution build/weld/my-search/solution.json --output build/weld/animation-plan.json
.venv/bin/python scripts/render_weld.py --plan build/weld/animation-plan.json \
  --output-dir build/weld/animation --approach-speedup 12
```

Use a new output directory. `--preview-only` produces four keyframe PNGs for
framing checks. The full renderer writes a 1280x800 MP4, a real-time weld-only
GIF and a provenance/validation JSON. It rejects stale model, mapping or motor
constraints, discontinuous stage endpoints and saved derivative-limit violations.
Every rendered pose is checked against the measured mapping, D3 closure and
scene clearances; the complete plan is densely checked during planning.

The full video shows standby -> approach -> ready -> welding -> hold. Approach
playback is accelerated and labeled; the weld uses the actual retimed clock at
1x. The offline plan starts at its saved standby, so it does not invent a
measured current pose or claim a physical homing run. The inset plots planned
tip positions and the speed bars use output-shaft derivatives, not model-angle
scaling or visual guesses. Chassis translation is assumed nominal feed and
never adds arm compensation for a disabled participation switch. This is
kinematic MuJoCo replay, not a loaded physics simulation or measured tracking.

### Where smoothing lives

The extra slew was added by FORGE to the DM-MC02 firmware (commit `9038ccd`),
not supplied as Damiao motor firmware. USB type 25 moves the last transmitted
target toward the endpoint by at most speed * 2 ms per CAN tick; its separate
10 deg/s cap belongs to manual jogging. The earlier operator trials did not
establish a jitter improvement. Native position-velocity mode 2 already exists.

Weld trajectories now use existing USB type 4 / native mode 2 directly. They
retain the controller's smooth time-indexed trajectory but bypass the second
manual-jog slew. Manual controls keep their existing behavior. This change needs
no flash, motor reset or parameter writes. Group-hold activation, feedback/command
deadlines, measured-speed/torque trips, fault latches and stop handling remain.
The CAN publisher still ticks at 500 Hz; host trajectory updates are 50 Hz by
default. Those are different rates, and physical tracking/timing is not qualified.

The ROS entry requires the source-API helper and simulation Python environment:

```bash
source scripts/ros2_env.sh
ros2 run --prefix .venv/bin/python controls control_weld --simulate --profile weld --cycles 1
```

## Measurements dictate controller model coordinates

The controller defaults to `arm_calibration_measured.json` and
`arm_initial_measured.json` in `src/robot_description/config/`. The earlier
`arm_calibration_candidate.json` and MuJoCo `initial_pose.json` remain preserved
source references. There are no geometry, mesh, link-length or global CAD-zero
changes. The controller applies the measured mapping to its MuJoCo configuration.

Reproduce the decision or regenerate the candidate files with:

```bash
.venv/bin/python scripts/fit_arm_model.py
.venv/bin/python scripts/fit_arm_model.py --output-dir src/robot_description/config
```

`calibration.py` compares offset-only and affine fits. The default is
`model_deg = -raw_output_shaft_deg + offset_deg`. The encoder is already on the
output-shaft coordinate system; the internal 9:1/40:1/10:1 ratios must not be
applied again. A1/A4/A5/A6 retain their full-precision Nov 12 initial anchors.
A2/A3 use the measured endpoints and the previous CAD stop estimates to fit the
offset, retaining the corrected Nov 13 A3 endpoint labels.

The old affine gains were -1.006584 (A2) and -0.997309 (A3). A unit-magnitude fit
leaves endpoint residuals of +/-0.4563 and +/-0.1058 degrees respectively. Two
endpoints fit an affine model exactly by construction; they cannot distinguish
CAD/contact uncertainty from a real transmission ratio. The generator therefore
keeps gain -1 and reports both fits. Explicit scale fitting requires at least
three independently matched poses; an independently known external ratio may
also be supplied to the fitting API. No new ratio is inferred from a return test.

The old A3 initial target maps beyond the measured upper endpoint. The generated
controller standby projects the prior intended raw pose into the measured range
with a declared 2-degree backoff; it does not enlarge that range. Its six targets:

| Axis | Raw output-shaft degrees | Basis |
| --- | ---: | --- |
| A1 | -174.691596 | measured initial anchor |
| A2 | -61.357358 | inherited model intent, not separately captured initial |
| A3 | +149.390000 | measured upper endpoint 151.39 minus 2-degree backoff |
| A4 | +173.183435 | measured initial anchor |
| A5 | +8.753680 | measured initial anchor |
| A6 | +64.161049 | measured initial anchor |

A3's controller model coordinate is now -22.662356 degrees. The generated pose
and fit report retain source fingerprints and each adjustment. This is a
calculated commissioning standby, not a relabeled six-axis physical measurement.
It still undergoes raw-branch, authored-joint, D3 closure and collision checks.
A4 is nonperiodic: CW/CCW 270 degrees from the preserved initial encoder anchor.
Raw travel is fixed at [-96.816565, +443.183435] degrees and does not reset at
startup. Exact spline extrema are checked against the cable envelope. A5 cable
endpoints must be recorded separately from its independent straight-pose offset;
see [the mirror procedure](ARM_RIG_TEST.md#independent-encoder-to-model-mirror).
`--mapping` and `--initial-pose` accept a matching reviewed alternative; generated
pose files carry their mapping fingerprint to prevent mixing coordinate systems.
Production/hardware qualification flags remain false.

## Better joint solution and repeatable timing

The planner compares the old fixed-D1 choice against two solutions that use D1
redundancy, with equal, rated-capability or executable-speed-weighted posture
regularization. The speed-weighted candidate discourages motion of the slow A1
axis and lets faster axes take more of the transverse motion where feasible. It
constrains wire position and direction while allowing rotation about the wire;
it does not unnecessarily prescribe all three orientation degrees of freedom.
`--lock-base-yaw` restricts solving to the original D1-zero experiment.

MuJoCo solves the real D3 linkage with its 150 mm connection-center spacing.
Spatial continuation solves each transverse position once and retraces the same
branch, preventing redundant-joint drift between cycles. Shape 1 keeps its
+/-10 mm stroke and dwell intervals, but quintic ramps replace instantaneous
velocity changes. A periodic cubic joint interpolant supplies continuous
velocity and acceleration; the model checks interpolated tool position and
wire direction as well as all joint and collision constraints.

Exact extrema of the piecewise polynomial determine the minimum common time
factor within each candidate, with a 2% margin when slowing is required. Both
arm frequency and nominal chassis feed use that factor. Candidate selection
minimizes executable cycle duration among the checked alternatives. Initial and
approach moves are separately retimed; a checked raised-tool route is used if a
direct joint transition clips the floor.

All added per-axis caps, including A1's temporary 20 deg/s cap, are removed.
Recalculate saved paths after this policy change;
the executable frequency/feed and predicted joint peaks are recorded in each
fresh plan, including the 2% margin whenever retiming is required.

This is the best checked candidate, not a global optimum or measured motor
performance. New geometry or settings must be solved again; reports save all
candidates, per-axis policy and limiting constraints. Requested and executable
timing remain distinct. The separate debug profile retains 5 deg/s / 30 deg/s^2.

## Damiao physical and deployment constraints

`src/controls/config/damiao_physics.json` pins each manufacturer's manual URL,
commit and model revision. The specifications on PDF page 7 were checked visually:

| Axis | Model | Rated output at 24 V | Rated output torque |
| --- | --- | ---: | ---: |
| A1 | J8009P-2EC V1.1 | 100 rpm / 600 deg/s | 20 Nm |
| A2/A3 | J4340P-2EC V1.1 | 36 rpm / 216 deg/s | 12 Nm |
| A4/A5/A6 | J4310P-2EC | 120 rpm / 720 deg/s | 3.5 Nm |

The 24 V specification is the selected manufacturer rating; the operator also
confirmed A1's 24 V supply. This is not a new under-load voltage measurement.
The 4310P hardware revision remains unverified. Manufacturer references:
[8009P](https://github.com/dmBots/DM-J8009P-2EC/tree/42eaaf7f05cccba18d182b51504ee0349e16cff6/manual),
[4340P](https://github.com/dmBots/DM-J4340P-2EC/tree/98a29a9df67c83512795f8bb9ca2bdd2423f66dd/manual),
[4310P](https://github.com/dmBots/DM-J4310P-2EC/tree/9e728c9a42b498380ae43964636b3d66d01f42c7/manual).

`limits.py` computes three distinct layers:

- Physical design speed: 50% of rated continuous output speed, also bounded by
  the actual configured protocol range. Never use no-load speed or peak torque
  as a continuous loaded operating rating.
- Executable speed: weld caps take the minimum of physical design speed,
  any explicit per-axis policy and 80% of the measured-speed trip. Debug and
  initial/approach stages additionally apply the selected host ceiling
  (both CLI defaults 5 deg/s; maximum 10 deg/s).
  Type 25 is bypassed for welding; its cap is not a native mode-2 hardware cap.
  Null policy entries for all six axes mean no extra per-axis cap; their
  physical design and 80% trip margin remain (currently 68.7549 deg/s commands).
  The feasibility report, path optimizer, plan acceptance and native adapter
  all enforce these caps. A saved plan with a larger cap is rejected even if its
  old trajectory would fit. Changing policy requires a fresh calculation and
  reopening an active owner; there is no live cap increase.
  The existing 1.50 rad/s measured-speed trips remain unchanged.
  No MCU capability, firmware, reactive trip or manual control policy is raised.
- Acceleration: MuJoCo mass and gravity/bias are projected through D3 closure to
  six raw shafts. From a 50%-rated torque budget, static demand is subtracted and
  half the reserve is allocated to a conservative simultaneous-acceleration box.
  Debug/home/approach component limits are additionally capped at 30 deg/s^2;
  the accepted weld stage uses the model box directly. Home/approach and weave
  are checked with their respective sampled inertia and separate stage envelopes.

Reports expose physical-only design timing separately from executable timing.
CAD inertias are provisional; unmodeled rotor inertia, friction, Coriolis at higher
speeds, payload, cables, thermal limits and power availability are not certified.
The model-derived acceleration box is a commissioning design estimate, not a
validated torque-speed envelope. Mode 2 sends position and a speed magnitude;
this implementation does not send model torques to motors.

## Independent debug outputs, no compensation

Both switches only gate command output. The solver always assumes the other
component performs its nominal motion, even when that component's output is OFF.
Turning chassis output OFF does not add longitudinal motion to the arm, change
its IK target, or select a different arm solution. Turning arm output OFF does
not ask the chassis to compensate. Chassis-only solving uses the same nominal
weave constraints and retimed clock, without opening/commanding an arm owner.
There is no odometry, slip or weld-gap compensation in this debugging stage.

This does not mean the joints have no feedback: native mode 2 performs motor
position control, and the host compares each requested raw angle with cached
encoder feedback. Preparation now supplies the checked stage's constant speed
allowance (capped by the selected preparation limit), instead of reducing that
allowance to 0.1 deg/s near each small reference step. Native position control
still receives positions on the checked path, without overshoot offsets or a
backlash kick. This changes a speed allowance, not motor torque or loop gains.

Preparation advances a common six-axis path clock: full progress below 0.5 deg
tracking error, slowing toward zero at 1.5 deg. Clock slew is bounded using exact
path speed/acceleration extrema so the reference remains within the stage
acceleration envelope; planning reserves 10% of that envelope for pacing.
The clock never reverses or skips a waypoint. Paced travel does not use nominal
path duration as a wall-clock deadline: doing so caused the recorded Nov 16
recovery to stop at 54.7% of its home path. Each axis lagging by at least 0.5 deg
must make 0.1 deg of net progress toward its reference within 10 seconds;
bounded encoder noise does not reset this watchdog. Once the reference reaches
the endpoint, a separate 10-second settling timer starts. The 3 deg error trip and
0.25 deg stationary arrival requirement are unchanged. Persistent lag still
pauses for inspection rather than advancing to the next stage.
Welding retains its existing time-indexed path and rate/error-based speed limit.
The panel shows path-rate percentage, largest-axis error and A1 error/allowance;
the table separately shows each joint's remaining angle to standby. The report
records actual path time, clock scale, tracking errors and pause reasons.
Preview shows nominal timing; physical preparation may take longer while pacing.
The default total preparation goal is 5 seconds; failure to meet it remains
visible. The saved 10 deg/s route is nominally 20.93 seconds including minimum
settling and has a 10.95-second optimistic lower bound under its current limits.
Route efficiency and mode-2 response must be addressed before that goal is met.
A separate offline candidate replaces the long standby detour with a lower
staging pose. It checks at 4.851 seconds nominal (4.92 seconds in ideal fake-shaft
replay), with 37.65 mm staging wire-tip height, <=10 deg/s speed and the unchanged
model acceleration/collision checks. It is saved under
`build/arm_tests/recovery-review-20261116/`; it is not promoted to hardware
standby, and its fake shafts do not establish physical five-second execution.

`weld_recording.py` writes cached evidence to a live JSONL file on a bounded writer
thread. It never polls a transport or renews a command lease. The accepted plan,
targets, feedback, native cached enabled/status/torque samples, pause/fault reasons
and cleanup are recorded without waiting for the window to close. Records are
flushed, not guaranteed durable against power loss; queue overflow/disk errors are
reported. A later cleanup failure cannot overwrite the first controller fault.

The encoder-to-model offsets are static calibration, not online wire-tip correction.
The current controller has no ESO, sensor-based Cartesian outer loop or load
feedforward. Those follow-up tasks are recorded under
[FEEDBACK-01](ARM_LAYER_TODOS.md#feedback-01--sensor-feedback-and-bounded-weld-tip-correction)
and LOAD-01, after the first welding run. Current targets, motor modes and limits
are unchanged by this research proposal.

Changing either switch pauses and invalidates the old plan; calculate again
before starting. An OFF arm output retains an existing powered hold; **Disable
arm torque** is separate. Space/Escape/focus loss pause motion, command chassis
zero and capture fresh measured arm hold targets. This also applies to intent loss
mid-dispatch after group hold is established; incomplete activation still disables
the group. No automatic resumption occurs.

Stale feedback, native transport faults and limit violations stop the flow.
A control gap over 100 ms pauses without catching up; GUI heartbeat loss pauses.
Tracking error over 3 degrees pauses both outputs. Arrival needs <=0.25 degrees
and stationary feedback for 0.2 s, with a 10 s endpoint-settling deadline. These
thresholds are commissioning policy, not welding accuracy acceptance.

Model validation samples complete interpolated motion at <=0.2-degree joint
travel per sample, with 2 mm recovery clearance, 0.5 mm approach/weld clearance and the existing 0.1 mm
adjacent-stop triangle check. Model collision masks/exclusions remain in effect.
No new clearance of unmodeled cables, fixtures or real stopping distance is claimed.

## Operator hardware entry points

The operator performs hardware sessions; development/tests did not operate motors.
Complete recovery retries are currently on hold pending the reported A5 physical
motion and actual tracking review. The commands below describe the interface,
not a recommendation to repeat the failed recovery as a diagnostic.
Use a supported fixture and close competing owners before starting one owner per
controller. The following commands require actual hardware review of the candidate:

```bash
.venv/bin/python scripts/control_weld.py --execute --arm --supported --profile weld --cycles 1
sudo .venv/bin/python scripts/control_weld.py --execute --arm --chassis \
  --supported --mapping-confirmed --chassis-profile /absolute/path/chassis.yaml --profile weld --cycles 1
```

Chassis feed requires the complete four-wheel mapping; omitted-wheel profiles
are rejected. Closing, faults and disable remove torque; support must carry the
links. Disable acknowledgement failures are reported. USB/CAN commands remain
sequential, not firmware-atomic simultaneous multi-axis application.

Planning runs in a separate process; one control thread owns transports and the
GUI reads snapshots. Reports are written after motor cleanup to unique files in
`build/weld/`: accepted trajectory, mapping/model fingerprints, candidate metrics,
physical constraints, target/feedback/rate/age traces and drop counts. Open work
remains in [the arm registry](ARM_LAYER_TODOS.md) and
[chassis/communication list](CHASSIS_MODEL_COMMS_TODOS.md).

## Verification

`test_weld_constraints.py` covers offset-versus-scale identifiability, reproducible
configuration, output-shaft units, manual/physical limit separation, smooth ramps
and exact retiming. Native fake-motor and Qt tests cover stage ordering, identical
arm targets with either chassis switch state, per-axis limits, cancellation,
heartbeat loss and faults. The full-model test validates the measured-bound
standby, raised-tool approach, optimized weave and fake-shaft replay. No physical
speed, loaded holding, stopping or welding quality was certified.
The native transport test also removes both custom smoothing capability bits and
verifies that all trajectory writes use type 4, with no type-25 requests.
