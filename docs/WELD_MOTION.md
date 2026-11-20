# Weld planning and execution

`controls/controller/` owns calibration, closed-chain kinematics, constraints,
retiming and search. `controls/hardware/` owns execution and the single transport
adapters; `controls/simulation/` is its sibling. Rendering lives in the separate
`simulation` package. See the [source map](SOURCE_LAYOUT.md),
[hardware API guide](../src/controls/controls/hardware/README.md), and
[current handoff](ARM_SESSION_HANDOFF.md).

## Run without hand-calculating timing

From the repository root, with the simulation/calibration environment installed:

```bash
.venv/bin/python scripts/control_weld.py --plan-only --profile weld --cycles 1
.venv/bin/python scripts/control_weld.py --simulate --profile weld --cycles 1
```

The ROS 2 package also exposes `ros2 run controls control_weld` after building and
sourcing the workspace. This entry point runs the same controller; it does not
establish a ROS lifecycle or a second transport owner. Production ROS lifecycle
qualification remains under ROS-02.

| Mode | Behavior |
| --- | --- |
| `--plan-only` | Solve and validate; no motor owner |
| `--simulate` | Same execution state machine with fake motors |
| **Play preview** | Kinematic playback of the accepted path; no motor commands |
| `--execute` | Existing real Damiao/ENCOS owners; explicit participation/fixture arguments |

Selection/planning do not enable an unpowered arm. Calculate, inspect, then choose
**Start recovery**. **Start welding motion** is a separate explicit action after
arrival at weld start. The application never energizes a welder.

To simulate forward travel during the weave, launch with both `--arm --chassis`
(also the simulation default when neither flag is given). Select **Arm output**
and **Chassis feed** before calculating the path. In **Current pose**, choose
**Top** to watch the robot feed forward along the amber seam while the cyan wire
tip follows the arm. Feed begins on **Start welding** and stops on **Pause / Hold**.
The distance and speed are estimates from simulated wheel rotation on a flat floor.
**Checked path** shows the planned travel during the weld segment.
Drag the viewer to orbit, right-drag to pan, and scroll to zoom. In **Top**,
dragging pans while keeping the camera overhead. **Reset view** restores the
selected camera. The panel scales to the display and stacks the viewer above
the dashboard when the window is narrow.

The embedded viewer also has standby/staging, overview, side, wrist and contact
views. A blocked plan has no playable route. Switching windows preserves
planning and preview, but cancels queued starts and pauses active motor motion.
Space/Escape and lost UI heartbeat cancel planning/motion. Returning never
resumes motion automatically.
`--start-report PATH` seeds fake shafts from saved six-axis data and is rejected
in hardware mode; `--no-preview` disables rendering.

`--a1-encoder-turns {-1,0,1}` explicitly selects a verified A1 encoder restart
branch relative to the saved mapping. It translates the session's A1 intercept,
raw bounds and inverse-mapped targets together, preserving model poses and raw
bus feedback. It never wraps samples automatically or rebases A4's cable limits.
See the [branch selection procedure](ARM_RIG_TEST.md#weld-panel-with-embedded-model-preview).

### Supported startup and recovery

Flow: fresh measured pose -> current-position hold verification -> optional
A3 endpoint retreat -> checked standby waypoints -> approach -> ready -> explicit
continuous weld -> Pause / Hold. Single-pass mode completes at a fixed endpoint hold.
Recovery tries a coordinated direct path, then a bounded joint-order graph
(maximum 64 poses / 192 directed edges). Every edge and the joined interpolant
are checked. Other colliding starts remain blocked; the only endpoint exception
is the reviewed A3 policy below.

`controller/endpoint_recovery.py` allows A3 startup up to raw 151.64 deg
(151.39 + 0.25 deg), then builds an inward-only path to 149.39 deg at <=1 deg/s
and <=5 deg/s². The original stop meshes must show bounded shallow contact that
separates along the retreat; all other pose/cable/clearance guards remain strict.
Fresh feedback is rechecked before preload and after stationary hold verification;
other axes retain their fixed pose. Normal preparation starts only after a
stationary, in-range endpoint. Pause requires a fresh checked plan. Its time is
reported separately from normal preparation. This is software/model evidence,
not physical retreat qualification.

Out-of-range measured poses remain visible with a warning for inspection. This
display never wraps/clamps readings or authorizes motion; the planning status
states whether a checked retreat or normal path is available.

The operator's intended startup is pickup from a settled hanging arm on a secured
base. Load pickup, passive drop/swing clearance and shutdown remain unqualified.
Use matching reviewed mapping/standby files; do not command the saved CAD initial
pose as hardware home. See [operator procedure](ARM_RIG_TEST.md#weld-panel-with-embedded-model-preview).

| Preparation policy | Value / behavior |
| --- | --- |
| Speed | 5 deg/s default; explicit request <=100 deg/s, bounded by the existing motor guards (currently 68.7549 deg/s) |
| Acceleration | <=30 deg/s² and the sampled model envelope |
| Tracking | Pause above 3 deg reference error |
| Arrival | <=0.25 deg and speed <0.03 rad/s for 0.2 s |
| Feedback age | <80 ms |
| Motion control gap | >100 ms pauses without catching up |
| Paced-motion progress | Each lagging axis must advance >=0.1 deg in 10 s |
| Endpoint settling | Separate 10 s timer after reference reaches the endpoint |
| Total time target | `--prep-seconds 5`; includes initial hold and waypoint/approach settling |

The panel now defaults to `--tracker pid`. Preparation runs the planned clock at
100%; a host position tracker corrects unequal shaft lag instead of routinely
slowing every joint. Native mode 2 still takes a position and positive speed
ceiling. The host command is the reference plus velocity lead and PID correction:
`q_target = q_ref + tau * v_ref + Kp * error + I + Kd * (v_ref - filtered_v)`.
This is position pre-emphasis, not a signed velocity/torque feedforward input to
the drive. There are no drive gain, mode, current or firmware changes.

The initial per-axis lag estimates come from the Nov 17 recorded approaches;
they are provisional. During preparation, sufficient same-direction movement
updates a bounded local lag estimate using the previous dispatched target and
cached feedback. Welding freezes that estimate across reversals. PID includes
filtered velocity feedback, conditional integration with a 0.5-degree integral
bound, a 12-degree virtual position-correction bound, and command speed and
acceleration shaping within the existing stage limits. Measured error is still
checked against the **unmodified reference**, with the same 3-degree trip.

During welding, host PID/velocity lead is applied only to joints whose planned
peak-to-peak travel is at least 0.3 deg. Smaller axes retain their nominal path
targets and have no host integral offset. This matches the provisional lag
model's identification floor (position error above 0.15 deg); it does not change
the path, drive gains or arrival/tracking tolerances. In the Nov 17 weld, lead
on the tiny A2–A5 moves unnecessarily reduced the shared clearance correction
factor for A1/A6. Recorded-feedback replay with the small-axis correction masked
retains full A1/A6 correction through the existing clearance checks. The later
tracker-v3 physical report also admits full correction in all 739 weld samples
and measures A1/A6 at approximately 1.731 Hz. Measured travel still exceeds the
plan, so amplitude and phase acceptance remain open. The admitted clearance
correction factor and corrected axes are recorded in each trajectory sample.
The [Nov 19 loaded review](measurements/arm-gun-load-review-2026-11-19.json)
also matches ~1.73 Hz but measures A1/A6 travel of 3.869/1.180 deg versus
2.260/0.427 deg planned. Frequency agreement does not establish amplitude
acceptance. Earlier sources are in the [Nov 17 archive](measurements/weld/2026-11-17/README.md).

Correction fades before finite endpoints. During preparation, both virtual
targets and shaped commands stay inside the exact position extrema of the current
finite leg. Mandatory recovery stops get separate bounds, so lead cannot enter
the next leg early. Command braking reserves this tick's travel plus `v²/(2a)`
stopping distance before an extremum; it does not wait for the reference clock
to finish. This fixes the recorded Nov 17 A6 command crossing below its approach
endpoint while decelerating. Speed/acceleration limits and arrival/tracking
tolerances are unchanged. These command bounds do not prove physical stopping.
Geometry limits virtual targets before
shaping; constrained correction gain returns gradually, rather than repeatedly
jumping to full gain. Approach/weld correction requests prefer 1.8 mm clearance
to leave shaping headroom; this does not change the nominal/measured 0.5 mm check.
Recovery retains 2 mm. Shaped dispatched targets are independently checked,
alongside measured poses and native raw/cable bounds; these sampled checks do
not certify the continuous physical swept volume between feedback samples. A
correction that cannot satisfy those checks pauses; it never bypasses clearance.
Arrival also waits for the tracker command to settle at the exact unshifted
endpoint before latching the native fixed hold. No integrator continues moving
the target while ready, complete or paused.

Online geometry checks use kinematics and collision detection without a redundant
dynamics solve. Nearby D3 closure updates use the analytic site Jacobian, retaining
the bounded least-squares fallback and existing assembly-branch continuation.
Full-forward contact equivalence and closure checks cover this optimization.
The [offline PID comparison](measurements/arm-pid-tracking-review-2026-11-17.json)
records the synthetic response, exact source fingerprints and remaining physical
qualification limits.

The table's **Measured Hz** is encoder-derived repeating-pattern frequency per
axis, separate from planned weave Hz and host command-update rate. It uses only
cached feedback, at most eight seconds of samples, and requires sufficient
motion and at least three repeatable periods. Alternating cycles may report a
two-cycle period. Low motion, insufficient evidence and stale data produce no
Hz value. Measured peak-to-peak travel is shown beside Hz because matching
frequency alone does not establish amplitude or phase tracking.
The live cards label a half-weave-frequency pattern as **Two-cycle response**,
and show measured travel alongside planned travel. A two-cycle pattern is not
50% travel or a 50% speed setting; the estimator is not forced to the planned Hz.

`--tracker paced` retains the previous controller for explicit comparison.
It streams a constant checked speed allowance and an acceleration-bounded
common path clock. For `q(s(t))`, acceleration is `q'' * s_dot² + q' * s_ddot`;
10% path-acceleration reserve supports clock slew. Each step uses signed cubic
velocity/acceleration bounds over the traversed interval, including internal
extrema and knot boundaries. This allows braking to cancel path acceleration;
the former global absolute bound could slow the clock too weakly and let A5
exceed the 3-degree tracking trip on fast preparation. The clock remains monotone,
with scale in [0, 1], bounded acceleration and unchanged tracking/arrival limits.
Soft slowing starts at 1 degree of lag and requests zero clock speed at 2 degrees;
acceleration-limited braking can temporarily exceed that soft threshold. The
separate 3-degree tracking trip remains. Progress detection still starts at
0.5 degree, independently of the pacing thresholds.
Pacing changes time, not path
geometry, motor gains, or target offsets. The nominal-timeout defect recorded in
[the recovery review](measurements/arm-recovery-review-2026-11-16.json) is fixed.
The [Nov 17 review](measurements/arm-preparation-review-2026-11-17.json) records
the two A5 tracking pauses and separate failed shutdown. Lagging fake shafts
reproduce the pacing defect; the correction does not establish physical timing,
clearance under tracking error, or loaded shutdown.

## Checked preparation route comparison

Add `--optimize-preparation` to `--simulate`, `--plan-only`, or explicit hardware
`--execute --arm --supported`. It compares the
requested standby with the current pose and three raw-joint interpolated staging
poses toward the same full weld start. Each candidate checks recovery at 2 mm, approach at the
normal clearance, fixed travel bounds and sampled reduced-inertia acceleration.
Exact cubic extrema set timing with a 2% margin and 10% pacing reserve.
Independent transfer timing retains the authored two-second final lowering
floor; it must not compress that short leg merely because its derivative limits
allow it. The explicit 2.3-second loaded-trial minimum still takes precedence.

The default `--preparation-planner direct-first` checks the current-pose route
first. For an explicit raised approach, it skips a staged candidate only when
ideal recovery plus its known approach spline timed at the full policy ceilings
already exceeds the best checked route. The bound includes the required stops,
lowering minimum and settling; it does not
claim a global optimum. `--preparation-planner staged` checks every candidate
for comparison. A failed direct route retains the staged search.

Static clearance checks include every cubic knot and bound each sampling interval
to at most 0.1 degree of raw travel on every axis, using exact interval derivative
extrema. Slow or stationary paths need fewer repeated geometry queries. This is
sampled clearance checking, not continuous collision detection. Private planner
models omit only chassis-only contact pairs while preserving all original pairs
that touch the arm. The runtime clearance thresholds and motion guards remain.

The persistent planner process reuses a private copy of the initialized scene
and a start-independent weave solution keyed by model contents, calibration,
settings and effective motor constraints. Each start still gets fresh recovery,
approach and arrival checks. Final scene fingerprint checks reject changes made
during calculation. Reports separate `planning_timing_s` from preparation time.
See the [seeded benchmark and reproduction commands](PLANNER_BENCHMARK.md).

```bash
.venv/bin/python scripts/control_weld.py --simulate --arm --profile weld \
  --mapping PATH_TO_REVIEWED_MAPPING --initial-pose PATH_TO_MATCHING_STANDBY \
  --start-report PATH_TO_SAVED_START --max-speed-deg-s 10 \
  --prep-seconds 5 --optimize-preparation --cycles 1
```

These file arguments are local operator artifacts, not files shipped by Git.
The tracked full-model regression reconstructs the Nov 16 case from dated evidence:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen \
  .venv/bin/python -m unittest discover -s tests -p test_weld_preparation_search.py -v
```

The earlier comparison for that case recorded 20.9272 s nominal versus 4.5526 s,
with 4.62 s ideal fake-shaft arrival including stationary checks. It changed the
intermediate pose. Hardware
requires explicit `--optimize-preparation`; the controller otherwise rejects
staging-search plans. Historical plans marked `offline_only` remain rejected
even with the option. The panel identifies optimized staging and reports the
actual nominal time against the goal; enabling it does not guarantee five seconds
from another start or at the default 5 deg/s. A synthetic lag
plant takes 14.44 s; ideal replay does not qualify five-second physical execution.
`preparation_search` retains the original standby, candidate failures and timing.
The preview displays the selected staging pose and minimum stationary intervals.

### Raised approach for unequal joint response

`--approach-lift-mm 30` requires a checked waypoint 30 mm above the weld tip target
and a lowering path to the unchanged weld-start pose. Zero retains automatic
route selection; explicit heights are bounded to 150 mm. The staging search uses
the same requirement for every candidate and cannot fall back to a direct blend.
The full path is still checked and retimed against existing limits. This is a
route alternative, not a change to the 0.5 mm approach clearance or 2 mm final
wire-tip gap, and not a proof for every possible tracking error.

`--lowering-seconds 2.3` sets a minimum duration for the final lowering move;
it requires an explicit `--approach-lift-mm`. Its cost is included when choosing
the staging route, and the stretching is applied after selection, so earlier movement timings and stationary
dwell remain unchanged. Only the time after the last stopped boundary is
stretched, preserving every geometric cubic and the final target. An already
slower lowering move is retained. The plan records `lowering_timing` and updates
its total preparation estimate; the requested preparation goal does not override this
minimum or measured settling. The default is zero additional minimum; optimized
raised routes still retain their authored two-second lowering floor.

The Nov 17 direct route reaches the modeled floor guard with unequal synthetic
joint lag. The raised route is exercised against those starts with the same
lagging fake shafts and unchanged runtime guards. Manual jogging differs from
this coordinated path: at 10 deg/s it permits up to 10 degrees of forward target
lead; legacy paced preparation slows at 1 degree error, while both trackers
retain the 3-degree tracking trip.
Normal manual operation therefore does not establish coordinated path timing.
The earlier disabled-start raised plan was 7.55 s nominal; preparation timing
depends on the starting pose and route.

### Post-preparation holding evidence

The reported symptom is that most/all joints reach the prep target and then sag
before welding begins. The normal completion path explicitly latches the final
mode-2 target and existing speed allowance. Native refresh and protection remain
active. At `ready`, pause/focus loss and a lost UI heartbeat preserve the fixed
hold, checked plan and explicit Start welding motion action. A fresh start
rechecks stationary feedback, the unchanged arrival tolerance and approach
clearance. No recalculation is required just for switching windows. At a settled
endpoint, pause preserves that fixed hold;
pause during unfinished travel captures fresh measured positions once. Repeated
pause/focus/recalculate actions preserve that first hold target and the last
checked speed allowance. Disable, fault
and close still remove torque. No gain, current or torque-feedforward writes exist.

The panel names the current stage goal and reports post-arrival `HOLD DRIFT` above
the unchanged arrival tolerance. Cached observations continue in ready/complete;
arrival, drift and recovery events record six-axis errors. The live JSONL journal
includes the selected arm profile, target, allowance, endpoint-hold state,
cached motor/MOS temperature, enabled/status/torque and
feedback age. Observation does not poll registers or renew a motion lease.

This implements a fixed-reference hold, not a measured increase in motor stiffness.
**HOLD-01** owns loaded acceptance and evidence-based drive tuning if needed;
see [the acceptance procedure](ARM_RIG_TEST.md#post-preparation-hold-acceptance).

### Execution timing and speed allowance

A moving tick reads and validates all six shafts once, computes the checked next
target, then sends one six-axis command batch. It no longer refreshes the previous
target and repeats all feedback reads before sending the new target. Samples are
consumed by one dispatch and aged again before sending; idle holding still polls
and refreshes every joint. USB dispatch is sequential, not firmware-atomic.

Mode-2 velocity is a limit. Both preparation and welding retain the checked stage
allowance instead of reducing it to reference speed plus position error. The old
welding expression could shrink to 0.1 deg/s at completion and carry that cap into
holding. Preparation respects the explicit preparation request; welding uses its
own checked stage limits. Positions still follow the checked path.

An earlier pre-sweep operator run reached ready in 18.92 s against a 7.80 s
nominal plan. All six endpoint targets remained fixed/enabled for the following
1.49 s until close; no weld was started. Two synthetic lag replays of that same
raised path improve from 22.76/17.04 s to 18.12/14.12 s with the new pacing.
These are separate hardware and model observations, not a five-second result.
The subsequent operator journal records 16.14 s to ready against 7.60 s nominal,
then 12.76 s holding all six fixed targets within 0.177 deg. Later disturbances
recovered to those same targets; the operator confirmed pushing the arm. This
different starting pose does not provide a controlled speed comparison, and
post-weld/loaded holding acceptance remains open.
Evidence is in [the Nov 17 review](measurements/arm-preparation-review-2026-11-17.json).

### Offline CMA-ES weight and branch search

```bash
.venv/bin/python -m pip install -r src/controls/search-requirements.txt
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/optimize_weld.py \
  --output-dir build/weld/my-search --seeds 11 23 47 \
  --generations 12 --population 12 --workers 6
.venv/bin/python scripts/control_weld.py --simulate --profile weld --cycles 1 \
  --solution build/weld/my-search/solution.json
```

Use a new output directory. `--warm-start PATH` validates a previous candidate
before refinement. Search uses bounded log mobility weights and yaw/wrist seeds;
larger mobility permits more joint movement. It penalizes required slowdown,
normalized speed/acceleration and reversals at the unchanged requested stroke.
It cannot improve its score by shrinking amplitude, dropping dwell or loosening
clearance. Diverse finalists receive dense task/clearance checks and complete-route
acceptance. Only an improving accepted candidate produces a solution.

`--solution` checks model, mapping, constraint and amplitude identity, then solves
and checks the entire route again. Historical search reached 1.815288 Hz /
10.891728 mm/s versus 1.730607 Hz for default candidate comparison, under provisional
CAD acceleration bounds. This is not a global optimum or a measured-load result.
Saved artifacts predating A4/A5 mapping changes must be regenerated.

### Render the actual optimized plan

```bash
.venv/bin/python scripts/control_weld.py --plan-only --profile weld --cycles 12 \
  --solution build/weld/my-search/solution.json --output build/weld/animation-plan.json
.venv/bin/python scripts/render_weld.py --plan build/weld/animation-plan.json \
  --output-dir build/weld/animation --approach-speedup 1
```

Pass `--mapping PATH` to both commands for a non-default mapping. `--preview-only`
exports framing keyframes. The full renderer writes MP4, a weld-only GIF and
provenance. It checks identity, stage continuity and derivative limits. Exported
video has presentation holds and configurable approach speedup; use plan/controller
timing for performance claims. Kinematic animation is not loaded-dynamics evidence.

### Where smoothing lives

Quintic task ramps and joint splines belong to the offline planner. Preparation
clock pacing belongs to the execution controller. Welding uses native Damiao
position/velocity mode 2 through USB type 4. The separate manual type-25 smoothing
experiment is not used here and did not resolve reported manual-travel jitter.

## Measurements dictate controller model coordinates

Mapping is `q_model_deg = model_per_raw * raw_output_shaft_deg + intercept_deg`.
Feedback is already geared; do not apply internal gear ratios again. Unit-magnitude
gains preserve the distinction between uncertain anchors and real transmission
ratios. Empirical scale fitting needs at least three independently matched poses.

The dated [reference capture](measurements/arm-reference-2026-11-12.capture.json),
[return observations](measurements/arm-return-checks-2026-11-13.json), and
[A5 candidate](measurements/arm-a5-alignment-2026-11-16.json) retain provenance.
The default and compact standby files are model candidates, not newly measured
six-axis homes. The [handoff](ARM_SESSION_HANDOFF.md#calibration-and-evidence)
records A3 endpoint correction, A4 cable bounds and the separate A5 candidate.
Mapping/standby fingerprints prevent mixing these coordinate systems.

## Better joint solution and repeatable timing

The solver compares fixed-D1 and redundant free-D1 branches with equal,
physical-capability and executable-speed weights. It constrains tool position
and wire direction while allowing roll about the wire. `--lock-base-yaw` restricts
the solve to the fixed-D1 experiment. Spatial continuation retraces the same
branch; periodic cubic interpolation avoids cycle-to-cycle null-space drift.

D3 closure preserves the 150 mm rod connection spacing. Shape 1 retains +/-10 mm
stroke and dwell; quintic ramps replace velocity steps. Exact polynomial extrema
set a common time factor, reducing arm frequency and nominal chassis feed together.
Initial and approach are retimed separately. A raised-tool path is checked when
a direct transition clips the floor. Model validation bounds sample travel to
0.2 deg and retains 0.5 mm arm/environment clearance and 0.1 mm adjacent-stop checks.

`weld_start_check` records target/tip positions, residual and floor standoff. The
authored 2 mm `WIRE_GAP` is independent of the reported post-prep joint sag.

## Damiao physical and deployment constraints

Manufacturer sources and derating inputs are pinned in
[`damiao_physics.json`](../src/controls/config/damiao_physics.json); drive-register
readbacks and their limits are described in [DAMIAO_TORQUE_LIMITS.md](DAMIAO_TORQUE_LIMITS.md).

- Physical design speed: 50% rated continuous output, bounded by protocol range.
- Weld command ceiling: minimum of physical design speed, explicit axis policy
  and 80% of the 1.5 rad/s measured-speed trip; currently 68.7549 deg/s on A1-A6.
  Null policy entries mean no additional cap. These are host guards, not drive gains.
- Preparation/debug additionally use the selected <=100 deg/s request ceiling.
  The operator raised this request range on Nov 17; it does not raise the
  68.7549 deg/s motor command guard or 1.5 rad/s measured-speed trip.
- Acceleration: D3-reduced inertia and gravity/bias use a derated torque reserve;
  preparation is also capped at 30 deg/s². The model box is provisional.

The operator estimates 6.5 kg moving load and 0.30 m COM extension at 24 V; CAD
A1-descendant mass is about 0.898 kg. Distribution, orientation, inertia and payload
remain unmeasured. Current mode 2 does not command model torques. LOAD-01 and
FEEDBACK-01 own later load compensation and sensor/ESO work.

## Independent debug outputs, no compensation

Arm/chassis switches only gate output. Planning assumes nominal counterpart motion
even when that output is OFF; no arm compensation is added for a disabled chassis.
Changing selection invalidates the plan. An existing powered hold is retained;
**Disable arm torque** is separate. Faults latch and never auto-reset/re-enable.
**Reset motor fault (R)** performs explicit recovery through the connected arm
owner. It verifies disabled/stationary feedback and saves the original fault
before clearing the latch. Retained startup faults keep the panel open for this
action. Failed recovery stays faulted; success leaves all motors disabled and
requires fresh selection, planning and recovery. Queued actions during reset are
discarded, and close waits for that same owner to finish recovery and cleanup.

The panel defaults to continuous welding. `--cycles` sets the finite checked
plan/preview block, which repeats until Pause / Hold; `--single-pass` restores
bounded execution of that block. Continuous execution accepts only the checked
periodic cubic weld path, whose position, velocity and acceleration close at the
boundary. Sampling wraps the path phase without a dwell, settling check or new
enable between blocks. Selected chassis feed remains continuous. Absolute elapsed
time and the repeat index remain in the bounded trace and live journal. Pause
captures a fixed measured arm hold and stops feed; fault, focus, heartbeat,
freshness, clearance, tracking and control-gap behavior remains unchanged.

## Torque profiles and offline observation

The explicit gun-load profile selects A2/A3 30 Nm and A4-A6 10 Nm reactive
trips with independently verified TMAX 40/12.5. A1 remains 20/54 Nm. Standard
profiles, planning torque budgets and drive gains remain unchanged. Matching
host/MC02 capabilities and disabled range readback are required; firmware setup
is separate from normal panel launch. See [deployment and current status](ARM_RIG_TEST.md#expanded-gun-load-profile-a2a3-30-nm-a4-a6-10-nm).

`hardware/hold_analysis.py` / `scripts/analyze_weld_hold.py` summarize the cached
journal offline: separate preparation attempts/stages, time to Ready, endpoint
retreat, speed, torque, motor/MOS temperature, fixed-target error and data gaps.
An attempt reaching Ready does not mean its later hold/weld passed; inspect its
segments and fault history. The GUI-01 dashboard presents the same cached
telemetry; display-rate trends and peaks complement the full-rate journal.

## Telemetry dashboard

The weld window presents the current stage and next action above the controls.
Output selection and active holding are distinct. **Calculate path**, **Start
recovery** and **Start welding** remain separate explicit actions. The always
visible Pause / Hold, Disable arm torque and Reset motor fault controls retain
their existing behavior and keyboard shortcuts.

**Load & temperature** shows A1-A6 signed reported torque in Nm, absolute torque
as a percentage of the active reactive trip, motor/MOS temperatures in °C and
sample age. A stale sample (80 ms or more), unavailable field, reset or fault
never displays a fabricated zero or old enabled state. No torque or temperature
is synthesized for fake shafts. The manual arm window has the same telemetry
tab alongside its existing joint table.

Each card has 30-second display-rate traces: torque uses the axis color, motor
temperature is orange and MOS temperature purple. Fresh-only torque peaks are
shown for the stage and recovery run; hover a card for both temperature peaks.
A new recovery resets run peaks; a stage transition resets stage peaks.
The manual panel starts a run when motion/holding becomes active. Histories
are bounded and independent of motor authorization. The full JSONL remains
the source for complete owner-rate analysis.

**Targets & response** keeps raw targets, measured angles, remaining distance,
frequency and travel together. **Diagnostics & evidence** expands the full
controller message, tracking details, original first-fault evidence, profile
trip/rating/TMAX distinctions and journal path. Evidence-write failures remain
visible even when diagnostics are collapsed. The first fault remains readable
after an explicit reset.

Calculation time is separate from measured preparation time, its nominal plan
and requested goal. Path progress describes the checked reference; 100% is not
an arrival confirmation. Arrival still requires the existing stationary/error
checks. A paused run freezes the preparation clock, while Ready retains its
completed preparation duration. Continuous welding shows block progress and
run elapsed time separately.

Pause events record `cause`: `operator_input`, `focus_loss`, `ui_heartbeat`,
`motion_guard`, or the corresponding selection/recalculation/disable action.
Ready focus/heartbeat pauses retain the existing checked plan and fixed hold;
interrupted travel requires recalculation. This is presentation and evidence
work, not drive tuning, new register polling or physical holding qualification.

## Operator hardware entry points

Follow [ARM_RIG_TEST.md](ARM_RIG_TEST.md#weld-panel-with-embedded-model-preview)
with the matching reviewed mapping/standby and supported fixture. Explicit
`--execute --arm --supported` selects the real arm; chassis also requires explicit
`--chassis --mapping-confirmed` and a complete four-wheel profile. Close competing
owners before starting a session. Add `--optimize-preparation` to select checked
staging on hardware; `--max-speed-deg-s 10` explicitly selects the speed used by
the recorded five-second comparison. Fresh feedback determines the route, and
the operator reviews the checked path before starting recovery.

## Verification

Run the [weld suites](../tests/README.md#supported-weld-motion-controller). They cover
planning, native firmware with fake CAN motors, execution, holds, cached evidence,
Qt focus/cancellation and actual model rendering. Fake motors do not qualify
physical stiffness, load pickup, stopping, thermal behavior or weld quality.
