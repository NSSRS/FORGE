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

The embedded viewer has current pose, standby/staging target and checked-path
views, with overview/side/wrist and explicit contact diagnostics. A blocked plan
has no playable route. Switching windows preserves planning and preview, but
cancels queued starts and pauses active motor motion. Space/Escape and lost UI
heartbeat cancel planning/motion. Returning never resumes motion automatically.
`--start-report PATH` seeds fake shafts from saved six-axis data and is rejected
in hardware mode; `--no-preview` disables rendering.

### Supported startup and recovery

Flow: fresh measured pose -> current-position hold verification -> checked
standby waypoints -> approach -> ready -> explicitly started weld -> endpoint hold.
Recovery tries a coordinated direct path, then a bounded joint-order graph
(maximum 64 poses / 192 directed edges). Every edge and the joined interpolant
are checked; there is no automatic escape from an already colliding start.

The operator's intended startup is pickup from a settled hanging arm on a secured
base. Load pickup, passive drop/swing clearance and shutdown remain unqualified.
Use matching reviewed mapping/standby files; do not command the saved CAD initial
pose as hardware home. See [operator procedure](ARM_RIG_TEST.md#weld-panel-with-embedded-model-preview).

| Preparation policy | Value / behavior |
| --- | --- |
| Speed | 5 deg/s default in both profiles; explicit ceiling <=10 deg/s |
| Acceleration | <=30 deg/s² and the sampled model envelope |
| Tracking | Pause above 3 deg reference error |
| Arrival | <=0.25 deg and speed <0.03 rad/s for 0.2 s |
| Feedback age | <80 ms |
| Motion control gap | >100 ms pauses without catching up |
| Paced-motion progress | Each lagging axis must advance >=0.1 deg in 10 s |
| Endpoint settling | Separate 10 s timer after reference reaches the endpoint |
| Total time target | `--prep-seconds 5`; includes initial hold and waypoint/approach settling |

Preparation streams a constant checked speed allowance and an acceleration-bounded
common path clock. For `q(s(t))`, acceleration is `q'' * s_dot² + q' * s_ddot`;
10% path-acceleration reserve supports clock slew. Pacing changes time, not path
geometry, motor gains, or target offsets. The nominal-timeout defect recorded in
[the recovery review](measurements/arm-recovery-review-2026-11-16.json) is fixed.

## Offline preparation route comparison

Add `--optimize-preparation` to `--simulate` or `--plan-only`. It compares the
requested standby with three raw-joint interpolated staging poses toward the
same full weld start. Each candidate checks recovery at 2 mm, approach at the
normal clearance, fixed travel bounds and sampled reduced-inertia acceleration.
Exact cubic extrema set timing with a 2% margin and 10% pacing reserve.

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

That case improves from 20.9272 s nominal to 4.5526 s; ideal fake-shaft arrival is
4.62 s including stationary checks. It changes the intermediate pose. The CLI
and controller reject experimental plans for physical owners. A synthetic lag
plant takes 14.44 s; ideal replay does not qualify five-second physical execution.
`preparation_search` retains the original standby, candidate failures and timing.
The preview displays the selected staging pose and minimum stationary intervals.

### Post-preparation holding evidence

The reported symptom is that most/all joints reach the prep target and then sag
before welding begins. The normal completion path explicitly latches the final
mode-2 target and existing speed allowance. Native refresh and protection remain
active. Pause/focus loss at an already settled endpoint preserve that fixed hold;
pause during unfinished travel captures fresh measured positions. Disable, fault
and close still remove torque. No gain, current or torque-feedforward writes exist.

The panel names the current stage goal and reports post-arrival `HOLD DRIFT` above
the unchanged arrival tolerance. Cached observations continue in ready/complete;
arrival, drift and recovery events record six-axis errors. The live JSONL journal
includes target, allowance, endpoint-hold state, cached enabled/status/torque and
feedback age. Observation does not poll registers or renew a motion lease.

This implements a fixed-reference hold, not a measured increase in motor stiffness.
**HOLD-01** owns per-axis drive-loop tuning and loaded acceptance on November 17;
see [the acceptance procedure](ARM_RIG_TEST.md#post-preparation-hold-acceptance).

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
- Preparation/debug additionally use the selected <=10 deg/s commissioning ceiling.
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

## Operator hardware entry points

Follow [ARM_RIG_TEST.md](ARM_RIG_TEST.md#weld-panel-with-embedded-model-preview)
with the matching reviewed mapping/standby and supported fixture. Explicit
`--execute --arm --supported` selects the real arm; chassis also requires explicit
`--chassis --mapping-confirmed` and a complete four-wheel profile. Close competing
owners before starting a session. Never use `--optimize-preparation` for hardware.

## Verification

Run the [weld suites](../tests/README.md#supported-weld-motion-controller). They cover
planning, native firmware with fake CAN motors, execution, holds, cached evidence,
Qt focus/cancellation and actual model rendering. Fake motors do not qualify
physical stiffness, load pickup, stopping, thermal behavior or weld quality.
