# Arm cross-layer TODO registry

Updated 2026-11-20, based on branch `nathan`. This registry contains **open work only**.
Read [current status](ARM_SESSION_HANDOFF.md) and [the base contract](ARM_BASE_LAYER.md)
first. Stable IDs link package responsibilities; update the owning entry when
work changes, and retire completed tasks instead of appending session history.

The six-axis fixture is connected; disabled identity/feedback and reference
capture are recorded, alongside operator-observed manual motion. Completed implementation
includes cached base identity/snapshots/health (former BASE-01..03), subset
checks, manual travel, guarded software recovery, disabled control-register
readback, fault reporting and the compact keyboard UI. These are documented
in the guides; full hardware qualification below remains separate.

## Immediate priorities

**GUI-01 is implemented locally; operator review remains.** Hardware acceptance
remains HOLD-01 (the untouched A5 holding fault), with CAL-01/SUPPORT-01 as
prerequisites. Then finish WEAVE-01/SPEED-01. LOAD-01 and FEEDBACK-01 remain
deferred until the first qualified arm welding run. Fixed-reference holding
software is implemented; HOLD-01's remaining work is physical diagnosis and
acceptance, not reimplementation of that fix.

### GUI-01 — operator review of the telemetry dashboard

Owner: `src/controls/controls/hardware/`, `src/teleop/teleop/` shared arm views.

Implementation and cached-data contracts are documented in
[the interface guide](WELD_MOTION.md#telemetry-dashboard). Offline regression
and screen-layout verification are complete; operator acceptance remains below.

- [ ] Review the offline-checked layouts on the actual NUC display and accept
  readability during normal, stale, planning, recovery, Ready, welding, pause
  and fault states. Confirm the telemetry and diagnostic views are useful for
  the operator's next supported session. This review does not qualify holding,
  temperatures, continuous load or drive tuning.

### SUPPORT-01 — base, gravity support and physical stopping

Owner: operator/mechanical integration, `src/bringup/`.

- [ ] Document/qualify the existing base with A1 rotation and cable clearance; establish
  supported rest poses and support for every attached link during torque loss.
- [ ] Define brakes/counterbalance/support, emergency stop, power loss, drive
  timeout and recovery. Distinguish controlled stopping from torque disable.
- [ ] Measure stop/hold behavior before unsupported loaded operation. The current
  default jogger disables torque at release; explicit group/weld modes hold
  position. Qualify both behaviors. MC02 disable is not a brake; the production
  coordinated ROS lifecycle remains under ROS-02.
- [ ] Resolve the 2026-11-12 capture session's stationary-disable/mode-restoration
  failure: A2 still reported -0.168498 rad/s after disabling. Preserve the saved
  [shutdown evidence](measurements/arm-reference-2026-11-12.session-summary.json)
  and verify shutdown on a supported fixture before treating cleanup as qualified.
  Qualify supported shutdown after powered recovery, including the A5 movement
  retained in the [Nov 17 evidence](measurements/arm-preparation-review-2026-11-17.json).
  The Nov 19 final loaded-session disable and mode restoration passed; qualify
  repeatability, stationary load support and fault/power-loss behavior separately.

### CAL-01 — motor-position initialization and physical calibration

Owner: `src/robot_description/`, `src/motors/`, ROS coordinate adapter.

Bench acceptance uses the [joint reference table and pose procedure](ARM_POSE_TEST.md).
Record fixture measurements before promoting any calibration to production.
The initial eight requested observations are preserved in the
[dated reference record](../src/robot_description/config/arm_reference_positions_2026-11-12.json).
Validation and full coordinate mapping remain open.
The separate `record_arm_positions.py --capture-file` recorder defaults to initial references for
A1/A4/A5/A6 and both CW/CCW ends for A2/A3, preserving all holds and saving all
joint angles for context. The longer two-ends-plus-initial workflow for every
motor remains available through `--capture-mode bounds`.
These observations are not automatic homing or globally valid collision limits.
See the [capture procedure](ARM_POSE_TEST.md#current-procedure-initial-positions-and-a2a3-travel-ends).

- [ ] Record known physical-pose/raw-angle pairs for each DOF: sign, zero offset,
  any external ratio, wrap branch and units. Separate model zero from session zero.
  First resolve the reported A5 encoder/model mismatch with the independent
  [pose mirror](ARM_RIG_TEST.md#independent-encoder-to-model-mirror) and its
  recorded A5 CW/CCW cable ends and independent straight-pose candidate
  (model = -raw +101.514058 degrees). Confirm it away from straight and qualify
  the 2-degree cable backoff and fresh recovery path; capture alone does not
  qualify or install the mapping for motion.
- [ ] Qualify the fixed A4 CW/CCW 270-degree cable envelope from its preserved
  initial anchor, including reversal, power cycling and changed cable routing.
- [ ] Complete encoder-branch coverage and compare normal CAN feedback with
  documented `p_m`/`xout` where supported; do not assume retained multi-turn state.
  The stationary six-axis cycle and one controlled off-power return per joint
  are recorded in [Nov 13 observations](measurements/arm-return-checks-2026-11-13.json);
  manual return differences do not isolate encoder precision.
- [ ] Qualify the implemented welding startup from fresh measured coordinates,
  geometry-checked disabled preload, stationary hold verification and checked
  recovery waypoints to standby. Reconcile the Nov 16 recorded pose with the
  actual mount and wrist alignment; that historical start failed modeled wrist
  self-contact and gun/chassis checks. Keep the gun envelope in the model for
  both attached and absent-payload comparisons. No jump toward guessed zero, automatic homing or
  automatic return to the simulation pose; a home move needs a separate request
  and a validated path.
- [ ] Identify physical travel/cable/clearance limits under support, save a
  versioned calibration, and verify round trips at additional poses. The
  [model-guided tool](ARM_MODEL_CALIBRATION.md) now derives pose-dependent paths
  and records operator constraints; physical qualification remains open. Keep null
  calibration/false production flags until measured acceptance is complete.

### SPEED-01 — faster commanded movement

Owner: `src/controls/`, `src/motors/arm_position.py`, `src/teleop/`, firmware/profile commissioning.

- [ ] Qualify the Nov 17 bounded host PID/velocity-lead tracker in
  `src/controls/controls/controller/tracking.py`. Compare actual recovery time,
  error, measured Hz/travel and physical smoothness against `--tracker paced`.
  Include unequal load, reversal, saturation, model-limited correction and final
  fixed holding. Synthetic first-order response and model clearance checks do not
  identify the loaded plant or establish physical jitter improvement. The Nov 19
  loaded run matches A1/A6 weave frequency but exceeds planned travel
  (3.869/2.260 deg and 1.180/0.427 deg respectively). Qualify amplitude and phase
  without changing the operator-accepted tiny A2–A5 nominal motions; see the
  [latest physical review](measurements/arm-gun-load-review-2026-11-19.json).
- [ ] Qualify coordinated preparation after the timeout/reference-braking fixes
  and the explicit raised-approach option. A5 manual operation is reported normal;
  resolve unequal-axis tracking and gun/floor clearance under load without
  inferring a gain defect from the failed nominal path. Software replay does not
  qualify physical recovery.
  Meet the current five-second physical preparation goal by reducing unnecessary travel
  and qualifying tracking, not by widening trips or speeding up the renderer.
  See [current evidence](ARM_SESSION_HANDOFF.md#calibration-and-evidence).
- [ ] **Broader manual-jog investigation deferred:** investigate moving-only A1/A4/A5/A6 jitter; A2/A3 are smooth
  and earlier manual holding was reported steady. The Nov 19 untouched loaded
  A5 hold fault is separately tracked under HOLD-01. The 5 deg/s MCU smoothing
  trial did not fix it. Its saved report verifies the build and clean exit, but
  evicted all jittering-joint jog traces; it cannot clear command timing.
  Compare 5 versus 10 deg/s with short saved runs, raw command/feedback timing
  and independent motion observation. Distinguish angle-repeating cogging/torque
  ripple or friction from drive-loop oscillation and time-repeating command gaps.
  Motor/gearbox differences are plausible, not a diagnosis: rotor speeds at
  5 deg/s output are 7.5 rpm (A1), 8.33 rpm (A4–A6), 33.33 rpm (A2/A3).
  Record actual gains/ACC/DEC/MAX_SPD while disabled; Deta was already 4 on all
  six, and MIT gain advice does not directly apply to the current mode-2 loop.
  Separately correlate A4's reported 2.4542 rad/s hold sample with encoder
  displacement before calling it physical overspeed or a reading error.
  Preserve trips and leases. See the [retained findings](ARM_SESSION_HANDOFF.md#current-status-and-next-session).
- [ ] Test the 5 deg/s baseline and opt-in 7.5/10 deg/s trials on the supported
  fixture. Compare A6 tracking, host sample timing and release travel with A4/A5.
  Record the accepted speed for each joint and direction.
- [ ] Review UI/adapter/profile/MCU caps and motor ACC/DEC/MAX_SPD/loop readback
  together. Preserve command/feedback deadlines and explicit fault recovery.
- [ ] Measure requested versus actual motion in both directions on each joint,
  including A6's earlier overspeed behavior, tracking error, peak speed,
  stopping distance, temperature and current/torque evidence. Qualify each
  increase before documenting it as an operating setting.

### BASE-04 — qualify bus timing

Owner: `src/damiao_firmware/`, `src/motors/arm_telemetry.py`, commissioning.
Record firmware/topology/bitrate, motor settings, command/feedback rates, sample
age distribution and maximum gaps under the intended five/six-motor workload.
Use CAN capture as well as cached counters; correlate retransmissions, FIFO
loss, bus-off and late ticks. Repeat the final split-bus layout. Investigate
A1's delayed register replies. Acceptance: explained losses, measured deadlines
and margin; a configured 500 Hz rate is not proof of achieved physical timing.
The [dual-family ROS telemetry](MOTOR_TELEMETRY.md) now exposes cached snapshots
and bus health. Measure real ROS command latency with these readers active;
fake-owner callback timing does not qualify physical USB/CAN timing. ENCOS
additionally needs independent CAN freshness verification because bridge PDO
observation timestamps can reflect retained frames.

### BASE-05 — typed errors and active configuration readback

Owner: `arm.c`, `wire.c`, `src/motors/arm.py`.
Expose stable rejection codes (state, route, stale feedback, limits, capability)
and actual active routes/bounds. Fault snapshots and disabled motor-register
readback already exist. Distinguish protocol ceilings, profile trips and measured
application limits. Preserve compatibility or negotiate a wire version; test
invalid/nonfinite requests, route changes and missing capabilities.

### BASE-06 — atomic multi-joint targets

Owner: firmware and Python base API.
Validate a complete batch before committing a generation/common application
time. Define partial/stale/replayed packet discard and mode support; an invalid
or incomplete batch must neither move a subset nor renew leases. Measure
cross-joint dispatch skew. Group enable alone is not synchronized trajectories.

### BASE-07 — physical fault-injection acceptance

Owner: firmware/bench commissioning, `src/bringup/`.
Record first-fault evidence and stop latency for host crash, USB loss, lost
commands/feedback, CAN errors, motor errors and interrupted tests. Verify
disable acknowledgement and explicit recovery without automatic re-enable.
CLI/keyboard software recovery is implemented and has been exercised on the
rig; this does not complete the full physical failure matrix. Coordinate
SUPPORT-01 before gravity-loaded tests.

### PROTECT-01 — current/protection verification

Owner: `src/motors/`, `src/encos_driver/`, commissioning profiles.
Follow [the protection checklist](MOTOR_PROTECTION_TODO.md) for Damiao and ENCOS.
A2–A6 disabled control-register readback is complete; still verify model/version
support, units/scaling, application limits, persistence and fault behavior.
Compare feedback with independent load/phase-current evidence. Reactive torque
trips and PSU DC current are not motor current clamps or measured shaft torque.
Qualify torque decoding and physical fault behavior with the deployed
A2/A3 30 Nm, A4-A6 10 Nm profile, then record representative holding/welding
torque and both temperatures. Range deployment/readback evidence belongs in the
[Nov 19 review](measurements/arm-gun-load-review-2026-11-19.json). No overload timer is added by operator request; continuous
operation above rated 12/3.5 Nm remains unqualified. The separate profile needs
expanded MC02 capability 0x800000 and retains both 80 C trips and all other
fault/lease behavior. See [the rig procedure](ARM_RIG_TEST.md#expanded-gun-load-profile-a2a3-30-nm-a4-a6-10-nm).

### HOLD-01 — post-preparation joint stiffness and fixed-pose acceptance

Owner: `src/controls/controls/hardware/`, `src/motors/`, operator.
Qualify fixed-target holding and stiffness across representative supported
poses, with and without the installed gun. The host already latches settled
targets and allowances, preserves them through idle/pause, and records cached
errors. Physical acceptance must explain the untouched A5 failure separately
from successful intervals; use the current handoff for dated evidence.

- [ ] Diagnose the Nov 19 **untouched A5 fixed-hold motion**: unchanged target,
  1.51648 rad/s speed trip, preceding approximately 6.2 Nm Ready load and
  motor-temperature rise 26 -> 46 C. Compare mechanical/load state and drive
  response with the later successful attempt at the same targets; do not infer
  external disturbance or weaken the speed guard. Preserve the two attempts
  separately in [the measurement review](measurements/arm-gun-load-review-2026-11-19.json).
- [ ] With the installed load supported, verify that target, mode, enable state
  and speed allowance remain unchanged across prep completion. Record six-axis
  error, velocity, reported torque, feedback age and pause/fault events.
  First compare the arm without its end effector against the same preparation
  and 15-second Ready hold with the operator-estimated 1.96 kg end effector.
  Match starts, checked routes and settings; retain timing, per-axis speed and
  torque feedback separately for both conditions. See the
  [matched-run procedure](ARM_RIG_TEST.md#matched-runs-with-and-without-the-end-effector).
- [ ] Verify at least 15 seconds of stationary endpoint holding within the
  existing 0.25-degree arrival tolerance, without visible drop, oscillation,
  unexpected disable or a target reset to a sagged measurement. Record peak and
  settled error for every axis; repeat across representative supported poses.
- [ ] If fixed-target sag persists, stiffen the native mode-2 position/velocity
  loops through measured per-axis tuning. First read and retain the existing
  supported KP_APR/KI_APR/KP_ASR/KI_ASR and relevant limits while disabled. Record
  hardware revision, original/new values, units, restoration and thermal/current
  response. Do not guess a shared gain multiplier or weaken reactive trips.
  The current host/firmware gain registers are read-only; any tuning interface
  needs explicit validation and a rollback path before use.
- [ ] Qualify fixed-endpoint pause/focus behavior, moving-trajectory pause,
  explicit disable and faults under the supported load. Never treat active hold
  as a mechanical brake. Keep SUPPORT-01 and shutdown failures open.
- [ ] Only after stable holding, measure complete prep-to-ready time against
  the current five-second goal. The latest successful loaded preparation took
  12.53 s; qualify the actual start, selected route and measured response.
  Keep calculation latency separate from physical movement time.

Procedure: [post-preparation hold acceptance](ARM_RIG_TEST.md#post-preparation-hold-acceptance).

### ROS-02 — production lifecycle and coordinated holding

Owner: ROS arm driver, base API and `src/bringup/launch/arm.launch.py`.

The keyboard now has opt-in selected-joint and complete-group mode-2 hold implementations; see
[its fixture procedure](ARM_RIG_TEST.md#optional-selected-joint-position-hold).
Physical hold drift, thermal behavior and fault/disable acceptance remain open.
The group path has an operator-run physical session; jitter, hold overspeed and
physical load/fault qualification remain open.
These keyboard paths do not complete the coordinated ROS lifecycle below.
Select the production mode explicitly; the ROS MIT path is not qualified by
mode-2 bench success. Separate configure/activate, stop, supported disable,
recover and re-arm. For holding, require fresh complete feedback, current-angle
preload and explicit authorization for all load-bearing joints. Separate
neutral/SE-release hold from drive disable while preserving the current jogger's
release-disable contract. Depends on BASE-06/SUPPORT-01. Acceptance requires
ROS tests and physical evidence for the selected mode; no automatic homing or
re-enable after a fault/reconnect.

### PLAN-01 — paths, trajectories and input acceptance

Owner: `src/controls/controls/controller/`, `src/simulation/`, `src/teleop/`.
The weld planner implements speed/acceleration bounds, sampled combined-pose/path
collision checks, D3 closure and common retiming. Qualify these against the
physical assembly; explicit jerk policy and atomic target timing remain open. Invalidate plans on stale
state or faults; do not replay old commands to hide host stalls. Depends on
CAL-01, ROS-02 and BASE-06. The existing D3-closed single-joint clearance study
is documented in [ARM_COLLISION_STUDY.md](ARM_COLLISION_STUDY.md); its sampled
intervals are not independently combinable hardware limits.

- [ ] Reduce the remaining joint-order recovery/staging planning delays and
  extend random-start coverage. Qualify the work-area starting region that meets
  the five-second preparation goal; broad starts still exceed it and one Nov 20
  case exceeded the planning deadline. Preserve checked paths and explicit
  failures. See the [planner benchmark](PLANNER_BENCHMARK.md).
- [ ] Add a visible encoder-drift check between **Calculate and check path**
  and **Start recovery**. Compare fresh feedback from all six joints with the
  encoder pose used to calculate the accepted path, while waiting and again
  immediately before starting recovery. If any joint exceeds the threshold,
  show the joint and angle change, invalidate the plan and require recalculation.
  Threshold to decide: **2 degrees**, with **5 degrees** as an alternative.
  Integrate with existing start-pose validation; do not silently loosen its guards.
- [ ] Qualify recovery from supported rest poses with the installed gun, cables
  and chassis. The bounded direct/joint-order recovery, pre-enable geometry gate,
  hold verification and waypoint settling are implemented; audit remaining
  collision exclusions and establish tracking/stopping margins on the fixture.
  The reviewed A3 upper-endpoint retreat is implemented with a 0.25 deg startup
  allowance, separating-contact checks and 1 deg/s / 5 deg/s² caps; its supported
  physical acceptance remains open. Other contact starts and unsuccessful route
  searches remain blocked; no general-purpose contact escape is implemented.
- [ ] Complete automatic startup from the operator's intended settled,
  gravity-hanging pose on a secured base, without hand-positioning each joint.
  Validate actual load pickup at the measured pose, clearance through the lift
  to compact standby, and passive drop/swing clearance after torque loss. The
  rate-limited fake-motor tests have no gravity or load model and do not complete
  this acceptance. Depends on SUPPORT-01 and independent mount/tool alignment.

Pocket physical acceptance still needs:

- [ ] Measured right-stick neutral and all four diagonal stop/resume behaviors;
  confirm the left vertical stick physically returns to neutral.
- [ ] Live selection isolation, neutral-before-SE priming, release/center/change
  stops, GUI STOP/close, disable and mode restoration with identity/log evidence.
- [ ] Transmitter-off, receiver-power, USB-loss and reconnection tests. Fresh CSV
  cannot identify hold-last RF failsafe; retain sample deadlines and fresh priming.

### WEAVE-01 — flat-ground coordinated weave experiment

Owner: `src/controls/controls/`, `src/simulation/`, operator.
The [implemented sequence](WELD_MOTION.md) is measured current pose -> derived
standby -> checked weld start -> explicit weld -> hold, with independent output
switches. Calibration, motor constraints, CMA-ES and the actual hardware path
exist; their animation/fake replay does not qualify physical execution.
The target remains 2 Hz / 12 mm/s. The best checked candidate reaches 1.815288 Hz
under provisional CAD acceleration bounds, with six equal 68.7549 deg/s weld
caps and 5 deg/s initial/approach. Temporary A1 limiting is withdrawn. No firmware
or reactive trip was raised. Chassis gaps are in [the chassis list](CHASSIS_MODEL_COMMS_TODOS.md).

- [ ] Validate the unit-gain sign/offset map and derived standby at independent
  physical poses; raw A3 149.39 deg is a calculated 2-degree backoff, not a newly
  measured initial anchor. Retain the original conflicting CAD pose as evidence.
- [ ] Run the complete supported arm sequence and save timing, feedback ages,
  dispatched targets, tracking, stops and attained frequency. Confirm arm-only,
  chassis-only and combined output gating without switch compensation. Keep
  physical load/hold acceptance separate from model feasibility.

- [ ] Confirm the flat butt joint and the 10° drag angle against the weld procedure.
  Keep the gun fixed in the DOF6 clamp. Verify wire extension and cable clearance
  on the rig. The fixed-mount simulation reaches the full width at 1–3 Hz with
  a measured peak edge overshoot of 0.32 mm. Agree on physical trial tolerances.
  Use the [simulation controller and tester guide](../src/controls/README.md).
  Record the accepted amplitude, feed and tracking tolerance for the intended seam.
  Physical pose teaching and A6 speed trials remain under CAL-01 and SPEED-01.

### LOAD-01 — weight compensation after the first arm welding run

Owner: `src/controls/controls/controller/`, `src/robot_description/`, operator.
Deferred by the operator until the arm's initial-pose -> weld-start -> welding
sequence has been run through under WEAVE-01. Keep this separate from chassis
participation: disabling one subsystem must not introduce arm compensation.

- [ ] Reconcile the operator estimate: A1 supply 24 V, moving load about 6.5 kg,
  COM extension about 0.30 m, and rough A2/A3 posture range 30-45 degrees.
  Confirm total versus added mass, angle convention, perpendicular COM radius
  and floor/wall orientation; measure each link and the gun/cable payload.
- [ ] Update link mass, COM and inertia in the model. The current CAD A1
  descendant mass is about 0.898 kg; a previously selected weld center had
  reduced A1 inertia about 0.0611 kg m^2. A 6.5 kg point mass at perpendicular
  radius 0.30 m contributes at least 0.585 kg m^2. Ground A1 is almost vertical;
  the worst 19.13 Nm gravity moment is not its ground-scene drive torque.
- [ ] Implement and qualify pose-dependent gravity/load compensation with
  explicit torque/current limits and supported hold/stop tests. Select a motor
  interface that actually accepts the required feedforward; native mode 2
  currently sends position/speed only.
- [ ] Recompute acceleration/torque margins and rerun CMA-ES with the measured
  load. Acceptance requires measured tracking and thermal/torque evidence;
  the present kinematic animation is not loaded-dynamics validation.

### FEEDBACK-01 — sensor feedback and bounded weld-tip correction

Owner: `src/controls/controls/controller/` (estimation/correction),
`src/controls/controls/hardware/` (execution and cached sensor interfaces),
sensor integration and `src/robot_description/` (frames/calibration).
Requested 2026-11-15; deferred until WEAVE-01's first arm welding run.
Coordinate with LOAD-01 and CHASSIS-04; this is not implemented compensation.

- [ ] Record a baseline using timestamped commanded/measured joint positions,
  velocities, actual dispatched position/speed limits and feedback ages.
  Measure delay, jitter, quantization and tracking error; reuse cached telemetry,
  without enabled-state register polling or another motor owner.
- [ ] Select and calibrate sensing of the wire tip relative to the weld seam,
  including lateral/height error and any required orientation. Establish camera/
  laser/sensor-to-tool/base transforms, timestamps, latency and valid-data rules.
  Encoder forward kinematics alone cannot distinguish configurations with the
  same joint readings but different base slip, tool mounting or structural flex.
- [ ] Implement a bounded outer correction around the existing planned weave:
  measured seam-relative error -> smooth Cartesian correction -> constrained
  IK -> current execution interface. Enforce correction magnitude/rate,
  joint speed/acceleration, closure and clearance after correction. Define
  pause behavior for missing/stale measurements and retain explicit restart.
  Participation OFF must not silently enable compensation for that subsystem.
- [ ] Evaluate a discrete linear extended state observer (LESO) offline first,
  using the measured input/output behavior of the selected control interface.
  Current mode 2 exposes position and a speed magnitude/limit, not commanded
  torque; identify its closed-loop gain/order/delay before defining observer
  input and disturbance units. Do not treat the current speed limit or position
  target as a torque input, or label an effective residual as measured N m.
  A later torque/current feedforward interface requires separate qualification.
- [ ] Compare a simple feedback baseline against ESO-assisted correction using
  replay and simulation with payload changes, friction, coupling, sensor noise,
  delay, dropped samples and saturation. First run the estimator without applying
  its correction. Select bandwidth from measured dynamics and sample timing,
  not a guessed multiple of the 2 Hz weave frequency. Bound correction and
  observer transients; account for the command actually applied after limiting.
- [ ] Combine any qualified LOAD-01 feedforward with estimation of residual
  disturbance without double compensation. An ESO does not create observations
  of unmeasured seam displacement or raise motor speed/torque/acceleration limits.
  Do not claim it makes the current 2 Hz request feasible without new evidence.

Acceptance: compare measured wire-tip RMS/peak error, joint tracking, phase lag,
command peaks and fault behavior against the same trajectory/load baseline;
define thresholds before promotion. Keep model-only, fake-motor and physical
evidence distinct. The present host update period is nominally 20 ms; effective
feedback timing and inner-drive dynamics still need measurement.

Research basis: [Han's ESO/ADRC formulation](https://doi.org/10.1109/TIE.2008.2011621)
uses measured output, known control input and an approximate input gain.
[A manipulator LESO study](https://doi.org/10.3390/electronics9091355) treats
friction/external disturbances with a torque-input model and simulation evidence;
it is not validation of FORGE's mode-2 interface.
[ESO measurement-noise analysis](https://arxiv.org/abs/2004.01483) documents the
trade-off between fast estimation and noise amplification. Applying these
methods to FORGE is an engineering proposal requiring the checks above.

### DEPLOY-01 — fixture, configuration and evidence

Owner: bringup/commissioning and `src/robot_description/ARM_COMMISSIONING.md`.
Record the new A1 base and explicit profile change, expected API/build/capabilities,
bus routes, IDs, calibration revision and operating limits with reports. Verify
A2–A6 ID/settings persistence after power cycling. Keep operator observations
distinct from saved automated stages. Routine successful reuse stays reset-free;
faults and topology/ID-session changes retain explicit recovery. Acceptance is
a reproducible manifest and attributable evidence for each claimed milestone.
