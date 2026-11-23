# Arm cross-layer TODO registry

Updated 2026-11-23, based on branch `nathan`. This registry contains **open work only**.
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

Prioritize wire-tip position and transverse stroke under ACCURACY-01/SPEED-01.
Allow one fixed lag for arm-only tip scoring and retain the same-time 1 deg
tool-orientation maximum as a secondary criterion. Apply the relevant
CAL-01/SUPPORT-01 checks. LOAD-01 and FEEDBACK-01 follow the first qualified
arm welding run.

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
- [ ] Qualify the fixed A4/A6 CW/CCW 270-degree cable envelopes from their
  preserved initial anchors, including reversal, power cycling and changed cable
  routing. A6's operator-confirmed limit is enforced in software; fake-CAN edge
  tests do not establish physical stopping margins.
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
  identify the loaded plant or establish physical jitter improvement. The Nov 23
  loaded baseline measures A1/A6 at 1.734/1.728 Hz versus the checked 1.730 Hz,
  but travel is 3.891/2.261 deg and 1.333/0.428 deg measured/planned respectively.
  Investigate host lead and command shaping using the saved dispatched targets;
  compare a bounded change at the same load, start and checked path. Qualify amplitude and phase
  without changing the operator-accepted tiny A2–A5 nominal motions; see the
  [latest physical review](measurements/arm-accuracy-speed-review-2026-11-23.json).
- [ ] Qualify coordinated preparation after the timeout/reference-braking fixes
  and the explicit raised-approach option. A5 manual operation is reported normal;
  resolve unequal-axis tracking and gun/floor clearance under load without
  inferring a gain defect from the failed nominal path. Software replay does not
  qualify physical recovery.
  The operator accepted the Nov 23 8.22 s preparation; the former five-second
  goal is retired. Preserve this accepted behavior while correcting welding
  tracking. Its approach nominal peak errors were A2 1.838, A5 1.348 and A6 1.552 deg.
  See [current evidence](ARM_SESSION_HANDOFF.md#calibration-and-evidence).
- [ ] **Broader manual-jog investigation deferred:** investigate moving-only A1/A4/A5/A6 jitter; A2/A3 are smooth
  and earlier manual holding was reported steady. The 5 deg/s MCU smoothing
  trial did not fix travel jitter. Its saved report verifies the build and clean exit, but
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

### ACCURACY-01 — measure motion accuracy during welding

Owner: `src/controls/controls/hardware/`, operator.
The [accuracy test procedure](ARM_RIG_TEST.md#weld-motion-accuracy-test) uses the
weld panel's cached journal and `scripts/analyze_weld_accuracy.py` after the pass.

- [ ] Reduce 3D tip error and both transverse edge errors. Allow one fixed lag
  for continuous arm-only position scoring and report the same-time result as
  well. Compare the standard tracker with the A1 shaper-only trial at 1.730 Hz,
  20 mm stroke and the same 1.96 kg payload, route and settings. Collect
  independent tip data before selecting a position tolerance in mm.
- [ ] Retain **0.5 deg target / 1 deg maximum same-time tool-orientation error**
  relative to the planned tool frame. The [Nov 23 baseline](measurements/arm-accuracy-speed-review-2026-11-23.json)
  has reconstructed full-orientation RMS 1.408 deg / peak 2.633 deg. Use
  joint tracking and dispatch errors to diagnose the response. End continuous
  welding with manual Pause / Hold. Repeat with chassis feed after wheel
  mapping is confirmed. Retain the runtime tracking and arrival guards.
- [ ] Independently measure synchronized tool orientation and 3D wire-tip
  positions in a fixed workpiece frame.
  Record the tool mounting, wire extension, seam axes and timing reference. Set
  the separate position tolerance in mm and compare against the actual checked
  weave and feed, including its retiming. Encoder reconstruction supplies a
  fixed-base estimate. Independent tip data measures the effect of slip, flex
  and tool alignment on workpiece-relative accuracy.

### WELDER-01 — welding machine Modbus API

Owner: [`src/welding_machine/`](../src/welding_machine/TODO.md).

- [ ] Confirm the welder model, connector, transport settings and register map.
  Verify a documented status point on the machine using the
  [read-only RTU/TCP probe](../src/welding_machine/README.md).
- [ ] Add machine-specific status decoding and supported commands after the
  map is confirmed. Verify encoding, readback and fault handling with a
  simulated endpoint before hardware commissioning.

### LOAD-01 — weight compensation after the first arm welding run

Owner: `src/controls/controls/controller/`, `src/robot_description/`, operator.
Deferred by the operator until the arm's initial-pose -> weld-start -> welding
sequence has been run through on the supported fixture. Keep this separate from chassis
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
Requested 2026-11-15; deferred until the first qualified arm welding run.
Coordinate with LOAD-01 and CHASSIS-04; this is not implemented compensation.

- [ ] Use the ACCURACY-01 baseline to evaluate delay, jitter, quantization and
  tracking error against timestamped positions, velocities, dispatched limits
  and feedback ages. Use the existing owner's cached telemetry.
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
