# Arm cross-layer TODO registry

Updated 2026-11-12, based on branch `nathan`. This registry contains **open work only**.
Read [current status](ARM_SESSION_HANDOFF.md) and [the base contract](ARM_BASE_LAYER.md)
first. Stable IDs link package responsibilities; update the owning entry when
work changes, and retire completed tasks instead of appending session history.

The operator verified manual movement of **A2–A6**. Completed implementation
includes cached base identity/snapshots/health (former BASE-01..03), subset
checks, manual travel, guarded software recovery, disabled control-register
readback, fault reporting and the compact keyboard UI. These are documented
in the guides; full hardware qualification below remains separate.

## Immediate priorities

1. Validate the recorded raw references (**CAL-01**): power-cycle repeatability,
   remaining travel bounds, A2/A3 initial poses and model sign/offset/ratio.
   The eight requested readings are saved in [the handoff](ARM_SESSION_HANDOFF.md).
2. Establish resting support for torque loss and finish loaded stopping/holding
   qualification (**SUPPORT-01**, **DEPLOY-01**).
3. Resume jitter investigation and qualify commanded speeds later (**SPEED-01**),
   explicitly deferred by the operator on 2026-11-12.

### SUPPORT-01 — base, gravity support and physical stopping

Owner: operator/mechanical integration, `src/bringup/`.

- [ ] Build and document the base with A1 rotation and cable clearance; establish
  supported rest poses and support for every attached link during torque loss.
- [ ] Define brakes/counterbalance/support, emergency stop, power loss, drive
  timeout and recovery. Distinguish controlled stopping from torque disable.
- [ ] Measure stop/hold behavior before unsupported loaded operation. The current
  jogger disables torque at release; nonselected joints stay disabled. MC02
  disable is not a brake. Coordinated holding belongs to ROS-02.
- [ ] Resolve the 2026-11-12 capture session's stationary-disable/mode-restoration
  failure: A2 still reported -0.168498 rad/s after disabling. Preserve the saved
  [shutdown evidence](measurements/arm-reference-2026-11-12.session-summary.json)
  and verify shutdown on a supported fixture before treating cleanup as qualified.

### CAL-01 — motor-position initialization and physical calibration

Owner: `src/robot_description/`, `src/motors/`, ROS coordinate adapter.

Bench acceptance uses the [joint reference table and pose procedure](ARM_POSE_TEST.md).
Record fixture measurements before promoting any calibration to production.
The initial eight requested observations are preserved in the
[dated reference record](../src/robot_description/config/arm_reference_positions_2026-11-12.json).
Validation and full coordinate mapping remain open.
The hold-all jogger's `--capture-file` defaults to initial references for
A1/A4/A5/A6 and both CW/CCW ends for A2/A3, preserving all holds and saving all
joint angles for context. The longer two-ends-plus-initial workflow for every
motor remains available through `--capture-mode bounds`.
These observations are not automatic homing or globally valid collision limits.
See the [capture procedure](ARM_POSE_TEST.md#current-procedure-initial-positions-and-a2a3-travel-ends).

- [ ] Record known physical-pose/raw-angle pairs for each DOF: sign, zero offset,
  any external ratio, wrap branch and units. Separate model zero from session zero.
- [ ] Verify disabled startup readings after power cycles, first without motion
  and then after controlled off-power movement. Check normal CAN feedback versus
  documented `p_m`/`xout` where supported; do not assume retained multi-turn state.
- [ ] Implement startup initialization from fresh measured coordinates and
  current-position preload. No jump toward guessed zero, automatic homing or
  automatic return to the simulation pose; a home move needs a separate request
  and a validated path.
- [ ] Identify physical travel/cable/clearance limits under support, save a
  versioned calibration, and verify round trips at additional poses. Keep null
  calibration/false production flags until measured acceptance is complete.

### SPEED-01 — faster commanded movement

Owner: `src/motors/arm_position.py`, `src/teleop/`, firmware/profile commissioning.

- [ ] **Deferred:** investigate moving-only A1/A4/A5/A6 jitter; A2/A3 are smooth
  and stationary holding is steady by operator report. The 5 deg/s MCU smoothing
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
  Preserve trips and leases. See the [research and evidence](ARM_SESSION_HANDOFF.md#research-physical-torque-ripple-versus-drivetiming-effects).
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

### ROS-01 — cached telemetry consumer

Owner: `src/motor_control/motor_control/arm_driver.py`.
Consume `base_info`, `snapshot` and `bus_health` on the existing serial owner.
Publish valid raw-shaft samples with age/status, generation/capture timing and
receipt-time accounting. Start with 20–50 Hz feedback / 1–5 Hz health, then
measure command latency. No enabled CAN register polling or telemetry-driven
command lease renewal; expose missing/stale samples rather than restamping them.

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

Owner: `src/simulation/`, `src/teleop/`, future planner.
Implement joint-space speed/acceleration/jerk policy, combined-pose/path collision
validation and coordinated timing above the transport. Invalidate plans on stale
state or faults; do not replay old commands to hide host stalls. Depends on
CAL-01, ROS-02 and BASE-06. The existing D3-closed single-joint clearance study
is documented in [ARM_COLLISION_STUDY.md](ARM_COLLISION_STUDY.md); its sampled
intervals are not independently combinable hardware limits.

Pocket physical acceptance still needs:

- [ ] Measured right-stick neutral and all four diagonal stop/resume behaviors;
  confirm the left vertical stick physically returns to neutral.
- [ ] Live selection isolation, neutral-before-SE priming, release/center/change
  stops, GUI STOP/close, disable and mode restoration with identity/log evidence.
- [ ] Transmitter-off, receiver-power, USB-loss and reconnection tests. Fresh CSV
  cannot identify hold-last RF failsafe; retain sample deadlines and fresh priming.

### WEAVE-01 — flat-ground coordinated weave experiment

- [ ] Confirm the flat butt joint and the 10° drag angle against the weld procedure.
  Keep the gun fixed in the DOF6 clamp. Verify wire extension and cable clearance
  on the rig. The fixed-mount simulation reaches the full width at 1–3 Hz with
  a measured peak edge overshoot of 0.32 mm. Agree on physical trial tolerances.
  Use the [simulation controller and tester guide](../src/controls/README.md).
  Record the accepted amplitude, feed and tracking tolerance for the intended seam.
  Physical pose teaching and A6 speed trials remain under CAL-01 and SPEED-01.

### DEPLOY-01 — fixture, configuration and evidence

Owner: bringup/commissioning and `src/robot_description/ARM_COMMISSIONING.md`.
Record the new A1 base and explicit profile change, expected API/build/capabilities,
bus routes, IDs, calibration revision and operating limits with reports. Verify
A2–A6 ID/settings persistence after power cycling. Keep operator observations
distinct from saved automated stages. Routine successful reuse stays reset-free;
faults and topology/ID-session changes retain explicit recovery. Acceptance is
a reproducible manifest and attributable evidence for each claimed milestone.
