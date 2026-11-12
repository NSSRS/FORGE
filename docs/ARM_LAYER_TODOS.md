# Arm cross-layer TODO registry

Updated 2026-11-11, based on branch `nathan`. This registry contains **open work only**.
Read [current status](ARM_SESSION_HANDOFF.md) and [the base contract](ARM_BASE_LAYER.md)
first. Stable IDs link package responsibilities; update the owning entry when
work changes, and retire completed tasks instead of appending session history.

The operator verified manual movement of **A2–A6**. Completed implementation
includes cached base identity/snapshots/health (former BASE-01..03), subset
checks, manual travel, guarded software recovery, disabled control-register
readback, fault reporting and the compact keyboard UI. These are documented
in the guides; full hardware qualification below remains separate.

## Immediate priorities

1. Operator builds a base allowing A1 to spin; commission A1 in that fixture
   (**SUPPORT-01**, **DEPLOY-01**).
2. Verify taught pose replay, implement production position initialization and identify travel limits
   (**CAL-01**).
3. Qualify faster commanded movement on the supported fixture (**SPEED-01**).

### SUPPORT-01 — base, gravity support and physical stopping

Owner: operator/mechanical integration, `src/bringup/`.

- [ ] Build and document the base with A1 rotation and cable clearance; establish
  supported rest poses and support for every attached link during torque loss.
- [ ] Define brakes/counterbalance/support, emergency stop, power loss, drive
  timeout and recovery. Distinguish controlled stopping from torque disable.
- [ ] Measure stop/hold behavior before unsupported loaded operation. The current
  jogger disables torque at release; nonselected joints stay disabled. MC02
  disable is not a brake. Coordinated holding belongs to ROS-02.

### CAL-01 — motor-position initialization and physical calibration

Owner: `src/robot_description/`, `src/motors/`, ROS coordinate adapter.

Bench acceptance uses the [joint reference table and pose procedure](ARM_POSE_TEST.md).
Record fixture measurements before promoting any calibration to production.

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
