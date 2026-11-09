# Arm cross-layer TODO registry

Read first: [base-layer contract](ARM_BASE_LAYER.md). Stable IDs below are
referenced beside the owning code. Update this registry with evidence and
links when completing work; do not erase unresolved requirements or infer
hardware validation from simulation. Current branch: `nathan`.

For the 2026-11-09 control/model changes, encoder startup semantics and ordered
assembled-arm procedure, read [the handoff](ASSEMBLED_ARM_BRINGUP.md). The operator
reports gravity-driven motion when unpowered. SUPPORT-01 is a prerequisite
for assembled tests; the current unloaded jog adapter removes holding torque.

## Implemented in the base layer (software-tested, hardware deployment pending)

- [x] BASE-01: expose capability/API/build identity and timing constants.
- [x] BASE-02: frozen six-slot cached snapshot, explicit validity/age, three USB
  exchanges, no extra CAN traffic, preserved fault/lease semantics.
- [x] BASE-03: cumulative bus/feedback and coarse timing diagnostics, Python API
  and passive CLI. Counter resets/wrap and unmeasured wire retries documented.

## Pending work, ownership and acceptance criteria

### BASE-04 — qualify five-motor bus timing (firmware + hardware commissioning)

Owner: `src/damiao_firmware/`, `src/motors/arm_telemetry.py`.
Depends on BASE-01..03 deployment. Record exact firmware build, topology,
bitrate, motor firmware/settings, host publication rates, command and feedback
rates, age distribution and maximum gaps under all-six motion. Measure with
CAN capture/logic analyzer as well as cached counters; correlate retries,
FIFO loss, bus-off and late ticks. Repeat final split-bus layout. Acceptance:
zero unexplained loss or watchdog failures; measured deadlines and operating
margin explicitly documented. Do not declare 500 Hz achieved from configuration
alone. Keep A1's observed delayed register replies as an open investigation;
bounded discovery retries do not validate live feedback latency.

### BASE-05 — typed command errors and runtime configuration (firmware + Python)

Owner: `arm.c`, `wire.c`, `src/motors/arm.py`.
Define stable rejection codes (state, route, stale feedback, limits, capability),
return active route/limit configuration, and distinguish static model/protocol
constraints from validated per-application limits. Preserve the old wire API
or negotiate a version. Do not simply remove limits or bypass enable checks.
Acceptance: host can explain a rejection and inspect actual bounds without
reflashing; negative tests cover NaNs, route changes and missing validation.

### BASE-06 — atomic multi-joint targets (firmware + Python)

Owner: `arm.c`, `wire.c`, `src/motors/arm.py`.
Stage a complete six-joint command set, validate all targets, then commit once
with a generation and common application time. Partial USB packets must not
renew any command lease or move a subset. Define timeout/discard behavior and
bench/production mode support. Acceptance: partial/replayed/stale/invalid batches
never apply; measured cross-joint dispatch skew is reported. Do not claim
synchronized trajectories merely because enable uses a group mask.

### BASE-07 — physical fault-injection acceptance (firmware + bringup)

Owner: firmware/bench tests, `src/bringup/`.
Test host crash, USB unplug, lost commands/feedback, CAN fault and recovery,
interrupted tests, motor error status and stop acknowledgement. Record stop
latency and first-fault evidence. Preserve faults until explicit recovery and
never auto-enable. Acceptance requires hardware evidence, not only fake-plant
unit tests. Coordinate SUPPORT-01 before any gravity-loaded testing.

### PROTECT-01 — motor-side current/protection verification (all motor layers)

Owner: `src/motors/`, `src/encos_driver/`, commissioning configs.
Use [MOTOR_PROTECTION_TODO.md](MOTOR_PROTECTION_TODO.md) for all Damiao AND ENCOS
units. Add model-supported read-only version/protection registers only after
checking exact manuals/firmware. Record current units, limits, timeout and
persistent settings per unit. Reactive reported-torque stops are not clamps.
Acceptance: recorded readback, persistence and fault behavior before loaded use.

### ROS-01 — cached telemetry consumer (ROS adapter)

Owner: `src/motor_control/motor_control/arm_driver.py`.
Consume `base_info`, `snapshot` and `bus_health` on the existing serial owner;
do not create a competing polling process. Publish raw-shaft JointState only
from valid, fresh samples; expose per-slot age/status and bus health separately.
Use generation/capture time and receipt-time accounting; do not stamp mixed
old samples as new. Start at 20–50 Hz feedback and 1–5 Hz health and measure
command latency. Acceptance: missing/aged/faulted samples are visible, no CAN
register reads while enabled, no hidden command lease renewal by telemetry.

### ROS-02 — production mode and lifecycle (ROS adapter + base API)

Owner: `arm_driver.py`, `src/bringup/launch/arm.launch.py`.
The ROS production path remains MIT; bench success is position-velocity only.
Select/implement the production mode explicitly. Separate configure/activate,
disable acknowledgement, recover and re-arm. One command owner, fresh complete
commands and explicit fault recovery are mandatory. Acceptance: ROS tests and
unloaded integration evidence for the selected mode; no promotion of template
hardware_confirmed flags as a shortcut. Coordinate BASE-06 before synchronization.

Assembled-arm increment: require fresh complete feedback, preload current
angles, explicitly arm and hold all load-bearing joints. Separate stopping
a jog at neutral/SE release from supported torque disable; retain the current
unloaded adapter behavior. A requested return to home needs a calibrated,
validated path. No startup homing or automatic re-enable after recovery.

### CAL-01 — physical joint calibration (robot_description + ROS)

Owner: `src/robot_description/config/arm_motors*.json`, `arm_driver.py`.
Measure direction, zero, physical travel, startup/wrap behavior and model units
for every DOF. Document transforms between raw shaft SI and robot coordinates;
keep simulation defaults separate. Acceptance: measured round trips and limit
checks on every joint before production flags become true. Distinct IDs and
free-shaft motion do not establish calibration.

Record known-pose/raw-angle pairs, external ratios if present and wrap branch
for every joint. With mechanical support and drives disabled, compare
power-cycle feedback first without movement, then after controlled off-power
movement. The manuals specify single-turn absolute output feedback; actual
startup behavior and normal-feedback versus p_m/xout readback still require
unit-specific verification. Do not treat session-origin capture as model
calibration or use deliberate arm impacts to establish routine limits.

### PLAN-01 — trajectories and collision policy (planning / teleop / simulation)

Owner: `src/simulation/`, `src/teleop/`, future planner package.
Implement joint-space speed/acceleration/jerk constraints, path validation,
collision handling and multi-joint timing above the transport. Consume measured
state age and command acknowledgement; invalidate trajectories on stale state
or faults. Acceptance: simulator and hardware-boundary tests, then separately
approved loaded tests. Do not put IK/trajectory planning into MC02 or replay
old commands to disguise host stalls. Depends on CAL-01, ROS-02, BASE-06.

Simulation evidence (2026-11-09): `simulation.arm_sweep` scans six independent
joints around the saved chassis/arm pose, solving D3 closure and querying the
full scene collision geometry with a configurable clearance margin. The native
MuJoCo viewer and optional MP4 show each sampled interval and its limiting pair.
See [method, candidate results and caveats](ARM_COLLISION_STUDY.md). These are
conditional model slices, not six independently combinable joint limits; search
endpoints are not mechanical stops. Contact exclusions/omitted surfaces remain
visible in the JSON audit. No hardware ranges, calibration flags or transport
settings are changed. Verification: six new scan/playback tests plus 33 existing
layout/model/overview/color/magnetic/terrain/actuator/mirror tests passed; the
48-second MP4 rendered all 1,152 poses with valid closure and clearance, and
the native viewer launched. PLAN-01 remains open for combined-pose/path validation,
geometry review, dynamics and the existing hardware acceptance items.

Dimension confirmation (2026-11-09): operator clarified that D3 is **150 mm
between ball/connection centers**, not a 150 mm bare rod. Integrated XML and
preview/terrain copies now use exactly 0.150 m; raw CAD and 140 mm bare-rod meshes
are unchanged. The former 150.002488 mm site distance needed only a 2.488 micrometre
rounding correction. Saved flat/curved/bumpy poses were reclosed by adjusting only
D3 passive coordinates (bearing change about +0.002618 degrees). Integration
retains the correction and rejects differences larger than 0.01 mm for geometry
review. The closure solver now checks an unchanged initial qpos before accepting it.
Verification: 43 model/geometry/sweep regressions passed; all six sampled candidate
intervals are unchanged at the current search resolution. The existing dynamic
arm preview passed its 1 mm closure tolerance (maximum 0.7234 mm). This remains
simulation evidence; hardware calibration and loaded acceptance remain pending.

### SUPPORT-01 — gravity-load stopping and physical stop chain (bringup/system)

Owner: `src/bringup/`, mechanical/electrical safety integration.
Define support/brakes, emergency-stop chain, power loss, drive timeout and
recovery under gravity. MC02 torque disable is not a brake. Establish which
faults may use a controlled stop and which require immediate drive disable,
without hiding fault conditions. Acceptance: documented physical design and
measured stop/hold behavior before an unsupported loaded arm is enabled.

The operator reports the assembled arm falls unpowered (2026-11-09). Establish
physical support/rest pose and loss-of-torque behavior before calibration or
holding tests. Current Pocket neutral/diagonal/release stops disable torque,
and nonselected joints stay disabled; preserving SE prime is authorization
only. Continuous coordinated holding needs the separate ROS-02 lifecycle.

### DEPLOY-01 — versioned configuration and evidence (bringup + commissioning)

Owner: `src/bringup/`, scripts, `ARM_COMMISSIONING.md`.
Record expected API/build/capabilities, bus topology, ID inventory, calibration
revision and limits with each report. Verify A2..A6 post-power-cycle persistence
explicitly and reconcile user-reported motion success with saved stage reports.
Do not conflate a build fingerprint with firmware authenticity. Keep routine
successful session reuse reset-free; retain explicit recovery for faults,
ID-session locks and wiring/route changes. Acceptance: reproducible deployment
manifest and attributable hardware reports for each claimed milestone.

## Agent workflow

1. Read the relevant local `TODO.md` and the owning registry entries.
2. Verify current branch, working tree and hardware evidence before editing.
3. Preserve other layers' units/contracts and unresolved confirmation flags.
4. Add tests at the boundary changed; record which checks are simulation versus hardware.
5. Update status/evidence here and the local pointer when the work is complete.

## Position port / manual pipeline increment (2026-11-05)

Implemented `PositionPort`, `ArmPositionPort` and offline adapter in
`src/motors/arm_position.py`, plus number-selected PyQt hold-to-jog window in
`src/teleop/teleop/arm_keyboard.py`. See [manual/API guide](ARM_POSITION_JOG.md).
This is a software-tested single-joint unloaded commissioning port, not a
completion of ROS-02, CAL-01, BASE-06, PLAN-01 or SUPPORT-01. The next agent should
integrate the explicit port at the ROS boundary, choose calibrated coordinates
and mode/lifecycle. Real key-release, focus-loss stop and held-key jogging now
exist for unloaded commissioning; production deadman/arbitration remain pending.
The base hardware adapter still uses the tested routed bench lifecycle; do not
silently treat its per-operation deadlines or disabled idle state as a loaded
production-arm control policy.

## Pocket manual input increment (2026-11-06)

Implemented left-stick chassis plus right-stick CW/CCW arm input with held SE,
SB/SC selection and a separate single-owner arm worker. Input deadlines survive
mailbox publication and are checked during enable as well as command refresh.
See [Pocket controls](POCKET_ARM_CONTROL.md). Policy, native C-backed cancellation
and real ROS/PTY offline-arm tests pass. This is an unloaded commissioning
adapter. Left-stick CH3/CH4 and SB/SC/SE CH6/CH7/CH8 readings were recorded
on 2026-11-06 after adding the missing EdgeTX mixes. Measured input replay
passes. Motor CW signs were subsequently recorded on 2026-11-09 (all -1;
operator observed keyboard Up=CCW on A1..A6). Live combined-control acceptance,
left-stick spring return and transmitter-loss behavior remain open.
PLAN-01 is advanced, not completed; ROS-02 production lifecycle, CAL-01, BASE-06,
PROTECT-01 and SUPPORT-01 remain open. No production confirmation flags changed.


The Pocket launch now includes a PyQt dashboard sharing the keyboard joint
readout. Raw-axis diagnostics expose neutral/primer/diagonal states; animated
shaft dials and chassis command values support the live simulation test.
Snapshots reuse the control owner's samples without new USB/CAN requests;
GUI STOP revokes both command paths, and window exit shuts down the launch.
This does not establish physical CW signs, RF-loss or loaded-arm acceptance.


### PLAN-01 — Pocket hardware acceptance handoff (2026-11-06)

The real DM-MC02 path already exists: `ArmPositionPort` in
`src/motors/arm_position.py`, selected by `pocket_teleop.arm_factory` when
`arm_execute:=true`. The keyboard GUI also supports `--execute`. This is
unloaded single-motor jogging, not a missing hardware implementation or a
validated loaded-arm controller. The command and next steps are in
[hardware commissioning](POCKET_ARM_CONTROL.md#hardware-commissioning-next-step).

- [x] Record left CH3/CH4 and SB/SC/SE CH6/CH7/CH8 readings from the operator.
- [x] Implement and software-test the Pocket hardware adapter, input withdrawal
  during enable, joint selection, GUI telemetry and GUI STOP.
- [x] Implement the 2026-11-09 requested corner behavior: all four diagonals
  command zero without revoking an existing SE prime. Returning to one axis
  resumes with SE held; releasing SE, selection changes, stale/invalid data
  and explicit stop still revoke authorization. The GUI shows an amber
  diagonal stop. Policy/worker, Qt and ROS/PTY simulation cover this boundary.
- [ ] Confirm the right-stick neutral reading and diagonal stop/resume on the
  physical controller. The original centered-stick conflict report did not
  include raw samples; do not adjust calibration without measurements.
- [ ] Confirm the left vertical stick physically returns to neutral.
- [x] Record raw-positive rotation for each A1..A6 on the secured unloaded
  fixture using the documented output-shaft viewpoint. On 2026-11-09 the
  operator reported keyboard Up=CCW for all six; `arm.cw_raw_signs` is now
  `[-1, -1, -1, -1, -1, -1]`. This removes the unknown-sign startup block,
  without changing production joint calibration or confirmation flags.
- [ ] Run live Pocket hardware acceptance: selection isolates one motor,
  neutral-before-SE priming works, SE release/center/selection change/diagonal
  stops motion, diagonal return resumes with SE held, GUI STOP and window close stop the session, and firmware
  disable/mode restoration succeed. Record logs and hardware identity.
- [ ] Test transmitter-off, receiver power loss, USB loss and reconnection.
  Fresh CSV alone cannot detect receiver hold-last failsafe. Preserve original
  sample deadlines and require re-selection/re-priming after a detected loss.

The latest reported simulated run demonstrated chassis targets and A2 selection,
but reported a right-stick conflict. The dashboard was added afterward. The
2026-11-09 keyboard direction observations establish the six shaft CW signs;
the operator subsequently reported working Pocket motor control after an ER12
restart. Full stop/failsafe acceptance and physical validation of the updated
diagonal behavior remain pending. Cached live telemetry showed the same +/-90
session bounds for all six motors, with A6 last sampled at -89.55 degrees.

## Handoff verification (2026-11-09)

Before push: 43 model/geometry/sweep regressions plus 26 Pocket policy/Qt,
three native position-port and one ROS/PTY integration test passed (73 total).
The terrain scenes were restored after generation checks. No physical motor
ports were opened. The [assembled-arm sequence](ASSEMBLED_ARM_BRINGUP.md)
records the remaining support, absolute-position calibration, combined-path,
coordinated-hold and fault/load qualification work without closing those IDs.
