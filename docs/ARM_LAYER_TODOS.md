# Arm cross-layer TODO registry

Read first: [base-layer contract](ARM_BASE_LAYER.md). Stable IDs below are
referenced beside the owning code. Update this registry with evidence and
links when completing work; do not erase unresolved requirements or infer
hardware validation from simulation. Current branch: `nathan`.

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

### CAL-01 — physical joint calibration (robot_description + ROS)

Owner: `src/robot_description/config/arm_motors*.json`, `arm_driver.py`.
Measure direction, zero, physical travel, startup/wrap behavior and model units
for every DOF. Document transforms between raw shaft SI and robot coordinates;
keep simulation defaults separate. Acceptance: measured round trips and limit
checks on every joint before production flags become true. Distinct IDs and
free-shaft motion do not establish calibration.

### PLAN-01 — trajectories and collision policy (planning / teleop / simulation)

Owner: `src/simulation/`, `src/teleop/`, future planner package.
Implement joint-space speed/acceleration/jerk constraints, path validation,
collision handling and multi-joint timing above the transport. Consume measured
state age and command acknowledgement; invalidate trajectories on stale state
or faults. Acceptance: simulator and hardware-boundary tests, then separately
approved loaded tests. Do not put IK/trajectory planning into MC02 or replay
old commands to disguise host stalls. Depends on CAL-01, ROS-02, BASE-06.

### SUPPORT-01 — gravity-load stopping and physical stop chain (bringup/system)

Owner: `src/bringup/`, mechanical/electrical safety integration.
Define support/brakes, emergency-stop chain, power loss, drive timeout and
recovery under gravity. MC02 torque disable is not a brake. Establish which
faults may use a controlled stop and which require immediate drive disable,
without hiding fault conditions. Acceptance: documented physical design and
measured stop/hold behavior before an unsupported loaded arm is enabled.

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
