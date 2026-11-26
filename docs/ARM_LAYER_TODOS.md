# Arm cross-layer TODO registry

Updated 2026-11-26 on `nathan`. This is the consolidated work list.
[Current status and completed checks](ARM_SESSION_HANDOFF.md) live in the handoff;
procedures live in [ARM_RIG_TEST.md](ARM_RIG_TEST.md), and contracts in
[ARM_BASE_LAYER.md](ARM_BASE_LAYER.md). Package TODOs point here rather than
creating another acceptance list.

## Remaining implementation and integration

These are work items, not a compulsory sequence of motor tests. Choose a test
for a concrete change, observed problem or missing result needed by the next
feature. Do not reopen completed captures, readbacks or motion checks because
a broader production qualification remains unmeasured.

### CAL-01 — motor-position initialization and physical calibration

Owner: robot description, motor APIs and the ROS coordinate adapter.

- [ ] Complete independent angle/encoder-branch validation and the versioned
  raw-to-model mapping needed by the next operating range or ROS integration.
  Use selected relevant poses and record any actual mismatch or new cable/tool
  constraint through the [calibration tool](ARM_MODEL_CALIBRATION.md).

The eight references, Nov 13 return round and Nov 26 A5 visual comparison are
recorded in the [handoff](ARM_SESSION_HANDOFF.md#calibration-and-evidence).
They are not requests for another capture round. Numeric angle accuracy and
unobserved branches remain unknown; retain the fixed cable references and
existing candidate/production status.

### WELDER-01 — welding machine Modbus API

Owner: [`src/welding_machine/`](../src/welding_machine/TODO.md).

- [ ] Confirm the machine/register map, verify a documented status read, then
  implement the required named status/command API. Machine communication is
  separate from arm movement; details stay in the owning package.

## Deferred integration, tests and tuning

### ROS-02 — production lifecycle and coordinated holding

**Deferred by the operator on Nov 26.** Resume on another day; no ROS control
implementation or physical ROS session is selected.

Owner: `src/motor_control/`, motor APIs and bringup.

- [ ] Integrate configure, activate, stop, supported disable, fault recovery and
  explicit re-arm through one arm owner. Use current-angle preload and the
  coordinated target API; verify the lifecycle with ROS/fake-motor tests before
  an operator run in the chosen motor mode.

Reuse the completed [BASE-06 batch API](ARM_BASE_LAYER.md#base-06-atomic-group-targets), the
[recorded command-loss/recovery check](ARM_SESSION_HANDOFF.md#base-07-command-loss-check)
for its tested behavior and CAL-01 for coordinates. Do not duplicate those tests
in the ROS checklist.

### PROTECT-01 — current/protection verification

**Deferred by the operator on Nov 26.** No current, thermal-endurance or overload
session is selected. The [protection notes](MOTOR_PROTECTION_TODO.md) retain the
remaining units/scaling, independent measurement and operating-limit questions
for when this work is resumed. COMMS-06 is folded into this entry.
Existing trips and fault behavior remain unchanged; deferral is not qualification.

### SPEED-01 — faster commanded movement

**Weld tuning and broader manual-jog investigation remain deferred.** When
resumed, address the observed startup/stroke error or travel jitter using the
[selected v7 baseline](measurements/arm-weld-v7-baseline-2026-11-24.json) and
[latest measurement review](measurements/arm-weld-accuracy-review-2026-11-26.json).
Keep preparation separate, with the accepted 10 s goal. Confirm live logging on
the next otherwise-needed session after the serialization fix; do not schedule
a special motor run just for that check.

Chassis work is deferred until assembly; its own work list is
[CHASSIS_MODEL_COMMS_TODOS.md](CHASSIS_MODEL_COMMS_TODOS.md).

## Future control features

These are development options, not extra acceptance tests for the current v7
workflow. A completed bench run does not automatically start these projects.

### LOAD-01 — weight compensation after the first arm welding run

Identify useful mass/COM/inertia data and a motor interface that accepts the
required feedforward before implementing gravity/load compensation. Mode 2
currently sends position and speed, not torque. Gun-off/gun-on paired hold
testing is excluded by operator request.

### FEEDBACK-01 — sensor feedback and bounded weld-tip correction

Calibrate tool/seam sensing and its timestamps, then evaluate bounded correction
against the saved baseline. External sensing is needed to observe base slip or
flex not represented by encoders. ESO is a possible later offline experiment,
not a required controller feature. Coordinate model data with LOAD-01 and
sensor frames with CHASSIS-04.

### ILC-01 — iterative learning for repeated welding (later future work)

**Deferred.** Revisit only for repeatable cycle errors after the selected
tracking baseline is understood. Evaluate offline first with explicit path/load
identity and bounded commands. No learning is active.

## Merged IDs

| Former duplicate | Canonical owner |
| --- | --- |
| BASE-05, DEPLOY-01, COMMS-04 | BASE-06: complete; [deployment and supported stationary group-hold evidence](measurements/arm-base06-deployment-2026-11-26.json) |
| SUPPORT-01, COMMS-03 | BASE-07: completed stationary command-loss/recovery check; scope and remaining unknowns are in the [handoff](ARM_SESSION_HANDOFF.md#base-07-command-loss-check) |
| COMMS-06 | PROTECT-01: deferred protection work |
| COMMS-05 | ROS-02 for arm lifecycle; CHASSIS-04 for chassis command arbitration |

These are merges, not claims of new hardware acceptance. Detailed implementation
contracts, measured evidence and unknown qualification fields remain in their
owning documents; none creates an additional automatic bench-test task.
