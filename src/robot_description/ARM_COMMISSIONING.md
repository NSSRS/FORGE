# Arm hardware commissioning handoff

Updated 2026-11-10. Current operator/agent status:
[ARM_SESSION_HANDOFF.md](../../docs/ARM_SESSION_HANDOFF.md). This document owns
hardware mapping and calibration context; it is not an executable profile.

## Verified hardware and next stage

The operator verified manual movement of all five active **A2–A6** motors on
2026-11-10. A1 is omitted from `config/arm_rig_a2_a6.json` while the operator
builds a base that permits A1 rotation. Next: commission A1 in that fixture,
implement measured motor-position initialization and identify joint travel,
then qualify faster commanded movement. See **SUPPORT-01, CAL-01, SPEED-01 and
DEPLOY-01** in [the open-work registry](../../docs/ARM_LAYER_TODOS.md).

All six motors previously passed the saved 14-stage unloaded group/individual
sequence on 2026-11-05. That earlier free-shaft result and the current five-motor
manual observation have different scopes; neither establishes loaded holding,
calibrated joint limits or production readiness.

## Mapping and wiring

| Joint | Motor | Current inventory bus | Command / feedback ID |
| --- | --- | --- | --- |
| A1 | DM-J8009P-2EC V1.1 | CAN1; omitted from active rig | 1 / 17 |
| A2, A3 | DM-J4340P-2EC V1.1 | CAN2 | 2 / 18, 3 / 19 |
| A4, A5, A6 | DM-J4310P-2EC | CAN2 | 4 / 20, 5 / 21, 6 / 22 |

IDs were written/read back; A1 ID persistence after power cycling was verified.
A2–A6 persistence still needs an explicit recorded check. See
[ID assignment/evidence](../../docs/ARM_MOTOR_IDS.md) and
`config/arm_commissioning_inventory.json`. The 4310P hardware V1.1 label remains
unverified. Planned final wiring moves A4–A6 to CAN3; do not apply it implicitly.
Classic CAN is 1 Mbps, target 500 Hz per motor; timing/load qualification is open.

8009P has one H/L connector, with no direct CAN pass-through. Smaller P-series
motors have two XT30(2+2) connectors. CAN3 pins 1/2 are 5 V/GND, not motor power;
pins 3/4 are H/L. Use separately rated motor power and bus-end termination.

## Position initialization and profiles

Record raw output-shaft angle versus repeatable physical/model poses, sign,
offset, any external transmission ratio and wrap branch for each joint. Verify
disabled startup after power cycles and controlled off-power movement. Session
origin capture does not write persistent encoder zero and is not calibration.
Read the [assembled-arm procedure](../../docs/ASSEMBLED_ARM_BRINGUP.md) before
implementing initialization, homing or coordinated holding.

`config/arm_motors.json` and `arm_motors_final.json` remain unconfirmed, with
null signs/zeros/limits. The rig profile's explicit support confirmation applies
to each invocation; basic movement does not qualify current protection or loaded
operation. Preserve production flags until their acceptance criteria are met.
The existing `individual_test_passed` entries describe the inventory's recorded
test provenance, not the new operator-reported manual-motion milestone.

`urdf/damiao_motor.xacro` identifies families; model poses/clearance-study ranges
are not hardware targets. `config/chassis_motors.json` remains ENCOS-only.
Use [the rig guide](../../docs/ARM_RIG_TEST.md) for current keyboard/disabled
checks and [the source map](../../docs/SOURCE_LAYOUT.md#arm-controls-and-rig-test-file-map)
for ownership. No automatic zeroing, re-enable or gravity holding is provided.
