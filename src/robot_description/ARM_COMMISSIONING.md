# Arm hardware commissioning handoff

Updated 2026-11-15; latest saved physical observations are from Nov 13. Current operator/agent status:
[ARM_SESSION_HANDOFF.md](../../docs/ARM_SESSION_HANDOFF.md). This document owns
hardware mapping and calibration context; it is not an executable profile.

## Verified hardware and next stage

The current supported fixture includes **A1-A6**, with six-axis disabled
identity/feedback checks recorded on Nov 12. Use `config/arm_rig_a1_a6.json`.
`config/arm_rig_a2_a6.json` is retained only for an explicitly disconnected A1. Use the [tester guide and joint
reference table](../../docs/ARM_POSE_TEST.md) for hand placement, model
targets, trimming, and staged speed trials. See **SUPPORT-01, CAL-01, SPEED-01 and
DEPLOY-01** in [the open-work registry](../../docs/ARM_LAYER_TODOS.md).

All six motors previously passed the saved 14-stage unloaded group/individual
sequence on 2026-11-05. That free-shaft result, manual motion and later
reference/return observations have different scopes; neither establishes loaded holding,
calibrated joint limits or production readiness.

## Mapping and wiring

| Joint | Motor | Current inventory bus | Command / feedback ID |
| --- | --- | --- | --- |
| A1 | DM-J8009P-2EC V1.1 | CAN1 | 1 / 17 |
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
Read the [assembled-arm procedure](../../docs/ASSEMBLED_ARM_BRINGUP.md) for
qualification of the implemented initial transition and group holding.

`config/arm_motors.json` and `arm_motors_final.json` remain unconfirmed, with
null signs/zeros/limits. The rig profile's explicit support confirmation applies
to each invocation; basic movement does not qualify current protection or loaded
operation. Preserve production flags until their acceptance criteria are met.
The existing `individual_test_passed` entries describe the inventory's recorded
test provenance, not the new operator-reported manual-motion milestone.

Candidate files `arm_calibration_measured.json`, `arm_initial_measured.json`
and `arm_alignment_report.json` now supply measured sign/offset alignment and
controller standby. Gain is one because encoder feedback is output-shaft angle.
Corrected A3 ends are 72.77/151.39 deg; standby raw A3 149.39 deg is a calculated
2-degree backoff. A2/A3 initial coordinates are derived, not separately recorded
initial anchors. Independently validate these candidates before production use.
See [weld motion](../../docs/WELD_MOTION.md); no loaded weld has been qualified.

`urdf/damiao_motor.xacro` identifies families; model poses/clearance-study ranges
are not hardware targets. `config/chassis_motors.json` remains ENCOS-only.
Use [the rig guide](../../docs/ARM_RIG_TEST.md) for current keyboard/disabled
checks and [the source map](../../docs/SOURCE_LAYOUT.md#arm-controls-and-rig-test-file-map)
for ownership. No automatic encoder zeroing/re-enable or hardware gravity feedforward is
provided. Explicit position holding exists; loaded holding remains unqualified.
