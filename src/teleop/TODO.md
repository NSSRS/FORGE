# Teleoperation: cross-layer arm/motor TODOs

Start with [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) and
[base API contract](../../docs/ARM_BASE_LAYER.md). Relevant stable IDs: **PLAN-01, ROS-02**.

Require fresh operator intent, arbitration and planner validity; never use telemetry polling as a command heartbeat.

Update the central entries with implementation/test evidence; do not duplicate
independent completion status here. Hardware acceptance is separate from unit tests.

The Pocket dual-stick adapter and radio-only calibration tool now exist; see
[Pocket controls](../../docs/POCKET_ARM_CONTROL.md). Left stick is chassis;
right stick jogs the SB/SC-selected motor only with SE held. Radio channels and
all six CW signs (-1, operator observations on 2026-11-09) are recorded. All
four diagonals now stop motion while preserving an existing SE prime; return
to one axis resumes while SE stays held. Confirm neutral and diagonal behavior
on the physical controller and complete Pocket/failsafe acceptance; see the
PLAN-01 hardware handoff in the central registry. Loaded behavior remains
separate acceptance work.

The single-joint position port and keyboard pipeline now exist; see
[ARM_POSITION_JOG.md](../../docs/ARM_POSITION_JOG.md). Preserve its explicit
per-selection authorization, key-release stop and fault behavior when adding new adapters.

For the next supported, assembled-arm stage, follow the
[handoff](../../docs/ASSEMBLED_ARM_BRINGUP.md) and its central TODO IDs.
Keep session jog bounds, sampled model intervals and calibrated physical
limits distinct; coordinated gravity holding remains a separate controller.
