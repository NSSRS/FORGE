# Simulation and planning: cross-layer arm/motor TODOs

Start with [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) and
[base API contract](../../docs/ARM_BASE_LAYER.md). Relevant stable IDs: **PLAN-01, CAL-01**.

The simulation-only [single-joint collision study](../../docs/ARM_COLLISION_STUDY.md)
implements candidate range scans and MuJoCo playback for PLAN-01. Combined-pose
validation, calibrated limits and loaded operation remain pending. The D3 rod
assembly uses the operator-confirmed 150 mm between connection centers; see the
[model guide](../../docs/ONSHAPE_ARM_PREVIEW.md#joint-coordinates-and-defaults).

Keep trajectory/collision policy above transport; simulation coordinates are not calibrated hardware coordinates.

Update the central entries with implementation/test evidence; do not duplicate
independent completion status here. Hardware acceptance is separate from unit tests.

For the next supported, assembled-arm stage, follow the
[handoff](../../docs/ASSEMBLED_ARM_BRINGUP.md) and its central TODO IDs.
Keep session jog bounds, sampled model intervals and calibrated physical
limits distinct; coordinated gravity holding remains a separate controller.

[WEAVE-01](../../docs/ARM_LAYER_TODOS.md#weave-01--flat-ground-coordinated-weave-experiment)
tracks the tester review of the [flat-ground chassis and arm weave](../controls/README.md).
