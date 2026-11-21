# Simulation and planning: cross-layer arm/motor TODOs

Start with [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) and
[base API contract](../../docs/ARM_BASE_LAYER.md). Relevant stable IDs: **PLAN-01, CAL-01**.

The simulation-only [single-joint collision study](../../docs/ARM_COLLISION_STUDY.md)
implements candidate range scans and MuJoCo playback for PLAN-01. The weld
planner now checks sampled combined poses and complete paths; its checked-plan
renderer is in `simulation/weld_animation.py`. Physical clearance, calibrated
limits and loaded dynamics remain unqualified. Follow LOAD-01 for model mass/
inertia identification and FEEDBACK-01 for sensor-frame validation. The D3 rod
assembly uses the operator-confirmed 150 mm between connection centers; see the
[model guide](../../docs/ONSHAPE_ARM_PREVIEW.md#joint-coordinates-and-defaults).

Keep trajectory/collision policy above transport; simulation coordinates are not calibrated hardware coordinates.

Update the central entries with implementation/test evidence; do not duplicate
independent completion status here. Hardware acceptance is separate from unit tests.

For the next supported, assembled-arm stage, follow the
[handoff](../../docs/ASSEMBLED_ARM_BRINGUP.md) and its central TODO IDs.
Keep session jog bounds, sampled model intervals and calibrated physical
limits distinct; coordinated gravity holding remains a separate controller.

[ACCURACY-01](../../docs/ARM_LAYER_TODOS.md#accuracy-01--measure-motion-accuracy-during-welding)
uses the checked model to reconstruct fixed-base wire-tip tracking from encoder logs.
