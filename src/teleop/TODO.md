# Teleoperation: open arm work

The [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) is the status authority.
Owning IDs: **SPEED-01, PLAN-01, ROS-02**. Read [current hardware status](../../docs/ARM_SESSION_HANDOFF.md)
and [the base contract](../../docs/ARM_BASE_LAYER.md) before continuing.

Next: qualify faster commanded speeds on the supported fixture and finish physical Pocket stop/failsafe acceptance. Preserve fresh intent, input deadlines, cancellation during enable, keyboard recovery staying disarmed, and the current release-disable behavior. GUI telemetry must not create another owner or command heartbeat.

Completed implementation and run instructions belong in the
[rig guide](../../docs/ARM_RIG_TEST.md) and
[source map](../../docs/SOURCE_LAYOUT.md#arm-controls-and-rig-test-file-map).
Update the owning registry entry instead of duplicating session histories here.
