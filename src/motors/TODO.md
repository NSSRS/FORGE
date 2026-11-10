# Python motor APIs: open arm work

The [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) is the status authority.
Owning IDs: **BASE-04, BASE-05, BASE-06, PROTECT-01, CAL-01, SPEED-01**. Read [current hardware status](../../docs/ARM_SESSION_HANDOFF.md)
and [the base contract](../../docs/ARM_BASE_LAYER.md) before continuing.

Next: measured position initialization and staged faster commands after the new base is ready. Keep raw-shaft SI, one transport owner, explicit recovery and cached telemetry. Coordinate initialization with robot_description; keep gravity holding in ROS-02.

Completed implementation and run instructions belong in the
[rig guide](../../docs/ARM_RIG_TEST.md) and
[source map](../../docs/SOURCE_LAYOUT.md#arm-controls-and-rig-test-file-map).
Update the owning registry entry instead of duplicating session histories here.
