# Bringup and deployment: open arm work

The [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) is the status authority.
Owning IDs: **SUPPORT-01, DEPLOY-01, BASE-07, ROS-02**. Read [current hardware status](../../docs/ARM_SESSION_HANDOFF.md)
and [the base contract](../../docs/ARM_BASE_LAYER.md) before continuing.

Next: record the operator-built base, commission A1 through an explicit profile, and preserve reproducible deployment/fault evidence. Keep radio discovery before node startup and exclude other devices; no implicit reconnect/re-arm. Rig radio profiles belong in config/ and setup.py; reports belong under build/arm_tests/.

Completed implementation and run instructions belong in the
[rig guide](../../docs/ARM_RIG_TEST.md) and
[source map](../../docs/SOURCE_LAYOUT.md#arm-controls-and-rig-test-file-map).
Update the owning registry entry instead of duplicating session histories here.
