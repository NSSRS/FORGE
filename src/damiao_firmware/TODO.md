# Firmware base: open arm work

The [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) is the status authority.
Owning IDs: **BASE-05, BASE-06, BASE-07, PROTECT-01, SPEED-01**. Read [current hardware status](../../docs/ARM_SESSION_HANDOFF.md)
and [the base contract](../../docs/ARM_BASE_LAYER.md) before continuing.

Next: validate active configuration readback on the rig, implement atomic multi-joint targets and qualify physical fault response. Coordinate speed commissioning with the host/profile layer. Keep deterministic routing, leases, cached diagnostics and latched faults here; planning, ROS coordinates and gravity policy belong above it.

Completed implementation and run instructions belong in the
[rig guide](../../docs/ARM_RIG_TEST.md) and
[source map](../../docs/SOURCE_LAYOUT.md#arm-controls-and-rig-test-file-map).
Update the owning registry entry instead of duplicating session histories here.
