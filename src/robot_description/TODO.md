# Robot description and calibration: open arm work

The [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) is the status authority.
Owning IDs: **SUPPORT-01, CAL-01, SPEED-01, DEPLOY-01**. Read [current hardware status](../../docs/ARM_SESSION_HANDOFF.md)
and [the base contract](../../docs/ARM_BASE_LAYER.md) before continuing.

Next: record the base allowing A1 rotation, then measure startup position/sign/offset/wrap/travel and save versioned calibration. IDs and simulator poses are not physical calibration. Preserve unknown values and false production flags until measured acceptance; faster-motion settings need separate qualification.

Completed implementation and run instructions belong in the
[rig guide](../../docs/ARM_RIG_TEST.md) and
[source map](../../docs/SOURCE_LAYOUT.md#arm-controls-and-rig-test-file-map).
Update the owning registry entry instead of duplicating session histories here.
