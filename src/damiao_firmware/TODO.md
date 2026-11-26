# Firmware base: open work

Use the [consolidated registry](../../docs/ARM_LAYER_TODOS.md) for status and
[base contract](../../docs/ARM_BASE_LAYER.md) for protocol details.

BASE-06 (including former BASE-05/DEPLOY-01/COMMS-04) is complete within its
[recorded deployment and stationary group-hold scope](../../docs/measurements/arm-base06-deployment-2026-11-26.json).
No additional base-firmware implementation task is selected.

The selected stationary BASE-07 command-loss/recovery check is complete;
reuse its [recorded scope](../../docs/ARM_SESSION_HANDOFF.md#base-07-command-loss-check).

PROTECT-01 and SPEED-01 testing are deferred. Preserve deterministic routing,
leases, cached diagnostics and latched faults. Planning and model/load policy
remain above the firmware layer.
