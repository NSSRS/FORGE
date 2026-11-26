# ENCOS chassis base: cross-layer arm/motor TODOs

Start with [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) and
[base API contract](../../docs/ARM_BASE_LAYER.md). Relevant stable IDs: **PROTECT-01**.

The additive cached snapshot ABI and EtherCAT health counters are implemented.
Host-loss/chassis testing and PROTECT-01 are deferred by the operator. Retain
ENCOS transport evidence for explicitly resumed work in the
[chassis registry](../../docs/CHASSIS_MODEL_COMMS_TODOS.md); former COMMS-06
is folded into PROTECT-01 rather than another protection checklist.

Update the central entries with implementation/test evidence; do not duplicate
independent completion status here. Hardware acceptance is separate from unit tests.
