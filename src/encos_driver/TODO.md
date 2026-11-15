# ENCOS chassis base: cross-layer arm/motor TODOs

Start with [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) and
[base API contract](../../docs/ARM_BASE_LAYER.md). Relevant stable IDs: **PROTECT-01**.

The additive cached snapshot ABI and EtherCAT health counters are implemented.
Verify physical CAN freshness: a newly observed bridge PDO may repeat an older
motor frame. See [telemetry](../../docs/MOTOR_TELEMETRY.md) and
[chassis communication work](../../docs/CHASSIS_MODEL_COMMS_TODOS.md).
Damiao telemetry does not validate ENCOS protections; use its own transport evidence.

Update the central entries with implementation/test evidence; do not duplicate
independent completion status here. Hardware acceptance is separate from unit tests.
