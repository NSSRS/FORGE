# Python motor APIs: cross-layer arm/motor TODOs

Start with [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) and
[base API contract](../../docs/ARM_BASE_LAYER.md). Relevant stable IDs: **BASE-04, BASE-05, BASE-06, PROTECT-01**.

One serial owner; expose raw shaft SI and explicit validity. No background command replay or telemetry-triggered CAN register polling.

Update the central entries with implementation/test evidence; do not duplicate
independent completion status here. Hardware acceptance is separate from unit tests.

The single-joint position port and keyboard pipeline now exist; see
[ARM_POSITION_JOG.md](../../docs/ARM_POSITION_JOG.md). Preserve its explicit
per-selection authorization, key-release stop and fault behavior when adding new adapters.
