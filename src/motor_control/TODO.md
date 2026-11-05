# ROS motor control: cross-layer arm/motor TODOs

Start with [cross-layer registry](../../docs/ARM_LAYER_TODOS.md) and
[base API contract](../../docs/ARM_BASE_LAYER.md). Relevant stable IDs: **ROS-01, ROS-02, CAL-01**.

Consume cached APIs on the existing owner. Current ROS MIT mode is not validated by the position-velocity bench.

Update the central entries with implementation/test evidence; do not duplicate
independent completion status here. Hardware acceptance is separate from unit tests.

The single-joint position port and keyboard pipeline now exist; see
[ARM_POSITION_JOG.md](../../docs/ARM_POSITION_JOG.md). Preserve its explicit
per-selection authorization, key-release stop and fault behavior when adding new adapters.
