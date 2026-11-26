# Python motor APIs: open work

Use the [consolidated registry](../../docs/ARM_LAYER_TODOS.md) for status and
[base contract](../../docs/ARM_BASE_LAYER.md) for API details.

- **CAL-01:** raw/model coordinate integration for the next selected operating range.
- **ROS-02:** production lifecycle integration is deferred by the operator on Nov 26.

SPEED-01 tuning and PROTECT-01 testing are deferred. Completed motor readbacks,
range setup, BASE-06 atomic deployment and the stationary BASE-07 check are not
pending tests; see the
[handoff](../../docs/ARM_SESSION_HANDOFF.md#base-07-command-loss-check).
Telemetry continues to consume cached
owner data without CAN polling or lease renewal.
