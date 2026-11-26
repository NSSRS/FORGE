# Teleoperation: open work

Use the [consolidated registry](../../docs/ARM_LAYER_TODOS.md).

- **ROS-02 (deferred by the operator on Nov 26):** connect teleoperation to the production lifecycle and shared
  BASE-07 stop/recovery behavior.
- **SPEED-01:** broader manual-jog investigation is deferred; the selected
  holding/teaching speeds are already implemented.

Preserve fresh intent, input deadlines, cancellation during enable, recovery
staying disarmed and the existing release/hold behavior. GUI telemetry must not
create another owner or command heartbeat.
