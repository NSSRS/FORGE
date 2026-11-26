# Bringup and deployment: open work

Use the [consolidated registry](../../docs/ARM_LAYER_TODOS.md) and
[current handoff](../../docs/ARM_SESSION_HANDOFF.md).

- **ROS-02 (deferred by the operator on Nov 26):** integrate the production arm
  lifecycle with explicit re-arm.

BASE-06 deployment and the supported stationary BASE-07 check are complete;
see the handoff for their scope. Chassis testing and PROTECT-01 are deferred. Preserve radio discovery before
node startup and exclude unrelated devices. Rig profiles belong in config/ and
setup.py; generated reports belong under build/.
