# Chassis model and motor communication: open work

Updated 2026-11-26. Chassis testing is deferred by the operator until assembly.
Four-motor identification, signed motion/stops, wheel mapping and isolated reply
loss/recovery are already recorded in the [handoff](ARM_SESSION_HANDOFF.md).
Do not repeat commissioning for routine use.

## Work after assembly

| ID | Concrete remaining work |
| --- | --- |
| CHASSIS-03 | Measure effective wheel radius/track and slip, and replace illustrative mass/contact/actuator values where needed for actual motion. |
| CHASSIS-04 | Complete ROS description/TF and sensor mounts, odometry/fusion, command-source arbitration and closed-loop path tracking. Includes the chassis part of former COMMS-05; coordinate seam sensing with FEEDBACK-01. |

The [combined four-motor Pocket/IMU/camera check](FULL_SYSTEM_TEST.md#select-the-four-motor-bench)
is also deferred. When resumed, use the explicit four-motor profile and its
secured unloaded 5 RPM / 2 A limits; no new ID assignment is needed.

## Deferred and shared work

- **COMMS-02:** host/link-loss stopping and restart behavior remain deferred by
  operator request on Nov 26. USB-debug/CAN-logger investigation is removed from
  the immediate plan. The retained [bench procedure](CAN2_BENCH.md) is reference
  material for an explicitly resumed session, not the next task.
- **PROTECT-01:** current/thermal/load testing is deferred by the operator.
  Former COMMS-06 is tracked once in the [arm registry](ARM_LAYER_TODOS.md).
- Damiao command/configuration deployment **BASE-06** (formerly COMMS-04) is complete;
  the selected stationary **BASE-07** command-loss/recovery check (formerly
  COMMS-03) is [recorded](ARM_SESSION_HANDOFF.md#base-07-command-loss-check).
- Arm production lifecycle is **ROS-02**, separate from chassis assembly.

Use the canonical [arm registry](ARM_LAYER_TODOS.md) for these shared entries.
There is no separate communications qualification checklist duplicating them.
