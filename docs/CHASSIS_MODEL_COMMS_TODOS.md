# Chassis model and motor communication: open work

Reviewed 2026-11-15 against local `nathan`. Model import, four-wheel differential
mixing, encoder mirroring and dual-family cached ROS telemetry are implemented.
This list distinguishes implementation from physical qualification; remove entries
when their acceptance evidence exists. Arm ownership remains in
[the cross-layer registry](ARM_LAYER_TODOS.md).

| ID | Remaining work | Acceptance |
| --- | --- | --- |
| CHASSIS-01 | Compare updated CAD wheel geometry with independently configured controller radius/track, currently 0.040/0.737 m. | Automated mismatch detection; reviewed geometry revision. |
| CHASSIS-02 | Complete four-wheel ID, CAN slot, direction and external gearing mapping; current bench explicitly omits motor 4. | Per-wheel signed physical checks and a versioned full-chassis profile. |
| CHASSIS-03 | Qualify effective radius/track, slip, mass/inertia, magnetic/contact and actuator parameters. | Measured motion, stopping and load evidence; no promotion of illustrative dynamics. |
| CHASSIS-04 | Complete full ROS description/TF, sensor mounting transforms, odometry, fusion and closed-loop path tracking; coordinate seam-relative correction with [FEEDBACK-01](ARM_LAYER_TODOS.md#feedback-01--sensor-feedback-and-bounded-weld-tip-correction). | Measured frame/pose agreement and defined path/endpoint tolerances. |
| COMMS-01 | Establish per-motor ENCOS CAN freshness and bridge cache/DLC/TxPDO Toggle semantics. | Version-specific vendor or RX-source evidence plus independent missing/identical reply tests. See [vendor review](MOTOR_TELEMETRY.md#encos-vendor-evidence). |
| COMMS-02 | Measure ENCOS host/link-loss behavior, including possible replay of the last command. | Recorded stopping latency and restart inhibition with the final bridge firmware. |
| COMMS-03 | Qualify six-axis Damiao USB/CAN timing and fault behavior. | [BASE-04/07](ARM_LAYER_TODOS.md), including maximum gaps and actual command latency with diagnostics active. |
| COMMS-04 | Implement typed rejection/active configuration readback and atomic multi-axis targets. | [BASE-05/06](ARM_LAYER_TODOS.md); no subset motion or renewed lease from invalid batches. |
| COMMS-05 | Add chassis source arbitration/system safety management and production ROS arm lifecycle. | Explicit activation, stop, supported disable, recovery and re-arm; [ROS-02](ARM_LAYER_TODOS.md). |
| COMMS-06 | Complete both families' current/thermal/fault/load acceptance. | [PROTECT-01/SUPPORT-01](ARM_LAYER_TODOS.md) and independent current/stop evidence. |

Prioritize geometry/configuration consistency, full wheel mapping and
feedback/host-loss qualification before localization and coordinated trajectories.
Welding motion implementation belongs under `src/controls/controls/hardware/`;
its offline tests do not retire the physical acceptance items above.
