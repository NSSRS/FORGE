# Arm hardware commissioning handoff

Updated 2026-11-05. This is a developer description, not an executable motor
profile. ENCOS is chassis; Damiao is arm. Never apply model joint poses directly
to uncalibrated hardware. See [current procedure](../../docs/DAMIAO_ARM.md).

## Current physical mapping (2026-11-05)

User identified DOF1=the tested 8009P, DOF2=the previously tested 4340P,
DOF3=the additional 4340P, DOF4=the tested 4310P, DOF5/6=the two additional
4310Ps. All six passed the saved 14-stage unloaded group/individual sequence.
See [ID assignment and connection order](../../docs/ARM_MOTOR_IDS.md) and
`config/arm_commissioning_inventory.json`. IDs 1..6 and feedback IDs 17..22 are written/read back. A1 post-power-cycle
persistence is verified; A2..A6 verification remains pending.
The older inventory below is historical; signs/zeros/loaded limits remain unset.

## Historical inventory and evidence (2026-11-03)

| Inventory | Status | Next action |
| --- | --- | --- |
| 1×DM-J8009P-2EC V1.1 | Not motion-tested | Discover alone on CAN1, record ranges and IDs; add bounded model-specific test |
| 2×DM-J4340P-2EC V1.1 | One unit tested alone on CAN2; physical joint mapping unknown | Repeat verified workflow for the second unit after discovery |
| 3×DM-J4310P-2EC | None motion-tested; user'sV1.1 hardware label unverified | Discover/test one unit at a time; use P-series manual, then verify all three |

Successful 4340P record: CAN2, commandID 1, replyID 0; PMAX12.5, VMAX10, TMAX28;
original mode 1; runtime timeout 100 ms; damping factor4. Position–velocity mode 2,
+45° relative target, 0.1 rad/s command: measured44.83° final, movement visible
by0.25 s, settled within 1° for 0.3 s, disable acknowledged, mode 1 restored.
Largest printed torque0.554 Nm, not a high-rate peak measurement. 3 Nm is a
feedback stop threshold, not a hard current/torque clamp. One unloaded test
does not validate motor torque ratings, loaded motion or full-arm control.

## Temporary wiring and planned final split

- Temporary: CAN1=8009P; CAN2=two 4340P+three 4310P; CAN3unused pending cable.
- Final: CAN1=8009P; CAN2=two 4340P; CAN3=three 4310P.
- Classic CAN 1 Mbps, target500 Hz per motor. Five motors at 500 Hz estimate
  56–68% bus load; throughput, latency and errors remain unmeasured.
- Profile IDs/feedback IDs now match programmed units, not factory defaults. Motor IDs must
  be unique on a shared bus before connecting multiple factory-configured units.
- 8009P has one separate H/L connector, no direct CAN pass-through. Smaller
  P-series motors have twoXT30(2+2) connectors. CAN3pins1/2 are5V/GND, not motor
  power; pins3/4 areH/L. Use separately rated motor power and bus-end termination.

## Prepared test tooling (2026-11-04)

The routed host/MCU implementation is now available in `scripts/test_dm_arm.py`.
See [single/full-arm commands](../../docs/DAMIAO_MOTOR_TESTS.md) and `bench_*.json`
templates. These use position-velocity mode, preserve CAN1 for the 8009P alone,
read actual ranges and restore modes. No additional hardware validation or
calibration is claimed; all new template identity/fixture flags remain false.

## Original next developer steps (2026-11-03; items 2/3 now implemented)


1. Save read-only discovery for the 8009P alone on CAN1 and first 4310P alone on
   its intended bus. Record exact model/revision, unit label, bus, command/reply
   IDs, PMAX/VMAX/TMAX, mode, timeout, damping and eventually firmware versions.
   Do not substitute non-P manuals or confuse manual and hardware revisions.
2. Generalize the verified bench path to an explicitly selected model/bus/ID,
   with matching firmware configuration, validated feedback decoding, positive
   damping, mode readback, disabled preloading, command/feedback watchdogs,
   mode restoration, and model-appropriate speed/travel/torque thresholds.
   Current firmware mode/timeout writes and bench motion are fixed toCAN2/ID 1;
   changing Python arguments or JSON alone cannot make it test other models.
3. Add fake-motor tests for each range/model, wrong identity/mode, missing
   replies, timeout, overspeed/torque and cleanup. Keep a single position–velocity
   bench implementation; do not reintroduce experimental MIT ramp variants.
4. Test each unit secured and unloaded, initially one at a time. Record actual
   startup, final error, settling, peak sampled speed/torque and disable/mode
   restoration. Extend angles/directions only within measured fixture limits.
5. Assign/verify distinct IDs with explicit commissioning steps, then measure
   multi-motor 500 Hz bus timing on temporary and final wiring. Diagnostics must
   stay out of active control traffic. No CAN-FD assumptions.
6. Map physical units to joints; measure sign, zero, travel, gravity/load and
   torque limits. Fill profiles only from evidence. Port position–velocity to
   multi-slot/ROS if that is the selected arm operating mode; current six-axis
   production interface is MIT and software-tested only.

## Description files

`config/arm_motors.json` and `config/arm_motors_final.json` remain deliberately
unconfirmed with null joint limits/signs/zeros. They describe temporary/final
wiring intent, not a commissioned robot. Do not mark `hardware_confirmed`,
`joint_order_confirmed` or `hardware_tested` true based on this one motor test.
`urdf/damiao_motor.xacro` identifies motor families; simulation geometry is not
encoder calibration. `config/chassis_motors.json` retains ENCOS classification.

## Removed experiments

Low-stiffness MIT bench ramps produced delayed breakaway and ~32° final motion
for 45° targets. The supported bench uses the motor's internal position–velocity
loops and unchanged internal gains. General MIT protocol support is retained
for the separate production interface, not exposed as a bench-test choice.

## Base-layer / upper-layer handoff

Read [base contract](../../docs/ARM_BASE_LAYER.md) and [cross-layer TODOs](../../docs/ARM_LAYER_TODOS.md) and local package TODO.md before extending control. Cached telemetry avoids additional CAN polling. Production ROS mode, calibration, current protection and loaded stopping remain separate acceptance tasks.
