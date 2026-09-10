# ENCOS motor bring-up on Ubuntu 24.04

Prepared 2026-09-08 from ENCOS V1.19EAP and the supplied CAN/EtherCAT materials. This is a design and commissioning guide, not a hardware-tested driver. Motor model, firmware, adapter, joint count and mechanical loading remain unknown. Page references below use the manual's printed page numbers (PDF page = printed page + 5).

## Recommended starting architecture

Updated after hardware identification: use NUC native Ubuntu 24.04 -> dedicated Ethernet -> the purchased ENCOS EtherCAT-CAN bridge -> motors. See ENCOS_TEST_BENCH.md for the step-by-step procedure. A separate USB-CAN adapter is not required for this route. The SocketCAN instructions later in this document apply only to the alternative USB-CAN route. Use classic CAN at 1 Mbps on the motor side, ENCOS request/response communication, and servo position control (mode 1).

Start with one mechanically secured, unloaded motor. The URDF is not needed to establish communication or test a small output-shaft movement. For the assembled robot, add a joint mapping, calibrated offsets and directions, physical limits, trajectory generation, and fault handling before movement.

ROS 2 Jazzy is a suitable Ubuntu 24.04 integration target. Proposed stack:

1. Planner / later MoveIt integration.
2. joint_trajectory_controller receiving FollowJointTrajectory goals.
3. Custom ros2_control SystemInterface with position commands and measured position/velocity states.
4. ENCOS packet codec and bounded CAN transmit/receive worker.
5. SocketCAN transport, or an optional EtherCAT transport.

Servo mode's velocity field is a nonnegative speed ceiling, not signed velocity feedforward. Initially expose a position command interface and configure this ceiling separately. Do not blindly map a ROS velocity command into that field. The trajectory controller interpolates positions over time; motor speed/acceleration limits can still introduce lag, which must be measured.

## Control mode choice

| Mode | Inputs | Recommended role |
|---|---|---|
| 1: servo position | Output position in degrees, speed ceiling in rpm, phase-current ceiling in A | First motion, holding, ordinary joint trajectory tracking |
| 0: force-position hybrid | Position rad, velocity rad/s, Kp, Kd, feedforward torque Nm | Compliant interaction, legged control, dynamic model-based tracking |
| 2: servo velocity | Output speed rpm and phase-current ceiling | Velocity-controlled applications; not the initial joint-position interface |
| 3: current/torque/braking | Signed current or torque, or brake submode | Only after outer-loop and stopping behavior are designed |

The motor closes its inner loops locally. Hybrid control implements approximately tau = Kp*(q_des-q) + Kd*(dq_des-dq) + tau_ff, then I = tau/Kt. Gravity-loaded hybrid position holding generally has static error without adequate feedforward. Hybrid gains are not interchangeable with servo-loop gains. The manual's unloaded Kp=15, Kd=0.5 is a reference, not a universal robot tuning prescription.

Do not substitute a generic MIT motor driver: ENCOS hybrid packing includes a 3-bit mode, 12-bit Kp and 9-bit Kd. It is not the common alternative MIT packet layout. No generic FF...FC enable command is documented here. After boot, valid control packets directly select/control the motor; a fitted holding brake must also be handled.

## Hardware and electrical preparation

- Correct model-specific motor harness and verified pinout: power positive/negative and CAN H/L. Do not infer connector pin numbering from wire color or a photo.
- Suitable motor supply: manual operating range 20-57 V, with 57-60 V overvoltage warning and possible damage above 60 V. Choose nominal voltage and continuous/peak capacity from the actual motor and robot load. Brake-equipped motors need adequate voltage to release the 24 V brake.
- Design for regenerative energy. The manual prefers a battery under load; a bench supply may not absorb returned energy. Select a suitably engineered absorption/clamp arrangement where required. Phase current is not equal to supply current.
- Fusing, appropriately rated wiring/distribution, mechanically secured motors and an accessible hardware stop. The manual lists XT30 connector continuous rating as 15 A and peak 30 A; do not daisy-chain aggregate robot power without checking connector/wire loading.
- CAN linear trunk with short stubs and a 120-ohm termination at each physical end. Count built-in termination first; do not install one resistor per motor. With everything unpowered, two parallel terminators normally measure about 60 ohms H-to-L. Follow adapter grounding/isolation requirements.
- Support gravity-loaded joints before brake release or torque removal. Communication timeout is not a guaranteed position hold or mechanical brake.
- No live power-connector plugging; wait for the power indicator to extinguish before rewiring.

## Windows commissioning

Use the supplied MotorTool with the matching ENCOS USB-CAN module to establish one known-good motor configuration. Record the motor's exact model, firmware/hardware versions, CAN ID, zero, protocol ranges, current limits, timeout and brake status. Assign unique IDs one motor at a time before assembling a shared bus. Preserve factory calibration and loop gains initially.

The supplied VESC tool is an optional vendor configuration path, not a required Linux runtime component. Avoid mixing active USB and CAN control. Vendor tuning tools may use different units (including ERPM), so do not transfer values without conversion. Do not flash generic VESC firmware.

## Ubuntu interface and read-only discovery

These commands are for the alternative USB-CAN route on this Ubuntu NUC. The primary EtherCAT route does not expose can0. They assume a driver-supported adapter already exposes can0 and the motor is configured for classic CAN at 1 Mbps:

```bash
sudo apt update
sudo apt install can-utils iproute2 build-essential cmake
ip -br link
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.765 berr-reporting on
sudo ip link set can0 up
ip -details -statistics link show can0
candump -tz can0
```

The manual specifies a 76.5% arbitration sample point. Check the actual timing reported by the adapter; not every controller can realize every setting exactly. Bus-off recovery should require the application to remain disarmed until the cause is understood and valid fresh state is recovered. Do not automatically resume movement after reconnecting.

In another terminal, with exactly one motor on the bus and at least 5 seconds after power-up:

```bash
# Discover its ID; replies use arbitration ID 7FF.
cansend can0 7FF#FFFF0082

# Following requests assume discovery established motor ID 1.
cansend can0 001#E001
cansend can0 001#E01E
cansend can0 001#E01F
```

These ask for position, firmware/hardware version (decimal 30), and CAN timeout (decimal 31). The manual's example uses E1 01 while the supplied code uses E0 01; section 9.3 calls the low five bits reserved/currently ineffective. Verify the reply from the installed firmware. A position reply has type 5 in byte 0, query code 1 in byte 1, and a big-endian float32 angle in degrees in bytes 2-5. The ID query reply is FF FF 01 ID_hi ID_lo.

Do not run broadcast discovery/reset on an assembled bus with unknown duplicate IDs. All matching motors can respond or be reconfigured. ID changes and zero/communication-mode settings require over 500 ms before the next transmitted CAN command according to pp. 4-5. Perform these before arming, not in the live control loop.

## Servo position packet

Arbitration ID = motor ID (1..0x7FE); standard CAN data frame; DLC 8. Multibyte data is high-order first.

| Field, in wire order | Width | Encoding |
|---|---:|---|
| Mode | 3 bits | 1 |
| Position | 32 bits | IEEE-754 float32, output angle in degrees |
| Speed ceiling | 15 bits | rpm * 10 |
| Phase-current ceiling | 12 bits | A * 10 |
| Requested feedback | 2 bits | 0 none; 1 compact state; 2 float position; 3 float speed |

An unambiguous packing expression is:

```text
P = IEEE754_float32_bits(position_deg)
S = quantized(speed_ceiling_rpm * 10)
C = quantized(current_ceiling_A * 10)
payload_u64 = (1 << 61) | (P << 29) | (S << 14) | (C << 2) | ack
payload = payload_u64 encoded as eight big-endian bytes
```

Validate finite inputs, field widths and actual motor/mechanical limits before packing. Protocol representable maxima are not safe motor operating limits. Reject invalid commands instead of allowing overflow into adjacent fields.

Manual regression vectors (for codec verification, not initial movement commands):

- 0 degrees, 20 rpm, 5 A, ack 2: `20 00 00 00 00 32 00 CA`.
- 90 degrees, 20 rpm, 5 A, ack 2: `28 56 80 00 00 32 00 CA`.
- -90 degrees, 20 rpm, 5 A, ack 2: `38 56 80 00 00 32 00 CA`.

For first control, read the actual position and initialize the target to it. Only then stream a small bounded trajectory with model-appropriate low speed/current. Do not start by commanding zero or by replaying a 90-degree example. Verify motion direction and feedback against physical movement.

## Feedback and hybrid encoding

Normal control feedback arrives on the same arbitration ID as the motor command. Validate ID, frame flags, DLC and expected packet type; exclude local transmit echoes from motor state. For normal response frames: type = byte0 >> 5, error = byte0 & 31. Handle special configuration replies separately, including FF FE prefixes.

- Type 1: position uint16 in bytes 1-2; velocity uint12 from byte3 and upper nibble of byte4; phase current uint12 from lower nibble of byte4 and byte5; winding temperature byte6; MOS temperature byte7. Manual lists position +/-12.5 rad and velocity +/-18 rad/s. Current range is model-dependent. Because hybrid ranges can be configured while this feedback section repeats fixed position/velocity ranges, validate feedback scaling for the actual firmware rather than assuming configuration changes affect both directions identically.
- Type 2: big-endian float32 output position in degrees, signed int16 current divided by 100 A, temperature. Good initial position feedback choice. Obtain velocity via scheduled type-3 replies or filtered differentiation with documented latency/noise.
- Type 3: big-endian float32 output speed in rpm, signed int16 current divided by 100 A, temperature.
- Temperature encoding for these control frames is (raw - 50) / 2.0 degrees C. Preserve signed and fractional results.
- Error values 1..7 identify overheating, overcurrent, overvoltage, undervoltage, encoder fault, brake overvoltage and driver fault.

For hybrid mode, quantize each configured range using u = floor((x-xmin)*(2^bits-1)/(xmax-xmin)), after bounds checking. The ordered bit widths are mode 3, Kp 12, Kd 9, position 16, velocity 12, feedforward torque 12. It always returns type 1. Query the actual ranges (query codes 23..28 decimal) before use. Do not use the old code's universal +/-30 Nm and +/-30 A assumptions.

## Timing and bus sizing

Start one motor at 100 Hz; consider 250-500 Hz trajectory updates after measuring latency, jitter and tracking. These are engineering starting points, not promises. The manual advertises up to 2 kHz and earliest feedback around 0.4 ms; this is not an all-motor rate guarantee.

Budget roughly 130 bits per classic 8-byte standard frame including stuffing/overhead for initial planning (actual traffic varies). With one command and one reply per motor, estimated utilization is N * f * 260 / 1,000,000:

| Motors on one bus | Per-motor update | Approximate utilization |
|---:|---:|---:|
| 6 | 250 Hz | 39% |
| 6 | 500 Hz | 78% |
| 6 | 1000 Hz | 156%, impossible |
| 3 | 500 Hz | 39% |

Leave headroom for configuration/diagnostics, arbitration, retransmissions and scheduling. Target roughly 50-60% during initial design, then verify worst-case behavior. Use additional independent CAN channels when needed. Command acknowledgments are ENCOS payload replies, distinct from CAN's hardware ACK bit.

Use a monotonic fixed-period worker, bounded queues, timestamps, per-motor freshness and command deadlines. Avoid waiting for every reply serially in a blocking joint loop. Track bus errors, maximum feedback age, following error, loop jitter and missed deadlines. CAN sends joints sequentially; these packets contain no shared execution timestamp. Scheduling all joint targets for the same host cycle does not make their actuation simultaneous.

The default watchdog is 500 ms. Query and test it. Choose a shorter nonzero timeout only after measuring scheduling margins and validating the mechanical stopping behavior. Configuration code 0x0B sets milliseconds; `C0 0B 01 F4` means 500 ms. Zero disables protection. A process watchdog and coordinated robot stop are needed as well; stopping CAN transmission may remove holding torque. Loss of any critical joint's fresh feedback should abort coordinated motion.

## Optional EtherCAT route

The supplied bridge provides two CAN buses, three command slots per side, six motors total; passages 1-3 go to CAN1 and 4-6 to CAN2. The bridge document states up to 1 kHz EtherCAT cycles and tests with 20 chained boards. It has onboard CAN termination; account for that in each bus. Supply the bridge separately at 5 V with at least 500 mA provision per board. This is not the motor supply voltage.

Use a dedicated NUC Ethernet interface -> ENCOS EtherCAT-CAN bridge -> motors. The supplied SOEM and IgH examples can inform a transport implementation. The IgH README targets Ubuntu 20.04 and old install instructions; compatibility with the Ubuntu 24.04 kernel must be checked. Neither example should be run unreviewed because its loop can issue movement commands immediately.

The bridge sequentially transmits CAN slots, so EtherCAT does not imply simultaneous motor execution. Do not confuse this bridge with native EtherCAT motor drives or assume a CiA-402 interface. CANopen is explicitly marked not enabled in this manual. CAN FD is documented at 1 Mbps arbitration / 5 Mbps data, but requires confirmation of motor firmware and every adapter/bridge on that segment; do not assume the supplied bridge supports FD.

## Precision, calibration and limits

The manual states 14-bit magnetic encoders with approximately 0.1-degree best measured angle accuracy and some units around 0.2 degree (p. 4). This is distinct from digital resolution: compact position encoding across 25 rad has about 0.0219-degree steps. Float position commands do not improve physical encoder accuracy.

At 0.5 m reach, a single 0.1-degree angular error contributes about 0.87 mm transverse error; 0.2 degree contributes about 1.75 mm. These are illustrative small-angle calculations, not a complete robot accuracy estimate. Multiple joints, backlash, compliance, thermal effects and geometry errors contribute; later propagate joint uncertainty through the robot Jacobian. Submillimeter absolute tool positioning may require calibration and external measurement.

The output encoder is single-turn absolute across restart. Powered operation tracks turns, but the turn count resets after power loss. Establish an unambiguous startup reference for joints that can cross turns. Do not reuse a saved turn count if a joint might move while unpowered; use homing/reference sensing or a verified restricted joint range.

For motor-output angle theta, external transmission ratio r = motor-output angle / joint angle, direction s = +/-1, and motor-output zero theta0:

```text
q_joint = s * (theta - theta0) / r
theta_command = theta0 + s * r * q_joint_command
```

Convert radians/degrees at the protocol boundary. ENCOS feedback is already output-shaft position: do not divide by the motor's internal gearbox ratio again. Use r=1 for a direct output-to-joint connection. Map velocity and torque consistently with external transmission and efficiency; phase current alone is not a calibrated joint torque measurement.

## Problems to fix before reusing the example code

1. `can_rv.c` uses STM32 CAN_Transmit/CanRxMsg APIs, not a Linux transport.
2. Hybrid torque/current ranges are hardcoded. Table 9-1 has different model ranges; its note also says planetary models shipped after 2025-01-15 default to Kd range 0..50, despite several table cells showing 0..5. Query the actual motor and confirm firmware behavior.
3. Receiver indexes an eight-element array with CAN ID minus one without validating the ID; it truncates IDs into uint8 and does not consistently validate DLC.
4. Temperature storage loses sign/fraction, and the type-1 decoder omits MOS temperature.
5. Old configuration code 0x02 means linkage/speed-KI in the sample but CAN/CAN-FD selection in V1.19; 0x03 is also an old gain-setting path. Do not send legacy configuration commands.
6. Current units differ: servo position/speed current ceilings use 0.1 A units; direct current commands and type-2/3 current feedback use 0.01 A units.
7. The manual itself has inconsistencies: its p. 43 hybrid worked binary example does not apply the specified range quantization; some configuration DLC/ACK descriptions conflict with examples (notably acceleration, communication mode and zero offset). Treat those as firmware-validation items, not ready-to-run recipes. Thermal descriptions on pp. 2-3 also differ; establish conservative model-specific operating limits with ENCOS.

## Acceptance sequence and remaining inputs

1. Single motor: wiring/termination/power checks; discovery and repeatable telemetry; record settings.
2. Secured motor: measured-position initialization, brake handling if fitted, tiny bounded moves, direction/units/current verification.
3. Controlled fault tests: missing command, stale feedback, process exit and CAN disconnect, with the mechanism supported. Verify actual coast/brake/hold behavior.
4. One robot joint: reference/zero procedure, software and physical limits, loaded step/ramp tracking and settling measurements.
5. Multiple joints: shared time-based trajectory, bus-load and worst-case feedback measurements, following-error aborts, hardware-stop checks.
6. URDF integration: validate axes, signs, origins, limits and transmissions; add robot planning only after the hardware interface reports truthful state.

Needed next: exact model per joint; motor count; firmware; adapter/bridge model; brake presence; supply; robot type and load; required angular or tool-position accuracy; motion speed/update target; external gearing; joint limits and homing method. URDF/YAML will supply geometry and configuration, but mass/inertia/center-of-mass and payload are also needed for dynamic feedforward.

Sources: ENCOS V1.19EAP pp. 4-8, 16, 23-44 and 45-48; supplied demo code/can_rv.c and .h; EtherCat-CAN bridge document sections 2, 4-6; SOEM/IgH archive READMEs and config headers. Windows executables were inventoried, not executed; EtherCAT examples were inspected, not compiled or hardware-tested.

External references: [Linux SocketCAN](https://www.kernel.org/doc/html/latest/networking/can.html), [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), [Jazzy joint trajectory controller](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).
