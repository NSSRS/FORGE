# Three-motor CAN2 chassis bench

The current control profile is `src/forge_motor_control/config/chassis_bench.yaml`:
IDs 1/2 on CAN1, ID 3 on CAN2, and motor 4 explicitly omitted. Use W/S/A/D or
one proportional radio stick through the same chassis command path.

- [Driver and W/S/A/D commands](DIFFERENTIAL_DRIVE.md#three-motor-differential-bench-motor-4-absent)
- [ESP32 radio input and calibration](RADIO_CONTROL.md)
- [Four-pane motor test and MuJoCo feedback](MOTOR_MIRROR.md)

The current bench requests 0.08 m/s forward/reverse and 0.28 rad/s yaw, with a
20 RPM per-motor ceiling and the provisional 2 A phase-current ceiling. Initial
position feedback from all three present motors is required at startup. Motor 4
is never synthesized or silently removed after a fault.

`can2_bench.yaml` remains a legacy direct-RPM driver profile with a 5 RPM ceiling.
It was used for the operator-confirmed 2026-09-29 CAN2 test; the teleop no longer
accepts it. The historical result remains in the
[bench record](ENCOS_TEST_BENCH.md#can2-bench-confirmation-2026-09-29).
Stop previous driver and teleop processes before changing profiles. Power off
before moving CAN wiring and verify termination on each separate bus.

## Slot routing

`motor_slots: [0, 1, 3]` explicitly maps the three motors to zero-based PDO slots.
Slots 0-2 route to CAN1, slots 3-5 to CAN2, independent of motor ID. The native
mapped-open API keeps the original ABI and contiguous defaults available.
The output span reaches slot 3 (`Motor_Num=4`); vacant slot 2 remains empty
(ID=0, DLC=0, no payload). It does not contain a query or an eight-byte motor
control packet. This layout was used for the operator-confirmed CAN2 test.
The fake bridge verifies software routing, forward/reverse/stop, and empty gaps;
it models replies for the same CAN ID replacing each other to catch accidental
query traffic masking control feedback. It does not establish bridge firmware behavior.

The initial split-bus implementation filled slot 2 with position queries for ID 1.
That can mask motor 1's control acknowledgements and reproduce the reported
250 ms feedback timeout. Removing that filler fixes the reproduced failure
without a startup delay or a longer watchdog. The subsequent operator test
confirmed motor 3 working on CAN2; no raw CAN capture was supplied to establish
the exact firmware reply-replacement behavior.

`encos_query` currently sends through slot 0/CAN1, even with several IDs listed;
it cannot verify ID 3 on CAN2. Use the chassis driver startup linked above before starting any teleop publisher.
