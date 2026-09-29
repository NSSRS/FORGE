# Three-motor CAN2 bench

Use `config/can2_bench.yaml` in the ROS package for IDs 1 and 2 on CAN1 and
ID 3 on CAN2. Motor 4 is absent. This is direct-RPM commissioning, not a
three-wheel differential chassis. For W/S/A/D testing with motor 4 absent, use
[the differential bench profile](DIFFERENTIAL_DRIVE.md#three-motor-differential-bench-motor-4-absent).
The default four-wheel chassis profile is unchanged.

Stop existing driver and keyboard processes. Power off before moving motor 3's
CAN harness to CAN2; verify termination on both separate buses. Keep the motors
secured and unloaded, with the hardware stop available. The profile uses a
5 RPM ceiling and the earlier provisional 2 A phase-current limit.

Rebuild from `/home/forge2/forge2_ws`:

```bash
./scripts/build.sh
source /opt/ros/jazzy/setup.bash
colcon --log-base logs/colcon build --base-paths src/forge_motor_control \
  --build-base build/ros2 --install-base install/ros2 --symlink-install
```

Driver pane (query-only until a command arrives):

```bash
sudo bash -lc '
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
export ROS_DOMAIN_ID=44
ros2 run forge_motor_control forge_motor_driver --ros-args \
  --params-file src/forge_motor_control/config/can2_bench.yaml \
  -p simulate:=false -p execute:=true
'
```

Feedback pane:

```bash
sudo bash -lc '
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
export ROS_DOMAIN_ID=44
ros2 topic echo /joint_states sensor_msgs/msg/JointState
'
```

Successful startup requires initial positions for all three motors. Check for
`motor_3_output` feedback before starting teleop. Initial velocity can be absent
while query-only; position feedback is the startup criterion.

Keyboard pane:

```bash
sudo bash -lc '
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
export ROS_DOMAIN_ID=44
ros2 run forge_motor_control forge_keyboard_teleop --ros-args \
  --params-file src/forge_motor_control/config/can2_bench.yaml
'
```

Hold W/S to request +5/-5 RPM on **all three motors**, release to request a stop.
A/D are not enabled in direct-RPM mode. Space/X requests zero speed; it is not a
hardware emergency stop. Verify motor 3 physically moves and stops as expected.
On 2026-09-29, the operator confirmed motor 3 working on CAN2 after the
empty-slot fix. See the [bench record](ENCOS_TEST_BENCH.md#can2-bench-confirmation-2026-09-29).

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
it cannot verify ID 3 on CAN2. Use the query-only startup above instead.
