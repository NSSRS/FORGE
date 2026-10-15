# ROS 2 motor-control status

The active control path is chassis W/S/A/D, using `keyboard_teleop` with
keyboard or proportional single-stick radio input. Both publish
`geometry_msgs/msg/TwistStamped` on `/cmd_vel`; `motor_driver` performs
differential mixing and owns MotorBus, C11/SOEM and the EtherCAT-CAN bridge.
See [radio calibration](RADIO_CONTROL.md) for ESP32 input.

## Current bench

`src/bringup/config/chassis_bench.yaml` configures IDs 1/2 on CAN1
(slots 0/1) and ID 3 on CAN2 (slot 3). Motor 4 is explicitly absent. Four-wheel
mixing is retained; only the three present motors are opened and commanded.
Forward wheel signs are `[-1,-1,+1,+1]` in the configured wheel order.

| Setting | Current bench value |
|---|---|
| Forward/reverse request | +/-0.08 m/s |
| Left/right yaw request | +/-0.28 rad/s |
| Per-motor ceiling | 20 output-shaft RPM |
| Provisional phase-current ceiling | 2 A per motor |
| Chassis command lease | 0.15 s |
| Native velocity slew | 60 RPM/s |
| Native command lease | 0.25 s |

The complete four-motor `chassis.yaml` is a different profile: 0.02 m/s,
0.07 rad/s and a 30 RPM ceiling. Use the same selected profile for teleop and
driver. Combined forward/turn requests are scaled together to fit the motor
ceiling. Physical wheel assignments and signs still need commissioning.

## Build and run

```bash
cd /home/forge2/forge2_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base logs/colcon build --base-paths src \
  --build-base build/ros2 --install-base install/ros2 --symlink-install
```

For the current four-pane bench workflow, see [motor visualization](MOTOR_MIRROR.md):
`./scripts/run_motor_test.sh --simulate` runs without motor hardware;
`./scripts/run_motor_test.sh` starts the configured hardware session.
For separate driver and teleop terminals, use the commands in
[differential bench testing](DIFFERENTIAL_DRIVE.md#three-motor-differential-bench-motor-4-absent).
Radio input replaces the keyboard process; never run both publishers together.
The launcher still selects keyboard input by default.

W/S request forward/reverse; A/D request left/right pivots. Space/X requests zero;
Q/Escape requests zero and exits. Terminal input has no key-release event, so a
second matching key event starts motion and key-repeat renews it. Release stops
the request after 0.15 s. Radio input instead uses continuously proportional
axes, with both axes of one stick controlling the entire chassis.

The driver publishes `/joint_states`: output-shaft position in radians and
velocity in radians/second, named `motor_<id>_output`. It stays query-only until
its first valid command. Chassis messages require a fresh nonzero timestamp,
`frame_id=base_link`, and only linear.x/angular.z nonzero. Source expiry requests
zero speed; native fault handling is a separate layer. Neither promises an
instantaneous mechanical stop or holding brake.

Hardware needs root for raw EtherCAT. On this host, mixed root/non-root DDS
payload delivery was unreliable despite discovery succeeding. Use the same
root account, `scripts/ros2_env.sh`, and ROS domain 44 for hardware driver and
teleop. Simulation can run as the normal user on an isolated domain.

## Legacy commissioning and evidence

The driver retains `command_mode: direct_rpm` on
`/drive/motor_rpm_commands` (`std_msgs/msg/Float64MultiArray`) for the existing
automated commissioning tools. `commissioning.yaml` and `can2_bench.yaml` are
legacy driver-only profiles. `run_ros_motor_driver.sh` still selects the former;
`run_ros_motor_command_test.py` publishes direct RPM and cannot drive a chassis
mode subscriber. These are not the current WASD/RC workflow, and the teleop
explicitly rejects direct-RPM mode.

The [2026-09-16 bench record](ENCOS_TEST_BENCH.md#commissioning-record-2026-09-16)
preserves the fault investigation and bounded unloaded motor tests. The
[2026-09-29 CAN2 result](ENCOS_TEST_BENCH.md#can2-bench-confirmation-2026-09-29)
is operator-confirmed. Those historical results are not radio, complete chassis,
loaded-stop or host-loss validation. URDF, odometry, trajectory tracking,
command-source arbitration and a safety supervisor remain future work.

## IMU sensor node

The independent `imu` package provides `/imu` via `./scripts/run_imu.sh`, reading the
HWT9052 directly from Ubuntu serial. It publishes `/imu/data_raw` (Imu),
`/imu/data_rpy` (Vector3Stamped, x/y/z = roll/pitch/yaw in radians), and
`/imu/mag` (MagneticField, tesla). Standard Imu orientation is currently marked
unavailable pending coordinate-frame validation. The launcher defaults to
domain 42; match domains explicitly when integrating with other robot nodes.
The standalone `scripts/read_imu.py` prints degrees but publishes no topics.
Only one reader should own the IMU port. See [IMU setup](IMU_SETUP.md) for the
complete interface and verified status. Neither reader starts calibration.
