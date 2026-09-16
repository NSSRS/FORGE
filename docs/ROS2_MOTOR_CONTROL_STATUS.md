# ROS 2 motor-control status

## Current scope

ROS 2 Jazzy is installed on the Ubuntu 24.04 NUC. The
`forge_motor_control` ROS package provides a direct-RPM motor driver node and a
dead-man keyboard publisher for commissioning. Chassis kinematics, geometry,
URDF, and autonomous control have not been added.

The temporary commissioning topic is:

```text
/drive/motor_rpm_commands  std_msgs/msg/Float64MultiArray
```

The array order is motor IDs 1, 2, and 3. The motor driver publishes output-shaft
position and velocity on `/joint_states`. Positions use radians and velocities
use radians per second, as required by `sensor_msgs/msg/JointState`.

The keyboard node is a publisher. The hardware node is both a subscriber (RPM
commands) and a publisher (encoder states). `W` and `S` command all three motors
in the same protocol direction; Space or `X` stops, and `Q` or Escape exits.
Nonzero commands expire when key-repeat stops. `A` and `D` are intentionally not
implemented because left/right kinematics depend on the future robot geometry,
wheel directions, and transmission configuration.

Terminal input reports key presses but not key releases. To avoid an initial
motion pulse followed by a deadman stop during Linux's key-repeat delay, the
teleop waits for the second matching `W` or `S` event before publishing motion.
Held-key repeat then refreshes commands normally; releasing the key retains the
0.15-second deadman stop. A single tap does not move the motors.

## Build and run

```bash
cd /home/forge2/forge2_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base ros2_ws/log build --base-paths ros2_ws/src \
  --build-base ros2_ws/build --install-base ros2_ws/install --symlink-install
```

Hardware driver terminal:

```bash
cd /home/forge2/forge2_ws
sudo ./scripts/run_ros_motor_driver.sh --execute
```

Keyboard terminal:

```bash
sudo bash -lc '
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
exec ros2 run forge_motor_control forge_keyboard_teleop
'
```

To test the ROS motor driver without the keyboard node, leave the driver running
and execute the guarded automatic publisher in a second terminal:

```bash
sudo bash -lc '
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
./scripts/run_ros_motor_command_test.py --execute --rpm 5
'
```

It requires fresh `/joint_states` data before moving, publishes equal targets
for motors 1-3 for exactly two seconds, then publishes zero for one second and
requires all three reported speeds to fall below 0.5 RPM. The default is +5 RPM;
signed values up to +/-30 RPM are accepted.

The hardware driver stays query-only until its first valid RPM message. If
messages stop for 0.20 seconds, it commands zero RPM. The native 0.25-second
command lease and the verified motor-side 500 ms CAN timeout remain additional
stop layers.

The driver needs root for raw EtherCAT access. On this host, Fast DDS endpoint
discovery crosses the root/non-root boundary but payload delivery is unreliable,
even with UDP-only loopback configuration. Run the temporary commissioning
driver, automatic publisher, and keyboard teleop under the same root account.

## Bench results and limitations

The secured, unloaded three-motor path passed bounded native Python velocity,
automatic ROS command/feedback, and keyboard commissioning tests. The complete
[2026-09-16 commissioning record](ENCOS_TEST_BENCH.md#commissioning-record-2026-09-16)
preserves the initial fault, subsequent diagnosis, measured values, firmware
discrepancy, operator decision about unknown brake status, and test limits.
Loaded stopping and host-loss behavior remain unverified.
