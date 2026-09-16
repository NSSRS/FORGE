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

## First ROS hardware run (2026-09-16)

The driver reached hardware-ready state on `enp86s0` for IDs 1, 2, and 3. When
the command path became active, the motors shook and the native driver latched
fault class 3 (`feedback/limit failure`). The session closed without automatic
re-arm. Powered testing is paused pending read-only telemetry and identification
of the specific motor/error/current/feedback condition.

The next read-only query returned fault-free position, hardware, version, and
500 ms timeout replies from all three motors. Positions were -151.493652 degrees
(ID 1), 69.675499 degrees (ID 2), and 2.895467 degrees (ID 3). Brake replies were
unknown. All three reported software 1.0.0, which conflicts with the first
preflight's 1.0.8 and 1.0.32 values for IDs 1 and 3. The discrepancy may indicate
cached or otherwise unverified bridge replies and must not be treated as proof
of firmware identity.

Motors 1, 2, and 3 then each passed the native isolated motion test
independently: +1 degree over two seconds, return over two seconds, a 2 A
phase-current ceiling, and final position near the measured start. This
establishes individual low-speed position control but does not yet clear
simultaneous three-motor velocity control.

All three motors subsequently passed the same +1 degree / return test
simultaneously with a 2 A phase-current ceiling per motor. ROS DDS is now forced
to loopback with `ROS_LOCALHOST_ONLY=1` so discovery and topic traffic do not use
the dedicated EtherCAT interface. The native driver now prints the exact motor
and measured condition before latching a fault.

The following ROS keyboard run reported the exact cause: motor 2 reached
720.049 degrees, outside the configured [-720, 720] degree commissioning range.
This was a software position-limit stop, not a reported current, temperature,
EtherCAT, or motor fault. Output-shaft position accumulates across turns while
the motor remains powered, so continuous wheel velocity control requires a
continuous-joint policy instead of the bounded-position policy used for initial
bench motion.

The driver now implements that policy: Mode 1 position control retains angular
bounds, while Mode 2 velocity control permits continuous accumulated output
rotation. Speed, phase-current, temperature, motor-error, feedback-age,
command-age, and EtherCAT protections remain enabled. ROS discovery is limited
to loopback with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` rather than the
deprecated `ROS_LOCALHOST_ONLY` variable.

Fast DDS may report the root driver's endpoints as `_NODE_NAME_UNKNOWN_`, so
`ros2 node list` is not a reliable liveness check for this temporary
commissioning arrangement. `scripts/ros2_env.sh` loads
`config/fastdds-udp-loopback.xml`, which restricts DDS to loopback and disables
data sharing, but this did not make mixed-account payload delivery reliable.

The guarded native Python test subsequently passed with motors 1, 2, and 3 at a
+30 RPM target for two seconds and a 2 A phase-current ceiling per motor. It
reported all three below 0.5 RPM during the stop window. The automatic ROS test
then passed at +5 RPM for two seconds with fresh `/joint_states` feedback and a
verified reported stop. Root-to-root keyboard commissioning also worked at
both +5 RPM and -5 RPM with dead-man stopping. These results validate the
current unloaded bench path; they do not establish loaded robot stopping or
host-loss behavior.

## Confirmed bench values (2026-09-16)

| Item | Value |
|---|---|
| EtherCAT interface | `enp86s0`, confirmed as the direct bridge connection |
| CAN motor IDs | 1, 2, 3 |
| Initial bus | CAN1; the three configured Python slots map to CAN1 |
| Mechanical condition | Motors are secured and unloaded; operator approved roughly one output revolution for a bench test |
| Bench PSU | 5 A maximum supply current |
| Requested provisional command ceiling | 2 A phase-current ceiling per motor; this is not a 6 A aggregate supply-current claim |

## Query-only hardware result (2026-09-16)

The bridge reached OPERATIONAL with the required 86-byte output / 92-byte input
PDO layout and WKC 3. No motion command was sent.

| CAN ID | Position | Hardware | Software | CAN timeout |
|---:|---:|---:|---:|---:|
| 1 | 119.436943 deg | 1.0.1 | 1.0.8 | 500 ms |
| 2 | -19.084883 deg | 1.0.1 | 1.0.0 | 500 ms |
| 3 | -85.408943 deg | 1.0.1 | 1.0.32 | 500 ms |

The PSU current and a motor's phase current are different quantities. The 2 A
value is a provisional command ceiling, not a model-derived safe operating
limit.

## Powered-motion status

Holding-brake presence and its documented release sequence are unknown. A
holding brake is an electromagnetic mechanism that can prevent shaft rotation;
the ENCOS manual says it should be opened before motor control. On 2026-09-16,
the operator explicitly instructed the bench test to proceed without treating
brake status as a motion gate. Stop immediately if a motor does not rotate,
draws unexpected current, heats, vibrates, or reports a fault.

`scripts/run_three_motor_velocity.py` implements the completed bounded
three-motor check: a +30 RPM target for a two-second command window, a 2 A
phase-current ceiling per motor, explicit zero-speed commands, and shutdown. A
fixed 60 RPM/s native slew limits abrupt velocity steps; it is intentionally not
another ROS/Python tuning parameter. The driver retains feedback, command-age,
current, temperature, overspeed, and EtherCAT fault checks.
