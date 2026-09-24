# Differential chassis control

Differential control is integrated into the existing `forge_motor_driver`.
The existing `forge_keyboard_teleop` handles both straight motion and turning:

```text
forge_keyboard_teleop (or one planner)
  -> /cmd_vel [geometry_msgs/TwistStamped]
  -> forge_motor_driver (differential mixing + command watchdog) -> MotorBus -> C11/SOEM -> EtherCAT/CAN -> ENCOS Mode 2
```

The driver still owns the bridge, publishes output-shaft `/joint_states`, and
enforces the existing current, speed, feedback and command protections. No
EtherCAT work occurs in the keyboard. The same driver and keyboard support
`command_mode: chassis` (default) or `command_mode: direct_rpm` (the three-motor
commissioning config). The driver subscribes to only one command topic for the
selected mode. There is no separate base-controller or chassis-keyboard executable.
Use only one chassis command publisher; source arbitration is not implemented.

## Model source and hardware mapping

Read against `main` at `7cc4cf0` and `forge_sim` at `7aa258d`.
The source is `forge_sim/velocity_control.py` and `model/robot.xml`, not the
older placeholder at `de5130d`. The source code currently sets DRIVE_RPM=60 and
TURN_RPM=30; older README examples still say 10 and 6. Hardware defaults here
are lower commissioning settings, not copied simulation speed requests.

| Wheel joint | Robot side | Forward joint sign | Example motor ID |
|---|---|---|---|
| wheel_1 | left | -1 | 1 |
| wheel_2 | left | -1 | 2 |
| wheel_3 | right | +1 | 3 |
| wheel_4 | right | +1 | 4 |
| wheel_5, wheel_6 | passive | no command | none |

The loaded model's driven joint centers have wall-frame X coordinates
`+0.3685,+0.3685,-0.3685,-0.3685` m: track width 0.737 m. Front/rear driven
centers are separated by 0.496 m. The nominal wheel radius is 0.040 m, matching
the radial mesh envelope of wheels 1, 2 and 4. Wheel 3's merged visual envelope
extends to about 0.04118 m; use the nominal rolling radius as a calibration
starting point, not a claim that every mesh vertex lies on its tread.
Suspension motion, skid slip and surface curvature can change effective track
and rolling radius. This controller provides nominal velocity mixing, not
closed-loop yaw tracking, odometry or a dynamic contact model.

Define the command `base_link` frame as x forward toward the CAD front, y robot
left, z outward from the work surface. At the initial wall pose, these correspond
to CAD/world +Z, +X, +Y. Positive angular.z means a left turn within the local
surface plane; it does not mean rotation about world vertical on a wall.
This is a command convention; no TF or URDF is published by this change.

For forward speed v and yaw rate w:

```text
left_m_s  = v - w * track_width_m / 2
right_m_s = v + w * track_width_m / 2
motor_RPM[i] = side_m_s * 60 / (2*pi*wheel_radius_m)
               * wheel_directions[i] * external_ratios[i] * wheel_rpm_scales[i]
```

The sim's `(forward_RPM +/- turn_RPM) * direction * scale` mixing is equivalent
under `v=forward_RPM*2*pi*r/60`, `w=turn_RPM*2*pi*r*2/(60*track)`.
Thus forward at 10 wheel RPM yields `[-10,-10,+10,+10]`; a left pivot at
6 wheel RPM yields `[+6,+6,+6,+6]` in CAD joint coordinates.
The ENCOS 36:1 internal reduction is already included in output-shaft RPM.
`external_ratios` covers only gearing between that shaft and a wheel.
The simulation's 12 Nm torque cap and provisional servo gain are not ENCOS
current/gain settings and are not copied into hardware packets.

All per-wheel arrays are ordered wheel_1 through wheel_4. `wheel_motor_ids`
maps those wheels to physical IDs; `motor_ids` is independently the driver's
PDO order. The controller reorders its output to match the latter. The first
three driver entries use CAN1, and the fourth uses CAN2; the number of a motor
ID does not choose its CAN bus. The driver uses one configuration for wheel mapping, PDO order and RPM
limits, validated before opening the bridge.

The example IDs and CAD signs are not verified physical wiring assignments.
Only IDs 1–3 on CAN1 have prior unloaded bench evidence. Confirm wheel-to-ID
assignment, signs, external gearing, and the fourth CAN2 slot before selecting
hardware mode. Set `wheel_directions` to the actual protocol sign that drives
each wheel forward; do not multiply a second CAD sign into it. Preserve
`motor_ids` slot order when changing wheel assignments.

## Build and run

On the Ubuntu ROS 2 Jazzy host, from the hardware checkout:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base logs/colcon build --base-paths src/forge_motor_control \
  --build-base build/ros2 --install-base install/ros2 --symlink-install
source scripts/ros2_env.sh
ros2 launch forge_motor_control chassis.launch.py
```

This starts the four-motor Python simulator, not MuJoCo or physical motors.
In another terminal in the same checkout/account:

```bash
source scripts/ros2_env.sh
ros2 run forge_motor_control forge_keyboard_teleop
```

Hold W/S for forward/reverse, A/D for left/right pivot. Space/X stops; Q/Escape
exits. A second matching key event starts motion; key repeat refreshes it and
release stops after 0.15 s. Unlike the simulation viewer, motion does not latch.
Default requests are 0.02 m/s and 0.07 rad/s (about 4.77 and 6.16 wheel RPM).
Keyboard speeds and `command_frame` are ROS parameters; to use an edited YAML,
pass `--ros-args --params-file /absolute/path/chassis.yaml` to the keyboard too.

For reviewed hardware mapping, use the same launch with an edited config:

```bash
ros2 launch forge_motor_control chassis.launch.py \
  config:=/absolute/path/chassis.yaml \
  simulate:=false execute:=true mapping_confirmed:=true
```

Use the existing root-account procedure from ROS2_MOTOR_CONTROL_STATUS.md for
both hardware launch and teleop, sourcing `scripts/ros2_env.sh` inside that
account. The launch flag acknowledges configuration review; it does not measure
the wiring. No physical motion was performed as part of this implementation.

## Commands, limits and verification

Publish `TwistStamped` on `/cmd_vel` with a fresh nonzero ROS timestamp and
`header.frame_id=base_link`. Only linear.x (m/s) and angular.z (rad/s) may be
nonzero. Publish faster than the 0.15 s command lease, typically 50 Hz. Use one
publisher, synchronized ROS clocks and system time on hardware. Empty/wrong
frames, nonfinite values, unsupported axes and stale/future/replayed timestamps
are rejected and cancel any active target. A backwards ROS clock jump stops
the old target; monotonic receipt time also expires commands if ROS time freezes.

Speed limiting scales all four outputs together to preserve the requested
turning ratio. Default motor ceiling remains 30 RPM and provisional phase-current
ceiling 2 A. Native 60 RPM/s slew remains unchanged, so transient wheel ratios
can differ from the requested steady-state curvature. Source expiry requests
zero RPM; it is not an instantaneous mechanical stop or a holding brake.

The driver remains query-only until a valid command arrives. It requests zero
RPM when the chassis lease expires (0.15 s in chassis.yaml). Direct-RPM
commissioning retains its 0.20 s receipt watchdog. The native 0.25 s lease
remains in place if the ROS driver stops refreshing motor commands. This does not add a safety supervisor,
fault reset, ESTOP, source mux or loaded wall-motion validation.

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -v
```

`test_differential_drive.py` checks signs, arcs, limits, configuration, timestamp
rejection, expiry and keyboard dead-man behavior. The native test suite also
sends forward/left/right/stop mappings through the actual C worker and fake
four-slot bridge. `test_ros_chassis.py` exercises real topic delivery and motor
feedback with `simulate=true`; it skips when ROS 2 is absent. Run that test on
an isolated ROS domain with no hardware driver or other command publishers.
