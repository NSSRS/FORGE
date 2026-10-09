# Live motor feedback in MuJoCo

This read-only viewer projects `/joint_states` encoder measurements onto the
existing `forge_sim` CAD wheel joints. MuJoCo is used because that branch already
provides MJCF and meshes; no Gazebo model conversion is required.

```text
hardware driver -> /joint_states -> read-only ROS relay -> loopback UDP -> MuJoCo
```

The viewer has no motor-command publisher and never opens EtherCAT. Closing it
or the relay does not stop the driver or change its watchdog behavior. Stop
motion using the keyboard/driver and the existing physical stop procedure.

## Open all four panes together

Stop the old driver, keyboard, relay and viewer, then run as your normal desktop
user (no sudo on the launcher):

```bash
cd /home/forge2/forge2_ws
./scripts/run_motor_test.sh
```

This opens a separate Terminator window with driver, W/S/A/D keyboard, feedback
relay and viewer panes. MuJoCo opens its own graphical window. The launcher uses keyboard input; to replace it with ESP32 single-stick input,
use the separate-terminal procedure in [radio control](RADIO_CONTROL.md).
The launcher
asks for your sudo password once (or uses an existing valid authentication);
the three hardware panes attach to that authenticated session. MuJoCo and
Terminator remain under your normal desktop account. No sudoers changes or
passwordless-sudo rules are installed. Wait for the driver's Ready message before
using the keyboard. The launcher uses `chassis_bench.yaml` and domain 44;
it does not change your saved Terminator layout. It refuses to launch if old
motor-test processes are still running. Keep the launching terminal open: it
owns the session. Closing the Terminator window or ending the launcher shuts
down the session’s remaining ROS processes. Closing an individual hardware
pane stops its attached process; it is not automatically restarted.

Use `./scripts/run_motor_test.sh --simulate` for hardware-free testing on domain
42, or `--dry-run` to inspect commands without opening anything. Space/X in
the keyboard pane requests a stop; stop each process with Ctrl+C when finished.
Closing only the MuJoCo window does not stop the motors.

## Run with the current hardware test

Keep the driver and differential keyboard using `chassis_bench.yaml`, domain 44.
Run these additional panes:

**Read-only relay, same root account as the hardware driver:**

```bash
sudo bash -lc '
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
export ROS_DOMAIN_ID=44
ros2 run forge_motor_control forge_motor_state_relay
'
```

**MuJoCo window, normal desktop account (no sudo):**

```bash
cd /home/forge2/forge2_ws
./scripts/run_motor_viewer.sh
```

The relay bridges the known root/non-root DDS delivery issue using UDP restricted
to 127.0.0.1, port 8766. No firewall or external network configuration is needed.
Use one relay per viewer. For another port, pass `--ros-args -p port:=8767` to
the relay and `--port 8767` to the viewer. Restart the relay after resetting the
ROS clock; non-increasing timestamps are rejected. Use the same ROS clock/domain
as the driver and only one `/joint_states` source.

## What you see

- Four modeled driven wheels. Wheels 1-3 rotate from measured shaft angles;
  wheel 4 is gray and marked ABSENT for `chassis_bench.yaml`.
- One large yellow pointer sits on each wheel’s outer face, perpendicular to its axle,
  and rotate with the measured wheel angle. Orbit the camera to view the wheel
  faces; the indicators naturally look edge-on from some angles. Gray dials
  identify the absent motor.
- The overlay switches to compact rows in smaller windows; portrait windows
  automatically gain camera distance while preserving your relative zoom.
- Green means live feedback. Orange means stale after 0.5 seconds. Gray means
  absent or waiting. The overlay reports wheel angle, wheel RPM and sample age.
- Positions are relative to each wheel's first received encoder value. This
  avoids an arbitrary CAD orientation jump; it is not encoder calibration.
- Stale joints hold their last measured pose, with velocity suppressed. They do
  not keep rotating from the last velocity command. A relay restart rezeros the
  visual baseline; restart both viewer and relay after resetting encoders.
- Mouse orbit and zoom work. The viewer has no W/S/A/D motor control; keep focus
  on the existing teleop terminal to drive.

The chassis, suspension and passive wheels retain the initial model pose.
No measured chassis pose, ground motion, contact simulation, torque/current or
temperature is claimed: `/joint_states` currently contains motor positions and
velocities only. Wheel encoders do not measure wall slip or global robot motion.
A moving chassis visualization needs odometry/IMU/localization feedback first.
The freshness indicator measures ROS sample delivery; it cannot independently
prove that cached bridge CAN feedback is fresh.

The projection uses the selected control YAML's wheel IDs, protocol forward
signs and external ratios. CAD forward signs are `[-1,-1,+1,+1]`, so the factor
from measured shaft angle to CAD wheel angle is
`CAD_forward_sign * wheel_directions / external_ratios`. The ENCOS internal
36:1 reduction is already included in feedback and is not applied again.
Command-only `wheel_rpm_scales` are not treated as mechanical gear ratios.

When motor 4 returns, update the control profile and use that same profile with
`./scripts/run_motor_viewer.sh --config /absolute/path/to/profile.yaml`.
Omission defaults to zero if absent. A missing required motor shows WAITING or
STALE; it is never fabricated from another motor or the requested command.

## Setup on a fresh checkout

```bash
cd /home/forge2/forge2_ws
./scripts/setup_motor_viewer.sh
source /opt/ros/jazzy/setup.bash
colcon --log-base logs/colcon build --base-paths src/forge_motor_control \
  --build-base build/ros2 --install-base install/ros2 --symlink-install
```

The helper extracts only `model/` from `forge_sim` commit
`7aa258d7105f1c088bb059556a06899433d30cab` into ignored `build/motor_viewer/` and
installs that branch's pinned MuJoCo 3.13.0 in a local virtual environment. It
does not switch branches or download Onshape credentials/exporter dependencies.
The system PyYAML comes from the existing ROS environment. A desktop OpenGL
session is needed for the interactive window.

## Verification

```bash
./scripts/run_motor_viewer.sh --check
build/motor_viewer/venv/bin/python -m unittest discover -s tests -p test_motor_mirror.py -v
source scripts/ros2_env.sh
ROS_DOMAIN_ID=83 python3 -m unittest discover -s tests -p test_ros_motor_mirror.py -v
```

The tests cover named mapping, direction/ratio conversion, omitted/missing
motors, malformed/replayed/stale packets, restart baselines, actual MuJoCo joint
projection with a fixed base, and real ROS-to-loopback delivery of synthetic
feedback. They do not command hardware. Use an isolated ROS domain for tests.
