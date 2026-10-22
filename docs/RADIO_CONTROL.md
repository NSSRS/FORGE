# Pocket single-stick chassis control

For the combined sensor/Pocket bench session, run `bash scripts/run_system_test.sh`.
This convenient command now starts **actual motors** on the commissioned
three-motor bench and requests sudo when needed. Use `simulate:=true` for
simulated motors. See [full-system setup](FULL_SYSTEM_TEST.md) for saved ports,
limits, viewer behavior, and the RGB-only 60 FPS diagnostic profile.

`keyboard_teleop` now accepts `input_source:=keyboard`, `radio`, or
`radio_monitor`. Keyboard and radio share the same `/cmd_vel` TwistStamped
publisher and chassis speed parameters. No virtual key presses or joystick
emulation are needed. The driver handles all wheel mixing and RPM limits.

One physical stick controls the whole chassis:

| Selected stick motion | Command |
|---|---|
| Up/down | W/S equivalent, proportional forward/reverse |
| Left/right | A/D equivalent, proportional left/right yaw |
| Diagonal | Combined forward/reverse and turn |
| Center | Zero command |
| Other stick | Unassigned; does not affect motion |

Choose either physical stick and measure its two channels. A self-centering
stick is appropriate for this center-zero mapping; a non-centering throttle
axis will retain its speed request when released. Channel numbers do not identify
physical sides by themselves. No CH1/CH2 or Mode 1/Mode 2 assignment is assumed.
There is no separate speed-setting control: deflection directly sets speed.

## Working serial contract

The existing sketch at
`/home/forge2/Documents/Codex/2026-10-07/i-ha/outputs/er12_to_pc/er12_to_pc.ino`
remains unchanged. ER12 CRSF TX connects to GPIO27, with common GND. Its CRSF RX
and connector positive pin remain disconnected. GPIO26 is configured as TX but
unconnected. The receiver is independently powered through XT30; the ESP32 board
uses PC USB power. GPIO numbers are board labels, not module pad numbers.

The ESP32 receives non-inverted 420000 baud 8N1 CRSF, validates length/CRC and
decodes type 0x16 into sixteen 11-bit raw ticks. USB output is **115200 baud**,
about 10 rows/s: `age_ms,ch1,...,ch16`. These are neither microseconds nor
normalized values. Sixteen fields do not imply sixteen independent controls.
The parser tolerates the startup banner and `NO_RECENT_CHANNEL_FRAMES` by
resetting radio motion to zero; those lines are never interpreted as channels.

The user confirms Pocket movement changes the displayed channels. ER12 firmware
4.1.0 is confirmed; the final Pocket version, EdgeTX version, packet rate,
switch mode, channel assignments and exact ESP32 module suffix remain unverified.
No additional Arduino libraries or sketch flash are needed for this PC bridge.

## Discover and calibrate without commanding motors

Close Arduino Serial Monitor. Discover the USB port again; do not assume the
old `/dev/ttyUSB1` assignment:

```bash
ls -l /dev/serial/by-id/
```

If that directory is absent, inspect the attached USB serial devices before
continuing. Serial access needs the appropriate permissions. The ROS package
now declares `python3-serial`; install that dependency if missing. Rebuild the
package using [the ROS build instructions](ROS2_MOTOR_CONTROL_STATUS.md#build-and-run).

Run the read-only monitor, replacing the path with the actual port:

```bash
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
ros2 run teleop keyboard_teleop --ros-args \
  -p input_source:=radio_monitor -p serial_port:=/dev/serial/by-id/ACTUAL_DEVICE
```

This mode has no command publisher and does not require a motor driver. Move
only one axis at a time and record all four axes to distinguish the two sticks:

| Physical axis | Channel | Raw minimum | Center | Raw maximum | Direction of increasing raw |
|---|---|---|---|---|---|
| Left up/down | measure | measure | measure | measure | measure |
| Left left/right | measure | measure | measure | measure | measure |
| Right up/down | measure | measure | measure | measure | measure |
| Right left/right | measure | measure | measure | measure | measure |

Copy `src/bringup/config/radio.yaml` to a local calibration file.
Set `serial_port`, and fill `forward_*` and `turn_*` from the same physical stick.
Channels are numbered 1..16. Set each sign so up produces positive forward and
left produces positive yaw. `radio` mode refuses the placeholder calibration.
Keep `input_source: radio_monitor` until the measurements are complete.

Each side of center is normalized using its own measured span, clamped to
[-1,1], then a 5% dead zone is removed and the remainder rescaled to full range.
The resulting axes multiply `speed_m_s` and `yaw_rate_rad_s`. Thus full stick
matches the keyboard request and half of the usable travel gives half speed.
The bench profile uses 0.08 m/s and 0.28 rad/s; the driver's 20 RPM ceiling
continues to scale mixed requests uniformly. These are existing bench settings,
not newly validated loaded-robot limits.

## Run with the chassis simulator first

Stop the monitor before opening the same USB device. In the driver terminal:

```bash
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
export ROS_DOMAIN_ID=42
ros2 launch bringup chassis.launch.py \
  config:=/home/forge2/forge2_ws/src/bringup/config/chassis_bench.yaml
```

In the radio terminal, replace the calibration path:

```bash
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
export ROS_DOMAIN_ID=42
ros2 run teleop keyboard_teleop --ros-args \
  --params-file src/bringup/config/chassis_bench.yaml \
  --params-file /absolute/path/to/calibrated-radio.yaml -p input_source:=radio
```

Center the selected stick for at least 0.3 seconds of fresh samples before
motion is accepted. Check `/cmd_vel` and simulated `/joint_states`: up/down,
left/right, diagonals, center, and moving only the unused stick. Ctrl+C exits
and attempts zero commands. Space/X applies only in keyboard mode.

For the already commissioned unloaded hardware bench, use the driver commands
in [differential bench testing](DIFFERENTIAL_DRIVE.md#three-motor-differential-bench-motor-4-absent)
and run the radio command under the same root account/domain 44. Do not also
start a keyboard publisher. The four-pane launcher still starts keyboard mode;
use separate terminals for radio input. Motor 4 remains omitted by that profile.

## Freshness and remaining commissioning

The PC loop publishes at approximately 50 Hz using the most recent valid sample;
the sketch still samples channels for USB output at about 10 Hz. This does not
increase RF or USB sampling rate. The default radio lease is 0.15 s minus the
reported ESP32 sample age. Malformed/stale input or serial silence clears the
motion target. A polling gap of 0.15 s or an oversized serial backlog discards
queued bytes and requires neutral again. USB exceptions exit with attempted
zero commands; reconnection requires restarting the process and neutral acquisition.

The existing CSV has no source sequence number or source timestamp. Small USB
buffering delays cannot be measured by the PC, so this is a diagnostic-protocol
bridge, not a proven end-to-end age guarantee. A sequence/timestamp and explicit
link-health protocol would improve the next firmware revision.

Fresh channel frames do not prove a healthy RF link. If the receiver continually
sends held channel values after transmitter loss, the current CSV cannot detect
that. With only the monitor/simulator running, switch off the Pocket and record
whether channels stop, go neutral, or hold, and whether frames continue. Verify
receiver failsafe and link statistics before physical radio driving. This node
does not implement an enable switch, RF-health decoding, command-source mux or
hardware ESTOP. Native motor protections remain active, but do not replace those
checks or establish stopping/holding behavior under load.

## Controller-only test at the motor-command boundary

Connect the ESP32/controller chain and leave motor hardware disconnected.
Use the calibration procedure above first; placeholder channel numbers and
endpoints intentionally prevent radio mode from starting. Close Serial Monitor
and the read-only monitor before starting radio mode.

In terminal 1, run the actual driver with its simulated MotorBus backend and
optional target logging:

```bash
cd /home/forge2/forge2_ws
source scripts/ros2_env.sh
export ROS_DOMAIN_ID=42
ros2 run motor_control motor_driver --ros-args \
  --params-file src/bringup/config/chassis_bench.yaml \
  -p simulate:=true -p execute:=false -p log_motor_targets:=true
```

In terminal 2, use the radio command from the simulator section above, with
your calibrated radio file and the same domain 42. No sudo is required.

The driver prints `MotorBus targets (RPM): ID 1=..., ID 2=..., ID 3=...` at
up to 10 Hz, immediately before calling `MotorBus.velocity(id, rpm)`. These are
the post-mixing, post-limit, active-motor targets, before the native slew limit
and C motor commands. Simulation does not load the C library or open EtherCAT.
This verifies the Python/ROS command path, not physical movement or C transport.

With the current bench configuration, full forward and no turn should print
approximately `ID 1=-19.099, ID 2=-19.099, ID 3=+19.099`. Full reverse flips
those signs. Full left pivot produces approximately `+20,+20,+20`; right
produces `-20,-20,-20`. Center produces zeros. Diagonal requests stay within
20 RPM per motor. Moving only the unused stick should not change targets.

Before the first radio message the driver stays query-only, so it prints no
targets. Center the selected stick for at least 0.3 s to acquire neutral.
Disconnecting USB or stopping radio input should change targets to zero;
the simulator's measured velocity can take longer because it models the
60 RPM/s slew. Ctrl+C stops each process.

## Measured Pocket right-stick profile (2026-10-09)

`src/bringup/config/radio_pocket.yaml` records the operator's
measurements: CH2 forward/reverse and CH1 steering, both minimum 174, center
992, maximum 1811. Forward sign is +1; steering sign is -1 because moving
left decreases CH1. The 5% dead zone covers the observed center variation
(CH1 previously read 996). Other channels do not affect motion.

Use this file instead of the placeholder `radio.yaml` in the controller-only
test above, with `-p input_source:=radio`. It retains monitor mode by default.
The recorded port is `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`;
rediscover it if the USB adapter changes. These are measured radio endpoints,
not confirmation of RF failsafe or physical motor directions.
