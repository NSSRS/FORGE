# Combined IMU, camera, Pocket and motor bench test

`system_test.launch.py` starts the existing HWT9052 reader, Gemini 338Le
camera, Pocket radio teleop and motor driver together. No estimator or navigation
stack is added. The convenient Bash wrapper defaults to **actual motors**,
with real sensors/controller and sudo elevation when required. It supplies the
commissioned bench execution/mapping/failsafe flags; explicit CLI flags override
these defaults. The underlying ROS launch retains its simulated defaults.
Use `bash scripts/run_system_test.sh simulate:=true` for simulated motors.
The selected profile is the confirmed three-motor bench: IDs 1/2 on CAN1,
ID 3 on CAN2, motor 4 absent. Pocket right stick uses CH2 forward and CH1 turn.
Limits remain 20 output RPM and the existing provisional 2 A phase-current
ceiling; these are unloaded bench settings, not loaded robot limits.

## Connections and preparation

| Device | Host connection |
| --- | --- |
| ENCOS bridge EtherCAT IN | Built-in `enp86s0`, direct cable, no IP address |
| Gemini 338Le | USB Ethernet `enx000ec8a22028`, host `192.168.1.100/24`, camera `192.168.1.10` |
| HWT9052 IMU | Its own USB-RS485 serial adapter, 9600 baud, address 80 |
| Pocket/ER12/ESP32 | Its own USB serial adapter, 115200 baud |

Camera preparation explicitly uses the USB Ethernet port even when both links
have carrier. Camera and EtherCAT cannot share a port. Neither the launcher nor
the camera preparation assigns an IP address to the motor port.

Close previous camera, IMU, teleop and motor sessions, Arduino Serial Monitor,
and standalone sensor readers. Only one EtherCAT master and one reader per
serial port should run. Close the old camera viewer too; the session opens one.

The operator identified the current adapters; these are now the launch defaults:

- IMU (`ttyUSB1`): `/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0`
- ESP32 radio (`ttyUSB0`): `/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0`

Keep them in the same physical USB sockets. The saved paths tolerate ttyUSB
number changes. Plain `bash scripts/run_system_test.sh` uses these defaults;
explicit `imu_port:=...` and `radio_port:=...` still override them.

If you move the adapters to other sockets, identify them again:

```bash
ls -l /dev/serial/by-path/
```

Three adapters were attached during preparation. Their by-id names are not
unique, so do not assume that the single by-id symlink is the controller.
Identify each by unplugging/reconnecting its USB cable while no readers run,
then use the corresponding by-path name. Do not unplug powered motor wiring.
Set these shell variables to the two actual paths (different devices):

```bash
export IMU_PORT=/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0
export RADIO_PORT=/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0
```

To verify assignments individually without motor commands:

```bash
cd ~/forge2_ws
source scripts/ros2_env.sh
python3 scripts/read_imu.py --port "$IMU_PORT" --samples 5
ros2 run teleop keyboard_teleop --ros-args -p input_source:=radio_monitor -p serial_port:="$RADIO_PORT"
```

Stop the radio monitor with Ctrl+C before the combined launch. Confirm the
right-stick channel mapping. In monitor/simulation only, turn off the transmitter
and verify receiver failsafe: held nonzero channel values must not continue to
arrive as fresh commands. The current ESP32 CSV cannot distinguish those from
a healthy RF link. Physical radio operation requires this verification.

The native library and all six ROS packages were built during preparation.
After future source updates, rebuild with:

```bash
./scripts/build.sh
source /opt/ros/jazzy/setup.bash
colcon --log-base logs/colcon build --base-paths src --build-base build/ros2 --install-base install/ros2 --symlink-install --packages-up-to bringup
```

## Optional first pass: sensors and controller, simulated motors

```bash
cd ~/forge2_ws
bash scripts/run_system_test.sh simulate:=true imu_port:="$IMU_PORT" radio_port:="$RADIO_PORT"
```

This explicit simulation command uses ROS domain 44 and three windows:
- Camera RGB video at a requested 1280x800/60 FPS, with CPU/RAM and received FPS.
  Current transport diagnostic profile disables depth/IR, alignment, and frame sync.
  The checker requires RGB by default; pass `--include-depth` when depth is enabled.
- IMU orientation: vendor roll/pitch/yaw, an illustrative 3D body and sensor axes,
  with WAITING/LIVE/STALE indication. The shaded body uses a right-handed
  view from +X/+Y/+Z; the small 0X/0Y/0Z triad stays fixed as a reference.
  The display applies Rz(yaw) Ry(-pitch) Rx(-roll), matching the observed
  bench roll and pitch directions. The enclosure is short along X and long along Y.
  The physical IMU +X/+Y markings point along model-local -X/-Y:
  the moving arrows point toward those sides and are labeled +X/+Y.
  The fixed 0X/0Y arrows remain the display reference. This annotation is not a calibrated ROS
  frame transform; the viewer retains a note about the mapping.
  Numeric angles and ROS messages retain the original sensor values.
  No position is inferred. The IMU's standard
  quaternion remains unavailable; this viewer consumes `/imu/data_rpy`.
- Existing MuJoCo motor model: measured `/joint_states` drive the saved wheel
  mapping, with motor 4 absent. The same `config` profile is passed to the driver
  and viewer. The chassis stays fixed; no uncalibrated IMU mounting transform
  is applied to it. This checkout's model is MJCF, not URDF.

Use `show_imu:=false` or `show_motor_model:=false` to disable either new window.
For a headless run, disable all three with `show_video:=false show_imu:=false
show_motor_model:=false`. The motor viewer uses the local environment/model
installed by `scripts/setup_motor_viewer.sh`; these are already available on this
host. Its UDP relay uses port 8766 by default (`motor_viewer_port:=...` overrides
both relay and viewer). Stop old motor relays/viewers before starting the combined
session. With sudo, the relay shares the motor driver's account, while MuJoCo
runs as the invoking desktop user. On this desktop, the model viewer uses the
X11 GLFW library. If its graphics thread stalls during shutdown, ROS launch
terminates that read-only viewer after bounded waits; motor shutdown proceeds
independently. The
camera preflight may ask for sudo only when its host IP needs configuring.
For serial access as the normal user, use the existing dialout setup. The
launcher rejects absent ports and two aliases for the same serial device.

Center the Pocket right stick for at least 0.3 seconds. Check forward/reverse,
left/right, diagonals and neutral. The driver logs the resulting motor RPM
requests. No native EtherCAT library is used in simulation.

In a second terminal:

```bash
cd ~/forge2_ws
source scripts/ros2_env.sh
ROS_DOMAIN_ID=44 ros2 run bringup check_system --duration 10
```

Require LIVE traffic on `/imu/data_raw`, `/stereo_cam/color/image_raw`,
`/stereo_cam/depth/image_raw`, `/cmd_vel` and `/joint_states`. The checker
prints rates and reported motor names and returns failure for absent/stale
streams. It never publishes commands. `/cmd_vel` traffic is not proof of RF
health: the teleop can correctly publish zero while no radio samples arrive.
Use the radio monitor for RF-loss testing. Camera IR/calibration can also be
checked using `ROS_DOMAIN_ID=44 ros2 run stereo_cam check_camera`.

## Second pass: secured, unloaded physical motors

Stop the simulated session with Ctrl+C. Keep the shafts/wheels clear, the
controller neutral, and the physical power disconnect accessible. Confirm
mapping, motor limits and receiver failsafe before setting the flags below.
This test is not authorization for loaded climbing/welding operation.

Prepare the dedicated EtherCAT interface once if NetworkManager is still
trying DHCP on it (internet stays on Wi-Fi):

```bash
sudo nmcli device set enp86s0 managed no
sudo ip link set enp86s0 up
ip -br address
```

If `enp86s0` still has the earlier temporary camera address, remove that exact
address with `sudo ip address del 192.168.1.100/24 dev enp86s0`. Do not flush
unrelated networking. The hardware launch refuses an IPv4-configured motor port.

Launch all nodes under the same root account because this host previously had
unreliable mixed root/user DDS delivery. The wrapper preserves desktop display credentials, requests sudo if needed,
and sources the ROS environment itself:

```bash
cd ~/forge2_ws
bash scripts/run_system_test.sh
```

If the root video window cannot access your desktop, use `show_video:=false`
and inspect topics; do not relax display access globally.

Read-only check from a second terminal, under the same account/domain:

```bash
cd ~/forge2_ws
sudo bash -c 'source scripts/ros2_env.sh; ROS_DOMAIN_ID=44 ros2 run bringup check_system --duration 10'
```

Wait for motor Ready, IMU LIVE and live footage before moving the stick. Keep
initial deflections small, check signs and return to neutral between directions.
The existing command leases remain active. Readiness is an operator check:
this launch does not gate motion on sensor-message freshness.

## Stop and limits

Ctrl+C stops the session. Closing any of the three viewers or any process exiting also shuts
down all launched processes. The driver/radio attempt zero commands on clean
shutdown, with existing native and ROS command expiry as fallback. This is not
a verified mechanical brake or independent emergency stop.

A process can remain running while its sensor data stops. The bounded checker
reports this when run; it is not a continuous safety supervisor. Camera/IMU data
is not yet fused, mounted-frame calibration remains pending, and all LIVE
results indicate communication only, not localization accuracy.

For a completely hardware-free launch smoke test, use:

```bash
ROS_DOMAIN_ID=93 bash scripts/run_system_test.sh simulate:=true use_camera:=false use_imu:=false use_radio:=false show_video:=false show_imu:=false show_motor_model:=false
```

This starts only the simulated motor driver. The operator has run the sensors
with physical motor teleop. Video reception still needs optimization: four
60 FPS camera streams produced repeated RTP packet-sequence warnings; the
RGB-only camera backend subsequently reported approximately 60 FPS. This is
not a measurement of end-to-end display latency.
