# Gemini 338Le stereo camera

`stereo_cam` integrates the official [OrbbecSDK_ROS2 v2 driver](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main).
The upstream support table selects `gemini_330_series.launch.py` for Gemini
338Le (listed firmware 1.8.17). Do not use the legacy `main` branch.
The project package follows the IMU's ament_python layout. The vendor driver
owns acquisition, factory calibration, optical TF and device controls;
`bringup` owns the shared launch/profile. Camera launch does not start motors.

## Install and launch

On native Ubuntu 24.04 with ROS 2 Jazzy:

```bash
./scripts/setup_stereo_cam.sh
source scripts/ros2_env.sh
./scripts/run_stereo_cam.sh
# Or select an Ethernet camera explicitly:
./scripts/run_stereo_cam.sh net_device_ip:=192.168.1.10
# Or select a USB camera by its actual serial number:
./scripts/run_stereo_cam.sh serial_number:=YOUR_SERIAL
```

The commissioned camera IP is `192.168.1.10`. For Ethernet, the camera must be powered,
the connected host interface must have carrier and a compatible subnet, and
the camera must be reachable. Network enumeration stays enabled, including when selecting by IP. Prefer
serial_number for a discovered Gemini camera; forcing the legacy direct TCP
connection caused a model-detection failure during commissioning.
Do not repurpose the motor EtherCAT interface without confirming the wiring.
For USB, install the vendor device permissions and reconnect the camera:

```bash
sudo cp /opt/ros/jazzy/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Setup requires administrator authentication for system packages. The script
builds project packages into `build/ros2` and `install/ros2`, matching the other
sensors. Current upstream source was downloaded for inspection to the ignored
`build/stereo_cam/upstream/OrbbecSDK_ROS2`, revision
`c153462518ad674650bafa4464fda72c27ab797a` (2.10.6). Runtime uses the installed
ROS package, not that checkout. The host's apt index offered 2.9.3 during setup;
all profile argument names were also checked against that 2.9.3 binary package.
Verify launch options if using a different driver release. The upstream source
and its graphics/description assets remain unchanged and outside project source.

## Persistent NUC camera connection

The NUC now has a saved NetworkManager profile `FORGE-camera`, bound to USB
adapter MAC `00:0E:C8:A2:20:28` / `enx000ec8a22028`, with automatic connection
and static address `192.168.1.100/24`. It has no gateway or DNS and never installs
a default route. This restores the address after reboot/reconnection without
manual `ip address add` commands. Wi-Fi and the EtherCAT port are unchanged.
The camera remains at `192.168.1.10`. This is host configuration, not a setting
installed by pulling the repository on another computer.

## Automatic network preparation

Every `ros2 launch bringup stereo_cam.launch.py` (including the Bash wrapper)
prepares the commissioned Ethernet connection before starting the driver:
- Detects carrier on `enp86s0` or `enx000ec8a22028`.
- Uses the saved host address when present. Otherwise, NetworkManager activates
  a dedicated `FORGE-camera-<interface>` profile with `192.168.1.100/24`, no
  default route, and autoconnect disabled. The launcher selects it each time;
  no manual IP command is needed on this NUC. Hosts without NetworkManager
  retain the temporary-address sudo fallback.
- Removes that exact address from the other port only if it is disconnected.
- Pings camera `192.168.1.10` three times, stopping launch if it is unreachable.

If both ports have carrier, select `camera_interface:=enx000ec8a22028` or
`camera_interface:=enp86s0`. Other IPv4 addresses or overlapping subnets cause
an error rather than being changed. Wi-Fi and default routes are not modified.
For another subnet, pass both `camera_ip:=...` and `camera_host_address:=...`.
An explicit `net_device_ip` also sets the ping target. To use USB cameras,
a custom network setup, or skip preparation, pass `prepare_camera_network:=false`.
The camera itself is not readdressed. This preparation is for the two
commissioned host ports; it is not a general network manager.

## Streams and verification

The default namespace is `/stereo_cam`. The default profile enables
only color at 1280x800/30 FPS. Depth, both IR streams, frame synchronization,
and software depth alignment are opt-in to control processing and bandwidth.
Each enabled stream publishes `image_raw` and `camera_info`; subscribers must
accept best-effort delivery. Depth/IR settings remain 848x530/30 FPS for explicit
re-enabling. Check calibration and rectify IR images before stereo estimation.

In a second terminal, use the same domain as the wrapper:

```bash
source scripts/ros2_env.sh
source scripts/camera_env.sh
export ROS_DOMAIN_ID=42
ros2 run stereo_cam check_camera --timeout 10 --streams color
ros2 topic list
ros2 topic hz /stereo_cam/color/image_raw
ros2 topic echo /stereo_cam/color/camera_info --once
```

The bounded check subscribes to all four image/calibration pairs and reports
frame counts, dimensions, encoding and frame ID, failing if a stream or valid
intrinsics are absent. For an intentionally reduced profile, use
`--streams color depth`. It does not validate inter-camera synchronization or
robot-frame extrinsics. View images in RViz2 or rqt_image_view when installed,
using best-effort QoS and a camera TF frame as the fixed frame. Optional tools:
`sudo apt-get install ros-jazzy-rviz2 ros-jazzy-rqt-image-view`.

The Bash launcher `./scripts/run_stereo_cam.sh` opens the RGB video window
automatically alongside the driver. Ctrl+C in that terminal stops both. Use
`./scripts/run_stereo_cam.sh show_video:=false` for headless operation.
To view an already-running driver separately, run `./scripts/view_stereo_cam.sh`.
It waits for `/stereo_cam/color/image_raw` and marks stale footage if streaming
stops. Close the window or press Escape/Q to exit. This viewer subscribes to
the running driver; it does not open a second camera connection.

## Localization controls

The shared profile is `src/bringup/config/stereo_cam.yaml`; all its keys are
launch arguments and command-line values override defaults. Examples:

```bash
./scripts/run_stereo_cam.sh enable_depth:=true depth_registration:=true enable_point_cloud:=true enable_colored_point_cloud:=true
./scripts/run_stereo_cam.sh enable_depth:=true enable_threshold_filter:=true threshold_filter_min:=200 threshold_filter_max:=3000
./scripts/run_stereo_cam.sh enable_depth:=true enable_spatial_filter:=true
./scripts/run_stereo_cam.sh enable_left_ir:=true enable_right_ir:=true
```

- RGB + aligned depth + CameraInfo provide input to an RGB-D odometry/SLAM pipeline.
- Left/right IR + calibration provide input to a calibrated stereo pipeline.
- Depth threshold bounds are in millimeters. The supplied bounds are examples,
  inactive until enabled; set them for the actual working distance.
- Spatial filtering may reduce noise. Decimation reduces depth load but also
  detail. Temporal smoothing and hole filling default off: evaluate motion lag,
  edge changes and invented depth before using these for odometry or obstacles.
- Frame synchronization is currently disabled; when enabled it operates inside the camera. It does not synchronize
  the separate HWT9052 IMU clock, guarantee image pairing, or establish PTP.
- Software noise removal inherits the vendor default (enabled); hardware noise
  removal, spatial smoothing, temporal smoothing, hole filling, threshold and
  decimation are disabled. Depth filters have no active depth stream in RGB-only mode.
- Auto exposure is enabled. Manual color/IR exposure is available via profile
  arguments after disabling the corresponding auto-exposure option; verify units
  and supported ranges against the device/firmware before tuning.
- Point clouds are optional to avoid unnecessary bandwidth and CPU load. Camera
  IMU outputs default off; confirm hardware support before enabling them.

Before localization, measure the camera-to-base transform and IMU extrinsics,
check optical-frame conventions, timestamps and synchronization, select frame
rates, and test texture/depth quality on the actual welding surfaces. The driver
publishes internal camera transforms only; no unmeasured base transform is
invented. No odometry, pose fusion, SLAM or collision protection is implemented
by this package.

## Validation status

The connected Gemini 338Le (serial CHL5663000HZ, firmware 1.8.17) has
streamed live RGB/depth/IR. The operator previously ran sensors with motor
teleop and has confirmed smooth camera-only RGB/depth viewing after the DDS fix.
An isolated RGB 1280x800/60 FPS subscriber measured 59.97 callbacks/s with
32.5 ms maximum gap, compared with 10.55 callbacks/s before the transport fix.
Host-wide UDP receive-buffer errors increased by 10,649 in the earlier test
and did not increase during the corrected test. SDK/pre-publication rates
were already about 59.9 FPS, locating the observed loss downstream of acquisition.

These are short measurements on this NUC, not a sustained 60 FPS guarantee
for every stream. A heavier simultaneous RGB/depth/cloud test with extra
measurement subscribers reached only about 37/17/17 FPS. The combined motor
launch has not been revalidated with this camera-only DDS profile. Calibration,
localization and robot-frame extrinsics remain unvalidated.

## Optional depth preview

When depth is explicitly re-enabled, open a separate viewer with:

```bash
bash scripts/view_stereo_cam.sh --topic /stereo_cam/depth/image_raw
```

The viewer accepts millimeter `16UC1` and meter `32FC1` images. It uses a fixed
0.25–6 m color scale: warm near, cool far, invalid samples black. Out-of-range
valid values saturate; ROS depth data is unchanged. Alignment can be enabled
explicitly for RGB-D consumers, at additional processing cost.

The camera window footer samples NUC CPU and RAM once per second. CPU is
whole-system utilization averaged across logical CPUs; busiest core exposes
single-core saturation. RAM usage is MemTotal minus MemAvailable (reclaimable
cache is not counted as unavailable memory). The overlay remains live while
waiting for images or showing a stale frame.

## ROS transport and image display

Camera-only wrappers source `scripts/camera_env.sh` after `scripts/ros2_env.sh`
and use domain 42 by default. The installed bringup profile
`fastdds-camera-shm.xml` provides a 64 MiB shared-memory transport and loopback
UDP, with synchronous publication and data sharing disabled. Driver and viewers
must run under the same account. Shared-memory transport avoids passing large
local image messages through the congested UDP path; this is not end-to-end
zero-copy. The camera-to-NUC Ethernet acquisition path is unchanged.

Jazzy's `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` injects additional transports.
The camera environment uses `SUBNET` so XML owns that choice; its UDP whitelist
still restricts traffic to loopback. The combined motor launch keeps its existing
UDP profile and domain 44. Do not source the camera environment for motor bringup.
For direct camera-only ROS commands, source both environment scripts and set
domain 42 as in the verification example above.

The viewer uses a dedicated ROS executor thread with best-effort, depth-one QoS.
Its callback retains only the newest image and bounded arrival timestamps.
Conversion, depth colorization, overlays and GUI updates run on the main thread;
slow display never creates an unbounded image queue. Python threads still share
the GIL. The display defaults to a 30 FPS cap; `--display-fps 60` raises it.

The footer reports **ROS subscriber callbacks** and **new images submitted to the
GUI** in the trailing second. Neither is sensor acquisition FPS or monitor refresh.
“Since callback” measures the age of the latest image's reception, not end-to-end
camera latency. CPU/RAM and stale-frame warnings remain enabled. Temporary plots,
CSV recording and standalone benchmark code have been removed.

## RGB, depth and point clouds at 60 FPS

Start one driver, then open viewers in separate terminals:

```bash
cd ~/forge2_ws
bash scripts/run_stereo_cam.sh show_video:=false color_fps:=60 enable_depth:=true depth_fps:=60 enable_point_cloud:=true
```

```bash
cd ~/forge2_ws
bash scripts/view_stereo_cam.sh --display-fps 60
```

```bash
cd ~/forge2_ws
bash scripts/view_stereo_cam.sh --topic /stereo_cam/depth/image_raw --display-fps 60
```

These overrides leave the shared 30 FPS RGB-only defaults unchanged. The
uncolored point cloud is `/stereo_cam/depth/points`; use a PointCloud2 display
in RViz2, with best-effort QoS and the cloud's frame as the fixed frame. Launch
RViz2 with the same account, domain and camera environment. Cloud production
may be demand-driven, so enable a subscriber when checking it. The image windows
do not render 3D clouds. IR and depth alignment remain off unless enabled.

Close Orbbec Viewer and any previous camera driver before launching: only one
process can control the device. A control-access conflict is not an image-viewer
problem. Closing an image window stops that subscriber; Ctrl+C in the driver's
terminal stops acquisition. Do this before switching Ethernet ports.

Camera-only launch selects the sole connected commissioned Ethernet port. If
both have carrier, pass `camera_interface:=...`. Combined motor bringup retains
the USB camera adapter and built-in EtherCAT mapping. After testing the camera
on the built-in port, deactivate its camera NetworkManager profile before
reconnecting the motor bridge; hardware launch rejects a motor port with IPv4.
