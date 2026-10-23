# Weld joint edge observation

**Experimental code — requires verification on actual weld joints before use
for guidance. Synthetic tests do not establish real-world detection accuracy.**

`seam_detection` consumes an existing RGB ROS stream. It finds image-space edge
pairs and draws a candidate centerline. It does not classify a weld seam,
estimate robot pose, or command motors. This first version targets a locally
straight, roughly image-vertical joint; curved joints, junctions and heavily
occluded edges need a later detector.

## Launch

Already built on the NUC. After pulling onto another built ROS workspace:

```bash
cd ~/forge2_ws
source scripts/ros2_env.sh
colcon --log-base logs/colcon build --base-paths src --build-base build/ros2 --install-base install/ros2 --symlink-install --packages-up-to bringup
```

If the camera is already streaming, leave its driver running and only start the
detector. Otherwise start RGB in one terminal:

```bash
cd ~/forge2_ws
bash scripts/run_stereo_cam.sh show_video:=false color_fps:=30
```

In another terminal:

```bash
cd ~/forge2_ws
bash scripts/run_seam_edges.sh
```

Both wrappers select camera-only shared-memory DDS on domain 42. Use the same
account for both. No depth or point-cloud stream is required. The detector
does not launch another camera connection. Avoid duplicate camera drivers.

## Window controls

- Left: RGB with search region, green/yellow candidate edges, magenta midpoint.
- Right: Canny edges inside the search region, black elsewhere.
- **R** opens an unannotated image: drag a region around the joint and press
  Enter/Space to accept, or Escape to cancel. Include both edges and a long
  visible section. The normalized region is printed for saving in the profile.
- **Canny low/high** sliders control edge sensitivity. Increase thresholds to
  suppress weak texture; decrease them if the joint is missing from the edge map.
- **Min/Max width px** constrain horizontal edge separation in the resized
  processing image. Invalid slider combinations are clamped to valid bounds.
- **Q**, Escape or closing the window exits this subscriber only.

Settings persist only when edited in `src/bringup/config/seam_edges.yaml`.
The default processing image is 640 pixels wide; the entire image is resized
before applying the region. Width bounds are **pixels, not millimeters**, and
must be retuned if processing width changes. The intended physical joint is
10–20 mm, but this version cannot verify that without calibration.

The overlay's edge support is the fraction of rows with nearby edge pixels on
both fitted boundaries. It is not a probability that the object is a weld seam.
Scratches, shadows, plate markings and other parallel features can be selected.
The strongest supported pair is selected without a temporal tracker; identity
can jump between competing pairs. A narrow search region is useful initially.

The footer shows ROS callback rate, processed-image rate, detection time, and
host CPU/RAM. Detection time excludes conversion, DDS and display. After one
second without a new image, the window clears the old result and shows stale
status. Failed detections do not reuse a prior centerline.

## ROS interface and implementation

Shared launch and parameters live in `bringup`; processing lives in
`src/seam_detection/seam_detection/`. A dedicated executor receives best-effort,
depth-one RGB messages into the existing bounded latest-frame buffer. Processing
and GUI run separately from reception, capped at 30 iterations/s by default.

The pipeline is resize → ROI → grayscale → Gaussian blur → Canny → probabilistic
Hough segments → pair filtering by orientation, common vertical span, pixel
width and edge support. Pair search is capped at 40 line segments. It supports
either bright or dark line pairs, with no depth filling or temporal smoothing.

Output images, generated only when subscribed:

| Topic | Encoding | Contents |
| --- | --- | --- |
| `/seam_detection/annotated` | bgr8 | Resized RGB with candidate edges and centerline |
| `/seam_detection/edges` | mono8 | Full resized image dimensions; ROI edges only |

Both preserve the source header timestamp and frame ID, but their pixel
coordinates are resized. Do not apply the original CameraInfo intrinsics
without scaling. Subscribers must accept best-effort delivery and check header
freshness; no new output is published when input stops. No metric path or
steering command is published.

Headless/custom-topic launches:

```bash
bash scripts/run_seam_edges.sh show_video:=false
bash scripts/run_seam_edges.sh input_topic:=/another_camera/color/image_raw
bash scripts/run_seam_edges.sh config:=/absolute/path/to/seam_edges.yaml
```

## Validation and next test

Synthetic tests cover a dark straight joint, tilted bright pair, blank frames,
a single boundary, horizontal features, width rejection, ROI exclusion, input
validation and loss of a previous candidate. A headless ROS test publishes a
synthetic image and verifies both output encodings, dimensions and source headers.
These checks do not establish accuracy or robustness on real steel or an active
welding arc. Start with an unwelded joint and arc off, select the region, and
inspect both boundaries while moving the camera before considering control.
