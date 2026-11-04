# Source organization

Native Linux is the deployment target. The ROS Python packages use ament_python:
`src/<package>/package.xml`, `setup.py`, and `<package>/` importable modules.

| Directory | Responsibility |
| --- | --- |
| encos_driver | C11 EtherCAT/SOEM driver and native utilities; built with CMake |
| motors | Non-ROS Python motor API and its simulator; editable installation via root pyproject.toml |
| motor_control | ROS driver, chassis mixing, bounded ROS commissioning |
| imu | Independent IMU serial protocol, CLI and ROS node |
| stereo_cam | Orbbec camera integration, RGB/depth viewer with CPU/RAM/FPS overlay, and bounded image/calibration check |
| seam_detection | RGB joint edge candidates and annotated/edge images; no motor output |
| teleop | Keyboard/radio command generation; uses motor_control command helpers |
| visualization | Read-only relay and measured-state MuJoCo display |
| simulation | MuJoCo preview/export implementations, terrain, linkage closure and simulation controls |
| robot_description | Shared MJCF scenes and meshes; resource lookup for simulation and visualization |
| bringup | Shared YAML/DDS profiles, ROS launch files, four-pane session manager |

The motor simulator remains with motors because it substitutes for hardware.
Full physics simulation and the arm/chassis CAD model live in this checkout.
The previews and motor overview read src/robot_description/mujoco/ directly.
The seam_detection package contains the initial image-space edge detector.
robot_description contains the shared MJCF model; no localization package has been created.

## Scripts audit

Moved reusable implementations out of scripts:
- mujoco_motor_viewer.py → visualization/mujoco_motor_viewer.py
- run_motor_test.py and motor_test_session.py → bringup modules
- run_three_motor_velocity.py → motors/commissioning.py
- run_ros_motor_command_test.py → motor_control/commissioning.py
- IMU decoding, CLI and ROS implementation → imu modules

Small wrappers retain the familiar commands. Build/setup/environment shell
helpers remain scripts, as do the short independent-motor example and guarded
shell bench sequence. The standalone motor_test_session.py helper is removed
from scripts because it is imported implementation, not a user command.
Temporary Windows IMU bridge and duplicate decoder were already removed.

Topics are unchanged. Packages, node names, executable names, YAML keys and
documentation now use unprefixed names. Class names with Forge prefixes are
internal Python identifiers and were retained to avoid unrelated changes.
The C binary encos_query remains a read-only query utility inside encos_driver.

## Build and validation

```bash
./scripts/build.sh
source /opt/ros/jazzy/setup.bash
colcon --log-base logs/colcon build --base-paths src --build-base build/ros2 --install-base install/ros2 --symlink-install
source scripts/ros2_env.sh
ROS_DOMAIN_ID=90 python3 -m unittest discover -s tests -p 'test_*.py'
./scripts/run_motor_test.sh --simulate --dry-run
./scripts/run_motor_viewer.sh --check
```

encos_driver and motors carry COLCON_IGNORE because their CMake and Python API
builds are independent of colcon. The environment helper adds the source API
to PYTHONPATH. The GUI pane manager and viewer wrappers are checkout tools;
they intentionally locate models/config relative to this workspace.

## Generated folders

| Path | Contents |
| --- | --- |
| `build/encos_driver/` | CMake objects, native utilities, shared libraries, and tests |
| `build/encos_driver/_deps/soem-src/` | Pinned SOEM dependency source used by the native build |
| `build/ros2/` | Colcon build products for the ROS packages |
| `build/motor_viewer/` | Viewer Python environment |
| `install/ros2/` | Installed ROS executables, resources, and environment setup |
| `logs/` | Colcon and local run logs |

These are ignored generated assets, not additional source packages. Keep the
viewer environment and SOEM dependency for offline reuse. Deleting them
requires rerunning their setup/build helpers and may require downloads.
The old encos_query build and before-rename ROS build/install folders were removed.
The source snapshot remains locally at `tmp/before-package-reorganization.tar.gz`.

## Robot descriptions and displays

The full-robot MuJoCo MJCF model lives in src/robot_description/mujoco/ and is
used by the previews and motor viewer. robot_description packages the MJCF assets; a calibrated URDF/Xacro remains unavailable.
Robot-to-IMU and robot-to-camera mounting transforms are not implemented yet.
The stereo camera driver supplies its internal optical transforms and image
streams; see [camera setup](STEREO_CAM_SETUP.md). The IMU publishes sensor readings; it is not a pose estimator.

The convenient combined command `bash scripts/run_system_test.sh` defaults to
actual motors on the commissioned bench; add `simulate:=true` for simulation.
Camera defaults are RGB-only 1280x800/30 FPS, with optional 60 FPS streams.
Camera-only wrappers source `scripts/camera_env.sh` to select bringup's installed
`fastdds-camera-shm.xml`; combined motor transport remains unchanged. The image
viewer uses `stereo_cam/frame_buffer.py` for bounded latest-image delivery.

## Full robot modeling

`src/robot_description/mujoco/` contains the current arm/chassis meshes and MJCF. `src/simulation/simulation/` contains
geometry, D3 closure, colors, terrain fitting, magnets and simulated actuation.
The scripts/ launchers and simulation package and read-only wheel overview use this model.
The overview reads src/robot_description/mujoco/scene.xml directly; old build/motor_viewer/model
snapshots are unused. New arm hardware drivers/firmware are not part of this update.

### Modeling package boundaries

- `src/simulation/simulation/`: all preview/export implementations and simulation helpers.
- `src/simulation/requirements.txt`: pinned simulation/export dependencies.
- `src/robot_description/mujoco/`: shared scenes, meshes, default pose and CAD export template.
- `src/robot_description/robot_description/paths.py`: one resource lookup for source and installed packages.
- `scripts/`: thin launchers only, including preview_native.py, preview_sim.py, preview_arm.py, export_onshape.py, integrate_sim.py and terrain.py.
- `tests/`: behavioral and package-layout regressions.

Both packages follow the existing ament_python setup.py/package.xml/resource layout.
Source launchers work from any current directory. Colcon installs the shared model
under share/robot_description/mujoco. Set FORGE_MODEL_DIR to an explicit writable
model copy when generating terrain from an installed package. CAD export/integration
requires a source checkout; generated backups go under build/simulation/backups/.
