# Source organization

Native Linux is the deployment target. The ROS Python packages use ament_python:
`src/<package>/package.xml`, `setup.py`, and `<package>/` importable modules.

| Directory | Responsibility |
| --- | --- |
| encos_driver | C11 EtherCAT/SOEM driver and native utilities; built with CMake |
| motors | ENCOS chassis API/simulator and separate Damiao USB arm API; editable installation via root pyproject.toml |
| damiao_firmware | Portable C11 arm controller and STM32H723 MC02 board firmware; independent build |
| motor_control | ROS driver, chassis mixing, bounded ROS commissioning |
| imu | Independent IMU serial protocol, CLI and ROS node |
| stereo_cam | Orbbec camera integration, RGB/depth viewer with CPU/RAM/FPS overlay, and bounded image/calibration check |
| seam_detection | RGB joint edge candidates and annotated/edge images; no motor output |
| teleop | Keyboard/radio command generation; uses motor_control command helpers |
| visualization | Read-only relay and measured-state MuJoCo display |
| simulation | MuJoCo preview/export implementations, terrain, linkage closure and simulation controls |
| robot_description | Shared MJCF/meshes and resource lookup, chassis/arm motor inventories and Xacro macros |
| bringup | Shared YAML/DDS profiles, ROS launch files, four-pane session manager |

The motor simulator remains with motors because it substitutes for hardware.
Full physics simulation and the arm/chassis CAD model live in this checkout.
The previews and motor overview read src/robot_description/mujoco/ directly.
The seam_detection package contains the initial image-space edge detector.
robot_description contains the shared MJCF model, motor inventories and Xacro macros; no localization package has been created.

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

The detailed [arm file map](#arm-controls-and-rig-test-file-map) below identifies
the rig test's sources, profiles and generated evidence.

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
| `build/arm_tests/` | Generated arm commissioning/check/jog JSON reports |
| `build/simulation/` | Generated clearance-study JSON, MP4 and still images |
| `install/ros2/` | Installed ROS executables, resources, and environment setup |
| `logs/` | Colcon and local run logs |
| `logs/arm_tests/` | Dated software-test/build evidence and artifact manifests; not physical acceptance |

These are ignored generated assets, not additional source packages. Keep the
viewer environment and SOEM dependency for offline reuse. Deleting them
requires rerunning their setup/build helpers and may require downloads.
The old encos_query build and before-rename ROS build/install folders were removed.
The source snapshot remains locally at `tmp/before-package-reorganization.tar.gz`.

## Robot descriptions and displays

The full-robot MuJoCo MJCF model lives in src/robot_description/mujoco/ and is
used by the previews and motor viewer. robot_description packages the MJCF assets; motor Xacro macros and inventories remain installed too; a calibrated full-robot URDF remains unavailable. See [Damiao arm](DAMIAO_ARM.md).
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
snapshots are unused. The separate Damiao host API, MC02 firmware, ROS interface and bench tests remain in their existing packages.

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

## Arm controls and rig test file map

All source paths below are relative to the FORGE checkout (locally
`/home/forge2/forge2_ws`). Edit source/config here; `build/` and `install/` are
generated. Rebuild the affected ROS package after changing an installed asset.

| What | Source location and owner |
| --- | --- |
| Current status and next session | [docs/ARM_SESSION_HANDOFF.md](ARM_SESSION_HANDOFF.md) |
| Operator procedure, expected behavior and run commands | [docs/ARM_RIG_TEST.md](ARM_RIG_TEST.md) |
| Future calibration, holding and acceptance | [docs/ASSEMBLED_ARM_BRINGUP.md](ASSEMBLED_ARM_BRINGUP.md), [docs/ARM_LAYER_TODOS.md](ARM_LAYER_TODOS.md) |
| Check entry point | [scripts/test_dm_arm.py](../scripts/test_dm_arm.py), a thin wrapper over [src/motors/arm_bench.py](../src/motors/arm_bench.py) |
| Motor transport, disabled probing/explicit recovery, manual position lifecycle and fault capture | [src/motors/arm.py](../src/motors/arm.py), [arm_bench.py](../src/motors/arm_bench.py), [arm_position.py](../src/motors/arm_position.py) |
| A2–A6 selection, motor IDs/routes and fixture test envelope | [src/robot_description/config/arm_rig_a2_a6.json](../src/robot_description/config/arm_rig_a2_a6.json) |
| Pocket channel mapping, measured CW signs, 5°/s and explicit manual travel | [src/bringup/config/pocket_controls_rig.json](../src/bringup/config/pocket_controls_rig.json) |
| Pocket input/primer policy and single motor owner | [src/teleop/teleop/pocket_input.py](../src/teleop/teleop/pocket_input.py), [pocket_arm.py](../src/teleop/teleop/pocket_arm.py), [pocket_teleop.py](../src/teleop/teleop/pocket_teleop.py) |
| Dashboard/readout | [src/teleop/teleop/pocket_gui.py](../src/teleop/teleop/pocket_gui.py), [pocket_display.py](../src/teleop/teleop/pocket_display.py), [arm_widgets.py](../src/teleop/teleop/arm_widgets.py) |
| Keyboard entry point and implementation | [scripts/jog_dm_arm.py](../scripts/jog_dm_arm.py) → [src/teleop/teleop/arm_keyboard.py](../src/teleop/teleop/arm_keyboard.py) |
| Startup radio USB identification | [src/bringup/bringup/radio_port.py](../src/bringup/bringup/radio_port.py), called before nodes start; [tests/test_radio_port.py](../tests/test_radio_port.py) covers stream identification/exclusions |
| Combined launch and argument wiring | [src/bringup/launch/pocket.launch.py](../src/bringup/launch/pocket.launch.py) → [src/bringup/bringup/system_test.py](../src/bringup/bringup/system_test.py) |
| Native fake-motor, policy, Qt, launch and ROS/PTY tests | [tests/README.md](../tests/README.md#a2a6-supported-rig-a1-disconnected) lists the `tests/test_damiao_*`, `test_pocket_*`, `test_system_test_launch.py` and `test_ros_pocket.py` modules |
| MuJoCo constraint study | [scripts/preview_arm_sweep.py](../scripts/preview_arm_sweep.py) → [src/simulation/simulation/arm_sweep.py](../src/simulation/simulation/arm_sweep.py); shared XML/meshes in [src/robot_description/mujoco/](../src/robot_description/mujoco/); method/results in [docs/ARM_COLLISION_STUDY.md](ARM_COLLISION_STUDY.md) |

The normal six-motor profiles `arm_jog.json` and `pocket_controls.json` remain
separate from the A2–A6 rig profiles. `robot_description/setup.py` installs motor
JSON profiles under `share/robot_description/config/`; `bringup/setup.py` lists
radio profiles under `share/bringup/config/`. Both rig profiles belong in those
install manifests as well as the source tree. No reusable implementation belongs
in a copied rig script or a new top-level package.

Runtime JSON belongs in `build/arm_tests/`: the rig guide names
`rig-a2-a6-check.json`, `rig-a2-a6-pocket.json`, `rig-a2-a6-keyboard.json` and
`rig-a2-a6-recovery.json`. Explicit recovery also saves unique
`rig-a2-a6-recovery.fault-<unique>.json` evidence beside its report before clearing
the MCU latch; later report overwrites do not replace those files.
Pocket receives its destination through `arm_report:=...`; its summary is
written when the worker/session closes, not continuously. Check/keyboard use
`--report`. Reusing a filename replaces that report, so use dated filenames
when preserving multiple runs. ROS launch logs use ROS's log directory
(normally `~/.ros/log/`), separate from these structured reports.

Generated commissioning evidence belongs under `logs/arm_tests/<dated-run>/`
and `build/arm_tests/`; model media/numerical results belong under
`build/simulation/`. These folders are ignored. Keep reproduction commands,
configuration and summarized measured outcomes in tracked documentation.

`docs/ARM_SESSION_HANDOFF.md` is the current operator/agent entry point.
`docs/ARM_LAYER_TODOS.md` holds open requirements, not a chronological log.
Package `TODO.md` files point to owning IDs. Completed implementation belongs
in the API/operator guides; old incidents and exact measurements stay in dated
evidence. This avoids conflicting deployment/acceptance status across files.
