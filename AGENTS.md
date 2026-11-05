# Workspace guidance

- Active software is the C11 driver/utilities in `src/encos_driver/`, the Python API in `src/motors/`, and the ROS 2 packages listed below. Native Ubuntu 24.04 is the deployment target; WSL is a temporary development environment.
- See `docs/ENCOS_WORKSPACE.md` for builds, `tests/README.md` for verification, and `docs/ROS2_MOTOR_CONTROL_STATUS.md` for ROS commissioning.
- Read `README.md`, then `docs/ENCOS_BRINGUP.md` and `docs/ENCOS_TEST_BENCH.md` when hardware context is needed.
- When the local supplier bundle is available, use the active references listed in `vendor/encos/README.md`; `vendor/` is intentionally excluded from Git.
- Do not scan or extract `vendor/encos/_archive/` unless the user asks for a Windows tool, USB-CAN path, bridge firmware work, IgH master, C++ demo/video, or CAD.
- Keep vendor file contents unchanged. Implement changes in project-owned `src/`, `tests/`, `scripts/`, or `docs/` files.

- ROS packages are motor_control, imu, stereo_cam, seam_detection, teleop, visualization, bringup, robot_description, and simulation. Shared launch/config belongs to bringup; scripts are wrappers/setup only. See docs/SOURCE_LAYOUT.md.

- ENCOS motors are chassis; Damiao motors are arm actuators on DM-MC02 USB/CAN. Read docs/DAMIAO_ARM.md and src/robot_description/ARM_COMMISSIONING.md before arm work. As of 2026-11-05, A1..A6 IDs 1..6/feedback17..22 are written/read back; the saved six-motor unloaded report passes all 14 group/individual stages with disable/mode restoration. A1 is on CAN1; A2..A6 are on CAN2. Use the current status sections, not historical single-motor notes. Six-axis profiles/ROS and loaded-arm operation remain unvalidated; preserve null calibration and false confirmation flags.

- Modeling follows the same source layout as other packages: reusable code in src/simulation/simulation/, shared MJCF/mesh assets in src/robot_description/mujoco/, and thin launchers/setup in scripts/. Do not add root Python entry points or top-level simulation/model directories. Read docs/ONSHAPE_ARM_PREVIEW.md before changing models; preserve D3 closure and simulation/hardware coordinate separation. Validate package layout, model compatibility, motor overview, body colors, magnetic, terrain and actuator tests. Never commit src/simulation/.env, src/robot_description/mujoco/config.json or generated backups.

- For arm/base-layer work, read `docs/ARM_BASE_LAYER.md`, `docs/ARM_LAYER_TODOS.md` and the local package `TODO.md`. Stable TODO IDs link firmware, Python, ROS, calibration, planning and bringup responsibilities. Telemetry must use cached MCU feedback, not new CAN queries while enabled. Preserve fault latches, single-owner transport and raw-shaft SI contracts.

- Manual arm API/jog: read `docs/ARM_POSITION_JOG.md`. `src/motors/arm_position.py` owns the raw-radian position port; `src/teleop/teleop/arm_keyboard.py` owns key policy. PyQt window: number selection + E once + held arrows (release stops) are unloaded commissioning, separate from ENCOS WASD. Production multi-joint/loaded operation remains pending.
