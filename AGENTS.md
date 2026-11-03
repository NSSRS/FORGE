# Workspace guidance

- Active software is the C11 driver/utilities in `src/encos_driver/`, the Python API in `src/motors/`, and the ROS 2 packages listed below. Native Ubuntu 24.04 is the deployment target; WSL is a temporary development environment.
- See `docs/ENCOS_WORKSPACE.md` for builds, `tests/README.md` for verification, and `docs/ROS2_MOTOR_CONTROL_STATUS.md` for ROS commissioning.
- Read `README.md`, then `docs/ENCOS_BRINGUP.md` and `docs/ENCOS_TEST_BENCH.md` when hardware context is needed.
- When the local supplier bundle is available, use the active references listed in `vendor/encos/README.md`; `vendor/` is intentionally excluded from Git.
- Do not scan or extract `vendor/encos/_archive/` unless the user asks for a Windows tool, USB-CAN path, bridge firmware work, IgH master, C++ demo/video, or CAD.
- Keep vendor file contents unchanged. Implement changes in project-owned `src/`, `tests/`, `scripts/`, or `docs/` files.

- ROS packages are motor_control, imu, stereo_cam, seam_detection, teleop, visualization, bringup, and robot_description. Shared launch/config belongs to bringup; scripts are wrappers/setup only. See docs/SOURCE_LAYOUT.md.

- ENCOS motors are chassis; Damiao motors are arm actuators on DM-MC02 USB/CAN. Read docs/DAMIAO_ARM.md and src/robot_description/ARM_COMMISSIONING.md before arm work. On 2026-11-03 one CAN2/ID 1/feedback0 J4340P reached44.83 degrees of45 in position–velocity mode; disable and mode restoration verified. Old MIT bench motion is removed. Next: 8009P, 4310P and remaining units individually, then multi-bus validation. Six-axis profiles/ROS and loaded-arm operation remain unvalidated; preserve null calibration and false confirmation flags.

- Current nathan also owns root MuJoCo preview/export tools, forge_sim/ and model/. Read docs/ONSHAPE_ARM_PREVIEW.md before changing model coordinates, viewers or collision rules. Preserve D3 closure and simulation-vs-hardware coordinate separation. Validate tests/test_model_compatibility.py plus relevant magnetic/terrain/actuator tests. Never commit .env, model/config.json or .sim-backups/.
