# Workspace guidance

- Active software is the C11 driver/utilities in `src/encos_driver/`, the Python API in `src/motors/`, and the ROS 2 packages listed below. Native Ubuntu 24.04 is the deployment target; WSL is a temporary development environment.
- See `docs/ENCOS_WORKSPACE.md` for builds, `tests/README.md` for verification, and `docs/ROS2_MOTOR_CONTROL_STATUS.md` for ROS commissioning.
- Read `README.md`, then `docs/ENCOS_BRINGUP.md` and `docs/ENCOS_TEST_BENCH.md` when hardware context is needed.
- When the local supplier bundle is available, use the active references listed in `vendor/encos/README.md`; `vendor/` is intentionally excluded from Git.
- Do not scan or extract `vendor/encos/_archive/` unless the user asks for a Windows tool, USB-CAN path, bridge firmware work, IgH master, C++ demo/video, or CAD.
- Keep vendor file contents unchanged. Implement changes in project-owned `src/`, `tests/`, `scripts/`, or `docs/` files.

- ROS packages are motor_control, imu, stereo_cam, seam_detection, teleop, visualization, bringup, and robot_description. Shared launch/config belongs to bringup; scripts are wrappers/setup only. See docs/SOURCE_LAYOUT.md.

- ENCOS motors are chassis motors; Damiao motors are arm motors on the independent DM-MC02 USB/CAN path. See `docs/DAMIAO_ARM.md` for firmware builds, wiring, and commissioning status. Hardware testing and flashing remain pending; do not treat software tests as motor validation.
