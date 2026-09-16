# Workspace guidance

- Active software is the C11 driver/utilities in `src/encos_query/`, the Python API in `python/forge_motors/`, and the ROS 2 package in `ros2_ws/src/forge_motor_control/`.
- See `docs/ENCOS_WORKSPACE.md` for builds, `tests/README.md` for verification, and `docs/ROS2_MOTOR_CONTROL_STATUS.md` for ROS commissioning.
- Read `README.md`, then `docs/ENCOS_BRINGUP.md` and `docs/ENCOS_TEST_BENCH.md` when hardware context is needed.
- When the local supplier bundle is available, use the active references listed in `vendor/encos/README.md`; `vendor/` is intentionally excluded from Git.
- Do not scan or extract `vendor/encos/_archive/` unless the user asks for a Windows tool, USB-CAN path, bridge firmware work, IgH master, C++ demo/video, or CAD.
- Keep vendor file contents unchanged. Implement changes in project-owned `src/`, `python/`, `ros2_ws/src/`, `config/`, `examples/`, `tests/`, `scripts/`, or `docs/` files.
