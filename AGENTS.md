# Workspace guidance

- Active software is under `src/encos_query/`.
- Read `README.md`, then `docs/ENCOS_BRINGUP.md` and `docs/ENCOS_TEST_BENCH.md` when hardware context is needed.
- Use the active vendor references listed in `vendor/encos/README.md`.
- Do not scan or extract `vendor/encos/_archive/` unless the user asks for a Windows tool, USB-CAN path, bridge firmware work, IgH master, C++ demo/video, or CAD.
- Keep vendor file contents unchanged. Implement changes in project-owned `src/`, `tests/`, `scripts/`, or `docs/` files.
