# Workspace guidance

- This branch is simulation-only: Onshape export, MJCF processing, and MuJoCo preview.
- Read README.md and docs/ONSHAPE_MUJOCO.md for setup and current model limitations.
- Keep simulation code in scripts/, models in src/forge_sim/model/, and tests in tests/sim/.
- Hardware drivers and ROS 2 belong on main; do not reintroduce them here.
- Never commit .env credentials, local config.json, exporter caches, or .sim-backups/.
- Local vendor files, if present, are unrelated to this branch and must remain untouched.
- Validate changes with python -m unittest discover -s tests/sim and python scripts/preview_sim.py --check.
