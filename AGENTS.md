# Workspace guidance

- This is the simulation-only forge_sim branch; hardware and ROS 2 remain on main.
- Read README.md for setup, export, GPU rendering, and model limitations.
- Keep Python helpers, tests, and setup files at the repository root; models belong in model/.
- Never commit .env, model/config.json, exporter caches, or .sim-backups/.
- Do not modify ignored vendor files or hardware records remaining locally.
- Validate with python -m unittest discover -s . -p 'test_*.py' and python preview_sim.py --check.
