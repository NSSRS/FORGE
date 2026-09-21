# Workspace guidance

- This is the simulation-only forge_sim branch; hardware and ROS 2 remain on main.
- Read README.md for setup, export, GPU rendering, and model limitations.
- Keep preview_sim.py, export_onshape.py, terrain.py and setup files at the root; helpers belong in forge_sim/, tests in tests/, and models in model/.
- Never commit .env, model/config.json, exporter caches, or .sim-backups/.
- Do not modify ignored vendor files or hardware records remaining locally.
- Validate with python -m unittest discover -s tests -p 'test_*.py' and python preview_sim.py --check.
