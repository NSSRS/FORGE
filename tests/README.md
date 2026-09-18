# Simulation tests

From the repository root with the simulation virtual environment activated:

```bash
python -m unittest discover -s tests/sim -v
python scripts/preview_sim.py --check
```

The tests cover continuous-joint export compatibility, failed-export preservation,
asset promotion, and merged geometry/joint structure. The smoke check loads the
committed scene and runs 500 physics steps without a display or Onshape credentials.
