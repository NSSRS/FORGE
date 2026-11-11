#!/usr/bin/env python3
"""Source-checkout launcher for the flat-ground weave experiment."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for package in ("simulation", "robot_description"):
    sys.path.insert(0, str(ROOT / "src" / package))

from simulation.controllers.weave_xy_z import main

if __name__ == "__main__":
    main()
