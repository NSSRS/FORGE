#!/usr/bin/env python3
"""Offline MuJoCo animation of a saved controller plan; no hardware access."""
from pathlib import Path
import sys

source = Path(__file__).resolve().parents[1] / "src"
sys.path[:0] = [
    str(source),
    *(
        str(source / name)
        for name in ("controls", "motor_control", "simulation", "robot_description")
    ),
]
from simulation.weld_animation import main

if __name__ == "__main__":
    raise SystemExit(main())
