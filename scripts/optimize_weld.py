#!/usr/bin/env python3
"""Offline CMA-ES search; this entry point has no motor-owner options."""
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
from controls.controller.weld_search import main

if __name__ == "__main__":
    raise SystemExit(main())
