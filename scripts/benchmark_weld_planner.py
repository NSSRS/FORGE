#!/usr/bin/env python3
"""Run reproducible, hardware-free arm planner benchmarks."""
from pathlib import Path
import sys

source = Path(__file__).resolve().parents[1] / "src"
sys.path[:0] = [str(source), *(str(source / name) for name in
    ("controls", "simulation", "robot_description", "motor_control"))]

from controls.simulation.planner_benchmark import main

if __name__ == "__main__":
    raise SystemExit(main())
