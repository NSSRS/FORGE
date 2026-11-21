#!/usr/bin/env python3
"""Analyze the motion accuracy of a recorded weld pass."""
from pathlib import Path
import sys

source = Path(__file__).resolve().parents[1] / "src"
sys.path[:0] = [
    str(source),
    *(str(source / package) for package in ("controls", "simulation", "robot_description")),
]
from controls.hardware.weld_accuracy import main

if __name__ == "__main__":
    raise SystemExit(main())
