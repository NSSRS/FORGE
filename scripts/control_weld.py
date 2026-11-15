#!/usr/bin/env python3
"""Run the supported weld-motion panel; no hardware unless --execute is passed."""
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
from controls.hardware.weld_app import main

if __name__ == "__main__":
    raise SystemExit(main())
