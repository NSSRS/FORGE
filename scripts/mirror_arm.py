#!/usr/bin/env python3
"""Inspect raw arm encoders and their model mapping; no movement controls."""
from pathlib import Path
import sys

source=Path(__file__).resolve().parents[1]/"src"
sys.path[:0]=[str(source),*(str(source/name) for name in ("visualization","simulation","robot_description"))]
from visualization.arm_pose_mirror import main

if __name__=="__main__":
    raise SystemExit(main())
