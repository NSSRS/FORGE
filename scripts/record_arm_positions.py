#!/usr/bin/env python3
"""Separate initial-reference / travel-end recorder."""
from pathlib import Path
import sys
source = Path(__file__).resolve().parents[1] / 'src'
sys.path[:0] = [str(source), str(source/'teleop'), str(source/'robot_description')]
from teleop.arm_recording import main
if __name__ == '__main__':
    raise SystemExit(main())
