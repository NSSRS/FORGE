#!/usr/bin/env python3
"""Rerunnable model-guided arm calibration; offline simulation by default."""
from pathlib import Path
import sys
source = Path(__file__).resolve().parents[1] / 'src'
sys.path[:0] = [str(source), *(str(source/name) for name in ('teleop','robot_description','simulation'))]
from teleop.arm_calibration import main
if __name__ == '__main__':
    raise SystemExit(main())
