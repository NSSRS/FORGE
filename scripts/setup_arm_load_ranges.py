#!/usr/bin/env python3
"""Thin operator launcher for disabled arm load-range commissioning."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from motors.arm_load_range_setup import main

if __name__ == '__main__':
    raise SystemExit(main())
