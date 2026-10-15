#!/usr/bin/env python3
"""Compatibility command wrapper for motors.commissioning."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(ROOT/'src/motor_control'))
from motors.commissioning import main
if __name__ == '__main__':
    raise SystemExit(main())
