#!/usr/bin/env python3
"""Compatibility command wrapper for motor_control.commissioning."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(ROOT/'src/motor_control'))
from motor_control.commissioning import main
if __name__ == '__main__':
    raise SystemExit(main())
