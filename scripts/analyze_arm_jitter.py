#!/usr/bin/env python3
"""Reproduce the saved group-hold jitter analysis without opening hardware."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from motors.arm_jitter_report import main

if __name__ == '__main__':
    raise SystemExit(main())
