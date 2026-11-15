#!/usr/bin/env python3
"""Inspect or regenerate the measurement-derived model alignment; no hardware."""
from pathlib import Path
import sys

source = Path(__file__).resolve().parents[1] / "src"
sys.path[:0] = [str(source), str(source / "controls")]
from controls.controller.calibration import main

if __name__ == "__main__":
    raise SystemExit(main())
