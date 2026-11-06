#!/usr/bin/env python3
"""Capture the existing Pocket setup's axes and switches; no motor access."""
from pathlib import Path
import sys
root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/'src/teleop'))
from teleop.pocket_calibration import main
if __name__ == '__main__':
    main()
