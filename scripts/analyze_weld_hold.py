#!/usr/bin/env python3
"""Summarize cached HOLD-01 journals without opening hardware."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/controls"))
from controls.hardware.hold_analysis import main

if __name__ == "__main__":
    raise SystemExit(main())
