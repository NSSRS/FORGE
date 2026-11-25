#!/usr/bin/env python3
"""Source-checkout launcher for passive ENCOS capture analysis."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from motors.encos_capture import main

if __name__ == "__main__":
    raise SystemExit(main())
