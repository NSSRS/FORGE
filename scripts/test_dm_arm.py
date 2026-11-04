#!/usr/bin/env python3
"""Run the explicit single-motor or six-motor Damiao commissioning plan."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from motors.arm_bench import main
if __name__ == '__main__':
    raise SystemExit(main())
