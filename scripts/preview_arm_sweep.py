#!/usr/bin/env python3
"""Source-checkout launcher; implementation lives in simulation."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
for package in ('simulation', 'robot_description'):
    sys.path.insert(0, str(ROOT / 'src' / package))
from simulation.arm_sweep import main
if __name__ == '__main__':
    main()
