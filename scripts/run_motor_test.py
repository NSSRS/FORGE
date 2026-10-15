#!/usr/bin/env python3
"""Source-checkout launcher; implementation lives in bringup."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
for package in ('motor_control','teleop','visualization','bringup','imu'):
    sys.path.insert(0, str(ROOT / 'src' / package))
from bringup.run_motor_test import command, layout, main
if __name__ == '__main__':
    main()
