#!/usr/bin/env python3
"""Source-checkout launcher; implementation lives in visualization."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
for package in ('motor_control','teleop','visualization','bringup','imu'):
    sys.path.insert(0, str(ROOT / 'src' / package))
from visualization.mujoco_motor_viewer import Projection, main
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
