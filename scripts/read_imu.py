#!/usr/bin/env python3
"""Run the IMU terminal reader from a source checkout without sourcing ROS."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/imu'))
from imu.cli import main

if __name__ == '__main__':
    raise SystemExit(main())
