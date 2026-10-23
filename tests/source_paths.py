from pathlib import Path as _Path
import sys as _sys
_root = _Path(__file__).resolve().parents[1]
for _name in ('motor_control','teleop','visualization','bringup','imu','stereo_cam','seam_detection'):
    _sys.path.insert(0, str(_root/'src'/_name))
_sys.path.insert(0, str(_root/'src'))
