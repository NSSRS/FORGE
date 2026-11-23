"""Fixed raw-shaft cable envelope, independent of session zero/model offset."""

import json
import math
from pathlib import Path
import sys


def cable_bounds(joint):
    if joint not in (4, 6):
        raise ValueError("Fixed cable policy is available for A4 and A6")
    filename = "arm_cable_limits.json" if joint == 4 else "arm_a6_cable_limits.json"
    path = Path(__file__).resolve().parents[1] / "robot_description/config" / filename
    if not path.is_file():
        path = Path(sys.prefix) / "share/robot_description/config" / filename
        if not path.is_file():
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory("robot_description")) / "config" / filename
    policy = json.loads(path.read_text())
    reference, cw, ccw = (policy[k] for k in (
        "reference_raw_deg", "cw_from_reference_deg", "ccw_from_reference_deg"))
    if (policy.get("version") != 1 or policy.get("joint") != joint
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in (reference, cw, ccw))
            or not 0 < cw <= 270 or not 0 < ccw <= 270
            or policy.get("automatic_wrap") is not False
            or policy.get("reset_reference_on_start") is not False):
        raise ValueError(f"Invalid fixed A{joint} cable envelope")
    # Existing rig direction convention: raw-positive CCW. Never normalize modulo 360.
    return reference - cw, reference + ccw


def a4_cable_bounds():
    return cable_bounds(4)


def a6_cable_bounds():
    return cable_bounds(6)


def check_cable(raw_deg, bounds, joint):
    if not math.isfinite(raw_deg) or not bounds[0] <= raw_deg <= bounds[1]:
        raise ValueError(f"A{joint} exceeds fixed cable travel (CW/CCW 270 degrees from initial); turn back, do not wrap")


def check_a4(raw_deg, bounds):
    check_cable(raw_deg, bounds, 4)
