"""EC-A4310-P2-36 output-shaft velocity servos (user-supplied datasheet).

Rated: 12 Nm, 75 RPM. Peak: 36 Nm, 89 RPM, not enabled for continuous use.
The velocity gain is a simulation tuning parameter, not a measured motor gain.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

DRIVEN_JOINTS = tuple(f"wheel_{i}" for i in range(1, 5))
RATED_TORQUE_NM = 12.0
RATED_RPM = 75.0
VELOCITY_GAIN = 2.0  # Nm / (rad/s), tune against measured speed response later.


def install_velocity_actuators(folder: Path):
    path = folder / "robot.xml"
    tree = ET.parse(path)
    root = tree.getroot()
    joints = {j.get("name") for j in root.findall(".//joint")}
    if not joints.intersection(DRIVEN_JOINTS):
        return  # Generic export fixtures/other models need no FORGE motor setup.
    if not set(DRIVEN_JOINTS).issubset(joints):
        raise ValueError("Motor setup requires wheel_1 through wheel_4")
    actuators = root.find("actuator")
    if actuators is None:
        actuators = ET.SubElement(root, "actuator")
    for elem in list(actuators):
        if elem.get("joint") in DRIVEN_JOINTS:
            actuators.remove(elem)
    limit = RATED_RPM * math.pi / 30
    for joint in DRIVEN_JOINTS:
        ET.SubElement(actuators, "velocity", name=f"{joint}_velocity", joint=joint,
                      gear="1", kv=str(VELOCITY_GAIN), ctrllimited="true",
                      ctrlrange=f"{-limit:.15g} {limit:.15g}", forcelimited="true",
                      forcerange=f"{-RATED_TORQUE_NM:g} {RATED_TORQUE_NM:g}")
    ET.indent(tree)
    tree.write(path, encoding="utf-8", xml_declaration=True)


def set_wheel_rpm(model, data, rpm):
    values = np.asarray(rpm, dtype=float)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("Supply four finite RPM values for wheel_1 through wheel_4")
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{j}_velocity")
           for j in DRIVEN_JOINTS]
    if min(ids) < 0:
        raise ValueError("Four wheel velocity actuators are required")
    data.ctrl[ids] = np.clip(values, -RATED_RPM, RATED_RPM) * math.pi / 30


# Keyboard tuning: ordering is wheel_1, wheel_2, wheel_3, wheel_4.
# W follows the CAD front (+Z at wall spawn); left/right are robot-relative.
DRIVE_RPM = 60.0
TURN_RPM = 30.0
WHEEL_SIDES = ("left", "left", "right", "right")
WHEEL_DIRECTIONS = (-1.0, -1.0, 1.0, 1.0)
WHEEL_RPM_SCALES = (1.0, 1.0, 1.0, 1.0)


def keyboard_rpm(key):
    """Latched WASD commands; X commands zero speed. Unknown keys return None."""
    key = key.upper()
    commands = {"W": (DRIVE_RPM, 0), "S": (-DRIVE_RPM, 0),
                "A": (0, TURN_RPM), "D": (0, -TURN_RPM), "X": (0, 0)}
    if key not in commands:
        return None
    if not all(len(v) == 4 for v in (WHEEL_SIDES, WHEEL_DIRECTIONS, WHEEL_RPM_SCALES)):
        raise ValueError("Keyboard wheel parameters must each contain four entries")
    if any(side not in ("left", "right") for side in WHEEL_SIDES):
        raise ValueError("Wheel sides must be left or right")
    forward, turn = commands[key]
    speeds = [(forward + (-turn if side == "left" else turn)) * direction * scale
              for side, direction, scale in zip(WHEEL_SIDES, WHEEL_DIRECTIONS, WHEEL_RPM_SCALES)]
    if not np.isfinite(speeds).all():
        raise ValueError("Wheel keyboard parameters must be finite")
    return np.clip(speeds, -RATED_RPM, RATED_RPM)
