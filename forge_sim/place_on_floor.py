"""Place the Onshape XZ-plane assembly wheel-side down on MuJoCo's XY floor."""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def place_on_floor(folder: Path, clearance: float = .001):
    scene = ET.parse(folder / "scene.xml")
    wall = scene.find(".//geom[@name='steel_wall']")
    if wall is not None:
        return place_on_wall(folder, wall, clearance=.0002)
    path = folder / "robot.xml"
    tree = ET.parse(path)
    world = tree.getroot().find("worldbody")
    frame = world.find("frame[@name='forge_floor_pose']")
    if frame is None:
        frame = ET.SubElement(world, "frame", name="forge_floor_pose")
        for body in list(world.findall("body")):
            world.remove(body)
            frame.append(body)
    # Quaternion is independent of the compiler's angle units. +90 degrees about X.
    frame.set("quat", "0.7071067811865476 0.7071067811865476 0 0")
    frame.set("pos", "0 0 0")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(folder / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lowest = np.inf
    for geom in range(model.ngeom):
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[geom]
        start, size = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = model.mesh_vert[start:start + size]
        world_z = vertices @ data.geom_xmat[geom].reshape(3, 3)[2] + data.geom_xpos[geom, 2]
        lowest = min(lowest, float(world_z.min()))
    if not np.isfinite(lowest):
        raise ValueError("No robot mesh found for floor placement")
    lift = clearance - lowest
    frame.set("pos", f"0 0 {lift:.12g}")
    ET.indent(tree)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    print(f"Floor pose: +90 deg about X, height offset {lift:.6f} m; clearance {clearance:.3f} m")
    return lift


def place_on_wall(folder: Path, wall, clearance: float = .0002):
    """Place the CAD wheel side toward the positive-Y face of an axis-aligned wall."""
    if any(key in wall.attrib for key in ("quat", "euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Wall spawn currently requires an axis-aligned steel_wall")
    wall_pos = np.fromstring(wall.get("pos"), sep=" ")
    wall_size = np.fromstring(wall.get("size"), sep=" ")
    path = folder / "robot.xml"
    tree = ET.parse(path)
    world = tree.getroot().find("worldbody")
    frame = world.find("frame[@name='forge_wall_pose']")
    if frame is None:
        frame = world.find("frame[@name='forge_floor_pose']")
    if frame is None:
        frame = ET.SubElement(world, "frame")
        for body in list(world.findall("body")):
            world.remove(body)
            frame.append(body)
    frame.set("name", "forge_wall_pose")
    frame.set("quat", "1 0 0 0")
    frame.set("pos", "0 0 0")
    roots = frame.findall("body")
    if len(roots) != 1:
        raise ValueError("Wall spawn requires a connected robot with one root")
    base = roots[0]
    if base.find("freejoint") is None and base.find("joint") is None:
        ET.SubElement(base, "freejoint", name="floating_base")
    # Compile a sibling temporary file so relative mesh paths remain valid.
    temporary = folder / ".wall-spawn.xml"
    try:
        tree.write(temporary, encoding="utf-8", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path(str(temporary))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        points = []
        for geom in range(model.ngeom):
            if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mesh = model.geom_dataid[geom]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            vertices = model.mesh_vert[start:start + count]
            points.append(vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom])
        points = np.concatenate(points)
        low, high = points.min(axis=0), points.max(axis=0)
        shift = np.array([wall_pos[0] - (low[0] + high[0])/2,
                          wall_pos[1] + wall_size[1] + clearance - low[1],
                          wall_pos[2] - (low[2] + high[2])/2])
        if high[0] - low[0] > 2*wall_size[0] or high[2] - low[2] > 2*wall_size[2]:
            raise ValueError("Robot does not fit within steel plate bounds")
        frame.set("pos", " ".join(f"{v:.12g}" for v in shift))
        ET.indent(tree)
        tree.write(path, encoding="utf-8", xml_declaration=True)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"Wall spawn: floating base; nearest surface gap {clearance*1000:.2f} mm")
    return shift
