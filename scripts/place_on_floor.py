"""Place the Onshape XZ-plane assembly wheel-side down on MuJoCo's XY floor."""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def place_on_floor(folder: Path, clearance: float = .001):
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
