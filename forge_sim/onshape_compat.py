"""Project-local compatibility fixes for onshape-to-robot 1.8.3."""
from copy import copy
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from stl import mesh as stl_mesh


def prepare_meshes(robot_xml: Path) -> None:
    """Convert unsupported STL inputs to OBJ without decimating geometry."""
    tree = ET.parse(robot_xml)
    compiler = tree.getroot().find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""
    changed = False
    for element in tree.getroot().findall("asset/mesh"):
        filename = element.get("file", "")
        if not filename.lower().endswith(".stl"):
            continue
        source = robot_xml.parent / meshdir / filename
        raw = source.read_bytes()
        count = struct.unpack_from("<I", raw, 80)[0] if len(raw) >= 84 else -1
        if 1 <= count <= 200000 and len(raw) == 84 + 50 * count:
            continue
        triangles = stl_mesh.Mesh.from_file(str(source)).vectors
        if not len(triangles) or not np.isfinite(triangles).all():
            raise ValueError(f"Empty or invalid mesh: {source.name}")
        vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
        target = source.with_suffix(".obj")
        with target.open("w", encoding="utf-8") as stream:
            np.savetxt(stream, vertices, fmt="v %.9g %.9g %.9g")
            np.savetxt(stream, indices.reshape(-1, 3) + 1, fmt="f %d %d %d")
        element.set("file", str(Path(filename).with_suffix(".obj")))
        if "content_type" in element.attrib:
            element.set("content_type", "model/obj")
        changed = True
        print(f"Converted {source.name} to OBJ ({len(triangles)} triangles, no decimation)")
    if changed:
        tree.write(robot_xml, encoding="utf-8", xml_declaration=True)


def add_joint_compat(original, exporter, joint):
    from onshape_to_robot.robot import Joint
    if joint.joint_type == Joint.CONTINUOUS:
        joint = copy(joint)
        joint.joint_type = Joint.REVOLUTE
        joint.limits = None
        joint.properties = {**joint.properties, "range": False}
    original(exporter, joint)


def main():
    from onshape_to_robot import export
    from onshape_to_robot.exporter_mujoco import ExporterMuJoCo
    original = ExporterMuJoCo.add_joint
    ExporterMuJoCo.add_joint = lambda self, joint: add_joint_compat(original, self, joint)
    try:
        return export.main()
    finally:
        ExporterMuJoCo.add_joint = original


if __name__ == "__main__":
    main()
