"""Consolidate an exported preview to one body-local OBJ per rigid body."""
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

import fast_simplification
import mujoco
import numpy as np


def rotation(quat):
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, quat)
    return matrix.reshape(3, 3)


def body_mesh(model, geom_ids, target_faces=0):
    vertices, faces = [], []
    offset = 0
    total = sum(int(model.mesh_facenum[model.geom_dataid[i]]) for i in geom_ids)
    for geom_id in geom_ids:
        mesh_id = model.geom_dataid[geom_id]
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            raise ValueError("Merge expects mesh-only exported robot geometry")
        va, vn = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        fa, fn = int(model.mesh_faceadr[mesh_id]), int(model.mesh_facenum[mesh_id])
        # Compiled geom pose already includes MuJoCo's mesh recentering transform.
        points = model.mesh_vert[va:va + vn].astype(float)
        triangles = model.mesh_face[fa:fa + fn].copy()
        # Reduce parts separately: global decimation can erase small disconnected pieces.
        # Leave ordinary parts intact; only reduce dense parts, retaining at least half.
        budget = max(1000, int(np.ceil(fn * .5)), int(target_faces * fn / total))
        if target_faces and fn > max(50000, budget):
            points, triangles = fast_simplification.simplify(
                points, triangles, target_count=budget, agg=5.0)
        vertices.append(points @ rotation(model.geom_quat[geom_id]).T + model.geom_pos[geom_id])
        faces.append(triangles + offset)
        offset += len(points)
    return np.concatenate(vertices), np.concatenate(faces)


def verify_structure(before, after):
    """Fail promotion if any joint, body inertia, or sampled link pose changes."""
    for obj, count in ((mujoco.mjtObj.mjOBJ_BODY, before.nbody),
                       (mujoco.mjtObj.mjOBJ_JOINT, before.njnt)):
        for index in range(count):
            if mujoco.mj_id2name(before, obj, index) != mujoco.mj_id2name(after, obj, index):
                raise ValueError("Body/joint ordering changed during mesh merge")
    for field in ("nbody", "njnt", "nq", "nv", "nu", "neq"):
        if getattr(before, field) != getattr(after, field):
            raise ValueError(f"Merge changed {field}")
    for field in ("body_parentid", "body_pos", "body_quat", "body_mass", "body_inertia",
                  "body_ipos", "body_iquat", "jnt_bodyid", "jnt_type", "jnt_pos",
                  "jnt_axis", "jnt_range", "jnt_limited", "qpos0", "dof_damping",
                  "dof_frictionloss", "dof_armature", "eq_data", "eq_obj1id", "eq_obj2id"):
        np.testing.assert_allclose(getattr(after, field), getattr(before, field),
                                   atol=1e-9, rtol=1e-9, err_msg=field)
    for angle in (0.0, 0.2, -0.2):
        states = [mujoco.MjData(before), mujoco.MjData(after)]
        for model, data in zip((before, after), states):
            for j in range(model.njnt):
                if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
                    data.qpos[model.jnt_qposadr[j]] = angle
            mujoco.mj_kinematics(model, data)
        np.testing.assert_allclose(states[0].xpos, states[1].xpos, atol=1e-9)
        np.testing.assert_allclose(states[0].xmat, states[1].xmat, atol=1e-9)


def merge_model(folder: Path, target_faces: int = 10000, preserve_collision=False):
    folder = folder.resolve()
    xml_path = folder / "robot.xml"
    before = mujoco.MjModel.from_xml_path(str(folder / "scene.xml"))
    tree = ET.parse(xml_path)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None or compiler.get("meshdir") != "assets":
        raise ValueError("Expected exporter meshdir=assets")
    # Geom-name references and authored primitive collision proxies need a separate workflow.
    if root.find("contact") is not None or root.find("sensor") is not None:
        raise ValueError("Contact/sensor references need review before geometry consolidation")
    assets = root.find("asset")
    if assets is None:
        assets = ET.SubElement(root, "asset")
    if not preserve_collision:
        for mesh in list(assets.findall("mesh")):
            assets.remove(mesh)
    merged_dir = folder / "merged_assets"
    merged_dir.mkdir()
    if preserve_collision:
        shutil.copytree(folder / "assets", merged_dir, dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns("*.part"))
    compiler.set("meshdir", "merged_assets")
    report = []
    for body in root.findall(".//worldbody//body"):
        body_id = mujoco.mj_name2id(before, mujoco.mjtObj.mjOBJ_BODY, body.get("name"))
        if body_id < 0:
            raise ValueError("All exported bodies must be named")
        geoms = body.findall("geom")
        if not geoms:
            continue
        if body.find("inertial") is None:
            raise ValueError("Explicit body inertia required before simplification")
        ids = list(range(before.body_geomadr[body_id],
                         before.body_geomadr[body_id] + before.body_geomnum[body_id]))
        visual = [i for i in ids if before.geom_group[i] == 2]
        if not visual:
            visual = ids
        old_faces = sum(int(before.mesh_facenum[before.geom_dataid[i]]) for i in visual)
        vertices, faces = body_mesh(before, visual, target_faces)
        if len(faces) < 4 or not np.isfinite(vertices).all():
            raise ValueError("Simplification produced invalid geometry")
        name = f"body_{body_id:02d}_" + re.sub(r"[^a-zA-Z0-9_]", "_", body.get("name"))
        with (merged_dir / f"{name}.obj").open("w") as stream:
            np.savetxt(stream, vertices, fmt="v %.17g %.17g %.17g")
            np.savetxt(stream, faces + 1, fmt="f %d %d %d")
        ET.SubElement(assets, "mesh", name=name, file=f"{name}.obj")
        for geom, gid in zip(geoms, ids):
            if preserve_collision and (before.geom_contype[gid] or before.geom_conaffinity[gid]):
                geom.set("group", "3")
                geom.set("mass", "0")
                if not geom.get("name"):
                    geom.set("name", f"collision_{gid}")
            else:
                body.remove(geom)
        # Explicit inertia stays unchanged; one color and convex collision hull per body.
        color = before.geom_rgba[visual[0]]
        mat_id = before.geom_matid[visual[0]]
        if mat_id >= 0:
            color = before.mat_rgba[mat_id]
        collision = [i for i in ids if before.geom_contype[i] or before.geom_conaffinity[i]]
        attrs = dict(name=name, type="mesh", mesh=name, group="2", mass="0",
                     rgba=" ".join(map(str, color)), contype="0", conaffinity="0")
        if collision and not preserve_collision:
            i = collision[0]
            attrs.update(contype=str(int(np.bitwise_or.reduce(before.geom_contype[collision]))),
                         conaffinity=str(int(np.bitwise_or.reduce(before.geom_conaffinity[collision]))),
                         friction=" ".join(map(str, before.geom_friction[i])))
        ET.SubElement(body, "geom", **attrs)
        report.append(dict(body=body.get("name"), parts=len(visual),
                           original_faces=old_faces, merged_faces=len(faces)))
    if not preserve_collision:
        for material in list(assets.findall("material")):
            assets.remove(material)
    ET.indent(tree)
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
    after = mujoco.MjModel.from_xml_path(str(folder / "scene.xml"))
    verify_structure(before, after)
    # Only remove temporary export assets after successful validation.
    old_assets = (folder / "assets").resolve()
    if old_assets.parent != folder or old_assets.is_symlink():
        raise ValueError("Unsafe staging assets directory")
    shutil.rmtree(old_assets)
    merged_dir.rename(old_assets)
    compiler.set("meshdir", "assets")
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
    verified = mujoco.MjModel.from_xml_path(str(folder / "scene.xml"))
    verify_structure(before, verified)
    return report
