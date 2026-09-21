"""Export Onshape CAD into the preview model, retaining the project scene."""
from __future__ import annotations

import json
import colorsys
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
from urllib.parse import urlparse

from dotenv import load_dotenv
import mujoco
from forge_sim.onshape_compat import prepare_meshes
from forge_sim.merge_mjcf import merge_model
from forge_sim.place_on_floor import place_on_floor
from forge_sim.velocity_control import install_velocity_actuators

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / "model"



def apply_wheel_material(folder: Path):
    tree = ET.parse(folder / "robot.xml")
    root = tree.getroot()
    geoms = [g for i in range(1, 7)
             for b in root.findall(f".//body[@name='linkage_wheel_{i}']")
             for g in b.findall("geom")]
    if not geoms:
        return
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    for tag, name in (("texture", "wheel_checker"), ("material", "wheel_checker_material")):
        for old in list(asset.findall(f"{tag}[@name='{name}']")):
            asset.remove(old)
    ET.SubElement(asset, "texture", name="wheel_checker", type="cube", builtin="checker",
                  width="256", height="256", rgb1="0.04 0.04 0.04", rgb2="0.95 0.95 0.95")
    ET.SubElement(asset, "material", name="wheel_checker_material", texture="wheel_checker",
                  rgba="1 1 1 1", texuniform="false", reflectance="0", shininess="0.1")
    for geom in geoms:
        geom.set("material", "wheel_checker_material")
        geom.set("rgba", "1 1 1 1")
    # Stable low-saturation colors for each non-wheel rigid body.
    body_colors = {"body": 0.58, "suspension_front": 0.08,
                   "suspension_rear_1": 0.32, "suspension_rear_2": 0.76,
                   "freedrive_1": 0.96, "freedrive_2": 0.46}
    for name, hue in body_colors.items():
        rgb = colorsys.hsv_to_rgb(hue, 0.28, 0.72)
        for body in root.findall(f".//body[@name='{name}']"):
            for geom in body.findall("geom"):
                geom.set("rgba", " ".join(f"{v:.4f}" for v in (*rgb, 1)))
    ET.indent(tree)
    tree.write(folder / "robot.xml", encoding="utf-8", xml_declaration=True)

def publish_model(stage: Path) -> Path:
    backup = ROOT / ".sim-backups" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    backup.mkdir(parents=True)
    shutil.copy2(MODEL / "scene.xml", backup / "scene.xml")
    moved, installed = [], []
    try:
        for name in ("assets", "robot.xml"):
            source = MODEL / name
            if source.resolve().parent != MODEL.resolve() or source.is_symlink():
                raise ValueError("Model paths must be inside the model directory")
            if source.exists():
                os.replace(source, backup / name)
                moved.append(name)
        for name in ("assets", "robot.xml"):
            os.replace(stage / name, MODEL / name)
            installed.append(name)
    except BaseException:
        for name in reversed(installed):
            os.replace(MODEL / name, stage / name)
        for name in reversed(moved):
            os.replace(backup / name, MODEL / name)
        raise
    return backup


def main(merge_existing=False, target_faces=10000) -> None:
    if merge_existing:
        with tempfile.TemporaryDirectory(prefix=".export-", dir=MODEL) as folder:
            stage = Path(folder)
            shutil.copy2(MODEL / "robot.xml", stage / "robot.xml")
            shutil.copy2(MODEL / "scene.xml", stage / "scene.xml")
            shutil.copytree(MODEL / "assets", stage / "assets")
            report = merge_model(stage, target_faces, preserve_collision=True)
            place_on_floor(stage)
            install_velocity_actuators(stage)
            apply_wheel_material(stage)
            mujoco.MjModel.from_xml_path(str(stage / "scene.xml"))
            backup = publish_model(stage)
        print_summary(report, backup)
        return
    config_path = MODEL / "config.json"
    if not config_path.exists():
        raise SystemExit("Copy config.example.json to config.json and set your assembly-tab URL first.")
    config = json.loads(config_path.read_text())
    url = urlparse(config.get("url", ""))
    if (url.scheme != "https" or url.hostname != "cad.onshape.com"
            or not url.path.startswith("/documents/") or "/e/" not in url.path
            or "YOUR_" in url.path):
        raise SystemExit("Set a real https://cad.onshape.com/documents/.../e/... assembly URL.")
    if (config.get("output_format") != "mujoco"
            or config.get("output_filename", "robot") != "robot"
            or config.get("assets_directory", "assets") != "assets"):
        raise SystemExit("Keep output_format=mujoco, output_filename=robot, assets_directory=assets.")
    load_dotenv(ROOT / ".env")
    if not all(os.environ.get(key) for key in ("ONSHAPE_ACCESS_KEY", "ONSHAPE_SECRET_KEY")):
        raise SystemExit("Set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY in the local .env or shell.")
    executable = Path(sys.executable).parent / "onshape-to-robot"
    if not executable.exists():
        raise SystemExit("Activate the simulation venv; run bash setup_sim.sh first.")
    # Failed API calls or invalid MJCF must not replace the working preview.
    with tempfile.TemporaryDirectory(prefix=".export-", dir=MODEL) as folder:
        stage = Path(folder)
        (stage / "config.json").write_text(json.dumps(config, indent=2) + "\n")
        subprocess.run([sys.executable, str(ROOT / "forge_sim" / "onshape_compat.py"), str(stage)], cwd=ROOT, check=True)
        prepare_meshes(stage / "robot.xml")
        shutil.copy2(MODEL / "scene.xml", stage / "scene.xml")
        mujoco.MjModel.from_xml_path(str(stage / "scene.xml"))
        raw_backup = ROOT / ".sim-backups" / ("raw-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
        raw_backup.mkdir(parents=True)
        shutil.copy2(stage / "robot.xml", raw_backup / "robot.xml")
        shutil.copy2(stage / "scene.xml", raw_backup / "scene.xml")
        shutil.copytree(stage / "assets", raw_backup / "assets")
        report = merge_model(stage, target_faces, preserve_collision=True)
        place_on_floor(stage)
        install_velocity_actuators(stage)
        apply_wheel_material(stage)
        model = mujoco.MjModel.from_xml_path(str(stage / "scene.xml"))
        mujoco.mj_forward(model, mujoco.MjData(model))
        backup = publish_model(stage)
    print_summary(report, backup)


def print_summary(report, backup):
    print(f"Export complete: {len(report)} merged body meshes; joints/inertia verified unchanged.")
    print(f"Triangles: {sum(x['original_faces'] for x in report)} -> {sum(x['merged_faces'] for x in report)}")
    print(f"Previous model backup: {backup}")
    print("Merged visual meshes; original per-part collision geoms retained.")
    print("Run python preview_sim.py to inspect the paused model.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--merge-existing", action="store_true", help="Merge local CAD without calling Onshape")
    parser.add_argument("--faces-per-body", type=int, default=10000, help="Triangle target per body; 0 disables reduction")
    args = parser.parse_args()
    if args.faces_per_body < 0:
        parser.error("--faces-per-body must be nonnegative")
    main(args.merge_existing, args.faces_per_body)
