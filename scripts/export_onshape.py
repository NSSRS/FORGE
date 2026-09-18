"""Export Onshape CAD into the preview model, retaining the project scene."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from urllib.parse import urlparse

from dotenv import load_dotenv
import mujoco

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "src/forge_sim/model"


def main() -> None:
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
        raise SystemExit("Activate the simulation venv; run bash scripts/setup_sim.sh first.")
    # Failed API calls or invalid MJCF must not replace the working preview.
    with tempfile.TemporaryDirectory(prefix=".export-", dir=MODEL) as folder:
        stage = Path(folder)
        (stage / "config.json").write_text(json.dumps(config, indent=2) + "\n")
        subprocess.run([str(executable), str(stage)], cwd=ROOT, check=True)
        shutil.copy2(MODEL / "scene.xml", stage / "scene.xml")
        model = mujoco.MjModel.from_xml_path(str(stage / "scene.xml"))
        mujoco.mj_forward(model, mujoco.MjData(model))
        if (stage / "assets").exists():
            shutil.copytree(stage / "assets", MODEL / "assets", dirs_exist_ok=True)
        os.replace(stage / "robot.xml", MODEL / "robot.xml")
    print("Export complete: robot.xml and assets updated; scene.xml retained.")
    print("Run python scripts/preview_sim.py to inspect the paused model.")


if __name__ == "__main__":
    main()
