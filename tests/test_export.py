"""Offline regression checks for preserving a working model during export."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
SPEC = importlib.util.spec_from_file_location("export_onshape", ROOT / "export_onshape.py")
export = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(export)


class ExportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.model = Path(self.temp.name) / "model"
        self.model.mkdir()
        shutil.copy2(ROOT / "model/scene.xml", self.model / "scene.xml")
        shutil.copy2(ROOT / "model/config.example.json", self.model / "config.example.json")
        (self.model / "robot.xml").write_text(
            '<mujoco model="forge_placeholder"><compiler angle="radian"/>'
            '<worldbody><body name="fixture"><geom type="box" size=".1 .1 .1"/>'
            '</body></worldbody></mujoco>'
        )
        self.original = (self.model / "robot.xml").read_bytes()
        self.scene = (self.model / "scene.xml").read_bytes()
        config = json.loads((self.model / "config.example.json").read_text())
        config["url"] = "https://cad.onshape.com/documents/test/w/test/e/test"
        (self.model / "config.json").write_text(json.dumps(config))
        for mocker in (
            patch.object(export, "MODEL", self.model),
            patch.object(export, "ROOT", Path(self.temp.name)),
            patch.object(export, "merge_model", return_value=[]),
            patch.object(export, "place_on_floor"),
            patch.object(export, "load_dotenv"),
            patch.dict(os.environ, ONSHAPE_ACCESS_KEY="offline-test", ONSHAPE_SECRET_KEY="offline-test"),
        ):
            mocker.start()
            self.addCleanup(mocker.stop)

    def test_failed_download_keeps_existing_model(self):
        with patch.object(export.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "export")):
            with self.assertRaises(subprocess.CalledProcessError):
                export.main()
        self.assertEqual((self.model / "robot.xml").read_bytes(), self.original)
        self.assertFalse(list(self.model.glob(".export-*")))

    def test_invalid_mjcf_keeps_existing_model(self):
        def invalid(command, **kwargs):
            (Path(command[-1]) / "robot.xml").write_text("<not-mjcf/>")
        with patch.object(export.subprocess, "run", side_effect=invalid):
            with self.assertRaises(ValueError):
                export.main()
        self.assertEqual((self.model / "robot.xml").read_bytes(), self.original)

    def test_valid_export_retains_scene_and_copies_assets(self):
        updated = self.original.replace(b"forge_placeholder", b"offline_export_fixture")
        updated = updated.replace(b'<compiler angle="radian"/>',
                                  b'<compiler angle="radian" meshdir="assets"/>')
        updated = updated.replace(b"<worldbody>",
                                  b'<asset><mesh name="fixture" file="fixture.obj"/></asset><worldbody>')

        def valid(command, **kwargs):
            stage = Path(command[-1])
            (stage / "robot.xml").write_bytes(updated)
            (stage / "scene.xml").write_text("invalid exporter scene deliberately replaced")
            (stage / "assets").mkdir()
            (stage / "assets/fixture.obj").write_text(
                "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
                "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n"
            )

        with patch.object(export.subprocess, "run", side_effect=valid):
            export.main()
        self.assertEqual((self.model / "robot.xml").read_bytes(), updated)
        self.assertEqual((self.model / "scene.xml").read_bytes(), self.scene)
        model = export.mujoco.MjModel.from_xml_path(str(self.model / "scene.xml"))
        self.assertEqual(model.nmesh, 1)


if __name__ == "__main__":
    unittest.main()
