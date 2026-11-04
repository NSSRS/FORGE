"""Guard the established src-package/scripts-wrapper organization."""
import source_paths
import ast
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from robot_description.paths import model_directory

ROOT = Path(__file__).resolve().parents[1]
ENTRIES = ('preview_native', 'preview_sim', 'preview_arm', 'terrain', 'export_onshape', 'integrate_sim')


class ModelingLayoutTests(unittest.TestCase):
    def test_no_modeling_implementations_or_assets_at_root(self):
        for name in ('forge_sim', 'model', 'requirements-sim.txt', *(n+'.py' for n in ENTRIES)):
            self.assertFalse((ROOT/name).exists(), name)
        for package in ('simulation','robot_description'):
            base=ROOT/'src'/package
            for name in ('package.xml','setup.py','setup.cfg','resource/'+package,package+'/__init__.py'):
                self.assertTrue((base/name).is_file(), name)

    def test_launchers_are_thin_and_work_outside_checkout(self):
        with tempfile.TemporaryDirectory() as cwd:
            for name in ENTRIES:
                with self.subTest(launcher=name):
                    path=ROOT/'scripts'/(name+'.py')
                    tree=ast.parse(path.read_text())
                    self.assertFalse(any(isinstance(n,(ast.FunctionDef,ast.ClassDef)) for n in ast.walk(tree)))
                    result=subprocess.run([sys.executable,str(path),'--help'],cwd=cwd,
                                          capture_output=True,text=True,timeout=30)
                    self.assertEqual(result.returncode,0,result.stderr)
                    self.assertIn('usage:',result.stdout)

    def test_consumers_share_one_model_resource(self):
        from simulation import preview_native,preview_sim,terrain
        from visualization import mujoco_motor_viewer
        expected=ROOT/'src/robot_description/mujoco'
        self.assertEqual(model_directory(),expected)
        for value in (preview_native.MODEL_DIR,preview_sim.MODEL_DIR,terrain.MODEL,
                      mujoco_motor_viewer.model_directory()):
            self.assertEqual(value,expected)

    def test_explicit_model_override_must_exist(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(os.environ,{'FORGE_MODEL_DIR':folder}):
                with self.assertRaises(FileNotFoundError):model_directory()
                (Path(folder)/'robot.xml').write_text('<mujoco/>')
                self.assertEqual(model_directory(),Path(folder))


if __name__=='__main__':unittest.main()
