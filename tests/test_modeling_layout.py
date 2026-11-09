"""Guard the established src-package/scripts-wrapper organization."""
import source_paths
import ast
import os
import runpy
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from robot_description.paths import model_directory

ROOT = Path(__file__).resolve().parents[1]
ENTRIES = ('preview_native', 'preview_sim', 'preview_arm', 'preview_arm_sweep', 'terrain', 'export_onshape', 'integrate_sim')


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
        from simulation import preview_native,preview_sim,terrain,arm_sweep
        from visualization import mujoco_motor_viewer
        expected=ROOT/'src/robot_description/mujoco'
        self.assertEqual(model_directory(),expected)
        for value in (preview_native.MODEL_DIR,preview_sim.MODEL_DIR,arm_sweep.MODEL_DIR,terrain.MODEL,
                      mujoco_motor_viewer.model_directory()):
            self.assertEqual(value,expected)

    def test_installed_description_keeps_motor_and_model_resources(self):
        # Packaging must retain Nathan's motor data when adopting main's MJCF layout.
        folder=ROOT/'src/robot_description'
        previous=Path.cwd()
        try:
            os.chdir(folder)
            with patch('setuptools.setup') as setup:
                runpy.run_path(str(folder/'setup.py'), run_name='__package_test__')
            options=setup.call_args.kwargs
        finally:
            os.chdir(previous)
        files=dict(options['data_files'])
        self.assertIn('robot_description', options['packages'])
        self.assertIn('mujoco/scene.xml', files['share/robot_description/mujoco'])
        self.assertIn('mujoco/robot.xml', files['share/robot_description/mujoco'])
        self.assertNotIn('mujoco/config.json', files['share/robot_description/mujoco'])
        for name in ('arm_motors.json', 'arm_motors_final.json', 'chassis_motors.json',
                     'bench_8009p.json', 'bench_4310p.json', 'bench_4340p.json',
                     'bench_arm.json', 'bench_arm_final.json'):
            self.assertIn('config/'+name, files['share/robot_description/config'])
        self.assertCountEqual(files['share/robot_description/urdf'],
                              ['urdf/damiao_motor.xacro', 'urdf/encos_chassis.xacro'])

    def test_explicit_model_override_must_exist(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(os.environ,{'FORGE_MODEL_DIR':folder}):
                with self.assertRaises(FileNotFoundError):model_directory()
                (Path(folder)/'robot.xml').write_text('<mujoco/>')
                self.assertEqual(model_directory(),Path(folder))


if __name__=='__main__':unittest.main()
