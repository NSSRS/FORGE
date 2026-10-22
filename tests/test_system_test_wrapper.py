"""Exercise convenient launch selection with stub commands, never hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SystemTestWrapper(unittest.TestCase):
    def run_wrapper(self, *args):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            scripts = root / 'scripts'
            scripts.mkdir()
            shutil.copy(ROOT / 'scripts/run_system_test.sh', scripts)
            (scripts / 'ros2_env.sh').write_text(':\n')
            for name in ('ros2', 'sudo'):
                path = root / name
                path.write_text('#!/bin/bash\nprintf "%s\\n" "' + name + '" "$@"\n')
                path.chmod(0o755)
            return subprocess.run(['bash', str(scripts/'run_system_test.sh'), *args],
                                  env={**os.environ, 'PATH': str(root)+':'+os.environ['PATH']},
                                  text=True, capture_output=True)

    def test_default_hardware_elevation_or_flags(self):
        result = self.run_wrapper()
        self.assertEqual(result.returncode, 0, result.stderr)
        if os.geteuid() == 0:
            self.assertIn('simulate:=false', result.stdout)
            self.assertIn('execute:=true', result.stdout)
        else:
            self.assertIn('sudo\n', result.stdout)
            self.assertIn('--preserve-env=DISPLAY,XAUTHORITY,ROS_DOMAIN_ID', result.stdout)

    def test_explicit_simulation_does_not_elevate(self):
        result = self.run_wrapper('simulate:=true')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('ros2\nlaunch\nbringup\nsystem_test.launch.py', result.stdout)
        self.assertIn('execute:=false', result.stdout)
        self.assertNotIn('sudo\n', result.stdout)

    def test_invalid_mode_rejected_before_commands(self):
        result = self.run_wrapper('simulate:=typo')
        self.assertEqual(result.returncode, 2)
        self.assertNotIn('sudo\n', result.stdout)
        self.assertNotIn('ros2\n', result.stdout)

    def test_last_mode_override_wins(self):
        result = self.run_wrapper('simulate:=false', 'simulate:=true', 'execute:=false')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('sudo\n', result.stdout)
        self.assertTrue(result.stdout.endswith('execute:=false\n'))
