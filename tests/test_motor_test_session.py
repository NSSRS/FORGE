"""Single-auth terminal session tests; only synthetic child programs are run."""
import source_paths  # source-checkout package imports
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from run_motor_test import command, layout


class LauncherTests(unittest.TestCase):
    def test_hardware_panes_attach_without_sudo_or_root_viewer(self):
        panes = layout(False, 44, Path('/tmp/example-session'))
        for role in ('driver', 'keyboard', 'relay'):
            self.assertIn('--session', panes[role]['command'])
            self.assertNotIn('sudo', panes[role]['command'])
            self.assertNotIn('sudo', command(role, False, 44, authenticated=True))
        self.assertNotIn('--session', panes['viewer']['command'])
        self.assertNotIn('sudo', command('viewer', False, 44))

    def test_session_owner_exit_cleans_up(self):
        self.exercise_session('owner')

    def test_ctrl_c_is_forwarded_to_child_terminal(self):
        self.exercise_session('ctrl_c')

    def exercise_session(self, stop):
        try:
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            with tempfile.TemporaryDirectory() as check:
                probe.bind(str(Path(check) / 'probe'))
            probe.close()
        except PermissionError:
            self.skipTest('requires local Unix sockets outside sandbox')
        with tempfile.TemporaryDirectory(prefix='forge-session-test-') as folder:
            path = Path(folder)
            read_fd, write_fd = os.pipe()
            # A real TTY is essential for the keyboard node's cbreak input.
            child = ("import os,sys,time; print('TTY='+str(sys.stdin.isatty()),flush=True); "
                     "print('PID='+str(os.getpid()),flush=True); "
                     "print('ECHO='+input(),flush=True); time.sleep(60)")
            code = ('import sys,os; sys.path.insert(0,sys.argv[1]); '
                    'from bringup.motor_test_session import serve; '
                    'serve(sys.argv[2], {"keyboard":[sys.executable,"-u","-c",sys.argv[4]]}, '
                    'os.getuid(),int(sys.argv[3]))')
            server = subprocess.Popen([sys.executable, '-c', code, str(ROOT/'src/bringup'),
                                       folder, str(read_fd), child], pass_fds=(read_fd,),
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            os.close(read_fd)
            try:
                deadline = time.monotonic() + 5
                while not (path/'ready').exists():
                    if server.poll() is not None:
                        self.fail(server.communicate()[1].decode())
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.02)
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                    sock.settimeout(5); sock.connect(str(path/'keyboard.sock'))
                    data = b''
                    while b'PID=' not in data or not data.endswith(b'\n'):
                        data += sock.recv(4096)
                    self.assertIn(b'TTY=True', data)
                    sock.sendall(b'w\n')
                    while b'ECHO=w' not in data:
                        data += sock.recv(4096)
                    if stop == 'owner':
                        os.close(write_fd); write_fd = None
                    else:
                        sock.sendall(b'\x03')
                    while sock.recv(4096):
                        pass
                if write_fd is not None:
                    os.close(write_fd); write_fd = None
                stdout, stderr = server.communicate(timeout=8)
                self.assertEqual(server.returncode, 0, stderr.decode())
                self.assertFalse((path/'ready').exists())
                self.assertFalse((path/'keyboard.sock').exists())
            finally:
                if write_fd is not None:
                    os.close(write_fd)
                if server.poll() is None:
                    server.terminate()
                    server.wait(timeout=8)
