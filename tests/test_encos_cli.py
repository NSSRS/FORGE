"""Real CLI processes linked only to socket-free fake SOEM."""
import fcntl
import os
from pathlib import Path
import subprocess
import unittest

@unittest.skipUnless(os.environ.get('ENCOS_QUERY_TEST'), 'requires fake CLI binaries')


class CliTests(unittest.TestCase):

    def run_cli(self, kind, *args):
        return subprocess.run([os.environ['ENCOS_' + kind + '_TEST'], *args], capture_output=True, text=True, timeout=5)

    def test_query_each_bus_is_read_only_and_keeps_other_passages_empty(self):
        for bus, expected in [(1, '25,0,0,0,0,0'), (2, '0,0,0,25,0,0')]:
            r = self.run_cli('QUERY', f'forge-cli-can{bus}', '--can-bus', str(bus))
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn('Discovered motor CAN ID: 1.', r.stdout)
            self.assertIn('position 12.500000 deg', r.stdout)
            self.assertIn('frames=' + expected, r.stdout)
            self.assertIn('config=0 controls=0 invalid=0 cleared=1', r.stdout)

    def test_known_id_query_and_default_can1(self):
        for extra in ([], ['--can-bus', '1']):
            r = self.run_cli('QUERY', 'forge-cli-can1-id4', '--motor-ids', '4', *extra)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertNotIn('Discovered motor', r.stdout)
            self.assertIn('Motor 4:', r.stdout)

    def test_wrong_transmit_bus_fails_but_receive_entry_order_is_not_assumed(self):
        r = self.run_cli('QUERY', 'forge-cli-can2', '--can-bus', '1')
        self.assertEqual(r.returncode, 1)
        self.assertIn('No valid motor-ID reply', r.stderr)
        self.assertIn('config=0 controls=0 invalid=0 cleared=1', r.stdout)
        r = self.run_cli('QUERY', 'forge-cli-can2-packed-reply', '--can-bus', '2')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('Discovered motor CAN ID: 1.', r.stdout)
        self.assertIn('frames=0,0,0,25,0,0', r.stdout)

    def test_assigns_once_on_selected_bus_then_clears(self):
        for bus, suffix in ((1, ''), (2, ''), (2, '-packed-reply')):
            r = self.run_cli('ASSIGN', f'forge-cli-can{bus}{suffix}', '1', '4', '--execute', '--can-bus', str(bus))
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn('CAN ID changed from 1 to 4', r.stdout)
            self.assertIn('config=1 controls=0 invalid=0 cleared=1', r.stdout)
            self.assertIn('frames=' + ('6,0,0,0,0,0' if bus == 1 else '0,0,0,6,0,0'), r.stdout)

    def test_assignment_old_id_mismatch_sends_no_configuration(self):
        r = self.run_cli('ASSIGN', 'forge-cli-can2', '2', '4', '--execute', '--can-bus', '2')
        self.assertEqual(r.returncode, 1)
        self.assertIn('no configuration command sent', r.stderr)
        self.assertIn('config=0 controls=0 invalid=0 cleared=1', r.stdout)

    def test_invalid_arguments_do_not_open_transport(self):
        for kind, args in [('QUERY', []), ('QUERY', ['forge-cli-can2', '--can-bus', '3']), ('QUERY', ['forge-cli-can2', '--can-bus', '2', '--can-bus', '1']), ('ASSIGN', ['forge-cli-can2', '1', '4']), ('ASSIGN', ['forge-cli-can2', '1', '4', '--execute', '--can-bus', '0'])]:
            r = self.run_cli(kind, *args)
            self.assertEqual(r.returncode, 2)
            self.assertNotIn('FAKE', r.stdout)

    def test_driver_interface_lock_blocks_cli_before_transport(self):
        path = Path('/tmp/forge-encos-forge-cli-locked.lock')
        with path.open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            r = self.run_cli('QUERY', 'forge-cli-locked')
        self.assertEqual(r.returncode, 1)
        self.assertIn('owned or its lock is unavailable', r.stderr)
        self.assertNotIn('Query-only mode', r.stdout)


if __name__ == '__main__':
    unittest.main()
