"""Network setup regression tests: no host networking is changed."""
import unittest
import source_paths  # noqa: F401
from stereo_cam.network import network_plan


def port(name, connected=True, address=None):
    return {'ifname': name, 'flags': ['UP', 'LOWER_UP'] if connected else ['UP'],
            'addr_info': ([{'family': 'inet', 'local': address, 'prefixlen': 24}]
                          if address else [])}


class CameraNetworkTests(unittest.TestCase):
    def plan(self, ports, interface='auto'):
        return network_plan(ports, interface, '192.168.1.100/24', '192.168.1.10')

    def test_existing_address_is_untouched(self):
        self.assertEqual(self.plan([port('enp86s0', address='192.168.1.100')]),
                         ('enp86s0', []))

    def test_new_connected_port_gets_address(self):
        name, commands = self.plan([port('enp86s0', False), port('enx000ec8a22028')])
        self.assertEqual(name, 'enx000ec8a22028')
        self.assertEqual(commands, [['ip', 'address', 'add', '192.168.1.100/24', 'dev', name]])

    def test_switch_moves_only_exact_camera_address(self):
        name, commands = self.plan([port('enp86s0', False, '192.168.1.100'),
                                   port('enx000ec8a22028')])
        self.assertEqual(commands[0], ['ip', 'address', 'del', '192.168.1.100/24', 'dev', 'enp86s0'])
        self.assertEqual(commands[1][-1], name)

    def test_ambiguous_or_absent_link_rejected(self):
        for ports in ([port('enp86s0'), port('enx000ec8a22028')],
                      [port('enp86s0', False)]):
            with self.assertRaises(ValueError):
                self.plan(ports)

    def test_other_networks_are_not_overwritten(self):
        for ports in ([port('enp86s0', address='10.0.0.2')],
                      [port('enp86s0'), port('wlo1', address='192.168.1.4')],
                      [port('enp86s0'), port('enx000ec8a22028', False, '192.168.1.2')]):
            with self.assertRaises(ValueError):
                self.plan(ports)

    def test_explicit_interface_disambiguates(self):
        self.assertEqual(self.plan([port('enp86s0'), port('enx000ec8a22028')],
                                   'enx000ec8a22028')[0], 'enx000ec8a22028')


class CameraProfileTests(unittest.TestCase):
    def test_new_profile_has_no_default_route_or_autoconnect(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        from stereo_cam.network import activate_camera_profile
        with patch('stereo_cam.network.subprocess.run', return_value=SimpleNamespace(returncode=10)) as run:
            activate_camera_profile('enp86s0', '192.168.1.100/24')
        commands = [call.args[0] for call in run.call_args_list]
        self.assertIn('add', commands[1])
        self.assertEqual(commands[1][commands[1].index('connection.autoconnect')+1], 'no')
        self.assertEqual(commands[1][commands[1].index('ipv4.never-default')+1], 'yes')
        self.assertIn('up', commands[2])

    def test_reuse_matching_profile_without_modification(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        from stereo_cam.network import activate_camera_profile
        with patch('stereo_cam.network.subprocess.run', return_value=SimpleNamespace(
                returncode=0, stdout='enp86s0\n192.168.1.100/24\n')) as run:
            activate_camera_profile('enp86s0', '192.168.1.100/24')
        self.assertEqual(run.call_count, 2)

    def test_conflicting_profile_is_not_activated(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        from stereo_cam.network import activate_camera_profile
        with patch('stereo_cam.network.subprocess.run', return_value=SimpleNamespace(
                returncode=0, stdout='other-port\n10.0.0.1/24\n')) as run:
            with self.assertRaises(RuntimeError):
                activate_camera_profile('enp86s0', '192.168.1.100/24')
        self.assertEqual(run.call_count, 1)


if __name__ == '__main__':
    unittest.main()
