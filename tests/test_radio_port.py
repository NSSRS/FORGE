"""Radio discovery uses USB metadata + received CSV, never motor probing."""
import source_paths  # noqa: F401
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from bringup.radio_port import probe_radio, resolve_radio_port


def adapter(path, vid=0x1A86, pid=0x7523):
    return SimpleNamespace(device=path, vid=vid, pid=pid)


def row(age=0):
    return (','.join(map(str, [age] + [992]*16)) + '\n').encode()


class RadioPortSelection(unittest.TestCase):
    def test_explicit_override_never_probes(self):
        probe = Mock(side_effect=AssertionError('Must not probe'))
        self.assertEqual(resolve_radio_port('/dev/pts/5', ports=[], probe=probe), '/dev/pts/5')
        probe.assert_not_called()
        with self.assertRaises(ValueError):
            resolve_radio_port('', ports=[], probe=probe)

    def test_reenumerated_radio_selected_by_stream_not_first_usb_or_by_id(self):
        probe = Mock(side_effect=lambda p: p == '/dev/ttyUSB7')
        result = resolve_radio_port(ports=[adapter('/dev/ttyUSB1'), adapter('/dev/ttyUSB7'),
                                          adapter('/dev/ttyACM0', 0x0483, 0x5740),
                                          adapter('/dev/ttyS0', None, None)], probe=probe)
        self.assertEqual(result, '/dev/ttyUSB7')
        self.assertEqual([c.args[0] for c in probe.call_args_list], ['/dev/ttyUSB1', '/dev/ttyUSB7'])

    def test_excluded_alias_and_duplicate_candidates_are_not_probed_twice(self):
        with tempfile.TemporaryDirectory() as directory:
            imu, radio = Path(directory)/'imu', Path(directory)/'radio'
            alias, duplicate = Path(directory)/'imu-by-path', Path(directory)/'radio-by-id'
            alias.symlink_to(imu); duplicate.symlink_to(radio)
            probe = Mock(return_value=True)
            result = resolve_radio_port(exclude=[str(alias)], probe=probe,
                                        ports=[adapter(str(imu)), adapter(str(radio)), adapter(str(duplicate))])
            self.assertEqual(result, str(radio))
            probe.assert_called_once_with(str(radio))

    def test_no_eligible_adapter_does_not_probe_motor_controller(self):
        probe = Mock()
        with self.assertRaisesRegex(ValueError, 'No eligible CH340'):
            resolve_radio_port(ports=[adapter('/dev/ttyACM0', 0x0483, 0x5740)], probe=probe)
        probe.assert_not_called()

    def test_missing_or_ambiguous_radio_fails(self):
        ports = [adapter('/dev/ttyUSB0'), adapter('/dev/ttyUSB1')]
        for found, message in ((False, 'No fresh ESP32'), (True, 'Multiple radio')):
            with self.subTest(found=found), self.assertRaisesRegex(ValueError, message):
                resolve_radio_port(ports=ports, probe=lambda _: found)

    def test_busy_candidate_prevents_silent_fallback(self):
        probe = Mock(side_effect=[PermissionError('busy'), True])
        with self.assertRaisesRegex(ValueError, 'discovery incomplete'):
            resolve_radio_port(ports=[adapter('/dev/ttyUSB0'), adapter('/dev/ttyUSB1')], probe=probe)


class ReceiveOnlyProbe(unittest.TestCase):
    def test_fresh_fragmented_csv_after_boot_text_and_stale_rows(self):
        stream = Mock()
        stream.in_waiting = 100
        stream.read.side_effect = [b'boot\n' + row(150)*5, row()*2 + row()[:10],
                                   row()[10:] + row()*2]
        factory = Mock(return_value=stream)
        self.assertTrue(probe_radio('/dev/test', serial_factory=factory))
        factory.assert_called_once_with(port=None, baudrate=115200, timeout=0, exclusive=True)
        self.assertFalse(stream.dtr)
        self.assertFalse(stream.rts)
        self.assertEqual(stream.port, '/dev/test')
        stream.write.assert_not_called()
        stream.close.assert_called_once()

    def test_stale_malformed_and_unterminated_input_do_not_identify_radio(self):
        for data in (row(150)*6, b'noise\n'*10, b'9'*600, row()*4):
            with self.subTest(data=data[:20]):
                stream = Mock()
                stream.in_waiting = len(data)
                stream.read.side_effect = [data] + [b'']*20
                self.assertFalse(probe_radio('/dev/test', timeout=.04,
                                            serial_factory=lambda **_: stream))
                stream.write.assert_not_called()
                stream.close.assert_called_once()

    def test_failed_open_always_closes(self):
        stream = Mock()
        stream.open.side_effect = OSError('disconnected')
        with self.assertRaises(OSError):
            probe_radio('/dev/test', serial_factory=lambda **_: stream)
        stream.close.assert_called_once()
        stream.write.assert_not_called()

    def test_real_pty_receive_and_exclusive_port_released_after_probe(self):
        import pty
        from serial import Serial
        master, slave = pty.openpty()
        finished = threading.Event()
        def emit():
            while not finished.wait(.02):
                os.write(master, row())
        sender = threading.Thread(target=emit)
        sender.start()
        try:
            port = os.ttyname(slave)
            self.assertTrue(probe_radio(port, timeout=.8))
            with Serial(port, 115200, timeout=0, exclusive=True):
                pass
        finally:
            finished.set(); sender.join()
            os.close(master); os.close(slave)
