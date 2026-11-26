"""Exercise the query-only probe using fake SOEM and a pseudo-terminal."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import tempfile
import termios
import time
import unittest
import source_paths  # noqa: F401
from motors.encos_debug import inspect_debug


@unittest.skipUnless(os.environ.get("ENCOS_FEEDBACK_PROBE_TEST"), "requires fake CLI binary")
class FeedbackProbeTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.report = Path(directory.name) / "probe.jsonl"
        self.binary = os.environ["ENCOS_FEEDBACK_PROBE_TEST"]

    def events(self):
        return [json.loads(line) for line in self.report.read_text().splitlines()]

    def test_four_routed_challenges_and_passive_serial(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        saved = termios.tcgetattr(slave)
        message = b"debug\x00\xff\n" + bytes.fromhex("a00141000000")
        with subprocess.Popen(
            [self.binary, "forge-probe", self.report, "--debug-port", os.ttyname(slave), "115200"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        ) as process:
            deadline = time.monotonic() + 3
            while termios.tcgetattr(slave) == saved and process.poll() is None:
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.01)
            os.write(master, message)
            stdout, stderr = process.communicate(timeout=15)
        self.assertEqual(process.returncode, 0, stderr)
        self.assertIn("config=0 controls=0 invalid=0 cleared=1", stdout)
        self.assertEqual(termios.tcgetattr(slave), saved)
        self.assertFalse(select.select([master], [], [], 0)[0], "probe transmitted serial data")
        events = self.events()
        serial = b"".join(bytes.fromhex(e["hex"]) for e in events if e["event"] == "debug_rx")
        self.assertEqual(serial, message)
        self.assertTrue(events[-1]["completed"])
        self.assertFalse(events[-1]["can_freshness_verified"])
        self.assertEqual(events[-1]["debug_rx_bytes"], len(message))
        with self.report.open() as journal:
            inspection = inspect_debug(journal)
        self.assertEqual(inspection["outcome"], "matching_reply_payloads_observed")
        self.assertIn("binary", inspection["same_stage_matches"][0]["formats"])
        bridge = next(e for e in events if e["event"] == "bridge")
        self.assertEqual(bridge["software_100a_hex"], b"5.11".hex())
        self.assertIsNone(bridge["hardware_1009_hex"])
        checks = [e for e in events if e["event"] == "stage_result"]
        self.assertEqual(len(checks), 13)
        self.assertTrue(all(e["expected_records_seen"] for e in checks))
        for event in (e for e in events if e["event"] == "stage"):
            out = bytes.fromhex(event["output_hex"])
            self.assertEqual(out[:2], b"\x05\x00")
            for motor, record in zip([1, 2, None, 3, 4, None], struct.iter_unpack("<IBB8s", out[2:])):
                if motor is None or (event["stage"] == "pause" and motor == event["id"]):
                    self.assertEqual(record, (0, 0, 0, bytes(8)))
                else:
                    code = 30 if event["stage"] == "version" and motor == event["id"] else 1
                    self.assertEqual(record, (motor, 0, 2, bytes([0xe0, code]) + bytes(6)))
        for event in (e for e in checks if e["stage"] == "pause"):
            expected = event["late_frames"] if event["id"] % 2 else 0
            self.assertGreater(event["late_frames"], 0)
            self.assertEqual(event["late_target_position_frames"], expected)
            self.assertEqual(event["late_target_matching_frames"], expected)
            self.assertEqual(event["late_target_zero_dlc_frames"],
                             event["late_frames"] if event["id"] == 2 else 0)
            self.assertGreater(event["late_toggle_edges"], 0)
            self.assertGreater(event["late_state_edges"], 0)

    def test_missing_challenge_response_still_collects_other_ids(self):
        result = subprocess.run([self.binary, "forge-probe-no-version", self.report],
                                capture_output=True, text=True, timeout=15)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("config=0 controls=0 invalid=0 cleared=1", result.stdout)
        results = [e for e in self.events() if e["event"] == "stage_result" and e["stage"] == "version"]
        self.assertEqual([(e["id"], e["expected_records_seen"]) for e in results],
                         [(1, False), (2, True), (3, True), (4, True)])
        self.assertFalse(self.events()[-1]["completed"])

    def wait_marker(self, process, marker, motor=4):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            if self.report.exists() and f'"id":{motor},"stage":"{marker}"' in self.report.read_text():
                return
            self.assertIsNone(process.poll(), 'probe exited before ' + marker)
            time.sleep(.01)
        self.fail('missing marker ' + marker)

    def reply_loss(self, name, *, mute=True, restore=True, peer_loss=False):
        self.report = self.report.with_name(name + '.jsonl')
        gate = self.report.with_suffix('.gate')
        with subprocess.Popen([self.binary, 'forge-probe-' + name, self.report, '--reply-loss', '4'],
                              env={**os.environ, 'ENCOS_TEST_REPLY_GATE': str(gate)},
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, text=True) as process:
            self.wait_marker(process, 'await_suppression')
            if mute:
                gate.write_text('4')
            process.stdin.write('\n')
            process.stdin.flush()
            if not peer_loss:
                self.wait_marker(process, 'await_restoration')
                if restore:
                    gate.unlink(missing_ok=True)
                process.stdin.write('\n')
                process.stdin.flush()
            stdout, stderr = process.communicate(timeout=8)
        self.assertIn('config=0 controls=0 invalid=0 cleared=1', stdout)
        events = self.events()
        self.assertEqual(events[-1]['completed'], process.returncode == 0)
        self.assertFalse(events[-1]['can_freshness_verified'])
        for event in (e for e in events if e['event'] == 'loss_query'):
            output = bytes.fromhex(event['output_hex'])
            records = list(struct.iter_unpack('<IBB8s', output[2:]))
            self.assertEqual(output[:2], bytes([5, 0]))
            self.assertIn(records[0][3][1], [1, 30])
            for route, (motor, rtr, dlc, data) in zip([1, 2, 0, 3, 4, 0], records):
                self.assertEqual((motor, rtr, dlc), (route, 0, 2 if route else 0))
                self.assertEqual(data, bytes([0xe0, records[0][3][1]]) + bytes(6) if route else bytes(8))
        return events, stderr

    def test_isolated_loss_with_clear_or_frozen_record_and_recovery(self):
        for kind in ['clear-muted', 'retained']:
            with self.subTest(kind=kind):
                events, stderr = self.reply_loss(kind)
                self.assertTrue(events[-1]['completed'], stderr)
                window = next(e for e in events if e['event'] == 'loss_window')
                self.assertEqual(window['target_type_changes'], 0)
                self.assertGreater(window['samples'], 0)
                self.assertEqual(window['visible_target_samples'],
                                 window['samples'] if kind == 'retained' else 0)
                self.assertIn('recovery_observed', [e.get('stage') for e in events])

    def test_loss_of_each_id_on_the_same_owner(self):
        gate = self.report.with_suffix('.gate')
        with subprocess.Popen([self.binary, 'forge-probe', self.report, '--reply-loss', 'all'],
                              env={**os.environ, 'ENCOS_TEST_REPLY_GATE': str(gate)},
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, text=True) as process:
            for motor in range(1, 5):
                self.wait_marker(process, 'await_suppression', motor)
                gate.write_text(str(motor))
                process.stdin.write('\n')
                process.stdin.flush()
                self.wait_marker(process, 'await_restoration', motor)
                gate.unlink()
                process.stdin.write('\n')
                process.stdin.flush()
            stdout, stderr = process.communicate(timeout=4)
        self.assertEqual(process.returncode, 0, stderr)
        self.assertIn('config=0 controls=0 invalid=0 cleared=1', stdout)
        self.assertEqual([e['id'] for e in self.events() if e.get('stage') == 'recovery_observed'],
                         [1, 2, 3, 4])

    def test_isolated_loss_rejects_no_loss_peer_loss_or_missing_recovery(self):
        for name, options, reason in [
                ('still-live', dict(mute=False), 'kept advancing'),
                ('mute-peer', dict(peer_loss=True), 'Peer ID 3'),
                ('stays-off', dict(restore=False), 'recovery')]:
            with self.subTest(name=name):
                events, stderr = self.reply_loss(name, **options)
                self.assertFalse(events[-1]['completed'])
                self.assertIn(reason, stderr)

    def test_missing_feedback_and_failed_exchange_leave_incomplete_evidence(self):
        for name in ("missing4", "drop-wkc"):
            with self.subTest(name=name):
                self.report = self.report.with_name(name + ".jsonl")
                result = subprocess.run([self.binary, "forge-probe-" + name, self.report],
                                        capture_output=True, text=True, timeout=3)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("config=0 controls=0 invalid=0 cleared=1", result.stdout)
                self.assertFalse(self.events()[-1]["completed"])

    def test_interrupt_clears_output_and_preserves_original_report_on_retry(self):
        with subprocess.Popen([self.binary, "forge-probe-interrupt", self.report],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as process:
            deadline = time.monotonic() + 3
            while not self.report.exists() or self.report.stat().st_size < 4096:
                self.assertIsNone(process.poll())
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.01)
            process.send_signal(signal.SIGINT)
            stdout, _ = process.communicate(timeout=2)
        self.assertNotEqual(process.returncode, 0)
        self.assertIn("cleared=1", stdout)
        self.assertTrue(self.events()[-1]["interrupted"])
        original = self.report.read_bytes()
        retry = subprocess.run([self.binary, "forge-probe-interrupt", self.report], capture_output=True)
        self.assertNotEqual(retry.returncode, 0)
        self.assertNotIn(b"FAKE", retry.stdout)
        self.assertEqual(self.report.read_bytes(), original)


class DebugInspectionTests(unittest.TestCase):
    def inspect(self, chunks, *, selected=True, complete=True):
        image = bytearray(92)
        struct.pack_into("<IBB8s", image, 2, 2, 0, 8, bytes.fromhex("a01e010001010000"))
        events = [dict(event="start", schema="forge.encos_feedback_probe", version=1,
                       serial_requested=selected),
                  dict(event="bridge", expected_wkc=3), dict(event="stage", stage="version", id=2),
                  dict(event="pdo", wkc=3, input_hex=image.hex())]
        events.extend(dict(event="debug_rx", hex=chunk.hex()) for chunk in chunks)
        if complete:
            events.append(dict(event="end", completed=True, debug_rx_bytes=sum(map(len, chunks))))
        return inspect_debug(map(json.dumps, events))

    def test_quiet_and_unrecognized_activity(self):
        self.assertEqual(self.inspect([], selected=False)["outcome"], "debug_port_not_selected")
        self.assertEqual(self.inspect([])["outcome"], "no_bytes_observed")
        result = self.inspect([b"debug ready\r\n"])
        self.assertEqual(result["outcome"], "bytes_observed_without_recognized_payloads")
        self.assertFalse(result["can_freshness_verified"])

    def test_hex_payload_across_chunks_and_incomplete_journal(self):
        result = self.inspect([b"A0 1E 01", b" 00 01 01 00 00"], complete=False)
        self.assertEqual(result["same_stage_matches"], [
            dict(stage="version", target_id=2, payload_hex="a01e010001010000", formats=["spaced_hex"])])
        self.assertFalse(result["completed"])
        self.assertFalse(result["footer_present"])
