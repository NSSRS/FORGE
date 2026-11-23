"""Crash-readable evidence without acquiring hardware or blocking the owner."""

import source_paths
from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import threading
import numpy as np

from controls.hardware.weld_recording import (
    SessionJournal, cached_arm_evidence, new_session_report,
)


class SessionOutputTests(unittest.TestCase):
    def test_relaunch_preserves_report_and_live_journal_in_separate_sessions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            latest = root / "build/weld/latest"
            first = new_session_report(root)
            journal = SessionJournal(first.with_suffix(".jsonl"))
            try:
                self.assertEqual((latest / "session.jsonl").resolve(), journal.path)
                journal.record(dict(run=1))
                second = new_session_report(root)
                self.assertNotEqual(first, second)
                journal.record(dict(run=1, after_next_launch=True))
                # Finishing the old run must not write into the latest run.
                first.write_text('{"run": 1}')
                second.write_text('{"run": 2}')
            finally:
                journal.close()
            self.assertEqual(json.loads(first.read_text()), dict(run=1))
            self.assertEqual(json.loads((latest / "session.json").read_text()), dict(run=2))
            self.assertEqual(len(journal.path.read_text().splitlines()), 2)
            self.assertFalse(second.with_suffix(".jsonl").exists())

    def test_relaunch_preserves_journal_from_crashed_run_without_final_report(self):
        with tempfile.TemporaryDirectory() as directory:
            first = new_session_report(Path(directory))
            first.with_suffix(".jsonl").write_text('{"unfinished": true}\n')
            second = new_session_report(Path(directory))
            self.assertNotEqual(first, second)
            self.assertFalse(first.exists())
            self.assertEqual(first.with_suffix(".jsonl").read_text(), '{"unfinished": true}\n')

    def test_cli_can_run_twice_without_output_argument(self):
        from controls.hardware import weld_app

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(weld_app, "new_session_report", side_effect=lambda root: new_session_report(Path(directory))), \
                patch.object(weld_app, "audit", return_value=dict(initial_issues=[], speed_report=dict(within_limits=True))), \
                redirect_stdout(io.StringIO()):
            self.assertEqual(weld_app.main(["--audit"]), 0)
            self.assertEqual(weld_app.main(["--audit"]), 0)
            root = Path(directory) / "build/weld"
            self.assertEqual(len(list((root / "sessions").glob("*/session.json"))), 2)
            self.assertEqual(json.loads((root / "latest/session.json").read_text())["initial_issues"], [])

    def test_explicit_output_keeps_existing_evidence_protected(self):
        from controls.hardware import weld_app

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(weld_app, "new_session_report") as allocate, \
                patch.object(weld_app, "audit", return_value=dict(initial_issues=[], speed_report=dict(within_limits=True))), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            path = Path(directory) / "custom.json"
            self.assertEqual(weld_app.main(["--audit", "--output", str(path)]), 0)
            original = path.read_bytes()
            self.assertEqual(weld_app.main(["--audit", "--output", str(path)]), 2)
            self.assertEqual(path.read_bytes(), original)
            allocate.assert_not_called()


class RecordingTests(unittest.TestCase):
    def test_records_are_visible_before_close_and_are_independent_copies(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'session.jsonl'
            journal = SessionJournal(path)
            value = dict(raw=[1, 2, 3])
            try:
                journal.record(value)
                value['raw'][0] = 100
                deadline = time.monotonic()+2
                while not path.read_text() and time.monotonic() < deadline:
                    time.sleep(.005)
                self.assertEqual(json.loads(path.read_text()), dict(raw=[1, 2, 3]))
            finally:
                journal.close()
            self.assertIsNone(journal.error)
            with self.assertRaises(FileExistsError):
                SessionJournal(path)

    def test_slow_disk_cannot_block_producer_and_overflow_is_reported(self):
        entered, release = threading.Event(), threading.Event()
        stream = Mock()
        stream.__enter__ = Mock(return_value=stream)
        stream.__exit__ = Mock(return_value=False)
        def write(value):
            entered.set()
            release.wait(2)
        stream.write.side_effect = write
        with tempfile.TemporaryDirectory() as directory, patch.object(Path, 'open', return_value=stream):
            journal = SessionJournal(Path(directory)/'test.jsonl')
            try:
                journal.record(dict(first=True))
                self.assertTrue(entered.wait(1))
                for n in range(600):
                    journal.record(dict(n=n))
                self.assertGreater(journal.dropped, 0)
                self.assertLessEqual(journal.pending.qsize(), 512)
            finally:
                release.set()
                journal.close()

    def test_disk_failure_is_reported_and_does_not_raise_on_control_thread(self):
        with tempfile.TemporaryDirectory() as directory:
            journal = SessionJournal(Path(directory)/'test.jsonl')
            journal.record(dict(nonfinite=float('nan')))
            journal.close()
            self.assertIsNotNone(journal.error)
            journal.record(dict(next=True))
            self.assertEqual(journal.dropped, 1)

    def test_native_evidence_only_reads_cached_python_fields(self):
        bus = Mock()
        port = SimpleNamespace(bus=bus, holding=True, path_active=True,
            hold_targets={0: 1.}, path_speeds=np.full(6, .1),
            report=dict(motion_summary={'1': dict(last_sample=dict(enabled=True, status=1))},
                        bus_diagnostics_after_stop={'1': {'health': {'tx_failed': 1}}}))
        result = cached_arm_evidence(SimpleNamespace(port=port))
        self.assertEqual(result['last_samples']['1']['status'], 1)
        self.assertEqual(result['bus_diagnostics_after_stop']['1']['health']['tx_failed'], 1)
        self.assertEqual(bus.mock_calls, [])


if __name__ == '__main__':
    unittest.main()
