"""Crash-readable evidence without acquiring hardware or blocking the owner."""

import source_paths
import json
from pathlib import Path
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import threading
import numpy as np

from controls.hardware.weld_recording import SessionJournal, cached_arm_evidence


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
            report=dict(motion_summary={'1': dict(last_sample=dict(enabled=True, status=1))}))
        result = cached_arm_evidence(SimpleNamespace(port=port))
        self.assertEqual(result['last_samples']['1']['status'], 1)
        self.assertEqual(bus.mock_calls, [])


if __name__ == '__main__':
    unittest.main()
