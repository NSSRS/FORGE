"""Append cached session evidence off the motor-owner thread."""

from copy import deepcopy
import json
from pathlib import Path
import queue
import threading


class SessionJournal:
    """Bounded, nonblocking producer; flush each JSONL record while running.

    Survives a normal process crash up to the last flushed record. This is not
    power-loss durability. No motor, GUI or control callbacks run in the writer.
    """

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("x")
        self.pending = queue.Queue(maxsize=512)
        self.stopping = threading.Event()
        self.dropped = 0
        self.error = None
        self.thread = threading.Thread(target=self._write, name="weld-evidence", daemon=True)
        self.thread.start()

    def record(self, value):
        if self.error is not None or self.stopping.is_set():
            self.dropped += 1
            return
        try:
            self.pending.put_nowait(deepcopy(value))
        except queue.Full:
            self.dropped += 1

    def _write(self):
        try:
            with self.stream:
                while not self.stopping.is_set() or not self.pending.empty():
                    try:
                        value = self.pending.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    self.stream.write(json.dumps(value, allow_nan=False) + "\n")
                    self.stream.flush()
        except Exception as exc:
            self.error = str(exc)

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=2.0)
        if self.thread.is_alive():
            self.error = "Journal writer did not finish within 2 seconds"


def cached_arm_evidence(arm):
    """Read owner-maintained Python data only; never query the transport."""
    port = getattr(arm, "port", None)
    if port is None:
        return None
    report = port.report
    return dict(
        holding=port.holding,
        path_active=port.path_active,
        target_raw_rad=dict(port.hold_targets),
        speed_allowance_rad_s=port.path_speeds.tolist(),
        last_samples={key: dict(value["last_sample"])
                      for key, value in report.get("motion_summary", {}).items()},
        error=report.get("error"),
        fault_snapshot=report.get("fault_snapshot"),
        failure_states=report.get("failure_states"),
    )
