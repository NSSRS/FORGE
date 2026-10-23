"""Thread-safe latest-image handoff; never queues full-resolution images."""
from collections import deque
from threading import Lock


class FrameBuffer:
    def __init__(self):
        self.lock = Lock()
        self.latest = None
        self.samples = deque(maxlen=4096)

    def put(self, message, received):
        with self.lock:
            self.latest = (message, received)
            # Lightweight telemetry only; bounded even if rendering stalls.
            self.samples.append(received)

    def take(self):
        with self.lock:
            latest, self.latest = self.latest, None
            samples = list(self.samples)
            self.samples.clear()
        return latest, samples
