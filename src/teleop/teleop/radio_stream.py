"""Single-owner, nonblocking reader for the existing ESP32 CSV protocol."""
import time


class RadioStream:
    def __init__(self, port, *, serial_factory=None):
        if serial_factory is None:
            from serial import Serial
            serial_factory = Serial
        self.stream = serial_factory(port, 115200, timeout=0, exclusive=True)
        self.stream.reset_input_buffer()
        self.pending = bytearray()
        self.last_poll = time.monotonic()

    def poll(self, now):
        waiting = self.stream.in_waiting
        if now-self.last_poll >= .15 or waiting+len(self.pending) > 256:
            self.stream.reset_input_buffer()
            self.pending.clear()
            self.last_poll = now
            return ['']  # Invalidates intent instead of replaying buffered input.
        self.last_poll = now
        self.pending.extend(self.stream.read(waiting))
        rows = []
        while b'\n' in self.pending:
            line, _, self.pending = self.pending.partition(b'\n')
            rows.append(line.decode('ascii', errors='replace').strip())
        return rows

    def close(self):
        self.stream.close()
