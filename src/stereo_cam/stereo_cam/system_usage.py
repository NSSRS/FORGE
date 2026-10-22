"""Low-cost Linux host CPU/RAM sampling for the video overlay."""
from pathlib import Path
import time


class SystemUsage:
    def __init__(self):
        self.previous = None
        self.updated = float('-inf')
        self.text = 'NUC CPU: sampling | RAM: sampling'

    def sample(self):
        now = time.monotonic()
        if now - self.updated < 1.0:
            return self.text
        self.updated = now
        try:
            # Exclude guest fields: they are already counted in user/nice.
            rows = [line.split() for line in Path('/proc/stat').read_text().splitlines()
                    if line.startswith('cpu')]
            ticks = {}
            for row in rows:
                values = [int(value) for value in row[1:9]]
                ticks[row[0]] = (sum(values), values[3] + values[4])
            rates = {}
            if self.previous is not None:
                for name, (total, idle) in ticks.items():
                    if name in self.previous:
                        old_total, old_idle = self.previous[name]
                        delta = total - old_total
                        if delta > 0:
                            rates[name] = max(0., min(100., 100 * (1 - (idle-old_idle)/delta)))
            self.previous = ticks
            memory = {parts[0].rstrip(':'): int(parts[1])
                      for line in Path('/proc/meminfo').read_text().splitlines()
                      if (parts := line.split())}
            total = memory['MemTotal']
            used = total - memory['MemAvailable']
            cpu = f"{rates['cpu']:.0f}%" if 'cpu' in rates else 'sampling'
            peak = max((value for name, value in rates.items() if name != 'cpu'), default=0.)
            self.text = (f'NUC CPU {cpu} | busiest core {peak:.0f}% | '
                         f'RAM {used/1048576:.1f}/{total/1048576:.1f} GiB ({100*used/total:.0f}%)')
        except (OSError, ValueError, KeyError, IndexError):
            self.text = 'NUC CPU/RAM unavailable'
        return self.text
