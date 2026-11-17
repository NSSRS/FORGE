"""Observation-only mechanical oscillation frequency from cached shaft angles."""

from collections import deque

import numpy as np


class MotionFrequency:
    """Estimate each shaft's repeating pattern; never substitute planned Hz.

    An eight-second bounded window and normalized autocorrelation distinguish
    a two-cycle response from its stronger harmonic. At least three periods,
    sufficient encoder excursion and a repeatable waveform are required.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.samples = [deque(maxlen=1000) for _ in range(6)]
        self.last_calculation = float("-inf")
        self.result = [dict(hz=None, peak_to_peak_deg=None, status="collecting") for _ in range(6)]

    def update(self, now, raw, ages):
        stale = set()
        for axis, (angle, age) in enumerate(zip(raw, ages)):
            if not np.isfinite(angle) or not np.isfinite(age) or not 0 <= age < 80:
                self.samples[axis].clear()
                self.result[axis] = dict(hz=None, peak_to_peak_deg=None, status="stale")
                stale.add(axis)
                continue
            stamp = now - age / 1000
            rows = self.samples[axis]
            if rows and stamp - rows[-1][0] > 0.1:
                rows.clear()
                self.result[axis] = dict(hz=None, peak_to_peak_deg=None, status="collecting")
            if not rows or stamp > rows[-1][0] + 1e-6:
                rows.append((stamp, float(angle)))
            while rows and stamp - rows[0][0] > 8:
                rows.popleft()
        if now - self.last_calculation >= 0.5:
            self.last_calculation = now
            self.result = [self.result[i] if i in stale else self.estimate(rows)
                           for i, rows in enumerate(self.samples)]
        return self.result

    @staticmethod
    def estimate(rows):
        result = dict(hz=None, peak_to_peak_deg=None, status="collecting")
        if len(rows) < 20:
            return result
        values = np.asarray(rows)
        span = values[-1, 0] - values[0, 0]
        excursion = float(np.ptp(values[:, 1]))
        result["peak_to_peak_deg"] = excursion
        if excursion < 0.12:
            result["status"] = "insufficient motion"
            return result
        if span < 2:
            return result
        # Uniform resampling is for observation only; never replay commands.
        dt = 0.02
        t = np.arange(values[0, 0], values[-1, 0], dt)
        q = np.interp(t, values[:, 0], values[:, 1])
        q -= q.mean()
        correlations = []
        lags = np.arange(5, len(q) // 3 + 1)  # <=10 Hz and >=3 complete repeats.
        for lag in lags:
            a, b = q[:-lag], q[lag:]
            norm = np.linalg.norm(a) * np.linalg.norm(b)
            correlations.append(float(a @ b / norm) if norm > 1e-12 else 0.0)
        correlations = np.asarray(correlations)
        peaks = [i for i in range(1, len(lags) - 1)
                 if correlations[i] >= 0.85 and correlations[i] > correlations[i-1]
                 and correlations[i] >= correlations[i+1]]
        result["status"] = "irregular / collecting"
        if not peaks:
            return result
        # Use the first sufficiently repeatable cycle, not whichever multiple
        # scores highest. Sampling a steep waveform at 25 Hz can make two
        # periods align better with the sample grid than one period does.
        i = peaks[0]
        left, mid, right = correlations[i-1:i+2]
        offset = 0.5 * (left - right) / (left - 2*mid + right)
        result.update(hz=float(1 / ((lags[i] + offset) * dt)),
                      status="measured", correlation=float(mid))
        return result
