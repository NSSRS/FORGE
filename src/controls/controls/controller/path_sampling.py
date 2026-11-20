"""Time-scale-independent sampling for static arm geometry checks."""

import numpy as np


def interval_peaks(path):
    """Exact absolute speed and acceleration bounds for every cubic interval."""
    dt = np.diff(path.times)[:, None]
    a, b, c = 3 * path.curve.c[0], 2 * path.curve.c[1], path.curve.c[2]
    vertex = np.clip(np.divide(-b, 2 * a, out=np.zeros_like(a), where=a != 0), 0, dt)
    speed = np.maximum.reduce([abs(c), abs((a * dt + b) * dt + c),
                               abs((a * vertex + b) * vertex + c)])
    acceleration = np.maximum(abs(b), abs(2 * a * dt + b))
    return speed, acceleration


def geometry_times(path, *, max_step_deg=0.1, budget=150000):
    """Include every knot and bound travel within a sample by 0.1 raw degree.

    Use interval derivative extrema, never endpoint distance (which misses
    reversals). A static pose needs no repeated time samples. Uniform retiming
    leaves this sampling rule unchanged. This is sampled clearance checking,
    not a continuous collision certificate or a change to live control ticks.
    """
    if not np.isfinite(max_step_deg) or max_step_deg <= 0 or budget < 2:
        raise ValueError("Invalid geometric validation resolution/budget")
    speed, _ = interval_peaks(path)
    count = np.maximum(1, np.ceil(np.diff(path.times) * speed.max(axis=1) / max_step_deg)).astype(int)
    if int(count.sum()) + 1 > budget:
        raise ValueError("Path exceeds offline validation budget; shorten the pass")
    return np.r_[np.concatenate([np.linspace(lo, hi, int(n), endpoint=False)
                                for lo, hi, n in zip(path.times[:-1], path.times[1:], count)]),
                 path.duration]
