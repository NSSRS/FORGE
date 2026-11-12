"""Shape 1 wire-tip trajectory in seam coordinates (metres and seconds)."""
import numpy as np


# Dimensionless phase and transverse displacement, traced from Shape 1.
PHASE = np.array([0, .10, .30, .40, .50, .60, .80, .90, 1])
LEVEL = np.array([0, 1, 1, 0, 0, -1, -1, 0, 0])


def reference(t, origin, hz, amplitude, speed):
    """Return wire position (m) and feedforward velocity (m/s)."""
    phase = (t * hz) % 1
    i = np.searchsorted(PHASE, phase, side="right") - 1
    slope = (LEVEL[i + 1] - LEVEL[i]) / (PHASE[i + 1] - PHASE[i])
    position = np.asarray(origin) + [speed * t, amplitude * np.interp(phase, PHASE, LEVEL), 0]
    return position, np.array([speed, amplitude * hz * slope, 0])
