"""Shared weld trajectory without the MuJoCo backend."""
import unittest

try:
    import numpy as np
except ImportError:
    raise unittest.SkipTest("requires NumPy")

import source_paths
from controls.weave_xy_z import reference


class WeaveReferenceTests(unittest.TestCase):
    def test_shape_one(self):
        origin = [.2, -.3, .1]
        for phase, lateral in [(0, 0), (.05, .5), (.11, 1), (.29, 1), (.35, .5),
                               (.45, 0), (.55, -.5), (.61, -1), (.79, -1), (.95, 0), (1, 0)]:
            t = phase / 2
            point, _ = reference(t, origin, 2, .002, .012)
            np.testing.assert_allclose(point - origin, [.012 * t, .002 * lateral, 0], atol=1e-12)
