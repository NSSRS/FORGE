"""Synthetic geometry tests; these do not validate discrimination on real steel."""
from dataclasses import replace
import unittest
import source_paths  # noqa: F401
import cv2
import numpy as np
from seam_detection.edges import Settings, detect


class SeamEdgeTests(unittest.TestCase):
    def setUp(self):
        self.settings = Settings(roi=(0., 0., 1., 1.), min_width=15, max_width=70)

    def scene(self, slope=0, width=40, bright=False):
        frame = np.full((400, 640, 3), 30 if bright else 200, np.uint8)
        polygon = np.array([[280, 0], [280+width, 0],
                            [280+width+slope, 399], [280+slope, 399]])
        cv2.fillPoly(frame, [polygon], (210,)*3 if bright else (20,)*3)
        return frame

    def test_dark_joint_centerline(self):
        result = detect(self.scene(), self.settings)
        self.assertIsNotNone(result.left)
        self.assertAlmostEqual(np.mean((result.left[:, 0]+result.right[:, 0])/2), 300, delta=3)
        self.assertAlmostEqual(np.mean(result.right[:, 0]-result.left[:, 0]), 40, delta=3)

    def test_tilted_bright_pair(self):
        result = detect(self.scene(slope=80, bright=True), self.settings)
        self.assertIsNotNone(result.left)
        self.assertAlmostEqual((result.left[1, 0]-result.left[0, 0]) /
                               (result.left[1, 1]-result.left[0, 1]), .2, delta=.03)

    def test_blank_and_single_boundary_do_not_form_joint(self):
        blank = np.full((400, 640, 3), 100, np.uint8)
        self.assertIsNone(detect(blank, self.settings).left)
        blank[:, 320:] = 230
        self.assertIsNone(detect(blank, self.settings).left)

    def test_width_outside_bounds_rejected(self):
        self.assertIsNone(detect(self.scene(width=110), self.settings).left)

    def test_horizontal_joint_rejected(self):
        frame = np.full((400, 640, 3), 200, np.uint8)
        frame[180:220] = 20
        self.assertIsNone(detect(frame, self.settings).left)

    def test_outside_roi_is_not_detected(self):
        result = detect(self.scene(), replace(self.settings, roi=(0., 0., .3, 1.)))
        self.assertIsNone(result.left)
        self.assertFalse(result.edges[:, 192:].any())

    def test_no_candidate_is_carried_into_next_frame(self):
        self.assertIsNotNone(detect(self.scene(), self.settings).left)
        self.assertIsNone(detect(np.zeros((400, 640, 3), np.uint8), self.settings).left)

    def test_invalid_settings_fail_clearly(self):
        for changes in [dict(canny_low=160), dict(roi=(0., 1., 1., 0.)),
                        dict(min_width=100, max_width=50), dict(max_angle=float('nan'))]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(self.settings, **changes).validate()


if __name__ == '__main__':
    unittest.main()
