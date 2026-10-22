"""Orientation-display math and stale/invalid sample rejection, no hardware."""
import importlib.util
import unittest
import source_paths  # noqa: F401


@unittest.skipUnless(importlib.util.find_spec('rclpy') and importlib.util.find_spec('cv2'),
                     'requires ROS 2 and OpenCV')
class ImuViewerTests(unittest.TestCase):
    def test_rotation_order_and_finite_values(self):
        import math
        import numpy as np
        from visualization.imu_viewer import rotation_matrix
        np.testing.assert_allclose(rotation_matrix((0,0,0)), np.eye(3))
        np.testing.assert_allclose(rotation_matrix((0,0,math.pi/2)) @ [1,0,0],
                                   [0,1,0], atol=1e-12)
        rotation = rotation_matrix((.3,-.4,.5))
        np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
        with self.assertRaises(ValueError):
            rotation_matrix((float('nan'),0,0))

    def test_camera_handedness_and_screen_directions(self):
        import numpy as np
        from visualization.imu_viewer import CAMERA_RIGHT, CAMERA_UP, CAMERA_DEPTH, project_world
        basis = np.array([CAMERA_RIGHT, CAMERA_UP, CAMERA_DEPTH])
        np.testing.assert_allclose(basis @ basis.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(basis), 1.)
        # Looking from +X,+Y,+Z: +X goes left/down, +Y right/down, +Z up.
        pixels = project_world(np.eye(3), center=(0,0), scale=1)
        self.assertLess(pixels[0,0], 0)
        self.assertGreater(pixels[0,1], 0)
        self.assertGreater(pixels[1,0], 0)
        self.assertGreater(pixels[1,1], 0)
        self.assertLess(pixels[2,1], 0)

    def test_positive_quarter_turns(self):
        import math
        import numpy as np
        from visualization.imu_viewer import rotation_matrix, render
        # Right-hand rule: roll Y->Z, pitch Z->X, yaw X->Y.
        for rpy, source, target in [((math.pi/2,0,0),[0,1,0],[0,0,1]),
                                    ((0,math.pi/2,0),[0,0,1],[1,0,0]),
                                    ((0,0,math.pi/2),[1,0,0],[0,1,0])]:
            np.testing.assert_allclose(rotation_matrix(rpy) @ source, target, atol=1e-12)
            self.assertEqual(render(rpy, .1).shape, (620,900,3))

    def test_rejects_old_replayed_and_invalid_samples(self):
        from types import SimpleNamespace as NS
        from visualization.imu_viewer import ImuViewer
        viewer = NS(last_stamp=0, rpy=None, received=None,
                    get_clock=lambda: NS(now=lambda: NS(nanoseconds=2_000_000_000)))
        message = NS(header=NS(stamp=NS(sec=1,nanosec=900_000_000)),
                     vector=NS(x=.1,y=.2,z=.3))
        ImuViewer.receive(viewer, message)
        self.assertEqual(viewer.rpy, (.1,.2,.3))
        first = viewer.received
        message.vector.x = 4
        ImuViewer.receive(viewer, message)
        self.assertEqual(viewer.received, first)
        message.header.stamp.nanosec = 950_000_000
        message.vector.x = float('nan')
        ImuViewer.receive(viewer, message)
        self.assertEqual(viewer.received, first)
        message.vector.x = 4
        message.header.stamp.nanosec = 0
        ImuViewer.receive(viewer, message)
        self.assertEqual(viewer.received, first)
