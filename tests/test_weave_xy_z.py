"""Regression for the shape, driven wheels and measured CAD wire tip."""
import importlib.util
import unittest
from contextlib import nullcontext
from unittest.mock import patch

import source_paths

if any(importlib.util.find_spec(n) is None for n in ("mujoco", "numpy", "scipy")):
    raise unittest.SkipTest("requires the simulation environment")

import numpy as np
from simulation.controllers import weave_xy_z
from simulation.controllers.weave_xy_z import (
    SEAM_WIDTH, SEAM_XY, TARGET_WIRE, WIRE_GAP, floor_scene, reference, run,
)


class WeaveTests(unittest.TestCase):
    def test_site_axis_points_from_cad_wire_center_to_distal_face(self):
        model, data = floor_scene()
        self.assertEqual(model.geom("preview_floor").pos[2], 0)
        self.assertEqual(data.actuator("b2_arm_d1_target").ctrl[0], 0)
        geom = data.geom("collision_795")  # APW503 solid3, the wire
        mesh = model.geom_dataid[geom.id]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = model.mesh_vert[start:start + count]
        center = geom.xpos + geom.xmat.reshape(3, 3) @ vertices.mean(axis=0)
        tip = data.site("wire_tip")
        direction = tip.xpos - center
        np.testing.assert_allclose(tip.xmat.reshape(3, 3)[:, 2], direction / np.linalg.norm(direction),
                                   atol=.0001)

    def test_shape_one(self):
        origin = np.array([.2, -.3, .1])
        for phase, lateral in [(0, 0), (.05, .5), (.11, 1), (.29, 1), (.35, .5),
                               (.45, 0), (.55, -.5), (.61, -1), (.79, -1), (.95, 0), (1, 0)]:
            t = phase / 2
            point, _ = reference(t, origin, 2, .002, .012)
            np.testing.assert_allclose(point - origin, [.012 * t, .002 * lateral, 0], atol=1e-12)

    def test_centered_floor_seam_and_full_width_reference(self):
        model, data = floor_scene()
        seam = model.site("floor_seam")
        self.assertAlmostEqual(seam.size[1] * 2, .020)
        self.assertAlmostEqual(seam.pos[1], data.body("b2_arm_d1").xpos[1], delta=.0001)
        self.assertEqual(seam.pos[2], data.geom("preview_floor").xpos[2])
        self.assertAlmostEqual(seam.pos[0] - seam.size[0], SEAM_XY[0])
        origin = np.array([*SEAM_XY, WIRE_GAP])
        np.testing.assert_allclose(TARGET_WIRE, [-np.sin(np.deg2rad(10)), 0, -np.cos(np.deg2rad(10))])
        for hz in (1, 2, 3):
            points = np.array([reference(t / hz, origin, hz, SEAM_WIDTH / 2, .012)[0]
                               for t in np.linspace(0, 1, 101)])
            np.testing.assert_allclose([points[:, 1].min(), points[:, 1].max()], [-.010, .010])
            np.testing.assert_allclose(points[:, 2], .002)
            np.testing.assert_allclose(points[-1] - points[0], [.012 / hz, 0, 0])
            point, velocity = reference(.17, origin, hz, 0, .012)
            np.testing.assert_allclose([point[1], velocity[1]], 0)

    def test_wire_is_fixed_in_the_exported_d6_frame(self):
        model, _ = floor_scene()
        wrist = model.body("l6_arm_linkage_d6").id
        self.assertEqual(model.site("wire_tip").bodyid[0], wrist)
        np.testing.assert_allclose(model.site("wire_tip").pos, [0, -.00054214, -.18834214])
        for n in (787, 789, 791, 793, 795):  # bracket, gun and wire
            self.assertEqual(model.geom(f"collision_{n}").bodyid[0], wrist)

    def test_driven_cart_and_wrist_across_seam(self):
        for hz in (1, 2, 3):
            with self.subTest(hz=hz):
                r = run(hz, .010, .012, 6)
                self.assertAlmostEqual(r[-1, 7] - r[0, 7], .012 * 6 / hz, delta=.003)
                np.testing.assert_array_equal(np.sign(r[:, 10:14].mean(axis=0)), [-1, -1, 1, 1])
                self.assertGreater(np.ptp(r[:, 19]), 20)  # actual DOF6 travel, degrees
                self.assertLess(abs(r[:, 18]).max(), .20)  # DOF1 holds its zero target
                self.assertLess(r[:, 14].max(), .16)
                np.testing.assert_allclose(r[:, 16], 80, atol=.06)
                self.assertTrue(np.all(r[:, 17] > 0))  # nozzle-to-wire direction points outward
                self.assertLess(r[:, 15].max(), .00006)
                self.assertLess(np.linalg.norm(r[:, 4:7] - r[:, 1:4], axis=1).max(), .00055)
                for cycle in range(6):
                    y = r[(r[:, 0] * hz >= cycle) & (r[:, 0] * hz < cycle + 1), 5]
                    np.testing.assert_allclose([y.min(), y.max()], [-.010, .010], atol=.00035)

    def test_unreachable_wire_angle_stops_wheel_feed(self):
        with self.assertRaisesRegex(RuntimeError, "Wheel feed stopped"):
            run(2, 0., .012, 1, target_wire=weave_xy_z.wire_direction(45))

    def test_loop_repeats_the_measured_pass_and_closes(self):
        starts = []

        def trace(scene, rows, hz, dt):
            if len(rows) == 1:
                starts.append(rows[0])

        def render(data, draw, labels, values):
            draw(None)
            return len(starts) < 3

        with patch.object(weave_xy_z, "weave_viewer", return_value=nullcontext(render)), \
             patch.object(weave_xy_z, "draw_traces", side_effect=trace), patch.object(weave_xy_z.time, "sleep"):
            looped = run(2, .010, .012, 1, loop=True)
        self.assertEqual(len(starts), 3)
        np.testing.assert_allclose(starts[1:], np.repeat([starts[0]], 2, axis=0), atol=1e-10, rtol=0)
        np.testing.assert_allclose(looped, run(2, .010, .012, 1), atol=1e-10, rtol=0)
