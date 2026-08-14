"""Regression tests for the confirmed flat-plate robot geometry."""

import unittest
from math import sqrt
from pathlib import Path

from main import PROJECT_DIR, build_robot, load_config
from contact import solve_flat_pose
from surface_model import FlatSurface
from simulation_view import SimulationWindow


class FlatPlateCaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(PROJECT_DIR / "config.yaml")
        cls.robot = build_robot(cls.config)
        cls.result = solve_flat_pose(
            cls.robot, FlatSurface(), cls.config["contact"]["tolerance"]
        )

    def test_all_six_wheels_contact(self):
        self.assertEqual(sum(self.result.valid_contacts.values()), 6)
        self.assertLessEqual(
            self.result.max_contact_residual, self.config["contact"]["tolerance"]
        )

    def test_pose_is_collision_free(self):
        self.assertEqual(self.result.collisions, [])
        self.assertTrue(self.result.feasible)

    def test_body_length_is_triangle_altitude(self):
        body = self.robot.body
        expected = sqrt(body.equal_side_length ** 2 - (body.rear_width / 2.0) ** 2)
        self.assertAlmostEqual(body.length, expected)

    def test_bump_recycles_only_after_leaving_view(self):
        window = SimulationWindow.__new__(SimulationWindow)
        window.terrain = "bumps"
        window.terrain_settings = {"bump_width": 0.300, "bump_respawn_margin": 0.020}
        window.bump_world_center = 0.0
        window._recycle_bump(travel=0.40, visible_left=-0.60, visible_right=1.00)
        self.assertEqual(window.bump_world_center, 0.0)
        window._recycle_bump(travel=0.80, visible_left=-0.60, visible_right=1.00)
        self.assertAlmostEqual(window.bump_world_center, 1.97)

    def test_duplicate_rear_modules_remain_explicit(self):
        self.assertEqual(
            self.result.wheel_centers["rear_left_rear"],
            self.result.wheel_centers["rear_right_rear"],
        )

    def test_front_suspension_is_transverse(self):
        self.assertEqual(self.robot.front_module.orientation, "transverse")
        self.assertTrue(self.robot.front_module.rotation_locked)
        self.assertEqual(self.result.pose.front_module_angle, 0.0)
        self.assertAlmostEqual(self.robot.front_module.caster_offset_x, -0.020)
        self.assertAlmostEqual(self.robot.front_module.caster_offset_z, -0.010)
        self.assertAlmostEqual(self.robot.wheel_width, 0.032)
        self.assertEqual(
            self.result.wheel_centers["front_rear"],
            self.result.wheel_centers["front_forward"],
        )
        left, right = self.robot.front_module.transverse_offsets
        self.assertGreater(left, 0.0)
        self.assertLess(right, 0.0)


if __name__ == "__main__":
    unittest.main()
