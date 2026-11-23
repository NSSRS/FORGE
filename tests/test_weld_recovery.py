"""Startup interlocks and obstacle-routing regressions; no motor hardware."""

import source_paths
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from controls.controller.recovery import recovery_path
from controls.controller.weld_plan import Settings, WeldGeometry, move_path
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from test_weld_hardware import Clock, FakeChassis, simple_plan


class Obstacle:
    """A2 must clear the chassis before A1 crosses; reversed fixture swaps axes."""
    validate_path = WeldGeometry.validate_path

    def __init__(self, reverse=False):
        self.reverse = reverse
        self.mapping = SimpleNamespace(raw_limits=lambda joint: (-2., 2.))
        # The production validator restores model/closure scratch state between
        # its coarse rejection pass and the ordered path traversal.
        self.data = SimpleNamespace(qpos=np.zeros(1))
        self.closure = SimpleNamespace(last=np.zeros(1))

    def check(self, raw, **kwargs):
        a, b = raw[:2][::-1] if self.reverse else raw[:2]
        if 0.2 < a < 0.8 and b < 0.8:
            raise ValueError("arm/chassis obstacle")


class RecoveryPlannerTests(unittest.TestCase):
    def test_joint_order_search_follows_a_clear_route_before_expanding_alternatives(self):
        class OrderedOnly:
            def check(self, raw, **kwargs):
                pass

            def validate_path(self, path, **kwargs):
                # Reject simultaneous travel, but permit individually stopped
                # axes and the fully joined route. All geometry checks still run.
                moving = (abs(np.diff(path.positions, axis=0)) > 1e-9).sum(axis=1)
                if np.any(moving > 1):
                    raise ValueError('simultaneous movement blocked')

        path, result = recovery_path(OrderedOnly(), [0]*6, [1]*6, Settings())
        self.assertEqual(result['joint_order'], [1, 2, 3, 4, 5, 6])
        self.assertEqual(result['checked_edges'], 7)
        np.testing.assert_array_equal(path.positions[-1], [1]*6)

    def test_joint_order_search_backtracks_from_a_checked_dead_end(self):
        class DeadEnd(Obstacle):
            def check(self, raw, **kwargs):
                a, b, c = raw[:3]
                if (0.2 < b < 0.8 and c < 0.8) or (0.2 < c < 0.8 and a > 0.2):
                    raise ValueError('wrong order blocks remaining joints')

        path, result = recovery_path(DeadEnd(), [0]*6, [1, 1, 1, 0, 0, 0], Settings())
        self.assertEqual(result['joint_order'], [3, 1, 2])
        self.assertLessEqual(result['checked_edges'], 13)
        DeadEnd().validate_path(path)

    def test_route_order_depends_on_obstacle_and_joined_path_is_clear(self):
        start, goal = np.zeros(6), np.array([1, 1, 0, 0, 0, 0])
        for reverse in (False, True):
            geometry = Obstacle(reverse)
            with self.assertRaises(ValueError):
                geometry.validate_path(move_path(start, goal, Settings()))
            path, result = recovery_path(geometry, start, goal, Settings())
            self.assertEqual(result['route'], 'checked joint-order recovery')
            self.assertEqual(len(result['waypoint_times_s']), 2)
            self.assertEqual(result['joint_order'], [1, 2] if reverse else [2, 1])
            first, rate = path.sample(result['waypoint_times_s'][0])
            np.testing.assert_allclose(first[:2], [1, 0] if reverse else [0, 1])
            np.testing.assert_allclose(rate, 0, atol=1e-8)
            geometry.validate_path(path)

    def test_obstructed_start_cannot_be_escaped_automatically(self):
        with self.assertRaisesRegex(ValueError, 'obstacle'):
            recovery_path(Obstacle(), [.5, 0, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0], Settings())

    def test_exhausted_search_fails_closed(self):
        class Wall(Obstacle):
            def check(self, raw, **kwargs):
                if .2 < raw[0] < .8:
                    raise ValueError('wall')
        with self.assertRaisesRegex(ValueError, 'No checked recovery route'):
            recovery_path(Wall(), [0]*6, [1, 1, 0, 0, 0, 0], Settings())


class RecoveryControlTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.arm = SimulatedArm([0]*6, clock=self.clock)
        self.check = Mock()
        self.control = WeldController(self.arm, FakeChassis(), clock=self.clock, pose_check=self.check)
        self.control.select(True, True)
        self.control.load_plan(simple_plan())

    def tick(self, n=1):
        for _ in range(n):
            self.clock.advance()
            self.control.tick()

    def test_selection_and_plan_do_not_enable_then_hold_settles_before_motion(self):
        self.assertFalse(self.arm.enabled)
        self.control.go_initial()
        self.assertTrue(self.arm.enabled)
        self.assertEqual(self.control.state, 'verifying_hold')
        self.tick(5)
        self.assertEqual(self.arm.commands, [])
        self.tick(10)
        self.assertEqual(self.control.state, 'initial')
        self.assertTrue(self.arm.commands)

    def test_collision_or_missing_guard_blocks_before_enable(self):
        self.check.side_effect = ValueError('arm/chassis collision')
        with self.assertRaisesRegex(ValueError, 'collision'):
            self.control.go_initial()
        self.assertFalse(self.arm.enabled)
        self.control.pose_check = None
        with self.assertRaisesRegex(ValueError, 'guard'):
            self.control.go_initial()
        self.assertFalse(self.arm.enabled)

    def test_changed_pose_during_preload_never_enables(self):
        original = self.arm.hold
        def shifted(expected, check):
            self.arm.q += .4
            original(expected, check)
        self.arm.hold = shifted
        self.control.go_initial()
        self.assertEqual(self.control.state, 'fault')
        self.assertFalse(self.arm.enabled)
        self.assertEqual(self.arm.commands, [])

    def test_hold_drift_requires_replan_and_never_starts_recovery(self):
        self.control.go_initial()
        self.arm.q += .4
        self.tick()
        self.assertEqual(self.control.state, 'paused')
        self.assertIsNone(self.control.plan)
        self.assertEqual(self.arm.commands, [])
        self.assertTrue(self.arm.enabled)

    def test_hold_requires_continuous_stationary_feedback_and_times_out(self):
        self.control.go_initial()
        read = self.arm.read
        def moving():
            result = read()
            result.velocity_deg_s[0] = 2
            return result
        self.arm.read = moving
        self.tick(110)
        self.assertEqual(self.control.state, 'paused')
        self.assertEqual(self.arm.commands, [])

    def test_pause_during_hold_never_auto_starts(self):
        self.control.go_initial()
        self.control.pause()
        self.tick(30)
        self.assertEqual(self.control.state, 'paused')
        self.assertEqual(self.arm.commands, [])

    def test_old_or_malformed_recovery_plan_rejected_while_disabled(self):
        for mutate in (lambda p: p.pop('recovery'),
                       lambda p: p['recovery'].update(waypoint_times_s=[.123]),
                       lambda p: p.update(current_raw_deg=[1]*6)):
            plan = simple_plan()
            mutate(plan)
            with self.assertRaises(ValueError):
                self.control.load_plan(plan)
            self.assertFalse(self.arm.enabled)

    def test_loaded_plan_cannot_be_changed_by_caller(self):
        plan = simple_plan()
        self.control.load_plan(plan)
        plan['home']['raw_deg'][0][0] = 20
        plan['recovery']['waypoint_times_s'][0] = 0
        self.control.go_initial()
        self.assertEqual(self.control.state, 'verifying_hold')

    def test_measured_collision_stops_before_next_command(self):
        self.control.go_initial()
        self.tick(15)
        before = len(self.arm.commands)
        self.check.side_effect = ValueError('collision')
        self.tick()
        self.assertEqual(self.control.state, 'fault')
        self.assertEqual(len(self.arm.commands), before)
        self.assertFalse(self.arm.enabled)

    def test_waypoint_waits_for_measured_arrival_before_next_joint_moves(self):
        home, recovery = recovery_path(Obstacle(), [0]*6, [1, 1, 0, 0, 0, 0], Settings())
        plan = simple_plan()
        plan.update(home=home.document(), recovery=recovery, initial_raw_deg=home.positions[-1].tolist())
        plan['approach'] = move_path(home.positions[-1], [0.6]*6, Settings()).document()
        self.control.load_plan(plan)
        self.control.go_initial()
        self.tick(15)
        # Keep reported A2 just outside arrival tolerance without exceeding
        # tracking tolerance. The next edge would move A1 through the obstacle.
        read = self.arm.read
        def lagged():
            result = read()
            result.raw_deg[1] = min(result.raw_deg[1], .7)
            return result
        self.arm.read = lagged
        self.tick(100)
        self.assertEqual(self.control.recovery_index, 0)
        self.assertEqual(self.control.state, 'initial')
        self.assertTrue(all(abs(q[0]) < 1e-9 for q in self.arm.commands))
        self.arm.read = read
        self.tick(30)
        self.assertGreaterEqual(self.control.recovery_index, 1)


if __name__ == '__main__':
    unittest.main()
