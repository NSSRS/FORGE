"""Full CAD/triangle-stop validation and fake-shaft replay; no hardware access."""

import source_paths
from dataclasses import asdict
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np

from motors.arm_calibration import CalibrationMap
from controls.hardware.weld_plan import build_plan, Settings, WeldGeometry
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from test_weld_hardware import Clock, FakeChassis

ROOT = Path(__file__).resolve().parents[1]


class FullModelWeldTests(unittest.TestCase):
    def test_kinematic_clearance_matches_full_forward_contacts(self):
        from controls.controller.weld_plan import JointPath
        fixture = json.loads((ROOT / "tests/fixtures/weld_tracking_20261117.json").read_text())
        geometry = WeldGeometry(CalibrationMap(fixture["mapping"]))
        for stage in ("home", "approach", "weld"):
            path = JointPath.from_document(fixture["plan"][stage])
            for t in np.linspace(0, path.duration, 25):
                geometry.check(path.sample(t)[0])
                positions = geometry.data.site_xpos.copy()
                contacts = [(tuple(c.geom), c.dist) for c in geometry.data.contact]
                geometry.mj.mj_forward(geometry.model, geometry.data)
                np.testing.assert_allclose(geometry.data.site_xpos, positions, atol=1e-12)
                self.assertEqual(len(contacts), geometry.data.ncon)
                for (geoms, distance), contact in zip(contacts, geometry.data.contact):
                    self.assertEqual(geoms, tuple(contact.geom))
                    self.assertAlmostEqual(distance, contact.dist, places=12)

    def test_nonzero_recovery_to_compact_standby_then_weld(self):
        mapping = CalibrationMap(json.loads(
            (ROOT / "src/robot_description/config/arm_calibration_measured.json").read_text()
        ))
        compact = json.loads(
            (ROOT / "src/robot_description/config/arm_standby_compact.json").read_text()
        )
        # The baseline is a model-valid example start, not the physical resting pose.
        start = list(mapping.simulation_pose().values())
        goal = compact["raw_output_shaft_deg"]
        self.assertFalse(compact["hardware_qualified"])
        plan = build_plan(dict(
            mapping=mapping.document, current_raw_deg=start, initial_raw_deg=goal,
            settings=asdict(Settings(hz=2.0, feed_m_s=0.012, cycles=1, limit_profile="weld")),
            arm_enabled=True, chassis_enabled=False,
        ))
        self.assertGreater(abs(goal[4] - start[4]), 70)
        np.testing.assert_allclose(np.asarray(plan["home"]["raw_deg"])[:, 1:3],
                                   np.tile(start[1:3], (len(plan["home"]["raw_deg"]), 1)))
        self.assertEqual(plan["recovery"]["clearance_mm"], 2.0)
        self.assertTrue(plan["speed_report"]["within_limits"])
        clock = Clock()
        arm = SimulatedArm(start, clock=clock)
        chassis = FakeChassis()
        geometry = WeldGeometry(mapping)
        control = WeldController(arm, chassis, clock=clock, pose_check=geometry.check)
        try:
            control.select(True, False)
            control.load_plan(plan)
            self.assertFalse(arm.enabled)
            self.assertEqual(arm.commands, [])
            control.go_initial()
            self.assertEqual(control.state, "verifying_hold")
            reached_standby = False
            explicitly_started_weld = False
            for _ in range(40000):
                previous = control.state
                clock.advance()
                control.tick()
                self.assertNotIn(control.state, ("fault", "paused"), control.message)
                if previous == "verifying_hold":
                    self.assertEqual(arm.commands, [])
                if previous == "initial" and control.state == "approach":
                    np.testing.assert_allclose(arm.read().raw_deg, goal, atol=control.arrival_deg)
                    reached_standby = True
                if control.state == "ready":
                    self.assertTrue(reached_standby)
                    control.start_weld()
                    explicitly_started_weld = True
                if control.state == "complete":
                    break
            self.assertTrue(explicitly_started_weld)
            self.assertEqual(control.state, "complete")
            self.assertTrue(arm.enabled)
            self.assertEqual(chassis.last_feed, 0)
            self.assertEqual(control.trace_dropped, 0)
            events = [event["event"] for event in control.events]
            self.assertLess(events.index("hold_verified"), events.index("recovery_waypoint_arrived"))
            self.assertEqual(events.count("recovery_waypoint_arrived"),
                             len(plan["recovery"]["waypoint_times_s"]))
        finally:
            control.close()

    def test_recorded_colliding_pose_rejected_before_weld_solving(self):
        mapping = CalibrationMap(json.loads(
            (ROOT / "src/robot_description/config/arm_calibration_measured.json").read_text()
        ))
        # Saved Nov 16 disabled feedback; this is an unresolved model/mount
        # alignment example, never an authorized hardware recovery start.
        raw = [-179.456390252915, -109.90763385942334, 136.35450881630624,
               160.94357231894566, -48.44593072625348, 145.40330735327922]
        with patch("controls.controller.weld_plan.solve_weave") as solve:
            with self.assertRaisesRegex(ValueError, "clearance"):
                build_plan(dict(mapping=mapping.document, current_raw_deg=raw,
                                initial_raw_deg=list(mapping.simulation_pose().values()),
                                arm_enabled=True, chassis_enabled=False))
            solve.assert_not_called()

    def test_checked_approach_continuous_weave_and_replay(self):
        mapping = CalibrationMap(
            json.loads(
                (ROOT / "src/robot_description/config/arm_calibration_measured.json").read_text()
            )
        )
        raw = list(mapping.simulation_pose().values())
        plan = build_plan(
            dict(
                mapping=mapping.document,
                current_raw_deg=raw,
                initial_raw_deg=raw,
                settings=asdict(Settings(hz=2.0, feed_m_s=0.012, cycles=1, limit_profile="weld")),
                arm_enabled=True,
                chassis_enabled=True,
            )
        )
        self.assertIn("raised-tool", plan["approach_route"])
        self.assertTrue(plan["speed_report"]["within_limits"])
        self.assertFalse(plan["hardware_qualified"])
        self.assertGreaterEqual(plan["optimization"]["total_time_scale"], 1)
        self.assertLessEqual(plan["effective_hz"], 2.0)
        self.assertAlmostEqual(plan["feed_m_s"] * 1000 / plan["effective_hz"], 6)
        self.assertEqual(plan["stage_limits"]["home"]["speed_deg_s"], [5] * 6)
        self.assertEqual(plan["stage_limits"]["approach"]["speed_deg_s"], [5] * 6)
        limits = [np.degrees(1.2)] * 6
        np.testing.assert_allclose(plan["stage_limits"]["weld"]["speed_deg_s"], limits)
        self.assertTrue(np.all(np.asarray(plan["speed_report"]["peak_raw_speed_deg_s"]) <= limits))
        self.assertEqual(plan["weld"]["interpolation"], "periodic_cubic")
        dynamics = plan["optimization"]["dynamics"]
        demand = np.asarray(dynamics["reduced_mass_absolute_envelope"]) @ np.radians(
            dynamics["raw_model_acceleration_deg_s2"]
        )
        reserve = np.asarray(dynamics["torque_budget_nm"]) - dynamics["max_static_torque_nm"]
        self.assertTrue(np.all(demand <= 0.5 * reserve + 1e-8))
        clock = Clock()
        arm = SimulatedArm(raw, clock=clock)
        chassis = FakeChassis()
        geometry = WeldGeometry(mapping)
        control = WeldController(
            arm, chassis, clock=clock, pose_check=geometry.check, continuous_weld=True
        )
        try:
            control.select(True, True)
            control.load_plan(plan)
            control.go_initial()
            for _ in range(40000):
                clock.advance()
                control.tick()
                if control.state == "ready":
                    control.start_weld()
                self.assertNotIn(control.state, ("fault", "paused"), control.message)
                if control.state == "welding" and clock() - control.started > 3.1 * control.paths["weld"].duration:
                    break
            self.assertEqual(control.state, "welding", control.message)
            self.assertGreaterEqual(control.trace[-1]["weld_repeat_index"], 3)
            self.assertEqual(chassis.last_feed, plan["feed_m_s"])
            control.pause()
            self.assertEqual(chassis.last_feed, 0)
            self.assertTrue(arm.enabled)
            target = arm.target.copy()
            for _ in range(50):
                clock.advance()
                control.tick()
            np.testing.assert_array_equal(arm.target, target)
            self.assertEqual(control.trace_dropped, 0)
        finally:
            control.close()


if __name__ == "__main__":
    unittest.main()
