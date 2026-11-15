"""Full CAD/triangle-stop validation and fake-shaft replay; no hardware access."""

import source_paths
from dataclasses import asdict
import json
from pathlib import Path
import unittest
import numpy as np

from motors.arm_calibration import CalibrationMap
from controls.hardware.weld_plan import build_plan, Settings
from controls.hardware.weld_control import WeldController
from controls.hardware.weld_ports import SimulatedArm
from test_weld_hardware import Clock, FakeChassis

ROOT = Path(__file__).resolve().parents[1]


class FullModelWeldTests(unittest.TestCase):
    def test_checked_approach_weave_and_replay(self):
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
        control = WeldController(arm, chassis, clock=clock)
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
                if control.state == "complete":
                    break
            self.assertEqual(control.state, "complete")
            self.assertEqual(chassis.last_feed, 0)
            self.assertTrue(arm.enabled)
            self.assertEqual(control.trace_dropped, 0)
        finally:
            control.close()


if __name__ == "__main__":
    unittest.main()
