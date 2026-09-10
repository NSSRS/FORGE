"""Independent sanity and regression checks for the welding thermal model.

Run with: python test.py
"""

import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


def load_model_section():
    """Load the real numerical definitions without importing GUI dependencies."""
    source_path = Path(__file__).with_name("heat_visualizer.py")
    tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
    numerical_nodes = []
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            break
        if isinstance(node, (ast.Assign, ast.FunctionDef)):
            numerical_nodes.append(node)
    namespace = {"np": np}
    numerical_module = ast.Module(body=numerical_nodes, type_ignores=[])
    exec(compile(numerical_module, str(source_path), "exec"), namespace)
    return SimpleNamespace(**namespace)


model = load_model_section()


class UnitAndFormulaChecks(unittest.TestCase):
    def test_length_and_speed_conversions(self):
        self.assertAlmostEqual(model.mm_to_m(50.0), 0.050, places=12)
        self.assertAlmostEqual(model.mm_to_m(300.0), 0.300, places=12)
        self.assertAlmostEqual(model.mm_to_m(20.0), 0.020, places=12)
        self.assertAlmostEqual(model.kw_to_w(9.0), 9000.0, places=12)

    def test_default_positions_match_hand_calculation(self):
        # The wheel begins 0.30 m inside the plate. At 20 s and 50 mm/s it has
        # moved another 1 m; the torch remains 300 mm ahead.
        weld_y, wheel_y = model.robot_positions(20.0, 0.050, 300.0)
        expected_wheel = model.ROBOT_START_Y + 1.0
        self.assertAlmostEqual(wheel_y, expected_wheel, places=12)
        self.assertAlmostEqual(weld_y, expected_wheel + 0.3, places=12)

    def test_weld_position_temperature_sampling(self):
        surface = np.arange(model.NY * model.NX).reshape(model.NY, model.NX)
        sampled = model.temperature_at_position(
            surface, model.PLATE_WIDTH / 2.0, model.PLATE_HEIGHT / 2.0
        )
        expected = float(surface[round((model.NY - 1) / 2), round((model.NX - 1) / 2)])
        self.assertEqual(sampled, expected)
        self.assertIsNone(model.temperature_at_position(surface, 0.4, 4.1))

    def test_contact_area_matches_radius(self):
        radius_m = math.sqrt(model.MAGNET_AREA / math.pi)
        reconstructed_area = math.pi * radius_m**2
        self.assertAlmostEqual(radius_m * 1000.0, 56.4189583548, places=8)
        self.assertAlmostEqual(reconstructed_area, 0.01, places=12)

    def test_uniform_source_does_not_concentrate_at_boundary(self):
        x = np.linspace(0.0, model.PLATE_WIDTH, model.NX)
        y = np.linspace(0.0, model.PLATE_HEIGHT, model.NY)
        dx, dy = x[1] - x[0], y[1] - y[0]
        xx, yy = np.meshgrid(x, y)
        requested_power_w = 9000.0

        full_flux = model.uniform_source_flux(
            xx, yy, model.PLATE_WIDTH / 2.0, 0.6,
            dx, dy, requested_power_w
        )
        edge_flux = model.uniform_source_flux(
            xx, yy, model.PLATE_WIDTH / 2.0, 0.0,
            dx, dy, requested_power_w
        )
        full_power = float(np.sum(full_flux) * dx * dy)
        edge_power = float(np.sum(edge_flux) * dx * dy)
        self.assertAlmostEqual(full_power / requested_power_w, 1.0, delta=0.03)
        self.assertAlmostEqual(edge_power / full_power, 0.5, delta=0.08)
        self.assertAlmostEqual(float(edge_flux.max()), float(full_flux.max()))

    def test_retention_matches_hand_formula(self):
        # At ambient and 20 mm: 20/(20+5) = 0.8 exactly.
        self.assertAlmostEqual(model.retention_percent(25.0, 20.0), 80.0)

        # At 75 C: thermal factor = 1 - 0.0011*(75-25) = 0.945.
        expected = 100.0 * (20.0 / 25.0) * 0.945
        self.assertAlmostEqual(model.retention_percent(75.0, 20.0), expected)

    def test_contact_time_constant_matches_hand_formula(self):
        expected_seconds = 0.25 * 460.0 / (500.0 * 0.01)
        self.assertAlmostEqual(
            model.contact_time_constant(500.0, 0.01), expected_seconds
        )
        self.assertAlmostEqual(expected_seconds, 23.0)
        self.assertTrue(math.isinf(model.contact_time_constant(0.0, 0.01)))

    def test_progress_callback_reaches_100_percent(self):
        updates = []
        model.simulate(
            0.05, 0.0, 0.0, 20.0, 0.0, 0.0, 0.1,
            progress_callback=updates.append,
        )
        self.assertTrue(updates)
        self.assertEqual(updates[-1], 100)

    def test_contact_diagnostics_record_active_state(self):
        diagnostics = {}
        model.simulate(
            0.0, 0.0, 200.0, 20.0, 0.0, 500.0, 0.1,
            diagnostics=diagnostics,
        )
        self.assertEqual(diagnostics["contact_active"], [True, True])
        self.assertEqual(len(diagnostics["contact_steel_temperature"]), 2)
        self.assertTrue(
            np.isfinite(diagnostics["contact_steel_temperature"]).all()
        )

    def test_explicit_diffusion_step_is_stable(self):
        dx = model.PLATE_WIDTH / (model.NX - 1)
        dy = model.PLATE_HEIGHT / (model.NY - 1)
        thickness_m = model.mm_to_m(20.0)
        nz = max(2, math.ceil(thickness_m / model.TARGET_DZ))
        dz = thickness_m / nz
        courant_sum = model.ALPHA * model.DT * (
            1.0 / dx**2 + 1.0 / dy**2 + 1.0 / dz**2
        )
        self.assertLessEqual(courant_sum, 0.5)

    def test_through_thickness_diffusion_is_not_instantaneous(self):
        thickness_m = model.mm_to_m(20.0)
        diffusion_time_s = thickness_m**2 / model.ALPHA
        self.assertAlmostEqual(diffusion_time_s, 34.8888889, places=5)
        self.assertGreater(diffusion_time_s, 20.0)


class ModelCombinationChecks(unittest.TestCase):
    CASES = (
        # distance_mm, thickness_mm, expected peak C, expected final retention %
        (0.0, 20.0, 49.383, 77.854),
        (100.0, 20.0, 50.933, 77.718),
        (300.0, 20.0, 36.423, 78.995),
        (800.0, 20.0, 27.824, 79.751),
    )

    def test_default_distance_combinations(self):
        for distance, thickness, expected_peak, expected_final in self.CASES:
            with self.subTest(distance_mm=distance, thickness_mm=thickness):
                plate, magnet, retention = model.simulate(
                    0.050, 9.0, distance, thickness, 10.0, 500.0, 20.0
                )
                self.assertEqual(plate.shape, (model.NY, model.NX))
                self.assertEqual(len(magnet), 400)
                self.assertTrue(np.isfinite(plate).all())
                self.assertAlmostEqual(max(magnet), expected_peak, delta=0.002)
                self.assertAlmostEqual(retention[-1], expected_final, delta=0.002)

    def test_plate_thickness_combinations_at_ambient_start(self):
        expected = {10.0: 66.6666667, 20.0: 80.0, 40.0: 88.8888889}
        for thickness, expected_retention in expected.items():
            with self.subTest(thickness_mm=thickness):
                _, _, retention = model.simulate(
                    0.050, 9.0, 300.0, thickness, 10.0, 500.0, 1.0
                )
                self.assertAlmostEqual(retention[0], expected_retention, places=5)

    def test_slower_speed_is_hotter_at_equal_travel_distance(self):
        travel_distance_m = 0.35
        slow_speed = 0.005
        fast_speed = 0.050
        slow_time = travel_distance_m / slow_speed
        fast_time = travel_distance_m / fast_speed

        _, slow_magnet, _ = model.simulate(
            slow_speed, 9.0, 300.0, 20.0, 10.0, 500.0, slow_time
        )
        _, fast_magnet, _ = model.simulate(
            fast_speed, 9.0, 300.0, 20.0, 10.0, 500.0, fast_time
        )
        self.assertGreater(max(slow_magnet), max(fast_magnet))

    def test_stationary_coincident_wheel_heats_above_ambient(self):
        _, magnet, _ = model.simulate(
            0.0, 9.0, 0.0, 20.0, 10.0, 500.0, 5.0
        )
        self.assertGreater(max(magnet), model.AMBIENT_TEMPERATURE + 1.0)


def print_sanity_table():
    print("\nReference comparison (50 mm/s, 9 kW, 20 mm plate, 20 s)")
    print("distance_mm  peak_magnet_C  final_retention_pct")
    for distance, thickness, _, _ in ModelCombinationChecks.CASES:
        _, magnet, retention = model.simulate(
            0.050, 9.0, distance, thickness, 10.0, 500.0, 20.0
        )
        print(f"{distance:11.0f}  {max(magnet):13.3f}  {retention[-1]:19.3f}")


if __name__ == "__main__":
    result = unittest.main(exit=False, verbosity=2)
    if result.result.wasSuccessful():
        print_sanity_table()
