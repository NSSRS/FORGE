"""Offline preview, saved-start validation and display transport boundaries."""

import source_paths
from copy import deepcopy
import json
import os
import pickle
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
from controls.hardware.weld_app import main, read_start_report
from controls.hardware.weld_ports import ArmOwner
from motors.arm_calibration import CalibrationMap, RawAngleLimitError
from simulation.weld_preview import (
    PlanPlayback,
    PoseRenderer,
    project_point,
    annotated_frame,
)
from test_weld_hardware import simple_plan

ROOT = Path(__file__).resolve().parents[1]


class PreviewContracts(unittest.TestCase):
    def setUp(self):
        self.mapping = CalibrationMap(
            json.loads(
                (
                    ROOT / "src/robot_description/config/arm_calibration_measured.json"
                ).read_text()
            )
        )

    def test_timeline_retains_recovery_approach_and_weld_endpoints(self):
        plan = simple_plan()
        timeline = PlanPlayback(plan)
        home, approach = timeline.paths["home"], timeline.paths["approach"]
        self.assertAlmostEqual(
            timeline.duration,
            home.duration + approach.duration + timeline.paths["weld"].duration + 0.6,
        )
        self.assertEqual(timeline.sample(0.1), ("home", [0.0] * 6))
        self.assertEqual(timeline.sample(0), ("home", [0.0] * 6))
        self.assertEqual(timeline.sample(0.2 + home.duration), ("home", [0.3] * 6))
        name, raw = timeline.sample(0.6 + home.duration + approach.duration + 0.5)
        self.assertEqual(name, "weld")
        np.testing.assert_allclose(raw, [0.9] * 6)
        np.testing.assert_allclose(timeline.sample(1e6)[1], [0.6] * 6)
        weld_start = timeline.duration - timeline.paths["weld"].duration
        self.assertEqual(timeline.chassis_travel(weld_start), 0.0)
        self.assertAlmostEqual(timeline.chassis_travel(weld_start + 0.5), 0.0005)
        self.assertAlmostEqual(timeline.chassis_travel(1e6), 0.001)
        self.assertEqual(PlanPlayback(simple_plan(chassis=False)).chassis_travel(1e6), 0.0)

    def test_start_report_orders_axes_and_rejects_missing_enabled_or_nonfinite(self):
        raw = list(self.mapping.simulation_pose().values())
        report = dict(
            passed=True,
            feedback=[
                dict(joint=i, enabled=False, status=0, raw_position_deg=q)
                for i, q in reversed(list(enumerate(raw, 1)))
            ],
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "disabled.json"
            path.write_text(json.dumps(report))
            self.assertEqual(read_start_report(path, self.mapping), raw)
            for change in ("enabled", "duplicate", "nonfinite", "failed"):
                bad = deepcopy(report)
                if change == "enabled":
                    bad["feedback"][0]["enabled"] = True
                if change == "duplicate":
                    bad["feedback"][0]["joint"] = 1
                if change == "nonfinite":
                    bad["feedback"][0]["raw_position_deg"] = float("nan")
                if change == "failed":
                    bad["passed"] = False
                path.write_text(json.dumps(bad))
                with self.subTest(change=change), self.assertRaises(ValueError):
                    read_start_report(path, self.mapping)

    def test_hardware_rejects_saved_start_before_opening_owner(self):
        with patch("controls.hardware.weld_ports.ArmOwner") as owner:
            with self.assertRaises(SystemExit) as error:
                main(
                    [
                        "--execute",
                        "--arm",
                        "--supported",
                        "--start-report",
                        "unused.json",
                    ]
                )
            self.assertEqual(error.exception.code, 2)
            owner.assert_not_called()

    def test_display_uses_only_aged_cache_and_explicit_start_refreshes_disabled_pose(
        self,
    ):
        owner = ArmOwner.__new__(ArmOwner)
        rows = [
            SimpleNamespace(joint=i, position_rad=i / 100, velocity_rad_s=0, age_ms=450)
            for i in range(1, 7)
        ]
        owner.port = Mock()
        owner.port.holding = False
        owner.port.cached_positions.return_value = rows
        owner.port.positions.return_value = rows
        feedback = owner.display_feedback()
        np.testing.assert_array_equal(feedback.age_ms, [450] * 6)
        self.assertEqual(
            [call[0] for call in owner.port.method_calls], ["cached_positions"]
        )
        owner.read_stationary()
        owner.port.stationary_position.assert_called_once_with()
        owner.port.reset_mock()
        owner.port.holding = True
        owner.read_stationary()
        owner.port.stationary_position.assert_not_called()
        owner.port.positions.assert_not_called()
        owner.port.cached_positions.assert_called_once_with()

    def test_real_renderer_shows_blocked_measured_pose_and_valid_standby(self):
        renderer = PoseRenderer(self.mapping.document)
        try:
            raw = [
                -172.7681597346118,
                -117.99468711974517,
                127.93960647582381,
                165.24937821323505,
                42.01997947207877,
                148.37580568006877,
            ]
            blocked = renderer.frame(
                dict(raw=raw, clearance_mm=2, token=1, view="Wrist")
            )
            self.assertIn("Arm/environment clearance", blocked["issue"])
            self.assertTrue(any(c["blocked"] for c in blocked["contacts"]))
            self.assertEqual(len(blocked["pixels"]), 640 * 400 * 3)
            standby = json.loads(
                (
                    ROOT / "src/robot_description/config/arm_standby_compact.json"
                ).read_text()
            )
            clear = renderer.frame(
                dict(raw=standby["raw_output_shaft_deg"], clearance_mm=2, token=2)
            )
            self.assertIsNone(clear["issue"])
            self.assertNotEqual(blocked["pixels"], clear["pixels"])
            geometry = renderer.geometry
            closure_error = np.linalg.norm(
                geometry.data.site_xpos[geometry.closure.sites[0]]
                - geometry.data.site_xpos[geometry.closure.sites[1]]
            )
            self.assertLess(closure_error, 1e-8)
        finally:
            renderer.close()

    def test_top_view_advances_the_robot_and_wire_tip_along_the_floor(self):
        renderer = PoseRenderer(self.mapping.document)
        try:
            geometry = renderer.geometry
            standby = json.loads(
                (ROOT / "src/robot_description/config/arm_standby_compact.json").read_text()
            )
            request = dict(raw=standby["raw_output_shaft_deg"], clearance_mm=2, token=1, view="Top")
            before = renderer.frame(request)
            base = geometry.data.body("body").xpos.copy()
            floor = geometry.data.geom("preview_floor").xpos.copy()
            moved = renderer.frame(dict(request, chassis_travel_m=0.12))
            np.testing.assert_allclose(geometry.data.body("body").xpos, base + [0.12, 0, 0])
            np.testing.assert_allclose(geometry.data.geom("preview_floor").xpos, floor)
            self.assertEqual(before["issue"], moved["issue"])
            self.assertNotEqual(before["pixels"], moved["pixels"])
            self.assertIsNotNone(before["wire_tip_xy"])
            self.assertIsNotNone(moved["wire_tip_xy"])
            self.assertGreater(np.linalg.norm(np.subtract(before["wire_tip_xy"], moved["wire_tip_xy"])), 20)
            self.assertTrue(renderer.camera.orthographic)
            zoomed = renderer.frame(dict(request, camera=(20, 30, 0.75, 0.05, 0.0, 0.0)))
            self.assertEqual(renderer.camera.elevation, -90)
            self.assertAlmostEqual(renderer.camera.distance, 1.55)
            self.assertTrue(renderer.renderer.scene.camera[0].orthographic)
            self.assertLess(np.linalg.norm(zoomed["pan_axes"][0]), np.linalg.norm(before["pan_axes"][0]))
            self.assertNotEqual(zoomed["wire_tip_xy"], before["wire_tip_xy"])
            renderer.frame(dict(request, view="Overview", camera=(20, 30, 1, 0, 0, 0)))
            np.testing.assert_allclose(geometry.data.body("body").xpos, base)
            self.assertEqual(renderer.camera.azimuth, 128)
            self.assertEqual(renderer.camera.elevation, 5)
            self.assertFalse(renderer.camera.orthographic)
            renderer.frame(dict(request, view="Overview"))
            self.assertEqual(renderer.camera.azimuth, 108)
            self.assertEqual(renderer.camera.elevation, -25)
        finally:
            renderer.close()

    def test_renderer_uses_replanned_branch_without_wrapping_raw_pose(self):
        from controls.controller.calibration import select_a1_encoder_branch

        renderer = PoseRenderer(select_a1_encoder_branch(self.mapping, 1).document)
        try:
            standby = json.loads((ROOT / 'src/robot_description/config/arm_standby_compact.json').read_text())
            raw = standby['raw_output_shaft_deg']
            result = renderer.frame(dict(raw=raw, mapping=self.mapping.document, clearance_mm=2, token=1))
            self.assertIsNone(result['issue'])
            self.assertEqual(renderer.geometry.mapping.fingerprint, self.mapping.fingerprint)
            self.assertLess(raw[0], 0)
        finally:
            renderer.close()

    def test_outside_endpoint_pose_is_visible_only_as_an_observation(self):
        document = deepcopy(self.mapping.document)
        document['joints']['5'].update(
            model_intercept_deg=101.51405787753104,
            raw_branch_deg=[-96.60760497201417, 114.836287036511],
        )
        # Nov 19 screenshot: A3 is 0.1114 degrees past its saved CCW endpoint.
        raw = [-136.7479, -103.3943, 151.5014, 154.6925, -41.2113, -94.8044]
        renderer = PoseRenderer(document)
        try:
            original = deepcopy(renderer.geometry.mapping.document)
            request = dict(raw=raw, clearance_mm=.5, token=1, measured_pose=True)
            result = renderer.frame(request)
            self.assertIn('0.1114 deg above the saved upper limit', result['issue'])
            self.assertIn('inspection only', result['issue'])
            self.assertNotIn('Motion planning blocked', result['issue'])
            self.assertIn('planning status above', result['issue'])
            self.assertEqual(len(result['pixels']), 640*400*3)
            # All six measured coordinates are rendered exactly; no clamp/wrap.
            expected = [np.radians(document['joints'][str(i)]['model_per_raw'] * q
                                   + document['joints'][str(i)]['model_intercept_deg'])
                        for i, q in enumerate(raw, 1)]
            np.testing.assert_allclose(renderer.geometry.data.qpos[renderer.geometry.addresses], expected)
            self.assertEqual(renderer.geometry.mapping.document, original)
            hardware = renderer.frame(dict(request, measured_hardware_pose=True))
            self.assertTrue(hardware['hardware_observation'])
            self.assertNotIn('saved upper limit', hardware['issue'] or '')
            self.assertIn('mechanical stop', hardware['issue'])
            np.testing.assert_allclose(renderer.geometry.data.qpos[renderer.geometry.addresses], expected)
            with self.assertRaises(RawAngleLimitError):
                renderer.geometry.check(raw)
            for measured in (False, None):
                with self.assertRaises(RawAngleLimitError):
                    renderer.frame(dict(request, measured_pose=measured))
            for bad in (raw[:-1], [float('nan')]+raw[1:]):
                with self.assertRaises(ValueError):
                    renderer.frame(dict(request, raw=bad))
            with patch.object(renderer.geometry.closure, 'apply', return_value=False):
                with self.assertRaisesRegex(ValueError, 'image withheld'):
                    renderer.frame(request)
        finally:
            renderer.close()

    def test_limit_error_survives_planning_process_serialization(self):
        with self.assertRaises(RawAngleLimitError) as caught:
            self.mapping.model(3, 151.5014)
        returned = pickle.loads(pickle.dumps(caught.exception))
        self.assertEqual(str(returned), str(caught.exception))
        self.assertEqual(returned.joint, 3)
        self.assertEqual(returned.upper, 151.39)

    def test_contact_projection_tracks_image_axes_and_rejects_points_behind_camera(
        self,
    ):
        camera = SimpleNamespace(
            pos=np.zeros(3),
            forward=np.array([0, 0, -1]),
            up=np.array([0, 1, 0]),
            frustum_near=0.1,
            frustum_top=0.1,
            frustum_bottom=-0.1,
            frustum_center=0.0,
            orthographic=False,
        )
        np.testing.assert_allclose(
            project_point(camera, [0, 0, -1], 640, 400), [320, 200]
        )
        self.assertGreater(project_point(camera, [0.1, 0, -1], 640, 400)[0], 320)
        self.assertLess(project_point(camera, [0, 0.1, -1], 640, 400)[1], 200)
        self.assertIsNone(project_point(camera, [0, 0, 1], 640, 400))

    def test_contact_ring_remains_visible_over_an_opaque_image(self):
        from PyQt5 import QtWidgets

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        contact = dict(
            labels=["Arm mounting platform", "D2 upper arm"],
            blocked=True,
            distance_mm=-0.096,
            convex_distance_mm=-0.096,
            basis="convex hull",
            screen_xy=[320.0, 200.0],
        )
        frame = annotated_frame(
            dict(pixels=bytes(640 * 400 * 3), width=640, height=400, contacts=[contact])
        )
        color = frame.pixelColor(331, 200)
        self.assertGreater(color.red(), 220)
        self.assertLess(color.green(), 110)

    def test_recorded_concave_pair_gets_named_projected_mesh_gap(self):
        document = deepcopy(self.mapping.document)
        capture = json.loads(
            (ROOT / "docs/measurements/arm-a5-alignment-2026-11-16.json").read_text()
        )["recorded_candidate"]
        document["joints"]["5"].update(
            model_intercept_deg=capture["model_intercept_deg"],
            raw_branch_deg=capture["raw_command_bounds_deg"],
        )
        renderer = PoseRenderer(document)
        try:
            raw = [-190.8657, -131.9613, 89.9959, 175.0850, -1.2786, 65.8877]
            frame = renderer.frame(
                dict(raw=raw, clearance_mm=2.0, token=1, view="Contact")
            )
            self.assertIsNone(frame["issue"])
            (contact,) = frame["contacts"]
            self.assertEqual(
                contact["labels"], ["Arm mounting platform", "D2 upper arm"]
            )
            self.assertFalse(contact["blocked"])
            self.assertLess(contact["convex_distance_mm"], 0)
            self.assertAlmostEqual(contact["distance_mm"], 14.8833, places=3)
            np.testing.assert_allclose(contact["screen_xy"], [320, 200], atol=0.01)
            # Passed gap diagnostics must not clutter normal views or animation.
            for view in ("Overview", "Side", "Wrist"):
                normal = renderer.frame(
                    dict(raw=raw, clearance_mm=2.0, token=4, view=view)
                )
                self.assertIsNone(normal["issue"])
                self.assertEqual(normal["contacts"], [])
                self.assertTrue(renderer.geometry.contact_report)
            # Isolating is a display operation: show precisely the two parts,
            # restore their original rendering properties and keep the guard result.
            model = renderer.geometry.model
            original = [
                getattr(model, name).copy()
                for name in (
                    "geom_group",
                    "geom_rgba",
                    "geom_matid",
                    "geom_contype",
                    "geom_conaffinity",
                )
            ]
            isolated = renderer.frame(
                dict(raw=raw, clearance_mm=2.0, token=3, view="Contact parts")
            )
            self.assertTrue(isolated["isolated"])
            self.assertIsNone(isolated["issue"])
            self.assertAlmostEqual(
                isolated["contacts"][0]["distance_mm"], contact["distance_mm"]
            )
            visible = {
                model.geom(g.objid).name
                for g in renderer.renderer.scene.geoms[: renderer.renderer.scene.ngeom]
            }
            self.assertEqual(visible, set(contact["geom_names"]))
            for name, before in zip(
                (
                    "geom_group",
                    "geom_rgba",
                    "geom_matid",
                    "geom_contype",
                    "geom_conaffinity",
                ),
                original,
            ):
                np.testing.assert_array_equal(getattr(model, name), before)
            # Refinement preserves the same 2 mm minimum and other collision pairs.
            raw[1] = -141.0
            blocked = renderer.frame(
                dict(raw=raw, clearance_mm=2.0, token=2, view="Contact")
            )
            self.assertIsNotNone(blocked["issue"])
            refined = next(
                c for c in blocked["contacts"] if c["basis"] == "triangle mesh"
            )
            self.assertTrue(refined["blocked"])
            self.assertGreater(refined["distance_mm"], 0)
            self.assertLess(refined["distance_mm"], 2)
            self.assertTrue(
                any(
                    c["blocked"] and c["basis"] == "convex hull"
                    for c in blocked["contacts"]
                )
            )
        finally:
            renderer.close()


if __name__ == "__main__":
    unittest.main()
