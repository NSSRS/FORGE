"""Independent pose inspection: wire readback, view mapping, and no motion."""

import source_paths
from copy import deepcopy
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from motors import arm_observation as observation
from motors import arm_bench as bench
from motors.arm_calibration import CalibrationMap
from visualization.arm_pose_mirror import mapped_pose, report_sample, save_snapshot, make_window
import test_damiao_bench as routed

ROOT = Path(__file__).resolve().parents[1]
MAPPING = ROOT / "src/robot_description/config/arm_calibration_measured.json"
RAW = [-172.7681597346118, -117.99468711974517, 127.93960647582381,
       165.24937821323505, 42.01997947207877, 148.37580568006877]


def mapping():
    return CalibrationMap(json.loads(MAPPING.read_text()))


def report(raw=RAW):
    return dict(feedback=[dict(joint=i, raw_position_rad=math.radians(q),
        raw_position_deg=q, age_ms=7, enabled=False, status=0) for i, q in enumerate(raw, 1)])


class ObservationWireTests(unittest.TestCase):
    setUpClass = classmethod(routed.RoutedRoundTrip.setUpClass.__func__)
    tearDownClass = classmethod(routed.RoutedRoundTrip.tearDownClass.__func__)
    setUp = routed.RoutedRoundTrip.setUp
    tearDown = routed.RoutedRoundTrip.tearDown

    def observer(self):
        return observation.ArmObservation(routed.plan(), bus=self.bus,
                                          refresh_disabled=True, supported=True)

    def test_disabled_readback_preserves_raw_units_routes_and_tracks_hand_motion(self):
        with patch.object(observation, "time", bench.time), \
             patch.object(self.bus, "_request", wraps=self.bus._request) as requests:
            source = self.observer()
            for i, value in enumerate(RAW):
                self.lib.fake_position(i, math.radians(value))
            first = source.sample()
            for i, row in enumerate(first["feedback"]):
                self.assertEqual(row["joint"], i+1)
                self.assertEqual((row["bus"], row["can_id"], row["feedback_id"]),
                                 (1 if i == 0 else 2, i+1, i+17))
                self.assertAlmostEqual(row["raw_position_rad"], math.radians(RAW[i]), delta=.0004)
                self.assertEqual(row["raw_position_deg"], math.degrees(row["raw_position_rad"]))
                self.assertLess(row["age_ms"], 80)
                self.assertFalse(row["enabled"])
            initialization_types = {call.args[0] for call in requests.call_args_list}
            self.assertTrue(initialization_types.isdisjoint({4,5,8,11,15,17,18,20,25}))
            requests.reset_mock()
            self.lib.fake_position(4, math.radians(RAW[4]+8))
            self.lib.fake_speed(4, .1)  # Manual motion must not be treated as a target.
            second = source.sample()
            self.assertAlmostEqual(second["feedback"][4]["raw_position_deg"]-first["feedback"][4]["raw_position_deg"], 8, delta=.03)
            self.assertEqual({call.args[0] for call in requests.call_args_list}, {6,22})
            self.assertTrue(source.evidence["identity_verified"])
            source.close()
        self.assertEqual(self.lib.fake_parameter_writes(), 0)
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)], [0]*6)
        self.assertEqual([self.lib.fake_status(i) for i in range(6)], [0]*6)
        self.assertEqual([self.lib.fake_mode(i) for i in range(6)], [1]*6)
        self.assertEqual(self.lib.fake_invalid(), 0)

    def test_passive_cache_never_refreshes_or_claims_verified_ids(self):
        with patch.object(observation, "time", bench.time):
            self.observer()
            source = observation.ArmObservation(routed.plan(), bus=self.bus)
            self.lib.fake_advance(500)
            before = [self.bus.bus_health(i)["tx_frames"] for i in (1,2)]
            with patch.object(self.bus, "_request", wraps=self.bus._request) as requests:
                packet = source.sample()
            after = [self.bus.bus_health(i)["tx_frames"] for i in (1,2)]
            self.assertEqual(before, after)
            self.assertEqual({call.args[0] for call in requests.call_args_list}, {22})
            self.assertTrue(all(row["age_ms"] >= 500 for row in packet["feedback"]))
            self.assertTrue(all(row["can_id"] is None for row in packet["feedback"]))
            self.assertFalse(source.evidence["identity_verified"])

    def test_enabled_or_faulted_controller_refused_before_disable_or_reset(self):
        for changed in (dict(enabled=1), dict(fault=1)):
            with self.subTest(changed=changed):
                if self.bus.closed:
                    self.bus = routed.ArmBus(transport=routed.NativeTransport(self.lib))
                state = replace(self.bus.state(0), **changed)
                with patch.object(self.bus, "state", return_value=state), \
                     patch.object(self.bus, "_request", wraps=self.bus._request) as requests:
                    with self.assertRaisesRegex(RuntimeError, "active or faulted"):
                        self.observer()
                self.assertTrue({call.args[0] for call in requests.call_args_list}.isdisjoint({6,8,5,4}))

    def test_missing_axis_fails_instead_of_showing_partial_pose(self):
        self.lib.fake_missing(3)
        with self.assertRaises((RuntimeError, TimeoutError)):
            self.observer()
        self.assertEqual([self.lib.fake_commands(i) for i in range(6)], [0]*6)

    def test_refresh_requires_support_before_opening_transport(self):
        with patch.object(observation, "ArmBus") as factory:
            with self.assertRaisesRegex(ValueError, "supported"):
                observation.ArmObservation(routed.plan(), refresh_disabled=True)
            p = routed.plan();p["active_joints"] = [2,3,4,5,6]
            with self.assertRaisesRegex(ValueError, "all six"):
                observation.ArmObservation(p)
            factory.assert_not_called()


class MirrorDataTests(unittest.TestCase):
    def test_direct_affine_mapping_never_wraps_or_applies_gear_ratio(self):
        m = mapping()
        raw = list(RAW);raw[0] += 720
        model, warnings = mapped_pose(raw, m.document)
        for i, q in enumerate(raw, 1):
            self.assertEqual(model[i-1], -q+m.joints[str(i)]["model_intercept_deg"])
        self.assertTrue(any("A1" in warning for warning in warnings))
        with self.assertRaises(ValueError):mapped_pose([float("nan")]*6, m.document)

    def test_report_orders_axes_checks_units_and_remains_historical(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"report.json"
            document = report();document["feedback"].reverse()
            path.write_text(json.dumps(document))
            sample = report_sample(path)
            self.assertEqual([row["joint"] for row in sample["feedback"]], list(range(1,7)))
            self.assertTrue(sample["historical"])
            self.assertIsNone(sample["host_received_monotonic_s"])
            document["feedback"][0]["raw_position_deg"] += 1
            path.write_text(json.dumps(document))
            with self.assertRaisesRegex(ValueError, "disagree"):report_sample(path)
            document["feedback"].pop();path.write_text(json.dumps(document))
            with self.assertRaisesRegex(ValueError, "exactly one"):report_sample(path)

    def test_snapshot_is_separate_evidence_not_a_new_anchor(self):
        m = mapping();original = deepcopy(m.document)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"report.json";path.write_text(json.dumps(report()))
            view = deepcopy(m.document);view["joints"]["5"]["model_per_raw"] = 1
            packet = dict(sample=report_sample(path), view_mapping=view, warning=None)
            result = json.loads(save_snapshot(directory, packet, m, RAW, "A5 upward").read_text())
            self.assertTrue(result["view_mapping_changed"])
            self.assertFalse(result["calibration_anchors_changed"])
            self.assertFalse(result["production_qualified"])
            self.assertFalse(result["motor_targets_sent"])
            self.assertEqual(m.document, original)
            self.assertAlmostEqual(result["delta_raw_deg"][4], 0)


class MirrorSceneTests(unittest.TestCase):
    def test_rendered_drives_match_numbers_even_when_pose_has_contacts(self):
        import numpy as np
        from simulation.arm_mirror_scene import ArmMirrorScene
        scene = ArmMirrorScene()
        try:
            model, _ = mapped_pose(RAW, mapping().document)
            with patch.object(scene.mj, "mj_step", side_effect=AssertionError("No physics")):
                pixels, _ = scene.render(model)
                self.assertEqual(len(pixels), 800*500*3)
                np.testing.assert_allclose(np.degrees(scene.data.qpos[scene.addresses]), model, atol=1e-9)
                self.assertGreater(scene.data.ncon, 0)
                self.assertLess(np.linalg.norm(scene.data.site_xpos[scene.closure.sites[0]]-
                                              scene.data.site_xpos[scene.closure.sites[1]]), 1e-6)
                model[3] += 15
                scene.render(model, show_chassis=False)
                np.testing.assert_allclose(np.degrees(scene.data.qpos[scene.addresses]), model, atol=1e-9)
                self.assertEqual(scene.data.time, 0)
                previous = scene.data.qpos.copy()
                reference = scene.a5_straight_reference()
                self.assertAlmostEqual(reference["model_deg"], 93.17253, delta=.0001)
                self.assertEqual(reference["kind"], "a5_straight_axes")
                self.assertLess(reference["axis_error"], 1e-10)
                self.assertFalse(reference["physical_alignment_verified"])
                np.testing.assert_array_equal(scene.data.qpos, previous)
            with patch.object(scene.closure, "apply", return_value=False):
                with self.assertRaisesRegex(ValueError, "withheld"):scene.render(model)
        finally:
            scene.close()


class MirrorWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        self.report = self.directory/"report.json";self.report.write_text(json.dumps(report()))
        self.window = None

    def tearDown(self):
        if self.window:
            self.window.close()
            self.wait_for(lambda: not self.window.reader.isRunning())
            self.window.close()
        self.temp.cleanup()

    def wait_for(self, predicate):
        end = time.monotonic()+5
        while not predicate() and time.monotonic()<end:
            self.app.processEvents();time.sleep(.005)
        self.app.processEvents()
        self.assertTrue(predicate(), self.window.reader.failure if self.window else "timeout")

    def open_window(self, hardware=False, **kwargs):
        class Renderer:
            def render(self, model, **camera):return bytes(800*500*3), None
            def a5_straight_reference(self):
                return dict(kind="a5_straight_axes", model_deg=93.17253,
                            physical_alignment_verified=False)
            def close(self):pass
        args = SimpleNamespace(port=None, report=self.report, disabled_refresh=False,
            supported=False, output_dir=self.directory, arm_profile=ROOT/"src/robot_description/config/arm_rig_a1_a6.json")
        if hardware:
            args.port="fake";args.report=None;args.disabled_refresh=True;args.supported=True
        self.window = make_window(args, mapping(), ROOT, renderer_factory=Renderer, **kwargs)
        self.window.show()
        self.wait_for(lambda: self.window.last_packet is not None)
        return self.window

    def test_buttons_capture_both_cable_ends_then_independent_straight_pose(self):
        class Source:
            angle = -95
            def __init__(self, *args, **kwargs):
                self.evidence = dict(identity_verified=True, synthetic_test=True)
            def sample(self):
                sample = report()
                sample.update(source="disabled-refresh", fault=False,
                              host_received_monotonic_s=time.monotonic())
                for row in sample["feedback"]:row.update(feedback_valid=True, velocity_rad_s=0)
                sample["feedback"][4].update(raw_position_deg=self.angle, raw_position_rad=math.radians(self.angle))
                return sample
            def close(self):pass
        w = self.open_window(hardware=True, source_factory=Source)
        for button, angle, kind in ((w.cw_capture,-95,"CW"),(w.ccw_capture,70,"CCW"),(w.straight_capture,-3,"straight")):
            Source.angle = angle
            self.wait_for(button.isEnabled)
            button.click()
            self.assertFalse(any(b.isEnabled() for b in w.endpoint_buttons))
            self.wait_for(lambda: not w.capture_busy)
            self.assertIn(f"A5 {kind} saved",w.cw_status.text())
            if kind != "straight":self.assertEqual(w.view_mapping["joints"],mapping().joints)
        self.wait_for(lambda: w.last_packet["revision"] == w.revision)
        self.assertAlmostEqual(w.last_packet["model_deg"][4],93.17253)
        self.assertEqual(w.view_mapping["joints"]["5"]["raw_branch_deg"],[-93,68])
        for key in "12346":self.assertEqual(w.view_mapping["joints"][key],mapping().joints[key])
        w.cw_capture.click()
        self.wait_for(lambda: not w.capture_busy)
        self.assertEqual(w.view_mapping["joints"],mapping().joints)
        self.assertIn("CCW",w.cw_status.text())

    def test_full_precision_mapping_and_delta_baseline_do_not_rezero(self):
        w = self.open_window()
        self.assertEqual(w.last_packet["view_mapping"]["joints"], mapping().document["joints"])
        original = deepcopy(w.last_packet["sample"])
        w.capture_baseline()
        self.assertEqual(w.last_packet["sample"], original)
        self.assertEqual(w.table.item(4,3).text(), "+0.00000")
        w.chassis.setChecked(False)
        self.wait_for(lambda: w.last_packet["revision"] == w.revision)
        self.assertEqual(w.last_packet["view_mapping"]["joints"], mapping().document["joints"])
        self.assertIn("not live", w.status.text())
        self.assertFalse(w.cw_capture.isEnabled())
        w.save_current()
        saved = json.loads(next(self.directory.glob("pose-*.json")).read_text())
        self.assertFalse(saved["view_mapping_changed"])

    def test_view_edits_change_only_mapping_and_invalid_edit_can_recover(self):
        w = self.open_window()
        raw = deepcopy(w.last_packet["sample"])
        w.editors[4][0].setValue(0)
        self.wait_for(lambda: w.last_packet["revision"] == w.revision)
        self.assertIsNone(w.last_packet["model_deg"])
        self.assertFalse(w.save.isEnabled())
        self.assertIsNone(w.reader.failure)
        w.editors[4][0].setValue(1)
        self.wait_for(lambda: w.last_packet["revision"] == w.revision)
        self.assertEqual(w.last_packet["sample"], raw)
        self.assertAlmostEqual(w.last_packet["model_deg"][4], RAW[4]+mapping().joints["5"]["model_intercept_deg"])
        w.reset_mapping()
        self.wait_for(lambda: w.last_packet["revision"] == w.revision)
        self.assertEqual(w.last_packet["view_mapping"]["joints"], mapping().document["joints"])


if __name__ == "__main__":
    unittest.main()
