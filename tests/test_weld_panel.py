"""Qt/worker tests with fake motors and controlled planning futures."""

import source_paths
from concurrent.futures import Future
import json
import os
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtWidgets, QtTest, QtCore
from controls.hardware import weld_app
from controls.hardware.weld_plan import Settings
from motors.arm_calibration import CalibrationMap, digest
from test_weld_hardware import simple_plan
from controls.hardware.weld_control import chassis_only_plan
from simulation import weld_preview

ROOT = Path(__file__).resolve().parents[1]


class Executor:
    def __init__(self, *args, **kwargs):
        self.future = None
        self.request = None

    def submit(self, func, request):
        self.request = request
        self.future = Future()
        self.future.set_running_or_notify_cancel()
        return self.future

    def shutdown(self, **kwargs):
        pass


class FakePreview:
    def __init__(self, *args):
        self.requests = []
        self.closed = False

    def submit(self, request):
        self.requests.append(request)
        return True

    def poll(self):
        return None

    def close(self):
        self.closed = True


class WeldPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.evidence = tempfile.TemporaryDirectory()
        self.journal_path = Path(self.evidence.name) / "session.jsonl"
        self.executor = Executor()
        self.patch = patch.object(
            weld_app, "ProcessPoolExecutor", return_value=self.executor
        )
        self.patch.start()
        self.geometry_patch = patch.object(weld_app, "WeldGeometry")
        self.geometry_patch.start()
        self.preview_patch = patch.object(weld_preview, "PreviewProcess", FakePreview)
        self.preview_patch.start()
        args = SimpleNamespace(
            arm=True,
            chassis=True,
            execute=False,
            port="not-a-hardware-port",
            arm_profile=ROOT / "src/robot_description/config/arm_rig_a1_a6.json",
            chassis_profile=ROOT / "src/bringup/config/chassis.yaml",
            mapping_confirmed=False,
            journal_path=self.journal_path,
        )
        self.mapping = CalibrationMap(
            json.loads(
                (
                    ROOT / "src/robot_description/config/arm_calibration_measured.json"
                ).read_text()
            )
        )
        initial = list(self.mapping.simulation_pose().values())
        self.window = weld_app.make_window(
            args, self.mapping, initial, Settings(cycles=1), ROOT
        )
        self.window.show()
        self.wait(lambda: self.window.worker.controller is not None)

    def tearDown(self):
        self.window.close()
        self.wait(lambda: not self.window.worker.isRunning())
        self.window.close()
        self.patch.stop()
        self.geometry_patch.stop()
        self.preview_patch.stop()
        self.evidence.cleanup()

    def wait(self, predicate, seconds=3):
        end = time.monotonic() + seconds
        while not predicate() and time.monotonic() < end:
            QtTest.QTest.qWait(10)
        self.assertTrue(predicate(), self.window.state_label.text())

    def test_ready_display_uses_weld_goal_and_reports_hold_drift(self):
        value = dict(
            state="ready",
            message="Weld start reached",
            pending=False,
            arm_on=True,
            chassis_on=False,
            raw_deg=[0.2] * 6,
            joint_goal=dict(label="Weld start", raw_deg=[0.6] * 6),
            hold_observation=dict(
                tracking_error_deg=[0.4] * 6, arrival_tolerance_deg=0.25
            ),
        )
        self.window.show_status(value)
        self.assertIn("Weld start", self.window.table.horizontalHeaderItem(2).text())
        for axis in range(6):
            self.assertEqual(self.window.table.item(axis, 2).text(), "+0.6000")
            self.assertEqual(self.window.table.item(axis, 3).text(), "+0.4000")
        self.assertIn("HOLD DRIFT", self.window.tracking_label.text())
        value.update(state="paused", joint_goal=None, hold_observation=None)
        self.window.show_status(value)
        self.assertEqual(self.window.table.item(0, 3).text(), "—")

    def set_foreground(self, active):
        with patch.object(self.window, "isActiveWindow", return_value=active):
            self.app.sendEvent(
                self.window, QtCore.QEvent(QtCore.QEvent.ActivationChange)
            )

    def finish_arm_plan(self):
        c = self.window.worker.controller
        plan = simple_plan(chassis=c.chassis_on)
        offset = c.arm.q.tolist()
        for key in ("home", "approach", "weld"):
            plan[key]["raw_deg"] = [
                [q + offset[i] for i, q in enumerate(row)]
                for row in plan[key]["raw_deg"]
            ]
        plan["current_raw_deg"] = plan["home"]["raw_deg"][0]
        plan["initial_raw_deg"] = plan["home"]["raw_deg"][-1]
        plan["request_fingerprint"] = digest(self.executor.request)
        self.executor.future.set_result(plan)
        self.wait(lambda: self.window.preview.play.isEnabled())

    def test_switches_are_independent_and_start_off(self):
        c = self.window.worker.controller
        self.assertFalse(c.arm_on or c.chassis_on)
        self.window.arm.click()
        self.wait(lambda: c.arm_on)
        self.assertFalse(c.arm_holding)
        self.assertFalse(c.arm.enabled)
        self.assertFalse(c.chassis_on)
        self.window.chassis.click()
        self.wait(lambda: c.chassis_on)
        self.window.arm.click()
        self.wait(lambda: not c.arm_on)
        self.assertFalse(c.arm_holding)
        self.assertTrue(c.chassis_on)

    def test_cancel_discards_late_planning_result(self):
        c = self.window.worker.controller
        self.window.arm.click()
        self.wait(lambda: c.arm_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not None)
        future = self.executor.future
        request = self.executor.request
        QtTest.QTest.keyClick(self.window, QtCore.Qt.Key_Space)
        self.wait(lambda: self.window.worker.pending is None)
        result = simple_plan(chassis=False)
        result["request_fingerprint"] = digest(request)
        future.set_result(result)
        QtTest.QTest.qWait(100)
        self.assertIsNone(c.plan)
        self.assertEqual(c.state, "paused")
        self.assertFalse(self.window.buttons["weld"].isEnabled())

    def test_gui_heartbeat_loss_pauses_active_feed(self):
        c = self.window.worker.controller
        self.window.chassis.click()
        self.wait(lambda: c.chassis_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not None)
        result = chassis_only_plan(Settings(cycles=1))
        result["request_fingerprint"] = digest(self.executor.request)
        self.executor.future.set_result(result)
        self.wait(lambda: c.state == "planned")
        self.window.worker.send("initial")
        self.wait(lambda: c.state == "ready")
        self.window.worker.send("weld")
        self.wait(lambda: c.state == "welding")
        self.window.timer.stop()
        self.wait(lambda: c.state == "paused")
        self.assertEqual(c.chassis.last_feed, 0)
        self.window.timer.start(30)

    def test_planning_display_distinguishes_pending_blocked_and_accepted(self):
        c = self.window.worker.controller
        self.window.chassis.click()
        self.wait(lambda: c.chassis_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.window.planning_progress.isVisible())
        self.assertTrue(self.window.state_label.text().startswith("Planning ("))
        self.assertFalse(self.window.buttons["initial"].isEnabled())
        self.assertFalse(c.arm.enabled)
        self.executor.future.set_exception(ValueError("No checked recovery route"))
        self.wait(lambda: "Blocked:" in self.window.state_label.text())
        self.assertFalse(self.window.planning_progress.isVisible())
        self.assertIn("No accepted plan", self.window.timing_label.text())
        self.assertTrue(self.window.buttons["calculate"].isEnabled())
        previous = self.executor.future
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not previous)
        result = chassis_only_plan(Settings(cycles=1))
        result["request_fingerprint"] = digest(self.executor.request)
        self.executor.future.set_result(result)
        self.wait(lambda: self.window.buttons["initial"].isEnabled())
        self.assertEqual(c.state, "planned")
        self.assertFalse(self.window.planning_progress.isVisible())
        self.assertIn("Calculated:", self.window.timing_label.text())

    def test_focus_loss_preserves_pending_plan_and_preview_playback(self):
        c = self.window.worker.controller
        self.window.arm.click()
        self.wait(lambda: c.arm_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.window.planning_progress.isVisible())
        future = self.executor.future
        self.set_foreground(False)
        QtTest.QTest.qWait(100)
        self.assertIs(self.window.worker.pending, future)
        self.assertTrue(self.window.planning_progress.isVisible())
        self.assertIn("You may switch windows", self.window.timing_label.text())
        self.finish_arm_plan()
        self.set_foreground(True)
        self.window.preview.play.click()
        self.wait(lambda: self.window.preview.elapsed > 0.1)
        elapsed = self.window.preview.elapsed
        self.set_foreground(False)
        self.wait(lambda: self.window.preview.elapsed > elapsed + 0.1)
        self.assertEqual(c.state, "planned")
        self.assertFalse(c.arm.enabled)
        self.assertEqual(c.arm.commands, [])

    def test_focus_loss_pauses_recovery_and_retains_hold_without_auto_resume(self):
        c = self.window.worker.controller
        self.window.arm.click()
        self.wait(lambda: c.arm_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not None)
        self.finish_arm_plan()
        self.window.worker.send("initial")
        self.wait(lambda: c.state == "verifying_hold")
        self.set_foreground(False)
        self.wait(lambda: "Window focus lost" in self.window.state_label.text())
        self.assertEqual(c.state, "paused")
        self.assertTrue(c.arm_holding)
        self.assertTrue(c.arm.enabled)
        self.assertEqual(c.arm.target.tolist(), c.arm.q.tolist())
        self.assertIsNone(c.plan)
        self.set_foreground(True)
        QtTest.QTest.qWait(100)
        self.assertEqual(c.state, "paused")
        self.assertFalse(self.window.buttons["initial"].isEnabled())

    def test_focus_loss_pauses_active_chassis_feed(self):
        c = self.window.worker.controller
        self.window.chassis.click()
        self.wait(lambda: c.chassis_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not None)
        plan = chassis_only_plan(Settings(cycles=1))
        plan["request_fingerprint"] = digest(self.executor.request)
        self.executor.future.set_result(plan)
        self.wait(lambda: c.state == "planned")
        self.window.worker.send("initial")
        self.wait(lambda: c.state == "ready")
        self.window.worker.send("weld")
        self.wait(lambda: c.chassis.last_feed > 0)
        self.set_foreground(False)
        self.wait(lambda: c.state == "paused")
        self.assertEqual(c.chassis.last_feed, 0)

    def test_brief_focus_loss_rejects_previously_queued_recovery(self):
        worker = self.window.worker
        c = worker.controller
        self.window.arm.click()
        self.wait(lambda: c.arm_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not None)
        self.finish_arm_plan()
        entered, release = threading.Event(), threading.Event()
        original_tick = c.tick

        def wait_at_tick():
            entered.set()
            release.wait(1)
            original_tick()

        with patch.object(c, "tick", side_effect=wait_at_tick):
            try:
                self.wait(entered.is_set)
                worker.send("initial")
                self.set_foreground(False)
                self.set_foreground(True)
                self.assertFalse(worker.motion_allowed())
            finally:
                release.set()
            self.wait(lambda: "Window focus changed" in c.message)
        self.assertEqual(c.state, "planned")
        self.assertFalse(c.arm.enabled)
        self.assertEqual(c.arm.commands, [])

    def test_focus_loss_interrupts_in_progress_enable(self):
        worker = self.window.worker
        c = worker.controller
        self.window.arm.click()
        self.wait(lambda: c.arm_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not None)
        self.finish_arm_plan()
        entered, release = threading.Event(), threading.Event()

        def gated_hold(*args):
            entered.set()
            release.wait(1)
            # The hardware adapter checks this same callback during group enable.
            if not worker.motion_allowed():
                raise RuntimeError("Enable cancelled after focus loss")
            self.fail("Enable remained allowed after focus loss")

        with patch.object(c.arm, "hold", side_effect=gated_hold):
            try:
                worker.send("initial")
                self.wait(entered.is_set)
                self.set_foreground(False)
                self.set_foreground(True)
            finally:
                release.set()
            self.wait(lambda: c.fault is not None)
        self.assertIn("Enable cancelled", c.fault)
        self.assertFalse(c.arm.enabled)
        self.assertEqual(c.arm.commands, [])

    def test_gui_heartbeat_loss_clears_planning_message(self):
        c = self.window.worker.controller
        self.window.chassis.click()
        self.wait(lambda: c.chassis_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.window.planning_progress.isVisible())
        self.window.timer.stop()
        self.wait(lambda: "UI heartbeat lost" in self.window.state_label.text())
        self.assertIsNone(self.window.worker.pending)
        self.assertFalse(self.window.planning_progress.isVisible())
        self.assertNotIn("Checking recovery", c.message)
        self.window.timer.start(30)

    def test_offline_window_render(self):
        out = ROOT / "build/weld/panel.png"
        out.parent.mkdir(parents=True, exist_ok=True)
        self.assertTrue(self.window.grab().save(str(out)))
        self.assertGreater(self.window.width(), 900)

    def test_preview_playback_never_enables_or_commands_arm(self):
        c = self.window.worker.controller
        self.window.arm.click()
        self.wait(lambda: c.arm_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not None)
        plan = simple_plan(chassis=False)
        offset = c.arm.q.tolist()
        for key in ("home", "approach", "weld"):
            plan[key]["raw_deg"] = [
                [q + offset[i] for i, q in enumerate(row)]
                for row in plan[key]["raw_deg"]
            ]
        plan["current_raw_deg"] = plan["home"]["raw_deg"][0]
        plan["initial_raw_deg"] = plan["home"]["raw_deg"][-1]
        plan["request_fingerprint"] = digest(self.executor.request)
        self.executor.future.set_result(plan)
        self.wait(lambda: self.window.preview.play.isEnabled())
        preview = self.window.preview
        preview.play.click()
        self.wait(lambda: preview.elapsed > 0.1)
        preview.scrub.setValue(800)
        preview.view.setCurrentText("Wrist")
        self.wait(lambda: any(r["view"] == "Wrist" for r in preview.backend.requests))
        self.assertEqual(c.state, "planned")
        self.assertFalse(c.arm.enabled)
        self.assertEqual(c.arm.commands, [])
        self.window.buttons["pause"].click()
        self.wait(lambda: preview.plan is None)
        self.assertFalse(preview.play.isEnabled())
        self.assertFalse(preview.playing)
        self.assertIsNone(c.plan)

    def test_preview_failure_does_not_block_pause_or_heartbeat(self):
        with patch.object(
            self.window.preview.backend,
            "poll",
            side_effect=RuntimeError("Renderer lost"),
        ):
            self.wait(
                lambda: "Renderer lost" in self.window.preview.geometry_status.text()
            )
        self.window.chassis.click()
        self.wait(lambda: self.window.worker.controller.chassis_on)
        self.window.buttons["pause"].click()
        self.wait(lambda: self.window.worker.controller.state == "paused")
        self.assertTrue(self.window.timer.isActive())

    def test_space_pauses_with_preview_selector_focused(self):
        self.window.chassis.click()
        self.wait(lambda: self.window.worker.controller.chassis_on)
        self.window.preview.mode.setFocus()
        QtTest.QTest.keyClick(self.window.preview.mode, QtCore.Qt.Key_Space)
        self.wait(lambda: self.window.worker.controller.state == "paused")

    def test_active_preparation_displays_tracking_and_speed_allowance(self):
        c = self.window.worker.controller
        self.window.arm.click()
        self.wait(lambda: c.arm_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not None)
        self.finish_arm_plan()
        self.window.worker.send("initial")
        self.wait(lambda: "A1 error:" in self.window.tracking_label.text())
        self.assertIn("Path rate:", self.window.tracking_label.text())
        self.assertIn("speed allowance: 5.00°/s", self.window.tracking_label.text())
        self.assertEqual(self.window.table.columnCount(), 4)
        measured = float(self.window.table.item(0, 1).text())
        target = float(self.window.table.item(0, 2).text())
        remaining = float(self.window.table.item(0, 3).text())
        self.assertAlmostEqual(remaining, target - measured, delta=0.0002)
        self.window.buttons["pause"].click()
        self.wait(lambda: not self.window.tracking_label.text())

        def records():
            return [
                json.loads(line)
                for line in self.journal_path.read_text().split("\n")[:-1]
            ]

        self.wait(
            lambda: any(
                row.get("state") == "paused" and row.get("last_command")
                for row in records()
            )
        )
        self.assertTrue(any(row.get("kind") == "accepted_plan" for row in records()))
        self.assertTrue(
            self.window.worker.isRunning()
        )  # Evidence exists before shutdown.


if __name__ == "__main__":
    unittest.main()
