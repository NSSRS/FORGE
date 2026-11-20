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
            args, self.mapping, initial, Settings(cycles=1, optimize_preparation=True), ROOT
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

    def test_pid_and_measured_motor_frequency_display(self):
        self.assertIsNotNone(self.window.worker.controller.tracker)
        self.assertIn("PID", self.window.controller_label.text())
        value = dict(state="welding", message="Continuous", pending=False,
                     arm_on=True, chassis_on=False, raw_deg=[0.2]*6,
                     timing=dict(effective_hz=1.73, feed_m_s=0.01),
                     planned_travel_deg=[2.26, 0.063, 0.132, 0.098, 0.072, 0.427],
                     measured_motion=[dict(hz=1.73, peak_to_peak_deg=1.6, status="measured")]
                     + [dict(hz=None, peak_to_peak_deg=0.02, status="insufficient motion")]*4
                     + [dict(hz=0.865, peak_to_peak_deg=0.45, status="measured")])
        self.window.show_status(value)
        self.assertEqual(self.window.table.item(0, 4).text(), "1.730")
        self.assertEqual(self.window.table.item(0, 5).text(), "1.600")
        self.assertEqual(self.window.table.item(1, 4).text(), "insufficient motion")
        self.assertEqual(self.window.frequency_values[0].text(), "1.730 Hz")
        self.assertEqual(self.window.frequency_values[5].text(), "0.865 Hz")
        self.assertIn("Two-cycle response", self.window.frequency_details[5].text())
        self.assertIn("Travel: 0.450° / 0.427° planned", self.window.frequency_details[5].text())
        self.assertIn("Too little motion", self.window.frequency_details[1].text())
        self.assertIn("Planned weave: 1.730 Hz", self.window.frequency_group.title())
        # A paused readout must not show old numeric measurements as live.
        value.update(state="paused")
        self.window.show_status(value)
        self.assertEqual(self.window.frequency_values[0].text(), "—")
        self.assertIn("Paused", self.window.frequency_status.text())
        value.update(measured_motion=None)
        self.window.show_status(value)
        self.assertEqual(self.window.table.item(0, 4).text(), "—")
        # A known planned rate must never fill a missing encoder measurement.
        value.update(state="welding")
        self.window.show_status(value)
        self.assertEqual(self.window.frequency_values[0].text(), "—")
        self.assertEqual(self.window.frequency_details[0].text(), "Collecting cycles…")

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

    def ready_arm(self):
        c = self.window.worker.controller
        self.set_foreground(True)
        self.window.arm.click()
        self.wait(lambda: c.arm_on)
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not None)
        self.finish_arm_plan()
        self.window.worker.send("initial")
        self.wait(lambda: c.state == "ready")
        return c

    def test_ready_survives_focus_pause_and_heartbeat_until_fresh_weld_click(self):
        c = self.ready_arm()
        plan, target = c.plan, c.arm.target.copy()
        future = self.executor.future
        sent = len(c.arm.commands)
        self.set_foreground(False)
        self.wait(lambda: "Window focus lost" in c.message)
        self.assertEqual(c.events[-1]["cause"], "focus_loss")
        QtTest.QTest.qWait(250)
        self.assertEqual(c.state, "ready")
        self.assertIs(c.plan, plan)
        self.assertTrue(c.arm.enabled)
        self.assertTrue(self.window.buttons["weld"].isEnabled())
        self.assertEqual(c.arm.target.tolist(), target.tolist())
        self.assertEqual(len(c.arm.commands), sent)
        self.window.timer.stop()
        self.wait(lambda: "UI heartbeat lost" in c.message)
        self.assertEqual(c.events[-1]["cause"], "ui_heartbeat")
        QtTest.QTest.qWait(200)
        self.assertEqual(c.state, "ready")
        self.assertIs(c.plan, plan)
        self.window.timer.start(30)
        self.set_foreground(True)
        QtTest.QTest.qWait(100)
        QtTest.QTest.keyClick(self.window, QtCore.Qt.Key_Space)
        QtTest.QTest.qWait(100)
        self.assertEqual(c.state, "ready")
        self.assertEqual(c.events[-1]["cause"], "operator_input")
        records = [json.loads(line) for line in self.journal_path.read_text().split('\n')[:-1]]
        causes = {event.get("cause") for row in records for event in row.get("events", []) if event["event"] == "pause"}
        self.assertTrue({"focus_loss", "ui_heartbeat", "operator_input"}.issubset(causes))
        self.assertIs(self.executor.future, future)
        self.window.buttons["weld"].click()
        self.wait(lambda: any(e["event"] == "weld_started" for e in c.events))
        self.assertEqual(c.state, "welding")

    def test_default_weld_keeps_running_until_space_then_holds(self):
        c = self.ready_arm()
        self.assertTrue(c.continuous_weld)
        self.assertIn("continuous until Pause", self.window.weld_run_label.text())
        self.wait(lambda: self.window.buttons["weld"].isEnabled())
        self.window.buttons["weld"].click()
        self.wait(lambda: c.state == "welding")
        self.wait(
            lambda: any(r.get("weld_repeat_index", 0) >= 2 for r in c.trace),
            seconds=4,
        )
        self.assertEqual(c.state, "welding", c.message)
        self.assertTrue(c.arm.enabled)
        self.window.table.setFocus()
        QtTest.QTest.keyClick(self.window.table, QtCore.Qt.Key_Space)
        self.wait(lambda: c.state == "paused")
        target = c.arm.target.copy()
        QtTest.QTest.qWait(150)
        self.assertTrue(c.arm.enabled)
        self.assertEqual(c.arm.target.tolist(), target.tolist())
        self.assertEqual(c.chassis.last_feed, 0)

    def test_ready_does_not_execute_a_start_queued_before_focus_changed(self):
        c = self.ready_arm()
        worker = self.window.worker
        entered, release = threading.Event(), threading.Event()
        tick = c.tick

        def blocked_tick():
            entered.set()
            release.wait(1)
            tick()

        with patch.object(c, "tick", side_effect=blocked_tick):
            try:
                self.wait(entered.is_set)
                worker.send("weld")
                self.set_foreground(False)
                self.set_foreground(True)
            finally:
                release.set()
            self.wait(lambda: "Window focus changed" in c.message)
        self.assertEqual(c.state, "ready")
        self.assertFalse(any(e["event"] == "weld_started" for e in c.events))
        self.assertTrue(c.arm.enabled)

    def test_focus_cancellation_inside_dispatch_keeps_precise_pause_cause(self):
        from controls.hardware.weld_control import TrajectoryPaused
        c = self.ready_arm()
        def interrupted(*args):
            self.window.worker.set_foreground(False)
            raise TrajectoryPaused("Intent cancelled inside dispatch")
        with patch.object(c.arm, "command", side_effect=interrupted):
            self.window.worker.send("weld")
            self.wait(lambda: c.state == "paused")
        self.assertEqual(c.pause_cause, "focus_loss")
        pauses = [event for event in c.events if event["event"] == "pause"]
        self.assertEqual(pauses[-1]["cause"], "focus_loss")
        self.assertIsNone(c.plan)
        self.assertTrue(c.arm.enabled)

    def test_reset_button_and_r_use_owner_and_drop_queued_motion(self):
        c = self.ready_arm()
        worker = self.window.worker
        with patch.object(c.arm, "recover", wraps=c.arm.recover) as recover:
            QtTest.QTest.keyClick(self.window, QtCore.Qt.Key_R)
            QtTest.QTest.qWait(50)
            recover.assert_not_called()
        c.fail(RuntimeError("A5 latched fault"))
        self.wait(lambda: self.window.buttons["reset"].isEnabled())
        entered, release = threading.Event(), threading.Event()
        original = c.arm.recover
        owner_thread = []

        def slow_reset(path):
            owner_thread.append(QtCore.QThread.currentThread())
            entered.set()
            release.wait(2)
            original(path)

        with patch.object(c.arm, "recover", side_effect=slow_reset) as recover:
            try:
                self.window.table.setFocus()
                QtTest.QTest.keyClick(self.window.table, QtCore.Qt.Key_R)
                self.wait(entered.is_set)
                self.wait(lambda: self.window.last_state == "resetting")
                self.assertFalse(self.window.buttons["reset"].isEnabled())
                with patch.object(c.arm, "poll", wraps=c.arm.poll) as poll:
                    worker.send("select", True, False)
                    worker.send("initial")
                    worker.send("weld")
                    worker.send("reset")
                    QtTest.QTest.qWait(100)
                    poll.assert_not_called()
                self.assertFalse(c.arm.enabled)
            finally:
                release.set()
            self.wait(lambda: c.state == "idle" and not worker.resetting.is_set())
            recover.assert_called_once()
        self.assertIs(owner_thread[0], worker)
        self.assertFalse(c.arm.enabled or c.arm_on or c.arm_holding)
        self.assertIsNone(c.plan)
        self.assertFalse(self.window.buttons["weld"].isEnabled())
        self.wait(lambda: not self.window.buttons["reset"].isEnabled())

    def test_reset_failure_remains_visible_and_can_be_retried(self):
        c = self.window.worker.controller
        c.fail(RuntimeError("Original A5 fault"))
        self.wait(lambda: self.window.buttons["reset"].isEnabled())
        with patch.object(c.arm, "recover", side_effect=ValueError("motor still moving")):
            self.window.buttons["reset"].click()
            self.wait(lambda: "motor reset failed" in self.window.state_label.text())
        self.assertEqual(c.fault, "Original A5 fault")
        self.assertTrue(self.window.buttons["reset"].isEnabled())
        self.window.buttons["reset"].click()
        self.wait(lambda: c.state == "idle")
        self.assertFalse(c.arm.enabled)

    def test_startup_fault_opens_reset_control_without_enabling(self):
        from controls.hardware import weld_ports

        self.window.close()
        self.wait(lambda: not self.window.worker.isRunning())
        self.window.close()
        initial = list(self.mapping.simulation_pose().values())
        arm = weld_ports.SimulatedArm(initial)
        arm.fault = "Retained controller fault"
        args = SimpleNamespace(
            arm=True, chassis=False, execute=False, mapping_confirmed=False,
            single_pass=True,
            journal_path=Path(self.evidence.name) / "startup.jsonl",
        )
        with patch.object(weld_ports, "SimulatedArm", return_value=arm):
            self.window = weld_app.make_window(
                args, self.mapping, initial, Settings(cycles=1), ROOT
            )
            self.window.show()
            self.wait(lambda: self.window.buttons["reset"].isEnabled())
        self.assertTrue(self.window.worker.isRunning())
        self.assertEqual(self.window.worker.controller.fault, arm.fault)
        self.assertFalse(self.window.worker.controller.continuous_weld)
        self.assertIn("one planned pass", self.window.weld_run_label.text())
        self.assertFalse(self.window.buttons["calculate"].isEnabled())
        self.assertFalse(arm.enabled or arm.commands)
        self.window.buttons["reset"].click()
        self.wait(lambda: self.window.last_state == "idle")
        self.assertFalse(arm.enabled or arm.commands)
        self.assertTrue(self.window.arm.isEnabled())

    def test_close_during_reset_waits_for_same_owner(self):
        c = self.window.worker.controller
        c.fail(RuntimeError("Fault before reset"))
        self.wait(lambda: self.window.buttons["reset"].isEnabled())
        entered, release = threading.Event(), threading.Event()
        original = c.arm.recover

        def slow_reset(path):
            entered.set()
            release.wait(2)
            original(path)

        with patch.object(c.arm, "recover", side_effect=slow_reset):
            try:
                self.window.buttons["reset"].click()
                self.wait(entered.is_set)
                self.window.close()
                self.assertTrue(self.window.worker.isRunning())
                self.assertFalse(c.arm.closed)
            finally:
                release.set()
            self.wait(lambda: not self.window.worker.isRunning())
        self.assertTrue(c.arm.closed)

    def test_switches_are_independent_and_start_off(self):
        c = self.window.worker.controller
        self.assertTrue(c.allow_optimized_preparation)
        self.assertIn("optimized staging", self.window.preparation_route_label.text())
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
        self.assertEqual(self.window.state_label.text(), "Checking path")
        self.assertIn("Calculation:", self.window.timing_label.text())
        self.assertFalse(self.window.buttons["initial"].isEnabled())
        self.assertFalse(c.arm.enabled)
        self.executor.future.set_exception(ValueError("No checked recovery route"))
        self.wait(lambda: "Blocked:" in self.window.state_label.text())
        self.assertFalse(self.window.planning_progress.isVisible())
        self.assertIn("No accepted path", self.window.timing_label.text())
        self.assertTrue(self.window.buttons["calculate"].isEnabled())
        def blocked_events():
            records = [json.loads(line) for line in self.journal_path.read_text().split('\n')[:-1]]
            return [event for row in records for event in row.get('events', [])
                    if event['event'] == 'action_blocked']
        self.wait(lambda: bool(blocked_events()))
        event = blocked_events()[-1]
        self.assertEqual(event['action'], 'planning_result')
        self.assertEqual(event['error'], 'No checked recovery route')
        self.assertEqual(event['planning_start_raw_deg'], self.executor.request['current_raw_deg'])
        self.assertFalse(c.arm.enabled)
        self.assertEqual(c.arm.commands, [])
        previous = self.executor.future
        self.window.buttons["calculate"].click()
        self.wait(lambda: self.executor.future is not previous)
        result = chassis_only_plan(Settings(cycles=1))
        result["request_fingerprint"] = digest(self.executor.request)
        self.executor.future.set_result(result)
        self.wait(lambda: self.window.buttons["initial"].isEnabled())
        self.assertEqual(c.state, "planned")
        self.assertFalse(self.window.planning_progress.isVisible())
        self.assertIn("Planned ", self.window.timing_label.text())

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
        self.assertIn("separate from motion time", self.window.timing_label.text())
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

    def test_dashboard_states_fit_small_screen_and_retain_first_fault(self):
        self.window.worker.status.disconnect(self.window.show_status)
        self.window.resize(1366, 768)
        value = dict(state="idle", message="Select outputs", pending=False,
                     arm_on=False, chassis_on=False, raw_deg=None)
        output = ROOT / "build/weld/gui-01"
        output.mkdir(parents=True, exist_ok=True)
        for state in ("idle", "planned", "initial", "ready", "welding", "paused", "fault", "resetting"):
            value.update(state=state, message="A5 speed trip" if state == "fault" else "State preview")
            if state == "fault":
                value.update(fault="Original A5 fault", fault_evidence=dict(fault_snapshot=dict(velocity_rad_s=1.51648)))
            self.window.show_status(value)
            self.app.processEvents()
            self.assertLessEqual(self.window.height(), 768)
            self.assertTrue(self.window.grab().save(str(output / f"layout-{state}.png")))
        self.window.details_toggle.setChecked(True)
        self.app.processEvents()
        self.assertIn("Original A5 fault", self.window.fault_detail.text())
        self.assertIn("1.51648", self.window.fault_detail.text())
        self.assertLessEqual(self.window.height(), 768)
        self.window.show_status(dict(value, state="idle", fault=None, fault_evidence=None))
        self.assertIn("Original A5 fault", self.window.fault_detail.text())

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

    def test_only_current_pose_requests_observation_rendering(self):
        preview = self.window.preview
        self.wait(lambda: bool(preview.backend.requests))
        self.assertTrue(preview.backend.requests[-1]['measured_pose'])
        preview.mode.setCurrentText('Standby target')
        self.wait(lambda: preview.backend.requests[-1]['measured_pose'] is False)
        c = self.window.worker.controller
        self.assertFalse(c.arm.enabled)
        self.assertEqual(c.arm.commands, [])

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
        self.assertEqual(self.window.table.columnCount(), 6)
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
