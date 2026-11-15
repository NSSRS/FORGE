"""Qt/worker tests with fake motors and controlled planning futures."""

import source_paths
from concurrent.futures import Future
import json
import os
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5 import QtWidgets, QtTest, QtCore
from controls.hardware import weld_app
from controls.hardware.weld_plan import Settings
from motors.arm_calibration import CalibrationMap, digest
from test_weld_hardware import simple_plan
from controls.hardware.weld_control import chassis_only_plan

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


class WeldPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.executor = Executor()
        self.patch = patch.object(weld_app, "ProcessPoolExecutor", return_value=self.executor)
        self.patch.start()
        args = SimpleNamespace(
            arm=True,
            chassis=True,
            execute=False,
            port="not-a-hardware-port",
            arm_profile=ROOT / "src/robot_description/config/arm_rig_a1_a6.json",
            chassis_profile=ROOT / "src/bringup/config/chassis.yaml",
            mapping_confirmed=False,
        )
        self.mapping = CalibrationMap(
            json.loads(
                (ROOT / "src/robot_description/config/arm_calibration_measured.json").read_text()
            )
        )
        initial = list(self.mapping.simulation_pose().values())
        self.window = weld_app.make_window(args, self.mapping, initial, Settings(cycles=1), ROOT)
        self.window.show()
        self.wait(lambda: self.window.worker.controller is not None)

    def tearDown(self):
        self.window.close()
        self.wait(lambda: not self.window.worker.isRunning())
        self.window.close()
        self.patch.stop()

    def wait(self, predicate, seconds=3):
        end = time.monotonic() + seconds
        while not predicate() and time.monotonic() < end:
            QtTest.QTest.qWait(10)
        self.assertTrue(predicate(), self.window.state_label.text())

    def test_switches_are_independent_and_start_off(self):
        c = self.window.worker.controller
        self.assertFalse(c.arm_on or c.chassis_on)
        self.window.arm.click()
        self.wait(lambda: c.arm_on)
        self.assertTrue(c.arm_holding)
        self.assertFalse(c.chassis_on)
        self.window.chassis.click()
        self.wait(lambda: c.chassis_on)
        self.window.arm.click()
        self.wait(lambda: not c.arm_on)
        self.assertTrue(c.arm_holding)
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

    def test_offline_window_render(self):
        out = ROOT / "build/weld/panel.png"
        out.parent.mkdir(parents=True, exist_ok=True)
        self.assertTrue(self.window.grab().save(str(out)))
        self.assertGreater(self.window.width(), 900)


if __name__ == "__main__":
    unittest.main()
