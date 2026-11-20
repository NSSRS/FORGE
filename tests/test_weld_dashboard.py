"""Cached GUI telemetry must age, preserve unknowns and never command motors."""
import source_paths
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from PyQt5 import QtWidgets
from controls.hardware.weld_dashboard import controller_presentation, state_summary
from controls.hardware.weld_ports import ArmOwner
from motors.arm_position import JointPosition
from teleop.arm_telemetry_widgets import TelemetryHistory, TelemetryPanel


def sample(**updates):
    row = dict(joint=1, age_ms=2, enabled=True, torque_feedback_nm=-6,
               torque_trip_nm=10, rotor_c=46, mos_c=30)
    row.update(updates)
    return row


class DashboardTests(unittest.TestCase):
    def test_delayed_missing_nonfinite_and_stale_values_do_not_become_live_or_zero(self):
        history = TelemetryHistory()
        def update(rows, now=10, captured=10):
            return history.update(rows, now=now, captured_at=captured, stage="ready", run=1)
        rows = update([sample()])
        self.assertEqual(rows[0]["torque_feedback_nm"], -6)
        self.assertIsNone(rows[1]["torque_feedback_nm"])
        rows = update([sample(torque_feedback_nm=99)], now=10.1)
        self.assertFalse(rows[0]["fresh"])
        self.assertIsNone(rows[0]["rotor_c"])
        self.assertEqual(history.run_peaks[0]["torque_feedback_nm"], 6)
        self.assertEqual(len(history.traces[0]), 1)
        rows = update([sample(rotor_c=float("nan"), mos_c=None)], now=11, captured=11)
        self.assertIsNone(rows[0]["rotor_c"])
        self.assertIsNone(rows[0]["mos_c"])
        self.assertEqual(history.run_peaks[0]["rotor_c"], 46)

    def test_stage_and_run_peaks_are_separate_and_trace_is_bounded(self):
        history = TelemetryHistory()
        for i in range(500):
            history.update([sample(torque_feedback_nm=-8 if i < 250 else 2)],
                           now=i*.1, captured_at=i*.1, stage="prepare" if i < 250 else "ready", run=1)
        self.assertEqual(history.stage_peaks[0]["torque_feedback_nm"], 2)
        self.assertEqual(history.run_peaks[0]["torque_feedback_nm"], 8)
        self.assertLessEqual(len(history.traces[0]), 302)
        history.update([], now=50, captured_at=50, stage="prepare", run=2)
        self.assertEqual(history.run_peaks[0], {})

    def test_adapter_copies_temperatures_from_existing_cache_without_transport(self):
        owner = object.__new__(ArmOwner)
        rows = [JointPosition(i, 0, 0, -1, 1, 2, True, 0, -6, 10, 46, 30) for i in range(1, 7)]
        owner.port = Mock()
        owner.port.cached_positions.return_value = rows
        result = owner.display_feedback()
        self.assertEqual(result.telemetry[4]["rotor_c"], 46)
        self.assertEqual(result.telemetry[4]["mos_c"], 30)
        self.assertEqual(owner.port.mock_calls, [unittest.mock.call.cached_positions()])

    def test_cards_show_signed_torque_trip_and_stale_without_old_numeric_values(self):
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        panel = TelemetryPanel()
        with patch("teleop.arm_telemetry_widgets.time.monotonic", return_value=10):
            panel.update_samples([sample()], captured_at=10, stage="ready", run=1)
        card = panel.cards[0]
        self.assertEqual(card["torque"].text(), "-6.00 Nm")
        self.assertEqual(card["load"].value(), 60)
        self.assertIn("Motor 46 °C", card["temperatures"].text())
        with patch("teleop.arm_telemetry_widgets.time.monotonic", return_value=10.2):
            panel.update_samples([sample()], captured_at=10, stage="ready", run=1)
        self.assertEqual(card["torque"].text(), "— Nm")
        self.assertIn("STALE", card["title"].text())
        self.assertIn("Motor — °C", card["temperatures"].text())
        self.assertIn("6.00", card["peaks"].text())
        panel.close()

    def test_preparation_clock_freezes_at_first_pause_or_ready_and_new_run_resets(self):
        c = SimpleNamespace(events=[dict(event="recovery_requested", time_s=5, state="verifying_hold")],
                            state="approach", ACTIVE=("approach",), paths={}, trace=deque(),
                            started=5, pause_cause=None)
        self.assertEqual(controller_presentation(c, 10)["preparation_elapsed_s"], 5)
        c.events.extend([dict(event="pause", time_s=11, state="paused"), dict(event="pause", time_s=20, state="paused")])
        self.assertEqual(controller_presentation(c, 30)["preparation_elapsed_s"], 6)
        c.events.extend([dict(event="recovery_requested", time_s=31, state="verifying_hold"),
                         dict(event="stage_arrived", time_s=40, state="ready")])
        result = controller_presentation(c, 50)
        self.assertEqual(result["preparation_elapsed_s"], 9)
        self.assertEqual(result["run_id"], 31)

    def test_pause_warning_fault_and_planning_have_distinct_summaries(self):
        for state, cause, expected in (("ready", "focus_loss", "Window focus lost"),
                                       ("paused", "ui_heartbeat", "UI heartbeat lost"),
                                       ("paused", "operator_input", "Operator Pause / Hold")):
            self.assertIn(expected, state_summary(dict(state=state, pause_cause=cause))[0])
        self.assertEqual(state_summary(dict(state="paused", pending=True))[2], "planning")
        self.assertEqual(state_summary(dict(state="fault"))[2], "fault")
        self.assertEqual(state_summary(dict(state="paused", message="Blocked: path collision"))[2], "blocked")
        title, action, severity = state_summary(dict(state="ready", arm_on=True, feedback_age_ms=80))
        self.assertIn("Feedback stale", title)
        self.assertEqual(severity, "warning")


if __name__ == "__main__":
    unittest.main()
