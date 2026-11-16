"""Supported weld-motion panel. Default is simulation; hardware is explicit."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from datetime import datetime, timezone
import json
import multiprocessing
from pathlib import Path
import queue
import sys
import threading
import time

from motors.arm_calibration import CalibrationMap, digest
from controls.controller.weld_plan import Settings, WeldGeometry, initial_from_model, build_plan, audit
from .weld_control import WeldController
from .weld_recording import SessionJournal, cached_arm_evidence


def read_start_report(path, mapping):
    """Load historical disabled feedback for offline reproduction, never hardware."""
    from controls.controller.weld_plan import finite_vector

    report = json.loads(Path(path).read_text())
    rows = report.get("feedback", [])
    if report.get("passed") is not True or len(rows) != 6:
        raise ValueError("Start report must be a passed six-axis disabled diagnostic")
    if sorted(row.get("joint", 0) for row in rows) != list(range(1, 7)):
        raise ValueError("Start report must contain each joint A1-A6 exactly once")
    if any(row.get("enabled") is not False or row.get("status") != 0 for row in rows):
        raise ValueError("Start report contains enabled or faulted motors")
    raw = finite_vector([row["raw_position_deg"] for row in sorted(rows, key=lambda row: row["joint"])],
                        "recorded start").tolist()
    mapping.pose({str(i): q for i, q in enumerate(raw, 1)})
    return raw


def make_window(args, mapping, initial, settings, root):
    from PyQt5 import QtCore, QtGui, QtWidgets
    from .weld_ports import ArmOwner, SimulatedArm, ChassisOwner

    class OwnerThread(QtCore.QThread):
        status = QtCore.pyqtSignal(object)
        preview_plan = QtCore.pyqtSignal(object)

        def __init__(self):
            super().__init__()
            self.commands = queue.Queue(maxsize=16)
            self.cancel = threading.Event()
            self.finish = threading.Event()
            self.foreground = threading.Event()
            self.focus_stop = threading.Event()
            self.focus_generation = 0
            self.heartbeat = time.monotonic()
            self.controller = None
            self.pending = None
            self.planning_started = None
            self.request = None
            self.report = None
            self.failed = False
            self.worker_error = None
            self.journal = None
            self.recorded_events = 0
            self.recorded_plan = None
            self.executor = ProcessPoolExecutor(
                max_workers=1, mp_context=multiprocessing.get_context("spawn")
            )

        def send(self, action, *values):
            if action in ("pause", "disable", "select"):
                self.cancel.set()
                while True:
                    try:
                        self.commands.get_nowait()
                    except queue.Empty:
                        break
            try:
                self.commands.put_nowait((action, values, self.focus_generation))
            except queue.Full:
                self.cancel.set()

        def set_foreground(self, active):
            if active:
                self.foreground.set()
            else:
                self.foreground.clear()
                self.focus_generation += 1
                # Latch even a brief loss until the owner handles it. This also
                # interrupts an enable already in progress, before another tick.
                self.focus_stop.set()

        def motion_allowed(self):
            return (self.foreground.is_set() and not self.focus_stop.is_set()
                    and not self.cancel.is_set() and not self.finish.is_set()
                    and time.monotonic() - self.heartbeat < 0.15)

        def pause_for_focus_loss(self):
            if self.foreground.is_set() and not self.focus_stop.is_set():
                return
            self.focus_stop.clear()
            c = self.controller
            if c.state in c.ACTIVE or c.state == "ready":
                c.pause()
                c.message = "Window focus lost; motion paused. " + c.message

        def clear_planning(self):
            if self.pending is not None:
                self.pending.cancel()
            self.pending = None
            self.planning_started = None
            self.request = None

        def record_evidence(self, kind="sample"):
            c = self.controller
            if self.journal is None or c is None:
                return
            if c.last_accepted_plan is not self.recorded_plan:
                self.recorded_plan = c.last_accepted_plan
                self.journal.record(dict(kind="accepted_plan", plan=self.recorded_plan))
            events = c.events[self.recorded_events:]
            self.recorded_events = len(c.events)
            if kind == "sample" and not events and not c.arm_holding and c.state not in c.ACTIVE:
                return
            self.journal.record(dict(
                kind=kind, time_s=time.monotonic(), state=c.state, message=c.message,
                events=events, fault=c.fault,
                observation=c.last_observation,
                last_command=None if not c.trace else c.trace[-1],
                arm=cached_arm_evidence(c.arm),
                dropped_records=self.journal.dropped,
            ))

        def run(self):
            arm = chassis = None
            c = None
            try:
                if getattr(args, "journal_path", None):
                    self.journal = SessionJournal(args.journal_path)
                    self.journal.record(dict(kind="session", schema="forge.weld_motion_journal",
                        version=1, simulated=not args.execute, settings=asdict(settings),
                        mapping=mapping.document, initial_raw_deg=list(initial),
                        created_utc=datetime.now(timezone.utc).isoformat()))
                # Load the model before opening hardware. This guard uses only
                # cached six-axis feedback and never acquires a transport owner.
                geometry = WeldGeometry(mapping) if args.arm else None
                if args.arm:
                    if args.execute:
                        from motors.arm_bench import read_arm_plan

                        plan, joints = read_arm_plan(args.arm_profile)
                        if tuple(joints) != tuple(range(1, 7)):
                            raise ValueError("All six arm joints are required")
                        plan.update(
                            fixture_confirmed=True,
                            fixture_mode="supported",
                            supported_confirmed=True,
                        )
                        from controls.controller.limits import motor_constraints

                        ceilings = motor_constraints(
                            root, settings.max_speed_deg_s, profile=settings.limit_profile
                        )
                        arm = ArmOwner(
                            plan,
                            mapping.document,
                            port=args.port,
                            max_speed_deg_s=max(
                                row["deployment_deg_s"] for row in ceilings["joints"]
                            ),
                            motion_allowed=self.motion_allowed,
                        )
                    else:
                        start = getattr(args, "start_raw_deg", None)
                        arm = SimulatedArm(start if start is not None else list(mapping.simulation_pose().values()))
                if args.chassis:
                    import yaml

                    chassis = ChassisOwner(
                        yaml.safe_load(args.chassis_profile.read_text()),
                        simulate=not args.execute,
                        execute=args.execute,
                        mapping_confirmed=args.mapping_confirmed,
                    )
                c = self.controller = WeldController(
                    arm, chassis, pose_check=None if geometry is None else geometry.check
                )
                last_display = 0.0
                displayed_plan = None
                while not self.finish.is_set():
                    tick_start = time.monotonic()
                    if self.cancel.is_set() or tick_start - self.heartbeat > 0.15:
                        interrupted_planning = self.pending is not None
                        self.clear_planning()
                        if interrupted_planning or c.state in c.ACTIVE or c.state in ("planned", "ready"):
                            c.pause()
                            if tick_start - self.heartbeat > 0.15:
                                c.message = "UI heartbeat lost; planning/motion cancelled. " + c.message
                        self.cancel.clear()
                    self.pause_for_focus_loss()
                    try:
                        action, values, focus_generation = self.commands.get_nowait()
                    except queue.Empty:
                        action = None
                    try:
                        if action in ("initial", "weld") and (
                            focus_generation != self.focus_generation or not self.motion_allowed()
                        ):
                            raise ValueError("Window focus changed; return to the panel and explicitly start again.")
                        if action == "select":
                            self.clear_planning()
                            c.select(*values)
                        elif action == "pause":
                            self.clear_planning()
                            c.pause()
                        elif action == "disable":
                            self.clear_planning()
                            c.disable_arm()
                        elif action == "calculate":
                            self.clear_planning()
                            c.pause()
                            if c.arm_on or c.chassis_on:
                                raw = (
                                    c.feedback(stationary=True).raw_deg.tolist()
                                    if c.arm_on
                                    else list(initial)
                                )
                                self.request = dict(
                                    mapping=mapping.document,
                                    current_raw_deg=raw,
                                    initial_raw_deg=list(initial),
                                    settings=asdict(settings),
                                    arm_enabled=c.arm_on,
                                    chassis_enabled=c.chassis_on,
                                )
                                if getattr(args, "scene_fingerprint", None):
                                    self.request["scene_fingerprint"] = args.scene_fingerprint
                                self.pending = self.executor.submit(build_plan, self.request)
                                self.planning_started = time.monotonic()
                                c.message = "Checking recovery, approach and weave; no new hold is enabled during planning."
                            else:
                                raise ValueError("Select arm and/or chassis first")
                        elif action == "initial":
                            c.go_initial()
                        elif action == "weld":
                            c.start_weld()
                        if self.pending is not None and self.pending.done():
                            future, self.pending = self.pending, None
                            self.planning_started = None
                            result = future.result()
                            if result["request_fingerprint"] != digest(self.request):
                                raise ValueError("Stale planning result")
                            c.load_plan(result)
                    except ValueError as exc:
                        c.message = "Blocked: " + str(exc)
                    except Exception as exc:
                        c.fail(exc)
                    if self.cancel.is_set():
                        self.clear_planning()
                        c.pause()
                        self.cancel.clear()
                    self.pause_for_focus_loss()
                    c.tick()
                    self.record_evidence()
                    if c.plan is not displayed_plan:
                        displayed_plan = c.plan
                        self.preview_plan.emit(displayed_plan)
                    if tick_start - last_display > 0.1:
                        rows = c.last_feedback
                        if arm is not None:
                            try:
                                rows = arm.display_feedback()
                            except (ValueError, RuntimeError):
                                rows = None
                        self.status.emit(
                            dict(
                                state=c.state,
                                message=c.message,
                                arm_on=c.arm_on,
                                chassis_on=c.chassis_on,
                                holding=c.arm_holding,
                                journal_error=None if self.journal is None else self.journal.error,
                                journal_dropped=0 if self.journal is None else self.journal.dropped,
                                raw_deg=None if rows is None else rows.raw_deg.tolist(),
                                feedback_age_ms=None if rows is None else float(max(rows.age_ms)),
                                pending=self.pending is not None,
                                planning_elapsed_s=(
                                    None if self.planning_started is None
                                    else max(0.0, time.monotonic() - self.planning_started)
                                ),
                                tracking=(dict(c.trace[-1]) if c.trace and
                                          c.trace[-1]["stage"] == c.state else None),
                                feed_m_s=0 if chassis is None else chassis.last_feed,
                                timing=(
                                    None
                                    if c.plan is None
                                    else {
                                        key: c.plan.get(key) for key in ("effective_hz", "feed_m_s")
                                    }
                                ),
                                preparation_timing=(None if c.last_accepted_plan is None else
                                                    c.last_accepted_plan.get("preparation_timing")),
                            )
                        )
                        last_display = tick_start
                    self.finish.wait(
                        max(0.0, settings.control_period_s - (time.monotonic() - tick_start))
                    )
            except Exception as exc:
                self.failed = True
                self.worker_error = str(exc)
                if self.journal is not None:
                    self.journal.record(dict(kind="worker_error", error=str(exc)))
                self.status.emit(
                    dict(
                        state="fault",
                        message=str(exc),
                        arm_on=False,
                        chassis_on=False,
                        holding=False,
                        raw_deg=None,
                        pending=False,
                        feed_m_s=0,
                    )
                )
            finally:
                self.clear_planning()
                if c is not None:
                    self.record_evidence("before_close")
                    c.close()
                    self.record_evidence("after_close")
                    self.failed = self.failed or c.fault is not None
                else:
                    for port in (chassis, arm):
                        if port:
                            try:
                                port.close()
                            except Exception:
                                self.failed = True
                self.report = dict(
                    schema="forge.weld_motion_session",
                    version=1,
                    created_utc=datetime.now(timezone.utc).isoformat(),
                    simulated=not args.execute,
                    start_report=str(getattr(args, "start_report", None) or "") or None,
                    settings=asdict(settings),
                    mapping_fingerprint=mapping.fingerprint,
                    initial_raw_deg=list(initial),
                    events=[] if c is None else c.events,
                    accepted_plan=None if c is None else c.last_accepted_plan,
                    trajectory_trace=[] if c is None else list(c.trace),
                    trace_dropped=0 if c is None else c.trace_dropped,
                    arm_session=getattr(getattr(arm, "port", None), "report", None),
                    fault=self.worker_error if c is None else c.fault or self.worker_error,
                    hardware_qualified=False,
                )
                if self.journal is not None:
                    self.journal.record(dict(kind="session_end", fault=self.report["fault"],
                        arm_session=self.report["arm_session"],
                        dropped_records=self.journal.dropped))
                    self.journal.close()
                    self.report["journal"] = dict(path=str(self.journal.path),
                        dropped_records=self.journal.dropped, error=self.journal.error)
                self.executor.shutdown(wait=False, cancel_futures=True)

    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.worker = OwnerThread()
            self.last_state = "idle"
            self.closing = False
            font = self.font()
            font.setPointSize(10)
            self.setFont(font)
            self.setWindowTitle(
                "FORGE Weld Motion Control — "
                + ("Hardware / Supported Fixture" if args.execute else "Offline Simulation")
            )
            layout = QtWidgets.QVBoxLayout(self)
            info = QtWidgets.QLabel(
                "Sequence: check measured pose → verify hold → standby → weld start → wait → weld motion\n"
                f"Requested: {settings.hz/settings.time_scale:.4f} Hz, "
                f"±{settings.amplitude_m*1000:g} mm, feed {settings.feed_m_s/settings.time_scale*1000:.3f} mm/s\n"
                f"Profile: {settings.limit_profile}; standby/approach speed limit: {settings.max_speed_deg_s:g}°/s. "
                "Weld speed is calculated from the selected constraints.\n"
                "Output switches do not add motion compensation. The calculated frequency is shown after planning."
            )
            info.setWordWrap(True)
            layout.addWidget(info)
            if getattr(args, "start_report", None):
                source = QtWidgets.QLabel(
                    f"Offline start: saved feedback from {args.start_report.name}. This is not live hardware."
                )
                source.setWordWrap(True)
                layout.addWidget(source)
            self.arm = QtWidgets.QCheckBox("Select arm (selection does not enable motors; existing hold is retained)")
            self.chassis = QtWidgets.QCheckBox("Chassis feed")
            self.arm.setEnabled(args.arm)
            self.chassis.setEnabled(args.chassis)
            layout.addWidget(self.arm)
            layout.addWidget(self.chassis)
            self.arm.clicked.connect(self.selection)
            self.chassis.clicked.connect(self.selection)
            buttons = QtWidgets.QHBoxLayout()
            layout.addLayout(buttons)
            self.buttons = {}
            for label, action in (
                ("Calculate and check path", "calculate"),
                ("Start recovery", "initial"),
                ("Start welding motion", "weld"),
                ("Pause / Hold (Space)", "pause"),
                ("Disable arm torque", "disable"),
            ):
                button = QtWidgets.QPushButton(label)
                button.setFocusPolicy(QtCore.Qt.NoFocus)
                button.clicked.connect(lambda checked=False, a=action: self.worker.send(a))
                buttons.addWidget(button)
                self.buttons[action] = button
            self.buttons["initial"].setToolTip(
                "Verify current-position hold, recover to standby, then move to the weld start."
            )
            self.state_label = QtWidgets.QLabel("Initializing the control thread…")
            self.state_label.setWordWrap(True)
            layout.addWidget(self.state_label)
            self.planning_progress = QtWidgets.QProgressBar()
            self.planning_progress.setRange(0, 0)
            self.planning_progress.setTextVisible(False)
            self.planning_progress.hide()
            layout.addWidget(self.planning_progress)
            self.timing_label = QtWidgets.QLabel(
                "Awaiting plan: components with output OFF are assumed to follow their nominal motion."
            )
            self.timing_label.setWordWrap(True)
            layout.addWidget(self.timing_label)
            self.preparation_label = QtWidgets.QLabel(f"Preparation time target: {settings.preparation_target_s:g} s (hold, standby, weld start).")
            self.preparation_label.setWordWrap(True)
            layout.addWidget(self.preparation_label)
            self.tracking_label = QtWidgets.QLabel()
            self.tracking_label.setWordWrap(True)
            layout.addWidget(self.tracking_label)
            self.journal_label = QtWidgets.QLabel()
            self.journal_label.setWordWrap(True)
            layout.addWidget(self.journal_label)
            self.table = QtWidgets.QTableWidget(6, 4)
            self.table.setHorizontalHeaderLabels(["Motor", "Measured raw angle (°)", "Standby raw target (°)", "Remaining to standby (°)"])
            self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            for i in range(6):
                self.table.setItem(i, 0, QtWidgets.QTableWidgetItem(f"A{i+1}"))
                self.table.setItem(i, 1, QtWidgets.QTableWidgetItem("—"))
                self.table.setItem(i, 2, QtWidgets.QTableWidgetItem(f"{initial[i]:+.4f}"))
                self.table.setItem(i, 3, QtWidgets.QTableWidgetItem("—"))
            content = QtWidgets.QHBoxLayout()
            content.addWidget(self.table, 2)
            self.preview = None
            if not getattr(args, "no_preview", False):
                from simulation.weld_preview import make_preview

                self.preview = make_preview(mapping, initial, simulated=not args.execute, parent=self)
                content.addWidget(self.preview, 3)
                self.worker.preview_plan.connect(self.preview.set_plan)
            layout.addLayout(content, 1)
            note = QtWidgets.QLabel(
                "This application controls motion only; it does not energize the welder. "
                "Pause stops the chassis and retains an existing arm hold. "
                "Disable, faults, or closing the window remove arm torque."
            )
            note.setWordWrap(True)
            layout.addWidget(note)
            self.worker.status.connect(self.show_status)
            self.pause_shortcuts = []
            for key in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape):
                shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(key), self)
                shortcut.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
                shortcut.setAutoRepeat(False)
                shortcut.activated.connect(lambda: self.worker.send("pause"))
                self.pause_shortcuts.append(shortcut)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.heartbeat)
            self.timer.start(30)
            self.resize(1240, 780 if self.preview is not None else 560)
            self.worker.start()

        def selection(self):
            self.worker.send("select", self.arm.isChecked(), self.chassis.isChecked())

        def heartbeat(self):
            self.worker.heartbeat = time.monotonic()
            if self.closing and not self.worker.isRunning():
                self.close()

        def show_status(self, value):
            self.last_state = value["state"]
            preparation = value.get("preparation_timing")
            if preparation:
                self.preparation_label.setText(
                    f"Preparation target: {preparation['target_s']:.2f} s | "
                    f"Nominal plan including minimum settling: {preparation['nominal_total_s']:.2f} s | "
                    + ("Nominal target met; physical time unverified." if preparation['nominal_meets_target'] else
                       "Target NOT met; feedback pacing can take longer.")
                )
            if value.get("journal_error") or value.get("journal_dropped"):
                self.journal_label.setText(
                    f"Evidence incomplete: {value.get('journal_error') or ''}; "
                    f"dropped records: {value.get('journal_dropped', 0)}"
                )
            elif getattr(args, "journal_path", None):
                self.journal_label.setText(f"Live evidence: {args.journal_path}")
            planning = value["pending"]
            self.planning_progress.setVisible(planning)
            if planning:
                elapsed = value.get("planning_elapsed_s") or 0.0
                self.state_label.setText(f"Planning ({elapsed:.1f} s): " + value["message"])
            else:
                self.state_label.setText(value["state"] + ": " + value["message"])
            timing = value.get("timing")
            if planning:
                self.timing_label.setText(
                    "Rechecking the complete path from the current pose. "
                    "You may switch windows; calculation and preview continue without starting motion."
                )
            elif timing and timing.get("effective_hz") is not None:
                self.timing_label.setText(
                    f"Calculated: {timing['effective_hz']:.5f} Hz; nominal feed {timing['feed_m_s']*1000:.4f} mm/s"
                )
            else:
                self.timing_label.setText(
                    "No accepted plan. Select outputs and calculate a path before starting recovery."
                )
            tracking = value.get("tracking")
            if tracking:
                errors = tracking["tracking_error_deg"]
                largest = max(range(6), key=lambda i: abs(errors[i]))
                self.tracking_label.setText(
                    f"Path rate: {tracking['path_clock_scale']*100:.0f}% | "
                    f"Intermediate reference error: A{largest+1} {errors[largest]:+.3f}° | "
                    f"A1 error: {errors[0]:+.3f}°; speed allowance: {tracking['speed_limit_deg_s'][0]:.2f}°/s"
                )
            else:
                self.tracking_label.clear()
            for widget, key in ((self.arm, "arm_on"), (self.chassis, "chassis_on")):
                widget.blockSignals(True)
                widget.setChecked(value[key])
                widget.blockSignals(False)
            self.buttons["initial"].setEnabled(value["state"] == "planned")
            self.buttons["weld"].setEnabled(value["state"] == "ready")
            self.buttons["calculate"].setEnabled(
                not value["pending"]
                and value["state"] not in WeldController.ACTIVE
                and value["state"] != "fault"
            )
            if value["raw_deg"] is not None:
                for i, angle in enumerate(value["raw_deg"]):
                    self.table.item(i, 1).setText(f"{angle:+.4f}")
                    self.table.item(i, 3).setText(f"{initial[i]-angle:+.4f}")
            else:
                for i in range(6):
                    for column in (1, 3):
                        self.table.item(i, column).setText("—")
            if self.preview is not None:
                self.preview.set_pose(value["raw_deg"], value.get("feedback_age_ms"))

        def keyPressEvent(self, event):
            if event.key() in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape):
                self.worker.send("pause")
                event.accept()
            else:
                super().keyPressEvent(event)

        def changeEvent(self, event):
            if event.type() == QtCore.QEvent.ActivationChange:
                self.worker.set_foreground(self.isActiveWindow())
            super().changeEvent(event)

        def closeEvent(self, event):
            if self.worker.isRunning():
                self.closing = True
                self.worker.cancel.set()
                self.worker.finish.set()
                self.state_label.setText("Stopping the chassis and confirming arm torque is disabled…")
                event.ignore()
            else:
                self.timer.stop()
                if self.preview is not None:
                    self.preview.shutdown()
                event.accept()

    return Window()


def main(argv=None):
    from robot_description.paths import model_directory

    root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--simulate", action="store_true")
    mode.add_argument(
        "--audit",
        action="store_true",
        help="offline initial-pose and speed audit; opens no hardware",
    )
    mode.add_argument(
        "--plan-only",
        action="store_true",
        help="solve and validate complete motion offline, without motor owners",
    )
    parser.add_argument(
        "--arm", action="store_true", help="make the arm owner available; UI motion starts OFF"
    )
    parser.add_argument(
        "--chassis",
        action="store_true",
        help="make the chassis owner available; UI feed starts OFF",
    )
    parser.add_argument("--supported", action="store_true")
    parser.add_argument("--mapping-confirmed", action="store_true")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument(
        "--mapping",
        type=Path,
        default=root / "src/robot_description/config/arm_calibration_measured.json",
    )
    parser.add_argument(
        "--initial-pose",
        type=Path,
        default=root / "src/robot_description/config/arm_initial_measured.json",
    )
    start_mode = parser.add_mutually_exclusive_group()
    start_mode.add_argument(
        "--demo-initial",
        action="store_true",
        help="simulation-only interior pose; never a hardware substitute",
    )
    start_mode.add_argument(
        "--start-report", type=Path,
        help="offline only: initialize fake shafts from a saved six-axis disabled diagnostic",
    )
    parser.add_argument("--no-preview", action="store_true", help="disable the embedded 3D renderer")
    parser.add_argument(
        "--arm-profile", type=Path, default=root / "src/robot_description/config/arm_rig_a1_a6.json"
    )
    parser.add_argument(
        "--chassis-profile", type=Path, default=root / "src/bringup/config/chassis.yaml"
    )
    parser.add_argument(
        "--profile",
        choices=("debug", "weld"),
        default="debug",
        help="weld uses the original 2 Hz target and native guarded model limits; debug retains low-speed policy",
    )
    parser.add_argument("--hz", type=float)
    parser.add_argument("--amplitude-mm", type=float, default=10.0)
    parser.add_argument("--feed-mm-s", type=float)
    parser.add_argument("--cycles", type=int, default=6)
    parser.add_argument("--prep-seconds", type=float, default=5.0,
                        help="total preparation timing goal, including hold/standby/approach; does not override limits")
    parser.add_argument(
        "--time-scale",
        type=float,
        default=1.0,
        help="explicit common slowdown of arm frequency and chassis feed",
    )
    parser.add_argument(
        "--strict-timing",
        action="store_true",
        help="reject infeasible timing instead of automatically slowing it",
    )
    parser.add_argument(
        "--lock-base-yaw",
        action="store_true",
        help="retain the old fixed-D1 experiment instead of using D1 redundancy",
    )
    parser.add_argument(
        "--max-speed-deg-s", type=float,
        help="preparation speed limit: default 5 deg/s; faster trials remain unqualified (maximum 10)",
    )
    parser.add_argument(
        "--solution",
        type=Path,
        help="reuse a CMA-ES candidate; revalidate fingerprints and the complete route",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.max_speed_deg_s is None:
        args.max_speed_deg_s = 5.0
    experiment = json.loads((root / "src/controls/config/weld_experiment.json").read_text())
    if args.hz is None:
        args.hz = experiment["frequency_hz"] if args.profile == "weld" else 2.0
    if args.feed_mm_s is None:
        args.feed_mm_s = (
            args.hz * experiment["feed_distance_per_cycle_mm"] if args.profile == "weld" else 12.0
        )
    if args.execute:
        if not (args.arm or args.chassis):
            parser.error("Select --arm and/or --chassis explicitly for hardware")
        if args.arm and not args.supported:
            parser.error("Arm hardware requires --supported")
        if args.chassis and not args.mapping_confirmed:
            parser.error("Chassis hardware requires --mapping-confirmed")
        if args.demo_initial:
            parser.error("--demo-initial is simulation-only")
        if args.start_report:
            parser.error("--start-report is offline-only; hardware always uses fresh feedback")
    else:
        if not (args.arm or args.chassis):
            args.arm = args.chassis = True
    if args.start_report and args.audit:
        parser.error("--start-report requires simulation or --plan-only")
    output = args.output or root / "build/weld" / f"weld-{time.time_ns()}.json"
    try:
        if output.exists():
            raise ValueError("Output already exists; choose a new report path")
        mapping = CalibrationMap(json.loads(args.mapping.read_text()))
        args.start_raw_deg = read_start_report(args.start_report, mapping) if args.start_report else None
        pose = json.loads(args.initial_pose.read_text())
        if pose.get("mapping_fingerprint", mapping.fingerprint) != mapping.fingerprint:
            raise ValueError(
                "Initial pose was derived for a different mapping; regenerate or use a reviewed matching file"
            )
        initial = initial_from_model(pose, mapping)
        if args.demo_initial:
            initial = list(mapping.simulation_pose().values())
        settings = Settings(
            hz=args.hz,
            amplitude_m=args.amplitude_mm / 1000,
            feed_m_s=args.feed_mm_s / 1000,
            cycles=args.cycles,
            time_scale=args.time_scale,
            max_speed_deg_s=args.max_speed_deg_s,
            auto_retime=not args.strict_timing,
            lock_base_yaw=args.lock_base_yaw,
            limit_profile=args.profile,
            search_solution=json.loads(args.solution.read_text()) if args.solution else None,
            preparation_target_s=args.prep_seconds,
        )
        if args.audit:
            result = audit(
                dict(mapping=mapping.document, initial_raw_deg=initial, settings=asdict(settings))
            )
            print(json.dumps(result, indent=2, ensure_ascii=False))
            exit_code = (
                0
                if not result["initial_issues"]
                and not result.get("weld_issue")
                and result.get("speed_report", {}).get("within_limits")
                else 2
            )
        elif args.plan_only:
            result = build_plan(
                dict(
                    mapping=mapping.document,
                    current_raw_deg=args.start_raw_deg if args.start_raw_deg is not None else initial,
                    initial_raw_deg=initial,
                    settings=asdict(settings),
                    arm_enabled=args.arm,
                    chassis_enabled=args.chassis,
                )
            )
            print(
                json.dumps(
                    dict(
                        initial_raw_deg=initial,
                        effective_hz=result["effective_hz"],
                        feed_mm_s=result["feed_m_s"] * 1000,
                        optimization=result["optimization"],
                    ),
                    indent=2,
                )
            )
            exit_code = 0
        else:
            from simulation.arm_calibration import scene_fingerprint

            args.scene_fingerprint = scene_fingerprint(model_directory() / "robot.xml")
            from PyQt5 import QtWidgets

            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])
            args.journal_path = output.with_suffix(".jsonl")
            print(f"Live journal: {args.journal_path}", flush=True)
            window = make_window(args, mapping, initial, settings, root)
            window.show()
            app.exec_()
            window.worker.wait()
            result = window.worker.report
            exit_code = 2 if window.worker.failed else 0
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False, ensure_ascii=False)
        print(f"Report: {output}")
        return exit_code
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
