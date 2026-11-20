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
from controls.controller.calibration import select_a1_encoder_branch
from controls.controller.limits import MAX_PREPARATION_SPEED_DEG_S
from controls.controller.tracking import TrackingSettings
from controls.controller.weld_plan import (
    Settings,
    WeldGeometry,
    initial_from_model,
    build_plan,
    audit,
)
from .weld_control import WeldController
from .weld_recording import SessionJournal, cached_arm_evidence
from .weld_dashboard import controller_presentation, state_summary


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
    raw = finite_vector(
        [row["raw_position_deg"] for row in sorted(rows, key=lambda row: row["joint"])],
        "recorded start",
    ).tolist()
    mapping.pose({str(i): q for i, q in enumerate(raw, 1)})
    return raw


def make_window(args, mapping, initial, settings, root):
    from PyQt5 import QtCore, QtGui, QtWidgets
    from .weld_ports import ArmOwner, SimulatedArm, ChassisOwner
    from teleop.arm_telemetry_widgets import TelemetryPanel, AXIS_COLORS

    tracker_settings = TrackingSettings() if getattr(args, "tracker", "pid") == "pid" else None
    tracker_document = None if tracker_settings is None else tracker_settings.document()

    class OwnerThread(QtCore.QThread):
        status = QtCore.pyqtSignal(object)
        preview_plan = QtCore.pyqtSignal(object)

        def __init__(self):
            super().__init__()
            self.commands = queue.Queue(maxsize=16)
            self.cancel = threading.Event()
            self.cancel_cause = "operator_input"
            self.finish = threading.Event()
            self.foreground = threading.Event()
            self.focus_stop = threading.Event()
            self.focus_generation = 0
            self.heartbeat = time.monotonic()
            self.heartbeat_expired = False
            self.resetting = threading.Event()
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
            if self.resetting.is_set():
                return  # Recovery owns the transport; require fresh input afterward.
            if action in ("pause", "disable", "select", "reset"):
                self.cancel_cause = {"select": "output_selection", "pause": "operator_input"}.get(action, action)
                self.cancel.set()
                while True:
                    try:
                        self.commands.get_nowait()
                    except queue.Empty:
                        break
            try:
                self.commands.put_nowait((action, values, self.focus_generation))
            except queue.Full:
                self.cancel_cause = "queue_full"
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
            return (
                self.foreground.is_set()
                and not self.focus_stop.is_set()
                and not self.cancel.is_set()
                and not self.finish.is_set()
                and time.monotonic() - self.heartbeat < 0.15
            )

        def pause_for_focus_loss(self):
            if self.foreground.is_set() and not self.focus_stop.is_set():
                return
            newly_lost = self.focus_stop.is_set()
            self.focus_stop.clear()
            c = self.controller
            if c.state in c.ACTIVE or (c.state == "ready" and newly_lost):
                c.pause(cause="focus_loss")
                c.message = "Window focus lost. " + c.message

        def intent_pause_cause(self):
            """Label cancellation during dispatch without changing stop behavior."""
            if self.focus_stop.is_set() or not self.foreground.is_set():
                return "focus_loss"
            if time.monotonic() - self.heartbeat >= .15:
                return "ui_heartbeat"
            if self.finish.is_set():
                return "close"
            return self.cancel_cause if self.cancel.is_set() else "intent_cancelled"

        def reset_motor_fault(self):
            c = self.controller
            if not c.fault or c.arm is None:
                raise ValueError("No connected arm fault to reset")
            self.clear_planning()
            self.resetting.set()
            try:
                self.record_evidence("before_motor_reset")
                self.status.emit(dict(
                    state="resetting", message="Saving fault evidence and verifying disabled, stationary motors…",
                    pending=False, arm_on=False, chassis_on=False, raw_deg=None,
                ))
                journal = getattr(args, "journal_path", None)
                evidence = (
                    Path(journal).with_suffix(".json") if journal else
                    root / "build" / "weld" / f"reset-{time.time_ns()}.json"
                )
                c.reset_motor_fault(evidence)
            finally:
                while True:
                    try:
                        self.commands.get_nowait()
                    except queue.Empty:
                        break
                self.cancel.clear()
                self.resetting.clear()
                self.record_evidence("after_motor_reset")

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
            events = c.events[self.recorded_events :]
            self.recorded_events = len(c.events)
            if (
                kind == "sample"
                and not events
                and not c.arm_holding
                and c.state not in c.ACTIVE
            ):
                return
            self.journal.record(
                dict(
                    kind=kind,
                    time_s=time.monotonic(),
                    state=c.state,
                    message=c.message,
                    events=events,
                    fault=c.fault,
                    pause_cause=c.pause_cause,
                    observation=c.last_observation,
                    measured_motion=c.motion_frequency.result if c.state == "welding" else None,
                    last_command=None if not c.trace else c.trace[-1],
                    arm=cached_arm_evidence(c.arm),
                    dropped_records=self.journal.dropped,
                )
            )

        def run(self):
            arm = chassis = None
            c = None
            try:
                if getattr(args, "journal_path", None):
                    self.journal = SessionJournal(args.journal_path)
                    self.journal.record(
                        dict(
                            kind="session",
                            schema="forge.weld_motion_journal",
                            version=1,
                            simulated=not args.execute,
                            continuous_weld=not getattr(args, "single_pass", False),
                            settings=asdict(settings),
                            tracker=tracker_document,
                            display_axis_colors={f"A{i+1}": color for i, color in enumerate(AXIS_COLORS)},
                            mapping=mapping.document,
                            initial_raw_deg=list(initial),
                            created_utc=datetime.now(timezone.utc).isoformat(),
                        )
                    )
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
                        if self.journal is not None:
                            self.journal.record(dict(
                                kind="arm_configuration", profile=plan,
                                profile_source=str(args.arm_profile),
                            ))
                        from controls.controller.limits import motor_constraints

                        ceilings = motor_constraints(
                            root,
                            settings.max_speed_deg_s,
                            profile=settings.limit_profile,
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
                        arm = SimulatedArm(
                            start
                            if start is not None
                            else list(mapping.simulation_pose().values())
                        )
                if args.chassis:
                    import yaml

                    chassis = ChassisOwner(
                        yaml.safe_load(args.chassis_profile.read_text()),
                        simulate=not args.execute,
                        execute=args.execute,
                        mapping_confirmed=args.mapping_confirmed,
                    )
                c = self.controller = WeldController(
                    arm,
                    chassis,
                    pose_check=None if geometry is None else geometry.check,
                    endpoint_check=None if geometry is None else geometry.check_endpoint,
                    allow_optimized_preparation=settings.optimize_preparation,
                    continuous_weld=not getattr(args, "single_pass", False),
                    tracking_settings=tracker_settings,
                    intent_pause_cause=self.intent_pause_cause,
                )
                if getattr(arm, "fault", None):
                    c.fail(RuntimeError(arm.fault))
                last_display = 0.0
                displayed_plan = None
                while not self.finish.is_set():
                    tick_start = time.monotonic()
                    expired = tick_start - self.heartbeat > 0.15
                    newly_expired = expired and not self.heartbeat_expired
                    self.heartbeat_expired = expired
                    if newly_expired:
                        self.focus_generation += 1
                    if self.cancel.is_set() or newly_expired:
                        interrupted_planning = self.pending is not None
                        self.clear_planning()
                        if (
                            interrupted_planning
                            or c.state in c.ACTIVE
                            or c.state in ("planned", "ready")
                        ):
                            c.pause(cause="ui_heartbeat" if newly_expired else self.cancel_cause)
                            if tick_start - self.heartbeat > 0.15:
                                c.message = (
                                    "UI heartbeat lost; planning/motion cancelled. "
                                    + c.message
                                )
                        self.cancel.clear()
                    self.pause_for_focus_loss()
                    try:
                        action, values, focus_generation = self.commands.get_nowait()
                    except queue.Empty:
                        action = None
                    try:
                        if action in ("initial", "weld") and (
                            focus_generation != self.focus_generation
                            or not self.motion_allowed()
                        ):
                            raise ValueError(
                                "Window focus changed; return to the panel and explicitly start again."
                            )
                        if action == "select":
                            self.clear_planning()
                            c.select(*values)
                        elif action == "pause":
                            self.clear_planning()
                            c.pause(cause="operator_input")
                        elif action == "disable":
                            self.clear_planning()
                            c.disable_arm()
                        elif action == "reset":
                            self.reset_motor_fault()
                        elif action == "calculate":
                            self.clear_planning()
                            c.pause(preserve_ready=False, cause="recalculate")
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
                                    self.request["scene_fingerprint"] = (
                                        args.scene_fingerprint
                                    )
                                self.pending = self.executor.submit(
                                    build_plan, self.request
                                )
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
                        c.message = (
                            f"FAULT: {c.fault}; motor reset failed: {exc}"
                            if action == "reset" and c.fault else "Blocked: " + str(exc)
                        )
                        c.event(
                            "action_blocked",
                            action=action or "planning_result",
                            error=str(exc),
                            planning_start_raw_deg=(
                                None if self.request is None
                                else self.request.get("current_raw_deg")
                            ),
                        )
                    except Exception as exc:
                        c.fail(exc)
                        if action == "reset":
                            c.message += f"; motor reset failed: {exc}"
                    if self.cancel.is_set():
                        self.clear_planning()
                        c.pause(cause=self.cancel_cause)
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
                                joint_goal=c.joint_goal(),
                                hold_observation=(
                                    c.last_observation
                                    if c.state in ("ready", "complete")
                                    else None
                                ),
                                state=c.state,
                                **controller_presentation(c, tick_start),
                                captured_at=time.monotonic(),
                                telemetry=None if rows is None else rows.telemetry,
                                fault=c.fault,
                                fault_evidence=cached_arm_evidence(arm) if c.fault else None,
                                message=c.message,
                                arm_on=c.arm_on,
                                chassis_on=c.chassis_on,
                                holding=c.arm_holding,
                                measured_motion=c.motion_frequency.result if c.state == "welding" else None,
                                planned_travel_deg=c.weld_travel_deg if c.plan is not None else None,
                                reset_available=c.fault is not None and arm is not None,
                                journal_error=(
                                    None if self.journal is None else self.journal.error
                                ),
                                journal_dropped=(
                                    0 if self.journal is None else self.journal.dropped
                                ),
                                raw_deg=None if rows is None else rows.raw_deg.tolist(),
                                feedback_age_ms=(
                                    None if rows is None else float(max(rows.age_ms))
                                ),
                                pending=self.pending is not None,
                                planning_elapsed_s=(
                                    None
                                    if self.planning_started is None
                                    else max(
                                        0.0, time.monotonic() - self.planning_started
                                    )
                                ),
                                tracking=(
                                    dict(c.trace[-1])
                                    if c.trace and c.trace[-1]["stage"] == c.state
                                    else None
                                ),
                                feed_m_s=0 if chassis is None else chassis.last_feed,
                                timing=(
                                    None
                                    if c.plan is None
                                    else {
                                        key: c.plan.get(key)
                                        for key in ("effective_hz", "feed_m_s")
                                    }
                                ),
                                preparation_timing=(
                                    None
                                    if c.last_accepted_plan is None
                                    else c.last_accepted_plan.get("preparation_timing")
                                ),
                            )
                        )
                        last_display = tick_start
                    self.finish.wait(
                        max(
                            0.0,
                            settings.control_period_s - (time.monotonic() - tick_start),
                        )
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
                    continuous_weld=not getattr(args, "single_pass", False),
                    start_report=str(getattr(args, "start_report", None) or "") or None,
                    settings=asdict(settings),
                    tracker=tracker_document,
                    mapping_fingerprint=mapping.fingerprint,
                    encoder_branch_selection=mapping.document.get("encoder_branch_selection"),
                    initial_raw_deg=list(initial),
                    events=[] if c is None else c.events,
                    accepted_plan=None if c is None else c.last_accepted_plan,
                    trajectory_trace=[] if c is None else list(c.trace),
                    trace_dropped=0 if c is None else c.trace_dropped,
                    arm_session=getattr(getattr(arm, "port", None), "report", None),
                    fault=(
                        self.worker_error if c is None else c.fault or self.worker_error
                    ),
                    hardware_qualified=False,
                )
                if self.journal is not None:
                    self.journal.record(
                        dict(
                            kind="session_end",
                            fault=self.report["fault"],
                            arm_session=self.report["arm_session"],
                            dropped_records=self.journal.dropped,
                        )
                    )
                    self.journal.close()
                    self.report["journal"] = dict(
                        path=str(self.journal.path),
                        dropped_records=self.journal.dropped,
                        error=self.journal.error,
                    )
                self.executor.shutdown(wait=False, cancel_futures=True)

    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.worker = OwnerThread()
            self.last_state = "idle"
            self.last_status = None
            self.first_fault = None
            self.next_telemetry_paint = 0
            self.closing = False
            font = self.font()
            font.setPointSize(10)
            self.setFont(font)
            self.setWindowTitle("FORGE Weld Motion · " + ("Supported hardware" if args.execute else "Offline simulation"))
            self.setStyleSheet("""
                QWidget { color: #172b46; }
                QPushButton { padding: 7px 10px; border: 1px solid #bccbdc; border-radius: 5px; background: #f8fafc; }
                QPushButton:disabled { color: #8b99aa; background: #f1f5f9; }
                QPushButton:hover:enabled { background: #e6eef9; }
                QGroupBox { font-weight: 600; }
                QTabWidget::pane { border: 1px solid #d8e1ec; background: #f8fafc; }
                QTabBar::tab { padding: 7px 14px; }
            """)
            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(12, 10, 12, 8)
            layout.setSpacing(7)
            header = QtWidgets.QHBoxLayout()
            title = QtWidgets.QLabel("FORGE  /  WELD MOTION")
            title.setStyleSheet("font-size: 17px; font-weight: 700")
            header.addWidget(title)
            header.addStretch()
            header.addWidget(QtWidgets.QLabel("SUPPORTED HARDWARE" if args.execute else "OFFLINE SIMULATION"))
            layout.addLayout(header)
            self.state_label = QtWidgets.QLabel("Initializing the control thread…")
            self.state_label.setWordWrap(True)
            self.state_label.setTextFormat(QtCore.Qt.PlainText)
            self.state_label.setMinimumHeight(40)
            layout.addWidget(self.state_label)
            self.next_action = QtWidgets.QLabel()
            self.next_action.setWordWrap(True)
            self.output_label = QtWidgets.QLabel("Arm output OFF · Chassis output OFF · Arm hold inactive")
            summary = QtWidgets.QHBoxLayout()
            summary.addWidget(self.next_action, 1)
            summary.addWidget(self.output_label)
            layout.addLayout(summary)
            controls = QtWidgets.QHBoxLayout()
            self.arm = QtWidgets.QCheckBox("Arm output")
            self.arm.setToolTip("Selection does not enable motors. Changing outputs keeps an existing hold and invalidates the plan.")
            self.chassis = QtWidgets.QCheckBox("Chassis feed")
            self.arm.setEnabled(args.arm)
            self.chassis.setEnabled(args.chassis)
            for widget in (self.arm, self.chassis):
                controls.addWidget(widget)
                widget.clicked.connect(self.selection)
            controls.addStretch()
            self.buttons = {}
            for label, action in (("1  Calculate path", "calculate"), ("2  Start recovery", "initial"), ("3  Start welding", "weld")):
                button = QtWidgets.QPushButton(label)
                button.setFocusPolicy(QtCore.Qt.NoFocus)
                button.clicked.connect(lambda checked=False, a=action: self.worker.send(a))
                controls.addWidget(button)
                self.buttons[action] = button
            layout.addLayout(controls)
            stop_row = QtWidgets.QHBoxLayout()
            self.timing_label = QtWidgets.QLabel("No accepted path")
            self.timing_label.setWordWrap(True)
            stop_row.addWidget(self.timing_label, 1)
            for label, action in (("Pause / Hold  [Space]", "pause"), ("Disable arm torque", "disable"), ("Reset motor fault  [R]", "reset")):
                button = QtWidgets.QPushButton(label)
                button.setFocusPolicy(QtCore.Qt.NoFocus)
                button.clicked.connect(lambda checked=False, a=action: self.worker.send(a))
                stop_row.addWidget(button)
                self.buttons[action] = button
            self.buttons["pause"].setStyleSheet("background: #fff0c2; color: #674a00; font-weight: 600")
            self.buttons["disable"].setStyleSheet("color: #a52c35; border-color: #d7a3a8")
            self.buttons["initial"].setToolTip("Verify current-position hold, recover to standby, then move to the weld start.")
            self.buttons["reset"].setEnabled(False)
            self.buttons["reset"].setToolTip("Save first-fault evidence and verify disabled, stationary motors through the existing owner. Success stays disabled.")
            layout.addLayout(stop_row)
            self.planning_progress = QtWidgets.QProgressBar()
            self.planning_progress.setRange(0, 0)
            self.planning_progress.setFixedHeight(5)
            self.planning_progress.setTextVisible(False)
            self.planning_progress.hide()
            layout.addWidget(self.planning_progress)
            self.motion_progress = QtWidgets.QProgressBar()
            self.motion_progress.setRange(0, 1000)
            self.motion_progress.setFixedHeight(19)
            self.motion_progress.hide()
            layout.addWidget(self.motion_progress)
            self.preparation_label = QtWidgets.QLabel(f"Preparation goal {settings.preparation_target_s:g} s · No measured run yet")
            layout.addWidget(self.preparation_label)
            content = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
            self.preview = None
            if not getattr(args, "no_preview", False):
                from simulation.weld_preview import make_preview
                self.preview = make_preview(mapping, initial, simulated=not args.execute, parent=self)
                preview_scroll = QtWidgets.QScrollArea()
                preview_scroll.setWidgetResizable(True)
                preview_scroll.setWidget(self.preview)
                content.addWidget(preview_scroll)
                self.worker.preview_plan.connect(self.preview.set_plan)
            self.tabs = QtWidgets.QTabWidget()
            telemetry_page = QtWidgets.QWidget()
            telemetry_layout = QtWidgets.QVBoxLayout(telemetry_page)
            legend = QtWidgets.QLabel("30 s trends · Torque: axis color · Temperature: motor orange / MOS purple")
            legend.setToolTip("Fresh-only peaks: current stage / recovery run. Hover a card for temperature peaks.")
            legend.setWordWrap(True)
            legend.setStyleSheet("color: #526176; font-size: 11px")
            telemetry_layout.addWidget(legend)
            self.telemetry = TelemetryPanel()
            telemetry_layout.addWidget(self.telemetry, 1)
            telemetry_scroll = QtWidgets.QScrollArea()
            telemetry_scroll.setWidgetResizable(True)
            telemetry_scroll.setWidget(telemetry_page)
            self.tabs.addTab(telemetry_scroll, "Load && temperature")
            motion_page = QtWidgets.QWidget()
            motion_layout = QtWidgets.QVBoxLayout(motion_page)
            self.frequency_group = QtWidgets.QGroupBox("Motor response")
            frequency_layout = QtWidgets.QVBoxLayout(self.frequency_group)
            frequency_grid = QtWidgets.QGridLayout()
            self.frequency_values, self.frequency_details = [], []
            for i, color in enumerate(AXIS_COLORS):
                card = QtWidgets.QWidget()
                box = QtWidgets.QVBoxLayout(card)
                box.setContentsMargins(3, 3, 3, 3)
                axis = QtWidgets.QLabel(f"A{i+1}")
                axis.setStyleSheet(f"color: {color}; font-weight: 700")
                reading = QtWidgets.QLabel("—")
                reading.setStyleSheet("font-size: 18px; font-weight: 600")
                detail = QtWidgets.QLabel("Waiting for welding")
                detail.setWordWrap(True)
                for widget in (axis, reading, detail):
                    box.addWidget(widget)
                frequency_grid.addWidget(card, i // 3, i % 3)
                self.frequency_values.append(reading)
                self.frequency_details.append(detail)
            frequency_layout.addLayout(frequency_grid)
            self.frequency_status = QtWidgets.QLabel()
            self.frequency_status.setWordWrap(True)
            frequency_layout.addWidget(self.frequency_status)
            self.table = QtWidgets.QTableWidget(6, 6)
            self.table.setHorizontalHeaderLabels(["Motor", "Measured raw\nangle (°)", "Stage goal raw\nangle (°)", "Remaining to\nstage goal (°)", "Measured Hz", "Travel\n(° p-p)"])
            self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            self.table.verticalHeader().hide()
            self.table.verticalHeader().setDefaultSectionSize(26)
            self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
            self.table.setColumnWidth(0, 48)
            for axis in range(6):
                for column in range(6):
                    item = QtWidgets.QTableWidgetItem(f"A{axis+1}" if column == 0 else "—")
                    if column == 0:
                        item.setForeground(QtGui.QColor(AXIS_COLORS[axis]))
                    self.table.setItem(axis, column, item)
            motion_layout.addWidget(self.table, 1)
            self.frequency_toggle = QtWidgets.QToolButton()
            self.frequency_toggle.setText("Frequency && travel details")
            self.frequency_toggle.setCheckable(True)
            self.frequency_toggle.setArrowType(QtCore.Qt.RightArrow)
            self.frequency_toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
            self.frequency_toggle.toggled.connect(self.frequency_group.setVisible)
            self.frequency_toggle.toggled.connect(lambda checked: self.frequency_toggle.setArrowType(QtCore.Qt.DownArrow if checked else QtCore.Qt.RightArrow))
            motion_layout.addWidget(self.frequency_toggle)
            motion_layout.addWidget(self.frequency_group)
            self.frequency_group.hide()
            motion_scroll = QtWidgets.QScrollArea()
            motion_scroll.setWidgetResizable(True)
            motion_scroll.setWidget(motion_page)
            self.tabs.addTab(motion_scroll, "Targets && response")
            content.addWidget(self.tabs)
            content.setChildrenCollapsible(False)
            content.setSizes([510, 740])
            layout.addWidget(content, 1)
            self.details_toggle = QtWidgets.QToolButton()
            self.details_toggle.setText("Diagnostics && evidence")
            self.details_toggle.setCheckable(True)
            self.details_toggle.setArrowType(QtCore.Qt.RightArrow)
            self.details_toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
            layout.addWidget(self.details_toggle)
            self.details = QtWidgets.QScrollArea()
            self.details.setWidgetResizable(True)
            self.details.setMaximumHeight(165)
            detail_body = QtWidgets.QWidget()
            detail_layout = QtWidgets.QVBoxLayout(detail_body)
            self.controller_label = QtWidgets.QLabel("Tracker: PID + velocity feedforward" if tracker_settings is not None else "Tracker: feedback-paced")
            self.weld_run_label = QtWidgets.QLabel(f"Welding: one planned pass ({settings.cycles} cycles), then hold." if getattr(args, "single_pass", False) else "Welding: continuous until Pause / Hold.")
            self.preparation_route_label = QtWidgets.QLabel("Route: " + ("optimized staging" if settings.optimize_preparation else "saved standby") + f" · request {settings.max_speed_deg_s:g}°/s · lift {settings.approach_lift_m*1000:g} mm · final lowering minimum {settings.lowering_min_s:g} s")
            self.tracking_label = QtWidgets.QLabel()
            self.message_detail = QtWidgets.QLabel()
            self.fault_detail = QtWidgets.QLabel()
            self.journal_label = QtWidgets.QLabel()
            for widget in (self.controller_label, self.weld_run_label, self.preparation_route_label, self.tracking_label, self.message_detail, self.fault_detail, self.journal_label):
                widget.setWordWrap(True)
                widget.setTextFormat(QtCore.Qt.PlainText)
                widget.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                detail_layout.addWidget(widget)
            profile = getattr(args, "arm_profile", None)
            profile_text = f"Arm profile: {profile or 'simulation'}\nTorque trip = reactive guard; TMAX = feedback encoding range; neither is a continuous rating. Temperature trips: motor / MOS 80 °C."
            if profile and profile.exists():
                document = json.loads(profile.read_text())
                ratings = document.get("torque_policy", {}).get("rated_reference_nm", {})
                profile_text += "\n" + " | ".join(f"{motor['unit']}: trip {motor['torque_trip_nm']:g} Nm, TMAX {motor['ranges']['tmax']:g} Nm, rated {ratings.get(motor['unit'], 'unknown')} Nm" for motor in document["motors"])
            branch = mapping.document.get("encoder_branch_selection")
            if branch:
                profile_text += f"\nA1 branch: {branch['turns']:+d} turn, {branch['raw_shift_deg']:+g}° raw shift. Verify physical pose."
            profile_label = QtWidgets.QLabel(profile_text)
            profile_label.setWordWrap(True)
            detail_layout.addWidget(profile_label)
            self.details.setWidget(detail_body)
            self.details.hide()
            self.details_toggle.toggled.connect(self.toggle_details)
            layout.addWidget(self.details)
            self.evidence_warning = QtWidgets.QLabel()
            self.evidence_warning.setWordWrap(True)
            self.evidence_warning.setStyleSheet("color: #a52c35")
            self.evidence_warning.hide()
            layout.addWidget(self.evidence_warning)
            note = QtWidgets.QLabel("Motion only · Pause retains arm hold · Disable / fault / close removes torque")
            note.setStyleSheet("color: #526176; font-size: 11px")
            layout.addWidget(note)
            self.show_motor_frequency(dict(state="idle", arm_on=False))
            self.worker.status.connect(self.show_status)
            self.pause_shortcuts = []
            for key in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape):
                shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(key), self)
                shortcut.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
                shortcut.setAutoRepeat(False)
                shortcut.activated.connect(lambda: self.worker.send("pause"))
                self.pause_shortcuts.append(shortcut)
            self.reset_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence("R"), self)
            self.reset_shortcut.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
            self.reset_shortcut.setAutoRepeat(False)
            self.reset_shortcut.activated.connect(self.buttons["reset"].click)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.heartbeat)
            self.timer.start(30)
            self.resize(1280, 800)
            self.worker.start()

        def toggle_details(self, expanded):
            self.details.setVisible(expanded)
            self.details_toggle.setArrowType(QtCore.Qt.DownArrow if expanded else QtCore.Qt.RightArrow)

        def selection(self):
            self.worker.send("select", self.arm.isChecked(), self.chassis.isChecked())

        def heartbeat(self):
            self.worker.heartbeat = time.monotonic()
            if self.last_status is not None and time.monotonic() >= self.next_telemetry_paint:
                self.refresh_telemetry()
            if self.closing and not self.worker.isRunning():
                self.close()

        def refresh_telemetry(self):
            value = self.last_status
            self.telemetry.update_samples(
                value.get("telemetry"), captured_at=value["captured_at"],
                stage=value["state"], run=value.get("run_id"),
                stale=value["state"] in ("fault", "resetting"),
            )
            self.next_telemetry_paint = time.monotonic() + .1

        def show_status(self, value):
            value = dict(value)
            value.setdefault("captured_at", time.monotonic())
            self.last_status = value
            self.last_state = value["state"]
            self.refresh_telemetry()
            preparation = value.get("preparation_timing")
            if preparation:
                measured = value.get("preparation_elapsed_s")
                self.preparation_label.setText(
                    f"Preparation · {'measured —' if measured is None else f'measured {measured:.2f} s'}"
                    f" / nominal {preparation['nominal_total_s']:.2f} s / goal {preparation['target_s']:.2f} s"
                    + (" · nominal goal exceeded" if not preparation["nominal_meets_target"] else "")
                )
            else:
                self.preparation_label.setText(f"Preparation goal {settings.preparation_target_s:g} s · No accepted timing")
            if value.get("journal_error") or value.get("journal_dropped"):
                self.journal_label.setText(
                    f"Evidence incomplete: {value.get('journal_error') or ''}; "
                    f"dropped records: {value.get('journal_dropped', 0)}"
                )
                self.evidence_warning.setText(self.journal_label.text())
                self.evidence_warning.show()
            elif getattr(args, "journal_path", None):
                self.journal_label.setText(f"Live evidence: {args.journal_path}")
                self.evidence_warning.hide()
            planning = value["pending"]
            self.planning_progress.setVisible(planning)
            title, action, severity = state_summary(value)
            colors = {"normal": ("#e8f3ee", "#215844"), "planning": ("#eaf0fd", "#294e91"),
                      "paused": ("#fff2cf", "#79560b"), "warning": ("#fff2cf", "#79560b"),
                      "blocked": ("#fff0e0", "#8c4a14"), "fault": ("#fbe8e9", "#a12535")}
            background, foreground = colors[severity]
            self.state_label.setStyleSheet(f"background: {background}; color: {foreground}; padding: 8px; border-radius: 5px; font-size: 17px; font-weight: 600")
            if severity in ("fault", "blocked"):
                message = value["message"]
                title += ": " + (message if len(message) <= 220 else message[:217] + "…")
            self.state_label.setText(title)
            self.state_label.setToolTip(value["message"])
            self.next_action.setText(action)
            self.message_detail.setText(value["message"])
            fault_evidence = value.get("fault_evidence") or {}
            if value["state"] == "fault" and self.first_fault is None:
                self.first_fault = dict(message=value.get("fault") or value["message"], evidence=fault_evidence)
            if self.first_fault is not None:
                self.fault_detail.setText("First fault (retained): " + str(self.first_fault["message"]) + "\n"
                                          + json.dumps(self.first_fault["evidence"], ensure_ascii=True, sort_keys=True))
            self.output_label.setText(
                f"Selected: arm {'ON' if value['arm_on'] else 'OFF'} / chassis {'ON' if value['chassis_on'] else 'OFF'}"
                f" · Arm hold {'active' if value.get('holding') else 'inactive'}"
            )
            progress = value.get("stage_progress")
            self.motion_progress.setVisible(progress is not None and not planning)
            if progress is not None:
                self.motion_progress.setValue(round(progress * 1000))
                elapsed, nominal = value.get("stage_elapsed_s"), value.get("stage_nominal_s")
                self.motion_progress.setFormat(
                    f"{'Block' if value['state'] == 'welding' else 'Path'} {progress*100:.0f}%"
                    + (f" · {'run' if value['state'] == 'welding' else 'leg'} elapsed {elapsed:.1f} s / nominal {'block' if value['state'] == 'welding' else 'leg'} {nominal:.1f} s" if elapsed is not None and nominal is not None else "")
                )
            timing = value.get("timing")
            if planning:
                self.timing_label.setText(
                    f"Calculation: {value.get('planning_elapsed_s') or 0:.1f} s · separate from motion time"
                )
            elif timing and timing.get("effective_hz") is not None:
                self.timing_label.setText(
                    f"Planned {timing['effective_hz']:.3f} Hz · {timing['feed_m_s']*1000:.2f} mm/s"
                )
            else:
                self.timing_label.setText(
                    "No accepted path"
                )
            tracking = value.get("tracking")
            if tracking:
                errors = tracking["tracking_error_deg"]
                largest = max(range(6), key=lambda i: abs(errors[i]))
                tracker = tracking.get("tracker") or {}
                correction_scale = tracker.get("clearance_correction_scale")
                clearance_note = ""
                if tracker.get("projected"):
                    if correction_scale is None:
                        clearance_note = " | PID correction constrained by clearance"
                    elif correction_scale < 1 - 1e-6:
                        clearance_note = (
                            f" | Clearance admits {100 * correction_scale:.0f}% of PID correction"
                        )
                self.tracking_label.setText(
                    f"Path rate: {tracking['path_clock_scale']*100:.0f}% | "
                    f"Intermediate reference error: A{largest+1} {errors[largest]:+.3f}° | "
                    f"A1 error: {errors[0]:+.3f}°; speed allowance: {tracking['speed_limit_deg_s'][0]:.2f}°/s"
                    + clearance_note
                )
            elif (
                value.get("hold_observation")
                and "tracking_error_deg" in value["hold_observation"]
            ):
                hold = value["hold_observation"]
                errors = hold["tracking_error_deg"]
                largest = max(range(6), key=lambda i: abs(errors[i]))
                self.tracking_label.setText(
                    f"Holding final target | Largest error: A{largest+1} {errors[largest]:+.3f}° | "
                    f"Arrival tolerance: {hold['arrival_tolerance_deg']:.2f}°"
                    + (
                        " | HOLD DRIFT"
                        if max(abs(e) for e in errors) > hold["arrival_tolerance_deg"]
                        else ""
                    )
                )
            else:
                self.tracking_label.clear()
            for widget, key in ((self.arm, "arm_on"), (self.chassis, "chassis_on")):
                widget.blockSignals(True)
                widget.setChecked(value[key])
                widget.blockSignals(False)
            self.buttons["initial"].setEnabled(value["state"] == "planned")
            self.buttons["weld"].setEnabled(value["state"] == "ready")
            self.buttons["reset"].setEnabled(bool(value.get("reset_available")))
            self.buttons["calculate"].setEnabled(
                not value["pending"]
                and value["state"] not in WeldController.ACTIVE
                and value["state"] not in ("fault", "resetting")
            )
            self.arm.setEnabled(args.arm and value["state"] not in ("fault", "resetting"))
            self.chassis.setEnabled(args.chassis and value["state"] not in ("fault", "resetting"))
            goal = value.get("joint_goal")
            motion = value.get("measured_motion") if value["state"] == "welding" else None
            self.show_motor_frequency(value)
            self.table.horizontalHeaderItem(2).setText(
                f"{goal['label']}\nraw angle (°)" if goal else "Stage goal raw\nangle (°)"
            )
            for i in range(6):
                measured = motion[i] if motion else {}
                hz = measured.get("hz")
                travel = measured.get("peak_to_peak_deg")
                self.table.item(i, 4).setText(
                    f"{hz:.3f}" if hz is not None else measured.get("status", "—")
                )
                self.table.item(i, 4).setToolTip(measured.get("status", ""))
                self.table.item(i, 5).setText(f"{travel:.3f}" if travel is not None else "—")
                self.table.item(i, 2).setText(
                    f"{goal['raw_deg'][i]:+.4f}" if goal else "—"
                )
            if value["raw_deg"] is not None:
                for i, angle in enumerate(value["raw_deg"]):
                    self.table.item(i, 1).setText(f"{angle:+.4f}")
                    self.table.item(i, 3).setText(
                        f"{goal['raw_deg'][i]-angle:+.4f}" if goal else "—"
                    )
            else:
                for i in range(6):
                    for column in (1, 3):
                        self.table.item(i, column).setText("—")
            if self.preview is not None:
                self.preview.set_pose(value["raw_deg"], value.get("feedback_age_ms"))

        def show_motor_frequency(self, value):
            """Render cached measurements only; never infer actual Hz from the plan."""
            planned = (value.get("timing") or {}).get("effective_hz")
            planned_text = f"{planned:.3f} Hz" if planned is not None else "—"
            self.frequency_group.setTitle(
                ("Measured motor frequency" if args.execute else "Simulated motor frequency")
                + f" — Requested weave: {settings.hz/settings.time_scale:.3f} Hz  |  Planned weave: {planned_text}"
            )
            live = value["state"] == "welding" and value.get("arm_on", False)
            motion = value.get("measured_motion") if live else None
            if live:
                status = "Encoder pattern frequency; travel is shown separately. A two-cycle response repeats every other weave cycle. Updates every 0.5 s."
            elif value["state"] == "paused":
                status = "Paused — live frequency measurement stopped."
            else:
                status = "Frequency is measured while the arm performs welding motion."
            self.frequency_status.setText(status)
            for i, (reading, detail) in enumerate(zip(self.frequency_values, self.frequency_details)):
                measured = motion[i] if motion else {}
                hz = measured.get("hz")
                reading.setText(f"{hz:.3f} Hz" if hz is not None else "—")
                if hz is not None:
                    if planned and 0.45 <= hz / planned <= 0.55:
                        text = "Two-cycle response"
                    else:
                        text = f"Pattern rate: {100*hz/planned:.0f}% of weave" if planned else "Measured from feedback"
                elif not live:
                    text = "Paused" if value["state"] == "paused" else "Waiting for welding"
                else:
                    text = {
                        "insufficient motion": "Too little motion",
                        "stale": "Stale feedback",
                        "irregular / collecting": "Collecting / irregular",
                    }.get(measured.get("status"), "Collecting cycles…")
                travel = measured.get("peak_to_peak_deg")
                planned_travel = value.get("planned_travel_deg")
                if travel is not None:
                    text += f"\nTravel: {travel:.3f}°"
                    if planned_travel is not None:
                        text += f" / {planned_travel[i]:.3f}° planned"
                detail.setText(text)
                reading.setToolTip(
                    "Measured repeating-pattern frequency from shaft feedback. "
                    "An alternating response can repeat over two planned cycles."
                )

        def keyPressEvent(self, event):
            if event.key() in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape):
                self.worker.send("pause")
                event.accept()
            elif event.key() == QtCore.Qt.Key_R:
                if not event.isAutoRepeat() and self.buttons["reset"].isEnabled():
                    self.worker.send("reset")
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
                self.state_label.setText(
                    "Stopping the chassis and confirming arm torque is disabled…"
                )
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
        "--arm",
        action="store_true",
        help="make the arm owner available; UI motion starts OFF",
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
    parser.add_argument(
        "--a1-encoder-turns",
        type=int,
        choices=(-1, 0, 1),
        default=0,
        help="explicit A1 restart branch relative to the saved mapping; +1 shifts raw bounds/targets by +360 deg, after physical pose verification",
    )
    start_mode = parser.add_mutually_exclusive_group()
    start_mode.add_argument(
        "--demo-initial",
        action="store_true",
        help="simulation-only interior pose; never a hardware substitute",
    )
    start_mode.add_argument(
        "--start-report",
        type=Path,
        help="offline only: initialize fake shafts from a saved six-axis disabled diagnostic",
    )
    parser.add_argument(
        "--no-preview", action="store_true", help="disable the embedded 3D renderer"
    )
    parser.add_argument(
        "--arm-profile",
        type=Path,
        default=root / "src/robot_description/config/arm_rig_a1_a6.json",
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
    parser.add_argument(
        "--cycles", type=int, default=6,
        help="cycles in the checked plan/preview block; the panel repeats it until Pause unless --single-pass is set",
    )
    parser.add_argument(
        "--tracker", choices=("pid", "paced"), default="pid",
        help="PID position tracking with bounded velocity feedforward (default), or legacy feedback-paced preparation",
    )
    parser.add_argument(
        "--single-pass", action="store_true",
        help="stop and hold after the planned --cycles; by default welding repeats until Pause / Hold",
    )
    parser.add_argument(
        "--optimize-preparation",
        action="store_true",
        help="compare shorter checked staging poses within selected limits; explicitly permits them on hardware",
    )
    parser.add_argument("--preparation-planner", choices=("direct-first", "staged"),
                        default="direct-first", help="prefer a checked direct raised approach, or compare all staging candidates")
    parser.add_argument(
        "--approach-lift-mm",
        type=float,
        default=0.0,
        help="require a checked raised-tool approach at this height above the weld point (0–150 mm); 0 retains automatic route selection",
    )
    parser.add_argument(
        "--lowering-seconds",
        type=float,
        default=0.0,
        help="minimum duration of the final lowering move; requires --approach-lift-mm, leaves earlier moves unchanged (0 keeps existing timing)",
    )
    parser.add_argument(
        "--prep-seconds",
        type=float,
        default=5.0,
        help="total preparation timing goal, including hold/standby/approach; does not override limits",
    )
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
        "--max-speed-deg-s",
        type=float,
        help=f"preparation speed request: default 5 deg/s, maximum {MAX_PREPARATION_SPEED_DEG_S:g}; actual commands remain bounded by motor guards",
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
    experiment = json.loads(
        (root / "src/controls/config/weld_experiment.json").read_text()
    )
    if args.hz is None:
        args.hz = experiment["frequency_hz"] if args.profile == "weld" else 2.0
    if args.feed_mm_s is None:
        args.feed_mm_s = (
            args.hz * experiment["feed_distance_per_cycle_mm"]
            if args.profile == "weld"
            else 12.0
        )
    if args.optimize_preparation and args.audit:
        parser.error("--optimize-preparation requires planning; it is unavailable with --audit")
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
            parser.error(
                "--start-report is offline-only; hardware always uses fresh feedback"
            )
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
        pose = json.loads(args.initial_pose.read_text())
        if pose.get("mapping_fingerprint", mapping.fingerprint) != mapping.fingerprint:
            raise ValueError(
                "Initial pose was derived for a different mapping; regenerate or use a reviewed matching file"
            )
        # Validate the source pair before translating either coordinate system.
        # Targets, planning, preview and hardware all receive the same session map.
        mapping = select_a1_encoder_branch(mapping, args.a1_encoder_turns)
        args.start_raw_deg = (
            read_start_report(args.start_report, mapping) if args.start_report else None
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
            search_solution=(
                json.loads(args.solution.read_text()) if args.solution else None
            ),
            preparation_target_s=args.prep_seconds,
            optimize_preparation=args.optimize_preparation,
            approach_lift_m=args.approach_lift_mm / 1000,
            lowering_min_s=args.lowering_seconds,
            preparation_planner=args.preparation_planner,
        )
        if args.audit:
            result = audit(
                dict(
                    mapping=mapping.document,
                    initial_raw_deg=initial,
                    settings=asdict(settings),
                )
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
                    current_raw_deg=(
                        args.start_raw_deg
                        if args.start_raw_deg is not None
                        else initial
                    ),
                    initial_raw_deg=initial,
                    settings=asdict(settings),
                    arm_enabled=args.arm,
                    chassis_enabled=args.chassis,
                )
            )
            print(
                json.dumps(
                    dict(
                        initial_raw_deg=result["initial_raw_deg"],
                        preparation_timing=result["preparation_timing"],
                        weld_start_check=result["weld_start_check"],
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

            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
                [sys.argv[0]]
            )
            args.journal_path = output.with_suffix(".jsonl")
            print(f"Live journal: {args.journal_path}", flush=True)
            window = make_window(args, mapping, initial, settings, root)
            window.show()
            app.exec_()
            window.worker.wait()
            result = window.worker.report
            exit_code = 2 if window.worker.failed else 0
        result["encoder_branch_selection"] = mapping.document.get("encoder_branch_selection")
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
