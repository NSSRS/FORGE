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

    tracker_settings = TrackingSettings() if getattr(args, "tracker", "pid") == "pid" else None
    tracker_document = None if tracker_settings is None else tracker_settings.document()

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
                c.pause()
                c.message = "Window focus lost. " + c.message

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
                            c.pause()
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
                            c.pause()
                        elif action == "disable":
                            self.clear_planning()
                            c.disable_arm()
                        elif action == "reset":
                            self.reset_motor_fault()
                        elif action == "calculate":
                            self.clear_planning()
                            c.pause(preserve_ready=False)
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
                                joint_goal=c.joint_goal(),
                                hold_observation=(
                                    c.last_observation
                                    if c.state in ("ready", "complete")
                                    else None
                                ),
                                state=c.state,
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
            self.closing = False
            font = self.font()
            font.setPointSize(10)
            self.setFont(font)
            self.setWindowTitle(
                "FORGE Weld Motion Control — "
                + (
                    "Hardware / Supported Fixture"
                    if args.execute
                    else "Offline Simulation"
                )
            )
            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(6, 6, 6, 6)
            layout.setSpacing(2)
            staging_label = "optimized staging" if settings.optimize_preparation else "standby"
            info = QtWidgets.QLabel(
                f"Sequence: check measured pose → verify hold → {staging_label} → weld start → wait → weld motion\n"
                f"Requested: {settings.hz/settings.time_scale:.4f} Hz, "
                f"±{settings.amplitude_m*1000:g} mm, feed {settings.feed_m_s/settings.time_scale*1000:.3f} mm/s\n"
                f"Profile: {settings.limit_profile}; preparation speed request: {settings.max_speed_deg_s:g}°/s, bounded by motor guards. "
                "Weld speed is calculated from the selected constraints.\n"
                "Output switches do not add motion compensation. The calculated frequency is shown after planning."
            )
            info.setWordWrap(True)
            layout.addWidget(info)
            self.controller_label = QtWidgets.QLabel(
                "Tracking: PID + velocity feedforward; planned preparation timing; fixed endpoint hold."
                if tracker_settings is not None else
                "Tracking: legacy feedback-paced preparation."
            )
            self.controller_label.setWordWrap(True)
            layout.addWidget(self.controller_label)
            self.weld_run_label = QtWidgets.QLabel(
                f"Welding: one planned pass ({settings.cycles} cycles), then hold."
                if getattr(args, "single_pass", False) else
                "Welding: continuous until Pause / Hold; each checked block repeats without stopping."
            )
            self.weld_run_label.setWordWrap(True)
            layout.addWidget(self.weld_run_label)
            self.preparation_route_label = QtWidgets.QLabel(
                "Preparation route: optimized staging enabled; the checked plan may replace the saved standby."
                if settings.optimize_preparation else "Preparation route: saved standby."
            )
            self.preparation_route_label.setWordWrap(True)
            layout.addWidget(self.preparation_route_label)
            if settings.approach_lift_m:
                lift_label = QtWidgets.QLabel(
                    f"Approach: via a point {settings.approach_lift_m*1000:g} mm above the weld point, then lower to the unchanged target."
                    + (f" Final lowering: at least {settings.lowering_min_s:g} s."
                       if settings.lowering_min_s else "")
                )
                lift_label.setWordWrap(True)
                layout.addWidget(lift_label)
            branch = mapping.document.get("encoder_branch_selection")
            if branch:
                layout.addWidget(QtWidgets.QLabel(
                    f"A1 encoder branch: {branch['turns']:+d} turn "
                    f"({branch['raw_shift_deg']:+g}° raw). Verify the physical pose before recovery."
                ))
            if getattr(args, "start_report", None):
                source = QtWidgets.QLabel(
                    f"Offline start: saved feedback from {args.start_report.name}. This is not live hardware."
                )
                source.setWordWrap(True)
                layout.addWidget(source)
            self.arm = QtWidgets.QCheckBox(
                "Select arm (selection does not enable motors; existing hold is retained)"
            )
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
                ("Reset motor fault (R)", "reset"),
            ):
                button = QtWidgets.QPushButton(label)
                button.setFocusPolicy(QtCore.Qt.NoFocus)
                button.clicked.connect(
                    lambda checked=False, a=action: self.worker.send(a)
                )
                buttons.addWidget(button)
                self.buttons[action] = button
            self.buttons["initial"].setToolTip(
                "Verify current-position hold, recover to standby, then move to the weld start."
            )
            self.buttons["reset"].setEnabled(False)
            self.buttons["reset"].setToolTip(
                "Save fault evidence and reset through the connected arm owner. "
                "Requires disabled, stationary motors; success leaves motors disabled."
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
            self.preparation_label = QtWidgets.QLabel(
                f"Preparation time target: {settings.preparation_target_s:g} s (hold, standby, weld start)."
            )
            self.preparation_label.setWordWrap(True)
            layout.addWidget(self.preparation_label)
            self.tracking_label = QtWidgets.QLabel()
            self.tracking_label.setWordWrap(True)
            layout.addWidget(self.tracking_label)
            self.journal_label = QtWidgets.QLabel()
            self.journal_label.setWordWrap(True)
            layout.addWidget(self.journal_label)
            self.frequency_group = QtWidgets.QGroupBox(
                "Measured motor frequency — live encoder feedback"
                if args.execute else "Simulated motor frequency — simulated shaft feedback"
            )
            frequency_layout = QtWidgets.QVBoxLayout(self.frequency_group)
            frequency_layout.setContentsMargins(6, 6, 6, 6)
            frequency_layout.setSpacing(3)
            frequency_row = QtWidgets.QHBoxLayout()
            self.frequency_values = []
            self.frequency_details = []
            for i in range(6):
                card = QtWidgets.QFrame()
                card.setFrameShape(QtWidgets.QFrame.StyledPanel)
                card_layout = QtWidgets.QVBoxLayout(card)
                card_layout.setContentsMargins(4, 4, 4, 4)
                card_layout.setSpacing(1)
                axis = QtWidgets.QLabel(f"A{i+1}")
                axis.setAlignment(QtCore.Qt.AlignCenter)
                card_layout.addWidget(axis)
                reading = QtWidgets.QLabel("—")
                reading.setAlignment(QtCore.Qt.AlignCenter)
                reading_font = reading.font()
                reading_font.setPointSize(20)
                reading_font.setBold(True)
                reading.setFont(reading_font)
                card_layout.addWidget(reading)
                detail = QtWidgets.QLabel("Waiting for welding")
                detail.setAlignment(QtCore.Qt.AlignCenter)
                detail.setWordWrap(True)
                card_layout.addWidget(detail)
                frequency_row.addWidget(card, 1)
                self.frequency_values.append(reading)
                self.frequency_details.append(detail)
            frequency_layout.addLayout(frequency_row)
            self.frequency_status = QtWidgets.QLabel()
            self.frequency_status.setWordWrap(True)
            frequency_layout.addWidget(self.frequency_status)
            layout.addWidget(self.frequency_group)
            self.show_motor_frequency(dict(state="idle", arm_on=False))
            self.table = QtWidgets.QTableWidget(6, 6)
            self.table.setHorizontalHeaderLabels(
                [
                    "Motor",
                    "Measured raw\nangle (°)",
                    "Stage goal raw\nangle (°)",
                    "Remaining to\nstage goal (°)",
                    "Measured Hz",
                    "Measured travel\n(° peak-to-peak)",
                ]
            )
            self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.table.horizontalHeader().setSectionResizeMode(
                QtWidgets.QHeaderView.Stretch
            )
            self.table.verticalHeader().hide()
            self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
            self.table.setColumnWidth(0, 48)
            for i in range(6):
                self.table.setItem(i, 0, QtWidgets.QTableWidgetItem(f"A{i+1}"))
                self.table.setItem(i, 1, QtWidgets.QTableWidgetItem("—"))
                self.table.setItem(
                    i, 2, QtWidgets.QTableWidgetItem(f"{initial[i]:+.4f}")
                )
                self.table.setItem(i, 3, QtWidgets.QTableWidgetItem("—"))
                for column in (4, 5):
                    self.table.setItem(i, column, QtWidgets.QTableWidgetItem("—"))
            content = QtWidgets.QHBoxLayout()
            content.addWidget(self.table, 3)
            self.preview = None
            if not getattr(args, "no_preview", False):
                from simulation.weld_preview import make_preview

                self.preview = make_preview(
                    mapping, initial, simulated=not args.execute, parent=self
                )
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
            self.reset_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence("R"), self)
            self.reset_shortcut.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
            self.reset_shortcut.setAutoRepeat(False)
            self.reset_shortcut.activated.connect(self.buttons["reset"].click)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.heartbeat)
            self.timer.start(30)
            self.resize(1400, 940 if self.preview is not None else 720)
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
                    + (
                        "Nominal target met; physical time unverified."
                        if preparation["nominal_meets_target"]
                        else "Target NOT met; measured settling can take longer."
                    )
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
                self.state_label.setText(
                    f"Planning ({elapsed:.1f} s): " + value["message"]
                )
            else:
                label = "A3 inward recovery" if value["state"] == "endpoint_recovery" else value["state"]
                self.state_label.setText(label + ": " + value["message"])
            timing = value.get("timing")
            if planning:
                self.timing_label.setText(
                    "Rechecking the complete path from the current pose. "
                    "You may switch windows; calculation and preview continue without starting motion."
                )
            elif timing and timing.get("effective_hz") is not None:
                self.timing_label.setText(
                    f"Calculated: {timing['effective_hz']:.5f} Hz (planned); nominal feed {timing['feed_m_s']*1000:.4f} mm/s. "
                    "Actual motor frequencies are shown in the live readout below."
                )
            else:
                self.timing_label.setText(
                    "No accepted plan. Select outputs and calculate a path before starting recovery."
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
            motion = value.get("measured_motion")
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
