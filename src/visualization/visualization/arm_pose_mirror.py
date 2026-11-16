"""Independent encoder-to-model inspection. No motor movement controls."""

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import os
import queue
from pathlib import Path
import threading
import time

from motors.arm_calibration import CalibrationMap, number


def mapped_pose(raw, document):
    """Show the affine mapping directly, including suspect branches; never wrap."""
    if len(raw) != 6:
        raise ValueError("All six raw angles are required")
    result, warnings = [], []
    for i, value in enumerate(raw, 1):
        value = number(value, f"A{i} raw angle")
        row = document["joints"][str(i)]
        gain = number(row["model_per_raw"], "gain")
        if abs(gain) < 1e-6:
            raise ValueError(f"A{i}: zero display gain is invalid")
        result.append(gain * value + number(row["model_intercept_deg"], "offset"))
        if not row["raw_branch_deg"][0] <= value <= row["raw_branch_deg"][1]:
            warnings.append(f"A{i}: outside candidate encoder branch (shown without wrapping)")
    return result, warnings


def report_sample(path):
    document = json.loads(Path(path).read_text())
    rows = document.get("feedback", [])
    if len(rows) != 6 or sorted(row["joint"] for row in rows) != list(range(1, 7)):
        raise ValueError("Report needs exactly one feedback row for A1-A6")
    rows = deepcopy(sorted(rows, key=lambda row: row["joint"]))
    for row in rows:
        radians = row.get("raw_position_rad")
        degrees = row.get("raw_position_deg")
        if radians is None:
            radians = math.radians(number(degrees, "raw_position_deg"))
        radians = number(radians, "raw_position_rad")
        if degrees is not None and not math.isclose(math.degrees(radians), number(degrees, "raw_position_deg"), abs_tol=1e-5):
            raise ValueError("Report radians/degrees disagree")
        row.update(raw_position_rad=radians, raw_position_deg=math.degrees(radians), feedback_valid=True)
    return dict(feedback=rows, source="saved-report", source_path=str(Path(path).resolve()),
                historical=True, fault=False, host_received_monotonic_s=None)


def save_snapshot(directory, packet, mapping, baseline, note):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"pose-{time.time_ns()}.json"
    sample = packet["sample"]
    raw = [row["raw_position_deg"] for row in sample["feedback"]]
    model, warnings = mapped_pose(raw, packet["view_mapping"])
    document = dict(schema="forge.arm_pose_observation", version=1,
        created_utc=datetime.now(timezone.utc).isoformat(), feedback=sample["feedback"],
        source=sample["source"], historical=sample.get("historical", False),
        source_path=sample.get("source_path"), host_received_monotonic_s=sample.get("host_received_monotonic_s"),
        source_evidence=packet.get("source_evidence"), model_deg=model,
        original_mapping_fingerprint=mapping.fingerprint, view_mapping=packet["view_mapping"],
        view_mapping_changed=packet["view_mapping"]["joints"] != mapping.document["joints"],
        baseline_raw_deg=baseline, delta_raw_deg=None if baseline is None else [q-b for q,b in zip(raw,baseline)],
        note=note, warnings=warnings, render_warning=packet.get("warning"),
        motor_targets_sent=False, production_qualified=False, calibration_anchors_changed=False)
    with path.open("x") as stream:
        json.dump(document, stream, indent=2, allow_nan=False)
    return path


def make_window(args, mapping, root, *, source_factory=None, renderer_factory=None):
    from PyQt5 import QtCore, QtGui, QtWidgets

    class Reader(QtCore.QThread):
        packet = QtCore.pyqtSignal(object)
        message = QtCore.pyqtSignal(str)
        endpoint_message = QtCore.pyqtSignal(str)
        endpoint_result = QtCore.pyqtSignal(object)

        def __init__(self):
            super().__init__()
            self.stop_event = threading.Event()
            self.settings = None
            self.failure = None
            self.capture_request = queue.Queue()
            self.endpoint_session = None

        def capture_endpoint(self, source, renderer, kind):
            from motors.arm_endpoint import A5Capture
            try:
                if source is None or not args.disabled_refresh:
                    raise ValueError("Capture requires live disabled-refresh mode")
                if self.endpoint_session is None:
                    self.endpoint_session = A5Capture(args.output_dir, mapping, source.evidence)
                reference = renderer.a5_straight_reference() if kind == "straight" else None
                self.endpoint_message.emit(f"Recording A5 {kind} — keep the arm still…")
                samples = []
                for _ in range(7):
                    if self.stop_event.is_set() or (samples and self.stop_event.wait(.1)):
                        raise ValueError("Capture canceled")
                    samples.append(source.sample())
                result = self.endpoint_session.record(kind, samples, time.monotonic(), reference=reference)
                self.endpoint_result.emit(result)
            except Exception as exc:
                self.endpoint_result.emit(dict(error=str(exc)))

        def run(self):
            source = renderer = None
            try:
                if args.port:
                    from motors.arm_observation import ArmObservation
                    from motors.arm_bench import read_arm_plan
                    plan, _ = read_arm_plan(args.arm_profile)
                    self.message.emit("Connecting to the idle controller; other serial owners are refused…")
                    source = (source_factory or ArmObservation)(plan, port=args.port,
                        refresh_disabled=args.disabled_refresh, supported=args.supported)
                self.message.emit("Loading the independent pose renderer…")
                from simulation.arm_mirror_scene import ArmMirrorScene
                renderer = (renderer_factory or ArmMirrorScene)()
                saved = report_sample(args.report) if args.report else None
                last_key = image = warning = None
                while not self.stop_event.is_set():
                    start = time.monotonic()
                    try:
                        kind = self.capture_request.get_nowait()
                    except queue.Empty:
                        kind = None
                    if kind is not None:
                        self.capture_endpoint(source, renderer, kind)
                        last_key = None
                    settings = self.settings
                    if source is not None:
                        sample = source.sample()
                    elif saved is not None:
                        sample = saved
                    else:
                        rows = [dict(joint=i, raw_position_rad=math.radians(q), raw_position_deg=q,
                                     age_ms=None, enabled=False, status=0, feedback_valid=True)
                                for i,q in enumerate(settings["offline_raw"], 1)]
                        sample = dict(feedback=rows, source="offline-sliders", historical=True,
                                      fault=False, host_received_monotonic_s=None)
                    raw = [row["raw_position_deg"] for row in sample["feedback"]]
                    try:
                        model, problems = mapped_pose(raw, settings["mapping"]) if all(q is not None for q in raw) else (None, ["Incomplete encoder feedback"])
                    except ValueError as exc:
                        model, problems = None, [str(exc)]
                    key = (tuple(raw), settings["revision"])
                    if key != last_key:
                        image, warning = None, None
                        if model is not None:
                            try:
                                image, warning = renderer.render(model, **settings["camera"])
                            except ValueError as exc:
                                warning = str(exc)
                        last_key = key
                    self.packet.emit(dict(sample=sample, model_deg=model, problems=problems,
                        pixels=image, warning=warning, view_mapping=settings["mapping"],
                        revision=settings["revision"], source_evidence=None if source is None else source.evidence))
                    self.stop_event.wait(max(0, 0.1 - (time.monotonic() - start)))
            except Exception as exc:
                self.failure = str(exc)
                self.message.emit("Stopped: " + str(exc))
            finally:
                for resource in (source, renderer):
                    if resource is not None:
                        try:
                            resource.close()
                        except Exception as exc:
                            self.failure = str(exc)
                            self.message.emit("Cleanup failed: " + str(exc))

    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.reader = Reader()
            self.last_packet = None
            self.baseline = None
            self.revision = 0
            self.closing = False
            self.editors = []
            self.view_mapping = deepcopy(mapping.document)
            self.capture_busy = False
            self.frame = None
            self.setWindowTitle("FORGE Arm Pose Mirror — Encoder / Mapping Inspection")
            layout = QtWidgets.QVBoxLayout(self)
            source_label = ("HARDWARE: disabled feedback refresh (disable frames only)" if args.disabled_refresh else
                            "HARDWARE: passive cache; disabled angles may be stale" if args.port else
                            f"SAVED REPORT: {args.report.name} (not live)" if args.report else "OFFLINE SLIDERS: no hardware")
            info = QtWidgets.QLabel(source_label + "\nModel deg = gain × raw shaft deg + offset. "
                "No automatic zero, angle wrapping, gear ratio or homing.\n"
                "Mapping edits affect this viewer only. The production mapping and motor settings are unchanged.")
            info.setWordWrap(True)
            layout.addWidget(info)
            self.status = QtWidgets.QLabel("Starting…")
            self.status.setWordWrap(True)
            layout.addWidget(self.status)
            self.table = QtWidgets.QTableWidget(6, 9)
            self.table.setHorizontalHeaderLabels(["Joint / CAN ID", "Raw rad", "Raw deg", "Delta raw deg",
                                                 "Model deg", "View gain", "View offset deg", "Age ms", "Motor state"])
            self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.table.verticalHeader().setDefaultSectionSize(32)
            self.table.setFixedHeight(6*32 + self.table.horizontalHeader().sizeHint().height() + 4)
            for i in range(6):
                for col in range(9):
                    self.table.setItem(i, col, QtWidgets.QTableWidgetItem("—"))
                self.table.item(i, 0).setText(f"A{i+1}")
                gain, offset = QtWidgets.QDoubleSpinBox(), QtWidgets.QDoubleSpinBox()
                gain.setRange(-10, 10);gain.setDecimals(6);gain.setSingleStep(1)
                offset.setRange(-1440, 1440);offset.setDecimals(6);offset.setSingleStep(1)
                row = mapping.joints[str(i+1)]
                gain.setValue(row["model_per_raw"]);offset.setValue(row["model_intercept_deg"])
                self.table.setCellWidget(i, 5, gain);self.table.setCellWidget(i, 6, offset)
                gain.valueChanged.connect(lambda value, joint=i+1: self.edit_mapping(joint, "model_per_raw", value))
                offset.valueChanged.connect(lambda value, joint=i+1: self.edit_mapping(joint, "model_intercept_deg", value))
                self.editors.append((gain, offset))
            layout.addWidget(self.table)
            toolbar = QtWidgets.QHBoxLayout()
            self.capture = QtWidgets.QPushButton("Set delta baseline")
            self.capture.clicked.connect(self.capture_baseline)
            self.reset = QtWidgets.QPushButton("Reset view mapping")
            self.reset.clicked.connect(self.reset_mapping)
            self.save = QtWidgets.QPushButton("Save snapshot")
            self.save.clicked.connect(self.save_current)
            self.capture.setEnabled(False);self.save.setEnabled(False)
            self.note = QtWidgets.QLineEdit()
            self.note.setPlaceholderText("Physical observation, e.g. A5 moved upward by hand")
            for widget in (self.capture, self.reset, self.save, self.note):toolbar.addWidget(widget)
            layout.addLayout(toolbar)
            endpoint_row = QtWidgets.QHBoxLayout()
            self.cw_capture = QtWidgets.QPushButton("Capture A5 CW limit")
            self.cw_capture.setEnabled(False)
            self.cw_capture.clicked.connect(lambda: self.request_endpoint_capture("CW"))
            self.ccw_capture = QtWidgets.QPushButton("Capture A5 CCW limit")
            self.ccw_capture.clicked.connect(lambda: self.request_endpoint_capture("CCW"))
            self.straight_capture = QtWidgets.QPushButton("Capture A5 straight pose")
            self.straight_capture.clicked.connect(lambda: self.request_endpoint_capture("straight"))
            self.endpoint_buttons = (self.cw_capture, self.ccw_capture, self.straight_capture)
            for button in self.endpoint_buttons:
                button.setEnabled(False)
                endpoint_row.addWidget(button)
            self.cw_status = QtWidgets.QLabel("Torque disabled: record both cable ends without tension, then the extended straight wrist (forearm and tool roll axes aligned). Ends set travel only; straight sets offset.")
            self.cw_status.setWordWrap(True)
            endpoint_row.addWidget(self.cw_status, 1)
            layout.addLayout(endpoint_row)
            self.picture = QtWidgets.QLabel("Loading arm model…")
            self.picture.setAlignment(QtCore.Qt.AlignCenter)
            self.picture.setMinimumSize(640, 300)
            self.picture.setStyleSheet("background: #182638; color: #dce6f0;")
            layout.addWidget(self.picture, 1)
            controls = QtWidgets.QHBoxLayout()
            self.chassis = QtWidgets.QCheckBox("Show chassis")
            self.chassis.setChecked(True)
            controls.addWidget(self.chassis)
            controls.addWidget(QtWidgets.QLabel("Focus"))
            self.focus = QtWidgets.QComboBox()
            self.focus.addItems(["Arm", "Wrist", "Robot"])
            controls.addWidget(self.focus)
            self.camera = []
            for title, low, high, value in (("Azimuth", -180, 180, 108), ("Elevation", -80, 80, -20), ("Distance m", .2, 3, .85)):
                controls.addWidget(QtWidgets.QLabel(title))
                box = QtWidgets.QDoubleSpinBox();box.setRange(low,high);box.setValue(value)
                box.setSingleStep(.1 if title == "Distance m" else 5)
                box.valueChanged.connect(self.update_settings);self.camera.append(box);controls.addWidget(box)
            self.chassis.toggled.connect(self.update_settings)
            self.focus.currentTextChanged.connect(self.change_focus)
            controls.addStretch(1)
            layout.addLayout(controls)
            self.offline_raw = []
            if not args.port and not args.report:
                controls = QtWidgets.QHBoxLayout()
                for i, q in enumerate(mapping.simulation_pose().values(), 1):
                    controls.addWidget(QtWidgets.QLabel(f"Raw A{i}"))
                    box=QtWidgets.QDoubleSpinBox();box.setRange(-720,720);box.setDecimals(4);box.setValue(q)
                    box.valueChanged.connect(self.update_settings);self.offline_raw.append(box);controls.addWidget(box)
                layout.addLayout(controls)
            self.detail = QtWidgets.QLabel("Gun included. Static posing only; contacts do not block display. D3 closure remains enforced.")
            self.detail.setWordWrap(True);layout.addWidget(self.detail)
            self.saved = QtWidgets.QLabel();self.saved.setWordWrap(True);layout.addWidget(self.saved)
            self.reader.packet.connect(self.show_packet)
            self.reader.message.connect(self.status.setText)
            self.reader.endpoint_message.connect(self.cw_status.setText)
            self.reader.endpoint_result.connect(self.endpoint_captured)
            self.timer = QtCore.QTimer(self);self.timer.timeout.connect(self.refresh_age);self.timer.start(100)
            self.update_settings()
            self.resize(1350, 900)
            self.reader.start()

        def update_settings(self, *_):
            document = deepcopy(self.view_mapping)
            # This copy is display-only and is never written to the source mapping.
            document["production_qualified"] = False
            self.revision += 1
            self.reader.settings = dict(mapping=document, revision=self.revision,
                offline_raw=[box.value() for box in self.offline_raw], camera=dict(
                    show_chassis=self.chassis.isChecked(), focus=self.focus.currentText(), azimuth=self.camera[0].value(),
                    elevation=self.camera[1].value(), distance=self.camera[2].value()))
            self.frame = None
            self.picture.clear()
            self.save.setEnabled(False)

        def change_focus(self, focus):
            self.camera[2].setValue({"Arm": .85, "Wrist": .4, "Robot": 1.8}[focus])
            self.update_settings()

        def request_endpoint_capture(self, kind):
            if self.capture_busy or self.reader.failure:return
            self.capture_busy = True
            for button in self.endpoint_buttons:button.setEnabled(False)
            self.cw_status.setText(f"Capturing A5 {kind}…")
            self.reader.capture_request.put(kind)

        def endpoint_captured(self, result):
            self.capture_busy = False
            if "error" in result:
                self.cw_status.setText("Not captured: " + result["error"])
                return
            # Partial/recaptured rounds must not keep displaying an obsolete fit.
            self.view_mapping = deepcopy(result["mapping"] or mapping.document)
            for i, (gain, offset) in enumerate(self.editors, 1):
                for box, key in ((gain, "model_per_raw"), (offset, "model_intercept_deg")):
                    box.blockSignals(True)
                    box.setValue(self.view_mapping["joints"][str(i)][key])
                    box.blockSignals(False)
            self.update_settings()
            pending = [k for k in ("CW", "CCW", "straight") if k not in result["captured"]]
            message = f"A5 {result['kind']} saved ({result['raw'][4]:+.5f}° raw). "
            if result["validation_error"]:
                message += result["validation_error"]
            elif pending:
                message += "Still needed: " + ", ".join(pending) + ". Mapping unchanged."
            else:
                message += "Candidate applied to view: straight-pose offset + cable bounds with 2° backoff. Compare poses away from straight."
            self.cw_status.setText(message)
            self.saved.setText("Saved observations: " + str(result["path"]) +
                (" | Candidate: " + str(result["path"].with_name("mapping.json")) if result["mapping"] else ""))

        def edit_mapping(self, joint, key, value):
            self.view_mapping["joints"][str(joint)][key] = value
            self.update_settings()

        def reset_mapping(self):
            self.view_mapping = deepcopy(mapping.document)
            for i, (gain, offset) in enumerate(self.editors, 1):
                for box, key in ((gain, "model_per_raw"), (offset, "model_intercept_deg")):
                    box.blockSignals(True);box.setValue(mapping.joints[str(i)][key]);box.blockSignals(False)
            self.update_settings()

        def show_packet(self, packet):
            self.last_packet = packet
            sample = packet["sample"]
            match = packet["revision"] == self.revision
            for i, row in enumerate(sample["feedback"]):
                route = f" / {row['bus']}:{row['can_id']}" if row.get("bus") is not None and row.get("can_id") is not None else " / route unverified"
                self.table.item(i, 0).setText(f"A{i+1}" + route)
                raw = row["raw_position_deg"]
                delta = None if raw is None or self.baseline is None else raw - self.baseline[i]
                model = packet["model_deg"][i] if match and packet["model_deg"] is not None else None
                for col, val in ((1,row["raw_position_rad"]),(2,raw),(3,delta),(4,model)):
                    self.table.item(i,col).setText("—" if val is None else f"{val:+.5f}")
                self.table.item(i,8).setText("ENABLED" if row.get("enabled") else f"Disabled / {row.get('status', '?')}")
            self.capture.setEnabled(all(row["raw_position_deg"] is not None for row in sample["feedback"]))
            self.save.setEnabled(self.capture.isEnabled() and match and packet["model_deg"] is not None)
            if match:
                self.frame = None if packet["pixels"] is None else QtGui.QImage(packet["pixels"],800,500,2400,QtGui.QImage.Format_RGB888).copy()
                if self.frame is None:self.picture.clear()
                self.detail.setText("; ".join(packet["problems"] + ([packet["warning"]] if packet["warning"] else [])) or
                    "Direct mapped pose. Gun included; contacts do not block this diagnostic view. No motor targets sent.")
            self.refresh_age()

        def refresh_age(self):
            if self.closing and not self.reader.isRunning():
                self.close();return
            if self.last_packet is None:return
            capture_enabled = bool(args.disabled_refresh and not self.capture_busy and
                not self.reader.failure and not self.closing and
                (self.last_packet.get("source_evidence") or {}).get("identity_verified"))
            for button in self.endpoint_buttons:button.setEnabled(capture_enabled)
            sample = self.last_packet["sample"]
            at = sample.get("host_received_monotonic_s")
            elapsed = 0 if at is None else max(0, (time.monotonic()-at)*1000)
            ages=[]
            for i,row in enumerate(sample["feedback"]):
                age = None if row.get("age_ms") is None else row["age_ms"]+elapsed
                self.table.item(i,7).setText("Saved" if at is None else "Missing" if age is None else f"{age:.0f}")
                if age is not None:ages.append(age)
            if not self.reader.failure:
                state = "Historical / offline pose — not live" if at is None else (
                    "LIVE feedback" if len(ages)==6 and max(ages)<80 else "STALE feedback — do not treat this image as the current arm pose")
                if sample.get("fault"):state = "FAULT in cached telemetry — no automatic reset"
                if at is not None and not args.disabled_refresh:state += " | Passive cache does not refresh disabled motors"
                self.status.setText(state)
            if self.frame is not None:
                self.picture.setPixmap(QtGui.QPixmap.fromImage(self.frame).scaled(
                    self.picture.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))

        def capture_baseline(self):
            if self.last_packet:
                self.baseline = [row["raw_position_deg"] for row in self.last_packet["sample"]["feedback"]]
                self.show_packet(self.last_packet)

        def save_current(self):
            if self.last_packet and self.last_packet["revision"] == self.revision:
                try:
                    path = save_snapshot(args.output_dir, self.last_packet, mapping, self.baseline, self.note.text())
                    self.saved.setText("Saved observation: " + str(path))
                except (ValueError,OSError) as exc:
                    self.saved.setText("Could not save: " + str(exc))

        def closeEvent(self,event):
            if self.reader.isRunning():
                self.closing=True;self.reader.stop_event.set();event.ignore()
            else:
                self.timer.stop();event.accept()

    return Window()


def main(argv=None):
    root=Path(__file__).resolve().parents[3]
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group()
    source.add_argument("--port",help="Exclusive idle MC02 serial port; no competing owner")
    source.add_argument("--report",type=Path,help="Inspect a saved six-axis feedback report offline")
    source.add_argument("--simulate",action="store_true",help="Offline raw-angle sliders (default)")
    parser.add_argument("--disabled-refresh",action="store_true",help="Fresh hand-position feedback using disable frames; never enables motors")
    parser.add_argument("--supported",action="store_true")
    parser.add_argument("--mapping",type=Path,default=root/"src/robot_description/config/arm_calibration_measured.json")
    parser.add_argument("--arm-profile",type=Path,default=root/"src/robot_description/config/arm_rig_a1_a6.json")
    parser.add_argument("--output-dir",type=Path,default=root/"build/arm_mirror")
    args=parser.parse_args(argv)
    if args.disabled_refresh and not (args.port and args.supported):
        parser.error("--disabled-refresh requires --port and --supported")
    if sys_platform_linux():os.environ.setdefault("MUJOCO_GL","egl")
    from PyQt5 import QtWidgets
    mapping=CalibrationMap(json.loads(args.mapping.read_text()))
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window=make_window(args,mapping,root)
    window.show();app.exec_();window.reader.wait()
    return 2 if window.reader.failure else 0


def sys_platform_linux():
    import sys
    return sys.platform.startswith("linux")
