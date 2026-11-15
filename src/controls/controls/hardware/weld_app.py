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
from controls.controller.weld_plan import Settings, initial_from_model, build_plan, audit
from .weld_control import WeldController


def make_window(args, mapping, initial, settings, root):
    from PyQt5 import QtCore, QtWidgets, QtGui
    from .weld_ports import ArmOwner, SimulatedArm, ChassisOwner

    # Use installed fonts, including the host's CJK fonts under WSL. No copied assets.
    database = QtGui.QFontDatabase()
    candidates = ("Noto Sans CJK SC", "Microsoft YaHei", "WenQuanYi Zen Hei")
    family = next((name for name in candidates if name in database.families()), None)
    if family is None:
        for filename in (
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/mnt/c/Windows/Fonts/msyh.ttc",
        ):
            if Path(filename).is_file():
                ident = database.addApplicationFont(filename)
                families = database.applicationFontFamilies(ident)
                if families:
                    family = families[0]
                    break

    class OwnerThread(QtCore.QThread):
        status = QtCore.pyqtSignal(object)

        def __init__(self):
            super().__init__()
            self.commands = queue.Queue(maxsize=16)
            self.cancel = threading.Event()
            self.finish = threading.Event()
            self.heartbeat = time.monotonic()
            self.controller = None
            self.pending = None
            self.request = None
            self.report = None
            self.failed = False
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
                self.commands.put_nowait((action, values))
            except queue.Full:
                self.cancel.set()

        def clear_planning(self):
            if self.pending is not None:
                self.pending.cancel()
            self.pending = None
            self.request = None

        def run(self):
            arm = chassis = None
            c = None
            try:
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
                            motion_allowed=lambda: not self.cancel.is_set()
                            and not self.finish.is_set()
                            and time.monotonic() - self.heartbeat < 0.15,
                        )
                    else:
                        arm = SimulatedArm(list(mapping.simulation_pose().values()))
                if args.chassis:
                    import yaml

                    chassis = ChassisOwner(
                        yaml.safe_load(args.chassis_profile.read_text()),
                        simulate=not args.execute,
                        execute=args.execute,
                        mapping_confirmed=args.mapping_confirmed,
                    )
                c = self.controller = WeldController(arm, chassis)
                last_display = 0.0
                while not self.finish.is_set():
                    tick_start = time.monotonic()
                    if self.cancel.is_set() or tick_start - self.heartbeat > 0.15:
                        self.clear_planning()
                        if c.state in c.ACTIVE or c.state in ("planned", "ready"):
                            c.pause()
                        self.cancel.clear()
                    try:
                        action, values = self.commands.get_nowait()
                    except queue.Empty:
                        action = None
                    try:
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
                                c.message = "Calculating initial, approach and weave paths; motors remain stationary."
                            else:
                                raise ValueError("Select arm and/or chassis first")
                        elif action == "initial":
                            c.go_initial()
                        elif action == "weld":
                            c.start_weld()
                        if self.pending is not None and self.pending.done():
                            future, self.pending = self.pending, None
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
                    c.tick()
                    if tick_start - last_display > 0.1:
                        rows = c.last_feedback
                        self.status.emit(
                            dict(
                                state=c.state,
                                message=c.message,
                                arm_on=c.arm_on,
                                chassis_on=c.chassis_on,
                                holding=c.arm_holding,
                                raw_deg=None if rows is None else rows.raw_deg.tolist(),
                                pending=self.pending is not None,
                                feed_m_s=0 if chassis is None else chassis.last_feed,
                                timing=(
                                    None
                                    if c.plan is None
                                    else {
                                        key: c.plan.get(key) for key in ("effective_hz", "feed_m_s")
                                    }
                                ),
                            )
                        )
                        last_display = tick_start
                    self.finish.wait(
                        max(0.0, settings.control_period_s - (time.monotonic() - tick_start))
                    )
            except Exception as exc:
                self.failed = True
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
                    c.close()
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
                    settings=asdict(settings),
                    mapping_fingerprint=mapping.fingerprint,
                    initial_raw_deg=list(initial),
                    events=[] if c is None else c.events,
                    accepted_plan=None if c is None else c.last_accepted_plan,
                    trajectory_trace=[] if c is None else list(c.trace),
                    trace_dropped=0 if c is None else c.trace_dropped,
                    arm_session=getattr(getattr(arm, "port", None), "report", None),
                    fault=None if c is None else c.fault,
                    hardware_qualified=False,
                )
                self.executor.shutdown(wait=False, cancel_futures=True)

    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.worker = OwnerThread()
            self.last_state = "idle"
            self.closing = False
            if family:
                self.setFont(QtGui.QFont(family, 10))
            self.setWindowTitle(
                "FORGE 焊枪运动控制 — " + ("实机／支撑夹具" if args.execute else "离线模拟")
            )
            layout = QtWidgets.QVBoxLayout(self)
            info = QtWidgets.QLabel(
                f"流程：初始姿态 → 焊接起点 → 等待启动 → 焊接运动\n"
                f"请求设定：{settings.hz/settings.time_scale:.4f} Hz，"
                f"±{settings.amplitude_m*1000:g} mm，进给 {settings.feed_m_s/settings.time_scale*1000:.3f} mm/s\n"
                f"配置：{settings.limit_profile}；到位速度上限 {settings.max_speed_deg_s:g}°/s；焊接速度由所选约束求解。\n"
                "开关只控制指令输出，不做补偿；求解后显示实际频率。"
            )
            info.setWordWrap(True)
            layout.addWidget(info)
            self.arm = QtWidgets.QCheckBox("机械臂运动（开启时保持当前位置；关闭时保留保持）")
            self.chassis = QtWidgets.QCheckBox("底盘进给")
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
                ("计算并检查路径", "calculate"),
                ("到初始姿态，再到焊接起点", "initial"),
                ("开始焊接运动", "weld"),
                ("暂停／保持（空格）", "pause"),
                ("机械臂卸力", "disable"),
            ):
                button = QtWidgets.QPushButton(label)
                button.setFocusPolicy(QtCore.Qt.NoFocus)
                button.clicked.connect(lambda checked=False, a=action: self.worker.send(a))
                buttons.addWidget(button)
                self.buttons[action] = button
            self.state_label = QtWidgets.QLabel("正在初始化独立控制线程…")
            self.state_label.setWordWrap(True)
            layout.addWidget(self.state_label)
            self.timing_label = QtWidgets.QLabel("待求解：未输出的一部分按正常执行名义轨迹处理。")
            layout.addWidget(self.timing_label)
            self.table = QtWidgets.QTableWidget(6, 3)
            self.table.setHorizontalHeaderLabels(["电机", "测得原始角度 °", "初始目标原始角度 °"])
            self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            for i in range(6):
                self.table.setItem(i, 0, QtWidgets.QTableWidgetItem(f"A{i+1}"))
                self.table.setItem(i, 1, QtWidgets.QTableWidgetItem("—"))
                self.table.setItem(i, 2, QtWidgets.QTableWidgetItem(f"{initial[i]:+.4f}"))
            layout.addWidget(self.table)
            note = QtWidgets.QLabel(
                "焊接模式仅控制运动，不控制焊机起弧。暂停会停止底盘并保持机械臂；"
                "卸力、故障或关闭窗口会撤销机械臂力矩。"
            )
            note.setWordWrap(True)
            layout.addWidget(note)
            self.worker.status.connect(self.show_status)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.heartbeat)
            self.timer.start(30)
            self.resize(1100, 500)
            self.worker.start()

        def selection(self):
            self.worker.send("select", self.arm.isChecked(), self.chassis.isChecked())

        def heartbeat(self):
            self.worker.heartbeat = time.monotonic()
            if self.closing and not self.worker.isRunning():
                self.close()

        def show_status(self, value):
            self.last_state = value["state"]
            self.state_label.setText(value["state"] + "：" + value["message"])
            timing = value.get("timing")
            if timing and timing.get("effective_hz") is not None:
                self.timing_label.setText(
                    f"求解结果：{timing['effective_hz']:.5f} Hz；名义进给 {timing['feed_m_s']*1000:.4f} mm/s"
                )
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

        def keyPressEvent(self, event):
            if event.key() in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape):
                self.worker.send("pause")
                event.accept()
            else:
                super().keyPressEvent(event)

        def changeEvent(self, event):
            if event.type() == QtCore.QEvent.ActivationChange and not self.isActiveWindow():
                self.worker.send("pause")
            super().changeEvent(event)

        def closeEvent(self, event):
            if self.worker.isRunning():
                self.closing = True
                self.worker.cancel.set()
                self.worker.finish.set()
                self.state_label.setText("正在停止底盘并确认机械臂卸力…")
                event.ignore()
            else:
                self.timer.stop()
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
    parser.add_argument(
        "--demo-initial",
        action="store_true",
        help="simulation-only interior pose; never a hardware substitute",
    )
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
    parser.add_argument("--max-speed-deg-s", type=float, default=5.0)
    parser.add_argument(
        "--solution",
        type=Path,
        help="reuse a CMA-ES candidate; revalidate fingerprints and the complete route",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
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
    else:
        if not (args.arm or args.chassis):
            args.arm = args.chassis = True
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
                    current_raw_deg=initial,
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
