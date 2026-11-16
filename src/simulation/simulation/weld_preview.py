"""Embedded kinematic preview. Rendering runs in a process with no motor owner."""

import multiprocessing
import os
import sys
import time


def project_point(camera, point, width, height):
    """Project into the rendered image; overlays intentionally ignore occlusion."""
    import numpy as np

    offset = np.asarray(point) - camera.pos
    depth = float(offset @ camera.forward)
    if depth <= camera.frustum_near:
        return None
    scale = 1.0 if camera.orthographic else camera.frustum_near / depth
    right = np.cross(camera.forward, camera.up)
    x, y = float(offset @ right) * scale, float(offset @ camera.up) * scale
    span = camera.frustum_top - camera.frustum_bottom
    px = width * (0.5 + (x - camera.frustum_center) / (span * width / height))
    py = height * (1.0 - (y - camera.frustum_bottom) / span)
    return [px, py] if 0 <= px < width and 0 <= py < height else None


def contact_text(contact):
    distance = contact["distance_mm"]
    if contact["basis"] == "triangle mesh":
        detail = "mesh intersection" if distance < 0 else f"mesh gap {distance:.3f} mm"
        convex = contact["convex_distance_mm"]
        detail += f"; hull {convex:+.3f} mm"
    else:
        detail = (
            f"hull overlap {-distance:.3f} mm"
            if distance < 0
            else f"gap {distance:.3f} mm"
        )
    return " / ".join(contact["labels"]) + ": " + detail


def annotated_frame(result):
    """Draw numbered screen-space rings even when contact points are hidden."""
    from PyQt5 import QtCore, QtGui

    frame = QtGui.QImage(
        result["pixels"],
        result["width"],
        result["height"],
        result["width"] * 3,
        QtGui.QImage.Format_RGB888,
    ).copy()
    painter = QtGui.QPainter(frame)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)
    painter.setFont(QtGui.QFont("Sans Serif", 9))
    contacts = result.get("contacts", [])
    if result.get("isolated") and contacts:
        contact = contacts[0]
        painter.fillRect(8, 8, frame.width() - 16, 47, QtGui.QColor(16, 28, 43, 235))
        painter.setPen(QtGui.QColor("#64c7ff"))
        painter.drawText(15, 25, "Blue: " + contact["labels"][0])
        painter.setPen(QtGui.QColor("#ffb366"))
        painter.drawText(300, 25, "Orange: " + contact["labels"][1])
        painter.setPen(QtGui.QColor("white"))
        painter.drawText(15, 45, contact_text(contact).split(": ", 1)[1])
        points = contact.get("nearest_screen_xy", [])
        if len(points) == 2 and all(point is not None for point in points):
            a, b = [QtCore.QPointF(*point) for point in points]
            painter.setPen(QtGui.QPen(QtGui.QColor("#182638"), 5))
            painter.drawLine(a, b)
            painter.setPen(QtGui.QPen(QtGui.QColor("white"), 2))
            painter.drawLine(a, b)
            painter.setBrush(QtGui.QColor("white"))
            for point in (a, b):
                painter.drawEllipse(point, 4, 4)
        painter.fillRect(
            8,
            frame.height() - 27,
            frame.width() - 16,
            20,
            QtGui.QColor(16, 28, 43, 235),
        )
        painter.setPen(QtGui.QColor("white"))
        painter.drawText(
            15,
            frame.height() - 13,
            "Only the two original meshes | White dots: nearest surface points | Model pose",
        )
        painter.end()
        return frame
    for index, contact in enumerate(contacts[:3], 1):
        color = QtGui.QColor("#ff493b" if contact["blocked"] else "#32c6ed")
        top = 8 + (index - 1) * 42
        painter.fillRect(8, top, frame.width() - 16, 39, QtGui.QColor(16, 28, 43, 235))
        painter.setPen(color)
        painter.drawText(15, top + 15, f"{index}. " + " / ".join(contact["labels"]))
        painter.setPen(QtGui.QColor("white"))
        detail = contact_text(contact).split(": ", 1)[1]
        painter.drawText(15, top + 31, detail)
        point = contact.get("screen_xy")
        if point is not None:
            center = QtCore.QPointF(*point)
            painter.setPen(QtGui.QPen(QtGui.QColor("white"), 5))
            painter.drawEllipse(center, 11, 11)
            painter.setPen(QtGui.QPen(color, 3))
            painter.drawEllipse(center, 11, 11)
            painter.drawLine(
                center + QtCore.QPointF(12, -12),
                QtCore.QPointF(frame.width() - 24, top + 20),
            )
            painter.drawText(center + QtCore.QPointF(16, 5), str(index))
    if contacts:
        painter.fillRect(
            8,
            frame.height() - 27,
            frame.width() - 16,
            20,
            QtGui.QColor(16, 28, 43, 235),
        )
        painter.setPen(QtGui.QColor("white"))
        painter.drawText(
            15,
            frame.height() - 13,
            "Projected through surfaces | Red: blocked | Blue: mesh gap passes",
        )
    painter.end()
    return frame


class PlanPlayback:
    """Sample the accepted joint paths without advancing the execution controller."""

    def __init__(self, plan):
        from controls.controller.weld_plan import JointPath

        self.paths = {
            name: JointPath.from_document(plan[name])
            for name in ("home", "approach", "weld")
        }
        # Include the same minimum stationary checks as preparation_timing.
        # Feedback pacing can still make actual execution longer.
        self.segments = [("home", 0.2, 0.0, False)]
        previous = 0.0
        for end in plan["recovery"]["waypoint_times_s"]:
            self.segments.extend(
                [("home", end - previous, previous, True), ("home", 0.2, end, False)]
            )
            previous = end
        approach = self.paths["approach"]
        self.segments.extend(
            [
                ("approach", approach.duration, 0.0, True),
                ("approach", 0.2, approach.duration, False),
                ("weld", self.paths["weld"].duration, 0.0, True),
            ]
        )
        self.duration = sum(segment[1] for segment in self.segments)

    def sample(self, elapsed):
        elapsed = max(0.0, min(float(elapsed), self.duration))
        for name, duration, offset, moving in self.segments:
            if elapsed <= duration or name == "weld":
                local = offset + min(elapsed, duration) if moving else offset
                raw, _ = self.paths[name].sample(local)
                return name, raw.tolist()
            elapsed -= duration


class PoseRenderer:
    def __init__(self, mapping):
        import mujoco as mj
        from motors.arm_calibration import CalibrationMap
        from controls.controller.weld_plan import WeldGeometry

        self.mj = mj
        self.geometry = WeldGeometry(CalibrationMap(mapping))
        model = self.geometry.model
        model.vis.global_.offwidth = max(640, model.vis.global_.offwidth)
        model.vis.global_.offheight = max(400, model.vis.global_.offheight)
        self.renderer = mj.Renderer(model, width=640, height=400)
        self.option = mj.MjvOption()
        self.option.geomgroup[3:] = 0
        self.option.sitegroup[:] = 0
        self.camera = mj.MjvCamera()

    def frame(self, request):
        geometry, mj = self.geometry, self.mj
        geometry.data.qpos[:] = geometry.seed
        # Validate the entire raw vector before changing any model coordinate.
        geometry.mapping.pose(
            {str(i): float(q) for i, q in enumerate(request["raw"], 1)}
        )
        issue = None
        try:
            geometry.check(request["raw"], clearance_mm=request["clearance_mm"])
        except ValueError as exc:
            issue = str(exc)
            if not issue.startswith(
                ("Arm/environment clearance:", "Adjacent mechanical stop:")
            ):
                raise
        camera = self.camera
        view = request.get("view", "Overview")
        camera.lookat[:] = [-0.06, 0, 0.18]
        camera.distance, camera.azimuth, camera.elevation = 1.30, 108, -25
        if view == "Side":
            camera.azimuth, camera.elevation = 90, -8
        elif view == "Wrist":
            camera.lookat[:] = geometry.data.body("l5_arm_linkage_d5").xpos
            camera.distance, camera.elevation = 0.38, -20
        contacts = [dict(row) for row in geometry.contact_report]
        if view not in ("Contact", "Contact parts"):
            # Passed mesh refinements are diagnostics, not warnings. Keep them
            # out of normal views without changing the complete clearance check.
            contacts = [row for row in contacts if row["blocked"]]
        isolated = view == "Contact parts" and bool(contacts)
        if view in ("Contact", "Contact parts") and contacts:
            first = next((row for row in contacts if row["blocked"]), contacts[0])
            camera.lookat[:] = first["point"]
            camera.distance, camera.elevation = 0.42, -12
        if isolated:
            import numpy as np

            model, data = geometry.model, geometry.data
            selected = [model.geom(name).id for name in first["geom_names"]]
            groups, rgba, materials = (
                model.geom_group.copy(),
                model.geom_rgba.copy(),
                model.geom_matid.copy(),
            )
            vertices = []
            for geom in selected:
                mesh = model.geom_dataid[geom]
                if model.geom_type[geom] == mj.mjtGeom.mjGEOM_MESH:
                    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                    vertices.extend(
                        model.mesh_vert[start : start + count]
                        @ data.geom_xmat[geom].reshape(3, 3).T
                        + data.geom_xpos[geom]
                    )
            if vertices:
                vertices = np.asarray(vertices)
                camera.lookat[:] = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
                camera.distance = max(
                    0.25, float(np.linalg.norm(np.ptp(vertices, axis=0))) * 2.1
                )
            camera.azimuth, camera.elevation = 90, -18
            try:
                # Visual groups/materials only; every collision and distance check
                # above still uses the complete robot at the unchanged pose.
                model.geom_group[:] = 5
                model.geom_group[selected] = 0
                model.geom_matid[selected] = -1
                model.geom_rgba[selected] = [
                    [0.18, 0.59, 0.95, 1],
                    [1.0, 0.48, 0.12, 1],
                ]
                self.renderer.update_scene(
                    data, camera=camera, scene_option=self.option
                )
                if len(vertices):
                    # Fit the two meshes in screen space, including room for the
                    # labels, rather than sizing from their foreshortened 3D box.
                    gl = mj.mjv_averageCamera(*self.renderer.scene.camera)
                    delta = vertices - camera.lookat
                    tangent = (gl.frustum_top - gl.frustum_bottom) / (
                        2 * gl.frustum_near
                    )
                    right = np.cross(gl.forward, gl.up)
                    required = (
                        np.maximum(
                            abs(delta @ right) / (tangent * 1.6 * 0.8),
                            abs(delta @ gl.up) / (tangent * 0.65),
                        )
                        - delta @ gl.forward
                    )
                    camera.distance = max(0.10, float(np.max(required)))
                    self.renderer.update_scene(
                        data, camera=camera, scene_option=self.option
                    )
            finally:
                model.geom_group[:], model.geom_rgba[:], model.geom_matid[:] = (
                    groups,
                    rgba,
                    materials,
                )
            contacts = [first]
        else:
            self.renderer.update_scene(
                geometry.data, camera=camera, scene_option=self.option
            )
        from simulation.arm_sweep import BODY_LABELS

        gl_camera = mj.mjv_averageCamera(*self.renderer.scene.camera)
        for contact in contacts:
            labels = []
            for name in contact["geom_names"]:
                geom = geometry.model.geom(name).id
                body = geometry.model.body(int(geometry.model.geom_bodyid[geom])).name
                labels.append(
                    "Arm mounting platform"
                    if name == "collision_139"
                    else BODY_LABELS.get(body, BODY_LABELS.get(name, name))
                )
            contact["labels"] = labels
            contact["screen_xy"] = project_point(gl_camera, contact["point"], 640, 400)
            contact["nearest_screen_xy"] = [
                project_point(gl_camera, p, 640, 400)
                for p in contact.get("nearest_points", [])
            ]
        return dict(
            token=request["token"],
            pixels=self.renderer.render().tobytes(),
            issue=issue,
            contacts=contacts,
            isolated=isolated,
            width=640,
            height=400,
        )

    def close(self):
        self.renderer.close()


def render_process(connection, mapping):
    # Choose the offscreen backend before importing MuJoCo in this child.
    if sys.platform.startswith("linux"):
        os.environ.setdefault("MUJOCO_GL", "egl")
    renderer = None
    try:
        renderer = PoseRenderer(mapping)
        while True:
            request = connection.recv()
            if request is None:
                break
            try:
                result = renderer.frame(request)
            except Exception as exc:
                result = dict(token=request["token"], error=str(exc))
            connection.send(result)
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send(dict(error="Preview renderer unavailable: " + str(exc)))
        except (EOFError, BrokenPipeError):
            pass
    finally:
        if renderer is not None:
            renderer.close()
        connection.close()


class PreviewProcess:
    """At most one frame in flight; no unbounded pose/frame queue."""

    def __init__(self, mapping):
        context = multiprocessing.get_context("spawn")
        self.connection, child = context.Pipe()
        self.process = context.Process(
            target=render_process, args=(child, mapping), daemon=True
        )
        self.busy = False
        self.process.start()
        child.close()

    def submit(self, request):
        if self.busy:
            return False
        self.connection.send(request)
        self.busy = True
        return True

    def poll(self):
        if self.connection.poll():
            self.busy = False
            return self.connection.recv()
        if not self.process.is_alive():
            raise RuntimeError(
                "Preview renderer stopped; motor controls are independent"
            )
        return None

    def close(self):
        self.connection.close()
        # This process owns only a model/renderer, never a motor transport.
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(timeout=0.02)


def make_preview(mapping, initial, *, simulated, parent=None, backend_factory=None):
    from PyQt5 import QtCore, QtGui, QtWidgets

    class Preview(QtWidgets.QGroupBox):
        def __init__(self):
            super().__init__("3D model / animation", parent)
            self.backend = None
            self.raw = None
            self.raw_seen = None
            self.raw_age_ms = 0.0
            self.plan = None
            self.standby = list(initial)
            self.elapsed = 0.0
            self.playing = False
            self.last_tick = time.monotonic()
            self.token = 0
            self.last_request = None
            self.closed = False
            layout = QtWidgets.QVBoxLayout(self)
            row = QtWidgets.QHBoxLayout()
            self.mode = QtWidgets.QComboBox()
            self.mode.addItems(["Current pose", "Standby target", "Checked path"])
            self.view = QtWidgets.QComboBox()
            self.view.addItems(
                ["Overview", "Side", "Wrist", "Contact", "Contact parts"]
            )
            row.addWidget(self.mode, 1)
            row.addWidget(self.view)
            layout.addLayout(row)
            self.picture = QtWidgets.QLabel("Loading model…")
            self.picture.setAlignment(QtCore.Qt.AlignCenter)
            self.picture.setMinimumSize(400, 250)
            self.picture.setStyleSheet("background: #182638; color: #dce6f0;")
            layout.addWidget(self.picture, 1)
            self.caption = QtWidgets.QLabel()
            self.caption.setWordWrap(True)
            layout.addWidget(self.caption)
            self.geometry_status = QtWidgets.QLabel("Waiting for a pose.")
            self.geometry_status.setWordWrap(True)
            layout.addWidget(self.geometry_status)
            row = QtWidgets.QHBoxLayout()
            self.play = QtWidgets.QPushButton("Play preview")
            self.play.setFocusPolicy(QtCore.Qt.NoFocus)
            self.play.setEnabled(False)
            self.play.setToolTip(
                "Calculate and check path first. Playback is available after the complete path passes."
            )
            self.scrub = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.scrub.setRange(0, 1000)
            self.scrub.setEnabled(False)
            self.speed = QtWidgets.QComboBox()
            self.speed.addItems(["1x", "2x", "4x"])
            row.addWidget(self.play)
            row.addWidget(self.scrub, 1)
            row.addWidget(self.speed)
            layout.addLayout(row)
            note = QtWidgets.QLabel(
                "Preview only — does not command motors. Gun included; chassis fixed. "
                "Kinematic model, not a camera or load simulation."
            )
            note.setWordWrap(True)
            layout.addWidget(note)
            self.mode.currentIndexChanged.connect(self.change_mode)
            self.view.currentIndexChanged.connect(self.invalidate_frame)
            self.play.clicked.connect(self.toggle_play)
            self.scrub.sliderPressed.connect(self.pause_playback)
            self.scrub.valueChanged.connect(self.seek)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(100)
            try:
                self.backend = (backend_factory or PreviewProcess)(mapping.document)
            except Exception as exc:
                self.geometry_status.setText("Preview unavailable: " + str(exc))
                self.timer.stop()

        def set_pose(self, raw, age_ms=None):
            self.raw = None if raw is None else list(raw)
            self.raw_seen = None if raw is None else time.monotonic()
            self.raw_age_ms = float(age_ms or 0.0)

        def set_plan(self, plan):
            self.pause_playback()
            self.standby = (
                list(plan.get("initial_raw_deg", initial)) if plan else list(initial)
            )
            self.plan = PlanPlayback(plan) if plan and plan.get("arm_enabled") else None
            self.elapsed = 0.0
            self.scrub.setValue(0)
            self.play.setEnabled(self.plan is not None)
            self.play.setToolTip(
                "Play the checked path without commanding motors."
                if self.plan is not None
                else "Calculate and check path first. Playback is available after the complete path passes."
            )
            self.scrub.setEnabled(self.plan is not None)
            self.invalidate_frame()

        def invalidate_frame(self, *_):
            self.token += 1
            self.last_request = None
            self.picture.clear()
            self.geometry_status.setText("Updating preview…")

        def change_mode(self, *_):
            self.pause_playback()
            self.invalidate_frame()

        def pause_playback(self):
            self.playing = False
            self.play.setText("Play preview")

        def toggle_play(self):
            if self.plan is None:
                return
            if self.mode.currentText() != "Checked path":
                self.mode.setCurrentText("Checked path")
            if self.elapsed >= self.plan.duration:
                self.elapsed = 0.0
            self.playing = not self.playing
            self.play.setText("Pause preview" if self.playing else "Play preview")
            self.last_tick = time.monotonic()

        def seek(self, value):
            if self.plan is not None:
                self.elapsed = value * self.plan.duration / 1000
                self.invalidate_frame()

        def tick(self):
            now = time.monotonic()
            delta, self.last_tick = min(now - self.last_tick, 0.25), now
            mode = self.mode.currentText()
            if mode == "Current pose":
                raw, clearance = self.raw, 0.5
                source = "Simulated feedback" if simulated else "Last motor feedback"
                age = (
                    None
                    if self.raw_seen is None
                    else self.raw_age_ms / 1000 + now - self.raw_seen
                )
                suffix = (
                    " — unavailable"
                    if age is None
                    else (
                        " — current fake-shaft pose"
                        if simulated
                        else f" — sample age {age:.1f} s"
                    )
                )
                if not simulated and age is not None and age >= 0.08:
                    suffix += (
                        " (historical pose; select/calculate to refresh while disabled)"
                    )
                self.caption.setText(source + suffix)
            elif mode == "Standby target":
                raw, clearance = self.standby, 2.0
                self.caption.setText("Standby target — display only")
            elif self.plan is None:
                raw, clearance = None, 2.0
                self.caption.setText(
                    "No checked path. Calculate first; a blocked plan cannot be animated."
                )
            else:
                if self.playing:
                    self.elapsed = min(
                        self.plan.duration,
                        self.elapsed + delta * float(self.speed.currentText()[:-1]),
                    )
                    if self.elapsed >= self.plan.duration:
                        self.pause_playback()
                stage, raw = self.plan.sample(self.elapsed)
                clearance = 2.0 if stage == "home" else 0.5
                names = dict(
                    home="Recovery to standby",
                    approach="Approach weld start",
                    weld="Weld motion",
                )
                self.caption.setText(
                    f"Preview: {names[stage]} | {self.elapsed:.1f} / {self.plan.duration:.1f} s"
                )
                self.scrub.blockSignals(True)
                self.scrub.setValue(round(self.elapsed / self.plan.duration * 1000))
                self.scrub.blockSignals(False)
            try:
                result = self.backend.poll()
                if (
                    result
                    and result.get("token", self.token) == self.token
                    and raw is not None
                ):
                    if "error" in result:
                        self.picture.clear()
                        self.geometry_status.setText(
                            "Preview unavailable: " + result["error"]
                        )
                    else:
                        frame = annotated_frame(result)
                        self.picture.setPixmap(
                            QtGui.QPixmap.fromImage(frame).scaled(
                                self.picture.size(),
                                QtCore.Qt.KeepAspectRatio,
                                QtCore.Qt.SmoothTransformation,
                            )
                        )
                        issue = result["issue"]
                        message = (
                            "Model warning: " + issue
                            if issue
                            else f"Displayed pose passes the {clearance:g} mm model check."
                        )
                        if result.get("contacts"):
                            message += (
                                " "
                                + contact_text(result["contacts"][0])
                                + ". Select Contact for a close-up."
                            )
                        self.geometry_status.setText(message)
                        self.geometry_status.setStyleSheet(
                            "color: #b43a20;" if issue else "color: #24704a;"
                        )
                if raw is None:
                    self.picture.clear()
                    self.geometry_status.setText("No pose/path to display.")
                    self.last_request = None
                    return
                key = (tuple(raw), self.view.currentText(), clearance, self.token)
                if key != self.last_request and self.backend.submit(
                    dict(
                        raw=raw,
                        view=self.view.currentText(),
                        clearance_mm=clearance,
                        token=self.token,
                    )
                ):
                    self.last_request = key
            except (OSError, EOFError, RuntimeError) as exc:
                self.timer.stop()
                self.picture.clear()
                self.geometry_status.setText("Preview unavailable: " + str(exc))

        def shutdown(self):
            if not self.closed:
                self.closed = True
                self.timer.stop()
                if self.backend is not None:
                    self.backend.close()

    return Preview()
