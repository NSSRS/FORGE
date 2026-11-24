"""Embedded kinematic preview. Rendering runs in a process with no motor owner."""

import math
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
        color = QtGui.QColor("#ff493b" if contact.get("warning", contact["blocked"]) else "#32c6ed")
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
            ("Projected through surfaces | Red: collision warning | Blue: mesh gap passes"
             if result.get("hardware_observation") else
             "Projected through surfaces | Red: blocked | Blue: mesh gap passes"),
        )
    if result.get("wire_tip_xy") is not None:
        tip = QtCore.QPointF(*result["wire_tip_xy"])
        painter.setPen(QtGui.QPen(QtGui.QColor("#182638"), 2))
        painter.setBrush(QtGui.QColor("#37efd0"))
        painter.drawEllipse(tip, 4, 4)
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
        self.weld_entry = (JointPath.from_document(plan["weld_entry"])
                           if plan.get("weld_entry") else None)
        if self.weld_entry is not None:
            self.paths["weld_entry"] = self.weld_entry
        self.endpoint = plan.get("endpoint_recovery")
        self.measured_start = plan.get("measured_start")
        # Include the same minimum stationary checks as preparation_timing.
        # Feedback pacing can still make actual execution longer.
        self.segments = [("home", 0.2, 0.0, False)]
        if self.endpoint is not None:
            path = JointPath.from_document(self.endpoint['path'])
            self.paths['endpoint_recovery'] = path
            self.segments = [('endpoint_recovery', .2, 0.0, False),
                             ('endpoint_recovery', path.duration, 0.0, True),
                             ('endpoint_recovery', .2, path.duration, False)]
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
            ]
        )
        if self.weld_entry is not None:
            self.segments.append(("weld_entry", self.weld_entry.duration, 0.0, True))
        self.segments.append(("weld", self.paths["weld"].duration, 0.0, True))
        self.duration = sum(segment[1] for segment in self.segments)
        self.feed_m_s = plan["feed_m_s"] if plan.get("chassis_enabled") else 0.0

    def chassis_travel(self, elapsed):
        weld_duration = self.paths["weld"].duration
        weld_elapsed = min(weld_duration, max(0.0, elapsed - (self.duration - weld_duration)))
        return self.feed_m_s * weld_elapsed

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
        import numpy as np
        from motors.arm_calibration import CalibrationMap
        from controls.controller.weld_plan import WeldGeometry
        from controls.simulation.weave_xy_z import WHEEL_RADIUS
        from simulation.velocity_control import DRIVEN_JOINTS, WHEEL_DIRECTIONS

        self.mj = mj
        self.geometry = WeldGeometry(CalibrationMap(mapping))
        model = self.geometry.model
        self.base_x = int(model.joint("floating_base").qposadr[0])
        self.wheel_addresses = [int(model.joint(name).qposadr[0]) for name in DRIVEN_JOINTS]
        self.wheel_roll_per_m = np.asarray(WHEEL_DIRECTIONS) / WHEEL_RADIUS
        model.vis.global_.offwidth = max(640, model.vis.global_.offwidth)
        model.vis.global_.offheight = max(400, model.vis.global_.offheight)
        self.renderer = mj.Renderer(model, width=640, height=400)
        self.option = mj.MjvOption()
        self.option.geomgroup[3:] = 0
        self.option.sitegroup[:] = 0
        self.camera = mj.MjvCamera()
        self.vertical_fov = model.vis.global_.fovy

    def observed_pose(self, raw, clearance_mm):
        """Render raw observations outside saved limits without authorizing a path.

        This fallback belongs only to the renderer. Command conversion, geometry
        guards and target/path previews retain the strict calibration checks.
        """
        import math
        from motors.arm_calibration import DRIVES, RawAngleLimitError

        geometry, mj = self.geometry, self.mj
        issues = []
        for i, (name, angle) in enumerate(zip(DRIVES, raw), 1):
            try:
                model_deg = geometry.mapping.model(i, float(angle))
            except RawAngleLimitError as exc:
                issues.append(str(exc))
                row = geometry.mapping.joints[str(i)]
                model_deg = row['model_per_raw'] * angle + row['model_intercept_deg']
            q = math.radians(model_deg)
            joint = geometry.model.joint(name).id
            if geometry.model.jnt_limited[joint] and not (
                geometry.model.jnt_range[joint, 0] <= q <= geometry.model.jnt_range[joint, 1]
            ):
                issues.append(f'A{i}: outside authored model joint range.')
            geometry.data.qpos[geometry.addresses[i - 1]] = q
        if not geometry.closure.apply(geometry.data):
            raise ValueError('D3 linkage cannot close at the measured pose; image withheld')
        mj.mj_collision(geometry.model, geometry.data)
        geometry.contact_report = geometry.inspect_contacts(clearance_mm)
        if any(row.get('warning', row['blocked']) for row in geometry.contact_report):
            issues.append('Measured pose has a model clearance warning.')
        stop = geometry.stops.check(0.1)
        if stop:
            issues.append(f'Adjacent mechanical stop: {stop}')
        return ('Measured pose shown for inspection only. ' + ' '.join(issues)
                + ' See the planning status above for the path-check result.')

    def update_view(self, data, controls):
        camera = self.camera
        yaw, pitch, zoom, *pan = controls or (0, 0, 1, 0, 0, 0)
        camera.lookat[:] += pan
        visual = self.geometry.model.vis.global_
        visual.orthographic = camera.orthographic
        visual.fovy = self.vertical_fov
        if camera.orthographic:
            # Zoom the orthographic span without moving the camera into the arm.
            visual.fovy = 2 * camera.distance * math.tan(math.radians(self.vertical_fov) / 2) * zoom
        else:
            camera.distance *= zoom
            camera.azimuth += yaw
            camera.elevation = max(-89.0, min(89.0, camera.elevation + pitch))
        self.renderer.update_scene(data, camera=camera, scene_option=self.option)

    def frame(self, request):
        import numpy as np
        from controls.controller.weld_plan import finite_vector
        from controls.simulation.weave_xy_z import SEAM_WIDTH, line
        from motors.arm_calibration import CalibrationMap, RawAngleLimitError

        geometry, mj = self.geometry, self.mj
        geometry.enforce_spatial_clearance = request.get('spatial_clearance_policy', 'enforced') != 'advisory'
        if request.get("mapping") is not None and request["mapping"] != geometry.mapping.document:
            geometry.mapping = CalibrationMap(request["mapping"])
        raw = finite_vector(request['raw'], 'preview pose')
        geometry.data.qpos[:] = geometry.seed
        issue = None
        try:
            if request.get('measured_hardware_pose') is True:
                warnings = geometry.observe(raw, clearance_mm=request['clearance_mm'])
                issue = '; '.join(warnings) or None
            elif request.get('endpoint_policy') is not None:
                geometry.check_endpoint(raw, request['endpoint_policy'])
                issue = 'A3 inward recovery only: endpoint allowance active; normal motion remains inside saved limits.'
            elif request.get('recovery_start') is not None:
                warnings = geometry.check_recovery(raw, request['recovery_start'], clearance_mm=request['clearance_mm'])
                issue = '; '.join(warnings) or None
            else:
                warnings = geometry.check(raw, clearance_mm=request["clearance_mm"])
                issue = '; '.join(warnings or []) or None
        except RawAngleLimitError:
            if request.get('measured_pose') is not True:
                raise
            issue = self.observed_pose(raw, request['clearance_mm'])
        except ValueError as exc:
            issue = str(exc)
            if not issue.startswith(
                ("Arm/environment clearance:", "Adjacent mechanical stop:")
            ):
                raise
        travel = request.get("chassis_travel_m", 0.0)
        offset = np.array([travel, 0.0, 0.0])
        if travel:
            # Render flat-floor travel after checking the arm in the planner frame.
            geometry.data.qpos[self.base_x] += travel
            geometry.data.qpos[self.wheel_addresses] += travel * self.wheel_roll_per_m
            mj.mj_kinematics(geometry.model, geometry.data)
            mj.mj_comPos(geometry.model, geometry.data)
        camera = self.camera
        view = request.get("view", "Overview")
        # Hold the start in view for the first part of the weld, then follow.
        camera.lookat[:] = [-0.06 + max(0.0, travel - 0.20), 0, 0.18]
        camera.distance, camera.azimuth, camera.elevation = 1.30, 108, -25
        camera.orthographic = view == "Top"
        if view == "Top":
            camera.distance, camera.azimuth, camera.elevation = 1.55, 90, -90
        elif view == "Side":
            camera.azimuth, camera.elevation = 90, -8
        elif view == "Wrist":
            camera.lookat[:] = geometry.data.body("l5_arm_linkage_d5").xpos
            camera.distance, camera.elevation = 0.38, -20
        contacts = [dict(row) for row in geometry.contact_report]
        if view not in ("Contact", "Contact parts"):
            # Passed mesh refinements are diagnostics, not warnings. Keep them
            # out of normal views without changing the complete clearance check.
            contacts = [row for row in contacts if row.get("warning", row["blocked"])]
        isolated = view == "Contact parts" and bool(contacts)
        if view in ("Contact", "Contact parts") and contacts:
            first = next((row for row in contacts if row.get("warning", row["blocked"])), contacts[0])
            camera.lookat[:] = first["point"] + offset
            camera.distance, camera.elevation = 0.42, -12
        if isolated:
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
                self.update_view(data, None)
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
                if len(vertices) or request.get("camera"):
                    self.update_view(data, request.get("camera"))
            finally:
                model.geom_group[:], model.geom_rgba[:], model.geom_matid[:] = (
                    groups,
                    rgba,
                    materials,
                )
            contacts = [first]
        else:
            self.update_view(geometry.data, request.get("camera"))
        from simulation.arm_sweep import BODY_LABELS

        gl_camera = mj.mjv_averageCamera(*self.renderer.scene.camera)
        view_height = gl_camera.frustum_top - gl_camera.frustum_bottom
        if not gl_camera.orthographic:
            view_height *= camera.distance / gl_camera.frustum_near
        pan_axes = (
            (view_height * np.cross(gl_camera.forward, gl_camera.up)).tolist(),
            (view_height * gl_camera.up).tolist(),
        )
        wire_tip_xy = None
        if view == "Top":
            x, y, _ = geometry.origin
            floor_z = geometry.data.geom("preview_floor").xpos[2] + 0.001
            scene = self.renderer.scene
            for light in scene.lights[:scene.nlight]:
                if light.headlight:
                    light.diffuse[:] *= 0.65
                    light.specular[:] = 0
            half_length = (travel + 1.5) / 2
            if scene.ngeom < scene.maxgeom:
                mj.mjv_initGeom(
                    scene.geoms[scene.ngeom],
                    mj.mjtGeom.mjGEOM_BOX,
                    [half_length, SEAM_WIDTH / 2, 0.0001],
                    [x + half_length, y, floor_z],
                    np.eye(3).ravel(),
                    [0.60, 0.31, 0.08, 1],
                )
                scene.ngeom += 1
            line(scene, [x, y - SEAM_WIDTH, floor_z + 0.001],
                 [x, y + SEAM_WIDTH, floor_z + 0.001], [0.60, 0.31, 0.08, 1], 3)
            wire_tip_xy = project_point(gl_camera, geometry.data.site("wire_tip").xpos, 640, 400)
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
            contact["screen_xy"] = project_point(gl_camera, contact["point"] + offset, 640, 400)
            contact["nearest_screen_xy"] = [
                project_point(gl_camera, p + offset, 640, 400)
                for p in contact.get("nearest_points", [])
            ]
        return dict(
            token=request["token"],
            pixels=self.renderer.render().tobytes(),
            issue=issue,
            hardware_observation=request.get('measured_hardware_pose') is True,
            contacts=contacts,
            isolated=isolated,
            wire_tip_xy=wire_tip_xy,
            pan_axes=pan_axes,
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

    class ModelViewport(QtWidgets.QLabel):
        dragged = QtCore.pyqtSignal(float, float, bool)
        zoomed = QtCore.pyqtSignal(float)

        def __init__(self):
            super().__init__("Loading model…")
            self.frame = None
            self.drag_button = QtCore.Qt.NoButton
            self.drag_position = QtCore.QPoint()
            self.setAlignment(QtCore.Qt.AlignCenter)
            self.setMinimumSize(300, 160)
            self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
            self.setStyleSheet("background: #182638; color: #dce6f0;")
            self.setCursor(QtCore.Qt.OpenHandCursor)

        def set_frame(self, frame):
            self.frame = QtGui.QPixmap.fromImage(frame)
            self.fit_frame()

        def fit_frame(self):
            if self.frame is not None:
                ratio = self.devicePixelRatioF()
                pixmap = self.frame.scaled(
                    self.size() * ratio, QtCore.Qt.KeepAspectRatio,
                    QtCore.Qt.SmoothTransformation,
                )
                pixmap.setDevicePixelRatio(ratio)
                self.setPixmap(pixmap)

        def resizeEvent(self, event):
            super().resizeEvent(event)
            self.fit_frame()

        def clear(self):
            self.frame = None
            super().clear()

        def mousePressEvent(self, event):
            if self.frame is not None and event.button() in (
                QtCore.Qt.LeftButton, QtCore.Qt.RightButton, QtCore.Qt.MiddleButton,
            ):
                self.drag_button = event.button()
                self.drag_position = event.pos()
                self.setCursor(QtCore.Qt.ClosedHandCursor)
                event.accept()
            else:
                super().mousePressEvent(event)

        def mouseMoveEvent(self, event):
            if not event.buttons() & self.drag_button:
                self.end_drag()
                super().mouseMoveEvent(event)
                return
            pixmap = self.pixmap()
            if pixmap is not None and not pixmap.isNull():
                delta = event.pos() - self.drag_position
                self.drag_position = event.pos()
                height = pixmap.height() / pixmap.devicePixelRatioF()
                self.dragged.emit(delta.x() / height, delta.y() / height,
                                  self.drag_button == QtCore.Qt.LeftButton)
            event.accept()

        def mouseReleaseEvent(self, event):
            if event.button() == self.drag_button:
                self.end_drag()
                event.accept()
            else:
                super().mouseReleaseEvent(event)

        def end_drag(self):
            self.drag_button = QtCore.Qt.NoButton
            self.setCursor(QtCore.Qt.OpenHandCursor)

        def wheelEvent(self, event):
            steps = event.angleDelta().y() / 120 or event.pixelDelta().y() / 40
            if self.frame is not None and steps:
                self.zoomed.emit(steps)
                event.accept()
            else:
                event.ignore()

    class StatusLabel(QtWidgets.QLabel):
        """Keep live captions from changing the viewport's allotted space."""

        def __init__(self, text="", lines=1):
            super().__init__(text)
            self.lines = lines
            self.setWordWrap(True)
            self.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)
            self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            self.setToolTip(text)

        def sizeHint(self):
            return QtCore.QSize(0, self.fontMetrics().lineSpacing() * self.lines + 4)

        def minimumSizeHint(self):
            return self.sizeHint()

        def heightForWidth(self, width):
            return self.sizeHint().height()

        def setText(self, text):
            super().setText(text)
            self.setToolTip(text)

        def paintEvent(self, event):
            # Keep the warning prefix visible and expose the complete text on
            # hover. Long contact details must not resize the model every frame.
            painter = QtGui.QPainter(self)
            painter.setPen(self.palette().color(self.foregroundRole()))
            rect = self.contentsRect()
            text = self.text().replace("\n", " ")
            layout = QtGui.QTextLayout(text, self.font())
            layout.beginLayout()
            for row in range(self.lines):
                line = layout.createLine()
                if not line.isValid():
                    break
                line.setLineWidth(rect.width())
                origin = QtCore.QPointF(rect.left(), rect.top() + row * self.fontMetrics().lineSpacing())
                if row == self.lines - 1:
                    remaining = text[line.textStart():]
                    painter.drawText(
                        origin + QtCore.QPointF(0, self.fontMetrics().ascent()),
                        self.fontMetrics().elidedText(remaining, QtCore.Qt.ElideRight, rect.width()),
                    )
                else:
                    line.draw(painter, origin)
            layout.endLayout()

    class Preview(QtWidgets.QGroupBox):
        def __init__(self):
            super().__init__("3D model / animation", parent)
            self.backend = None
            self.raw = None
            self.raw_seen = None
            self.raw_age_ms = 0.0
            self.chassis_motion = None
            self.plan = None
            self.mapping_document = mapping.document
            self.initial = list(initial)
            self.standby = list(initial)
            self.elapsed = 0.0
            self.playing = False
            self.last_tick = time.monotonic()
            self.token = 0
            self.last_request = None
            self.closed = False
            self.camera_yaw = self.camera_pitch = 0.0
            self.camera_zoom = 1.0
            self.camera_pan = (0.0, 0.0, 0.0)
            self.pan_axes = None
            layout = QtWidgets.QVBoxLayout(self)
            row = QtWidgets.QHBoxLayout()
            self.mode = QtWidgets.QComboBox()
            self.mode.addItems(["Current pose", "Standby target", "Checked path"])
            self.view = QtWidgets.QComboBox()
            self.view.addItems(
                ["Overview", "Top", "Side", "Wrist", "Contact", "Contact parts"]
            )
            row.addWidget(self.mode, 1)
            row.addWidget(self.view)
            self.reset_view = QtWidgets.QToolButton()
            self.reset_view.setText("Reset view")
            self.reset_view.setToolTip("Restore the selected camera preset")
            row.addWidget(self.reset_view)
            layout.addLayout(row)
            self.picture = ModelViewport()
            layout.addWidget(self.picture, 1)
            self.caption = StatusLabel()
            layout.addWidget(self.caption)
            self.geometry_status = StatusLabel("Waiting for a pose.", lines=2)
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
            self.camera_hint = QtWidgets.QLabel()
            self.camera_hint.setWordWrap(True)
            layout.addWidget(self.camera_hint)
            self.mode.currentIndexChanged.connect(self.change_mode)
            self.view.currentIndexChanged.connect(self.reset_camera)
            self.reset_view.clicked.connect(self.reset_camera)
            self.picture.dragged.connect(self.drag_camera)
            self.picture.zoomed.connect(self.zoom_camera)
            self.reset_camera()
            self.play.clicked.connect(self.toggle_play)
            self.scrub.sliderPressed.connect(self.pause_playback)
            self.scrub.valueChanged.connect(self.seek)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(100)
            self.camera_idle = QtCore.QTimer(self)
            self.camera_idle.setSingleShot(True)
            self.camera_idle.timeout.connect(lambda: self.timer.setInterval(100))
            try:
                self.backend = (backend_factory or PreviewProcess)(mapping.document)
            except Exception as exc:
                self.geometry_status.setText("Preview unavailable: " + str(exc))
                self.timer.stop()

        def set_pose(self, raw, age_ms=None, *, chassis_motion=None):
            self.raw = None if raw is None else list(raw)
            self.raw_seen = None if raw is None else time.monotonic()
            self.raw_age_ms = float(age_ms or 0.0)
            self.chassis_motion = chassis_motion if simulated else None

        def set_plan(self, plan):
            self.pause_playback()
            if plan and plan.get("mapping") is not None:
                self.mapping_document = plan["mapping"]
            self.standby = (
                list(plan.get("initial_raw_deg", self.initial)) if plan else list(self.initial)
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

        def set_mapping(self, document, initial):
            self.mapping_document = document
            self.initial = list(initial)
            self.set_plan(None)

        def invalidate_frame(self, *_):
            self.token += 1
            self.last_request = None
            self.picture.clear()
            self.geometry_status.setText("Updating preview…")

        def reset_camera(self, *_):
            self.camera_yaw = self.camera_pitch = 0.0
            self.camera_zoom = 1.0
            self.camera_pan = (0.0, 0.0, 0.0)
            self.pan_axes = None
            if self.view.currentText() == "Top":
                hint = "Drag to pan · Scroll to zoom · Amber seam · Cyan wire tip"
            else:
                hint = "Drag to orbit · Right-drag to pan · Scroll to zoom"
            self.camera_hint.setText(hint)
            self.picture.setToolTip(hint)
            self.invalidate_frame()

        def drag_camera(self, dx, dy, orbit):
            if orbit and self.view.currentText() != "Top":
                self.camera_yaw = (self.camera_yaw - dx * 180) % 360
                self.camera_pitch = max(-178.0, min(178.0, self.camera_pitch + dy * 180))
            elif self.pan_axes is not None:
                right, up = self.pan_axes
                self.camera_pan = tuple(
                    offset - dx * horizontal + dy * vertical
                    for offset, horizontal, vertical in zip(self.camera_pan, right, up)
                )
            self.refresh_camera()

        def zoom_camera(self, steps):
            log_zoom = math.log(self.camera_zoom) - steps * 0.20
            self.camera_zoom = math.exp(max(math.log(0.2), min(math.log(5.0), log_zoom)))
            self.refresh_camera()

        def refresh_camera(self):
            if self.timer.isActive():
                if self.timer.interval() != 33:
                    self.timer.setInterval(33)
                self.camera_idle.start(250)

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
            endpoint_policy = None
            recovery_start = None
            travel = 0.0
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
                if self.chassis_motion is not None:
                    travel = self.chassis_motion["distance_m"]
                    if raw is None:
                        raw = initial
                        source, suffix = "Saved arm pose", ""
                    speed_mm_s = self.chassis_motion["speed_m_s"] * 1000
                    suffix += f" · Chassis {speed_mm_s:.2f} mm/s · {travel * 1000:.1f} mm traveled"
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
                clearance = 2.0 if stage in ("home", "endpoint_recovery") else 0.5
                if stage == 'endpoint_recovery':
                    endpoint_policy = self.plan.endpoint['policy']
                elif stage == 'home':
                    recovery_start = self.plan.measured_start
                names = dict(
                    endpoint_recovery="A3 inward backoff (≤1°/s)",
                    home="Recovery to standby",
                    approach="Approach weld start",
                    weld="Weld motion",
                    weld_entry="Weld startup",
                )
                travel = self.plan.chassis_travel(self.elapsed)
                caption = f"Preview: {names[stage]} | {self.elapsed:.1f} / {self.plan.duration:.1f} s"
                if self.plan.feed_m_s:
                    caption += f" · Planned chassis travel {travel * 1000:.1f} mm"
                self.caption.setText(caption)
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
                        self.pan_axes = result.get("pan_axes")
                        self.picture.set_frame(annotated_frame(result))
                        issue = result["issue"]
                        message = (
                            "Model warning: " + issue
                            if issue
                            else (f"Measured pose: no model collision within {clearance:g} mm."
                                  if result.get("hardware_observation") else
                                  f"Displayed pose passes the {clearance:g} mm model check.")
                        )
                        if result.get("contacts"):
                            message += (
                                " "
                                + contact_text(result["contacts"][0])
                                + ". Select Contact for a close-up."
                            )
                        if not simulated:
                            message += " Hardware model space is advisory; it does not block motion."
                        self.geometry_status.setText(message)
                        self.geometry_status.setStyleSheet(
                            "color: #b43a20;" if issue else "color: #24704a;"
                        )
                if raw is None:
                    self.picture.clear()
                    self.geometry_status.setText(
                        "Current pose unavailable: complete valid A1-A6 feedback required."
                        if mode == "Current pose" else "No pose/path to display."
                    )
                    self.geometry_status.setStyleSheet("color: #64748b;")
                    self.last_request = None
                    return
                camera = (
                    self.camera_yaw, self.camera_pitch, self.camera_zoom, *self.camera_pan,
                )
                key = (tuple(raw), self.view.currentText(), clearance, travel, camera, self.token)
                if key != self.last_request and self.backend.submit(
                    dict(
                        raw=raw,
                        mapping=self.mapping_document,
                        spatial_clearance_policy="enforced" if simulated else "advisory",
                        view=self.view.currentText(),
                        clearance_mm=clearance,
                        token=self.token,
                        measured_pose=mode == "Current pose" and self.raw is not None,
                        measured_hardware_pose=not simulated and mode == "Current pose" and self.raw is not None,
                        endpoint_policy=endpoint_policy,
                        recovery_start=recovery_start,
                        chassis_travel_m=travel,
                        camera=camera,
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
                self.camera_idle.stop()
                if self.backend is not None:
                    self.backend.close()

    return Preview()
