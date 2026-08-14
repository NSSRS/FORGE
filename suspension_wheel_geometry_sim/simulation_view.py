"""Interactive Tk visualization for the flat-plate kinematic simulation."""

from __future__ import annotations

import math
import time
import tkinter as tk
from dataclasses import replace

from contact import CaseResult, attach_runtime_constraints, solve_flat_pose
from robot_model import (
    RobotGeometry, RobotPose, all_wheel_centers, front_link_endpoints,
    module_arm_endpoints, module_wheel_centers, transformed_body_outline,
)
from surface_model import FlatSurface


class SimulationWindow:
    def __init__(self, robot: RobotGeometry, result: CaseResult, config: dict):
        self.config = config
        self.robot = robot
        self.initial_robot = robot
        self.result = result
        animation = config.get("animation", {})
        duration_value = animation.get("duration_seconds")
        self.duration = (
            None if duration_value is None or float(duration_value) <= 0.0
            else float(duration_value)
        )
        self.speed = float(animation.get("forward_speed_m_per_s", 0.05))
        self.fps = max(10, int(animation.get("frames_per_second", 60)))
        self.track_width = robot.body.rear_width
        self.terrain_settings = animation
        self.terrain_names = ("flat", "waves", "bumps", "weld")
        requested = str(animation.get("default_terrain", "bumps")).lower()
        self.terrain = requested if requested in self.terrain_names else "flat"
        self.bump_world_center = float(animation.get("bump_initial_position", 0.300))
        self.running = False
        self.finished = False
        self.start_time = 0.0
        self.elapsed = 0.0
        self.dynamic_pose = result.pose
        self._slider_job = None
        self.control_vars: dict[str, tk.DoubleVar] = {}

        self.root = tk.Tk()
        self.root.title("Magnetic Wall-Climbing Robot — Flat Plate Simulation")
        self.root.geometry("1500x820")
        self.root.minsize(1100, 620)
        self.root.title("Magnetic Wall-Climbing Robot — Geometry Simulation")
        self.root.configure(bg="#e2e8f0")

        main = tk.Frame(self.root, bg="#e2e8f0")
        main.pack(fill="both", expand=True, padx=12, pady=(12, 4))
        self.canvas = tk.Canvas(main, bg="#f8fafc", highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)

        controls_shell = tk.Frame(main, bg="#f1f5f9", width=330)
        controls_shell.pack(side="right", fill="y", padx=(10, 0))
        controls_shell.pack_propagate(False)
        controls_canvas = tk.Canvas(
            controls_shell, bg="#f1f5f9", highlightthickness=0, width=310
        )
        controls_scroll = tk.Scrollbar(
            controls_shell, orient="vertical", command=controls_canvas.yview
        )
        controls_canvas.configure(yscrollcommand=controls_scroll.set)
        controls_scroll.pack(side="right", fill="y")
        controls_canvas.pack(side="left", fill="both", expand=True)
        controls_frame = tk.Frame(controls_canvas, bg="#f1f5f9")
        controls_window = controls_canvas.create_window(
            (0, 0), window=controls_frame, anchor="nw"
        )
        controls_frame.bind(
            "<Configure>",
            lambda _event: controls_canvas.configure(scrollregion=controls_canvas.bbox("all")),
        )
        controls_canvas.bind(
            "<Configure>",
            lambda event: controls_canvas.itemconfigure(controls_window, width=event.width),
        )
        controls_canvas.bind(
            "<MouseWheel>",
            lambda event: controls_canvas.yview_scroll(int(-event.delta / 120), "units"),
        )
        self._create_controls(controls_frame)
        self.status = tk.StringVar(value=self._ready_text())
        tk.Label(
            self.root, textvariable=self.status, bg="#e2e8f0", fg="#0f172a",
            font=("Segoe UI", 12, "bold"), pady=8,
        ).pack(fill="x")
        self.root.bind("<space>", self._toggle_pause)
        self.root.bind("<r>", self._reset_simulation)
        self.root.bind("<R>", self._reset_simulation)
        for index in range(1, 5):
            self.root.bind(str(index), lambda _event, value=index: self._select_terrain(value))
        self.root.bind("<Escape>", lambda _event: self.root.destroy())
        self.canvas.bind("<Configure>", lambda _event: self._draw())
        self.root.after(50, self._draw)

    def _create_controls(self, parent: tk.Frame) -> None:
        tk.Label(
            parent, text="LIVE GEOMETRY", bg="#f1f5f9", fg="#0f172a",
            font=("Segoe UI", 12, "bold"), pady=10,
        ).pack(fill="x")
        body, rear, front = self.robot.body, self.robot.rear_module, self.robot.front_module
        sections = [
            ("Body and rigid link", [
                ("body_rear_width", "Triangle rear width", body.rear_width * 1000, 200, 1000, 5, "mm"),
                ("body_equal_side", "Triangle equal-side length", body.equal_side_length * 1000, 150, 1000, 5, "mm"),
                ("body_thickness", "Body thickness", body.thickness * 1000, 30, 250, 5, "mm"),
                ("front_link_length", "Front link length", body.front_pivot_extension * 1000, 0, 250, 5, "mm"),
                ("front_link_thickness", "Front link thickness", body.front_link_thickness * 1000, 5, 100, 1, "mm"),
            ]),
            ("Wheels", [
                ("wheel_radius", "Wheel radius", self.robot.wheel_radius * 1000, 20, 160, 1, "mm"),
                ("wheel_width", "Wheel tread width", self.robot.wheel_width * 1000, 5, 120, 1, "mm"),
            ]),
            ("Rear suspension ×2", [
                ("rear_rear_length", "Rearward arm length", rear.rear_arm_length * 1000, 50, 550, 5, "mm"),
                ("rear_rear_angle", "Rearward arm angle", rear.rear_arm_angle_deg, 5, 85, 1, "deg"),
                ("rear_forward_length", "Forward arm length", rear.forward_arm_length * 1000, 50, 550, 5, "mm"),
                ("rear_forward_angle", "Forward arm angle", rear.forward_arm_angle_deg, 5, 85, 1, "deg"),
                ("rear_arm_thickness", "Arm thickness", rear.arm_thickness * 1000, 5, 100, 1, "mm"),
            ]),
            ("Fixed front suspension", [
                ("front_left_length", "Left arm length", front.rear_arm_length * 1000, 50, 450, 5, "mm"),
                ("front_left_angle", "Left arm angle", front.rear_arm_angle_deg, 5, 85, 1, "deg"),
                ("front_right_length", "Right arm length", front.forward_arm_length * 1000, 50, 450, 5, "mm"),
                ("front_right_angle", "Right arm angle", front.forward_arm_angle_deg, 5, 85, 1, "deg"),
                ("front_arm_thickness", "Arm thickness", front.arm_thickness * 1000, 5, 100, 1, "mm"),
            ]),
            ("Front casters", [
                ("caster_offset_x", "Caster x offset", front.caster_offset_x * 1000, -100, 100, 1, "mm"),
                ("caster_offset_z", "Caster z offset", front.caster_offset_z * 1000, -100, 50, 1, "mm"),
                ("caster_initial_yaw", "Initial yaw", front.caster_initial_yaw_deg, -90, 90, 1, "deg"),
                ("caster_alignment", "Alignment rate", front.caster_alignment_rate, 0.2, 8.0, 0.1, "1/s"),
            ]),
        ]
        for title, sliders in sections:
            tk.Label(
                parent, text=title, anchor="w", bg="#dbe4ee", fg="#334155",
                font=("Segoe UI", 10, "bold"), padx=8, pady=5,
            ).pack(fill="x", padx=8, pady=(8, 2))
            for spec in sliders:
                self._add_slider(parent, *spec)
        tk.Button(
            parent, text="Reset geometry", command=self._reset_geometry,
            bg="#475569", fg="white", activebackground="#334155",
            activeforeground="white", relief="flat", pady=7,
        ).pack(fill="x", padx=10, pady=12)

    def _add_slider(self, parent, key, label, value, minimum, maximum, resolution, unit):
        variable = tk.DoubleVar(value=value)
        self.control_vars[key] = variable
        tk.Scale(
            parent, variable=variable, from_=minimum, to=maximum,
            resolution=resolution, orient="horizontal", label=f"{label} ({unit})",
            command=lambda _value, name=key: self._schedule_geometry_update(name),
            bg="#f1f5f9", fg="#0f172a", highlightthickness=0,
            troughcolor="#cbd5e1", activebackground="#2563eb", length=270,
            font=("Segoe UI", 9),
        ).pack(fill="x", padx=10, pady=1)

    def _schedule_geometry_update(self, _key=None) -> None:
        if self._slider_job is not None:
            self.root.after_cancel(self._slider_job)
        self._slider_job = self.root.after(35, self._apply_geometry_controls)

    def _apply_geometry_controls(self) -> None:
        self._slider_job = None

        def value(key):
            return self.control_vars[key].get()

        rear_width = value("body_rear_width") / 1000.0
        equal_side = value("body_equal_side") / 1000.0
        minimum_side = rear_width / 2.0 + 0.005
        if equal_side < minimum_side:
            equal_side = minimum_side
            self.control_vars["body_equal_side"].set(equal_side * 1000.0)
        body_length = math.sqrt(equal_side ** 2 - (rear_width / 2.0) ** 2)
        body_thickness = value("body_thickness") / 1000.0
        body = replace(
            self.robot.body,
            length=body_length,
            rear_width=rear_width,
            equal_side_length=equal_side,
            thickness=body_thickness,
            front_pivot_extension=value("front_link_length") / 1000.0,
            front_link_thickness=value("front_link_thickness") / 1000.0,
            center_of_mass=(body_length / 3.0, body_thickness / 2.0),
        )
        rear = replace(
            self.robot.rear_module,
            rear_arm_length=value("rear_rear_length") / 1000.0,
            rear_arm_angle_deg=value("rear_rear_angle"),
            forward_arm_length=value("rear_forward_length") / 1000.0,
            forward_arm_angle_deg=value("rear_forward_angle"),
            arm_thickness=value("rear_arm_thickness") / 1000.0,
        )
        front = replace(
            self.robot.front_module,
            rear_arm_length=value("front_left_length") / 1000.0,
            rear_arm_angle_deg=value("front_left_angle"),
            forward_arm_length=value("front_right_length") / 1000.0,
            forward_arm_angle_deg=value("front_right_angle"),
            arm_thickness=value("front_arm_thickness") / 1000.0,
            caster_offset_x=value("caster_offset_x") / 1000.0,
            caster_offset_z=value("caster_offset_z") / 1000.0,
            caster_initial_yaw_deg=value("caster_initial_yaw"),
            caster_alignment_rate=value("caster_alignment"),
        )
        updated = replace(
            self.robot, body=body, rear_module=rear, front_module=front,
            wheel_radius=value("wheel_radius") / 1000.0,
            wheel_width=value("wheel_width") / 1000.0,
        )
        contact = self.config["contact"]
        updated = attach_runtime_constraints(
            updated, contact["minimum_wheel_gap"], contact["minimum_wheel_body_gap"],
            self.config["robot"]["minimum_clearance"],
        )
        self.robot = updated
        self.track_width = rear_width
        try:
            solved = solve_flat_pose(updated, FlatSurface(), contact["tolerance"])
            self.result = solved
            self.dynamic_pose = solved.pose
            solve_text = (
                f"LIVE GEOMETRY — {'FEASIBLE' if solved.feasible else 'INFEASIBLE'} | "
                f"pitch {math.degrees(solved.pose.pitch):.2f}° | "
                f"body length {body_length * 1000:.1f} mm | "
                f"clearance {solved.min_body_clearance * 1000:.1f} mm"
            )
        except RuntimeError as error:
            solve_text = f"LIVE GEOMETRY — NO EXACT 6-WHEEL FLAT POSE: {error}"
        if self.running:
            solve_text += " | SPACE pauses"
        self.status.set(solve_text)
        self._draw()

    def _reset_geometry(self) -> None:
        initial = self.initial_robot
        defaults = {
            "body_rear_width": initial.body.rear_width * 1000,
            "body_equal_side": initial.body.equal_side_length * 1000,
            "body_thickness": initial.body.thickness * 1000,
            "front_link_length": initial.body.front_pivot_extension * 1000,
            "front_link_thickness": initial.body.front_link_thickness * 1000,
            "wheel_radius": initial.wheel_radius * 1000,
            "wheel_width": initial.wheel_width * 1000,
            "rear_rear_length": initial.rear_module.rear_arm_length * 1000,
            "rear_rear_angle": initial.rear_module.rear_arm_angle_deg,
            "rear_forward_length": initial.rear_module.forward_arm_length * 1000,
            "rear_forward_angle": initial.rear_module.forward_arm_angle_deg,
            "rear_arm_thickness": initial.rear_module.arm_thickness * 1000,
            "front_left_length": initial.front_module.rear_arm_length * 1000,
            "front_left_angle": initial.front_module.rear_arm_angle_deg,
            "front_right_length": initial.front_module.forward_arm_length * 1000,
            "front_right_angle": initial.front_module.forward_arm_angle_deg,
            "front_arm_thickness": initial.front_module.arm_thickness * 1000,
            "caster_offset_x": initial.front_module.caster_offset_x * 1000,
            "caster_offset_z": initial.front_module.caster_offset_z * 1000,
            "caster_initial_yaw": initial.front_module.caster_initial_yaw_deg,
            "caster_alignment": initial.front_module.caster_alignment_rate,
        }
        for key, default in defaults.items():
            self.control_vars[key].set(default)
        self._schedule_geometry_update()

    def run(self) -> None:
        self.root.mainloop()

    def _ready_text(self) -> str:
        return f"Terrain: {self.terrain.upper()} | SPACE starts/pauses | R resets | 1 Flat  2 Waves  3 Bumps  4 Weld"

    def _reset_simulation(self, _event=None) -> None:
        """Reset motion state while preserving live geometry slider values."""
        self.running = False
        self.finished = False
        self.start_time = 0.0
        self.elapsed = 0.0
        self.dynamic_pose = self.result.pose
        self.bump_world_center = float(
            self.terrain_settings.get("bump_initial_position", 0.300)
        )
        self.status.set(self._ready_text())
        self._draw()

    def _toggle_pause(self, _event=None) -> None:
        if self.running:
            self.elapsed = time.perf_counter() - self.start_time
            self.running = False
            self.status.set(
                f"PAUSED at {self.elapsed:.2f} s | SPACE resumes | "
                "1 Flat  2 Waves  3 Bumps  4 Weld"
            )
            self._draw()
            return
        if self.finished:
            self.elapsed = 0.0
            self.finished = False
            self.dynamic_pose = self.result.pose
            if self.terrain == "bumps":
                self.bump_world_center = float(
                    self.terrain_settings.get("bump_initial_position", 0.300)
                )
        self.running = True
        self.start_time = time.perf_counter() - self.elapsed
        self.status.set(
            f"RUNNING — {self.terrain.upper()} | SPACE pauses | four rear wheels driven"
        )
        self._tick()

    def _select_terrain(self, index: int) -> None:
        self.terrain = self.terrain_names[index - 1]
        if self.terrain == "bumps":
            self.bump_world_center = (
                self.speed * self.elapsed
                + float(self.terrain_settings.get("bump_initial_position", 0.300))
            )
        self.dynamic_pose = self.result.pose
        if self.running:
            self.status.set(
                f"RUNNING — {self.terrain.upper()} | SPACE pauses | four rear wheels driven"
            )
        elif self.finished:
            self.status.set(
                f"Terrain: {self.terrain.upper()} | SPACE restarts | 1 Flat  2 Waves  3 Bumps  4 Weld"
            )
        else:
            self.status.set(self._ready_text() if self.elapsed == 0 else
                            f"PAUSED at {self.elapsed:.2f} s — terrain changed to {self.terrain.upper()} | SPACE resumes")
        self._draw()

    def _tick(self) -> None:
        if not self.running:
            return
        elapsed = time.perf_counter() - self.start_time
        self.elapsed = elapsed if self.duration is None else min(elapsed, self.duration)
        self._draw()
        if self.duration is not None and self.elapsed >= self.duration:
            self.running = False
            self.finished = True
            distance = self.speed * self.duration
            self.status.set(
                f"Complete — {self.duration:.1f} s, {distance:.3f} m travelled | "
                "SPACE restarts | 1 Flat  2 Waves  3 Bumps  4 Weld"
            )
            return
        self.root.after(round(1000 / self.fps), self._tick)

    def _terrain_height(self, world_x: float) -> float:
        settings = self.terrain_settings
        if self.terrain == "waves":
            amplitude = float(settings.get("wave_amplitude", 0.020))
            wavelength = float(settings.get("wave_length", 0.600))
            return amplitude * math.sin(2.0 * math.pi * world_x / wavelength)
        if self.terrain == "weld":
            height = float(settings.get("weld_height", 0.025))
            width = max(0.001, float(settings.get("weld_width", 0.025)))
            position = float(settings.get("weld_position", 0.850))
            return height * math.exp(-0.5 * ((world_x - position) / width) ** 2)
        if self.terrain == "bumps":
            height = float(settings.get("bump_height", 0.035))
            bump_width = max(0.001, float(settings.get("bump_width", 0.300)))
            phase = world_x - self.bump_world_center
            if abs(phase) >= bump_width / 2.0:
                return 0.0
            return height * 0.5 * (1.0 + math.cos(2.0 * math.pi * phase / bump_width))
        return 0.0

    def _recycle_bump(self, travel: float, visible_left: float, visible_right: float) -> None:
        """Spawn one replacement only after the current bump fully leaves view."""
        if self.terrain != "bumps":
            return
        bump_width = max(0.001, float(self.terrain_settings.get("bump_width", 0.300)))
        local_center = self.bump_world_center - travel
        if local_center + bump_width / 2.0 < visible_left:
            margin = max(0.0, float(self.terrain_settings.get("bump_respawn_margin", 0.020)))
            self.bump_world_center = travel + visible_right + bump_width / 2.0 + margin

    def _draw_topographic_terrain(
        self, x0, y0, width, height, plan_xy, travel, local_left, local_right, scale
    ):
        """Draw terrain as elevation bands and contour lines behind the plan view."""
        c = self.canvas
        palette = ("#eef4df", "#dcebb9", "#bedb91", "#93bd6f", "#67964f")
        contour = "#526b43"
        # Coordinates covered by the plan-view inset.
        world_left = local_left + travel
        world_right = local_right + travel

        if self.terrain == "flat":
            c.create_rectangle(x0 + 2, y0 + 2, x0 + width - 2, y0 + height - 2,
                               fill=palette[0], outline="")
            return

        if self.terrain == "bumps":
            c.create_rectangle(x0 + 2, y0 + 2, x0 + width - 2, y0 + height - 2,
                               fill=palette[0], outline="")
            bump_width = max(0.001, float(self.terrain_settings.get("bump_width", 0.300)))
            lateral_width = max(
                0.001, float(self.terrain_settings.get("bump_lateral_width", 0.800))
            )
            # Outer-to-inner rings create filled topographic elevation bands.
            radii = (0.50, 0.40, 0.30, 0.20, 0.10)
            center_local = self.bump_world_center - travel
            if local_left - bump_width / 2.0 <= center_local <= local_right + bump_width / 2.0:
                cx, cy = plan_xy(center_local, 0.0)
                for band, fraction in enumerate(radii):
                    rx = bump_width * fraction * scale
                    ry = lateral_width * fraction * scale
                    c.create_oval(cx - rx, cy - ry, cx + rx, cy + ry,
                                  fill=palette[band], outline=contour, width=1)
            return

        # Waves and welds are ridges across the plate. Sample narrow vertical
        # elevation bands, then emphasize quantized contour boundaries.
        samples = 150
        previous_band = None
        amplitude = max(
            float(self.terrain_settings.get("wave_amplitude", 0.020)),
            float(self.terrain_settings.get("weld_height", 0.025)),
            1e-9,
        )
        for index in range(samples):
            local_a = local_left + (local_right - local_left) * index / samples
            local_b = local_left + (local_right - local_left) * (index + 1) / samples
            value = self._terrain_height((local_a + local_b) / 2.0 + travel)
            normalized = ((value / amplitude + 1.0) / 2.0
                          if self.terrain == "waves" else max(0.0, value / amplitude))
            band = min(4, max(0, int(5.0 * normalized)))
            xa = plan_xy(local_a, 0.0)[0]
            xb = plan_xy(local_b, 0.0)[0]
            c.create_rectangle(xa, y0 + 2, xb + 1, y0 + height - 2,
                               fill=palette[band], outline="")
            if previous_band is not None and band != previous_band:
                c.create_line(xa, y0 + 2, xa, y0 + height - 2, fill=contour, width=1)
            previous_band = band

    def _fit_pose_to_terrain(self, travel: float) -> RobotPose:
        """Quasi-static least-squares fit; preserves each module as one rigid assembly."""
        pose = self.dynamic_pose
        values = [pose.z, pose.pitch, pose.rear_module_angle, pose.front_module_angle]

        def make_pose(candidate):
            front_angle = 0.0 if self.robot.front_module.rotation_locked else candidate[3]
            return RobotPose(0.0, candidate[0], candidate[1], candidate[2], front_angle)

        def objective(candidate):
            trial = make_pose(candidate)
            wheels = all_wheel_centers(self.robot, trial)
            residuals = [
                center[1] - self.robot.wheel_radius - self._terrain_height(center[0] + travel)
                for name, center in wheels.items()
                if not name.startswith("rear_right")
            ]
            # A small pitch regularizer prevents visually implausible equivalent fits.
            return sum(value * value for value in residuals) + 1e-5 * trial.pitch * trial.pitch

        steps = [0.012, math.radians(2.0), math.radians(4.0), math.radians(4.0)]
        best = objective(values)
        for _round in range(5):
            improved = True
            while improved:
                improved = False
                variables = (0, 1, 2) if self.robot.front_module.rotation_locked else (0, 1, 2, 3)
                for variable in variables:
                    step = steps[variable]
                    for direction in (-1.0, 1.0):
                        candidate = values.copy()
                        candidate[variable] += direction * step
                        score = objective(candidate)
                        if score + 1e-12 < best:
                            values, best, improved = candidate, score, True
            steps = [step * 0.5 for step in steps]
        self.dynamic_pose = make_pose(values)
        return self.dynamic_pose

    def _draw(self) -> None:
        c = self.canvas
        c.delete("all")
        width = max(c.winfo_width(), 650)
        height = max(c.winfo_height(), 500)
        side_top, side_bottom = 55, height * 0.49
        inset_x0, inset_y0 = 35, height * 0.55
        inset_w, inset_h = width - 70, height * 0.40
        # The side view is intentionally a close-up. The top view uses its own
        # wider scale so the full track width and terrain contours remain visible.
        scale = min(
            (width - 70) / 1.10,
            (side_bottom - side_top) / 0.52,
        )
        origin_x = width * 0.42
        surface_y = side_bottom
        travel = self.speed * self.elapsed
        # Canvas y increases downward, so increasing phase is clockwise: the
        # correct rotation direction for a wheel rolling forward to the right.
        wheel_phase = travel / self.robot.wheel_radius
        pose = self._fit_pose_to_terrain(travel)
        dynamic_wheels = all_wheel_centers(self.robot, pose)

        def side_xy(point):
            # Keep the robot centered; moving plate marks convey translation.
            return origin_x + point[0] * scale, surface_y - point[1] * scale

        c.create_text(
            18, 18, anchor="nw", text="LONGITUDINAL SIDE VIEW",
            fill="#475569", font=("Segoe UI", 10, "bold")
        )
        terrain_points = []
        sample_count = max(200, width // 4)
        for index in range(sample_count + 1):
            screen_x = index * width / sample_count
            local_x = (screen_x - origin_x) / scale
            terrain_points.extend((screen_x, surface_y - self._terrain_height(local_x + travel) * scale))
        c.create_line(*terrain_points, fill="#334155", width=5, smooth=True)

        outline = transformed_body_outline(self.robot, pose)
        c.create_polygon(
            *[coordinate for p in outline for coordinate in side_xy(p)],
            fill="#cbd5e1", outline="#0f172a", width=3,
        )
        link_start, link_end = front_link_endpoints(self.robot, pose)
        c.create_line(*side_xy(link_start), *side_xy(link_end), fill="#475569",
                      width=max(4, round(self.robot.body.front_link_thickness * scale)),
                      capstyle=tk.ROUND)
        specs = [
            ((0.0, 0.0), self.robot.rear_module, pose.rear_module_angle),
            ((self.robot.body.front_pivot_x, 0.0), self.robot.front_module, pose.front_module_angle),
        ]
        for pivot_body, module, angle in specs:
            pivot = pose.transform_body_point(pivot_body)
            arm_ends = module_arm_endpoints(pose, pivot_body, module, angle)
            wheel_centers = module_wheel_centers(pose, pivot_body, module, angle)
            for endpoint in arm_ends:
                c.create_line(*side_xy(pivot), *side_xy(endpoint), fill="#2563eb",
                              width=max(4, round(module.arm_thickness * scale)), capstyle=tk.ROUND)
            for endpoint, wheel_center in zip(arm_ends, wheel_centers):
                if endpoint != wheel_center:
                    c.create_line(*side_xy(endpoint), *side_xy(wheel_center), fill="#7c3aed",
                                  width=5, capstyle=tk.ROUND)
            px, py = side_xy(pivot)
            c.create_oval(px - 7, py - 7, px + 7, py + 7, fill="#f59e0b", outline="#78350f", width=2)

        side_wheels = [
            (dynamic_wheels["rear_left_rear"], True),
            (dynamic_wheels["rear_left_forward"], True),
            (dynamic_wheels["front_rear"], False),
            (dynamic_wheels["front_forward"], False),
        ]
        for center, driven in side_wheels:
            self._wheel(side_xy(center), self.robot.wheel_radius * scale, wheel_phase, driven)

        # Plan-view inset makes the four rear driven wheels explicit.
        c.create_rectangle(inset_x0, inset_y0, inset_x0 + inset_w, inset_y0 + inset_h,
                           fill="#eef4df", outline="#94a3b8", width=2)
        plan_scale = min(inset_w / 1.35, inset_h / 0.80)
        cx, cy = inset_x0 + inset_w / 2, inset_y0 + inset_h / 2 + 8

        side_left = -origin_x / scale
        side_right = (width - origin_x) / scale
        top_left = 0.20 + (inset_x0 - cx) / plan_scale
        top_right = 0.20 + (inset_x0 + inset_w - cx) / plan_scale
        self._recycle_bump(
            travel, min(side_left, top_left), max(side_right, top_right)
        )

        def plan_xy(x, y):
            return cx + (x - 0.20) * plan_scale, cy - y * plan_scale

        local_left = 0.20 + (inset_x0 - cx) / plan_scale
        local_right = 0.20 + (inset_x0 + inset_w - cx) / plan_scale

        self._draw_topographic_terrain(
            inset_x0, inset_y0, inset_w, inset_h, plan_xy, travel,
            local_left, local_right, plan_scale,
        )
        # Restore the inset frame after terrain fills and contours.
        c.create_rectangle(inset_x0, inset_y0, inset_x0 + inset_w, inset_y0 + inset_h,
                           fill="", outline="#94a3b8", width=2)
        c.create_text(inset_x0 + 12, inset_y0 + 10, anchor="nw",
                      text="TOPOGRAPHIC TOP VIEW — GREEN WHEELS = DRIVEN",
                      fill="#334155", font=("Segoe UI", 10, "bold"))

        rear_left = plan_xy(0.0, self.track_width / 2)
        rear_right = plan_xy(0.0, -self.track_width / 2)
        nose = plan_xy(self.robot.body.length, 0.0)
        c.create_polygon(*rear_left, *rear_right, *nose, fill="#cbd5e1", outline="#0f172a", width=2)
        front_pivot_plan = plan_xy(self.robot.body.front_pivot_x, 0.0)
        c.create_line(*nose, *front_pivot_plan, fill="#475569",
                      width=max(3, round(self.robot.body.front_link_thickness * plan_scale)),
                      capstyle=tk.ROUND)
        wheel_half_length = self.robot.wheel_radius * plan_scale
        wheel_half_width = self.robot.wheel_width * plan_scale / 2.0
        for y in (self.track_width / 2, -self.track_width / 2):
            rear_vectors = self.robot.rear_module.reference_vectors
            for x_offset in (rear_vectors[0][0], rear_vectors[1][0]):
                pos = plan_xy(x_offset, y)
                self._plan_wheel(pos, wheel_half_length, wheel_half_width, driven=True)
        front_lateral = self.robot.front_module.transverse_offsets
        caster_yaw = math.radians(self.robot.front_module.caster_initial_yaw_deg)
        if self.running or self.elapsed > 0.0:
            caster_yaw *= math.exp(-self.robot.front_module.caster_alignment_rate * self.elapsed)
        front_pivot_xy = plan_xy(self.robot.body.front_pivot_x, 0.0)
        for lateral_offset in front_lateral:
            swivel = plan_xy(self.robot.body.front_pivot_x, lateral_offset)
            c.create_line(*front_pivot_xy, *swivel, fill="#2563eb",
                          width=max(4, round(self.robot.front_module.arm_thickness * plan_scale)))
            wheel_x = self.robot.body.front_pivot_x + self.robot.front_module.caster_offset_x * math.cos(caster_yaw)
            wheel_y = lateral_offset + self.robot.front_module.caster_offset_x * math.sin(caster_yaw)
            wheel_center = plan_xy(wheel_x, wheel_y)
            c.create_line(*swivel, *wheel_center, fill="#7c3aed", width=4)
            self._plan_wheel(
                wheel_center, wheel_half_length, wheel_half_width,
                driven=False, yaw=caster_yaw,
            )
        self._scale_bar(55, side_bottom - 24, scale, "100 mm")
        self._scale_bar(inset_x0 + 18, inset_y0 + inset_h - 22, plan_scale, "100 mm")
        duration_label = "infinite" if self.duration is None else f"{self.duration:.1f} s"
        c.create_text(
            width - 20, 18, anchor="ne",
            text=(f"terrain = {self.terrain}\n"
                  f"t = {self.elapsed:05.2f} / {duration_label}\n"
                  f"speed = {self.speed:.3f} m/s\ndistance = {travel:.3f} m"),
            fill="#0f172a", font=("Consolas", 12), justify="right",
        )

    def _wheel(self, center, radius, phase, driven):
        cx, cy = center
        color = "#16a34a" if driven else "#64748b"
        fill = "#bbf7d0" if driven else "#e2e8f0"
        self.canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius,
                                fill=fill, outline=color, width=3)
        for spoke in (phase, phase + math.pi / 2):
            dx, dy = math.cos(spoke) * radius * 0.78, math.sin(spoke) * radius * 0.78
            self.canvas.create_line(cx - dx, cy - dy, cx + dx, cy + dy, fill=color, width=3)
        self.canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#0f172a")
        if driven:
            self.canvas.create_text(cx, cy - radius - 13, text="DRIVE →", fill="#15803d",
                                    font=("Segoe UI", 9, "bold"))

    def _plan_wheel(self, center, half_length, half_width, driven, yaw=0.0):
        cx, cy = center
        color = "#16a34a" if driven else "#64748b"
        corners = []
        for local_x, local_y in ((-half_length, -half_width), (half_length, -half_width),
                                 (half_length, half_width), (-half_length, half_width)):
            x = cx + local_x * math.cos(yaw) - local_y * math.sin(yaw)
            y = cy + local_x * math.sin(yaw) + local_y * math.cos(yaw)
            corners.extend((x, y))
        self.canvas.create_polygon(*corners, fill="#bbf7d0" if driven else "#e2e8f0",
                                   outline=color, width=2)

    def _scale_bar(self, x, y, scale, label):
        length = 0.100 * scale
        self.canvas.create_line(x, y, x + length, y, fill="#0f172a", width=3)
        self.canvas.create_line(x, y - 4, x, y + 4, fill="#0f172a", width=2)
        self.canvas.create_line(x + length, y - 4, x + length, y + 4, fill="#0f172a", width=2)
        self.canvas.create_text(x + length + 7, y, anchor="w", text=label,
                                fill="#0f172a", font=("Segoe UI", 9, "bold"))


def show_simulation(robot: RobotGeometry, result: CaseResult, config: dict) -> None:
    SimulationWindow(robot, result, config).run()
