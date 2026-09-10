import sys

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QFrame,
    QLabel,
    QPushButton,
    QProgressBar,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


# Steel properties
RHO = 7850.0  # kg/m3
CP = 500.0  # J/(kg K)
K = 45.0  # W/(m K)
ALPHA = K / (RHO * CP)
AMBIENT_TEMPERATURE = 25.0  # deg C

# Magnet and empirical adhesion properties
NOMINAL_FORCE = 270.0  # N
TEMPERATURE_COEFFICIENT = 0.0011  # 1/K, reversible loss
MAGNET_MASS = 0.25  # kg
MAGNET_CP = 460.0  # J/(kg K)
MAGNET_AREA = 0.01  # m2
MAGNET_CONTACT_RADIUS = np.sqrt(MAGNET_AREA / np.pi)
ROBOT_START_Y = 0.30  # m, initial wheel-centre position on the plate
SATURATION_THICKNESS_MM = 5.0

# Numerical/model geometry constants
PLATE_WIDTH = 0.8  # m
PLATE_HEIGHT = 4.0  # m, travel direction
NX = 100
NY = 500
DT = 0.05  # s
SOURCE_RADIUS = 0.070  # m; uniform disk, matched to 2*former Gaussian sigma
TARGET_DZ = 0.002  # m, nominal through-thickness cell size
MM_PER_M = 1000.0
W_PER_KW = 1000.0


def mm_to_m(length_mm):
    return length_mm / MM_PER_M


def kw_to_w(power_kw):
    return power_kw * W_PER_KW


def robot_positions(time_s, robot_speed_m_s, wheel_distance_mm):
    # Spawn the robot well inside the plate so neither the wheel contact patch
    # nor a zero-offset heat source is clipped by the lower boundary.
    wheel_y_m = ROBOT_START_Y + robot_speed_m_s * time_s
    weld_y_m = wheel_y_m + mm_to_m(wheel_distance_mm)
    return weld_y_m, wheel_y_m


def retention_percent(magnet_temperature_c, thickness_mm):
    thickness_factor = thickness_mm / (
        thickness_mm + SATURATION_THICKNESS_MM
    )
    temperature_rise = max(magnet_temperature_c - AMBIENT_TEMPERATURE, 0.0)
    temperature_factor = max(
        0.0, 1.0 - TEMPERATURE_COEFFICIENT * temperature_rise
    )
    return thickness_factor * temperature_factor * 100.0


def contact_time_constant(h_contact, contact_area=MAGNET_AREA):
    """Lumped magnet time constant m*cp/(h*A), in seconds."""
    if h_contact <= 0.0 or contact_area <= 0.0:
        return np.inf
    return MAGNET_MASS * MAGNET_CP / (h_contact * contact_area)


def uniform_source_flux(xx, yy, center_x, center_y, dx, dy, power_w):
    """Return a uniform disk flux; off-plate source power is not concentrated."""
    source_mask = (
        (xx - center_x) ** 2 + (yy - center_y) ** 2 <= SOURCE_RADIUS**2
    )
    heat_flux = np.zeros_like(xx)
    heat_flux[source_mask] = power_w / (np.pi * SOURCE_RADIUS**2)
    return heat_flux


def temperature_at_position(surface_temperature, x_m, y_m):
    """Sample the nearest top-surface grid point, or return None off-plate."""
    if not (0.0 <= x_m <= PLATE_WIDTH and 0.0 <= y_m <= PLATE_HEIGHT):
        return None
    ix = int(round(x_m / PLATE_WIDTH * (NX - 1)))
    iy = int(round(y_m / PLATE_HEIGHT * (NY - 1)))
    return float(surface_temperature[iy, ix])


def simulate(
    robot_speed_m_s,
    heat_power_kw,
    wheel_distance_mm,
    thickness_mm,
    cooling,
    h_contact,
    sim_time,
    progress_callback=None,
    diagnostics=None,
):
    """Run the 3-D plate-conduction and lumped-magnet thermal model.

    robot_speed_m_s is in m/s. heat_power_kw is in kW; wheel_distance_mm and
    thickness_mm are in mm. Returned adhesion is percent of nominal force.
    """
    if robot_speed_m_s < 0 or heat_power_kw < 0 or wheel_distance_mm < 0:
        raise ValueError("Speed, heat input, and wheel distance cannot be negative")
    if thickness_mm <= 0 or cooling < 0 or h_contact < 0 or sim_time < 0:
        raise ValueError("Invalid thermal parameter")

    plate_thickness_m = mm_to_m(thickness_mm)
    x = np.linspace(0.0, PLATE_WIDTH, NX)
    y = np.linspace(0.0, PLATE_HEIGHT, NY)
    dx = x[1] - x[0]
    dy = y[1] - y[0]
    xx, yy = np.meshgrid(x, y)

    nz = max(2, int(np.ceil(plate_thickness_m / TARGET_DZ)))
    dz = plate_thickness_m / nz
    stability = ALPHA * DT * (
        1.0 / dx**2 + 1.0 / dy**2 + 1.0 / dz**2
    )
    if stability > 0.5:
        raise ValueError("Time step is too large for the thermal grid")

    temperature = np.full((nz, NY, NX), AMBIENT_TEMPERATURE)
    magnet_temperature = AMBIENT_TEMPERATURE
    magnet_history = []
    force_history = []
    if diagnostics is not None:
        diagnostics.clear()
        diagnostics["contact_active"] = []
        diagnostics["contact_steel_temperature"] = []
        diagnostics["weld_steel_temperature"] = []

    steps = int(np.ceil(sim_time / DT))
    heat_power_w = kw_to_w(heat_power_kw)
    contact_radius = MAGNET_CONTACT_RADIUS
    cell_heat_capacity = RHO * CP * dz * dx * dy

    for step in range(steps):
        # Use the beginning of each explicit interval; the final interval is
        # shortened when sim_time is not an exact multiple of DT.
        time = step * DT
        step_dt = min(DT, sim_time - time)
        weld_x = PLATE_WIDTH / 2.0
        weld_y, magnet_y = robot_positions(
            time, robot_speed_m_s, wheel_distance_mm
        )

        heat_flux = uniform_source_flux(
            xx, yy, weld_x, weld_y, dx, dy, heat_power_w
        )

        # Adiabatic lateral edges. Top and bottom surface losses are applied
        # separately below.
        padded = np.pad(temperature, ((0, 0), (1, 1), (1, 1)), mode="edge")
        laplacian = (
            (padded[:, 1:-1, 2:] - 2.0 * temperature + padded[:, 1:-1, :-2])
            / dx**2
            + (padded[:, 2:, 1:-1] - 2.0 * temperature + padded[:, :-2, 1:-1])
            / dy**2
        )
        through_thickness = np.zeros_like(temperature)
        through_thickness[1:-1] = (
            temperature[2:] - 2.0 * temperature[1:-1] + temperature[:-2]
        ) / dz**2
        through_thickness[0] = (temperature[1] - temperature[0]) / dz**2
        through_thickness[-1] = (temperature[-2] - temperature[-1]) / dz**2
        laplacian += through_thickness

        temperature += ALPHA * step_dt * laplacian
        temperature[0] += heat_flux * step_dt / (RHO * CP * dz)
        temperature[0] -= (
            cooling
            * (temperature[0] - AMBIENT_TEMPERATURE)
            * step_dt
            / (RHO * CP * dz)
        )
        temperature[-1] -= (
            cooling
            * (temperature[-1] - AMBIENT_TEMPERATURE)
            * step_dt
            / (RHO * CP * dz)
        )

        magnet_x = weld_x
        contact_active = (
            0.0 <= magnet_x <= PLATE_WIDTH
            and 0.0 <= magnet_y <= PLATE_HEIGHT
        )
        contact_steel_temperature = np.nan
        if contact_active:
            contact_mask = (
                (xx - magnet_x) ** 2 + (yy - magnet_y) ** 2 <= contact_radius**2
            )
            contact_cells = int(np.count_nonzero(contact_mask))
            if contact_cells == 0:
                raise RuntimeError("Contact patch is smaller than the grid")

            steel_contact_temperature = float(np.mean(temperature[0][contact_mask]))
            contact_steel_temperature = steel_contact_temperature
            represented_area = contact_cells * dx * dy
            time_constant = contact_time_constant(h_contact, represented_area)
            if np.isfinite(time_constant):
                magnet_delta = (
                    (steel_contact_temperature - magnet_temperature)
                    * step_dt
                    / time_constant
                )
            else:
                magnet_delta = 0.0
            exchanged_energy = MAGNET_MASS * MAGNET_CP * magnet_delta
            magnet_temperature += magnet_delta
            temperature[0][contact_mask] -= (
                exchanged_energy / contact_cells / cell_heat_capacity
            )

        magnet_history.append(magnet_temperature)
        force_history.append(retention_percent(magnet_temperature, thickness_mm))

        if diagnostics is not None:
            weld_steel_temperature = temperature_at_position(
                temperature[0], weld_x, weld_y
            )
            diagnostics["contact_active"].append(contact_active)
            diagnostics["contact_steel_temperature"].append(
                contact_steel_temperature
            )
            diagnostics["weld_steel_temperature"].append(
                weld_steel_temperature
                if weld_steel_temperature is not None
                else np.nan
            )

        if progress_callback is not None:
            update_interval = max(1, steps // 100)
            if (step + 1) % update_interval == 0 or step + 1 == steps:
                progress_callback(int(round((step + 1) / steps * 100.0)))

    if steps == 0 and progress_callback is not None:
        progress_callback(100)

    return temperature[0], magnet_history, force_history


class PlotCanvas(FigureCanvasQTAgg):
    def __init__(self):
        self.fig = Figure(figsize=(9, 6))
        super().__init__(self.fig)
        self.ax_heat = self.fig.add_subplot(131)
        self.ax_temp = self.fig.add_subplot(132)
        self.ax_force = self.fig.add_subplot(133)
        self.colorbar = None

    def update_plot(
        self, temperature, temp_history, force_history, contact_history=None
    ):
        self.ax_heat.clear()
        image = self.ax_heat.imshow(
            temperature,
            origin="lower",
            cmap="hot",
            aspect="auto",
            extent=(0, PLATE_WIDTH, 0, PLATE_HEIGHT),
        )
        self.ax_heat.set_title(r"Steel top surface: $T_s(x,y,t_f)$")
        self.ax_heat.set_xlabel(r"Transverse position, $x$ (m)")
        self.ax_heat.set_ylabel(r"Travel position, $y$ (m)")
        if self.colorbar is None:
            self.colorbar = self.fig.colorbar(image, ax=self.ax_heat)
            self.colorbar.set_label(r"$T_s$ (deg C)")
        else:
            self.colorbar.update_normal(image)

        time_axis = np.arange(len(temp_history)) * DT
        self.ax_temp.clear()
        self.ax_temp.plot(time_axis, temp_history)
        if contact_history is not None:
            self.ax_temp.plot(
                time_axis,
                contact_history,
                linestyle="--",
                linewidth=1.8,
                label=r"Steel under wheel, $T_{s,c}$",
            )
            self.ax_temp.lines[0].set_label(r"Magnet, $T_m$")
            self.ax_temp.legend(loc="best", fontsize=9)
        self.ax_temp.set_title(r"Magnet temperature: $T_m(t)$")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel(r"$T_m$ (deg C)")

        self.ax_force.clear()
        self.ax_force.plot(time_axis, force_history)
        self.ax_force.set_title(r"Magnetic retention: $F_{mag}(t)/F_0$")
        self.ax_force.set_xlabel("Time (s)")
        self.ax_force.set_ylabel(r"$F_{mag}/F_0$ (%)")

        self.fig.tight_layout()
        self.draw()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Forge Thermal & Adhesion Simulator")
        self.resize(1500, 1100)
        self.setStyleSheet("""
            QWidget {
                background: #f4f7fb;
                color: #172033;
                font-family: "Segoe UI";
                font-size: 15px;
            }
            QFrame#card, QFrame#controlsCard {
                background: white;
                border: 1px solid #dfe6f0;
                border-radius: 12px;
            }
            QLabel#cardTitle {
                color: #596780;
                font-size: 14px;
                font-weight: 700;
                letter-spacing: 1px;
            }
            QLabel#sectionTitle {
                color: #172033;
                font-size: 19px;
                font-weight: 700;
            }
            QSpinBox#valueInput {
                background: #eef4ff;
                color: #175cd3;
                border: 1px solid #c9dbff;
                border-radius: 7px;
                padding: 6px 8px;
                min-width: 86px;
                font-weight: 700;
                font-size: 16px;
                selection-background-color: #175cd3;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background: #dfe6f0;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #2878f0;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                width: 18px;
                margin: -6px 0;
                background: white;
                border: 3px solid #2878f0;
                border-radius: 9px;
            }
            QPushButton {
                background: #175cd3;
                color: white;
                border: none;
                border-radius: 9px;
                padding: 12px 24px;
                font-size: 17px;
                font-weight: 700;
            }
            QPushButton:hover { background: #124fb8; }
            QPushButton:pressed { background: #0e429a; }
            QPushButton:disabled { background: #98a2b3; color: #f2f4f7; }
            QProgressBar {
                background: #e4e7ec;
                border: none;
                border-radius: 7px;
                height: 14px;
                text-align: center;
                color: #344054;
                font-size: 13px;
                font-weight: 700;
            }
            QProgressBar::chunk {
                background: #2878f0;
                border-radius: 7px;
            }
        """)

        main = QVBoxLayout()
        main.setContentsMargins(18, 16, 18, 18)
        main.setSpacing(12)

        heading = QLabel("Welding Thermal & Magnetic Adhesion")
        heading.setStyleSheet("font-size: 28px; font-weight: 750; color: #101828;")
        subtitle = QLabel(
            "Moving surface heat source · 3D plate conduction · lumped magnet response"
        )
        subtitle.setStyleSheet("color: #667085; font-size: 15px;")
        main.addWidget(heading)
        main.addWidget(subtitle)

        self.canvas = PlotCanvas()
        main.addWidget(self.canvas)

        dashboard = QHBoxLayout()
        dashboard.setSpacing(12)
        input_card = QFrame()
        input_card.setObjectName("card")
        input_layout = QVBoxLayout(input_card)
        input_layout.setContentsMargins(18, 14, 18, 14)
        input_title = QLabel("SIMULATION SETUP")
        input_title.setObjectName("cardTitle")
        self.input_box = QLabel()
        self.input_box.setWordWrap(True)
        self.input_box.setTextFormat(Qt.RichText)
        self.input_box.setStyleSheet("font-size: 17px; line-height: 1.35;")
        input_layout.addWidget(input_title)
        input_layout.addWidget(self.input_box)

        result_card = QFrame()
        result_card.setObjectName("card")
        result_layout = QVBoxLayout(result_card)
        result_layout.setContentsMargins(18, 14, 18, 14)
        result_title = QLabel("THERMAL & ADHESION RESULTS")
        result_title.setObjectName("cardTitle")
        self.result_box = QLabel()
        self.result_box.setWordWrap(True)
        self.result_box.setTextFormat(Qt.RichText)
        self.result_box.setStyleSheet("font-size: 17px; line-height: 1.35;")
        result_layout.addWidget(result_title)
        result_layout.addWidget(self.result_box)

        dashboard.addWidget(input_card, 1)
        dashboard.addWidget(result_card, 1)
        main.addLayout(dashboard)

        self.sliders = {}
        parameters = [
            ("Robot Speed mm/s", 8, 0, 200, "Typical GMAW examples: 4–9 mm/s"),
            ("Heat Input kW", 9, 1, 15, "Screening range: 3–10 kW net to plate"),
            ("Wheel Distance mm", 300, 0, 1000, "Use measured torch-to-wheel geometry"),
            ("Steel Thickness mm", 20, 10, 40, "Use the actual hull plate thickness"),
            ("Cooling W/m2K", 10, 0, 15, "Natural air starting range: 5–15"),
            ("Contact Conductance W/m2K", 500, 100, 2000, "Sensitivity range only; calibrate by test"),
            ("Simulation Time s", 60, 5, 150, "Run for several τ; large offsets diffuse slowly"),
        ]

        controls_card = QFrame()
        controls_card.setObjectName("controlsCard")
        controls_layout = QVBoxLayout(controls_card)
        controls_layout.setContentsMargins(18, 14, 18, 16)
        controls_title = QLabel("Model controls")
        controls_title.setObjectName("sectionTitle")
        controls_layout.addWidget(controls_title)
        grid = QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(9)
        for row, (name, value, minimum, maximum, recommendation) in enumerate(parameters):
            label = QLabel(
                f"<b>{name}</b><br>"
                f"<span style='color:#7b879c; font-size:13px'>{recommendation}</span>"
            )
            label.setTextFormat(Qt.RichText)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(minimum, maximum)
            slider.setValue(value)
            value_input = QSpinBox()
            value_input.setObjectName("valueInput")
            value_input.setRange(minimum, maximum)
            value_input.setValue(value)
            value_input.setAlignment(Qt.AlignCenter)
            value_input.setKeyboardTracking(False)
            slider.valueChanged.connect(value_input.setValue)
            value_input.valueChanged.connect(slider.setValue)
            self.sliders[name] = slider
            grid.addWidget(label, row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(value_input, row, 2)
        controls_layout.addLayout(grid)
        main.addWidget(controls_card)

        self.run_button = QPushButton("Run simulation  →")
        self.run_button.clicked.connect(self.run)
        main.addWidget(self.run_button)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Ready")
        self.progress_bar.setTextVisible(True)
        main.addWidget(self.progress_bar)
        self.setLayout(main)

        self.input_box.setText(
            "<span style='color:#667085'>Adjust the controls and run the model.</span>"
        )
        self.result_box.setText(
            "<span style='color:#667085'>Results will appear here with separate "
            "thermal and thickness effects.</span>"
        )

    def run(self):
        speed_mm_s = self.sliders["Robot Speed mm/s"].value()
        speed_m_s = mm_to_m(speed_mm_s)
        heat = self.sliders["Heat Input kW"].value()
        distance = self.sliders["Wheel Distance mm"].value()
        thickness = self.sliders["Steel Thickness mm"].value()
        cooling = self.sliders["Cooling W/m2K"].value()
        h_contact = self.sliders["Contact Conductance W/m2K"].value()
        sim_time = self.sliders["Simulation Time s"].value()

        self.run_button.setEnabled(False)
        self.run_button.setText("Running simulation…")
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Calculating… %p%")
        QApplication.processEvents()

        try:
            diagnostics = {}
            temperature, magnet_temp, retention = simulate(
                speed_m_s,
                heat,
                distance,
                thickness,
                cooling,
                h_contact,
                sim_time,
                progress_callback=self.update_progress,
                diagnostics=diagnostics,
            )
        except Exception:
            self.progress_bar.setFormat("Simulation failed")
            self.run_button.setText("Run simulation  →")
            self.run_button.setEnabled(True)
            raise
        contact_history = diagnostics["contact_steel_temperature"]
        self.canvas.update_plot(
            temperature, magnet_temp, retention, contact_history
        )

        steps = int(np.ceil(sim_time / DT))
        final_source_time = (steps - 1) * DT if steps else 0.0
        final_weld_y, _ = robot_positions(final_source_time, speed_m_s, distance)
        weld_temperature = temperature_at_position(
            temperature, PLATE_WIDTH / 2.0, final_weld_y
        )
        weld_temperature_text = (
            f"{weld_temperature:.1f} deg C"
            if weld_temperature is not None
            else "outside modeled plate"
        )
        nominal_time_constant = contact_time_constant(h_contact)
        time_constant_text = (
            f"{nominal_time_constant:.1f} s"
            if np.isfinite(nominal_time_constant)
            else "infinite (no thermal contact)"
        )

        peak_temperature = max(magnet_temp, default=AMBIENT_TEMPERATURE)
        peak_surface_temperature = float(np.max(temperature))
        thickness_baseline = (
            thickness / (thickness + SATURATION_THICKNESS_MM) * 100.0
        )
        final_retention = retention[-1] if retention else (
            thickness_baseline
        )
        thermal_loss_points = max(thickness_baseline - final_retention, 0.0)
        predicted_force = NOMINAL_FORCE * final_retention / 100.0
        offset_diffusion_time = mm_to_m(distance) ** 2 / ALPHA
        active_contact_steps = int(np.count_nonzero(diagnostics["contact_active"]))
        finite_contact_temperatures = np.asarray(contact_history, dtype=float)
        finite_contact_temperatures = finite_contact_temperatures[
            np.isfinite(finite_contact_temperatures)
        ]
        if finite_contact_temperatures.size:
            peak_contact_temperature = float(np.max(finite_contact_temperatures))
            final_contact_temperature = float(finite_contact_temperatures[-1])
        else:
            peak_contact_temperature = np.nan
            final_contact_temperature = np.nan
        status = "WARNING" if peak_temperature > 80.0 else "SAFE"
        status_color = "#b42318" if status == "WARNING" else "#067647"
        status_background = "#fef3f2" if status == "WARNING" else "#ecfdf3"

        self.input_box.setText(
            f"""
            <table cellspacing='7' cellpadding='2'>
              <tr><td width='275' style='color:#667085'>Robot speed</td>
                  <td align='left'><b>{speed_mm_s} mm/s</b> &nbsp;({speed_m_s:.3f} m/s)</td></tr>
              <tr><td style='color:#667085'>Net plate power</td>
                  <td align='left'><b>{heat} kW</b></td></tr>
              <tr><td style='color:#667085'>Torch → wheel offset</td>
                  <td align='left'><b>{distance} mm</b></td></tr>
              <tr><td style='color:#667085'>Initial wheel centre, y<sub>m,0</sub></td>
                  <td align='left'><b>{ROBOT_START_Y:.3f} m</b> · fully on plate</td></tr>
              <tr><td style='color:#667085'>Offset diffusion scale, d²/α</td>
                  <td align='left'><b>{offset_diffusion_time:.0f} s</b></td></tr>
              <tr><td style='color:#667085'>Wheel contact state</td>
                  <td align='left'><b>{active_contact_steps}/{len(magnet_temp)} steps active</b></td></tr>
              <tr><td style='color:#667085'>Steel thickness</td>
                  <td align='left'><b>{thickness} mm</b></td></tr>
              <tr><td style='color:#667085'>Surface cooling, h<sub>c</sub></td>
                  <td align='left'><b>{cooling} W/(m² K)</b></td></tr>
              <tr><td style='color:#667085'>Contact conductance, h<sub>contact</sub></td>
                  <td align='left'><b>{h_contact} W/(m² K)</b></td></tr>
              <tr><td style='color:#667085'>Magnet time constant, τ</td>
                  <td align='left'><b>{time_constant_text}</b></td></tr>
              <tr><td style='color:#667085'>Final torch position, y<sub>w</sub></td>
                  <td align='left'><b>{final_weld_y:.3f} m</b></td></tr>
              <tr><td style='color:#667085'>Local weld temperature, T<sub>s,w</sub></td>
                  <td align='left'><b>{weld_temperature_text}</b></td></tr>
            </table>"""
        )
        self.result_box.setText(
            f"""
            <table cellspacing='7' cellpadding='2'>
              <tr>
                <td width='275' style='color:#667085'>Peak steel surface</td>
                <td align='left'><span style='font-size:28px; font-weight:700'>{peak_surface_temperature:.1f} °C</span></td>
              </tr>
              <tr>
                <td style='color:#667085'>Peak magnet, T<sub>m,max</sub></td>
                <td align='left'><span style='font-size:28px; font-weight:700'>{peak_temperature:.1f} °C</span></td>
              </tr>
              <tr><td style='color:#667085'>Peak steel under wheel, T<sub>s,c,max</sub></td>
                  <td align='left'><b>{peak_contact_temperature:.2f} °C</b></td></tr>
              <tr><td style='color:#667085'>Final steel under wheel, T<sub>s,c,f</sub></td>
                  <td align='left'><b>{final_contact_temperature:.2f} °C</b></td></tr>
              <tr><td colspan='2'><hr style='color:#e4e7ec'></td></tr>
              <tr><td style='color:#667085'>Thickness-only baseline</td>
                  <td align='left'><b>{thickness_baseline:.1f}%</b></td></tr>
              <tr><td style='color:#667085'>Additional loss from heat</td>
                  <td align='left'><b>−{thermal_loss_points:.2f} points</b></td></tr>
              <tr>
                <td style='color:#667085'>Final magnetic retention</td>
                <td align='left'><span style='font-size:30px; color:#175cd3; font-weight:750'>{final_retention:.1f}%</span></td>
              </tr>
              <tr><td style='color:#667085'>Predicted holding force</td>
                  <td align='left'><b>{predicted_force:.1f} N</b> / {NOMINAL_FORCE:.0f} N nominal</td></tr>
              <tr><td colspan='2' style='padding-top:8px'>
                <span style='background:{status_background}; color:{status_color}; font-weight:700'>
                &nbsp; {status} · 80 °C magnet threshold &nbsp;</span>
              </td></tr>
            </table>"""
        )

        self.progress_bar.setValue(100)
        self.progress_bar.setFormat("Simulation complete · 100%")
        self.run_button.setText("Run simulation again  →")
        self.run_button.setEnabled(True)

    def update_progress(self, percent):
        self.progress_bar.setValue(percent)
        QApplication.processEvents()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
