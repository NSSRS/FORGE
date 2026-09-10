"""船体焊接机器人热传导与磁吸附仿真器（简体中文版）。"""

import sys

import matplotlib as mpl
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from mag_thermal_thickness_sim.heat_visualizer import (
    ALPHA,
    AMBIENT_TEMPERATURE,
    DT,
    MAGNET_CONTACT_RADIUS,
    NOMINAL_FORCE,
    PLATE_HEIGHT,
    PLATE_WIDTH,
    ROBOT_START_Y,
    SATURATION_THICKNESS_MM,
    contact_time_constant,
    mm_to_m,
    robot_positions,
    simulate,
    temperature_at_position,
)


# 优先使用常见中文字体；若系统缺少这些字体，则回退到默认字体。
mpl.rcParams["font.sans-serif"] = [
    "Microsoft YaHei",
    "SimHei",
    "Noto Sans CJK SC",
    "DejaVu Sans",
]
mpl.rcParams["axes.unicode_minus"] = False


class ChinesePlotCanvas(FigureCanvasQTAgg):
    """显示钢板温度场、磁体温度和磁吸力保持率。"""

    def __init__(self):
        self.fig = Figure(figsize=(9, 6))
        super().__init__(self.fig)
        self.plate_ax = self.fig.add_subplot(131)
        self.magnet_ax = self.fig.add_subplot(132)
        self.force_ax = self.fig.add_subplot(133)
        self.colorbar = None

    def update_plot(self, surface_temperature, magnet_history, retention_history, contact_history=None):
        self.plate_ax.clear()
        image = self.plate_ax.imshow(
            surface_temperature,
            origin="lower",
            cmap="hot",
            aspect="auto",
            extent=(0, PLATE_WIDTH, 0, PLATE_HEIGHT),
        )
        self.plate_ax.set_title(r"钢板上表面温度：$T_s(x,y,t_f)$")
        self.plate_ax.set_xlabel(r"横向位置 $x$（米）")
        self.plate_ax.set_ylabel(r"机器人行进位置 $y$（米）")
        if self.colorbar is None:
            self.colorbar = self.fig.colorbar(image, ax=self.plate_ax)
            self.colorbar.set_label(r"$T_s$（摄氏度）")
        else:
            self.colorbar.update_normal(image)

        time_axis = np.arange(len(magnet_history)) * DT
        self.magnet_ax.clear()
        self.magnet_ax.plot(time_axis, magnet_history, label=r"磁体 $T_m$")
        if contact_history is not None:
            self.magnet_ax.plot(
                time_axis,
                contact_history,
                linestyle="--",
                linewidth=1.8,
                label=r"轮下钢板 $T_{s,c}$",
            )
            self.magnet_ax.legend(loc="best", fontsize=9)
        self.magnet_ax.set_title(r"磁体与接触区温度")
        self.magnet_ax.set_xlabel("时间（秒）")
        self.magnet_ax.set_ylabel("温度（摄氏度）")

        self.force_ax.clear()
        self.force_ax.plot(time_axis, retention_history)
        self.force_ax.set_title(r"磁吸力保持率 $F_{mag}(t)/F_0$")
        self.force_ax.set_xlabel("时间（秒）")
        self.force_ax.set_ylabel("相对于标称吸力的百分比（%）")

        self.fig.tight_layout()
        self.draw()


class ChineseMainWindow(QWidget):
    """简体中文工程仪表板。"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("船体焊接热传导与磁吸附仿真器")
        self.resize(1500, 1100)
        self.setStyleSheet(
            """
            QWidget {
                background: #f4f7fb;
                color: #172033;
                font-family: "Microsoft YaHei", "SimHei";
                font-size: 15px;
            }
            QFrame#信息卡片, QFrame#控制卡片 {
                background: white;
                border: 1px solid #dfe6f0;
                border-radius: 12px;
            }
            QLabel#卡片标题 {
                color: #596780;
                font-size: 14px;
                font-weight: 700;
            }
            QLabel#分区标题 {
                color: #172033;
                font-size: 19px;
                font-weight: 700;
            }
            QSpinBox#数值输入 {
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
            """
        )

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(18, 16, 18, 18)
        main_layout.setSpacing(12)

        heading = QLabel("焊接热传导与磁吸附安全评估")
        heading.setStyleSheet("font-size: 28px; font-weight: 750; color: #101828;")
        subtitle = QLabel("移动式表面热源 · 三维钢板热传导 · 磁体集中参数热响应")
        subtitle.setStyleSheet("color: #667085; font-size: 15px;")
        main_layout.addWidget(heading)
        main_layout.addWidget(subtitle)

        self.canvas = ChinesePlotCanvas()
        main_layout.addWidget(self.canvas)

        dashboard_layout = QHBoxLayout()
        dashboard_layout.setSpacing(12)

        input_card = QFrame()
        input_card.setObjectName("信息卡片")
        input_card_layout = QVBoxLayout(input_card)
        input_card_layout.setContentsMargins(18, 14, 18, 14)
        input_title = QLabel("仿真设置")
        input_title.setObjectName("卡片标题")
        self.input_summary = QLabel()
        self.input_summary.setWordWrap(True)
        self.input_summary.setTextFormat(Qt.RichText)
        self.input_summary.setStyleSheet("font-size: 17px; line-height: 1.35;")
        input_card_layout.addWidget(input_title)
        input_card_layout.addWidget(self.input_summary)

        result_card = QFrame()
        result_card.setObjectName("信息卡片")
        result_card_layout = QVBoxLayout(result_card)
        result_card_layout.setContentsMargins(18, 14, 18, 14)
        result_title = QLabel("温度与磁吸附结果")
        result_title.setObjectName("卡片标题")
        self.result_summary = QLabel()
        self.result_summary.setWordWrap(True)
        self.result_summary.setTextFormat(Qt.RichText)
        self.result_summary.setStyleSheet("font-size: 17px; line-height: 1.35;")
        result_card_layout.addWidget(result_title)
        result_card_layout.addWidget(self.result_summary)

        dashboard_layout.addWidget(input_card, 1)
        dashboard_layout.addWidget(result_card, 1)
        main_layout.addLayout(dashboard_layout)

        self.slider = {}
        parameters = [
            ("机器人速度 mm/s", 8, 0, 200, "典型气保焊参考：4–9 mm/s"),
            ("净热输入 kW", 9, 1, 15, "工程筛选参考：3–10 kW 输入钢板"),
            ("焊枪至磁轮距离 mm", 200, 0, 1000, "应按机器人实际结构尺寸填写"),
            ("钢板厚度 mm", 20, 10, 40, "应填写船体钢板实际厚度"),
            ("表面对流换热 W/m²K", 10, 0, 15, "自然空气冷却参考：5–15"),
            ("接触换热系数 W/m²K", 500, 100, 2000, "仅用于敏感性分析，应通过试验标定"),
            ("仿真时间 s", 60, 5, 150, "建议覆盖多个时间常数 τ"),
        ]

        controls_card = QFrame()
        controls_card.setObjectName("控制卡片")
        controls_card_layout = QVBoxLayout(controls_card)
        controls_card_layout.setContentsMargins(18, 14, 18, 16)
        controls_title = QLabel("模型参数")
        controls_title.setObjectName("分区标题")
        controls_card_layout.addWidget(controls_title)

        parameter_grid = QGridLayout()
        parameter_grid.setHorizontalSpacing(14)
        parameter_grid.setVerticalSpacing(9)
        for row, (name, initial, minimum, maximum, recommendation) in enumerate(parameters):
            label = QLabel(
                f"<b>{name}</b><br>"
                f"<span style='color:#7b879c; font-size:13px'>{recommendation}</span>"
            )
            label.setTextFormat(Qt.RichText)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(minimum, maximum)
            slider.setValue(initial)
            value_input = QSpinBox()
            value_input.setObjectName("数值输入")
            value_input.setRange(minimum, maximum)
            value_input.setValue(initial)
            value_input.setAlignment(Qt.AlignCenter)
            value_input.setKeyboardTracking(False)
            slider.valueChanged.connect(value_input.setValue)
            value_input.valueChanged.connect(slider.setValue)
            self.slider[name] = slider
            parameter_grid.addWidget(label, row, 0)
            parameter_grid.addWidget(slider, row, 1)
            parameter_grid.addWidget(value_input, row, 2)

        controls_card_layout.addLayout(parameter_grid)
        main_layout.addWidget(controls_card)

        self.run_button = QPushButton("运行仿真  →")
        self.run_button.clicked.connect(self.run_simulation)
        main_layout.addWidget(self.run_button)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("准备就绪")
        self.progress_bar.setTextVisible(True)
        main_layout.addWidget(self.progress_bar)

        self.setLayout(main_layout)
        self.input_summary.setText("<span style='color:#667085'>请调整参数并运行仿真。</span>")
        self.result_summary.setText(
            "<span style='color:#667085'>运行后将在此显示温度、厚度修正与磁吸附结果。</span>"
        )

    def run_simulation(self):
        """读取参数、运行数值模型并刷新中文仪表板。"""
        speed_mm_s = self.slider["机器人速度 mm/s"].value()
        speed_m_s = mm_to_m(speed_mm_s)
        heat_input = self.slider["净热输入 kW"].value()
        wheel_distance = self.slider["焊枪至磁轮距离 mm"].value()
        thickness = self.slider["钢板厚度 mm"].value()
        cooling = self.slider["表面对流换热 W/m²K"].value()
        contact_conductance = self.slider["接触换热系数 W/m²K"].value()
        sim_time = self.slider["仿真时间 s"].value()

        self.run_button.setEnabled(False)
        self.run_button.setText("正在计算……")
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("计算中…… %p%")
        QApplication.processEvents()

        try:
            diagnostics = {}
            plate_temperature, magnet_temperature, retention = simulate(
                speed_m_s,
                heat_input,
                wheel_distance,
                thickness,
                cooling,
                contact_conductance,
                sim_time,
                progress_callback=self.update_progress,
                diagnostics=diagnostics,
            )
        except Exception:
            self.progress_bar.setFormat("仿真失败")
            self.run_button.setText("重新运行仿真  →")
            self.run_button.setEnabled(True)
            raise

        contact_history = diagnostics["contact_steel_temperature"]
        self.canvas.update_plot(plate_temperature, magnet_temperature, retention, contact_history)

        steps = int(np.ceil(sim_time / DT))
        final_source_time = (steps - 1) * DT if steps else 0.0
        final_weld_position, _ = robot_positions(final_source_time, speed_m_s, wheel_distance)
        weld_temperature = temperature_at_position(
            plate_temperature, PLATE_WIDTH / 2.0, final_weld_position
        )
        weld_temperature_text = (
            f"{weld_temperature:.1f} ℃" if weld_temperature is not None else "已超出钢板计算区域"
        )

        time_constant = contact_time_constant(contact_conductance)
        time_constant_text = f"{time_constant:.1f} 秒" if np.isfinite(time_constant) else "无穷大"
        peak_magnet_temperature = max(magnet_temperature, default=AMBIENT_TEMPERATURE)
        peak_plate_temperature = float(np.max(plate_temperature))
        thickness_baseline = thickness / (thickness + SATURATION_THICKNESS_MM) * 100.0
        final_retention = retention[-1] if retention else thickness_baseline
        thermal_loss = max(thickness_baseline - final_retention, 0.0)
        predicted_force = NOMINAL_FORCE * final_retention / 100.0
        diffusion_time_scale = mm_to_m(wheel_distance) ** 2 / ALPHA

        active_contact_steps = int(np.count_nonzero(diagnostics["contact_active"]))
        finite_contact_temperatures = np.asarray(contact_history, dtype=float)
        finite_contact_temperatures = finite_contact_temperatures[np.isfinite(finite_contact_temperatures)]
        if finite_contact_temperatures.size:
            peak_contact_temperature = float(np.max(finite_contact_temperatures))
            final_contact_temperature = float(finite_contact_temperatures[-1])
        else:
            peak_contact_temperature = np.nan
            final_contact_temperature = np.nan

        status = "警告" if peak_magnet_temperature > 80.0 else "安全"
        status_color = "#b42318" if status == "警告" else "#067647"
        status_background = "#fef3f2" if status == "警告" else "#ecfdf3"

        self.input_summary.setText(
            f"""
            <table cellspacing='7' cellpadding='2'>
              <tr><td width='275' style='color:#667085'>机器人速度</td><td><b>{speed_mm_s} mm/s</b>（{speed_m_s:.3f} m/s）</td></tr>
              <tr><td style='color:#667085'>钢板净热输入</td><td><b>{heat_input} kW</b></td></tr>
              <tr><td style='color:#667085'>焊枪至磁轮距离</td><td><b>{wheel_distance} mm</b></td></tr>
              <tr><td style='color:#667085'>磁轮初始中心位置 y<sub>m,0</sub></td><td><b>{ROBOT_START_Y:.3f} m</b></td></tr>
              <tr><td style='color:#667085'>距离扩散时间尺度 d²/α</td><td><b>{diffusion_time_scale:.0f} 秒</b></td></tr>
              <tr><td style='color:#667085'>磁轮接触状态</td><td><b>{active_contact_steps}/{len(magnet_temperature)} 步有效</b></td></tr>
              <tr><td style='color:#667085'>钢板厚度</td><td><b>{thickness} mm</b></td></tr>
              <tr><td style='color:#667085'>表面对流换热系数 h<sub>c</sub></td><td><b>{cooling} W/(m² K)</b></td></tr>
              <tr><td style='color:#667085'>接触换热系数 h<sub>contact</sub></td><td><b>{contact_conductance} W/(m² K)</b></td></tr>
              <tr><td style='color:#667085'>磁体接触时间常数 τ</td><td><b>{time_constant_text}</b></td></tr>
              <tr><td style='color:#667085'>最终焊点位置 y<sub>w</sub></td><td><b>{final_weld_position:.3f} m</b></td></tr>
              <tr><td style='color:#667085'>最终焊点表面温度</td><td><b>{weld_temperature_text}</b></td></tr>
            </table>"""
        )

        self.result_summary.setText(
            f"""
            <table cellspacing='7' cellpadding='2'>
              <tr><td width='275' style='color:#667085'>钢板表面峰值温度</td><td><span style='font-size:28px; font-weight:700'>{peak_plate_temperature:.1f} ℃</span></td></tr>
              <tr><td style='color:#667085'>磁体峰值温度 T<sub>m,max</sub></td><td><span style='font-size:28px; font-weight:700'>{peak_magnet_temperature:.1f} ℃</span></td></tr>
              <tr><td style='color:#667085'>轮下钢板峰值温度</td><td><b>{peak_contact_temperature:.2f} ℃</b></td></tr>
              <tr><td style='color:#667085'>轮下钢板最终温度</td><td><b>{final_contact_temperature:.2f} ℃</b></td></tr>
              <tr><td colspan='2'><hr style='color:#e4e7ec'></td></tr>
              <tr><td style='color:#667085'>仅考虑厚度的保持率基准</td><td><b>{thickness_baseline:.1f}%</b></td></tr>
              <tr><td style='color:#667085'>温升导致的额外损失</td><td><b>−{thermal_loss:.2f} 个百分点</b></td></tr>
              <tr><td style='color:#667085'>最终磁吸力保持率</td><td><span style='font-size:30px; color:#175cd3; font-weight:750'>{final_retention:.1f}%</span></td></tr>
              <tr><td style='color:#667085'>预测磁吸力</td><td><b>{predicted_force:.1f} N</b> / 标称 {NOMINAL_FORCE:.0f} N</td></tr>
              <tr><td colspan='2' style='padding-top:8px'><span style='background:{status_background}; color:{status_color}; font-weight:700'>&nbsp; {status} · 磁体温度阈值 80 ℃ &nbsp;</span></td></tr>
            </table>"""
        )

        self.progress_bar.setValue(100)
        self.progress_bar.setFormat("仿真完成 · 100%")
        self.run_button.setText("再次运行仿真  →")
        self.run_button.setEnabled(True)

    def update_progress(self, percent):
        """刷新进度条并处理界面重绘事件。"""
        self.progress_bar.setValue(percent)
        QApplication.processEvents()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ChineseMainWindow()
    window.show()
    sys.exit(app.exec_())
