"""Read-only arm telemetry views shared by manual and coordinated controls.

Inputs are copies of cached owner data. This module has no transport or command
interface. History is display-rate evidence, not a replacement for the journal.
"""

from collections import deque
import math
import time

from PyQt5 import QtCore, QtGui, QtWidgets


AXIS_COLORS = ("#2563eb", "#0e8679", "#9333b8", "#c66b12", "#cf3e67", "#5265b0")
FIELDS = ("torque_feedback_nm", "rotor_c", "mos_c")


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def position_telemetry(rows):
    return [dict(joint=r.joint, age_ms=r.age_ms, enabled=r.enabled,
                 torque_feedback_nm=r.torque_feedback_nm, torque_trip_nm=r.torque_trip_nm,
                 rotor_c=r.rotor_c, mos_c=r.mos_c) for r in rows]


class TelemetryHistory:
    """Bounded 30-second traces and fresh-only peaks for a stage and run."""

    def __init__(self):
        self.stage = self.run = None
        self.traces = [deque(maxlen=320) for _ in range(6)]
        self.stage_peaks = [dict() for _ in range(6)]
        self.run_peaks = [dict() for _ in range(6)]

    def update(self, rows, *, now, captured_at, stage, run, stale=False):
        if run != self.run:
            self.run = run
            self.run_peaks = [dict() for _ in range(6)]
            for trace in self.traces:
                trace.clear()
        if (run, stage) != self.stage:
            self.stage = (run, stage)
            self.stage_peaks = [dict() for _ in range(6)]
        by_joint = {row.get("joint"): row for row in rows or []}
        result = []
        for axis in range(6):
            row = dict(by_joint.get(axis + 1, {}))
            age = row.get("age_ms")
            # UINT32_MAX means the MCU has no valid feedback. Cached display
            # snapshots may already have added host elapsed time to that value.
            age = age + max(0, now - captured_at) * 1000 if finite(age) and 0 <= age < 0xffffffff else None
            fresh = not stale and age is not None and age < 80
            row.update(age_ms=age, fresh=fresh)
            values = {key: row.get(key) if fresh and finite(row.get(key)) else None for key in FIELDS}
            row.update(values)
            trace = self.traces[axis]
            # A duplicated queued UI snapshot must not extend its history.
            if not trace or captured_at > trace[-1][0]:
                trace.append((captured_at, values))
                for key, value in values.items():
                    if value is not None:
                        peak = abs(value) if key == FIELDS[0] else value
                        for peaks in (self.stage_peaks[axis], self.run_peaks[axis]):
                            peaks[key] = max(peaks.get(key, peak), peak)
            while trace and trace[0][0] < now - 30:
                trace.popleft()
            result.append(row)
        return result


class Sparkline(QtWidgets.QWidget):
    def __init__(self, fields, colors, parent=None):
        super().__init__(parent)
        self.fields, self.colors = fields, colors
        self.trace = []
        self.now = 0
        self.setFixedHeight(18)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect().adjusted(2, 3, -2, -3)
        painter.fillRect(rect, QtGui.QColor("#f1f5f9"))
        values = [row[key] for _, row in self.trace for key in self.fields if row[key] is not None]
        if not values:
            painter.setPen(QtGui.QColor("#64748b"))
            painter.drawText(rect, QtCore.Qt.AlignCenter, "No fresh samples")
            return
        lo, hi = min(values), max(values)
        if self.fields == (FIELDS[0],):
            lo, hi = min(lo, 0), max(hi, 0)
        margin = max((hi - lo) * .15, .1)
        lo, hi = lo - margin, hi + margin
        for key, color in zip(self.fields, self.colors):
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 1.7))
            previous = None
            for at, row in self.trace:
                value = row[key]
                if value is None:
                    previous = None
                    continue
                point = QtCore.QPointF(rect.left() + rect.width() * (1 - (self.now - at) / 30),
                                      rect.bottom() - rect.height() * (value - lo) / (hi - lo))
                if previous is not None and at - previous[0] < .3:
                    painter.drawLine(previous[1], point)
                else:
                    painter.drawPoint(point)
                previous = at, point


class ReadingLabel(QtWidgets.QLabel):
    """Reserve a stable width for changing digits, signs and sample states."""

    def __init__(self, text, width_sample):
        super().__init__(text)
        self.width_sample = width_sample

    def sizeHint(self):
        size = super().sizeHint()
        size.setWidth(self.fontMetrics().horizontalAdvance(self.width_sample) + 2 * self.margin())
        return size

    def minimumSizeHint(self):
        return self.sizeHint()


class TelemetryPanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.history = TelemetryHistory()
        self.cards = []
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        for column in range(3):
            layout.setColumnStretch(column, 1)
        for row in range(2):
            layout.setRowStretch(row, 1)
        for axis, color in enumerate(AXIS_COLORS):
            card = QtWidgets.QFrame()
            card.setObjectName("telemetryCard")
            card.setStyleSheet(f"QFrame#telemetryCard {{ background: white; border: 1px solid #d8e1ec; border-top: 3px solid {color}; border-radius: 6px; }}")
            box = QtWidgets.QVBoxLayout(card)
            box.setContentsMargins(9, 5, 9, 5)
            box.setSpacing(2)
            title = ReadingLabel(f"A{axis+1}  ·  Unavailable", "A6  ·  Disabled · 9999 ms")
            title.setStyleSheet(f"color: {color}; font-weight: 600")
            box.addWidget(title)
            torque = ReadingLabel("— Nm", "-54.00 Nm")
            torque.setStyleSheet("font-size: 20px; font-weight: 600; color: #172b46")
            box.addWidget(torque)
            load = QtWidgets.QProgressBar()
            load.setRange(0, 100)
            load.setFixedHeight(17)
            tint = QtGui.QColor(color).lighter(175).name()
            load.setStyleSheet(f"QProgressBar {{border: 0; background: #edf2f7; text-align: center; color: #172b46}} QProgressBar::chunk {{background: {tint}}}")
            box.addWidget(load)
            torque_plot = Sparkline((FIELDS[0],), (color,))
            box.addWidget(torque_plot)
            temperatures = ReadingLabel("Motor — °C  ·  MOS — °C", "Motor 80 °C  ·  MOS 80 °C")
            box.addWidget(temperatures)
            temp_plot = Sparkline(FIELDS[1:], ("#c66b12", "#5265b0"))
            box.addWidget(temp_plot)
            peaks = ReadingLabel("Peak |Nm| · stage — / run —", "Peak |Nm| · stage 54.00 / run 54.00")
            peaks.setStyleSheet("color: #526176; font-size: 11px")
            box.addWidget(peaks)
            layout.addWidget(card, axis // 3, axis % 3)
            self.cards.append(dict(title=title, torque=torque, load=load, temperatures=temperatures,
                                   torque_plot=torque_plot, temp_plot=temp_plot, peaks=peaks, frame=card))

    def set_display_scale(self, scale):
        text_height = self.fontMetrics().height()
        for card in self.cards:
            card["torque"].setStyleSheet(
                f"font-size: {round(20 * scale)}px; font-weight: 600; color: #172b46"
            )
            card["peaks"].setStyleSheet(f"color: #526176; font-size: {round(11 * scale)}px")
            card["load"].setFixedHeight(max(round(17 * scale), text_height + 2))
            for name in ("torque_plot", "temp_plot"):
                card[name].setFixedHeight(max(round(18 * scale), text_height + 6))

    def update_samples(self, rows, *, captured_at=None, stage="idle", run=None, stale=False, live_expected=True):
        now = time.monotonic()
        captured_at = now if captured_at is None else captured_at
        readings = self.history.update(rows, now=now, captured_at=captured_at, stage=stage, run=run, stale=stale)
        for axis, (row, card) in enumerate(zip(readings, self.cards)):
            age = row["age_ms"]
            state = ("Enabled" if row.get("enabled") is True else "Disabled" if row.get("enabled") is False else "State unknown") if row["fresh"] else "STALE"
            label = "Unavailable" if age is None else f"{state} · {age:.0f} ms"
            if age is not None and not row["fresh"] and (stale or not live_expected):
                label = f"Cached · {age / 1000:.1f} s"
            card["title"].setText(f"A{axis+1}  ·  {label}")
            card["title"].setToolTip(
                "No valid feedback sample is available." if age is None else
                "Age of the displayed cached sample. Idle or fault snapshots are historical; "
                "they do not establish the motor's current enabled state. "
                "Motion requires feedback within 80 ms."
            )
            torque, trip = row[FIELDS[0]], row.get("torque_trip_nm")
            card["torque"].setText("— Nm" if torque is None else f"{torque:+.2f} Nm")
            ratio = abs(torque) / trip * 100 if torque is not None and finite(trip) and trip > 0 else None
            card["load"].setValue(0 if ratio is None else min(100, round(ratio)))
            card["load"].setFormat(("Load —" if ratio is None else f"{ratio:.0f}%") + (f" · trip {trip:g} Nm" if finite(trip) else " · trip unknown"))
            card["load"].setToolTip("Absolute reported torque / configured reactive trip. This trip is not a continuous motor rating; feedback TMAX is an encoding range.")
            def temperature(key):
                return "—" if row[key] is None else f"{row[key]:.0f}"
            card["temperatures"].setText(f"Motor {temperature('rotor_c')} °C  ·  MOS {temperature('mos_c')} °C")
            stage_peaks, run_peaks = self.history.stage_peaks[axis], self.history.run_peaks[axis]
            def peak(peaks, key, precision=2):
                return "—" if key not in peaks else f"{peaks[key]:.{precision}f}"
            card["peaks"].setText(f"Peak |Nm| · stage {peak(stage_peaks, FIELDS[0])} / run {peak(run_peaks, FIELDS[0])}")
            details = "\n".join(f"{name} peak: stage {peak(stage_peaks, key)} / run {peak(run_peaks, key)} {unit}"
                                for name, key, unit in (("|Torque|", FIELDS[0], "Nm"), ("Motor", "rotor_c", "°C"), ("MOS", "mos_c", "°C")))
            card["frame"].setToolTip(details + "\n30 s display-rate trends; temperature: orange motor, purple MOS.\nPeaks use fresh samples only. Full-rate evidence remains in the journal.")
            for key in ("torque_plot", "temp_plot"):
                plot = card[key]
                plot.trace, plot.now = list(self.history.traces[axis]), now
                plot.update()
        return readings
