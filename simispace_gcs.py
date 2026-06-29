"""
╔══════════════════════════════════════════════════════════════════════════════╗
║   SIMISPACE — GROUND CONTROL STATION v2                                     ║
║   Estación Terrena en tiempo real · PyQt6 + pyserial                        ║
╠══════════════════════════════════════════════════════════════════════════════╣
║   DEPENDENCIAS (instalar UNA sola vez):                                     ║
║     pip3 install PyQt6 pyserial                                             ║
║                                                                             ║
║   CORRER:                                                                   ║
║     python3 simispace_gcs.py                                                ║
║                                                                             ║
║   FORMATO SERIAL ESPERADO DEL HELTEC (115200 bps, una línea por paquete):  ║
║     TELEM,<presión_Pa>,<temp_°C>,<altitud_m>,<humedad_%>,                  ║
║           <lat>,<lon>,<roll_°>,<pitch_°>,<heading_°>\n                      ║
║   Ejemplo:                                                                  ║
║     TELEM,101325.0,23.4,156.2,48.3,19.4326,-99.1332,2.1,-1.3,45.6         ║
║                                                                             ║
║   ARDUINO / Heltec — snippet mínimo para mandar el paquete:                ║
║     Serial.print("TELEM,");                                                 ║
║     Serial.print(pressure,1);    Serial.print(",");                         ║
║     Serial.print(temperature,1); Serial.print(",");                         ║
║     Serial.print(altitude,1);    Serial.print(",");                         ║
║     Serial.print(humidity,1);    Serial.print(",");                         ║
║     Serial.print(lat,5);         Serial.print(",");                         ║
║     Serial.print(lon,5);         Serial.print(",");                         ║
║     Serial.print(roll,1);        Serial.print(",");                         ║
║     Serial.print(pitch,1);       Serial.print(",");                         ║
║     Serial.println(heading,1);                                              ║
║                                                                             ║
║   DATOS GRABADOS: ~/Downloads/simispace_telem_YYYYMMDD_HHMMSS.csv          ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import sys
import os
import csv
import time
import math
import random
import datetime
from collections import deque
from typing import Optional

import serial
import serial.tools.list_ports

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QComboBox, QPushButton, QTextEdit,
    QSizePolicy, QDialog, QFormLayout,
    QSpinBox, QDialogButtonBox, QMessageBox,
)
from PyQt6.QtCore import (
    Qt, QThread, pyqtSignal, QTimer, QPointF,
)
from PyQt6.QtGui import (
    QColor, QPainter, QPen, QBrush, QPainterPath, QFont,
    QLinearGradient, QPolygonF, QRadialGradient,
)

# ═══════════════════════════════════════════════════════════════════════════════
# PALETA DE COLORES
# ═══════════════════════════════════════════════════════════════════════════════

C = {
    "bg":        "#070D18",
    "panel":     "#101B2E",
    "panel_alt": "#0C1422",
    "border":    "#1E2C42",
    "ink":       "#EAF0F6",
    "ink_dim":   "#90A4BD",
    "muted":     "#56677F",
    "accent":    "#4FB3E8",
    "green":     "#3ED598",
    "amber":     "#F2A93C",
    "red":       "#E5484D",
}

def qc(hex_str: str) -> QColor:
    return QColor(hex_str)

# ═══════════════════════════════════════════════════════════════════════════════
# UMBRALES
# ═══════════════════════════════════════════════════════════════════════════════

THRESHOLDS = {
    "pressure":    {"caution_lo": 90000,  "caution_hi": 106000,
                    "critical_lo": 85000, "critical_hi": 110000},
    "temperature": {"caution_lo": -5,     "caution_hi": 32,
                    "critical_lo": -15,   "critical_hi": 42},
    "humidity":    {"caution_lo": 12,     "caution_hi": 78,
                    "critical_lo": 5,     "critical_hi": 90},
}

def get_status(key: str, value: float) -> str:
    t = THRESHOLDS.get(key)
    if not t:
        return "nominal"
    if value <= t["critical_lo"] or value >= t["critical_hi"]:
        return "critical"
    if value <= t["caution_lo"] or value >= t["caution_hi"]:
        return "caution"
    return "nominal"

def status_color(status: str) -> str:
    return {"nominal": C["ink"], "caution": C["amber"], "critical": C["red"]}.get(status, C["ink"])

def status_accent(status: str) -> str:
    return {"nominal": C["accent"], "caution": C["amber"], "critical": C["red"]}.get(status, C["accent"])

# ═══════════════════════════════════════════════════════════════════════════════
# HELPER: INTERPOLACIÓN LINEAL (lerp)
# ═══════════════════════════════════════════════════════════════════════════════

def lerp(a: float, b: float, t: float) -> float:
    """Interpola linealmente entre a y b con factor t ∈ [0,1]."""
    return a + (b - a) * t

def lerp_angle(a: float, b: float, t: float) -> float:
    """
    Interpolación de ángulos corta (toma el camino más corto en el círculo).
    Evita que el heading salte de 359° a 0° dando una vuelta entera.
    """
    diff = (b - a + 180) % 360 - 180
    return a + diff * t

# ═══════════════════════════════════════════════════════════════════════════════
# DATOS DE TELEMETRÍA
# ═══════════════════════════════════════════════════════════════════════════════

class TelemetryPacket:
    __slots__ = ("pressure", "temperature", "altitude", "humidity",
                 "lat", "lon", "roll", "pitch", "heading", "timestamp")
    def __init__(self):
        self.pressure    = 101325.0
        self.temperature = 20.0
        self.altitude    = 0.0
        self.humidity    = 50.0
        self.lat         = 19.4326
        self.lon         = -99.1332
        self.roll        = 0.0
        self.pitch       = 0.0
        self.heading     = 0.0
        self.timestamp   = time.time()

# ═══════════════════════════════════════════════════════════════════════════════
# PARSER
# ═══════════════════════════════════════════════════════════════════════════════

def parse_line(line: str) -> Optional[TelemetryPacket]:
    """
    Parsea una línea TELEM del Heltec (CSV con prefijo "TELEM").
    Para JSON: reemplazar el cuerpo con json.loads(line) y mapear campos.
    """
    line = line.strip()
    if not line.startswith("TELEM"):
        return None
    parts = line.split(",")
    if len(parts) < 10:
        return None
    try:
        pkt = TelemetryPacket()
        pkt.pressure    = float(parts[1])
        pkt.temperature = float(parts[2])
        pkt.altitude    = float(parts[3])
        pkt.humidity    = float(parts[4])
        pkt.lat         = float(parts[5])
        pkt.lon         = float(parts[6])
        pkt.roll        = float(parts[7])
        pkt.pitch       = float(parts[8])
        pkt.heading     = float(parts[9])
        pkt.timestamp   = time.time()
        return pkt
    except (ValueError, IndexError):
        return None

# ═══════════════════════════════════════════════════════════════════════════════
# WORKER SERIAL
# ═══════════════════════════════════════════════════════════════════════════════

class SerialWorker(QThread):
    packet_received    = pyqtSignal(object)
    raw_line           = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)
    error              = pyqtSignal(str)

    def __init__(self, port: str, baudrate: int = 115200):
        super().__init__()
        self.port     = port
        self.baudrate = baudrate
        self._running = True
        self._ser: Optional[serial.Serial] = None

    def run(self):
        try:
            self._ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=2.0)
            self.connection_changed.emit(True)
        except serial.SerialException as e:
            self.error.emit(f"No se pudo abrir {self.port}: {e}")
            self.connection_changed.emit(False)
            return

        while self._running:
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                self.raw_line.emit(line)
                pkt = parse_line(line)
                if pkt:
                    self.packet_received.emit(pkt)
            except serial.SerialException as e:
                self.error.emit(f"Error de lectura serial: {e}")
                self.connection_changed.emit(False)
                break

        if self._ser and self._ser.is_open:
            self._ser.close()
        self.connection_changed.emit(False)

    def stop(self):
        self._running = False
        self.wait(2000)

# ═══════════════════════════════════════════════════════════════════════════════
# WORKER DE SIMULACIÓN
# ═══════════════════════════════════════════════════════════════════════════════

class SimWorker(QThread):
    packet_received    = pyqtSignal(object)
    raw_line           = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)
    error              = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._running     = True
        self._pressure    = 101325.0
        self._temperature = 21.0
        self._altitude    = 0.0
        self._humidity    = 48.0
        self._lat         = 19.4326
        self._lon         = -99.1332
        self._heading     = 0.0
        self._t           = 0

    def _rw(self, val, lo, hi, step, bias=0.0):
        v = val + (random.random() - 0.5) * step + bias * step * 0.3
        return max(lo, min(hi, v))

    def run(self):
        self.connection_changed.emit(True)
        while self._running:
            self._t += 1
            self._pressure    = self._rw(self._pressure,    80000, 103000, 220)
            self._temperature = self._rw(self._temperature, -20,   45,     0.6)
            self._altitude    = self._rw(self._altitude,    0,     3200,   9, 0.6)
            self._humidity    = self._rw(self._humidity,    5,     100,    2.2)
            self._lat        += (random.random() - 0.5) * 0.0006 + 0.00008
            self._lon        += (random.random() - 0.5) * 0.0006 + 0.00006
            self._heading     = (self._heading + 0.6 + (random.random()-0.5)*0.4) % 360
            roll  = 15 * math.sin(self._t / 6) + (random.random()-0.5)*2
            pitch =  8 * math.sin(self._t / 9 + 1) + (random.random()-0.5)*1.5

            pkt = TelemetryPacket()
            pkt.pressure = self._pressure; pkt.temperature = self._temperature
            pkt.altitude = self._altitude; pkt.humidity    = self._humidity
            pkt.lat      = self._lat;      pkt.lon         = self._lon
            pkt.roll     = roll;           pkt.pitch       = pitch
            pkt.heading  = self._heading

            line = (f"TELEM,{pkt.pressure:.1f},{pkt.temperature:.1f},"
                    f"{pkt.altitude:.1f},{pkt.humidity:.1f},"
                    f"{pkt.lat:.5f},{pkt.lon:.5f},"
                    f"{pkt.roll:.1f},{pkt.pitch:.1f},{pkt.heading:.1f}")
            self.raw_line.emit(line)
            self.packet_received.emit(pkt)
            self.msleep(1000)

        self.connection_changed.emit(False)

    def stop(self):
        self._running = False
        self.wait(2000)

# ═══════════════════════════════════════════════════════════════════════════════
# WIDGET: SPARKLINE
# ═══════════════════════════════════════════════════════════════════════════════

class SparklineWidget(QWidget):
    def __init__(self, color: str = C["accent"], max_points: int = 60, parent=None):
        super().__init__(parent)
        self._data: deque = deque(maxlen=max_points)
        self._color = QColor(color)
        self.setMinimumSize(80, 40)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_color(self, hex_color: str):
        self._color = QColor(hex_color)
        self.update()

    def push(self, value: float):
        self._data.append(value)
        self.update()

    def paintEvent(self, event):
        if len(self._data) < 2:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        data = list(self._data)
        lo, hi = min(data), max(data)
        rng = hi - lo or 1.0
        w, h = self.width(), self.height()
        pad = 4

        def to_pt(i, v):
            x = pad + (i / (len(data) - 1)) * (w - 2*pad)
            y = h - pad - ((v - lo) / rng) * (h - 2*pad)
            return QPointF(x, y)

        pts = [to_pt(i, v) for i, v in enumerate(data)]

        path = QPainterPath()
        path.moveTo(QPointF(pts[0].x(), h))
        for pt in pts:
            path.lineTo(pt)
        path.lineTo(QPointF(pts[-1].x(), h))
        path.closeSubpath()

        grad = QLinearGradient(0, 0, 0, h)
        c_top = QColor(self._color); c_top.setAlpha(80)
        c_bot = QColor(self._color); c_bot.setAlpha(0)
        grad.setColorAt(0, c_top); grad.setColorAt(1, c_bot)
        p.fillPath(path, QBrush(grad))

        pen = QPen(self._color, 1.6)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        p.setPen(pen)
        p.drawPolyline(QPolygonF(pts))
        p.end()

# ═══════════════════════════════════════════════════════════════════════════════
# WIDGET: TARJETA DE TELEMETRÍA
# ═══════════════════════════════════════════════════════════════════════════════

class TelemetryCard(QFrame):
    def __init__(self, label: str, unit: str, threshold_key: str,
                 decimals: int = 1, parent=None):
        super().__init__(parent)
        self._threshold_key = threshold_key
        self._decimals      = decimals
        self._status        = "nominal"

        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setObjectName("TelemetryCard")
        self._apply_style(C["border"])

        outer = QHBoxLayout(self)
        outer.setContentsMargins(12, 10, 12, 10)
        outer.setSpacing(10)

        left = QVBoxLayout()
        left.setSpacing(2)

        self._lbl_title = QLabel(label.upper())
        self._lbl_title.setStyleSheet(
            f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 2px;")

        self._lbl_value = QLabel("—")
        self._lbl_value.setStyleSheet(
            f"color: {C['ink']}; font-size: 28px; font-weight: bold;")

        self._lbl_unit = QLabel(unit)
        self._lbl_unit.setStyleSheet(
            f"color: {C['ink_dim']}; font-size: 12px;")

        self._lbl_status = QLabel("NOMINAL")
        self._lbl_status.setStyleSheet(
            f"color: {C['accent']}; font-size: 10px; font-weight: bold; letter-spacing: 2px;")

        val_row = QHBoxLayout()
        val_row.setSpacing(4)
        val_row.addWidget(self._lbl_value)
        val_row.addWidget(self._lbl_unit)
        val_row.addStretch()

        left.addWidget(self._lbl_title)
        left.addLayout(val_row)
        left.addWidget(self._lbl_status)

        self._spark = SparklineWidget(color=C["accent"])
        self._spark.setFixedWidth(100)

        outer.addLayout(left, stretch=1)
        outer.addWidget(self._spark)

    def _apply_style(self, border_color: str):
        self.setStyleSheet(f"""
            #TelemetryCard {{
                background: {C['panel']};
                border: 1px solid {border_color};
                border-radius: 8px;
            }}
        """)

    def update_value(self, value: float):
        status     = get_status(self._threshold_key, value)
        col_val    = status_color(status)
        col_accent = status_accent(status)

        if status != self._status:
            self._status = status
            self._lbl_value.setStyleSheet(
                f"color: {col_val}; font-size: 28px; font-weight: bold;")
            self._lbl_status.setStyleSheet(
                f"color: {col_accent}; font-size: 10px; font-weight: bold; letter-spacing: 2px;")
            self._spark.set_color(col_accent)
            self._apply_style(col_accent if status != "nominal" else C["border"])

        self._lbl_status.setText(status.upper())
        self._lbl_value.setText(f"{value:.{self._decimals}f}")
        self._spark.push(value)

# ═══════════════════════════════════════════════════════════════════════════════
# WIDGET: STAT CARD
# ═══════════════════════════════════════════════════════════════════════════════

class StatCard(QFrame):
    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self.setObjectName("StatCard")
        self.setStyleSheet(f"""
            #StatCard {{
                background: {C['panel']};
                border: 1px solid {C['border']};
                border-radius: 8px;
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(2)

        self._lbl_label = QLabel(label.upper())
        self._lbl_label.setStyleSheet(
            f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 2px;")
        self._lbl_value = QLabel("—")
        self._lbl_value.setStyleSheet(
            f"color: {C['accent']}; font-size: 14px; font-weight: bold;")

        layout.addWidget(self._lbl_label)
        layout.addWidget(self._lbl_value)

    def set_value(self, text: str, color: str = C["accent"]):
        self._lbl_value.setText(text)
        self._lbl_value.setStyleSheet(
            f"color: {color}; font-size: 14px; font-weight: bold;")

# ═══════════════════════════════════════════════════════════════════════════════
# WIDGET: CONSOLA DE LOGS
# ═══════════════════════════════════════════════════════════════════════════════

class LogConsole(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("LogConsole")
        self.setStyleSheet(f"""
            #LogConsole {{
                background: {C['panel']};
                border: 1px solid {C['border']};
                border-radius: 8px;
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        header = QWidget()
        header.setFixedHeight(34)
        header.setStyleSheet(f"background: transparent; border-bottom: 1px solid {C['border']};")
        h_lay = QHBoxLayout(header)
        h_lay.setContentsMargins(12, 0, 12, 0)
        lbl = QLabel("MISSION EVENTS / LOG")
        lbl.setStyleSheet(f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 2px;")
        h_lay.addWidget(lbl)

        self._text = QTextEdit()
        self._text.setReadOnly(True)
        self._text.setStyleSheet(f"""
            QTextEdit {{
                background: transparent;
                border: none;
                font-family: 'Courier New', monospace;
                font-size: 11px;
                color: {C['ink_dim']};
            }}
        """)
        layout.addWidget(header)
        layout.addWidget(self._text)

    def log(self, level: str, message: str, met: str = ""):
        ts    = f"[T+{met}]" if met else ""
        color = {"INFO": C["ink_dim"], "WARN": C["amber"],
                 "CRIT": C["red"],     "RAW":  C["muted"]}.get(level, C["ink_dim"])
        lvl_s = f"[{level}]" if level != "RAW" else ""
        html  = (f'<span style="color:{C["muted"]};">{ts}</span> '
                 f'<span style="color:{color}; font-weight:bold;">{lvl_s}</span> '
                 f'<span style="color:{C["ink"]};">{message}</span>')
        self._text.append(html)
        sb = self._text.verticalScrollBar()
        sb.setValue(sb.maximum())

# ═══════════════════════════════════════════════════════════════════════════════
# WIDGET: TRAYECTORIA 2D
# ═══════════════════════════════════════════════════════════════════════════════

class TrajectoryWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._points: deque   = deque(maxlen=200)
        self._heading         = 0.0
        self._origin_lat      = None
        self._origin_lon      = None
        self.setMinimumSize(160, 150)

    def _to_rel(self, lat, lon):
        if self._origin_lat is None:
            self._origin_lat = lat
            self._origin_lon = lon
        scale = 0.012
        x = (lon - self._origin_lon) / scale
        y = (lat - self._origin_lat) / scale
        return max(-1, min(1, x)), max(-1, min(1, y))

    def push(self, lat: float, lon: float, heading: float):
        rx, ry = self._to_rel(lat, lon)
        self._points.append((rx, ry))
        self._heading = heading
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h   = self.width(), self.height()
        pad    = 14
        cx, cy = w / 2, h / 2
        rx, ry = (w - 2*pad) / 2, (h - 2*pad) / 2

        p.setPen(QPen(qc(C["border"]), 1))
        for i in range(5):
            fx = pad + i * (w - 2*pad) / 4
            fy = pad + i * (h - 2*pad) / 4
            p.drawLine(int(fx), pad, int(fx), h - pad)
            p.drawLine(pad, int(fy), w - pad, int(fy))

        p.setFont(QFont("Courier New", 8))
        p.setPen(QPen(qc(C["muted"]), 1))
        p.drawText(int(cx) + 3, pad + 10, "N")

        if len(self._points) < 2:
            p.end()
            return

        pen = QPen(qc(C["accent"]), 1.5)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        p.setPen(pen)

        def to_px(px_r, py_r):
            return QPointF(cx + px_r * rx, cy - py_r * ry)

        pts  = [to_px(x, y) for x, y in self._points]
        p.drawPolyline(QPolygonF(pts))

        last = pts[-1]
        rad  = math.radians(self._heading)
        tip  = QPointF(last.x() + math.sin(rad)*10, last.y() - math.cos(rad)*10)
        lft  = QPointF(last.x() + math.sin(rad+2.5)*5, last.y() - math.cos(rad+2.5)*5)
        rgt  = QPointF(last.x() + math.sin(rad-2.5)*5, last.y() - math.cos(rad-2.5)*5)
        p.setBrush(QBrush(qc(C["ink"])))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(QPolygonF([tip, lft, rgt]))
        p.end()

# ═══════════════════════════════════════════════════════════════════════════════
# WIDGET: HORIZONTE ARTIFICIAL — FLUIDO A 60 FPS CON INTERPOLACIÓN
# ═══════════════════════════════════════════════════════════════════════════════

class AttitudeWidget(QWidget):
    """
    Horizonte artificial que corre a 60 fps con interpolación suave (lerp)
    entre los valores reales que llegan del sensor (1 Hz).

    Mecánica:
      - set_target(roll, pitch) se llama cada vez que llega un paquete real.
      - Un QTimer interno a 16 ms (~60 fps) avanza los valores actuales
        (_roll, _pitch) hacia los objetivos (_target_roll, _target_pitch)
        usando lerp con factor SMOOTH (0 = sin movimiento, 1 = instantáneo).
      - Así el widget nunca "inventa" datos: solo interpola entre dos lecturas
        reales consecutivas, igual que hacen los EFIS modernos de aviación.
    """

    SMOOTH = 0.12   # Factor de suavizado por frame a 60 fps.
                    # 0.10–0.15 = fluido y fiel. Subir para más respuesta.

    def __init__(self, parent=None):
        super().__init__(parent)
        # Valores interpolados (lo que se dibuja)
        self._roll  = 0.0
        self._pitch = 0.0
        # Objetivos (valor real del sensor)
        self._target_roll  = 0.0
        self._target_pitch = 0.0

        self.setMinimumSize(150, 140)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Timer de render a ~60 fps
        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._step)
        self._render_timer.start(16)   # 16 ms ≈ 60 fps

    def set_target(self, roll: float, pitch: float):
        """Recibe los valores reales del sensor (llamado desde _on_packet)."""
        self._target_roll  = roll
        self._target_pitch = pitch

    def _step(self):
        """Avanza los valores interpolados hacia los objetivos y redibuja."""
        self._roll  = lerp(self._roll,  self._target_roll,  self.SMOOTH)
        self._pitch = lerp(self._pitch, self._target_pitch, self.SMOOTH)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h   = self.width(), self.height()
        cx, cy = w / 2, h / 2
        r      = min(w, h) / 2 - 8

        # ── Clip circular ────────────────────────────────────────────────────
        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), r, r)
        p.setClipPath(clip)

        pitch_px = max(-r * 0.7, min(r * 0.7,
                       self._pitch * (r / 30)))  # 30° ocupa todo el radio

        # ── Cielo + tierra rotados por roll y desplazados por pitch ──────────
        p.save()
        p.translate(cx, cy)
        p.rotate(-self._roll)
        p.translate(0, pitch_px)

        ext = r * 2.5  # extensión extra para que no haya huecos al rotar
        # Cielo — gradiente azul marino
        sky_grad = QLinearGradient(0, -ext, 0, 0)
        sky_grad.setColorAt(0, QColor("#0A1F35"))
        sky_grad.setColorAt(1, QColor("#1C4A72"))
        p.fillRect(int(-ext), int(-ext), int(ext * 2), int(ext),
                   QBrush(sky_grad))

        # Tierra — gradiente marrón
        gnd_grad = QLinearGradient(0, 0, 0, ext)
        gnd_grad.setColorAt(0, QColor("#3D2008"))
        gnd_grad.setColorAt(1, QColor("#1A0D04"))
        p.fillRect(int(-ext), 0, int(ext * 2), int(ext),
                   QBrush(gnd_grad))

        # Línea de horizonte
        p.setPen(QPen(QColor("#D0E8FF"), 1.8))
        p.drawLine(int(-ext), 0, int(ext), 0)

        # Marcas de pitch ±5°, ±10°, ±20° (se mueven con pitch+roll)
        pitch_scale = r / 30.0
        p.setFont(QFont("Courier New", 7))
        for deg in (-20, -10, -5, 5, 10, 20):
            y_mark = -deg * pitch_scale
            half_w = 18 if abs(deg) == 10 or abs(deg) == 20 else 10
            p.setPen(QPen(QColor("#D0E8FF"), 1))
            p.drawLine(int(-half_w), int(y_mark), int(half_w), int(y_mark))
            p.setPen(QPen(QColor("#90A4BD"), 1))
            p.drawText(int(half_w + 3), int(y_mark + 4), f"{abs(deg)}")

        p.restore()

        # ── Arco de banco (bank arc) en la parte superior del instrumento ────
        p.setClipping(False)
        p.save()
        p.translate(cx, cy)

        arc_r = r - 4
        p.setPen(QPen(QColor("#2A3F5A"), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        # Marcas de 10°, 20°, 30°, 45°, 60° a cada lado
        for deg in (10, 20, 30, 45, 60):
            for sign in (-1, 1):
                angle_rad = math.radians(sign * deg - 90)
                tick_len  = 6 if deg in (30, 60) else 4
                x1 = arc_r * math.cos(angle_rad)
                y1 = arc_r * math.sin(angle_rad)
                x2 = (arc_r - tick_len) * math.cos(angle_rad)
                y2 = (arc_r - tick_len) * math.sin(angle_rad)
                p.setPen(QPen(QColor("#3A5270"), 1.2))
                p.drawLine(int(x1), int(y1), int(x2), int(y2))

        # Triángulo indicador de banco (rota con _roll)
        p.rotate(-self._roll)
        tri_y = -(arc_r - 2)
        tri = QPolygonF([
            QPointF(-5, tri_y - 8),
            QPointF(5,  tri_y - 8),
            QPointF(0,  tri_y),
        ])
        p.setBrush(QBrush(QColor(C["accent"])))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(tri)

        p.restore()

        # ── Borde del instrumento ────────────────────────────────────────────
        p.setPen(QPen(qc(C["border"]), 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(cx, cy), r, r)

        # ── Índice fijo del vehículo (siempre centrado, no rota) ─────────────
        idx_pen = QPen(QColor(C["amber"]), 2.5)
        idx_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        p.setPen(idx_pen)
        # Alas del avión
        p.drawLine(int(cx) - 28, int(cy), int(cx) - 10, int(cy))
        p.drawLine(int(cx) + 10, int(cy), int(cx) + 28, int(cy))
        p.drawLine(int(cx) - 10, int(cy), int(cx) - 10, int(cy) + 6)
        p.drawLine(int(cx) + 10, int(cy), int(cx) + 10, int(cy) + 6)
        # Centro
        p.drawLine(int(cx) - 3, int(cy), int(cx) + 3, int(cy))
        p.drawLine(int(cx), int(cy) - 5, int(cx), int(cy) + 5)

        p.end()

# ═══════════════════════════════════════════════════════════════════════════════
# DIÁLOGO DE CONEXIÓN
# ═══════════════════════════════════════════════════════════════════════════════

class SerialDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("SIMISPACE — Configurar Enlace Serial")
        self.setModal(True)
        self.setFixedWidth(480)
        self.setStyleSheet(f"""
            QDialog  {{ background: {C['bg']}; color: {C['ink']}; }}
            QLabel   {{ color: {C['ink_dim']}; font-size: 12px; }}
            QComboBox, QSpinBox {{
                background: {C['panel_alt']}; color: {C['ink']};
                border: 1px solid {C['border']}; border-radius: 4px;
                padding: 4px 8px; font-size: 12px; min-height: 28px;
            }}
            QPushButton {{
                background: {C['panel']}; color: {C['accent']};
                border: 1px solid {C['border']}; border-radius: 4px;
                padding: 8px 16px; font-size: 12px;
            }}
            QPushButton:hover {{ background: {C['panel_alt']}; }}
        """)

        self.port_combo    = QComboBox()
        self.baud_spin     = QSpinBox()
        self._sim_selected = False

        self.baud_spin.setRange(1200, 3000000)
        self.baud_spin.setValue(115200)
        self.baud_spin.setSingleStep(9600)

        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(
                f"{port.device}  —  {port.description}", port.device)
        if not ports:
            self.port_combo.addItem("⚠ No se detectaron puertos COM", "")

        layout = QVBoxLayout(self)
        layout.setSpacing(14)
        layout.setContentsMargins(24, 24, 24, 24)

        title = QLabel("CONFIGURAR ENLACE SERIAL")
        title.setStyleSheet(
            f"color: {C['ink']}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
        layout.addWidget(title)

        sub = QLabel("Selecciona el puerto COM donde está conectado el Heltec WiFi LoRa 32.")
        sub.setWordWrap(True)
        layout.addWidget(sub)

        form = QFormLayout()
        form.setSpacing(10)
        form.addRow("Puerto COM:", self.port_combo)
        form.addRow("Baudrate:", self.baud_spin)
        layout.addLayout(form)

        btn_connect = QPushButton("⚡  Conectar al Puerto")
        btn_connect.clicked.connect(self._use_serial)
        layout.addWidget(btn_connect)

        sep = QLabel("— O —")
        sep.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sep.setStyleSheet(f"color: {C['muted']}; font-size: 11px;")
        layout.addWidget(sep)

        btn_sim = QPushButton("▶  Modo Simulación (sin hardware)")
        btn_sim.clicked.connect(self._use_sim)
        layout.addWidget(btn_sim)

    def _use_sim(self):
        self._sim_selected = True
        self.accept()

    def _use_serial(self):
        self._sim_selected = False
        if not self.port_combo.currentData():
            QMessageBox.warning(self, "Sin puerto",
                                "No hay un puerto COM válido seleccionado.")
            return
        self.accept()

    @property
    def selected_port(self) -> str:
        return self.port_combo.currentData() or ""

    @property
    def selected_baud(self) -> int:
        return self.baud_spin.value()

    @property
    def use_sim(self) -> bool:
        return self._sim_selected

# ═══════════════════════════════════════════════════════════════════════════════
# VENTANA PRINCIPAL
# ═══════════════════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("SIMISPACE — Ground Control Station")
        self.resize(1440, 920)
        self.setMinimumSize(960, 640)
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background: {C['bg']}; color: {C['ink']}; }}
            QLabel {{ background: transparent; }}
            QScrollBar:vertical {{
                background: {C['panel_alt']}; width: 6px; border: none;
            }}
            QScrollBar::handle:vertical {{
                background: {C['border']}; border-radius: 3px;
            }}
        """)

        # ── Estado ──────────────────────────────────────────────────────────
        self._connected     = False
        self._worker        = None
        self._met_seconds   = 0
        self._met_running   = False
        self._alt_baseline  = None
        self._alt_trigger   = 2.0        # metros para arrancar el MET
        self._packet_count  = 0

        # ── CSV logging ─────────────────────────────────────────────────────
        self._csv_file      = None
        self._csv_writer    = None
        self._csv_path      = None

        self._build_ui()

        # MET timer (1 Hz, independiente del enlace)
        self._met_timer = QTimer(self)
        self._met_timer.timeout.connect(self._tick_met)
        self._met_timer.start(1000)

        QTimer.singleShot(0, self._open_connection_dialog)

    # ─────────────────────────────────────────────────────────────────────────
    # CONSTRUCCIÓN DE UI
    # ─────────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        ml = QVBoxLayout(root)
        ml.setContentsMargins(12, 12, 12, 12)
        ml.setSpacing(10)

        ml.addWidget(self._build_header())

        body = QWidget()
        bl = QGridLayout(body)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(10)

        # Cámara — columna 0+1, fila 0
        bl.addWidget(self._build_camera_panel(), 0, 0, 1, 2)

        # Telemetría — columna 2, fila 0
        telem_col = QWidget()
        tl = QVBoxLayout(telem_col)
        tl.setContentsMargins(0, 0, 0, 0)
        tl.setSpacing(8)
        self._card_pressure    = TelemetryCard("Pressure",    "Pa",  "pressure",    decimals=0)
        self._card_temperature = TelemetryCard("Temperature", "°C",  "temperature", decimals=1)
        self._card_altitude    = TelemetryCard("Altitude",    "m",   "",            decimals=1)
        self._card_humidity    = TelemetryCard("Humidity",    "%",   "humidity",    decimals=0)
        for c in (self._card_pressure, self._card_temperature,
                  self._card_altitude, self._card_humidity):
            c.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            tl.addWidget(c)
        bl.addWidget(telem_col, 0, 2, 1, 1)

        # Posición — columna 0, fila 1
        bl.addWidget(self._build_position_panel(), 1, 0, 1, 1)
        # Stats — columna 1, fila 1
        bl.addWidget(self._build_stats_panel(), 1, 1, 1, 1)
        # Log — columna 2, fila 1
        self._log = LogConsole()
        bl.addWidget(self._log, 1, 2, 1, 1)

        bl.setColumnStretch(0, 3)
        bl.setColumnStretch(1, 2)
        bl.setColumnStretch(2, 3)
        bl.setRowStretch(0, 3)
        bl.setRowStretch(1, 2)

        ml.addWidget(body, stretch=1)

    def _make_panel(self, title: str):
        frame = QFrame()
        frame.setObjectName("Panel")
        frame.setStyleSheet(f"""
            #Panel {{
                background: {C['panel']};
                border: 1px solid {C['border']};
                border-radius: 8px;
            }}
        """)
        outer = QVBoxLayout(frame)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        hdr = QWidget()
        hdr.setFixedHeight(34)
        hdr.setStyleSheet(f"border-bottom: 1px solid {C['border']};")
        h_lay = QHBoxLayout(hdr)
        h_lay.setContentsMargins(12, 0, 12, 0)
        lbl = QLabel(title.upper())
        lbl.setStyleSheet(
            f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 2px; border: none;")
        h_lay.addWidget(lbl)

        body = QWidget()
        body_lay = QVBoxLayout(body)
        body_lay.setContentsMargins(10, 10, 10, 10)
        body_lay.setSpacing(8)

        outer.addWidget(hdr)
        outer.addWidget(body, stretch=1)
        return frame, body_lay

    def _build_header(self) -> QWidget:
        container = QFrame()
        container.setObjectName("Header")
        container.setStyleSheet(f"""
            #Header {{
                background: {C['panel']};
                border: 1px solid {C['border']};
                border-radius: 8px;
            }}
        """)
        container.setFixedHeight(72)
        layout = QHBoxLayout(container)
        layout.setContentsMargins(16, 0, 16, 0)
        layout.setSpacing(20)

        logo = QLabel("◎")
        logo.setStyleSheet(f"color: {C['accent']}; font-size: 20px;")
        name = QLabel("SIMISPACE")
        name.setStyleSheet(
            f"color: {C['ink']}; font-size: 18px; font-weight: bold; letter-spacing: 3px;")
        sub = QLabel("· GROUND CONTROL — LIVE TELEMETRY")
        sub.setStyleSheet(f"color: {C['ink_dim']}; font-size: 12px;")
        layout.addWidget(logo)
        layout.addWidget(name)
        layout.addWidget(sub)
        layout.addStretch()

        def sep():
            s = QFrame()
            s.setFrameShape(QFrame.Shape.VLine)
            s.setStyleSheet(f"color: {C['border']};")
            return s

        layout.addWidget(sep())

        met_col = QVBoxLayout(); met_col.setSpacing(0)
        met_top = QLabel("MET")
        met_top.setStyleSheet(
            f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 3px;")
        self._lbl_met = QLabel("00:00:00")
        self._lbl_met.setStyleSheet(
            f"color: {C['ink']}; font-size: 28px; font-weight: bold;"
            f" font-family: 'Courier New', monospace;")
        met_col.addWidget(met_top); met_col.addWidget(self._lbl_met)
        layout.addLayout(met_col)

        layout.addWidget(sep())

        st_col = QVBoxLayout(); st_col.setSpacing(2)
        st_top = QLabel("STATUS")
        st_top.setStyleSheet(f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 2px;")
        row = QHBoxLayout()
        self._lbl_status = QLabel("WAITING")
        self._lbl_status.setStyleSheet(
            f"color: {C['muted']}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
        self._dot_status = QLabel("●")
        self._dot_status.setStyleSheet(f"color: {C['muted']}; font-size: 14px;")
        row.addWidget(self._lbl_status); row.addWidget(self._dot_status)
        row.addStretch()
        st_col.addWidget(st_top); st_col.addLayout(row)
        layout.addLayout(st_col)

        layout.addWidget(sep())

        # Indicador de grabación CSV
        self._lbl_rec = QLabel("⏺ REC OFF")
        self._lbl_rec.setStyleSheet(
            f"color: {C['muted']}; font-size: 11px; font-weight: bold;")
        layout.addWidget(self._lbl_rec)

        layout.addWidget(sep())

        self._btn_connect = QPushButton("⚡ Cambiar Conexión")
        self._btn_connect.setFixedHeight(34)
        self._btn_connect.setStyleSheet(f"""
            QPushButton {{
                background: {C['panel_alt']}; color: {C['accent']};
                border: 1px solid {C['border']}; border-radius: 4px;
                padding: 0 14px; font-size: 11px;
            }}
            QPushButton:hover {{ background: {C['border']}; }}
        """)
        self._btn_connect.clicked.connect(self._open_connection_dialog)
        layout.addWidget(self._btn_connect)
        return container

    def _build_camera_panel(self) -> QFrame:
        frame, body = self._make_panel("Onboard Camera Feed — Live")

        class CamPlaceholder(QWidget):
            def paintEvent(self_inner, event):
                p = QPainter(self_inner)
                p.setRenderHint(QPainter.RenderHint.Antialiasing)
                w, h = self_inner.width(), self_inner.height()
                p.fillRect(0, 0, w, h, QColor("#040810"))
                pen = QPen(QColor(C["border"]), 1)
                p.setPen(pen)
                for x in range(0, w, 30):
                    p.drawLine(x, 0, x, h)
                for y in range(0, h, 30):
                    p.drawLine(0, y, w, y)
                grad = QLinearGradient(0, h*0.5, 0, h)
                c1 = QColor(C["accent"]); c1.setAlpha(40)
                c2 = QColor(C["accent"]); c2.setAlpha(0)
                grad.setColorAt(0, c2); grad.setColorAt(1, c1)
                p.fillRect(0, int(h*0.5), w, h, QBrush(grad))
                cl = 20
                cp = QPen(QColor("rgba(255,255,255,150)"), 1.5)
                p.setPen(cp)
                for cx_, cy_, dx, dy in [(0,0,1,1),(w,0,-1,1),(0,h,1,-1),(w,h,-1,-1)]:
                    p.drawLine(cx_, cy_, cx_+dx*cl, cy_)
                    p.drawLine(cx_, cy_, cx_, cy_+dy*cl)
                cx_, cy_ = w//2, h//2
                mp = QPen(QColor("rgba(255,255,255,150)"), 1)
                p.setPen(mp)
                p.drawLine(cx_-40, cy_, cx_+40, cy_)
                p.drawLine(cx_, cy_-40, cx_, cy_+40)
                p.drawEllipse(QPointF(cx_, cy_), 22, 22)
                p.end()

        self._cam_widget = CamPlaceholder()
        self._cam_widget.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._cam_widget.setMinimumHeight(240)

        ovl = QHBoxLayout()
        self._cam_alt_lbl  = QLabel("ALT: 0.0 m")
        self._cam_hdg_lbl  = QLabel("HDG: 000°")
        self._cam_live_lbl = QLabel("● WAITING")
        for lbl in (self._cam_alt_lbl, self._cam_hdg_lbl, self._cam_live_lbl):
            lbl.setStyleSheet(
                f"color: {C['ink_dim']}; font-family: 'Courier New'; font-size: 11px;")
        ovl.addWidget(self._cam_alt_lbl)
        ovl.addWidget(self._cam_hdg_lbl)
        ovl.addStretch()
        ovl.addWidget(self._cam_live_lbl)

        body.addWidget(self._cam_widget, stretch=1)
        body.addLayout(ovl)
        return frame

    def _build_position_panel(self) -> QFrame:
        frame, body = self._make_panel("Position & Trajectory")

        self._pos_labels = {}
        pos_grid = QGridLayout()
        pos_grid.setSpacing(4)
        for i, key in enumerate(["LAT", "LON", "HDG", "ROLL", "PITCH"]):
            lk = QLabel(key)
            lk.setStyleSheet(
                f"color: {C['muted']}; font-size: 10px; letter-spacing: 2px;")
            lv = QLabel("—")
            lv.setStyleSheet(
                f"color: {C['accent']}; font-size: 11px; font-family: 'Courier New';")
            lv.setAlignment(Qt.AlignmentFlag.AlignRight)
            pos_grid.addWidget(lk, i, 0)
            pos_grid.addWidget(lv, i, 1)
            self._pos_labels[key] = lv
        body.addLayout(pos_grid)

        viz = QHBoxLayout(); viz.setSpacing(8)

        tc = QVBoxLayout()
        tl = QLabel("TRAJECTORY")
        tl.setStyleSheet(
            f"color: {C['muted']}; font-size: 9px; letter-spacing: 2px;")
        tl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._trajectory = TrajectoryWidget()
        tc.addWidget(self._trajectory); tc.addWidget(tl)

        ac = QVBoxLayout()
        al = QLabel("ATTITUDE  (interpolado 60 fps)")
        al.setStyleSheet(
            f"color: {C['muted']}; font-size: 9px; letter-spacing: 1px;")
        al.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._attitude = AttitudeWidget()
        ac.addWidget(self._attitude); ac.addWidget(al)

        viz.addLayout(tc); viz.addLayout(ac)
        body.addLayout(viz)
        return frame

    def _build_stats_panel(self) -> QFrame:
        frame, body = self._make_panel("Vehicle Status")
        self._stat_signal  = StatCard("Signal")
        self._stat_link    = StatCard("Comm Link")
        self._stat_packets = StatCard("Packets RX")
        self._stat_port    = StatCard("Port")
        self._stat_csv     = StatCard("CSV Log")
        for s in (self._stat_signal, self._stat_link, self._stat_packets,
                  self._stat_port, self._stat_csv):
            s.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            body.addWidget(s)
        self._stat_port.set_value("—", C["muted"])
        self._stat_signal.set_value("—", C["muted"])
        self._stat_link.set_value("DISCONNECTED", C["red"])
        self._stat_packets.set_value("0", C["ink_dim"])
        self._stat_csv.set_value("OFF", C["muted"])
        return frame

    # ─────────────────────────────────────────────────────────────────────────
    # CONEXIÓN
    # ─────────────────────────────────────────────────────────────────────────

    def _open_connection_dialog(self):
        self._stop_worker()
        self._close_csv()
        dlg = SerialDialog(self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return

        if dlg.use_sim:
            self._log.log("INFO", "Iniciando modo simulación (sin hardware)",
                          self._fmt_met())
            self._worker = SimWorker()
            self._stat_port.set_value("SIMULACIÓN", C["accent"])
        else:
            port = dlg.selected_port
            baud = dlg.selected_baud
            self._log.log("INFO", f"Conectando a {port} @ {baud} bps…",
                          self._fmt_met())
            self._worker = SerialWorker(port, baud)
            self._stat_port.set_value(f"{port} @ {baud}", C["accent"])

        self._worker.packet_received.connect(self._on_packet)
        self._worker.raw_line.connect(self._on_raw_line)
        self._worker.connection_changed.connect(self._on_connection_changed)
        self._worker.error.connect(self._on_error)
        self._worker.start()

        # Reset estado de misión
        self._met_seconds  = 0
        self._met_running  = False
        self._alt_baseline = None
        self._packet_count = 0
        self._lbl_met.setText("00:00:00")

    def _stop_worker(self):
        if self._worker:
            self._worker.stop()
            self._worker = None

    # ─────────────────────────────────────────────────────────────────────────
    # CSV LOGGING
    # ─────────────────────────────────────────────────────────────────────────

    def _open_csv(self):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.expanduser(
            f"~/Downloads/simispace_telem_{ts}.csv")
        self._csv_file   = open(self._csv_path, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "timestamp", "met_s",
            "pressure_pa", "temperature_c", "altitude_m", "humidity_pct",
            "lat", "lon", "roll_deg", "pitch_deg", "heading_deg",
        ])
        self._log.log("INFO", f"📄 Grabando datos → {self._csv_path}",
                      self._fmt_met())
        self._stat_csv.set_value("GRABANDO", C["green"])
        self._lbl_rec.setText("⏺ REC")
        self._lbl_rec.setStyleSheet(
            f"color: {C['red']}; font-size: 11px; font-weight: bold;")

    def _write_csv_row(self, pkt: TelemetryPacket):
        if not self._csv_writer:
            return
        self._csv_writer.writerow([
            time.strftime("%Y-%m-%d %H:%M:%S"),
            self._met_seconds,
            round(pkt.pressure, 2),
            round(pkt.temperature, 2),
            round(pkt.altitude, 2),
            round(pkt.humidity, 2),
            round(pkt.lat, 6),
            round(pkt.lon, 6),
            round(pkt.roll, 2),
            round(pkt.pitch, 2),
            round(pkt.heading, 2),
        ])
        self._csv_file.flush()   # escribe al disco ahora, no al cerrar

    def _close_csv(self):
        if self._csv_file:
            self._csv_file.close()
            self._csv_file   = None
            self._csv_writer = None
            self._log.log("INFO", f"✅ CSV guardado en: {self._csv_path}",
                          self._fmt_met())
            self._stat_csv.set_value("GUARDADO", C["ink_dim"])
            self._lbl_rec.setText("⏺ REC OFF")
            self._lbl_rec.setStyleSheet(
                f"color: {C['muted']}; font-size: 11px; font-weight: bold;")

    # ─────────────────────────────────────────────────────────────────────────
    # SLOTS
    # ─────────────────────────────────────────────────────────────────────────

    def _on_connection_changed(self, connected: bool):
        self._connected = connected
        if connected:
            self._lbl_status.setText("NOMINAL")
            col = C["green"]
            self._stat_link.set_value("ACTIVE", C["accent"])
            self._stat_signal.set_value("GOOD", C["green"])
            # Abrir CSV al conectar
            self._open_csv()
        else:
            self._lbl_status.setText("DISCONNECTED")
            col = C["red"]
            self._stat_link.set_value("LOST", C["red"])
            self._stat_signal.set_value("LOST", C["red"])
            self._cam_live_lbl.setText("○ SIGNAL LOST")
            self._cam_live_lbl.setStyleSheet(
                f"color: {C['red']}; font-family: 'Courier New'; font-size: 11px;")

        self._lbl_status.setStyleSheet(
            f"color: {col}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
        self._dot_status.setStyleSheet(f"color: {col}; font-size: 14px;")

    def _on_error(self, msg: str):
        self._log.log("CRIT", msg, self._fmt_met())

    def _on_raw_line(self, line: str):
        if not line.startswith("TELEM"):
            self._log.log("RAW", line, self._fmt_met())

    def _on_packet(self, pkt: TelemetryPacket):
        """Actualiza todos los widgets de la UI con el paquete recibido."""

        self._packet_count += 1
        self._stat_packets.set_value(str(self._packet_count), C["ink_dim"])

        # Overlay cámara
        self._cam_alt_lbl.setText(f"ALT: {pkt.altitude:.1f} m")
        self._cam_hdg_lbl.setText(f"HDG: {pkt.heading:.0f}°")
        self._cam_live_lbl.setText("● LIVE")
        self._cam_live_lbl.setStyleSheet(
            f"color: {C['accent']}; font-family: 'Courier New'; font-size: 11px;")

        # Tarjetas de telemetría
        self._card_pressure.update_value(pkt.pressure)
        self._card_temperature.update_value(pkt.temperature)
        self._card_altitude.update_value(pkt.altitude)
        self._card_humidity.update_value(pkt.humidity)

        # Posición textual
        self._pos_labels["LAT"].setText(f"{pkt.lat:.5f}°")
        self._pos_labels["LON"].setText(f"{pkt.lon:.5f}°")
        self._pos_labels["HDG"].setText(f"{pkt.heading:.1f}°")
        self._pos_labels["ROLL"].setText(f"{pkt.roll:.1f}°")
        self._pos_labels["PITCH"].setText(f"{pkt.pitch:.1f}°")

        # Trayectoria (recibe lat/lon reales)
        self._trajectory.push(pkt.lat, pkt.lon, pkt.heading)

        # Horizonte artificial — solo actualizamos el OBJETIVO;
        # el widget se encarga de interpolarlo a 60 fps internamente.
        self._attitude.set_target(pkt.roll, pkt.pitch)

        # Status maestro
        statuses = [
            get_status("pressure", pkt.pressure),
            get_status("temperature", pkt.temperature),
            get_status("humidity", pkt.humidity),
        ]
        if "critical" in statuses:
            master_col = C["red"];   master_txt = "CRITICAL"
        elif "caution" in statuses:
            master_col = C["amber"]; master_txt = "CAUTION"
        else:
            master_col = C["green"]; master_txt = "NOMINAL"

        self._lbl_status.setText(master_txt)
        self._lbl_status.setStyleSheet(
            f"color: {master_col}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
        self._dot_status.setStyleSheet(f"color: {master_col}; font-size: 14px;")

        # ── LÓGICA DEL MET ───────────────────────────────────────────────────
        # El MET arranca cuando se detecta Δaltitud > alt_trigger metros.
        # Una vez iniciado, sigue corriendo aunque la altitud baje.
        if not self._met_running:
            if self._alt_baseline is None:
                self._alt_baseline = pkt.altitude
                self._log.log(
                    "INFO",
                    f"Baseline de altitud: {pkt.altitude:.1f} m — "
                    f"MET arrancará con Δ > {self._alt_trigger} m",
                    "00:00:00")
            else:
                delta = pkt.altitude - self._alt_baseline
                if delta >= self._alt_trigger:
                    self._met_running = True
                    self._log.log(
                        "INFO",
                        f"🚀 Despegue detectado! Δaltitud = {delta:.2f} m → MET iniciado",
                        "00:00:00")

        # ── GRABAR EN CSV ────────────────────────────────────────────────────
        self._write_csv_row(pkt)

    # ─────────────────────────────────────────────────────────────────────────
    # MET TIMER
    # ─────────────────────────────────────────────────────────────────────────

    def _tick_met(self):
        if self._met_running:
            self._met_seconds += 1
            self._lbl_met.setText(self._fmt_met())

    def _fmt_met(self) -> str:
        s = self._met_seconds
        return f"{s//3600:02d}:{(s%3600)//60:02d}:{s%60:02d}"

    # ─────────────────────────────────────────────────────────────────────────
    # CIERRE
    # ─────────────────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        self._stop_worker()
        self._close_csv()
        event.accept()

# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    app = QApplication(sys.argv)
    app.setApplicationName("SIMISPACE GCS")
    app.setFont(QFont("Segoe UI", 10))
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
