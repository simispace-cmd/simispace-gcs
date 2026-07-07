"""
╔══════════════════════════════════════════════════════════════════════════════╗
║   SIMISPACE — GROUND CONTROL STATION v3                                     ║
║   Estación Terrena en tiempo real · PyQt6 + pyserial                        ║
╠══════════════════════════════════════════════════════════════════════════════╣
║   DEPENDENCIAS (instalar UNA sola vez):                                     ║
║     pip3 install PyQt6 pyserial opencv-python pyqtgraph                     ║
║                                                                             ║
║   CORRER:                                                                   ║
║     python3 simispace_gcs.py                                                ║
║                                                                             ║
║   FORMATO SERIAL ESPERADO DEL HELTEC (115200 bps):                         ║
║     TELEM,<presión_hPa>,<temp_°C>,<altitud_m>,<humedad_%>,                 ║
║           <roll_°>,<pitch_°>,<heading_°>[,<rssi_dBm>]\n                    ║
║   Ejemplo:                                                                  ║
║     TELEM,682.46,24.46,3335.99,49.40,2.73,0.96,107.64,-78                  ║
║                                                                             ║
║   RSSI (opcional) — agregar al final del paquete en el receptor:           ║
║     LoRa.print(","); LoRa.println(LoRa.packetRssi());                      ║
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
import subprocess
from collections import deque
from typing import Optional

import serial
import serial.tools.list_ports

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QComboBox, QPushButton, QTextEdit,
    QSizePolicy, QDialog, QFormLayout,
    QSpinBox, QDialogButtonBox, QMessageBox, QDoubleSpinBox,
    QFileDialog, QProgressBar, QSlider,
)
from PyQt6.QtCore import (
    Qt, QThread, pyqtSignal, QTimer, QPointF,
)
from PyQt6.QtGui import (
    QColor, QPainter, QPen, QBrush, QPainterPath, QFont,
    QLinearGradient, QPolygonF, QRadialGradient, QImage, QPixmap,
)

# pyqtgraph — gráfica de altitud en tiempo real (opcional, degrada a nada si no instalado)
try:
    import pyqtgraph as pg
    pg.setConfigOption('background', '#101B2E')
    pg.setConfigOption('foreground', '#90A4BD')
    PYQTGRAPH_AVAILABLE = True
except ImportError:
    PYQTGRAPH_AVAILABLE = False

# reportlab — exportar PDF de reporte de misión (opcional)
try:
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.units import cm
    from reportlab.lib import colors as rl_colors
    from reportlab.platypus import (
        SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, HRFlowable
    )
    REPORTLAB_AVAILABLE = True
except ImportError:
    REPORTLAB_AVAILABLE = False

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
    "pressure":    {"caution_lo": 50,   "caution_hi": 9999,
                    "critical_lo": 10,  "critical_hi": 99999},
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
                 "lat", "lon", "roll", "pitch", "heading", "rssi", "timestamp")
    def __init__(self):
        self.pressure    = 1013.25
        self.temperature = 20.0
        self.altitude    = 0.0
        self.humidity    = 50.0
        self.lat         = 19.4326
        self.lon         = -99.1332
        self.roll        = 0.0
        self.pitch       = 0.0
        self.heading     = 0.0
        self.rssi        = None   # dBm — None si el Heltec no lo manda
        self.timestamp   = time.time()

# ═══════════════════════════════════════════════════════════════════════════════
# PARSER
# ═══════════════════════════════════════════════════════════════════════════════

def parse_line(line: str) -> Optional[TelemetryPacket]:
    """
    Parsea una línea TELEM del Heltec.
    Formato (sin GPS, RSSI opcional al final):
        TELEM,<presión_hPa>,<temp_°C>,<altitud_m>,<humedad_%>,<roll>,<pitch>,<heading>[,<rssi>]
    """
    line = line.strip().rstrip(",")
    if not line.startswith("TELEM"):
        return None
    parts = line.split(",")
    if len(parts) < 5:
        return None
    try:
        pkt = TelemetryPacket()
        pkt.pressure    = float(parts[1])
        pkt.temperature = float(parts[2])
        pkt.altitude    = float(parts[3])
        pkt.humidity    = float(parts[4])
        pkt.lat         = 19.43260
        pkt.lon         = -99.13320
        pkt.roll        = float(parts[5]) if len(parts) > 5 and parts[5] else 0.0
        pkt.pitch       = float(parts[6]) if len(parts) > 6 and parts[6] else 0.0
        pkt.heading     = float(parts[7]) if len(parts) > 7 and parts[7] else 0.0
        pkt.rssi        = float(parts[8]) if len(parts) > 8 and parts[8] else None
        pkt.timestamp   = time.time()
        return pkt
    except (ValueError, IndexError):
        return None

# ═══════════════════════════════════════════════════════════════════════════════
# WORKER DE CÁMARA (captura frames de la webcam en un hilo separado)
# Requiere: pip3 install opencv-python
# Si opencv no está instalado, la cámara queda desactivada sin crashear.
# ═══════════════════════════════════════════════════════════════════════════════

try:
    import cv2 as _cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

class CameraWorker(QThread):
    """
    Captura frames de la webcam a ~30 fps y los emite como QImage.
    Se detiene limpiamente al llamar stop().
    """
    frame_ready = pyqtSignal(QImage)
    cam_error   = pyqtSignal(str)

    def __init__(self, cam_index: int = 0):
        super().__init__()
        self._cam_index = cam_index
        self._running   = True

    def run(self):
        if not CV2_AVAILABLE:
            self.cam_error.emit("opencv-python no instalado. Corre: pip3 install opencv-python")
            return
        cap = _cv2.VideoCapture(self._cam_index)
        if not cap.isOpened():
            self.cam_error.emit("No se pudo abrir la cámara. Verifica permisos en Ajustes → Privacidad → Cámara.")
            return
        while self._running:
            ok, frame = cap.read()
            if not ok:
                break
            # BGR → RGB
            rgb = _cv2.cvtColor(frame, _cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            img = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
            self.frame_ready.emit(img.copy())
            self.msleep(33)   # ~30 fps
        cap.release()

    def stop(self):
        self._running = False
        self.wait(2000)

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
        self._pressure    = 1013.25
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
            self._pressure    = self._rw(self._pressure,    600, 1060, 2.2)
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
            f"color: {C['ink']}; font-size: 20px; font-weight: bold;")

        self._lbl_unit = QLabel(unit)
        self._lbl_unit.setStyleSheet(
            f"color: {C['ink_dim']}; font-size: 11px;")

        self._lbl_status = QLabel("NOMINAL")
        self._lbl_status.setStyleSheet(
            f"color: {C['accent']}; font-size: 9px; font-weight: bold; letter-spacing: 2px;")

        val_row = QHBoxLayout()
        val_row.setSpacing(4)
        val_row.addWidget(self._lbl_value)
        val_row.addWidget(self._lbl_unit)
        val_row.addStretch()

        left.addWidget(self._lbl_title)
        left.addLayout(val_row)
        left.addWidget(self._lbl_status)

        self._spark = SparklineWidget(color=C["accent"])
        self._spark.setFixedWidth(72)

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
                f"color: {col_val}; font-size: 22px; font-weight: bold;")
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
    """Tarjeta de estado compacta: etiqueta arriba, valor abajo. Fondo sólido."""
    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self.setObjectName("StatCard")
        # Fondo sólido explícito — impide que texto de otras tarjetas sangre
        self.setAutoFillBackground(True)
        self.setFixedHeight(40)
        self.setStyleSheet(f"""
            #StatCard {{
                background-color: {C['panel_alt']};
                border: 1px solid {C['border']};
                border-radius: 6px;
            }}
            QLabel {{
                background-color: transparent;
                border: none;
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 4, 10, 4)
        layout.setSpacing(1)

        self._lbl_label = QLabel(label.upper())
        self._lbl_label.setStyleSheet(
            f"color: {C['muted']}; font-size: 9px; letter-spacing: 1px;")

        self._lbl_value = QLabel("—")
        self._lbl_value.setStyleSheet(
            f"color: {C['accent']}; font-size: 12px; font-weight: bold;")

        layout.addWidget(self._lbl_label)
        layout.addWidget(self._lbl_value)

    def set_value(self, text: str, color: str = C["accent"]):
        self._lbl_value.setText(text)
        self._lbl_value.setStyleSheet(
            f"color: {color}; font-size: 12px; font-weight: bold;"
            f" background-color: transparent;")

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
        header.setFixedHeight(28)
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
        self.setMinimumSize(120, 110)

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

        self.setMinimumSize(120, 110)
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
        self.resize(1280, 780)
        self.setMinimumSize(900, 580)
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
        self._connected          = False
        self._worker             = None
        self._met_seconds        = 0
        self._met_running        = False
        self._met_started        = False
        self._packet_count       = 0
        self._last_master_status = "nominal"

        # ── CSV logging ─────────────────────────────────────────────────────
        self._csv_file      = None
        self._csv_writer    = None
        self._csv_path      = None

        # ── Datos de altitud para la gráfica en tiempo real ─────────────────
        self._alt_times : deque = deque(maxlen=300)
        self._alt_values: deque = deque(maxlen=300)
        self._mission_t0: Optional[float] = None

        # ── Detección de fases de misión ─────────────────────────────────────
        self._phase           = "PRE-LAUNCH"
        self._alt_history     : deque = deque(maxlen=5)  # últimos 5 valores para tendencia
        self._apogee_detected = False
        self._max_altitude    = 0.0

        # ── Estadísticas de misión (para el PDF) ────────────────────────────
        self._stats = {
            "start_time":   None,    # datetime de inicio de misión
            "max_altitude": 0.0,
            "min_temp":     9999.0,
            "max_temp":     -9999.0,
            "min_pressure": 9999.0,
            "max_pressure": 0.0,
            "packets_rx":   0,
        }

        # ── Replay de misión ─────────────────────────────────────────────────
        self._replay_packets : list = []
        self._replay_index   : int  = 0
        self._replay_speed   : int  = 1
        self._replay_timer   : Optional[QTimer] = None
        self._replay_mode    : bool = False

        # ── T-MINUS COUNTDOWN ────────────────────────────────────────────────
        self._tminus_seconds = 48 * 3600
        self._tminus_running = False

        self._build_ui()

        # MET timer (1 Hz)
        self._met_timer = QTimer(self)
        self._met_timer.timeout.connect(self._tick_met)
        self._met_timer.start(1000)

        # T-MINUS timer (1 Hz)
        self._tminus_timer = QTimer(self)
        self._tminus_timer.timeout.connect(self._tick_tminus)
        self._tminus_timer.start(1000)

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
        ml.addWidget(self._build_phase_bar())

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
        self._card_pressure    = TelemetryCard("Pressure",    "hPa", "pressure",    decimals=1)
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

        # Gráfica de altitud — fila 2, ancho completo

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
        hdr.setFixedHeight(28)
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
        container.setFixedHeight(56)
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
            f"color: {C['ink']}; font-size: 22px; font-weight: bold;"
            f" font-family: 'Courier New', monospace;")
        met_col.addWidget(met_top); met_col.addWidget(self._lbl_met)

        self._btn_met_start = QPushButton("▶ START MET")
        self._btn_met_start.setFixedHeight(22)
        self._btn_met_start.setStyleSheet(f"""
            QPushButton {{
                background: {C['panel_alt']}; color: {C['green']};
                border: 1px solid {C['border']}; border-radius: 3px;
                font-size: 10px; font-weight: bold;
            }}
            QPushButton:hover {{ background: {C['border']}; }}
        """)
        self._btn_met_start.clicked.connect(self._manual_start_met)
        met_col.addWidget(self._btn_met_start)
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

        btn_replay = QPushButton("▶ REPLAY")
        btn_replay.setFixedHeight(34)
        btn_replay.setStyleSheet(f"""
            QPushButton {{
                background: {C['panel_alt']}; color: {C['ink_dim']};
                border: 1px solid {C['border']}; border-radius: 4px;
                padding: 0 14px; font-size: 11px;
            }}
            QPushButton:hover {{ background: {C['border']}; }}
        """)
        btn_replay.clicked.connect(self._open_replay_dialog)
        layout.addWidget(btn_replay)

        btn_pdf = QPushButton("📄 PDF")
        btn_pdf.setFixedHeight(34)
        btn_pdf.setStyleSheet(f"""
            QPushButton {{
                background: {C['panel_alt']}; color: {C['ink_dim']};
                border: 1px solid {C['border']}; border-radius: 4px;
                padding: 0 14px; font-size: 11px;
            }}
            QPushButton:hover {{ background: {C['border']}; }}
        """)
        btn_pdf.clicked.connect(self._export_pdf)
        layout.addWidget(btn_pdf)

        return container

    # ─────────────────────────────────────────────────────────────────────────
    # BARRA T-MINUS
    # ─────────────────────────────────────────────────────────────────────────

    def _build_tminus_bar(self) -> QFrame:
        container = QFrame()
        container.setObjectName("TMinusBar")
        container.setStyleSheet(f"""
            #TMinusBar {{
                background: {C['panel']};
                border: 1px solid {C['border']};
                border-radius: 8px;
            }}
        """)
        container.setFixedHeight(40)
        layout = QHBoxLayout(container)
        layout.setContentsMargins(12, 0, 12, 0)
        layout.setSpacing(14)

        lbl = QLabel("MISSION COUNTDOWN")
        lbl.setStyleSheet(f"color: {C['muted']}; font-size: 10px; letter-spacing: 2px;")
        layout.addWidget(lbl)

        self._lbl_tminus = QLabel(self._fmt_tminus())
        self._lbl_tminus.setStyleSheet(
            f"color: {C['accent']}; font-size: 22px; font-weight: bold;"
            f" font-family: 'Courier New', monospace;")
        layout.addWidget(self._lbl_tminus)
        layout.addStretch()

        lbl_h = QLabel("HORAS:")
        lbl_h.setStyleSheet(f"color: {C['ink_dim']}; font-size: 11px;")
        layout.addWidget(lbl_h)

        self._spin_tminus_hours = QDoubleSpinBox()
        self._spin_tminus_hours.setRange(0.0, 999.0)
        self._spin_tminus_hours.setDecimals(1)
        self._spin_tminus_hours.setSingleStep(1.0)
        self._spin_tminus_hours.setValue(48.0)
        self._spin_tminus_hours.setSuffix(" h")
        self._spin_tminus_hours.setFixedWidth(90)
        self._spin_tminus_hours.setStyleSheet(f"""
            QDoubleSpinBox {{
                background: {C['panel_alt']}; color: {C['ink']};
                border: 1px solid {C['border']}; border-radius: 4px;
                padding: 4px 6px; font-size: 12px;
            }}
        """)
        self._spin_tminus_hours.valueChanged.connect(self._on_tminus_hours_changed)
        layout.addWidget(self._spin_tminus_hours)

        self._btn_tminus = QPushButton("▶  START")
        self._btn_tminus.setFixedHeight(32)
        self._btn_tminus.setFixedWidth(110)
        self._btn_tminus.setStyleSheet(f"""
            QPushButton {{
                background: {C['panel_alt']}; color: {C['green']};
                border: 1px solid {C['border']}; border-radius: 4px;
                font-size: 12px; font-weight: bold;
            }}
            QPushButton:hover {{ background: {C['border']}; }}
        """)
        self._btn_tminus.clicked.connect(self._toggle_tminus)
        layout.addWidget(self._btn_tminus)

        btn_reset = QPushButton("RESET")
        btn_reset.setFixedHeight(32)
        btn_reset.setFixedWidth(72)
        btn_reset.setStyleSheet(f"""
            QPushButton {{
                background: {C['panel_alt']}; color: {C['ink_dim']};
                border: 1px solid {C['border']}; border-radius: 4px;
                font-size: 11px;
            }}
            QPushButton:hover {{ background: {C['border']}; }}
        """)
        btn_reset.clicked.connect(self._reset_tminus)
        layout.addWidget(btn_reset)
        return container

    def _fmt_tminus(self) -> str:
        s = max(0, self._tminus_seconds)
        sign = "-" if self._tminus_seconds > 0 else "+"
        return f"T{sign}{s//3600:02d}:{(s%3600)//60:02d}:{s%60:02d}"

    def _tick_tminus(self):
        if self._tminus_running and self._tminus_seconds > 0:
            self._tminus_seconds -= 1
            self._lbl_tminus.setText(self._fmt_tminus())
            if self._tminus_seconds == 0:
                self._tminus_running = False
                self._btn_tminus.setText("▶  START")
                self._log.log("WARN", "T-0 alcanzado — countdown finalizado",
                              self._fmt_met())
                self._start_met()

    def _on_tminus_hours_changed(self, value: float):
        if not self._tminus_running:
            self._tminus_seconds = int(value * 3600)
            self._lbl_tminus.setText(self._fmt_tminus())

    def _toggle_tminus(self):
        if self._tminus_running:
            self._tminus_running = False
            self._btn_tminus.setText("▶  RESUME")
            self._btn_tminus.setStyleSheet(f"""
                QPushButton {{
                    background: {C['panel_alt']}; color: {C['amber']};
                    border: 1px solid {C['amber']}; border-radius: 4px;
                    font-size: 12px; font-weight: bold;
                }}
                QPushButton:hover {{ background: {C['border']}; }}
            """)
            self._log.log("WARN",
                f"⏸ HOLD — countdown detenido en {self._fmt_tminus()}",
                self._fmt_met())
        else:
            if self._tminus_seconds <= 0:
                self._tminus_seconds = int(self._spin_tminus_hours.value() * 3600)
            self._tminus_running = True
            self._btn_tminus.setText("⏸  HOLD")
            self._btn_tminus.setStyleSheet(f"""
                QPushButton {{
                    background: {C['panel_alt']}; color: {C['green']};
                    border: 1px solid {C['border']}; border-radius: 4px;
                    font-size: 12px; font-weight: bold;
                }}
                QPushButton:hover {{ background: {C['border']}; }}
            """)
            self._log.log("INFO",
                f"Countdown iniciado/reanudado: {self._fmt_tminus()}",
                self._fmt_met())

    def _reset_tminus(self):
        self._tminus_running = False
        self._tminus_seconds = int(self._spin_tminus_hours.value() * 3600)
        self._lbl_tminus.setText(self._fmt_tminus())
        self._btn_tminus.setText("▶  START")
        self._btn_tminus.setStyleSheet(f"""
            QPushButton {{
                background: {C['panel_alt']}; color: {C['green']};
                border: 1px solid {C['border']}; border-radius: 4px;
                font-size: 12px; font-weight: bold;
            }}
            QPushButton:hover {{ background: {C['border']}; }}
        """)
        self._log.log("INFO", "Countdown reiniciado", self._fmt_met())

    # ─────────────────────────────────────────────────────────────────────────
    # BARRA DE FASES DE MISIÓN
    # ─────────────────────────────────────────────────────────────────────────

    PHASES = ["PRE-LAUNCH", "ASCENT", "APOGEE", "DESCENT", "RECOVERY"]

    def _build_phase_bar(self) -> QFrame:
        container = QFrame()
        container.setObjectName("PhaseBar")
        container.setStyleSheet(f"""
            #PhaseBar {{
                background: {C['panel']};
                border: 1px solid {C['border']};
                border-radius: 8px;
            }}
        """)
        container.setFixedHeight(36)
        layout = QHBoxLayout(container)
        layout.setContentsMargins(12, 0, 12, 0)
        layout.setSpacing(4)

        lbl = QLabel("MISSION PHASE")
        lbl.setStyleSheet(
            f"color: {C['muted']}; font-size: 10px; letter-spacing: 2px;")
        layout.addWidget(lbl)
        layout.addSpacing(8)

        self._phase_btns = {}
        for i, phase in enumerate(self.PHASES):
            btn = QLabel(phase)
            btn.setAlignment(Qt.AlignmentFlag.AlignCenter)
            btn.setFixedHeight(20)
            btn.setMinimumWidth(90)
            btn.setStyleSheet(f"""
                QLabel {{
                    background: {C['panel_alt']};
                    color: {C['muted']};
                    border: 1px solid {C['border']};
                    border-radius: 4px;
                    font-size: 10px;
                    font-weight: bold;
                    letter-spacing: 1px;
                    padding: 0 8px;
                }}
            """)
            layout.addWidget(btn)
            self._phase_btns[phase] = btn

            if i < len(self.PHASES) - 1:
                arrow = QLabel("→")
                arrow.setStyleSheet(f"color: {C['border']}; font-size: 12px;")
                layout.addWidget(arrow)

        layout.addStretch()
        self._set_phase("PRE-LAUNCH")
        return container

    def _set_phase(self, phase: str):
        """Actualiza la fase visual y loggea el cambio si es nuevo."""
        if phase == self._phase and phase != "PRE-LAUNCH":
            return
        old_phase = self._phase
        self._phase = phase

        phase_colors = {
            "PRE-LAUNCH": C["muted"],
            "ASCENT":     C["accent"],
            "APOGEE":     C["amber"],
            "DESCENT":    C["ink"],
            "RECOVERY":   C["green"],
        }
        for p, btn in self._phase_btns.items():
            if p == phase:
                col = phase_colors.get(p, C["accent"])
                btn.setStyleSheet(f"""
                    QLabel {{
                        background: {col};
                        color: {C['bg']};
                        border: 1px solid {col};
                        border-radius: 4px;
                        font-size: 10px;
                        font-weight: bold;
                        letter-spacing: 1px;
                        padding: 0 8px;
                    }}
                """)
            else:
                btn.setStyleSheet(f"""
                    QLabel {{
                        background: {C['panel_alt']};
                        color: {C['muted']};
                        border: 1px solid {C['border']};
                        border-radius: 4px;
                        font-size: 10px;
                        font-weight: bold;
                        letter-spacing: 1px;
                        padding: 0 8px;
                    }}
                """)

        if old_phase != phase:
            self._log.log("INFO", f"Fase de misión: {old_phase} → {phase}",
                          self._fmt_met())

    def _detect_phase(self, altitude: float):
        """
        Detecta la fase de misión basándose en la tendencia de altitud.
        Requiere al menos 4 puntos para tomar una decisión.
        """
        self._alt_history.append(altitude)
        self._max_altitude = max(self._max_altitude, altitude)

        if len(self._alt_history) < 4:
            return

        deltas = [self._alt_history[i] - self._alt_history[i-1]
                  for i in range(1, len(self._alt_history))]
        avg_delta = sum(deltas) / len(deltas)

        if not self._met_started:
            self._set_phase("PRE-LAUNCH")
        elif avg_delta > 0.5:
            self._set_phase("ASCENT")
            self._apogee_detected = False
        elif avg_delta < -0.5:
            if not self._apogee_detected and self._phase == "ASCENT":
                self._set_phase("APOGEE")
                self._apogee_detected = True
                QTimer.singleShot(3000, lambda: self._set_phase("DESCENT"))
            elif self._apogee_detected:
                self._set_phase("DESCENT")
        elif abs(avg_delta) <= 0.5 and self._phase == "DESCENT":
            self._set_phase("RECOVERY")

    def _build_camera_panel(self) -> QFrame:
        frame, body = self._make_panel("Onboard Camera Feed — Live")

        self._cam_label = QLabel()
        self._cam_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._cam_label.setMinimumHeight(180)
        self._cam_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._cam_label.setStyleSheet("background: #040810;")

        self._draw_cam_placeholder()

        self._cam_worker = CameraWorker(cam_index=0)
        self._cam_worker.frame_ready.connect(self._on_cam_frame)
        self._cam_worker.cam_error.connect(self._on_cam_error)
        self._cam_worker.start()

        ovl = QHBoxLayout()
        self._cam_alt_lbl  = QLabel("ALT: 0.0 m")
        self._cam_hdg_lbl  = QLabel("HDG: 000°")
        self._cam_live_lbl = QLabel("● CAM INIT")
        for lbl in (self._cam_alt_lbl, self._cam_hdg_lbl, self._cam_live_lbl):
            lbl.setStyleSheet(
                f"color: {C['ink_dim']}; font-family: 'Courier New'; font-size: 11px;")
        ovl.addWidget(self._cam_alt_lbl)
        ovl.addWidget(self._cam_hdg_lbl)
        ovl.addStretch()
        ovl.addWidget(self._cam_live_lbl)

        body.addWidget(self._cam_label, stretch=1)
        body.addLayout(ovl)
        return frame

    def _draw_cam_placeholder(self):
        """Dibuja el placeholder de cámara cuando no hay feed disponible."""
        w = max(self._cam_label.width(), 640)
        h = max(self._cam_label.height(), 240)
        img = QImage(w, h, QImage.Format.Format_RGB888)
        img.fill(QColor("#040810"))
        p = QPainter(img)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        pen = QPen(QColor(C["border"]), 1)
        p.setPen(pen)
        for x in range(0, w, 30):
            p.drawLine(x, 0, x, h)
        for y in range(0, h, 30):
            p.drawLine(0, y, w, y)
        # Mira central
        cx_, cy_ = w // 2, h // 2
        p.setPen(QPen(QColor("rgba(255,255,255,120)"), 1))
        p.drawLine(cx_ - 40, cy_, cx_ + 40, cy_)
        p.drawLine(cx_, cy_ - 40, cx_, cy_ + 40)
        p.drawEllipse(QPointF(cx_, cy_), 22, 22)
        p.end()
        self._cam_label.setPixmap(
            QPixmap.fromImage(img).scaled(
                self._cam_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))

    def _on_cam_frame(self, img: QImage):
        """Recibe cada frame de la webcam, dibuja el overlay HUD y lo muestra."""
        pix = QPixmap.fromImage(img)

        # Dibujar overlay de telemetría directamente sobre el frame
        p = QPainter(pix)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        pkt = getattr(self, '_last_pkt', None)
        met_str = f"T+{self._fmt_met()}"

        font_mono = QFont("Courier New", 10)
        font_mono.setBold(True)
        p.setFont(font_mono)

        # Fondo semitransparente para el overlay
        overlay_color = QColor(0, 0, 0, 140)

        def draw_hud_text(x, y, text, color=C["ink"], bg=True):
            fm = p.fontMetrics()
            tw = fm.horizontalAdvance(text) + 8
            th = fm.height() + 4
            if bg:
                p.fillRect(x - 4, y - th + 4, tw, th, overlay_color)
            p.setPen(QPen(QColor(color)))
            p.drawText(x, y, text)

        w, h = pix.width(), pix.height()

        # Esquinas tipo HUD
        corner = 20
        p.setPen(QPen(QColor("white"), 1.5))
        for cx_, cy_, dx, dy in [(6,6,1,1),(w-6,6,-1,1),(6,h-6,1,-1),(w-6,h-6,-1,-1)]:
            p.drawLine(cx_, cy_, cx_+dx*corner, cy_)
            p.drawLine(cx_, cy_, cx_, cy_+dy*corner)

        # Mira central
        p.setPen(QPen(QColor("rgba(255,255,255,160)"), 1))
        cx_, cy_ = w//2, h//2
        p.drawLine(cx_-30, cy_, cx_+30, cy_)
        p.drawLine(cx_, cy_-30, cx_, cy_+30)
        p.drawEllipse(QPointF(cx_, cy_), 14, 14)

        # Overlay superior izquierdo
        draw_hud_text(10, 22, "SIMISPACE · CAM-01", C["accent"])
        draw_hud_text(10, 38, met_str, C["ink"])

        if pkt:
            # Overlay superior derecho
            draw_hud_text(w - 160, 22, f"ALT  {pkt.altitude:>7.1f} m",  C["ink"])
            draw_hud_text(w - 160, 38, f"HDG  {pkt.heading:>7.1f} °",   C["ink"])
            draw_hud_text(w - 160, 54, f"TEMP {pkt.temperature:>6.1f} °C", C["ink"])

            # Overlay inferior izquierdo
            draw_hud_text(10, h - 30, f"ROLL  {pkt.roll:>6.1f}°", C["ink_dim"])
            draw_hud_text(10, h - 14, f"PITCH {pkt.pitch:>6.1f}°", C["ink_dim"])

            # Indicador REC
            draw_hud_text(w - 60, h - 14, "● REC", C["red"])

        p.end()

        scaled = pix.scaled(
            self._cam_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation)
        self._cam_label.setPixmap(scaled)
        self._cam_live_lbl.setText("● CAM LIVE")
        self._cam_live_lbl.setStyleSheet(
            f"color: {C['green']}; font-family: 'Courier New'; font-size: 11px; font-weight: bold;")

    def _on_cam_error(self, msg: str):
        """Muestra el error de cámara en el log y deja el placeholder."""
        self._log.log("WARN", f"Cámara: {msg}", self._fmt_met())
        self._cam_live_lbl.setText("○ CAM OFF")
        self._cam_live_lbl.setStyleSheet(
            f"color: {C['muted']}; font-family: 'Courier New'; font-size: 11px;")

    def _build_altitude_chart(self) -> QFrame:
        """
        Gráfica de altitud en tiempo real usando pyqtgraph.
        Si pyqtgraph no está instalado, muestra un placeholder con instrucciones.
        """
        frame, body = self._make_panel("Altitude — Real Time Profile")

        if not PYQTGRAPH_AVAILABLE:
            lbl = QLabel("pyqtgraph no instalado — corre: pip3 install pyqtgraph")
            lbl.setStyleSheet(f"color: {C['muted']}; font-size: 11px;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            body.addWidget(lbl)
            self._alt_curve = None
            return frame

        self._alt_plot = pg.PlotWidget()
        self._alt_plot.setBackground(C["panel"])
        self._alt_plot.setMinimumHeight(90)
        self._alt_plot.showGrid(x=True, y=True, alpha=0.15)
        self._alt_plot.setLabel("left",  "Altitud (m)",
                                color=C["ink_dim"], size="10pt")
        self._alt_plot.setLabel("bottom", "Tiempo de misión (s)",
                                color=C["ink_dim"], size="10pt")
        self._alt_plot.getAxis("left").setTextPen(C["ink_dim"])
        self._alt_plot.getAxis("bottom").setTextPen(C["ink_dim"])

        # Rango inicial fijo y seguro — sin autorange hasta tener datos reales
        self._alt_plot.disableAutoRange()
        self._alt_plot.setXRange(0, 60, padding=0)
        self._alt_plot.setYRange(0, 500, padding=0)

        pen_alt = pg.mkPen(color=C["accent"], width=2)
        self._alt_curve = self._alt_plot.plot([], [], pen=pen_alt)

        fill = pg.FillBetweenItem(
            self._alt_curve,
            self._alt_plot.plot([0], [0], pen=pg.mkPen(None)),
            brush=pg.mkBrush(QColor(C["accent"]).darker(300))
        )
        self._alt_plot.addItem(fill)

        self._alt_max_line = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen(color=C["amber"], width=1,
                         style=Qt.PenStyle.DashLine))
        self._alt_plot.addItem(self._alt_max_line)
        self._alt_max_line.setVisible(False)   # oculta hasta tener datos

        self._alt_max_label = pg.TextItem(
            text="", color=C["amber"], anchor=(0, 1))
        self._alt_plot.addItem(self._alt_max_label)

        # Mensaje placeholder mientras no hay datos
        self._alt_placeholder = pg.TextItem(
            text="Esperando datos de altitud...",
            color=C["muted"], anchor=(0.5, 0.5))
        self._alt_plot.addItem(self._alt_placeholder)
        self._alt_placeholder.setPos(30, 250)

        body.addWidget(self._alt_plot)
        return frame

    def _update_altitude_chart(self, altitude: float):
        if not PYQTGRAPH_AVAILABLE or self._alt_curve is None:
            return
        # No graficar valores basura pre-vuelo
        if not self._met_started and altitude < 1.0:
            return

        if self._mission_t0 is None:
            self._mission_t0 = time.time()
            # Primer dato real: activar autorange y ocultar placeholder
            self._alt_plot.enableAutoRange()
            self._alt_placeholder.setText("")
            self._alt_max_line.setVisible(True)

        t = time.time() - self._mission_t0
        self._alt_times.append(t)
        self._alt_values.append(altitude)

        if len(self._alt_values) < 2:
            return

        xs = list(self._alt_times)
        ys = list(self._alt_values)
        self._alt_curve.setData(xs, ys)

        max_alt = max(ys)
        if max_alt > 0:
            self._alt_max_line.setValue(max_alt)
            self._alt_max_label.setText(f"MAX: {max_alt:.0f} m")
            self._alt_max_label.setPos(xs[0], max_alt)

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
        self._stat_rssi    = StatCard("RSSI")

        # Grid 2 columnas × 3 filas — evita el apilado vertical que
        # causaba que las tarjetas se aplastaran y sangraran texto
        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setContentsMargins(0, 0, 0, 0)
        cards = [
            self._stat_signal, self._stat_link,
            self._stat_packets, self._stat_port,
            self._stat_csv, self._stat_rssi,
        ]
        for i, card in enumerate(cards):
            grid.addWidget(card, i // 2, i % 2)

        body.addLayout(grid)
        body.addStretch()

        self._stat_port.set_value("—", C["muted"])
        self._stat_signal.set_value("—", C["muted"])
        self._stat_link.set_value("DISCONNECTED", C["red"])
        self._stat_packets.set_value("0", C["ink_dim"])
        self._stat_csv.set_value("OFF", C["muted"])
        self._stat_rssi.set_value("— dBm", C["muted"])
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
        self._met_seconds        = 0
        self._met_running        = False
        self._met_started        = False
        self._packet_count       = 0
        self._last_master_status = "nominal"
        self._last_pkt           = None
        self._mission_t0         = None
        self._alt_times.clear()
        self._alt_values.clear()
        self._lbl_met.setText("00:00:00")
        if PYQTGRAPH_AVAILABLE and hasattr(self, "_alt_curve") and self._alt_curve:
            self._alt_curve.setData([], [])
        # Reset fases
        self._phase           = "PRE-LAUNCH"
        self._apogee_detected = False
        self._max_altitude    = 0.0
        self._alt_history.clear()
        self._set_phase("PRE-LAUNCH")
        # Reset estadísticas
        self._stats = {
            "start_time":   None,
            "max_altitude": 0.0,
            "min_temp":     9999.0,
            "max_temp":     -9999.0,
            "min_pressure": 9999.0,
            "max_pressure": 0.0,
            "packets_rx":   0,
        }

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
            self._open_csv()
        else:
            self._lbl_status.setText("SIGNAL LOST")
            col = C["red"]
            self._stat_link.set_value("LOST", C["red"])
            self._stat_signal.set_value("LOST", C["red"])
            self._cam_live_lbl.setText("○ SIGNAL LOST")
            self._cam_live_lbl.setStyleSheet(
                f"color: {C['red']}; font-family: 'Courier New'; font-size: 11px;")

            # ── ALERTA DE SEÑAL PERDIDA ──────────────────────────────────────
            # Sonido distintivo (3 pitidos) + log de emergencia.
            # Solo se dispara si ya habíamos recibido al menos un paquete,
            # para no sonar en la primera conexión fallida antes del vuelo.
            if self._stats["packets_rx"] > 0:
                self._play_alert(kind="signal_lost")
                self._log.log("CRIT",
                    "⚠ SIGNAL LOST — Enlace con el vehículo interrumpido. "
                    "Verificar antena, alimentación y alcance LoRa.",
                    self._fmt_met())
                self._log.log("CRIT",
                    "  → Activar protocolo de recuperación. "
                    "Consultar último GPS / beacon Zoleo.",
                    self._fmt_met())
            else:
                self._log.log("WARN",
                    "Conexión no establecida. Verificar puerto COM y Heltec.",
                    self._fmt_met())

        self._lbl_status.setStyleSheet(
            f"color: {col}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
        self._dot_status.setStyleSheet(f"color: {col}; font-size: 14px;")

    def _on_error(self, msg: str):
        self._log.log("CRIT", msg, self._fmt_met())

    def _on_raw_line(self, line: str):
        line_up = line.upper()

        # ── Detectar mensajes de estado que manda el Heltec directamente ────
        # El Heltec receptor manda estas líneas cuando pierde/recupera el enlace
        # LoRa. No son paquetes TELEM, son strings de texto libre — los
        # interceptamos aquí para que la GUI reaccione igual que si fuera una
        # desconexión física del puerto.
        if "LOST SIGNAL" in line_up or "LOSS" in line_up:
            # Tratar como pérdida de señal real
            if self._stats["packets_rx"] > 0:
                self._play_alert(kind="signal_lost")
                self._lbl_status.setText("SIGNAL LOST")
                self._lbl_status.setStyleSheet(
                    f"color: {C['red']}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
                self._dot_status.setStyleSheet(f"color: {C['red']}; font-size: 14px;")
                self._stat_link.set_value("LOST", C["red"])
                self._stat_signal.set_value("LOST", C["red"])
                self._cam_live_lbl.setText("○ SIGNAL LOST")
                self._cam_live_lbl.setStyleSheet(
                    f"color: {C['red']}; font-family: 'Courier New'; font-size: 11px;")
                self._log.log("CRIT",
                    "⚠ SIGNAL LOST — Heltec reporta pérdida de enlace LoRa.",
                    self._fmt_met())
                self._log.log("CRIT",
                    "  → Activar protocolo de recuperación. Consultar beacon Zoleo.",
                    self._fmt_met())
            return

        if "SIGNAL RESTORED" in line_up or "RESTORED" in line_up:
            self._lbl_status.setText("NOMINAL")
            self._lbl_status.setStyleSheet(
                f"color: {C['green']}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
            self._dot_status.setStyleSheet(f"color: {C['green']}; font-size: 14px;")
            self._stat_link.set_value("ACTIVE", C["accent"])
            self._stat_signal.set_value("GOOD", C["green"])
            self._cam_live_lbl.setText("● LIVE")
            self._cam_live_lbl.setStyleSheet(
                f"color: {C['accent']}; font-family: 'Courier New'; font-size: 11px;")
            self._log.log("INFO",
                "✓ Señal restaurada — enlace LoRa reestablecido.",
                self._fmt_met())
            return

        # Línea genérica no-TELEM: mostrar en log como RAW
        if not line.startswith("TELEM"):
            self._log.log("RAW", line, self._fmt_met())

    def _on_packet(self, pkt: TelemetryPacket):
        self._packet_count += 1
        self._stat_packets.set_value(str(self._packet_count), C["ink_dim"])
        self._cam_alt_lbl.setText(f"ALT: {pkt.altitude:.1f} m")
        self._cam_hdg_lbl.setText(f"HDG: {pkt.heading:.0f}°")
        self._cam_live_lbl.setText("● LIVE")
        self._cam_live_lbl.setStyleSheet(
            f"color: {C['accent']}; font-family: 'Courier New'; font-size: 11px;")
        self._last_pkt = pkt
        self._card_pressure.update_value(pkt.pressure)
        self._card_temperature.update_value(pkt.temperature)
        self._card_altitude.update_value(pkt.altitude)
        self._card_humidity.update_value(pkt.humidity)
        self._pos_labels["LAT"].setText(f"{pkt.lat:.5f}°")
        self._pos_labels["LON"].setText(f"{pkt.lon:.5f}°")
        self._pos_labels["HDG"].setText(f"{pkt.heading:.1f}°")
        self._pos_labels["ROLL"].setText(f"{pkt.roll:.1f}°")
        self._pos_labels["PITCH"].setText(f"{pkt.pitch:.1f}°")
        self._trajectory.push(pkt.lat, pkt.lon, pkt.heading)
        self._attitude.set_target(pkt.roll, pkt.pitch)
        if pkt.rssi is not None:
            rssi_col = (C["green"] if pkt.rssi > -80
                        else C["amber"] if pkt.rssi > -100 else C["red"])
            self._stat_rssi.set_value(f"{pkt.rssi:.0f} dBm", rssi_col)
        self._update_altitude_chart(pkt.altitude)
        statuses = [
            get_status("pressure",    pkt.pressure),
            get_status("temperature", pkt.temperature),
            get_status("humidity",    pkt.humidity),
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
        new_status = master_txt.lower()
        if new_status == "critical" and self._last_master_status != "critical":
            self._play_alert()
        self._last_master_status = new_status

        # ── DETECCIÓN DE FASES ───────────────────────────────────────────────
        self._detect_phase(pkt.altitude)

        # ── ESTADÍSTICAS DE MISIÓN (para el PDF) ────────────────────────────
        self._stats["packets_rx"]  += 1
        self._stats["max_altitude"] = max(self._stats["max_altitude"], pkt.altitude)
        self._stats["min_temp"]     = min(self._stats["min_temp"],     pkt.temperature)
        self._stats["max_temp"]     = max(self._stats["max_temp"],     pkt.temperature)
        self._stats["min_pressure"] = min(self._stats["min_pressure"], pkt.pressure)
        self._stats["max_pressure"] = max(self._stats["max_pressure"], pkt.pressure)
        if self._stats["start_time"] is None:
            self._stats["start_time"] = datetime.datetime.now()

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

    def _play_alert(self, kind: str = "critical"):
        """
        Sonido de alerta. Corre en hilo separado para no bloquear la UI.

        kind = "critical"     → Basso.aiff  (1 golpe grave) — anomalía de telemetría
        kind = "signal_lost"  → Sosumi.aiff × 3 con pausa   — pérdida de señal/vehículo

        Usa afplay en Mac (sin dependencias extra).
        Fallback a QApplication.beep() en cualquier otro sistema.
        """
        import threading

        def _beep_critical():
            try:
                subprocess.Popen(
                    ["afplay", "/System/Library/Sounds/Basso.aiff"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except FileNotFoundError:
                QApplication.beep()

        def _beep_signal_lost():
            # Tres pitidos distintivos con pausa entre ellos —
            # inconfundible incluso si no estás mirando la pantalla.
            for _ in range(3):
                try:
                    proc = subprocess.Popen(
                        ["afplay", "/System/Library/Sounds/Sosumi.aiff"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    proc.wait()
                    time.sleep(0.25)
                except FileNotFoundError:
                    QApplication.beep()
                    time.sleep(0.4)

        target = _beep_signal_lost if kind == "signal_lost" else _beep_critical
        threading.Thread(target=target, daemon=True).start()

    def _start_met(self):
        """Arranca el MET automáticamente desde T-0."""
        if not self._met_started:
            self._met_started = True
            self._met_running = True
            self._met_seconds = 0
            self._btn_met_start.setText("● MET ON")
            self._btn_met_start.setStyleSheet(f"""
                QPushButton {{
                    background: {C['panel_alt']}; color: {C['accent']};
                    border: 1px solid {C['accent']}; border-radius: 3px;
                    font-size: 10px; font-weight: bold;
                }}
            """)
            self._log.log("INFO", "🚀 T-0 — MET iniciado automáticamente", "00:00:00")

    def _manual_start_met(self):
        """Arranca o reinicia el MET manualmente desde el botón del header."""
        self._met_started = True
        self._met_running = True
        self._met_seconds = 0
        self._lbl_met.setText("00:00:00")
        self._btn_met_start.setText("● MET ON")
        self._btn_met_start.setStyleSheet(f"""
            QPushButton {{
                background: {C['panel_alt']}; color: {C['accent']};
                border: 1px solid {C['accent']}; border-radius: 3px;
                font-size: 10px; font-weight: bold;
            }}
        """)
        self._log.log("INFO", "▶ MET iniciado manualmente", self._fmt_met())

    # ─────────────────────────────────────────────────────────────────────────
    # CIERRE
    # ─────────────────────────────────────────────────────────────────────────

    # ─────────────────────────────────────────────────────────────────────────
    # REPLAY DE MISIÓN
    # ─────────────────────────────────────────────────────────────────────────

    def _open_replay_dialog(self):
        """Abre un selector de archivo CSV y arranca el replay."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Abrir CSV de misión", os.path.expanduser("~/Downloads"),
            "CSV Files (*.csv);;All Files (*)")
        if not path:
            return

        # Cargar y parsear el CSV
        packets = []
        try:
            with open(path, newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        pkt = TelemetryPacket()
                        pkt.pressure    = float(row.get("pressure_pa",    1013.25))
                        pkt.temperature = float(row.get("temperature_c",  20.0))
                        pkt.altitude    = float(row.get("altitude_m",     0.0))
                        pkt.humidity    = float(row.get("humidity_pct",   50.0))
                        pkt.roll        = float(row.get("roll_deg",       0.0))
                        pkt.pitch       = float(row.get("pitch_deg",      0.0))
                        pkt.heading     = float(row.get("heading_deg",    0.0))
                        pkt.rssi        = None
                        pkt.timestamp   = time.time()
                        packets.append(pkt)
                    except (ValueError, KeyError):
                        continue
        except Exception as e:
            QMessageBox.warning(self, "Error", f"No se pudo leer el CSV:\n{e}")
            return

        if not packets:
            QMessageBox.warning(self, "Error", "El CSV no contiene datos válidos.")
            return

        # Abrir diálogo de control de replay
        self._show_replay_controls(packets, path)

    def _show_replay_controls(self, packets: list, path: str):
        """Muestra el diálogo de control de replay con barra de progreso y velocidad."""
        dlg = QDialog(self)
        dlg.setWindowTitle("SIMISPACE — Mission Replay")
        dlg.setFixedSize(480, 220)
        dlg.setStyleSheet(f"""
            QDialog  {{ background: {C['bg']}; color: {C['ink']}; }}
            QLabel   {{ color: {C['ink_dim']}; font-size: 11px; }}
            QPushButton {{
                background: {C['panel']}; color: {C['accent']};
                border: 1px solid {C['border']}; border-radius: 4px;
                padding: 6px 16px; font-size: 11px;
            }}
            QPushButton:hover {{ background: {C['panel_alt']}; }}
            QSlider::groove:horizontal {{
                background: {C['border']}; height: 4px; border-radius: 2px;
            }}
            QSlider::handle:horizontal {{
                background: {C['accent']}; width: 14px; height: 14px;
                margin: -5px 0; border-radius: 7px;
            }}
            QProgressBar {{
                background: {C['panel_alt']}; border: 1px solid {C['border']};
                border-radius: 4px; height: 10px; text-align: center;
                color: {C['ink_dim']}; font-size: 10px;
            }}
            QProgressBar::chunk {{ background: {C['accent']}; border-radius: 3px; }}
        """)

        layout = QVBoxLayout(dlg)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        title = QLabel(f"REPLAY: {os.path.basename(path)}")
        title.setStyleSheet(
            f"color: {C['ink']}; font-size: 13px; font-weight: bold;")
        layout.addWidget(title)

        info = QLabel(f"{len(packets)} paquetes · duración estimada: "
                      f"{len(packets)//60}m {len(packets)%60}s @ 1x")
        layout.addWidget(info)

        # Barra de progreso
        progress = QProgressBar()
        progress.setRange(0, len(packets))
        progress.setValue(0)
        layout.addWidget(progress)

        # Control de velocidad
        speed_row = QHBoxLayout()
        speed_lbl = QLabel("Velocidad:")
        self._replay_speed_lbl = QLabel("1×")
        self._replay_speed_lbl.setStyleSheet(
            f"color: {C['accent']}; font-weight: bold;")
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, 3)
        slider.setValue(0)
        slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        speeds = [1, 2, 5, 10]

        def on_speed_change(v):
            self._replay_speed = speeds[v]
            self._replay_speed_lbl.setText(f"{speeds[v]}×")

        slider.valueChanged.connect(on_speed_change)
        speed_row.addWidget(speed_lbl)
        speed_row.addWidget(slider)
        speed_row.addWidget(self._replay_speed_lbl)
        layout.addLayout(speed_row)

        # Botones
        btn_row = QHBoxLayout()
        btn_start = QPushButton("▶ Iniciar Replay")
        btn_stop  = QPushButton("■ Detener")
        btn_row.addWidget(btn_start)
        btn_row.addWidget(btn_stop)
        layout.addLayout(btn_row)

        # Estado del replay
        self._replay_packets = packets
        self._replay_index   = 0
        self._replay_mode    = False

        def start_replay():
            if self._replay_timer:
                self._replay_timer.stop()
            self._replay_index  = 0
            self._replay_mode   = True
            self._stop_worker()
            interval_ms = max(50, 1000 // self._replay_speed)
            self._replay_timer = QTimer(self)

            def step():
                if self._replay_index >= len(self._replay_packets):
                    self._replay_timer.stop()
                    self._replay_mode = False
                    self._log.log("INFO", "Replay completado", self._fmt_met())
                    return
                pkt = self._replay_packets[self._replay_index]
                self._on_packet(pkt)
                progress.setValue(self._replay_index)
                self._replay_index += 1

            self._replay_timer.timeout.connect(step)
            self._replay_timer.start(interval_ms)
            self._log.log("INFO",
                f"Replay iniciado: {len(packets)} pkts @ {self._replay_speed}×",
                self._fmt_met())

        def stop_replay():
            if self._replay_timer:
                self._replay_timer.stop()
            self._replay_mode = False
            self._log.log("INFO", "Replay detenido manualmente", self._fmt_met())

        btn_start.clicked.connect(start_replay)
        btn_stop.clicked.connect(stop_replay)
        dlg.exec()

    # ─────────────────────────────────────────────────────────────────────────
    # EXPORT PDF
    # ─────────────────────────────────────────────────────────────────────────

    def _export_pdf(self, auto: bool = False):
        """
        Genera un reporte PDF profesional de la misión.
        Si auto=True (llamado al cerrar), guarda en Downloads sin preguntar.
        Si auto=False (botón manual), muestra selector de ruta.
        """
        if not REPORTLAB_AVAILABLE:
            QMessageBox.warning(self, "reportlab no instalado",
                "Instala reportlab para exportar PDF:\n\npip3 install reportlab")
            return

        ts  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_path = os.path.expanduser(f"~/Downloads/simispace_report_{ts}.pdf")

        if auto:
            pdf_path = default_path
        else:
            pdf_path, _ = QFileDialog.getSaveFileName(
                self, "Guardar Reporte PDF",
                default_path, "PDF Files (*.pdf)")
            if not pdf_path:
                return

        try:
            doc = SimpleDocTemplate(
                pdf_path, pagesize=A4,
                rightMargin=2*cm, leftMargin=2*cm,
                topMargin=2*cm, bottomMargin=2*cm)

            styles = getSampleStyleSheet()

            # Estilos personalizados
            title_style = ParagraphStyle(
                "Title", parent=styles["Normal"],
                fontSize=22, fontName="Helvetica-Bold",
                textColor=rl_colors.HexColor("#4FB3E8"),
                spaceAfter=4)

            sub_style = ParagraphStyle(
                "Sub", parent=styles["Normal"],
                fontSize=11, fontName="Helvetica",
                textColor=rl_colors.HexColor("#90A4BD"),
                spaceAfter=16)

            section_style = ParagraphStyle(
                "Section", parent=styles["Normal"],
                fontSize=12, fontName="Helvetica-Bold",
                textColor=rl_colors.HexColor("#EAF0F6"),
                spaceBefore=12, spaceAfter=6)

            body_style = ParagraphStyle(
                "Body", parent=styles["Normal"],
                fontSize=10, fontName="Helvetica",
                textColor=rl_colors.HexColor("#90A4BD"),
                spaceAfter=4)

            now_str = datetime.datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
            met_str = self._fmt_met()
            phase   = self._phase
            s       = self._stats

            story = [
                Paragraph("SIMISPACE", title_style),
                Paragraph("GROUND CONTROL STATION — MISSION REPORT", sub_style),
                HRFlowable(width="100%", thickness=1,
                           color=rl_colors.HexColor("#1E2C42"), spaceAfter=16),

                Paragraph("MISSION OVERVIEW", section_style),
            ]

            # Tabla de resumen
            overview_data = [
                ["Field", "Value"],
                ["Report Generated",    now_str],
                ["Mission Duration (MET)", met_str],
                ["Final Phase",          phase],
                ["Total Packets RX",     str(s["packets_rx"])],
                ["CSV Data File",        str(self._csv_path or "—")],
            ]
            t1 = Table(overview_data, colWidths=[5*cm, 12*cm])
            t1.setStyle(TableStyle([
                ("BACKGROUND",   (0, 0), (-1, 0), rl_colors.HexColor("#101B2E")),
                ("TEXTCOLOR",    (0, 0), (-1, 0), rl_colors.HexColor("#4FB3E8")),
                ("FONTNAME",     (0, 0), (-1, 0), "Helvetica-Bold"),
                ("FONTSIZE",     (0, 0), (-1, -1), 9),
                ("FONTNAME",     (0, 1), (0, -1), "Helvetica-Bold"),
                ("TEXTCOLOR",    (0, 1), (0, -1), rl_colors.HexColor("#EAF0F6")),
                ("TEXTCOLOR",    (1, 1), (1, -1), rl_colors.HexColor("#90A4BD")),
                ("BACKGROUND",   (0, 1), (-1, -1), rl_colors.HexColor("#0C1422")),
                ("ROWBACKGROUNDS",(0, 1), (-1, -1),
                 [rl_colors.HexColor("#0C1422"), rl_colors.HexColor("#101B2E")]),
                ("GRID",         (0, 0), (-1, -1), 0.5, rl_colors.HexColor("#1E2C42")),
                ("VALIGN",       (0, 0), (-1, -1), "MIDDLE"),
                ("TOPPADDING",   (0, 0), (-1, -1), 5),
                ("BOTTOMPADDING",(0, 0), (-1, -1), 5),
            ]))
            story.append(t1)
            story.append(Spacer(1, 0.5*cm))
            story.append(Paragraph("TELEMETRY STATISTICS", section_style))

            stats_data = [
                ["Parameter", "Min", "Max", "Units"],
                ["Altitude",
                 "—",
                 f"{s['max_altitude']:.1f}",
                 "m"],
                ["Temperature",
                 f"{s['min_temp']:.1f}" if s['min_temp'] < 9999 else "—",
                 f"{s['max_temp']:.1f}" if s['max_temp'] > -9999 else "—",
                 "°C"],
                ["Pressure",
                 f"{s['min_pressure']:.1f}" if s['min_pressure'] < 9999 else "—",
                 f"{s['max_pressure']:.1f}" if s['max_pressure'] > 0 else "—",
                 "hPa"],
            ]
            t2 = Table(stats_data, colWidths=[6*cm, 3*cm, 3*cm, 3*cm])
            t2.setStyle(TableStyle([
                ("BACKGROUND",   (0, 0), (-1, 0), rl_colors.HexColor("#101B2E")),
                ("TEXTCOLOR",    (0, 0), (-1, 0), rl_colors.HexColor("#4FB3E8")),
                ("FONTNAME",     (0, 0), (-1, 0), "Helvetica-Bold"),
                ("FONTSIZE",     (0, 0), (-1, -1), 9),
                ("FONTNAME",     (0, 1), (0, -1), "Helvetica-Bold"),
                ("TEXTCOLOR",    (0, 1), (0, -1), rl_colors.HexColor("#EAF0F6")),
                ("TEXTCOLOR",    (1, 1), (-1, -1), rl_colors.HexColor("#90A4BD")),
                ("ROWBACKGROUNDS",(0, 1), (-1, -1),
                 [rl_colors.HexColor("#0C1422"), rl_colors.HexColor("#101B2E")]),
                ("GRID",         (0, 0), (-1, -1), 0.5, rl_colors.HexColor("#1E2C42")),
                ("ALIGN",        (1, 0), (-1, -1), "CENTER"),
                ("VALIGN",       (0, 0), (-1, -1), "MIDDLE"),
                ("TOPPADDING",   (0, 0), (-1, -1), 5),
                ("BOTTOMPADDING",(0, 0), (-1, -1), 5),
            ]))
            story.append(t2)
            story.append(Spacer(1, 0.5*cm))
            story.append(HRFlowable(
                width="100%", thickness=0.5,
                color=rl_colors.HexColor("#1E2C42"), spaceAfter=8))
            story.append(Paragraph(
                f"Generated by SIMISPACE Ground Control Station · {now_str}",
                body_style))

            doc.build(story)
            self._log.log("INFO", f"📄 PDF guardado: {pdf_path}", self._fmt_met())

            if not auto:
                QMessageBox.information(
                    self, "PDF Generado",
                    f"Reporte guardado en:\n{pdf_path}")

        except Exception as e:
            self._log.log("CRIT", f"Error generando PDF: {e}", self._fmt_met())

    def closeEvent(self, event):
        # Detener replay si está corriendo
        if self._replay_timer:
            self._replay_timer.stop()
        self._stop_worker()
        self._close_csv()
        if hasattr(self, '_cam_worker') and self._cam_worker:
            self._cam_worker.stop()
        # Exportar PDF automáticamente al cerrar si hubo datos
        if self._stats["packets_rx"] > 0:
            self._export_pdf(auto=True)
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