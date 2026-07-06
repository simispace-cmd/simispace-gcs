"""
╔══════════════════════════════════════════════════════════════════════════════╗
║   SIMISPACE — GROUND CONTROL STATION                                        ║
║   Estación Terrena en tiempo real · PyQt6 + pyserial                       ║
╠══════════════════════════════════════════════════════════════════════════════╣
║   DEPENDENCIAS (instalar UNA sola vez en la terminal):                      ║
║     pip install PyQt6 pyserial                                              ║
║                                                                             ║
║   CORRER:                                                                   ║
║     python simispace_gcs.py                                                 ║
║                                                                             ║
║   FORMATO SERIAL ESPERADO DEL HELTEC (una línea por paquete, a 115200 bps):║
║     TELEM,<presión_Pa>,<temp_°C>,<altitud_m>,<humedad_%>,<lat>,<lon>,      ║
║           <roll_°>,<pitch_°>,<heading_°>\n                                  ║
║                                                                             ║
║   Ejemplo de línea real:                                                    ║
║     TELEM,101325.0,23.4,156.2,48.3,19.4326,-99.1332,2.1,-1.3,45.6\n       ║
║                                                                             ║
║   Si tu Heltec manda JSON o CSV con otro orden → ver sección               ║
║   "PARSER DE TELEMETRÍA" más abajo y ajustar parse_line().                 ║
║                                                                             ║
║   ARDUINO / Heltec — código mínimo para mandar el paquete correcto:        ║
║     // Incluir <Wire.h>, <Adafruit_BMP280.h> o lo que uses                 ║
║     Serial.print("TELEM,");                                                 ║
║     Serial.print(bmp.readPressure(), 1); Serial.print(",");                 ║
║     Serial.print(bmp.readTemperature(), 1); Serial.print(",");              ║
║     Serial.print(bmp.readAltitude(1013.25), 1); Serial.print(",");         ║
║     Serial.print(humidity, 1); Serial.print(",");                           ║
║     Serial.print(gps.location.lat(), 5); Serial.print(",");                 ║
║     Serial.print(gps.location.lng(), 5); Serial.print(",");                 ║
║     Serial.print(imu.roll, 1); Serial.print(",");                           ║
║     Serial.print(imu.pitch, 1); Serial.print(",");                          ║
║     Serial.println(imu.heading, 1);   // <-- println agrega \n             ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import sys
import time
import math
import random
from collections import deque
from typing import Optional

import serial
import serial.tools.list_ports

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QComboBox, QPushButton, QTextEdit,
    QSplitter, QSizePolicy, QDialog, QFormLayout,
    QSpinBox, QDialogButtonBox, QMessageBox,
)
from PyQt6.QtCore import (
    Qt, QThread, pyqtSignal, QTimer, QSize, QRect, QPointF,
)
from PyQt6.QtGui import (
    QColor, QPainter, QPen, QBrush, QPainterPath, QFont,
    QFontDatabase, QPalette, QLinearGradient, QPolygonF,
)

# ═══════════════════════════════════════════════════════════════════════════════
# PALETA DE COLORES (misma lógica que la web app)
# ═══════════════════════════════════════════════════════════════════════════════

C = {
    "bg":        "#070D18",
    "panel":     "#101B2E",
    "panel_alt": "#0C1422",
    "border":    "#1E2C42",
    "ink":       "#EAF0F6",
    "ink_dim":   "#90A4BD",
    "muted":     "#56677F",
    "accent":    "#4FB3E8",   # cian — dato nominal vivo
    "green":     "#3ED598",   # semáforo maestro OK
    "amber":     "#F2A93C",   # CAUTION
    "red":       "#E5484D",   # CRITICAL
}

def qc(hex_str: str) -> QColor:
    """Convierte hex string a QColor."""
    return QColor(hex_str)


# ═══════════════════════════════════════════════════════════════════════════════
# UMBRALES DE SEGURIDAD
# ═══════════════════════════════════════════════════════════════════════════════

THRESHOLDS = {
    "pressure":    {"caution_lo": 90000, "caution_hi": 106000,
                    "critical_lo": 85000, "critical_hi": 110000},
    "temperature": {"caution_lo": -5,    "caution_hi": 32,
                    "critical_lo": -15,  "critical_hi": 42},
    "humidity":    {"caution_lo": 12,    "caution_hi": 78,
                    "critical_lo": 5,    "critical_hi": 90},
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
# ESTRUCTURA DE DATOS DE TELEMETRÍA
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
# PARSER DE TELEMETRÍA
# ═══════════════════════════════════════════════════════════════════════════════
# Si tu Heltec manda un formato distinto, modifica parse_line() aquí.
# La función recibe un str (línea del puerto serial) y devuelve
# TelemetryPacket o None si la línea no es válida.

def parse_line(line: str) -> Optional[TelemetryPacket]:
    """
    Parsea una línea TELEM del Heltec.
    Formato esperado (CSV, prefijo "TELEM"):
        TELEM,<pressure>,<temperature>,<altitude>,<humidity>,
              <lat>,<lon>,<roll>,<pitch>,<heading>

    Para cambiar a JSON:
        import json
        data = json.loads(line)
        pkt.pressure = data["p"]
        ... etc.
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
# WORKER SERIAL (hilo separado — nunca bloquea la UI)
# ═══════════════════════════════════════════════════════════════════════════════

class SerialWorker(QThread):
    """
    Lee el puerto COM en un hilo de fondo.
    Emite:
      packet_received(TelemetryPacket) — cada paquete válido
      raw_line(str)                    — cada línea cruda (para logs)
      connection_changed(bool)         — True=conectado, False=desconectado
      error(str)                       — mensaje de error legible
    """
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
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=2.0,
            )
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
# WORKER DE SIMULACIÓN (para pruebas sin Heltec conectado)
# ═══════════════════════════════════════════════════════════════════════════════

class SimWorker(QThread):
    """
    Genera telemetría simulada a 1 Hz para probar la GUI sin hardware.
    Comparte la misma interfaz de señales que SerialWorker.
    """
    packet_received    = pyqtSignal(object)
    raw_line           = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)
    error              = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._running = True
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
            self._heading     = (self._heading + 0.6 + (random.random() - 0.5) * 0.4) % 360
            roll  = 15 * math.sin(self._t / 6) + (random.random() - 0.5) * 2
            pitch = 8  * math.sin(self._t / 9 + 1) + (random.random() - 0.5) * 1.5

            pkt = TelemetryPacket()
            pkt.pressure    = self._pressure
            pkt.temperature = self._temperature
            pkt.altitude    = self._altitude
            pkt.humidity    = self._humidity
            pkt.lat         = self._lat
            pkt.lon         = self._lon
            pkt.roll        = roll
            pkt.pitch       = pitch
            pkt.heading     = self._heading

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
    """Mini gráfico de tendencia dibujado con QPainter. Sin dependencias extra."""

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
            x = pad + (i / (len(data) - 1)) * (w - 2 * pad)
            y = h - pad - ((v - lo) / rng) * (h - 2 * pad)
            return QPointF(x, y)

        pts = [to_pt(i, v) for i, v in enumerate(data)]

        # Relleno con gradiente
        path = QPainterPath()
        path.moveTo(QPointF(pts[0].x(), h))
        for pt in pts:
            path.lineTo(pt)
        path.lineTo(QPointF(pts[-1].x(), h))
        path.closeSubpath()

        grad = QLinearGradient(0, 0, 0, h)
        c_top = QColor(self._color)
        c_top.setAlpha(80)
        c_bot = QColor(self._color)
        c_bot.setAlpha(0)
        grad.setColorAt(0, c_top)
        grad.setColorAt(1, c_bot)
        p.fillPath(path, QBrush(grad))

        # Línea
        pen = QPen(self._color, 1.6)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        p.setPen(pen)
        poly = QPolygonF(pts)
        p.drawPolyline(poly)
        p.end()


# ═══════════════════════════════════════════════════════════════════════════════
# WIDGET: TARJETA DE TELEMETRÍA
# ═══════════════════════════════════════════════════════════════════════════════

class TelemetryCard(QFrame):
    """
    Tarjeta individual: label / valor grande / unidad / status badge / sparkline.
    Estética SpaceX glass-cockpit, consistente con la web app.
    """

    def __init__(self, label: str, unit: str, threshold_key: str,
                 decimals: int = 1, secondary: str = "", parent=None):
        super().__init__(parent)
        self._label         = label
        self._unit          = unit
        self._threshold_key = threshold_key
        self._decimals      = decimals
        self._secondary     = secondary
        self._status        = "nominal"

        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setObjectName("TelemetryCard")
        self.setStyleSheet(f"""
            #TelemetryCard {{
                background: {C['panel']};
                border: 1px solid {C['border']};
                border-radius: 8px;
            }}
        """)

        # ── Layout ────────────────────────────────────────────────────────────
        outer = QHBoxLayout(self)
        outer.setContentsMargins(12, 10, 12, 10)
        outer.setSpacing(10)

        # Columna izquierda: texto
        left = QVBoxLayout()
        left.setSpacing(2)

        self._lbl_title = QLabel(label.upper())
        self._lbl_title.setStyleSheet(f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 2px;")

        self._lbl_value = QLabel("—")
        self._lbl_value.setStyleSheet(f"color: {C['ink']}; font-size: 28px; font-weight: bold;")

        self._lbl_unit = QLabel(unit)
        self._lbl_unit.setStyleSheet(f"color: {C['ink_dim']}; font-size: 12px;")

        self._lbl_status = QLabel("NOMINAL")
        self._lbl_status.setStyleSheet(f"color: {C['accent']}; font-size: 10px; font-weight: bold; letter-spacing: 2px;")

        val_row = QHBoxLayout()
        val_row.setSpacing(4)
        val_row.addWidget(self._lbl_value)
        val_row.addWidget(self._lbl_unit)
        val_row.addStretch()

        left.addWidget(self._lbl_title)
        left.addLayout(val_row)
        left.addWidget(self._lbl_status)

        # Columna derecha: sparkline
        self._spark = SparklineWidget(color=C["accent"])
        self._spark.setFixedWidth(100)

        outer.addLayout(left, stretch=1)
        outer.addWidget(self._spark)

    def update_value(self, value: float, secondary: str = ""):
        status = get_status(self._threshold_key, value)
        if status != self._status:
            self._status = status
            col_val    = status_color(status)
            col_accent = status_accent(status)
            self._lbl_value.setStyleSheet(
                f"color: {col_val}; font-size: 28px; font-weight: bold;")
            self._lbl_status.setStyleSheet(
                f"color: {col_accent}; font-size: 10px; font-weight: bold; letter-spacing: 2px;")
            self._spark.set_color(col_accent)
            border_col = col_accent if status != "nominal" else C["border"]
            self.setStyleSheet(f"""
                #TelemetryCard {{
                    background: {C['panel']};
                    border: 1px solid {border_col};
                    border-radius: 8px;
                }}
            """)

        self._lbl_status.setText(status.upper())
        self._lbl_value.setText(f"{value:.{self._decimals}f}")
        self._spark.push(value)


# ═══════════════════════════════════════════════════════════════════════════════
# WIDGET: STAT CARD (batería / señal / enlace)
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
        self._lbl_label.setStyleSheet(f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 2px;")

        self._lbl_value = QLabel("—")
        self._lbl_value.setStyleSheet(f"color: {C['accent']}; font-size: 14px; font-weight: bold;")

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
        header.setStyleSheet(f"""
            background: transparent;
            border-bottom: 1px solid {C['border']};
        """)
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
        ts = f"[T+{met}]" if met else ""
        color = {
            "INFO": C["ink_dim"],
            "WARN": C["amber"],
            "CRIT": C["red"],
            "RAW":  C["muted"],
        }.get(level, C["ink_dim"])

        level_str = f"[{level}]" if level != "RAW" else ""
        html = (f'<span style="color:{C["muted"]};">{ts}</span> '
                f'<span style="color:{color}; font-weight:bold;">{level_str}</span> '
                f'<span style="color:{C["ink"]};">{message}</span>')
        self._text.append(html)

        sb = self._text.verticalScrollBar()
        sb.setValue(sb.maximum())


# ═══════════════════════════════════════════════════════════════════════════════
# WIDGET: VISUALIZADOR DE TRAYECTORIA (mapa en cuadrícula)
# ═══════════════════════════════════════════════════════════════════════════════

class TrajectoryWidget(QWidget):
    """
    Visualiza la traza 2D de posición relativa al punto de lanzamiento.
    La flecha indica el heading actual del vehículo.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self._points: deque = deque(maxlen=200)
        self._heading = 0.0
        self._origin_lat = None
        self._origin_lon = None
        self.setMinimumSize(160, 150)

    def _to_rel(self, lat, lon):
        """Convierte lat/lon a coordenadas relativas normalizadas [-1, 1]."""
        if self._origin_lat is None:
            self._origin_lat = lat
            self._origin_lon = lon
        scale = 0.012  # ~1.3 km visible
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
        w, h = self.width(), self.height()
        pad = 14
        cx, cy = w / 2, h / 2
        rx, ry = (w - 2 * pad) / 2, (h - 2 * pad) / 2

        # Cuadrícula
        grid_pen = QPen(qc(C["border"]), 1)
        p.setPen(grid_pen)
        for i in range(5):
            fx = pad + i * (w - 2 * pad) / 4
            fy = pad + i * (h - 2 * pad) / 4
            p.drawLine(int(fx), pad, int(fx), h - pad)
            p.drawLine(pad, int(fy), w - pad, int(fy))

        # Ejes cruzados
        p.drawLine(int(cx), pad, int(cx), h - pad)
        p.drawLine(pad, int(cy), w - pad, int(cy))

        # Etiqueta N
        p.setPen(QPen(qc(C["muted"]), 1))
        p.setFont(QFont("Courier New", 8))
        p.drawText(int(cx) + 3, pad + 10, "N")

        if len(self._points) < 2:
            p.end()
            return

        # Traza
        pen = QPen(qc(C["accent"]), 1.5)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        p.setPen(pen)

        def to_px(px_r, py_r):
            return QPointF(cx + px_r * rx, cy - py_r * ry)

        pts = [to_px(x, y) for x, y in self._points]
        poly = QPolygonF(pts)
        p.drawPolyline(poly)

        # Marcador de posición actual con flecha de heading
        last = pts[-1]
        rad = math.radians(self._heading)
        tip_l = 10
        tip = QPointF(last.x() + math.sin(rad) * tip_l,
                      last.y() - math.cos(rad) * tip_l)
        left = QPointF(last.x() + math.sin(rad + 2.5) * 5,
                       last.y() - math.cos(rad + 2.5) * 5)
        right = QPointF(last.x() + math.sin(rad - 2.5) * 5,
                        last.y() - math.cos(rad - 2.5) * 5)
        arrow = QPolygonF([tip, left, right])
        p.setBrush(QBrush(qc(C["ink"])))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(arrow)
        p.end()


# ═══════════════════════════════════════════════════════════════════════════════
# WIDGET: HORIZONTE ARTIFICIAL
# ═══════════════════════════════════════════════════════════════════════════════

class AttitudeWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._roll  = 0.0
        self._pitch = 0.0
        self.setMinimumSize(140, 130)

    def set_attitude(self, roll: float, pitch: float):
        self._roll  = roll
        self._pitch = pitch
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        r = min(w, h) / 2 - 8

        # Clip al círculo
        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), r, r)
        p.setClipPath(clip)

        pitch_offset = max(-30, min(30, self._pitch)) * 0.9

        # Rotar y desplazar la vista por pitch
        p.save()
        p.translate(cx, cy)
        p.rotate(-self._roll)
        p.translate(0, pitch_offset)

        # Cielo
        p.fillRect(int(-r - 10), int(-r * 2 - 10), int(r * 2 + 20), int(r * 2 + 10),
                   QBrush(QColor("#142233")))
        # Tierra
        p.fillRect(int(-r - 10), 0, int(r * 2 + 20), int(r * 2 + 10),
                   QBrush(QColor("#241B12")))
        # Línea de horizonte
        p.setPen(QPen(qc(C["ink"]), 1.5))
        p.drawLine(int(-r - 10), 0, int(r + 10), 0)

        p.restore()

        # Borde del instrumento
        p.setClipping(False)
        p.setPen(QPen(qc(C["border"]), 1.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(cx, cy), r, r)

        # Índice fijo del vehículo (no rota)
        p.setPen(QPen(qc(C["accent"]), 2))
        p.drawLine(int(cx) - 18, int(cy), int(cx) + 18, int(cy))
        p.drawLine(int(cx), int(cy) - 5, int(cx), int(cy) + 5)
        p.end()


# ═══════════════════════════════════════════════════════════════════════════════
# DIÁLOGO DE SELECCIÓN DE PUERTO SERIAL
# ═══════════════════════════════════════════════════════════════════════════════

class SerialDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Conexión Serial")
        self.setModal(True)
        self.setStyleSheet(f"""
            QDialog {{ background: {C['bg']}; color: {C['ink']}; }}
            QLabel  {{ color: {C['ink_dim']}; font-size: 12px; }}
            QComboBox, QSpinBox {{
                background: {C['panel_alt']};
                color: {C['ink']};
                border: 1px solid {C['border']};
                border-radius: 4px;
                padding: 4px 8px;
                font-size: 12px;
                min-height: 28px;
            }}
            QPushButton {{
                background: {C['panel']};
                color: {C['accent']};
                border: 1px solid {C['border']};
                border-radius: 4px;
                padding: 6px 16px;
                font-size: 12px;
            }}
            QPushButton:hover {{ background: {C['panel_alt']}; }}
        """)

        self.port_combo    = QComboBox()
        self.baud_spin     = QSpinBox()
        self.sim_btn       = QPushButton("▶ Modo Simulación (sin hardware)")
        self.connect_btn   = QPushButton("Conectar al Puerto")
        self._sim_selected = False

        self.baud_spin.setRange(1200, 3000000)
        self.baud_spin.setValue(115200)
        self.baud_spin.setSingleStep(9600)

        # Listar puertos disponibles
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device}  —  {port.description}", port.device)
        if not ports:
            self.port_combo.addItem("⚠ No se detectaron puertos COM", "")

        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(20, 20, 20, 20)

        title = QLabel("SIMISPACE — CONFIGURAR ENLACE SERIAL")
        title.setStyleSheet(f"color: {C['ink']}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
        layout.addWidget(title)

        form = QFormLayout()
        form.setSpacing(8)
        form.addRow("Puerto COM:", self.port_combo)
        form.addRow("Baudrate:", self.baud_spin)
        layout.addLayout(form)

        layout.addWidget(self.connect_btn)

        sep = QLabel("— O —")
        sep.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sep.setStyleSheet(f"color: {C['muted']}; font-size: 11px;")
        layout.addWidget(sep)
        layout.addWidget(self.sim_btn)

        self.sim_btn.clicked.connect(self._use_sim)
        self.connect_btn.clicked.connect(self._use_serial)

    def _use_sim(self):
        self._sim_selected = True
        self.accept()

    def _use_serial(self):
        self._sim_selected = False
        if not self.port_combo.currentData():
            QMessageBox.warning(self, "Sin puerto", "No hay un puerto COM válido seleccionado.")
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
        self.resize(1400, 900)
        self.setMinimumSize(900, 600)

        # ── Estado interno ───────────────────────────────────────────────────
        self._connected     = False
        self._worker        = None

        # MET: empieza en 0, solo avanza cuando se detecta Δaltitud > 2 m
        self._met_seconds   = 0
        self._met_running   = False
        self._alt_baseline  = None      # altitud en el momento del primer packet
        self._alt_trigger   = 2.0       # metros de cambio para arrancar el MET

        # ── Estilos globales ─────────────────────────────────────────────────
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background: {C['bg']}; color: {C['ink']}; }}
            QLabel {{ background: transparent; }}
            QScrollBar:vertical {{
                background: {C['panel_alt']};
                width: 6px;
                border: none;
            }}
            QScrollBar::handle:vertical {{
                background: {C['border']};
                border-radius: 3px;
            }}
        """)

        # ── Construir la UI ──────────────────────────────────────────────────
        self._build_ui()

        # ── Timer del MET (corre siempre, pero solo incrementa si _met_running)
        self._met_timer = QTimer(self)
        self._met_timer.timeout.connect(self._tick_met)
        self._met_timer.start(1000)

        # ── Abrir diálogo de conexión al iniciar ─────────────────────────────
        QTimer.singleShot(0, self._open_connection_dialog)

    # ─────────────────────────────────────────────────────────────────────────
    # CONSTRUCCIÓN DE LA UI
    # ─────────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        # ── HEADER ────────────────────────────────────────────────────────────
        header = self._build_header()
        main_layout.addWidget(header)

        # ── BODY (grid de paneles) ────────────────────────────────────────────
        body = QWidget()
        body_layout = QGridLayout(body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(10)

        # Cámara (placeholder)
        cam_panel = self._build_camera_panel()
        body_layout.addWidget(cam_panel, 0, 0, 1, 2)

        # Tarjetas de telemetría (columna derecha)
        telem_col = QWidget()
        telem_layout = QVBoxLayout(telem_col)
        telem_layout.setContentsMargins(0, 0, 0, 0)
        telem_layout.setSpacing(8)

        self._card_pressure    = TelemetryCard("Pressure",    "Pa",  "pressure",    decimals=0)
        self._card_temperature = TelemetryCard("Temperature", "°C",  "temperature", decimals=1)
        self._card_altitude    = TelemetryCard("Altitude",    "m",   "",            decimals=1)
        self._card_humidity    = TelemetryCard("Humidity",    "%",   "humidity",    decimals=0)

        for card in (self._card_pressure, self._card_temperature,
                     self._card_altitude, self._card_humidity):
            card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            telem_layout.addWidget(card)

        body_layout.addWidget(telem_col, 0, 2, 1, 1)

        # Posición + actitud
        pos_panel = self._build_position_panel()
        body_layout.addWidget(pos_panel, 1, 0, 1, 1)

        # Stats (batería / señal / comm)
        stats_panel = self._build_stats_panel()
        body_layout.addWidget(stats_panel, 1, 1, 1, 1)

        # Log
        self._log = LogConsole()
        body_layout.addWidget(self._log, 1, 2, 1, 1)

        body_layout.setColumnStretch(0, 3)
        body_layout.setColumnStretch(1, 2)
        body_layout.setColumnStretch(2, 3)
        body_layout.setRowStretch(0, 3)
        body_layout.setRowStretch(1, 2)

        main_layout.addWidget(body, stretch=1)

    def _make_panel(self, title: str) -> tuple[QFrame, QVBoxLayout]:
        """Crea un frame de panel estilizado y devuelve (frame, body_layout)."""
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

        header = QWidget()
        header.setFixedHeight(34)
        header.setStyleSheet(f"border-bottom: 1px solid {C['border']};")
        h_lay = QHBoxLayout(header)
        h_lay.setContentsMargins(12, 0, 12, 0)
        lbl = QLabel(title.upper())
        lbl.setStyleSheet(f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 2px; border: none;")
        h_lay.addWidget(lbl)

        body = QWidget()
        body_lay = QVBoxLayout(body)
        body_lay.setContentsMargins(10, 10, 10, 10)
        body_lay.setSpacing(8)

        outer.addWidget(header)
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

        # Logo + nombre
        logo_lbl = QLabel("◎")
        logo_lbl.setStyleSheet(f"color: {C['accent']}; font-size: 20px;")
        name_lbl = QLabel("SIMISPACE")
        name_lbl.setStyleSheet(f"color: {C['ink']}; font-size: 18px; font-weight: bold; letter-spacing: 3px;")
        sub_lbl = QLabel("· GROUND CONTROL — LIVE TELEMETRY")
        sub_lbl.setStyleSheet(f"color: {C['ink_dim']}; font-size: 12px;")

        layout.addWidget(logo_lbl)
        layout.addWidget(name_lbl)
        layout.addWidget(sub_lbl)
        layout.addStretch()

        # Separador
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet(f"color: {C['border']};")
        layout.addWidget(sep)

        # MET
        met_col = QVBoxLayout()
        met_col.setSpacing(0)
        met_lbl = QLabel("MET")
        met_lbl.setStyleSheet(f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 3px;")
        self._lbl_met = QLabel("00:00:00")
        self._lbl_met.setStyleSheet(f"color: {C['ink']}; font-size: 28px; font-weight: bold; font-family: 'Courier New', monospace;")
        met_col.addWidget(met_lbl)
        met_col.addWidget(self._lbl_met)
        layout.addLayout(met_col)

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.Shape.VLine)
        sep2.setStyleSheet(f"color: {C['border']};")
        layout.addWidget(sep2)

        # Status maestro
        status_col = QVBoxLayout()
        status_col.setSpacing(2)
        status_top = QLabel("STATUS")
        status_top.setStyleSheet(f"color: {C['ink_dim']}; font-size: 10px; letter-spacing: 2px;")
        row = QHBoxLayout()
        self._lbl_status = QLabel("WAITING")
        self._lbl_status.setStyleSheet(f"color: {C['muted']}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
        self._dot_status = QLabel("●")
        self._dot_status.setStyleSheet(f"color: {C['muted']}; font-size: 14px;")
        row.addWidget(self._lbl_status)
        row.addWidget(self._dot_status)
        row.addStretch()
        status_col.addWidget(status_top)
        status_col.addLayout(row)
        layout.addLayout(status_col)

        sep3 = QFrame()
        sep3.setFrameShape(QFrame.Shape.VLine)
        sep3.setStyleSheet(f"color: {C['border']};")
        layout.addWidget(sep3)

        # Botón de (re)conexión
        self._btn_connect = QPushButton("⚡ Cambiar Conexión")
        self._btn_connect.setFixedHeight(34)
        self._btn_connect.setStyleSheet(f"""
            QPushButton {{
                background: {C['panel_alt']};
                color: {C['accent']};
                border: 1px solid {C['border']};
                border-radius: 4px;
                padding: 0 14px;
                font-size: 11px;
            }}
            QPushButton:hover {{ background: {C['border']}; }}
        """)
        self._btn_connect.clicked.connect(self._open_connection_dialog)
        layout.addWidget(self._btn_connect)

        return container

    def _build_camera_panel(self) -> QFrame:
        frame, body = self._make_panel("Onboard Camera Feed — Live")

        # Placeholder de cámara (fondo negro con cuadrícula sutil + mira)
        cam_widget = QWidget()
        cam_widget.setMinimumHeight(240)
        cam_widget.setStyleSheet("background: #040810; border-radius: 4px;")

        class CamPlaceholder(QWidget):
            def paintEvent(self_inner, event):
                p = QPainter(self_inner)
                p.setRenderHint(QPainter.RenderHint.Antialiasing)
                w, h = self_inner.width(), self_inner.height()
                p.fillRect(0, 0, w, h, QColor("#040810"))

                # Cuadrícula
                pen = QPen(QColor(C["border"]), 1)
                p.setPen(pen)
                step = 30
                for x in range(0, w, step):
                    p.drawLine(x, 0, x, h)
                for y in range(0, h, step):
                    p.drawLine(0, y, w, y)

                # Brillo de limbo inferior
                grad = QLinearGradient(0, h * 0.5, 0, h)
                c1 = QColor(C["accent"])
                c1.setAlpha(40)
                c2 = QColor(C["accent"])
                c2.setAlpha(0)
                grad.setColorAt(0, c2)
                grad.setColorAt(1, c1)
                p.fillRect(0, int(h * 0.5), w, h, QBrush(grad))

                # Esquinas
                corner_len = 20
                corner_pen = QPen(QColor("rgba(255,255,255,0.5)"), 1.5)
                p.setPen(corner_pen)
                for cx_, cy_, dx, dy in [
                    (0, 0, 1, 1), (w, 0, -1, 1), (0, h, 1, -1), (w, h, -1, -1)
                ]:
                    p.drawLine(cx_, cy_, cx_ + dx * corner_len, cy_)
                    p.drawLine(cx_, cy_, cx_, cy_ + dy * corner_len)

                # Mira central
                cx_, cy_ = w // 2, h // 2
                mira_pen = QPen(QColor("rgba(255,255,255,0.6)"), 1)
                p.setPen(mira_pen)
                p.drawLine(cx_ - 40, cy_, cx_ + 40, cy_)
                p.drawLine(cx_, cy_ - 40, cx_, cy_ + 40)
                p.drawEllipse(QPointF(cx_, cy_), 22, 22)

                p.end()

        self._cam_widget = CamPlaceholder()
        self._cam_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Overlay de datos en la cámara
        overlay_layout = QHBoxLayout()
        self._cam_alt_lbl = QLabel("ALT: 0.0 m")
        self._cam_hdg_lbl = QLabel("HDG: 000°")
        self._cam_live_lbl = QLabel("● WAITING")
        for lbl in (self._cam_alt_lbl, self._cam_hdg_lbl, self._cam_live_lbl):
            lbl.setStyleSheet(f"color: {C['ink_dim']}; font-family: 'Courier New'; font-size: 11px;")
        overlay_layout.addWidget(self._cam_alt_lbl)
        overlay_layout.addWidget(self._cam_hdg_lbl)
        overlay_layout.addStretch()
        overlay_layout.addWidget(self._cam_live_lbl)

        body.addWidget(self._cam_widget, stretch=1)
        body.addLayout(overlay_layout)

        return frame

    def _build_position_panel(self) -> QFrame:
        frame, body = self._make_panel("Position & Trajectory")

        # Lecturas de posición
        self._pos_labels = {}
        pos_grid = QGridLayout()
        pos_grid.setSpacing(4)
        for i, (key, unit) in enumerate([
            ("LAT", "°"), ("LON", "°"), ("HDG", "°"), ("ROLL", "°"), ("PITCH", "°"),
        ]):
            lbl_key = QLabel(key)
            lbl_key.setStyleSheet(f"color: {C['muted']}; font-size: 10px; letter-spacing: 2px;")
            lbl_val = QLabel("—")
            lbl_val.setStyleSheet(f"color: {C['accent']}; font-size: 11px; font-family: 'Courier New';")
            lbl_val.setAlignment(Qt.AlignmentFlag.AlignRight)
            pos_grid.addWidget(lbl_key, i, 0)
            pos_grid.addWidget(lbl_val, i, 1)
            self._pos_labels[key] = lbl_val

        body.addLayout(pos_grid)

        # Instrumentos visuales
        viz_row = QHBoxLayout()
        viz_row.setSpacing(8)

        traj_container = QVBoxLayout()
        traj_lbl = QLabel("TRAJECTORY")
        traj_lbl.setStyleSheet(f"color: {C['muted']}; font-size: 9px; letter-spacing: 2px;")
        traj_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._trajectory = TrajectoryWidget()
        traj_container.addWidget(self._trajectory)
        traj_container.addWidget(traj_lbl)

        att_container = QVBoxLayout()
        att_lbl = QLabel("ATTITUDE")
        att_lbl.setStyleSheet(f"color: {C['muted']}; font-size: 9px; letter-spacing: 2px;")
        att_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._attitude = AttitudeWidget()
        att_container.addWidget(self._attitude)
        att_container.addWidget(att_lbl)

        viz_row.addLayout(traj_container)
        viz_row.addLayout(att_container)
        body.addLayout(viz_row)

        return frame

    def _build_stats_panel(self) -> QFrame:
        frame, body = self._make_panel("Vehicle Status")

        self._stat_signal  = StatCard("Signal")
        self._stat_link    = StatCard("Comm Link")
        self._stat_packets = StatCard("Packets RX")
        self._stat_port    = StatCard("Port")

        for s in (self._stat_signal, self._stat_link,
                  self._stat_packets, self._stat_port):
            s.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            body.addWidget(s)

        self._stat_port.set_value("—", C["muted"])
        self._stat_signal.set_value("—", C["muted"])
        self._stat_link.set_value("DISCONNECTED", C["red"])
        self._stat_packets.set_value("0", C["ink_dim"])

        self._packet_count = 0
        return frame

    # ─────────────────────────────────────────────────────────────────────────
    # LÓGICA DE CONEXIÓN
    # ─────────────────────────────────────────────────────────────────────────

    def _open_connection_dialog(self):
        self._stop_worker()
        dlg = SerialDialog(self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return

        if dlg.use_sim:
            self._log.log("INFO", "Iniciando modo simulación (sin hardware)", self._fmt_met())
            self._worker = SimWorker()
            self._stat_port.set_value("SIM", C["accent"])
        else:
            port  = dlg.selected_port
            baud  = dlg.selected_baud
            self._log.log("INFO", f"Conectando a {port} @ {baud} bps…", self._fmt_met())
            self._worker = SerialWorker(port, baud)
            self._stat_port.set_value(f"{port} @ {baud}", C["accent"])

        self._worker.packet_received.connect(self._on_packet)
        self._worker.raw_line.connect(self._on_raw_line)
        self._worker.connection_changed.connect(self._on_connection_changed)
        self._worker.error.connect(self._on_error)
        self._worker.start()

        # Reiniciar el MET y la baseline de altitud
        self._met_seconds  = 0
        self._met_running  = False
        self._alt_baseline = None
        self._packet_count = 0
        self._lbl_met.setText("00:00:00")

    def _stop_worker(self):
        if self._worker is not None:
            self._worker.stop()
            self._worker = None

    # ─────────────────────────────────────────────────────────────────────────
    # SLOTS DE SEÑALES DEL WORKER
    # ─────────────────────────────────────────────────────────────────────────

    def _on_connection_changed(self, connected: bool):
        self._connected = connected
        if connected:
            self._lbl_status.setText("NOMINAL")
            self._lbl_status.setStyleSheet(
                f"color: {C['green']}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
            self._dot_status.setStyleSheet(f"color: {C['green']}; font-size: 14px;")
            self._stat_link.set_value("ACTIVE", C["accent"])
            self._stat_signal.set_value("GOOD", C["green"])
        else:
            self._lbl_status.setText("DISCONNECTED")
            self._lbl_status.setStyleSheet(
                f"color: {C['red']}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
            self._dot_status.setStyleSheet(f"color: {C['red']}; font-size: 14px;")
            self._stat_link.set_value("LOST", C["red"])
            self._stat_signal.set_value("LOST", C["red"])
            self._cam_live_lbl.setText("○ SIGNAL LOST")
            self._cam_live_lbl.setStyleSheet(
                f"color: {C['red']}; font-family: 'Courier New'; font-size: 11px;")

    def _on_error(self, msg: str):
        self._log.log("CRIT", msg, self._fmt_met())

    def _on_raw_line(self, line: str):
        # Mostrar líneas crudas en el log solo si no son paquetes válidos
        # (para no saturar la consola con duplicados, ya que los paquetes
        # válidos generan su propio log en _on_packet).
        if not line.startswith("TELEM"):
            self._log.log("RAW", line, self._fmt_met())

    def _on_packet(self, pkt: TelemetryPacket):
        """
        Slot principal: recibe un TelemetryPacket parseado del worker y
        actualiza todos los widgets de la UI.
        También contiene la lógica de arranque del MET por Δaltitud.
        """
        # ── Contador de paquetes ────────────────────────────────────────────
        self._packet_count += 1
        self._stat_packets.set_value(str(self._packet_count), C["ink_dim"])

        # ── Actualizar cámara overlay ────────────────────────────────────────
        self._cam_alt_lbl.setText(f"ALT: {pkt.altitude:.1f} m")
        self._cam_hdg_lbl.setText(f"HDG: {pkt.heading:.0f}°")
        self._cam_live_lbl.setText("● LIVE")
        self._cam_live_lbl.setStyleSheet(
            f"color: {C['accent']}; font-family: 'Courier New'; font-size: 11px;")

        # ── Telemetría ───────────────────────────────────────────────────────
        self._card_pressure.update_value(pkt.pressure)
        self._card_temperature.update_value(pkt.temperature)
        self._card_altitude.update_value(pkt.altitude)
        self._card_humidity.update_value(pkt.humidity)

        # ── Posición y actitud ───────────────────────────────────────────────
        self._pos_labels["LAT"].setText(f"{pkt.lat:.5f}°")
        self._pos_labels["LON"].setText(f"{pkt.lon:.5f}°")
        self._pos_labels["HDG"].setText(f"{pkt.heading:.1f}°")
        self._pos_labels["ROLL"].setText(f"{pkt.roll:.1f}°")
        self._pos_labels["PITCH"].setText(f"{pkt.pitch:.1f}°")
        self._trajectory.push(pkt.lat, pkt.lon, pkt.heading)
        self._attitude.set_attitude(pkt.roll, pkt.pitch)

        # ── Status maestro ───────────────────────────────────────────────────
        statuses = [
            get_status("pressure", pkt.pressure),
            get_status("temperature", pkt.temperature),
            get_status("humidity", pkt.humidity),
        ]
        if "critical" in statuses:
            master = "critical"
        elif "caution" in statuses:
            master = "caution"
        else:
            master = "nominal"

        if master != "nominal":
            col = C["red"] if master == "critical" else C["amber"]
            self._lbl_status.setText(master.upper())
            self._lbl_status.setStyleSheet(
                f"color: {col}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
            self._dot_status.setStyleSheet(f"color: {col}; font-size: 14px;")
        elif self._connected:
            self._lbl_status.setText("NOMINAL")
            self._lbl_status.setStyleSheet(
                f"color: {C['green']}; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
            self._dot_status.setStyleSheet(f"color: {C['green']}; font-size: 14px;")

        # ── LÓGICA DEL MET: arranca cuando Δaltitud > 2 m ──────────────────
        # El primer paquete establece la baseline de altitud.
        # Cuando la altitud sube más de self._alt_trigger metros respecto a
        # esa baseline, el MET comienza a contar. Una vez iniciado, no se detiene
        # aunque la altitud baje (para mantener la coherencia de la misión).
        if not self._met_running:
            if self._alt_baseline is None:
                self._alt_baseline = pkt.altitude
                self._log.log("INFO",
                    f"Baseline de altitud establecida: {pkt.altitude:.1f} m — "
                    f"MET arrancará con Δ > {self._alt_trigger} m",
                    "00:00:00")
            else:
                delta = pkt.altitude - self._alt_baseline
                if delta >= self._alt_trigger:
                    self._met_running = True
                    self._log.log("INFO",
                        f"¡Despegue detectado! Δaltitud = {delta:.2f} m → MET iniciado",
                        "00:00:00")

    # ─────────────────────────────────────────────────────────────────────────
    # TIMER DEL MET
    # ─────────────────────────────────────────────────────────────────────────

    def _tick_met(self):
        if self._met_running:
            self._met_seconds += 1
            self._lbl_met.setText(self._fmt_met())

    def _fmt_met(self) -> str:
        s = self._met_seconds
        h = s // 3600
        m = (s % 3600) // 60
        sec = s % 60
        return f"{h:02d}:{m:02d}:{sec:02d}"

    # ─────────────────────────────────────────────────────────────────────────
    # CIERRE LIMPIO
    # ─────────────────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        self._stop_worker()
        event.accept()


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    app = QApplication(sys.argv)
    app.setApplicationName("SIMISPACE GCS")

    # Fuente monoespaciada global para los datos numéricos
    app.setFont(QFont("Segoe UI", 10))

    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
