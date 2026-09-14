# -*- coding: utf-8 -*-
"""
Simulador de control PID para un motor de corriente directa (DC)
================================================================

La aplicacion reproduce el diagrama de bloques del control a lazo cerrado
basico:

    r(t) ---->(S)----> e(t) ----> [ PID ] ----> u(t) ----> [ Motor DC ] ----> y(t)
               ^ -                                                             |
               |___________________ realimentacion __________________________|

Modelo de la planta (motor DC controlado por armadura):

    La * dia/dt = Va - Ra*ia - Ke*w        (malla electrica de armadura)
    J  * dw/dt  = Kt*ia - B*w - TL         (equilibrio mecanico del rotor)
    tau         = Kt * ia                  (par o torque del motor)

donde  Va = u(t)  es la salida del controlador (voltaje de armadura) y
       y(t) = w   es la velocidad angular medida y realimentada.

Requiere: PyQt5, pyqtgraph y numpy.
Ejecutar con:  .venv\\Scripts\\python simulador.py   (o el archivo ejecutar.bat)

La vista 3D del motor se dibuja por software con QPainter (proyeccion en
perspectiva + algoritmo del pintor), asi que no depende de OpenGL ni del
controlador de video.

La tercera vista, "Control de Motor Fisico", deja la simulacion a un lado y
gobierna un motor real: Arduino + puente H L298N + sensor optico de ranura.
Se comunica por el puerto serie con el sketch del microcontrolador, que
acepta ordenes de una linea (rpm, pwm, kp, ki, kd, s, r, m) y devuelve un
reporte periodico con las RPM medidas, el PWM aplicado y los pulsos contados.
"""

from __future__ import annotations

import math
import re
import sys
from dataclasses import dataclass
from string import Template

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QElapsedTimer, Qt, QTimer, pyqtSignal

# QtSerialPort viaja en la rueda oficial de PyQt5, pero no en todos los
# empaquetados: si falta, la vista del motor real lo avisa y el resto del
# simulador sigue funcionando igual.
try:
    from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
except ImportError:                     # pragma: no cover
    QSerialPort = QSerialPortInfo = None


# =====================================================================
#  Paleta y hoja de estilo
# =====================================================================

# ---------------------------------------------------------------------
#  Presupuesto de tiempo del bucle principal
# ---------------------------------------------------------------------
#  La aplicacion es de un solo hilo: si un fotograma tarda mas que su
#  periodo, el bucle de eventos de Qt nunca se vacia y Windows marca la
#  ventana como "no responde".  Estas constantes separan el ritmo de la
#  simulacion del ritmo de los repintados y acotan el trabajo por
#  fotograma, de modo que la interfaz siempre conserva tiempo libre.
MS_SIMULACION = 16          # periodo del bucle de simulacion            [ms]
MS_GRAFICAS = 33            # refresco de curvas y telemetria (~30 fps)  [ms]
MS_MOTOR_3D = 50            # refresco de la vista 3D (~20 fps)          [ms]
PUNTOS_CURVA = 700          # muestras dibujadas por curva
CAPACIDAD_HISTORIAL = 20000 # muestras guardadas del ensayo completo
SUBPASOS_FOTOGRAMA = 6000   # tope de integraciones RK4 por fotograma

# Grafica del motor real.  No tiene ventana deslizante: guarda todo el
# historial y lo comprime en el eje de tiempo, asi que el dibujo se diezma
# (conservando picos) y su refresco se acota aparte del de la vista 3D.
MS_GRAFICA_FISICA = 100       # refresco maximo de la grafica           [ms]
PUNTOS_GRAFICA_FISICA = 2000  # tope de puntos dibujados de la curva
T_MIN_GRAFICA_FISICA = 10.0   # ancho minimo del eje de tiempo           [s]


C = {
    "fondo":     "#0E1116",
    "panel":     "#161A22",
    "panel_alt": "#1D222C",
    "borde":     "#2A3140",
    "texto":     "#E6EAF2",
    "texto_dim": "#8B95A7",
    "acento":    "#4D8DFF",
    "ref":       "#8B95A7",
    "salida":    "#2ED3A7",
    "error":     "#FF7A6B",
    "control":   "#FFB84D",
    "stop":      "#FF5E57",
}

QSS_PLANTILLA = """
* { font-family: 'Segoe UI', 'Inter', 'Arial'; }
QWidget { color: $texto; font-size: 12px; }
QMainWindow, #Fondo { background: $fondo; }
QLabel { background: transparent; }

#Tarjeta   { background: $panel; border: 1px solid $borde; border-radius: 14px; }
#Chip      { background: $panel_alt; border: 1px solid $borde; border-radius: 10px; }
#Separador { background: $borde; max-height: 1px; min-height: 1px; border: none; }
#ContenedorScroll { background: transparent; }

#Titulo        { font-size: 18px; font-weight: 700; color: $texto; }
#Subtitulo     { color: $texto_dim; font-size: 11px; }
#TituloGrupo   { color: $texto_dim; font-size: 10px; font-weight: 700; letter-spacing: 1.2px; }
#EtiquetaCampo { color: $texto_dim; font-size: 11px; }
#Pista         { color: $texto_dim; font-size: 10px; }
#NombreMetrica { color: $texto_dim; font-size: 10px; letter-spacing: 0.4px; }
#Formula       { font-family: 'Consolas', 'Segoe UI'; font-size: 12px; color: $texto; }
#Pestanas      { background: $panel_alt; border: 1px solid $borde; border-radius: 12px; }
#ValorMetrica  { font-family: 'Consolas', 'Segoe UI'; font-size: 15px; font-weight: 700; }

QDoubleSpinBox, QSpinBox, QComboBox, QLineEdit {
    background: $panel_alt; border: 1px solid $borde; border-radius: 9px;
    padding: 5px 9px; color: $texto; selection-background-color: $acento;
}
QDoubleSpinBox:focus, QSpinBox:focus, QComboBox:focus { border: 1px solid $acento; }
QDoubleSpinBox:disabled, QSpinBox:disabled, QComboBox:disabled {
    color: #59637A; background: #171B23; border: 1px solid #212736;
}
QComboBox::drop-down { border: none; width: 18px; }
QComboBox QAbstractItemView {
    background: $panel_alt; border: 1px solid $borde; border-radius: 9px; padding: 4px;
    selection-background-color: $acento; selection-color: #071426; outline: none;
}

QPushButton { border: none; border-radius: 12px; padding: 9px 16px; font-weight: 600; }
#BtnPrimario          { background: $acento; color: #071426; }
#BtnPrimario:hover    { background: #6AA1FF; }
#BtnPrimario:pressed  { background: #3C7BEB; }
#BtnPrimario:disabled { background: #1E2836; color: #4E5A6E; }
#BtnPeligro           { background: $stop; color: #2A0A08; }
#BtnPeligro:hover     { background: #FF7B75; }
#BtnPeligro:pressed   { background: #E64C46; }
#BtnPeligro:disabled  { background: #26191B; color: #6B4B4B; }
#BtnVista             { background: transparent; color: $texto_dim; border-radius: 9px;
                        padding: 8px 18px; }
#BtnVista:hover       { color: $texto; }
#BtnVista:checked     { background: $acento; color: #071426; }
#BtnNeutro            { background: $panel_alt; color: $texto; border: 1px solid $borde; }
#BtnNeutro:hover      { background: #262D3A; }
#BtnNeutro:disabled   { color: #4E5A6E; }

QSlider::groove:horizontal   { height: 6px; background: $panel_alt; border-radius: 3px; }
QSlider::sub-page:horizontal { background: $acento; border-radius: 3px; }
QSlider::add-page:horizontal { background: $panel_alt; border-radius: 3px; }
QSlider::handle:horizontal   { background: $texto; width: 15px; height: 15px;
                               margin: -5px 0; border-radius: 7px; }
QSlider::handle:horizontal:hover      { background: #FFFFFF; }
QSlider::groove:horizontal:disabled   { background: #1A1F29; }
QSlider::sub-page:horizontal:disabled { background: #2B3646; }

QCheckBox { spacing: 8px; color: $texto_dim; font-size: 11px; }
QCheckBox::indicator { width: 15px; height: 15px; border-radius: 5px;
                       border: 1px solid $borde; background: $panel_alt; }
QCheckBox::indicator:checked { background: $acento; border: 1px solid $acento; }

QScrollArea { background: transparent; border: none; }
QScrollArea > QWidget > QWidget { background: transparent; }
QScrollBar:vertical { background: transparent; width: 9px; margin: 2px 0; }
QScrollBar::handle:vertical { background: $borde; border-radius: 4px; min-height: 28px; }
QScrollBar::handle:vertical:hover { background: #3B475F; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
QScrollBar:horizontal { height: 0px; }

#ValorGrande   { font-family: 'Consolas', 'Segoe UI'; font-weight: 700; }
#UnidadGrande  { color: $texto_dim; font-size: 13px; font-weight: 700; letter-spacing: 1.5px; }
#InfoPuerto    { font-family: 'Consolas', 'Cascadia Mono'; font-size: 11px; color: $texto_dim; }

QPlainTextEdit#Consola {
    background: #0A0D12; border: 1px solid $borde; border-radius: 10px; padding: 6px;
    font-family: 'Consolas', 'Cascadia Mono'; font-size: 11px; color: #9FB0C8;
    selection-background-color: $acento;
}

QToolTip { background: $panel_alt; color: $texto; border: 1px solid $borde;
           border-radius: 8px; padding: 5px; }
"""


def qcolor(hexa: str, alfa: int = 255) -> QtGui.QColor:
    c = QtGui.QColor(hexa)
    c.setAlpha(alfa)
    return c


def escala(x: float, unidad: str) -> str:
    """Formatea una magnitud que puede valer decimas o millares.

    La aceleracion angular pasa de ~1 rad/s2 en regimen a varios miles
    durante un escalon, asi que un numero fijo de decimales o la deja en
    cero o la llena de digitos inutiles.
    """
    a = abs(x)
    if a < 100.0:
        return "%.2f %s" % (x, unidad)
    if a < 10000.0:
        return "%.1f %s" % (x, unidad)
    return "%.0f %s" % (x, unidad)


def rgba(hexa: str, alfa: float = 1.0):
    c = QtGui.QColor(hexa)
    return (c.redF(), c.greenF(), c.blueF(), alfa)


# =====================================================================
#  Modelo de la planta:  motor DC controlado por armadura
# =====================================================================

@dataclass
class ParametrosMotor:
    """Parametros fisicos del motor DC."""
    Ra: float = 1.00         # resistencia de armadura                  [ohm]
    La: float = 0.010        # inductancia de armadura                  [H]
    Kt: float = 0.100        # constante de par                         [N.m/A]
    Ke: float = 0.100        # constante de fuerza contraelectromotriz  [V.s/rad]
    J: float = 0.010         # momento de inercia del rotor             [kg.m^2]
    B: float = 0.005         # coeficiente de friccion viscosa          [N.m.s/rad]
    TL: float = 0.000        # par de carga externo                     [N.m]
    Va_max: float = 24.0     # voltaje de armadura maximo (saturacion)  [V]
    ia0: float = 0.0         # corriente de armadura inicial            [A]
    w0: float = 0.0          # velocidad angular inicial                [rad/s]

    def sanear(self) -> "ParametrosMotor":
        self.Ra = max(float(self.Ra), 1e-4)
        self.La = max(float(self.La), 1e-6)
        self.J = max(float(self.J), 1e-7)
        self.B = max(float(self.B), 0.0)
        self.Va_max = max(float(self.Va_max), 1e-3)
        return self


class MotorDC:
    """Integra el modelo de 2.o orden del motor con Runge-Kutta 4."""

    def __init__(self, p: ParametrosMotor):
        self.p = p
        self.reiniciar()

    def reiniciar(self) -> None:
        self.ia = float(self.p.ia0)      # corriente de armadura  [A]
        self.w = float(self.p.w0)        # velocidad angular      [rad/s]
        self.theta = 0.0                 # posicion angular       [rad]

    def _derivadas(self, ia: float, w: float, va: float):
        p = self.p
        dia = (va - p.Ra * ia - p.Ke * w) / p.La
        dw = (p.Kt * ia - p.B * w - p.TL) / p.J
        return dia, dw

    def avanzar(self, va: float, dt: float, subpasos: int = 1) -> None:
        """Avanza dt segundos con Va constante (retenedor de orden cero)."""
        h = dt / subpasos
        for _ in range(subpasos):
            ia, w = self.ia, self.w
            k1a, k1w = self._derivadas(ia, w, va)
            k2a, k2w = self._derivadas(ia + 0.5 * h * k1a, w + 0.5 * h * k1w, va)
            k3a, k3w = self._derivadas(ia + 0.5 * h * k2a, w + 0.5 * h * k2w, va)
            k4a, k4w = self._derivadas(ia + h * k3a, w + h * k3w, va)
            self.ia = ia + (h / 6.0) * (k1a + 2.0 * k2a + 2.0 * k3a + k4a)
            self.w = w + (h / 6.0) * (k1w + 2.0 * k2w + 2.0 * k3w + k4w)
            self.theta += 0.5 * (w + self.w) * h

    @property
    def par(self) -> float:
        """Par o torque desarrollado por el motor [N.m]."""
        return self.p.Kt * self.ia

    @property
    def par_friccion(self) -> float:
        """Par resistente de la friccion viscosa,  tau_f = B * w  [N.m]."""
        return self.p.B * self.w

    @property
    def aceleracion(self) -> float:
        """Aceleracion angular  dw/dt = (tau - tau_f - TL) / J  [rad/s^2].

        Es la misma expresion que integra _derivadas; se expone aparte para
        poder mostrarla sin repetir la formula en la interfaz.
        """
        return (self.par - self.par_friccion - self.p.TL) / self.p.J

    @property
    def estable(self) -> bool:
        return math.isfinite(self.ia) and math.isfinite(self.w)

    def subpasos_para(self, dt: float) -> int:
        """Subdivisiones de RK4 segun las constantes de tiempo del motor."""
        p = self.p
        tau_e = p.La / p.Ra                              # constante electrica
        b_eq = p.B + p.Kt * p.Ke / p.Ra
        tau_m = p.J / max(b_eq, 1e-9)                    # constante mecanica
        h_max = 0.15 * min(tau_e, tau_m)
        return int(min(400, max(1, math.ceil(dt / max(h_max, 1e-12)))))


# =====================================================================
#  Controlador PID
# =====================================================================

class ControladorPID:
    """PID en paralelo, con derivada filtrada y anti-windup."""

    def __init__(self, kp=0.15, ki=2.0, kd=0.005, u_min=-24.0, u_max=24.0,
                 n_filtro=20.0, anti_windup=True, derivada_medicion=True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.u_min = u_min
        self.u_max = u_max
        self.n_filtro = n_filtro
        self.anti_windup = anti_windup
        self.derivada_medicion = derivada_medicion
        self.reiniciar()

    def reiniciar(self) -> None:
        self.integral = 0.0
        self.d_filtrada = 0.0
        self.e_ant = None
        self.y_ant = None
        self.P = self.I = self.D = 0.0
        self.saturado = False

    def limites(self, u_min: float, u_max: float) -> None:
        self.u_min, self.u_max = u_min, u_max

    def calcular(self, r: float, y: float, dt: float):
        """Devuelve (u, e) para la muestra actual."""
        e = r - y

        # --- accion proporcional
        P = self.kp * e

        # --- accion derivativa con filtro pasa-bajas de primer orden
        if self.derivada_medicion:
            bruta = -(y - self.y_ant) / dt if self.y_ant is not None else 0.0
        else:
            bruta = (e - self.e_ant) / dt if self.e_ant is not None else 0.0
        if self.kd > 0.0:
            tf = self.kd / max(self.n_filtro, 1e-3)
            alfa = dt / (tf + dt)
            self.d_filtrada += alfa * (self.kd * bruta - self.d_filtrada)
        else:
            self.d_filtrada = 0.0
        D = self.d_filtrada

        # --- accion integral con anti-windup (integracion condicional)
        integral = self.integral + self.ki * e * dt
        u = P + integral + D
        u_sat = min(max(u, self.u_min), self.u_max)
        if self.anti_windup and u != u_sat and (u - u_sat) * e > 0.0:
            integral = self.integral                  # se congela el integrador
            u = P + integral + D
            u_sat = min(max(u, self.u_min), self.u_max)

        self.integral = integral
        self.e_ant, self.y_ant = e, y
        self.P, self.I, self.D = P, integral, D
        self.saturado = (u != u_sat)
        return u_sat, e


# =====================================================================
#  Historial de senales (ensayo completo con memoria acotada)
# =====================================================================

class Historial:
    """Guarda la respuesta entera desde t = 0 sin que la memoria crezca.

    Cuando el buffer se llena se queda con una de cada dos muestras y, desde
    ahi, solo anota una de cada 'cada' llamadas.  La resolucion temporal baja
    a medida que el ensayo se alarga -- igual que la de la grafica, que lo
    comprime todo en el mismo ancho --, pero el transitorio inicial nunca se
    descarta, como pasaba con el antiguo buffer de ventana deslizante.
    """
    CAMPOS = ("t", "r", "y", "e", "u", "ia", "tau")

    def __init__(self, capacidad: int = CAPACIDAD_HISTORIAL):
        self.cap = max(64, int(capacidad)) // 2 * 2    # par: se parte en dos
        self.buf = {c: np.zeros(self.cap, dtype=np.float64) for c in self.CAMPOS}
        self.idx = 0
        self.cada = 1          # se anota una muestra de cada 'cada' llamadas
        self._llamadas = 0

    def limpiar(self) -> None:
        for c in self.CAMPOS:
            self.buf[c][:] = 0.0
        self.idx = 0
        self.cada = 1
        self._llamadas = 0

    def agregar(self, **datos) -> None:
        omitir = self._llamadas % self.cada
        self._llamadas += 1
        if omitir:
            return
        if self.idx >= self.cap:
            # Las muestras que quedan (0, 2, 4...) caen justo en el nuevo paso
            # de 2*cada, y la que llega ahora tambien: la serie sigue uniforme.
            mitad = self.cap // 2
            for c in self.CAMPOS:
                self.buf[c][:mitad] = self.buf[c][0:self.cap:2].copy()
            self.idx = mitad
            self.cada *= 2
        i = self.idx
        for c, v in datos.items():
            self.buf[c][i] = v
        self.idx = i + 1

    def vista(self, campo: str) -> np.ndarray:
        return self.buf[campo][:self.idx]

    def __len__(self) -> int:
        return self.idx


# =====================================================================
#  Piezas reutilizables de la interfaz
# =====================================================================

def campo_num(minimo, maximo, valor, decimales=3, paso=0.1, ancho=94):
    sb = QtWidgets.QDoubleSpinBox()
    sb.setRange(minimo, maximo)
    sb.setDecimals(decimales)
    sb.setSingleStep(paso)
    sb.setValue(valor)
    sb.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
    sb.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    sb.setKeyboardTracking(False)
    sb.setFixedWidth(ancho)
    return sb


def etiqueta_campo(nombre, unidad=""):
    txt = nombre
    if unidad:
        txt += '&nbsp;&nbsp;<span style="color:#5F6979">%s</span>' % unidad
    lb = QtWidgets.QLabel(txt)
    lb.setObjectName("EtiquetaCampo")
    lb.setTextFormat(Qt.RichText)
    lb.setWordWrap(True)
    return lb


def separador():
    ln = QtWidgets.QFrame()
    ln.setObjectName("Separador")
    ln.setFixedHeight(1)
    return ln


class Tarjeta(QtWidgets.QFrame):
    """Panel con esquinas redondeadas y titulo opcional."""

    def __init__(self, titulo=None, parent=None):
        super().__init__(parent)
        self.setObjectName("Tarjeta")
        self.cuerpo = QtWidgets.QVBoxLayout(self)
        self.cuerpo.setContentsMargins(14, 12, 14, 13)
        self.cuerpo.setSpacing(10)
        if titulo:
            lb = QtWidgets.QLabel(titulo.upper())
            lb.setObjectName("TituloGrupo")
            self.cuerpo.addWidget(lb)

    def agregar(self, elemento, stretch=0):
        if isinstance(elemento, QtWidgets.QLayout):
            self.cuerpo.addLayout(elemento, stretch)
        else:
            self.cuerpo.addWidget(elemento, stretch)
        return elemento


class Rejilla(QtWidgets.QWidget):
    """Filas 'etiqueta  -  campo' alineadas."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.g = QtWidgets.QGridLayout(self)
        self.g.setContentsMargins(0, 0, 0, 0)
        self.g.setHorizontalSpacing(8)
        self.g.setVerticalSpacing(7)
        self.g.setColumnStretch(0, 1)
        self._fila = 0

    def agregar(self, nombre, unidad, widget, tip=""):
        lb = etiqueta_campo(nombre, unidad)
        if tip:
            lb.setToolTip(tip)
            widget.setToolTip(tip)
        self.g.addWidget(lb, self._fila, 0)
        self.g.addWidget(widget, self._fila, 1, Qt.AlignRight)
        self._fila += 1
        return widget


class Metrica(QtWidgets.QFrame):
    """Casilla compacta con nombre y valor numerico."""

    def __init__(self, nombre, color=None, parent=None):
        super().__init__(parent)
        self.setObjectName("Chip")
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(10, 6, 10, 7)
        v.setSpacing(1)
        lb = QtWidgets.QLabel(nombre)
        lb.setObjectName("NombreMetrica")
        self.valor = QtWidgets.QLabel("--")
        self.valor.setObjectName("ValorMetrica")
        self.valor.setStyleSheet("color: %s;" % (color or C["texto"]))
        v.addWidget(lb)
        v.addWidget(self.valor)

    def set(self, texto):
        self.valor.setText(texto)


class MetricaGrande(QtWidgets.QFrame):
    """Lectura destacada: numero grande, unidad y nombre del canal.

    Es la misma casilla que Metrica, con el cuerpo del numero ampliado: en
    la vista del motor real la velocidad tiene que leerse a distancia, con
    el equipo montado en la mesa de trabajo.
    """

    def __init__(self, nombre, unidad="", color=None, tam=30, parent=None):
        super().__init__(parent)
        self.setObjectName("Chip")
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(12, 7, 12, 9)
        v.setSpacing(2)

        lb = QtWidgets.QLabel(nombre)
        lb.setObjectName("NombreMetrica")

        fila = QtWidgets.QHBoxLayout()
        fila.setSpacing(7)
        self.valor = QtWidgets.QLabel("--")
        self.valor.setObjectName("ValorGrande")
        self.valor.setStyleSheet("font-size:%dpx; color:%s;"
                                 % (tam, color or C["texto"]))
        fila.addWidget(self.valor, 0, Qt.AlignBottom)
        if unidad:
            lb_u = QtWidgets.QLabel(unidad)
            lb_u.setObjectName("UnidadGrande")
            fila.addWidget(lb_u, 0, Qt.AlignBottom)
        fila.addStretch(1)

        v.addWidget(lb)
        v.addLayout(fila)

    def set(self, texto):
        self.valor.setText(texto)


class ControlGanancia(QtWidgets.QWidget):
    """Slider + campo numerico sincronizados para una ganancia del PID."""

    cambiado = pyqtSignal(float)

    def __init__(self, nombre, descripcion, minimo, maximo, valor,
                 decimales, color, parent=None):
        super().__init__(parent)
        self.escala = 10 ** decimales

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(5)

        fila = QtWidgets.QHBoxLayout()
        fila.setSpacing(8)
        punto = QtWidgets.QLabel()
        punto.setFixedSize(8, 8)
        punto.setStyleSheet("background:%s; border-radius:4px;" % color)
        lb = QtWidgets.QLabel(nombre)
        lb.setStyleSheet("font-weight:700; font-size:13px; color:%s;" % C["texto"])
        lb_desc = QtWidgets.QLabel(descripcion)
        lb_desc.setObjectName("Pista")
        self.spin = campo_num(minimo, maximo, valor, decimales, 10.0 ** -decimales, 88)
        fila.addWidget(punto)
        fila.addWidget(lb)
        fila.addWidget(lb_desc)
        fila.addStretch(1)
        fila.addWidget(self.spin)

        self.slider = QtWidgets.QSlider(Qt.Horizontal)
        self.slider.setRange(int(round(minimo * self.escala)),
                             int(round(maximo * self.escala)))
        self.slider.setValue(int(round(valor * self.escala)))
        self.slider.setCursor(Qt.PointingHandCursor)

        v.addLayout(fila)
        v.addWidget(self.slider)

        self.slider.valueChanged.connect(self._desde_slider)
        self.spin.valueChanged.connect(self._desde_spin)

    def valor(self) -> float:
        return self.slider.value() / self.escala

    def _desde_slider(self, v):
        val = v / self.escala
        self.spin.blockSignals(True)
        self.spin.setValue(val)
        self.spin.blockSignals(False)
        self.cambiado.emit(val)

    def _desde_spin(self, val):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(val * self.escala)))
        self.slider.blockSignals(False)
        self.cambiado.emit(val)


# =====================================================================
#  Limitador de repintados
# =====================================================================

class RepintadoLimitado:
    """Acota la frecuencia de repintado de un widget dibujado a mano.

    Los widgets pintados con QPainter (la vista 3D y el diagrama) cuestan
    varios milisegundos por fotograma.  Si cada cambio de estado llamara a
    update() -- y durante un arrastre del raton llegan decenas por segundo --
    el bucle de eventos se saturaria.  En su lugar se pide el repintado por
    aqui: se pinta como mucho una vez cada 'ms' y las peticiones que caen
    dentro de ese intervalo se funden en un unico repintado diferido.
    """

    def _iniciar_limite(self, ms):
        self._ms_limite = ms
        self._reloj_pintura = QElapsedTimer()
        self._reloj_pintura.start()
        self._diferido = QTimer(self)
        self._diferido.setSingleShot(True)
        self._diferido.timeout.connect(self._repintar_ya)

    def _repintar_ya(self):
        self._reloj_pintura.restart()
        self.update()

    def pedir_repintado(self):
        # Una pestana oculta no recibe eventos de pintado: pedirlos seria
        # trabajo tirado.  Al volver a mostrarse, Qt la repinta con los
        # ultimos valores que haya guardado el widget.
        if not self.isVisible():
            return
        falta = self._ms_limite - self._reloj_pintura.elapsed()
        if falta <= 0:
            self._diferido.stop()
            self._repintar_ya()
        elif not self._diferido.isActive():
            self._diferido.start(int(falta))


# =====================================================================
#  Diagrama de bloques del lazo cerrado
# =====================================================================

class DiagramaBloques(RepintadoLimitado, QtWidgets.QWidget):
    """Dibuja el lazo cerrado y muestra el valor vivo de cada senal."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(148)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Fixed)
        self.v = {"r": 0.0, "e": 0.0, "u": 0.0, "y": 0.0,
                  "kp": 0.0, "ki": 0.0, "kd": 0.0, "sat": False}
        self._iniciar_limite(MS_GRAFICAS)

    def actualizar(self, **kw):
        self.v.update(kw)
        self.pedir_repintado()

    # ------------------------------------------------------------------
    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)

        f_tit = QtGui.QFont("Segoe UI", 9, QtGui.QFont.DemiBold)
        f_sub = QtGui.QFont("Segoe UI", 7)
        f_sig = QtGui.QFont("Segoe UI", 8, QtGui.QFont.DemiBold)
        f_sum = QtGui.QFont("Segoe UI", 12, QtGui.QFont.Bold)

        W = float(self.width())
        m = 10.0
        anchos = [102.0, 130.0, 148.0, 108.0]        # ref, pid, planta, salida
        d_sum = 32.0
        fijo = sum(anchos) + d_sum
        disp = W - 2.0 * m
        hueco = (disp - fijo) / 4.0
        if hueco < 28.0:
            k = max(0.5, (disp - 4.0 * 28.0) / fijo)
            anchos = [a * k for a in anchos]
            d_sum *= k
            hueco = 28.0

        yc, h_caja, y_fb = 50.0, 50.0, 116.0
        x = m
        x_ref = x
        x = x_ref + anchos[0] + hueco
        x_sum = x
        x = x_sum + d_sum + hueco
        x_pid = x
        x = x_pid + anchos[1] + hueco
        x_pla = x
        x = x_pla + anchos[2] + hueco
        x_sal = x

        def caja(x0, ancho, titulo, sub, color_borde):
            rect = QtCore.QRectF(x0, yc - h_caja / 2.0, ancho, h_caja)
            p.setPen(QtGui.QPen(qcolor(color_borde), 1.4))
            p.setBrush(QtGui.QBrush(qcolor(C["panel_alt"])))
            p.drawRoundedRect(rect, 11.0, 11.0)
            p.setPen(qcolor(C["texto"]))
            p.setFont(f_tit)
            p.drawText(QtCore.QRectF(x0, yc - h_caja / 2.0 + 7.0, ancho, 15.0),
                       Qt.AlignCenter, titulo)
            p.setPen(qcolor(C["texto_dim"]))
            p.setFont(f_sub)
            p.drawText(QtCore.QRectF(x0 + 4.0, yc - h_caja / 2.0 + 23.0, ancho - 8.0, 22.0),
                       Qt.AlignHCenter | Qt.AlignTop | Qt.TextWordWrap, sub)

        def flecha(x1, x2, color, nombre, valor):
            p.setPen(QtGui.QPen(qcolor(color), 1.6))
            p.setBrush(Qt.NoBrush)
            p.drawLine(QtCore.QPointF(x1, yc), QtCore.QPointF(x2 - 7.0, yc))
            p.setPen(Qt.NoPen)
            p.setBrush(QtGui.QBrush(qcolor(color)))
            p.drawPolygon(QtGui.QPolygonF([
                QtCore.QPointF(x2, yc),
                QtCore.QPointF(x2 - 8.0, yc - 4.5),
                QtCore.QPointF(x2 - 8.0, yc + 4.5)]))
            p.setFont(f_sig)
            p.setPen(qcolor(color))
            p.drawText(QtCore.QRectF(x1 - 12.0, yc - 31.0, x2 - x1 + 24.0, 14.0),
                       Qt.AlignCenter, nombre)
            p.setFont(f_sub)
            p.setPen(qcolor(C["texto_dim"]))
            p.drawText(QtCore.QRectF(x1 - 12.0, yc + 9.0, x2 - x1 + 24.0, 14.0),
                       Qt.AlignCenter, valor)

        v = self.v

        # ---- bloques
        caja(x_ref, anchos[0], "Referencia", "consigna r(t)", C["ref"])
        caja(x_pid, anchos[1], "Controlador PID",
             "Kp %.3g   Ki %.3g   Kd %.3g" % (v["kp"], v["ki"], v["kd"]), C["acento"])
        caja(x_pla, anchos[2], "Planta - Motor DC",
             "electrica + mecanica", C["texto_dim"])
        caja(x_sal, anchos[3], "Salida", "velocidad w(t)", C["salida"])

        # ---- nodo suma
        rc = QtCore.QRectF(x_sum, yc - d_sum / 2.0, d_sum, d_sum)
        p.setPen(QtGui.QPen(qcolor(C["texto_dim"]), 1.4))
        p.setBrush(QtGui.QBrush(qcolor(C["panel_alt"])))
        p.drawEllipse(rc)
        p.setFont(f_sum)
        p.setPen(qcolor(C["texto"]))
        p.drawText(rc, Qt.AlignCenter, "Σ")
        p.setFont(f_sub)
        p.setPen(qcolor(C["ref"]))
        p.drawText(QtCore.QRectF(x_sum - 17.0, yc - 21.0, 16.0, 12.0), Qt.AlignCenter, "+")
        p.setPen(qcolor(C["error"]))
        p.drawText(QtCore.QRectF(x_sum + d_sum / 2.0 + 5.0, yc + d_sum / 2.0 + 1.0,
                                 14.0, 12.0), Qt.AlignCenter, "−")

        # ---- flechas del camino directo
        flecha(x_ref + anchos[0], x_sum, C["ref"], "r(t)", "%.1f rad/s" % v["r"])
        flecha(x_sum + d_sum, x_pid, C["error"], "e(t)", "%.2f rad/s" % v["e"])
        etiqueta_u = "%.2f V" % v["u"] + ("  saturado" if v["sat"] else "")
        flecha(x_pid + anchos[1], x_pla, C["control"], "u(t) = Va", etiqueta_u)
        flecha(x_pla + anchos[2], x_sal, C["salida"], "y(t)", "%.1f rad/s" % v["y"])

        # ---- camino de realimentacion
        x_br = W - m - 3.0
        x_cx = x_sum + d_sum / 2.0
        y_head = yc + d_sum / 2.0
        ruta = QtGui.QPainterPath(QtCore.QPointF(x_sal + anchos[3], yc))
        ruta.lineTo(x_br, yc)
        ruta.lineTo(x_br, y_fb)
        ruta.lineTo(x_cx, y_fb)
        ruta.lineTo(x_cx, y_head + 8.0)
        p.setPen(QtGui.QPen(qcolor(C["salida"]), 1.5))
        p.setBrush(Qt.NoBrush)
        p.drawPath(ruta)
        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QBrush(qcolor(C["salida"])))
        p.drawPolygon(QtGui.QPolygonF([
            QtCore.QPointF(x_cx, y_head),
            QtCore.QPointF(x_cx - 4.5, y_head + 8.0),
            QtCore.QPointF(x_cx + 4.5, y_head + 8.0)]))

        texto = "realimentacion  ·  y(t) medida"
        p.setFont(f_sub)
        fm = QtGui.QFontMetricsF(f_sub)
        ancho_txt = fm.horizontalAdvance(texto) + 16.0
        centro = (x_cx + x_br) / 2.0
        r_txt = QtCore.QRectF(centro - ancho_txt / 2.0, y_fb - 9.0, ancho_txt, 18.0)
        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QBrush(qcolor(C["panel"])))
        p.drawRect(r_txt)
        p.setPen(qcolor(C["texto_dim"]))
        p.drawText(r_txt, Qt.AlignCenter, texto)
        p.end()


# =====================================================================
#  Diagrama de la dinamica del motor  (circuito + balance de pares)
# =====================================================================

class DiagramaDinamica(RepintadoLimitado, QtWidgets.QWidget):
    """Dibuja el modelo fisico del motor DC con sus valores instantaneos.

    A la izquierda, la malla electrica de armadura:

        Va = Ra*ia + La*(dia/dt) + Ve        con   Ve = Ke*w

    A la derecha, el equilibrio mecanico del rotor:

        J*(dw/dt) = par_motor - par_friccion - par_carga

    Los dos dominios quedan acoplados por las constantes del motor: la
    corriente genera par (Kt*ia) y la velocidad genera fuerza
    contraelectromotriz (Ke*w).
    """

    ALTO = 236

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(self.ALTO)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Fixed)
        self.v = {"va": 0.0, "ia": 0.0, "ve": 0.0, "w": 0.0, "tm": 0.0,
                  "tf": 0.0, "tl": 0.0, "alfa": 0.0,
                  "Ra": 1.0, "La": 0.010, "J": 0.010, "B": 0.005,
                  "Kt": 0.100, "Ke": 0.100}
        self._iniciar_limite(MS_GRAFICAS)

    def parametros(self, p):
        """Fija las constantes del modelo (solo cambian al reconstruirlo)."""
        self.v.update(Ra=p.Ra, La=p.La, J=p.J, B=p.B, Kt=p.Kt, Ke=p.Ke)
        self.pedir_repintado()

    def actualizar(self, **kw):
        self.v.update(kw)
        self.pedir_repintado()

    # ------------------------------------------------------------------
    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)

        f_gru = QtGui.QFont("Segoe UI", 7, QtGui.QFont.DemiBold)
        f_sim = QtGui.QFont("Segoe UI", 8, QtGui.QFont.DemiBold)
        f_val = QtGui.QFont("Segoe UI", 7)
        f_ecu = QtGui.QFont("Consolas", 8)
        f_ine = QtGui.QFont("Segoe UI", 11, QtGui.QFont.Bold)

        v = self.v
        W = float(self.width())
        m = 14.0
        x_div = W * 0.53

        def texto(x, y, ancho, alto, txt, fuente, color, alin=Qt.AlignCenter):
            p.setFont(fuente)
            p.setPen(qcolor(color))
            p.drawText(QtCore.QRectF(x, y, ancho, alto),
                       alin | Qt.AlignVCenter, txt)

        def flecha(x1, y1, x2, y2, color, grosor=1.6):
            d = math.hypot(x2 - x1, y2 - y1)
            if d < 1e-6:
                return
            dx, dy = (x2 - x1) / d, (y2 - y1) / d
            p.setPen(QtGui.QPen(qcolor(color), grosor))
            p.setBrush(Qt.NoBrush)
            p.drawLine(QtCore.QPointF(x1, y1),
                       QtCore.QPointF(x2 - 7.0 * dx, y2 - 7.0 * dy))
            bx, by = x2 - 8.0 * dx, y2 - 8.0 * dy
            p.setPen(Qt.NoPen)
            p.setBrush(QtGui.QBrush(qcolor(color)))
            p.drawPolygon(QtGui.QPolygonF([
                QtCore.QPointF(x2, y2),
                QtCore.QPointF(bx - 4.5 * dy, by + 4.5 * dx),
                QtCore.QPointF(bx + 4.5 * dy, by - 4.5 * dx)]))

        def fuente_cc(cx, cy, r, color):
            """Simbolo de fuente de continua: circulo con su polaridad."""
            p.setPen(QtGui.QPen(qcolor(color), 1.7))
            p.setBrush(QtGui.QBrush(qcolor(C["panel"])))
            p.drawEllipse(QtCore.QRectF(cx - r, cy - r, 2.0 * r, 2.0 * r))
            texto(cx - r, cy - r + 2.0, 2.0 * r, 11.0, "+", f_val, color)
            texto(cx - r, cy + r - 13.0, 2.0 * r, 11.0, "−", f_val, color)

        # ================= malla electrica de armadura =================
        texto(m, 6.0, x_div - m, 14.0, "MALLA ELECTRICA DE ARMADURA",
              f_gru, C["texto_dim"], Qt.AlignLeft)

        ya, yb = 74.0, 160.0
        yc = 0.5 * (ya + yb)
        xa, xb = m + 40.0, x_div - 46.0
        r_f = 15.0
        largo = max(xb - xa, 60.0)
        x1r = xa + 0.14 * largo
        x2r = x1r + 0.29 * largo
        x1l = x2r + 0.14 * largo
        x2l = x1l + 0.29 * largo

        p.setPen(QtGui.QPen(qcolor(C["texto_dim"]), 1.6))
        p.setBrush(Qt.NoBrush)
        for a, b in ((QtCore.QPointF(xa, ya), QtCore.QPointF(x1r, ya)),
                     (QtCore.QPointF(x2r, ya), QtCore.QPointF(x1l, ya)),
                     (QtCore.QPointF(x2l, ya), QtCore.QPointF(xb, ya)),
                     (QtCore.QPointF(xa, yb), QtCore.QPointF(xb, yb)),
                     (QtCore.QPointF(xa, ya), QtCore.QPointF(xa, yc - r_f)),
                     (QtCore.QPointF(xa, yc + r_f), QtCore.QPointF(xa, yb)),
                     (QtCore.QPointF(xb, ya), QtCore.QPointF(xb, yc - r_f)),
                     (QtCore.QPointF(xb, yc + r_f), QtCore.QPointF(xb, yb))):
            p.drawLine(a, b)

        # ---- resistencia de armadura: zigzag
        amp, n_z = 8.0, 6
        paso = (x2r - x1r) / n_z
        pts = [QtCore.QPointF(x1r, ya)]
        for k in range(n_z):
            pts.append(QtCore.QPointF(x1r + paso * (k + 0.5),
                                      ya - amp if k % 2 == 0 else ya + amp))
        pts.append(QtCore.QPointF(x2r, ya))
        p.setPen(QtGui.QPen(qcolor(C["control"]), 1.8))
        p.drawPolyline(QtGui.QPolygonF(pts))

        # ---- inductancia de armadura: cuatro espiras
        n_e, alto_e = 4, 9.0
        d_e = (x2l - x1l) / n_e
        bobina = QtGui.QPainterPath(QtCore.QPointF(x1l, ya))
        for k in range(n_e):
            bobina.arcTo(QtCore.QRectF(x1l + k * d_e, ya - alto_e,
                                       d_e, 2.0 * alto_e), 180.0, -180.0)
        p.setPen(QtGui.QPen(qcolor(C["acento"]), 1.8))
        p.drawPath(bobina)

        # ---- fuentes: voltaje aplicado y fuerza contraelectromotriz
        fuente_cc(xa, yc, r_f, C["control"])
        fuente_cc(xb, yc, r_f, C["salida"])

        # ---- sentido de la corriente de armadura
        x_i = 0.5 * (xa + x1r)
        flecha(x_i - 13.0, ya - 18.0, x_i + 13.0, ya - 18.0, C["error"], 1.4)

        # ---- rotulos: nombre sobre el hilo, valor vivo bajo el hilo.  El de
        #      la corriente se desplaza para no caer sobre el hilo vertical
        #      que baja hasta la fuente Va.
        for x0, x1, dx, nom, val, col in (
                (xa, x1r, 20.0, "ia", "%.2f A" % v["ia"], C["error"]),
                (x1r, x2r, 0.0, "Ra", "%.3g Ω" % v["Ra"], C["control"]),
                (x1l, x2l, 0.0, "La", "%.3g H" % v["La"], C["acento"])):
            texto(x0 - 18.0, 42.0, x1 - x0 + 36.0, 13.0, nom, f_sim, col)
            texto(x0 - 18.0 + dx, 88.0, x1 - x0 + 36.0 - dx, 13.0, val, f_val,
                  C["texto_dim"])

        texto(xa + 24.0, 102.0, 128.0, 14.0, "Va = u(t)", f_sim,
              C["control"], Qt.AlignLeft)
        texto(xa + 24.0, 116.0, 128.0, 14.0, "%.2f V" % v["va"], f_val,
              C["texto_dim"], Qt.AlignLeft)
        texto(xb - 152.0, 102.0, 128.0, 14.0, "Ve = Ke·ω", f_sim,
              C["salida"], Qt.AlignRight)
        texto(xb - 152.0, 116.0, 128.0, 14.0, "%.2f V" % v["ve"], f_val,
              C["texto_dim"], Qt.AlignRight)

        texto(m, 194.0, x_div - m - 12.0, 16.0,
              "Va = Ra·ia + La·(dia/dt) + Ve", f_ecu, C["texto"])

        # ---- frontera entre los dos dominios
        p.setPen(QtGui.QPen(qcolor(C["borde"]), 1.0, Qt.DashLine))
        p.drawLine(QtCore.QPointF(x_div, 26.0), QtCore.QPointF(x_div, 212.0))

        # ================= equilibrio mecanico del rotor =================
        ancho_der = W - m - x_div - 10.0
        texto(x_div + 10.0, 6.0, ancho_der, 14.0,
              "EQUILIBRIO MECANICO DEL ROTOR", f_gru, C["texto_dim"],
              Qt.AlignLeft)

        cx, r_d = x_div + 96.0, 34.0

        # el par del motor entra por el eje desde el lado electrico
        flecha(xb + 16.0, yc, cx - r_d - 8.0, yc, C["acento"])
        texto(xb + 4.0, 96.0, cx - r_d - xb - 4.0, 13.0, "par del motor",
              f_sim, C["acento"])
        texto(xb + 4.0, 128.0, cx - r_d - xb - 4.0, 13.0,
              "Kt·ia = %.3f N·m" % v["tm"], f_val, C["texto_dim"])

        # pares resistentes: friccion viscosa arriba, carga abajo
        flecha(cx, 74.0, cx, yc - r_d - 2.0, C["error"], 1.5)
        texto(cx - 84.0, 42.0, 168.0, 13.0, "par de friccion", f_sim,
              C["error"])
        texto(cx - 84.0, 56.0, 168.0, 13.0,
              "B·ω = %.3f N·m" % v["tf"], f_val, C["texto_dim"])

        flecha(cx, 160.0, cx, yc + r_d + 2.0, C["control"], 1.5)
        texto(cx - 84.0, 164.0, 168.0, 13.0,
              "TL = %.3f N·m" % v["tl"], f_val, C["texto_dim"])
        texto(cx - 84.0, 178.0, 168.0, 13.0, "par de la carga", f_sim,
              C["control"])

        # el rotor: la inercia que integra el par neto
        p.setPen(QtGui.QPen(qcolor(C["texto_dim"]), 1.8))
        p.setBrush(QtGui.QBrush(qcolor(C["panel_alt"])))
        p.drawEllipse(QtCore.QRectF(cx - r_d, yc - r_d, 2.0 * r_d, 2.0 * r_d))
        texto(cx - r_d, yc - 16.0, 2.0 * r_d, 18.0, "J", f_ine, C["texto"])
        texto(cx - r_d, yc + 3.0, 2.0 * r_d, 12.0,
              "%.3g kg·m²" % v["J"], f_val, C["texto_dim"])

        # resultado del balance: aceleracion angular y velocidad
        flecha(cx + r_d + 8.0, yc, cx + r_d + 46.0, yc, C["salida"])
        x_res = cx + r_d + 54.0
        ancho_res = max(60.0, W - m - x_res)
        texto(x_res, 94.0, ancho_res, 13.0, "aceleracion angular", f_sim,
              C["salida"], Qt.AlignLeft)
        texto(x_res, 108.0, ancho_res, 14.0,
              "dω/dt = " + escala(v["alfa"], "rad/s²"), f_val,
              C["texto_dim"], Qt.AlignLeft)
        texto(x_res, 130.0, ancho_res, 13.0, "velocidad angular", f_sim,
              C["salida"], Qt.AlignLeft)
        texto(x_res, 144.0, ancho_res, 14.0,
              "ω = %.1f rad/s" % v["w"], f_val, C["texto_dim"],
              Qt.AlignLeft)

        texto(x_div + 10.0, 194.0, ancho_der, 16.0,
              "J·(dω/dt) = Kt·ia − B·ω − TL",
              f_ecu, C["texto"])
        p.end()


# =====================================================================
#  Vista 3D del motor  (render por software con QPainter)
# =====================================================================

class _Malla:
    """Acumula vertices y caras planas para el renderizador."""

    def __init__(self):
        self.v = []
        self.f = []
        self.col = []

    def _agregar(self, verts, caras, color):
        base = len(self.v)
        self.v.extend(verts)
        c = QtGui.QColor(color)
        rgb = (c.redF(), c.greenF(), c.blueF())
        for cara in caras:
            if len(cara) == 3:                     # los triangulos se rellenan
                cara = (cara[0], cara[1], cara[2], cara[2])
            self.f.append(tuple(base + i for i in cara))
            self.col.append(rgb)

    def cilindro(self, x0, x1, r0, r1, n, color, tapa_ini=False, tapa_fin=False):
        """Cilindro o cono truncado cuyo eje de revolucion es el eje X."""
        ang = [2.0 * math.pi * i / n for i in range(n)]
        vs = [(x0, r0 * math.cos(a), r0 * math.sin(a)) for a in ang]
        vs += [(x1, r1 * math.cos(a), r1 * math.sin(a)) for a in ang]
        caras = [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
        k = len(vs)
        if tapa_ini:
            vs.append((x0, 0.0, 0.0))
            caras += [(k, (i + 1) % n, i) for i in range(n)]
            k += 1
        if tapa_fin:
            vs.append((x1, 0.0, 0.0))
            caras += [(k, n + i, n + (i + 1) % n) for i in range(n)]
        self._agregar(vs, caras, color)

    def caja(self, cx, cy, cz, sx, sy, sz, color, giro=0.0):
        """Caja centrada en (cx, cy, cz), girada 'giro' grados alrededor del eje X."""
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
        esquinas = ((-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
                    (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz))
        c, s = math.cos(math.radians(giro)), math.sin(math.radians(giro))
        vs = []
        for dx, dy, dz in esquinas:
            y, z = cy + dy, cz + dz
            vs.append((cx + dx, y * c - z * s, y * s + z * c))
        caras = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1),
                 (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
        self._agregar(vs, caras, color)

    def arrays(self):
        if not self.v:
            return (np.zeros((0, 3)), np.zeros((0, 4), dtype=np.int32),
                    np.zeros((0, 3)))
        return (np.array(self.v, dtype=np.float64),
                np.array(self.f, dtype=np.int32),
                np.array(self.col, dtype=np.float64))


class VistaMotor3D(RepintadoLimitado, QtWidgets.QWidget):
    """Motor DC en 3D dibujado con QPainter.

    La carcasa permanece fija y el eje, el acople frontal y el ventilador
    giran con la velocidad angular de la simulacion.  El render usa
    proyeccion en perspectiva, sombreado plano por cara y el algoritmo del
    pintor: no necesita OpenGL, por lo que funciona en cualquier equipo.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(228)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)
        self.setCursor(Qt.OpenHandCursor)
        self.setToolTip("Arrastre con el raton para girar la camara")
        self.azimut = 34.0
        self.elevacion = 17.0
        self.distancia = 7.6
        luz = np.array([-0.36, -0.66, 0.66])
        self.luz = luz / np.linalg.norm(luz)
        self._ang = 0.0
        self._ajuste = None
        self._arrastre = None
        self._iniciar_limite(MS_MOTOR_3D)
        self._construir()

    # ---------------- geometria ----------------
    def _construir(self):
        fijo, rotor = _Malla(), _Malla()

        # --- soporte y patas
        fijo.caja(0.0, 0.0, -1.18, 3.1, 2.4, 0.16, "#1B212C")
        fijo.caja(-1.00, 0.0, -1.04, 0.36, 1.7, 0.14, "#242B38")
        fijo.caja(1.00, 0.0, -1.04, 0.36, 1.7, 0.14, "#242B38")

        # --- carcasa (estator) y campanas de los extremos
        fijo.cilindro(-1.12, 1.12, 1.00, 1.00, 26, "#3C4657")
        fijo.cilindro(-1.42, -1.12, 0.66, 1.00, 26, "#4E596C", tapa_ini=True)
        fijo.cilindro(1.12, 1.42, 1.00, 0.66, 26, "#4E596C", tapa_fin=True)

        # --- bornes de alimentacion
        fijo.caja(-0.62, -0.23, 1.05, 0.17, 0.17, 0.28, C["stop"])
        fijo.caja(-0.62, 0.23, 1.05, 0.17, 0.17, 0.28, "#93A0B4")

        # --- eje del rotor
        rotor.cilindro(-1.86, 1.76, 0.10, 0.10, 16, "#C6CEDC",
                       tapa_ini=True, tapa_fin=True)

        # --- acople frontal con la marca que evidencia el giro
        rotor.cilindro(1.44, 1.66, 0.30, 0.30, 20, "#79839A",
                       tapa_ini=True, tapa_fin=True)
        rotor.caja(1.55, 0.0, 0.22, 0.26, 0.09, 0.36, C["acento"])

        # --- ventilador trasero: buje y seis aspas
        rotor.cilindro(-1.70, -1.44, 0.28, 0.28, 20, "#79839A", tapa_ini=True)
        for k in range(6):
            color = C["acento"] if k == 0 else "#68738A"
            rotor.caja(-1.57, 0.0, 0.55, 0.22, 0.07, 0.62, color, giro=k * 60.0)

        vf, ff, cf = fijo.arrays()
        vr, fr, cr = rotor.arrays()
        self._v_fijo = vf
        self._v_rotor = vr
        self._caras = np.vstack([ff, fr + len(vf)])
        self._colores = np.vstack([cf, cr])

    # ---------------- proyeccion ----------------
    def _a_camara(self, pts):
        """Mundo -> camara (x a la derecha, y profundidad, z hacia arriba)."""
        az = math.radians(self.azimut)
        el = math.radians(self.elevacion)
        ca, sa = math.cos(az), math.sin(az)
        ce, se = math.cos(el), math.sin(el)
        x = pts[:, 0] * ca - pts[:, 1] * sa
        y = pts[:, 0] * sa + pts[:, 1] * ca
        z = pts[:, 2]
        return np.column_stack([x, y * ce + z * se + self.distancia,
                                -y * se + z * ce])

    def _normalizado(self, cam):
        prof = np.maximum(cam[:, 1], 0.05)
        return cam[:, 0] / prof, -cam[:, 2] / prof

    def _calcular_ajuste(self, w, h):
        cam = self._a_camara(np.vstack([self._v_fijo, self._v_rotor]))
        nx, ny = self._normalizado(cam)
        ancho = max(nx.max() - nx.min(), 1e-6)
        alto = max(ny.max() - ny.min(), 1e-6)
        esc = 0.88 * min(w / ancho, h / alto)
        return (esc,
                w / 2.0 - esc * (nx.max() + nx.min()) / 2.0,
                h / 2.0 - esc * (ny.max() + ny.min()) / 2.0)

    # ---------------- animacion ----------------
    def set_angulo(self, theta_rad):
        ang = math.degrees(theta_rad) % 360.0
        if abs(ang - self._ang) < 0.08:
            return
        self._ang = ang
        self.pedir_repintado()

    def resizeEvent(self, ev):
        self._ajuste = None
        super().resizeEvent(ev)

    # ---------------- camara con el raton ----------------
    def mousePressEvent(self, ev):
        self._arrastre = ev.pos()
        self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, ev):
        if self._arrastre is None:
            return
        d = ev.pos() - self._arrastre
        self._arrastre = ev.pos()
        self.azimut = (self.azimut + d.x() * 0.4) % 360.0
        self.elevacion = max(-80.0, min(80.0, self.elevacion + d.y() * 0.3))
        self._ajuste = None
        self.pedir_repintado()

    def mouseReleaseEvent(self, _ev):
        self._arrastre = None
        self.setCursor(Qt.OpenHandCursor)

    # ---------------- dibujo ----------------
    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, qcolor(C["panel"]))
        if w < 16 or h < 16:
            p.end()
            return

        # el rotor gira alrededor del eje X
        th = math.radians(self._ang)
        c, s = math.cos(th), math.sin(th)
        vr = self._v_rotor
        rot = np.column_stack([vr[:, 0],
                               vr[:, 1] * c - vr[:, 2] * s,
                               vr[:, 1] * s + vr[:, 2] * c])
        cam = self._a_camara(np.vstack([self._v_fijo, rot]))

        if self._ajuste is None:
            self._ajuste = self._calcular_ajuste(w, h)
        esc, cx, cy = self._ajuste
        nx, ny = self._normalizado(cam)
        px = cx + esc * nx
        py = cy + esc * ny

        caras = self._caras
        pol = cam[caras]                                    # (F, 4, 3)
        centro = pol.mean(axis=1)
        n = np.cross(pol[:, 1] - pol[:, 0], pol[:, 2] - pol[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
        n *= np.where((n * centro).sum(axis=1) > 0.0, -1.0, 1.0)[:, None]
        inten = 0.30 + 0.70 * np.clip(n @ self.luz, 0.0, 1.0)

        # Todo lo vectorizable se resuelve antes en numpy y se pasa a listas de
        # Python: el bucle solo reutiliza los objetos de Qt en vez de crear un
        # QColor, un QPen y un QPolygonF por cada una de las ~350 caras.
        rgb = (np.clip(self._colores * inten[:, None], 0.0, 1.0)
               * 255.0).astype(np.int32).tolist()
        xs = px[caras].tolist()
        ys = py[caras].tolist()
        orden = np.argsort(-centro[:, 1]).tolist()          # de lejos a cerca

        color = QtGui.QColor()
        pluma = QtGui.QPen(color, 1.0)                      # cierra las costuras
        brocha = QtGui.QBrush(Qt.SolidPattern)
        cara = QtGui.QPolygonF([QtCore.QPointF(), QtCore.QPointF(),
                                QtCore.QPointF(), QtCore.QPointF()])
        for i in orden:
            color.setRgb(*rgb[i])
            pluma.setColor(color)
            brocha.setColor(color)
            p.setPen(pluma)
            p.setBrush(brocha)
            fx, fy = xs[i], ys[i]
            for j in range(4):                              # las caras son cuadrilateros
                cara[j] = QtCore.QPointF(fx[j], fy[j])
            p.drawConvexPolygon(cara)
        p.end()


# =====================================================================
#  Enlace serie con el motor real
# =====================================================================

class EnlaceSerieMotor(QtCore.QObject):
    """Puente entre la interfaz y el firmware del Arduino.

    Habla el mismo protocolo de texto del sketch: ordenes de una linea
    ("rpm 1500", "pwm 60", "kp 3.5", "s"...) y reportes periodicos del tipo

        [PID] SP: 1500  RPM: 1493.2  PWM: 62.3 %  err: 6.8  pulsos: 245
        [MAN] RPM: 0.0  PWM: 0.0 %  pulsos: 0

    La lectura llega por la senal readyRead, nunca bloqueante: la ventana
    sigue respondiendo aunque la placa se quede muda o se desconecte en
    caliente.
    """

    linea_recibida = pyqtSignal(str)        # texto crudo, para el monitor
    telemetria = pyqtSignal(dict)           # campos ya interpretados
    conexion_cambiada = pyqtSignal(bool, str)

    _RE_RPM = re.compile(r"RPM:\s*(-?\d+(?:\.\d+)?)")
    _RE_PWM = re.compile(r"PWM:\s*(-?\d+(?:\.\d+)?)")
    _RE_PULSOS = re.compile(r"pulsos:\s*(\d+)")
    _RE_SP = re.compile(r"SP:\s*(-?\d+(?:\.\d+)?)")
    _RE_ERR = re.compile(r"err:\s*(-?\d+(?:\.\d+)?)")

    LIMITE_BUFFER = 4096        # trama sin fin de linea: se descarta

    def __init__(self, parent=None):
        super().__init__(parent)
        self.puerto = None
        self._buffer = ""

    # ---------------- puertos disponibles ----------------
    @staticmethod
    def disponible() -> bool:
        return QSerialPort is not None

    @staticmethod
    def puertos():
        if QSerialPortInfo is None:
            return []
        return list(QSerialPortInfo.availablePorts())

    @staticmethod
    def resumen(info) -> str:
        """Una linea: lo que se ve en el desplegable."""
        desc = info.description()
        return info.portName() + ("  ·  " + desc if desc else "")

    @staticmethod
    def ficha(info) -> str:
        """Ficha completa del puerto, tal y como la reporta el sistema."""
        filas = [("Puerto", info.portName()),
                 ("Ubicacion", info.systemLocation()),
                 ("Descripcion", info.description() or "(sin descripcion)"),
                 ("Fabricante", info.manufacturer() or "(desconocido)"),
                 ("Numero de serie", info.serialNumber() or "(no informado)")]
        if info.hasVendorIdentifier():
            filas.append(("ID de fabricante", "0x%04X" % info.vendorIdentifier()))
        if info.hasProductIdentifier():
            filas.append(("ID de producto", "0x%04X" % info.productIdentifier()))
        # Aqui NO se consulta isBusy(): en Windows abre el puerto para saberlo
        # y en un COM serie sobre Bluetooth esa apertura tarda unos 13 s, que
        # con varios puertos dejaba la ventana colgada solo con enumerarlos.
        # Si el puerto esta ocupado, ya lo dice el error de open() al conectar.
        return "\n".join("%-17s %s" % (n + ":", v) for n, v in filas)

    # ---------------- conexion ----------------
    def conectado(self) -> bool:
        return self.puerto is not None and self.puerto.isOpen()

    def conectar(self, nombre, baudios) -> bool:
        self.desconectar()
        if QSerialPort is None or not nombre:
            self.conexion_cambiada.emit(False, "QtSerialPort no esta disponible")
            return False

        puerto = QSerialPort(self)
        puerto.setPortName(nombre)
        puerto.setBaudRate(int(baudios))
        puerto.setDataBits(QSerialPort.Data8)
        puerto.setParity(QSerialPort.NoParity)
        puerto.setStopBits(QSerialPort.OneStop)
        puerto.setFlowControl(QSerialPort.NoFlowControl)

        if not puerto.open(QtCore.QIODevice.ReadWrite):
            motivo = puerto.errorString()
            puerto.deleteLater()
            self.conexion_cambiada.emit(False, motivo)
            return False

        # Igual que el monitor serie del IDE: el pulso de DTR reinicia la
        # placa, asi que el firmware arranca en un estado conocido (motor
        # parado, PID desactivado) y empieza imprimiendo su ayuda.
        puerto.setDataTerminalReady(True)
        puerto.readyRead.connect(self._leer)
        if hasattr(puerto, "errorOccurred"):
            puerto.errorOccurred.connect(self._fallo)

        self.puerto = puerto
        self._buffer = ""
        self.conexion_cambiada.emit(True, nombre)
        return True

    def desconectar(self, motivo="") -> None:
        if self.puerto is None:
            return
        puerto, self.puerto = self.puerto, None
        self._buffer = ""
        try:
            puerto.readyRead.disconnect()
        except (TypeError, RuntimeError):
            pass
        if puerto.isOpen():
            puerto.waitForBytesWritten(200)   # que salga la ultima orden
            puerto.close()
        puerto.deleteLater()
        self.conexion_cambiada.emit(False, motivo)

    # ---------------- envio ----------------
    def enviar(self, texto) -> bool:
        if not self.conectado():
            return False
        # El firmware procesa la linea al recibir el fin de linea, no por
        # tiempo: sin el '\n' la orden se quedaria a medias en su buffer.
        self.puerto.write((texto.strip() + "\n").encode("ascii", "ignore"))
        return True

    # ---------------- recepcion ----------------
    def _leer(self) -> None:
        if self.puerto is None:
            return
        crudo = bytes(self.puerto.readAll())
        if not crudo:
            return
        self._buffer += crudo.decode("utf-8", "replace")

        while True:
            corte = self._buffer.find("\n")
            if corte < 0:
                break
            linea = self._buffer[:corte].replace("\r", "")
            self._buffer = self._buffer[corte + 1:]
            self.linea_recibida.emit(linea)
            datos = self._interpretar(linea)
            if datos:
                self.telemetria.emit(datos)

        if len(self._buffer) > self.LIMITE_BUFFER:
            self._buffer = ""

    def _fallo(self, error) -> None:
        if error == QSerialPort.NoError:
            return
        # Desconexion en caliente (se retiro el cable USB) o el puerto lo
        # tomo otra aplicacion: se cierra el enlace en vez de dejar la
        # interfaz esperando datos que ya no van a llegar.
        if self.puerto is not None:
            self.desconectar(self.puerto.errorString())

    def _interpretar(self, linea) -> dict:
        """Extrae del texto los campos que la interfaz sabe mostrar."""
        datos = {}
        m = self._RE_RPM.search(linea)
        if m:
            datos["rpm"] = float(m.group(1))
        m = self._RE_PWM.search(linea)
        if m:
            datos["pwm"] = float(m.group(1))
        m = self._RE_PULSOS.search(linea)
        if m:
            datos["pulsos"] = int(m.group(1))
        m = self._RE_SP.search(linea)
        if m:
            datos["sp"] = float(m.group(1))
        m = self._RE_ERR.search(linea)
        if m:
            datos["err"] = float(m.group(1))

        if "[PID]" in linea:
            datos["modo"] = "PID"
        elif "[MAN]" in linea:
            datos["modo"] = "MANUAL"

        # El reporte periodico imprime la magnitud del PWM, sin signo; el
        # sentido solo aparece en las lineas de confirmacion.  "antihorario"
        # se comprueba antes porque contiene "horario".
        if "antihorario" in linea:
            datos["sentido"] = -1
        elif "horario" in linea:
            datos["sentido"] = 1
        elif "detenido" in linea:
            datos["sentido"] = 0
        return datos


# =====================================================================
#  Fabrica de graficas
# =====================================================================

def crear_grafica(titulo, formula, unidad, color):
    tarjeta = Tarjeta()
    tarjeta.cuerpo.setContentsMargins(12, 10, 12, 10)
    tarjeta.cuerpo.setSpacing(4)

    enc = QtWidgets.QHBoxLayout()
    enc.setSpacing(8)
    punto = QtWidgets.QLabel()
    punto.setFixedSize(9, 9)
    punto.setStyleSheet("background:%s; border-radius:4px;" % color)
    lb1 = QtWidgets.QLabel(titulo)
    lb1.setStyleSheet("font-weight:700; font-size:12px; color:%s;" % C["texto"])
    lb2 = QtWidgets.QLabel(formula)
    lb2.setObjectName("Pista")
    lb3 = QtWidgets.QLabel(unidad)
    lb3.setObjectName("Pista")
    enc.addWidget(punto)
    enc.addWidget(lb1)
    enc.addWidget(lb2)
    enc.addStretch(1)
    enc.addWidget(lb3)

    pw = pg.PlotWidget()
    pw.setBackground(C["panel"])
    pw.setMinimumHeight(128)
    pi = pw.getPlotItem()
    pi.showGrid(x=True, y=True, alpha=0.10)
    pi.hideButtons()
    pi.setMenuEnabled(False)
    for nombre in ("left", "bottom"):
        ax = pi.getAxis(nombre)
        ax.setPen(pg.mkPen(qcolor(C["borde"])))
        try:
            ax.setTextPen(pg.mkPen(qcolor(C["texto_dim"])))
        except Exception:
            pass
        ax.setStyle(tickFont=QtGui.QFont("Segoe UI", 8), tickLength=-4)
    pi.getAxis("left").setWidth(58)
    pi.getAxis("bottom").setHeight(22)
    vb = pi.getViewBox()
    vb.setMouseEnabled(x=False, y=False)
    # El escalado vertical se calcula a mano en SimuladorPID._ajustar_y.
    # El auto-rango de pyqtgraph combinado con setXRange en cada fotograma
    # realimenta el rango X con el Y y bloquea la aplicacion.
    vb.disableAutoRange()

    tarjeta.cuerpo.addLayout(enc)
    tarjeta.cuerpo.addWidget(pw, 1)
    return tarjeta, pw


def curva(pw, color, ancho=1.8, guion=False):
    """Curva ligera: PlotCurveItem directo, sin suavizado ni comprobaciones.

    El suavizado de pyqtgraph multiplica por dos el coste de rasterizar cada
    polilinea, y PlotDataItem anade una capa de proceso que aqui no se usa
    (no hay dispersion, ni submuestreo automatico, ni auto-rango).
    """
    pluma = pg.mkPen(qcolor(color), width=ancho,
                     style=Qt.DashLine if guion else Qt.SolidLine)
    c = pg.PlotCurveItem(pen=pluma, antialias=False, skipFiniteCheck=True)
    pw.getPlotItem().addItem(c)
    return c


def diezmar_picos(t, y, n_max):
    """Reduce una serie a unos n_max puntos sin perder maximos ni minimos.

    Tomar una muestra de cada k puede saltarse justo el pico del
    sobreimpulso.  Aqui cada tramo aporta su minimo y su maximo, en orden
    temporal, de modo que la envolvente de la respuesta se conserva.
    """
    n = len(t)
    if n <= n_max:
        return t, y
    tramos = max(1, n_max // 2)
    largo = n // tramos
    m = tramos * largo
    tt = t[:m].reshape(tramos, largo)
    yy = y[:m].reshape(tramos, largo)
    i_min = yy.argmin(axis=1)
    i_max = yy.argmax(axis=1)
    a = np.minimum(i_min, i_max)
    b = np.maximum(i_min, i_max)
    filas = np.arange(tramos)
    td = np.column_stack([tt[filas, a], tt[filas, b]]).ravel()
    yd = np.column_stack([yy[filas, a], yy[filas, b]]).ravel()
    # La cola que no llena un tramo va tal cual; si no hay cola se repite la
    # ultima muestra, que es la que se esta mirando en vivo.
    cola = slice(m, n) if m < n else slice(n - 1, n)
    return np.concatenate([td, t[cola]]), np.concatenate([yd, y[cola]])


def escalones(t, v, t_fin):
    """Cambios puntuales (t_i, v_i) -> linea escalonada que llega a t_fin.

    Con t = [a, b] y v = [p, q] devuelve los vertices (a,p) (b,p) (b,q)
    (t_fin,q): cada valor se mantiene hasta el instante del siguiente cambio.
    """
    x = np.append(np.repeat(t, 2)[1:], max(t_fin, float(t[-1])))
    return x, np.repeat(v, 2)


# =====================================================================
#  Ventana principal
# =====================================================================

class SimuladorPID(QtWidgets.QMainWindow):

    SUBTITULOS = (
        "Lazo cerrado:  referencia → error → controlador → planta → salida "
        "→ realimentacion",
        "Planta:  malla electrica de armadura + equilibrio mecanico del rotor "
        " ·  senales de la misma simulacion",
        "Motor real:  Arduino + puente H + encoder optico  ·  medida y mando por el puerto serie")

    TIPOS_REF = ("Escalon (constante)", "Onda cuadrada (+/-)", "Senoidal")
    ESCALAS_GIRO = (("Tiempo real", 1.0), ("1 : 5", 0.2),
                    ("1 : 10", 0.1), ("1 : 20", 0.05))
    SENTIDOS = {1: "horario", -1: "antihorario", 0: "detenido"}

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulador de control PID  -  Motor DC")
        self.resize(1520, 930)
        self.setMinimumSize(1160, 720)

        self.motor = None
        self.pid = ControladorPID()
        self.hist = Historial()
        self.t = 0.0
        self.Ts = 0.001
        self.ventana = 1.0
        self.subpasos = 1
        self.velocidad = 0.1
        self.escala_giro = 0.2
        self.ref_tipo, self.ref_amp, self.ref_periodo = 0, 10.47, 4.0
        self.corriendo = False
        self.pendiente_reset = True
        self.acumulador = 0.0
        self.ult = {"r": 0.0, "y": 0.0, "e": 0.0, "u": 0.0}
        self._rangos_y = {}

        self.reloj = QElapsedTimer()        # tiempo real entre fotogramas
        self.crono = QElapsedTimer()        # coste del fotograma en curso
        self.reloj_vista = QElapsedTimer()  # ritmo de refresco de las vistas
        self.reloj_vista.start()
        self._t_curvas = -MS_GRAFICAS

        # Temporizador de un solo disparo: el siguiente fotograma se programa
        # al terminar el actual, descontando lo que ha costado.  Asi, si un
        # fotograma se pasa de tiempo, el bucle se espacia solo en vez de
        # encadenar disparos y dejar sin turno a los clics del usuario.
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self._tick)

        # ---------------- enlace con el motor real ----------------
        self.serie = EnlaceSerieMotor(self)
        self.rpm_fis = 0.0          # ultima velocidad informada por la placa
        self.pwm_fis = 0.0
        self.sp_fis = 0.0
        self.pulsos_fis = 0         # pulsos del ultimo reporte (0.5 s)
        self.pulsos_total = 0
        self.sentido_fis = 0
        self.modo_fis = "--"
        self.theta_fis = 0.0        # angulo integrado para la vista 3D
        self.escala_giro_fis = 0.1
        self._puerto_activo = ""
        self._fichas = {}
        self._mudo = False
        self._estado_sim = ("Listo", C["texto_dim"])
        self._ganancias_pendientes = set()

        self.reloj_fis = QElapsedTimer()    # dt de la animacion del motor real
        self.reloj_datos = QElapsedTimer()  # silencio de la placa
        self.reloj_fis.start()
        self.reloj_datos.start()
        self.timer_fis = QTimer(self)
        self.timer_fis.timeout.connect(self._tick_fisico)
        self.timer_ganancias = QTimer(self)
        self.timer_ganancias.setSingleShot(True)
        self.timer_ganancias.timeout.connect(self._volcar_ganancias)

        # Historial de la grafica RPM vs tiempo.  El setpoint se guarda solo
        # cuando cambia (NaN = modo manual, sin consigna) y se dibuja en
        # escalones; las RPM se guardan en cada reporte.
        self.err_fis = 0.0
        self.g_t, self.g_rpm = [], []
        self.g_t_sp, self.g_sp = [], []
        self._grafica_sucia = False
        self.reloj_grafica = QElapsedTimer()     # origen de tiempos de la grafica
        self.reloj_repinte_g = QElapsedTimer()   # ritmo de redibujo
        self.reloj_grafica.start()
        self.reloj_repinte_g.start()

        self._construir_ui()
        self._conectar()
        self.reiniciar()

    # =================================================================
    #  Construccion de la interfaz
    # =================================================================
    def _construir_ui(self):
        fondo = QtWidgets.QWidget()
        fondo.setObjectName("Fondo")
        raiz = QtWidgets.QVBoxLayout(fondo)
        raiz.setContentsMargins(16, 14, 16, 14)
        raiz.setSpacing(12)
        raiz.addWidget(self._encabezado())

        cuerpo = QtWidgets.QHBoxLayout()
        cuerpo.setSpacing(12)
        self.col_izq = self._columna_izquierda()
        cuerpo.addWidget(self.col_izq, 0)
        cuerpo.addWidget(self._columna_central(), 1)
        self.col_der = self._columna_derecha()
        cuerpo.addWidget(self.col_der, 0)
        raiz.addLayout(cuerpo, 1)
        self.setCentralWidget(fondo)

    # ------------------------------------------------------------------
    def _encabezado(self):
        marco = QtWidgets.QFrame()
        marco.setObjectName("Tarjeta")
        h = QtWidgets.QHBoxLayout(marco)
        h.setContentsMargins(18, 12, 14, 12)
        h.setSpacing(14)

        col = QtWidgets.QVBoxLayout()
        col.setSpacing(1)
        t = QtWidgets.QLabel("Simulador de control PID  ·  Motor DC")
        t.setObjectName("Titulo")
        self.lb_sub = QtWidgets.QLabel(self.SUBTITULOS[0])
        self.lb_sub.setObjectName("Subtitulo")
        col.addWidget(t)
        col.addWidget(self.lb_sub)
        h.addLayout(col)
        h.addStretch(1)
        h.addWidget(self._selector_vista())
        h.addStretch(1)

        self.lb_estado = QtWidgets.QLabel("●  Listo")
        h.addWidget(self.lb_estado)

        self.btn_iniciar = QtWidgets.QPushButton("▶  Iniciar simulacion")
        self.btn_iniciar.setObjectName("BtnPrimario")
        self.btn_detener = QtWidgets.QPushButton("■  Detener")
        self.btn_detener.setObjectName("BtnPeligro")
        self.btn_reiniciar = QtWidgets.QPushButton("↺  Reiniciar")
        self.btn_reiniciar.setObjectName("BtnNeutro")
        for b in (self.btn_iniciar, self.btn_detener, self.btn_reiniciar):
            b.setCursor(Qt.PointingHandCursor)
            b.setMinimumHeight(38)
            h.addWidget(b)
        return marco

    # ------------------------------------------------------------------
    def _selector_vista(self):
        """Conmutador entre la vista de control y la de dinamica del motor."""
        marco = QtWidgets.QFrame()
        marco.setObjectName("Pestanas")
        h = QtWidgets.QHBoxLayout(marco)
        h.setContentsMargins(4, 4, 4, 4)
        h.setSpacing(4)

        self.btn_v_pid = QtWidgets.QPushButton("Control PID")
        self.btn_v_din = QtWidgets.QPushButton("Dinamica del Motor")
        self.btn_v_fis = QtWidgets.QPushButton("Control de Motor Fisico")
        self.grupo_vista = QtWidgets.QButtonGroup(self)
        self.grupo_vista.setExclusive(True)
        for i, b in enumerate((self.btn_v_pid, self.btn_v_din,
                               self.btn_v_fis)):
            b.setObjectName("BtnVista")
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.setMinimumHeight(30)
            self.grupo_vista.addButton(b, i)
            h.addWidget(b)
        self.btn_v_pid.setChecked(True)
        return marco

    # ------------------------------------------------------------------
    def _columna_izquierda(self):
        cont = QtWidgets.QWidget()
        cont.setObjectName("ContenedorScroll")
        v = QtWidgets.QVBoxLayout(cont)
        v.setContentsMargins(0, 0, 8, 0)
        v.setSpacing(12)

        # ---------------- planta ----------------
        t1 = Tarjeta("Planta · Motor DC")
        r1 = Rejilla()
        self.f_Va = r1.agregar("Voltaje de armadura V<sub>a</sub> max.", "V",
                               campo_num(0.1, 1000.0, 6.0, 2, 1.0),
                               "Saturacion del actuador. La salida del controlador u(t) "
                               "es el voltaje de armadura aplicado al motor.")
        self.f_Ra = r1.agregar("Resistencia de armadura R<sub>a</sub>", "Ω",
                               campo_num(1e-3, 1000.0, 5.0, 4, 0.1))
        self.f_La = r1.agregar("Inductancia de armadura L<sub>a</sub>", "H",
                               campo_num(1e-6, 10.0, 0.002, 6, 0.001))
        self.f_Kt = r1.agregar("Constante de par K<sub>t</sub>", "N·m/A",
                               campo_num(1e-4, 100.0, 0.080, 4, 0.01),
                               "Relaciona la corriente de armadura con el par: tau = Kt · ia")
        self.f_Ke = r1.agregar("Constante FEM K<sub>e</sub>", "V·s/rad",
                               campo_num(1e-4, 100.0, 0.080, 4, 0.01),
                               "Fuerza contraelectromotriz inducida: e = Ke · w")
        self.f_J = r1.agregar("Momento de inercia del rotor J", "kg·m²",
                              campo_num(1e-7, 100.0, 0.00013, 6, 0.0001))
        self.f_B = r1.agregar("Coeficiente de friccion viscosa B", "N·m·s",
                              campo_num(0.0, 100.0, 0.00001, 6, 0.00001))
        self.f_TL = r1.agregar("Par de carga T<sub>L</sub>", "N·m",
                               campo_num(-100.0, 100.0, 0.0144, 4, 0.001),
                               "Perturbacion de par. Se puede modificar durante la simulacion.")
        t1.agregar(r1)
        v.addWidget(t1)

        # ---------------- condiciones iniciales ----------------
        t2 = Tarjeta("Condiciones iniciales")
        r2 = Rejilla()
        self.f_ia0 = r2.agregar("Corriente de armadura i<sub>a</sub>(0)", "A",
                                campo_num(-1000.0, 1000.0, 0.0, 3, 0.1))
        self.f_w0 = r2.agregar("Velocidad angular ω(0)", "rad/s",
                               campo_num(-10000.0, 10000.0, 0.0, 2, 1.0))
        t2.agregar(r2)
        v.addWidget(t2)

        # ---------------- referencia ----------------
        t3 = Tarjeta("Referencia r(t)")
        r3 = Rejilla()
        self.cb_ref = QtWidgets.QComboBox()
        self.cb_ref.addItems(self.TIPOS_REF)
        self.cb_ref.setCurrentIndex(0)
        self.cb_ref.setFixedWidth(150)
        self.cb_ref.setCursor(Qt.PointingHandCursor)
        r3.agregar("Tipo de senal", "", self.cb_ref)
        self.f_ref = r3.agregar("Amplitud de la consigna", "rad/s",
                                campo_num(-10000.0, 10000.0, 10.47, 2, 1.0))
        self.f_per = r3.agregar("Periodo", "s", campo_num(0.05, 600.0, 4.0, 2, 0.5))
        t3.agregar(r3)
        self.lb_rpm = QtWidgets.QLabel("")
        self.lb_rpm.setObjectName("Pista")
        t3.agregar(self.lb_rpm)
        v.addWidget(t3)

        # ---------------- simulacion ----------------
        t4 = Tarjeta("Simulacion")
        r4 = Rejilla()
        self.f_Ts = r4.agregar("Periodo de muestreo T<sub>s</sub>", "ms",
                               campo_num(0.05, 100.0, 1.0, 3, 0.5),
                               "Periodo con el que el PID calcula u(t). La planta se "
                               "integra con subpasos internos de Runge-Kutta 4.")
        self.f_ventana = r4.agregar("Ventana inicial", "s",
                                    campo_num(0.2, 120.0, 1.0, 2, 0.5),
                                    "Ancho con el que arranca el eje de tiempo. Cuando la "
                                    "simulacion lo rebasa, la grafica se comprime para "
                                    "seguir mostrando todo desde t = 0.")
        self.f_vel = r4.agregar("Velocidad de simulacion", "×",
                                campo_num(0.05, 20.0, 0.1, 2, 0.05),
                                "0.1 = diez veces mas lento que el tiempo real, para "
                                "ver como se forma el transitorio.")
        t4.agregar(r4)
        pista = QtWidgets.QLabel("Estos campos definen el modelo: se bloquean mientras "
                                 "la simulacion esta en marcha.")
        pista.setObjectName("Pista")
        pista.setWordWrap(True)
        t4.agregar(pista)
        v.addWidget(t4)

        v.addStretch(1)

        sc = QtWidgets.QScrollArea()
        sc.setWidgetResizable(True)
        sc.setFrameShape(QtWidgets.QFrame.NoFrame)
        sc.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sc.setFixedWidth(336)
        sc.setWidget(cont)
        return sc

    # ------------------------------------------------------------------
    def _columna_central(self):
        """Las tres vistas comparten sitio en un QStackedWidget.

        Solo la pagina visible se redibuja (ver _pintar_curvas), asi que
        anadir la segunda vista no encarece el fotograma.
        """
        self.paginas = QtWidgets.QStackedWidget()
        self.paginas.addWidget(self._pagina_pid())        # indice 0
        self.paginas.addWidget(self._pagina_dinamica())   # indice 1
        self.paginas.addWidget(self._pagina_fisico())     # indice 2
        return self.paginas

    # ------------------------------------------------------------------
    def _pagina_pid(self):
        cont = QtWidgets.QWidget()
        cont.setObjectName("ContenedorScroll")
        v = QtWidgets.QVBoxLayout(cont)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        t_diag = Tarjeta("Diagrama de bloques del lazo cerrado")
        self.diagrama = DiagramaBloques()
        t_diag.agregar(self.diagrama)
        v.addWidget(t_diag)

        t_e, self.pw_e = crear_grafica("Error", "e(t) = r(t) − y(t)",
                                       "rad/s", C["error"])
        t_u, self.pw_u = crear_grafica("Salida del controlador",
                                       "u(t) = Kp·e + Ki∫e + Kd·de/dt",
                                       "V", C["control"])
        t_y, self.pw_y = crear_grafica("Salida del proceso",
                                       "y(t) = ω(t)   (linea gris: referencia)",
                                       "rad/s", C["salida"])

        self.pw_e.addItem(pg.InfiniteLine(pos=0.0, angle=0,
                                          pen=pg.mkPen(qcolor(C["borde"]), width=1)))
        self.ln_sup = pg.InfiniteLine(pos=24.0, angle=0, pen=pg.mkPen(
            qcolor(C["stop"], 130), width=1, style=Qt.DashLine))
        self.ln_inf = pg.InfiniteLine(pos=-24.0, angle=0, pen=pg.mkPen(
            qcolor(C["stop"], 130), width=1, style=Qt.DashLine))
        self.pw_u.addItem(self.ln_sup)
        self.pw_u.addItem(self.ln_inf)

        self.c_e = curva(self.pw_e, C["error"], 1.8)
        self.c_u = curva(self.pw_u, C["control"], 1.8)
        self.c_r = curva(self.pw_y, C["ref"], 1.5, guion=True)
        self.c_y = curva(self.pw_y, C["salida"], 2.0)

        for w in (t_e, t_u, t_y):
            v.addWidget(w, 1)
        return cont

    # ------------------------------------------------------------------
    def _pagina_dinamica(self):
        """Vista de la planta sola: circuito, pares y sus dos respuestas."""
        cont = QtWidgets.QWidget()
        cont.setObjectName("ContenedorScroll")
        v = QtWidgets.QVBoxLayout(cont)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        t_cir = Tarjeta("Modelo fisico del motor DC")
        self.dinamica = DiagramaDinamica()
        t_cir.agregar(self.dinamica)
        v.addWidget(t_cir)

        t_ec = Tarjeta("Ecuaciones del modelo")
        g = QtWidgets.QGridLayout()
        g.setHorizontalSpacing(16)
        g.setVerticalSpacing(5)
        ecuaciones = (
            ("Malla electrica",
             "di<sub>a</sub>/dt = (V<sub>a</sub> − R<sub>a</sub>·i<sub>a</sub>"
             " − K<sub>e</sub>·ω) / L<sub>a</sub>"),
            ("Fuerza contraelectromotriz", "V<sub>e</sub> = K<sub>e</sub>·ω"),
            ("Par del motor", "τ<sub>m</sub> = K<sub>t</sub>·i<sub>a</sub>"),
            ("Par de friccion viscosa", "τ<sub>f</sub> = B·ω"),
            ("Equilibrio mecanico",
             "J·(dω/dt) = τ<sub>m</sub> − τ<sub>f</sub> − T<sub>L</sub>"),
            ("Aceleracion angular",
             "α = dω/dt = (K<sub>t</sub>·i<sub>a</sub> − B·ω"
             " − T<sub>L</sub>) / J"),
        )
        for i, (nombre, ecuacion) in enumerate(ecuaciones):
            lb_n = QtWidgets.QLabel(nombre)
            lb_n.setObjectName("EtiquetaCampo")
            lb_e = QtWidgets.QLabel(ecuacion)
            lb_e.setObjectName("Formula")
            g.addWidget(lb_n, i % 3, 2 * (i // 3))
            g.addWidget(lb_e, i % 3, 2 * (i // 3) + 1)
        g.setColumnStretch(1, 1)
        g.setColumnStretch(3, 1)
        t_ec.agregar(g)
        v.addWidget(t_ec)

        t_ia, self.pw_ia = crear_grafica(
            "Corriente de armadura",
            "ia(t)   ·   La·(dia/dt) = Va − Ra·ia − Ke·ω", "A", C["error"])
        t_w, self.pw_w = crear_grafica(
            "Velocidad angular",
            "ω(t)   ·   J·(dω/dt) = Kt·ia − B·ω − TL", "rad/s", C["salida"])
        self.c_ia = curva(self.pw_ia, C["error"], 1.8)
        self.c_w = curva(self.pw_w, C["salida"], 2.0)

        for w in (t_ia, t_w):
            v.addWidget(w, 1)
        return cont

    # ------------------------------------------------------------------
    def _pagina_fisico(self):
        """Vista de control del motor real conectado por el puerto serie.

        Mientras no hay enlace solo se ve el selector de puerto; al conectar
        aparecen la vista 3D del motor, las lecturas del encoder, las
        consignas, las ganancias del PID del firmware y el monitor serie.
        """
        cont = QtWidgets.QWidget()
        cont.setObjectName("ContenedorScroll")
        v = QtWidgets.QVBoxLayout(cont)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        # El monitor se arma primero aunque se muestre el ultimo: la tarjeta
        # de conexion ya escribe en su ficha al enumerar los puertos.
        monitor = self._tarjeta_monitor()
        v.addWidget(self._tarjeta_conexion())

        self.panel_fisico = QtWidgets.QWidget()
        self.panel_fisico.setObjectName("ContenedorScroll")
        # Dos columnas de altura completa: la derecha tiene que mostrar de una
        # vez las lecturas, las consignas y los tres deslizadores del PID, asi
        # que el monitor se apila bajo la vista 3D en vez de cruzar la pagina.
        pf = QtWidgets.QHBoxLayout(self.panel_fisico)
        pf.setContentsMargins(0, 0, 0, 0)
        pf.setSpacing(12)
        izq = QtWidgets.QVBoxLayout()
        izq.setSpacing(12)
        sup = QtWidgets.QHBoxLayout()
        sup.setSpacing(12)
        sup.addWidget(self._tarjeta_motor_real(), 2)
        sup.addWidget(self._tarjeta_grafica_fisica(), 3)
        izq.addLayout(sup, 1)
        izq.addWidget(self._tarjeta_estado(), 0)
        izq.addWidget(monitor, 0)
        pf.addLayout(izq, 1)
        pf.addWidget(self._columna_fisica(), 0)
        self.panel_fisico.setVisible(False)
        v.addWidget(self.panel_fisico, 1)

        self.lb_espera = QtWidgets.QLabel(
            "Elija el puerto COM en el que esta el Arduino y pulse «Conectar».\n\n"
            "Al conectar apareceran la vista 3D del motor, la grafica de RPM "
            "contra el tiempo con el setpoint, la velocidad medida por el "
            "encoder, el PWM aplicado, los pulsos contados y los deslizadores "
            "de Kp, Ki y Kd.")
        self.lb_espera.setObjectName("Pista")
        self.lb_espera.setAlignment(Qt.AlignCenter)
        self.lb_espera.setWordWrap(True)
        v.addWidget(self.lb_espera, 1)
        return cont

    # ------------------------------------------------------------------
    def _tarjeta_conexion(self):
        """Selector de puerto y boton de conexion: lo unico visible al entrar."""
        t = Tarjeta("Conexion con el motor real")

        fila = QtWidgets.QHBoxLayout()
        fila.setSpacing(8)
        lb_p = QtWidgets.QLabel("Puerto COM")
        lb_p.setObjectName("EtiquetaCampo")
        self.cb_puerto = QtWidgets.QComboBox()
        self.cb_puerto.setMinimumWidth(300)
        self.cb_puerto.setCursor(Qt.PointingHandCursor)
        self.btn_puertos = QtWidgets.QPushButton("↻")
        self.btn_puertos.setObjectName("BtnNeutro")
        self.btn_puertos.setFixedWidth(42)
        self.btn_puertos.setMinimumHeight(36)
        self.btn_puertos.setCursor(Qt.PointingHandCursor)
        self.btn_puertos.setToolTip("Volver a buscar los puertos disponibles")
        lb_b = QtWidgets.QLabel("Baudios")
        lb_b.setObjectName("EtiquetaCampo")
        self.cb_baud = QtWidgets.QComboBox()
        self.cb_baud.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.cb_baud.setFixedWidth(96)
        self.cb_baud.setCursor(Qt.PointingHandCursor)
        self.cb_baud.setToolTip("El sketch del Arduino abre el puerto a 9600 baudios")
        self.btn_conectar = QtWidgets.QPushButton("Conectar")
        self.btn_conectar.setObjectName("BtnPrimario")
        self.btn_conectar.setMinimumHeight(36)
        self.btn_conectar.setMinimumWidth(150)
        self.btn_conectar.setCursor(Qt.PointingHandCursor)

        fila.addWidget(lb_p)
        fila.addWidget(self.cb_puerto, 1)
        fila.addWidget(self.btn_puertos)
        fila.addSpacing(12)
        fila.addWidget(lb_b)
        fila.addWidget(self.cb_baud)
        fila.addSpacing(12)
        fila.addWidget(self.btn_conectar)
        t.agregar(fila)

        self.lb_serie = QtWidgets.QLabel("")
        self.lb_serie.setObjectName("Pista")
        self.lb_serie.setWordWrap(True)
        t.agregar(self.lb_serie)

        if not EnlaceSerieMotor.disponible():
            self.cb_puerto.addItem("(PyQt5.QtSerialPort no esta instalado)", "")
            for w in (self.cb_puerto, self.btn_puertos, self.cb_baud,
                      self.btn_conectar):
                w.setEnabled(False)
            self.lb_serie.setText(
                "Falta el modulo PyQt5.QtSerialPort.  Instalelo con:  "
                ".venv\\Scripts\\python -m pip install --upgrade PyQt5")
        else:
            # La lista se rellena la primera vez que se entra en esta vista,
            # no al arrancar: asi el simulador abre igual de rapido que antes
            # aunque nunca se llegue a usar el motor real.
            self.cb_puerto.addItem("(pulse ↻ para buscar los puertos)", "")
            self.btn_conectar.setEnabled(False)
        return t

    # ------------------------------------------------------------------
    def _tarjeta_motor_real(self):
        """Vista 3D del motor, girando con las RPM que informa el encoder."""
        t = Tarjeta("Motor real en movimiento")
        self.vista_motor_fis = VistaMotor3D()
        self.vista_motor_fis.setMinimumHeight(300)
        t.agregar(self.vista_motor_fis, 1)

        fila = QtWidgets.QHBoxLayout()
        fila.setSpacing(8)
        lb = QtWidgets.QLabel("Giro visual")
        lb.setObjectName("EtiquetaCampo")
        self.cb_giro_fis = QtWidgets.QComboBox()
        self.cb_giro_fis.addItems([n for n, _ in self.ESCALAS_GIRO])
        self.cb_giro_fis.setCurrentIndex(2)
        self.cb_giro_fis.setFixedWidth(120)
        self.cb_giro_fis.setCursor(Qt.PointingHandCursor)
        self.cb_giro_fis.setToolTip(
            "Escala visual del giro. No afecta a la medida: a 3000 rpm el eje da "
            "50 vueltas por segundo y ninguna pantalla puede mostrarlo.")
        fila.addStretch(1)
        fila.addWidget(lb)
        fila.addWidget(self.cb_giro_fis)
        t.agregar(fila)
        return t

    # ------------------------------------------------------------------
    def _tarjeta_grafica_fisica(self):
        """RPM medidas contra el tiempo, con el setpoint como referencia.

        No hay ventana deslizante como en la simulacion: el eje de tiempo
        arranca en 0 y se estira con cada reporte, asi que la respuesta
        completa (subida, sobreimpulso, oscilacion y asentamiento) queda
        siempre a la vista, cada vez mas comprimida.
        """
        t, self.pw_fis = crear_grafica(
            "Respuesta del motor",
            "RPM medidas vs tiempo [s]   (linea naranja: setpoint)",
            "rpm", C["salida"])
        # El setpoint necesita connect="finite": en modo manual vale NaN y la
        # linea tiene que cortarse, no caer a cero.
        self.c_sp_fis = pg.PlotCurveItem(
            pen=pg.mkPen(qcolor(C["control"]), width=2.0), antialias=False)
        self.pw_fis.getPlotItem().addItem(self.c_sp_fis)
        self.c_rpm_fis = curva(self.pw_fis, C["salida"], 2.0)

        self.btn_reset_g = QtWidgets.QPushButton("↺  Reiniciar")
        self.btn_reset_g.setObjectName("BtnNeutro")
        self.btn_reset_g.setStyleSheet("padding: 3px 10px; border-radius: 8px;")
        self.btn_reset_g.setCursor(Qt.PointingHandCursor)
        self.btn_reset_g.setToolTip(
            "Borra la grafica y pone el tiempo en cero. Util justo antes de "
            "aplicar un nuevo escalon de setpoint.")
        t.cuerpo.itemAt(0).layout().addWidget(self.btn_reset_g)
        return t

    # ------------------------------------------------------------------
    def _tarjeta_estado(self):
        """Como esta el firmware ahora mismo: consigna, error, modo y sentido."""
        t = Tarjeta("Estado del firmware")
        fila = QtWidgets.QHBoxLayout()
        fila.setSpacing(8)
        self.mf_sp = Metrica("SETPOINT", C["control"])
        self.mf_err = Metrica("ERROR  SP − RPM", C["error"])
        self.mf_modo = Metrica("MODO")
        self.mf_sentido = Metrica("SENTIDO")
        self.mf_total = Metrica("PULSOS ACUMULADOS")
        for m in (self.mf_sp, self.mf_err, self.mf_modo, self.mf_sentido,
                  self.mf_total):
            fila.addWidget(m, 1)
        t.agregar(fila)
        return t

    # ------------------------------------------------------------------
    def _columna_fisica(self):
        """Lecturas del encoder, consignas y ganancias del PID del firmware."""
        cont = QtWidgets.QWidget()
        cont.setObjectName("ContenedorScroll")
        v = QtWidgets.QVBoxLayout(cont)
        v.setContentsMargins(0, 0, 8, 0)
        v.setSpacing(12)

        # ---------------- lecturas del encoder ----------------
        t_med = Tarjeta("Medicion del encoder")
        self.mf_rpm = MetricaGrande("VELOCIDAD ANGULAR", "RPM", C["salida"], 44)
        t_med.agregar(self.mf_rpm)

        fila = QtWidgets.QHBoxLayout()
        fila.setSpacing(8)
        self.mf_pwm = MetricaGrande("PWM APLICADO", "%", C["control"], 28)
        self.mf_pulsos = MetricaGrande("PULSOS / 0.5 s", "", C["acento"], 28)
        fila.addWidget(self.mf_pwm)
        fila.addWidget(self.mf_pulsos)
        t_med.agregar(fila)
        v.addWidget(t_med)

        # ---------------- consignas ----------------
        t_con = Tarjeta("Consigna del motor")
        gc = QtWidgets.QGridLayout()
        gc.setHorizontalSpacing(8)
        gc.setVerticalSpacing(7)
        gc.setColumnStretch(0, 1)
        filas = (
            ("Setpoint del PID", "rpm",
             campo_num(-4000.0, 4000.0, 1000.0, 0, 50.0, 86), "rpm",
             "Envia «rpm v» al Arduino: fija la consigna y activa su PID."),
            ("PWM manual", "%",
             campo_num(-100.0, 100.0, 40.0, 1, 5.0, 86), "pwm",
             "Envia «pwm v»: desactiva el PID y fija el ciclo de trabajo."),
            ("Ranuras por vuelta", "",
             campo_num(1.0, 1000.0, 20.0, 0, 1.0, 86), "r",
             "Envia «r v»: ranuras del disco encoder, con las que el firmware "
             "convierte los pulsos en RPM."),
        )
        self.campos_fis = {}
        for i, (nombre, unidad, campo, orden, tip) in enumerate(filas):
            lb = etiqueta_campo(nombre, unidad)
            lb.setToolTip(tip)
            campo.setToolTip(tip)
            btn = QtWidgets.QPushButton("Aplicar")
            btn.setObjectName("BtnNeutro")
            btn.setFixedWidth(78)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda _c=False, o=orden: self._enviar_consigna(o))
            gc.addWidget(lb, i, 0)
            gc.addWidget(campo, i, 1)
            gc.addWidget(btn, i, 2)
            self.campos_fis[orden] = campo
        t_con.agregar(gc)

        self.btn_parar_fis = QtWidgets.QPushButton("■  Detener el motor")
        self.btn_parar_fis.setObjectName("BtnPeligro")
        self.btn_parar_fis.setMinimumHeight(36)
        self.btn_parar_fis.setCursor(Qt.PointingHandCursor)
        self.btn_parar_fis.setToolTip("Envia «s»: para el motor y desactiva el PID. "
                                      "Tambien se manda al desconectar y al cerrar "
                                      "la ventana.")
        t_con.agregar(self.btn_parar_fis)

        pista = QtWidgets.QLabel("El signo decide el sentido de giro: el PID solo "
                                 "regula la magnitud.")
        pista.setObjectName("Pista")
        pista.setWordWrap(True)
        t_con.agregar(pista)
        v.addWidget(t_con)

        # ---------------- ganancias del PID ----------------
        t_pid = Tarjeta("Ganancias del PID del firmware")
        self.gf_kp = ControlGanancia("Kp", "proporcional", 1.0, 10.0, 3.00,
                                     2, C["acento"])
        self.gf_ki = ControlGanancia("Ki", "integral", 0.0, 200.0, 40.00,
                                     2, C["salida"])
        self.gf_kd = ControlGanancia("Kd", "derivativa", 0.0, 2.0, 0.000,
                                     3, C["control"])
        for w in (self.gf_kp, self.gf_ki, self.gf_kd):
            t_pid.agregar(w)

        self.btn_pid_fis = QtWidgets.QPushButton("Reenviar las tres ganancias")
        self.btn_pid_fis.setObjectName("BtnNeutro")
        self.btn_pid_fis.setMinimumHeight(32)
        self.btn_pid_fis.setCursor(Qt.PointingHandCursor)
        self.btn_pid_fis.setToolTip("Util despues de reiniciar la placa: vuelve a "
                                    "mandar kp, ki y kd tal y como estan aqui.")
        t_pid.agregar(self.btn_pid_fis)

        pista2 = QtWidgets.QLabel("Rangos que admite el firmware:  "
                                  "Kp 1 a 10  ·  Ki 0 a 200  ·  Kd 0 a 2.")
        pista2.setObjectName("Pista")
        pista2.setWordWrap(True)
        t_pid.agregar(pista2)
        v.addWidget(t_pid)
        v.addStretch(1)

        sc = QtWidgets.QScrollArea()
        sc.setWidgetResizable(True)
        sc.setFrameShape(QtWidgets.QFrame.NoFrame)
        sc.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sc.setFixedWidth(408)
        sc.setWidget(cont)
        return sc

    # ------------------------------------------------------------------
    def _tarjeta_monitor(self):
        """Ficha del puerto y texto completo que entra y sale por el serie."""
        t = Tarjeta("Monitor del puerto serie")
        h = QtWidgets.QHBoxLayout()
        h.setSpacing(12)

        self.lb_info_puerto = QtWidgets.QLabel("")
        self.lb_info_puerto.setObjectName("InfoPuerto")
        self.lb_info_puerto.setTextFormat(Qt.PlainText)
        self.lb_info_puerto.setFixedWidth(300)
        self.lb_info_puerto.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.lb_info_puerto.setTextInteractionFlags(Qt.TextSelectableByMouse)
        h.addWidget(self.lb_info_puerto)

        col = QtWidgets.QVBoxLayout()
        col.setSpacing(7)
        self.consola = QtWidgets.QPlainTextEdit()
        self.consola.setObjectName("Consola")
        self.consola.setReadOnly(True)
        self.consola.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        # Registro acotado: sin tope, media hora de reportes a 2 Hz llenaria
        # el documento de miles de bloques y cada repintado costaria mas.
        self.consola.setMaximumBlockCount(600)
        self.consola.setMinimumHeight(120)
        col.addWidget(self.consola, 1)

        fila = QtWidgets.QHBoxLayout()
        fila.setSpacing(8)
        self.ed_comando = QtWidgets.QLineEdit()
        self.ed_comando.setPlaceholderText(
            "Orden para el Arduino:  rpm 1500 · pwm 60 · kp 3.5 · s · m · ?")
        self.btn_enviar = QtWidgets.QPushButton("Enviar")
        self.btn_enviar.setObjectName("BtnNeutro")
        self.btn_enviar.setFixedWidth(84)
        self.btn_enviar.setCursor(Qt.PointingHandCursor)
        self.btn_limpiar = QtWidgets.QPushButton("Limpiar")
        self.btn_limpiar.setObjectName("BtnNeutro")
        self.btn_limpiar.setFixedWidth(84)
        self.btn_limpiar.setCursor(Qt.PointingHandCursor)
        self.chk_auto = QtWidgets.QCheckBox("Autodesplazar")
        self.chk_auto.setChecked(True)
        self.chk_auto.setCursor(Qt.PointingHandCursor)
        fila.addWidget(self.ed_comando, 1)
        fila.addWidget(self.btn_enviar)
        fila.addWidget(self.btn_limpiar)
        fila.addWidget(self.chk_auto)
        col.addLayout(fila)

        h.addLayout(col, 1)
        t.agregar(h)
        return t

    # ------------------------------------------------------------------
    def _columna_derecha(self):
        cont = QtWidgets.QWidget()
        cont.setObjectName("ContenedorScroll")
        v = QtWidgets.QVBoxLayout(cont)
        v.setContentsMargins(0, 0, 8, 0)
        v.setSpacing(12)

        # ---------------- vista del motor ----------------
        t3d = Tarjeta("Motor en movimiento")
        self.vista_motor = VistaMotor3D()
        t3d.agregar(self.vista_motor, 1)

        fila = QtWidgets.QHBoxLayout()
        fila.setSpacing(8)
        lb = QtWidgets.QLabel("Giro visual")
        lb.setObjectName("EtiquetaCampo")
        self.cb_giro = QtWidgets.QComboBox()
        self.cb_giro.addItems([n for n, _ in self.ESCALAS_GIRO])
        self.cb_giro.setCurrentIndex(1)
        self.cb_giro.setFixedWidth(120)
        self.cb_giro.setCursor(Qt.PointingHandCursor)
        self.cb_giro.setToolTip("Escala visual del giro del modelo 3D. No afecta la "
                                "simulacion: solo hace visible la rotacion a velocidades altas.")
        fila.addWidget(lb)
        fila.addStretch(1)
        fila.addWidget(self.cb_giro)
        t3d.agregar(fila)
        v.addWidget(t3d)

        # ---------------- telemetria ----------------
        t_m = Tarjeta("Senales en tiempo real")
        g = QtWidgets.QGridLayout()
        g.setHorizontalSpacing(8)
        g.setVerticalSpacing(8)
        self.m_w = Metrica("VELOCIDAD ω", C["salida"])
        self.m_rpm = Metrica("VELOCIDAD")
        self.m_e = Metrica("ERROR e(t)", C["error"])
        self.m_u = Metrica("CONTROL u(t)", C["control"])
        self.m_ia = Metrica("CORRIENTE iₐ")
        self.m_tau = Metrica("PAR τ")
        self.m_r = Metrica("REFERENCIA r(t)", C["ref"])
        self.m_t = Metrica("TIEMPO")
        for i, m in enumerate((self.m_w, self.m_rpm, self.m_e, self.m_u,
                               self.m_ia, self.m_tau, self.m_r, self.m_t)):
            g.addWidget(m, i // 2, i % 2)
        t_m.agregar(g)
        v.addWidget(t_m)

        # ---------------- balance de pares (vista de dinamica) ----------------
        self.t_pares = Tarjeta("Balance de pares del rotor")
        gp = QtWidgets.QGridLayout()
        gp.setHorizontalSpacing(8)
        gp.setVerticalSpacing(8)
        self.m_tm = Metrica("PAR DEL MOTOR", C["acento"])
        self.m_tf = Metrica("PAR DE FRICCION", C["error"])
        self.m_tl = Metrica("PAR DE CARGA", C["control"])
        self.m_alfa = Metrica("ACELERACION dω/dt", C["salida"])
        for i, met in enumerate((self.m_tm, self.m_tf, self.m_tl, self.m_alfa)):
            gp.addWidget(met, i // 2, i % 2)
        self.t_pares.agregar(gp)
        pista_p = QtWidgets.QLabel("El par neto (motor − friccion − carga) es "
                                   "el que acelera la inercia J del rotor.")
        pista_p.setObjectName("Pista")
        pista_p.setWordWrap(True)
        self.t_pares.agregar(pista_p)
        self.t_pares.setVisible(False)
        v.addWidget(self.t_pares)

        # ---------------- ganancias PID ----------------
        t_pid = self.t_pid = Tarjeta("Ganancias del PID")
        self.g_kp = ControlGanancia("Kp", "proporcional", 0.0, 10.0, 0.030, 3, C["acento"])
        self.g_ki = ControlGanancia("Ki", "integral", 0.0, 200.0, 5.60, 2, C["salida"])
        self.g_kd = ControlGanancia("Kd", "derivativa", 0.0, 2.0, 0.0, 4, C["control"])
        for w in (self.g_kp, self.g_ki, self.g_kd):
            t_pid.agregar(w)
        t_pid.agregar(separador())

        self.chk_windup = QtWidgets.QCheckBox("Anti-windup del integrador")
        self.chk_windup.setChecked(True)
        self.chk_deriv = QtWidgets.QCheckBox("Derivada sobre la medicion (evita el kick)")
        self.chk_deriv.setChecked(True)
        for c in (self.chk_windup, self.chk_deriv):
            c.setCursor(Qt.PointingHandCursor)
            t_pid.agregar(c)
        r5 = Rejilla()
        self.f_N = r5.agregar("Filtro derivativo N", "",
                              campo_num(1.0, 200.0, 20.0, 1, 1.0),
                              "Constante del filtro pasa-bajas de la accion derivativa "
                              "(Tf = Kd / N).")
        t_pid.agregar(r5)
        pista = QtWidgets.QLabel("Kp, Ki, Kd, la referencia y el par de carga se pueden "
                                 "ajustar mientras la simulacion corre.")
        pista.setObjectName("Pista")
        pista.setWordWrap(True)
        t_pid.agregar(pista)
        v.addWidget(t_pid)
        v.addStretch(1)

        sc = QtWidgets.QScrollArea()
        sc.setWidgetResizable(True)
        sc.setFrameShape(QtWidgets.QFrame.NoFrame)
        sc.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sc.setFixedWidth(364)
        sc.setWidget(cont)
        return sc

    # =================================================================
    #  Conexiones
    # =================================================================
    def _campos_estructurales(self):
        return (self.f_Ra, self.f_La, self.f_Kt, self.f_Ke, self.f_J, self.f_B,
                self.f_ia0, self.f_w0, self.f_Ts, self.f_ventana)

    def _conectar(self):
        self.btn_iniciar.clicked.connect(self.iniciar)
        self.btn_detener.clicked.connect(self.detener)
        self.btn_reiniciar.clicked.connect(self.reiniciar)

        for c in (self.g_kp, self.g_ki, self.g_kd):
            c.cambiado.connect(self._aplicar_pid)
        self.chk_windup.toggled.connect(self._aplicar_pid)
        self.chk_deriv.toggled.connect(self._aplicar_pid)
        self.f_N.valueChanged.connect(self._aplicar_pid)

        self.f_Va.valueChanged.connect(self._aplicar_actuador)
        self.f_TL.valueChanged.connect(self._aplicar_carga)
        self.cb_ref.currentIndexChanged.connect(self._aplicar_referencia)
        self.f_ref.valueChanged.connect(self._aplicar_referencia)
        self.f_per.valueChanged.connect(self._aplicar_referencia)
        self.f_vel.valueChanged.connect(self._aplicar_velocidad)
        self.cb_giro.currentIndexChanged.connect(self._aplicar_giro)

        for f in self._campos_estructurales():
            f.valueChanged.connect(self._marcar_estructura)

        self.grupo_vista.idClicked.connect(self._cambiar_vista)
        self._conectar_serie()

    def _cambiar_vista(self, indice):
        """Conmuta entre control PID, dinamica del motor y motor real.

        La vista del motor real ocupa toda la ventana: los paneles de la
        simulacion no describen la planta que hay en la mesa, asi que se
        retiran junto con los botones del bucle simulado.
        """
        es_fisico = (indice == 2)
        if es_fisico and self.corriendo:
            self.detener()      # nadie ve las curvas: no vale gastar CPU

        self.paginas.setCurrentIndex(indice)
        self.col_izq.setVisible(not es_fisico)
        self.col_der.setVisible(not es_fisico)
        for b in (self.btn_iniciar, self.btn_detener, self.btn_reiniciar):
            b.setVisible(not es_fisico)
        self.t_pid.setVisible(indice == 0)
        self.t_pares.setVisible(indice == 1)
        self.lb_sub.setText(self.SUBTITULOS[indice])

        if es_fisico:
            if EnlaceSerieMotor.disponible() and not self.serie.conectado():
                self._refrescar_puertos()   # puede haberse enchufado ahora
            self._estado_conexion()
        else:
            self._pintar_estado(*self._estado_sim)
            self._refrescar(forzar=True)

    # =================================================================
    #  Ajustes en caliente
    # =================================================================
    def _aplicar_pid(self, *_):
        self.pid.kp = self.g_kp.valor()
        self.pid.ki = self.g_ki.valor()
        self.pid.kd = self.g_kd.valor()
        self.pid.anti_windup = self.chk_windup.isChecked()
        self.pid.derivada_medicion = self.chk_deriv.isChecked()
        self.pid.n_filtro = self.f_N.value()
        self.diagrama.actualizar(kp=self.pid.kp, ki=self.pid.ki, kd=self.pid.kd)

    def _aplicar_actuador(self, *_):
        va = self.f_Va.value()
        if self.motor is not None:
            self.motor.p.Va_max = va
        self.pid.limites(-va, va)
        self.ln_sup.setValue(va)
        self.ln_inf.setValue(-va)
        self.pw_u.setYRange(-va * 1.18, va * 1.18, padding=0)

    def _aplicar_carga(self, *_):
        tl = self.f_TL.value()
        if self.motor is not None:
            self.motor.p.TL = tl
        self.dinamica.actualizar(tl=tl)

    def _aplicar_referencia(self, *_):
        self.ref_tipo = self.cb_ref.currentIndex()
        self.ref_amp = self.f_ref.value()
        self.ref_periodo = max(self.f_per.value(), 1e-3)
        self.f_per.setEnabled(self.ref_tipo != 0)
        self.lb_rpm.setText("Amplitud equivalente:  %.0f rpm"
                            % (self.ref_amp * 60.0 / (2.0 * math.pi)))

    def _aplicar_velocidad(self, *_):
        self.velocidad = self.f_vel.value()

    def _aplicar_giro(self, *_):
        self.escala_giro = self.ESCALAS_GIRO[self.cb_giro.currentIndex()][1]

    def _marcar_estructura(self, *_):
        self.pendiente_reset = True
        if not self.corriendo:
            self.btn_iniciar.setText("▶  Iniciar simulacion")
            self._estado("Listo", C["texto_dim"])

    # =================================================================
    #  Control de la simulacion
    # =================================================================
    def _construir_modelo(self):
        p = ParametrosMotor(
            Ra=self.f_Ra.value(), La=self.f_La.value(),
            Kt=self.f_Kt.value(), Ke=self.f_Ke.value(),
            J=self.f_J.value(), B=self.f_B.value(),
            TL=self.f_TL.value(), Va_max=self.f_Va.value(),
            ia0=self.f_ia0.value(), w0=self.f_w0.value()).sanear()

        self.motor = MotorDC(p)
        self.Ts = max(self.f_Ts.value(), 0.01) / 1000.0
        self.ventana = max(self.f_ventana.value(), 0.2)
        self.subpasos = self.motor.subpasos_para(self.Ts)
        self.hist = Historial()
        self.t = 0.0
        self.acumulador = 0.0
        self._rangos_y = {}
        self.pid.reiniciar()

        self._aplicar_pid()
        self._aplicar_actuador()
        self._aplicar_referencia()
        self._aplicar_velocidad()
        self._aplicar_giro()

        self.dinamica.parametros(p)

        r0 = self._referencia(0.0)
        self.ult = {"r": r0, "y": self.motor.w, "e": r0 - self.motor.w, "u": 0.0}
        self.pendiente_reset = False

    def iniciar(self):
        if self.corriendo:
            return
        if self.pendiente_reset or self.motor is None:
            self._construir_modelo()
        self.corriendo = True
        self.acumulador = 0.0
        self.reloj.start()
        self._programar_tick(0)
        self._bloquear_estructura(True)
        self.btn_iniciar.setEnabled(False)
        self.btn_detener.setEnabled(True)
        self._estado("Simulando", C["salida"])

    def detener(self):
        if not self.corriendo:
            return
        self.timer.stop()
        self.corriendo = False
        self._bloquear_estructura(False)
        self.btn_iniciar.setEnabled(True)
        self.btn_iniciar.setText("▶  Reanudar")
        self.btn_detener.setEnabled(False)
        self._estado("Detenido", C["stop"])
        self._refrescar(forzar=True)

    def reiniciar(self):
        self.timer.stop()
        self.corriendo = False
        self._construir_modelo()
        self._bloquear_estructura(False)
        self.btn_iniciar.setEnabled(True)
        self.btn_iniciar.setText("▶  Iniciar simulacion")
        self.btn_detener.setEnabled(False)
        self._estado("Listo", C["texto_dim"])
        self._refrescar(forzar=True)

    def _bloquear_estructura(self, bloquear):
        for f in self._campos_estructurales():
            f.setEnabled(not bloquear)

    def _estado(self, texto, color):
        # La etiqueta del encabezado la comparten la simulacion y el enlace
        # serie: se guarda el ultimo estado simulado para restituirlo al
        # volver de la vista del motor real.
        self._estado_sim = (texto, color)
        if self.paginas.currentIndex() != 2:
            self._pintar_estado(texto, color)

    def _pintar_estado(self, texto, color):
        self.lb_estado.setText("●  " + texto)
        self.lb_estado.setStyleSheet("color:%s; font-weight:600;" % color)

    @staticmethod
    def _estilo_boton(boton, nombre):
        """Cambia el papel de un boton (primario / peligro) en caliente."""
        boton.setObjectName(nombre)
        boton.style().unpolish(boton)
        boton.style().polish(boton)

    # =================================================================
    #  Motor real: puerto serie
    # =================================================================
    def _conectar_serie(self):
        self.serie.linea_recibida.connect(self._serie_linea)
        self.serie.telemetria.connect(self._serie_telemetria)
        self.serie.conexion_cambiada.connect(self._serie_conexion)

        self.btn_puertos.clicked.connect(self._refrescar_puertos)
        self.cb_puerto.currentIndexChanged.connect(self._mostrar_info_puerto)
        self.btn_conectar.clicked.connect(self._alternar_conexion)

        self.btn_parar_fis.clicked.connect(
            lambda: self._enviar_consigna("s"))
        self.btn_pid_fis.clicked.connect(
            lambda: self._enviar_ganancias(("kp", "ki", "kd")))
        for nombre, control in (("kp", self.gf_kp), ("ki", self.gf_ki),
                                ("kd", self.gf_kd)):
            control.cambiado.connect(
                lambda _v, n=nombre: self._marcar_ganancia(n))

        self.btn_enviar.clicked.connect(self._enviar_comando_manual)
        self.ed_comando.returnPressed.connect(self._enviar_comando_manual)
        self.btn_limpiar.clicked.connect(self.consola.clear)
        self.cb_giro_fis.currentIndexChanged.connect(self._aplicar_giro_fisico)
        self.btn_reset_g.clicked.connect(self._reiniciar_grafica)

    # ------------------------------------------------------------------
    def _refrescar_puertos(self, *_):
        """Vuelve a enumerar los puertos COM sin perder el que estaba elegido."""
        anterior = self.cb_puerto.currentData()
        self.cb_puerto.blockSignals(True)
        self.cb_puerto.clear()
        self._fichas = {}
        for info in EnlaceSerieMotor.puertos():
            self.cb_puerto.addItem(EnlaceSerieMotor.resumen(info), info.portName())
            self._fichas[info.portName()] = EnlaceSerieMotor.ficha(info)
        if self.cb_puerto.count() == 0:
            self.cb_puerto.addItem("(no se encontro ningun puerto COM)", "")
        indice = self.cb_puerto.findData(anterior)
        if indice >= 0:
            self.cb_puerto.setCurrentIndex(indice)
        self.cb_puerto.blockSignals(False)

        hay_puerto = bool(self.cb_puerto.currentData())
        self.btn_conectar.setEnabled(hay_puerto)
        self._mostrar_info_puerto()
        if not self.serie.conectado():
            self.lb_serie.setText(
                "Puerto sin abrir.  Pulse «Conectar» para ver el motor y "
                "gobernarlo desde aqui."
                if hay_puerto else
                "No se encontro ningun puerto COM.  Conecte el Arduino por USB "
                "y pulse ↻ para buscar de nuevo.")

    def _mostrar_info_puerto(self, *_):
        nombre = self.cb_puerto.currentData()
        self.lb_info_puerto.setText(self._fichas.get(
            nombre, "Ningun puerto seleccionado.\n\n"
                    "Conecte el Arduino por USB y pulse ↻ para buscarlo."))

    # ------------------------------------------------------------------
    def _alternar_conexion(self):
        if self.serie.conectado():
            # Cerrar el puerto no para el motor: el firmware conserva la
            # ultima orden y seguiria girando sin nadie al mando.
            self._enviar_serie("s")
            self.serie.desconectar()
            return
        nombre = self.cb_puerto.currentData()
        if not nombre:
            return
        self._puerto_activo = nombre
        self.serie.conectar(nombre, int(self.cb_baud.currentText()))

    def _serie_conexion(self, conectado, mensaje):
        """Muestra u oculta todo el panel del motor segun haya enlace o no."""
        self.panel_fisico.setVisible(conectado)
        self.lb_espera.setVisible(not conectado)
        self.btn_conectar.setText("Desconectar" if conectado else "Conectar")
        self._estilo_boton(self.btn_conectar,
                           "BtnPeligro" if conectado else "BtnPrimario")
        for w in (self.cb_puerto, self.cb_baud, self.btn_puertos):
            w.setEnabled(not conectado)

        if conectado:
            self.consola.clear()
            self.pulsos_total = 0
            self.pulsos_fis = 0
            self.rpm_fis = self.pwm_fis = self.sp_fis = self.err_fis = 0.0
            self.sentido_fis = 0
            self.modo_fis = "--"
            self._mudo = False
            self._pintar_fisico()
            self._reiniciar_grafica()
            self.reloj_fis.restart()
            self.reloj_datos.restart()
            self.timer_fis.start(MS_MOTOR_3D)
            self._log_consola("· puerto %s abierto a %s baudios"
                              % (mensaje, self.cb_baud.currentText()))
            # Abrir el puerto pulsa DTR y eso reinicia la placa: hay que
            # dejarla arrancar antes de mandarle nada o la orden se pierde.
            QTimer.singleShot(2500,
                              lambda: self._enviar_ganancias(("kp", "ki", "kd")))
        else:
            self.timer_fis.stop()
            self.rpm_fis = self.pwm_fis = 0.0
            self.pulsos_fis = 0
            self.sentido_fis = 0
            self._mudo = False
            self._pintar_fisico()
            if mensaje:
                self._log_consola("· enlace cerrado: " + mensaje)
        self._estado_conexion(mensaje)

    def _estado_conexion(self, mensaje=""):
        """Refleja el enlace en el encabezado y en la pista de la tarjeta."""
        if self.serie.conectado():
            if self._mudo:
                texto, color = "Conectado · sin datos", C["control"]
                pista = ("El puerto esta abierto pero la placa no envia reportes. "
                         "Compruebe que el sketch esta cargado, que los baudios "
                         "coinciden y que el reporte esta activo (orden «m»).")
            else:
                texto, color = "Motor conectado", C["salida"]
                pista = ("Enlace abierto con %s a %s baudios."
                         % (self._puerto_activo, self.cb_baud.currentText()))
        else:
            texto, color = "Sin conexion", C["texto_dim"]
            # El mismo aviso sirve para una apertura fallida y para una
            # desconexion en caliente: en los dos casos no hay enlace.
            pista = (("Sin enlace con el motor: " + mensaje + "  Si el puerto "
                       "esta ocupado, cierre el Monitor Serie del IDE de Arduino.")
                     if mensaje
                     else "Puerto sin abrir.  Pulse «Conectar» para empezar.")
        if self.paginas.currentIndex() == 2:
            self._pintar_estado(texto, color)
        self.lb_serie.setText(pista)

    # ------------------------------------------------------------------
    def _serie_linea(self, linea):
        if linea.strip():
            self._log_consola(linea)

    def _serie_telemetria(self, datos):
        if "rpm" in datos:
            self.rpm_fis = datos["rpm"]
        if "pwm" in datos:
            self.pwm_fis = datos["pwm"]
        if "pulsos" in datos:
            self.pulsos_fis = datos["pulsos"]
            self.pulsos_total += datos["pulsos"]
        if "sp" in datos:
            self.sp_fis = datos["sp"]
        if "modo" in datos:
            self.modo_fis = datos["modo"]
        if "sentido" in datos:
            self.sentido_fis = datos["sentido"]
        if "err" in datos:
            self.err_fis = datos["err"]
        self._grafica_agregar(self.reloj_grafica.elapsed() / 1000.0, datos)

        self.reloj_datos.restart()
        if self._mudo:
            self._mudo = False
            self._estado_conexion()
        self._pintar_fisico()

    def _log_consola(self, texto):
        # Con el autodesplazo apagado se restituye la posicion de la barra:
        # appendPlainText siempre lleva el cursor al final y arrastraria la
        # vista mientras se lee un tramo anterior del registro.
        barra = self.consola.verticalScrollBar()
        posicion = barra.value()
        self.consola.appendPlainText(texto)
        barra.setValue(barra.maximum() if self.chk_auto.isChecked() else posicion)

    def _pintar_fisico(self):
        self.mf_rpm.set("%.0f" % abs(self.rpm_fis))
        self.mf_pwm.set("%.1f" % abs(self.pwm_fis))
        self.mf_pulsos.set("%d" % self.pulsos_fis)
        en_pid = (self.modo_fis == "PID")
        self.mf_sp.set(("%.0f rpm" % abs(self.sp_fis)) if en_pid else "--")
        self.mf_err.set(("%+.1f rpm" % self.err_fis) if en_pid else "--")
        self.mf_modo.set(self.modo_fis)
        self.mf_sentido.set(self.SENTIDOS.get(self.sentido_fis, "--"))
        self.mf_total.set("%d" % self.pulsos_total)

    # ------------------------------------------------------------------
    def _reiniciar_grafica(self, *_):
        """Vacia el historial y pone el origen de tiempos en este instante."""
        self.g_t, self.g_rpm = [], []
        self.g_t_sp, self.g_sp = [], []
        self.reloj_grafica.restart()
        # La consigna vigente sigue valiendo: se arrastra al nuevo origen
        # para que la linea del setpoint no desaparezca hasta el proximo
        # reporte.
        if self.modo_fis == "PID":
            self.g_t_sp.append(0.0)
            self.g_sp.append(abs(self.sp_fis))
        self._rangos_y.pop("rpm_fis", None)
        self._redibujar_grafica_fisica()

    def _grafica_agregar(self, t, datos):
        """Anota un reporte del firmware en el historial de la grafica."""
        if datos.get("modo") == "MANUAL":
            sp = math.nan                      # sin consigna: la linea se corta
        elif "sp" in datos:
            sp = abs(datos["sp"])              # la confirmacion lo trae con signo
        else:
            sp = None                          # la linea no dice nada del SP
        if sp is not None:
            previo = self.g_sp[-1] if self.g_sp else None
            ambos_nan = (previo is not None and math.isnan(sp)
                         and math.isnan(previo))
            if not ambos_nan and sp != previo:
                self.g_t_sp.append(t)
                self.g_sp.append(sp)
        if "rpm" in datos:
            self.g_t.append(t)
            self.g_rpm.append(abs(datos["rpm"]))
        self._grafica_sucia = True

    def _redibujar_grafica_fisica(self):
        self._grafica_sucia = False
        self.reloj_repinte_g.restart()
        vacio = np.zeros(0)
        t_fin = max(self.g_t[-1] if self.g_t else 0.0,
                    self.g_t_sp[-1] if self.g_t_sp else 0.0)
        tope = 100.0

        if self.g_t:
            t = np.asarray(self.g_t)
            rpm = np.asarray(self.g_rpm)
            self.c_rpm_fis.setData(*diezmar_picos(t, rpm, PUNTOS_GRAFICA_FISICA))
            tope = max(tope, float(rpm.max()))
        else:
            self.c_rpm_fis.setData(vacio, vacio)

        if self.g_t_sp:
            xs, ys = escalones(np.asarray(self.g_t_sp), np.asarray(self.g_sp), t_fin)
            self.c_sp_fis.setData(xs, ys, connect="finite")
            finitos = ys[np.isfinite(ys)]
            if finitos.size:
                tope = max(tope, float(finitos.max()))
        else:
            self.c_sp_fis.setData(vacio, vacio)

        # Todo el historial, de 0 al ultimo reporte: cuanto mas dura el
        # ensayo, mas se comprime la curva en el mismo ancho.
        self._ajustar_y(self.pw_fis, "rpm_fis", 0.0, tope)
        self.pw_fis.setXRange(0.0, max(t_fin, T_MIN_GRAFICA_FISICA), padding=0)

    # ------------------------------------------------------------------
    def _enviar_serie(self, texto):
        if not self.serie.enviar(texto):
            self._log_consola("· sin conexion: no se envio «%s»" % texto)
            return False
        self._log_consola(">>  " + texto)
        return True

    def _enviar_consigna(self, orden):
        """Traduce un campo de la interfaz a la orden que espera el firmware."""
        if orden == "s":
            self.sentido_fis = 0
            self._enviar_serie("s")
            return
        valor = self.campos_fis[orden].value()
        if orden in ("rpm", "pwm"):
            # El reporte periodico da la magnitud, no el signo: el sentido se
            # toma de la consigna que se acaba de mandar.
            self.sentido_fis = 0 if valor == 0.0 else (1 if valor > 0.0 else -1)
        self._enviar_serie({"rpm": "rpm %.0f", "pwm": "pwm %.1f",
                            "r": "r %.0f"}[orden] % valor)

    def _enviar_comando_manual(self):
        texto = self.ed_comando.text().strip()
        if texto:
            self._enviar_serie(texto)
            self.ed_comando.clear()

    def _marcar_ganancia(self, cual):
        # Arrastrar un deslizador dispara decenas de cambios por segundo y a
        # 9600 baudios cada orden tarda unos 5 ms en salir: se agrupan y solo
        # se manda el ultimo valor de cada ganancia.
        self._ganancias_pendientes.add(cual)
        if not self.timer_ganancias.isActive():
            self.timer_ganancias.start(180)

    def _volcar_ganancias(self):
        self._enviar_ganancias(tuple(self._ganancias_pendientes))

    def _enviar_ganancias(self, cuales):
        self._ganancias_pendientes.clear()
        if not self.serie.conectado():
            return
        valores = {"kp": self.gf_kp.valor(), "ki": self.gf_ki.valor(),
                   "kd": self.gf_kd.valor()}
        for cual in ("kp", "ki", "kd"):
            if cual in cuales:
                self._enviar_serie("%s %.3f" % (cual, valores[cual]))

    def _aplicar_giro_fisico(self, *_):
        self.escala_giro_fis = self.ESCALAS_GIRO[
            self.cb_giro_fis.currentIndex()][1]

    def _tick_fisico(self):
        """Anima la vista 3D con la ultima velocidad informada por la placa."""
        dt = self.reloj_fis.restart() / 1000.0
        sentido = self.sentido_fis if self.sentido_fis else 1
        w = abs(self.rpm_fis) * (2.0 * math.pi / 60.0) * sentido
        self.theta_fis += w * dt * self.escala_giro_fis
        self.vista_motor_fis.set_angulo(self.theta_fis)

        # El firmware reporta cada 500 ms: mas de dos segundos de silencio
        # significan que la placa se reinicio, se colgo o dejo de reportar.
        mudo = self.reloj_datos.elapsed() > 2500
        if mudo != self._mudo:
            self._mudo = mudo
            self._estado_conexion()

        # La grafica se redibuja a lo sumo cada MS_GRAFICA_FISICA y solo si
        # esta a la vista: si el sketch reporta mas deprisa, los reportes que
        # lleguen entretanto se pintan juntos en el siguiente redibujo.
        if (self._grafica_sucia and self.paginas.currentIndex() == 2
                and self.reloj_repinte_g.elapsed() >= MS_GRAFICA_FISICA):
            self._redibujar_grafica_fisica()

    # =================================================================
    #  Nucleo de simulacion
    # =================================================================
    def _referencia(self, t):
        A = self.ref_amp
        if self.ref_tipo == 0:
            return A
        T = self.ref_periodo
        if self.ref_tipo == 1:
            return A if (t % T) < T * 0.5 else -A
        return A * math.sin(2.0 * math.pi * t / T)

    def _paso(self):
        r = self._referencia(self.t)
        y = self.motor.w
        u, e = self.pid.calcular(r, y, self.Ts)
        self.hist.agregar(t=self.t, r=r, y=y, e=e, u=u,
                          ia=self.motor.ia, tau=self.motor.par)
        self.ult = {"r": r, "y": y, "e": e, "u": u}
        self.motor.avanzar(u, self.Ts, self.subpasos)
        self.t += self.Ts

    def _programar_tick(self, ms_gastados):
        """Arma el siguiente fotograma descontando lo que costo el actual."""
        if self.corriendo:
            self.timer.start(max(1, MS_SIMULACION - int(ms_gastados)))

    def _tick(self):
        self.crono.start()

        dt_real = min(self.reloj.restart() / 1000.0, 0.05)
        self.acumulador += dt_real * self.velocidad

        # El coste de un fotograma es n * subpasos evaluaciones de RK4, y
        # subpasos crece al reducir La, J o Ts.  Por eso se acota el producto y
        # no solo n: de lo contrario una combinacion como La = 1e-5 con
        # velocidad x20 pedia millones de integraciones en un unico fotograma
        # y la ventana se quedaba bloqueada varios segundos.
        n = int(self.acumulador / self.Ts)
        n_max = max(1, SUBPASOS_FOTOGRAMA // max(self.subpasos, 1))
        if n > n_max:              # el equipo no da abasto: se sacrifica tiempo
            n = n_max              # simulado, nunca la respuesta de la interfaz
            self.acumulador = 0.0
        else:
            self.acumulador -= n * self.Ts

        for _ in range(n):
            self._paso()

        if not self.motor.estable:
            self.detener()
            QtWidgets.QMessageBox.warning(
                self, "Simulacion inestable",
                "La integracion diverge con los parametros actuales.\n\n"
                "Revise la inductancia La, la inercia J y el periodo de muestreo Ts, "
                "o reduzca las ganancias del PID.")
            return

        self._refrescar()
        self._programar_tick(self.crono.elapsed())

    # =================================================================
    #  Refresco de vistas
    # =================================================================
    def _ajustar_y(self, pw, clave, lo, hi, escala=0.0):
        """Escala el eje Y con histeresis para que no vibre en cada fotograma.

        `escala` es la magnitud natural de la senal (la amplitud de la
        consigna, la corriente de arranque...): el eje nunca muestra menos del
        20 % de ella.  Sin ese tope, al asentarse la respuesta el eje seguia
        al residuo del transitorio, que decae hasta 1e-9 y luego al ruido de
        redondeo, y una linea plana se veia como una oscilacion enorme.
        """
        if not (math.isfinite(lo) and math.isfinite(hi)):
            return
        minimo = 0.2 * abs(escala)
        if hi - lo < minimo:
            centro = 0.5 * (lo + hi)
            lo, hi = centro - 0.5 * minimo, centro + 0.5 * minimo
        if hi - lo < 1e-9:
            centro = 0.5 * (lo + hi)
            lo, hi = centro - 1.0, centro + 1.0
        margen = 0.12 * (hi - lo)
        lo, hi = lo - margen, hi + margen
        ant = self._rangos_y.get(clave)
        if ant is not None:
            a_lo, a_hi = ant
            dentro = (lo >= a_lo and hi <= a_hi)
            aprovecha = (hi - lo) > 0.55 * (a_hi - a_lo)
            if dentro and aprovecha:
                return
        self._rangos_y[clave] = (lo, hi)
        pw.setYRange(lo, hi, padding=0)

    def _refrescar(self, forzar=False):
        """Reparte el trabajo de dibujo entre los ritmos de cada vista.

        La simulacion avanza a MS_SIMULACION, pero repintar las tres graficas
        cuesta del orden de 10 ms y la vista 3D otros 9: hacerlo en cada
        fotograma consumia mas tiempo del que dura el fotograma y el bucle de
        eventos nunca llegaba a atender el raton.  Las curvas y la telemetria
        se refrescan a MS_GRAFICAS, y la vista 3D se limita a si misma.
        """
        if self.paginas.currentIndex() == 2:
            return          # la vista del motor real no dibuja curvas

        ahora = self.reloj_vista.elapsed()
        if forzar or ahora - self._t_curvas >= MS_GRAFICAS:
            self._t_curvas = ahora
            self._pintar_curvas()
            self._pintar_telemetria()

        self.vista_motor.set_angulo(self.motor.theta * self.escala_giro)

    def _pintar_curvas(self):
        # Solo se rehacen las curvas de la pagina visible: reconstruir la
        # polilinea de una grafica escondida cuesta igual que la de una a la
        # vista, asi que la segunda pestana no encarece el fotograma.
        es_pid = (self.paginas.currentIndex() == 0)
        n = len(self.hist)

        if n > 0:
            t = self.hist.vista("t")
            # Se dibuja como mucho PUNTOS_CURVA puntos por curva: por encima
            # de eso la pantalla ya no distingue nada y el coste de rasterizar
            # la polilinea crece sin aportar informacion.  Se diezma
            # conservando maximos y minimos: con un diezmado 1:k el pico del
            # sobreimpulso aparecia y desaparecia segun donde cayera la
            # muestra al irse comprimiendo la grafica.  Se entregan copias: el
            # historial reescribe su memoria cuando se compacta.
            def serie(campo):
                tt, yy = diezmar_picos(t, self.hist.vista(campo), PUNTOS_CURVA)
                return np.array(tt), np.array(yy)

            if es_pid:
                e = self.hist.vista("e")
                r = self.hist.vista("r")
                y = self.hist.vista("y")
                self.c_e.setData(*serie("e"))
                self.c_u.setData(*serie("u"))
                self.c_r.setData(*serie("r"))
                self.c_y.setData(*serie("y"))
                ref = abs(self.ref_amp) or 1.0
                self._ajustar_y(self.pw_e, "e", min(0.0, float(e.min())),
                                max(0.0, float(e.max())), ref)
                self._ajustar_y(self.pw_y, "y",
                                min(float(y.min()), float(r.min())),
                                max(float(y.max()), float(r.max())), ref)
                graficas = (self.pw_e, self.pw_u, self.pw_y)
            else:
                ia = self.hist.vista("ia")
                w = self.hist.vista("y")
                self.c_ia.setData(*serie("ia"))
                self.c_w.setData(*serie("y"))
                p = self.motor.p
                self._ajustar_y(self.pw_ia, "ia", min(0.0, float(ia.min())),
                                max(0.0, float(ia.max())), p.Va_max / p.Ra)
                self._ajustar_y(self.pw_w, "w", min(0.0, float(w.min())),
                                max(0.0, float(w.max())),
                                abs(self.ref_amp) or 1.0)
                graficas = (self.pw_ia, self.pw_w)

            # Sin ventana deslizante: el eje arranca con el ancho de la
            # ventana y, cuando la simulacion la rebasa, se estira desde 0
            # hasta el instante actual, asi que el transitorio inicial nunca
            # sale de la grafica.
            x0, x1 = 0.0, max(float(t[-1]), self.ventana)
        else:
            vacio = np.zeros(0)
            amp = max(abs(self.ref_amp), 1.0)
            if es_pid:
                for c in (self.c_e, self.c_u, self.c_r, self.c_y):
                    c.setData(vacio, vacio)
                self._ajustar_y(self.pw_e, "e", -amp, amp)
                self._ajustar_y(self.pw_y, "y", min(0.0, -amp), amp)
                graficas = (self.pw_e, self.pw_u, self.pw_y)
            else:
                for c in (self.c_ia, self.c_w):
                    c.setData(vacio, vacio)
                self._ajustar_y(self.pw_ia, "ia", -1.0, 1.0)
                self._ajustar_y(self.pw_w, "w", min(0.0, -amp), amp)
                graficas = (self.pw_ia, self.pw_w)
            x0, x1 = 0.0, self.ventana

        for pw in graficas:
            pw.setXRange(x0, x1, padding=0)

    def _pintar_telemetria(self):
        u = self.ult
        mot = self.motor
        w = mot.w
        self.m_w.set("%.1f rad/s" % w)
        self.m_rpm.set("%.0f rpm" % (w * 60.0 / (2.0 * math.pi)))
        self.m_e.set("%.2f rad/s" % u["e"])
        self.m_u.set("%.2f V" % u["u"])
        self.m_ia.set("%.2f A" % mot.ia)
        self.m_tau.set("%.3f N·m" % mot.par)
        self.m_r.set("%.1f rad/s" % u["r"])
        self.m_t.set("%.2f s" % self.t)

        # Cada vista tiene su propio diagrama y sus propias metricas; solo se
        # refresca el que esta a la vista.
        if self.paginas.currentIndex() == 0:
            self.diagrama.actualizar(r=u["r"], e=u["e"], u=u["u"], y=w,
                                     sat=self.pid.saturado)
        else:
            par, friccion = mot.par, mot.par_friccion
            alfa, carga = mot.aceleracion, mot.p.TL
            self.m_tm.set("%.3f N·m" % par)
            self.m_tf.set("%.3f N·m" % friccion)
            self.m_tl.set("%.3f N·m" % carga)
            self.m_alfa.set(escala(alfa, "rad/s²"))
            self.dinamica.actualizar(va=u["u"], ia=mot.ia, ve=mot.p.Ke * w,
                                     w=w, tm=par, tf=friccion, tl=carga,
                                     alfa=alfa)

    # ------------------------------------------------------------------
    def closeEvent(self, ev):
        self.timer.stop()
        self.timer_fis.stop()
        if self.serie.conectado():
            self.serie.enviar("s")      # no dejar el motor girando solo
        self.serie.desconectar()
        super().closeEvent(ev)


# =====================================================================
#  Punto de entrada
# =====================================================================

def main():
    QtWidgets.QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    # El suavizado de pyqtgraph duplica el coste de rasterizar cada curva.
    # La vista 3D y el diagrama activan el suyo por separado en su paintEvent.
    pg.setConfigOptions(antialias=False)

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(Template(QSS_PLANTILLA).substitute(C))

    ventana = SimuladorPID()
    ventana.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
