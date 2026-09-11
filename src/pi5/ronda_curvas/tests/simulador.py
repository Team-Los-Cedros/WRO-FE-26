# Simulador minimo de pista + robot para probar la FSM sin pista real.
#
# No pretende sustituir una corrida: pretende que las CONDICIONES
# GEOMETRICAS de la maquina de estados se puedan comprobar de forma
# repetible, que es justo lo que no se podia hacer con el diseño
# anterior (habia que ir a la pista para saber si un cambio arreglaba o
# rompia una esquina).
#
# Lo importante del montaje: el barrido que se le entrega a la
# navegacion pasa por el ProcesadorLidar DE VERDAD. Los clusters, el
# frontal_muro, el angulo_muro y los sectores los calcula el mismo
# codigo que corre en el robot; aqui solo se generan los rayos.
#
# El robot simulado usa radios de giro algo DISTINTOS de los que supone
# geometria_evasion.py: si la maniobra solo funciona cuando el modelo es
# exacto, no sirve. Pero el desajuste tiene que ser realista, no
# inventado: los 239/341 que habia aqui venian de la epoca en que se
# creia que el robot giraba mucho mas abierto a la derecha, y hacian que
# el robot SIMULADO no se pareciera al de verdad.
#
# Los radios reales, medidos con cinta el 10-09-2026 sobre el centro del
# eje trasero, son 242.5 (izq) y 243.8 (der) -- practicamente simetricos.
# Aqui se aplica un +-5% de desajuste, que es del orden de la dispersion
# real de las medidas (+-16 mm entre las dos ruedas traseras del mismo
# giro), para que el modelo siga sin ser exacto.
import math
import os
import sys

CARPETA = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if CARPETA not in sys.path:
    sys.path.insert(0, CARPETA)
COMUN = os.path.join(os.path.dirname(CARPETA), "comun")
if COMUN not in sys.path:
    sys.path.append(COMUN)

import geometria_robot as geo          # noqa: E402
import optica                          # noqa: E402
from lidar_geometria import ProcesadorLidar   # noqa: E402

# Radios REALES del simulador, deliberadamente distintos de los que
# geometria_evasion da por buenos.
RADIO_REAL_IZQ = 231.0   # 242.5 medido, -5%
RADIO_REAL_DER = 256.0   # 243.8 medido, +5%
BATALLA_REAL_IZQ = RADIO_REAL_IZQ * math.tan(math.radians(25.0))
BATALLA_REAL_DER = RADIO_REAL_DER * math.tan(math.radians(20.0))

DT = 0.118          # s por barrido, ~8.5 Hz como el C1
ALCANCE_LIDAR = 4000.0
ALCANCE_CAMARA = 1600.0

MEDIO_POSTE = geo.LADO_POSTE / 2.0


def _segmentos_rect(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


class Pista:
    """Anillo cuadrado del reglamento: 3000x3000 exterior, 1000x1000
    interior, pasillo de 1000mm. Los pilares son cuadrados de 50mm."""

    def __init__(self, pilares=()):
        self.segmentos = (_segmentos_rect(-1500, -1500, 1500, 1500)
                          + _segmentos_rect(-500, -500, 500, 500))
        self.pilares = list(pilares)   # [(x, y, "ROJO"/"VERDE")]

    def segmentos_totales(self, ocultar=()):
        segs = list(self.segmentos)
        for i, (px, py, _c) in enumerate(self.pilares):
            if i in ocultar:
                continue
            segs.extend(_segmentos_rect(px - MEDIO_POSTE, py - MEDIO_POSTE,
                                        px + MEDIO_POSTE, py + MEDIO_POSTE))
        return segs


def _corta_rayo(ox, oy, dx, dy, seg):
    (ax, ay), (bx, by) = seg
    ex, ey = bx - ax, by - ay
    den = dx * ey - dy * ex
    if abs(den) < 1e-9:
        return None
    t = ((ax - ox) * ey - (ay - oy) * ex) / den
    u = ((ax - ox) * dy - (ay - oy) * dx) / den
    if t > 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


class Robot:
    """Bicicleta Ackermann con el limitador de servo del robot real."""

    def __init__(self, x, y, theta_grados):
        self.x = x
        self.y = y
        self.theta = math.radians(theta_grados)   # CCW desde +x
        self.theta0 = self.theta

    @property
    def heading(self):
        # Lo que entrega la IMU: yaw acumulado, POSITIVO A LA IZQUIERDA
        return math.degrees(self.theta - self.theta0)

    def pos_lidar(self):
        return (self.x + geo.LIDAR_X * math.cos(self.theta),
                self.y + geo.LIDAR_X * math.sin(self.theta))

    def pos_camara(self):
        # optica.py situa la camara en el mastil, ~100mm DETRAS del
        # LiDAR. geometria_robot todavia tiene el montaje viejo (+47mm
        # por delante); aqui manda optica, que es el medido para este
        # montaje. Ver DISENO_CURVAS.md, contradiccion 4.
        lx, ly = self.pos_lidar()
        ux, uy = math.cos(self.theta), math.sin(self.theta)
        rx, ry = math.sin(self.theta), -math.cos(self.theta)
        return (lx + optica.CAM_Y * ux + optica.CAM_X * rx,
                ly + optica.CAM_Y * uy + optica.CAM_X * ry)

    def a_marco_robot(self, px, py, origen=None):
        ox, oy = origen if origen else self.pos_lidar()
        dx, dy = px - ox, py - oy
        adelante = dx * math.cos(self.theta) + dy * math.sin(self.theta)
        derecha = dx * math.sin(self.theta) - dy * math.cos(self.theta)
        return (derecha, adelante)

    def avanzar(self, velocidad_pwm, cmd_grados, dt=DT):
        v = 400.0 * velocidad_pwm / 100.0     # mm/s, curva medida
        if abs(cmd_grados) < 0.15:
            omega = 0.0
        else:
            batalla = BATALLA_REAL_IZQ if cmd_grados > 0 else BATALLA_REAL_DER
            radio = batalla / math.tan(math.radians(abs(cmd_grados)))
            omega = (v / radio) * (1.0 if cmd_grados > 0 else -1.0)
        self.theta += omega * dt
        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)

    def esquinas(self):
        # Rectangulo del chasis alrededor del eje trasero
        ux, uy = math.cos(self.theta), math.sin(self.theta)
        rx, ry = math.sin(self.theta), -math.cos(self.theta)
        a = geo.ANCHO_ROBOT / 2.0
        delante = geo.LARGO_ROBOT - geo.VOLADIZO_TRASERO
        detras = -geo.VOLADIZO_TRASERO
        pts = []
        for adel in (delante, detras):
            for lat in (a, -a):
                pts.append((self.x + adel * ux + lat * rx,
                            self.y + adel * uy + lat * ry))
        return pts


class Mundo:
    def __init__(self, pista, robot, ocultar_lidar=(), ocultar_camara=()):
        self.pista = pista
        self.robot = robot
        self.geo = ProcesadorLidar()
        self.ocultar_lidar = set(ocultar_lidar)
        self.ocultar_camara = set(ocultar_camara)
        self._cos = [math.cos(math.radians(a)) for a in range(360)]
        self._sin = [math.sin(math.radians(a)) for a in range(360)]

    def barrido(self):
        # Un rayo por grado, en la convencion del C1: 0 = frente y los
        # angulos crecen en sentido HORARIO.
        ox, oy = self.robot.pos_lidar()
        segs = self.pista.segmentos_totales(self.ocultar_lidar)
        th = self.robot.theta
        ct, st = math.cos(th), math.sin(th)
        scan = []
        for a in range(360):
            # rumbo a (horario) -> angulo mundo th - a
            ca, sa = self._cos[a], self._sin[a]
            dx = ct * ca + st * sa
            dy = st * ca - ct * sa
            mejor = ALCANCE_LIDAR
            for seg in segs:
                t = _corta_rayo(ox, oy, dx, dy, seg)
                if t is not None and t < mejor:
                    mejor = t
            if mejor < ALCANCE_LIDAR:
                scan.append((float(a), mejor))
        return scan

    def medicion(self):
        return self.geo.procesar(self.barrido())

    def camara(self):
        # Devuelve (color, cx) como vision.get_deteccion(): el pilar de
        # mayor area aparente dentro del FOV medido.
        ox, oy = self.pos_cam = self.robot.pos_camara()
        mejor = None
        for i, (px, py, color) in enumerate(self.pista.pilares):
            if i in self.ocultar_camara:
                continue
            x_b, y_b = self.robot.a_marco_robot(px, py, origen=(ox, oy))
            if y_b <= 0:
                continue
            d = math.hypot(x_b, y_b)
            if d > ALCANCE_CAMARA:
                continue
            r = math.degrees(math.atan2(x_b, y_b))
            if abs(r) > optica.HFOV_EFECTIVO / 2.0:
                continue
            area = 1.0 / (d * d)
            if mejor is None or area > mejor[0]:
                mejor = (area, color, optica.cx_de_rumbo(r))
        if mejor is None:
            return (None, None)
        return (mejor[1], mejor[2])

    def distancia_min_a_pilares(self):
        # Distancia del CUERPO del robot a la superficie de cada pilar.
        # Negativa = contacto.
        peor = 1e9
        for (px, py, _c) in self.pista.pilares:
            for (cx, cy) in self.robot.esquinas():
                peor = min(peor, math.hypot(cx - px, cy - py) - MEDIO_POSTE)
            x_b, y_b = self.robot.a_marco_robot(px, py, origen=(self.robot.x,
                                                               self.robot.y))
            peor = min(peor, self._dist_rect(x_b, y_b))
        return peor

    def _dist_rect(self, x_b, y_b):
        a = geo.ANCHO_ROBOT / 2.0
        dx = max(abs(x_b) - a, 0.0)
        dy = max(max(-geo.VOLADIZO_TRASERO - y_b,
                     y_b - (geo.LARGO_ROBOT - geo.VOLADIZO_TRASERO)), 0.0)
        return math.hypot(dx, dy) - MEDIO_POSTE

    def choca_con_muro(self):
        for (cx, cy) in self.robot.esquinas():
            fuera = not (-1500 < cx < 1500 and -1500 < cy < 1500)
            dentro = (-500 < cx < 500 and -500 < cy < 500)
            if fuera or dentro:
                return True
        return False

    def lado_del_pilar(self, indice):
        # +1 si el pilar esta a la derecha del robot, -1 si a la izquierda
        px, py, _c = self.pista.pilares[indice]
        x_b, _ = self.robot.a_marco_robot(px, py)
        return 1 if x_b > 0 else -1
