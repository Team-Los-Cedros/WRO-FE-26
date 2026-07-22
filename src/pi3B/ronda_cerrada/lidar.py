# Driver del RPLIDAR C1 y geometria del barrido.
# Parsea el protocolo binario del C1, junta los puntos hasta completar una
# vuelta (wrap-around del angulo) y por cada barrido entrega una Medicion
# con las distancias por sector y los clusters que parecen postes.
#
# Convenciones: 0 grados = frente, los angulos crecen en sentido horario.
# Cartesianas: x+ = derecha, y+ = frente (en mm).
import time
import math
import threading

import serial

PUERTO_LIDAR   = '/dev/ttyUSB0'
BAUDRATE_LIDAR = 460800

START_MOTOR_CMD = b'\xa5\xf0\x02\x94\x02\xc1\x02'
START_SCAN_CMD  = b'\xa5\x20'
STOP_CMD        = b'\xa5\x25'

# ==========================================
# SECTORES DE PARED (grados)
# ==========================================
ANGULO_MIN_DER = 30
ANGULO_MAX_DER = 90
ANGULO_MIN_IZQ = 270
ANGULO_MAX_IZQ = 330
ANGULO_MIN_TRAS = 170
ANGULO_MAX_TRAS = 190

# Sector frontal por defecto (350 -> 10, cruza el 0). La navegacion lo
# ensancha durante la evasion para no perder el poste al girar.
SECTOR_FRONTAL_NORMAL = (350.0, 10.0)

# Por encima de esto la pared se da por perdida y se sostiene el ultimo
# valor valido (modo Inercial). Saltar a un valor fijo daba giros bruscos
# en las curvas cerradas.
DIST_PARED_VALIDA_MAX = 4000.0

# ==========================================
# CLUSTERING ABD (Adaptive Breakpoint Detection)
# Si el salto radial entre dos puntos seguidos supera r*FACTOR + OFFSET,
# ahi se corta el cluster. Calibrado para el C1 (~15mm de ruido).
# ==========================================
ABD_FACTOR           = 0.04
ABD_OFFSET           = 40.0    # mm
MIN_PUNTOS_CLUSTER   = 3
MAX_PUNTOS_OBSTACULO = 30      # un poste de 10cm no genera mas de 30 puntos

DIST_MAX_OBSTACULO    = 1200.0  # mm, postes mas lejos no interesan todavia
EXT_ANG_MAX_OBSTACULO = 15.0    # grados, arco maximo de un poste de 10cm
EXT_ANG_MIN_MURO      = 20.0    # un muro siempre ocupa mas que esto


class Medicion:
    # Resultado de un barrido completo
    __slots__ = ("frontal", "izquierda", "derecha", "trasera",
                 "clusters_obstaculo", "timestamp")

    def __init__(self, frontal, izquierda, derecha, trasera, clusters):
        self.frontal   = frontal
        self.izquierda = izquierda
        self.derecha   = derecha
        self.trasera   = trasera
        self.clusters_obstaculo = clusters
        self.timestamp = time.time()


def en_sector(ang, sector):
    # Soporta sectores que cruzan el 0 (ej: 350 -> 10)
    a_min, a_max = sector
    if a_min <= a_max:
        return a_min <= ang <= a_max
    return ang >= a_min or ang <= a_max


def centroide_xy_cluster(cluster):
    # Centroide cartesiano del cluster. x+ = derecha, y+ = frente (mm)
    sx, sy = 0.0, 0.0
    for ang_deg, dist_mm in cluster:
        ang_rad = math.radians(ang_deg)
        sx += dist_mm * math.sin(ang_rad)
        sy += dist_mm * math.cos(ang_rad)
    n = len(cluster)
    return sx / n, sy / n


def segmentar_clusters_abd(scan):
    if len(scan) < 2:
        return []
    clusters, actual = [], [scan[0]]
    for i in range(1, len(scan)):
        r_prev, r_curr = scan[i - 1][1], scan[i][1]
        if abs(r_curr - r_prev) <= r_prev * ABD_FACTOR + ABD_OFFSET:
            actual.append(scan[i])
        else:
            if len(actual) >= MIN_PUNTOS_CLUSTER:
                clusters.append(actual)
            actual = [scan[i]]
    if len(actual) >= MIN_PUNTOS_CLUSTER:
        clusters.append(actual)
    return clusters


def es_cluster_obstaculo(cluster):
    # Firma geometrica de un poste de ~10cm de diametro
    n = len(cluster)
    ext_ang = cluster[-1][0] - cluster[0][0]
    if ext_ang < 0:                       # cluster que cruza el 0 (355 -> 5)
        ext_ang += 360.0
    dist_min = min(p[1] for p in cluster)
    return (MIN_PUNTOS_CLUSTER <= n <= MAX_PUNTOS_OBSTACULO
            and ext_ang < EXT_ANG_MAX_OBSTACULO
            and dist_min < DIST_MAX_OBSTACULO)


class LidarC1:
    def __init__(self, puerto=PUERTO_LIDAR, baudrate=BAUDRATE_LIDAR):
        self._puerto   = puerto
        self._baudrate = baudrate
        self._ser      = None

        self._lock_sector    = threading.Lock()
        self._sector_frontal = SECTOR_FRONTAL_NORMAL

        # Ultimo valor valido de cada pared para el modo Inercial
        self._ultima_der = 2000.0
        self._ultima_izq = 2000.0

    def fijar_sector_frontal(self, a_min, a_max):
        with self._lock_sector:
            self._sector_frontal = (float(a_min), float(a_max))

    def sector_frontal_normal(self):
        self.fijar_sector_frontal(*SECTOR_FRONTAL_NORMAL)

    def _procesar_barrido(self, scan):
        with self._lock_sector:
            sector_frontal = self._sector_frontal

        d_der = d_izq = d_front = d_tras = 8000.0
        for ang, dist in scan:
            if ANGULO_MIN_DER <= ang <= ANGULO_MAX_DER:
                d_der = min(d_der, dist)
            elif ANGULO_MIN_IZQ <= ang <= ANGULO_MAX_IZQ:
                d_izq = min(d_izq, dist)
            if ANGULO_MIN_TRAS <= ang <= ANGULO_MAX_TRAS:
                d_tras = min(d_tras, dist)
            if en_sector(ang, sector_frontal):
                d_front = min(d_front, dist)

        # Modo Inercial en las paredes laterales
        if d_der < DIST_PARED_VALIDA_MAX:
            self._ultima_der = d_der
        else:
            d_der = self._ultima_der
        if d_izq < DIST_PARED_VALIDA_MAX:
            self._ultima_izq = d_izq
        else:
            d_izq = self._ultima_izq

        # Clustering solo adelante y a los lados, la trasera no hace falta
        # y ahorra CPU en la Pi 3B
        scan_relevante = [p for p in scan if not (120.0 < p[0] < 240.0)]
        clusters = [c for c in segmentar_clusters_abd(scan_relevante)
                    if es_cluster_obstaculo(c)]

        return Medicion(d_front, d_izq, d_der, d_tras, clusters)

    def hilo_lectura(self, obtener_corriendo, al_barrido):
        # al_barrido(Medicion) se llama una vez por barrido completo
        try:
            self._ser = serial.Serial(self._puerto, baudrate=self._baudrate, timeout=1)
            time.sleep(0.5)
            self._ser.write(START_MOTOR_CMD)
            time.sleep(1.5)
            self._ser.reset_input_buffer()
            self._ser.write(START_SCAN_CMD)
            time.sleep(0.5)
            if self._ser.in_waiting >= 7:          # descartar cabecera de respuesta
                self._ser.read(7)
            print("[+] Telemetria LiDAR activa.")

            angulo_previo = 0.0
            buffer_barrido = []

            while obtener_corriendo():
                b0 = self._ser.read(1)
                if not b0:
                    continue
                byte0 = b0[0]
                # En un paquete valido el bit de start y su inverso difieren
                if (byte0 & 0x01) == ((byte0 >> 1) & 0x01):
                    continue

                resto = self._ser.read(4)
                if len(resto) < 4:
                    continue
                byte1, byte2, byte3, byte4 = resto

                if (byte1 & 0x01) != 1:            # check bit del campo angulo
                    continue

                angle       = ((byte2 << 7) | (byte1 >> 1)) / 64.0
                distance_mm = ((byte4 << 8) | byte3) / 4.0

                if not (0 < distance_mm < 6000):
                    continue

                # Wrap-around del angulo = barrido completo listo
                if angle < angulo_previo and (angulo_previo - angle) > 300.0:
                    if buffer_barrido:
                        al_barrido(self._procesar_barrido(buffer_barrido))
                    buffer_barrido = []
                angulo_previo = angle
                buffer_barrido.append((angle, distance_mm))

        except Exception as e:
            if obtener_corriendo():
                print(f"[-] Falla en hilo LiDAR: {e}")

    def cerrar(self):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(STOP_CMD)
                self._ser.close()
            except Exception:
                pass
