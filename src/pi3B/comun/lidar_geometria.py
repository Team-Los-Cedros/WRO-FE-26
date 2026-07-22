# Interpretacion geometrica de un barrido crudo del LiDAR (ver lidar_driver.py
# para el protocolo/hilo): distancias por sector (con modo Inercial) y
# clustering ABD para separar postes de paredes. No sabe nada del puerto
# serial ni del protocolo binario del C1.
#
# Convenciones: 0 grados = frente, los angulos crecen en sentido horario.
# Cartesianas: x+ = derecha, y+ = frente (en mm).
import time
import math
import threading

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


class ProcesadorLidar:
    # Convierte barridos crudos (de lidar_driver.LidarDriver) en Medicion.
    # Mantiene el estado de interpretacion: sector frontal vigente (lo
    # reconfigura la FSM de evasion) y el modo Inercial de cada pared.
    def __init__(self):
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

    def procesar(self, scan):
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
