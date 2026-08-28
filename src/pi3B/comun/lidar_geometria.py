# Interpretacion geometrica de un barrido crudo del LiDAR (ver lidar_driver.py
# para el protocolo/hilo). Construye un perfil de distancia minima en los
# 360 grados completos en cada ciclo (1 bin por grado) y todo lo demas
# (sectores de pared, diagonales traseras, modo Inercial) se deriva de ese
# perfil -- no hay sectores calculados por separado con su propio loop.
# Tambien hace clustering ABD para separar postes de paredes. No sabe nada
# del puerto serial ni del protocolo binario del C1.
#
# Convenciones: 0 grados = frente, los angulos crecen en sentido horario.
# Cartesianas: x+ = derecha, y+ = frente (en mm).
import time
import math
import threading

# ==========================================
# PERFIL 360 GRADOS (1 bin por grado)
# ==========================================
NUM_BINS       = 360
GRADOS_POR_BIN = 360.0 / NUM_BINS

# ==========================================
# SECTORES DE PARED Y DIAGONALES TRASERAS (grados)
# Las diagonales traseras cubren el hueco entre "derecha"/"izquierda" y
# "trasera" -- las usa el retroceso de emergencia para saber de que lado
# hay mas espacio libre en vivo (ver navegacion.py, estado RETROCESO).
# ==========================================
ANGULO_MIN_DER = 30
ANGULO_MAX_DER = 90
ANGULO_MIN_IZQ = 270
ANGULO_MAX_IZQ = 330
ANGULO_MIN_TRAS = 170
ANGULO_MAX_TRAS = 190

ANGULO_MIN_TRASDER = 90
ANGULO_MAX_TRASDER = 170
ANGULO_MIN_TRASIZQ = 190
ANGULO_MAX_TRASIZQ = 270

# Haces perpendiculares y diagonales para calculo cinematico de guiñada
ANGULO_MIN_PERP_DER = 80
ANGULO_MAX_PERP_DER = 100
ANGULO_MIN_PERP_IZQ = 260
ANGULO_MAX_PERP_IZQ = 280

ANGULO_MIN_DIAG_DER = 40
ANGULO_MAX_DIAG_DER = 50
ANGULO_MIN_DIAG_IZQ = 310
ANGULO_MAX_DIAG_IZQ = 320

# Sector frontal por defecto (350 -> 10, cruza el 0). La navegacion lo
# ensancha durante la evasion para no perder el poste al girar.
SECTOR_FRONTAL_NORMAL = (350.0, 10.0)

# Por encima de esto la pared se da por perdida y se sostiene el ultimo
# valor valido (modo Inercial). Saltar a un valor fijo daba giros bruscos
# en las curvas cerradas.
DIST_PARED_VALIDA_MAX = 4000.0

# Valor de "no hay pared frontal medible" (ver distancia_en_rango_sin_bins).
# Deliberadamente grande: significa via libre, no pared lejana.
SIN_PARED_FRONTAL = 8000.0

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

# Ancho fisico maximo (longitud de arco, mm) para dar un cluster por
# "objeto estrecho" y no por pared. Un poste del reglamento mide 100mm;
# 260mm deja margen de sobra para el ruido del C1 y para el ensanche por
# el propio haz, sin llegar a admitir un tramo de muro (que a 500mm ya
# pasa de 500mm de arco en cuanto ocupa 60 grados). Ver es_objeto_estrecho.
ANCHO_MAX_OBJETO_MM = 260.0


class Medicion:
    # Resultado de un barrido completo
    __slots__ = ("frontal", "frontal_muro", "izquierda", "derecha", "trasera",
                 "trasera_derecha", "trasera_izquierda",
                 "clusters_obstaculo", "perfil", "timestamp",
                 "d_perp_izq", "d_perp_der", "d_diag_izq", "d_diag_der", "angulo_muro")

    def __init__(self, frontal, izquierda, derecha, trasera,
                 trasera_derecha, trasera_izquierda, clusters, perfil,
                 d_perp_izq=2000.0, d_perp_der=2000.0, d_diag_izq=2000.0, d_diag_der=2000.0,
                 angulo_muro=0.0, frontal_muro=None):
        self.frontal   = frontal
        # Distancia a la PARED de enfrente, ignorando los postes que
        # tapan el sector (ver distancia_en_rango_sin_bins). Sin este
        # dato la navegacion cae a `frontal`, que es lo que hacia antes.
        self.frontal_muro = frontal if frontal_muro is None else frontal_muro
        self.izquierda = izquierda
        self.derecha   = derecha
        self.trasera   = trasera
        self.trasera_derecha   = trasera_derecha
        self.trasera_izquierda = trasera_izquierda
        self.clusters_obstaculo = clusters
        self.perfil = perfil    # 360 floats, perfil[i] = distancia min en el grado i
        self.timestamp = time.time()
        self.d_perp_izq = d_perp_izq
        self.d_perp_der = d_perp_der
        self.d_diag_izq = d_diag_izq
        self.d_diag_der = d_diag_der
        self.angulo_muro = angulo_muro


def construir_perfil_360(scan):
    # Distancia minima por cada grado del circulo completo
    perfil = [8000.0] * NUM_BINS
    for ang, dist in scan:
        i = int(ang / GRADOS_POR_BIN) % NUM_BINS
        if dist < perfil[i]:
            perfil[i] = dist
    return perfil


def _indices_de_rango(ang_min, ang_max):
    # Bins que cubre un rango angular, soportando el cruce por el 0
    # (ej 350 -> 10, como el sector frontal por defecto).
    i_min = int(ang_min / GRADOS_POR_BIN) % NUM_BINS
    i_max = int(ang_max / GRADOS_POR_BIN) % NUM_BINS
    if i_min <= i_max:
        return range(i_min, i_max + 1)
    return list(range(i_min, NUM_BINS)) + list(range(0, i_max + 1))


def distancia_en_rango(perfil, ang_min, ang_max):
    # Minima distancia entre ang_min y ang_max.
    i_min = int(ang_min / GRADOS_POR_BIN) % NUM_BINS
    i_max = int(ang_max / GRADOS_POR_BIN) % NUM_BINS
    if i_min <= i_max:
        return min(perfil[i_min:i_max + 1])
    return min(min(perfil[i_min:]), min(perfil[:i_max + 1]))


def bins_de_clusters(clusters):
    # Bins del perfil ocupados por los clusters dados
    ocupados = set()
    for cluster in clusters:
        for ang_deg, _ in cluster:
            ocupados.add(int(ang_deg / GRADOS_POR_BIN) % NUM_BINS)
    return ocupados


def es_objeto_estrecho(cluster):
    # Filtro por ancho FISICO (longitud de arco), no angular.
    #
    # es_cluster_obstaculo() acota el arco en grados (EXT_ANG_MAX_OBSTACULO
    # = 15), y eso solo vale de lejos: un poste de 100mm subtiende 15
    # grados a 380mm, pero 25.6 grados a 220mm. O sea que el poste deja
    # de reconocerse justo cuando esta encima -- que es cuando tapa el
    # sector frontal. Por eso este filtro es aparte y mide milimetros:
    # ancho = arco_en_radianes * distancia, que da ~100mm para un poste
    # a cualquier distancia y varios cientos para un tramo de muro.
    #
    # No se toca es_cluster_obstaculo(): la evasion depende de ese
    # criterio y hoy funciona (8 evasiones correctas en la corrida del
    # README 8.5). Este filtro solo decide que bins ignora el control de
    # pared, no que persigue la FSM.
    ext_ang = cluster[-1][0] - cluster[0][0]
    if ext_ang < 0:                       # cluster que cruza el 0
        ext_ang += 360.0
    d_min = min(p[1] for p in cluster)
    ancho_mm = math.radians(ext_ang) * d_min
    return ancho_mm <= ANCHO_MAX_OBJETO_MM and d_min < DIST_MAX_OBSTACULO


def distancia_en_rango_sin_bins(perfil, ang_min, ang_max, excluidos):
    # Como distancia_en_rango pero ignorando los bins indicados. Se usa
    # para separar "pared de frente" de "poste de frente": el perfil
    # guarda el minimo por bin, asi que un poste tapa la pared que tiene
    # detras y ese bin no dice nada de donde esta la pared.
    #
    # Si TODOS los bins del sector estan tapados por postes no hay pared
    # medible, que no es lo mismo que tenerla encima: se devuelve el
    # valor de "sin pared a la vista" para que el control no reaccione a
    # un poste como si fuera un muro (los postes los maneja la FSM de
    # evasion, ver navegacion.py).
    vals = [perfil[i] for i in _indices_de_rango(ang_min, ang_max)
            if i not in excluidos]
    return min(vals) if vals else SIN_PARED_FRONTAL


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

        perfil = construir_perfil_360(scan)

        d_front     = distancia_en_rango(perfil, *sector_frontal)
        d_der       = distancia_en_rango(perfil, ANGULO_MIN_DER, ANGULO_MAX_DER)
        d_izq       = distancia_en_rango(perfil, ANGULO_MIN_IZQ, ANGULO_MAX_IZQ)
        d_tras      = distancia_en_rango(perfil, ANGULO_MIN_TRAS, ANGULO_MAX_TRAS)
        d_tras_der  = distancia_en_rango(perfil, ANGULO_MIN_TRASDER, ANGULO_MAX_TRASDER)
        d_tras_izq  = distancia_en_rango(perfil, ANGULO_MIN_TRASIZQ, ANGULO_MAX_TRASIZQ)

        d_perp_der  = distancia_en_rango(perfil, ANGULO_MIN_PERP_DER, ANGULO_MAX_PERP_DER)
        d_perp_izq  = distancia_en_rango(perfil, ANGULO_MIN_PERP_IZQ, ANGULO_MAX_PERP_IZQ)
        d_diag_der  = distancia_en_rango(perfil, ANGULO_MIN_DIAG_DER, ANGULO_MAX_DIAG_DER)
        d_diag_izq  = distancia_en_rango(perfil, ANGULO_MIN_DIAG_IZQ, ANGULO_MAX_DIAG_IZQ)

        # Calculo de angulo respecto a la pared (orientacion de guiñada en grados)
        ang_muro_der = 0.0
        if d_perp_der < DIST_PARED_VALIDA_MAX and d_diag_der < DIST_PARED_VALIDA_MAX:
            dx = d_diag_der * 0.7071 - d_perp_der
            dy = d_diag_der * 0.7071
            if dy > 1.0:
                ang_muro_der = math.degrees(math.atan2(dx, dy))

        ang_muro_izq = 0.0
        if d_perp_izq < DIST_PARED_VALIDA_MAX and d_diag_izq < DIST_PARED_VALIDA_MAX:
            dx = d_perp_izq - d_diag_izq * 0.7071
            dy = d_diag_izq * 0.7071
            if dy > 1.0:
                ang_muro_izq = math.degrees(math.atan2(dx, dy))

        if d_perp_der < DIST_PARED_VALIDA_MAX and d_perp_izq < DIST_PARED_VALIDA_MAX:
            angulo_muro = (ang_muro_der + ang_muro_izq) / 2.0
        elif d_perp_der < DIST_PARED_VALIDA_MAX:
            angulo_muro = ang_muro_der
        elif d_perp_izq < DIST_PARED_VALIDA_MAX:
            angulo_muro = ang_muro_izq
        else:
            angulo_muro = 0.0

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
        todos = segmentar_clusters_abd(scan_relevante)
        clusters = [c for c in todos if es_cluster_obstaculo(c)]
        estrechos = [c for c in todos if es_objeto_estrecho(c)]

        # Pared frontal con los postes descontados. Medido en pista
        # (README 8.5): un poste de 10cm es mas estrecho que el sector
        # frontal de 20 grados, asi que al avanzar entra y sale del
        # sector y el minimo salta entre el poste (~200mm) y el pasillo
        # de detras (~3000mm) en ciclos seguidos -- 24 saltos de factor
        # >=3x en una corrida, el 83% con un poste confirmado delante.
        # `frontal` sigue incluyendolos porque la emergencia anti-choque
        # SI tiene que ver los postes; el control de pared, no.
        d_front_muro = distancia_en_rango_sin_bins(
            perfil, sector_frontal[0], sector_frontal[1], bins_de_clusters(estrechos))

        return Medicion(d_front, d_izq, d_der, d_tras,
                         d_tras_der, d_tras_izq, clusters, perfil,
                         d_perp_izq=d_perp_izq, d_perp_der=d_perp_der,
                         d_diag_izq=d_diag_izq, d_diag_der=d_diag_der,
                         angulo_muro=angulo_muro, frontal_muro=d_front_muro)
