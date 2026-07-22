"""
LIDAR — Parseo de paquetes RPLIDAR C1, seguimiento de pared y
clustering ABD (Adaptive Breakpoint Detection) para separar postes de
obstáculo de las paredes del circuito.

Este módulo no importa `tracker` ni `Close2_round` -- expone su estado
(distancias de pared, buffer de barrido) como atributos de módulo, y el
script principal le inyecta callbacks para enterarse de cuándo hay un
barrido completo listo, en vez de que este módulo llame de vuelta a la
lógica de la máquina de estados (evita import circular).
"""
import time
import math
import threading

import serial

PUERTO_LIDAR   = '/dev/ttyUSB0'
BAUDRATE_LIDAR = 460800

START_MOTOR_CMD = b'\xa5\xf0\x02\x94\x02\xc1\x02'
START_SCAN_CMD  = b'\xa5\x20'
STOP_CMD        = b'\xa5\x25'

# Sectores LiDAR de pared (grados)
ANGULO_MIN_DER = 30
ANGULO_MAX_DER = 90
ANGULO_MIN_IZQ = 270
ANGULO_MAX_IZQ = 330

# Sector frontal dinámico -- la FSM de evasión en Close2_round.py lo
# ensancha durante DETECTADO/ESQUIVANDO para no perder el cluster del
# obstáculo al girar, y lo restablece a 350-10 en CARRERA.
ANGULO_MIN_FRONTAL = 350
ANGULO_MAX_FRONTAL = 10

# ==========================================
# CONSTANTES DEL CLUSTERING ABD
# Adaptive Breakpoint Detection — O(n), sin librerías extra.
# Calibrado para RPLIDAR C1 (~15mm ruido, paso angular ~0.45 deg).
# ==========================================
ABD_FACTOR = 0.04          # 4% de r_i como umbral relativo
ABD_OFFSET = 40.0          # mm — umbral mínimo absoluto (ruido sensor)
MIN_PUNTOS_CLUSTER = 3     # Clusters con menos puntos son ruido
MAX_PUNTOS_OBSTACULO = 30  # Un poste de 10cm no produce más de 30 puntos en 2D

# Geometría de la pista WRO FE (mm)
DIST_MAX_OBSTACULO     = 1200.0   # Solo obstáculos a menos de 1.2m
EXT_ANG_MAX_OBSTACULO  = 15.0     # Arco máximo que ocupa un poste (~10cm Ø a 200mm)
EXT_ANG_MIN_MURO       = 20.0     # Un muro siempre ocupa más de 20 deg

ser_lidar     = None
angulo_previo = 0.0

dist_derecha_min   = 8000.0
dist_izquierda_min = 8000.0
dist_frontal_min   = 8000.0
dist_trasera_min   = 8000.0

# Ultimo valor válido (<4000mm) de cada pared, para el modo "Inercial"
# (ver README seccion 5.3-A): si un lado no tiene lectura valida en un
# barrido, se sostiene el ultimo valor conocido en vez de saltar a un
# valor fijo arbitrario.
ultimo_dist_derecha_valida   = 2000.0
ultimo_dist_izquierda_valida = 2000.0

# Scan buffer: acumula el barrido completo antes de procesar (crítico para ABD)
lock_scan = threading.Lock()
scan_buffer_acumulando = []   # (angulo_deg, distancia_mm) — ciclo en curso
scan_buffer_listo      = []   # Último barrido completo disponible


def segmentar_clusters_abd(scan):
    """
    Recibe lista de (angulo_deg, distancia_mm) ordenada por ángulo creciente.
    Devuelve lista de clusters (cada cluster = lista de tuplas del mismo tipo).

    Principio: si la diferencia de distancia radial entre dos puntos consecutivos
    supera el umbral ABD_FACTOR*r + ABD_OFFSET, se produce una ruptura de cluster.
    Esto detecta saltos en profundidad que indican objetos distintos.
    """
    if len(scan) < 2:
        return []

    clusters = []
    cluster_actual = [scan[0]]

    for i in range(1, len(scan)):
        r_prev = scan[i - 1][1]
        r_curr = scan[i][1]

        umbral = r_prev * ABD_FACTOR + ABD_OFFSET
        diff   = abs(r_curr - r_prev)

        if diff <= umbral:
            cluster_actual.append(scan[i])
        else:
            if len(cluster_actual) >= MIN_PUNTOS_CLUSTER:
                clusters.append(cluster_actual)
            cluster_actual = [scan[i]]

    if len(cluster_actual) >= MIN_PUNTOS_CLUSTER:
        clusters.append(cluster_actual)

    return clusters


def clasificar_cluster(cluster):
    """
    Clasifica un cluster como 'OBSTACULO', 'MURO' o 'RUIDO'.

    OBSTACULO (poste ~10cm diámetro):
        - n_puntos in [3, 30]
        - extension angular < 15 deg
        - dist_min < 1200 mm

    MURO (pared >= 0.5m de longitud):
        - extension angular >= 20 deg  O  n_puntos > 30

    RUIDO: todo lo demás.

    Devuelve (tipo_str, dist_min_mm, ext_ang_deg, n_puntos).
    """
    n       = len(cluster)
    ang_min = cluster[0][0]
    ang_max = cluster[-1][0]
    ext_ang = ang_max - ang_min

    # Corrección para clusters que cruzan 0 deg (ej: 355 deg -> 5 deg)
    if ext_ang < 0:
        ext_ang += 360.0

    dist_min = min(p[1] for p in cluster)

    if (MIN_PUNTOS_CLUSTER <= n <= MAX_PUNTOS_OBSTACULO
            and ext_ang < EXT_ANG_MAX_OBSTACULO
            and dist_min < DIST_MAX_OBSTACULO):
        return "OBSTACULO", dist_min, ext_ang, n

    if ext_ang >= EXT_ANG_MIN_MURO or n > MAX_PUNTOS_OBSTACULO:
        return "MURO", dist_min, ext_ang, n

    return "RUIDO", dist_min, ext_ang, n


def centroide_xy_cluster(cluster):
    """
    Convierte el cluster de coordenadas polares a cartesianas y calcula centroide.
    Convención: x+ = derecha del robot, y+ = frente del robot.
    (0 deg = frente, angulos crecen en sentido horario)
    """
    sx, sy = 0.0, 0.0
    for ang_deg, dist_mm in cluster:
        ang_rad = math.radians(ang_deg)
        sx += dist_mm * math.sin(ang_rad)   # componente lateral
        sy += dist_mm * math.cos(ang_rad)   # componente frontal
    n = len(cluster)
    return sx / n, sy / n


def procesar_scan(scan):
    """
    Procesa un barrido completo del RPLIDAR C1:
    1. Actualiza dist_*_min (seguimiento de pared, con modo "Inercial").
    2. Clustering ABD en zona relevante (excluye trasera 120-240 deg).
    3. Clasifica clusters y devuelve solo los marcados como OBSTACULO.

    La actualización del tracker (IMU + asociación de clusters) queda
    fuera de este módulo -- la orquesta Close2_round.py llamando a
    tracker.py con la lista devuelta aquí.
    """
    global dist_derecha_min, dist_izquierda_min, dist_frontal_min, dist_trasera_min
    global ultimo_dist_derecha_valida, ultimo_dist_izquierda_valida

    if not scan:
        return []

    # --- 1. Distancias de pared ---
    d_der   = 8000.0
    d_izq   = 8000.0
    d_front = 8000.0
    d_tras  = 8000.0

    for ang, dist in scan:
        if ANGULO_MIN_DER <= ang <= ANGULO_MAX_DER:
            if dist < d_der:   d_der = dist
        elif ANGULO_MIN_IZQ <= ang <= ANGULO_MAX_IZQ:
            if dist < d_izq:   d_izq = dist
        if 170.0 <= ang <= 190.0:
            if dist < d_tras:  d_tras = dist
        if ang >= ANGULO_MIN_FRONTAL or ang <= ANGULO_MAX_FRONTAL:
            if dist < d_front: d_front = dist

    # Modo "Inercial": si un lado no tuvo lectura valida en este barrido,
    # se sostiene el ultimo valor valido conocido en vez de saltar a un
    # valor fijo -- evita un giro brusco cuando una pared se pierde
    # momentaneamente (tipico en curvas cerradas). Ver README seccion 5.3-A.
    if d_der < 4000:
        dist_derecha_min = d_der
        ultimo_dist_derecha_valida = d_der
    else:
        dist_derecha_min = ultimo_dist_derecha_valida

    if d_izq < 4000:
        dist_izquierda_min = d_izq
        ultimo_dist_izquierda_valida = d_izq
    else:
        dist_izquierda_min = ultimo_dist_izquierda_valida

    dist_trasera_min   = d_tras
    dist_frontal_min   = d_front

    # --- 2. Clustering ABD en zona relevante (excluye trasera pura) ---
    # Para reducir carga CPU en Pi 3B, solo procesamos la mitad delantera + laterales.
    scan_relevante = [p for p in scan if not (120.0 < p[0] < 240.0)]
    clusters_raw   = segmentar_clusters_abd(scan_relevante)

    # --- 3. Clasificar clusters ---
    clusters_obstaculos = []
    for clust in clusters_raw:
        tipo, dist_min_c, ext_ang, n_pts = clasificar_cluster(clust)
        if tipo == "OBSTACULO":
            clusters_obstaculos.append(clust)

    return clusters_obstaculos


def hilo_lidar(obtener_corriendo, obtener_fase, al_listo, al_completar_scan):
    """
    obtener_corriendo: () -> bool, True mientras el sistema deba seguir activo.
    obtener_fase: () -> str, fase_actual de la carrera (gate ESPERANDO_BOTON).
    al_listo: () -> None, invocado una vez cuando el LiDAR ya esta escaneando.
    al_completar_scan: (clusters_obstaculos: list) -> None, invocado una vez
        por barrido completo, con los clusters clasificados como OBSTACULO.
    """
    global ser_lidar, angulo_previo, scan_buffer_acumulando, scan_buffer_listo

    try:
        ser_lidar = serial.Serial(PUERTO_LIDAR, baudrate=BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
        ser_lidar.write(START_MOTOR_CMD)
        time.sleep(1.5)
        ser_lidar.reset_input_buffer()
        ser_lidar.write(START_SCAN_CMD)
        time.sleep(0.5)

        if ser_lidar.in_waiting >= 7:
            ser_lidar.read(7)

        print("[+] Telemetria LiDAR activa.")
        al_listo()

        while obtener_corriendo():
            if obtener_fase() == "ESPERANDO_BOTON":
                time.sleep(0.1)
                continue

            b0 = ser_lidar.read(1)
            if not b0:
                continue
            byte0     = b0[0]
            start_bit = byte0 & 0x01
            start_bit_inv = (byte0 >> 1) & 0x01

            if start_bit != start_bit_inv:
                resto = ser_lidar.read(4)
                if len(resto) < 4:
                    continue
                byte1, byte2, byte3, byte4 = resto[0], resto[1], resto[2], resto[3]

                if (byte1 & 0x01) == 1:
                    raw_angle   = (byte2 << 7) | (byte1 >> 1)
                    angle       = raw_angle / 64.0
                    distance    = (byte4 << 8) | byte3
                    distance_mm = distance / 4.0

                    if 0 < distance_mm < 6000:
                        # Detectar wrap-around angular (inicio de nuevo barrido)
                        if angle < angulo_previo and (angulo_previo - angle) > 300.0:
                            # Ciclo completo: transferir buffer al slot "listo" y procesar
                            with lock_scan:
                                scan_buffer_listo     = list(scan_buffer_acumulando)
                                scan_buffer_acumulando = []

                            clusters_obstaculos = procesar_scan(scan_buffer_listo)
                            al_completar_scan(clusters_obstaculos)

                        angulo_previo = angle

                        # Acumular punto en el buffer del ciclo en curso
                        scan_buffer_acumulando.append((angle, distance_mm))

    except Exception as e:
        if obtener_corriendo():
            print(f"[-] Falla en hilo LiDAR: {e}")
