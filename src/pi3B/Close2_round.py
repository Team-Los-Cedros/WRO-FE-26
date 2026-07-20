import time
import math
import threading
import serial
import sys
import signal
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import cv2

# ==========================================
# CONFIGURACIÓN DE PUERTOS Y COMUNICACIÓN
# ==========================================
PUERTO_LIDAR = '/dev/ttyUSB0'
PUERTO_PICO = '/dev/ttyACM0'
BAUDRATE_LIDAR = 460800
BAUDRATE_PICO = 115200

START_MOTOR_CMD = b'\xa5\xf0\x02\x94\x02\xc1\x02'
START_SCAN_CMD = b'\xa5\x20'
STOP_CMD = b'\xa5\x25'

corriendo = True
ser_lidar = None
ser_pico = None

# ==========================================
# BOTÓN DE ARRANQUE (GP21)
# ==========================================
PIN_BOTON = 21
GPIO.setmode(GPIO.BCM)
try:
    GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
except Exception as e:
    print(f"[!] GPIO ocupado, liberando y reintentando... ({e})")
    try:
        GPIO.cleanup()
    except Exception:
        pass
    time.sleep(0.3)
    GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ==========================================
# CONSTANTES DE NAVEGACIÓN
# ==========================================
KP_LATERAL           = 0.14
VELOCIDAD_CRUCERO    = 55
VELOCIDAD_PARQUEO    = 20
VELOCIDAD_EVASION    = 40       # Conservador durante evasión

TIMEOUT_BUSQUEDA_PARQUEO = 6.0
tiempo_inicio_parqueo    = 0.0

# Sectores LiDAR de pared
ANGULO_MIN_DER = 30
ANGULO_MAX_DER = 90
ANGULO_MIN_IZQ = 270
ANGULO_MAX_IZQ = 330

dist_derecha_min   = 8000.0
dist_izquierda_min = 8000.0
dist_trasera_min   = 8000.0
angulo_previo      = 0.0
sentido_giro       = "DESCONOCIDO"

fase_actual       = "ESPERANDO_BOTON"
initial_derecha   = 0.0
initial_izquierda = 0.0

angulo_inicial_imu     = None
angulo_acumulado_robot = 0.0

# ==========================================
# CONSTANTES DE VISIÓN (postes rojo / verde)
# ==========================================
ANCHO_FRAME = 320
ALTO_FRAME  = 240

AREA_MIN_DETECCION  = 350
AREA_EVASION_FUERTE = 4500

KP_EVASION_LATERAL = 0.09
GANANCIA_CERCANIA  = 0.02
MAX_ANGULO_EVASION = 32.0

CONFIRMACIONES_PARA_ENTRAR = 2
CONFIRMACIONES_PARA_SALIR  = 4

MAX_DELTA_ANGULO_POR_CICLO = 6.0

# Sector frontal dinámico del LiDAR
ANGULO_MIN_FRONTAL = 350
ANGULO_MAX_FRONTAL = 10
dist_frontal_min   = 8000.0

DIST_FRENADO_INICIO      = 900.0
DIST_FRENADO_MIN         = 300.0
VELOCIDAD_MIN_EN_FRENADO = 25

# Rangos HSV
ROJO_BAJO_1 = np.array([0,   151,  99]);  ROJO_ALTO_1 = np.array([15,  255, 255])
ROJO_BAJO_2 = np.array([158, 160,  82]);  ROJO_ALTO_2 = np.array([179, 255, 255])
VERDE_BAJO  = np.array([43,   68,  50]);  VERDE_ALTO  = np.array([85,  255, 255])

# Estado compartido visión
lock_vision  = threading.Lock()
color_crudo  = None
cx_crudo     = None
area_cruda   = 0

poste_color = None
poste_cx    = None
poste_area  = 0
_contador_entrada = 0
_contador_salida  = 0

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
EXT_ANG_MAX_OBSTACULO  = 15.0    # Arco máximo que ocupa un poste (~10cm Ø a 200mm)
EXT_ANG_MIN_MURO       = 20.0    # Un muro siempre ocupa más de 20 deg

# Scan buffer: acumula el barrido completo antes de procesar (crítico para ABD)
lock_scan = threading.Lock()
scan_buffer_acumulando = []   # (angulo_deg, distancia_mm) — ciclo en curso
scan_buffer_listo      = []   # Último barrido completo disponible

# ==========================================
# OBJECT PERSISTENCE TRACKER
# Mantiene la posición estimada del obstáculo activo en coordenadas
# del robot, incluso cuando la cámara lo pierde durante la evasión.
# Actualizado por rotación IMU en cada ciclo LiDAR.
# ==========================================
lock_tracker = threading.Lock()
tracker = {
    "activo"        : False,
    "color"         : None,    # "ROJO" o "VERDE"
    "x"             : 0.0,     # mm, positivo = derecha del robot
    "y"             : 0.0,     # mm, positivo = adelante del robot
    "heading_ref"   : 0.0,     # heading IMU en el instante de captura
    "timestamp"     : 0.0,
    "confirmaciones": 0,
}
TIMEOUT_TRACKER    = 4.0     # s — máx. predicción pura sin re-detección
DISTANCIA_SUPERADO = 280.0   # mm detrás del eje trasero (cola ~200mm + margen)

# ==========================================
# ESTADO DE EVASIÓN
# FSM: CARRERA → DETECTADO → ESQUIVANDO → PASANDO → RECENTRANDO → CARRERA
#      Cualquier estado → RETROCEDIENDO → FORZANDO_GIRO → CARRERA  (emergencia)
# ==========================================
estado_evasion        = "CARRERA"
EVADIR_POR_IZQUIERDA  = True
heading_base_evasion  = 0.0
tiempo_inicio_evasion = 0.0
ultimo_angulo_aplicado = 0.0
KP_IMU = 1.0


# ==========================================
# APAGADO SEGURO
# ==========================================
_apagando_en_curso = False

def apagar_sistema(sig, frame):
    global corriendo, ser_lidar, ser_pico, _apagando_en_curso

    # Evita doble ejecución (ej. doble Ctrl+C) reentrando aquí mientras
    # el primer apagado todavía está en curso, lo cual dejaba a GPIO.cleanup()
    # operando sobre un handle ya cerrado y tumbaba el script con una excepción.
    if _apagando_en_curso:
        return
    _apagando_en_curso = True

    print("\n[!] Deteniendo sistema de forma segura...")
    corriendo = False
    time.sleep(0.2)
    if ser_pico and ser_pico.is_open:
        try:
            ser_pico.write(b"0,0\n")
            ser_pico.close()
        except: pass
    if ser_lidar and ser_lidar.is_open:
        try:
            ser_lidar.write(STOP_CMD)
            ser_lidar.close()
        except: pass
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f"[-] GPIO.cleanup() fallo (ignorado): {e}")
    sys.exit(0)

signal.signal(signal.SIGINT, apagar_sistema)


# ==========================================
# HISTÉRESIS DE VISIÓN
# ==========================================
def _aplicar_histeresis():
    global poste_color, poste_cx, poste_area
    global _contador_entrada, _contador_salida

    if poste_color is None:
        if color_crudo is not None:
            _contador_entrada += 1
            _contador_salida = 0
            if _contador_entrada >= CONFIRMACIONES_PARA_ENTRAR:
                poste_color = color_crudo
                poste_cx    = cx_crudo
                poste_area  = area_cruda
                _contador_entrada = 0
        else:
            _contador_entrada = 0
    else:
        if color_crudo == poste_color:
            poste_cx   = cx_crudo
            poste_area = area_cruda
            _contador_salida = 0
        else:
            _contador_salida += 1
            if _contador_salida >= CONFIRMACIONES_PARA_SALIR:
                poste_color = None
                poste_cx    = None
                poste_area  = 0
                _contador_salida  = 0
                _contador_entrada = 0


# ==========================================
# HILO: CÁMARA — Detección HSV de postes rojo/verde
# ==========================================
def hilo_camara():
    global poste_color, poste_cx, poste_area
    global color_crudo, cx_crudo, area_cruda

    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (ANCHO_FRAME, ALTO_FRAME), "format": "RGB888"},
            controls={"ScalerCrop": (0, 0, 4608, 2592)}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(1.0)
        print("[+] Camara Pi Cam Module 3 inicializada (FOV Full-Sensor).")
    except Exception as e:
        print(f"[-] Error inicializando camara: {e}")
        return

    kernel = np.ones((5, 5), np.uint8)

    while corriendo:
        try:
            frame = picam2.capture_array()
            # Picamera2 con formato "RGB888" entrega los bytes en orden BGR
            # (comportamiento documentado de la librería, pese al nombre).
            hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask_rojo  = cv2.inRange(hsv, ROJO_BAJO_1, ROJO_ALTO_1) | \
                         cv2.inRange(hsv, ROJO_BAJO_2, ROJO_ALTO_2)
            mask_verde = cv2.inRange(hsv, VERDE_BAJO, VERDE_ALTO)

            mask_rojo  = cv2.morphologyEx(mask_rojo,  cv2.MORPH_OPEN, kernel)
            mask_verde = cv2.morphologyEx(mask_verde, cv2.MORPH_OPEN, kernel)

            cont_rojo,  _ = cv2.findContours(mask_rojo,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cont_verde, _ = cv2.findContours(mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            mejor_color, mejor_cx, mejor_area = None, None, 0

            for c in cont_rojo:
                area = cv2.contourArea(c)
                if area > mejor_area and area > AREA_MIN_DETECCION:
                    x, y, w, h = cv2.boundingRect(c)
                    cy = y + h // 2
                    if cy < 180 and h > (w * 0.7):
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            mejor_color = "ROJO"
                            mejor_cx    = int(M["m10"] / M["m00"])
                            mejor_area  = area

            for c in cont_verde:
                area = cv2.contourArea(c)
                if area > mejor_area and area > AREA_MIN_DETECCION:
                    x, y, w, h = cv2.boundingRect(c)
                    cy = y + h // 2
                    if cy < 180 and h > (w * 0.7):
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            mejor_color = "VERDE"
                            mejor_cx    = int(M["m10"] / M["m00"])
                            mejor_area  = area

            with lock_vision:
                if mejor_area > 0:
                    color_crudo = mejor_color
                    cx_crudo    = mejor_cx
                    area_cruda  = mejor_area
                else:
                    color_crudo = None
                    cx_crudo    = None
                    area_cruda  = 0
                _aplicar_histeresis()

        except Exception as e:
            print(f"[-] Falla en hilo camara: {e}")
            time.sleep(0.1)

        time.sleep(0.03)  # ~30 fps


# ==========================================
# HILO: COMUNICACIÓN CON LA PICO 2 (sin cambios)
# ==========================================
def hilo_comunicacion_pico():
    global ser_pico, angulo_acumulado_robot, fase_actual, tiempo_inicio_parqueo, angulo_inicial_imu
    try:
        ser_pico = serial.Serial(PUERTO_PICO, baudrate=BAUDRATE_PICO, timeout=0.05)
        print("[+] Conexion serial establecida con Raspberry Pi Pico 2.")
    except Exception as e:
        print(f"[-] Error conectando a la Pi Pico 2: {e}")
        return

    while corriendo:
        if ser_pico.in_waiting > 0:
            try:
                linea = ser_pico.readline().decode('utf-8').strip()
                if linea.startswith("IMU:"):
                    valor_crudo_imu = float(linea.split(":")[1])

                    if fase_actual in ["ESPERANDO_BOTON", "CALIBRANDO"] or angulo_inicial_imu is None:
                        angulo_inicial_imu = valor_crudo_imu

                    angulo_acumulado_robot = valor_crudo_imu - angulo_inicial_imu

                    if fase_actual == "CARRERA" and abs(angulo_acumulado_robot) >= 1010.0:
                        fase_actual = "BUSCANDO_PARQUEO"
                        tiempo_inicio_parqueo = time.time()
                        print(f"[!] Ultima vuelta completada ({angulo_acumulado_robot:.1f} deg). Modo Parqueo.")
            except:
                pass
        time.sleep(0.01)


# ==========================================
# CLUSTERING ABD — Adaptive Breakpoint Detection
# Complejidad O(n), sin dependencias externas. Óptimo para Pi 3B.
# ==========================================
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


# ==========================================
# OBJECT PERSISTENCE TRACKER
# ==========================================
def iniciar_tracker(color, cx_mm, cy_mm, heading_actual):
    """Inicia el tracker con la posición cartesiana (mm) del obstáculo."""
    global tracker
    with lock_tracker:
        tracker["activo"]         = True
        tracker["color"]          = color
        tracker["x"]              = cx_mm
        tracker["y"]              = cy_mm
        tracker["heading_ref"]    = heading_actual
        tracker["timestamp"]      = time.time()
        tracker["confirmaciones"] = 1
    print(f"[TRACKER] Iniciado: {color} en ({cx_mm:.0f}, {cy_mm:.0f})mm | heading={heading_actual:.1f} deg")


def actualizar_tracker_imu(heading_actual):
    """
    Rota la posición predicha del obstáculo según el delta de heading IMU,
    manteniendo las coordenadas relativas correctas al robot.
    Llama ANTES de leer tracker['x'] y tracker['y'] en la lógica de control.
    """
    global tracker
    if not tracker["activo"]:
        return

    if time.time() - tracker["timestamp"] > TIMEOUT_TRACKER:
        with lock_tracker:
            tracker["activo"] = False
        print("[TRACKER] Expirado (timeout).")
        return

    # Delta de heading desde la última actualización (deg)
    delta_h_deg = heading_actual - tracker["heading_ref"]
    delta_h_rad = math.radians(delta_h_deg)

    # Rotación 2D: sistema de referencia del robot gira -> posición del obstáculo rota inversamente
    cos_d = math.cos(-delta_h_rad)
    sin_d = math.sin(-delta_h_rad)

    with lock_tracker:
        x_old = tracker["x"]
        y_old = tracker["y"]
        tracker["x"]          = x_old * cos_d - y_old * sin_d
        tracker["y"]          = x_old * sin_d + y_old * cos_d
        tracker["heading_ref"] = heading_actual   # Avanzar la referencia


def asociar_cluster_al_tracker(clusters_obstaculos):
    """
    Busca el cluster de obstáculo más cercano a la posición predicha del tracker.
    Si está a menos de 250mm, actualiza la posición con la medición real.
    Devuelve True si hubo asociación (corrección).
    """
    global tracker
    if not tracker["activo"] or not clusters_obstaculos:
        return False

    mejor_dist    = 250.0  # mm — umbral de asociación
    mejor_cluster = None

    for clust in clusters_obstaculos:
        cx, cy = centroide_xy_cluster(clust)
        d = math.sqrt((cx - tracker["x"])**2 + (cy - tracker["y"])**2)
        if d < mejor_dist:
            mejor_dist    = d
            mejor_cluster = clust

    if mejor_cluster is not None:
        cx, cy = centroide_xy_cluster(mejor_cluster)
        with lock_tracker:
            tracker["x"]              = cx
            tracker["y"]              = cy
            tracker["timestamp"]      = time.time()
            tracker["confirmaciones"] += 1
        return True
    return False


def desactivar_tracker(razon=""):
    """Desactiva el tracker y opcionalmente imprime la razón."""
    global tracker
    with lock_tracker:
        tracker["activo"] = False
    if razon:
        print(f"[TRACKER] Desactivado: {razon}")


def obstaculo_al_frente():
    """True si el obstáculo predicho sigue en zona frontal del robot (y > -50mm)."""
    return tracker["activo"] and tracker["y"] > -50.0


def obstaculo_fue_superado():
    """
    True cuando la posición predicha indica que el obstáculo quedo DETRAS
    del robot (y < -DISTANCIA_SUPERADO, es decir, más allá de la cola).
    """
    return tracker["activo"] and tracker["y"] < -DISTANCIA_SUPERADO


def obstaculo_al_costado(lado):
    """
    True si el obstáculo predicho está en zona lateral del robot.
    lado='DER' -> x positivo, lado='IZQ' -> x negativo.
    Zona lateral: |y| < 350mm y |x| > 50mm.
    """
    if not tracker["activo"]:
        return False
    al_costado_y = abs(tracker["y"]) < 350.0
    if lado == "DER":
        return al_costado_y and tracker["x"] > 50.0
    if lado == "IZQ":
        return al_costado_y and tracker["x"] < -50.0
    return False


# ==========================================
# PROCESAMIENTO DEL SCAN COMPLETO
# Actualiza distancias de pared + ejecuta clustering ABD + actualiza tracker.
# Llamado internamente desde procesar_ciclo_completo_lidar().
# ==========================================
def _procesar_scan_interno(scan):
    """
    Procesa un barrido completo del RPLIDAR C1.
    1. Actualiza dist_*_min (seguimiento de pared, igual que código original).
    2. Clustering ABD en zona relevante (excluye trasera 120-240 deg).
    3. Clasifica clusters en OBSTACULO / MURO / RUIDO.
    4. Actualiza el tracker con IMU + asociación de clusters reales.
    """
    global dist_derecha_min, dist_izquierda_min, dist_frontal_min, dist_trasera_min

    if not scan:
        return

    # --- 1. Distancias de pared (idéntico al código original) ---
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

    dist_derecha_min   = d_der   if d_der   < 4000 else 2000.0
    dist_izquierda_min = d_izq   if d_izq   < 4000 else 2000.0
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

    # --- 4. Actualizar tracker ---
    # Primero corrección por IMU (predicción)
    actualizar_tracker_imu(angulo_acumulado_robot)

    if tracker["activo"]:
        # Intentar asociar cluster real al tracker existente (corrección de deriva)
        asociar_cluster_al_tracker(clusters_obstaculos)
    else:
        # Si no hay tracker activo, crear uno si cámara confirma color Y hay cluster frontal
        with lock_vision:
            color_cam = poste_color

        if color_cam is not None and clusters_obstaculos:
            cluster_frente = None
            dist_minima    = DIST_MAX_OBSTACULO

            for clust in clusters_obstaculos:
                cx_c, cy_c = centroide_xy_cluster(clust)
                # El cluster debe estar en zona frontal (y > 0, no muy lateral)
                if cy_c > 80.0 and abs(cx_c) < 450.0:
                    d = math.sqrt(cx_c**2 + cy_c**2)
                    if d < dist_minima:
                        dist_minima    = d
                        cluster_frente = clust

            if cluster_frente is not None:
                cx_f, cy_f = centroide_xy_cluster(cluster_frente)
                iniciar_tracker(color_cam, cx_f, cy_f, angulo_acumulado_robot)


# ==========================================
# CONTROL PRINCIPAL — Procesamiento de un barrido completo del LiDAR
# ==========================================
def procesar_ciclo_completo_lidar():
    """
    Llamado una vez por barrido completo del RPLIDAR C1.
    Primero procesa el scan (clustering + tracker), luego ejecuta la FSM de navegación.
    """
    global dist_derecha_min, dist_izquierda_min, fase_actual
    global initial_derecha, initial_izquierda, ser_pico, angulo_acumulado_robot
    global tiempo_inicio_parqueo
    global estado_evasion, ultimo_angulo_aplicado, dist_frontal_min, heading_base_evasion
    global tiempo_inicio_evasion, ANGULO_MIN_FRONTAL, ANGULO_MAX_FRONTAL
    global dist_trasera_min, sentido_giro, EVADIR_POR_IZQUIERDA

    if ser_pico is None or not ser_pico.is_open:
        return

    # Obtener el scan del buffer listo (copiado atómicamente en hilo_lidar)
    with lock_scan:
        scan = list(scan_buffer_listo)

    if scan:
        _procesar_scan_interno(scan)

    # ─────────────────────────────────────────────────────────────────────────
    # FASE: CAPTURA_INICIAL
    # ─────────────────────────────────────────────────────────────────────────
    if fase_actual == "CAPTURA_INICIAL":
        initial_derecha   = dist_derecha_min
        initial_izquierda = dist_izquierda_min
        fase_actual = "CARRERA"
        print(f"[+] Firma de Parqueo: Izq={initial_izquierda:.0f}mm | Der={initial_derecha:.0f}mm")
        print("[INICIO] Carrera con obstaculos iniciada!")
        return

    # ─────────────────────────────────────────────────────────────────────────
    # FASE: CARRERA
    # ─────────────────────────────────────────────────────────────────────────
    if fase_actual == "CARRERA":
        frontal = dist_frontal_min if dist_frontal_min < 4000 else 4000.0

        # 0. Detectar sentido de giro de la pista
        if sentido_giro == "DESCONOCIDO":
            if angulo_acumulado_robot < -45.0:
                sentido_giro = "HORARIO"
                print("[GIRO] Sentido: HORARIO (derecha)")
            elif angulo_acumulado_robot > 45.0:
                sentido_giro = "ANTIHORARIO"
                print("[GIRO] Sentido: ANTIHORARIO (izquierda)")

        # 1. Leer estado de visión
        with lock_vision:
            color = poste_color

        # 2. Frenado progresivo por distancia frontal LiDAR
        if frontal <= DIST_FRENADO_MIN:
            factor_frenado = VELOCIDAD_MIN_EN_FRENADO / VELOCIDAD_CRUCERO
        elif frontal >= DIST_FRENADO_INICIO:
            factor_frenado = 1.0
        else:
            proporcion     = (frontal - DIST_FRENADO_MIN) / (DIST_FRENADO_INICIO - DIST_FRENADO_MIN)
            factor_frenado = (VELOCIDAD_MIN_EN_FRENADO / VELOCIDAD_CRUCERO +
                              proporcion * (1.0 - VELOCIDAD_MIN_EN_FRENADO / VELOCIDAD_CRUCERO))

        # 3. Sistema Anti-Choque de Emergencia (mantenido del código original)
        if frontal < 120 or dist_izquierda_min < 80 or dist_derecha_min < 80:
            if estado_evasion not in ["RETROCEDIENDO", "FORZANDO_GIRO"]:
                estado_evasion        = "RETROCEDIENDO"
                tiempo_inicio_evasion = time.time()
                print(f"[EMERGENCIA] COLISION INMINENTE! F:{frontal:.0f} I:{dist_izquierda_min:.0f} D:{dist_derecha_min:.0f}mm")

        # ─────────────────────────────────────────────────────────────────────
        # 4. MÁQUINA DE ESTADOS DE EVASIÓN — 6 estados con transiciones geométricas
        # ─────────────────────────────────────────────────────────────────────

        # ── Estado CARRERA (seguimiento de pared normal) ──────────────────────
        if estado_evasion == "CARRERA":
            trk_activo = tracker["activo"]
            # El tracker confirma obstáculo frontal: y > 50mm (adelante) y < 800mm (en rango)
            trk_al_frente = trk_activo and 50.0 < tracker["y"] < 800.0

            if trk_al_frente or (50 < frontal < 700 and color is not None):
                estado_evasion        = "DETECTADO"
                heading_base_evasion  = angulo_acumulado_robot
                tiempo_inicio_evasion = time.time()

                # Determinar lado de evasión (tracker tiene prioridad sobre cámara)
                # Regla WRO: ROJO -> evadir por la DERECHA, VERDE -> evadir por la IZQUIERDA
                color_det = tracker["color"] if trk_activo and tracker["color"] else color
                EVADIR_POR_IZQUIERDA = (color_det == "VERDE")

                # Ampliar sector frontal dinámicamente para no perder el cluster al girar
                if EVADIR_POR_IZQUIERDA:
                    ANGULO_MIN_FRONTAL = 330
                    ANGULO_MAX_FRONTAL = 20
                else:
                    ANGULO_MIN_FRONTAL = 340
                    ANGULO_MAX_FRONTAL = 30

                lado_str = "IZQUIERDA" if EVADIR_POR_IZQUIERDA else "DERECHA"
                print(f"[FSM] CARRERA -> DETECTADO | {color_det} | Evadir x {lado_str} | F:{frontal:.0f}mm")

        # ── Estado DETECTADO (confirmando obstáculo, frenando suavemente) ──────
        elif estado_evasion == "DETECTADO":
            trk_confirmado  = tracker["activo"] and tracker["confirmaciones"] >= 2
            tiempo_detectado = time.time() - tiempo_inicio_evasion

            # Transición a ESQUIVANDO si el tracker confirma Y el obstáculo está a <600mm,
            # o si el obstáculo está muy cerca (por seguridad), o tras 0.3s (fallback).
            if (trk_confirmado and frontal < 600) or frontal < 400 or tiempo_detectado > 0.3:
                estado_evasion        = "ESQUIVANDO"
                tiempo_inicio_evasion = time.time()
                lado_str = "IZQUIERDA" if EVADIR_POR_IZQUIERDA else "DERECHA"
                print(f"[FSM] DETECTADO -> ESQUIVANDO {lado_str} | F:{frontal:.0f}mm")

        # ── Estado ESQUIVANDO (giro lateral activo) ───────────────────────────
        elif estado_evasion == "ESQUIVANDO":
            tiempo_esquivando = time.time() - tiempo_inicio_evasion

            # El frente LiDAR quedó libre (obstáculo ya no bloquea el camino)
            frente_libre = frontal > 750.0

            # El tracker indica que el obstáculo pasó al costado
            lado_costado = "DER" if not EVADIR_POR_IZQUIERDA else "IZQ"
            obst_lateral = obstaculo_al_costado(lado_costado)

            # El tracker indica que el obstáculo ya quedó detrás
            obst_superado = obstaculo_fue_superado()

            # Tiempo mínimo de giro para que el servo actúe (0.2s)
            if tiempo_esquivando > 0.2 and (frente_libre or obst_lateral or obst_superado):
                estado_evasion        = "PASANDO"
                tiempo_inicio_evasion = time.time()
                ANGULO_MIN_FRONTAL    = 350
                ANGULO_MAX_FRONTAL    = 10
                print(f"[FSM] ESQUIVANDO -> PASANDO | Frente libre:{frente_libre} Lateral:{obst_lateral}")

        # ── Estado PASANDO (robot pasa junto al obstáculo, rumbo paralelo) ─────
        elif estado_evasion == "PASANDO":
            # Criterio GEOMÉTRICO principal: tracker confirma que el obstáculo
            # está ya detrás del robot (y < -DISTANCIA_SUPERADO).
            # Criterio de seguridad (timeout): 1.2s por si el tracker falla.
            superado  = obstaculo_fue_superado()
            tiempo_p  = time.time() - tiempo_inicio_evasion
            timeout_p = tiempo_p > 1.2

            if superado or timeout_p:
                estado_evasion        = "RECENTRANDO"
                tiempo_inicio_evasion = time.time()
                razon = "geometrico" if superado else "timeout"
                desactivar_tracker(f"Superado ({razon})")
                print(f"[FSM] PASANDO -> RECENTRANDO | {razon}")

        # ── Estado RECENTRANDO (volver al heading original) ──────────────────
        elif estado_evasion == "RECENTRANDO":
            error_heading      = abs(heading_base_evasion - angulo_acumulado_robot)
            tiempo_recentrando = time.time() - tiempo_inicio_evasion

            if error_heading < 4.0 or tiempo_recentrando > 1.5:
                estado_evasion = "CARRERA"
                ANGULO_MIN_FRONTAL = 350
                ANGULO_MAX_FRONTAL = 10
                print(f"[FSM] RECENTRANDO -> CARRERA | Error heading={error_heading:.1f} deg")

        # ── Estado RETROCEDIENDO (anti-choque, idéntico al código original) ───
        elif estado_evasion == "RETROCEDIENDO":
            if sentido_giro == "HORARIO":
                angulo_objetivo_crudo = 30.0
            elif sentido_giro == "ANTIHORARIO":
                angulo_objetivo_crudo = -30.0
            else:
                angulo_objetivo_crudo = 0.0

            velocidad_base    = -35
            tiempo_retroceso  = time.time() - tiempo_inicio_evasion
            choque_trasero    = dist_trasera_min < 250.0

            if choque_trasero or tiempo_retroceso > 3.5:
                estado_evasion        = "FORZANDO_GIRO"
                tiempo_inicio_evasion = time.time()
                razon = "obstaculo trasero" if choque_trasero else "tiempo maximo"
                print(f"[FSM] RETROCEDIENDO -> FORZANDO_GIRO ({razon})")

        # ── Estado FORZANDO_GIRO (anti-choque, idéntico al código original) ───
        elif estado_evasion == "FORZANDO_GIRO":
            if sentido_giro == "HORARIO":
                angulo_objetivo_crudo = -30.0
            elif sentido_giro == "ANTIHORARIO":
                angulo_objetivo_crudo = 30.0
            else:
                angulo_objetivo_crudo = 0.0

            velocidad_base = VELOCIDAD_EVASION
            if (time.time() - tiempo_inicio_evasion) > 0.6:
                estado_evasion = "CARRERA"
                print("[FSM] FORZANDO_GIRO -> CARRERA")

        # ─────────────────────────────────────────────────────────────────────
        # 5. CÁLCULO DEL ÁNGULO OBJETIVO Y VELOCIDAD según estado
        # ─────────────────────────────────────────────────────────────────────

        if estado_evasion == "RETROCEDIENDO":
            if sentido_giro == "HORARIO":
                angulo_objetivo_crudo = 30.0
            elif sentido_giro == "ANTIHORARIO":
                angulo_objetivo_crudo = -30.0
            else:
                angulo_objetivo_crudo = 0.0
            velocidad_base = -35

        elif estado_evasion == "FORZANDO_GIRO":
            if sentido_giro == "HORARIO":
                angulo_objetivo_crudo = -30.0
            elif sentido_giro == "ANTIHORARIO":
                angulo_objetivo_crudo = 30.0
            else:
                angulo_objetivo_crudo = 0.0
            velocidad_base = VELOCIDAD_EVASION

        elif estado_evasion == "DETECTADO":
            # Pre-giro suave mientras confirmamos el obstáculo
            angulo_objetivo_crudo = 15.0 if EVADIR_POR_IZQUIERDA else -15.0
            velocidad_base = max(VELOCIDAD_MIN_EN_FRENADO,
                                 int(VELOCIDAD_CRUCERO * factor_frenado))

        elif estado_evasion == "ESQUIVANDO":
            # Giro completo
            angulo_objetivo_crudo = 28.0 if EVADIR_POR_IZQUIERDA else -28.0
            velocidad_base = VELOCIDAD_EVASION

        elif estado_evasion == "PASANDO":
            # Control P con IMU: mantener rumbo paralelo a las paredes
            error_h = heading_base_evasion - angulo_acumulado_robot
            angulo_objetivo_crudo = max(-22.0, min(22.0, error_h * KP_IMU))
            velocidad_base = VELOCIDAD_EVASION

        elif estado_evasion == "RECENTRANDO":
            # Control P más agresivo para volver al heading original
            error_h = heading_base_evasion - angulo_acumulado_robot
            angulo_objetivo_crudo = max(-25.0, min(25.0, error_h * KP_IMU * 1.2))
            velocidad_base = VELOCIDAD_EVASION

        else:
            # CARRERA normal: centrado entre paredes por LiDAR
            error_lateral = dist_izquierda_min - dist_derecha_min
            angulo_objetivo_crudo = error_lateral * KP_LATERAL
            velocidad_base = VELOCIDAD_CRUCERO
            ANGULO_MIN_FRONTAL = 350
            ANGULO_MAX_FRONTAL = 10

        # Frenado de emergencia adicional en modo CARRERA
        if frontal < DIST_FRENADO_MIN and estado_evasion == "CARRERA":
            velocidad_base = VELOCIDAD_MIN_EN_FRENADO

        # 6. RATE LIMITER: limitar máxima variación de ángulo por ciclo
        delta = angulo_objetivo_crudo - ultimo_angulo_aplicado
        delta = max(-MAX_DELTA_ANGULO_POR_CICLO, min(MAX_DELTA_ANGULO_POR_CICLO, delta))
        angulo_objetivo        = ultimo_angulo_aplicado + delta
        ultimo_angulo_aplicado = angulo_objetivo

        if estado_evasion in ["RETROCEDIENDO", "FORZANDO_GIRO"]:
            velocidad = int(velocidad_base)
        else:
            velocidad = max(VELOCIDAD_MIN_EN_FRENADO,
                            int(velocidad_base * factor_frenado))

        comando = f"{velocidad},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())

    # ─────────────────────────────────────────────────────────────────────────
    # FASE: BUSCANDO_PARQUEO (sin cambios respecto al código original)
    # ─────────────────────────────────────────────────────────────────────────
    elif fase_actual == "BUSCANDO_PARQUEO":
        error_lateral   = dist_izquierda_min - dist_derecha_min
        angulo_objetivo = error_lateral * KP_LATERAL
        comando = f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())

        match_firma = (abs(dist_derecha_min   - initial_derecha)   < 80.0 and
                       abs(dist_izquierda_min - initial_izquierda) < 80.0)
        tiempo_t     = time.time() - tiempo_inicio_parqueo
        timeout      = tiempo_t > TIMEOUT_BUSQUEDA_PARQUEO

        if match_firma or timeout:
            fase_actual = "DETENIDO"
            if timeout:
                print(f"[PARQUEO] Timeout ({tiempo_t:.1f}s). Deteniendo en zona segura.")
            else:
                print("[PARQUEO] Firma detectada! Estacionando...")
            for _ in range(5):
                ser_pico.write(b"0,0\n")
                time.sleep(0.01)
            apagar_sistema(None, None)


# ==========================================
# HILO: LIDAR — Parseo de paquetes RPLIDAR C1 y acumulacion del scan buffer
# ==========================================
def hilo_lidar():
    global ser_lidar, corriendo, angulo_previo, fase_actual
    global scan_buffer_acumulando, scan_buffer_listo
    global ANGULO_MIN_FRONTAL, ANGULO_MAX_FRONTAL

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
        if fase_actual == "CALIBRANDO":
            fase_actual = "CAPTURA_INICIAL"

        while corriendo:
            if fase_actual == "ESPERANDO_BOTON":
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

                            procesar_ciclo_completo_lidar()

                        angulo_previo = angle

                        # Acumular punto en el buffer del ciclo en curso
                        scan_buffer_acumulando.append((angle, distance_mm))

    except Exception as e:
        if corriendo:
            print(f"[-] Falla en hilo LiDAR: {e}")


# ==========================================
# PUNTO DE ENTRADA
# ==========================================
if __name__ == '__main__':
    # 1. Cámara primero (necesita calentamiento / AE)
    t_camara = threading.Thread(target=hilo_camara, daemon=True)
    t_camara.start()

    # 2. Canal de comunicación con la Pico
    t_pico = threading.Thread(target=hilo_comunicacion_pico, daemon=True)
    t_pico.start()

    time.sleep(0.5)
    if ser_pico and ser_pico.is_open:
        ser_pico.write(b"0,0\n")
        print("[INIT] Direccion centrada.")

    print("\n[LISTO] SISTEMA LISTO (RONDA CON OBSTACULOS). Coloca el robot y presiona el Boton (GP21)...")
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        if ser_pico and ser_pico.is_open:
            ser_pico.write(b"0,0\n")
        time.sleep(0.05)

    print("\n[START] Boton detectado! Iniciando carrera con obstaculos...")
    fase_actual = "CALIBRANDO"
    time.sleep(0.1)

    # 3. Hilo LiDAR (arranca después del botón para sincronizar el cero IMU)
    t_lidar = threading.Thread(target=hilo_lidar, daemon=True)
    t_lidar.start()

    while corriendo:
        time.sleep(1)