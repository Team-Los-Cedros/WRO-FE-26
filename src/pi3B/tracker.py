"""
OBJECT PERSISTENCE TRACKER

Mantiene la posición estimada del obstáculo activo en coordenadas del
robot (x, y en mm; x+ = derecha, y+ = frente), incluso cuando la cámara
lo pierde durante la evasión. Se actualiza por rotación IMU en cada
ciclo LiDAR y se corrige cuando el clustering ABD (lidar.py) vuelve a
asociar un cluster real cercano a la posición predicha.

Este módulo no importa nada de Close2_round.py -- lo orquesta el
script principal, que le pasa el heading IMU y los clusters detectados
en cada ciclo.
"""
import math
import time
import threading

from lidar import centroide_xy_cluster

TIMEOUT_TRACKER    = 4.0     # s -- máx. predicción pura sin re-detección
DISTANCIA_SUPERADO = 280.0   # mm detrás del eje trasero (cola ~200mm + margen)

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

    mejor_dist    = 250.0  # mm -- umbral de asociación
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
