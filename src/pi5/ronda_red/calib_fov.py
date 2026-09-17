#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibracion del modelo cx -> rumbo con DOS pilares, que es lo unico que
separa la focal del centro optico.

Por que hace falta (ver README seccion 7): todas las medidas anteriores
usaban UNA referencia, y con una sola no se pueden despejar dos
incognitas. La misma esquina da f=1261 si se supone el centro optico en
el centro geometrico, y f=1436 si se admite el sesgo que midio el
barrido del pilar. El barrido tampoco los separa: sus rumbos cubren 7.7
grados y ahi f y c0 se compensan. Con dos pilares bien separados el
sistema queda determinado:

    cx = c0 + f * tan(rumbo)        dos ecuaciones, dos incognitas

Diferencias con herramientas/calib_fov.py, que nunca se corrio:

  - NO rota 180 grados. La camara quedo derecha en el mastil.
  - Usa la configuracion de carrera (raw 2304x1296 -> 640x360), asi que
    el resultado se pega en optica.py sin reescalar nada.
  - Corrige el PARALAJE. La camara va ~100mm detras del LiDAR, asi que
    el rumbo con que el LiDAR ve un poste NO es el rumbo con que lo ve
    la camara. Ignorarlo mete hasta 5 grados a distancia corta y
    contamina la focal, que es justo lo que se quiere medir.
  - Promedia sobre muchas muestras y da el residuo, en vez de resolver
    con un solo par de puntos.

COLOCACION: un pilar bien a la izquierda y otro bien a la derecha, a
~700-1000mm, separados al menos 30 grados de rumbo. Cuanto mas abierta
la base, mejor condicionado el ajuste; el script avisa si se queda corta.

Robot QUIETO: no instancia EnlacePico.
"""
import math
import statistics
import threading
import time

import cv2
import numpy as np

import optica
import vision
from camara_driver import CamaraDriver
from lidar_driver import LidarDriver
from lidar_geometria import (ProcesadorLidar, centroide_xy_cluster,
                             es_objeto_estrecho)

N_MUESTRAS = 30
BASE_MINIMA_GRADOS = 25.0   # por debajo de esto el ajuste es fragil
# Cuanto puede desviarse el rumbo que predice el modelo ACTUAL del rumbo
# real del cluster para darlos por el mismo poste. Generoso porque el
# modelo esta sin cerrar (de eso va este script), pero muy por debajo de
# la separacion entre dos pilares bien puestos.
VENTANA_APAREO = 20.0

corriendo = True
ultimo_frame = [None]
ultimo_scan = [None]


def cx_por_color(frame):
    """cx y area del mayor blob de cada color. Mismos umbrales que vision.py."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = {
        "ROJO":  cv2.inRange(hsv, vision.ROJO_BAJO_1, vision.ROJO_ALTO_1) |
                 cv2.inRange(hsv, vision.ROJO_BAJO_2, vision.ROJO_ALTO_2),
        "VERDE": cv2.inRange(hsv, vision.VERDE_BAJO, vision.VERDE_ALTO),
    }
    out = {}
    for nombre, m in masks.items():
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, vision._KERNEL)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mejor, mejor_area = None, 0
        for c in cnts:
            a = cv2.contourArea(c)
            if a > mejor_area and a > vision.AREA_MIN_DETECCION:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    mejor, mejor_area = M["m10"] / M["m00"], a
        if mejor is not None:
            out[nombre] = (mejor, mejor_area)
    return out


print("[*] Arrancando camara (mastil, sin rotacion)...")
camara = CamaraDriver()
threading.Thread(target=camara.hilo_captura,
                 args=(lambda: corriendo, lambda f: ultimo_frame.__setitem__(0, f)),
                 daemon=True).start()
time.sleep(2.5)

print("[*] Arrancando LiDAR. El robot NO se mueve.")
lidar = LidarDriver()
geo = ProcesadorLidar()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, lambda s: ultimo_scan.__setitem__(0, s)),
                 daemon=True).start()
time.sleep(2.0)

print("[*] Tomando %d muestras...\n" % N_MUESTRAS)
puntos = []     # (color, cx, rumbo_camara, dist)
descartes = {"sin_dos_colores": 0, "sin_dos_clusters": 0, "sin_apareo": 0}

for _ in range(N_MUESTRAS):
    time.sleep(0.25)
    frame, scan = ultimo_frame[0], ultimo_scan[0]
    if frame is None or scan is None:
        continue

    colores = cx_por_color(frame)
    if len(colores) < 2:
        descartes["sin_dos_colores"] += 1
        continue

    med = geo.procesar(scan)
    # Solo objetos ESTRECHOS. `clusters_obstaculo` admite hasta 15 grados
    # de arco, y con eso se cuelan esquinas de muro: en la primera
    # corrida el LiDAR entregaba 3-5 clusters con dos pilares en pista.
    # es_objeto_estrecho filtra por ancho fisico (<=260mm de arco), que
    # un tramo de muro no pasa.
    clusters = []
    for c in med.clusters_obstaculo:
        if not es_objeto_estrecho(c):
            continue
        x, y = centroide_xy_cluster(c)
        if y > 200:
            # Rumbo visto desde la CAMARA, no desde el LiDAR: es el unico
            # que se puede comparar con un pixel.
            clusters.append((optica.rumbo_camara_de_cluster(x, y), math.hypot(x, y)))
    if len(clusters) < 2:
        descartes["sin_dos_clusters"] += 1
        continue

    # Emparejar cada color con el cluster que mejor case en rumbo,
    # usando el modelo ACTUAL como pista. Emparejar "por orden" no vale:
    # basta un cluster espurio a un lado para desplazar la lista entera
    # y asignarle a un color el rumbo de un muro -- que es exactamente lo
    # que paso en la primera corrida (verde en +23 grados de camara
    # emparejado con un cluster en -22.5). El modelo actual puede estar
    # equivocado en la MAGNITUD, pero no tanto como para confundir dos
    # postes separados 50 grados, asi que sirve de semilla.
    libres = list(clusters)
    asignados = []
    for nombre, (cx, _area) in sorted(colores.items(),
                                      key=lambda kv: abs(optica.rumbo_de_cx(kv[1][0])),
                                      reverse=True):
        if not libres:
            break
        pred = optica.rumbo_de_cx(cx)
        mejor = min(libres, key=lambda c: abs(c[0] - pred))
        if abs(mejor[0] - pred) > VENTANA_APAREO:
            continue
        libres.remove(mejor)
        asignados.append((nombre, cx, mejor[0], mejor[1]))
    if len(asignados) < 2:
        descartes["sin_apareo"] += 1
        continue
    puntos.extend(asignados)

corriendo = False
time.sleep(0.4)
lidar.cerrar()
camara.cerrar()

if len(puntos) < 6:
    print("[-] Solo %d puntos utiles. Descartes: %s" % (len(puntos), descartes))
    print("    Revisa que los DOS pilares esten en cuadro y que el LiDAR")
    print("    los vea como clusters (y > 200mm).")
    raise SystemExit(1)

print("[+] %d puntos utiles.  Descartes: %s\n" % (len(puntos), descartes))
print("=== Puntos por color ===")
for color in ("ROJO", "VERDE"):
    ps = [p for p in puntos if p[0] == color]
    if not ps:
        print("  %-6s sin puntos" % color)
        continue
    print("  %-6s n=%3d  cx=%7.1f  rumbo=%+6.2f deg  dist=%6.0f mm"
          % (color, len(ps), statistics.median(p[1] for p in ps),
             statistics.median(p[2] for p in ps), statistics.median(p[3] for p in ps)))

base = max(p[2] for p in puntos) - min(p[2] for p in puntos)
if base < BASE_MINIMA_GRADOS:
    print("\n  base angular = %.1f grados   [!] POR DEBAJO DE %.0f" % (base, BASE_MINIMA_GRADOS))
    print("      f y c0 se compensan y el ajuste sera fragil.")
    print("      Separa mas los pilares y repite.")
else:
    print("\n  base angular = %.1f grados   (suficiente para separar focal y centro)" % base)

# --- Minimos cuadrados de cx = c0 + f*tan(rumbo) ---
A = np.array([[1.0, math.tan(math.radians(p[2]))] for p in puntos])
b = np.array([p[1] for p in puntos])
(c0, f), *_ = np.linalg.lstsq(A, b, rcond=None)
res_px = b - A.dot(np.array([c0, f]))
res_deg = [math.degrees(math.atan2(p[1] - c0, f)) - p[2] for p in puntos]
hfov = 2.0 * math.degrees(math.atan((optica.ANCHO_FRAME / 2.0) / f))
sesgo = math.degrees(math.atan((c0 - optica.ANCHO_FRAME / 2.0) / f))

print("\n=== AJUSTE  cx = c0 + f*tan(rumbo)   (%d puntos) ===" % len(puntos))
print("  focal          f  = %7.1f px" % f)
print("  centro optico  c0 = %7.1f px   (centro geometrico %.0f, sesgo %+.2f grados)"
      % (c0, optica.ANCHO_FRAME / 2.0, sesgo))
print("  HFOV efectivo     = %6.1f grados" % hfov)
print("  residuo           = %.2f grados de desviacion, maximo %.2f  (%.1f px)"
      % (statistics.pstdev(res_deg), max(abs(r) for r in res_deg), float(np.std(res_px))))

print("\n=== Contraste con lo que usa optica.py ahora ===")
print("  %-18s %10s %10s" % ("", "ahora", "medido"))
print("  %-18s %10.1f %10.1f" % ("FOCAL_PX", optica.FOCAL_PX, f))
print("  %-18s %10.1f %10.1f" % ("CX_CENTRO_OPTICO", optica.CX_CENTRO_OPTICO, c0))
print("  %-18s %10.1f %10.1f" % ("HFOV_EFECTIVO", optica.HFOV_EFECTIVO, hfov))

print("\n=== Para pegar en optica.py ===")
print("  FOCAL_SENSOR_PX     = %.0f" % (f * (4608.0 / optica.ANCHO_FRAME)))
print("  SESGO_CENTRO_GRADOS = %.2f" % sesgo)
print("  (FOCAL_PX y CX_CENTRO_OPTICO se recalculan solos a partir de esos")
print("   dos, asi que el modelo sobrevive a otro cambio de modo de camara.)")
