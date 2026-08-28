#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test de percepcion estatica: valida el clustering LiDAR + la vision
contra los pilares fisicos que hay delante del robot.

NO instancia EnlacePico y NO manda ninguna consigna: el robot no se mueve.
"""
import math
import threading
import time

import vision
from camara_driver import CamaraDriver
from lidar_driver import LidarDriver
from lidar_geometria import (ProcesadorLidar, es_cluster_obstaculo,
                             centroide_xy_cluster, segmentar_clusters_abd)

corriendo = True
barridos = []


def al_barrido(scan):
    if len(barridos) < 12:
        barridos.append(scan)


print("[*] Arrancando camara...")
camara = CamaraDriver()
threading.Thread(target=camara.hilo_captura,
                 args=(lambda: corriendo, vision.procesar_frame), daemon=True).start()
time.sleep(2.5)

print("[*] Arrancando LiDAR (gira, el robot no se mueve)...")
lidar = LidarDriver()
geo = ProcesadorLidar()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, al_barrido), daemon=True).start()

t0 = time.time()
while len(barridos) < 12 and time.time() - t0 < 25:
    time.sleep(0.1)

corriendo = False
time.sleep(0.4)
lidar.cerrar()

print(f"\n[*] Barridos capturados: {len(barridos)}")
print(f"[*] Color estable de camara: {vision.get_color()}")

if not barridos:
    print("[-] Sin barridos. El LiDAR no entrego datos.")
    raise SystemExit(1)

for idx, scan in enumerate(barridos[-4:]):
    med = geo.procesar(scan)
    print(f"\n=== Barrido {idx + 1}  ({len(scan)} puntos) ===")
    print(f"  frontal={med.frontal:7.1f}  izq={med.izquierda:7.1f}  der={med.derecha:7.1f}")

    todos = segmentar_clusters_abd(scan)
    obst = med.clusters_obstaculo
    print(f"  clusters totales={len(todos)}  clasificados como OBSTACULO={len(obst)}")

    for c in obst:
        x, y = centroide_xy_cluster(c)
        ext = c[-1][0] - c[0][0]
        if ext < 0:
            ext += 360.0
        dist = math.hypot(x, y)
        brg = math.degrees(math.atan2(x, y))
        print(f"    POSTE: n={len(c):3d}  arco={ext:5.2f}deg  dist={dist:6.1f}mm  "
              f"rumbo={brg:+6.1f}deg  x={x:+7.1f}  y={y:+7.1f}")

    # Clusters descartados que aun asi estan cerca y al frente: utiles para
    # ver que un muro NO se cuela como poste.
    for c in todos:
        if es_cluster_obstaculo(c):
            continue
        x, y = centroide_xy_cluster(c)
        if y < 200 or math.hypot(x, y) > 1500:
            continue
        ext = c[-1][0] - c[0][0]
        if ext < 0:
            ext += 360.0
        brg = math.degrees(math.atan2(x, y))
        print(f"    descartado (muro): n={len(c):3d}  arco={ext:6.2f}deg  "
              f"rumbo={brg:+6.1f}deg  dist={math.hypot(x, y):6.1f}mm")
