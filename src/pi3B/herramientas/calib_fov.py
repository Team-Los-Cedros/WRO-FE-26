#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibracion del mapeo cx (pixel de camara) -> rumbo (grados), usando los
dos pilares fisicos como puntos de referencia: la camara da su cx y el
LiDAR da su rumbo real en el mismo instante.

Sirve para aparear deteccion de color con cluster LiDAR por rumbo en vez
de por "el mas cercano".

NO manda ninguna consigna: el robot no se mueve.
"""
import math
import threading
import time

import cv2
import numpy as np
from picamera2 import Picamera2

from lidar_driver import LidarDriver
from lidar_geometria import ProcesadorLidar, centroide_xy_cluster

import vision  # para reutilizar exactamente los mismos umbrales HSV

ANCHO, ALTO = 320, 240

# ---------- Camara: cx de CADA color por separado ----------
picam = Picamera2()
picam.configure(picam.create_video_configuration(main={"size": (ANCHO, ALTO), "format": "RGB888"}))
picam.start()
time.sleep(2.5)

acum = {"ROJO": [], "VERDE": []}
for _ in range(10):
    # La camara va montada invertida: hay que rotar 180 como hace camara_driver
    frame = cv2.rotate(picam.capture_array()[:, :, :3], cv2.ROTATE_180)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = {
        "ROJO":  cv2.inRange(hsv, vision.ROJO_BAJO_1, vision.ROJO_ALTO_1) |
                 cv2.inRange(hsv, vision.ROJO_BAJO_2, vision.ROJO_ALTO_2),
        "VERDE": cv2.inRange(hsv, vision.VERDE_BAJO, vision.VERDE_ALTO),
    }
    for nombre, m in masks.items():
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, vision._KERNEL)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mejor, mejor_area = None, 0
        for c in cnts:
            a = cv2.contourArea(c)
            if a > mejor_area and a > vision.AREA_MIN_DETECCION:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    mejor, mejor_area = int(M["m10"] / M["m00"]), a
        if mejor is not None:
            acum[nombre].append((mejor, mejor_area))
    time.sleep(0.05)

picam.stop()

cam = {}
print("=== CAMARA ===")
for nombre, vals in acum.items():
    if vals:
        cx = sum(v[0] for v in vals) / len(vals)
        ar = sum(v[1] for v in vals) / len(vals)
        cam[nombre] = cx
        print(f"  {nombre:6s} cx={cx:6.1f} px   area={ar:7.0f}   ({len(vals)}/10 frames)")
    else:
        print(f"  {nombre:6s} NO DETECTADO")

# ---------- LiDAR: rumbo de cada cluster de poste ----------
corriendo = True
barridos = []
lidar = LidarDriver()
geo = ProcesadorLidar()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, lambda s: barridos.append(s) if len(barridos) < 8 else None),
                 daemon=True).start()
t0 = time.time()
while len(barridos) < 8 and time.time() - t0 < 25:
    time.sleep(0.1)
corriendo = False
time.sleep(0.3)
lidar.cerrar()

print(f"\n=== LIDAR ({len(barridos)} barridos) ===")
postes = []
for scan in barridos[-3:]:
    med = geo.procesar(scan)
    for c in med.clusters_obstaculo:
        x, y = centroide_xy_cluster(c)
        if y > 200 and abs(x) < 500:
            postes.append((math.degrees(math.atan2(x, y)), math.hypot(x, y)))

if postes:
    postes.sort()
    print("  rumbos detectados (ordenados izq -> der):")
    for b, d in postes:
        print(f"    rumbo={b:+6.2f}deg  dist={d:6.1f}mm")

    # Agrupar rumbos parecidos
    grupos = []
    for b, d in postes:
        for g in grupos:
            if abs(g[0][0] - b) < 3.0:
                g.append((b, d)); break
        else:
            grupos.append([(b, d)])
    print("\n  postes consolidados:")
    rumbos = []
    for g in grupos:
        bm = sum(x[0] for x in g) / len(g)
        dm = sum(x[1] for x in g) / len(g)
        rumbos.append(bm)
        print(f"    rumbo={bm:+6.2f}deg  dist={dm:6.1f}mm  (n={len(g)})")

    # ---------- Ajuste cx -> rumbo ----------
    # Emparejar: en la imagen, cx crece hacia la derecha; el rumbo tambien.
    if len(cam) == 2 and len(rumbos) >= 2:
        rumbos.sort()
        pares = []
        orden_cam = sorted(cam.items(), key=lambda kv: kv[1])  # menor cx = mas a la izq
        for (nombre, cx), brg in zip(orden_cam, rumbos):
            pares.append((nombre, cx, brg))
        print("\n=== EMPAREJAMIENTO ===")
        for nombre, cx, brg in pares:
            print(f"  {nombre:6s} cx={cx:6.1f} px  <->  rumbo={brg:+6.2f}deg")

        (n1, cx1, b1), (n2, cx2, b2) = pares[0], pares[1]
        t1, t2 = math.tan(math.radians(b1)), math.tan(math.radians(b2))
        # cx = c0 + f*tan(rumbo)  ->  dos ecuaciones, dos incognitas
        f = (cx2 - cx1) / (t2 - t1)
        c0 = cx1 - f * t1
        hfov = 2 * math.degrees(math.atan((ANCHO / 2) / f))
        print(f"\n=== MODELO cx = c0 + f*tan(rumbo) ===")
        print(f"  centro optico c0 = {c0:6.1f} px  (centro geometrico = {ANCHO/2:.0f})")
        print(f"  focal        f   = {f:6.1f} px")
        print(f"  HFOV efectivo    = {hfov:5.1f} deg")
        print(f"\n  inversa:  rumbo = atan((cx - {c0:.1f}) / {f:.1f})")
else:
    print("  (sin clusters de poste al frente)")
