#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Comprueba el modelo cx -> rumbo de navegacion.py SIN necesitar pilares,
usando las esquinas de la pista como puntos de referencia.

navegacion.py convierte pixel a rumbo con HFOV_CAMARA = 102 grados, que
es el dato de catalogo de la Camera Module 3 WIDE, anotado como "sin
calibrar" (calib_fov.py existe pero nunca se corrio). Si la camara
montada es la Module 3 estandar (66 grados) el rumbo sale ~1.6x
exagerado y el apareo por rumbo empareja el color con el poste
equivocado -- que es justo el fallo que ese apareo venia a arreglar.

El LiDAR da la verdad: una discontinuidad grande de distancia entre dos
grados seguidos es una esquina fisica, y su rumbo se conoce con
exactitud. Este script busca esas esquinas, las traslada al origen de la
CAMARA (que va montada ~100mm detras del LiDAR, en el mastil) y dibuja
sobre la foto donde deberia caer cada una segun cada hipotesis de FOV.
La que coincida con la esquina visible es la buena.

Robot QUIETO: no manda consignas. Deja la imagen anotada en
capturas_mastil/5_superpuesto.jpg
"""
import math
import os
import threading
import time

import cv2
import numpy as np
from picamera2 import Picamera2

from lidar_driver import LidarDriver
from lidar_geometria import construir_perfil_360

DIR = "capturas_mastil"
os.makedirs(DIR, exist_ok=True)
ANCHO, ALTO = 1280, 960

# Posicion de la camara respecto del LiDAR, deducida del propio barrido:
# mapa_oclusion.py situa el mastil en el rumbo 176 grados a ~100mm, o sea
# casi en linea recta por detras del sensor.
RUMBO_MASTIL_DEG = 176.0
DIST_MASTIL_MM   = 100.0
CAM_X = DIST_MASTIL_MM * math.sin(math.radians(RUMBO_MASTIL_DEG))
CAM_Y = DIST_MASTIL_MM * math.cos(math.radians(RUMBO_MASTIL_DEG))

HIPOTESIS = [
    ("Module 3 WIDE  (HFOV 102, el que usa navegacion.py)", 102.0, (0, 0, 255)),
    ("Module 3 estandar (HFOV 66)",                          66.0, (0, 200, 0)),
]

SECTOR = 70          # grados a cada lado del frente donde buscar esquinas
SALTO_ESQUINA = 250.0  # mm de discontinuidad para dar un grado por esquina

# ---------- LiDAR ----------
corriendo = True
barridos = []


def al_barrido(scan):
    if len(barridos) < 10:
        barridos.append(scan)


print("[*] LiDAR arrancando...")
lidar = LidarDriver()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, al_barrido), daemon=True).start()
t0 = time.time()
while len(barridos) < 10 and time.time() - t0 < 40:
    time.sleep(0.1)
corriendo = False
time.sleep(0.4)
lidar.cerrar()
if len(barridos) < 3:
    raise SystemExit("[-] Sin datos de LiDAR.")

acum = [[] for _ in range(360)]
for scan in barridos:
    p = construir_perfil_360(scan)
    for i in range(360):
        if p[i] < 8000.0:
            acum[i].append(p[i])
perfil = [8000.0] * 360
for i in range(360):
    if acum[i]:
        acum[i].sort()
        perfil[i] = acum[i][len(acum[i]) // 2]

# ---------- Camara (sin rotar: el mastil la dejo derecha) ----------
print("[*] Capturando frame...")
picam = Picamera2()
picam.configure(picam.create_video_configuration(
    main={"size": (ANCHO, ALTO), "format": "RGB888"},
    controls={"ScalerCrop": (0, 0, 4608, 2592)}))
picam.start()
time.sleep(2.0)
frame = picam.capture_array()[:, :, :3].copy()
picam.stop()

# ---------- Esquinas segun el LiDAR ----------
grados = [(g % 360) for g in range(-SECTOR, SECTOR + 1)]
esquinas = []
for k in range(len(grados) - 1):
    a, b = grados[k], grados[k + 1]
    da, db = perfil[a], perfil[b]
    if da >= 8000.0 or db >= 8000.0:
        continue
    if abs(db - da) < SALTO_ESQUINA:
        continue
    # El borde fisico esta al lado CERCANO del salto
    if da < db:
        g_borde, d_borde = a, da
    else:
        g_borde, d_borde = b, db
    esquinas.append((g_borde, d_borde, abs(db - da)))

# quedarse con las mas marcadas y no repetir esquinas contiguas
esquinas.sort(key=lambda e: -e[2])
elegidas = []
for g, d, salto in esquinas:
    if all(min(abs(g - g2), 360 - abs(g - g2)) > 5 for g2, _, _ in elegidas):
        elegidas.append((g, d, salto))
    if len(elegidas) == 4:
        break

print(f"\n=== Esquinas detectadas por el LiDAR (frontal +-{SECTOR} grados) ===")
if not elegidas:
    print("  ninguna: el robot no tiene esquinas marcadas en el campo de vision.")

anot = frame.copy()
cv2.line(anot, (ANCHO // 2, 0), (ANCHO // 2, ALTO), (255, 255, 0), 1)

for g, d, salto in elegidas:
    rad = math.radians(g)
    x, y = d * math.sin(rad), d * math.cos(rad)
    # trasladar al origen de la camara
    xc, yc = x - CAM_X, y - CAM_Y
    rumbo_cam = math.degrees(math.atan2(xc, yc))
    rumbo_lidar = math.degrees(math.atan2(x, y))
    print(f"  grado {g:3d}: d={d:6.0f}mm  salto={salto:6.0f}mm  "
          f"rumbo_lidar={rumbo_lidar:+6.1f}  rumbo_camara={rumbo_cam:+6.1f}")
    for etiqueta, hfov, color in HIPOTESIS:
        f_px = (ANCHO / 2.0) / math.tan(math.radians(hfov / 2.0))
        col = int(round(ANCHO / 2.0 + f_px * math.tan(math.radians(rumbo_cam))))
        print(f"       {etiqueta:52s} -> columna {col}")
        if 0 <= col < ANCHO:
            cv2.line(anot, (col, 0), (col, ALTO), color, 3)
            cv2.putText(anot, f"{hfov:.0f}", (max(2, col - 18), 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

cv2.imwrite(f"{DIR}/5_superpuesto.jpg", anot)
print(f"\n[+] {DIR}/5_superpuesto.jpg  (rojo=HFOV 102, verde=HFOV 66, "
      "cian=centro optico)")
print("    La hipotesis correcta es la que cae sobre la esquina visible.")
