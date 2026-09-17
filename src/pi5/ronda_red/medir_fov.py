#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mide el FOV horizontal EFECTIVO de la camara tal y como la configura
camara_driver.py, sin pilares y sin cinta metrica: usa una esquina de la
pista como referencia, con el LiDAR dando su rumbo real.

Por que hace falta: navegacion.py convierte pixel->rumbo con
HFOV_CAMARA = 102 grados, el dato de CATALOGO de la Module 3 Wide. Pero
el FOV que importa no es el del catalogo, es el que sobrevive a la
configuracion: el sensor entrega 1536x864 (16:9) y camara_driver pide un
frame 4:3, asi que libcamera recorta los lados antes de escalar. Lo que
llega a vision.py es mas estrecho que la hoja de datos, y el modelo de
rumbo no lo sabe.

Metodo: el borde vertical negro/blanco mas marcado de la mitad izquierda
(o derecha) de la imagen es la esquina que el LiDAR ve como salto de
distancia. Con el rumbo real (LiDAR) y la columna medida (camara) sale
la focal directamente:  f = (col - c0) / tan(rumbo).

Robot QUIETO.
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
# Se mide con un frame grande para localizar el borde con precision, pero
# con EL MISMO recorte de sensor que usa camara_driver (raw 2304x1296 +
# salida 16:9). Asi la focal medida aqui se traslada al frame de carrera
# con solo escalar por la anchura.
ANCHO, ALTO = 1920, 1080
MODO_SENSOR = (2304, 1296)

RUMBO_MASTIL_DEG = 176.0     # medido con mapa_oclusion.py
DIST_MASTIL_MM   = 100.0
CAM_X = DIST_MASTIL_MM * math.sin(math.radians(RUMBO_MASTIL_DEG))
CAM_Y = DIST_MASTIL_MM * math.cos(math.radians(RUMBO_MASTIL_DEG))

SALTO_ESQUINA = 400.0
SECTOR = 60

# ---------- LiDAR ----------
corriendo = True
barridos = []
lidar = LidarDriver()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, lambda s: barridos.append(s) if len(barridos) < 10 else None),
                 daemon=True).start()
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

# ---------- Camara ----------
picam = Picamera2()
picam.configure(picam.create_video_configuration(
    main={"size": (ANCHO, ALTO), "format": "RGB888"},
    raw={"size": MODO_SENSOR}))
picam.start()
time.sleep(2.0)
frame = picam.capture_array()[:, :, :3].copy()
picam.stop()

gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# El suelo de la pista es blanco (V~150) y los muros negros (V~50):
# un umbral por Otsu separa los dos sin numeros magicos.
_, binaria = cv2.threshold(gris, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
oscuro = binaria == 0

# Banda de filas donde el bloque negro cercano es solido y el muro del
# fondo aun no lo ha alcanzado: se toma el tercio superior sin las 40
# primeras filas (que son muro de fondo de lado a lado).
FILA_INI, FILA_FIN = int(ALTO*0.16), int(ALTO*0.34)
banda = oscuro[FILA_INI:FILA_FIN, :]
cols_oscuras = banda.mean(axis=0)

# Borde derecho del bloque de la izquierda: ultima columna de la mitad
# izquierda que sigue mayoritariamente oscura.
col_borde_izq = None
for c in range(ANCHO // 2, -1, -1):
    if cols_oscuras[c] > 0.6:
        col_borde_izq = c
        break
# Borde izquierdo del bloque de la derecha
col_borde_der = None
for c in range(ANCHO // 2, ANCHO):
    if cols_oscuras[c] > 0.6:
        col_borde_der = c
        break

print(f"=== Bordes medidos en la imagen (filas {FILA_INI}-{FILA_FIN}) ===")
print(f"  borde del bloque IZQUIERDO : columna {col_borde_izq}")
print(f"  borde del bloque DERECHO   : columna {col_borde_der}")

# ---------- Esquinas del LiDAR ----------
grados = [(g % 360) for g in range(-SECTOR, SECTOR + 1)]
esquinas = []
for k in range(len(grados) - 1):
    a, b = grados[k], grados[k + 1]
    da, db = perfil[a], perfil[b]
    if da >= 8000.0 or db >= 8000.0 or abs(db - da) < SALTO_ESQUINA:
        continue
    g_borde, d_borde = (a, da) if da < db else (b, db)
    rad = math.radians(g_borde)
    x, y = d_borde * math.sin(rad) - CAM_X, d_borde * math.cos(rad) - CAM_Y
    esquinas.append((g_borde, d_borde, math.degrees(math.atan2(x, y)), abs(db - da)))

print("\n=== Esquinas del LiDAR trasladadas al origen de la camara ===")
for g, d, rumbo, salto in esquinas:
    print(f"  grado {g:3d}  d={d:6.0f}mm  salto={salto:6.0f}mm  rumbo_camara={rumbo:+6.1f}deg")

print("\n=== Focal implicada por cada emparejamiento borde<->esquina ===")
c0 = ANCHO / 2.0
for col, lado in ((col_borde_izq, "IZQUIERDO"), (col_borde_der, "DERECHO")):
    if col is None:
        continue
    for g, d, rumbo, _ in esquinas:
        if (rumbo < 0) != (col < c0):
            continue
        t = math.tan(math.radians(rumbo))
        if abs(t) < 1e-6:
            continue
        f_px = (col - c0) / t
        if f_px <= 0:
            continue
        hfov = 2.0 * math.degrees(math.atan((ANCHO / 2.0) / f_px))
        f_carrera = f_px * (640.0 / ANCHO)
        print(f"  borde {lado} col={col:4d}  <->  esquina grado {g:3d} "
              f"({rumbo:+.1f}deg)   f={f_px:7.1f}px   HFOV = {hfov:5.1f} grados"
              f"   -> f a 640px = {f_carrera:.1f}")

print("\n=== Referencia: que HFOV efectivo predice cada catalogo ===")
# El sensor entrega 16:9 y se pide 4:3 -> se recorta el 25% del ancho.
for nombre, hfov_cat in (("Module 3 estandar", 66.0), ("Module 3 Wide", 102.0)):
    ef = 2.0 * math.degrees(math.atan(0.75 * math.tan(math.radians(hfov_cat / 2.0))))
    print(f"  {nombre:20s} catalogo {hfov_cat:5.1f}  ->  {ef:5.1f} grados en un frame 4:3")

anot = frame.copy()
cv2.rectangle(anot, (0, FILA_INI), (ANCHO - 1, FILA_FIN), (255, 255, 0), 2)
for col, color in ((col_borde_izq, (0, 255, 0)), (col_borde_der, (255, 0, 255))):
    if col is not None:
        cv2.line(anot, (col, 0), (col, ALTO), color, 3)
cv2.imwrite(f"{DIR}/6_bordes.jpg", anot)
print(f"\n[+] {DIR}/6_bordes.jpg")
