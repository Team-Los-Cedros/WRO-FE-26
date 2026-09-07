#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Prueba en vivo camara + LiDAR con un pilar de verdad delante. Es lo unico
de esta carpeta que necesita una accion fisica: hay que ir acercando un
pilar rojo o verde al robot mientras corre.

Mide las tres cosas que el cambio de montaje deja sin validar y que no se
pueden deducir del codigo ni de una foto de la pista vacia:

  1. El filtro `cy < 180` de vision.py. Se calibro con la camara vieja,
     baja y adelantada. Desde un mastil alto el suelo ocupa mas cuadro y
     el centroide del pilar cae mas abajo al acercarse, asi que ese
     umbral decide a que distancia se PIERDE la deteccion. La columna
     cy/filtro dice a cual, emparejada con la distancia del LiDAR.
  2. Los umbrales HSV, con la iluminacion y el angulo nuevos: la columna
     area dice si el blob sobrevive a AREA_MIN_DETECCION.
  3. El apareo camara<->LiDAR. Imprime el rumbo del pilar segun el LiDAR
     (la verdad) y segun la camara con los dos modelos: el de
     navegacion.py (HFOV 102, de catalogo) y el de optica.py (medido).

No se apoya en vision.py para el blob porque vision.py no publica el cy
(solo color y cx), y el cy es justo lo que hay que medir. Reutiliza sus
umbrales HSV y su AREA_MIN_DETECCION para que lo medido sea lo mismo que
decide en carrera.

Uso: python3 test_pilar.py   y se acerca el pilar de ~1200mm a ~200mm.
Cortar con Ctrl-C. El robot NO se mueve: no instancia EnlacePico.
"""
import math
import signal
import threading
import time

import cv2
import numpy as np

import optica
import vision
from camara_driver import CamaraDriver
from lidar_driver import LidarDriver
from lidar_geometria import ProcesadorLidar, centroide_xy_cluster

ALTO_FRAME = 360
UMBRAL_CY = vision.UMBRAL_CY     # el de vision.py, reescalado a 640x360

# Modelo viejo, para comparar: catalogo Wide sobre el frame viejo
HFOV_VIEJO = 102.0
FOCAL_VIEJA = (320.0 / 2.0) / math.tan(math.radians(HFOV_VIEJO / 2.0))
TOLERANCIA_VIEJA = 20.0

corriendo = True
ultimo_scan = [None]
ultimo_frame = [None]


def parar(*_):
    global corriendo
    corriendo = False


signal.signal(signal.SIGINT, parar)


def blob_mas_grande(frame):
    """cx, cy, area y color del mayor blob rojo/verde, SIN filtrar por cy.

    Se salta el filtro a proposito: el objetivo es medir donde cae el
    centroide, no aplicar el umbral que esta en duda.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = {
        "ROJO":  cv2.inRange(hsv, vision.ROJO_BAJO_1, vision.ROJO_ALTO_1) |
                 cv2.inRange(hsv, vision.ROJO_BAJO_2, vision.ROJO_ALTO_2),
        "VERDE": cv2.inRange(hsv, vision.VERDE_BAJO, vision.VERDE_ALTO),
    }
    mejor = None
    for color, m in masks.items():
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, vision._KERNEL)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < vision.AREA_MIN_DETECCION:
                continue
            if mejor is not None and area <= mejor[3]:
                continue
            x, y, w, h = cv2.boundingRect(c)
            M = cv2.moments(c)
            if M["m00"] <= 0:
                continue
            mejor = (color, int(M["m10"] / M["m00"]), y + h // 2, area, w, h)
    return mejor


print("[*] Arrancando camara (sin rotacion, montaje de mastil)...")
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

print("\nAcerca el pilar despacio, de ~1200mm a ~200mm. Ctrl-C para parar.\n")
print(f"  color   cx   cy  area  cy<{UMBRAL_CY} | LiDAR dist  rumbo | nuevo({optica.HFOV_EFECTIVO:.0f}deg) err  viejo(102deg) err")
print("  " + "-" * 100)

try:
    while corriendo:
        time.sleep(0.35)
        scan, frame = ultimo_scan[0], ultimo_frame[0]
        if scan is None or frame is None:
            continue

        blob = blob_mas_grande(frame)

        med = geo.procesar(scan)
        mejor_cl = None
        for c in med.clusters_obstaculo:
            x, y = centroide_xy_cluster(c)
            if y <= 0:
                continue
            d = math.hypot(x, y)
            if mejor_cl is None or d < mejor_cl[0]:
                mejor_cl = (d, math.degrees(math.atan2(x, y)))

        if blob is None and mejor_cl is None:
            continue

        if blob is None:
            print(f"  {'(sin color)':<11s} {'':>17s} | "
                  f"{mejor_cl[0]:9.0f} {mejor_cl[1]:+6.1f} | el LiDAR lo ve, la camara no")
            continue

        color, cx, cy, area, w, h = blob
        pasa = "PASA" if cy < UMBRAL_CY else "PIERDE"
        r_nuevo = optica.rumbo_de_cx(cx)
        r_viejo = math.degrees(math.atan2(cx * 0.5 - 160.0, FOCAL_VIEJA))

        if mejor_cl is None:
            print(f"  {color:<7s} {cx:4d} {cy:4d} {area:5.0f}  {pasa:>6s} | "
                  f"        -      - | {r_nuevo:+6.1f}    -    {r_viejo:+6.1f}    -")
            continue

        d_lidar, r_lidar = mejor_cl
        e_n, e_v = r_nuevo - r_lidar, r_viejo - r_lidar
        m_n = "OK" if abs(e_n) <= optica.TOLERANCIA_APAREO_GRADOS else "NO"
        m_v = "OK" if abs(e_v) <= TOLERANCIA_VIEJA else "NO"
        print(f"  {color:<7s} {cx:4d} {cy:4d} {area:5.0f}  {pasa:>6s} | "
              f"{d_lidar:9.0f} {r_lidar:+6.1f} | {r_nuevo:+6.1f} {e_n:+6.1f} {m_n}  "
              f"{r_viejo:+6.1f} {e_v:+6.1f} {m_v}")
except KeyboardInterrupt:
    pass
finally:
    corriendo = False
    time.sleep(0.4)
    lidar.cerrar()
    camara.cerrar()
    print("\n[*] Parado.")
