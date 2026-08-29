#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ayuda para colocar los dos pilares antes de correr calib_fov.py.

calib_fov necesita que la camara vea los DOS a la vez y que esten bien
separados en rumbo. Son dos condiciones que tiran en sentidos opuestos:
mas separacion condiciona mejor el ajuste, pero pasado el borde del
cuadro el pilar simplemente desaparece. Con HFOV 74.5 el limite duro es
+-37 grados, asi que la ventana util para cada pilar es +-25 a +-33.

Imprime en vivo el estado de cada color para poder moverlos hasta que
los dos digan OK. Robot QUIETO.
"""
import math
import signal
import threading
import time

import cv2

import optica
import vision
from camara_driver import CamaraDriver
from lidar_driver import LidarDriver
from lidar_geometria import ProcesadorLidar, centroide_xy_cluster

LIMITE_CUADRO = optica.HFOV_EFECTIVO / 2.0     # borde real del frame
OBJETIVO_MIN = 20.0                            # rumbo minimo deseable por pilar
BASE_MINIMA = 25.0                             # separacion minima entre los dos

corriendo = True
ultimo_frame = [None]
ultimo_scan = [None]


def parar(*_):
    global corriendo
    corriendo = False


signal.signal(signal.SIGINT, parar)


def blobs(frame):
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
            if a > mejor_area:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    mejor, mejor_area = M["m10"] / M["m00"], a
        if mejor is not None:
            out[nombre] = (mejor, mejor_area)
    return out


camara = CamaraDriver()
threading.Thread(target=camara.hilo_captura,
                 args=(lambda: corriendo, lambda f: ultimo_frame.__setitem__(0, f)),
                 daemon=True).start()
time.sleep(2.5)
lidar = LidarDriver()
geo = ProcesadorLidar()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, lambda s: ultimo_scan.__setitem__(0, s)),
                 daemon=True).start()
time.sleep(2.0)

print("\nMueve los pilares hasta que los DOS digan OK. Ctrl-C para parar.")
print("  borde del cuadro: +-%.0f grados   |   area minima: %d\n"
      % (LIMITE_CUADRO, vision.AREA_MIN_DETECCION))

try:
    while corriendo:
        time.sleep(0.5)
        frame, scan = ultimo_frame[0], ultimo_scan[0]
        if frame is None or scan is None:
            continue

        bl = blobs(frame)
        med = geo.procesar(scan)
        cl = []
        for c in med.clusters_obstaculo:
            x, y = centroide_xy_cluster(c)
            if y > 200:
                cl.append((optica.rumbo_camara_de_cluster(x, y), math.hypot(x, y)))
        cl.sort()

        linea = []
        for color in ("ROJO", "VERDE"):
            if color not in bl:
                linea.append("%-6s NO SE VE            " % color)
                continue
            cx, area = bl[color]
            rumbo = optica.rumbo_de_cx(cx)
            if area < vision.AREA_MIN_DETECCION:
                estado = "area %d < %d" % (area, vision.AREA_MIN_DETECCION)
            elif abs(rumbo) > LIMITE_CUADRO - 3:
                estado = "AL BORDE, metelo"
            elif abs(rumbo) < OBJETIVO_MIN:
                estado = "muy centrado, abrelo"
            else:
                estado = "OK"
            linea.append("%-6s %+6.1fdeg a=%5d %-20s" % (color, rumbo, area, estado))

        base = ""
        if len(bl) == 2:
            r = [optica.rumbo_de_cx(bl[c][0]) for c in ("ROJO", "VERDE")]
            sep = abs(r[0] - r[1])
            base = "| base %5.1fdeg %s" % (sep, "OK" if sep >= BASE_MINIMA else "CORTA")
        print("  " + " ".join(linea) + " " + base + "  | LiDAR: %d clusters" % len(cl))
except KeyboardInterrupt:
    pass
finally:
    corriendo = False
    time.sleep(0.4)
    lidar.cerrar()
    camara.cerrar()
    print("\n[*] Parado.")
