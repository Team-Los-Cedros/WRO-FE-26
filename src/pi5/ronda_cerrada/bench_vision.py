"""Mide el coste del pipeline de vision a distintas resoluciones.

No abre camara ni Pico: procesa un frame real ya capturado, reescalado.
"""
import copy
import json
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, "/home/pi/wro_nueva_20260831_1435_fix_rumbo")
from ronda_nueva.config import cargar_configuracion
from ronda_nueva.vision_ligera import VisionLigera

RUTA_FRAME = sys.argv[1]
base = cargar_configuracion(
    "/home/pi/wro_nueva_20260831_1435_fix_rumbo/ronda_nueva/"
    "configuracion_prueba_pista_1435_fix_rumbo.json"
)
frame0 = cv2.imread(RUTA_FRAME)
if frame0 is None:
    raise SystemExit("no se pudo leer %s" % RUTA_FRAME)

MODOS = [(640, 360), (1280, 720), (2304, 1296)]
print("%-12s %8s %8s %9s %10s" % ("modo", "ms/frame", "fps_max", "vs 640x360", "detec."))
referencia = None
for w, h in MODOS:
    cfg = copy.deepcopy(base)
    cfg["camera"]["width"] = w
    cfg["camera"]["height"] = h
    # El HFOV no cambia con la resolucion, pero el centro optico si escala.
    cfg["camera"]["principal_x_px"] = base["camera"]["principal_x_px"] * w / 640.0
    vision = VisionLigera(cfg)
    frame = cv2.resize(frame0, (w, h), interpolation=cv2.INTER_AREA)

    vision.procesar(frame, 0.0)  # calentamiento
    n = 12
    t0 = time.monotonic()
    for i in range(n):
        res = vision.procesar(frame, float(i))
    ms = (time.monotonic() - t0) * 1000.0 / n
    if referencia is None:
        referencia = ms
    ndet = len(getattr(res, "detecciones", ()) or ())
    print("%-12s %8.1f %8.1f %8.1fx %10d" % (
        "%dx%d" % (w, h), ms, 1000.0 / ms, ms / referencia, ndet))

print()
print("presupuesto: a 15 fps hay 66.7 ms por frame, y el ciclo de control")
print("comparte CPU con LiDAR, fusion y control.")
