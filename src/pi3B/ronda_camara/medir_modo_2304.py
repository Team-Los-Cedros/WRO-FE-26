#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Que gana y que cuesta pedirle a la camara el modo 2304x1296.

No necesita la pista montada. El FOV horizontal depende solo de cuanto
ancho de sensor sobrevive al ScalerCrop que libcamera aplica en cada
configuracion, y ese rectangulo se lee de la metadata del propio frame.
Con el HFOV ya medido contra una esquina real (53.8 grados en la
configuracion actual) queda fijada la focal EN PIXELES DE SENSOR, y de
ahi sale el HFOV de cualquier otro modo por geometria pura.

Mide ademas los fps sostenidos, que en una Pi 3B son la otra mitad de la
decision: un frame de 2304x1296 son 9 MB por captura y el pipeline de
vision.py hace conversion a HSV mas dos morfologias sobre el.
"""
import math, time
from picamera2 import Picamera2

HFOV_MEDIDO = 53.8      # medido con medir_fov.py en la config ACTUAL

CONFIGS = [
    ("ACTUAL  main 320x240 (4:3)", dict(main={"size": (320, 240), "format": "RGB888"},
                                        controls={"ScalerCrop": (0, 0, 4608, 2592)})),
    ("main 2304x1296 (16:9)",      dict(main={"size": (2304, 1296), "format": "RGB888"})),
    ("raw 2304x1296 + main 384x216 (16:9)",
                                   dict(main={"size": (384, 216), "format": "RGB888"},
                                        raw={"size": (2304, 1296)})),
    ("raw 2304x1296 + main 320x240 (4:3)",
                                   dict(main={"size": (320, 240), "format": "RGB888"},
                                        raw={"size": (2304, 1296)})),
]

resultados = []
for etiqueta, kw in CONFIGS:
    try:
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(**kw))
        cam.start(); time.sleep(1.5)
        md = cam.capture_metadata()
        f = cam.capture_array()
        n, t0 = 10, time.time()
        for _ in range(n):
            cam.capture_array()
        fps = n / (time.time() - t0)
        crop = md.get("ScalerCrop")
        cam.stop(); cam.close()
    except Exception as e:
        print(f"{etiqueta}: [-] fallo: {e}")
        continue
    h, w = f.shape[:2]
    resultados.append((etiqueta, w, h, crop, fps))

if not resultados:
    raise SystemExit("[-] ninguna configuracion arranco")

# La primera es la de referencia: su ancho de recorte corresponde al HFOV medido
_, _, _, crop_ref, _ = resultados[0]
ancho_ref = crop_ref[2]
focal_sensor_px = (ancho_ref / 2.0) / math.tan(math.radians(HFOV_MEDIDO / 2.0))
print(f"focal del sensor deducida del HFOV medido: {focal_sensor_px:.0f} px de sensor\n")

print(f"{'configuracion':38s} {'frame':>11s} {'ScalerCrop (sensor)':>26s} {'HFOV':>7s} {'fps':>7s}")
for etiqueta, w, h, crop, fps in resultados:
    hfov = 2.0 * math.degrees(math.atan((crop[2] / 2.0) / focal_sensor_px))
    print(f"{etiqueta:38s} {w:5d}x{h:<5d} {str(crop):>26s} {hfov:6.1f} {fps:7.1f}")

print("\nfocal en pixeles del FRAME (lo que necesita optica.py):")
for etiqueta, w, h, crop, fps in resultados:
    hfov = 2.0 * math.degrees(math.atan((crop[2] / 2.0) / focal_sensor_px))
    print(f"  {etiqueta:38s} f = {(w/2.0)/math.tan(math.radians(hfov/2.0)):7.1f} px")
