#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Comprueba si la linea naranja de la pista dispara la mascara ROJA de
vision.py. Usa los umbrales reales del modulo, con la rotacion correcta.
"""
import time

import cv2
import numpy as np
from picamera2 import Picamera2

import vision

SALIDA = "/home/pi/diag_color"
ANCHO, ALTO = 320, 240

picam = Picamera2()
picam.configure(picam.create_video_configuration(
    main={"size": (ANCHO, ALTO), "format": "RGB888"},
    controls={"ScalerCrop": (0, 0, 4608, 2592)}))
picam.start()
time.sleep(2.5)

frame = cv2.rotate(picam.capture_array()[:, :, :3], cv2.ROTATE_180)
picam.stop()

cv2.imwrite(f"{SALIDA}/naranja_escena.jpg", frame)
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

mask_r = (cv2.inRange(hsv, vision.ROJO_BAJO_1, vision.ROJO_ALTO_1) |
          cv2.inRange(hsv, vision.ROJO_BAJO_2, vision.ROJO_ALTO_2))
mask_v = cv2.inRange(hsv, vision.VERDE_BAJO, vision.VERDE_ALTO)
mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, vision._KERNEL)
mask_v = cv2.morphologyEx(mask_v, cv2.MORPH_OPEN, vision._KERNEL)
cv2.imwrite(f"{SALIDA}/naranja_mask_rojo.jpg", mask_r)

print(f"pixeles en mascara ROJA  = {int(mask_r.sum()//255)}")
print(f"pixeles en mascara VERDE = {int(mask_v.sum()//255)}")
print()

for nombre, mask in (("ROJO", mask_r), ("VERDE", mask_v)):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    grandes = [c for c in cnts if cv2.contourArea(c) > vision.AREA_MIN_DETECCION]
    print(f"--- {nombre}: {len(cnts)} contornos, {len(grandes)} sobre AREA_MIN ({vision.AREA_MIN_DETECCION}) ---")
    for c in sorted(grandes, key=cv2.contourArea, reverse=True)[:4]:
        area = cv2.contourArea(c)
        x, y, w, h = cv2.boundingRect(c)
        cy = y + h // 2
        f_alto = cy < 180
        f_esbelto = h > (w * 0.7)
        veredicto = "ACEPTADO como poste" if (f_alto and f_esbelto) else "rechazado"
        print(f"    area={area:7.0f} bbox=({x:3d},{y:3d},{w:3d}x{h:3d}) cy={cy:3d} "
              f"h/w={h/w:5.2f} | cy<180:{'SI' if f_alto else 'NO'} "
              f"h>0.7w:{'SI' if f_esbelto else 'NO'} -> {veredicto}")

# Que tono tiene realmente lo que cae en la mascara roja
if mask_r.sum() > 0:
    tonos = hsv[:, :, 0][mask_r > 0]
    sats = hsv[:, :, 1][mask_r > 0]
    print()
    print(f"tono (H) de los pixeles rojos: min={tonos.min()} max={tonos.max()} "
          f"mediana={int(np.median(tonos))}")
    print(f"saturacion (S):                min={sats.min()} max={sats.max()} "
          f"mediana={int(np.median(sats))}")
    print("  (un pilar rojo real da H cerca de 0 o de 179; el naranja de pista, H entre 5 y 20)")
