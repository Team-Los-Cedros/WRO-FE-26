#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mide donde cae realmente el magenta de los delimitadores del cajon en el
espacio HSV, y si invade las mascaras de rojo/verde de vision.py.

El magenta puro RGB(255,0,255) es tono 150 en OpenCV y el segundo rango
de rojo empieza en 158: solo 8 unidades de margen. Este script dice si
ese margen sobrevive a la iluminacion real.

No mueve el robot.
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

cv2.imwrite(f"{SALIDA}/magenta_escena.jpg", frame)
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

# --- 1. Contaminacion de las mascaras existentes ---
mask_r = (cv2.inRange(hsv, vision.ROJO_BAJO_1, vision.ROJO_ALTO_1) |
          cv2.inRange(hsv, vision.ROJO_BAJO_2, vision.ROJO_ALTO_2))
mask_v = cv2.inRange(hsv, vision.VERDE_BAJO, vision.VERDE_ALTO)
mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, vision._KERNEL)
mask_v = cv2.morphologyEx(mask_v, cv2.MORPH_OPEN, vision._KERNEL)
cv2.imwrite(f"{SALIDA}/magenta_mask_rojo.jpg", mask_r)

print("--- mascaras actuales de vision.py con el magenta en el encuadre ---")
for nombre, m in (("ROJO", mask_r), ("VERDE", mask_v)):
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    grandes = [c for c in cnts if cv2.contourArea(c) > vision.AREA_MIN_DETECCION]
    print(f"  {nombre}: {int(m.sum()//255):6d} px, {len(grandes)} contornos sobre area minima")
    for c in sorted(grandes, key=cv2.contourArea, reverse=True)[:3]:
        x, y, w, h = cv2.boundingRect(c)
        cy = y + h // 2
        pasa = (cy < 180 and h > (w * 0.7))
        print(f"      area={cv2.contourArea(c):7.0f} bbox=({x},{y},{w}x{h}) "
              f"h/w={h/w:.2f} -> {'ACEPTADO COMO POSTE' if pasa else 'rechazado por geometria'}")

# --- 2. Donde cae el color saturado de la escena ---
sat = (S > 90) & (V > 60)
print(f"\n--- histograma de tono de los pixeles saturados (S>90, V>60): {int(sat.sum())} px ---")
tonos = H[sat]
if tonos.size:
    hist, _ = np.histogram(tonos, bins=36, range=(0, 180))
    for i, n in enumerate(hist):
        if n > 0:
            lo, hi = i * 5, i * 5 + 4
            marca = ""
            if 0 <= lo <= 15 or 158 <= hi <= 179:
                marca = "  <-- dentro del rango ROJO de vision.py"
            elif 43 <= lo <= 85:
                marca = "  <-- dentro del rango VERDE"
            print(f"  H {lo:3d}-{hi:3d}: {n:6d} px {'#' * min(60, n // 40)}{marca}")

# --- 3. Estadistica del pico magenta (tono 130-165) ---
zona = sat & (H >= 130) & (H <= 165)
print(f"\n--- pico magenta (tono 130-165): {int(zona.sum())} px ---")
if zona.sum() > 50:
    h_m, s_m, v_m = H[zona], S[zona], V[zona]
    for nom, arr in (("tono H", h_m), ("saturacion S", s_m), ("valor V", v_m)):
        p = np.percentile(arr, [1, 50, 99])
        print(f"  {nom:14s} min={arr.min():3d} p1={p[0]:5.0f} mediana={p[1]:5.0f} "
              f"p99={p[2]:5.0f} max={arr.max():3d}")
    solapa = int((zona & (H >= 158)).sum())
    print(f"\n  pixeles magenta con H>=158 (invaden ROJO_BAJO_2): {solapa} "
          f"({100.0*solapa/zona.sum():.1f}% del magenta)")
else:
    print("  (no se ve magenta: comprueba que los delimitadores esten en el encuadre)")
