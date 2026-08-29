#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Diagnostico de la camara en su montaje nuevo (mastil trasero, mas alto y
mas atras que antes). Responde tres cosas que cambian al mover la camara
y que no se pueden deducir del codigo:

  1. ORIENTACION. camara_driver.py rota 180 grados porque el modulo
     estaba montado invertido. Si al remontarlo quedo derecho, esa
     rotacion ahora ESTROPEA la imagen (espeja el cx y invierte el
     filtro cy<180 de vision.py). Se guardan las dos versiones para
     poder mirarlas.
  2. ENCUADRE. Desde mas atras y mas alto, parte del propio robot puede
     entrar en cuadro. Se mide cuanto ocupa el brillo/color propio en
     las filas de abajo.
  3. HORIZONTE. El filtro cy < 180 de vision.py se calibro con la
     camara vieja. Se informa a que altura de imagen caen los objetos
     detectados ahora, que es lo que decide si ese umbral sigue valiendo.

Guarda en ./capturas_mastil/. Robot QUIETO, no manda consignas.
"""
import os
import time

import cv2
import numpy as np
from picamera2 import Picamera2

import vision

DIR = "capturas_mastil"
os.makedirs(DIR, exist_ok=True)
ANCHO, ALTO = 320, 240

print("[*] Inicializando camara (misma config que camara_driver.py)...")
picam = Picamera2()
picam.configure(picam.create_video_configuration(
    main={"size": (ANCHO, ALTO), "format": "RGB888"},
    controls={"ScalerCrop": (0, 0, 4608, 2592)}))
picam.start()
time.sleep(2.0)

crudo = picam.capture_array()[:, :, :3]
rotado = cv2.rotate(crudo, cv2.ROTATE_180)

cv2.imwrite(f"{DIR}/1_crudo.jpg", crudo)
cv2.imwrite(f"{DIR}/2_rotado180.jpg", rotado)

# Version grande, para mirarla con detalle sin el downscale de 320x240
picam.stop()
picam.configure(picam.create_video_configuration(
    main={"size": (1280, 960), "format": "RGB888"},
    controls={"ScalerCrop": (0, 0, 4608, 2592)}))
picam.start()
time.sleep(1.5)
grande = picam.capture_array()[:, :, :3]
cv2.imwrite(f"{DIR}/3_grande_crudo.jpg", grande)
cv2.imwrite(f"{DIR}/4_grande_rotado.jpg", cv2.rotate(grande, cv2.ROTATE_180))
picam.stop()
print(f"[+] Capturas en {os.path.abspath(DIR)}/")


def informe(nombre, frame):
    print(f"\n=== {nombre} ===")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]

    # Brillo medio por bandas horizontales: el suelo de la pista es
    # blanco y muy brillante; una banda oscura abajo suele ser chasis.
    bandas = 6
    paso = h // bandas
    print("  brillo medio por banda (arriba -> abajo):")
    for b in range(bandas):
        y0, y1 = b * paso, (b + 1) * paso
        v = float(np.mean(hsv[y0:y1, :, 2]))
        s = float(np.mean(hsv[y0:y1, :, 1]))
        print(f"    filas {y0:4d}-{y1:4d}  V={v:6.1f}  S={s:6.1f}")

    # Blobs de color con los MISMOS umbrales que vision.py
    mask_r = (cv2.inRange(hsv, vision.ROJO_BAJO_1, vision.ROJO_ALTO_1) |
              cv2.inRange(hsv, vision.ROJO_BAJO_2, vision.ROJO_ALTO_2))
    mask_v = cv2.inRange(hsv, vision.VERDE_BAJO, vision.VERDE_ALTO)
    kernel = np.ones((5, 5), np.uint8)
    for etiqueta, mask in (("ROJO", mask_r), ("VERDE", mask_v)):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        conts = sorted(conts, key=cv2.contourArea, reverse=True)[:3]
        for c in conts:
            area = cv2.contourArea(c)
            if area < vision.AREA_MIN_DETECCION:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            cy = y + bh // 2
            pasa_cy = cy < 180 * (h / 240.0)
            pasa_forma = bh > bw * 0.7
            print(f"  blob {etiqueta}: area={area:7.0f} cx={x + bw // 2:4d} cy={cy:4d} "
                  f"(cy/alto={cy / h:.2f})  w={bw} h={bh}  "
                  f"filtro_cy={'PASA' if pasa_cy else 'FALLA'} "
                  f"filtro_forma={'PASA' if pasa_forma else 'FALLA'}")


informe("CRUDO (sin rotar)", crudo)
informe("ROTADO 180 (lo que vision.py recibe hoy)", rotado)
