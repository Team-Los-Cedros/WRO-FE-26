#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Corre vision.py de verdad (mismos rangos HSV, misma rotacion, mismo
filtro geometrico) en vivo, sin motores, e imprime + guarda a disco
cada vez que detecta CUALQUIER color crudo (antes de la histeresis).
Sirve para reproducir y ver exactamente que disparo el falso ROJO de
la prueba de esquina.

NO instancia EnlacePico, NO manda ninguna consigna: el robot no se mueve.
"""
import os
import time

import cv2

import vision
from camara_driver import CamaraDriver

SALIDA = "/home/pi/diag_falso_rojo"
os.makedirs(SALIDA, exist_ok=True)

n_guardados = 0
ultimo_color = None

def al_frame(frame):
    global n_guardados, ultimo_color
    vision.procesar_frame(frame)
    color = vision.color_crudo
    if color is not None:
        marca = time.strftime("%H:%M:%S")
        print(f"[{marca}] color_crudo={color}  area={vision.area_cruda}  "
              f"cx={vision.cx_crudo}  (estable={vision.poste_color})")
        if color != ultimo_color:
            n_guardados += 1
            ts = time.strftime("%H%M%S")
            cv2.imwrite(f"{SALIDA}/frame_{ts}_{color}.jpg", frame)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = (cv2.inRange(hsv, vision.ROJO_BAJO_1, vision.ROJO_ALTO_1) |
                    cv2.inRange(hsv, vision.ROJO_BAJO_2, vision.ROJO_ALTO_2)
                    if color == "ROJO" else
                    cv2.inRange(hsv, vision.VERDE_BAJO, vision.VERDE_ALTO))
            cv2.imwrite(f"{SALIDA}/mask_{ts}_{color}.jpg", mask)
            print(f"    -> guardado frame_{ts}_{color}.jpg")
    ultimo_color = color

print("[*] Arrancando camara (sin motores, sin LiDAR)...")
camara = CamaraDriver()
corriendo = True
import threading
threading.Thread(target=camara.hilo_captura,
                 args=(lambda: corriendo, al_frame), daemon=True).start()
print("[*] Listo. Observando color_crudo en vivo. Ctrl+C para salir.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    corriendo = False
    print(f"\n[*] Detenido. {n_guardados} eventos de color guardados en {SALIDA}/")
