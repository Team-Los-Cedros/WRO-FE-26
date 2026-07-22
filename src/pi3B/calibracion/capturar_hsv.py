"""
Herramienta de diagnóstico HSV sin GUI.

A diferencia de calibrar_hsv.py (que necesita una laptop con pantalla
recibiendo el streaming), este script corre 100% en la Raspberry Pi y
simplemente GUARDA archivos de imagen (frame crudo + máscara roja +
máscara verde) usando los umbrales HSV actuales de vision.py.

Uso:
    python3 capturar_hsv.py [num_capturas] [intervalo_segundos]

Ejemplo (5 capturas, una cada 3 segundos -- tiempo para reposicionar
el robot frente a cada pilar):
    python3 capturar_hsv.py 5 3

Los archivos quedan en ./hsv_captures/ con timestamp en el nombre.
Copialos a tu laptop (por ejemplo con scp) y compártelos para que se
puedan ajustar los rangos HSV con evidencia real de la iluminación
de tu pista.
"""
import sys
import os
import time
from datetime import datetime

import numpy as np
import cv2
from picamera2 import Picamera2

ANCHO_FRAME = 320
ALTO_FRAME = 240

# Deben coincidir con los valores vigentes en vision.py
ROJO_BAJO_1 = np.array([0, 151, 99]);   ROJO_ALTO_1 = np.array([15, 255, 255])
ROJO_BAJO_2 = np.array([158, 160, 82]); ROJO_ALTO_2 = np.array([179, 255, 255])
VERDE_BAJO = np.array([43, 68, 50]);    VERDE_ALTO = np.array([85, 255, 255])

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hsv_captures")


def main():
    num_capturas = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    intervalo = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    os.makedirs(OUT_DIR, exist_ok=True)

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (ANCHO_FRAME, ALTO_FRAME), "format": "RGB888"},
        controls={"ScalerCrop": (0, 0, 4608, 2592)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)
    print("[+] Camara inicializada.")

    kernel = np.ones((5, 5), np.uint8)

    for i in range(num_capturas):
        print(f"\n[{i+1}/{num_capturas}] Capturando en {intervalo:.0f}s... "
              f"apunta el robot al pilar que quieras evaluar.")
        time.sleep(intervalo)

        frame = picam2.capture_array()
        # Picamera2 "RGB888" entrega bytes en orden BGR (quirk conocido)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_rojo = cv2.inRange(hsv, ROJO_BAJO_1, ROJO_ALTO_1) | \
                    cv2.inRange(hsv, ROJO_BAJO_2, ROJO_ALTO_2)
        mask_verde = cv2.inRange(hsv, VERDE_BAJO, VERDE_ALTO)

        mask_rojo = cv2.morphologyEx(mask_rojo, cv2.MORPH_OPEN, kernel)
        mask_verde = cv2.morphologyEx(mask_verde, cv2.MORPH_OPEN, kernel)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        cv2.imwrite(os.path.join(OUT_DIR, f"{ts}_frame.png"), frame)
        cv2.imwrite(os.path.join(OUT_DIR, f"{ts}_mask_rojo.png"), mask_rojo)
        cv2.imwrite(os.path.join(OUT_DIR, f"{ts}_mask_verde.png"), mask_verde)
        print(f"    Guardado: {ts}_frame.png / _mask_rojo.png / _mask_verde.png")

    picam2.stop()
    print(f"\n[OK] {num_capturas} capturas guardadas en {OUT_DIR}")
    print("Copialas a tu laptop, por ejemplo:")
    print(f'  scp pi@192.168.0.107:{OUT_DIR}/*.png "C:\\Users\\usuaio\\Downloads\\hsv_captures\\"')


if __name__ == "__main__":
    main()
