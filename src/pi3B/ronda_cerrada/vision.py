"""
CÁMARA — Detección HSV de postes rojo/verde (Pi Camera Module 3)

Hilo independiente que captura frames, aísla los bloques de color por
umbralización HSV y aplica histéresis para estabilizar la detección
(evita que un post detectado parpadee frame a frame por ruido).

No depende de ningún otro módulo del proyecto -- Close2_round.py solo
llama a `hilo_camara()` en un thread y lee el resultado con `get_color()`.
"""
import time
import threading

import numpy as np
import cv2
from picamera2 import Picamera2

ANCHO_FRAME = 320
ALTO_FRAME  = 240

AREA_MIN_DETECCION = 350

CONFIRMACIONES_PARA_ENTRAR = 2
CONFIRMACIONES_PARA_SALIR  = 4

# Rangos HSV
ROJO_BAJO_1 = np.array([0,   151,  99]);  ROJO_ALTO_1 = np.array([15,  255, 255])
ROJO_BAJO_2 = np.array([158, 160,  82]);  ROJO_ALTO_2 = np.array([179, 255, 255])
VERDE_BAJO  = np.array([43,   68,  50]);  VERDE_ALTO  = np.array([85,  255, 255])

# Estado compartido (protegido por lock_vision)
lock_vision  = threading.Lock()
color_crudo  = None
cx_crudo     = None
area_cruda   = 0

poste_color = None
poste_cx    = None
poste_area  = 0
_contador_entrada = 0
_contador_salida  = 0


def _aplicar_histeresis():
    global poste_color, poste_cx, poste_area
    global _contador_entrada, _contador_salida

    if poste_color is None:
        if color_crudo is not None:
            _contador_entrada += 1
            _contador_salida = 0
            if _contador_entrada >= CONFIRMACIONES_PARA_ENTRAR:
                poste_color = color_crudo
                poste_cx    = cx_crudo
                poste_area  = area_cruda
                _contador_entrada = 0
        else:
            _contador_entrada = 0
    else:
        if color_crudo == poste_color:
            poste_cx   = cx_crudo
            poste_area = area_cruda
            _contador_salida = 0
        else:
            _contador_salida += 1
            if _contador_salida >= CONFIRMACIONES_PARA_SALIR:
                poste_color = None
                poste_cx    = None
                poste_area  = 0
                _contador_salida  = 0
                _contador_entrada = 0


def get_color():
    """Lectura segura (con lock) del color detectado y estabilizado por histéresis."""
    with lock_vision:
        return poste_color


def hilo_camara(obtener_corriendo):
    """
    obtener_corriendo: función sin argumentos que devuelve True mientras
    el sistema deba seguir activo (equivalente al `corriendo` global del
    script principal).
    """
    global poste_color, poste_cx, poste_area
    global color_crudo, cx_crudo, area_cruda

    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (ANCHO_FRAME, ALTO_FRAME), "format": "RGB888"},
            controls={"ScalerCrop": (0, 0, 4608, 2592)}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(1.0)
        print("[+] Camara Pi Cam Module 3 inicializada (FOV Full-Sensor).")
    except Exception as e:
        print(f"[-] Error inicializando camara: {e}")
        return

    kernel = np.ones((5, 5), np.uint8)

    while obtener_corriendo():
        try:
            frame = picam2.capture_array()
            # Picamera2 con formato "RGB888" entrega los bytes en orden BGR
            # (comportamiento documentado de la librería, pese al nombre).
            hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask_rojo  = cv2.inRange(hsv, ROJO_BAJO_1, ROJO_ALTO_1) | \
                         cv2.inRange(hsv, ROJO_BAJO_2, ROJO_ALTO_2)
            mask_verde = cv2.inRange(hsv, VERDE_BAJO, VERDE_ALTO)

            mask_rojo  = cv2.morphologyEx(mask_rojo,  cv2.MORPH_OPEN, kernel)
            mask_verde = cv2.morphologyEx(mask_verde, cv2.MORPH_OPEN, kernel)

            cont_rojo,  _ = cv2.findContours(mask_rojo,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cont_verde, _ = cv2.findContours(mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            mejor_color, mejor_cx, mejor_area = None, None, 0

            for c in cont_rojo:
                area = cv2.contourArea(c)
                if area > mejor_area and area > AREA_MIN_DETECCION:
                    x, y, w, h = cv2.boundingRect(c)
                    cy = y + h // 2
                    if cy < 180 and h > (w * 0.7):
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            mejor_color = "ROJO"
                            mejor_cx    = int(M["m10"] / M["m00"])
                            mejor_area  = area

            for c in cont_verde:
                area = cv2.contourArea(c)
                if area > mejor_area and area > AREA_MIN_DETECCION:
                    x, y, w, h = cv2.boundingRect(c)
                    cy = y + h // 2
                    if cy < 180 and h > (w * 0.7):
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            mejor_color = "VERDE"
                            mejor_cx    = int(M["m10"] / M["m00"])
                            mejor_area  = area

            with lock_vision:
                if mejor_area > 0:
                    color_crudo = mejor_color
                    cx_crudo    = mejor_cx
                    area_cruda  = mejor_area
                else:
                    color_crudo = None
                    cx_crudo    = None
                    area_cruda  = 0
                _aplicar_histeresis()

        except Exception as e:
            print(f"[-] Falla en hilo camara: {e}")
            time.sleep(0.1)

        time.sleep(0.03)  # ~30 fps
