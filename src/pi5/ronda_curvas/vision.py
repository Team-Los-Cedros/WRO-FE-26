# Deteccion HSV de postes rojo/verde con histeresis de estabilizacion.
# No captura frames -- los recibe por callback desde camara_driver.py
# via procesar_frame(). Umbrales y logica de contorno sin cambios.
import os
import threading
import time

import numpy as np
import cv2

# --- Umbrales reescalados al frame de 640x360 (ver camara_driver.py) ---
#
# Los dos numeros de aqui abajo estaban en pixeles de un frame 320x240
# sacado de un recorte de 3072 px de sensor. Con el modo 2304x1296 el
# frame es 640x360 desde el sensor entero, asi que el mismo objeto ocupa
# otra cantidad de pixeles y hay que reexpresarlos. Se reescalan por
# ANGULO, que es lo unico que no depende del modo:
#
#   horizontal: 315.4 -> 420.8 px/rad  (x1.334, el frame gana resolucion)
#   vertical  : 420.6 -> 420.8 px/rad  (x1.000, practicamente igual --
#              el frame viejo era anamorfico, estirado 1.33x en vertical,
#              y el nuevo tiene pixeles cuadrados)
#
# El area escala con el producto de los dos: 350 * 1.334 = 467.
AREA_MIN_DETECCION = 467

# Antes era `cy < 180` sobre 240 filas: 60 px por debajo del centro
# optico, o sea 8.1 grados. Sobre 360 filas con la focal nueva, esos
# mismos 8.1 grados caen 60 px por debajo del centro (180), o sea 240.
#
# Medido con el barrido del pilar en la config vieja: este filtro casi
# nunca llegaba a rechazar nada, porque al acercarse el blob se recorta
# contra el borde inferior y el centroide satura (4 muestras de 230
# tocaron el umbral). Con el VFOV nuevo (46.3 en vez de 31.9 grados) el
# pilar se recorta MENOS, asi que el filtro puede empezar a morder de
# verdad. Es lo primero que hay que volver a medir con test_pilar.py.
UMBRAL_CY = 240

# Depuracion opcional: guarda a disco el frame + mascara cada vez que
# color_crudo CAMBIA (entra, sale o cambia de color), para poder auditar
# despues un falso positivo que ocurrio en movimiento y que no se pudo
# reproducir con el robot quieto (ver sesion del dia 2: "ROJO" detectado
# sin ningun pilar en pista, solo durante un giro rapido). Apagado por
# defecto -- no agrega I/O a disco en carrera normal. Activar con
# WRO_DEBUG_VISION=1 antes de lanzar el script.
DEPURAR_FRAMES = os.environ.get("WRO_DEBUG_VISION") == "1"
_DIR_DEPURACION = "/home/pi/diag_vision"
_MAX_FRAMES_DEPURACION = 200  # limite duro, no llenar la SD en una corrida larga
_n_frames_guardados = 0
if DEPURAR_FRAMES:
    os.makedirs(_DIR_DEPURACION, exist_ok=True)
    print(f"[vision] WRO_DEBUG_VISION=1: guardando transiciones de color en {_DIR_DEPURACION}/")

CONFIRMACIONES_PARA_ENTRAR = 2
CONFIRMACIONES_PARA_SALIR  = 4

# Rangos HSV
ROJO_BAJO_1 = np.array([0,   151,  99]);  ROJO_ALTO_1 = np.array([15,  255, 255])
ROJO_BAJO_2 = np.array([158, 160,  82]);  ROJO_ALTO_2 = np.array([179, 255, 255])
VERDE_BAJO  = np.array([43,   68,  50]);  VERDE_ALTO  = np.array([85,  255, 255])

_KERNEL = np.ones((5, 5), np.uint8)

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


def get_deteccion():
    """Color y posición horizontal del poste, como (color, cx) en píxeles.

    `poste_cx` ya se calculaba pero no salía de este módulo, así que
    `navegacion.py` sabía QUÉ color hay delante pero no DÓNDE, y para
    aparearlo con el LiDAR usaba "el cluster más cercano" -- un criterio
    independiente del que eligió el color (el blob de mayor área). Con
    dos postes en el mismo frame los dos criterios pueden caer en postes
    distintos y el color acaba pegado a la posición equivocada, con la
    evasión saliendo hacia el lado contrario al que manda el reglamento.
    Devolviendo también el cx, el apareo se puede hacer por ángulo.

    cx va de 0 (borde izquierdo) a ANCHO_FRAME (derecho); es None cuando
    no hay detección estable.
    """
    with lock_vision:
        return poste_color, poste_cx


def _guardar_depuracion(frame, hsv, color_nuevo, area):
    # Solo se llama si DEPURAR_FRAMES esta activo. Guarda el frame crudo
    # (BGR real, tal como lo ve procesar_frame) y la mascara del color en
    # cuestion, para poder inspeccionar despues exactamente que disparo
    # la deteccion -- reflejo, objeto real, ruido de sensor, etc.
    global _n_frames_guardados
    if _n_frames_guardados >= _MAX_FRAMES_DEPURACION:
        return
    _n_frames_guardados += 1
    ts = time.strftime("%H%M%S")
    etiqueta = color_nuevo if color_nuevo else "NINGUNO"
    try:
        cv2.imwrite(f"{_DIR_DEPURACION}/{_n_frames_guardados:03d}_{ts}_{etiqueta}.jpg", frame)
        if color_nuevo:
            mask = (cv2.inRange(hsv, ROJO_BAJO_1, ROJO_ALTO_1) | cv2.inRange(hsv, ROJO_BAJO_2, ROJO_ALTO_2)
                    if color_nuevo == "ROJO" else cv2.inRange(hsv, VERDE_BAJO, VERDE_ALTO))
            cv2.imwrite(f"{_DIR_DEPURACION}/{_n_frames_guardados:03d}_{ts}_{etiqueta}_mask.jpg", mask)
        print(f"[vision] deteccion #{_n_frames_guardados}: {etiqueta} area={area} -> guardado")
    except Exception as e:
        print(f"[vision] fallo guardando frame de depuracion: {e}")


def procesar_frame(frame):
    """
    Procesa un frame (llamado por camara_driver.hilo_captura, uno por
    captura). Segmenta por color, elige el mejor contorno y actualiza el
    estado compartido con histéresis.
    """
    global color_crudo, cx_crudo, area_cruda

    try:
        # Picamera2 con formato "RGB888" entrega los bytes en orden BGR
        # (comportamiento documentado de la librería, pese al nombre).
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_rojo  = cv2.inRange(hsv, ROJO_BAJO_1, ROJO_ALTO_1) | \
                     cv2.inRange(hsv, ROJO_BAJO_2, ROJO_ALTO_2)
        mask_verde = cv2.inRange(hsv, VERDE_BAJO, VERDE_ALTO)

        mask_rojo  = cv2.morphologyEx(mask_rojo,  cv2.MORPH_OPEN, _KERNEL)
        mask_verde = cv2.morphologyEx(mask_verde, cv2.MORPH_OPEN, _KERNEL)

        cont_rojo,  _ = cv2.findContours(mask_rojo,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cont_verde, _ = cv2.findContours(mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mejor_color, mejor_cx, mejor_area = None, None, 0

        for c in cont_rojo:
            area = cv2.contourArea(c)
            if area > mejor_area and area > AREA_MIN_DETECCION:
                x, y, w, h = cv2.boundingRect(c)
                cy = y + h // 2
                if cy < UMBRAL_CY and h > (w * 0.7):
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
                if cy < UMBRAL_CY and h > (w * 0.7):
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        mejor_color = "VERDE"
                        mejor_cx    = int(M["m10"] / M["m00"])
                        mejor_area  = area

        with lock_vision:
            color_anterior = color_crudo
            if mejor_area > 0:
                color_crudo = mejor_color
                cx_crudo    = mejor_cx
                area_cruda  = mejor_area
            else:
                color_crudo = None
                cx_crudo    = None
                area_cruda  = 0
            _aplicar_histeresis()

            if DEPURAR_FRAMES and color_crudo != color_anterior:
                _guardar_depuracion(frame, hsv, color_crudo, area_cruda)

    except Exception as e:
        print(f"[-] Falla procesando frame de camara: {e}")
