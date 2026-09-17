"""Deteccion de pilares con la red en el Hailo-8, en paralelo al HSV.

Corre EN SOMBRA: observa y anota, pero no manda nada sobre el control. La ronda
sigue decidiendo con `vision.py` exactamente igual que antes. El unico objetivo
es tener, sobre las mismas corridas, lo que vio cada detector, y poder decidir
con datos si la red esta lista para sustituir al HSV.

Se activa con WRO_RED=1. Sin esa variable este modulo ni se importa.

    WRO_RED=1 python3 ronda_camara.py
    WRO_RED=1 WRO_RED_HEF=~/otro.hef WRO_RED_HZ=5 python3 ronda_camara.py

Deja un CSV por corrida en logs/red_<fecha>.csv con una fila por inferencia:
las dos detecciones enfrentadas, para comparar despues.

La inferencia va en un hilo aparte con un solo hueco de cuadro: si la red no
llega, se pierde una muestra antes que retrasar el lazo de control, que es el
que hace 12 de 12.
"""

import os
import csv
import json

from dos_pilares import Cuadro, decodificar
import atexit
import threading
import time

import numpy as np

RUTA_HEF = os.path.expanduser(os.environ.get("WRO_RED_HEF", "~/pilares_v0.hef"))
HZ = float(os.environ.get("WRO_RED_HZ", "5"))
UMBRAL = float(os.environ.get("WRO_RED_UMBRAL", "0.35"))

# El orden de clases es el del entrenamiento: 0 rojo, 1 verde.
COLORES = ("ROJO", "VERDE")

_lock = threading.Lock()
_pendiente = None        # (cuadro, color_hsv, cx_hsv, area_hsv, t)
_ultima = None           # (color, cx, score, t)
_corriendo = False
_hilo = None
_fichero = None
_csv = None
_n = 0
_secuencia = 0
_cuadro_ultimo = None


def _preparar(cuadro):
    """Encaje con bandas a 640x640, el mismo que uso el entrenamiento.

    Devuelve tambien la escala y el desplazamiento para poder llevar las cajas
    de vuelta a las coordenadas del cuadro original.
    """
    import cv2
    alto, ancho = cuadro.shape[:2]
    e = 640.0 / max(ancho, alto)
    na, nl = int(round(ancho * e)), int(round(alto * e))
    chico = cv2.resize(cuadro, (na, nl))
    lienzo = np.full((640, 640, 3), 114, dtype=np.uint8)
    dx, dy = (640 - na) // 2, (640 - nl) // 2
    lienzo[dy:dy + nl, dx:dx + na] = chico
    # El sensor entrega BGR bajo el nombre RGB888; la red entreno con RGB.
    return np.ascontiguousarray(lienzo[:, :, ::-1]), e, dx, dy


def _mejor_deteccion(salida, e, dx):
    """De todas las cajas, la del pilar mas grande: el que importa es el de delante."""
    mejor = None
    for clase, cajas in enumerate(salida):
        c = np.array(cajas)
        if c.size == 0 or c.ndim != 2 or c.shape[1] < 5:
            continue
        for fila in c:
            y0, x0, y1, x1, score = fila[:5]
            if score < UMBRAL:
                continue
            alto = (y1 - y0) * 640.0
            if mejor is None or alto > mejor[3]:
                cx = ((x0 + x1) / 2.0 * 640.0 - dx) / e
                mejor = (COLORES[clase] if clase < len(COLORES) else str(clase),
                         cx, float(score), alto)
    return mejor


def _bucle():
    global _ultima, _n, _pendiente, _cuadro_ultimo
    from hailo_platform import (VDevice, HEF, ConfigureParams, HailoStreamInterface,
                                InputVStreamParams, OutputVStreamParams, InferVStreams,
                                FormatType)

    hef = HEF(RUTA_HEF)
    entrada = hef.get_input_vstream_infos()[0]
    salida = hef.get_output_vstream_infos()[0]
    espera = 1.0 / HZ if HZ > 0 else 0.0

    with VDevice() as dispositivo:
        params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        red = dispositivo.configure(hef, params)[0]
        ivp = InputVStreamParams.make(red, format_type=FormatType.UINT8)
        ovp = OutputVStreamParams.make(red, format_type=FormatType.FLOAT32)
        with red.activate(red.create_params()):
            with InferVStreams(red, ivp, ovp) as tuberia:
                print("[red] %s cargado, %.1f inferencias/s, umbral %.2f"
                      % (os.path.basename(RUTA_HEF), HZ, UMBRAL))
                while _corriendo:
                    t0 = time.time()
                    with _lock:
                        tarea = _pendiente
                        _pendiente = None
                    if tarea is None:
                        time.sleep(0.01)
                        continue

                    cuadro, color_hsv, cx_hsv, area_hsv, t_cuadro, t_mono, secuencia = tarea
                    try:
                        lienzo, e, dx, dy = _preparar(cuadro)
                        res = tuberia.infer({entrada.name: np.expand_dims(lienzo, 0)})
                        cajas = decodificar(res[salida.name][0], e, dx, dy,
                                             cuadro.shape[1], cuadro.shape[0], UMBRAL)
                        mayor = cajas[0] if cajas else None
                        det = (mayor.color, mayor.cx, mayor.score, mayor.alto) if mayor else None
                    except Exception as err:
                        print("[red] fallo en la inferencia: %s" % err)
                        det = None
                        cajas = ()

                    ahora = time.time()
                    with _lock:
                        _cuadro_ultimo = Cuadro(secuencia, t_mono, t_cuadro, cajas)
                        _ultima = (det[0], det[1], det[2], ahora) if det else (None, None, 0.0, ahora)

                    if _csv is not None:
                        _csv.writerow([
                            "%.3f" % t_cuadro,
                            "%.1f" % ((ahora - t_cuadro) * 1000),
                            color_hsv or "",
                            "" if cx_hsv is None else "%.1f" % cx_hsv,
                            area_hsv or 0,
                            det[0] if det else "",
                            "" if not det else "%.1f" % det[1],
                            "" if not det else "%.3f" % det[2],
                            "1" if (det and color_hsv and det[0] == color_hsv) else
                            ("0" if (det or color_hsv) else "1"),
                            secuencia, "%.6f" % t_mono, len(cajas),
                            json.dumps([c.__dict__ for c in cajas], separators=(",", ":")),
                        ])
                        _n += 1
                        if _n % 20 == 0 and _fichero is not None:
                            _fichero.flush()

                    resto = espera - (time.time() - t0)
                    if resto > 0:
                        time.sleep(resto)


def arrancar(directorio_logs="logs"):
    """Abre el registro y lanza el hilo. Devuelve False si no se pudo."""
    global _corriendo, _hilo, _fichero, _csv
    if not os.path.exists(RUTA_HEF):
        print("[red] no existe %s: la red queda apagada" % RUTA_HEF)
        return False
    try:
        os.makedirs(directorio_logs, exist_ok=True)
        ruta = os.path.join(directorio_logs, "red_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        _fichero = open(ruta, "w", newline="")
        _csv = csv.writer(_fichero)
        _csv.writerow(["t", "latencia_ms", "hsv_color", "hsv_cx", "hsv_area",
                       "red_color", "red_cx", "red_score", "coinciden",
                       "secuencia", "captura_mono", "n_cajas", "cajas_json"])
        _corriendo = True
        _hilo = threading.Thread(target=_bucle, daemon=True)
        _hilo.start()
        atexit.register(parar)
        print("[red] comparacion en %s" % ruta)
        return True
    except Exception as err:
        print("[red] no arranco: %s" % err)
        _corriendo = False
        return False


def publicar(cuadro, color_hsv, cx_hsv, area_hsv):
    """Deja el cuadro para que la red lo mire cuando pueda.

    Se sobrescribe el anterior a proposito: interesa la foto mas reciente, no
    acumular una cola que envejece.
    """
    if not _corriendo:
        return
    with _lock:
        global _pendiente, _secuencia
        _secuencia += 1
        _pendiente = (cuadro.copy(), color_hsv, cx_hsv, area_hsv,
                      time.time(), time.monotonic(), _secuencia)


def ultimo_cuadro():
    """Cajas inmutables con tiempo de captura, no de final de inferencia."""
    with _lock:
        return _cuadro_ultimo


def ultima_deteccion():
    """Lo ultimo que vio la red, como (color, cx, score, antiguedad_s)."""
    with _lock:
        if _ultima is None:
            return None, None, 0.0, None
        color, cx, score, t = _ultima
        return color, cx, score, time.time() - t


def parar():
    global _corriendo
    _corriendo = False
    if _hilo is not None and _hilo is not threading.current_thread():
        _hilo.join(timeout=2.0)
    if _hilo is not None and _hilo.is_alive():
        return  # El hilo conserva su registro hasta terminar.
    if _fichero is not None:
        try:
            _fichero.flush()
            _fichero.close()
        except Exception:
            pass
    print("[red] %d inferencias registradas" % _n)
