#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Captura estatica cámara + LiDAR para calibrar la extrinseca entre ambos.

No abre la Pico ni manda consignas: el robot no se mueve. Guarda un frame con
la configuracion EXACTA que usa la ronda (640x360 sobre el modo de sensor
2304x1296) y la mediana de varios barridos del LiDAR, para que el analisis se
haga despues en el PC sin volver a pisar la pista.

Por que importa el modo de sensor: el 1536x864 es un RECORTE y deja fuera un
tercio del campo. Un pilar que el LiDAR ve a 29 grados puede no aparecer
siquiera en la imagen si se captura en ese modo. Aqui se fuerza el mismo modo
que la ronda para que los bearings sean comparables.

Uso (en la Pi, desde /home/pi/wro_nueva_actual):

    python3 capturar_extrinseca.py /home/pi/extrinseca_01
"""
import json
import os
import sys
import threading
import time

try:                                    # despliegue por paquetes
    from comun.lidar_driver import LidarDriver
    from comun.lidar_geometria import construir_perfil_360
except ImportError:                     # despliegue plano
    from lidar_driver import LidarDriver
    from lidar_geometria import construir_perfil_360

BARRIDOS = 21
ASENTAR_CAMARA_S = 2.5


def _cargar_config_camara():
    """Los parametros opticos vivos, para que el analisis no los adivine."""
    for ruta in ("ronda_nueva/configuracion.json", "configuracion.json"):
        if os.path.exists(ruta):
            with open(ruta, encoding="utf-8") as f:
                return json.load(f)["camera"], ruta
    raise SystemExit("[-] No encuentro configuracion.json junto al despliegue.")


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    destino = sys.argv[1]
    if os.path.exists(destino):
        raise SystemExit("[-] {} ya existe; elige otro nombre.".format(destino))
    os.makedirs(destino)

    camara, ruta_config = _cargar_config_camara()
    print("[i] optica desde {}".format(ruta_config))

    # LiDAR primero: tarda en estabilizar el giro.
    barridos = []
    corriendo = [True]
    lidar = LidarDriver()
    threading.Thread(
        target=lidar.hilo_lectura,
        args=(lambda: corriendo[0], lambda s: barridos.append(construir_perfil_360(s))),
        daemon=True,
    ).start()

    from picamera2 import Picamera2
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (int(camara["width"]), int(camara["height"])),
              "format": camara.get("picamera_format", "RGB888")},
        raw={"size": tuple(camara["raw_sensor_size"])},
        controls={"FrameRate": float(camara.get("fps", 15))},
    ))
    cam.start()
    time.sleep(ASENTAR_CAMARA_S)          # el auto-exposure necesita asentarse

    while len(barridos) < BARRIDOS:
        time.sleep(0.1)
    cam.capture_file(os.path.join(destino, "frame.jpg"))
    usados = list(barridos)                # se congela aqui: mismo instante
    cam.stop()
    cam.close()
    corriendo[0] = False
    time.sleep(0.3)
    lidar.cerrar()

    n = len(usados)
    mediana = [sorted(b[i] for b in usados)[n // 2] for i in range(360)]
    with open(os.path.join(destino, "perfil.json"), "w") as f:
        json.dump(mediana, f)
    with open(os.path.join(destino, "meta.json"), "w") as f:
        json.dump({"camara": camara, "barridos": n,
                   "timestamp": time.time()}, f, indent=1)
    print("[+] {} barridos y un frame en {}".format(n, destino))


if __name__ == "__main__":
    main()
