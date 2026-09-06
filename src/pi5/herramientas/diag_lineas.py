# -*- coding: utf-8 -*-
"""Mide DONDE caen las lineas de piso, a la resolucion real de la ronda.

La pregunta que responde: desde el punto en que esta el robot, ¿a cuantos
milimetros ve la linea azul y la naranja, y sigue dentro de la ventana que el
piloto exige para hacerle caso?

POR QUE A LA RESOLUCION DE LA RONDA Y NO A LA DEL SENSOR
--------------------------------------------------------
Una foto a 4608x2592 ve la linea mucho antes que la ronda, y ademas la ronda
no puede correr ahi: son 14 fps y el ciclo de control necesita mas. Medir el
umbral con una imagen que el robot nunca va a tener es medirse a uno mismo.
Esta herramienta abre la camara con la MISMA configuracion del JSON -- main,
raw, process_scale y umbrales HSV incluidos -- asi que lo que reporta es lo
que el piloto va a ver de verdad.

Ojo con un detalle del IMX708: el campo depende del modo RAW, no del tamano
pedido. Con main 1536x864 y raw 2304x1296 el recorte sigue siendo el sensor
entero; si se dejara elegir a libcamera, el modo nativo 1536x864 recortaria a
3072x1728 y perderia un tercio del campo.

QUE SE MIDE A LA VEZ
--------------------
El LiDAR frontal, porque hoy la esquina se dispara por distancia al muro
(`corner_front_trigger_mm`) y no por la linea. Tener las dos cosas en la
misma fila es lo que permite decidir si la linea sigue visible cuando el muro
llega al umbral, o si para entonces ya paso por debajo del robot.

Uso, con el robot QUIETO en el punto que se quiera evaluar::

    python3 herramientas/diag_lineas.py
    python3 herramientas/diag_lineas.py --segundos 15 --guardar vista.jpg
"""
from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from ronda_nueva.config import cargar_configuracion  # noqa: E402
from ronda_nueva.percepcion_lidar import PercepcionLidar  # noqa: E402
from ronda_nueva.vision_pista import VisionPista  # noqa: E402


def _driver_lidar():
    try:
        from comun.lidar_driver import LidarDriver
    except ImportError:
        from ..comun.lidar_driver import LidarDriver  # type: ignore
    return LidarDriver


def medir(config, segundos: float, guardar: Path | None) -> int:
    from ronda_nueva.hardware import FuenteCamara

    vision = VisionPista(config)
    percepcion = PercepcionLidar(config)
    control = config.get("control", {})
    # Los mismos numeros que usa el piloto para decidir si una linea cuenta.
    max_avance = float(control.get("line_max_forward_mm", 1600.0))
    max_lateral = float(control.get("line_max_lateral_mm", 320.0))

    seguir = True
    ultimo = {"paquete": None, "frame": None}
    tiempos = []

    def al_frame(frame, timestamp):
        t0 = time.monotonic()
        paquete = vision.procesar(frame, timestamp)
        tiempos.append((time.monotonic() - t0) * 1000.0)
        ultimo["paquete"] = paquete
        ultimo["frame"] = frame

    camara = FuenteCamara(config["camera"])
    hilo_camara = threading.Thread(
        target=camara.bucle, args=(lambda: seguir, al_frame), daemon=True
    )
    hilo_camara.start()

    frontales = []
    barridos = [0]
    errores = []

    def al_barrido(scan, timestamp):
        barridos[0] += 1
        try:
            # procesar() devuelve (paredes, objetos, hueco), no un objeto.
            paredes, _objetos, _hueco = percepcion.procesar(scan, timestamp)
            valor = getattr(paredes, "frontal_min_mm", None)
            if valor is None and getattr(paredes, "frontal", None) is not None:
                valor = paredes.frontal.distancia_mm
            if valor is not None and np.isfinite(valor):
                frontales.append(float(valor))
        except Exception as exc:
            if not frontales:
                errores.append("{}: {}".format(type(exc).__name__, exc))

    LidarDriver = _driver_lidar()
    lidar = LidarDriver(
        config["hardware"]["lidar_port"], int(config["hardware"]["lidar_baudrate"])
    )
    hilo_lidar = threading.Thread(
        target=lidar.hilo_lectura, args=(lambda: seguir, al_barrido), daemon=True
    )
    hilo_lidar.start()

    print("[i] Midiendo {:.0f} s con el robot QUIETO...".format(segundos))
    observaciones = []
    limite = time.monotonic() + segundos
    vistos = 0
    while time.monotonic() < limite:
        paquete = ultimo["paquete"]
        if paquete is not None and paquete is not observaciones[-1:] and paquete.lineas:
            for linea in paquete.lineas:
                observaciones.append(linea)
        vistos = max(vistos, 1 if paquete is not None else 0)
        time.sleep(0.05)

    seguir = False
    time.sleep(0.3)
    lidar.cerrar()

    print()
    print("CAMARA")
    print("------")
    if ultimo["frame"] is None:
        print("  [-] no llego ningun cuadro")
        return 2
    alto, ancho = ultimo["frame"].shape[:2]
    print("  resolucion       {}x{}  (raw {})".format(
        ancho, alto, config["camera"].get("raw_sensor_size")))
    if tiempos:
        print("  proceso/cuadro   mediana {:.1f} ms  max {:.1f} ms  ({} cuadros)".format(
            statistics.median(tiempos), max(tiempos), len(tiempos)))
        print("  cadencia util    {:.1f} Hz".format(1000.0 / statistics.median(tiempos)))

    print()
    print("LINEAS DE PISO")
    print("--------------")
    print("  ventana del piloto: y < {:.0f} mm  y  |x| < {:.0f} mm".format(
        max_avance, max_lateral))
    if not observaciones:
        print("  [-] no se detecto ninguna linea desde aqui")
    else:
        por_color = {}
        for linea in observaciones:
            por_color.setdefault(linea.color, []).append(linea)
        for color, muestras in sorted(por_color.items()):
            ys = [m.y_mm for m in muestras]
            xs = [m.x_mm for m in muestras]
            areas = [m.area_px for m in muestras]
            dentro = sum(
                1 for m in muestras
                if m.y_mm < max_avance and abs(m.x_mm) < max_lateral
            )
            print("  {:8s} n={:3d}  y={:6.0f} mm (min {:.0f} max {:.0f})"
                  "  x={:+6.0f} mm  area={:5.0f} px".format(
                      color, len(muestras), statistics.median(ys),
                      min(ys), max(ys), statistics.median(xs),
                      statistics.median(areas)))
            print("           {:d} de {:d} pasan el filtro del piloto ({:.0f} %)".format(
                dentro, len(muestras), 100.0 * dentro / len(muestras)))

    print()
    print("LIDAR (lo que dispara la esquina hoy)")
    print("-------------------------------------")
    if frontales:
        print("  frontal_min      mediana {:.0f} mm  (min {:.0f} max {:.0f}, {} barridos)".format(
            statistics.median(frontales), min(frontales), max(frontales), barridos[0]))
        umbral = float(control.get("corner_front_trigger_mm", 0.0)) or None
        if umbral:
            print("  umbral de giro   {:.0f} mm".format(umbral))
            print("  margen actual    {:+.0f} mm hasta que dispare".format(
                statistics.median(frontales) - umbral))
    else:
        print("  [-] sin barridos utiles" +
              ("  " + errores[0] if errores else ""))

    if guardar is not None and ultimo["frame"] is not None:
        import cv2

        cv2.imwrite(str(guardar), ultimo["frame"])
        print()
        print("[i] Cuadro guardado en {}".format(guardar))
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Donde caen las lineas de piso, a la resolucion de la ronda"
    )
    parser.add_argument("--config", default=None)
    parser.add_argument("--segundos", type=float, default=10.0)
    parser.add_argument("--guardar", type=Path, default=None)
    args = parser.parse_args(argv)
    config = cargar_configuracion(args.config)
    return medir(config, args.segundos, args.guardar)


if __name__ == "__main__":
    raise SystemExit(main())
