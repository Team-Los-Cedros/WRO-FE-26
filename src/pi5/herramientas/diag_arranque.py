"""Comprobacion de arranque en el robot: camara, LiDAR, Pico y tiempos.

NO MUEVE EL ROBOT. No envia ni una consigna a la Pico; solo abre los tres
sensores, mide y describe lo que ve. Es lo primero que hay que correr despues
de tocar el montaje, porque contesta a las tres preguntas que deciden si tiene
sentido seguir:

1. ¿La camara ve el suelo, y desde donde hasta donde?
2. ¿El LiDAR entrega paredes creibles con el robot donde esta?
3. ¿La Pico esta hablando?

Uso::

    cd /home/pi/wro_pi5_...
    python3 herramientas/diag_arranque.py --segundos 4 --guardar /tmp/vista.jpg
"""

from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from ronda_nueva.config import cargar_configuracion  # noqa: E402
from ronda_nueva.geometria_suelo import ProyectorSuelo  # noqa: E402
from ronda_nueva.percepcion_lidar import PercepcionLidar  # noqa: E402
from ronda_nueva.vision_pista import VisionPista  # noqa: E402


def _driver_lidar():
    try:
        from comun.lidar_driver import LidarDriver
    except ImportError:
        from ..comun.lidar_driver import LidarDriver  # type: ignore
    return LidarDriver


def _titulo(texto: str) -> None:
    print()
    print(texto)
    print("-" * len(texto))


def diagnosticar(config, segundos: float, guardar: Path | None) -> int:
    from ronda_nueva.hardware import EnlacePicoNuevo, FuenteCamara

    problemas = []
    seguir = threading.Event()
    seguir.set()

    vision = VisionPista(config)
    percepcion = PercepcionLidar(config)
    ultimo = {"frame": None, "paquete": None, "scan": None, "t_scan": 0.0}
    tiempos_vision = []
    barridos = []

    def al_frame(frame, timestamp):
        inicio = time.perf_counter()
        paquete = vision.procesar(frame, timestamp)
        tiempos_vision.append((time.perf_counter() - inicio) * 1000.0)
        ultimo["frame"] = frame
        ultimo["paquete"] = paquete

    def al_barrido(scan, timestamp):
        ultimo["scan"] = list(scan)
        ultimo["t_scan"] = timestamp
        barridos.append(timestamp)

    # --- camara ---------------------------------------------------------
    camara = FuenteCamara(config["camera"])
    hilo_camara = threading.Thread(
        target=camara.bucle, args=(seguir.is_set, al_frame), daemon=True
    )
    hilo_camara.start()

    # --- lidar ----------------------------------------------------------
    LidarDriver = _driver_lidar()
    lidar = None
    hilo_lidar = None
    try:
        lidar = LidarDriver(
            config["hardware"]["lidar_port"],
            int(config["hardware"].get("lidar_baudrate", 460800)),
        )
        hilo_lidar = threading.Thread(
            target=lidar.hilo_lectura, args=(seguir.is_set, al_barrido), daemon=True
        )
        hilo_lidar.start()
    except Exception as exc:
        problemas.append(f"LiDAR: {exc}")

    # --- pico -----------------------------------------------------------
    enlace = None
    try:
        enlace = EnlacePicoNuevo(
            config["hardware"]["pico_port"],
            int(config["hardware"].get("pico_baudrate", 115200)),
        )
        # Abrir el puerto puede REINICIAR la Pico: sin esperar parece muda
        # cuando no lo esta.
        time.sleep(2.0)
    except Exception as exc:
        problemas.append(f"Pico: {exc}")

    time.sleep(max(1.0, float(segundos)))
    seguir.clear()

    # ---------------------------------------------------------------- camara
    _titulo("CAMARA")
    if camara.ultimo_error:
        problemas.append(f"camara: {camara.ultimo_error}")
        print(f"  [-] {camara.ultimo_error}")
    elif ultimo["frame"] is None:
        problemas.append("camara: no llego ningun cuadro")
        print("  [-] no llego ningun cuadro")
    else:
        alto, ancho = ultimo["frame"].shape[:2]
        print(f"  resolucion       {ancho}x{alto}")
        if tiempos_vision:
            print(
                f"  proceso/cuadro   mediana {statistics.median(tiempos_vision):.1f} ms"
                f"  max {max(tiempos_vision):.1f} ms  ({len(tiempos_vision)} cuadros)"
            )
        if camara.ultima_advertencia:
            print(f"  aviso            {camara.ultima_advertencia}")

        paquete = ultimo["paquete"]
        cobertura = 100.0 * paquete.suelo_px / float(ancho * alto)
        print(f"  suelo detectado  {cobertura:.1f} % del cuadro")
        if cobertura < 12.0:
            problemas.append(
                "el poligono de suelo cubre muy poco: revisar floor_ranges o "
                "floor_seed_norm"
            )
        print(f"  pilares          {len(paquete.pilares)}")
        for pilar in paquete.pilares:
            print(
                f"    {pilar.color:6s} x={pilar.x_mm:7.0f}  y={pilar.y_mm:7.0f}"
                f"  fuente={pilar.fuente}  conf={pilar.confianza:.2f}"
            )
        print(f"  magenta          {len(paquete.magenta)}")
        print(f"  lineas de piso   {[l.color for l in paquete.lineas]}")

        proyector = ProyectorSuelo.desde_config(config["camera"])
        if proyector is None:
            print(
                "  homografia       SIN CALIBRAR (la distancia sale de la altura"
                " del blob)"
            )
        else:
            cerca = proyector.punto_suelo(ancho / 2.0, alto - 1)
            lejos = None
            for v in range(alto):
                punto = proyector.punto_suelo(ancho / 2.0, v)
                if punto is not None and punto[1] <= 3200.0:
                    lejos = punto[1]
                    break
            print(
                f"  suelo visible    desde {cerca[1]:.0f} mm hasta"
                f" {lejos:.0f} mm" if cerca and lejos else "  suelo visible    ?"
            )

        if guardar is not None:
            import cv2

            cv2.imwrite(str(guardar), vision.orientar(ultimo["frame"]))
            print(f"  cuadro guardado  {guardar}")

    # ----------------------------------------------------------------- lidar
    _titulo("LIDAR")
    if not barridos:
        problemas.append("LiDAR: no llego ningun barrido")
        print("  [-] no llego ningun barrido")
    else:
        periodos = [b - a for a, b in zip(barridos, barridos[1:])]
        if periodos:
            print(
                f"  cadencia         {1.0 / statistics.median(periodos):.1f} Hz"
                f"  ({len(barridos)} barridos)"
            )
        scan = ultimo["scan"]
        print(f"  puntos/barrido   {len(scan)}")
        paredes, objetos, _hueco = percepcion.procesar(scan, ultimo["t_scan"])
        for nombre in ("frontal", "izquierda", "derecha", "trasera"):
            recta = getattr(paredes, nombre)
            if recta is None:
                print(f"  pared {nombre:10s} no encontrada")
            else:
                print(
                    f"  pared {nombre:10s} {recta.distancia_mm:7.0f} mm"
                    f"  normal {recta.angulo_deg:+7.1f} deg"
                    f"  residuo {recta.residuo_mm:5.1f} mm  ({recta.puntos} pts)"
                )
        print(f"  corredor libre   {paredes.corredor_mm:.0f} mm")
        print(f"  objetos          {len(objetos)}")
        for objeto in objetos[:6]:
            print(
                f"    x={objeto.x_mm:7.0f}  y={objeto.y_mm:7.0f}"
                f"  ancho={objeto.ancho_mm:5.0f} mm  ({objeto.puntos} pts)"
            )
        if paredes.frontal is None and paredes.trasera is None:
            problemas.append(
                "el LiDAR no encuentra ninguna pared: ¿esta el robot en la pista?"
            )

    # ------------------------------------------------------------------ pico
    _titulo("PICO")
    if enlace is None:
        print("  [-] no se pudo abrir el puerto")
    else:
        viva = enlace.telemetria_valida()
        print(f"  telemetria       {'OK' if viva else 'MUDA'}")
        print(f"  rumbo IMU        {enlace.heading():+.2f} deg")
        print(f"  color de piso    {enlace.color_piso()}")
        print(f"  ultrasonido      {enlace.distancia_ultrasonido_mm()}")
        estado = enlace.estado_watchdog_comando()
        print(f"  watchdog         {estado}")
        if not viva:
            problemas.append(
                "la Pico no habla: 'cd /home/pi/pico_nuevo && bash deploy_pico.sh"
                " --reiniciar'"
            )
        elif estado == "STOP":
            print(
                "  (WD:STOP con el robot parado es NORMAL: nadie le manda"
                " consignas)"
            )

    # --------------------------------------------------------------- resumen
    _titulo("RESUMEN")
    if problemas:
        for problema in problemas:
            print(f"  [-] {problema}")
    else:
        print("  [OK] los tres sensores responden y la percepcion los entiende.")

    for objeto in (lidar, enlace):
        try:
            objeto.cerrar()
        except Exception:
            pass
    return 1 if problemas else 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=None)
    parser.add_argument("--segundos", type=float, default=4.0)
    parser.add_argument("--guardar", default=None, help="ruta del cuadro a guardar")
    args = parser.parse_args(argv)

    config = cargar_configuracion(args.config)
    guardar = Path(args.guardar) if args.guardar else None
    return diagnosticar(config, args.segundos, guardar)


if __name__ == "__main__":
    raise SystemExit(main())
