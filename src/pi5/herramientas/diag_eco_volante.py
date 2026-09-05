"""De donde sale el eco que cierra el corredor cuando el volante va a tope.

MUEVE EL VOLANTE, NO EL ROBOT: manda velocidad 0 en todas las consignas y solo
cambia el angulo del servo.  El robot se queda donde este; conviene ponerlo con
un metro largo despejado delante para que cualquier eco cercano sea suyo y no
de la pista.

La corrida 3 del 04-09 dejo la correlacion hecha: con el volante recto el
corredor libre nunca bajo de 339 mm en 122 muestras, y a tope de volante bajo
de 300 en 169 de 312, con minimos de 10 mm y ambos muros a mas de 600.  Esto
mide el eco directamente, angulo por angulo, para poder enmascararlo con
numeros en vez de con una teoria.

Uso::

    cd /home/pi/wro_pi5_...
    python3 herramientas/diag_eco_volante.py --angulos -20 -10 0 10 20
"""

from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ronda_nueva.config import cargar_configuracion  # noqa: E402
from ronda_nueva.percepcion_lidar import normalizar_barrido  # noqa: E402


def _driver_lidar():
    try:
        from comun.lidar_driver import LidarDriver
    except ImportError:
        from ..comun.lidar_driver import LidarDriver  # type: ignore
    return LidarDriver


def _firmado(grados: float) -> float:
    """Rumbo en [-180, 180): mas legible que 341 para algo casi de frente."""

    return grados - 360.0 if grados >= 180.0 else grados


def medir(config, angulos, segundos: float, rango_mm: float, sector_deg: float):
    from ronda_nueva.hardware import EnlacePicoNuevo

    seguir = threading.Event()
    seguir.set()
    ultimo = {"scan": None}

    def al_barrido(scan, _timestamp):
        ultimo["scan"] = list(scan)

    LidarDriver = _driver_lidar()
    lidar = LidarDriver(
        config["hardware"]["lidar_port"],
        int(config["hardware"].get("lidar_baudrate", 460800)),
    )
    hilo = threading.Thread(
        target=lidar.hilo_lectura, args=(seguir.is_set, al_barrido), daemon=True
    )
    hilo.start()

    enlace = EnlacePicoNuevo(
        config["hardware"]["pico_port"],
        int(config["hardware"].get("pico_baudrate", 115200)),
    )
    time.sleep(2.0)  # abrir el puerto reinicia la Pico

    resultados = []
    try:
        for angulo in angulos:
            fin = time.time() + segundos
            muestras = []
            while time.time() < fin:
                # Velocidad 0 SIEMPRE: esto no es una consigna de marcha, es
                # mantener vivo el watchdog mientras el servo esta girado.
                enlace.enviar(0, float(angulo))
                time.sleep(0.05)
                scan = ultimo["scan"]
                if not scan:
                    continue
                for grados, distancia in normalizar_barrido(scan, 3500.0):
                    firmado = _firmado(grados)
                    if abs(firmado) <= sector_deg and distancia <= rango_mm:
                        muestras.append((firmado, distancia))
            resultados.append((angulo, muestras))
    finally:
        enlace.detener()
        enlace.cerrar()
        seguir.clear()
        time.sleep(0.3)
        try:
            lidar.cerrar()
        except Exception:
            pass
    return resultados


def informar(resultados, rango_mm: float, sector_deg: float) -> None:
    print(f"\nEcos a menos de {rango_mm:.0f} mm dentro de +-{sector_deg:.0f} grados del frente")
    print(f"{'servo':>6}{'n':>6}{'min_mm':>8}{'mediana':>9}  reparto por rumbo")
    for angulo, muestras in resultados:
        if not muestras:
            print(f"{angulo:6.0f}{0:6d}{'-':>8}{'-':>9}  (limpio)")
            continue
        distancias = [d for _a, d in muestras]
        cubos = {}
        for rumbo, _d in muestras:
            clave = int(round(rumbo / 10.0) * 10)
            cubos[clave] = cubos.get(clave, 0) + 1
        reparto = " ".join(
            f"{k:+d}:{v}" for k, v in sorted(cubos.items()) if v >= 2
        )
        print(
            f"{angulo:6.0f}{len(muestras):6d}{min(distancias):8.0f}"
            f"{statistics.median(distancias):9.0f}  {reparto}"
        )
    print(
        "\nLo que se busca: rumbos que SOLO aparecen con el volante girado.\n"
        "Eso es el propio robot y hay que enmascararlo; lo que sale igual con\n"
        "el volante recto es pista de verdad."
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None)
    parser.add_argument("--angulos", type=float, nargs="+", default=[0, -10, -20, 10, 20])
    parser.add_argument("--segundos", type=float, default=2.5)
    parser.add_argument("--rango-mm", type=float, default=400.0)
    parser.add_argument("--sector-deg", type=float, default=70.0)
    args = parser.parse_args(argv)

    raiz = Path(__file__).resolve().parents[1]
    config = cargar_configuracion(args.config or str(raiz / "ronda_nueva" / "configuracion.json"))
    print("[!] El volante SE VA A MOVER. El motor se queda a cero.")
    resultados = medir(config, args.angulos, args.segundos, args.rango_mm, args.sector_deg)
    informar(resultados, args.rango_mm, args.sector_deg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
