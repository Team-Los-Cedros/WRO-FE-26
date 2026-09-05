"""Cuanto cuesta un cuadro de vision, a la resolucion que se le pida.

NO MUEVE NADA: solo abre la camara y procesa cuadros reales de la pista.

Existe para decidir la resolucion con un numero.  El presupuesto es el
periodo del cuadro: a 30 fps son 33,3 ms, y si ``procesar`` tarda mas, la
vision deja de ir al ritmo de la camara y el control empieza a decidir sobre
fotos viejas.  Ese margen no se puede estimar desde el PC: depende de la
resolucion, de ``process_scale`` y de cuantas mascaras HSV haya activas.

Cada resolucion que se pide se prueba con su configuracion REESCALADA
(centro optico, homografia y umbrales en pixeles), que es la unica
comparacion honesta: si solo se cambiara ``width``/``height``, el pipeline
detectaria otras cosas y los tiempos no serian comparables.

Uso::

    cd /home/pi/wro_pi5_...
    python3 herramientas/bench_vision.py                 # la de la config
    python3 herramientas/bench_vision.py --resolucion 1280x720 --resolucion 1536x864
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ronda_nueva.config import cargar_configuracion  # noqa: E402
from ronda_nueva.hardware import FuenteCamara  # noqa: E402
from ronda_nueva.vision_pista import VisionPista  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "reescalar_camara", Path(__file__).with_name("reescalar_camara.py")
)
_resc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_resc)


def _medir(config: dict, cuadros: int, calentamiento: int) -> dict:
    """Procesa DENTRO del callback, igual que la ronda.

    En ``ronda_nueva`` la vision corre en el hilo de la camara: si un cuadro
    cuesta mas que el periodo, no se acumula retraso -- se pierden cuadros.
    Medir sobre una lista guardada daria el coste por cuadro pero no la
    cadencia REAL, que es lo que ve la fusion.  Por eso se mide aqui la
    misma tuberia que corre en pista.
    """

    vision = VisionPista(config)
    fuente = FuenteCamara(config["camera"])

    seguir = threading.Event()
    seguir.set()
    tiempos: list = []
    llegadas: list = []
    formas: list = []
    detecciones = [0]
    tope = cuadros + calentamiento

    def al_frame(frame, instante):
        if len(tiempos) >= tope:
            seguir.clear()
            return
        llegadas.append(time.perf_counter())
        t0 = time.perf_counter()
        paquete = vision.procesar(frame, instante)
        tiempos.append((time.perf_counter() - t0) * 1000.0)
        if not formas:
            formas.append(tuple(frame.shape))
        detecciones[0] += len(getattr(paquete, "pilares", ()) or ())

    hilo = threading.Thread(
        target=fuente.bucle, args=(seguir.is_set, al_frame), daemon=True
    )
    hilo.start()

    limite = time.monotonic() + 40.0
    while seguir.is_set() and len(tiempos) < tope and time.monotonic() < limite:
        time.sleep(0.05)
    seguir.clear()
    time.sleep(0.3)

    # FuenteCamara no cierra la Picamera2 -- en la ronda el proceso termina y
    # da igual --, pero aqui se abre una por resolucion y la segunda fallaba
    # con "Camera __init__ sequence did not complete".  Se libera a mano.
    camara = getattr(fuente, "_camara", None)
    if camara is not None:
        for metodo in ("stop", "close"):
            try:
                getattr(camara, metodo)()
            except Exception:
                pass
    hilo.join(timeout=2.0)

    if fuente.ultimo_error:
        raise RuntimeError(fuente.ultimo_error)
    if len(tiempos) <= calentamiento:
        raise RuntimeError("la camara solo entrego {} cuadros".format(len(tiempos)))

    utiles = sorted(tiempos[calentamiento:])
    ventana = llegadas[calentamiento:]
    hz = (
        (len(ventana) - 1) / (ventana[-1] - ventana[0])
        if len(ventana) > 1 and ventana[-1] > ventana[0]
        else float("nan")
    )
    return {
        "cuadros": len(utiles),
        "mediana_ms": statistics.median(utiles),
        "p90_ms": utiles[min(len(utiles) - 1, int(0.90 * (len(utiles) - 1)))],
        "max_ms": utiles[-1],
        "hz_entregados": hz,
        "pilares_por_cuadro": detecciones[0] / max(1, len(tiempos)),
        "forma": formas[0] if formas else None,
    }


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--resolucion",
        action="append",
        default=None,
        help="ANCHOxALTO; repetible.  Por defecto, la de la configuracion",
    )
    p.add_argument(
        "--process-scale",
        dest="process_scale",
        type=float,
        default=None,
        help="sobrescribe vision.process_scale solo para la medida",
    )
    p.add_argument("--cuadros", type=int, default=40)
    p.add_argument("--calentamiento", type=int, default=5)
    p.add_argument("--json", default=None)
    args = p.parse_args(argv)

    base = cargar_configuracion()
    if args.process_scale is not None:
        base["vision"]["process_scale"] = args.process_scale
    fps = float(base["camera"].get("fps", 30.0))
    presupuesto = 1000.0 / max(1.0, fps)
    print("presupuesto por cuadro a {:.0f} fps: {:.1f} ms".format(fps, presupuesto))
    print("process_scale: {}".format(base["vision"].get("process_scale")))
    print()

    peticiones = args.resolucion or [
        "{}x{}".format(base["camera"]["width"], base["camera"]["height"])
    ]
    salida = {}
    for texto in peticiones:
        ancho, alto = (int(v) for v in texto.lower().split("x"))
        config = (
            base
            if (ancho, alto) == (base["camera"]["width"], base["camera"]["height"])
            else _resc.reescalar(base, ancho, alto)
        )
        try:
            medida = _medir(config, args.cuadros, args.calentamiento)
        except Exception as error:
            print("{:>9}: FALLO  {}".format(texto, error))
            continue
        holgura = presupuesto - medida["p90_ms"]
        salida[texto] = medida
        print(
            "{:>9}: mediana {:6.1f} ms   p90 {:6.1f}   max {:6.1f}   "
            "holgura p90 {:+6.1f} ms   ENTREGA {:5.1f} Hz   pilares/cuadro {:.2f}".format(
                texto,
                medida["mediana_ms"],
                medida["p90_ms"],
                medida["max_ms"],
                holgura,
                medida["hz_entregados"],
                medida["pilares_por_cuadro"],
            )
        )
        if holgura < 0:
            print(
                "           por encima del presupuesto: la camara entrega"
                " menos cuadros, no se acumula retraso"
            )

    if args.json and salida:
        Path(args.json).write_text(
            json.dumps(salida, indent=1, default=str), encoding="utf-8", newline="\n"
        )
        print("[+] escrito en {}".format(args.json))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
