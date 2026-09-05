"""Cambia la RESOLUCION de salida de la camara sin perder la calibracion.

Subir la resolucion no es tocar dos numeros.  Media configuracion esta
expresada en PIXELES del cuadro, y todos esos numeros describen el mismo
angulo o el mismo tamaño real solo si se reescalan a la vez:

* ``principal_x_px`` / ``principal_y_px`` -- el centro optico esta en el mismo
  sitio de la lente, pero le corresponde otro pixel.
* ``ground_homography.matrix`` -- mapea PIXEL a milimetros de suelo.  Si el
  cuadro crece 1,2 veces y la matriz no se toca, cada pixel se proyecta a
  1,2 veces su distancia real y todo el mapa se estira.  Se corrige
  componiendo con la escala: ``H_nueva = H_vieja @ diag(1/s, 1/s, 1)``.
* los umbrales de vision en pixeles -- las AREAS van con ``s**2`` y las
  LONGITUDES con ``s``, o si no cambia la sensibilidad de la deteccion.

Lo que NO cambia: ``hfov_deg`` (es optica), ``raw_sensor_size`` (es el modo
del sensor, y es lo que fija el campo de verdad: 2304x1296 usa el area
completa mientras el modo nativo 1536x864 del IMX708 recorta a 3072x1728 y
tira un tercio del angulo), y todo lo que ya esta en fracciones del cuadro
(``roi_*_ratio``, ``floor_seed_norm``, ``robot_mask_polygons_norm``).

Solo admite cambios de escala UNIFORME: si la relacion de aspecto cambiara,
el reescalado de la homografia y del HFOV dejarian de ser una sola division
y habria que recalibrar de verdad.

Uso::

    python3 herramientas/reescalar_camara.py 1536 864            # simula
    python3 herramientas/reescalar_camara.py 1536 864 --aplicar
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ronda_nueva import config as _config  # noqa: E402
from ronda_nueva.config import validar_configuracion  # noqa: E402

RUTA_CONFIG = Path(_config.__file__).with_name("configuracion.json")

# Umbrales de ``vision`` que viven en pixeles del cuadro COMPLETO.
LONGITUDES_PX = (
    "min_height_px",
    "morph_open_px",
    "morph_close_px",
    "line_polygon_dilate_px",
)
AREAS_PX = (
    "min_area_px",
    "magenta_min_area_px",
    "line_min_area_px",
)


class ErrorReescalado(Exception):
    pass


def reescalar(config: dict, ancho: int, alto: int) -> dict:
    """Devuelve una copia de ``config`` con el cuadro a ``ancho`` x ``alto``."""

    camara = config["camera"]
    ancho_viejo = int(camara["width"])
    alto_viejo = int(camara["height"])
    if ancho <= 0 or alto <= 0:
        raise ErrorReescalado("la resolucion debe ser positiva")

    sx = ancho / float(ancho_viejo)
    sy = alto / float(alto_viejo)
    if abs(sx - sy) > 1e-6:
        raise ErrorReescalado(
            "escala no uniforme ({:.4f} en x, {:.4f} en y): cambiaria la"
            " relacion de aspecto y hay que recalibrar, no reescalar".format(sx, sy)
        )
    s = sx

    nuevo = json.loads(json.dumps(config))
    cam = nuevo["camera"]
    cam["width"] = int(ancho)
    cam["height"] = int(alto)
    for clave, viejo in (
        ("principal_x_px", camara.get("principal_x_px")),
        ("principal_y_px", camara.get("principal_y_px")),
    ):
        if viejo is not None:
            cam[clave] = round(float(viejo) * s, 4)

    suelo = cam.get("ground_homography") or {}
    matriz = suelo.get("matrix")
    if matriz:
        # H_nueva = H_vieja @ diag(1/s, 1/s, 1): un pixel del cuadro nuevo se
        # traduce primero al cuadro con el que se calibro y luego al suelo.
        suelo["matrix"] = [
            [fila[0] / s, fila[1] / s, fila[2]] for fila in matriz
        ]

    vision = nuevo.get("vision", {})
    for clave in LONGITUDES_PX:
        if clave in vision:
            vision[clave] = max(1, int(round(float(vision[clave]) * s)))
    for clave in AREAS_PX:
        if clave in vision:
            vision[clave] = max(1, int(round(float(vision[clave]) * s * s)))

    return nuevo


def _resumen(viejo: dict, nuevo: dict) -> None:
    def plano(d, pre=""):
        salida = {}
        for k, v in d.items():
            if isinstance(v, dict):
                salida.update(plano(v, pre + k + "."))
            else:
                salida[pre + k] = v
        return salida

    a, b = plano(viejo), plano(nuevo)
    for clave in sorted(set(a) | set(b)):
        if a.get(clave) != b.get(clave):
            print("  {}\n      antes: {}\n      ahora: {}".format(
                clave, a.get(clave), b.get(clave)))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("ancho", type=int)
    p.add_argument("alto", type=int)
    p.add_argument("--config", default=None)
    p.add_argument("--aplicar", action="store_true")
    args = p.parse_args(argv)

    ruta = Path(args.config) if args.config else RUTA_CONFIG
    viejo = json.loads(ruta.read_text(encoding="utf-8"))
    try:
        nuevo = reescalar(viejo, args.ancho, args.alto)
    except ErrorReescalado as error:
        print("[-] {}".format(error))
        return 2

    print("{}x{} -> {}x{}".format(
        viejo["camera"]["width"], viejo["camera"]["height"], args.ancho, args.alto))
    _resumen(viejo, nuevo)

    validar_configuracion(nuevo)
    print("[+] la configuracion resultante pasa validar_configuracion")

    if args.aplicar:
        ruta.write_text(
            json.dumps(nuevo, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
            newline="\n",
        )
        print("[+] escrito en {}".format(ruta))
    else:
        print("(simulacion; usa --aplicar para escribirlo)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
