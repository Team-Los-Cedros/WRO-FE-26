"""Mide la oclusion FIJA del LiDAR (mastil de la camara) y propone la mascara.

NO MUEVE NADA: abre solo el LiDAR y escucha con el robot parado.

Que separa una pieza del robot de un obstaculo del mundo, con el robot
quieto: la pieza esta SIEMPRE ahi.  El mastil devuelve eco en practicamente
el 100 % de los barridos, siempre a la misma distancia (unos pocos mm de
dispersion), mientras que un objeto real a esa distancia entra y sale segun
el ruido del sensor y esta mas lejos.  Por eso el criterio son DOS medidas
por grado -- persistencia y dispersion -- y no una distancia minima.

El antecedente que obliga a remedir: en la Pi 3B la mascara heredada era
163-191 y el arranque se negaba a armar por "estructura fuera de la mascara
en 192"; el eco del mastil llegaba a 193.  Dos grados de menos bastan para
bloquear la ronda, asi que la mascara se MIDE en cada montaje, no se hereda.

Uso::

    cd /home/pi/wro_pi5_...
    python3 herramientas/diag_mastil.py --segundos 12
    python3 herramientas/diag_mastil.py --segundos 12 --aplicar
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ronda_nueva import config as _config  # noqa: E402
from ronda_nueva.config import cargar_configuracion  # noqa: E402

RUTA_CONFIG = Path(_config.__file__).with_name("configuracion.json")


def _driver_lidar():
    try:
        from comun.lidar_driver import LidarDriver
    except ImportError:
        from ..comun.lidar_driver import LidarDriver  # type: ignore
    return LidarDriver


def _capturar(config, segundos: float) -> list:
    seguir = threading.Event()
    seguir.set()
    barridos: list = []

    def al_barrido(scan, *_):
        if seguir.is_set():
            barridos.append(scan)

    hw = config.get("hardware", {}) if isinstance(config, dict) else {}
    lidar = _driver_lidar()(
        hw.get("lidar_port", "/dev/ttyUSB0"),
        int(hw.get("lidar_baudrate", 460800)),
    )
    hilo = threading.Thread(
        target=lidar.hilo_lectura, args=(seguir.is_set, al_barrido), daemon=True
    )
    hilo.start()
    time.sleep(segundos)
    seguir.clear()
    time.sleep(0.4)
    lidar.cerrar()
    return barridos


def _perfil(barridos: list, cerca_mm: float):
    """Por grado entero: cuantos barridos lo MIRARON y en cuantos vio cerca.

    El denominador no puede ser el total de barridos.  El C1 entrega unos
    465 puntos por vuelta, o sea 1,3 muestras por grado: un grado cualquiera
    se queda sin muestra en una vuelta de cada cuatro por puro reparto
    angular, y contra el total de barridos hasta una pieza solida sale al
    75 %.  La persistencia que dice algo es "de las veces que ese grado tuvo
    muestra, en cuantas la muestra estaba cerca".

    Se guarda el eco MAS CERCANO de cada grado y barrido: si el mastil tapa
    parcialmente, el rayo que lo roza vuelve del fondo y el que le da vuelve
    de la estructura; el minimo es el que describe la pieza.
    """

    cerca = defaultdict(list)
    mirados = defaultdict(int)
    for scan in barridos:
        visto = {}
        muestreados = set()
        for angulo, distancia in scan:
            if distancia <= 0.0:
                continue
            g = int(round(angulo)) % 360
            muestreados.add(g)
            if distancia >= cerca_mm:
                continue
            if g not in visto or distancia < visto[g]:
                visto[g] = distancia
        for g in muestreados:
            mirados[g] += 1
        for g, d in visto.items():
            cerca[g].append(d)
    return cerca, mirados


def _arcos(grados: list) -> list:
    """Agrupa grados sueltos en arcos contiguos, con el cierre en 359->0."""

    if not grados:
        return []
    orden = sorted(grados)
    arcos = []
    inicio = anterior = orden[0]
    for g in orden[1:]:
        if g == anterior + 1:
            anterior = g
            continue
        arcos.append((inicio, anterior))
        inicio = anterior = g
    arcos.append((inicio, anterior))
    if len(arcos) > 1 and arcos[0][0] == 0 and arcos[-1][1] == 359:
        primero = arcos.pop(0)
        ultimo = arcos.pop()
        arcos.append((ultimo[0], primero[1] + 360))
    return arcos


def diagnosticar(args) -> int:
    config = cargar_configuracion()
    barridos = _capturar(config, args.segundos)
    if not barridos:
        print("[-] El LiDAR no entrego ni un barrido.")
        return 1

    total = len(barridos)
    print("barridos: {}  ({:.1f} Hz)".format(total, total / args.segundos))
    print(
        "puntos por barrido: mediana {:.0f}".format(
            statistics.median(len(b) for b in barridos)
        )
    )
    mascara_actual = [
        [float(a), float(b)]
        for a, b in (config.get("lidar", {}).get("blind_sectors_deg") or [])
    ]
    print("mascara en la configuracion: {}".format(mascara_actual))

    por_grado, mirados = _perfil(barridos, args.cerca_mm)
    print()
    print(
        "Ecos de menos de {:.0f} mm por grado.  El % es sobre los barridos que"
        " MIRARON ese grado".format(args.cerca_mm)
    )
    print("  grado   cerca/mirado    %     mediana    p10     p90   dispersion")
    fijos = []
    detalle = {}
    for g in sorted(por_grado):
        v = sorted(por_grado[g])
        visto = mirados.get(g, 0)
        pct = 100.0 * len(v) / visto if visto else 0.0
        mediana = statistics.median(v)
        p10 = v[max(0, int(0.10 * (len(v) - 1)))]
        p90 = v[min(len(v) - 1, int(0.90 * (len(v) - 1)))]
        dispersion = p90 - p10
        detalle[g] = {
            "cerca": len(v),
            "mirado": visto,
            "pct": pct,
            "mediana_mm": mediana,
            "p10_mm": p10,
            "p90_mm": p90,
            "dispersion_mm": dispersion,
        }
        if pct < args.min_pct_listado:
            continue
        rigido = pct >= args.min_pct and dispersion <= args.max_dispersion_mm
        if rigido:
            fijos.append(g)
        print(
            "  {:5d}   {:5d}/{:<5d}  {:6.1f}   {:7.1f} {:7.1f} {:7.1f} {:7.1f}{}".format(
                g, len(v), visto, pct, mediana, p10, p90, dispersion,
                "  FIJO" if rigido else "",
            )
        )

    arcos = _arcos(fijos)
    print()
    print("Arcos de estructura FIJA (persistente y sin dispersion)")
    if not arcos:
        print("  ninguno: con este umbral no hay nada pegado al robot")
    propuesta = []
    for a, b in arcos:
        grados = [g % 360 for g in range(a, b + 1)]
        distancias = [detalle[g]["mediana_mm"] for g in grados]
        print(
            "  {:3d}..{:3d}  ({:2d} grados)  distancia {:.0f}-{:.0f} mm".format(
                a % 360, b % 360, b - a + 1, min(distancias), max(distancias)
            )
        )
        if b - a + 1 < args.min_arco_deg:
            print("     (arco demasiado corto para ser el mastil; no se propone)")
            continue
        # _en_sector es inclusivo en los dos extremos, asi que el sector
        # se escribe con el primer y el ultimo grado a tapar.
        propuesta.append([float(a - args.margen_deg), float(b + args.margen_deg)])

    print()
    if propuesta:
        print(
            "Mascara propuesta (con {} grado(s) de margen):".format(args.margen_deg)
        )
        print("  blind_sectors_deg = {}".format(json.dumps(propuesta)))
        if propuesta != mascara_actual:
            print("  DISTINTA de la que hay en la configuracion.")
        else:
            print("  Coincide con la que hay en la configuracion.")
    else:
        print("Sin propuesta: no se encontro un arco fijo suficientemente ancho.")

    if args.json:
        Path(args.json).write_text(
            json.dumps(
                {
                    "barridos": total,
                    "segundos": args.segundos,
                    "cerca_mm": args.cerca_mm,
                    "min_pct": args.min_pct,
                    "por_grado": detalle,
                    "arcos_fijos": [[a % 360, b % 360] for a, b in arcos],
                    "mascara_actual": mascara_actual,
                    "mascara_propuesta": propuesta,
                },
                indent=1,
            ),
            encoding="utf-8",
            newline="\n",
        )
        print("[+] Detalle escrito en {}".format(args.json))

    if args.aplicar:
        if not propuesta:
            print("[-] No hay propuesta que aplicar.")
            return 1
        crudo = json.loads(RUTA_CONFIG.read_text(encoding="utf-8"))
        crudo["lidar"]["blind_sectors_deg"] = propuesta
        RUTA_CONFIG.write_text(
            json.dumps(crudo, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
            newline="\n",
        )
        print("[+] {} actualizado a {}".format(RUTA_CONFIG, json.dumps(propuesta)))
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--segundos", type=float, default=12.0)
    p.add_argument(
        "--cerca-mm",
        dest="cerca_mm",
        type=float,
        default=260.0,
        help="por encima de esto ya no puede ser una pieza del robot",
    )
    p.add_argument("--min-pct", dest="min_pct", type=float, default=85.0)
    p.add_argument(
        "--min-pct-listado",
        dest="min_pct_listado",
        type=float,
        default=25.0,
        help="umbral solo para IMPRIMIR la fila; el de decision es --min-pct",
    )
    p.add_argument(
        "--max-dispersion-mm",
        dest="max_dispersion_mm",
        type=float,
        default=20.0,
        help="p90-p10 de la distancia; una pieza rigida no se mueve",
    )
    p.add_argument("--min-arco-deg", dest="min_arco_deg", type=int, default=4)
    p.add_argument("--margen-deg", dest="margen_deg", type=int, default=1)
    p.add_argument("--json", default=None)
    p.add_argument(
        "--aplicar",
        action="store_true",
        help="escribe blind_sectors_deg en configuracion.json",
    )
    return diagnosticar(p.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
