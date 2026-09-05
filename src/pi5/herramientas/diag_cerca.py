"""Que ve el LiDAR CERCA del robot, con el robot parado.

NO MUEVE NADA: abre solo el LiDAR y mira los ecos de menos de 600 mm.

Existe por un hallazgo de los CSV del 05-09: en las tres corridas de la linea
base el ``corredor`` libre se cerraba por debajo de 400 mm en el 26-88 % de los
ciclos, y el punto que lo cerraba caia siempre en el mismo sitio del marco del
robot -- x entre -100 y +50 mm, y entre 100 y 300 mm -- **sin depender del
angulo del volante**.  Un pilar no se queda pegado a 200 mm del morro durante
media corrida: o es una pieza del propio robot, o es un eco fantasma.  Esta
herramienta lo decide sin gastar una corrida.

La lectura es directa: si con la pista despejada por delante siguen saliendo
ecos a 100-300 mm en el sector frontal, son del robot.

Uso::

    cd /home/pi/wro_pi5_...
    python3 herramientas/diag_cerca.py --segundos 8
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ronda_nueva.config import cargar_configuracion  # noqa: E402


def _driver_lidar():
    try:
        from comun.lidar_driver import LidarDriver
    except ImportError:
        from ..comun.lidar_driver import LidarDriver  # type: ignore
    return LidarDriver


def _xy(angulo_deg: float, distancia_mm: float):
    # Misma convencion que percepcion_lidar: 0 grados al frente, +x a la
    # derecha.  Se centra el angulo en [-180, 180) para poder promediarlo.
    a = math.radians(angulo_deg)
    return distancia_mm * math.sin(a), distancia_mm * math.cos(a)


def _centrado(angulo_deg: float) -> float:
    a = angulo_deg % 360.0
    return a - 360.0 if a >= 180.0 else a


def diagnosticar(config, segundos: float, cerca_mm: float) -> int:
    lidar_cfg = config.get("lidar", {}) if isinstance(config, dict) else {}
    medio_ancho = float(lidar_cfg.get("corridor_half_width_mm", 95.0) or 95.0)
    emergencia = float(
        (config.get("control", {}) if isinstance(config, dict) else {}).get(
            "emergency_front_mm", 145.0
        )
        or 145.0
    )

    seguir = threading.Event()
    seguir.set()
    barridos: list = []

    def al_barrido(scan, *_):
        if seguir.is_set():
            barridos.append(scan)

    LidarDriver = _driver_lidar()
    hw = config.get("hardware", {}) if isinstance(config, dict) else {}
    lidar = LidarDriver(
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

    if not barridos:
        print("[-] El LiDAR no entrego ni un barrido.")
        return 1

    print(f"barridos: {len(barridos)}  ({len(barridos)/segundos:.1f} Hz)")
    print(f"puntos por barrido: mediana {statistics.median(len(b) for b in barridos):.0f}")

    # 1. Ocupacion por rumbo: cuantos barridos traen algun eco cerca en cada bin
    bins_cerca = defaultdict(list)
    for scan in barridos:
        visto = {}
        for angulo, distancia in scan:
            if not (0.0 < distancia < cerca_mm):
                continue
            a = _centrado(angulo)
            k = int(math.floor(a / 5.0)) * 5
            if k not in visto or distancia < visto[k]:
                visto[k] = distancia
        for k, d in visto.items():
            bins_cerca[k].append(d)

    print()
    print(f"Ecos de menos de {cerca_mm:.0f} mm, por rumbo (5 grados por fila)")
    print("  rumbo    barridos   %    dist_mediana    x       y")
    for k in sorted(bins_cerca):
        v = bins_cerca[k]
        pct = 100.0 * len(v) / len(barridos)
        if pct < 5.0:
            continue
        d = statistics.median(v)
        x, y = _xy(k + 2.5, d)
        marca = "  <-- DENTRO DEL CORREDOR" if abs(x) <= medio_ancho and y > 0 else ""
        print(
            f"  {k:+4d}..{k+5:+4d}  {len(v):6d}  {pct:5.1f}   {d:8.1f}  {x:+7.1f} {y:+7.1f}{marca}"
        )

    # 2. El corredor libre, calculado igual que en percepcion_lidar
    corredores = []
    for scan in barridos:
        mejor, rumbo = float("inf"), float("nan")
        for angulo, distancia in scan:
            x, y = _xy(angulo, distancia)
            if y > 0.0 and abs(x) <= medio_ancho and y < mejor:
                mejor, rumbo = y, _centrado(angulo)
        corredores.append((mejor, rumbo))

    finitos = [c for c, _ in corredores if math.isfinite(c)]
    print()
    print(f"Corredor libre (banda de +-{medio_ancho:.0f} mm), con el robot QUIETO")
    if finitos:
        finitos.sort()
        print(f"  mediana {statistics.median(finitos):7.1f} mm")
        print(f"  minimo  {finitos[0]:7.1f} mm    maximo {finitos[-1]:7.1f} mm")
    bajo_400 = sum(1 for c, _ in corredores if c < 400.0)
    bajo_emg = sum(1 for c, _ in corredores if c < emergencia)
    print(f"  por debajo de 400 mm: {bajo_400} de {len(corredores)}"
          f" ({100.0*bajo_400/len(corredores):.0f} %)")
    print(f"  por debajo de {emergencia:.0f} mm (emergencia): {bajo_emg} de {len(corredores)}"
          f" ({100.0*bajo_emg/len(corredores):.0f} %)")

    cierres = [(c, r) for c, r in corredores if c < 400.0 and math.isfinite(r)]
    if cierres:
        xs, ys = [], []
        for c, r in cierres:
            d = c / max(math.cos(math.radians(r)), 1e-6)
            x, y = _xy(r, d)
            xs.append(x)
            ys.append(y)
        print(f"  quien lo cierra: x mediana {statistics.median(xs):+7.1f} mm,"
              f" y mediana {statistics.median(ys):7.1f} mm,"
              f" rumbo mediano {statistics.median([r for _, r in cierres]):+6.1f} deg")

    print()
    print("Lectura: con la pista despejada por delante, todo eco estable de")
    print("menos de 400 mm en el sector frontal es del propio robot.")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--segundos", type=float, default=8.0)
    parser.add_argument("--cerca-mm", type=float, default=600.0)
    parser.add_argument("--config", type=Path, default=None)
    args = parser.parse_args(argv)
    config = cargar_configuracion(args.config) if args.config else cargar_configuracion()
    return diagnosticar(config, args.segundos, args.cerca_mm)


if __name__ == "__main__":
    raise SystemExit(main())
