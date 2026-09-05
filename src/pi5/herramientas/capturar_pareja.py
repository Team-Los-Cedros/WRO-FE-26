"""Captura UNA correspondencia por pilar visible y la acumula en un JSON.

POR QUE EXISTE, TENIENDO YA ``calibrar_suelo.py --usar-lidar``

Aquella captura las cuatro poses en un solo proceso interactivo, y dentro de
cada pose **anota una pareja por ciclo**: con un solo poste delante salen ~30
parejas casi identicas.  El DLT necesita cuatro puntos BIEN REPARTIDOS, no
cuatro puntos repetidos treinta veces; treinta copias del mismo punto no
condicionan nada y ademas disfrazan el recuento de "conviene 12".

Aqui cada pose deja **una** pareja por poste, con la mediana de todos los
ciclos (que si quita ruido), y se anade a un archivo.  Asi la calibracion se
puede hacer en varias sesiones, revisando entre pose y pose, y al final:

    python3 herramientas/calibrar_suelo.py --desde-json pares.json

El criterio de emparejado es el MISMO de ``calibrar_suelo`` -- se importa de
alli, no se reescribe -- para que no puedan separarse.

Uso, una vez por pose::

    python3 herramientas/capturar_pareja.py --salida pares.json --segundos 5
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from herramientas.calibrar_suelo import _emparejar  # noqa: E402
from ronda_nueva.config import cargar_configuracion  # noqa: E402


def _driver_lidar():
    try:
        from comun.lidar_driver import LidarDriver
    except ImportError:
        from ronda_nueva import ronda_nueva as _entrada

        LidarDriver = _entrada._driver_lidar()
    return LidarDriver


def corregir_cara(mundo, medio_pilar_mm: float):
    """Del punto que mide el LiDAR al CENTRO del poste.

    El LiDAR solo ve la cara frontal, asi que su centroide cae medio poste mas
    cerca de lo que esta el eje.  La camara, en cambio, da el punto donde la
    silueta toca el suelo, que si esta bajo el eje.  Emparejar los dos sin
    corregir mete un sesgo sistematico de 25 mm HACIA el robot, y por eso la
    herramienta original descarta todo lo que este a menos de 600 mm.

    Corrigiendo el sesgo se pueden usar postes cercanos, que es justo lo que
    falta: entre 600 y 900 mm el punto de contacto solo recorre 44 filas del
    cuadro, y bajar a 400 mm anade 68 mas.  Sin filas no hay homografia: el
    DLT se queda extrapolando de una franja a toda la pista.
    """

    x, y = mundo
    distancia = math.hypot(x, y)
    if distancia < 1e-6:
        return mundo
    escala = (distancia + medio_pilar_mm) / distancia
    return (x * escala, y * escala)


def capturar(config_path, segundos: float, distancia_min_mm: float, puerta_mm: float,
             medio_pilar_mm: float = 0.0):
    from ronda_nueva.hardware import FuenteCamara
    from ronda_nueva.percepcion_lidar import PercepcionLidar
    from ronda_nueva.vision_pista import VisionPista

    config = cargar_configuracion(config_path)
    # Igual que la calibracion: se necesita la vision SIN homografia, en pixeles.
    config["camera"]["ground_homography"]["ready"] = False
    vision = VisionPista(config)
    percepcion = PercepcionLidar(config)

    seguir = threading.Event()
    seguir.set()
    ultimo = {"vision": None, "lidar": None}

    def al_frame(frame, timestamp):
        ultimo["vision"] = vision.procesar(frame, timestamp)

    def al_barrido(scan, timestamp):
        ultimo["lidar"] = percepcion.procesar(scan, timestamp)

    camara = FuenteCamara(config["camera"])
    hilo_camara = threading.Thread(
        target=camara.bucle, args=(seguir.is_set, al_frame), daemon=True
    )
    hilo_camara.start()
    LidarDriver = _driver_lidar()
    lidar = LidarDriver(
        config["hardware"]["lidar_port"], int(config["hardware"]["lidar_baudrate"])
    )
    hilo_lidar = threading.Thread(
        target=lidar.hilo_lectura, args=(seguir.is_set, al_barrido), daemon=True
    )
    hilo_lidar.start()

    crudas = []
    ambiguos = []
    ciclos = 0
    try:
        limite = time.monotonic() + segundos
        while time.monotonic() < limite:
            paquete = ultimo["vision"]
            lectura = ultimo["lidar"]
            if paquete and lectura:
                ciclos += 1
                _paredes, objetos, _hueco = lectura
                lejanos = [o for o in objetos if o.distancia_mm > distancia_min_mm]
                choques = ambiguedad(lejanos, puerta_mm)
                if choques:
                    ambiguos.append((len(lejanos), choques))
                parejas = _emparejar(paquete.pilares, objetos, distancia_min_mm, puerta_mm)
                if medio_pilar_mm:
                    parejas = [(pix, corregir_cara(mundo, medio_pilar_mm))
                               for pix, mundo in parejas]
                crudas.extend(parejas)
            time.sleep(0.1)
    finally:
        seguir.clear()
        try:
            lidar.cerrar()
        except Exception:
            pass
    return crudas, ciclos, ambiguos


def ambiguedad(objetos, puerta_mm: float):
    """Postes que el emparejador NO puede distinguir entre si.

    ``_emparejar`` asocia por DISTANCIA y con una puerta de 180 mm.  Eso es
    correcto -- la distancia es lo unico que los dos sensores miden bien -- pero
    solo funciona si los postes estan separados EN DISTANCIA mas que la puerta.
    Con cuatro postes repartidos en abanico a la misma distancia, sus distancias
    caen dentro de la puerta unas de otras y el emparejador asigna el blob de un
    poste al objeto de otro, en silencio.

    Paso el 05-09: cuatro postes a 676, 682, 741 y 748 mm (72 mm de rango, con
    la puerta en 180) dieron tres parejas cruzadas de cuatro.  Ajustadas a un
    modelo pinhole, sus residuos eran de -360, +275 y +216 px, contra +10..+30
    de las buenas.  Habrian entrado en la homografia sin avisar.

    Devuelve la lista de choques (i, j, separacion_mm).
    """

    choques = []
    for i in range(len(objetos)):
        for j in range(i + 1, len(objetos)):
            sep = abs(objetos[i].distancia_mm - objetos[j].distancia_mm)
            if sep < puerta_mm:
                choques.append((i, j, sep))
    return choques


def agrupar(crudas, radio_mm: float = 150.0):
    """Una entrada por poste: agrupa por la posicion que da el LiDAR."""

    grupos: list = []
    for pix, mundo in crudas:
        for grupo in grupos:
            gx, gy = grupo["mundo"][0]
            if abs(mundo[0] - gx) <= radio_mm and abs(mundo[1] - gy) <= radio_mm:
                grupo["pix"].append(pix)
                grupo["mundo"].append(mundo)
                break
        else:
            grupos.append({"pix": [pix], "mundo": [mundo]})

    salida = []
    for grupo in grupos:
        px = statistics.median([p[0] for p in grupo["pix"]])
        py = statistics.median([p[1] for p in grupo["pix"]])
        mx = statistics.median([m[0] for m in grupo["mundo"]])
        my = statistics.median([m[1] for m in grupo["mundo"]])
        disp_x = max(m[0] for m in grupo["mundo"]) - min(m[0] for m in grupo["mundo"])
        disp_y = max(m[1] for m in grupo["mundo"]) - min(m[1] for m in grupo["mundo"])
        salida.append(((px, py), (mx, my), len(grupo["pix"]), disp_x, disp_y))
    return salida


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--salida", default="pares.json")
    parser.add_argument("--segundos", type=float, default=5.0)
    parser.add_argument("--distancia-min-mm", type=float, default=600.0)
    parser.add_argument("--puerta-mm", type=float, default=180.0)
    parser.add_argument("--config", default=None)
    parser.add_argument(
        "--corregir-cara",
        action="store_true",
        help="lleva el punto del LiDAR de la cara al centro del poste "
             "(imprescindible si se baja --distancia-min-mm de 600)",
    )
    parser.add_argument(
        "--forzar",
        action="store_true",
        help="guarda aunque la pose sea ambigua (no lo uses para calibrar)",
    )
    parser.add_argument(
        "--min-ciclos",
        type=int,
        default=10,
        help="una pareja con menos apariciones que esto no se guarda",
    )
    args = parser.parse_args(argv)

    medio_pilar = 0.0
    if args.corregir_cara:
        config = cargar_configuracion(args.config)
        medio_pilar = float(config.get("track", {}).get("pillar_width_mm", 50.0)) / 2.0
        print(f"[i] Corrigiendo la cara del poste: +{medio_pilar:.0f} mm en el rayo.")

    crudas, ciclos, ambiguos = capturar(
        args.config, args.segundos, args.distancia_min_mm, args.puerta_mm, medio_pilar
    )
    print(f"\n{ciclos} ciclos, {len(crudas)} emparejamientos crudos")

    if ambiguos and not args.forzar:
        peor = min(min(c[2] for c in ch) for _n, ch in ambiguos)
        print(f"\n[-] POSE AMBIGUA: en {len(ambiguos)} de {ciclos} ciclos hay dos")
        print(f"    postes separados solo {peor:.0f} mm EN DISTANCIA, con la puerta")
        print(f"    de emparejado en {args.puerta_mm:.0f} mm.  El emparejador asocia por")
        print("    distancia y no puede distinguirlos: cruzaria los blobs sin avisar.")
        print("    No se guarda nada.")
        print("\n    Separa los postes MAS DE 180 mm EN DISTANCIA (no en angulo),")
        print("    o -- mas seguro -- deja UN SOLO poste por pose.")
        return 1

    if not crudas:
        print("[-] Ninguna pareja.  Comprueba con diag_pilares.py que los DOS")
        print("    sensores ven el poste y que esta a mas de 600 mm.")
        return 1

    grupos = agrupar(crudas)
    destino = Path(args.salida)
    previas = []
    if destino.is_file():
        previas = json.loads(destino.read_text(encoding="utf-8"))

    nuevas = []
    for pix, mundo, n, dx, dy in grupos:
        if n < args.min_ciclos:
            print(f"  [ ] descartada: solo {n} apariciones  ({mundo[0]:.0f}, {mundo[1]:.0f}) mm")
            continue
        print(
            f"  [+] pixel ({pix[0]:7.1f}, {pix[1]:7.1f})  ->  "
            f"({mundo[0]:7.1f}, {mundo[1]:7.1f}) mm   "
            f"{n} apariciones, dispersion {dx:.0f}x{dy:.0f} mm"
        )
        nuevas.append([[pix[0], pix[1]], [mundo[0], mundo[1]]])

    if not nuevas:
        print("[-] Nada estable que guardar en esta pose.")
        return 1

    todas = previas + nuevas
    destino.write_text(json.dumps(todas, indent=1), encoding="utf-8")
    print(f"\n[i] {len(nuevas)} anadidas.  {destino} tiene ya {len(todas)} correspondencias.")

    # El DLT necesita reparto, no cantidad: avisar si estan apinadas.
    xs = [p[1][0] for p in todas]
    ys = [p[1][1] for p in todas]
    print(f"[i] reparto actual: x de {min(xs):.0f} a {max(xs):.0f} mm, "
          f"y de {min(ys):.0f} a {max(ys):.0f} mm")
    if len(todas) >= 4 and (max(xs) - min(xs)) < 300.0:
        print("[!] Poco reparto LATERAL. Con los postes casi en linea el DLT")
        print("    queda mal condicionado: mueve el siguiente bien a un lado.")
    if len(todas) < 4:
        print(f"[i] Faltan {4 - len(todas)} para poder resolver (y conviene llegar a 8-12).")
    else:
        print("[i] Ya se puede resolver:")
        print(f"    python3 herramientas/calibrar_suelo.py --desde-json {destino}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
