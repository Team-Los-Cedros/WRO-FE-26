"""Calibra (y comprueba) la homografia del suelo de la camara del mastil.

TRES MODOS, EN EL ORDEN EN QUE SE USAN

1. ``--cobertura``  No toca el robot y no necesita ni camara.  Dada una altura
   y un cabeceo, dice QUE TROZO DE SUELO entra en el cuadro: desde cuantos mm
   por delante hasta cuantos, y cuanto de ancho.  Sirve MIENTRAS se monta la
   camara, para elegir altura e inclinacion con un numero en vez de a ojo.

2. ``--usar-lidar``  La calibracion buena.  Se colocan tres o cuatro pilares
   repartidos por el campo, se capturan varias poses y en cada una se emparejan
   los blobs de la camara con los objetos del LiDAR.  Cada pareja da una
   correspondencia (pixel donde el poste toca el suelo) -> (x, y en mm), y con
   cuatro o mas sale la homografia por DLT.  El LiDAR es la regla: mide en
   milimetros y ya esta en el marco del robot.

3. ``--comprobar``  Vuelve a medir el error de reproyeccion sobre datos nuevos.
   Es lo unico que dice si la calibracion sigue valiendo despues de tocar el
   mastil.

DOS REGLAS DEL METODO QUE COSTARON DESCUBRIR (sesion del 02-09, siguen valiendo)

* **Emparejar por DISTANCIA, nunca por bearing.**  La distancia es la magnitud
  que los dos sensores miden bien; el bearing es justo lo que se esta
  calibrando y usarlo mete la respuesta en la pregunta.
* **Descartar los pilares a menos de 600 mm.**  El LiDAR ve solo la cara
  frontal y la camara la silueta entera, asi que sus centroides no coinciden
  de cerca.  Aqui ademas se usa el punto de CONTACTO con el suelo, no el
  centroide, lo que reduce el problema pero no lo elimina.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ronda_nueva.geometria_suelo import (  # noqa: E402
    ErrorHomografia,
    ProyectorSuelo,
    desde_correspondencias,
    desde_montaje,
    error_de_reproyeccion,
)


def informe_cobertura(
    ancho: int,
    alto: int,
    hfov_deg: float,
    altura_mm: float,
    cabeceo_deg: float,
    adelante_mm: float,
) -> str:
    """Que trozo de suelo ve la camara con ese montaje."""

    H = desde_montaje(
        ancho, alto, hfov_deg, altura_mm, cabeceo_deg, adelante_mm=adelante_mm
    )
    proyector = ProyectorSuelo(H, ancho, alto, alcance_max_mm=6000.0, avance_min_mm=0.0)

    esquinas = {
        "abajo-centro": (ancho / 2.0, alto - 1),
        "abajo-izq": (0.0, alto - 1),
        "abajo-der": (ancho - 1.0, alto - 1),
        "arriba-centro": (ancho / 2.0, 0.0),
        "arriba-izq": (0.0, 0.0),
        "arriba-der": (ancho - 1.0, 0.0),
    }
    lineas = [
        f"Camara {ancho}x{alto}, HFOV {hfov_deg:.2f} deg",
        f"Altura {altura_mm:.0f} mm, cabeceo {cabeceo_deg:.1f} deg, "
        f"adelante {adelante_mm:+.0f} mm",
        "",
    ]
    for nombre, (u, v) in esquinas.items():
        punto = proyector.punto_suelo(u, v)
        if punto is None:
            lineas.append(f"  {nombre:14s} -> por encima del horizonte")
        else:
            lineas.append(f"  {nombre:14s} -> x={punto[0]:8.0f} mm  y={punto[1]:8.0f} mm")

    cerca = proyector.punto_suelo(ancho / 2.0, alto - 1)
    lineas.append("")
    if cerca:
        lineas.append(f"  Suelo visible desde:  {cerca[1]:8.0f} mm por delante")

    # El "alcance" util no es el del borde superior del cuadro: si el horizonte
    # cae dentro de la imagen, ese borde apunta al infinito y el numero no dice
    # nada.  Interesa hasta donde llega el suelo dentro del tope de medida, y en
    # que fila cae el horizonte, porque todo lo que queda por encima es muro.
    fila_horizonte = None
    for v in range(alto):
        if proyector.punto_suelo(ancho / 2.0, v) is not None:
            fila_horizonte = v
            break

    alcance = None
    if fila_horizonte is not None:
        for v in range(fila_horizonte, alto):
            punto = proyector.punto_suelo(ancho / 2.0, v)
            if punto is not None and punto[1] <= 3200.0:
                alcance = punto[1]
                break

    if alcance is not None:
        lineas.append(f"  Alcance util:         {alcance:8.0f} mm (tope 3200)")
    if fila_horizonte:
        porcentaje = 100.0 * fila_horizonte / alto
        lineas.append(
            f"  Horizonte en la fila: {fila_horizonte:8d}  ({porcentaje:.0f} %"
            " del cuadro es muro)"
        )
    else:
        lineas.append("  Horizonte:            fuera del cuadro (todo es suelo)")

    lineas.append("")
    if alcance is None or alcance < 1600.0:
        lineas.append(
            "  AVISO: con menos de 1,6 m de alcance el mapa de la pista solo ve"
        )
        lineas.append(
            "  el pilar inminente y pierde la ventaja de anticipar los otros dos,"
        )
        lineas.append("  que es de donde sale el tiempo que se gana en las vueltas 2 y 3.")
        lineas.append("  Sube la camara o reduce el cabeceo.")
    elif cerca and cerca[1] > 450.0:
        lineas.append(
            "  AVISO: el suelo no empieza hasta muy lejos.  Un pilar que el robot"
        )
        lineas.append(
            "  tiene ya al lado se sale del cuadro por abajo y solo lo vera el"
        )
        lineas.append("  LiDAR.  Aumenta el cabeceo o baja la camara.")
    else:
        lineas.append("  Cobertura razonable para el mapa de la pista.")
    return "\n".join(lineas)


def _emparejar(
    visuales: Sequence, objetos: Sequence, distancia_min_mm: float, puerta_mm: float
) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Empareja blobs y objetos LiDAR por DISTANCIA, nunca por bearing."""

    parejas = []
    usados = set()
    for visual in visuales:
        pie_x = visual.bbox[0] + visual.bbox[2] / 2.0
        pie_y = float(visual.bbox[1] + visual.bbox[3])
        distancia_visual = visual.distancia_por_altura_mm
        if distancia_visual <= distancia_min_mm:
            continue
        mejor, mejor_error = None, puerta_mm
        for indice, objeto in enumerate(objetos):
            if indice in usados or objeto.distancia_mm <= distancia_min_mm:
                continue
            error = abs(objeto.distancia_mm - distancia_visual)
            if error < mejor_error:
                mejor, mejor_error = indice, error
        if mejor is None:
            continue
        usados.add(mejor)
        objeto = objetos[mejor]
        parejas.append(((pie_x, pie_y), (objeto.x_mm, objeto.y_mm)))
    return parejas


def capturar_con_lidar(config_path: Optional[str], poses: int, segundos: float):
    """Captura correspondencias con el robot QUIETO en varias poses."""

    from ronda_nueva.config import cargar_configuracion
    from ronda_nueva.hardware import FuenteCamara
    from ronda_nueva.percepcion_lidar import PercepcionLidar
    from ronda_nueva.vision_pista import VisionPista

    try:
        from comun.lidar_driver import LidarDriver
    except ImportError:
        from ronda_nueva import ronda_nueva as _entrada

        LidarDriver = _entrada._driver_lidar()

    import threading

    config = cargar_configuracion(config_path)
    # Para calibrar hace falta la vision SIN homografia: se usan los pixeles.
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
    lidar = LidarDriver(
        config["hardware"]["lidar_port"], int(config["hardware"]["lidar_baudrate"])
    )
    hilo_lidar = threading.Thread(
        target=lidar.hilo_lectura, args=(seguir.is_set, al_barrido), daemon=True
    )
    hilo_lidar.start()

    parejas = []
    try:
        for pose in range(1, poses + 1):
            input(
                f"\n[{pose}/{poses}] Coloca el robot y los pilares, y pulsa Enter. "
                "Reparte los postes en distancia y en angulo."
            )
            limite = time.monotonic() + segundos
            nuevas = []
            while time.monotonic() < limite:
                paquete = ultimo["vision"]
                lectura = ultimo["lidar"]
                if paquete and lectura:
                    _paredes, objetos, _hueco = lectura
                    nuevas.extend(_emparejar(paquete.pilares, objetos, 600.0, 180.0))
                time.sleep(0.1)
            print(f"    {len(nuevas)} correspondencias en esta pose")
            parejas.extend(nuevas)
    finally:
        seguir.clear()
        try:
            lidar.cerrar()
        except Exception:
            pass
    return parejas


def resolver(parejas, ancho: int, alto: int, hfov_deg: float):
    if len(parejas) < 4:
        raise ErrorHomografia(
            f"solo {len(parejas)} correspondencias; hacen falta 4 y conviene 12"
        )
    H = desde_correspondencias(parejas)
    focal = (ancho / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    proyector = ProyectorSuelo(H, ancho, alto, focal_px=focal)
    medio, maximo = error_de_reproyeccion(proyector, parejas)
    return H, medio, maximo


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=None)
    parser.add_argument("--cobertura", action="store_true")
    parser.add_argument("--usar-lidar", action="store_true")
    parser.add_argument("--desde-json", default=None, help="archivo de pares")
    parser.add_argument("--poses", type=int, default=4)
    parser.add_argument("--segundos", type=float, default=3.0)
    parser.add_argument("--altura-mm", type=float, default=300.0)
    parser.add_argument("--cabeceo-deg", type=float, default=30.0)
    parser.add_argument("--adelante-mm", type=float, default=-80.0)
    parser.add_argument("--ancho", type=int, default=1280)
    parser.add_argument("--alto", type=int, default=720)
    parser.add_argument("--hfov-deg", type=float, default=68.16865)
    parser.add_argument(
        "--escribir",
        action="store_true",
        help="escribe la matriz en el JSON de configuracion",
    )
    args = parser.parse_args(argv)

    if args.cobertura:
        print(
            informe_cobertura(
                args.ancho,
                args.alto,
                args.hfov_deg,
                args.altura_mm,
                args.cabeceo_deg,
                args.adelante_mm,
            )
        )
        return 0

    if args.desde_json:
        datos = json.loads(Path(args.desde_json).read_text(encoding="utf-8"))
        parejas = [((p[0][0], p[0][1]), (p[1][0], p[1][1])) for p in datos]
    elif args.usar_lidar:
        parejas = capturar_con_lidar(args.config, args.poses, args.segundos)
    else:
        parser.error("elige --cobertura, --usar-lidar o --desde-json")
        return 2

    print(f"\n[i] {len(parejas)} correspondencias")
    try:
        H, medio, maximo = resolver(parejas, args.ancho, args.alto, args.hfov_deg)
    except ErrorHomografia as exc:
        print(f"[-] {exc}")
        return 1

    print(f"[i] Error de reproyeccion: medio {medio:.1f} mm, maximo {maximo:.1f} mm")
    if medio > 60.0:
        print(
            "[-] Error alto. Suele significar postes repartidos en muy poco "
            "angulo, o que el pie de algun blob estaba cortado por el borde."
        )
    print("[i] matrix:")
    print(json.dumps([[round(v, 9) for v in fila] for fila in H.tolist()], indent=2))

    if args.escribir:
        from ronda_nueva.config import cargar_configuracion

        ruta = Path(args.config) if args.config else (
            Path(__file__).resolve().parents[1] / "ronda_nueva" / "configuracion.json"
        )
        config = json.loads(ruta.read_text(encoding="utf-8"))
        config["camera"]["ground_homography"]["matrix"] = [
            [float(v) for v in fila] for fila in H.tolist()
        ]
        config["camera"]["ground_homography"]["ready"] = True
        config["calibration"]["camera_ground_ready"] = True
        ruta.write_text(
            json.dumps(config, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print(f"[+] Escrito en {ruta}")
        cargar_configuracion(str(ruta))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
