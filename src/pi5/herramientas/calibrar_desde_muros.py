"""Calibra la homografia del suelo con las PAREDES, sin colocar nada.

LA IDEA
La linea donde un muro toca la lona es, por definicion, una recta del plano del
suelo. El LiDAR mide esa recta en milimetros (distancia perpendicular y angulo)
y la camara la ve como el borde superior de la region de suelo. Emparejarlas da
la calibracion sin sacar un solo pilar ni medir nada con regla.

Frente a la calibracion con pilares tiene tres ventajas practicas:

* No hace falta colocar nada: basta con poner el robot en la pista.
* No da cuatro correspondencias, da CIENTOS -- una por columna de la imagen --,
  repartidas por todo el ancho del cuadro, que es justo lo que le falta a una
  homografia ajustada con cuatro puntos apiñados en el centro.
* El sesgo del centroide (el LiDAR ve la cara frontal del pilar y la camara la
  silueta) no existe: el muro es plano y su base es la misma linea para los dos.

COMO SE AJUSTA
No se resuelve por DLT directo, porque las correspondencias no son punto a
punto: se sabe que el pixel cae SOBRE una recta, no en que sitio de la recta.
Asi que se ajustan los tres parametros del montaje -- altura, cabeceo y
guiñada -- minimizando la distancia de cada punto proyectado a la recta mas
cercana. Tres parametros contra cientos de observaciones repartidas en tres
paredes con orientaciones distintas: el problema esta muy sobredeterminado.

La perdida esta acotada (Huber) porque la parte alta del cuadro tiene la pared
de la sala, sombras y el muro del otro extremo del campo, y esos puntos no caen
sobre ninguna de las tres rectas.

USO
    cd /home/pi/wro_pi5_...
    python3 herramientas/calibrar_desde_muros.py --escribir

Conviene repetirlo desde DOS poses distintas y comprobar que sale lo mismo: si
la altura cambia 30 mm entre poses, algo no esta midiendo lo que se cree.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

# Centro optico que se usa al ajustar.  TIENE que ser el mismo que use la
# ronda al reconstruir la homografia: se ajusto con 640 (el nominal) mientras
# la ronda reconstruia con 704,148 (el medido), y esos 64 pixeles son casi
# cuatro grados de guiñada -- 48 mm de error lateral sobre un poste a 700 mm.
PRINCIPAL = [None, None]

from ronda_nueva.config import cargar_configuracion  # noqa: E402
from ronda_nueva.geometria_suelo import ProyectorSuelo, desde_montaje  # noqa: E402
from ronda_nueva.percepcion_lidar import PercepcionLidar  # noqa: E402
from ronda_nueva.vision_pista import VisionPista  # noqa: E402


def borde_superior_del_suelo(
    libre: np.ndarray, paso: int = 4, margen_inferior: int = 12
) -> List[Tuple[float, float]]:
    """Primer pixel de suelo de cada columna, mirando de abajo hacia arriba.

    Se recorre desde abajo y se para en el primer hueco: asi el borde que sale
    es el del suelo CONTINUO bajo el robot, y no el de un trozo de lona que se
    vea por encima de una pared.
    """

    alto, ancho = libre.shape
    puntos: List[Tuple[float, float]] = []
    for u in range(0, ancho, paso):
        columna = libre[:, u]
        v = alto - 1 - margen_inferior
        while v >= 0 and columna[v]:
            v -= 1
        if v < 0 or v >= alto - margen_inferior - 2:
            continue
        puntos.append((float(u), float(v + 1)))
    return puntos


def _rectas_de_pared(paredes) -> List[Tuple[float, float, float]]:
    """(nx, ny, d) de cada pared vista, en el marco del robot."""

    salida = []
    for nombre in ("frontal", "izquierda", "derecha", "trasera"):
        recta = getattr(paredes, nombre)
        if recta is None:
            continue
        rad = math.radians(recta.angulo_deg)
        salida.append((math.sin(rad), math.cos(rad), recta.distancia_mm))
    return salida


def residuo(
    parametros: Tuple[float, float, float],
    puntos: Sequence[Tuple[float, float]],
    rectas: Sequence[Tuple[float, float, float]],
    ancho: int,
    alto: int,
    hfov_deg: float,
    adelante_mm: float,
    corte_px: float,
) -> Tuple[float, int, float]:
    """Error en PIXELES entre el borde observado y el que predice el montaje.

    POR QUE EN PIXELES Y NO EN MILIMETROS
    La primera version media la distancia del punto proyectado a la recta del
    muro, en el suelo.  Esta mal condicionado: cerca del horizonte un pixel vale
    mas de 100 mm, asi que dos filas de error en la parte alta del cuadro pesan
    lo mismo que veinte centimetros de error real y el ajuste se va detras del
    ruido.  El ruido esta en la imagen -- la mascara de suelo se equivoca de un
    pixel o dos --, asi que la comparacion tiene que hacerse ahi.

    Para cada columna se lanza el rayo del suelo desde la camara, se corta con
    la pared mas cercana que tenga delante, y ese punto se vuelve a proyectar a
    la imagen: la fila que sale es la que el borde deberia tener.
    """

    altura, cabeceo, guinada = parametros
    if not (60.0 < altura < 900.0) or not (3.0 < cabeceo < 80.0):
        return float("inf"), 0, float("inf")
    try:
        H = desde_montaje(
            ancho,
            alto,
            hfov_deg,
            altura,
            cabeceo,
            guinada,
            adelante_mm=adelante_mm,
            principal_x_px=PRINCIPAL[0],
            principal_y_px=PRINCIPAL[1],
        )
    except Exception:
        return float("inf"), 0, float("inf")
    proyector = ProyectorSuelo(H, ancho, alto, alcance_max_mm=1e9, avance_min_mm=-1e9)

    origen = np.array([0.0, adelante_mm])
    columnas = np.asarray([u for u, _v in puntos], dtype=float)
    filas = np.asarray([v for _u, v in puntos], dtype=float)

    # Direccion del rayo de suelo de cada columna: dos filas bastan, porque en
    # el plano del suelo la imagen de una columna es una recta.
    base = proyector.proyectar_muchos(
        np.column_stack((columnas, np.full_like(columnas, alto - 1.0)))
    )
    medio = proyector.proyectar_muchos(
        np.column_stack((columnas, np.full_like(columnas, alto * 0.6)))
    )
    direccion = medio - base
    normas = np.hypot(direccion[:, 0], direccion[:, 1])
    utilizable = np.isfinite(normas) & (normas > 1e-6)
    direccion[utilizable] /= normas[utilizable, None]

    errores = np.full(len(puntos), corte_px)
    for indice in np.nonzero(utilizable)[0]:
        inicio = base[indice]
        if not np.all(np.isfinite(inicio)):
            continue
        mejor_t = None
        for nx, ny, d in rectas:
            denominador = direccion[indice, 0] * nx + direccion[indice, 1] * ny
            if abs(denominador) < 1e-9:
                continue
            t = (d - (inicio[0] * nx + inicio[1] * ny)) / denominador
            if t > 1.0 and (mejor_t is None or t < mejor_t):
                mejor_t = t
        if mejor_t is None:
            continue
        choque = inicio + mejor_t * direccion[indice]
        pixel = proyector.punto_imagen(float(choque[0]), float(choque[1]))
        if pixel is None:
            continue
        errores[indice] = min(abs(pixel[1] - filas[indice]), corte_px)

    dentro = int(np.count_nonzero(errores < corte_px))
    mediana = float(np.median(errores[errores < corte_px])) if dentro else float("inf")
    return float(errores.mean()), dentro, mediana


def ajustar(
    puntos, rectas, ancho, alto, hfov_deg, adelante_mm, corte_px=25.0, altura_fija=None
):
    """Rejilla gruesa y despues refinado: tres parametros, sin dependencias.

    Con ``altura_fija`` -- la que se mide con regla en treinta segundos -- solo
    quedan dos incognitas y el ajuste deja de tener el valle en el que altura y
    cabeceo se compensan.
    """

    alturas = [float(altura_fija)] if altura_fija else list(np.arange(120.0, 700.0, 20.0))
    mejor = None
    for altura in alturas:
        for cabeceo in np.arange(8.0, 60.0, 2.0):
            valor, dentro, mediana = residuo(
                (altura, cabeceo, 0.0), puntos, rectas, ancho, alto, hfov_deg,
                adelante_mm, corte_px,
            )
            if mejor is None or valor < mejor[0]:
                mejor = (valor, altura, cabeceo, 0.0, dentro, mediana)

    _valor, altura, cabeceo, guinada, _dentro, _mediana = mejor
    paso = (0.0 if altura_fija else 10.0, 1.0, 2.0)
    for _ronda in range(7):
        mejoro = False
        for indice, delta in enumerate(paso):
            if delta <= 0.0:
                continue
            for signo in (1.0, -1.0):
                candidato = [altura, cabeceo, guinada]
                candidato[indice] += signo * delta
                valor, dentro, mediana = residuo(
                    tuple(candidato), puntos, rectas, ancho, alto, hfov_deg,
                    adelante_mm, corte_px,
                )
                if valor < mejor[0] - 1e-6:
                    mejor = (valor, *candidato, dentro, mediana)
                    altura, cabeceo, guinada = candidato
                    mejoro = True
        if not mejoro:
            paso = tuple(p / 2.0 for p in paso)
            if max(paso) < 0.05:
                break
    return mejor


def dibujar_verificacion(frame, rectas, H, ancho, alto, puntos, destino: Path) -> None:
    """Superpone al cuadro lo que la homografia CREE que hay en el suelo.

    Si las lineas dibujadas caen sobre la base de los muros y la rejilla sigue
    las juntas de la lona, la calibracion es buena.  Es la comprobacion que
    convence: un residuo en pixeles se puede discutir, una linea que cae dos
    centimetros fuera de la pared se ve.
    """

    import cv2

    proyector = ProyectorSuelo(H, ancho, alto, alcance_max_mm=6000.0, avance_min_mm=0.0)
    lienzo = frame.copy()

    # Rejilla del suelo cada 250 mm.
    for y in range(250, 3001, 250):
        traza = []
        for x in range(-1500, 1501, 50):
            pixel = proyector.punto_imagen(float(x), float(y))
            if pixel and -50 < pixel[0] < ancho + 50 and 0 <= pixel[1] < alto:
                traza.append((int(pixel[0]), int(pixel[1])))
        for a, b in zip(traza, traza[1:]):
            cv2.line(lienzo, a, b, (90, 90, 90), 1)

    # Las rectas que el LiDAR midio, proyectadas al cuadro.
    for nx, ny, d in rectas:
        traza = []
        for t in range(-3000, 3001, 25):
            x = nx * d - ny * t
            y = ny * d + nx * t
            # Solo lo que esta DELANTE: sin este filtro, la prolongacion de
            # una pared lateral por detras del robot vuelve a entrar en el
            # cuadro y dibuja un aspa que no significa nada.
            if y < 150.0:
                continue
            pixel = proyector.punto_imagen(x, y)
            if pixel and -50 < pixel[0] < ancho + 50 and 0 <= pixel[1] < alto:
                traza.append((int(pixel[0]), int(pixel[1])))
        for a, b in zip(traza, traza[1:]):
            cv2.line(lienzo, a, b, (0, 0, 255), 2)

    # El borde de suelo que la camara detecto.
    for u, v in puntos:
        cv2.circle(lienzo, (int(u), int(v)), 1, (0, 255, 255), -1)

    cv2.imwrite(str(destino), lienzo)


def capturar(config, segundos: float):
    from ronda_nueva.hardware import FuenteCamara

    try:
        from comun.lidar_driver import LidarDriver
    except ImportError:
        from ronda_nueva.ronda_nueva import _driver_lidar

        LidarDriver = _driver_lidar()

    # La vision se instancia SIN homografia: aqui se usan pixeles.
    config_sin = json.loads(json.dumps(config))
    config_sin["camera"]["ground_homography"] = {"ready": False}
    vision = VisionPista(config_sin)
    percepcion = PercepcionLidar(config)

    seguir = threading.Event()
    seguir.set()
    ultimo = {"libre": None, "scan": None, "t": 0.0, "frame": None}

    def al_frame(frame, timestamp):
        vision.procesar(frame, timestamp)
        ultimo["libre"] = vision.ultimo_espacio_libre
        ultimo["frame"] = vision.orientar(frame)

    def al_barrido(scan, timestamp):
        ultimo["scan"] = list(scan)
        ultimo["t"] = timestamp

    camara = FuenteCamara(config["camera"])
    threading.Thread(
        target=camara.bucle, args=(seguir.is_set, al_frame), daemon=True
    ).start()
    lidar = LidarDriver(
        config["hardware"]["lidar_port"],
        int(config["hardware"].get("lidar_baudrate", 460800)),
    )
    threading.Thread(
        target=lidar.hilo_lectura, args=(seguir.is_set, al_barrido), daemon=True
    ).start()

    # Esperar a tener las DOS cosas, no un tiempo fijo: el LiDAR tarda lo
    # suyo en arrancar el motor y sincronizar, y la camara en bloquear la
    # exposicion.  Un sleep corto daba "no entrego ningun barrido" cuando lo
    # unico que pasaba era que aun no habia llegado.
    limite = time.monotonic() + max(6.0, segundos + 6.0)
    while time.monotonic() < limite:
        if ultimo["libre"] is not None and ultimo["scan"] is not None:
            break
        time.sleep(0.1)
    # Un poco mas para quedarse con un cuadro y un barrido ya estabilizados.
    time.sleep(max(1.0, segundos))
    seguir.clear()
    time.sleep(0.3)
    try:
        lidar.cerrar()
    except Exception:
        pass

    if ultimo["libre"] is None:
        raise SystemExit("[-] la camara no entrego ningun cuadro")
    if ultimo["scan"] is None:
        raise SystemExit("[-] el LiDAR no entrego ningun barrido")
    paredes, _objetos, _hueco = percepcion.procesar(ultimo["scan"], ultimo["t"])
    return ultimo["libre"], paredes, ultimo["frame"]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=None)
    parser.add_argument("--segundos", type=float, default=4.0)
    parser.add_argument("--paso-columnas", type=int, default=4)
    parser.add_argument("--adelante-mm", type=float, default=None)
    parser.add_argument("--escribir", action="store_true")
    parser.add_argument(
        "--guardar-captura",
        default=None,
        help="guarda borde y paredes en JSON para reajustar sin el robot",
    )
    parser.add_argument(
        "--desde-captura", default=None, help="reajusta sobre un JSON ya guardado"
    )
    parser.add_argument(
        "--dibujar",
        default=None,
        help="guarda el cuadro con las paredes del LiDAR y una rejilla encima",
    )
    parser.add_argument(
        "--altura-mm",
        type=float,
        default=None,
        help="fija la altura medida con regla y ajusta solo cabeceo y guiñada",
    )
    args = parser.parse_args(argv)

    config = cargar_configuracion(args.config)
    PRINCIPAL[0] = config["camera"].get("principal_x_px")
    PRINCIPAL[1] = config["camera"].get("principal_y_px")
    ancho = int(config["camera"]["width"])
    alto = int(config["camera"]["height"])
    hfov = float(config["camera"]["hfov_deg"])
    adelante = (
        args.adelante_mm
        if args.adelante_mm is not None
        else float(config["camera"]["ground_homography"].get("forward_mm", -80.0))
    )

    if args.desde_captura:
        datos = json.loads(Path(args.desde_captura).read_text(encoding="utf-8"))
        puntos = [tuple(p) for p in datos["puntos"]]
        rectas = [tuple(r) for r in datos["rectas"]]
        print(f"[i] captura leida: {len(puntos)} puntos, {len(rectas)} paredes")
    else:
        libre, paredes, frame = capturar(config, args.segundos)
        rectas = _rectas_de_pared(paredes)
        print(f"[i] paredes vistas: {len(rectas)}")
        for nombre in ("frontal", "izquierda", "derecha", "trasera"):
            recta = getattr(paredes, nombre)
            if recta is not None:
                print(
                    f"    {nombre:10s} {recta.distancia_mm:7.0f} mm"
                    f"  normal {recta.angulo_deg:+7.1f} deg"
                )
        puntos = borde_superior_del_suelo(libre > 0, paso=args.paso_columnas)
        if args.guardar_captura:
            Path(args.guardar_captura).write_text(
                json.dumps({"puntos": puntos, "rectas": rectas}), encoding="utf-8"
            )
            print(f"[+] captura guardada en {args.guardar_captura}")

    if len(rectas) < 2:
        print("[-] hacen falta al menos dos paredes. Coloca el robot en la pista.")
        return 1
    print(f"[i] puntos de borde suelo/pared: {len(puntos)}")
    if len(puntos) < 40:
        print("[-] muy pocos: revisa floor_ranges y floor_seed_norm.")
        return 1

    valor, altura, cabeceo, guinada, dentro, mediana = ajustar(
        puntos, rectas, ancho, alto, hfov, adelante, altura_fija=args.altura_mm
    )
    print()
    print(f"  altura      {altura:7.1f} mm")
    print(f"  cabeceo     {cabeceo:7.2f} deg")
    print(f"  guiñada     {guinada:+7.2f} deg")
    print(f"  residuo     {valor:7.2f} px de media, {mediana:.2f} px de mediana")
    print(f"  puntos utiles {dentro} de {len(puntos)}")

    if dentro < 0.45 * len(puntos) or mediana > 6.0:
        print()
        print("[-] El ajuste no es fiable. Causas tipicas:")
        print("    - el robot no esta dentro de la pista")
        print("    - floor_ranges deja fuera parte de la lona")
        print("    - hay algo apoyado contra un muro que rompe el borde")
        return 1

    H = desde_montaje(
        ancho, alto, hfov, altura, cabeceo, guinada,
        adelante_mm=adelante,
        principal_x_px=PRINCIPAL[0],
        principal_y_px=PRINCIPAL[1],
    )
    proyector = ProyectorSuelo(H, ancho, alto)
    cerca = proyector.punto_suelo(ancho / 2.0, alto - 1)
    print()
    if cerca:
        print(f"  suelo visible desde {cerca[1]:.0f} mm por delante")

    if args.dibujar and not args.desde_captura:
        dibujar_verificacion(
            frame, rectas, H, ancho, alto, puntos, Path(args.dibujar)
        )
        print(f"[+] verificacion visual en {args.dibujar}")

    if args.escribir:
        ruta = Path(args.config) if args.config else (
            Path(__file__).resolve().parents[1] / "ronda_nueva" / "configuracion.json"
        )
        datos = json.loads(ruta.read_text(encoding="utf-8"))
        suelo = datos["camera"]["ground_homography"]
        suelo["height_mm"] = round(float(altura), 1)
        suelo["pitch_deg"] = round(float(cabeceo), 2)
        suelo["yaw_deg"] = round(float(guinada), 2)
        suelo["forward_mm"] = float(adelante)
        # Se escribe la MATRIZ, no solo el montaje: asi la ronda usa
        # exactamente la homografia que se ajusto, sin depender de que
        # reconstruya con los mismos parametros de optica.
        suelo["matrix"] = [[float(v) for v in fila] for fila in H.tolist()]
        suelo["ready"] = True
        datos["calibration"]["camera_ground_ready"] = True
        ruta.write_text(
            json.dumps(datos, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print(f"[+] escrito en {ruta}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
