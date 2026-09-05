"""Comprobacion de pilares, magenta y bahia con la pista ya montada.

NO MUEVE EL ROBOT.  Es la unica pieza de percepcion que no se puede validar sin
poner cosas en el campo, asi que esta herramienta esta hecha para gastar el
menor tiempo posible de pista: se lanza una vez, mira N segundos y contesta a
todo de golpe.

QUE CONTESTA
* ¿Se ven los pilares, y de que color?  Con su posicion en milimetros y de que
  fuente salio (CAMARA, LIDAR o FUSION).
* ¿Cuadra la camara con el LiDAR?  Cuando los dos ven el mismo poste, imprime
  la discrepancia.  Es la unica medida honesta de si la homografia esta bien.
* ¿En que casilla del mapa cae cada uno?  Traduce a (avance, offset) de la
  recta y dice la casilla, que es lo que el planificador va a usar.
* ¿Se ve el cajon de parqueo?  Muros magenta y hueco confirmado por LiDAR.
* ¿Es ESTABLE?  Repite N ciclos y da la tasa de deteccion y la dispersion.

SI ALGO NO SE VE
``--muestrear`` imprime el HSV de las manchas saturadas que hay sobre la lona,
que es exactamente lo que hace falta para reajustar un rango de color sin
adivinar.  Asi se arreglaron el azul y el naranja el 04-09.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from ronda_nueva.config import cargar_configuracion  # noqa: E402
from ronda_nueva.fusion import FusionPilares  # noqa: E402
from ronda_nueva.localizacion import Localizador  # noqa: E402
from ronda_nueva.mapa_pista import MapaPista  # noqa: E402
from ronda_nueva.percepcion_lidar import PercepcionLidar  # noqa: E402
from ronda_nueva.vision_pista import VisionPista  # noqa: E402


def _driver_lidar():
    try:
        from comun.lidar_driver import LidarDriver
    except ImportError:
        from ronda_nueva.ronda_nueva import _driver_lidar as _f

        return _f()
    return LidarDriver


def _titulo(texto: str) -> None:
    print()
    print(texto)
    print("-" * len(texto))


def muestrear_hsv(frame, vision) -> None:
    """HSV de lo que esta saturado y apoyado en la lona.

    Sirve para reajustar un rango que no ve nada: en vez de mover umbrales a
    ciegas, se mira que colores hay de verdad delante del robot.
    """

    import cv2

    hsv = vision._a_hsv(vision.orientar(frame))
    libre = vision.ultimo_espacio_libre
    if libre is None:
        print("  [-] sin poligono de suelo; no se puede acotar la muestra")
        return
    libre = cv2.resize(libre, (hsv.shape[1], hsv.shape[0]), interpolation=cv2.INTER_NEAREST)
    saturado = (hsv[:, :, 1] > 70) & (hsv[:, :, 2] > 50) & (libre > 0)
    # El poligono de suelo se rellena, asi que incluye la cupula del LiDAR:
    # sin quitarla, el muestreo la cuenta como si fuera pista.
    if vision._mascara_robot_completa is not None:
        saturado &= vision._mascara_robot_completa > 0
    total = int(np.count_nonzero(saturado))
    print(f"  pixeles saturados sobre la lona: {total}")
    if total < 200:
        print("  (muy pocos: ¿hay algo de color delante del robot?)")
        return

    h = hsv[:, :, 0][saturado]
    s = hsv[:, :, 1][saturado]
    v = hsv[:, :, 2][saturado]
    conteo = Counter((h // 5 * 5).tolist())
    print("  familias de matiz con mas de 150 pixeles:")
    for matiz in sorted(conteo):
        if conteo[matiz] < 150:
            continue
        sel = (h >= matiz) & (h < matiz + 5)
        print(
            f"    H {matiz:3d}-{matiz + 4:3d}  n={conteo[matiz]:6d}"
            f"  S {np.percentile(s[sel], [5, 50, 95]).round(0)}"
            f"  V {np.percentile(v[sel], [5, 50, 95]).round(0)}"
        )
    print("  (rojo cruza el 0: mira las familias de H bajo Y las de H alto)")


def dibujar(frame, vision, paquete, destino: Path) -> None:
    import cv2

    lienzo = vision.orientar(frame).copy()
    for pilar in paquete.pilares:
        x, y, ancho, alto = pilar.bbox
        color = (0, 0, 255) if pilar.color == "ROJO" else (0, 200, 0)
        cv2.rectangle(lienzo, (x, y), (x + ancho, y + alto), color, 2)
        cv2.putText(
            lienzo,
            f"{pilar.color} {pilar.y_mm:.0f}mm",
            (x, max(14, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )
    for pared in paquete.magenta:
        cv2.putText(
            lienzo,
            f"MAGENTA {pared.y_mm:.0f}mm",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (200, 0, 200),
            2,
            cv2.LINE_AA,
        )
    cv2.imwrite(str(destino), lienzo)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=None)
    parser.add_argument("--segundos", type=float, default=8.0)
    parser.add_argument("--guardar", default=None)
    parser.add_argument("--muestrear", action="store_true")
    parser.add_argument(
        "--sentido",
        type=int,
        default=1,
        choices=(1, -1),
        help="+1 horario (exterior a la izquierda), -1 antihorario",
    )
    parser.add_argument(
        "--lado-bahia",
        type=int,
        default=0,
        choices=(-1, 0, 1),
        help="-1 izquierda, +1 derecha; 0 no busca el hueco",
    )
    args = parser.parse_args(argv)

    config = cargar_configuracion(args.config)
    from ronda_nueva.hardware import FuenteCamara

    vision = VisionPista(config)
    percepcion = PercepcionLidar(config)
    fusion = FusionPilares(config)
    localizador = Localizador(config)
    mapa = MapaPista(config)

    seguir = threading.Event()
    seguir.set()
    ultimo = {"frame": None, "paquete": None, "scan": None, "t": 0.0}

    def al_frame(frame, timestamp):
        ultimo["paquete"] = vision.procesar(frame, timestamp)
        ultimo["frame"] = frame

    def al_barrido(scan, timestamp):
        ultimo["scan"] = list(scan)
        ultimo["t"] = timestamp

    camara = FuenteCamara(config["camera"])
    threading.Thread(
        target=camara.bucle, args=(seguir.is_set, al_frame), daemon=True
    ).start()
    LidarDriver = _driver_lidar()
    lidar = LidarDriver(
        config["hardware"]["lidar_port"],
        int(config["hardware"].get("lidar_baudrate", 460800)),
    )
    threading.Thread(
        target=lidar.hilo_lectura, args=(seguir.is_set, al_barrido), daemon=True
    ).start()

    limite = time.monotonic() + 12.0
    while time.monotonic() < limite:
        if ultimo["paquete"] is not None and ultimo["scan"] is not None:
            break
        time.sleep(0.1)

    historial = defaultdict(list)
    ciclos = 0
    discrepancias = []
    magentas = 0
    huecos = 0
    ultima_lectura = None

    fin = time.monotonic() + max(2.0, args.segundos)
    while time.monotonic() < fin:
        paquete = ultimo["paquete"]
        scan = ultimo["scan"]
        if paquete is None or scan is None:
            time.sleep(0.05)
            continue
        paredes, objetos, hueco = percepcion.procesar(
            scan, ultimo["t"], lado_parqueo=args.lado_bahia
        )
        pilares = fusion.asociar(paquete.pilares, objetos, ultimo["t"])
        ciclos += 1
        ultima_lectura = (paredes, objetos, pilares, paquete, hueco)

        for pilar in pilares:
            if not pilar.color:
                continue
            historial[pilar.color].append((pilar.x_mm, pilar.y_mm, pilar.fuente))
        # Camara contra LiDAR sobre el MISMO poste: la unica medida honesta.
        for visual in paquete.pilares:
            for objeto in objetos:
                separacion = float(
                    np.hypot(visual.x_mm - objeto.x_mm, visual.y_mm - objeto.y_mm)
                )
                if separacion < 260.0:
                    discrepancias.append((visual.y_mm, separacion))
        magentas += len(paquete.magenta)
        huecos += 1 if hueco is not None else 0
        time.sleep(0.1)

    seguir.clear()
    try:
        lidar.cerrar()
    except Exception:
        pass

    if ultima_lectura is None:
        print("[-] no llegaron datos de los dos sensores")
        return 1
    paredes, objetos, pilares, paquete, hueco = ultima_lectura

    _titulo(f"ULTIMO CICLO  ({ciclos} ciclos analizados)")
    for nombre in ("frontal", "izquierda", "derecha", "trasera"):
        recta = getattr(paredes, nombre)
        print(
            f"  pared {nombre:10s} "
            + ("no encontrada" if recta is None else f"{recta.distancia_mm:7.0f} mm")
        )

    print(f"  lineas de piso   {[(l.color, round(l.x_mm), round(l.y_mm)) for l in paquete.lineas]}")
    print(f"  objetos LiDAR    {len(objetos)}")
    print(f"  blobs de camara  {len(paquete.pilares)}")
    print(f"  pilares fusionados {len(pilares)}")
    for pilar in pilares:
        avance = localizador.avance_de_punto(pilar.x_mm, pilar.y_mm, paredes)
        offset = localizador.offset_de_punto(
            pilar.x_mm, pilar.y_mm, paredes, args.sentido
        )
        casilla = ""
        if avance is not None and offset is not None:
            coordenadas = mapa.coordenadas_de_pilar(avance, offset)
            en_recta = mapa.pertenece_a_esta_recta(avance, offset)
            if coordenadas is not None:
                salto, avance_recta, offset_recta = coordenadas
                indice = mapa._indice_por_avance(avance_recta)
                casilla = (
                    f"  -> recta {'siguiente' if salto else 'actual'}"
                    f" avance {avance_recta:.0f} offset {offset_recta:.0f}"
                    f" casilla {'?' if indice is None else indice}"
                    f"{'' if en_recta else '  (es de la recta de al lado)'}"
                )
        print(
            f"    {pilar.color or '(sin color)':11s} x={pilar.x_mm:7.0f} y={pilar.y_mm:7.0f}"
            f"  {pilar.fuente:7s} conf {pilar.confianza:.2f}{casilla}"
        )

    _titulo("ESTABILIDAD")
    if not historial:
        print("  [-] NINGUN pilar con color en todo el barrido de tiempo.")
        print("      Revisa los rangos HSV: lanza otra vez con --muestrear.")
    for color, muestras in historial.items():
        xs = [m[0] for m in muestras]
        ys = [m[1] for m in muestras]
        fuentes = Counter(m[2] for m in muestras)
        print(
            f"  {color:6s} {len(muestras):4d} detecciones en {ciclos} ciclos"
            f"  x {statistics.mean(xs):7.0f} +-{(statistics.pstdev(xs) if len(xs) > 1 else 0):5.0f}"
            f"  y {statistics.mean(ys):7.0f} +-{(statistics.pstdev(ys) if len(ys) > 1 else 0):5.0f}"
            f"  fuentes {dict(fuentes)}"
        )

    if discrepancias:
        separaciones = [d[1] for d in discrepancias]
        print()
        print(
            f"  camara vs LiDAR sobre el mismo poste: {len(separaciones)} parejas,"
            f" mediana {statistics.median(separaciones):.0f} mm,"
            f" max {max(separaciones):.0f} mm"
        )
        print("  (por debajo de ~80 mm la homografia esta fina)")

    _titulo("PARQUEO")
    print(f"  muros magenta vistos en {magentas} ciclos")
    for pared in paquete.magenta:
        print(f"    x={pared.x_mm:7.0f}  y={pared.y_mm:7.0f}")
    if args.lado_bahia:
        print(f"  hueco confirmado por LiDAR en {huecos} de {ciclos} ciclos")
        if huecos == 0:
            print(
                "    (si el robot esta DENTRO del cajon esto es lo correcto: el"
                " detector busca un hueco AL LADO, no debajo)"
            )
        if hueco is not None:
            print(
                f"    separacion {hueco.separacion_mm:.0f} mm"
                f"  lateral {hueco.distancia_lateral_mm:.0f} mm"
                f"  confianza {hueco.confianza:.2f}"
            )
    else:
        print("  (usa --lado-bahia -1 o 1 para buscar el hueco con el LiDAR)")

    if args.muestrear:
        _titulo("MUESTREO HSV")
        muestrear_hsv(ultimo["frame"], vision)

    if args.guardar:
        dibujar(ultimo["frame"], vision, paquete, Path(args.guardar))
        print()
        print(f"[+] imagen anotada en {args.guardar}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
