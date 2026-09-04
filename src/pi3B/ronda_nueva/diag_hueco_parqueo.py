# -*- coding: utf-8 -*-
"""Diagnostico estatico del detector de hueco de parqueo.

Dos corridas de `--solo-parqueo` murieron con "timeout buscando hueco" y
`hueco_confianza` VACIO en los 287 ciclos: el detector no devolvio ni un
candidato. El CSV no distingue "no habia separadores a la vista" de "los
habia y un filtro los descarto", asi que hace falta abrir `_buscar_hueco`
y ver que rechaza cada cluster.

NO instancia EnlacePico y NO manda consignas: el robot no se mueve. Se
coloca a mano donde deberia detectar la bahia y se ejecuta.

Uso (en la Pi, dentro del despliegue):
    python3 -m ronda_nueva.diag_hueco_parqueo --lado derecha
    python3 -m ronda_nueva.diag_hueco_parqueo --lado izquierda --barridos 5
"""
import argparse
import math
import threading
import time

from .config import cargar_configuracion
from .percepcion_lidar import (
    PercepcionLidar,
    _normalizar_barrido,
    _segmentar_normalizado,
)
from .ronda_nueva import _drivers_comunes


def _formato(valor):
    return "%8.1f" % valor if valor is not None else "       -"


def main():
    analizador = argparse.ArgumentParser(description=__doc__)
    analizador.add_argument("--config", default=None)
    analizador.add_argument(
        "--lado",
        choices=("izquierda", "derecha"),
        required=True,
        help="lado donde esta la bahia vista por el robot",
    )
    analizador.add_argument("--barridos", type=int, default=3)
    analizador.add_argument(
        "--volcar",
        action="store_true",
        help="imprime los puntos crudos de cada cluster del lado de la bahia",
    )
    analizador.add_argument(
        "--guardar",
        default=None,
        help="escribe los barridos crudos en un JSON para reproducir el caso "
             "offline y usarlo como fixture de test",
    )
    args = analizador.parse_args()

    lado = -1 if args.lado == "izquierda" else 1
    config = cargar_configuracion(args.config) if args.config else cargar_configuracion()
    lidar_cfg = config["lidar"]
    hardware = config["hardware"]

    # Los umbrales que decidiran cada descarte, impresos antes de medir para
    # que el log de la sesion baste para reinterpretar los numeros despues.
    longitud_min = float(lidar_cfg.get("bay_separator_min_length_mm", 125.0))
    longitud_max = float(lidar_cfg.get("bay_separator_max_length_mm", 290.0))
    separacion_esperada = float(lidar_cfg.get("bay_expected_separation_mm", 353.0))
    tolerancia = float(lidar_cfg.get("bay_separation_tolerance_mm", 105.0))
    lateral_min = float(lidar_cfg.get("bay_lateral_min_mm", 140.0))
    lateral_max = float(lidar_cfg.get("bay_lateral_max_mm", 900.0))
    longitudinal_max = float(lidar_cfg.get("bay_max_longitudinal_mm", 1100.0))
    min_puntos = max(3, int(lidar_cfg.get("object_min_points", 3)))
    cos_max_desvio = math.cos(math.radians(25.0))
    max_residuo = min(55.0, float(lidar_cfg.get("wall_max_residual_mm", 75.0)))

    print("[*] lado de busqueda: %s (%+d)" % (args.lado, lado))
    print("[*] umbrales: longitud %.0f-%.0f mm | lateral %.0f-%.0f mm | "
          "|y| <= %.0f mm" % (longitud_min, longitud_max, lateral_min,
                              lateral_max, longitudinal_max))
    print("[*] separacion esperada %.0f +- %.0f mm | alineacion >= %.3f | "
          "residuo <= %.0f mm" % (separacion_esperada, tolerancia,
                                  cos_max_desvio, max_residuo))
    print()

    LidarDriver, ProcesadorLidar = _drivers_comunes()
    barridos = []

    def al_barrido(scan):
        if len(barridos) < args.barridos:
            barridos.append(list(scan))

    driver = LidarDriver(hardware["lidar_port"], int(hardware["lidar_baudrate"]))
    corriendo = True
    hilo = threading.Thread(
        target=driver.hilo_lectura,
        args=(lambda: corriendo, al_barrido),
        daemon=True,
    )
    print("[*] arrancando LiDAR...")
    hilo.start()

    limite = time.time() + 15.0
    while len(barridos) < args.barridos and time.time() < limite:
        time.sleep(0.1)
    corriendo = False

    if not barridos:
        print("[!] no llego ningun barrido")
        return 1

    if args.guardar:
        import json

        with open(args.guardar, "w", encoding="utf-8") as archivo:
            json.dump(
                {
                    "lado": lado,
                    "barridos": [[[a, d] for a, d in scan] for scan in barridos],
                },
                archivo,
            )
        print("[OK] %d barridos guardados en %s" % (len(barridos), args.guardar))
        print()

    geo = ProcesadorLidar()
    percepcion = PercepcionLidar(config)

    for indice, scan in enumerate(barridos):
        print("=" * 78)
        print("BARRIDO %d  (%d muestras)" % (indice + 1, len(scan)))
        print("=" * 78)

        # La verdad de la prueba: lo que devuelve el detector de verdad, con
        # su persistencia de `bay_confirm_scans`. La tabla siguiente solo
        # explica por que.
        medicion = geo.procesar(scan)
        confirmado = percepcion.procesar(
            scan, medicion, timestamp=10.0 + indice, lado_parqueo=lado
        ).hueco
        if confirmado is not None:
            print("*** HUECO CONFIRMADO: separacion %.0f mm, lateral %.0f mm, "
                  "confianza %.2f" % (confirmado.separacion_mm,
                                      confirmado.distancia_lateral_mm,
                                      confirmado.confianza))
        puntos = _normalizar_barrido(
            scan, float(lidar_cfg.get("max_distance_mm", 4000.0))
        )
        clusters = _segmentar_normalizado(
            puntos,
            float(lidar_cfg.get("abd_factor", 0.04)),
            float(lidar_cfg.get("abd_offset_mm", 40.0)),
            4.0,
            min_puntos,
        )
        # Mismo split que aplica `_buscar_hueco`: sin esto la tabla de abajo
        # no describe lo que el detector real hace con el barrido.
        max_desvio = float(lidar_cfg.get("bay_split_max_deviation_mm", 30.0))
        if max_desvio > 0.0:
            partidos = []
            for cluster in clusters:
                partidos.extend(
                    percepcion._partir_en_rectas(cluster, max_desvio, min_puntos)
                )
            print("clusters: %d  (%d tras partir en rectas a %.0f mm)"
                  % (len(clusters), len(partidos), max_desvio))
            clusters = partidos
        else:
            print("clusters: %d  (split desactivado)" % len(clusters))
        print()
        print("    x_mm     y_mm  lateral  largo  alin  resid  pts  veredicto")

        aceptados = []
        filas = []
        volcados = []
        for cluster in clusters:
            if len(cluster) < min_puntos:
                continue
            segmento = percepcion._segmento_pca(cluster)
            if segmento is None:
                continue
            lateral = lado * segmento.x_mm

            # Solo interesa lo que cae del lado de la bahia; el resto del
            # barrido (pared opuesta, bloque central) llenaria la tabla.
            if lateral < -200.0:
                continue

            motivos = []
            if not (longitud_min <= segmento.longitud_mm <= longitud_max):
                motivos.append("largo")
            if segmento.alineacion < cos_max_desvio:
                motivos.append("alineacion")
            if segmento.residuo_mm > max_residuo:
                motivos.append("residuo")
            if not (lateral_min <= lateral <= lateral_max):
                motivos.append("lateral")
            if abs(segmento.y_mm) > longitudinal_max:
                motivos.append("longitudinal")

            veredicto = "ACEPTADO" if not motivos else "descartado: " + ",".join(motivos)
            filas.append(
                "%s %s %s %6.0f %5.3f %6.1f %4d  %s"
                % (
                    _formato(segmento.x_mm),
                    _formato(segmento.y_mm),
                    _formato(lateral),
                    segmento.longitud_mm,
                    segmento.alineacion,
                    segmento.residuo_mm,
                    segmento.puntos,
                    veredicto,
                )
            )
            if not motivos:
                aceptados.append(segmento)

            # Un cluster con residuo alto y alineacion intermedia puede ser un
            # separador fundido con el muro. La media y la longitud PCA no lo
            # distinguen de un objeto girado: hacen falta los puntos.
            if args.volcar:
                puntos_xy = [
                    (
                        d * math.sin(math.radians(a)),
                        d * math.cos(math.radians(a)),
                        a,
                        d,
                    )
                    for a, d in cluster
                ]
                volcados.append(
                    (segmento.y_mm, segmento.longitud_mm, segmento.alineacion,
                     puntos_xy)
                )

        for fila in sorted(filas):
            print(fila)

        for y_mm, largo, alineacion, puntos_xy in sorted(volcados):
            print()
            print("  puntos del cluster y=%.0f (largo %.0f, alineacion %.3f):"
                  % (y_mm, largo, alineacion))
            print("     ang     dist        x        y")
            for x, y, ang, dist in puntos_xy:
                print("  %6.1f %8.1f %8.1f %8.1f" % (ang, dist, x, y))

        print()
        print("segmentos aceptados: %d" % len(aceptados))
        if len(aceptados) < 2:
            print("-> sin pareja posible; el detector devuelve None")
        else:
            print()
            print("  parejas (separacion esperada %.0f +- %.0f):" %
                  (separacion_esperada, tolerancia))
            for i, primero in enumerate(aceptados):
                for segundo in aceptados[i + 1:]:
                    separacion = abs(segundo.y_mm - primero.y_mm)
                    error = abs(separacion - separacion_esperada)
                    dif_lateral = abs(lado * primero.x_mm - lado * segundo.x_mm)
                    estado = "OK"
                    if error > tolerancia:
                        estado = "separacion fuera por %.0f mm" % (error - tolerancia)
                    elif dif_lateral > tolerancia:
                        estado = "desalineadas por %.0f mm" % dif_lateral
                    print("    y=%.0f / y=%.0f -> sep %.0f mm, dif lat %.0f mm : %s"
                          % (primero.y_mm, segundo.y_mm, separacion, dif_lateral, estado))
        print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
