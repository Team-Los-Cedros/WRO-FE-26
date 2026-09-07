#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sonda estatica de la FSM: con el robot QUIETO, dice que esta percibiendo
y que decidiria la maquina de estados en este instante, sin mandar
ninguna consigna.

Es la herramienta de VERIFICACION DE SIGNOS del diseño nuevo (ver
DISENO_CURVAS.md). Colocando el robot y un pilar a mano responde las
preguntas que no se pueden contestar leyendo codigo:

  1. Que ve cada sensor (sectores, clusters, color y su rumbo).
  2. Que lado obligatorio sale del color, y en que lado esta el pilar.
  3. Si existe algun comando ejecutable que deje el pilar de su lado, o
     si hay que abrir primero.
  4. El conjunto de comandos que las paredes permiten, y cual elegiria
     el arbitraje.

No instancia EnlacePico: el robot no se mueve.
"""
import math
import threading
import time

import geometria_evasion as gev
import geometria_robot as geo
import navegacion
import optica
import vision
from camara_driver import CamaraDriver
from lidar_driver import LidarDriver
from lidar_geometria import (ProcesadorLidar, ancho_cluster,
                             centroide_xy_cluster, es_objeto_estrecho)

corriendo = True
ultimo_frame = [None]
ultimo_scan = [None]

camara = CamaraDriver()
threading.Thread(target=camara.hilo_captura,
                 args=(lambda: corriendo, lambda f: (ultimo_frame.__setitem__(0, f),
                                                     vision.procesar_frame(f))),
                 daemon=True).start()
time.sleep(2.5)
lidar = LidarDriver()
proc = ProcesadorLidar()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, lambda s: ultimo_scan.__setitem__(0, s)),
                 daemon=True).start()
# El C1 tarda ~2s en arrancar el motor y entregar el primer barrido
t0 = time.time()
while ultimo_scan[0] is None and time.time() - t0 < 25:
    time.sleep(0.2)
time.sleep(1.0)

scan = ultimo_scan[0]
corriendo = False
time.sleep(0.4)
lidar.cerrar()
camara.cerrar()

if scan is None:
    raise SystemExit("[-] Sin barrido de LiDAR.")

med = proc.procesar(scan)
import lidar_mascara            # noqa: E402
lidar_mascara.aplicar(med)

print("\n=== LiDAR: sectores ===")
print("  frontal      %7.0f mm   (frontal_muro %7.0f)" % (med.frontal, med.frontal_muro))
print("  izquierda    %7.0f mm" % med.izquierda)
print("  derecha      %7.0f mm" % med.derecha)
print("  trasera      %7.0f mm  (con mascara del mastil)" % med.trasera)
print("  angulo_muro  %7.1f grados" % med.angulo_muro)

print("\n=== LiDAR: objetos estrechos delante ===")
estrechos = []
for c in med.clusters_obstaculo:
    if not es_objeto_estrecho(c):
        continue
    x, y = centroide_xy_cluster(c)
    if y <= 0:
        continue
    estrechos.append((math.hypot(x, y), gev.rumbo(x, y),
                      optica.rumbo_camara_de_cluster(x, y), x, y,
                      ancho_cluster(c), len(c)))
estrechos.sort()
if not estrechos:
    print("  ninguno")
    print("  (si hay un pilar justo delante y no aparece, comprueba el cierre")
    print("   del circulo en segmentar_clusters_abd: el barrido se corta en 0)")
for d, r_lidar, r_cam, x, y, ancho, n in estrechos:
    print("  dist=%6.0f mm  rumbo_lidar=%+6.1f  rumbo_camara=%+6.1f  "
          "(x=%+6.0f y=%6.0f)  ancho=%4.0fmm  %d pts"
          % (d, r_lidar, r_cam, x, y, ancho, n))

color, cx = vision.get_deteccion()
print("\n=== Camara ===")
if color is None:
    print("  sin color estable")
    rumbo_cam = None
else:
    rumbo_cam = optica.rumbo_de_cx(cx)
    print("  color=%s  cx=%d  ->  rumbo=%+.1f grados  (FOV util +-%.0f)"
          % (color, cx, rumbo_cam, optica.HFOV_EFECTIVO / 2.0))

print("\n=== Regla ===")
if color is None:
    print("  sin color no hay regla que aplicar: el pilar es solo un obstaculo")
else:
    s = navegacion.lado_obligatorio(color)
    print("  %s -> el pilar debe quedar a la %s del robot (s_lado=%+d)"
          % (color, "DERECHA" if s > 0 else "IZQUIERDA", s))
    print("  o sea, el robot pasa por la %s DEL PILAR"
          % ("IZQUIERDA" if s > 0 else "DERECHA"))

    if estrechos:
        # El pilar apareado por rumbo, igual que _intentar_capturar
        if rumbo_cam is not None:
            en_rumbo = [e for e in estrechos
                        if abs(e[2] - rumbo_cam) <= optica.TOLERANCIA_APAREO_GRADOS]
            elegido = min(en_rumbo, key=lambda e: abs(e[2] - rumbo_cam)) if en_rumbo else None
        else:
            elegido = estrechos[0]

        if elegido is None:
            print("\n  [!] la camara ve %s pero ningun cluster esta en ese rumbo:"
                  % color)
            print("      no se capturaria objetivo (no se inventa el apareo)")
        else:
            x_l, y_l = elegido[3], elegido[4]
            x_r, y_r = geo.lidar_a_eje_trasero(x_l, y_l)
            u = s * x_r
            holgura = navegacion.HOLGURA_BASE + navegacion.HOLGURA_POR_SIGMA * 30.0
            print("\n=== Geometria de la maniobra ===")
            print("  pilar en marco EJE TRASERO: x=%+.0f y=%.0f mm" % (x_r, y_r))
            print("  separacion por el lado bueno: %+.0f mm  (minimo %.0f)"
                  % (u, navegacion.SEPARACION_MIN_VALIDA))
            req = gev.separacion_requerida(y_r, holgura)
            print("  separacion requerida para poder envolver: %s"
                  % ("no alcanzable todavia, hay que avanzar" if req is None
                     else "%.0f mm" % req))
            factible, radio = gev.radio_envolvente(x_r, y_r, s, holgura)
            if factible:
                cmd = gev.comando_de_radio(radio, hacia_izquierda=(s < 0))
                print("  ENVOLVENTE factible: R=%.0fmm -> comando %+.1f (%s)"
                      % (radio, cmd, "IZQUIERDA" if cmd > 0 else "DERECHA"))
            else:
                cmd = gev.comando_apertura(x_r, y_r, s,
                                           (req or navegacion.SEPARACION_MIN_VALIDA)
                                           + navegacion.MARGEN_APERTURA)
                print("  ENVOLVENTE no factible -> APERTURA con comando %+.1f (%s)"
                      % (cmd, "IZQUIERDA" if cmd > 0 else "DERECHA"))
                print("  (abrir y envolver tienen SIGNOS OPUESTOS: es lo que hay")
                print("   que ver en pista para validar la convencion)")

            nav = navegacion.Navegador(proc)
            nav.tracker.iniciar(color, s, x_l, y_l, 0.0)
            seguros, horizonte = nav._comandos_seguros(med, navegacion.VELOCIDAD_EVASION)
            compat = nav._compatibles(seguros, (x_r, y_r), holgura, "lado")
            print("\n=== Arbitraje ===")
            print("  horizonte de prediccion: %.0f mm" % horizonte)
            print("  comandos que las PAREDES permiten (%d de %d): %s"
                  % (len(seguros), len(navegacion.CANDIDATOS),
                     "ninguno -> marcha atras" if not seguros
                     else "[%+.1f .. %+.1f]" % (min(seguros), max(seguros))))
            print("  de esos, los que dejan el pilar de su lado (%d): %s"
                  % (len(compat),
                     "ninguno -> hay que ABRIR" if not compat
                     else "[%+.1f .. %+.1f]" % (min(compat), max(compat))))
            if compat and all(c * s < 0 for c in compat):
                print("  todos giran HACIA el pilar: la unica salida es envolver")
