#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sonda estatica de la FSM: con el robot QUIETO, dice que esta percibiendo
y que decidiria la maquina de estados en este instante, sin mandar
ninguna consigna.

Sirve para depurar la evasion sin tener que lanzar una corrida: pone el
robot y el poste donde interesa y esto responde las tres preguntas que
importan para el fallo de "gira hacia el bloque":

  1. Que ve cada sensor (sectores, clusters estrechos, color y su rumbo).
  2. Si CRUCERO entraria en APROXIMACION, y POR QUE via: por tracker
     (LiDAR, sabe donde esta el poste) o por camara (`vision_en_rango`,
     que combina un color cualquiera con la distancia frontal SIN
     comprobar que sean el mismo objeto).
  3. Que angulo mandaria la evasion, con la regla nueva y con la vieja.

No instancia EnlacePico: el robot no se mueve.
"""
import math
import threading
import time

import cv2

import navegacion
import optica
import vision
from camara_driver import CamaraDriver
from lidar_driver import LidarDriver
from lidar_geometria import (ProcesadorLidar, centroide_xy_cluster,
                             es_objeto_estrecho)

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
geo = ProcesadorLidar()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, lambda s: ultimo_scan.__setitem__(0, s)),
                 daemon=True).start()
# El C1 tarda ~2s en arrancar el motor y entregar el primer barrido
# completo; esperar un tiempo fijo se quedaba corto la mitad de las veces.
t0 = time.time()
while ultimo_scan[0] is None and time.time() - t0 < 25:
    time.sleep(0.2)
time.sleep(1.0)          # un par de barridos mas, para que el perfil se asiente

scan = ultimo_scan[0]
corriendo = False
time.sleep(0.4)
lidar.cerrar()
camara.cerrar()

if scan is None:
    raise SystemExit("[-] Sin barrido de LiDAR.")

med = geo.procesar(scan)
import lidar_mascara
lidar_mascara.aplicar(med)

print("\n=== LiDAR: sectores ===")
print("  frontal      %7.0f mm   (frontal_muro %7.0f)" % (med.frontal, med.frontal_muro))
print("  izquierda    %7.0f mm" % med.izquierda)
print("  derecha      %7.0f mm" % med.derecha)
print("  trasera      %7.0f mm  (con mascara del mastil)" % med.trasera)

print("\n=== LiDAR: objetos estrechos delante ===")
estrechos = []
for c in med.clusters_obstaculo:
    if not es_objeto_estrecho(c):
        continue
    x, y = centroide_xy_cluster(c)
    if y <= 0:
        continue
    estrechos.append((math.hypot(x, y), math.degrees(math.atan2(x, y)),
                      optica.rumbo_camara_de_cluster(x, y), x, y))
estrechos.sort()
if not estrechos:
    print("  ninguno")
for d, r_lidar, r_cam, x, y in estrechos:
    print("  dist=%6.0f mm  rumbo_lidar=%+6.1f  rumbo_camara=%+6.1f  (x=%+6.0f y=%6.0f)"
          % (d, r_lidar, r_cam, x, y))

color, cx = vision.get_deteccion()
print("\n=== Camara ===")
if color is None:
    print("  sin color estable")
    rumbo_cam = None
else:
    rumbo_cam = optica.rumbo_de_cx(cx)
    print("  color=%s  cx=%d  ->  rumbo=%+.1f grados" % (color, cx, rumbo_cam))

print("\n=== Que decidiria CRUCERO ===")
vision_en_rango = color is not None and 50.0 < med.frontal < navegacion.DIST_INICIO_EVASION_CAM
print("  vision_en_rango = %s   (color=%s  y  50 < frontal=%.0f < %.0f)"
      % (vision_en_rango, color, med.frontal, navegacion.DIST_INICIO_EVASION_CAM))
if vision_en_rango:
    # Lo de delante, es el poste o es la pared?
    if med.frontal_muro > med.frontal * 1.5:
        print("    lo que bloquea el frente es un OBJETO ESTRECHO")
        print("    (frontal_muro %.0f >> frontal %.0f)" % (med.frontal_muro, med.frontal))
    else:
        print("    [!] lo que bloquea el frente es LA PARED, no el poste")
        print("        (frontal_muro %.0f ~= frontal %.0f). El color puede ser"
              % (med.frontal_muro, med.frontal))
        print("        un poste de mas alla: entraria en evasion por una")
        print("        coincidencia, no porque el poste estorbe.")

if color is not None:
    evadir_izq = (color == "VERDE")
    print("\n=== Que mandaria APROXIMACION (evasion a ciegas) ===")
    print("  color %s -> hay que pasar por %s DEL POSTE"
          % (color, "la IZQUIERDA" if evadir_izq else "la DERECHA"))
    signo_viejo = 1.0 if evadir_izq else -1.0
    ang_viejo = signo_viejo * navegacion.ANGULO_EVASION_CIEGA
    if rumbo_cam is None:
        print("  sin cx: las dos reglas mandan lo mismo, %+.1f" % ang_viejo)
    else:
        signo = -1.0 if evadir_izq else 1.0
        rumbo_obj = rumbo_cam + signo * navegacion.MARGEN_PASO_GRADOS
        ang_nuevo = max(-navegacion.ANGULO_EVASION_CIEGA,
                        min(navegacion.ANGULO_EVASION_CIEGA,
                            -rumbo_obj * navegacion.KP_PURSUIT))
        print("  poste en rumbo %+.1f, hay que apuntar a %+.1f" % (rumbo_cam, rumbo_obj))
        print("  REGLA NUEVA (geometrica): %+6.1f  -> %s"
              % (ang_nuevo, "IZQUIERDA" if ang_nuevo > 1 else
                            ("DERECHA" if ang_nuevo < -1 else "RECTO")))
        print("  REGLA VIEJA (signo fijo): %+6.1f  -> %s"
              % (ang_viejo, "IZQUIERDA" if ang_viejo > 1 else
                            ("DERECHA" if ang_viejo < -1 else "RECTO")))
        if abs(ang_nuevo - ang_viejo) > 2.0:
            print("  >>> LAS DOS REGLAS DISCREPAN en %.0f grados." % abs(ang_nuevo - ang_viejo))
            if (ang_viejo < 0 and rumbo_cam > 0) or (ang_viejo > 0 and rumbo_cam < 0):
                print("      La vieja gira HACIA el bloque (poste en %+.0f, giro %+.0f)."
                      % (rumbo_cam, ang_viejo))
