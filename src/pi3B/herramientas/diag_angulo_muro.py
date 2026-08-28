#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Diagnostico de solo-percepcion: imprime frontal/izquierda/derecha y el
angulo_muro (triangulado por perp+diag, calculado en lidar_geometria.py
pero nunca usado en navegacion.py) en vivo, una vez por barrido.

NO instancia EnlacePico, NO manda ninguna consigna: el robot no se mueve.
Se puede empujar/cargar el robot a mano con el LiDAR corriendo.
"""
import sys
import time

from lidar_driver import LidarDriver
from lidar_geometria import ProcesadorLidar

corriendo = True
geo = ProcesadorLidar()
ultimo_print = 0.0

def al_barrido(scan):
    global ultimo_print
    med = geo.procesar(scan)
    ahora = time.time()
    if ahora - ultimo_print < 0.3:
        return
    ultimo_print = ahora
    sys.stdout.write(
        f"F={med.frontal:6.0f}  I={med.izquierda:6.0f}  D={med.derecha:6.0f}  "
        f"| perp_izq={med.d_perp_izq:6.0f} diag_izq={med.d_diag_izq:6.0f} "
        f"| perp_der={med.d_perp_der:6.0f} diag_der={med.d_diag_der:6.0f} "
        f"| angulo_muro={med.angulo_muro:+7.2f}\n"
    )
    sys.stdout.flush()

print("[*] Arrancando LiDAR (gira, el robot NO se mueve, no hay enlace con la Pico)...")
lidar = LidarDriver()
try:
    import threading
    threading.Thread(target=lidar.hilo_lectura,
                     args=(lambda: corriendo, al_barrido), daemon=True).start()
    print("[*] Listo. Ctrl+C para salir.")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    corriendo = False
    lidar.cerrar()
    print("\n[*] Detenido.")
