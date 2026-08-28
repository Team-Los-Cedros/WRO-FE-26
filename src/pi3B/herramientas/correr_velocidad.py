#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Envoltorio no interactivo de calibracion/medir_velocidad.py: una tirada
por invocacion, para poder coordinar la colocacion del robot entre una y
otra. Reutiliza tal cual la logica de medida del script original
(ajuste por minimos cuadrados, descarte de la rampa, tau de arranque);
lo unico que reemplaza es el input() por una cuenta atras, porque por
SSH no hay stdin.

Uso:  python3 correr_velocidad.py <pwm> [etiqueta]
"""
import builtins
import json
import os
import sys
import threading
import time

PWM = int(sys.argv[1]) if len(sys.argv) > 1 else 55
ETIQUETA = sys.argv[2] if len(sys.argv) > 2 else "bateria_llena"

# La cuenta atras sustituye al input() del script original
def _cuenta_atras(prompt=""):
    if prompt:
        print(prompt)
    for i in (3, 2, 1):
        print(f"    lanzando en {i}...", flush=True)
        time.sleep(1.0)
    return ""

builtins.input = _cuenta_atras

import medir_velocidad as mv
from lidar_driver import LidarDriver
from enlace_pico import EnlacePico

enlace = EnlacePico()
enlace.enviar(0, 0.0)

medidor = mv.MedidorVelocidad()
lidar = LidarDriver()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: medidor.corriendo, medidor.al_barrido),
                 daemon=True).start()

print(f"=== tirada PWM {PWM}% ({ETIQUETA}) ===")
resultado = None
try:
    resultado = mv.una_tirada(medidor, enlace, PWM)
finally:
    enlace.enviar(0, 0.0)
    time.sleep(0.2)
    medidor.corriendo = False
    enlace.cerrar()
    lidar.cerrar()

if resultado:
    os.makedirs(mv.CARPETA_SALIDA, exist_ok=True)
    ruta = os.path.join(mv.CARPETA_SALIDA, f"velocidad_{ETIQUETA}_pwm{PWM}.json")
    with open(ruta, "w") as fh:
        json.dump(resultado, fh, indent=1)
    print(f"\n  guardado en {ruta}")
    v = resultado.get("velocidad_mm_s")
    if v:
        print(f"  RESUMEN  pwm={PWM}  v={v:.0f} mm/s = {v/1000:.3f} m/s  "
              f"tau90={resultado.get('t_90pct_s')}s  rms={resultado.get('residuo_rms_mm')}mm")
