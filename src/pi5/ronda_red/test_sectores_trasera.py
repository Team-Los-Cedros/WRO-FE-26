#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Que le hace el mastil de la camara a los sectores traseros REALES que
consume navegacion.py. No razona sobre el codigo: instancia el
ProcesadorLidar de verdad e imprime los campos de Medicion, mas las
alternativas que propone lidar_mascara.py, para poder compararlos lado
a lado con el robot quieto.

Los tres numeros que importan:
  med.trasera            -> condicion de salida de RETROCESO
                            (< EMERGENCIA_TRASERA = 250mm lo aborta)
  med.trasera_derecha    -> termino positivo del control P de RETROCESO
  med.trasera_izquierda  -> termino negativo del mismo control

Robot QUIETO: no instancia EnlacePico, no manda consignas.
"""
import statistics
import threading
import time

from lidar_driver import LidarDriver
from lidar_geometria import ProcesadorLidar, distancia_en_rango
import lidar_mascara as msk

N_BARRIDOS = 25
EMERGENCIA_TRASERA = 250.0   # copiado de navegacion.py, solo para el veredicto

corriendo = True
barridos = []


def al_barrido(scan):
    if len(barridos) < N_BARRIDOS:
        barridos.append(scan)


print("[*] LiDAR arrancando. El robot NO se mueve.")
lidar = LidarDriver()
geo = ProcesadorLidar()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, al_barrido), daemon=True).start()

t0 = time.time()
while len(barridos) < N_BARRIDOS and time.time() - t0 < 45:
    time.sleep(0.1)
corriendo = False
time.sleep(0.4)
lidar.cerrar()

print(f"[*] Barridos: {len(barridos)}\n")
if len(barridos) < 5:
    raise SystemExit("[-] Sin datos suficientes.")

col = {k: [] for k in ("frontal", "izq", "der", "tras", "tras_der", "tras_izq",
                       "m_tras", "m_tras_der", "m_tras_izq")}

for scan in barridos:
    med = geo.procesar(scan)
    col["frontal"].append(med.frontal)
    col["izq"].append(med.izquierda)
    col["der"].append(med.derecha)
    col["tras"].append(med.trasera)
    col["tras_der"].append(med.trasera_derecha)
    col["tras_izq"].append(med.trasera_izquierda)
    col["m_tras"].append(msk.distancia_trasera(med.perfil))
    col["m_tras_der"].append(msk.distancia_trasera_derecha(med.perfil))
    col["m_tras_izq"].append(msk.distancia_trasera_izquierda(med.perfil))


def linea(nombre, vals):
    print(f"  {nombre:24s} mediana={statistics.median(vals):8.1f}  "
          f"min={min(vals):8.1f}  max={max(vals):8.1f}")


print("=== Sectores ACTUALES (lidar_geometria, sin mascara) ===")
linea("frontal", col["frontal"])
linea("izquierda", col["izq"])
linea("derecha", col["der"])
linea("trasera        [170,190]", col["tras"])
linea("trasera_der     [90,170]", col["tras_der"])
linea("trasera_izq    [190,270]", col["tras_izq"])

print("\n=== Sectores CON MASCARA (lidar_mascara) ===")
linea("trasera  (proyectada)", col["m_tras"])
linea("trasera_der", col["m_tras_der"])
linea("trasera_izq", col["m_tras_izq"])

print("\n=== Veredicto para el estado RETROCESO de navegacion.py ===")
t_med = statistics.median(col["tras"])
m_med = statistics.median(col["m_tras"])
if t_med < EMERGENCIA_TRASERA:
    print(f"  [X] med.trasera={t_med:.0f}mm < {EMERGENCIA_TRASERA:.0f}mm: RETROCESO aborta")
    print("      en su primer ciclo SIEMPRE, este donde este el robot.")
else:
    print(f"  [OK] med.trasera={t_med:.0f}mm, por encima del umbral.")
if m_med < EMERGENCIA_TRASERA:
    print(f"  [!] con mascara={m_med:.0f}mm: sigue por debajo -- hay algo REAL detras.")
else:
    print(f"  [OK] con mascara={m_med:.0f}mm: el sector vuelve a medir el entorno.")

e_act = statistics.median(col["tras_der"]) - statistics.median(col["tras_izq"])
e_msk = statistics.median(col["m_tras_der"]) - statistics.median(col["m_tras_izq"])
print(f"\n  error del control P de RETROCESO (tras_der - tras_izq):")
print(f"    actual      = {e_act:+9.1f} mm")
print(f"    con mascara = {e_msk:+9.1f} mm")
if abs(e_act - e_msk) > 300:
    print("    -> el signo/magnitud del giro en reversa lo dictaba el mastil,")
    print("       no el espacio libre real.")
