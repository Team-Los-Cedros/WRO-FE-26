#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mapa de oclusion del LiDAR: mide que grados del barrido de 360 estan
tapados por la propia estructura del robot (desde que la camara se monto
en un mastil trasero, el mastil se mete en el campo de vision del C1).

Se corre con el ROBOT QUIETO: no instancia EnlacePico, no manda consignas.
Captura N barridos y para cada grado calcula:

  - tasa de eco: en que fraccion de barridos ese grado devolvio algo.
  - mediana / min / max de la distancia.
  - dispersion (max-min): una pieza atornillada al chasis mide SIEMPRE lo
    mismo; una pared vista desde un robot quieto tambien, pero esta a
    cientos de mm. El discriminante fuerte es la distancia, la dispersion
    solo confirma que es estructura y no ruido.

Clasificacion por grado:
  BLOQUEADO  -> eco casi siempre a distancia de chasis (< UMBRAL_ESTRUCTURA)
  SIN_ECO    -> casi nunca devuelve nada (el haz muere en la pieza o cae
                por debajo del rango minimo del C1). Es igual de peligroso
                que el anterior: construir_perfil_360 rellena esos bins
                con 8000.0, o sea "via libre", que es justo lo contrario.
  LIBRE      -> mide el entorno de verdad.

Al final imprime un mapa ASCII de los 360 grados y los rangos a excluir,
listos para pegarlos en lidar_mascara.py.

Uso:  python3 mapa_oclusion.py [n_barridos]
"""
import sys
import threading
import time

from lidar_driver import LidarDriver

# Un poste del reglamento a la distancia mas corta que interesa esta a
# ~150mm del sensor; cualquier eco estable por debajo de esto no es pista,
# es el propio robot.
UMBRAL_ESTRUCTURA = 150.0   # mm
# Fraccion de barridos que tienen que coincidir para dar un grado por
# tapado (y no por un eco suelto de ruido).
TASA_MIN_BLOQUEO  = 0.60
# Por debajo de esta tasa de eco el grado se considera ciego.
TASA_MAX_SIN_ECO  = 0.25

N_BARRIDOS = int(sys.argv[1]) if len(sys.argv) > 1 else 40

corriendo = True
barridos = []


def al_barrido(scan):
    if len(barridos) < N_BARRIDOS:
        barridos.append(scan)


print(f"[*] Arrancando LiDAR. El robot NO se mueve. Objetivo: {N_BARRIDOS} barridos.")
lidar = LidarDriver()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, al_barrido), daemon=True).start()

t0 = time.time()
while len(barridos) < N_BARRIDOS and time.time() - t0 < 60:
    time.sleep(0.1)

corriendo = False
time.sleep(0.4)
lidar.cerrar()

n = len(barridos)
print(f"[*] Barridos capturados: {n}")
if n < 5:
    print("[-] Muy pocos barridos, el LiDAR no entrego datos suficientes.")
    raise SystemExit(1)

# Por grado: lista con la distancia minima de cada barrido que lo vio
por_grado = [[] for _ in range(360)]
for scan in barridos:
    minimos = [None] * 360
    for ang, dist in scan:
        i = int(ang) % 360
        if minimos[i] is None or dist < minimos[i]:
            minimos[i] = dist
    for i in range(360):
        if minimos[i] is not None:
            por_grado[i].append(minimos[i])

estado = [None] * 360
for i in range(360):
    vals = por_grado[i]
    tasa = len(vals) / n
    if tasa <= TASA_MAX_SIN_ECO:
        estado[i] = "SIN_ECO"
        continue
    vals_ord = sorted(vals)
    mediana = vals_ord[len(vals_ord) // 2]
    cercanos = sum(1 for v in vals if v < UMBRAL_ESTRUCTURA) / len(vals)
    if mediana < UMBRAL_ESTRUCTURA and cercanos >= TASA_MIN_BLOQUEO:
        estado[i] = "BLOQUEADO"
    else:
        estado[i] = "LIBRE"

# ---------- tabla ----------
print("\n=== Grados NO utilizables (bloqueados o ciegos) ===")
print("  gr   estado      tasa_eco  mediana   min     max    disper")
for i in range(360):
    if estado[i] == "LIBRE":
        continue
    vals = por_grado[i]
    tasa = len(vals) / n
    if vals:
        vals_ord = sorted(vals)
        med = vals_ord[len(vals_ord) // 2]
        print(f"  {i:3d}  {estado[i]:10s}  {tasa:6.2f}  {med:7.1f} {min(vals):7.1f} "
              f"{max(vals):7.1f} {max(vals) - min(vals):7.1f}")
    else:
        print(f"  {i:3d}  {estado[i]:10s}  {tasa:6.2f}        -       -       -       -")

# ---------- mapa ASCII ----------
SIMBOLO = {"BLOQUEADO": "#", "SIN_ECO": ".", "LIBRE": "-"}
print("\n=== Mapa 360 (0=frente, crece horario)  '#'=tapado  '.'=sin eco  '-'=libre ===")
for base in range(0, 360, 60):
    fila = "".join(SIMBOLO[estado[i]] for i in range(base, base + 60))
    print(f"  {base:3d}-{base + 59:3d}  {fila}")

# ---------- rangos ----------
def rangos_de(pred):
    out, ini = [], None
    for i in range(360):
        if pred(estado[i]):
            if ini is None:
                ini = i
        elif ini is not None:
            out.append((ini, i - 1))
            ini = None
    if ini is not None:
        out.append((ini, 359))
    # unir el rango que cruza el 0
    if len(out) >= 2 and out[0][0] == 0 and out[-1][1] == 359:
        out[0] = (out[-1][0], out[0][1])
        out.pop()
    return out

no_util = rangos_de(lambda e: e != "LIBRE")
print("\n=== Rangos a excluir (para lidar_mascara.BINS_CIEGOS) ===")
if not no_util:
    print("  ninguno: los 360 grados miden entorno real.")
for a, b in no_util:
    ancho = (b - a) % 360 + 1
    print(f"  ({a:3d}, {b:3d})   ancho={ancho:3d} grados")

# ---------- impacto en los sectores que usa la navegacion ----------
SECTORES = [
    ("frontal   ", 350,  10),
    ("derecha   ",  30,  90),
    ("izquierda ", 270, 330),
    ("trasera   ", 170, 190),
    ("tras-der  ",  90, 170),
    ("tras-izq  ", 190, 270),
    ("perp_der  ",  80, 100),
    ("perp_izq  ", 260, 280),
    ("diag_der  ",  40,  50),
    ("diag_izq  ", 310, 320),
]
print("\n=== Impacto por sector de lidar_geometria ===")
for nombre, a, b in SECTORES:
    idx = list(range(a, b + 1)) if a <= b else list(range(a, 360)) + list(range(0, b + 1))
    malos = [i for i in idx if estado[i] != "LIBRE"]
    pct = 100.0 * len(malos) / len(idx)
    marca = "  <-- INUTILIZABLE" if pct >= 90 else ("  <-- degradado" if pct >= 25 else "")
    print(f"  {nombre} [{a:3d},{b:3d}]  {len(malos):3d}/{len(idx):3d} grados tapados "
          f"({pct:5.1f}%){marca}")
