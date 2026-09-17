#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vista cenital ASCII de lo que ve el LiDAR, para saber DONDE esta el robot
en la pista sin depender de la camara del techo. Dibuja el barrido en
cartesianas (x+ = derecha, y+ = frente) con el robot en el centro, marca
el arco ciego del mastil y lista las distancias cada 10 grados.

Robot QUIETO: no instancia EnlacePico, no manda consignas.
"""
import threading
import time
import math

from lidar_driver import LidarDriver
from lidar_geometria import construir_perfil_360
import lidar_mascara as msk

N_BARRIDOS = 12
ALCANCE_MM = 2600.0     # media anchura de la ventana dibujada
COLS, FILAS = 79, 39

corriendo = True
barridos = []


def al_barrido(scan):
    if len(barridos) < N_BARRIDOS:
        barridos.append(scan)


print("[*] LiDAR arrancando. El robot NO se mueve.")
lidar = LidarDriver()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, al_barrido), daemon=True).start()
t0 = time.time()
while len(barridos) < N_BARRIDOS and time.time() - t0 < 40:
    time.sleep(0.1)
corriendo = False
time.sleep(0.4)
lidar.cerrar()

if len(barridos) < 3:
    raise SystemExit("[-] Sin datos suficientes.")

# Perfil consolidado: mediana por grado sobre todos los barridos
acum = [[] for _ in range(360)]
for scan in barridos:
    p = construir_perfil_360(scan)
    for i in range(360):
        if p[i] < 8000.0:
            acum[i].append(p[i])
perfil = [8000.0] * 360
for i in range(360):
    if acum[i]:
        acum[i].sort()
        perfil[i] = acum[i][len(acum[i]) // 2]

lienzo = [[" "] * COLS for _ in range(FILAS)]
esc_x = (COLS // 2) / ALCANCE_MM
esc_y = (FILAS // 2) / ALCANCE_MM

for i in range(360):
    d = perfil[i]
    if d >= 8000.0:
        continue
    rad = math.radians(i)
    x, y = d * math.sin(rad), d * math.cos(rad)
    cx = COLS // 2 + int(round(x * esc_x))
    cy = FILAS // 2 - int(round(y * esc_y))
    if 0 <= cx < COLS and 0 <= cy < FILAS:
        lienzo[cy][cx] = "X" if i in msk.BINS_CIEGOS else "o"

c = COLS // 2
f = FILAS // 2
lienzo[f][c] = "R"
lienzo[f - 1][c] = "^"          # morro del robot

print(f"\n=== Cenital LiDAR ({len(barridos)} barridos, mediana por grado) ===")
print(f"    ventana +-{ALCANCE_MM:.0f}mm    'R'=robot  '^'=frente  "
      f"'o'=eco  'X'=arco ciego del mastil")
print("   +" + "-" * COLS + "+")
for fila in lienzo:
    print("   |" + "".join(fila) + "|")
print("   +" + "-" * COLS + "+")

print("\n=== Distancias cada 10 grados (mm) ===")
for base in range(0, 360, 90):
    fila = []
    for a in range(base, base + 90, 10):
        d = perfil[a]
        marca = "*" if a in msk.BINS_CIEGOS else " "
        fila.append(f"{a:3d}{marca}{'  ---' if d >= 8000 else f'{d:6.0f}'}")
    print("  " + "  ".join(fila))
print("  ('*' = grado dentro del arco ciego del mastil)")

ok, msj = msk.diagnosticar(perfil)
print(f"\n=== Autochequeo de la mascara ===\n  [{'OK' if ok else 'X'}] {msj}")
