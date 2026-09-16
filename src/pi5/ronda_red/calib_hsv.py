#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Recalibracion de los umbrales HSV para el montaje de mastil.

Motivo (ver README seccion 9): en la corrida instrumentada del 2026-08-29
el 47% de los ciclos no detecto ningun color, con una racha de 12.9
segundos seguidos a ciegas. Y eso no es solo perder una evasion: un poste
sin color de frente no lo mira NADIE -- el control de pared lo ignora a
proposito (usa frontal_muro, que con un objeto estrecho delante vale 8000
= "via libre") y la evasion no arranca sin color. Las tres emergencias de
esa corrida fueron eso.

EL PUNTO CLAVE DEL METODO: no se buscan los pilares por color. Si los
umbrales estan mal -- que es justo la hipotesis -- buscar por color solo
encontraria los casos que ya funcionan, y la muestra saldria sesgada
precisamente hacia lo que no falla. Aqui el pilar lo localiza el LiDAR
(cluster estrecho) y se proyecta a la imagen con el modelo optico ya
calibrado (optica.py, residuo 0.2 grados). Dentro de esa franja de
columnas se toma el mayor blob SATURADO, sea del color que sea.

Asi que el filtro de "que es pilar" es geometrico, no cromatico:
  - franja de columnas: la que subtiende el cluster del LiDAR
  - pixel candidato: saturado y no oscuro (el suelo es blanco -> S bajo;
    los muros son negros -> V bajo; las lineas de la pista son saturadas
    pero alargadas y tumbadas, y caen por el filtro de forma)
  - blob: el mayor con alto > 0.7*ancho, como en vision.py

Se registra tambien la exposicion y la ganancia de cada frame: la lona es
muy blanca y el auto-exposure puede estar cerrandose y desaturando los
pilares, en cuyo caso la solucion no es abrir los umbrales sino fijar la
exposicion.

USO: correr y pasear los DOS pilares por delante del robot, de lejos
(~1200mm) a cerca (~250mm), y por los lados. Cuantas mas distancias y
angulos, mejor. Ctrl-C para terminar. Robot QUIETO: no manda consignas.
"""
import math
import os
import signal
import threading
import time

import cv2
import numpy as np

import optica
import vision
from camara_driver import CamaraDriver, ANCHO_FRAME, ALTO_FRAME
from lidar_driver import LidarDriver
from lidar_geometria import (ProcesadorLidar, centroide_xy_cluster,
                             es_objeto_estrecho)

DIR = "capturas_hsv"
os.makedirs(DIR, exist_ok=True)

# Puerta MUY laxa para decidir que un pixel "tiene color". No es un umbral
# de deteccion: solo separa el pilar del suelo blanco (S bajo) y del muro
# negro (V bajo). Si se aprieta aqui se vuelve a sesgar la muestra.
S_MIN_CANDIDATO = 50
V_MIN_CANDIDATO = 35
AREA_MIN_BLOB   = 120

MARGEN_COLUMNAS = 12      # px de holgura a cada lado de la franja del cluster
_KERNEL = np.ones((3, 3), np.uint8)

corriendo = True
ultimo_frame = [None]
ultimo_scan = [None]
muestras = []             # (h, s, v, dist, n_px)
# Por cada pilar que el LiDAR ve, se anota si vision.py lo detecto y, si
# no, cual de sus filtros lo tumbo. Es la medida que explica el 47% de
# ciclos sin color de la corrida: dice si el color se pierde por umbral
# HSV, por area, por el filtro de altura cy o por la forma.
fallos = {"detectado": 0, "area": 0, "cy": 0, "forma": 0, "sin_blob_hsv": 0}
detalle_fallos = []
n_guardadas = [0]


def parar(*_):
    global corriendo
    corriendo = False


signal.signal(signal.SIGINT, parar)


def franja_de_cluster(cluster):
    """Columnas de imagen que ocupa un cluster del LiDAR."""
    rumbos = []
    for ang_deg, dist_mm in cluster:
        rad = math.radians(ang_deg)
        x, y = dist_mm * math.sin(rad), dist_mm * math.cos(rad)
        if y <= 0:
            continue
        rumbos.append(optica.rumbo_camara_de_cluster(x, y))
    if not rumbos:
        return None
    c_ini = optica.cx_de_rumbo(min(rumbos)) - MARGEN_COLUMNAS
    c_fin = optica.cx_de_rumbo(max(rumbos)) + MARGEN_COLUMNAS
    c_ini = int(max(0, min(ANCHO_FRAME - 1, c_ini)))
    c_fin = int(max(0, min(ANCHO_FRAME - 1, c_fin)))
    return (c_ini, c_fin) if c_fin - c_ini >= 4 else None


print("[*] Arrancando camara...")
camara = CamaraDriver()
threading.Thread(target=camara.hilo_captura,
                 args=(lambda: corriendo, lambda f: ultimo_frame.__setitem__(0, f)),
                 daemon=True).start()
time.sleep(2.5)

print("[*] Arrancando LiDAR. El robot NO se mueve.")
lidar = LidarDriver()
geo = ProcesadorLidar()
threading.Thread(target=lidar.hilo_lectura,
                 args=(lambda: corriendo, lambda s: ultimo_scan.__setitem__(0, s)),
                 daemon=True).start()
t0 = time.time()
while ultimo_scan[0] is None and time.time() - t0 < 25:
    time.sleep(0.2)

print("\nPasea los DOS pilares por delante, de ~1200mm a ~250mm y por los")
print("lados. Ctrl-C para terminar.\n")
print("  dist    columnas    n_px |   H     S     V   | exposicion")
print("  " + "-" * 62)

try:
    while corriendo:
        time.sleep(0.3)
        frame, scan = ultimo_frame[0], ultimo_scan[0]
        if frame is None or scan is None:
            continue

        med = geo.procesar(scan)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for c in med.clusters_obstaculo:
            if not es_objeto_estrecho(c):
                continue
            x, y = centroide_xy_cluster(c)
            if y <= 0:
                continue
            dist = math.hypot(x, y)
            fr = franja_de_cluster(c)
            if fr is None:
                continue
            c_ini, c_fin = fr

            sub = hsv[:, c_ini:c_fin + 1]
            mask = ((sub[:, :, 1] >= S_MIN_CANDIDATO) &
                    (sub[:, :, 2] >= V_MIN_CANDIDATO)).astype(np.uint8) * 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            mejor, mejor_area = None, 0
            for cn in cnts:
                a = cv2.contourArea(cn)
                if a < AREA_MIN_BLOB or a <= mejor_area:
                    continue
                bx, by, bw, bh = cv2.boundingRect(cn)
                if bh <= bw * 0.7:      # los pilares son mas altos que anchos
                    continue
                mejor, mejor_area = (bx, by, bw, bh), a
            if mejor is None:
                continue

            bx, by, bw, bh = mejor
            # Nucleo del blob: se recorta un 25% por cada lado para no
            # muestrear el borde, donde el pixel mezcla pilar y fondo y
            # ensucia justo las colas de la distribucion.
            rx0, rx1 = bx + bw // 4, bx + bw - bw // 4
            ry0, ry1 = by + bh // 4, by + bh - bh // 4
            nucleo = sub[ry0:ry1, rx0:rx1].reshape(-1, 3)
            nucleo = nucleo[(nucleo[:, 1] >= S_MIN_CANDIDATO) &
                            (nucleo[:, 2] >= V_MIN_CANDIDATO)]
            if len(nucleo) < 30:
                continue

            # --- Que dice vision.py de ESTE mismo pilar? ---
            # Se repite aqui su pipeline exacto, restringido a la franja
            # de columnas del cluster, para saber cual de sus filtros lo
            # descarta cuando el LiDAR si lo ve.
            m_r = (cv2.inRange(sub, vision.ROJO_BAJO_1, vision.ROJO_ALTO_1) |
                   cv2.inRange(sub, vision.ROJO_BAJO_2, vision.ROJO_ALTO_2))
            m_v = cv2.inRange(sub, vision.VERDE_BAJO, vision.VERDE_ALTO)
            veredicto = "sin_blob_hsv"
            for mm in (m_r, m_v):
                mm = cv2.morphologyEx(mm, cv2.MORPH_OPEN, vision._KERNEL)
                cs, _ = cv2.findContours(mm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cn in cs:
                    ar = cv2.contourArea(cn)
                    if ar <= 0:
                        continue
                    vx, vy, vw, vh = cv2.boundingRect(cn)
                    vcy = vy + vh // 2
                    if ar <= vision.AREA_MIN_DETECCION:
                        if veredicto in ("sin_blob_hsv",):
                            veredicto = "area"
                        continue
                    if vcy >= vision.UMBRAL_CY:
                        veredicto = "cy"
                        continue
                    if vh <= vw * 0.7:
                        veredicto = "forma"
                        continue
                    veredicto = "detectado"
                    break
                if veredicto == "detectado":
                    break
            fallos[veredicto] = fallos.get(veredicto, 0) + 1
            if veredicto != "detectado":
                detalle_fallos.append((veredicto, dist, bh, by + bh // 2, mejor_area))

            hm = float(np.median(nucleo[:, 0]))
            sm = float(np.median(nucleo[:, 1]))
            vm = float(np.median(nucleo[:, 2]))
            muestras.append((hm, sm, vm, dist, len(nucleo)))
            print("  %6.0f  %4d-%-4d %6d | %5.1f %5.1f %5.1f |"
                  % (dist, c_ini, c_fin, len(nucleo), hm, sm, vm))

            if n_guardadas[0] < 25:
                n_guardadas[0] += 1
                anot = frame.copy()
                cv2.rectangle(anot, (c_ini, 0), (c_fin, ALTO_FRAME - 1), (255, 255, 0), 1)
                cv2.rectangle(anot, (c_ini + bx, by), (c_ini + bx + bw, by + bh),
                              (0, 0, 255), 2)
                cv2.imwrite("%s/%02d_d%04.0f_h%03.0f.jpg"
                            % (DIR, n_guardadas[0], dist, hm), anot)
except KeyboardInterrupt:
    pass
finally:
    corriendo = False
    time.sleep(0.4)
    lidar.cerrar()
    camara.cerrar()

if len(muestras) < 10:
    raise SystemExit("\n[-] Solo %d muestras. Hace falta pasear los pilares "
                     "delante del robot mientras corre." % len(muestras))

print("\n[+] %d muestras. Recortes anotados en %s/" % (len(muestras), DIR))

# --- La medida que de verdad explica el 47% de ciclos sin color ---
# Para cada pilar que el LiDAR ve, si vision.py NO lo detecta, cual de
# sus filtros lo tumbo. Distingue "los umbrales HSV estan mal" de "el
# umbral de area es demasiado alto para el frame nuevo" y de "el filtro
# cy lo descarta", que piden arreglos completamente distintos.
total_f = sum(fallos.values())
print("\n" + "=" * 64)
print("=== Pilares que el LiDAR ve: los detecta vision.py? (%d casos) ===" % total_f)
_etq = {
    "detectado":    "SI, detectado",
    "area":         "NO: area < AREA_MIN_DETECCION (%d)" % vision.AREA_MIN_DETECCION,
    "cy":           "NO: centroide por debajo de UMBRAL_CY (%d)" % vision.UMBRAL_CY,
    "forma":        "NO: alto <= 0.7*ancho",
    "sin_blob_hsv": "NO: ningun pixel pasa los umbrales HSV",
}
for k in ("detectado", "area", "cy", "forma", "sin_blob_hsv"):
    v = fallos.get(k, 0)
    print("  %-48s %4d  (%3.0f%%)" % (_etq[k], v, 100.0 * v / max(1, total_f)))
if detalle_fallos:
    print("\n  ejemplos de fallo (motivo, dist, alto_blob, cy, area):")
    for d in detalle_fallos[:12]:
        print("    %-13s %6.0fmm  alto=%3d  cy=%3d  area=%.0f" % d)
    # A que distancias falla, que es lo que decide si importa o no
    import collections as _c
    por_motivo = _c.defaultdict(list)
    for motivo, dist, _bh, _cy, _ar in detalle_fallos:
        por_motivo[motivo].append(dist)
    print("\n  distancias en las que falla cada motivo:")
    for motivo, ds in por_motivo.items():
        print("    %-13s n=%3d   %4.0f - %4.0f mm   (mediana %4.0f)"
              % (motivo, len(ds), min(ds), max(ds), sorted(ds)[len(ds) // 2]))

# Separar por tono: el rojo vive en las dos puntas del circulo H (0-20 y
# 155-180) y el verde en el medio (35-90). Se agrupa por ahi, no por los
# umbrales viejos, que son justo lo que se esta poniendo en duda.
rojos  = [m for m in muestras if m[0] <= 25 or m[0] >= 150]
verdes = [m for m in muestras if 30 <= m[0] <= 95]
otros  = [m for m in muestras if m not in rojos and m not in verdes]


def informe(nombre, ms, bajo_actual, alto_actual):
    if not ms:
        print("\n=== %s: sin muestras ===" % nombre)
        return
    a = np.array([[m[0], m[1], m[2]] for m in ms])
    d = np.array([m[3] for m in ms])
    print("\n=== %s: %d muestras, distancias %.0f-%.0f mm ===" % (nombre, len(ms), d.min(), d.max()))
    for i, canal in enumerate("HSV"):
        p = np.percentile(a[:, i], [1, 5, 50, 95, 99])
        print("  %s  min=%5.1f  p5=%5.1f  mediana=%5.1f  p95=%5.1f  max=%5.1f"
              % (canal, p[0], p[1], p[2], p[3], p[4]))
    # Cuantas muestras pasan los umbrales ACTUALES
    ok = 0
    for m in ms:
        h, s, v = m[0], m[1], m[2]
        for lo, hi in zip(bajo_actual, alto_actual):
            if lo[0] <= h <= hi[0] and lo[1] <= s <= hi[1] and lo[2] <= v <= hi[2]:
                ok += 1
                break
    print("  --> los umbrales ACTUALES aceptan %d de %d (%.0f%%)"
          % (ok, len(ms), 100.0 * ok / len(ms)))
    return a


print("\n" + "=" * 64)
a_r = informe("ROJO", rojos,
              [vision.ROJO_BAJO_1, vision.ROJO_BAJO_2],
              [vision.ROJO_ALTO_1, vision.ROJO_ALTO_2])
a_v = informe("VERDE", verdes, [vision.VERDE_BAJO], [vision.VERDE_ALTO])
if otros:
    print("\n=== %d muestras con tono fuera de rojo/verde ===" % len(otros))
    for m in otros[:8]:
        print("  H=%.0f S=%.0f V=%.0f a %.0f mm" % (m[0], m[1], m[2], m[3]))
    print("  (si son muchas, el blob elegido no era el pilar: revisa los")
    print("   recortes de %s/ antes de fiarte de nada)" % DIR)

print("\n" + "=" * 64)
print("=== Umbrales propuestos (p1/p99 con margen) ===")


def propuesta(nombre, a, margen_h=8, margen_s=25, margen_v=25):
    if a is None or len(a) == 0:
        return
    hs = a[:, 0]
    if nombre == "ROJO" and hs.max() - hs.min() > 100:
        bajo = hs[hs <= 25]
        alto = hs[hs >= 150]
        print("  ROJO_BAJO_1 = [%3d, %3d, %3d]   ROJO_ALTO_1 = [%3d, 255, 255]"
              % (0, max(0, np.percentile(a[:, 1], 1) - margen_s),
                 max(0, np.percentile(a[:, 2], 1) - margen_v),
                 min(179, bajo.max() + margen_h) if len(bajo) else 15))
        print("  ROJO_BAJO_2 = [%3d, %3d, %3d]   ROJO_ALTO_2 = [179, 255, 255]"
              % (max(0, alto.min() - margen_h) if len(alto) else 158,
                 max(0, np.percentile(a[:, 1], 1) - margen_s),
                 max(0, np.percentile(a[:, 2], 1) - margen_v)))
    else:
        print("  %s_BAJO = [%3d, %3d, %3d]   %s_ALTO = [%3d, 255, 255]"
              % (nombre, max(0, np.percentile(hs, 1) - margen_h),
                 max(0, np.percentile(a[:, 1], 1) - margen_s),
                 max(0, np.percentile(a[:, 2], 1) - margen_v),
                 nombre, min(179, np.percentile(hs, 99) + margen_h)))


propuesta("ROJO", a_r)
propuesta("VERDE", a_v)
print("\n  Margenes: H +-8, S y V -25 sobre el percentil 1. No pegarlos sin")
print("  mirar antes los recortes de %s/: si el blob no era el pilar," % DIR)
print("  estos numeros son basura bien formateada.")
