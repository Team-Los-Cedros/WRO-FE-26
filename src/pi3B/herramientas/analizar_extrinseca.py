#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Calibra la guiñada cámara-LiDAR a partir de capturas de `capturar_extrinseca.py`.

Se ejecuta en el PC, no en la Pi: solo lee archivos.

Método
------
Un pilar visto por los dos sensores da dos bearings. Comparándolos sale el
error de montaje, pero hay dos trampas que este script evita:

1. **Asociar por distancia, no por bearing.** El bearing es justo la magnitud
   que se está midiendo; emparejar por él daría por bueno lo que se quiere
   comprobar. La cámara estima la distancia por la altura del blob (el pilar
   mide 100 mm reales) y con eso se empareja. Además, cuando las dos
   distancias no cuadran, la pareja se descarta sola.

2. **Descartar los pilares cercanos.** Por debajo de ~600 mm un pilar subtiende
   varios grados y el LiDAR ve solo su cara frontal mientras la cámara ve la
   silueta frontal más la lateral: los centroides no son el mismo punto y el
   bearing sale sesgado. Se detecta porque la distancia falla mucho.

Guiñada contra punto principal: un error de guiñada desplaza todos los
bearings por igual; uno de punto principal escala con `cos²` del bearing y por
tanto se nota más en los bordes. Con objetivos repartidos por el campo, la
dispersión entre desacuerdos distingue una causa de la otra.

Uso:

    python analizar_extrinseca.py CAPTURA [CAPTURA...]
"""
import colorsys
import json
import math
import os
import sys
from collections import deque

try:
    from PIL import Image
except ImportError:
    raise SystemExit("[-] Falta Pillow: pip install pillow")

ALTO_PILAR_MM = 100.0
DIST_MIN_FIABLE_MM = 600.0     # por debajo, los centroides no son comparables
TOLERANCIA_DISTANCIA = 0.15    # 15 % entre cámara y LiDAR para aceptar la pareja
MIN_PIXELES = 100
GATE_ASOCIACION_DEG = 20.0    # 6 veces el error esperado: acota sin sesgar

UMBRALES = {
    "rojo": lambda h, s, v: s > 0.45 and v > 0.25 and (h < 0.045 or h > 0.955),
    "verde": lambda h, s, v: s > 0.35 and v > 0.20 and 0.22 < h < 0.45,
}


def _blobs(im, prueba):
    ancho, alto = im.size
    px = im.load()
    def ok(x, y):
        r, g, b = px[x, y]
        return prueba(*colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0))
    visto = [[False] * ancho for _ in range(alto)]
    salida = []
    for y0 in range(alto):
        for x0 in range(ancho):
            if visto[y0][x0] or not ok(x0, y0):
                continue
            cola = deque([(x0, y0)])
            visto[y0][x0] = True
            puntos = []
            while cola:
                x, y = cola.popleft()
                puntos.append((x, y))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < ancho and 0 <= ny < alto
                            and not visto[ny][nx] and ok(nx, ny)):
                        visto[ny][nx] = True
                        cola.append((nx, ny))
            if len(puntos) >= MIN_PIXELES:
                salida.append(puntos)
    return salida


def objetos_camara(ruta, camara):
    im = Image.open(os.path.join(ruta, "frame.jpg")).convert("RGB")
    ancho = im.size[0]
    f = (ancho / 2.0) / math.tan(math.radians(camara["hfov_deg"]) / 2.0)
    c0 = float(camara["principal_x_px"])
    salida = []
    for color, prueba in UMBRALES.items():
        for puntos in _blobs(im, prueba):
            xs = [p[0] for p in puntos]
            ys = [p[1] for p in puntos]
            alto_px = max(ys) - min(ys) + 1
            salida.append({
                "color": color,
                "bearing": math.degrees(math.atan((sum(xs) / len(xs) - c0) / f)),
                "distancia": ALTO_PILAR_MM * f / alto_px,
                "pixeles": len(puntos),
            })
    return salida


def objetos_lidar(ruta):
    """Grupos contiguos que destacan del fondo: candidatos a pilar."""
    perfil = json.load(open(os.path.join(ruta, "perfil.json")))
    grupos, actual = [], []
    for a in range(360):
        d = perfil[a]
        valido = 150.0 < d < 3000.0
        if valido and actual and abs(d - perfil[a - 1]) < 80.0:
            actual.append(a)
        else:
            if len(actual) >= 1:
                grupos.append(actual)
            actual = [a] if valido else []
    if actual:
        grupos.append(actual)

    salida = []
    for g in grupos:
        ds = [perfil[a] for a in g]
        dm = sum(ds) / len(ds)
        ancho_mm = math.radians(len(g)) * dm
        if not 25.0 <= ancho_mm <= 130.0:
            continue
        # Un pilar está SUELTO: por los dos lados, a pocos grados, hay algo
        # mucho más lejos (la pared del fondo, o la sombra que él mismo
        # proyecta). Comparar contra la mediana de una ventana ancha no sirve:
        # esa ventana recoge las paredes laterales, que están más cerca que el
        # pilar, y el pilar deja de parecer un objeto.
        # Basta con UN lado: el C1 deja huecos sin dato, y un pilar puede tener
        # otro pilar pegado en bearing. Exigir los dos lados descarta pilares
        # buenos. Los falsos positivos que esto deja pasar los filtra despues
        # la asociacion por distancia.
        def hay_fondo(desde, paso):
            for k in range(1, 13):
                d = perfil[(desde + paso * k) % 360]
                if 150.0 < d < 7500.0 and d - dm >= 250.0:
                    return True
            return False

        if not (hay_fondo(g[0], -1) or hay_fondo(g[-1], 1)):
            continue
        centro = sum(g) / float(len(g))
        salida.append({
            "bearing": centro if centro <= 180.0 else centro - 360.0,
            "distancia": dm,
            "ancho_mm": ancho_mm,
        })
    return salida


def bearing_predicho(obj, camara, yaw=0.0):
    a = math.radians(obj["bearing"])
    x = obj["distancia"] * math.sin(a) - float(camara["right_from_lidar_mm"])
    y = obj["distancia"] * math.cos(a) - float(camara["forward_from_lidar_mm"])
    return math.degrees(math.atan2(x, y)) - yaw


def analizar(ruta):
    camara = json.load(open(os.path.join(ruta, "meta.json")))["camara"]
    cam = objetos_camara(ruta, camara)
    lid = objetos_lidar(ruta)
    print("\n=== {} ===".format(ruta))
    print("  camara: {} blobs | lidar: {} candidatos".format(len(cam), len(lid)))

    parejas = []
    for c in cam:
        # Se empareja por distancia, pero dentro de una ventana angular
        # generosa: el error que se busca son unos pocos grados, asi que
        # descartar parejas separadas mas de GATE no sesga el resultado y
        # evita casar un pilar con otro que esta al otro lado del robot.
        cercanos = [l for l in lid
                    if abs(c["bearing"] - bearing_predicho(l, camara)) <= GATE_ASOCIACION_DEG]
        if not cercanos:
            print("  {:6} cam {:6.0f} mm @ {:+6.2f}  <->  sin candidato dentro de"
                  " {:.0f} deg".format(c["color"], c["distancia"], c["bearing"],
                                       GATE_ASOCIACION_DEG))
            continue
        mejor = min(cercanos, key=lambda l: abs(l["distancia"] - c["distancia"]))
        error = abs(mejor["distancia"] - c["distancia"]) / c["distancia"]
        pred = bearing_predicho(mejor, camara)
        fiable = error <= TOLERANCIA_DISTANCIA and c["distancia"] >= DIST_MIN_FIABLE_MM
        print("  {:6} cam {:6.0f} mm @ {:+6.2f}  <->  lidar {:6.0f} mm @ {:+6.2f}"
              "  dist {:+5.1f}%  desacuerdo {:+5.2f}  {}".format(
                  c["color"], c["distancia"], c["bearing"],
                  mejor["distancia"], mejor["bearing"],
                  100.0 * (mejor["distancia"] - c["distancia"]) / c["distancia"],
                  c["bearing"] - pred, "OK" if fiable else "descartado"))
        if fiable:
            parejas.append((c["bearing"], c["bearing"] - pred))
    return parejas


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    todas = []
    for ruta in sys.argv[1:]:
        todas.extend(analizar(ruta))
    if not todas:
        raise SystemExit("\n[-] Ninguna pareja fiable. Hacen falta pilares a mas de "
                         "{:.0f} mm.".format(DIST_MIN_FIABLE_MM))
    ds = [d for _, d in todas]
    media = sum(ds) / len(ds)
    disp = max(ds) - min(ds)
    print("\n=== resultado sobre {} parejas fiables ===".format(len(ds)))
    print("  desacuerdos: " + ", ".join("{:+.2f}".format(d) for d in ds))
    print("  media {:+.2f} deg, dispersion {:.2f} deg".format(media, disp))
    print("\n  camera.yaw_from_lidar_deg deberia valer {:+.2f}".format(-media))
    if disp > 1.0:
        print("  [!] dispersion alta: no es solo guiñada. Revisar punto principal,")
        print("      o alguna pareja mal asociada.")
    else:
        print("  Dispersion baja: el desacuerdo es constante en todo el campo,")
        print("  que es la firma de un error de guiñada y no de punto principal.")


if __name__ == "__main__":
    main()
