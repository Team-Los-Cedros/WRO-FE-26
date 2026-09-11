# Que le pasa a la mascara HSV con un pilar LEJOS.
#
# El problema, medido en tres corridas: el LiDAR ve el poste hasta 1400 mm
# y la camara solo le pone color desde ~1100 hacia dentro. Sin color no hay
# lado obligatorio y el poste se pasa por donde salga, que termina el
# recorrido.
#
# Dos intentos de arreglarlo desde el CONTROL fracasaron (frenar al verlo,
# y encararlo). Asi que el corte esta en la vision, y esto lo mira de
# frente: con el robot quieto y un pilar a una distancia conocida, dice
# cuanto blob sobrevive a cada paso de la cadena y que pasaria con los
# umbrales relajados.
#
#   python3 sonda_color.py 1200        (distancia del pilar en mm, opcional)
#
# No mueve el robot ni toca el motor.
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, "/home/pi/ronda_curvas")

import optica                              # noqa: E402
import vision                              # noqa: E402
from camara_driver import CamaraDriver     # noqa: E402

DIST = float(sys.argv[1]) if len(sys.argv) > 1 else None
N_FRAMES = 12

_frames = []


def _al_frame(f):
    if len(_frames) < N_FRAMES:
        _frames.append(f.copy())


def blob_mayor(mask):
    """(area, w, h) del contorno mayor, o (0,0,0)."""
    cont, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mejor = (0, 0, 0)
    for c in cont:
        x, y, w, h = cv2.boundingRect(c)
        a = cv2.contourArea(c)
        if a > mejor[0]:
            mejor = (a, w, h)
    return mejor


def mide(hsv, bajo, alto, bajo2=None, alto2=None):
    m = cv2.inRange(hsv, bajo, alto)
    if bajo2 is not None:
        m = m | cv2.inRange(hsv, bajo2, alto2)
    crudo = blob_mayor(m)
    abierto = blob_mayor(cv2.morphologyEx(m, cv2.MORPH_OPEN, vision._KERNEL))
    return crudo, abierto, int(m.sum() // 255)


def main():
    cam = CamaraDriver()
    corriendo = [True]
    threading.Thread(target=cam.hilo_captura,
                     args=(lambda: corriendo[0], _al_frame), daemon=True).start()
    t0 = time.time()
    while len(_frames) < N_FRAMES and time.time() - t0 < 20:
        time.sleep(0.05)
    corriendo[0] = False
    if not _frames:
        print("[-] sin frames")
        return 1

    f = _frames[len(_frames) // 2]
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    print("frame %dx%d | umbral de area %d" % (f.shape[1], f.shape[0],
                                               vision.AREA_MIN_DETECCION))
    if DIST:
        lado = optica.FOCAL_PX * 50.0 / DIST
        print("pilar a %.0f mm -> deberia medir %.0f x %.0f px, area ideal %.0f"
              % (DIST, lado, lado * 2, lado * lado * 2))
    print()

    print("=== LO QUE SOBREVIVE A CADA PASO ===")
    for etq, args in (("ROJO ", (vision.ROJO_BAJO_1, vision.ROJO_ALTO_1,
                                 vision.ROJO_BAJO_2, vision.ROJO_ALTO_2)),
                      ("VERDE", (vision.VERDE_BAJO, vision.VERDE_ALTO))):
        crudo, abierto, pix = mide(hsv, *args)
        print("  %s  mascara %6d px | blob crudo %6.0f (%3dx%3d) | "
              "tras la apertura %6.0f  %s"
              % (etq, pix, crudo[0], crudo[1], crudo[2], abierto[0],
                 "PASA" if abierto[0] > vision.AREA_MIN_DETECCION else "NO LLEGA"))

    print()
    print("=== QUE PASARIA RELAJANDO SATURACION Y VALOR ===")
    print("  (el tono no se toca: es lo que distingue rojo de verde)")
    for etq, bajo, alto, b2, a2 in (
            ("ROJO ", vision.ROJO_BAJO_1, vision.ROJO_ALTO_1,
             vision.ROJO_BAJO_2, vision.ROJO_ALTO_2),
            ("VERDE", vision.VERDE_BAJO, vision.VERDE_ALTO, None, None)):
        print("  %s  S_min V_min |  blob tras apertura" % etq)
        for fs, fv in ((1.0, 1.0), (0.75, 0.75), (0.5, 0.5), (0.35, 0.35)):
            b = bajo.copy(); b[1] = int(bajo[1] * fs); b[2] = int(bajo[2] * fv)
            bb = None
            if b2 is not None:
                bb = b2.copy(); bb[1] = int(b2[1] * fs); bb[2] = int(b2[2] * fv)
            _, abierto, pix = mide(hsv, b, alto, bb, a2)
            print("         %3d  %3d  |  %7.0f  %s"
                  % (b[1], b[2], abierto[0],
                     "PASA" if abierto[0] > vision.AREA_MIN_DETECCION else "no"))

    print()
    print("=== HSV DEL CENTRO DEL FRAME (donde deberia estar el pilar) ===")
    h, w = hsv.shape[:2]
    roi = hsv[h // 2 - 30:h // 2 + 30, w // 2 - 20:w // 2 + 20]
    print("  H %3.0f+-%2.0f | S %3.0f+-%2.0f | V %3.0f+-%2.0f"
          % (roi[:, :, 0].mean(), roi[:, :, 0].std(),
             roi[:, :, 1].mean(), roi[:, :, 1].std(),
             roi[:, :, 2].mean(), roi[:, :, 2].std()))
    cv2.imwrite("/home/pi/ronda_curvas/logs/sonda_color.jpg", f)
    print("\nframe guardado en logs/sonda_color.jpg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
