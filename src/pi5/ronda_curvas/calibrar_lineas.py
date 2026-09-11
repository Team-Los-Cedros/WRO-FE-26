# Calibra el ORDEN de las lineas de esquina para deducir el sentido.
#
# Las lineas naranja y azul del tapete son la UNICA evidencia absoluta del
# sentido de la vuelta: no dependen del yaw del robot ni de interpretar la
# geometria del pasillo. Pero hay que saber que orden corresponde a que
# sentido, y eso es una observacion, no una deduccion.
#
# COMO SE USA (el robot NO se mueve solo; se empuja a mano):
#   1. Coloca el robot en la pista mirando en el sentido que quieras
#      llamar de referencia.
#   2. Lanza esto y empujalo a mano UNA VUELTA COMPLETA en ese sentido,
#      pasando por encima de las lineas de las cuatro esquinas.
#   3. El script imprime la secuencia de cruces. Si sale NARANJA -> AZUL
#      de forma consistente, ese es el orden de ese sentido.
#   4. Escribe el resultado en sentido_vuelta.py:
#         ORDEN_LINEAS = ("NARANJA", "AZUL")   # el par de ESE sentido
#         USAR_LINEAS  = True
#      con el convenio de que ese orden significa sentido +1 (ANTIHORARIO).
#      Si empujaste en horario, invierte el par.
#
# Tambien sirve de comprobacion del sensor: si no imprime nada, el
# TCS3472 no esta viendo las lineas y hay que mirar altura, iluminacion o
# la calibracion del blanco (`calibrar_suelo_inicial` en el firmware).
import sys, time
sys.path.insert(0, "/home/pi/ronda_curvas")
from enlace_pico import EnlacePico

SEGUNDOS = int(sys.argv[1]) if len(sys.argv) > 1 else 90

enlace = EnlacePico()
time.sleep(2.2)
print("[i] Empuja el robot A MANO una vuelta completa, en sentido conocido.")
print("[i] Grabando %d s. Ctrl+C para terminar antes.\n" % SEGUNDOS)

secuencia = []
ultimo = None
t0 = time.time()
try:
    while time.time() - t0 < SEGUNDOS:
        c = enlace.color_piso()
        if c in ("NARANJA", "AZUL") and c != ultimo:
            secuencia.append((time.time() - t0, c))
            ultimo = c
            print("  %6.1f s   %s" % secuencia[-1])
        elif c is None:
            ultimo = None          # volvio a pista blanca: listo para el siguiente
        time.sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    enlace.cerrar()

print("\n=== SECUENCIA (%d cruces) ===" % len(secuencia))
if len(secuencia) < 2:
    print("  Menos de dos cruces. El sensor no esta leyendo las lineas:")
    print("  revisa altura al suelo, iluminacion y que COLOR: no diga SIN_SENSOR.")
    sys.exit(1)
pares = {}
for (t1, a), (t2, b) in zip(secuencia, secuencia[1:]):
    if t2 - t1 < 3.0:              # dos lineas de la MISMA esquina
        pares[(a, b)] = pares.get((a, b), 0) + 1
print("  pares dentro de una misma esquina (<3 s):")
for k, v in sorted(pares.items(), key=lambda x: -x[1]):
    print("    %-8s -> %-8s   x%d" % (k[0], k[1], v))
if pares:
    mejor = max(pares, key=pares.get)
    print("\n  >>> ORDEN_LINEAS = (\"%s\", \"%s\")" % mejor)
    print("  >>> USAR_LINEAS  = True")
    print("  >>> (ese par significa el sentido en el que acabas de empujar)")
