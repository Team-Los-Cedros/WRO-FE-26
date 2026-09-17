# Mide el RADIO DE GIRO REAL con cinta metrica, no con reloj.
#
# Por que hace falta: geometria_evasion.RADIO_MIN_IZQ/DER (260/360) estan
# marcados "[DERIVADO, NO MEDIDO DIRECTAMENTE]" -- salen de dividir una
# velocidad de rumbo por una velocidad lineal, las dos estimadas, y ADEMAS
# se tomaron con la amortiguacion por giroscopio del firmware activa, que
# desvia el servo unos 2 grados en giro sostenido. El banco del 06-09
# dedujo 228/260 por otra via. Toda la geometria de la evasion y del
# termino de esquina descansa en estos dos numeros.
#
# Metodo: el robot gira a tope hasta que la IMU acumula 90 grados y para
# solo. La cuerda entre la marca inicial y la final da el radio sin
# depender de la velocidad ni del tiempo:
#
#       R = cuerda / raiz(2)          (cuerda de un cuarto de circunferencia)
#
# kd=0 en la consigna apaga la amortiguacion, para que el angulo que se
# manda sea el que llega al servo.
#
#   uso:  python3 medir_radio.py IZQ|DER [--reversa] [--velocidad 25]
#
# ANTES DE CORRERLO: espacio libre de ~1 m a los lados y por delante (o
# por detras en reversa), y marcar en el suelo el punto del EJE TRASERO
# (entre las dos ruedas de atras) y hacia donde apunta el morro.
import sys, time, math
sys.path.insert(0, "/home/pi/ronda_curvas")
from enlace_pico import EnlacePico
import geometria_evasion as gev

LADO = (sys.argv[1].upper() if len(sys.argv) > 1 else "IZQ")
REVERSA = "--reversa" in sys.argv
VEL = 25
if "--velocidad" in sys.argv:
    VEL = int(sys.argv[sys.argv.index("--velocidad") + 1])
if REVERSA:
    VEL = -VEL

ANGULO = gev.COMANDO_MAX_IZQ if LADO == "IZQ" else -gev.COMANDO_MAX_DER
OBJETIVO = 90.0
TIMEOUT = 20.0

enlace = EnlacePico()
time.sleep(2.2)
if not enlace.heading_valido():
    print("[-] La Pico no reporta IMU. Sin yaw no se puede parar en 90 grados.")
    sys.exit(1)

enlace.fijar_cero()
time.sleep(0.2)
print("[i] lado=%s  reversa=%s  angulo=%+.1f  velocidad=%d  kd=0 (amortiguacion OFF)"
      % (LADO, REVERSA, ANGULO, VEL))
print("[i] marca AHORA el eje trasero y el rumbo. Arranca en 3 s...")
for q in (3, 2, 1):
    enlace.enviar(0, ANGULO, kd=0.0)   # pre-posiciona el volante quieto
    time.sleep(1.0)

t0 = time.time()
pico_yaw = 0.0
try:
    while True:
        h = abs(enlace.heading())
        pico_yaw = max(pico_yaw, h)
        if h >= OBJETIVO or (time.time() - t0) > TIMEOUT:
            break
        enlace.enviar(VEL, ANGULO, kd=0.0)
        time.sleep(0.02)
finally:
    enlace.detener()

dt = time.time() - t0
yaw = enlace.heading()
print("\n[+] parado con yaw = %+.1f grados en %.2f s  (%.1f grados/s)"
      % (yaw, dt, abs(yaw) / dt if dt else 0))
if abs(yaw) < 80.0:
    print("[!] no llego a 90 grados: repite con mas espacio o mas tiempo.")
print("\n>>> MIDE AHORA la cuerda entre las dos marcas del eje trasero, en mm,")
print(">>> y calcula:   R = cuerda / 1.4142")
print(">>>")
print(">>> El modelo cree ahora mismo:  IZQ=%.0f mm   DER=%.0f mm"
      % (gev.RADIO_MIN_IZQ, gev.RADIO_MIN_DER))
print(">>> El banco del 06-09 dedujo:   IZQ=228 mm   DER=260 mm")
enlace.cerrar()
