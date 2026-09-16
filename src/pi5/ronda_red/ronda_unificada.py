# Ronda unificada: salida del estacionamiento + ronda con obstaculos.
#
# Une dos cosas que ya funcionaban por separado, sin tocar ninguna:
#
#   ~/ronda_curvas/salida_manual.py   la salida por pasos, con el sentido
#                                     decidido por LiDAR / sorteo / camara
#   ronda_camara.py (este paquete)    la carrera, con la red y los dos
#                                     pilares encendidos por defecto
#
# El callback de carrera, el registro, el apagado y el autochequeo son los
# de ronda_camara.py, importados, no copiados: lo que corre aqui es lo
# mismo que paso las corridas del banco del 16-09.
#
# ORDEN, Y POR QUE. salida_manual.py abre por su cuenta la camara, el LiDAR
# y la Pico, asi que durante la salida este proceso no puede tener ninguno
# de los tres abierto. Todo se abre al terminarla. La camara va primero y
# se le da un segundo con el motor parado: necesita ese tiempo para
# asentar la exposicion, y arrancar la carrera con el primer frame bueno
# es justo lo que hacia que al principio tardase en reconocer el color.
#
# EL SENTIDO. La salida deja escrito el sentido en
# ~/ronda_curvas/logs/sentido_ronda.txt y aqui se siembra en la carrera
# (revisable, ver SentidoPorGeometria.sembrar). Solo se acepta si el
# archivo es de ESTA salida: el de una corrida anterior daria un sentido
# viejo con toda la apariencia de uno bueno.
#
# PARADA. Solo SIGINT. Durante la salida el SIGINT se reenvia al proceso
# hijo, que para su propio motor, y se le espera: `subprocess.run` lo
# mataria con SIGKILL al interrumpirse y dejaria el motor con la ultima
# consigna.
import os

# Antes de importar vision y navegacion, que leen estas variables al cargar.
# setdefault: una variable puesta a mano sigue mandando.
os.environ.setdefault("WRO_RED", "1")
os.environ.setdefault("WRO_RED_HZ", "8")
os.environ.setdefault("WRO_DOS_PILARES", "activo")
os.environ.pop("WRO_ARRANQUE_AUTO", None)

import signal
import subprocess
import sys
import threading
import time

import RPi.GPIO as GPIO

import navegacion
import ronda_camara as rc
import vision
from camara_driver import CamaraDriver
from enlace_pico import EnlacePico
from lidar_driver import LidarDriver
from lidar_geometria import ProcesadorLidar
from registro_metricas import RegistroMetricas

SALIDA_MANUAL = os.path.expanduser("~/ronda_curvas/salida_manual.py")
ARCHIVO_SENTIDO = os.path.expanduser("~/ronda_curvas/logs/sentido_ronda.txt")
ASENTAR_CAMARA_S = 1.0


def leer_sentido(desde):
    """Sentido que dejo ESTA salida, o 0 si no hay uno fiable."""
    try:
        if os.path.getmtime(ARCHIVO_SENTIDO) < desde:
            print("[-] %s es de una salida anterior: no se usa." % ARCHIVO_SENTIDO)
            return 0
        with open(ARCHIVO_SENTIDO) as f:
            valor = int(f.read().strip())
    except (OSError, ValueError) as e:
        print("[-] Sin sentido de la salida (%s)." % e)
        return 0
    return valor if valor in (-1, 1) else 0


def correr_salida():
    """Ejecuta la salida y devuelve (codigo, sentido)."""
    if not os.path.exists(SALIDA_MANUAL):
        print("[-] No existe %s." % SALIDA_MANUAL)
        return 1, 0
    print("\n=== FASE 1: SALIDA DEL ESTACIONAMIENTO ===")
    t0 = time.time()
    hijo = subprocess.Popen([sys.executable, "-u", SALIDA_MANUAL],
                            cwd=os.path.dirname(SALIDA_MANUAL))
    cortado = [False]

    def reenviar(*_):
        cortado[0] = True
        print("\n[!] SIGINT durante la salida: se reenvia y se espera.")
        if hijo.poll() is None:
            hijo.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, reenviar)
    codigo = hijo.wait()
    signal.signal(signal.SIGINT, rc.apagar_sistema)
    if cortado[0]:
        return 130, 0
    return codigo, leer_sentido(t0)


def main():
    signal.signal(signal.SIGINT, rc.apagar_sistema)
    rc.preparar_gpio()

    print("\n[LISTO] RONDA UNIFICADA. Robot en la bahia de estacionamiento; "
          "presiona el Boton (GP21)...")
    while GPIO.input(rc.PIN_BOTON) == GPIO.HIGH:
        time.sleep(0.05)
    print("\n[START] Boton detectado! Saliendo del estacionamiento...")

    codigo, sentido = correr_salida()
    if codigo != 0:
        # Sin salida completa el robot sigue dentro o a medias: correr desde
        # ahi es chocar. Se queda quieto.
        print("[-] La salida termino con codigo %d. NO se arranca la carrera." % codigo)
        rc.apagar_sistema()
        return

    print("\n=== FASE 2: CARRERA ===")
    camara = CamaraDriver()
    threading.Thread(target=camara.hilo_captura,
                     args=(lambda: rc.corriendo, vision.procesar_frame),
                     daemon=True).start()
    try:
        rc.enlace = EnlacePico()
        print("[+] Conexion serial establecida con Raspberry Pi Pico 2.")
    except Exception as e:
        print("[-] Error conectando a la Pi Pico 2: %s" % e)
        rc.apagar_sistema()
        return
    fin = time.time() + ASENTAR_CAMARA_S
    while time.time() < fin:
        rc.enlace.enviar(0, 0.0)
        time.sleep(0.05)

    rc.enlace.fijar_cero()
    rc.registro = RegistroMetricas("ronda_unificada")
    rc.lidar_driver = LidarDriver()
    rc.lidar_geo = ProcesadorLidar()
    rc._comprobar_ultrasonido()
    rc.navegador = navegacion.Navegador(control_sector=rc.lidar_geo)
    if sentido:
        rc.navegador.pista.sembrar(sentido)
    print("[+] Red %s, dos pilares %s." % (
        "encendida" if vision._RED else "APAGADA", os.environ["WRO_DOS_PILARES"]))
    rc._t_ultimo_barrido = time.time()
    threading.Thread(target=rc.lidar_driver.hilo_lectura,
                     args=(lambda: rc.corriendo, rc.al_barrido), daemon=True).start()

    while rc.corriendo:
        sin_barridos = time.time() - rc._t_ultimo_barrido
        if sin_barridos > rc.WATCHDOG_LIDAR and rc.navegador.fase in ("CARRERA", "PARQUEO"):
            rc.enlace.enviar(0, 0.0)
            if sin_barridos > 5.0:
                print("[-] LiDAR sin datos por 5s. Abortando carrera.")
                rc.apagar_sistema()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
