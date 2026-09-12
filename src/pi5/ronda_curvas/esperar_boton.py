#!/usr/bin/env python3
"""Puerta de arranque de la ronda: LED parpadeando = listo, boton = ya.

Existe porque la secuencia de competencia son DOS procesos encadenados
(parqueo.py y luego ronda_camara.py) y el boton tiene que abrir el
primero, no el segundo. Antes solo `ronda_camara.py` esperaba el boton,
asi que la salida del estacionamiento arrancaba sola.

Lo que hace, y nada mas:

  1. Abre el enlace con la Pico y centra la direccion.
  2. Pone el LED de la Pico a PARPADEAR: eso significa "cargado y listo".
  3. Espera a que se pulse el boton (GP21 de la Raspberry).
  4. Apaga el LED y suelta el puerto y el GPIO para que los abra el
     proceso siguiente.

Sale con 0 si se pulso el boton y con 1 si se corto antes. El lanzador
mira ese codigo para no arrancar una ronda que nadie pidio.

  WRO_ARRANQUE_AUTO=1  se salta el boton (depuracion por SSH). En
  competencia no se usa: el reglamento pide una accion fisica sobre el
  robot.
"""
import os
import signal
import sys
import time

sys.path.insert(0, "/home/pi/ronda_curvas")

import RPi.GPIO as GPIO                    # noqa: E402
from enlace_pico import EnlacePico         # noqa: E402

PIN_BOTON = 21
HZ = 20.0


def preparar_gpio():
    GPIO.setmode(GPIO.BCM)
    try:
        GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        return True
    except Exception as e:
        print("[!] GPIO ocupado, liberando y reintentando... (%s)" % e)
        try:
            GPIO.cleanup()
        except Exception:
            pass
        time.sleep(0.3)
        # cleanup() borra tambien el modo de numeracion: hay que volver a
        # fijarlo ANTES de reintentar, o el error de verdad queda tapado.
        GPIO.setmode(GPIO.BCM)
        try:
            GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            return True
        except Exception:
            print("")
            print("[-] EL GPIO ESTA OCUPADO POR OTRO PROCESO.")
            print("    Casi seguro es el servicio de arranque automatico:")
            print("")
            print("      sudo systemctl stop wro.service")
            print("")
            return False


def main():
    enlace = None

    def al_cortar(*_):
        if enlace is not None:
            enlace.led("OFF")
            enlace.cerrar()
        try:
            GPIO.cleanup()
        except Exception:
            pass
        sys.exit(1)

    signal.signal(signal.SIGINT, al_cortar)
    signal.signal(signal.SIGTERM, al_cortar)

    try:
        enlace = EnlacePico()
    except Exception as e:
        print("[-] no hay enlace con la Pico: %s" % e)
        return 1
    enlace.enviar(0, 0.0)

    if os.environ.get("WRO_ARRANQUE_AUTO") == "1":
        print("[LISTO] arranque automatico (depuracion, sin boton).")
        enlace.cerrar()
        return 0

    if not preparar_gpio():
        enlace.cerrar()
        return 1

    enlace.led("BLINK")
    # El aviso cambia con la ronda: en la abierta no hay estacionamiento,
    # y leer "colocalo en el estacionamiento" ahi solo confunde.
    ronda = sys.argv[1] if len(sys.argv) > 1 else "obstaculos"
    donde = ("en el tramo de salida que te toco"
             if ronda == "abierta" else "en el estacionamiento")
    print("[LISTO] LED PARPADEANDO = cargado y listo (ronda %s)." % ronda)
    print("        Coloca el robot %s y pulsa el boton (GP21)." % donde)
    try:
        while GPIO.input(PIN_BOTON) == GPIO.HIGH:
            # Se sigue mandando consigna cero: mantiene la direccion
            # centrada y el watchdog de la Pico contento mientras espera.
            enlace.enviar(0, 0.0)
            time.sleep(1.0 / HZ)
    finally:
        enlace.led("OFF")

    print("[START] boton pulsado.")
    enlace.cerrar()
    # Hay que soltar el GPIO: el proceso de carrera lo vuelve a pedir.
    try:
        GPIO.cleanup()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
