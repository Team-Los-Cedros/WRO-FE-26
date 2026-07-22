# /home/pi/ronda_abierta.py
#
# Ronda Abierta - punto de entrada. Reutiliza los drivers compartidos con
# la Ronda Cerrada (src/pi3B/comun/): protocolo binario del LiDAR,
# geometria de paredes (modo Inercial) y el enlace serial con la Pico.
# Solo implementa lo propio de esta ronda -- seguimiento de pared simple
# y deteccion de parqueo por firma de pared -- sin camara ni evasion.
import sys
import time
import signal
import threading

import RPi.GPIO as GPIO

from lidar_driver import LidarDriver
from lidar_geometria import ProcesadorLidar
from enlace_pico import EnlacePico

PIN_BOTON = 21

KP_LATERAL        = 0.14
VELOCIDAD_CRUCERO = 100
VELOCIDAD_PARQUEO = 60

# Limites mecanicos del servo (deben coincidir con CENTRO/LIMITE_DER/
# LIMITE_IZQ de src/pico/main.py)
SERVO_CENTRO  = 90
SERVO_MAX_DER = 70
SERVO_MAX_IZQ = 115
DELTA_MAX_DER = SERVO_MAX_DER - SERVO_CENTRO
DELTA_MAX_IZQ = SERVO_MAX_IZQ - SERVO_CENTRO

# Limita la variacion maxima de angulo por ciclo para evitar giros bruscos
# (mismo mecanismo que ronda_cerrada.py, ya validado en pista)
MAX_DELTA_ANGULO_POR_CICLO = 6.0

TIMEOUT_BUSQUEDA_PARQUEO = 4.0
UMBRAL_VUELTAS           = 1010.0  # grados de yaw neto, ~3 vueltas
TOLERANCIA_FIRMA         = 80.0    # mm contra la firma de pared inicial

corriendo    = True
enlace       = None
lidar_driver = None
lidar_geo    = None

fase_actual      = "ESPERANDO_BOTON"
firma_izquierda  = 0.0
firma_derecha    = 0.0
t_inicio_parqueo = 0.0
ultimo_angulo    = 0.0

_apagando = False


def apagar_sistema(sig=None, frame=None):
    global corriendo, _apagando
    if _apagando:                 # doble Ctrl+C no debe reentrar aca
        return
    _apagando = True
    print("\n[!] Deteniendo sistema de forma segura...")
    corriendo = False
    time.sleep(0.2)
    if enlace:
        enlace.cerrar()
    if lidar_driver:
        lidar_driver.cerrar()
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f"[-] GPIO.cleanup() fallo (ignorado): {e}")
    sys.exit(0)


def al_barrido(scan):
    # Callback del hilo LiDAR: un ciclo de decision por barrido completo
    global fase_actual, firma_izquierda, firma_derecha, t_inicio_parqueo, ultimo_angulo

    medicion = lidar_geo.procesar(scan)

    if fase_actual == "CAPTURA_INICIAL":
        firma_izquierda, firma_derecha = medicion.izquierda, medicion.derecha
        fase_actual = "CARRERA"
        print(f"[+] Firma de parqueo: Izq={firma_izquierda:.0f} Der={firma_derecha:.0f}mm")
        print("[INICIO] Corriendo")
        return

    # Centrado proporcional entre paredes, con los mismos limites fisicos
    # y el mismo limitador de tasa que ya estaban validados en pista
    error_lateral   = medicion.izquierda - medicion.derecha
    angulo_crudo    = max(DELTA_MAX_DER, min(DELTA_MAX_IZQ, error_lateral * KP_LATERAL))
    delta           = max(-MAX_DELTA_ANGULO_POR_CICLO,
                           min(MAX_DELTA_ANGULO_POR_CICLO, angulo_crudo - ultimo_angulo))
    angulo_objetivo = ultimo_angulo + delta
    ultimo_angulo   = angulo_objetivo

    if fase_actual == "CARRERA":
        enlace.enviar(VELOCIDAD_CRUCERO, angulo_objetivo)

        # Fin de vuelta 3 -> parqueo. abs() porque el sentido de giro de
        # la pista (horario o antihorario) no se conoce de antemano.
        heading = enlace.heading()
        if abs(heading) >= UMBRAL_VUELTAS:
            fase_actual      = "BUSCANDO_PARQUEO"
            t_inicio_parqueo = time.time()
            print(f"[!] Ultima vuelta completada ({heading:.1f} deg). Modo Parqueo.")

    elif fase_actual == "BUSCANDO_PARQUEO":
        enlace.enviar(VELOCIDAD_PARQUEO, angulo_objetivo)

        match_firma = (abs(medicion.derecha - firma_derecha) < TOLERANCIA_FIRMA and
                       abs(medicion.izquierda - firma_izquierda) < TOLERANCIA_FIRMA)
        timeout     = (time.time() - t_inicio_parqueo) > TIMEOUT_BUSQUEDA_PARQUEO

        if match_firma or timeout:
            print("[PARQUEO] " + ("Firma detectada! Estacionando..." if match_firma
                                  else "Timeout. Deteniendo en zona segura."))
            apagar_sistema()


def preparar_gpio():
    GPIO.setmode(GPIO.BCM)
    try:
        GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    except Exception as e:
        print(f"[!] GPIO ocupado, liberando y reintentando... ({e})")
        try:
            GPIO.cleanup()
        except Exception:
            pass
        time.sleep(0.3)
        GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, apagar_sistema)
    preparar_gpio()

    try:
        enlace = EnlacePico()
        print("[+] Conexion serial establecida con Raspberry Pi Pico 2.")
    except Exception as e:
        print(f"[-] Error conectando a la Pi Pico 2: {e}")
        sys.exit(1)
    enlace.enviar(0, 0.0)
    print(f"[INIT] Direccion alineada y bloqueada en el centro ({SERVO_CENTRO} grados).")

    print("\n[LISTO] SISTEMA LISTO. Coloca el robot y presiona el Boton (GP21)...")
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        enlace.enviar(0, 0.0)
        time.sleep(0.05)

    print("\n[START] Boton detectado! Iniciando carrera...")
    enlace.fijar_cero()           # el yaw de este instante es el 0 de carrera
    fase_actual = "CAPTURA_INICIAL"

    # El LiDAR arranca despues del boton para que su primer barrido
    # capture la firma de pared del punto de partida
    lidar_driver = LidarDriver()
    lidar_geo    = ProcesadorLidar()
    threading.Thread(target=lidar_driver.hilo_lectura,
                     args=(lambda: corriendo, al_barrido), daemon=True).start()

    while corriendo:
        time.sleep(1)
