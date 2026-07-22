# Ronda Cerrada - punto de entrada. Este script solo conecta las piezas:
#   camara_driver.py  hilo de adquisicion de frames (Pi Camera Module 3)
#   vision.py         procesa cada frame: HSV + histeresis
#   lidar_driver.py   hilo del RPLIDAR C1, entrega el barrido crudo
#   lidar_geometria.py  interpreta el barrido: paredes + clustering ABD
#   tracker.py         posicion del poste activo (lo usa navegacion)
#   navegacion.py      maquina de estados, decide (velocidad, angulo)
#   enlace_pico.py      serial con la Pico 2 (consignas + IMU)
#
# Secuencia: armar hilos -> esperar boton GP21 -> fijar cero IMU ->
# arrancar LiDAR -> por cada barrido navegacion decide y se manda a la
# Pico. El bucle principal solo vigila que el LiDAR siga vivo y apaga.
import sys
import time
import signal
import threading

import RPi.GPIO as GPIO

import vision
import navegacion
from camara_driver import CamaraDriver
from lidar_driver import LidarDriver
from lidar_geometria import ProcesadorLidar
from enlace_pico import EnlacePico

PIN_BOTON = 21

# Si el LiDAR no entrega barridos en este tiempo con el robot en marcha
# se corta la traccion: sin percepcion no se navega
WATCHDOG_LIDAR = 0.8

corriendo = True
enlace       = None
lidar_driver = None
lidar_geo    = None
navegador    = None

_t_ultimo_barrido = 0.0
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
    global _t_ultimo_barrido
    medicion = lidar_geo.procesar(scan)
    _t_ultimo_barrido = medicion.timestamp

    consigna = navegador.procesar(medicion, vision.get_color(), enlace.heading())
    if consigna is None:          # carrera terminada (parqueo o timeout)
        apagar_sistema()
        return
    enlace.enviar(*consigna)


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

    # 1. Camara primero, necesita ~1s para estabilizar la exposicion
    camara = CamaraDriver()
    threading.Thread(target=camara.hilo_captura,
                     args=(lambda: corriendo, vision.procesar_frame), daemon=True).start()

    # 2. Enlace con la Pico, direccion centrada mientras se espera
    try:
        enlace = EnlacePico()
        print("[+] Conexion serial establecida con Raspberry Pi Pico 2.")
    except Exception as e:
        print(f"[-] Error conectando a la Pi Pico 2: {e}")
        sys.exit(1)
    enlace.enviar(0, 0.0)

    print("\n[LISTO] SISTEMA LISTO (RONDA CON OBSTACULOS). "
          "Coloca el robot y presiona el Boton (GP21)...")
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        enlace.enviar(0, 0.0)
        time.sleep(0.05)

    print("\n[START] Boton detectado! Iniciando carrera con obstaculos...")
    enlace.fijar_cero()           # el yaw de este instante es el 0 de carrera

    # 3. LiDAR y navegacion. El LiDAR arranca despues del boton para que
    #    su primer barrido capture la firma de pared del punto de partida
    lidar_driver = LidarDriver()
    lidar_geo    = ProcesadorLidar()
    navegador    = navegacion.Navegador(control_sector=lidar_geo)
    _t_ultimo_barrido = time.time()
    threading.Thread(target=lidar_driver.hilo_lectura,
                     args=(lambda: corriendo, al_barrido), daemon=True).start()

    # 4. Vigilancia: si la percepcion muere el robot se detiene
    while corriendo:
        sin_barridos = time.time() - _t_ultimo_barrido
        if sin_barridos > WATCHDOG_LIDAR and navegador.fase in ("CARRERA", "PARQUEO"):
            enlace.enviar(0, 0.0)
            if sin_barridos > 5.0:
                print("[-] LiDAR sin datos por 5s. Abortando carrera.")
                apagar_sistema()
        time.sleep(0.1)
