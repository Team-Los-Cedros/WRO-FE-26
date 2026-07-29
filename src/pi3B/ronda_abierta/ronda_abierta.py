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
from registro_metricas import RegistroMetricas

PIN_BOTON = 21

PIN_BOTON = 21

# Distancia del LiDAR al eje trasero (centro de giro Ackermann) en mm
OFFSET_LIDAR_EJE_TRASERO = 204.0

# Parametros del controlador PD + Guiñada Muro + Lookahead
KP_LATERAL        = 0.08
KD_LATERAL        = 0.05
K_ANGULO_MURO     = 0.6
K_LOOKAHEAD_CURVA = 16.0
DIST_LOOKAHEAD    = 850.0   # mm, empieza a anticipar giro si d_frontal < 850mm

# Guardias de seguridad (evita atascamiento contra pared)
DIST_MIN_FRONTAL_RETROCESO = 150.0  # mm
DIST_MIN_PARED_RETROCESO   = 60.0   # mm

VELOCIDAD_CRUCERO = 100
VELOCIDAD_PARQUEO = 60

# Limites mecanicos del servo (deben coincidir con CENTRO/LIMITE_DER/
# LIMITE_IZQ de src/pico/main.py)
SERVO_CENTRO  = 90
SERVO_MAX_DER = 70
SERVO_MAX_IZQ = 115
DELTA_MAX_DER = SERVO_MAX_DER - SERVO_CENTRO
DELTA_MAX_IZQ = SERVO_MAX_IZQ - SERVO_CENTRO

# Rate limiter para suavizar variacion angular por ciclo (10 Hz)
MAX_DELTA_ANGULO_POR_CICLO = 12.0

TIMEOUT_BUSQUEDA_PARQUEO = 4.0
UMBRAL_VUELTAS           = 1010.0  # grados de yaw neto, ~3 vueltas
TOLERANCIA_FIRMA         = 80.0    # mm contra la firma de pared inicial

corriendo    = True
enlace       = None
lidar_driver = None
lidar_geo    = None
registro     = None

fase_actual      = "ESPERANDO_BOTON"
firma_izquierda  = 0.0
firma_derecha    = 0.0
t_inicio_parqueo = 0.0
ultimo_angulo    = 0.0
ultimo_error_eje = 0.0
ultimo_t         = time.time()

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
    if registro:
        registro.cerrar()
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f"[-] GPIO.cleanup() fallo (ignorado): {e}")
    sys.exit(0)


def al_barrido(scan):
    # Callback del hilo LiDAR: un ciclo de decision por barrido completo
    global fase_actual, firma_izquierda, firma_derecha, t_inicio_parqueo
    global ultimo_angulo, ultimo_error_eje, ultimo_t

    medicion = lidar_geo.procesar(scan)

    if fase_actual == "CAPTURA_INICIAL":
        firma_izquierda, firma_derecha = medicion.d_perp_izq, medicion.d_perp_der
        fase_actual = "CARRERA"
        ultimo_t = time.time()
        print(f"[+] Firma de parqueo: Izq={firma_izquierda:.0f} Der={firma_derecha:.0f}mm")
        print("[INICIO] Corriendo")
        return

    t_actual = time.time()
    dt = max(0.01, t_actual - ultimo_t)
    ultimo_t = t_actual

    # 1. Guardia de Seguridad / Retroceso si esta peligrosamente cerca de la pared
    if medicion.frontal < DIST_MIN_FRONTAL_RETROCESO or min(medicion.d_perp_izq, medicion.d_perp_der) < DIST_MIN_PARED_RETROCESO:
        print("[!] ALERTA COLISION: Distancia critica detectada. Aplicando retroceso de emergencia.")
        enlace.enviar(-60, 0.0)
        time.sleep(0.35)
        return

    # 2. Compensacion cinematica de la posicion del LiDAR (20.4 cm delante del eje trasero)
    ang_rad = math.radians(medicion.angulo_muro)
    error_front = medicion.d_perp_izq - medicion.d_perp_der
    comp_cinematica = 2.0 * OFFSET_LIDAR_EJE_TRASERO * math.sin(ang_rad)
    error_eje = error_front + comp_cinematica

    # 3. Termino Derivativo del error en el eje
    der_error = (error_eje - ultimo_error_eje) / dt
    ultimo_error_eje = error_eje

    # 4. Ley de Control Base (PD Lateral + Guiñada Paralela a Muros)
    angulo_base = (error_eje * KP_LATERAL) + (der_error * KD_LATERAL) - (medicion.angulo_muro * K_ANGULO_MURO)

    # 5. Anticipacion de Curvas por LiDAR Frontal (Lookahead)
    angulo_curva = 0.0
    if medicion.frontal < DIST_LOOKAHEAD:
        factor_dist = (1.0 - (medicion.frontal / DIST_LOOKAHEAD))
        bias_lado = 1.0 if medicion.d_perp_izq > medicion.d_perp_der else -1.0
        angulo_curva = bias_lado * factor_dist * K_LOOKAHEAD_CURVA

    angulo_crudo = max(DELTA_MAX_DER, min(DELTA_MAX_IZQ, angulo_base + angulo_curva))

    # Rate Limiter suave
    delta = max(-MAX_DELTA_ANGULO_POR_CICLO,
                min(MAX_DELTA_ANGULO_POR_CICLO, angulo_crudo - ultimo_angulo))
    angulo_objetivo = ultimo_angulo + delta
    ultimo_angulo   = angulo_objetivo

    heading = enlace.heading()

    if fase_actual == "CARRERA":
        enlace.enviar(VELOCIDAD_CRUCERO, angulo_objetivo)
        if registro:
            registro.registrar(fase=fase_actual, heading=f"{heading:.2f}",
                                error_lateral=f"{error_eje:.1f}",
                                angulo=f"{angulo_objetivo:.2f}", velocidad=VELOCIDAD_CRUCERO)

        if abs(heading) >= UMBRAL_VUELTAS:
            fase_actual      = "BUSCANDO_PARQUEO"
            t_inicio_parqueo = time.time()
            print(f"[!] Ultima vuelta completada ({heading:.1f} deg). Modo Parqueo.")

    elif fase_actual == "BUSCANDO_PARQUEO":
        enlace.enviar(VELOCIDAD_PARQUEO, angulo_objetivo)
        if registro:
            registro.registrar(fase=fase_actual, heading=f"{heading:.2f}",
                                error_lateral=f"{error_eje:.1f}",
                                angulo=f"{angulo_objetivo:.2f}", velocidad=VELOCIDAD_PARQUEO)

        match_firma = (abs(medicion.d_perp_der - firma_derecha) < TOLERANCIA_FIRMA and
                       abs(medicion.d_perp_izq - firma_izquierda) < TOLERANCIA_FIRMA)
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
    registro = RegistroMetricas("ronda_abierta")

    # El LiDAR arranca despues del boton para que su primer barrido
    # capture la firma de pared del punto de partida
    lidar_driver = LidarDriver()
    lidar_geo    = ProcesadorLidar()
    threading.Thread(target=lidar_driver.hilo_lectura,
                     args=(lambda: corriendo, al_barrido), daemon=True).start()

    while corriendo:
        time.sleep(1)
