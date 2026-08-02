# /home/pi/ronda_abierta.py
#
# Ronda Abierta - punto de entrada, autocontenido. Implementa su propio
# parseo del protocolo binario del RPLIDAR C1 y su propio manejo serial
# de la Pico 2, sin depender de src/pi3B/comun/ -- version validada en
# pista con estacionamiento por firma de pared incluido.
import time
import threading
import serial
import sys
import signal
import RPi.GPIO as GPIO

from registro_metricas import RegistroMetricas

# ==========================================
# CONFIGURACIÓN DE PUERTOS Y COMUNICACIÓN
# ==========================================
PUERTO_LIDAR = '/dev/ttyUSB0'
PUERTO_PICO = '/dev/ttyACM0'
BAUDRATE_LIDAR = 460800
BAUDRATE_PICO = 115200

START_MOTOR_CMD = b'\xa5\xf0\x02\x94\x02\xc1\x02'
START_SCAN_CMD = b'\xa5\x20'
STOP_CMD = b'\xa5\x25'

corriendo = True
ser_lidar = None
ser_pico = None
registro = None

# ==========================================
# CONFIGURACIÓN DEL BOTÓN DE ARRANQUE (GP21)
# ==========================================
PIN_BOTON = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ==========================================
# CONSTANTES DE NAVEGACIÓN Y CONFIGURACIÓN
# ==========================================
KP_LATERAL = 0.14
VELOCIDAD_CRUCERO = 90
VELOCIDAD_PARQUEO = 40

# Tiempo máximo de búsqueda de estacionamiento, en segundos
TIMEOUT_BUSQUEDA_PARQUEO = 10.0
tiempo_inicio_parqueo = 0.0

ANGULO_MIN_DER = 30
ANGULO_MAX_DER = 90
ANGULO_MIN_IZQ = 270
ANGULO_MAX_IZQ = 330

dist_derecha_min = 8000.0
dist_izquierda_min = 8000.0
angulo_previo = 0.0

# Fases de la carrera
fase_actual = "ESPERANDO_BOTON"
initial_derecha = 0.0
initial_izquierda = 0.0

# Control de desfase para la telemetría de la Pico 2
angulo_inicial_imu = None  # Valor base con el que arranca la carrera
angulo_acumulado_robot = 0.0


def apagar_sistema(sig, frame):
    global corriendo, ser_lidar, ser_pico
    print("\n[!] Deteniendo sistema de forma segura...")
    corriendo = False
    time.sleep(0.2)
    if ser_pico and ser_pico.is_open:
        try:
            ser_pico.write(b"0,0\n")
            ser_pico.close()
        except Exception:
            pass
    if ser_lidar and ser_lidar.is_open:
        try:
            ser_lidar.write(STOP_CMD)
            ser_lidar.close()
        except Exception:
            pass
    if registro:
        registro.cerrar()
    GPIO.cleanup()
    sys.exit(0)


signal.signal(signal.SIGINT, apagar_sistema)


def hilo_comunicacion_pico():
    global ser_pico, angulo_acumulado_robot, fase_actual, tiempo_inicio_parqueo, angulo_inicial_imu
    try:
        ser_pico = serial.Serial(PUERTO_PICO, baudrate=BAUDRATE_PICO, timeout=0.05)
        print("[+] Conexion serial establecida con Raspberry Pi Pico 2.")
    except Exception as e:
        print(f"[-] Error conectando a la Pi Pico 2: {e}")
        return

    while corriendo:
        if ser_pico.in_waiting > 0:
            try:
                linea = ser_pico.readline().decode('utf-8').strip()
                if linea.startswith("IMU:"):
                    valor_crudo_imu = abs(float(linea.split(":")[1]))

                    # Mientras se espera el boton o se calibra, el cero se
                    # sigue actualizando con cada lectura
                    if fase_actual in ["ESPERANDO_BOTON", "CALIBRANDO"] or angulo_inicial_imu is None:
                        angulo_inicial_imu = valor_crudo_imu

                    # Angulo neto de esta carrera: valor actual menos el cero
                    angulo_acumulado_robot = valor_crudo_imu - angulo_inicial_imu

                    # Transicion a parqueo al completar ~3 vueltas (1010 grados netos)
                    if fase_actual == "CARRERA" and angulo_acumulado_robot >= 1010.0:
                        fase_actual = "BUSCANDO_PARQUEO"
                        tiempo_inicio_parqueo = time.time()
                        print(f"[!] Ultima vuelta completada (angulo neto: {angulo_acumulado_robot:.1f} grados). Modo parqueo activo.")
            except Exception:
                pass
        time.sleep(0.01)


def procesar_ciclo_completo_lidar():
    global dist_derecha_min, dist_izquierda_min, fase_actual
    global initial_derecha, initial_izquierda, ser_pico, angulo_acumulado_robot, tiempo_inicio_parqueo

    if ser_pico is None or not ser_pico.is_open:
        return

    if dist_derecha_min > 4000:
        dist_derecha_min = 2000.0
    if dist_izquierda_min > 4000:
        dist_izquierda_min = 2000.0

    if fase_actual == "CAPTURA_INICIAL":
        initial_derecha = dist_derecha_min
        initial_izquierda = dist_izquierda_min
        fase_actual = "CARRERA"
        print(f"[+] Firma de parqueo guardada -> Izq: {initial_izquierda:.0f}mm | Der: {dist_derecha_min:.0f}mm")
        print("[INICIO] Comienza la carrera.")
        return

    if fase_actual == "CARRERA":
        error_lateral = dist_izquierda_min - dist_derecha_min
        angulo_objetivo = error_lateral * KP_LATERAL
        comando = f"{VELOCIDAD_CRUCERO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())
        if registro:
            registro.registrar(fase=fase_actual, heading=f"{angulo_acumulado_robot:.2f}",
                                error_lateral=f"{error_lateral:.1f}",
                                angulo=f"{angulo_objetivo:.2f}", velocidad=VELOCIDAD_CRUCERO)

    elif fase_actual == "BUSCANDO_PARQUEO":
        error_lateral = dist_izquierda_min - dist_derecha_min
        angulo_objetivo = error_lateral * KP_LATERAL
        comando = f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())
        if registro:
            registro.registrar(fase=fase_actual, heading=f"{angulo_acumulado_robot:.2f}",
                                error_lateral=f"{error_lateral:.1f}",
                                angulo=f"{angulo_objetivo:.2f}", velocidad=VELOCIDAD_PARQUEO)

        # Condiciones de parada: firma de pared coincide o se agoto el tiempo
        match_firma_original = abs(dist_derecha_min - initial_derecha) < 80.0 and abs(dist_izquierda_min - initial_izquierda) < 80.0
        tiempo_transcurrido = time.time() - tiempo_inicio_parqueo
        timeout_alcanzado = tiempo_transcurrido > TIMEOUT_BUSQUEDA_PARQUEO

        if match_firma_original or timeout_alcanzado:
            fase_actual = "DETENIDO"
            if timeout_alcanzado:
                print(f"[!] Deteccion por timeout ({tiempo_transcurrido:.1f}s). El robot se detuvo en zona segura.")
            else:
                print("[+] Firma de parqueo detectada geometricamente. Estacionando...")

            for _ in range(5):
                ser_pico.write(b"0,0\n")
                time.sleep(0.01)
            apagar_sistema(None, None)


def hilo_lidar():
    global ser_lidar, corriendo, angulo_previo
    global dist_derecha_min, dist_izquierda_min, fase_actual

    try:
        ser_lidar = serial.Serial(PUERTO_LIDAR, baudrate=BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
        ser_lidar.write(START_MOTOR_CMD)
        time.sleep(1.5)
        ser_lidar.reset_input_buffer()
        ser_lidar.write(START_SCAN_CMD)
        time.sleep(0.5)

        if ser_lidar.in_waiting >= 7:
            ser_lidar.read(7)

        print("[+] Telemetria LiDAR activa.")
        if fase_actual == "CALIBRANDO":
            fase_actual = "CAPTURA_INICIAL"

        while corriendo:
            if fase_actual == "ESPERANDO_BOTON":
                time.sleep(0.1)
                continue

            b0 = ser_lidar.read(1)
            if not b0:
                continue
            byte0 = b0[0]
            start_bit = byte0 & 0x01
            start_bit_inverse = (byte0 >> 1) & 0x01

            if start_bit != start_bit_inverse:
                resto = ser_lidar.read(4)
                if len(resto) < 4:
                    continue
                byte1, byte2, byte3, byte4 = resto[0], resto[1], resto[2], resto[3]

                if (byte1 & 0x01) == 1:
                    raw_angle = (byte2 << 7) | (byte1 >> 1)
                    angle = raw_angle / 64.0
                    distance = (byte4 << 8) | byte3
                    distance_mm = distance / 4.0

                    if 0 < distance_mm < 6000:
                        # Wrap-around del angulo: barrido completo listo
                        if angle < angulo_previo and (angulo_previo - angle) > 300.0:
                            procesar_ciclo_completo_lidar()
                            dist_derecha_min = 8000.0
                            dist_izquierda_min = 8000.0

                        angulo_previo = angle

                        if ANGULO_MIN_DER <= angle <= ANGULO_MAX_DER:
                            if distance_mm < dist_derecha_min:
                                dist_derecha_min = distance_mm
                        elif ANGULO_MIN_IZQ <= angle <= ANGULO_MAX_IZQ:
                            if distance_mm < dist_izquierda_min:
                                dist_izquierda_min = distance_mm

    except Exception as e:
        if corriendo:
            print(f"[-] Falla en el bucle LiDAR: {e}")


if __name__ == '__main__':
    # 1. Encender canal de comunicacion con la Pico
    t_pico = threading.Thread(target=hilo_comunicacion_pico, daemon=True)
    t_pico.start()

    # 2. Direccion recta previa
    time.sleep(0.5)
    if ser_pico and ser_pico.is_open:
        ser_pico.write(b"0,0\n")
        print("[+] Direccion alineada y bloqueada en el centro (90 grados).")

    # 3. Espera del boton de arranque
    print("\n[LISTO] Sistema listo. Coloca el robot en la salida y presiona el boton (GP21).")
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        if ser_pico and ser_pico.is_open:
            ser_pico.write(b"0,0\n")
        time.sleep(0.05)

    print("\n[START] Boton detectado. Reseteando odometria IMU local...")
    fase_actual = "CALIBRANDO"
    registro = RegistroMetricas("ronda_abierta")
    time.sleep(0.1)  # Breve pausa para asegurar la captura del cero absoluto

    # 4. Iniciar hilo del LiDAR justo despues del boton
    t_lidar = threading.Thread(target=hilo_lidar, daemon=True)
    t_lidar.start()

    while corriendo:
        time.sleep(1)
