import time
import threading
import serial
import sys
import signal
import RPi.GPIO as GPIO

# ==========================================
# CONFIGURACION DE PUERTOS Y COMUNICACION
# ==========================================
PUERTO_LIDAR = '/dev/ttyUSB0'
PUERTO_PICO = '/dev/ttyACM0'  
BAUDRATE_LIDAR = 460800
BAUDRATE_PICO = 115200

START_MOTOR_CMD = b'\xa5\xf0\x02\x94\x02\xc1\x02'
START_SCAN_CMD = b'\xa5\x20'
STOP_CMD = b'\xa5\x25'

# ==========================================
# CONFIGURACION DEL BOTON FISICO (GPIO 21)
# ==========================================
PIN_BOTON = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ==========================================
# CONSTANTES DE NAVEGACION Y CONTEO NARANJA
# ==========================================
KP_LATERAL = 0.14  
VELOCIDAD_CRUCERO = 90
VELOCIDAD_PARQUEO = 55

# Ajusta LINEAS_POR_VUELTA:
# - Pon 4 si el sentido de giro hace que pises 4 lineas naranjas por vuelta.
# - Pon 1 si solo hay 1 linea naranja por vuelta (linea de meta).
VUELTAS_OBJETIVO = 3
LINEAS_POR_VUELTA = 4
TOTAL_LINEAS_OBJETIVO = VUELTAS_OBJETIVO * LINEAS_POR_VUELTA

TIEMPO_AVANCE_70CM = 3.8  # Tiempo para avanzar los 70 cm dentro del cajon

# Sectores angulares del LiDAR (Angulos en grados)
ANGULO_MIN_DER = 30
ANGULO_MAX_DER = 90
ANGULO_MIN_IZQ = 270
ANGULO_MAX_IZQ = 330

# ==========================================
# VARIABLES GLOBALES DE ESTADO
# ==========================================
corriendo = True
ser_lidar = None
ser_pico = None

dist_derecha_min = 8000.0
dist_izquierda_min = 8000.0
angulo_previo = 0.0

fase_actual = "ESPERANDO_BOTON"
initial_derecha = 0.0
initial_izquierda = 0.0

angulo_inicial_imu = None
angulo_acumulado_robot = 0.0
color_actual = "PISTA"

lineas_naranjas_detectadas = 0
en_linea_color = False
ultimo_tiempo_linea = 0.0
tiempo_fuera_linea = 0.0

tiempo_inicio_parqueo = 0.0
tiempo_inicio_avance = 0.0


def apagar_sistema(sig, frame):
    global corriendo, ser_lidar, ser_pico
    print("\n[INFO] Deteniendo sistema de forma segura...")
    corriendo = False
    time.sleep(0.2)
    if ser_pico and ser_pico.is_open:
        try:
            for _ in range(5):
                ser_pico.write(b"0,0\n")
                time.sleep(0.01)
            ser_pico.close()
        except Exception:
            pass
    if ser_lidar and ser_lidar.is_open:
        try:
            ser_lidar.write(STOP_CMD)
            ser_lidar.close()
        except Exception:
            pass
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, apagar_sistema)


def hilo_comunicacion_pico():
    global ser_pico, angulo_acumulado_robot, fase_actual, tiempo_inicio_parqueo
    global angulo_inicial_imu, color_actual, lineas_naranjas_detectadas
    global en_linea_color, ultimo_tiempo_linea, tiempo_fuera_linea, tiempo_inicio_avance

    try:
        ser_pico = serial.Serial(PUERTO_PICO, baudrate=BAUDRATE_PICO, timeout=0.05)
        print("[INFO] Conexion serial establecida con Raspberry Pi Pico 2.")
    except Exception as e:
        print(f"[ERROR] No se pudo conectar a la Pi Pico 2: {e}")
        return

    while corriendo:
        if ser_pico.in_waiting > 0:
            try:
                linea = ser_pico.readline().decode('utf-8').strip()
                
                if "IMU:" in linea and "COLOR:" in linea:
                    partes = linea.split(',')
                    valor_crudo_imu = abs(float(partes[0].split(':')[1]))
                    color_actual = partes[1].split(':')[1]

                    if fase_actual in ["ESPERANDO_BOTON", "CALIBRANDO"] or angulo_inicial_imu is None:
                        angulo_inicial_imu = valor_crudo_imu

                    angulo_acumulado_robot = valor_crudo_imu - angulo_inicial_imu
                    tiempo_actual = time.time()

                    # Evaluacion exclusiva para lineas NARANJAS (Ignora Azul)
                    if color_actual == "NARANJA":
                        if not en_linea_color and (tiempo_actual - ultimo_tiempo_linea > 1.2):
                            en_linea_color = True
                            lineas_naranjas_detectadas += 1
                            ultimo_tiempo_linea = tiempo_actual
                            tiempo_fuera_linea = 0.0
                            print(f"[COLOR] Linea NARANJA #{lineas_naranjas_detectadas} detectada.")

                            # Al detectar la penultima linea naranja, reduce velocidad
                            if lineas_naranjas_detectadas == (TOTAL_LINEAS_OBJETIVO - 1):
                                fase_actual = "BUSCANDO_PARQUEO"
                                tiempo_inicio_parqueo = time.time()
                                print(f"[INFO] Linea naranja #{lineas_naranjas_detectadas} detectada. Reduciendo velocidad para buscar meta.")

                            # Al detectar la ultima linea naranja (Meta), inicia avance final hacia el cajon
                            elif lineas_naranjas_detectadas >= TOTAL_LINEAS_OBJETIVO:
                                fase_actual = "AVANZANDO_AL_PARQUEO"
                                tiempo_inicio_avance = time.time()
                                print(f"[PARQUEO] Linea naranja #{lineas_naranjas_detectadas} (Meta) detectada. Avanzando 70 cm hacia el cajon.")

                    else:
                        if color_actual == "PISTA":
                            if en_linea_color:
                                if tiempo_fuera_linea == 0.0:
                                    tiempo_fuera_linea = tiempo_actual
                                elif (tiempo_actual - tiempo_fuera_linea) > 0.3:
                                    en_linea_color = False
                                    tiempo_fuera_linea = 0.0
                        else:
                            tiempo_fuera_linea = 0.0

            except Exception:
                pass
        time.sleep(0.005)


def procesar_ciclo_completo_lidar():
    global dist_derecha_min, dist_izquierda_min, fase_actual
    global initial_derecha, initial_izquierda, ser_pico, tiempo_inicio_avance

    if ser_pico is None or not ser_pico.is_open:
        return

    if dist_derecha_min > 4000: dist_derecha_min = 2000.0
    if dist_izquierda_min > 4000: dist_izquierda_min = 2000.0

    # Guardado de dimensiones iniciales del cajon de salida
    if fase_actual == "CAPTURA_INICIAL":
        initial_derecha = dist_derecha_min
        initial_izquierda = dist_izquierda_min
        fase_actual = "CARRERA"
        print(f"[INFO] Firma geometrica guardada -> Izq: {initial_izquierda:.0f}mm | Der: {initial_derecha:.0f}mm")
        print("[INFO] Inicio de carrera a velocidad crucero.")
        return

    # Navegacion a velocidad crucero
    if fase_actual == "CARRERA":
        error_lateral = dist_izquierda_min - dist_derecha_min
        angulo_objetivo = error_lateral * KP_LATERAL
        comando = f"{VELOCIDAD_CRUCERO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())

    # Velocidad reducida aproximandose a la linea final
    elif fase_actual == "BUSCANDO_PARQUEO":
        error_lateral = dist_izquierda_min - dist_derecha_min
        angulo_objetivo = error_lateral * KP_LATERAL
        comando = f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())

    # Avance centrado de 70 cm tras cruzar la ultima linea naranja
    elif fase_actual == "AVANZANDO_AL_PARQUEO":
        error_lateral = dist_izquierda_min - dist_derecha_min
        angulo_objetivo = error_lateral * KP_LATERAL
        comando = f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())

        tiempo_transcurrido = time.time() - tiempo_inicio_avance
        
        match_firma_izq = abs(dist_izquierda_min - initial_izquierda) < 80.0
        match_firma_der = abs(dist_derecha_min - initial_derecha) < 80.0
        coincidencia_geometrica = match_firma_izq and match_firma_der

        if tiempo_transcurrido >= TIEMPO_AVANCE_70CM or coincidencia_geometrica:
            fase_actual = "PARANDO"
            print(f"[PARQUEO] Estacionamiento completado en {tiempo_transcurrido:.2f}s. Deteniendo vehiculo.")
            
            for _ in range(8):
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
            
        print("[INFO] Sensor LiDAR iniciado y transmitiendo.")
        if fase_actual == "CALIBRANDO":
            fase_actual = "CAPTURA_INICIAL"

        while corriendo:
            if fase_actual == "ESPERANDO_BOTON":
                time.sleep(0.1)
                continue
                
            b0 = ser_lidar.read(1)
            if not b0: continue
            byte0 = b0[0]
            start_bit = byte0 & 0x01
            start_bit_inverse = (byte0 >> 1) & 0x01
            
            if start_bit != start_bit_inverse:
                resto = ser_lidar.read(4)
                if len(resto) < 4: continue
                byte1, byte2, byte3, byte4 = resto[0], resto[1], resto[2], resto[3]
                
                if (byte1 & 0x01) == 1:
                    raw_angle = (byte2 << 7) | (byte1 >> 1)
                    angle = raw_angle / 64.0  
                    distance = (byte4 << 8) | byte3
                    distance_mm = distance / 4.0
                    
                    if 0 < distance_mm < 6000:
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
            print(f"[ERROR] Excepcion en lectura de LiDAR: {e}")


if __name__ == '__main__':
    t_pico = threading.Thread(target=hilo_comunicacion_pico, daemon=True)
    t_pico.start()
    
    time.sleep(0.5)
    if ser_pico and ser_pico.is_open:
        ser_pico.write(b"0,0\n")
        print("[INFO] Servo de direccion alineado al centro (90 deg).")

    print("\n[READY] Sistema armado. Coloque el vehiculo en la salida y presione el boton (GPIO 21)...")
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        if ser_pico and ser_pico.is_open:
            ser_pico.write(b"0,0\n")
        time.sleep(0.05)
        
    print("\n[START] Boton detectado. Inicializando lectura de lineas naranjas...")
    fase_actual = "CALIBRANDO"
    time.sleep(0.1)
    
    t_lidar = threading.Thread(target=hilo_lidar, daemon=True)
    t_lidar.start()
    
    while corriendo:
        time.sleep(1)
