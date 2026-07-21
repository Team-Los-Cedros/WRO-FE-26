# /home/pi/Open_round.py
import time
import threading
import serial
import sys
import signal
import RPi.GPIO as GPIO  # Librería para controlar los botones físicos

# CONFIGURACIÓN DE PUERTOS Y COMUNICACIÓN

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

# CONFIGURACIÓN DEL BOTÓN DE ARRANQUE (GP21)

PIN_BOTON = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# CONSTANTES DE NAVEGACIÓN Y CONFIGURACIÓN

KP_LATERAL = 0.14
VELOCIDAD_CRUCERO = 100
VELOCIDAD_PARQUEO = 60

# LÍMITES MECÁNICOS DEL SERVO (Calibracion de Angulos GeekServo)
# Deben coincidir con CENTRO/LIMITE_DER/LIMITE_IZQ de src/pico/main.py.
SERVO_CENTRO = 90
SERVO_MAX_DER = 70
SERVO_MAX_IZQ = 115
DELTA_MAX_DER = SERVO_MAX_DER - SERVO_CENTRO
DELTA_MAX_IZQ = SERVO_MAX_IZQ - SERVO_CENTRO

# Limita la variacion maxima de angulo por ciclo para evitar giros bruscos
# (mismo mecanismo que Close2_round.py, ya validado en pista)
MAX_DELTA_ANGULO_POR_CICLO = 6.0
ultimo_angulo_aplicado = 0.0

# TIEMPO MÁXIMO DE BÚSQUEDA DE ESTACIONAMIENTO (en segundos)
TIMEOUT_BUSQUEDA_PARQUEO = 4.0  
tiempo_inicio_parqueo = 0.0

ANGULO_MIN_DER = 30
ANGULO_MAX_DER = 90
ANGULO_MIN_IZQ = 270
ANGULO_MAX_IZQ = 330

dist_derecha_min = 8000.0
dist_izquierda_min = 8000.0
angulo_previo = 0.0

# Inicialización de fases de carrera
fase_actual = "ESPERANDO_BOTON"
initial_derecha = 0.0
initial_izquierda = 0.0

# CONTROL DE DESFASE PARA LA PICO 2
angulo_inicial_imu = None  # Guardará el valor base con el que inicia la carrera
angulo_acumulado_robot = 0.0

def apagar_sistema(sig, frame):
    global corriendo, ser_lidar, ser_pico
    print("\n Deteniendo sistema")
    corriendo = False
    time.sleep(0.2)
    if ser_pico and ser_pico.is_open:
        try:
            ser_pico.write(b"0,0\n") 
            ser_pico.close()
        except: pass
    if ser_lidar and ser_lidar.is_open:
        try:
            ser_lidar.write(STOP_CMD)
            ser_lidar.close()
        except: pass
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, apagar_sistema)

def hilo_comunicacion_pico():
    global ser_pico, angulo_acumulado_robot, fase_actual, tiempo_inicio_parqueo, angulo_inicial_imu
    try:
        ser_pico = serial.Serial(PUERTO_PICO, baudrate=BAUDRATE_PICO, timeout=0.05)
        print("Conexión serial establecida con Raspberry Pi Pico 2.")
    except Exception as e:
        print(f"Error conectando a la Pi Pico 2: {e}")
        return

    while corriendo:
        if ser_pico.in_waiting > 0:
            try:
                linea = ser_pico.readline().decode('utf-8').strip()
                if linea.startswith("IMU:"):
                    valor_crudo_imu = abs(float(linea.split(":")[1]))
                    
                    # SI ESTAMOS ESPERANDO EL BOTÓN O EN CALIBRACIÓN, ACTUALIZAMOS EL "PUNTO CERO"
                    if fase_actual in ["ESPERANDO_BOTON", "CALIBRANDO"] or angulo_inicial_imu is None:
                        angulo_inicial_imu = valor_crudo_imu
                    
                    # El ángulo real de ESTA carrera es la resta del valor de la Pico menos nuestro Punto Cero
                    angulo_acumulado_robot = valor_crudo_imu - angulo_inicial_imu
                    
                    # Transición a parqueo al completar las 3 vueltas (~1010 grados acumulados NETOS)
                    if fase_actual == "CARRERA" and angulo_acumulado_robot >= 1010.0:
                        fase_actual = "BUSCANDO_PARQUEO"
                        tiempo_inicio_parqueo = time.time() 
                        print(f"Ultima vuelta completada (Ángulo Neto: {angulo_acumulado_robot:.1f}°). Modo Parqueo Activo")
            except:
                pass
        time.sleep(0.01)

def procesar_ciclo_completo_lidar():
    global dist_derecha_min, dist_izquierda_min, fase_actual
    global initial_derecha, initial_izquierda, ser_pico, angulo_acumulado_robot, tiempo_inicio_parqueo
    global ultimo_angulo_aplicado

    if ser_pico is None or not ser_pico.is_open:
        return

    if dist_derecha_min > 4000: dist_derecha_min = 2000.0
    if dist_izquierda_min > 4000: dist_izquierda_min = 2000.0

    if fase_actual == "CAPTURA_INICIAL":
        initial_derecha = dist_derecha_min
        initial_izquierda = dist_izquierda_min
        fase_actual = "CARRERA"
        print(f"Parqueo Guardado -> Izq: {initial_izquierda:.0f}mm | Der: {dist_derecha_min:.0f}mm")
        print("Corriendo")
        return

    # Lógica centralizada de cálculo de dirección
    error_lateral = dist_izquierda_min - dist_derecha_min
    angulo_objetivo_crudo = error_lateral * KP_LATERAL

    # Restringir (clamp) el ángulo comandado a los topes físicos calibrados
    angulo_objetivo_crudo = max(DELTA_MAX_DER, min(DELTA_MAX_IZQ, angulo_objetivo_crudo))

    # Limitador de tasa: evita saltos bruscos de ángulo ciclo a ciclo
    # (mismo mecanismo ya validado en pista en Close2_round.py)
    delta = angulo_objetivo_crudo - ultimo_angulo_aplicado
    delta = max(-MAX_DELTA_ANGULO_POR_CICLO, min(MAX_DELTA_ANGULO_POR_CICLO, delta))
    angulo_objetivo        = ultimo_angulo_aplicado + delta
    ultimo_angulo_aplicado = angulo_objetivo

    if fase_actual == "CARRERA":
        comando = f"{VELOCIDAD_CRUCERO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())

    elif fase_actual == "BUSCANDO_PARQUEO":
        comando = f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())
        
        # Evaluar condiciones de parada
        match_firma_original = abs(dist_derecha_min - initial_derecha) < 80.0 and abs(dist_izquierda_min - initial_izquierda) < 80.0
        tiempo_transcurrido = time.time() - tiempo_inicio_parqueo
        timeout_alcanzado = tiempo_transcurrido > TIMEOUT_BUSQUEDA_PARQUEO
        
        if match_firma_original or timeout_alcanzado:
            fase_actual = "DETENIDO"
            if timeout_alcanzado:
                print(f" DETENCIÓN POR TIMEOUT ({tiempo_transcurrido:.1f}s). El robot se detuvo en zona segura")
            else:
                print("Parqueo detectado, Estacionando")
                
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
            
        print("Telemetría LiDAR activa.")
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
        if corriendo: print(f"Falla en el bucle LiDAR: {e}")

if __name__ == '__main__':
    # 1. Encender canal de comunicación con la Pico
    t_pico = threading.Thread(target=hilo_comunicacion_pico, daemon=True)
    t_pico.start()
    
    # 2. DIRECCIÓN RECTA PREVIA
    time.sleep(0.5)
    if ser_pico and ser_pico.is_open:
        ser_pico.write(b"0,0\n")
        print(f" Dirección alineada y bloqueada en el centro ({SERVO_CENTRO}°).")

    # 3. BUCLE DE ESPERA DEL BOTÓN DE CARRERA
    print("\n SISTEMA LISTO. Coloca el robot en la salida y presiona el Botón 1 ")
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        if ser_pico and ser_pico.is_open:
            ser_pico.write(b"0,0\n")
        time.sleep(0.05)
        
    print("\n BOTÓN DETECTADO Reseteando odometría IMU local")
    fase_actual = "CALIBRANDO"
    time.sleep(0.1) # Breve pausa para asegurar la captura del cero absoluto
    
    # 4. Iniciar hilo del LiDAR justo después del botón
    t_lidar = threading.Thread(target=hilo_lidar, daemon=True)
    t_lidar.start()
    
    while corriendo:
        time.sleep(1)