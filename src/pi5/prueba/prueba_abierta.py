import time
import threading
import serial
import sys
import signal
from gpiozero import Button

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
# CONFIGURACION DEL BOTON FISICO (GPIO 21, compatible con Raspberry Pi 5)
# ==========================================
PIN_BOTON = 21
boton_inicio = Button(PIN_BOTON, pull_up=True, bounce_time=0.05)

# ==========================================
# CONSTANTES DE NAVEGACION Y CONTEO POR IMU
# ==========================================
KP_LATERAL = 0.14  
VELOCIDAD_CRUCERO = 90
VELOCIDAD_CURVA = 90
VELOCIDAD_PARQUEO = 55

# Limites relativos al centro del servo (90 grados). Tambien mantienen cada
# orden dentro del rango admitido por el protocolo seguro de la Pico (±45).
LIMITE_DIRECCION_DER = -20.0
LIMITE_DIRECCION_IZQ = 25.0
UMBRAL_CURVA = 18.0

VUELTAS_OBJETIVO = 3
GRADOS_POR_VUELTA = 360.0
ANGULO_OBJETIVO_TOTAL = VUELTAS_OBJETIVO * GRADOS_POR_VUELTA
ANTICIPACION_PARQUEO_GRADOS = 90.0

TIEMPO_AVANCE_70CM = 0.0  # Tiempo para avanzar los 70 cm dentro del cajon
TIEMPO_MINIMO_PARA_FIRMA = 1.0  # Evita detenerse apenas comienza el avance final

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

angulo_imu_previo = None
angulo_acumulado_robot = 0.0

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
    boton_inicio.close()
    sys.exit(0)

signal.signal(signal.SIGINT, apagar_sistema)


def hilo_comunicacion_pico():
    global ser_pico, angulo_acumulado_robot, fase_actual
    global angulo_imu_previo, tiempo_inicio_avance

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
                
                if "IMU:" not in linea:
                    continue

                # Acepta tanto "IMU:123.4" como el formato anterior
                # "IMU:123.4,COLOR:PISTA"; el dato de color se ignora.
                campos = {}
                for parte in linea.split(','):
                    if ':' in parte:
                        clave, valor = parte.split(':', 1)
                        campos[clave.strip().upper()] = valor.strip()

                if "IMU" not in campos:
                    continue

                valor_crudo_imu = float(campos["IMU"])

                if fase_actual in ["ESPERANDO_BOTON", "CALIBRANDO"] or angulo_imu_previo is None:
                    angulo_imu_previo = valor_crudo_imu
                    angulo_acumulado_robot = 0.0
                    continue

                # Integra los cambios de la IMU. La normalizacion tambien permite
                # usar una IMU que entregue el angulo envuelto entre 0 y 360 grados.
                delta_angulo = valor_crudo_imu - angulo_imu_previo
                if delta_angulo > 180.0:
                    delta_angulo -= 360.0
                elif delta_angulo < -180.0:
                    delta_angulo += 360.0

                angulo_acumulado_robot += delta_angulo
                angulo_imu_previo = valor_crudo_imu
                progreso_angular = abs(angulo_acumulado_robot)

                if (
                    fase_actual == "CARRERA"
                    and progreso_angular >= ANGULO_OBJETIVO_TOTAL - ANTICIPACION_PARQUEO_GRADOS
                ):
                    fase_actual = "BUSCANDO_PARQUEO"
                    print(
                        f"[INFO] {progreso_angular:.1f} grados recorridos. "
                        "Reduciendo velocidad antes de completar las vueltas."
                    )

                elif (
                    fase_actual == "BUSCANDO_PARQUEO"
                    and progreso_angular >= ANGULO_OBJETIVO_TOTAL
                ):
                    fase_actual = "AVANZANDO_AL_PARQUEO"
                    tiempo_inicio_avance = time.time()
                    print(
                        f"[PARQUEO] {VUELTAS_OBJETIVO} vueltas completadas por IMU. "
                        "Avanzando 70 cm hacia el cajon."
                    )

            except (UnicodeDecodeError, ValueError, IndexError) as e:
                print(f"[WARN] Dato invalido recibido de la Pico: {e}")
        time.sleep(0.005)


def enviar_comando_navegacion(velocidad_base):
    """Calcula y envia una consigna que la Pico siempre pueda aceptar."""
    error_lateral = dist_izquierda_min - dist_derecha_min
    angulo_crudo = error_lateral * KP_LATERAL
    angulo_objetivo = max(
        LIMITE_DIRECCION_DER,
        min(LIMITE_DIRECCION_IZQ, angulo_crudo),
    )

    # Al acercarse al limite de direccion, baja la velocidad para tomar la
    # curva sin perder el control lateral.
    velocidad = velocidad_base
    if abs(angulo_objetivo) >= UMBRAL_CURVA:
        velocidad = min(velocidad, VELOCIDAD_CURVA)

    comando = f"{velocidad},{angulo_objetivo:.2f}\n"
    ser_pico.write(comando.encode())


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
        enviar_comando_navegacion(VELOCIDAD_CRUCERO)

    # Velocidad reducida durante los ultimos grados de la vuelta final
    elif fase_actual == "BUSCANDO_PARQUEO":
        error_lateral = dist_izquierda_min - dist_derecha_min
        angulo_objetivo = error_lateral * KP_LATERAL
        comando = f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())
        enviar_comando_navegacion(VELOCIDAD_PARQUEO)

    # Avance centrado de 70 cm al completar las vueltas medidas por la IMU
    elif fase_actual == "AVANZANDO_AL_PARQUEO":
        error_lateral = dist_izquierda_min - dist_derecha_min
        angulo_objetivo = error_lateral * KP_LATERAL
        comando = f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())
        enviar_comando_navegacion(VELOCIDAD_PARQUEO)

        tiempo_transcurrido = time.time() - tiempo_inicio_avance
        
        match_firma_izq = abs(dist_izquierda_min - initial_izquierda) < 80.0
        match_firma_der = abs(dist_derecha_min - initial_derecha) < 80.0
        coincidencia_geometrica = (
            tiempo_transcurrido >= TIEMPO_MINIMO_PARA_FIRMA
            and match_firma_izq
            and match_firma_der
        )

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
                    # El cambio 359 -> 0 marca una revolucion del LiDAR aunque
                    # la distancia de ese punto concreto sea invalida. Antes se
                    # filtraba primero la distancia y en una curva podia dejar
                    # de enviarse comandos durante mas de 500 ms.
                    if angle < angulo_previo and (angulo_previo - angle) > 300.0:
                        procesar_ciclo_completo_lidar()
                        dist_derecha_min = 8000.0
                        dist_izquierda_min = 8000.0

                    angulo_previo = angle

                    if 0 < distance_mm < 6000:
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
    
    time.sleep(1.0)
    if not ser_pico or not ser_pico.is_open:
        print("[ERROR] La Raspberry Pi Pico no esta disponible. Programa detenido.")
        boton_inicio.close()
        sys.exit(1)

    ser_pico.write(b"0,0\n")
    print("[INFO] Servo de direccion alineado al centro (90 deg).")

    print("\n[READY] Sistema armado. Coloque el vehiculo en la salida y presione el boton (GPIO 21)...")
    while not boton_inicio.is_pressed:
        if ser_pico and ser_pico.is_open:
            ser_pico.write(b"0,0\n")
        time.sleep(0.05)
        
    print("\n[START] Boton detectado. Iniciando conteo de vueltas con la IMU...")
    fase_actual = "CALIBRANDO"
    time.sleep(0.1)
    
    t_lidar = threading.Thread(target=hilo_lidar, daemon=True)
    t_lidar.start()
    
    while corriendo:
        time.sleep(1)
