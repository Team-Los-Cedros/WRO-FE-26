import time
import threading
import serial
import sys
import signal
import RPi.GPIO as GPIO
import cv2
import numpy as np

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
# CONVENCIÓN DE DIRECCIÓN (CALIBRADA):
# Positivo (+) = Giro hacia la IZQUIERDA
# Negativo (-) = Giro hacia la DERECHA
# ==========================================
KP_LATERAL = 0.14  
VELOCIDAD_CRUCERO = 85
VELOCIDAD_EVASION = 70
VELOCIDAD_REVERSA = -65

# ==========================================
# VARIABLES GLOBALES DE ESTADO
# ==========================================
corriendo = True
ser_lidar = None
ser_pico = None
picam2 = None

fase_actual = "ESPERANDO_BOTON"

# Sectores Lidar (Mínimos en mm)
dist_frente = 8000.0       # 345° a 15°
dist_frente_der = 8000.0   # 15° a 60°
dist_derecha = 8000.0      # 60° a 120°
dist_izquierda = 8000.0    # 240° a 300°
dist_frente_izq = 8000.0   # 300° a 345°

angulo_imu_actual = 0.0
angulo_imu_inicial = None
angulo_imu_objetivo = 0.0
lado_evasion = "NINGUNO"
tiempo_inicio_evasion = 0.0

def apagar_sistema(sig, frame):
    global corriendo, ser_lidar, ser_pico
    print("\n[INFO] Deteniendo robot de forma segura...")
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

# ==========================================
# VISION (Picamera2) - Calibración RGBA nativa
# ==========================================
def inicializar_camara():
    global picam2
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        print("[INFO] Picamera2 iniciada y lista.")
    except Exception as e:
        print(f"[WARN] Error al iniciar Picamera2: {e}")

def buscar_obstaculo_vision():
    """
    Captura frame nativo, rota 180°, extrae los 3 canales RGB
    y detecta pilares Rojos y Verdes con sus centroides y ángulos.
    """
    if picam2 is None:
        return "NONE", 320, 0.0, 0
    try:
        raw = picam2.capture_array()
        # Invertir 180° y tomar canales RGB
        frame_rgb = cv2.rotate(raw[:, :, :3], cv2.ROTATE_180)
        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
        
        # Máscara Verde
        mask_green = cv2.inRange(hsv, np.array([35, 40, 40]), np.array([85, 255, 255]))
        
        # Máscara Roja (extremos 0° y 180°)
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 50, 40]), np.array([15, 255, 255])),
            cv2.inRange(hsv, np.array([165, 50, 40]), np.array([180, 255, 255]))
        )
        
        cnts_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        area_red = max([cv2.contourArea(c) for c in cnts_red], default=0)
        area_green = max([cv2.contourArea(c) for c in cnts_green], default=0)
        
        color = "NONE"
        cx = 320
        max_area = 0
        
        # Umbral mínimo de detección
        if area_red > 1200 and area_red > area_green:
            color = "ROJO"
            max_area = area_red
            best = max(cnts_red, key=cv2.contourArea)
            M = cv2.moments(best)
            cx = int(M['m10'] / M['m00']) if M['m00'] > 0 else 320
        elif area_green > 1200:
            color = "VERDE"
            max_area = area_green
            best = max(cnts_green, key=cv2.contourArea)
            M = cv2.moments(best)
            cx = int(M['m10'] / M['m00']) if M['m00'] > 0 else 320
            
        ang_visual = ((cx - 320) / 320.0) * 33.0
        return color, cx, ang_visual, max_area
    except Exception as e:
        print(f"[ERROR Visión]: {e}")
        return "NONE", 320, 0.0, 0

# ==========================================
# HILO PICO (IMU y Telemetría)
# ==========================================
def hilo_comunicacion_pico():
    global ser_pico, angulo_imu_actual, angulo_imu_inicial
    try:
        ser_pico = serial.Serial(PUERTO_PICO, baudrate=BAUDRATE_PICO, timeout=0.05)
        print("[INFO] Conexión establecida con Pi Pico 2.")
    except Exception as e:
        print(f"[ERROR Pico]: {e}")
        return

    while corriendo:
        if ser_pico.in_waiting > 0:
            try:
                linea = ser_pico.readline().decode('utf-8', errors='ignore').strip()
                if "IMU:" in linea:
                    partes = linea.split(',')
                    raw_imu = float(partes[0].split(':')[1])
                    if angulo_imu_inicial is None:
                        angulo_imu_inicial = raw_imu
                    angulo_imu_actual = raw_imu - angulo_imu_inicial
            except Exception:
                pass
        time.sleep(0.005)

# ==========================================
# HILO LIDAR (Sectores Horarios)
# ==========================================
def hilo_lidar():
    global ser_lidar, corriendo
    global dist_frente, dist_frente_der, dist_frente_izq, dist_derecha, dist_izquierda
    
    try:
        ser_lidar = serial.Serial(PUERTO_LIDAR, baudrate=BAUDRATE_LIDAR, timeout=1)
        ser_lidar.write(START_MOTOR_CMD)
        time.sleep(1.5)
        ser_lidar.reset_input_buffer()
        ser_lidar.write(START_SCAN_CMD)
        time.sleep(0.5)
        if ser_lidar.in_waiting >= 7:
            ser_lidar.read(7)
            
        print("[INFO] LiDAR iniciado y transmitiendo datos.")

        tf, tfd, td, ti, tfi = 8000.0, 8000.0, 8000.0, 8000.0, 8000.0
        angulo_previo = 0.0
        
        while corriendo:
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
                        # Revolución completada
                        if angle < angulo_previo and (angulo_previo - angle) > 300.0:
                            dist_frente, dist_frente_der, dist_derecha, dist_izquierda, dist_frente_izq = tf, tfd, td, ti, tfi
                            tf, tfd, td, ti, tfi = 8000.0, 8000.0, 8000.0, 8000.0, 8000.0
                        
                        angulo_previo = angle
                        
                        # Sectores angulares (0° = Frente, Horario)
                        if angle >= 345.0 or angle <= 15.0:
                            tf = min(tf, distance_mm)
                        elif 15.0 < angle <= 60.0:
                            tfd = min(tfd, distance_mm)
                        elif 60.0 < angle <= 120.0:
                            td = min(td, distance_mm)
                        elif 240.0 <= angle < 300.0:
                            ti = min(ti, distance_mm)
                        elif 300.0 <= angle < 345.0:
                            tfi = min(tfi, distance_mm)
                                
    except Exception as e:
        if corriendo:
            print(f"[ERROR LiDAR]: {e}")

# ==========================================
# BUCLE PRINCIPAL (MÁQUINA DE ESTADOS FSM)
# ==========================================
if __name__ == '__main__':
    t_pico = threading.Thread(target=hilo_comunicacion_pico, daemon=True)
    t_pico.start()
    
    t_lidar = threading.Thread(target=hilo_lidar, daemon=True)
    t_lidar.start()
    
    inicializar_camara()
    
    time.sleep(1)
    if ser_pico and ser_pico.is_open:
        ser_pico.write(b"0,0\n")

    print("\n[READY] Robot listo. Presiona el botón en GPIO 21 para iniciar...")
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        time.sleep(0.05)
        
    print("\n[START] ¡Carrera de Obstáculos Iniciada!")
    fase_actual = "NAVEGACION_CARRIL"
    
    while corriendo:
        # ----------------------------------------------------
        # 1. NAVEGACIÓN EN CARRIL (BÚSQUEDA ACTIVA DE OBSTÁCULOS)
        # ----------------------------------------------------
        if fase_actual == "NAVEGACION_CARRIL":
            # Verificar si hay pilar visible por cámara
            color, cx, ang_v, area = buscar_obstaculo_vision()
            
            # Condición de Obstáculo:
            # - La cámara ve un pilar claramente (Área > 1500 px) Y el LiDAR ve algo a < 1100mm
            # - O el LiDAR frontal detecta obstáculo cercano (< 600mm)
            if color != "NONE" and (dist_frente < 1100 or dist_frente_der < 700 or dist_frente_izq < 700):
                print(f"[FSM] Obstáculo detectado: {color} (Área: {area:.0f}px, Ángulo: {ang_v:+.1f}°) | Frente: {dist_frente:.0f}mm")
                
                # Regla WRO FE:
                # - ROJO: Pasar por la DERECHA (girar a la DERECHA -> ángulo NEGATIVO)
                # - VERDE: Pasar por la IZQUIERDA (girar a la IZQUIERDA -> ángulo POSITIVO)
                if color == "ROJO":
                    lado_evasion = "DERECHA"
                else:
                    lado_evasion = "IZQUIERDA"
                    
                fase_actual = "ESQUIVANDO"
                tiempo_inicio_evasion = time.time()
                
            # Condición de Esquina: Muro frontal muy cercano (< 300mm) y SIN pilar de color
            elif dist_frente < 350 and color == "NONE":
                print(f"[FSM] Muro frontal detectado a {dist_frente:.0f}mm. Iniciando aproximación a esquina...")
                fase_actual = "ESQUINA_APROXIMACION"
                
            else:
                # Centrado proporcional dentro del carril
                error_lateral = dist_izquierda - dist_derecha
                
                # Failsafe si se pierde una pared
                if dist_izquierda > 1800 and dist_derecha < 900:
                    error_lateral = 400 - dist_derecha
                elif dist_derecha > 1800 and dist_izquierda < 900:
                    error_lateral = dist_izquierda - 400
                    
                angulo_objetivo = error_lateral * KP_LATERAL
                angulo_objetivo = max(-30, min(30, angulo_objetivo))
                
                if ser_pico:
                    ser_pico.write(f"{VELOCIDAD_CRUCERO},{angulo_objetivo:.2f}\n".encode())
                    
        # ----------------------------------------------------
        # 2. ESQUIVANDO OBSTÁCULO (MANIOBRA FLUIDA)
        # ----------------------------------------------------
        elif fase_actual == "ESQUIVANDO":
            # DERECHA = Ángulo negativo (-25°) | IZQUIERDA = Ángulo positivo (+25°)
            if lado_evasion == "DERECHA":
                angulo_objetivo = -26.0
            else:
                angulo_objetivo = +26.0
                
            if ser_pico:
                ser_pico.write(f"{VELOCIDAD_EVASION},{angulo_objetivo:.2f}\n".encode())
                
            # Tras 0.8s o cuando el obstáculo ya no esté en el cono frontal
            tiempo_evadiendo = time.time() - tiempo_inicio_evasion
            if tiempo_evadiendo > 0.7:
                # Verificar si el frente ya está despejado
                if dist_frente > 750 or tiempo_evadiendo > 1.4:
                    print("[FSM] Maniobra de evasión superada. Retomando centrado de carril.")
                    fase_actual = "NAVEGACION_CARRIL"
                    
        # ----------------------------------------------------
        # 3. APROXIMACIÓN FINAL A ESQUINA
        # ----------------------------------------------------
        elif fase_actual == "ESQUINA_APROXIMACION":
            # Avanzar recto despacio hasta 22 cm del muro
            if ser_pico:
                ser_pico.write(f"{VELOCIDAD_EVASION},0.0\n".encode())
                
            if dist_frente <= 220:
                print(f"[FSM] Distancia mínima al muro alcanzada ({dist_frente:.0f}mm).")
                if ser_pico:
                    ser_pico.write(b"0,0\n")
                time.sleep(0.15)
                
                # Determinar sentido del giro por el lado con mayor espacio libre
                if dist_izquierda > dist_derecha:
                    # La pista continúa a la IZQUIERDA.
                    # Para apuntar el morro a la IZQUIERDA en reversa:
                    # Ruedas a la DERECHA (ángulo NEGATIVO: -35°) y marcha atrás.
                    print("[FSM] Esquina hacia la IZQUIERDA. Ejecutando reversa con ruedas a la Derecha (-35°)...")
                    angulo_ruedas_reversa = -35.0
                    angulo_imu_objetivo = angulo_imu_actual - 80.0
                else:
                    # La pista continúa a la DERECHA.
                    # Para apuntar el morro a la DERECHA en reversa:
                    # Ruedas a la IZQUIERDA (ángulo POSITIVO: +35°) y marcha atrás.
                    print("[FSM] Esquina hacia la DERECHA. Ejecutando reversa con ruedas a la Izquierda (+35°)...")
                    angulo_ruedas_reversa = +35.0
                    angulo_imu_objetivo = angulo_imu_actual + 80.0
                    
                fase_actual = "ESQUINA_MANIOBRA"
                
        # ----------------------------------------------------
        # 4. MANIOBRA DE REVERSA (3 PUNTOS) GUIADA POR IMU
        # ----------------------------------------------------
        elif fase_actual == "ESQUINA_MANIOBRA":
            if ser_pico:
                ser_pico.write(f"{VELOCIDAD_REVERSA},{angulo_ruedas_reversa:.2f}\n".encode())
                
            # Verificar si el giro de ~80° se completó o si el frente ya ve el nuevo carril (> 1200mm)
            dif_angular = abs(angulo_imu_actual - angulo_imu_objetivo)
            if dif_angular <= 8.0 or dist_frente > 1200:
                print(f"[FSM] Giro completado (dif={dif_angular:.1f}°, frente={dist_frente:.0f}mm). Enderezando y reanudando.")
                if ser_pico:
                    ser_pico.write(b"0,0\n")
                time.sleep(0.15)
                fase_actual = "NAVEGACION_CARRIL"
                
        time.sleep(0.02)
