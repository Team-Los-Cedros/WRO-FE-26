import time
import threading
import serial
import sys
import signal
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import cv2

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

# ==========================================
# BOTÓN DE ARRANQUE (GP21)
# ==========================================
PIN_BOTON = 21
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

# ==========================================
# CONSTANTES DE NAVEGACIÓN (paredes - igual que rondas 1 y 2)
# ==========================================
KP_LATERAL = 0.14
VELOCIDAD_CRUCERO = 55          # Un poco más lento que en rondas sin obstáculos
VELOCIDAD_PARQUEO = 20
VELOCIDAD_EVASION = 45          # Velocidad mientras se evade un poste

TIMEOUT_BUSQUEDA_PARQUEO = 6.0
tiempo_inicio_parqueo = 0.0

ANGULO_MIN_DER = 30
ANGULO_MAX_DER = 90
ANGULO_MIN_IZQ = 270
ANGULO_MAX_IZQ = 330

dist_derecha_min = 8000.0
dist_izquierda_min = 8000.0
angulo_previo = 0.0

fase_actual = "ESPERANDO_BOTON"
initial_derecha = 0.0
initial_izquierda = 0.0

angulo_inicial_imu = None
angulo_acumulado_robot = 0.0

# ==========================================
# CONSTANTES DE VISIÓN (postes rojo / verde)
# ==========================================
ANCHO_FRAME = 320
ALTO_FRAME = 240

# Umbral de "poste cerca / relevante" según área del blob en píxeles.
AREA_MIN_DETECCION = 350      # Ignora ruido / postes muy lejanos
AREA_EVASION_FUERTE = 4500    # Poste muy cerca -> viraje más agresivo

# Ganancias del control de evasión (proporcional al desplazamiento
# horizontal del centroide respecto al centro del frame + a la cercanía)
KP_EVASION_LATERAL = 0.09
GANANCIA_CERCANIA = 0.02

# Ángulo máximo de viraje que puede pedir la visión (grados sobre el objetivo)
MAX_ANGULO_EVASION = 32.0

# --- ANTI-FLICKER / HISTÉRESIS ---
# Nº de lecturas seguidas que deben confirmar (o desconfirmar) un color
# antes de que el estado "poste_color" cambie de verdad. Esto elimina el
# parpadeo detectado en las pruebas P1/P2/P4 (el volante ya no "tiembla").
CONFIRMACIONES_PARA_ENTRAR = 2
CONFIRMACIONES_PARA_SALIR = 4   # más exigente para salir -> evita soltar antes de tiempo

# --- RATE LIMITER DEL ÁNGULO ---
# Máximo cambio de ángulo permitido por ciclo de LiDAR (grados). Evita los
# "snaps" bruscos que vimos justo antes de cada choque.
MAX_DELTA_ANGULO_POR_CICLO = 6.0

# --- ZONA FRONTAL DEL LIDAR (para distancia real al obstáculo, no solo el
#     área 2D de la cámara que reacciona tarde en postes de perfil) ---
# [Fix 4]: Inicializados como variables dinámicas reconfigurables en carrera
ANGULO_MIN_FRONTAL = 350   # junto con 0-10 cubre el frente del robot
ANGULO_MAX_FRONTAL = 10
dist_frontal_min = 8000.0

# Distancia frontal (mm) bajo la cual se reduce la velocidad progresivamente
DIST_FRENADO_INICIO = 900.0
DIST_FRENADO_MIN = 300.0
VELOCIDAD_MIN_EN_FRENADO = 25

# Rangos HSV (ajustar en cancha según iluminación real)
ROJO_BAJO_1 = np.array([0, 151, 99]);   ROJO_ALTO_1 = np.array([15, 255, 255])
ROJO_BAJO_2 = np.array([158, 160, 82]); ROJO_ALTO_2 = np.array([179, 255, 255])
VERDE_BAJO = np.array([43,68, 50]);    VERDE_ALTO = np.array([85, 255, 255])

# Estado compartido de visión (lo actualiza el hilo de cámara)
lock_visión = threading.Lock()
color_crudo = None        # Lectura instantánea del frame actual (sin filtrar)
cx_crudo = None
area_cruda = 0

# Estado FILTRADO (con histéresis) que realmente usa el control
poste_color = None        # "ROJO", "VERDE" o None -> ya estabilizado
poste_cx = None
poste_area = 0
_contador_entrada = 0
_contador_salida = 0
# Variables de la nueva estrategia IMU (sin cámara)
EVADIR_POR_IZQUIERDA = True
estado_evasion = "CARRERA"  # "CARRERA", "EVADIENDO_SALIDA", "EVADIENDO_RECTO"
heading_base_evasion = 0.0
tiempo_inicio_evasion = 0.0
ultimo_angulo_aplicado = 0.0
area_anterior = 0.0
KP_IMU = 1.0
def apagar_sistema(sig, frame):
    global corriendo, ser_lidar, ser_pico
    print("\n[!] Deteniendo sistema de forma segura...")
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


def _aplicar_histeresis():
    global poste_color, poste_cx, poste_area
    global _contador_entrada, _contador_salida

    if poste_color is None:
        if color_crudo is not None:
            _contador_entrada += 1
            _contador_salida = 0
            if _contador_entrada >= CONFIRMACIONES_PARA_ENTRAR:
                poste_color = color_crudo
                poste_cx = cx_crudo
                poste_area = area_cruda
                _contador_entrada = 0
        else:
            _contador_entrada = 0
    else:
        if color_crudo == poste_color:
            poste_cx = cx_crudo
            poste_area = area_cruda
            _contador_salida = 0
        else:
            _contador_salida += 1
            if _contador_salida >= CONFIRMACIONES_PARA_SALIR:
                poste_color = None
                poste_cx = None
                poste_area = 0
                _contador_salida = 0
                _contador_entrada = 0


# ==========================================
# HILO: CÁMARA - DETECCIÓN DE POSTES ROJO/VERDE
# ==========================================
def hilo_camara():
    global poste_color, poste_cx, poste_area
    global color_crudo, cx_crudo, area_cruda

    try:
        picam2 = Picamera2()
        # [Fix 3]: Forzar ScalerCrop a full-sensor para ampliar el campo de visión lateral efectivo
        config = picam2.create_video_configuration(
            main={"size": (ANCHO_FRAME, ALTO_FRAME), "format": "RGB888"},
            controls={"ScalerCrop": (0, 0, 4608, 2592)}  # Ajuste nativo Cam Module 3
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(1.0)
        print("[+] Cámara Pi Cam Module 3 inicializada con FOV Full-Sensor [Fix 3].")
    except Exception as e:
        print(f"[-] Error inicializando cámara: {e}")
        return

    while corriendo:
        try:
            frame = picam2.capture_array()
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

            mask_rojo = cv2.inRange(hsv, ROJO_BAJO_1, ROJO_ALTO_1) | \
                        cv2.inRange(hsv, ROJO_BAJO_2, ROJO_ALTO_2)
            mask_verde = cv2.inRange(hsv, VERDE_BAJO, VERDE_ALTO)

            # Limpieza de ruido
            kernel = np.ones((5, 5), np.uint8)
            mask_rojo = cv2.morphologyEx(mask_rojo, cv2.MORPH_OPEN, kernel)
            mask_verde = cv2.morphologyEx(mask_verde, cv2.MORPH_OPEN, kernel)

            cont_rojo, _ = cv2.findContours(mask_rojo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cont_verde, _ = cv2.findContours(mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            mejor_color, mejor_cx, mejor_area = None, None, 0

            # Buscar contornos rojos válidos
            for c in cont_rojo:
                area = cv2.contourArea(c)
                if area > mejor_area and area > AREA_MIN_DETECCION:
                    x, y, w, h = cv2.boundingRect(c)
                    cy = y + h // 2
                    # Filtro anti-reflejos: Ignorar si está en el tercio inferior (piso) o si es más ancho que alto
                    if cy < 180 and h > (w * 0.7):
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            mejor_color = "ROJO"
                            mejor_cx = int(M["m10"] / M["m00"])
                            mejor_area = area

            # Buscar contornos verdes válidos
            for c in cont_verde:
                area = cv2.contourArea(c)
                if area > mejor_area and area > AREA_MIN_DETECCION:
                    x, y, w, h = cv2.boundingRect(c)
                    cy = y + h // 2
                    if cy < 180 and h > (w * 0.7):
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            mejor_color = "VERDE"
                            mejor_cx = int(M["m10"] / M["m00"])
                            mejor_area = area

            with lock_visión:
                if mejor_area > 0:
                    color_crudo = mejor_color
                    cx_crudo = mejor_cx
                    area_cruda = mejor_area
                else:
                    color_crudo = None
                    cx_crudo = None
                    area_cruda = 0
                _aplicar_histeresis()

        except Exception as e:
            print(f"[-] Falla en el hilo de cámara: {e}")
            time.sleep(0.1)

        time.sleep(0.03)  # ~30 fps aprox., suficiente para postes


# ==========================================
# HILO: COMUNICACIÓN CON LA PICO 2 (IMU / vueltas)
# ==========================================
def hilo_comunicacion_pico():
    global ser_pico, angulo_acumulado_robot, fase_actual, tiempo_inicio_parqueo, angulo_inicial_imu
    try:
        ser_pico = serial.Serial(PUERTO_PICO, baudrate=BAUDRATE_PICO, timeout=0.05)
        print("[+] Conexión serial establecida con Raspberry Pi Pico 2.")
    except Exception as e:
        print(f"[-] Error conectando a la Pi Pico 2: {e}")
        return

    while corriendo:
        if ser_pico.in_waiting > 0:
            try:
                linea = ser_pico.readline().decode('utf-8').strip()
                if linea.startswith("IMU:"):
                    valor_crudo_imu = float(linea.split(":")[1])

                    if fase_actual in ["ESPERANDO_BOTON", "CALIBRANDO"] or angulo_inicial_imu is None:
                        angulo_inicial_imu = valor_crudo_imu

                    angulo_acumulado_robot = valor_crudo_imu - angulo_inicial_imu

                    if fase_actual == "CARRERA" and abs(angulo_acumulado_robot) >= 1010.0:
                        fase_actual = "BUSCANDO_PARQUEO"
                        tiempo_inicio_parqueo = time.time()
                        print(f"[!] Última vuelta completada (Ángulo Neto: {angulo_acumulado_robot:.1f}°). Modo Parqueo Activo.")
            except:
                pass
        time.sleep(0.01)


# ==========================================
# LÓGICA DE EVASIÓN DE POSTES (usa la visión)
# ==========================================
def calcular_angulo_evasion(color, cx, area):
    error_centro = cx - (ANCHO_FRAME / 2.0)
    componente_lateral = error_centro * KP_EVASION_LATERAL
    componente_cercania = area * GANANCIA_CERCANIA / 1000.0

    magnitud = componente_lateral + componente_cercania

    if color == "ROJO":
        angulo_evasion = -abs(magnitud) - 6.0
    else:
        angulo_evasion = abs(magnitud) + 6.0

    angulo_evasion = max(-MAX_ANGULO_EVASION, min(MAX_ANGULO_EVASION, angulo_evasion))
    return angulo_evasion


# ==========================================
# PROCESAMIENTO DE UNA VUELTA COMPLETA DE LIDAR (paredes + fases)
# ==========================================
def procesar_ciclo_completo_lidar():
    global dist_derecha_min, dist_izquierda_min, fase_actual
    global initial_derecha, initial_izquierda, ser_pico, angulo_acumulado_robot, tiempo_inicio_parqueo
    global estado_evasion, ultimo_angulo_aplicado, dist_frontal_min, heading_base_evasion
    global tiempo_inicio_evasion, area_anterior, ANGULO_MIN_FRONTAL, ANGULO_MAX_FRONTAL

    if ser_pico is None or not ser_pico.is_open:
        return

    if dist_derecha_min > 4000: dist_derecha_min = 2000.0
    if dist_izquierda_min > 4000: dist_izquierda_min = 2000.0

    if fase_actual == "CAPTURA_INICIAL":
        initial_derecha = dist_derecha_min
        initial_izquierda = dist_izquierda_min
        fase_actual = "CARRERA"
        print(f"[+] Firma de Parqueo Guardada -> Izq: {initial_izquierda:.0f}mm | Der: {dist_derecha_min:.0f}mm")
        print("[➔] ¡Comienza la carrera con obstáculos!")
        return

    if fase_actual == "CARRERA":
        # 1. Usamos la cámara SOLO para clasificar el objeto y filtrar falsos positivos
        with lock_visión:
            color = poste_color
            cx = poste_cx
            area = poste_area

        # 2. Frenado progresivo según distancia FRONTAL real del LiDAR
        frontal = dist_frontal_min if dist_frontal_min < 4000 else 4000.0
        if frontal <= DIST_FRENADO_MIN:
            factor_frenado = VELOCIDAD_MIN_EN_FRENADO / VELOCIDAD_CRUCERO
        elif frontal >= DIST_FRENADO_INICIO:
            factor_frenado = 1.0
        else:
            proporcion = (frontal - DIST_FRENADO_MIN) / (DIST_FRENADO_INICIO - DIST_FRENADO_MIN)
            factor_frenado = VELOCIDAD_MIN_EN_FRENADO / VELOCIDAD_CRUCERO + \
                             proporcion * (1.0 - VELOCIDAD_MIN_EN_FRENADO / VELOCIDAD_CRUCERO)

        # 2.5. Sistema Anti-Choques de Emergencia
        # Dimensiones del robot: 11.3cm ancho (5.65cm al borde), LiDAR a 2cm del frente
        if frontal < 120 or dist_izquierda_min < 80 or dist_derecha_min < 80:
            if estado_evasion != "RETROCEDIENDO":
                estado_evasion = "RETROCEDIENDO"
                tiempo_inicio_evasion = time.time()
                print(f"[🛑] ¡COLISIÓN INMINENTE! F:{frontal:.0f} I:{dist_izquierda_min:.0f} D:{dist_derecha_min:.0f}mm. Reversa activada.")

        # 3. Máquina de Estados basada en LiDAR Frontal y Cámara (para confirmar que ES un poste)
        if estado_evasion == "CARRERA":
            # Detectar obstáculo: LiDAR dice que hay algo cerca, y la Cámara confirma que es un poste de color
            if 50 < frontal < 800 and color is not None:
                estado_evasion = "EVADIENDO_SALIDA"
                heading_base_evasion = angulo_acumulado_robot
                tiempo_inicio_evasion = time.time()
                
                # Asignación dinámica del lado según el color
                global EVADIR_POR_IZQUIERDA
                EVADIR_POR_IZQUIERDA = (color == "ROJO") # Rojo -> Evadir por izquierda
                
                print(f"[!] Poste {color} detectado al frente a {frontal:.0f}mm. Iniciando Cambio de Carril con IMU.")
        
        elif estado_evasion == "EVADIENDO_SALIDA":
            # Mantener el ángulo de salida hasta que el frente esté libre
            # Como el LiDAR está a 2cm del frente, al dejar de ver el poste significa que ya no chocaremos de frente.
            if frontal > 750 or (time.time() - tiempo_inicio_evasion) > 1.2:
                estado_evasion = "EVADIENDO_RECTO"
                tiempo_inicio_evasion = time.time()
                print(f"[!] Frente libre. Enderezando rumbo paralelo (IMU Base: {heading_base_evasion:.1f}°).")

        elif estado_evasion == "EVADIENDO_RECTO":
            # Conducimos recto (paralelo al muro) usando el IMU para que pase el cuerpo entero (22.4 cm).
            # Como el LiDAR está casi en la punta, requerimos avanzar para que pase la cola de 20cm.
            # A 50 cm/s, pasar 25 cm toma ~0.5s. Damos un poco más de margen.
            if (time.time() - tiempo_inicio_evasion) > 0.8:
                estado_evasion = "CARRERA"
                print(f"[✔️] Obstáculo superado completamente. Retornando a seguimiento de pared.")

        # 4. Cálculo del Ángulo Objetivo y Velocidad
        if estado_evasion == "RETROCEDIENDO":
            # Echar reversa con las ruedas derechas por 1.0 segundo
            angulo_objetivo_crudo = 0.0
            velocidad_base = -35  # Velocidad negativa hacia atrás
            
            # Tras 1 segundo de reversa, volver a centrarse
            if (time.time() - tiempo_inicio_evasion) > 1.0:
                estado_evasion = "CARRERA"
                print("[♻️] Reversa completada. Intentando retomar pista.")
                
        elif estado_evasion == "EVADIENDO_SALIDA":
            # Dirección del giro agresiva pero controlada
            angulo_objetivo_crudo = 30.0 if EVADIR_POR_IZQUIERDA else -30.0
            velocidad_base = VELOCIDAD_EVASION
            
            # Ampliamos el ángulo frontal dinámicamente para no perder el objeto tan rápido durante el giro
            if EVADIR_POR_IZQUIERDA: # Poste a la derecha
                ANGULO_MIN_FRONTAL = 330
                ANGULO_MAX_FRONTAL = 20
            else: # Poste a la izquierda
                ANGULO_MIN_FRONTAL = 340
                ANGULO_MAX_FRONTAL = 30

        elif estado_evasion == "EVADIENDO_RECTO":
            # Control Proporcional con IMU para volver a la orientación original
            error_heading = heading_base_evasion - angulo_acumulado_robot
            angulo_objetivo_crudo = error_heading * KP_IMU
            
            # Acotamos el esfuerzo de giro para no desestabilizar
            angulo_objetivo_crudo = max(-25.0, min(25.0, angulo_objetivo_crudo))
            velocidad_base = VELOCIDAD_EVASION
            
            ANGULO_MIN_FRONTAL = 350
            ANGULO_MAX_FRONTAL = 10

        else:
            # CARRERA NORMAL: Centrado clásico de dos paredes
            error_lateral = dist_izquierda_min - dist_derecha_min
            angulo_objetivo_crudo = error_lateral * KP_LATERAL
            velocidad_base = VELOCIDAD_CRUCERO
            ANGULO_MIN_FRONTAL = 350
            ANGULO_MAX_FRONTAL = 10

        # Frenado de emergencia general (si no estamos ya retrocediendo)
        if frontal < DIST_FRENADO_MIN and estado_evasion == "CARRERA":
            velocidad_base = VELOCIDAD_MIN_EN_FRENADO

        # 5. RATE LIMITER: limita cuánto puede cambiar el ángulo entre un ciclo y el siguiente
        delta = angulo_objetivo_crudo - ultimo_angulo_aplicado
        delta = max(-MAX_DELTA_ANGULO_POR_CICLO, min(MAX_DELTA_ANGULO_POR_CICLO, delta))
        angulo_objetivo = ultimo_angulo_aplicado + delta
        ultimo_angulo_aplicado = angulo_objetivo

        if estado_evasion == "RETROCEDIENDO":
            velocidad = int(velocidad_base) # Permitir valores negativos puros sin recorte
        else:
            velocidad = max(VELOCIDAD_MIN_EN_FRENADO, int(velocidad_base * factor_frenado))

        comando = f"{velocidad},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())

    elif fase_actual == "BUSCANDO_PARQUEO":
        error_lateral = dist_izquierda_min - dist_derecha_min
        angulo_objetivo = error_lateral * KP_LATERAL
        comando = f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())

        match_firma_original = abs(dist_derecha_min - initial_derecha) < 80.0 and abs(dist_izquierda_min - initial_izquierda) < 80.0
        tiempo_transcurrido = time.time() - tiempo_inicio_parqueo
        timeout_alcanzado = tiempo_transcurrido > TIMEOUT_BUSQUEDA_PARQUEO

        if match_firma_original or timeout_alcanzado:
            fase_actual = "DETENIDO"
            if timeout_alcanzado:
                print(f"[⏱️] DETENCIÓN POR TIMEOUT ({tiempo_transcurrido:.1f}s). El robot se detuvo en zona segura.")
            else:
                print("[🏁] ¡Firma de parqueo detectada geométricamente! Estacionando...")

            for _ in range(5):
                ser_pico.write(b"0,0\n")
                time.sleep(0.01)
            apagar_sistema(None, None)


# ==========================================
# HILO: LIDAR
# ==========================================
def hilo_lidar():
    global ser_lidar, corriendo, angulo_previo
    global dist_derecha_min, dist_izquierda_min, fase_actual
    global dist_frontal_min, ANGULO_MIN_FRONTAL, ANGULO_MAX_FRONTAL

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

        print("[+] Telemetría LiDAR activa.")
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
                            dist_frontal_min = 8000.0

                        angulo_previo = angle

                        if ANGULO_MIN_DER <= angle <= ANGULO_MAX_DER:
                            if distance_mm < dist_derecha_min:
                                dist_derecha_min = distance_mm
                        elif ANGULO_MIN_IZQ <= angle <= ANGULO_MAX_IZQ:
                            if distance_mm < dist_izquierda_min:
                                dist_izquierda_min = distance_mm

                        # Sector frontal dinámico y adaptativo [Fix 4]
                        if angle >= ANGULO_MIN_FRONTAL or angle <= ANGULO_MAX_FRONTAL:
                            if distance_mm < dist_frontal_min:
                                dist_frontal_min = distance_mm

    except Exception as e:
        if corriendo: print(f"[-] Falla en el bucle LiDAR: {e}")


if __name__ == '__main__':
    # 1. Cámara primero (necesita tiempo de calentamiento/AE)
    t_camara = threading.Thread(target=hilo_camara, daemon=True)
    t_camara.start()

    # 2. Canal de comunicación con la Pico
    t_pico = threading.Thread(target=hilo_comunicacion_pico, daemon=True)
    t_pico.start()

    time.sleep(0.5)
    if ser_pico and ser_pico.is_open:
        ser_pico.write(b"0,0\n")
        print("[🔧] Dirección alineada y bloqueada en el centro (90°).")

    print("\n[🚦] SISTEMA LISTO (RONDA CON OBSTÁCULOS). Coloca el robot en la salida y presiona el Botón (GP21)...")
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        if ser_pico and ser_pico.is_open:
            ser_pico.write(b"0,0\n")
        time.sleep(0.05)

    print("\n[🚀] ¡BOTÓN DETECTADO! Reseteando odometría IMU local...")
    fase_actual = "CALIBRANDO"
    time.sleep(0.1)

    t_lidar = threading.Thread(target=hilo_lidar, daemon=True)
    t_lidar.start()

    while corriendo:
        time.sleep(1)
