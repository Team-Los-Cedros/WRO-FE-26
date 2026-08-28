#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
WRO Future Engineers 2026 - Team Los Cedros
Script de Carrera: Ronda Abierta Optimizada (v2.0)
=============================================================================
Características:
1. Navegación por LiDAR RPLIDAR C1 con amplio margen de seguridad (evita colisiones).
2. Detección de sentido de carrera mediante el Sensor de Color del piso en la Pi Pico 2:
   - Primer color reconocido AZUL   -> Sentido ANTIHORARIO (giros a la izquierda).
   - Primer color reconocido NARANJA -> Sentido HORARIO (giros a la derecha).
3. Control inercial IMU (MPU6050) con candado anti-retorno (bloquea giros de 180°).
4. Retardo de 600 ms tras presionar el botón físico (GPIO 21) para soltar el robot.
5. Diagnóstico de Visión en Background: detecta y REGISTRA como falsos positivos
   cualquier blob rojo/verde en la pista sin interrumpir ni desviar la navegación.
6. Servidor Web de Telemetría en Vivo (Puerto 8080) para feedback en tiempo real
   desde smartphone o laptop (http://<IP_PI>:8080).
=============================================================================
"""

import os
import sys
import time
import json
import math
import signal
import threading
import http.server
import socketserver
import serial
import RPi.GPIO as GPIO

# Intentar importar librerías de visión en modo seguro (no bloqueante)
try:
    import cv2
    import numpy as np
    HAY_OPENCV = True
except ImportError:
    HAY_OPENCV = False

try:
    from picamera2 import Picamera2
    HAY_PICAMERA2 = True
except ImportError:
    HAY_PICAMERA2 = False


# =============================================================================
# CONFIGURACIÓN DE HARDWARE Y COMUNICACIONES
# =============================================================================
PUERTO_LIDAR   = '/dev/ttyUSB0'
PUERTO_PICO    = '/dev/ttyACM0'
BAUDRATE_LIDAR = 460800
BAUDRATE_PICO  = 115200

START_MOTOR_CMD = b'\xa5\xf0\x02\x94\x02\xc1\x02'
START_SCAN_CMD  = b'\xa5\x20'
STOP_CMD        = b'\xa5\x25'

PIN_BOTON = 21
PUERTO_WEB = 8080

# =============================================================================
# PARÁMETROS DE NAVEGACIÓN Y CONTROL
# =============================================================================
KP_LATERAL         = 0.14    # Ganancia proporcional de centrado entre paredes
KP_HEADING         = 0.8     # Ganancia de alineación de rumbo IMU

VELOCIDAD_CRUCERO  = 80      # Velocidad estándar en tramo recto (% PWM)
VELOCIDAD_ESQUINA  = 55      # Velocidad reducida en aproximación/giro de esquina
VELOCIDAD_REVERSA  = -55     # Velocidad de retroceso de emergencia
VELOCIDAD_MINIMA   = 35      # Piso del frenado progresivo

# Márgenes de Seguridad Aumentados (mm)
DIST_ESQUINA_ANTICIPADA = 600.0  # Inicia preparación de giro al ver muro frontal a esta distancia
DIST_FRENADO_INICIO     = 850.0  # Rampa de desaceleración frontal
DIST_FRENADO_MIN        = 400.0  # Distancia de frenado máximo
DIST_EMERGENCIA_FRONTAL = 300.0  # Margen crítico frontal: retrocede antes de chocar
DIST_EMERGENCIA_LATERAL = 110.0  # Margen crítico lateral

# Límites del Servo de Dirección (grados sobre centro)
MAX_ANGULO_SERVO    = 25.0
MAX_DELTA_ANGULO    = 6.0    # Rate limiter para evitar movimientos bruscos

# Configuración de Carrera
VUELTAS_OBJETIVO    = 1      # 1 vuelta (4 esquinas) o 3 vueltas (12 esquinas)
ESQUINAS_TOTALES    = VUELTAS_OBJETIVO * 4
RETARDO_ARRANQUE_S  = 0.6    # 600 ms de espera tras presionar el botón

# Rangos HSV para Diagnóstico de Falsos Positivos de Visión
HSV_ROJO_1_MIN = (0, 70, 50);    HSV_ROJO_1_MAX = (15, 255, 255)
HSV_ROJO_2_MIN = (165, 70, 50);  HSV_ROJO_2_MAX = (180, 255, 255)
HSV_VERDE_MIN  = (35, 50, 40);   HSV_VERDE_MAX  = (85, 255, 255)
AREA_MIN_FALSO_POSITIVO = 900.0  # px


# =============================================================================
# ESTADO GLOBAL DEL SISTEMA (THREAD-SAFE)
# =============================================================================
estado_lock = threading.Lock()

corriendo = True
fase_actual = "ESPERANDO_BOTON"
t_inicio_carrera = 0.0

# Telemetría de la Pi Pico 2
angulo_imu_actual   = 0.0
angulo_imu_cero     = None
color_piso_actual   = "DESCONOCIDO"   # AZUL, NARANJA, PISTA, etc.
sentido_carrera     = "DESCONOCIDO"   # ANTIHORARIO (Azul) o HORARIO (Naranja)
esquinas_lado       = "NINGUNO"       # IZQUIERDA o DERECHA
signo_giro          = 0.0             # +1.0 (Izquierda/Antihorario), -1.0 (Derecha/Horario)

# Odometría de Esquinas
esquinas_completadas = 0
heading_base_esquina = 0.0
heading_objetivo     = 0.0

# Lecturas del LiDAR (mm)
dist_frontal      = 8000.0
dist_frente_der   = 8000.0
dist_frente_izq   = 8000.0
dist_derecha      = 8000.0
dist_izquierda    = 8000.0
dist_trasera      = 8000.0
dist_trasera_der  = 8000.0
dist_trasera_izq  = 8000.0

# Comandos de Actuadores en Vivo
cmd_velocidad_actual = 0
cmd_angulo_actual    = 0.0
ultimo_angulo_enviado = 0.0

# Diagnóstico de Falsos Positivos de Visión
contador_falsos_positivos = 0
ultimos_falsos_positivos = []  # Lista de dicts con historial

# Eventos para el Dashboard Web
registro_eventos = []

# Enlaces Seriales
ser_lidar = None
ser_pico  = None
picam2_obj = None


def registrar_evento(mensaje, nivel="INFO"):
    """Registra un evento con marca de tiempo para consola y dashboard."""
    marca = time.strftime("%H:%M:%S")
    texto = f"[{marca}] [{nivel}] {mensaje}"
    print(texto)
    with estado_lock:
        registro_eventos.append({"hora": marca, "nivel": nivel, "mensaje": mensaje})
        if len(registro_eventos) > 40:
            registro_eventos.pop(0)


# =============================================================================
# APAGADO SEGURO
# =============================================================================
def apagar_sistema(sig=None, frame=None):
    global corriendo, ser_lidar, ser_pico
    print("\n[INFO] Deteniendo vehículo de forma segura...")
    corriendo = False
    time.sleep(0.15)
    
    if ser_pico and ser_pico.is_open:
        try:
            for _ in range(6):
                ser_pico.write(b"0,0.0\n")
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

    try:
        GPIO.cleanup()
    except Exception:
        pass

    registrar_evento("Sistema detenido completamente.", "SHUTDOWN")
    sys.exit(0)

signal.signal(signal.SIGINT, apagar_sistema)


# =============================================================================
# HILO: COMUNICACIÓN SERIAL CON PI PICO 2 (IMU + SENSOR DE COLOR)
# =============================================================================
def hilo_comunicacion_pico():
    global ser_pico, angulo_imu_actual, angulo_imu_cero, color_piso_actual
    global sentido_carrera, esquinas_lado, signo_giro, fase_actual
    
    try:
        ser_pico = serial.Serial(PUERTO_PICO, baudrate=BAUDRATE_PICO, timeout=0.05)
        registrar_evento("Conexión serial establecida con Pi Pico 2 (115200 bps).", "OK")
    except Exception as e:
        registrar_evento(f"Error conectando a Pi Pico 2: {e}", "ERROR")
        return

    while corriendo:
        if ser_pico.in_waiting > 0:
            try:
                linea = ser_pico.readline().decode('utf-8', errors='ignore').strip()
                if not linea:
                    continue
                
                # Parsear tramas flexibles: "IMU:<val>,COLOR:<val>" o "IMU:<val>"
                partes = linea.split(',')
                for p in partes:
                    if p.startswith("IMU:"):
                        raw_imu = float(p.split(":")[1])
                        with estado_lock:
                            if angulo_imu_cero is None:
                                angulo_imu_cero = raw_imu
                            angulo_imu_actual = raw_imu - angulo_imu_cero
                    
                    elif p.startswith("COLOR:"):
                        color_leido = p.split(":")[1].strip().upper()
                        with estado_lock:
                            color_piso_actual = color_leido
                            
                            # Identificación de sentido por primera línea reconocida
                            if fase_actual in ["BUSCANDO_SENTIDO", "NAVEGACION_CARRIL"] and sentido_carrera == "DESCONOCIDO":
                                if "AZUL" in color_leido:
                                    sentido_carrera = "ANTIHORARIO"
                                    esquinas_lado   = "IZQUIERDA"
                                    signo_giro      = +1.0
                                    registrar_evento("¡Línea AZUL detectada por sensor de piso! -> Sentido ANTIHORARIO fijado (Giros a la IZQUIERDA).", "COLOR")
                                elif "NARANJA" in color_leido:
                                    sentido_carrera = "HORARIO"
                                    esquinas_lado   = "DERECHA"
                                    signo_giro      = -1.0
                                    registrar_evento("¡Línea NARANJA detectada por sensor de piso! -> Sentido HORARIO fijado (Giros a la DERECHA).", "COLOR")

            except Exception:
                pass
        time.sleep(0.005)


def enviar_comando_pico(vel, angulo):
    """Envía la consigna de velocidad y ángulo de dirección a la Pico."""
    global cmd_velocidad_actual, cmd_angulo_actual, ultimo_angulo_enviado
    if ser_pico and ser_pico.is_open:
        try:
            # Rate limiter de seguridad en el ángulo
            delta = max(-MAX_DELTA_ANGULO, min(MAX_DELTA_ANGULO, angulo - ultimo_angulo_enviado))
            ang_filtrado = ultimo_angulo_enviado + delta
            ang_filtrado = max(-MAX_ANGULO_SERVO, min(MAX_ANGULO_SERVO, ang_filtrado))
            ultimo_angulo_enviado = ang_filtrado
            
            with estado_lock:
                cmd_velocidad_actual = int(vel)
                cmd_angulo_actual    = round(ang_filtrado, 2)

            trama = f"{int(vel)},{ang_filtrado:.2f}\n".encode()
            ser_pico.write(trama)
        except Exception:
            pass


# =============================================================================
# HILO: LIDAR RPLIDAR C1 (BARRIDO Y PERFIL GEOMÉTRICO 360°)
# =============================================================================
def hilo_lidar():
    global ser_lidar, corriendo
    global dist_frontal, dist_frente_der, dist_frente_izq
    global dist_derecha, dist_izquierda, dist_trasera, dist_trasera_der, dist_trasera_izq
    
    try:
        ser_lidar = serial.Serial(PUERTO_LIDAR, baudrate=BAUDRATE_LIDAR, timeout=1)
        ser_lidar.write(START_MOTOR_CMD)
        time.sleep(1.2)
        ser_lidar.reset_input_buffer()
        ser_lidar.write(START_SCAN_CMD)
        time.sleep(0.5)
        if ser_lidar.in_waiting >= 7:
            ser_lidar.read(7)
            
        registrar_evento("Sensor RPLIDAR C1 iniciado y transmitiendo datos.", "OK")
        
        tf, tfd, tfi = 8000.0, 8000.0, 8000.0
        td, ti, tt   = 8000.0, 8000.0, 8000.0
        ttd, tti     = 8000.0, 8000.0
        angulo_previo = 0.0
        
        # Modo Inercial (últimos valores válidos para cuando se pierde una pared)
        u_izq, u_der = 450.0, 450.0

        while corriendo:
            b0 = ser_lidar.read(1)
            if not b0:
                continue
            byte0 = b0[0]
            if (byte0 & 0x01) == ((byte0 >> 1) & 0x01):
                continue
                
            resto = ser_lidar.read(4)
            if len(resto) < 4:
                continue
            byte1, byte2, byte3, byte4 = resto
            
            if (byte1 & 0x01) == 1:
                raw_angle = (byte2 << 7) | (byte1 >> 1)
                angle = raw_angle / 64.0
                distance_mm = ((byte4 << 8) | byte3) / 4.0
                
                if 0 < distance_mm < 6000:
                    # Revolución completada
                    if angle < angulo_previo and (angulo_previo - angle) > 300.0:
                        with estado_lock:
                            dist_frontal      = tf
                            dist_frente_der   = tfd
                            dist_frente_izq   = tfi
                            dist_derecha      = td if td < 3500 else u_der
                            dist_izquierda    = ti if ti < 3500 else u_izq
                            dist_trasera      = tt
                            dist_trasera_der  = ttd
                            dist_trasera_izq  = tti
                            
                            if td < 3500: u_der = td
                            if ti < 3500: u_izq = ti
                            
                        tf, tfd, tfi = 8000.0, 8000.0, 8000.0
                        td, ti, tt   = 8000.0, 8000.0, 8000.0
                        ttd, tti     = 8000.0, 8000.0
                    
                    angulo_previo = angle
                    
                    # Clasificación por sectores angulares (0° = Frente, Horario)
                    if angle >= 345.0 or angle <= 15.0:
                        tf = min(tf, distance_mm)
                    elif 15.0 < angle <= 60.0:
                        tfd = min(tfd, distance_mm)
                    elif 60.0 < angle <= 120.0:
                        td = min(td, distance_mm)
                    elif 120.0 < angle <= 170.0:
                        ttd = min(ttd, distance_mm)
                    elif 170.0 < angle <= 190.0:
                        tt = min(tt, distance_mm)
                    elif 190.0 < angle <= 240.0:
                        tti = min(tti, distance_mm)
                    elif 240.0 <= angle < 300.0:
                        ti = min(ti, distance_mm)
                    elif 300.0 <= angle < 345.0:
                        tfi = min(tfi, distance_mm)
                        
    except Exception as e:
        if corriendo:
            registrar_evento(f"Error en hilo LiDAR: {e}", "ERROR")


# =============================================================================
# HILO: DIAGNÓSTICO DE VISIÓN (REGISTRO Y SUPRESIÓN DE FALSOS POSITIVOS)
# =============================================================================
def hilo_vision_falsos_positivos():
    global picam2_obj, contador_falsos_positivos, ultimos_falsos_positivos
    
    if not HAY_OPENCV or not HAY_PICAMERA2:
        registrar_evento("Picamera2 u OpenCV no disponibles. Diagnóstico de visión omitido.", "WARN")
        return
        
    try:
        picam2_obj = Picamera2()
        config = picam2_obj.create_video_configuration(main={"size": (320, 240), "format": "RGB888"})
        picam2_obj.configure(config)
        picam2_obj.start()
        registrar_evento("Diagnóstico de visión Picamera2 iniciado (320x240 @ 10fps).", "OK")
    except Exception as e:
        registrar_evento(f"No se pudo iniciar Picamera2 para diagnóstico: {e}", "WARN")
        return

    kernel = np.ones((5, 5), np.uint8)
    ultimo_tiempo_fp = 0.0

    while corriendo:
        try:
            raw = picam2_obj.capture_array()
            # Rotar 180° si la cámara está montada invertida y convertir a HSV
            frame_rgb = cv2.rotate(raw[:, :, :3], cv2.ROTATE_180)
            hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
            
            # Máscaras HSV
            mask_v = cv2.inRange(hsv, np.array(HSV_VERDE_MIN), np.array(HSV_VERDE_MAX))
            mask_r = cv2.bitwise_or(
                cv2.inRange(hsv, np.array(HSV_ROJO_1_MIN), np.array(HSV_ROJO_1_MAX)),
                cv2.inRange(hsv, np.array(HSV_ROJO_2_MIN), np.array(HSV_ROJO_2_MAX))
            )
            
            mask_v = cv2.morphologyEx(mask_v, cv2.MORPH_OPEN, kernel)
            mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, kernel)
            
            cnts_r, _ = cv2.findContours(mask_r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_v, _ = cv2.findContours(mask_v, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            area_r = max([cv2.contourArea(c) for c in cnts_r], default=0)
            area_v = max([cv2.contourArea(c) for c in cnts_v], default=0)
            
            color_detectado = None
            max_area = 0
            cx, cy = 160, 120
            
            if area_r > AREA_MIN_FALSO_POSITIVO and area_r > area_v:
                color_detectado = "ROJO"
                max_area = area_r
                best = max(cnts_r, key=cv2.contourArea)
                M = cv2.moments(best)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
            elif area_v > AREA_MIN_FALSO_POSITIVO:
                color_detectado = "VERDE"
                max_area = area_v
                best = max(cnts_v, key=cv2.contourArea)
                M = cv2.moments(best)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
            # Si se detecta un pilar en esta pista limpia -> REGISTRAR FALSO POSITIVO
            ahora = time.time()
            if color_detectado and (ahora - ultimo_tiempo_fp > 1.2):
                ultimo_tiempo_fp = ahora
                with estado_lock:
                    contador_falsos_positivos += 1
                    info_fp = {
                        "id": contador_falsos_positivos,
                        "hora": time.strftime("%H:%M:%S"),
                        "color": color_detectado,
                        "area_px": int(max_area),
                        "centroide": f"({cx}, {cy})",
                        "dist_frente_lidar": f"{dist_frontal:.0f}mm",
                        "fase_fsm": fase_actual,
                        "imu": f"{angulo_imu_actual:+.1f}°"
                    }
                    ultimos_falsos_positivos.append(info_fp)
                    if len(ultimos_falsos_positivos) > 20:
                        ultimos_falsos_positivos.pop(0)
                        
                registrar_evento(
                    f"FALSO POSITIVO #{contador_falsos_positivos} detectado: {color_detectado} "
                    f"(Área: {max_area:.0f}px en {cx},{cy} | Frente: {dist_frontal:.0f}mm). "
                    f"¡SUPRIMIDO automáticamente!", "FALSO_POSITIVO"
                )

        except Exception:
            pass
            
        time.sleep(0.08)  # ~12 fps para no sobrecargar la CPU


# =============================================================================
# SERVIDOR WEB DE TELEMETRÍA EN TIEMPO REAL (PUERTO 8080)
# =============================================================================
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Telemetría WRO - Team Los Cedros</title>
    <style>
        :root {
            --bg: #0f172a;
            --card: #1e293b;
            --text: #f8fafc;
            --accent: #38bdf8;
            --warn: #f59e0b;
            --danger: #ef4444;
            --success: #22c55e;
            --border: #334155;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            background: var(--bg);
            color: var(--text);
            margin: 0;
            padding: 12px;
        }
        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 2px solid var(--border);
            padding-bottom: 8px;
            margin-bottom: 12px;
        }
        .header h1 { font-size: 1.2rem; margin: 0; color: var(--accent); }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
            gap: 10px;
            margin-bottom: 12px;
        }
        .card {
            background: var(--card);
            padding: 10px;
            border-radius: 8px;
            border: 1px solid var(--border);
        }
        .card-title { font-size: 0.75rem; color: #94a3b8; text-transform: uppercase; margin-bottom: 4px; }
        .card-val { font-size: 1.3rem; font-weight: bold; }
        .badge {
            display: inline-block;
            padding: 3px 8px;
            border-radius: 4px;
            font-size: 0.8rem;
            font-weight: bold;
        }
        .badge-info { background: #0284c7; color: white; }
        .badge-warn { background: var(--warn); color: black; }
        .badge-danger { background: var(--danger); color: white; }
        .badge-success { background: var(--success); color: black; }
        
        .lidar-box {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 6px;
            text-align: center;
            margin-top: 8px;
        }
        .sensor-cell {
            background: #0f172a;
            padding: 6px;
            border-radius: 6px;
            border: 1px solid var(--border);
        }
        .sensor-cell strong { display: block; font-size: 1rem; color: var(--accent); }
        .sensor-cell span { font-size: 0.7rem; color: #94a3b8; }
        
        .log-box {
            background: #090d16;
            border: 1px solid var(--border);
            border-radius: 8px;
            padding: 8px;
            height: 180px;
            overflow-y: auto;
            font-family: monospace;
            font-size: 0.78rem;
        }
        .log-entry { margin-bottom: 4px; border-bottom: 1px solid #1e293b; padding-bottom: 2px; }
        .btn-stop {
            width: 100%;
            background: var(--danger);
            color: white;
            border: none;
            padding: 12px;
            font-size: 1rem;
            font-weight: bold;
            border-radius: 8px;
            cursor: pointer;
            margin-top: 10px;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🚗 WRO FE 2026 Telemetría</h1>
        <span id="tiempo_carrera" class="badge badge-info">00:00.0</span>
    </div>

    <div class="grid">
        <div class="card">
            <div class="card-title">Fase FSM</div>
            <div id="fsm_estado" class="card-val" style="color: var(--accent);">ESPERANDO_BOTON</div>
        </div>
        <div class="card">
            <div class="card-title">Sentido Carrera</div>
            <div id="sentido_carrera" class="card-val">DESCONOCIDO</div>
        </div>
        <div class="card">
            <div class="card-title">Color de Piso (Pico)</div>
            <div id="color_piso" class="card-val">PISTA</div>
        </div>
        <div class="card">
            <div class="card-title">Rumbo IMU (Yaw)</div>
            <div id="imu_yaw" class="card-val">+0.0°</div>
        </div>
        <div class="card">
            <div class="card-title">Esquinas / Meta</div>
            <div id="esquinas_completadas" class="card-val">0 / 4</div>
        </div>
        <div class="card">
            <div class="card-title">Consignas (PWM, Servo)</div>
            <div id="consignas" class="card-val">0, 0.0°</div>
        </div>
        <div class="card" style="border-color: var(--danger);">
            <div class="card-title" style="color: var(--danger);">Falsos Positivos Visión</div>
            <div id="fp_contador" class="card-val" style="color: var(--danger);">0 Suprimidos</div>
        </div>
    </div>

    <div class="card" style="margin-bottom: 12px;">
        <div class="card-title">Sectores LiDAR (Distancias en mm)</div>
        <div class="lidar-box">
            <div class="sensor-cell"><span>Frente-Izq</span><strong id="d_fizq">--</strong></div>
            <div class="sensor-cell" style="border-color: var(--accent);"><span>Frente</span><strong id="d_frente" style="color: #38bdf8; font-size: 1.2rem;">--</strong></div>
            <div class="sensor-cell"><span>Frente-Der</span><strong id="d_fder">--</strong></div>
            <div class="sensor-cell"><span>Izquierda</span><strong id="d_izq">--</strong></div>
            <div class="sensor-cell"><span>Centro Robot</span><span id="d_error" style="color:#94a3b8;">err: 0mm</span></div>
            <div class="sensor-cell"><span>Derecha</span><strong id="d_der">--</strong></div>
            <div class="sensor-cell"><span>Tras-Izq</span><strong id="d_tizq">--</strong></div>
            <div class="sensor-cell"><span>Trasera</span><strong id="d_tras">--</strong></div>
            <div class="sensor-cell"><span>Tras-Der</span><strong id="d_tder">--</strong></div>
        </div>
    </div>

    <div class="card">
        <div class="card-title">Registro de Eventos y Detecciones en Vivo</div>
        <div id="log_eventos" class="log-box"></div>
    </div>

    <button class="btn-stop" onclick="fetch('/api/stop')">🛑 DETENER ROBOT (EMERGENCIA)</button>

    <script>
        function actualizar() {
            fetch('/api/telemetria')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('fsm_estado').innerText = data.fase;
                    document.getElementById('sentido_carrera').innerText = data.sentido;
                    document.getElementById('color_piso').innerText = data.color_piso;
                    document.getElementById('imu_yaw').innerText = (data.imu >= 0 ? '+' : '') + data.imu.toFixed(1) + '°';
                    document.getElementById('esquinas_completadas').innerText = data.esquinas + ' / ' + data.esquinas_totales;
                    document.getElementById('consignas').innerText = data.vel + ' PWM, ' + data.servo.toFixed(1) + '°';
                    document.getElementById('fp_contador').innerText = data.fp_count + ' Suprimidos';
                    document.getElementById('tiempo_carrera').innerText = data.tiempo;
                    
                    document.getElementById('d_frente').innerText = data.lidar.frente + 'mm';
                    document.getElementById('d_fizq').innerText = data.lidar.frente_izq + 'mm';
                    document.getElementById('d_fder').innerText = data.lidar.frente_der + 'mm';
                    document.getElementById('d_izq').innerText = data.lidar.izq + 'mm';
                    document.getElementById('d_der').innerText = data.lidar.der + 'mm';
                    document.getElementById('d_tras').innerText = data.lidar.tras + 'mm';
                    document.getElementById('d_tizq').innerText = data.lidar.tras_izq + 'mm';
                    document.getElementById('d_tder').innerText = data.lidar.tras_der + 'mm';
                    
                    let err = data.lidar.izq - data.lidar.der;
                    document.getElementById('d_error').innerText = 'err: ' + err + 'mm';

                    let logBox = document.getElementById('log_eventos');
                    logBox.innerHTML = '';
                    data.eventos.slice().reverse().forEach(ev => {
                        let colorStyle = ev.nivel === 'FALSO_POSITIVO' ? 'color:#f87171;' : (ev.nivel === 'COLOR' ? 'color:#38bdf8;' : 'color:#e2e8f0;');
                        logBox.innerHTML += `<div class="log-entry" style="${colorStyle}">[${ev.hora}] <b>${ev.nivel}</b>: ${ev.mensaje}</div>`;
                    });
                })
                .catch(() => {});
        }
        setInterval(actualizar, 200); // 5 Hz
    </script>
</body>
</html>
"""

class TelemetriaHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass # Desactivar logs estándar de HTTP para no ensuciar la consola

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(DASHBOARD_HTML.encode('utf-8'))

        elif self.path == '/api/telemetria':
            with estado_lock:
                t_transcurrido = time.time() - t_inicio_carrera if t_inicio_carrera > 0 else 0.0
                minutos = int(t_transcurrido // 60)
                segundos = t_transcurrido % 60
                tiempo_str = f"{minutos:02d}:{segundos:04.1f}"
                
                datos = {
                    "fase": fase_actual,
                    "sentido": sentido_carrera,
                    "color_piso": color_piso_actual,
                    "imu": angulo_imu_actual,
                    "esquinas": esquinas_completadas,
                    "esquinas_totales": ESQUINAS_TOTALES,
                    "vel": cmd_velocidad_actual,
                    "servo": cmd_angulo_actual,
                    "fp_count": contador_falsos_positivos,
                    "tiempo": tiempo_str,
                    "lidar": {
                        "frente": int(dist_frontal),
                        "frente_izq": int(dist_frente_izq),
                        "frente_der": int(dist_frente_der),
                        "izq": int(dist_izquierda),
                        "der": int(dist_derecha),
                        "tras": int(dist_trasera),
                        "tras_izq": int(dist_trasera_izq),
                        "tras_der": int(dist_trasera_der)
                    },
                    "eventos": registro_eventos[-15:]
                }
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(datos).encode('utf-8'))

        elif self.path == '/api/stop':
            self.send_response(200)
            self.end_headers()
            threading.Thread(target=apagar_sistema).start()
        else:
            self.send_response(404)
            self.end_headers()


def iniciar_servidor_web():
    try:
        socketserver.TCPServer.allow_reuse_address = True
        httpd = socketserver.TCPServer(("", PUERTO_WEB), TelemetriaHandler)
        registrar_evento(f"Servidor Web de Telemetría iniciado en http://0.0.0.0:{PUERTO_WEB}", "OK")
        httpd.serve_forever()
    except Exception as e:
        registrar_evento(f"No se pudo iniciar Servidor Web en puerto {PUERTO_WEB}: {e}", "WARN")


# =============================================================================
# BUCLE PRINCIPAL DE NAVEGACIÓN Y CONTROL (MÁQUINA DE ESTADOS FSM)
# =============================================================================
if __name__ == '__main__':
    registrar_evento("Iniciando arquitectura de software WRO Ronda Abierta v2.0...")
    
    # 1. Configuración de GPIO para el Botón Físico
    GPIO.setmode(GPIO.BCM)
    try:
        GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    except Exception:
        try: GPIO.cleanup()
        except: pass
        time.sleep(0.2)
        GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # 2. Iniciar Hilos de Comunicación, Sensores y Dashboard
    t_pico = threading.Thread(target=hilo_comunicacion_pico, daemon=True)
    t_pico.start()

    t_lidar = threading.Thread(target=hilo_lidar, daemon=True)
    t_lidar.start()

    t_vision = threading.Thread(target=hilo_vision_falsos_positivos, daemon=True)
    t_vision.start()

    t_web = threading.Thread(target=iniciar_servidor_web, daemon=True)
    t_web.start()

    time.sleep(0.8)
    enviar_comando_pico(0, 0.0)

    registrar_evento("SISTEMA ARMADO. Presiona el botón en GPIO 21 para iniciar la carrera.", "READY")
    print(f"\n========================================================")
    print(f"👉 DASHBOARD EN VIVO DISPONIBLE EN: http://<IP_PI>:{PUERTO_WEB}")
    print(f"========================================================\n")

    # Espera del Botón Físico
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        time.sleep(0.04)

    registrar_evento("¡BOTÓN DETECTADO! Iniciando retardo de seguridad...", "START")
    
    # Retardo de seguridad de 600 ms para retirar la mano
    time.sleep(RETARDO_ARRANQUE_S)
    
    # Reset del cero de la IMU en el momento exacto del arranque
    with estado_lock:
        angulo_imu_cero = None
        t_inicio_carrera = time.time()
        fase_actual = "BUSCANDO_SENTIDO"

    registrar_evento("¡CARRERA INICIADA! Calibración IMU establecida.", "RUN")

    # Bucle Principal de Control
    while corriendo:
        with estado_lock:
            fase = fase_actual
            df = dist_frontal
            d_izq = dist_izquierda
            d_der = dist_derecha
            yaw = angulo_imu_actual
            sentido = sentido_carrera
            esquinas = esquinas_completadas

        # ---------------------------------------------------------------------
        # SEGURIDAD GLOBAL: EMERGENCIA ANTI-CHOQUE (Se evalúa en cualquier fase)
        # ---------------------------------------------------------------------
        if df < DIST_EMERGENCIA_FRONTAL or d_izq < DIST_EMERGENCIA_LATERAL or d_der < DIST_EMERGENCIA_LATERAL:
            if fase != "EMERGENCIA_RETROCESO":
                registrar_evento(f"¡EMERGENCIA ANTI-CHOQUE! F:{df:.0f}mm, I:{d_izq:.0f}mm, D:{d_der:.0f}mm -> RETROCEDIENDO", "WARN")
                with estado_lock:
                    fase_actual = "EMERGENCIA_RETROCESO"
                fase = "EMERGENCIA_RETROCESO"

        # ---------------------------------------------------------------------
        # 1. FASE: BUSCANDO SENTIDO (Avance centrado buscando línea de color)
        # ---------------------------------------------------------------------
        if fase == "BUSCANDO_SENTIDO":
            # Si ya se identificó el sentido por el sensor de piso de la Pico
            if sentido != "DESCONOCIDO":
                with estado_lock:
                    fase_actual = "NAVEGACION_CARRIL"
            
            # Centrado proporcional suave mientras busca la línea
            error_lat = d_izq - d_der
            ang_obj = error_lat * KP_LATERAL
            enviar_comando_pico(VELOCIDAD_CRUCERO, ang_obj)

        # ---------------------------------------------------------------------
        # 2. FASE: NAVEGACIÓN EN CARRIL (Centrado y detección anticipada)
        # ---------------------------------------------------------------------
        elif fase == "NAVEGACION_CARRIL":
            # Detección Anticipada de Esquina (Muro frontal < 600 mm)
            if df < DIST_ESQUINA_ANTICIPADA:
                with estado_lock:
                    fase_actual = "ESQUINA_ANTICIPACION"
                    heading_base_esquina = yaw
                    # Rumbo esperado tras completar la esquina de 90°
                    heading_objetivo = yaw + (signo_giro * 90.0)
                registrar_evento(f"Esquina aproximándose (Frente={df:.0f}mm). Iniciando viraje hacia {esquinas_lado}...", "ESQUINA")
            
            else:
                # Centrado proporcional entre muros
                error_lat = d_izq - d_der
                
                # Failsafe si se pierde temporalmente un muro
                if d_izq > 1800 and d_der < 900:
                    error_lat = 450 - d_der
                elif d_der > 1800 and d_izq < 900:
                    error_lat = d_izq - 450
                    
                ang_obj = error_lat * KP_LATERAL
                
                # Frenado progresivo si el frente baja de 850 mm
                if df < DIST_FRENADO_INICIO:
                    factor = (df - DIST_FRENADO_MIN) / (DIST_FRENADO_INICIO - DIST_FRENADO_MIN)
                    factor = max(0.0, min(1.0, factor))
                    vel_cmd = VELOCIDAD_MINIMA + factor * (VELOCIDAD_CRUCERO - VELOCIDAD_MINIMA)
                else:
                    vel_cmd = VELOCIDAD_CRUCERO
                    
                enviar_comando_pico(vel_cmd, ang_obj)

        # ---------------------------------------------------------------------
        # 3. FASE: ESQUINA ANTICIPACIÓN Y GIRO (Guiado por IMU y Anti-Retorno)
        # ---------------------------------------------------------------------
        elif fase == "ESQUINA_ANTICIPACION":
            # Ángulo de giro hacia el lado confirmado (Izquierda = +25°, Derecha = -25°)
            ang_giro = +MAX_ANGULO_SERVO if esquinas_lado == "IZQUIERDA" else -MAX_ANGULO_SERVO
            enviar_comando_pico(VELOCIDAD_ESQUINA, ang_giro)
            
            # Verificación inercial de la esquina
            error_heading = abs(yaw - heading_objetivo)
            
            # Giro completado si la variación de rumbo alcanzó ~80° o si el frente ya ve el nuevo carril (> 1000 mm)
            if error_heading <= 12.0 or (df > 1000.0 and abs(yaw - heading_base_esquina) > 55.0):
                with estado_lock:
                    esquinas_completadas += 1
                    registrar_evento(f"¡Esquina #{esquinas_completadas} completada! (Yaw: {yaw:+.1f}°). Enderezando rumbo.", "ESQUINA")
                    
                    if esquinas_completadas >= ESQUINAS_TOTALES:
                        fase_actual = "FINALIZANDO"
                    else:
                        fase_actual = "NAVEGACION_CARRIL"

        # ---------------------------------------------------------------------
        # 4. FASE: EMERGENCIA RETROCESO (Seguridad Anti-Colisión)
        # ---------------------------------------------------------------------
        elif fase == "EMERGENCIA_RETROCESO":
            # Retroceder apuntando el morro hacia el lado de la pista libre
            ang_reversa = -MAX_ANGULO_SERVO if esquinas_lado == "IZQUIERDA" else +MAX_ANGULO_SERVO
            enviar_comando_pico(VELOCIDAD_REVERSA, ang_reversa)
            
            # Si el frente se despejó (> 550 mm), retomar navegación
            if df > 550.0:
                registrar_evento("Espacio frontal despejado tras reversa. Retomando carrera.", "INFO")
                with estado_lock:
                    fase_actual = "NAVEGACION_CARRIL"

        # ---------------------------------------------------------------------
        # 5. FASE: FINALIZANDO / PARADA
        # ---------------------------------------------------------------------
        elif fase == "FINALIZANDO":
            registrar_evento(f"¡CARRERA COMPLETADA ({VUELTAS_OBJETIVO} vueltas)! Deteniendo vehículo.", "SUCCESS")
            for _ in range(8):
                enviar_comando_pico(0, 0.0)
                time.sleep(0.02)
            apagar_sistema()

        time.sleep(0.02)  # Bucle a 50 Hz
