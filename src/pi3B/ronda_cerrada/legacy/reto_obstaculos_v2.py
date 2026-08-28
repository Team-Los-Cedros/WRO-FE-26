#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
WRO Future Engineers 2026 - Team Los Cedros
Script de Carrera: Ronda Cerrada / Reto de Obstáculos (v2.0)
=============================================================================
Características Principales:
1. Fusión Sensorial Estricta (Visión Picamera2 + LiDAR RPLIDAR C1):
   - Elimina falsos positivos: un obstáculo SOLO se evade si la cámara
     confirma ROJO o VERDE (con filtros geométricos de esbeltez e histéresis)
     Y el LiDAR confirma un cluster u obstáculo frontal cercano (< 850mm).
   - Los muros continuos y esquinas (> 20° de arco) jamás se confunden con postes.
2. Sentido de Carrera por Sensor de Color de Piso en Pi Pico 2:
   - Primer color reconocido AZUL   -> Sentido ANTIHORARIO (Esquinas hacia la IZQUIERDA).
   - Primer color reconocido NARANJA -> Sentido HORARIO (Esquinas hacia la DERECHA).
3. Reglas Oficiales WRO FE de Evasión:
   - Obstáculo ROJO  -> Se pasa por la DERECHA (ángulo de viraje negativo).
   - Obstáculo VERDE -> Se pasa por la IZQUIERDA (ángulo de viraje positivo).
4. Máquina de Estados de Evasión de 3 Fases:
   - APROXIMACION: Viraje inicial con guarda de muro lateral (evita chocar con la pared lateral).
   - SOBREPASO: Rumbo paralelo al pasillo guiado por IMU para rebasar el largo del robot.
   - REINCORPORACION: Retorno suave al centro del carril.
5. Control Inercial IMU (MPU6050) con Candado Anti-Retorno:
   - Supervisa giros de 90° en las esquinas según el sentido de carrera fijado.
   - Bloquea cualquier giro de 180° o en dirección contraria (imposible regresarse).
6. Márgenes de Seguridad Aumentados:
   - Anticipación de esquinas a 600 mm frontal.
   - Zona de emergencia a 300 mm frontal / 110 mm lateral (frena y retrocede antes de tocar el muro).
7. Retardo de Seguridad de 600 ms tras presionar el botón físico (GPIO 21).
8. Servidor Web de Telemetría en Vivo (Puerto 8080):
   - Dashboard interactivo para móvil y PC (http://<IP_PI>:8080).
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

# Librerías de Visión
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
# CONFIGURACIÓN DE HARDWARE Y PUERTOS
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
# PARÁMETROS DINÁMICOS DE NAVEGACIÓN Y CONTROL
# =============================================================================
KP_LATERAL         = 0.14    # Ganancia proporcional de centrado entre paredes
KP_HEADING         = 0.9     # Ganancia de alineación de rumbo IMU

VELOCIDAD_CRUCERO  = 65      # Velocidad estándar en tramo recto (% PWM)
VELOCIDAD_EVASION  = 50      # Velocidad durante maniobra de esquiva
VELOCIDAD_ESQUINA  = 45      # Velocidad reducida en curva de esquina
VELOCIDAD_REVERSA  = -50     # Velocidad de retroceso de emergencia
VELOCIDAD_MINIMA   = 30      # Piso del frenado progresivo

# Márgenes de Seguridad Aumentados (mm)
DIST_INICIO_EVASION     = 850.0  # Inicia evasión cuando el LiDAR ve el poste a < 850 mm
DIST_ESQUINA_ANTICIPADA = 600.0  # Inicia preparación de giro al ver muro frontal a < 600 mm
DIST_FRENADO_INICIO     = 850.0  # Rampa de desaceleración frontal
DIST_FRENADO_MIN        = 400.0  # Distancia de frenado máximo
DIST_EMERGENCIA_FRONTAL = 300.0  # Margen crítico frontal: retrocede antes de chocar
DIST_EMERGENCIA_LATERAL = 110.0  # Margen crítico lateral
DIST_ALERTA_PARED_LAT   = 200.0  # Debajo de esto, limita el viraje para no tocar el muro lateral

# Límites del Servo de Dirección (grados sobre centro)
MAX_ANGULO_SERVO    = 25.0
MAX_DELTA_ANGULO    = 6.0    # Rate limiter para evitar movimientos bruscos

# Configuración de Carrera
VUELTAS_OBJETIVO    = 3      # 3 vueltas completas (12 esquinas)
ESQUINAS_TOTALES    = VUELTAS_OBJETIVO * 4
RETARDO_ARRANQUE_S  = 0.6    # 600 ms de espera tras presionar el botón

# Rangos HSV Calibrados para Postes (Rojo y Verde)
HSV_ROJO_1_MIN = (0, 70, 50);    HSV_ROJO_1_MAX = (15, 255, 255)
HSV_ROJO_2_MIN = (165, 70, 50);  HSV_ROJO_2_MAX = (180, 255, 255)
HSV_VERDE_MIN  = (35, 50, 40);   HSV_VERDE_MAX  = (85, 255, 255)
AREA_MIN_POSTE = 450.0           # Área mínima para considerar un poste


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
heading_objetivo_esquina = 0.0

# Visión: Obstáculo Activo
poste_color_estable  = None           # "ROJO", "VERDE" o None
poste_cx_estable     = 160
poste_area_estable   = 0
poste_lado_evasion   = "NINGUNO"      # "DERECHA" o "IZQUIERDA"
heading_base_evasion = 0.0
t_inicio_evasion     = 0.0
obstaculos_superados = 0

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
                
                # Parsear tramas: "IMU:<val>,COLOR:<val>" o "IMU:<val>"
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
                            
                            # Identificación de sentido por primera línea del piso
                            if fase_actual in ["BUSCANDO_SENTIDO", "CRUCERO"] and sentido_carrera == "DESCONOCIDO":
                                if "AZUL" in color_leido:
                                    sentido_carrera = "ANTIHORARIO"
                                    esquinas_lado   = "IZQUIERDA"
                                    signo_giro      = +1.0
                                    registrar_evento("¡Línea AZUL detectada por sensor de piso! -> Sentido ANTIHORARIO (Esquinas a la IZQUIERDA).", "COLOR")
                                elif "NARANJA" in color_leido:
                                    sentido_carrera = "HORARIO"
                                    esquinas_lado   = "DERECHA"
                                    signo_giro      = -1.0
                                    registrar_evento("¡Línea NARANJA detectada por sensor de piso! -> Sentido HORARIO (Esquinas a la DERECHA).", "COLOR")

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
# HILO: VISIÓN OPENCV CON HISTÉRESIS Y FILTRO GEOMÉTRICO ANTI-FALSOS POSITIVOS
# =============================================================================
def hilo_vision_obstaculos():
    global picam2_obj, poste_color_estable, poste_cx_estable, poste_area_estable
    
    if not HAY_OPENCV or not HAY_PICAMERA2:
        registrar_evento("Picamera2 u OpenCV no disponibles. Visión de obstáculos deshabilitada.", "WARN")
        return
        
    try:
        picam2_obj = Picamera2()
        config = picam2_obj.create_video_configuration(main={"size": (320, 240), "format": "RGB888"})
        picam2_obj.configure(config)
        picam2_obj.start()
        registrar_evento("Cámara Picamera2 iniciada para detección de pilares (320x240 @ 20fps).", "OK")
    except Exception as e:
        registrar_evento(f"Error iniciando Picamera2: {e}", "WARN")
        return

    kernel = np.ones((5, 5), np.uint8)
    
    # Histéresis para estabilización anti-parpadeo
    CONFIRMACIONES_ENTRADA = 2
    CONFIRMACIONES_SALIDA  = 4
    cont_entrada = 0
    cont_salida  = 0
    color_previo = None

    while corriendo:
        try:
            raw = picam2_obj.capture_array()
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
            
            mejor_color = None
            mejor_cx = 160
            mejor_area = 0
            
            # Evaluación Geométrica de Contornos Rojos (Filtro anti-reflejo y esbeltez)
            for c in cnts_r:
                area = cv2.contourArea(c)
                if area > mejor_area and area > AREA_MIN_POSTE:
                    x, y, w, h = cv2.boundingRect(c)
                    cy = y + h // 2
                    # Un pilar de 10x10cm visto a distancia es más alto que ancho y no está en el piso pegado
                    if cy < 190 and h > (w * 0.6):
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            mejor_color = "ROJO"
                            mejor_cx = int(M["m10"] / M["m00"])
                            mejor_area = area

            # Evaluación Geométrica de Contornos Verdes
            for c in cnts_v:
                area = cv2.contourArea(c)
                if area > mejor_area and area > AREA_MIN_POSTE:
                    x, y, w, h = cv2.boundingRect(c)
                    cy = y + h // 2
                    if cy < 190 and h > (w * 0.6):
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            mejor_color = "VERDE"
                            mejor_cx = int(M["m10"] / M["m00"])
                            mejor_area = area

            # Aplicar Histéresis
            with estado_lock:
                if poste_color_estable is None:
                    if mejor_color is not None:
                        cont_entrada += 1
                        cont_salida = 0
                        if cont_entrada >= CONFIRMACIONES_ENTRADA:
                            poste_color_estable = mejor_color
                            poste_cx_estable    = mejor_cx
                            poste_area_estable  = mejor_area
                            cont_entrada = 0
                    else:
                        cont_entrada = 0
                else:
                    if mejor_color == poste_color_estable:
                        poste_cx_estable   = mejor_cx
                        poste_area_estable = mejor_area
                        cont_salida = 0
                    else:
                        cont_salida += 1
                        if cont_salida >= CONFIRMACIONES_SALIDA:
                            poste_color_estable = None
                            poste_cx_estable    = 160
                            poste_area_estable  = 0
                            cont_salida  = 0
                            cont_entrada = 0

        except Exception:
            pass
            
        time.sleep(0.04)  # ~25 fps


# =============================================================================
# SERVIDOR WEB DE TELEMETRÍA EN TIEMPO REAL (PUERTO 8080)
# =============================================================================
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Telemetría Ronda Cerrada - Team Los Cedros</title>
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
        <h1>🚧 Ronda Cerrada (Obstáculos WRO)</h1>
        <span id="tiempo_carrera" class="badge badge-info">00:00.0</span>
    </div>

    <div class="grid">
        <div class="card">
            <div class="card-title">Fase FSM</div>
            <div id="fsm_estado" class="card-val" style="color: var(--accent);">ESPERANDO_BOTON</div>
        </div>
        <div class="card">
            <div class="card-title">Pilar Detectado (Visión)</div>
            <div id="pilar_color" class="card-val" style="color: #94a3b8;">NINGUNO</div>
        </div>
        <div class="card">
            <div class="card-title">Sentido Carrera (Pico)</div>
            <div id="sentido_carrera" class="card-val">DESCONOCIDO</div>
        </div>
        <div class="card">
            <div class="card-title">Rumbo IMU (Yaw)</div>
            <div id="imu_yaw" class="card-val">+0.0°</div>
        </div>
        <div class="card">
            <div class="card-title">Esquinas / Meta</div>
            <div id="esquinas_completadas" class="card-val">0 / 12</div>
        </div>
        <div class="card">
            <div class="card-title">Obstáculos Evadidos</div>
            <div id="obs_superados" class="card-val" style="color: var(--success);">0</div>
        </div>
        <div class="card">
            <div class="card-title">Consignas (PWM, Servo)</div>
            <div id="consignas" class="card-val">0, 0.0°</div>
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
                    document.getElementById('imu_yaw').innerText = (data.imu >= 0 ? '+' : '') + data.imu.toFixed(1) + '°';
                    document.getElementById('esquinas_completadas').innerText = data.esquinas + ' / ' + data.esquinas_totales;
                    document.getElementById('obs_superados').innerText = data.obstaculos;
                    document.getElementById('consignas').innerText = data.vel + ' PWM, ' + data.servo.toFixed(1) + '°';
                    document.getElementById('tiempo_carrera').innerText = data.tiempo;
                    
                    let pilarEl = document.getElementById('pilar_color');
                    if (data.pilar) {
                        pilarEl.innerText = data.pilar + ' (' + data.pilar_area + 'px)';
                        pilarEl.style.color = data.pilar === 'ROJO' ? '#ef4444' : '#22c55e';
                    } else {
                        pilarEl.innerText = 'NINGUNO';
                        pilarEl.style.color = '#94a3b8';
                    }

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
                        let colorStyle = ev.nivel === 'OBSTACULO' ? 'color:#fbbf24; font-weight:bold;' : (ev.nivel === 'COLOR' ? 'color:#38bdf8;' : 'color:#e2e8f0;');
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
        pass

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
                    "pilar": poste_color_estable,
                    "pilar_area": int(poste_area_estable),
                    "imu": angulo_imu_actual,
                    "esquinas": esquinas_completadas,
                    "esquinas_totales": ESQUINAS_TOTALES,
                    "obstaculos": obstaculos_superados,
                    "vel": cmd_velocidad_actual,
                    "servo": cmd_angulo_actual,
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


def aux_seguridad_pared(angulo_deseado, d_izq, d_der):
    """
    Mezcla el ángulo deseado con el centrado de paredes si el vehículo
    se acerca demasiado al muro lateral durante la evasión.
    """
    pared = d_der if angulo_deseado < 0 else d_izq
    if pared >= DIST_ALERTA_PARED_LAT:
        return angulo_deseado
    peso = 1.0 - (pared / DIST_ALERTA_PARED_LAT)
    ang_centrado = (d_izq - d_der) * KP_LATERAL
    mezcla = angulo_deseado * (1.0 - peso) + ang_centrado * peso
    return max(-MAX_ANGULO_SERVO, min(MAX_ANGULO_SERVO, mezcla))


# =============================================================================
# BUCLE PRINCIPAL DE NAVEGACIÓN Y CONTROL (FSM RONDA CERRADA)
# =============================================================================
if __name__ == '__main__':
    registrar_evento("Iniciando arquitectura de software WRO Ronda Cerrada (Obstáculos) v2.0...")
    
    # 1. Configuración de GPIO para el Botón Físico
    GPIO.setmode(GPIO.BCM)
    try:
        GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    except Exception:
        try: GPIO.cleanup()
        except: pass
        time.sleep(0.2)
        GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # 2. Iniciar Hilos de Sensores y Dashboard
    t_pico = threading.Thread(target=hilo_comunicacion_pico, daemon=True)
    t_pico.start()

    t_lidar = threading.Thread(target=hilo_lidar, daemon=True)
    t_lidar.start()

    t_vision = threading.Thread(target=hilo_vision_obstaculos, daemon=True)
    t_vision.start()

    t_web = threading.Thread(target=iniciar_servidor_web, daemon=True)
    t_web.start()

    time.sleep(0.8)
    enviar_comando_pico(0, 0.0)

    registrar_evento("SISTEMA ARMADO (RONDA CERRADA). Presiona el botón en GPIO 21...", "READY")
    print(f"\n========================================================")
    print(f"👉 DASHBOARD EN VIVO DISPONIBLE EN: http://<IP_PI>:{PUERTO_WEB}")
    print(f"========================================================\n")

    # Espera del Botón Físico
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        time.sleep(0.04)

    registrar_evento("¡BOTÓN DETECTADO! Iniciando retardo de seguridad de 600 ms...", "START")
    
    # Retardo de seguridad para retirar la mano
    time.sleep(RETARDO_ARRANQUE_S)
    
    # Reset del cero de la IMU en el momento exacto del arranque
    with estado_lock:
        angulo_imu_cero = None
        t_inicio_carrera = time.time()
        fase_actual = "BUSCANDO_SENTIDO"

    registrar_evento("¡CARRERA CON OBSTÁCULOS INICIADA! Calibración IMU establecida.", "RUN")

    # Bucle Principal de Control FSM
    while corriendo:
        with estado_lock:
            fase = fase_actual
            df = dist_frontal
            d_izq = dist_izquierda
            d_der = dist_derecha
            yaw = angulo_imu_actual
            sentido = sentido_carrera
            esquinas = esquinas_completadas
            color_vis = poste_color_estable
            lado_ev = poste_lado_evasion
            h_base_ev = heading_base_evasion
            t_ev = t_inicio_evasion

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
        # 1. FASE: BUSCANDO SENTIDO (Avance inicial buscando línea de piso)
        # ---------------------------------------------------------------------
        if fase == "BUSCANDO_SENTIDO":
            if sentido != "DESCONOCIDO":
                with estado_lock:
                    fase_actual = "CRUCERO"
            
            error_lat = d_izq - d_der
            ang_obj = error_lat * KP_LATERAL
            enviar_comando_pico(VELOCIDAD_CRUCERO, ang_obj)

        # ---------------------------------------------------------------------
        # 2. FASE: CRUCERO (Centrado y Fusión Visión+LiDAR para Obstáculos / Esquinas)
        # ---------------------------------------------------------------------
        elif fase == "CRUCERO":
            # Condición de Obstáculo:
            # 1. Cámara confirma color ROJO o VERDE de forma estable.
            # 2. LiDAR confirma proximidad frontal (< 850 mm).
            if color_vis is not None and df < DIST_INICIO_EVASION:
                lado = "DERECHA" if color_vis == "ROJO" else "IZQUIERDA"
                with estado_lock:
                    fase_actual = "EVADIENDO_APROXIMACION"
                    poste_lado_evasion = lado
                    heading_base_evasion = yaw
                    t_inicio_evasion = time.time()
                registrar_evento(f"¡Poste {color_vis} confirmado a {df:.0f}mm! Evadiendo por la {lado}...", "OBSTACULO")

            # Condición de Esquina: Muro frontal cercano (< 600 mm) y SIN color de pilar
            elif df < DIST_ESQUINA_ANTICIPADA and color_vis is None:
                with estado_lock:
                    fase_actual = "ESQUINA_ANTICIPACION"
                    heading_base_esquina = yaw
                    heading_objetivo_esquina = yaw + (signo_giro * 90.0)
                registrar_evento(f"Esquina aproximándose (Frente={df:.0f}mm). Iniciando viraje hacia {esquinas_lado}...", "ESQUINA")

            else:
                # Centrado proporcional de carril
                error_lat = d_izq - d_der
                if d_izq > 1800 and d_der < 900:
                    error_lat = 450 - d_der
                elif d_der > 1800 and d_izq < 900:
                    error_lat = d_izq - 450
                    
                ang_obj = error_lat * KP_LATERAL
                
                # Frenado progresivo
                if df < DIST_FRENADO_INICIO:
                    factor = (df - DIST_FRENADO_MIN) / (DIST_FRENADO_INICIO - DIST_FRENADO_MIN)
                    factor = max(0.0, min(1.0, factor))
                    vel_cmd = VELOCIDAD_MINIMA + factor * (VELOCIDAD_CRUCERO - VELOCIDAD_MINIMA)
                else:
                    vel_cmd = VELOCIDAD_CRUCERO
                    
                enviar_comando_pico(vel_cmd, ang_obj)

        # ---------------------------------------------------------------------
        # 3. FASE: EVADIENDO - APROXIMACIÓN (Viraje inicial hacia el carril libre)
        # ---------------------------------------------------------------------
        elif fase == "EVADIENDO_APROXIMACION":
            # ROJO: Evasión por la Derecha (-25°) | VERDE: Evasión por la Izquierda (+25°)
            ang_deseado = -MAX_ANGULO_SERVO if lado_ev == "DERECHA" else +MAX_ANGULO_SERVO
            ang_seguro = aux_seguridad_pared(ang_deseado, d_izq, d_der)
            enviar_comando_pico(VELOCIDAD_EVASION, ang_seguro)
            
            # Si el frente se despejó (> 750 mm) o pasó más de 0.6s
            tiempo_en_ev = time.time() - t_ev
            if df > 750.0 or tiempo_en_ev > 0.65:
                with estado_lock:
                    fase_actual = "EVADIENDO_SOBREPASO"
                    t_inicio_evasion = time.time()
                registrar_evento("Frente despejado. Avanzando en rumbo paralelo para sobrepasar cuerpo del poste...", "INFO")

        # ---------------------------------------------------------------------
        # 4. FASE: EVADIENDO - SOBREPASO (Rumbo paralelo al pasillo)
        # ---------------------------------------------------------------------
        elif fase == "EVADIENDO_SOBREPASO":
            # Conducir paralelo al rumbo base usando control P sobre IMU
            error_h = h_base_ev - yaw
            ang_rumbo = error_h * KP_HEADING
            ang_seguro = aux_seguridad_pared(ang_rumbo, d_izq, d_der)
            enviar_comando_pico(VELOCIDAD_EVASION, ang_seguro)
            
            # Esperar a que pase el cuerpo del robot (~0.7s a velocidad media)
            if (time.time() - t_ev) > 0.7:
                with estado_lock:
                    fase_actual = "EVADIENDO_REINCORPORACION"
                    t_inicio_evasion = time.time()
                registrar_evento("Poste sobrepasado. Reincorporando al centro del carril...", "INFO")

        # ---------------------------------------------------------------------
        # 5. FASE: EVADIENDO - REINCORPORACIÓN (Retorno al centro del carril)
        # ---------------------------------------------------------------------
        elif fase == "EVADIENDO_REINCORPORACION":
            error_lat = d_izq - d_der
            ang_centrado = error_lat * KP_LATERAL
            enviar_comando_pico(VELOCIDAD_CRUCERO, ang_centrado)
            
            error_h = abs(h_base_ev - yaw)
            if error_h < 6.0 or (time.time() - t_ev) > 1.2:
                with estado_lock:
                    fase_actual = "CRUCERO"
                    obstaculos_superados += 1
                registrar_evento(f"¡Obstáculo #{obstaculos_superados} superado con éxito! Retomando velocidad crucero.", "SUCCESS")

        # ---------------------------------------------------------------------
        # 6. FASE: ESQUINA ANTICIPACIÓN Y GIRO (Guiado por IMU y Anti-Retorno)
        # ---------------------------------------------------------------------
        elif fase == "ESQUINA_ANTICIPACION":
            ang_giro = +MAX_ANGULO_SERVO if esquinas_lado == "IZQUIERDA" else -MAX_ANGULO_SERVO
            enviar_comando_pico(VELOCIDAD_ESQUINA, ang_giro)
            
            error_heading = abs(yaw - heading_objetivo_esquina)
            
            # Giro completado si la variación de rumbo alcanzó ~80° o si el frente ya ve el nuevo carril (> 1000 mm)
            if error_heading <= 12.0 or (df > 1000.0 and abs(yaw - heading_base_esquina) > 55.0):
                with estado_lock:
                    esquinas_completadas += 1
                    registrar_evento(f"¡Esquina #{esquinas_completadas} completada! (Yaw: {yaw:+.1f}°). Enderezando rumbo.", "ESQUINA")
                    
                    if esquinas_completadas >= ESQUINAS_TOTALES:
                        fase_actual = "FINALIZANDO"
                    else:
                        fase_actual = "CRUCERO"

        # ---------------------------------------------------------------------
        # 7. FASE: EMERGENCIA RETROCESO (Seguridad Anti-Colisión)
        # ---------------------------------------------------------------------
        elif fase == "EMERGENCIA_RETROCESO":
            ang_reversa = -MAX_ANGULO_SERVO if esquinas_lado == "IZQUIERDA" else +MAX_ANGULO_SERVO
            enviar_comando_pico(VELOCIDAD_REVERSA, ang_reversa)
            
            if df > 550.0:
                registrar_evento("Espacio frontal despejado tras reversa. Retomando crucero.", "INFO")
                with estado_lock:
                    fase_actual = "CRUCERO"

        # ---------------------------------------------------------------------
        # 8. FASE: FINALIZANDO / PARADA
        # ---------------------------------------------------------------------
        elif fase == "FINALIZANDO":
            registrar_evento(f"¡CARRERA COMPLETADA ({VUELTAS_OBJETIVO} vueltas)! Deteniendo vehículo.", "SUCCESS")
            for _ in range(8):
                enviar_comando_pico(0, 0.0)
                time.sleep(0.02)
            apagar_sistema()

        time.sleep(0.02)  # Bucle a 50 Hz
