import time
import threading
import serial
import sys
import signal
import RPi.GPIO as GPIO

import vision
import lidar
import tracker

# ==========================================
# CONFIGURACIÓN DE PUERTOS Y COMUNICACIÓN (Pico 2)
# ==========================================
PUERTO_PICO   = '/dev/ttyACM0'
BAUDRATE_PICO = 115200

corriendo = True
ser_pico  = None

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
# CONSTANTES DE NAVEGACIÓN
# ==========================================
KP_LATERAL           = 0.14
VELOCIDAD_CRUCERO    = 55
VELOCIDAD_PARQUEO    = 20
VELOCIDAD_EVASION    = 40       # Conservador durante evasión

TIMEOUT_BUSQUEDA_PARQUEO = 6.0
tiempo_inicio_parqueo    = 0.0

fase_actual       = "ESPERANDO_BOTON"
initial_derecha   = 0.0
initial_izquierda = 0.0

angulo_inicial_imu     = None
angulo_acumulado_robot = 0.0

sentido_giro = "DESCONOCIDO"

# ==========================================
# CONSTANTES DE EVASIÓN (postes rojo / verde)
# ==========================================
KP_EVASION_LATERAL = 0.09
GANANCIA_CERCANIA  = 0.02
MAX_ANGULO_EVASION = 32.0

# Ángulos base (sesgo direccional) de los estados DETECTADO/ESQUIVANDO,
# corregidos proporcionalmente por KP_EVASION_LATERAL * tracker["x"]
ANGULO_BASE_DETECTADO   = 15.0
ANGULO_BASE_ESQUIVANDO  = 28.0

# Tiempo máximo en DETECTADO antes de forzar ESQUIVANDO como red de
# seguridad, solo si el LiDAR nunca confirma el cluster ni cruza los
# umbrales de distancia (600mm/400mm). En el escenario normal, la
# confirmación por distancia real ocurre bastante antes de este límite.
TIMEOUT_DETECTADO = 1.2

# Tiempo máximo para que RECENTRANDO corrija el rumbo antes de rendirse.
# En pista se observaron errores de hasta ~68 grados que no alcanzaban a
# corregirse en 1.5s, dejando al robot desalineado y disparando la
# emergencia anti-colisión justo al volver a CARRERA.
TIMEOUT_RECENTRANDO = 3.0

MAX_DELTA_ANGULO_POR_CICLO = 6.0

DIST_FRENADO_INICIO      = 900.0
DIST_FRENADO_MIN         = 300.0
VELOCIDAD_MIN_EN_FRENADO = 25

# ==========================================
# ESTADO DE EVASIÓN
# FSM: CARRERA → DETECTADO → ESQUIVANDO → PASANDO → RECENTRANDO → CARRERA
#      Cualquier estado → RETROCEDIENDO → FORZANDO_GIRO → CARRERA  (emergencia)
# ==========================================
estado_evasion         = "CARRERA"
EVADIR_POR_IZQUIERDA   = True
heading_base_evasion   = 0.0
tiempo_inicio_evasion  = 0.0
ultimo_angulo_aplicado = 0.0
KP_IMU = 1.0


# ==========================================
# APAGADO SEGURO
# ==========================================
_apagando_en_curso = False

def apagar_sistema(sig, frame):
    global corriendo, ser_pico, _apagando_en_curso

    # Evita doble ejecución (ej. doble Ctrl+C) reentrando aquí mientras
    # el primer apagado todavía está en curso, lo cual dejaba a GPIO.cleanup()
    # operando sobre un handle ya cerrado y tumbaba el script con una excepción.
    if _apagando_en_curso:
        return
    _apagando_en_curso = True

    print("\n[!] Deteniendo sistema de forma segura...")
    corriendo = False
    time.sleep(0.2)
    if ser_pico and ser_pico.is_open:
        try:
            ser_pico.write(b"0,0\n")
            ser_pico.close()
        except: pass
    if lidar.ser_lidar and lidar.ser_lidar.is_open:
        try:
            lidar.ser_lidar.write(lidar.STOP_CMD)
            lidar.ser_lidar.close()
        except: pass
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f"[-] GPIO.cleanup() fallo (ignorado): {e}")
    sys.exit(0)

signal.signal(signal.SIGINT, apagar_sistema)


# ==========================================
# HILO: COMUNICACIÓN CON LA PICO 2
# ==========================================
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
                    valor_crudo_imu = float(linea.split(":")[1])

                    if fase_actual in ["ESPERANDO_BOTON", "CALIBRANDO"] or angulo_inicial_imu is None:
                        angulo_inicial_imu = valor_crudo_imu

                    angulo_acumulado_robot = valor_crudo_imu - angulo_inicial_imu

                    if fase_actual == "CARRERA" and abs(angulo_acumulado_robot) >= 1010.0:
                        fase_actual = "BUSCANDO_PARQUEO"
                        tiempo_inicio_parqueo = time.time()
                        print(f"[!] Ultima vuelta completada ({angulo_acumulado_robot:.1f} deg). Modo Parqueo.")
            except:
                pass
        time.sleep(0.01)


# ==========================================
# CALLBACKS DEL HILO LIDAR (ver lidar.hilo_lidar)
# ==========================================
def _al_listo_lidar():
    """Se ejecuta una vez cuando el LiDAR ya está escaneando (tras el botón)."""
    global fase_actual
    if fase_actual == "CALIBRANDO":
        fase_actual = "CAPTURA_INICIAL"


def _al_completar_scan(clusters_obstaculos):
    """
    Se ejecuta una vez por barrido completo del LiDAR (ver lidar.hilo_lidar).
    Actualiza el tracker (IMU + asociación de clusters reales) y luego
    ejecuta la máquina de estados de navegación/evasión.
    """
    tracker.actualizar_tracker_imu(angulo_acumulado_robot)

    if tracker.tracker["activo"]:
        # Intentar asociar cluster real al tracker existente (corrección de deriva)
        tracker.asociar_cluster_al_tracker(clusters_obstaculos)
    else:
        # Si no hay tracker activo, crear uno si cámara confirma color Y hay cluster frontal
        color_cam = vision.get_color()

        if color_cam is not None and clusters_obstaculos:
            cluster_frente = None
            dist_minima    = lidar.DIST_MAX_OBSTACULO

            for clust in clusters_obstaculos:
                cx_c, cy_c = lidar.centroide_xy_cluster(clust)
                # El cluster debe estar en zona frontal (y > 0, no muy lateral)
                if cy_c > 80.0 and abs(cx_c) < 450.0:
                    d = (cx_c**2 + cy_c**2) ** 0.5
                    if d < dist_minima:
                        dist_minima    = d
                        cluster_frente = clust

            if cluster_frente is not None:
                cx_f, cy_f = lidar.centroide_xy_cluster(cluster_frente)
                tracker.iniciar_tracker(color_cam, cx_f, cy_f, angulo_acumulado_robot)

    procesar_ciclo_completo_lidar()


# ==========================================
# CONTROL PRINCIPAL — Máquina de estados de navegación y evasión
# ==========================================
def procesar_ciclo_completo_lidar():
    """
    Se ejecuta una vez por barrido completo del LiDAR, después de que
    _al_completar_scan() ya actualizó el tracker. Decide la consigna de
    velocidad/ángulo según fase_actual y estado_evasion, y la envía a
    la Pico 2 por UART.
    """
    global fase_actual
    global initial_derecha, initial_izquierda, ser_pico
    global tiempo_inicio_parqueo
    global estado_evasion, ultimo_angulo_aplicado, heading_base_evasion
    global tiempo_inicio_evasion
    global sentido_giro, EVADIR_POR_IZQUIERDA

    if ser_pico is None or not ser_pico.is_open:
        return

    # ─────────────────────────────────────────────────────────────────────────
    # FASE: CAPTURA_INICIAL
    # ─────────────────────────────────────────────────────────────────────────
    if fase_actual == "CAPTURA_INICIAL":
        initial_derecha   = lidar.dist_derecha_min
        initial_izquierda = lidar.dist_izquierda_min
        fase_actual = "CARRERA"
        print(f"[+] Firma de Parqueo: Izq={initial_izquierda:.0f}mm | Der={initial_derecha:.0f}mm")
        print("[INICIO] Carrera con obstaculos iniciada!")
        return

    # ─────────────────────────────────────────────────────────────────────────
    # FASE: CARRERA
    # ─────────────────────────────────────────────────────────────────────────
    if fase_actual == "CARRERA":
        frontal = lidar.dist_frontal_min if lidar.dist_frontal_min < 4000 else 4000.0

        # 0. Detectar sentido de giro de la pista
        if sentido_giro == "DESCONOCIDO":
            if angulo_acumulado_robot < -45.0:
                sentido_giro = "HORARIO"
                print("[GIRO] Sentido: HORARIO (derecha)")
            elif angulo_acumulado_robot > 45.0:
                sentido_giro = "ANTIHORARIO"
                print("[GIRO] Sentido: ANTIHORARIO (izquierda)")

        # 1. Leer estado de visión
        color = vision.get_color()

        # 2. Frenado progresivo por distancia frontal LiDAR
        if frontal <= DIST_FRENADO_MIN:
            factor_frenado = VELOCIDAD_MIN_EN_FRENADO / VELOCIDAD_CRUCERO
        elif frontal >= DIST_FRENADO_INICIO:
            factor_frenado = 1.0
        else:
            proporcion     = (frontal - DIST_FRENADO_MIN) / (DIST_FRENADO_INICIO - DIST_FRENADO_MIN)
            factor_frenado = (VELOCIDAD_MIN_EN_FRENADO / VELOCIDAD_CRUCERO +
                              proporcion * (1.0 - VELOCIDAD_MIN_EN_FRENADO / VELOCIDAD_CRUCERO))

        # 3. Sistema Anti-Choque de Emergencia (mantenido del código original)
        if frontal < 120 or lidar.dist_izquierda_min < 80 or lidar.dist_derecha_min < 80:
            if estado_evasion not in ["RETROCEDIENDO", "FORZANDO_GIRO"]:
                estado_evasion        = "RETROCEDIENDO"
                tiempo_inicio_evasion = time.time()
                print(f"[EMERGENCIA] COLISION INMINENTE! F:{frontal:.0f} I:{lidar.dist_izquierda_min:.0f} D:{lidar.dist_derecha_min:.0f}mm")

        # ─────────────────────────────────────────────────────────────────────
        # 4. MÁQUINA DE ESTADOS DE EVASIÓN — 6 estados con transiciones geométricas
        # ─────────────────────────────────────────────────────────────────────

        # ── Estado CARRERA (seguimiento de pared normal) ──────────────────────
        if estado_evasion == "CARRERA":
            trk_activo = tracker.tracker["activo"]
            # El tracker confirma obstáculo frontal: y > 50mm (adelante) y < 800mm (en rango)
            trk_al_frente = trk_activo and 50.0 < tracker.tracker["y"] < 800.0

            if trk_al_frente or (50 < frontal < 700 and color is not None):
                estado_evasion        = "DETECTADO"
                heading_base_evasion  = angulo_acumulado_robot
                tiempo_inicio_evasion = time.time()

                # Determinar lado de evasión (tracker tiene prioridad sobre cámara)
                # Regla WRO: ROJO -> evadir por la DERECHA, VERDE -> evadir por la IZQUIERDA
                color_det = tracker.tracker["color"] if trk_activo and tracker.tracker["color"] else color
                EVADIR_POR_IZQUIERDA = (color_det == "VERDE")

                # Ampliar sector frontal dinámicamente para no perder el cluster al girar
                if EVADIR_POR_IZQUIERDA:
                    lidar.ANGULO_MIN_FRONTAL = 330
                    lidar.ANGULO_MAX_FRONTAL = 20
                else:
                    lidar.ANGULO_MIN_FRONTAL = 340
                    lidar.ANGULO_MAX_FRONTAL = 30

                lado_str = "IZQUIERDA" if EVADIR_POR_IZQUIERDA else "DERECHA"
                print(f"[FSM] CARRERA -> DETECTADO | {color_det} | Evadir x {lado_str} | F:{frontal:.0f}mm")

        # ── Estado DETECTADO (confirmando obstáculo, frenando suavemente) ──────
        elif estado_evasion == "DETECTADO":
            trk_confirmado  = tracker.tracker["activo"] and tracker.tracker["confirmaciones"] >= 2
            tiempo_detectado = time.time() - tiempo_inicio_evasion

            # Transición a ESQUIVANDO si el tracker confirma Y el obstáculo está a <600mm,
            # o si el obstáculo está muy cerca (por seguridad), o tras TIMEOUT_DETECTADO
            # (red de seguridad si el LiDAR nunca llega a confirmar el cluster -- antes
            # era 0.3s, tan corto que casi siempre ganaba él antes que la confirmación
            # real de distancia, haciendo que el robot evadiera "a ciegas" sin esperar
            # al LiDAR).
            if (trk_confirmado and frontal < 600) or frontal < 400 or tiempo_detectado > TIMEOUT_DETECTADO:
                estado_evasion        = "ESQUIVANDO"
                tiempo_inicio_evasion = time.time()
                lado_str = "IZQUIERDA" if EVADIR_POR_IZQUIERDA else "DERECHA"
                print(f"[FSM] DETECTADO -> ESQUIVANDO {lado_str} | F:{frontal:.0f}mm")

        # ── Estado ESQUIVANDO (giro lateral activo) ───────────────────────────
        elif estado_evasion == "ESQUIVANDO":
            tiempo_esquivando = time.time() - tiempo_inicio_evasion

            # El frente LiDAR quedó libre (obstáculo ya no bloquea el camino)
            frente_libre = frontal > 750.0

            # El tracker indica que el obstáculo pasó al costado
            lado_costado = "DER" if not EVADIR_POR_IZQUIERDA else "IZQ"
            obst_lateral = tracker.obstaculo_al_costado(lado_costado)

            # El tracker indica que el obstáculo ya quedó detrás
            obst_superado = tracker.obstaculo_fue_superado()

            # Tiempo mínimo de giro para que el servo actúe (0.2s)
            if tiempo_esquivando > 0.2 and (frente_libre or obst_lateral or obst_superado):
                estado_evasion        = "PASANDO"
                tiempo_inicio_evasion = time.time()
                lidar.ANGULO_MIN_FRONTAL = 350
                lidar.ANGULO_MAX_FRONTAL = 10
                print(f"[FSM] ESQUIVANDO -> PASANDO | Frente libre:{frente_libre} Lateral:{obst_lateral}")

        # ── Estado PASANDO (robot pasa junto al obstáculo, rumbo paralelo) ─────
        elif estado_evasion == "PASANDO":
            # Criterio GEOMÉTRICO principal: tracker confirma que el obstáculo
            # está ya detrás del robot (y < -DISTANCIA_SUPERADO).
            # Criterio de seguridad (timeout): 1.2s por si el tracker falla.
            superado  = tracker.obstaculo_fue_superado()
            tiempo_p  = time.time() - tiempo_inicio_evasion
            timeout_p = tiempo_p > 1.2

            if superado or timeout_p:
                estado_evasion        = "RECENTRANDO"
                tiempo_inicio_evasion = time.time()
                razon = "geometrico" if superado else "timeout"
                tracker.desactivar_tracker(f"Superado ({razon})")
                print(f"[FSM] PASANDO -> RECENTRANDO | {razon}")

        # ── Estado RECENTRANDO (volver al heading original) ──────────────────
        elif estado_evasion == "RECENTRANDO":
            error_heading      = abs(heading_base_evasion - angulo_acumulado_robot)
            tiempo_recentrando = time.time() - tiempo_inicio_evasion

            if error_heading < 4.0 or tiempo_recentrando > TIMEOUT_RECENTRANDO:
                estado_evasion = "CARRERA"
                lidar.ANGULO_MIN_FRONTAL = 350
                lidar.ANGULO_MAX_FRONTAL = 10
                print(f"[FSM] RECENTRANDO -> CARRERA | Error heading={error_heading:.1f} deg")

        # ── Estado RETROCEDIENDO (anti-choque, idéntico al código original) ───
        elif estado_evasion == "RETROCEDIENDO":
            if sentido_giro == "HORARIO":
                angulo_objetivo_crudo = 30.0
            elif sentido_giro == "ANTIHORARIO":
                angulo_objetivo_crudo = -30.0
            else:
                angulo_objetivo_crudo = 0.0

            velocidad_base    = -35
            tiempo_retroceso  = time.time() - tiempo_inicio_evasion
            choque_trasero    = lidar.dist_trasera_min < 250.0

            if choque_trasero or tiempo_retroceso > 3.5:
                estado_evasion        = "FORZANDO_GIRO"
                tiempo_inicio_evasion = time.time()
                razon = "obstaculo trasero" if choque_trasero else "tiempo maximo"
                print(f"[FSM] RETROCEDIENDO -> FORZANDO_GIRO ({razon})")

        # ── Estado FORZANDO_GIRO (anti-choque, idéntico al código original) ───
        elif estado_evasion == "FORZANDO_GIRO":
            if sentido_giro == "HORARIO":
                angulo_objetivo_crudo = -30.0
            elif sentido_giro == "ANTIHORARIO":
                angulo_objetivo_crudo = 30.0
            else:
                angulo_objetivo_crudo = 0.0

            velocidad_base = VELOCIDAD_EVASION
            if (time.time() - tiempo_inicio_evasion) > 0.6:
                estado_evasion = "CARRERA"
                print("[FSM] FORZANDO_GIRO -> CARRERA")

        # ─────────────────────────────────────────────────────────────────────
        # 5. CÁLCULO DEL ÁNGULO OBJETIVO Y VELOCIDAD según estado
        # ─────────────────────────────────────────────────────────────────────

        if estado_evasion == "RETROCEDIENDO":
            if sentido_giro == "HORARIO":
                angulo_objetivo_crudo = 30.0
            elif sentido_giro == "ANTIHORARIO":
                angulo_objetivo_crudo = -30.0
            else:
                angulo_objetivo_crudo = 0.0
            velocidad_base = -35

        elif estado_evasion == "FORZANDO_GIRO":
            if sentido_giro == "HORARIO":
                angulo_objetivo_crudo = -30.0
            elif sentido_giro == "ANTIHORARIO":
                angulo_objetivo_crudo = 30.0
            else:
                angulo_objetivo_crudo = 0.0
            velocidad_base = VELOCIDAD_EVASION

        elif estado_evasion == "DETECTADO":
            # Pre-giro proporcional: un sesgo base indica el lado (regla de
            # color/WRO) y se corrige con la posición lateral real del
            # obstáculo medida por el tracker LiDAR (tracker["x"], mm;
            # x+ = derecha del robot). Alejarse de un obstáculo a la
            # derecha (x>0) exige ángulo positivo (izquierda), y viceversa,
            # así que la corrección tiene el mismo signo que tracker["x"].
            signo_evasion = 1.0 if EVADIR_POR_IZQUIERDA else -1.0
            correccion_lidar = tracker.tracker["x"] * KP_EVASION_LATERAL if tracker.tracker["activo"] else 0.0
            angulo_objetivo_crudo = signo_evasion * ANGULO_BASE_DETECTADO + correccion_lidar
            angulo_objetivo_crudo = max(-MAX_ANGULO_EVASION, min(MAX_ANGULO_EVASION, angulo_objetivo_crudo))
            velocidad_base = max(VELOCIDAD_MIN_EN_FRENADO,
                                 int(VELOCIDAD_CRUCERO * factor_frenado))

        elif estado_evasion == "ESQUIVANDO":
            # Mismo control proporcional que DETECTADO, con mayor sesgo
            # base (giro más comprometido) ya que el obstáculo fue confirmado.
            signo_evasion = 1.0 if EVADIR_POR_IZQUIERDA else -1.0
            correccion_lidar = tracker.tracker["x"] * KP_EVASION_LATERAL if tracker.tracker["activo"] else 0.0
            angulo_objetivo_crudo = signo_evasion * ANGULO_BASE_ESQUIVANDO + correccion_lidar
            angulo_objetivo_crudo = max(-MAX_ANGULO_EVASION, min(MAX_ANGULO_EVASION, angulo_objetivo_crudo))
            velocidad_base = VELOCIDAD_EVASION

        elif estado_evasion == "PASANDO":
            # Control P con IMU: mantener rumbo paralelo a las paredes
            error_h = heading_base_evasion - angulo_acumulado_robot
            angulo_objetivo_crudo = max(-22.0, min(22.0, error_h * KP_IMU))
            velocidad_base = VELOCIDAD_EVASION

        elif estado_evasion == "RECENTRANDO":
            # Control P más agresivo para volver al heading original. Antes se
            # capaba a +-25 grados y el estado se rendia por timeout (1.5s) sin
            # converger en errores grandes (~68 grados vistos en pista), dejando
            # al robot desalineado y disparando emergencias en cadena al volver
            # a CARRERA. Ahora usa el mismo limite MAX_ANGULO_EVASION que el
            # resto de la maniobra y tiene mas tiempo (TIMEOUT_RECENTRANDO) para
            # converger de verdad antes de rendirse.
            error_h = heading_base_evasion - angulo_acumulado_robot
            angulo_objetivo_crudo = max(-MAX_ANGULO_EVASION, min(MAX_ANGULO_EVASION, error_h * KP_IMU * 1.2))
            velocidad_base = VELOCIDAD_EVASION

        else:
            # CARRERA normal: centrado entre paredes por LiDAR
            error_lateral = lidar.dist_izquierda_min - lidar.dist_derecha_min
            angulo_objetivo_crudo = error_lateral * KP_LATERAL
            velocidad_base = VELOCIDAD_CRUCERO
            lidar.ANGULO_MIN_FRONTAL = 350
            lidar.ANGULO_MAX_FRONTAL = 10

        # Frenado de emergencia adicional en modo CARRERA
        if frontal < DIST_FRENADO_MIN and estado_evasion == "CARRERA":
            velocidad_base = VELOCIDAD_MIN_EN_FRENADO

        # 6. RATE LIMITER: limitar máxima variación de ángulo por ciclo
        delta = angulo_objetivo_crudo - ultimo_angulo_aplicado
        delta = max(-MAX_DELTA_ANGULO_POR_CICLO, min(MAX_DELTA_ANGULO_POR_CICLO, delta))
        angulo_objetivo        = ultimo_angulo_aplicado + delta
        ultimo_angulo_aplicado = angulo_objetivo

        # El frenado progresivo por distancia frontal (factor_frenado) solo
        # tiene sentido en CARRERA (frenar ante una pared/obstaculo que se
        # acerca de frente). En DETECTADO ya se aplico una vez al calcular
        # velocidad_base -- aplicarlo de nuevo aqui lo elevaba al cuadrado,
        # frenando de mas. En ESQUIVANDO/PASANDO/RECENTRANDO/RETROCEDIENDO/
        # FORZANDO_GIRO, "frontal" se mide con el sector frontal ensanchado
        # para no perder el cluster del obstaculo que se esta evadiendo --
        # frenar por esa lectura iba en contra de VELOCIDAD_EVASION justo
        # cuando el robot necesita mantener el impulso para completar el giro.
        if estado_evasion == "CARRERA":
            velocidad = max(VELOCIDAD_MIN_EN_FRENADO,
                            int(velocidad_base * factor_frenado))
        else:
            velocidad = int(velocidad_base)

        comando = f"{velocidad},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())

    # ─────────────────────────────────────────────────────────────────────────
    # FASE: BUSCANDO_PARQUEO (sin cambios respecto al código original)
    # ─────────────────────────────────────────────────────────────────────────
    elif fase_actual == "BUSCANDO_PARQUEO":
        error_lateral   = lidar.dist_izquierda_min - lidar.dist_derecha_min
        angulo_objetivo = error_lateral * KP_LATERAL
        comando = f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())

        match_firma = (abs(lidar.dist_derecha_min   - initial_derecha)   < 80.0 and
                       abs(lidar.dist_izquierda_min - initial_izquierda) < 80.0)
        tiempo_t     = time.time() - tiempo_inicio_parqueo
        timeout      = tiempo_t > TIMEOUT_BUSQUEDA_PARQUEO

        if match_firma or timeout:
            fase_actual = "DETENIDO"
            if timeout:
                print(f"[PARQUEO] Timeout ({tiempo_t:.1f}s). Deteniendo en zona segura.")
            else:
                print("[PARQUEO] Firma detectada! Estacionando...")
            for _ in range(5):
                ser_pico.write(b"0,0\n")
                time.sleep(0.01)
            apagar_sistema(None, None)


# ==========================================
# PUNTO DE ENTRADA
# ==========================================
if __name__ == '__main__':
    # 1. Cámara primero (necesita calentamiento / AE)
    t_camara = threading.Thread(target=vision.hilo_camara, args=(lambda: corriendo,), daemon=True)
    t_camara.start()

    # 2. Canal de comunicación con la Pico
    t_pico = threading.Thread(target=hilo_comunicacion_pico, daemon=True)
    t_pico.start()

    time.sleep(0.5)
    if ser_pico and ser_pico.is_open:
        ser_pico.write(b"0,0\n")
        print("[INIT] Direccion centrada.")

    print("\n[LISTO] SISTEMA LISTO (RONDA CON OBSTACULOS). Coloca el robot y presiona el Boton (GP21)...")
    while GPIO.input(PIN_BOTON) == GPIO.HIGH:
        if ser_pico and ser_pico.is_open:
            ser_pico.write(b"0,0\n")
        time.sleep(0.05)

    print("\n[START] Boton detectado! Iniciando carrera con obstaculos...")
    fase_actual = "CALIBRANDO"
    time.sleep(0.1)

    # 3. Hilo LiDAR (arranca después del botón para sincronizar el cero IMU)
    t_lidar = threading.Thread(
        target=lidar.hilo_lidar,
        args=(lambda: corriendo, lambda: fase_actual, _al_listo_lidar, _al_completar_scan),
        daemon=True
    )
    t_lidar.start()

    while corriendo:
        time.sleep(1)
