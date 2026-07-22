# Cerebro de la Ronda Cerrada. La clase Navegador recibe por cada barrido
# del LiDAR la medicion, el color de la camara y el yaw de la IMU, y
# devuelve la consigna (velocidad, angulo) para la Pico. No abre puertos
# ni hilos, asi que se puede probar fuera del robot con barridos grabados.
#
# Fases: CAPTURA_FIRMA -> CARRERA -> PARQUEO -> FIN
#
# Estados de evasion dentro de CARRERA:
#   CRUCERO          centrado entre paredes (P sobre el error lateral)
#   APROXIMACION     pure pursuit hacia el punto de paso al lado del poste
#                    (ROJO -> por la derecha, VERDE -> por la izquierda)
#   SOBREPASO        rumbo paralelo al pasillo hasta dejar el poste atras
#   REINCORPORACION  volver al rumbo original con P sobre la IMU
#   RETROCESO        emergencia anti-choque: reversa con contra-giro
#   REORIENTACION    avance corto girando hacia el sentido de la pista
#
# La emergencia se chequea en todos los ciclos sin importar el estado.
# Angulo positivo = giro a la izquierda. El servo solo da -20/+25 grados
# reales, de ahi todos los clamps.
import math
import time

import tracker as tracker_mod
from lidar import centroide_xy_cluster

# ==========================================
# VELOCIDADES (% PWM)
# ==========================================
VELOCIDAD_CRUCERO  = 55
VELOCIDAD_EVASION  = 40
VELOCIDAD_PARQUEO  = 20
VELOCIDAD_REVERSA  = -35
VELOCIDAD_MINIMA   = 25      # piso del frenado progresivo

# ==========================================
# SEGUIMIENTO DE PARED Y FRENADO
# ==========================================
KP_LATERAL = 0.14            # calibrado en pista, mismo valor que la Ronda Abierta

DIST_FRENADO_INICIO = 900.0  # mm, empieza a bajar velocidad
DIST_FRENADO_MIN    = 300.0  # mm, velocidad minima alcanzada

# ==========================================
# EVASION
# ==========================================
SEPARACION_LATERAL   = 260.0  # mm entre el centro del poste y el punto de paso
KP_PURSUIT           = 1.0    # grados de comando por grado de bearing
KP_HEADING           = 1.0    # P de rumbo en SOBREPASO / REINCORPORACION
MAX_ANGULO_EVASION   = 25.0   # tope fisico util del servo
ANGULO_EVASION_CIEGA = 18.0   # sesgo fijo si hay color pero todavia no hay cluster

DIST_INICIO_EVASION_TRK = 900.0   # mm, poste confirmado por tracker
DIST_INICIO_EVASION_CAM = 700.0   # mm, frontal LiDAR + color de camara
Y_POSTE_EN_PASO         = 180.0   # mm, el poste ya esta a la altura del morro

TIMEOUT_APROXIMACION   = 1.5   # s, red de seguridad si el tracker no resuelve
TIMEOUT_SOBREPASO      = 1.2
TIMEOUT_REINCORPORACION = 2.5
ERROR_HEADING_OK       = 5.0   # grados para dar la reincorporacion por buena

# ==========================================
# EMERGENCIA ANTI-CHOQUE
# ==========================================
EMERGENCIA_FRONTAL  = 120.0
EMERGENCIA_LATERAL  = 80.0
EMERGENCIA_TRASERA  = 250.0
TIMEOUT_RETROCESO   = 3.5
DURACION_REORIENTAR = 0.6
ANGULO_EMERGENCIA   = 25.0

# ==========================================
# CARRERA / PARQUEO
# ==========================================
UMBRAL_VUELTAS       = 1010.0  # grados de yaw neto, ~3 vueltas
TOLERANCIA_FIRMA     = 80.0    # mm contra la firma de pared inicial
TIMEOUT_PARQUEO      = 6.0     # s

# Sectores frontales ensanchados durante la evasion para no perder el poste
SECTOR_EVASION_IZQ = (330.0, 20.0)
SECTOR_EVASION_DER = (340.0, 30.0)

# Rate limiter del servo en marcha normal (la emergencia lo salta)
MAX_DELTA_ANGULO = 6.0


def _clamp(v, lim):
    return max(-lim, min(lim, v))


class Navegador:
    def __init__(self, control_sector):
        # control_sector: objeto con fijar_sector_frontal() y
        # sector_frontal_normal(), normalmente el LidarC1
        self._sector = control_sector
        self.tracker = tracker_mod.TrackerObstaculo()

        self.fase   = "CAPTURA_FIRMA"
        self.estado = "CRUCERO"

        self._firma_izq = 0.0
        self._firma_der = 0.0

        self._sentido = "DESCONOCIDO"          # HORARIO / ANTIHORARIO
        self._evadir_por_izquierda = True
        self._heading_base  = 0.0              # rumbo del pasillo al iniciar la evasion
        self._t_estado      = 0.0
        self._t_parqueo     = 0.0

        self._ultimo_angulo = 0.0              # para el rate limiter
        self._ultima_vel    = 0
        self._t_ultimo_ciclo = None

    def procesar(self, med, color_cam, heading, ahora=None):
        # Una llamada por barrido completo. Devuelve (velocidad, angulo)
        # o None cuando la carrera termino.
        if ahora is None:
            ahora = time.time()

        # dt del ciclo para la odometria del tracker
        dt = 0.0 if self._t_ultimo_ciclo is None else min(0.3, ahora - self._t_ultimo_ciclo)
        self._t_ultimo_ciclo = ahora

        if self.fase == "CAPTURA_FIRMA":
            self._firma_izq, self._firma_der = med.izquierda, med.derecha
            self.fase = "CARRERA"
            print(f"[+] Firma de parqueo: Izq={self._firma_izq:.0f} Der={self._firma_der:.0f}mm")
            print("[INICIO] Carrera con obstaculos iniciada!")
            return (0, 0.0)

        if self.fase == "CARRERA":
            return self._ciclo_carrera(med, color_cam, heading, ahora, dt)

        if self.fase == "PARQUEO":
            return self._ciclo_parqueo(med, ahora)

        return None    # FIN

    # ==========================================
    # FASE CARRERA
    # ==========================================
    def _ciclo_carrera(self, med, color_cam, heading, ahora, dt):
        # 0. Odometria del tracker (rotacion IMU + avance estimado)
        avance_mm = (self._ultima_vel / 100.0) * tracker_mod.MM_POR_SEG_A_PWM100 * dt
        self.tracker.predecir(heading, avance_mm)
        if self.tracker.activo:
            self.tracker.asociar(med.clusters_obstaculo, centroide_xy_cluster)
        else:
            self._intentar_capturar_poste(med, color_cam, heading)

        # 1. Sentido de giro de la pista, se detecta una sola vez
        if self._sentido == "DESCONOCIDO":
            if heading < -45.0:
                self._sentido = "HORARIO"
                print("[GIRO] Sentido: HORARIO (derecha)")
            elif heading > 45.0:
                self._sentido = "ANTIHORARIO"
                print("[GIRO] Sentido: ANTIHORARIO (izquierda)")

        # 2. Emergencia anti-choque, prioridad sobre cualquier estado
        if (med.frontal < EMERGENCIA_FRONTAL
                or med.izquierda < EMERGENCIA_LATERAL
                or med.derecha < EMERGENCIA_LATERAL):
            if self.estado not in ("RETROCESO", "REORIENTACION"):
                self._entrar("RETROCESO", ahora)
                self.tracker.desactivar("emergencia")
                self._sector.sector_frontal_normal()
                print(f"[EMERGENCIA] F:{med.frontal:.0f} I:{med.izquierda:.0f} "
                      f"D:{med.derecha:.0f}mm -> RETROCESO")

        # 3. Vueltas completas -> parqueo. Solo desde CRUCERO para no
        #    abandonar una evasion a medias con un poste al lado
        if abs(heading) >= UMBRAL_VUELTAS and self.estado == "CRUCERO":
            self.fase = "PARQUEO"
            self._t_parqueo = ahora
            print(f"[!] {heading:.0f} grados acumulados. Modo Parqueo.")
            return (VELOCIDAD_PARQUEO, self._centrado_paredes(med))

        # 4. Despacho por estado
        manejador = {
            "CRUCERO":         self._est_crucero,
            "APROXIMACION":    self._est_aproximacion,
            "SOBREPASO":       self._est_sobrepaso,
            "REINCORPORACION": self._est_reincorporacion,
            "RETROCESO":       self._est_retroceso,
            "REORIENTACION":   self._est_reorientacion,
        }[self.estado]
        velocidad, angulo = manejador(med, color_cam, heading, ahora)

        # 5. Rate limiter del servo. En emergencia no se aplica, ahi el
        #    giro completo tiene que entrar de una
        if self.estado in ("RETROCESO", "REORIENTACION"):
            self._ultimo_angulo = angulo
        else:
            delta = _clamp(angulo - self._ultimo_angulo, MAX_DELTA_ANGULO)
            angulo = self._ultimo_angulo + delta
            self._ultimo_angulo = angulo

        self._ultima_vel = velocidad
        return (velocidad, angulo)

    def _est_crucero(self, med, color_cam, heading, ahora):
        trk = self.tracker
        poste_en_rango = trk.confirmado() and 50.0 < trk.y < DIST_INICIO_EVASION_TRK
        vision_en_rango = color_cam is not None and 50.0 < med.frontal < DIST_INICIO_EVASION_CAM

        if poste_en_rango or vision_en_rango:
            color = trk.color if trk.activo and trk.color else color_cam
            self._evadir_por_izquierda = (color == "VERDE")
            self._heading_base = heading
            self._entrar("APROXIMACION", ahora)
            if self._evadir_por_izquierda:
                self._sector.fijar_sector_frontal(*SECTOR_EVASION_IZQ)
            else:
                self._sector.fijar_sector_frontal(*SECTOR_EVASION_DER)
            lado = "IZQUIERDA" if self._evadir_por_izquierda else "DERECHA"
            print(f"[FSM] CRUCERO -> APROXIMACION | {color} | paso por {lado} | "
                  f"F:{med.frontal:.0f}mm")

        velocidad = self._con_frenado(VELOCIDAD_CRUCERO, med.frontal)
        return (velocidad, self._centrado_paredes(med))

    def _est_aproximacion(self, med, color_cam, heading, ahora):
        trk = self.tracker
        t_en_estado = ahora - self._t_estado

        # Evadiendo por la izquierda el poste queda a la derecha del robot
        lado_poste = "DER" if self._evadir_por_izquierda else "IZQ"
        if trk.activo and (trk.y < Y_POSTE_EN_PASO or trk.al_costado(lado_poste)
                           or trk.superado()):
            self._entrar("SOBREPASO", ahora)
            print(f"[FSM] APROXIMACION -> SOBREPASO | poste en ({trk.x:.0f},{trk.y:.0f})mm")
        elif t_en_estado > TIMEOUT_APROXIMACION:
            # Sin tracker resuelto asumimos que ya avanzamos lo suficiente
            self._entrar("SOBREPASO", ahora)
            print("[FSM] APROXIMACION -> SOBREPASO | timeout")

        # Pure pursuit hacia el punto de paso al lado del poste
        if trk.activo:
            signo = -1.0 if self._evadir_por_izquierda else 1.0
            x_obj = trk.x + signo * SEPARACION_LATERAL
            y_obj = max(150.0, trk.y)          # evita el atan2 degenerado de cerca
            bearing = math.degrees(math.atan2(x_obj, y_obj))
            angulo = _clamp(-bearing * KP_PURSUIT, MAX_ANGULO_EVASION)
        else:
            # Evasion a ciegas: solo la regla de color, sesgo fijo moderado
            signo = 1.0 if self._evadir_por_izquierda else -1.0
            angulo = signo * ANGULO_EVASION_CIEGA

        velocidad = self._con_frenado(VELOCIDAD_EVASION, med.frontal)
        return (max(VELOCIDAD_MINIMA, velocidad), angulo)

    def _est_sobrepaso(self, med, color_cam, heading, ahora):
        t_en_estado = ahora - self._t_estado

        if self.tracker.superado() or t_en_estado > TIMEOUT_SOBREPASO:
            razon = "geometrico" if self.tracker.superado() else "timeout"
            self.tracker.desactivar(f"superado ({razon})")
            self._entrar("REINCORPORACION", ahora)
            self._sector.sector_frontal_normal()
            print(f"[FSM] SOBREPASO -> REINCORPORACION | {razon}")

        # Rumbo paralelo al pasillo mientras el poste pasa por el costado
        error_h = self._heading_base - heading
        angulo = _clamp(error_h * KP_HEADING, 22.0)
        return (VELOCIDAD_EVASION, angulo)

    def _est_reincorporacion(self, med, color_cam, heading, ahora):
        error_h = self._heading_base - heading
        t_en_estado = ahora - self._t_estado

        if abs(error_h) < ERROR_HEADING_OK or t_en_estado > TIMEOUT_REINCORPORACION:
            self._entrar("CRUCERO", ahora)
            print(f"[FSM] REINCORPORACION -> CRUCERO | error rumbo {error_h:.1f} grados")
            return (VELOCIDAD_CRUCERO, self._centrado_paredes(med))

        angulo = _clamp(error_h * KP_HEADING * 1.2, MAX_ANGULO_EVASION)
        return (VELOCIDAD_EVASION, angulo)

    def _est_retroceso(self, med, color_cam, heading, ahora):
        t_en_estado = ahora - self._t_estado
        if med.trasera < EMERGENCIA_TRASERA or t_en_estado > TIMEOUT_RETROCESO:
            razon = "obstaculo trasero" if med.trasera < EMERGENCIA_TRASERA else "tiempo maximo"
            self._entrar("REORIENTACION", ahora)
            print(f"[FSM] RETROCESO -> REORIENTACION ({razon})")

        # Reversa con las ruedas hacia la pared exterior: al retroceder
        # el morro apunta hacia adentro de la pista
        angulo = self._angulo_por_sentido(ANGULO_EMERGENCIA)
        return (VELOCIDAD_REVERSA, angulo)

    def _est_reorientacion(self, med, color_cam, heading, ahora):
        if ahora - self._t_estado > DURACION_REORIENTAR:
            self._entrar("CRUCERO", ahora)
            print("[FSM] REORIENTACION -> CRUCERO")

        angulo = self._angulo_por_sentido(-ANGULO_EMERGENCIA)
        return (VELOCIDAD_EVASION, angulo)

    # ==========================================
    # FASE PARQUEO
    # ==========================================
    def _ciclo_parqueo(self, med, ahora):
        match_firma = (abs(med.derecha - self._firma_der) < TOLERANCIA_FIRMA and
                       abs(med.izquierda - self._firma_izq) < TOLERANCIA_FIRMA)
        timeout = (ahora - self._t_parqueo) > TIMEOUT_PARQUEO

        if match_firma or timeout:
            self.fase = "FIN"
            print("[PARQUEO] " + ("Firma detectada! Estacionando..." if match_firma
                                  else "Timeout. Deteniendo en zona segura."))
            return None

        return (VELOCIDAD_PARQUEO, self._centrado_paredes(med))

    # ==========================================
    # AUXILIARES
    # ==========================================
    def _entrar(self, estado, ahora):
        self.estado    = estado
        self._t_estado = ahora

    def _centrado_paredes(self, med):
        # Control P clasico de centrado entre las dos paredes
        return (med.izquierda - med.derecha) * KP_LATERAL

    def _con_frenado(self, velocidad_base, frontal):
        # Rampa lineal de velocidad segun la distancia frontal libre
        if frontal >= DIST_FRENADO_INICIO:
            return velocidad_base
        if frontal <= DIST_FRENADO_MIN:
            return VELOCIDAD_MINIMA
        proporcion = (frontal - DIST_FRENADO_MIN) / (DIST_FRENADO_INICIO - DIST_FRENADO_MIN)
        return int(VELOCIDAD_MINIMA + proporcion * (velocidad_base - VELOCIDAD_MINIMA))

    def _angulo_por_sentido(self, magnitud):
        # +magnitud si la pista es horaria, -magnitud si es antihoraria
        if self._sentido == "HORARIO":
            return magnitud
        if self._sentido == "ANTIHORARIO":
            return -magnitud
        return 0.0

    def _intentar_capturar_poste(self, med, color_cam, heading):
        # Crea el tracker cuando camara y LiDAR coinciden en un poste frontal
        if color_cam is None or not med.clusters_obstaculo:
            return

        mejor_d, mejor_xy = 1e9, None
        for clust in med.clusters_obstaculo:
            cx, cy = centroide_xy_cluster(clust)
            if cy > 80.0 and abs(cx) < 450.0:      # zona frontal razonable
                d = math.hypot(cx, cy)
                if d < mejor_d:
                    mejor_d, mejor_xy = d, (cx, cy)

        if mejor_xy is not None:
            self.tracker.iniciar(color_cam, mejor_xy[0], mejor_xy[1], heading)
