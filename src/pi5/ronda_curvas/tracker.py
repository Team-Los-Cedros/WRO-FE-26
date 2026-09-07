# Objetivo persistente: el pilar que la maniobra esta rodeando ahora
# mismo. Coordenadas del robot (x+ = derecha, y+ = frente, en mm), marco
# del LiDAR, que es el que devuelve centroide_xy_cluster.
#
# Lo que hace este modulo, y que el anterior no hacia:
#
#   1. IDENTIDAD. El objetivo tiene un id, un color y un lado obligatorio
#      congelados al confirmarlo. Ninguna observacion posterior puede
#      cambiarlos: un blob nuevo de otro color no reasigna nada, como
#      mucho VETA una asociacion.
#   2. INCERTIDUMBRE EXPLICITA. En vez de un timeout, la prediccion
#      acumula sigma (mm) y la puerta de asociacion se abre con ella. El
#      objetivo se da por perdido cuando sigma supera la escala a la que
#      ya no se puede distinguir de otro objeto (SIGMA_PERDIDO), no
#      cuando se cumple un reloj.
#   3. PUERTAS MULTIPLES. Posicion, rumbo, radio y ANCHO FISICO. El
#      ancho es el que rechaza las esquinas de muro, que era la forma
#      silenciosa de perder el pilar en una curva: un unico umbral de
#      250mm en distancia euclidea deja que el objetivo salte a la
#      quilla de una pared sin que nada lo note.
#   4. AMBIGUEDAD. Con dos candidatos igual de buenos NO se asocia. Es
#      preferible seguir prediciendo un ciclo que cambiar de pilar a
#      mitad de maniobra (el caso "dos pilares visibles a la vez").
#   5. PROGRESO GEOMETRICO. Se acumula el barrido del rumbo del pilar en
#      marco MUNDO (phi = rumbo_robot - yaw). Girar sobre el sitio no lo
#      mueve; solo lo mueve trasladarse alrededor del pilar. Es la unica
#      medida de "ya lo rodee" que no se puede falsificar rotando el
#      chasis, que es exactamente lo que hacia superado() (y < -280mm en
#      marco chasis) en una curva cerrada.
import math
import threading
import time

import geometria_robot as geo
from geometria_evasion import normalizar_180, rumbo

# Avance del robot a PWM 100%, en mm/s. MEDIDO en pista por odometria
# LiDAR sobre la lona, bateria llena:
#
#   PWM 40 -> 158 mm/s     PWM 70 -> 285 mm/s     PWM 90 -> 358 mm/s
#
# El ajuste da v = 4.02*pwm - 1.0, o sea practicamente proporcional, y un
# modelo proporcional con 400 clava los tres puntos dentro del 2%. La
# validacion cruzada tambien sale: predice 220 mm/s a PWM 55 y en carrera
# se midieron 215 sobre el CSV de metricas.
#
# Si se cambia la traccion, las ruedas o el voltaje del riel, hay que
# volver a medir: es una constante fisica, no un parametro de ajuste.
MM_POR_SEG_A_PWM100 = 400.0

# ==========================================
# INCERTIDUMBRE
# ==========================================
# Sigma tras una asociacion con el LiDAR. El C1 tiene ~15mm de ruido
# radial y el centroide de un cluster de un poste de 50mm anda por ahi.
SIGMA_MEDICION = 30.0

# Crecimiento por ciclo predicho. El primero es el error del modelo de
# avance (la curva de traccion clava los puntos al 2%, pero la bateria
# baja, el deslizamiento y la aceleracion no medida piden mas margen);
# el segundo es el error de escala de la IMU aplicado al brazo hasta el
# pilar, que es lo que domina en una curva cerrada.
SIGMA_AVANCE_REL = 0.12
SIGMA_GIRO_REL   = 0.10
SIGMA_BASE       = 2.0

# Escala a la que la estimacion deja de identificar al pilar: por encima
# de esto la puerta de asociacion es tan ancha que admitiria un pilar
# vecino o una esquina, asi que asociar seria peor que no hacerlo. El
# reglamento separa los pilares por celdas de la pista; 300mm es del
# orden del propio diametro del circulo donde se colocan (200mm).
SIGMA_PERDIDO = 300.0

# Red de seguridad, no criterio principal: si en 3 segundos no ha habido
# ni una sola medicion, algo va mal aunque la sigma diga otra cosa.
TIMEOUT_SIN_MEDICION = 3.0

# ==========================================
# PUERTAS DE ASOCIACION
# ==========================================
PUERTA_POS_MIN   = 90.0     # mm, con la estimacion recien medida
PUERTA_POS_MAX   = 280.0    # mm, tope aunque sigma crezca
PUERTA_RUMBO     = 14.0     # grados
PUERTA_RADIAL_MIN = 80.0    # mm
PUERTA_RADIAL_MAX = 220.0   # mm

# Ancho fisico admisible de un cluster para ser ESTE pilar. El poste del
# reglamento mide 50mm; el haz del C1 lo ensancha y el ruido lo estira,
# pero un tramo de muro o la quilla de una esquina se van muy por
# encima. lidar_geometria.es_objeto_estrecho ya filtra a 260mm para el
# control de pared; aqui se aprieta porque la pregunta es otra: no "es
# estrecho" sino "es el mismo objeto de 50mm que vengo siguiendo".
ANCHO_MIN_PILAR = 10.0
ANCHO_MAX_PILAR = 180.0

# Dos candidatos cuyos costes se parezcan menos de esto son
# indistinguibles: no se asocia ninguno y se sigue prediciendo.
MARGEN_AMBIGUEDAD = 0.25

_siguiente_id = [0]


class TrackerObstaculo:
    def __init__(self):
        self._lock = threading.Lock()
        self.activo = False
        self.id = 0
        self.color = None          # "ROJO" o "VERDE", congelado
        self.s_lado = 0            # +1 pilar a la derecha, -1 a la izquierda
        self.sembrado = False      # sembrado solo con vision, sin cluster
        self.x = 0.0
        self.y = 0.0
        self.sigma = SIGMA_PERDIDO
        self.confianza = 0.0
        self.asociaciones = 0
        self.ciclos_predichos = 0
        self.ambiguo = False

        # Progreso geometrico alrededor del pilar
        self.barrido = 0.0         # grados de rumbo mundo barridos (con signo)
        self.progreso = 0.0        # barrido * s_lado, crece rodeando bien

        # Aproximacion maxima registrada durante la maniobra
        self.d_min = float("inf")
        self.u_en_d_min = 0.0      # separacion lateral por el lado bueno
        self.sigma_en_d_min = 0.0

        self._heading = 0.0
        self._heading_prev = None
        self._phi_prev = None
        self._phi_acum = 0.0
        self._phi_commit = 0.0
        self._t_medicion = 0.0

    # ==========================================
    # CICLO DE VIDA
    # ==========================================
    def iniciar(self, color, s_lado, x_mm, y_mm, heading, ahora=None,
                sigma=SIGMA_MEDICION, sembrado=False):
        # Congela identidad, color y lado. A partir de aqui nada los
        # cambia: solo soltar() termina el objetivo.
        ahora = time.time() if ahora is None else ahora
        _siguiente_id[0] += 1
        with self._lock:
            self.activo = True
            self.id = _siguiente_id[0]
            self.color = color
            self.s_lado = s_lado
            self.sembrado = sembrado
            self.x, self.y = x_mm, y_mm
            self.sigma = sigma
            self.confianza = max(0.0, 1.0 - sigma / SIGMA_PERDIDO)
            self.asociaciones = 1
            self.ciclos_predichos = 0
            self.ambiguo = False
            self.barrido = 0.0
            self.progreso = 0.0
            self.d_min = math.hypot(x_mm, y_mm)
            self.u_en_d_min = s_lado * x_mm
            self.sigma_en_d_min = sigma
            self._heading = heading
            self._heading_prev = heading
            self._phi_prev = rumbo(x_mm, y_mm) - heading
            self._phi_acum = self._phi_prev
            self._phi_commit = self._phi_prev
            self._t_medicion = ahora
        lado = "DERECHA" if s_lado > 0 else "IZQUIERDA"
        print("[OBJETIVO] #%d %s en (%.0f, %.0f)mm, debe quedar a la %s"
              % (self.id, color, x_mm, y_mm, lado))

    def marcar_compromiso(self):
        # La maniobra empieza AQUI: el barrido y la aproximacion maxima se
        # miden desde el compromiso, no desde que el pilar se vio por
        # primera vez. Un pilar detectado de lejos y seguido durante dos
        # segundos ya habria acumulado barrido sin haber rodeado nada.
        with self._lock:
            self._phi_commit = self._phi_acum
            self.barrido = 0.0
            self.progreso = 0.0
            xr, yr = geo.lidar_a_eje_trasero(self.x, self.y)
            self.d_min = math.hypot(xr, yr)
            self.u_en_d_min = self.s_lado * xr
            self.sigma_en_d_min = self.sigma

    def soltar(self, razon=""):
        with self._lock:
            estaba = self.activo
            self.activo = False
            self.confianza = 0.0
        if estaba and razon:
            print("[OBJETIVO] #%d soltado: %s" % (self.id, razon))

    # ==========================================
    # PREDICCION
    # ==========================================
    def predecir(self, heading, avance_mm, ahora=None):
        """Una vez por barrido, ANTES de leer x/y en la navegacion.

        El movimiento entre ciclos es un ARCO, no una rotacion seguida de
        una traslacion recta: con el servo al tope el robot gira 2-3
        grados por ciclo mientras avanza 20mm, y el desplazamiento
        lateral que eso produce no lo veia el modelo anterior. Se integra
        en el marco del EJE TRASERO, que es donde el arco es exacto: el
        LiDAR va 128mm por delante y describe otra curva distinta.
        """
        if not self.activo:
            return
        ahora = time.time() if ahora is None else ahora

        giro = 0.0 if self._heading_prev is None else (heading - self._heading_prev)
        giro = normalizar_180(giro)
        self._heading_prev = heading
        self._heading = heading

        with self._lock:
            xr, yr = geo.lidar_a_eje_trasero(self.x, self.y)
            th = math.radians(giro)
            if abs(th) < 1e-4:
                dx, dy = 0.0, avance_mm
            else:
                radio = avance_mm / th
                dx = -radio * (1.0 - math.cos(th))
                dy = radio * math.sin(th)
            xa, ya = xr - dx, yr - dy
            c, s = math.cos(th), math.sin(th)
            xr2 = xa * c + ya * s
            yr2 = -xa * s + ya * c
            self.x, self.y = geo.eje_trasero_a_lidar(xr2, yr2)

            brazo = math.hypot(xr2, yr2)
            self.sigma += (SIGMA_BASE
                           + SIGMA_AVANCE_REL * abs(avance_mm)
                           + SIGMA_GIRO_REL * brazo * abs(th))
            self.ciclos_predichos += 1
            self.confianza = max(0.0, 1.0 - self.sigma / SIGMA_PERDIDO)

        self._refrescar_progreso()

    # ==========================================
    # ASOCIACION
    # ==========================================
    def asociar(self, candidatos):
        """Corrige la prediccion con una medicion del LiDAR.

        `candidatos` son tuplas (x, y, ancho_mm) en marco LiDAR, ya
        filtradas por la navegacion. Devuelve True si se asocio.

        Un candidato pasa si supera TODAS las puertas -- posicion, rumbo,
        radio y ancho -- y ademas es claramente mejor que el segundo. Con
        dos pilares en el mismo frame, o con una esquina de muro justo
        detras del pilar, la puerta unica de 250mm en distancia euclidea
        del tracker anterior elegia "el mas cercano a la prediccion" sin
        margen para dudar: bastaba un ciclo malo para cambiar de objeto y
        seguir la maniobra contra el objeto equivocado.
        """
        if not self.activo or not candidatos:
            return False

        with self._lock:
            xp, yp = self.x, self.y
            sigma = self.sigma
        rp = math.hypot(xp, yp)
        bp = rumbo(xp, yp)

        puerta_pos = min(PUERTA_POS_MAX, max(PUERTA_POS_MIN, 3.0 * sigma))
        puerta_rad = min(PUERTA_RADIAL_MAX, max(PUERTA_RADIAL_MIN, 3.0 * sigma))

        mejores = []
        for cand in candidatos:
            cx, cy = cand[0], cand[1]
            ancho = cand[2] if len(cand) > 2 else 50.0
            if not (ANCHO_MIN_PILAR <= ancho <= ANCHO_MAX_PILAR):
                continue
            d = math.hypot(cx - xp, cy - yp)
            if d > puerta_pos:
                continue
            if abs(normalizar_180(rumbo(cx, cy) - bp)) > PUERTA_RUMBO:
                continue
            if abs(math.hypot(cx, cy) - rp) > puerta_rad:
                continue
            mejores.append((d / puerta_pos, cx, cy))

        self.ambiguo = False
        if not mejores:
            return False
        mejores.sort()
        if len(mejores) > 1:
            # Coste casi identico = no se puede decidir. Predecir un
            # ciclo mas cuesta sigma; equivocarse de pilar cuesta la
            # maniobra entera.
            if (mejores[1][0] - mejores[0][0]) < MARGEN_AMBIGUEDAD:
                self.ambiguo = True
                return False

        _, cx, cy = mejores[0]
        with self._lock:
            self.x, self.y = cx, cy
            self.sigma = SIGMA_MEDICION
            self.confianza = 1.0
            self.asociaciones += 1
            self.ciclos_predichos = 0
            self._t_medicion = time.time()
        self._refrescar_progreso()
        self._refrescar_aproximacion()
        return True

    def corregir_rumbo(self, delta_grados, factor_sigma=0.7):
        """Readquisicion por camara: la camara da RUMBO, no distancia.

        Se rota la estimacion alrededor del robot para que su rumbo
        coincida con el que ve la camara, conservando el radio. Reduce
        sigma solo parcialmente: se ha corregido una de las dos
        coordenadas, no las dos.
        """
        if not self.activo:
            return
        with self._lock:
            r = math.hypot(self.x, self.y)
            b = rumbo(self.x, self.y) + delta_grados
            self.x = r * math.sin(math.radians(b))
            self.y = r * math.cos(math.radians(b))
            self.sigma = max(SIGMA_MEDICION, self.sigma * factor_sigma)
            self.confianza = max(0.0, 1.0 - self.sigma / SIGMA_PERDIDO)
        self._refrescar_progreso()

    # ==========================================
    # DERIVADOS
    # ==========================================
    def _refrescar_progreso(self):
        # phi = rumbo del pilar en marco MUNDO, con la convencion horaria
        # del LiDAR. Girando el chasis a la izquierda (yaw +D), un objeto
        # fijo se ve moverse hacia la DERECHA del robot, o sea su rumbo
        # crece tambien +D; la RESTA es por tanto lo invariante a la
        # rotacion. Girando sobre el sitio, phi no se mueve: solo se
        # mueve trasladandose alrededor del pilar, que es justo lo que
        # hay que medir para saber si se ha rodeado.
        with self._lock:
            phi = rumbo(self.x, self.y) - self._heading
            if self._phi_prev is None:
                self._phi_prev = phi
                self._phi_acum = phi
                self._phi_commit = phi
            else:
                self._phi_acum += normalizar_180(phi - self._phi_prev)
                self._phi_prev = phi
            self.barrido = self._phi_acum - self._phi_commit
            self.progreso = self.s_lado * self.barrido

    def _refrescar_aproximacion(self):
        with self._lock:
            xr, yr = geo.lidar_a_eje_trasero(self.x, self.y)
            d = math.hypot(xr, yr)
            if d < self.d_min:
                self.d_min = d
                self.u_en_d_min = self.s_lado * xr
                self.sigma_en_d_min = self.sigma

    def xy_eje(self):
        # El pilar en el marco del EJE TRASERO, que es el unico donde la
        # cinematica de bicicleta es valida. Todo geometria_evasion.py
        # trabaja en este marco.
        with self._lock:
            return geo.lidar_a_eje_trasero(self.x, self.y)

    def separacion_lateral(self):
        # Separacion por el lado bueno (mm). Negativa = el pilar esta del
        # lado PROHIBIDO.
        xr, _ = self.xy_eje()
        return self.s_lado * xr

    def distancia(self):
        xr, yr = self.xy_eje()
        return math.hypot(xr, yr)

    def separacion_minima_garantizada(self):
        # La aproximacion maxima que hubo, descontando la incertidumbre
        # que tenia la estimacion en ese instante. Es el numero con el
        # que se declara legal el paso: si la estimacion era mala, el
        # invariante exige mas separacion real.
        return self.u_en_d_min - 1.5 * self.sigma_en_d_min

    def sin_medicion_desde(self, ahora=None):
        ahora = time.time() if ahora is None else ahora
        return ahora - self._t_medicion

    def degradado(self):
        # Estimacion viva pero ya no fiable para geometria fina.
        return self.activo and self.confianza < 0.45

    def perdido(self, ahora=None):
        if not self.activo:
            return True
        return (self.sigma >= SIGMA_PERDIDO
                or self.sin_medicion_desde(ahora) > TIMEOUT_SIN_MEDICION)

    def confirmado(self):
        # Dos mediciones reales del LiDAR: no es un falso positivo.
        return self.activo and self.asociaciones >= 2
