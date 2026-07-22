# Tracker del poste activo en coordenadas del robot (x+ = derecha,
# y+ = frente, en mm). Mantiene la posicion estimada aunque la camara
# pierda el poste durante el giro:
#   - predice con la IMU (rotacion) y con el avance estimado del robot
#     (velocidad comandada * dt), asi el poste "retrocede" en el marco
#     del robot aunque no haya medicion nueva
#   - cuando el LiDAR entrega un cluster cerca de la prediccion, la
#     posicion se corrige con la medicion real
import math
import time
import threading

# Avance del robot a PWM 100%, en mm/s. Calibrar en pista: cronometrar
# 2m a velocidad fija y despejar. Si queda impreciso no pasa nada grave,
# el LiDAR re-ancla la posicion en cada barrido.
MM_POR_SEG_A_PWM100 = 900.0

UMBRAL_ASOCIACION  = 250.0   # mm entre cluster y prediccion para asociar
TIMEOUT_TRACKER    = 4.0     # s maximos prediciendo sin re-deteccion
DISTANCIA_SUPERADO = 280.0   # mm detras del robot (cola ~200mm + margen)
ZONA_LATERAL_Y     = 350.0   # |y| menor a esto cuenta como "al costado"


class TrackerObstaculo:
    def __init__(self):
        self._lock = threading.Lock()
        self.activo         = False
        self.color          = None     # "ROJO" o "VERDE"
        self.x              = 0.0
        self.y              = 0.0
        self.confirmaciones = 0
        self._heading_ref   = 0.0
        self._timestamp     = 0.0

    def iniciar(self, color, x_mm, y_mm, heading):
        with self._lock:
            self.activo         = True
            self.color          = color
            self.x              = x_mm
            self.y              = y_mm
            self.confirmaciones = 1
            self._heading_ref   = heading
            self._timestamp     = time.time()
        print(f"[TRACKER] Iniciado: {color} en ({x_mm:.0f}, {y_mm:.0f})mm")

    def desactivar(self, razon=""):
        with self._lock:
            self.activo = False
        if razon:
            print(f"[TRACKER] Desactivado: {razon}")

    def predecir(self, heading, avance_mm):
        # Llamar una vez por barrido, antes de leer x/y en la navegacion
        if not self.activo:
            return
        if time.time() - self._timestamp > TIMEOUT_TRACKER:
            self.desactivar("timeout de prediccion")
            return

        delta_rad = math.radians(heading - self._heading_ref)
        cos_d = math.cos(-delta_rad)
        sin_d = math.sin(-delta_rad)
        with self._lock:
            x_old, y_old = self.x, self.y
            # El robot giro delta -> el punto rota al reves en su marco
            self.x = x_old * cos_d - y_old * sin_d
            self.y = x_old * sin_d + y_old * cos_d
            # El robot avanzo -> el punto retrocede
            self.y -= avance_mm
            self._heading_ref = heading

    def asociar(self, clusters, centroide_fn):
        # Corrige la prediccion con el cluster real mas cercano.
        # centroide_fn viene de lidar.py, se pasa como argumento para no
        # importar lidar desde aqui.
        if not self.activo or not clusters:
            return False

        mejor_d, mejor_xy = UMBRAL_ASOCIACION, None
        for clust in clusters:
            cx, cy = centroide_fn(clust)
            d = math.hypot(cx - self.x, cy - self.y)
            if d < mejor_d:
                mejor_d, mejor_xy = d, (cx, cy)

        if mejor_xy is None:
            return False
        with self._lock:
            self.x, self.y = mejor_xy
            self.confirmaciones += 1
            self._timestamp = time.time()
        return True

    # Predicados que usa la maquina de estados

    def confirmado(self):
        # El LiDAR corrigio la posicion al menos 2 veces, no es falso positivo
        return self.activo and self.confirmaciones >= 2

    def al_frente(self):
        return self.activo and self.y > 50.0

    def al_costado(self, lado):
        if not self.activo or abs(self.y) >= ZONA_LATERAL_Y:
            return False
        return self.x > 50.0 if lado == "DER" else self.x < -50.0

    def superado(self):
        # El poste quedo detras de la cola del robot
        return self.activo and self.y < -DISTANCIA_SUPERADO
