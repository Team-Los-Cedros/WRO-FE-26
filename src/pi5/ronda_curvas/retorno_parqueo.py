"""Retorno al segmento de salida y parqueo por medidas, sin I/O de hardware.

El yaw habilita la busqueda tras tres vueltas; las paredes del origen y los
dos delimitadores confirman el lugar. Un reloj agotado siempre es FALLO.
La bahia usa el detector y la FSM de varios tiempos conservados en parqueo/.
"""
import json
import math
from dataclasses import replace
from pathlib import Path
from statistics import median

import geometria_evasion as gev
import geometria_robot as geo
from parqueo.estacionamiento import ControlEstacionamiento, FALLO
from parqueo.modelos import Consigna
from parqueo.percepcion_lidar import PercepcionLidar
from parqueo.seguidor import SeguidorBahia


def cargar_config():
    with Path(__file__).with_name("parqueo_config.json").open(encoding="utf-8") as archivo:
        return json.load(archivo)


class RetornoParqueo:
    def __init__(self, config=None):
        self.config = cargar_config() if config is None else config
        self.percepcion = PercepcionLidar(self.config)
        self.control = ControlEstacionamiento(self.config)
        self.paredes = None
        self.hueco = None
        self.scan = []
        self.origen = None
        self._muestras = []
        self._inicio = None
        self._h0 = None
        self._hprev = None
        self.sentido = 0
        self.lado = 0
        self.esquinas = 0
        self._votos_esquina = 0
        self._distancia_esquina = 0.0
        self._confirmaciones = 0
        self._t_busqueda = None
        self.error = ""
        self._sin_ultrasonido = 0
        self._ancla_bahia = None
        self.seguidor = None

    def observar(self, med, heading, ahora, angulo, velocidad, dt):
        if not math.isfinite(heading):
            return
        if self._hprev is not None and abs(heading - self._hprev) > 45.0:
            self.error = "salto de IMU incompatible con un barrido"
        self._hprev = heading
        perfil = getattr(med, "perfil", None)
        if perfil is None:  # dobles de las pruebas antiguas, sin hardware
            return
        self.scan = [(float(a), float(d)) for a, d in enumerate(perfil)
                     if math.isfinite(d) and 0.0 < d < 7999.0]
        if self._h0 is not None:
            relativo = heading - self._h0
            if not self.sentido and abs(relativo) >= 60.0:
                self.sentido = 1 if relativo > 0 else -1
            if self.sentido and not self.lado:
                self.lado = self.sentido  # el exterior es izq en horario
            neto = relativo * self.sentido
            # Cada cuadrante necesita avance y tres barridos persistentes.
            # Los retrocesos no aportan distancia ni reinician las esquinas.
            self._distancia_esquina += max(0, velocidad) * 4.0 * max(0, dt)
            siguiente = 90.0 * (self.esquinas + 1)
            cerca = siguiente - 15.0 <= neto <= siguiente + 45.0
            self._votos_esquina = self._votos_esquina + 1 if cerca else 0
            if (self.esquinas < 12 and self._votos_esquina >= 3
                    and self._distancia_esquina >= 500.0):
                self.esquinas += 1
                self._votos_esquina = 0
                self._distancia_esquina = 0.0
            if neto >= 990.0 and self._t_busqueda is None:
                self._t_busqueda = ahora
            if self.control._t_inicio is None and self._t_busqueda is not None:
                if ahora - self._t_busqueda > 60.0 or neto > 1170.0:
                    self.error = "no se confirmo la bahia del origen tras tres vueltas"
        lado_busqueda = self.lado if self._t_busqueda is not None else 0
        self.paredes, _, self.hueco = self.percepcion.procesar(
            self.scan, ahora, angulo_servo_deg=angulo, lado_parqueo=lado_busqueda)
        if lado_busqueda:
            self._actualizar_ancla_bahia(ahora)

    def _actualizar_ancla_bahia(self, ahora):
        # Al quedar a la altura de un separador se ve su canto de 20 mm,
        # no la cara de 200: exigir los dos simultaneamente rompe ALINEAR.
        # Anclar una pareja CONFIRMADA a los dos muros de su segmento permite
        # actualizarla por medidas, sin integrar PWM ni conservar y obsoleta.
        p = self.paredes
        lateral = p.izquierda if self.lado < 0 else p.derecha
        if p.frontal is None or lateral is None:
            return
        a = math.radians(p.frontal.angulo_deg)
        if abs(p.frontal.angulo_deg) > 25.0:
            return
        h = self.hueco
        if h is not None and abs(h.timestamp - ahora) < 1e-6:
            x = self.lado * h.distancia_lateral_mm
            q_tras = x * math.sin(a) + h.borde_trasero_y_mm * math.cos(a)
            q_del = x * math.sin(a) + h.borde_delantero_y_mm * math.cos(a)
            self._ancla_bahia = (p.frontal.distancia_mm - q_tras,
                                p.frontal.distancia_mm - q_del,
                                lateral.distancia_mm - h.distancia_lateral_mm, h, ahora)
        elif self._ancla_bahia is not None:
            atras, delante, profundidad, previo, visto = self._ancla_bahia
            if ahora - visto > 12.0:
                return
            x = self.lado * (lateral.distancia_mm - profundidad)
            y_atras = (p.frontal.distancia_mm - atras - x * math.sin(a)) / math.cos(a)
            y_delante = (p.frontal.distancia_mm - delante - x * math.sin(a)) / math.cos(a)
            if abs(y_delante) > 1100.0:
                return
            self.hueco = replace(previo, timestamp=ahora,
                                 borde_trasero_y_mm=y_atras,
                                 borde_delantero_y_mm=y_delante,
                                 centro_y_mm=(y_atras + y_delante) / 2.0,
                                 distancia_lateral_mm=abs(x))

    def capturar_origen(self, med, heading, ahora):
        if not hasattr(med, "perfil"):
            return True  # solo compatibilidad con dobles de pruebas
        if self._inicio is None:
            self._inicio = ahora
        if self.error:
            return False
        if ahora - self._inicio > 6.0:
            self.error = "no hay paredes estables para registrar el origen"
            return False
        p = self.paredes
        if p is None or p.frontal is None:
            self._muestras.clear()
            return False
        laterales = [r for r in (p.izquierda, p.derecha) if r is not None]
        if not laterales:
            self._muestras.clear()
            return False
        muestra = (p.frontal.distancia_mm, heading)
        if self._muestras and (abs(muestra[0] - self._muestras[-1][0]) > 60.0
                               or abs(heading - self._muestras[-1][1]) > 3.0):
            self._muestras.clear()
        self._muestras.append(muestra)
        if len(self._muestras) < 5:
            return False
        self._h0 = median(v[1] for v in self._muestras)
        self.origen = {"frontal_mm": median(v[0] for v in self._muestras),
                       "normal_frontal": p.frontal.angulo_deg}
        # Dentro de la bahia el muro exterior es inequivocamente el cercano.
        # En el centro del carril se espera al sentido confirmado de carrera.
        if p.izquierda and p.izquierda.distancia_mm < 250.0:
            self.lado = -1
        elif p.derecha and p.derecha.distancia_mm < 250.0:
            self.lado = 1
        return True

    def listo_para_parquear(self, heading, ahora):
        if self.error or self.origen is None or self._h0 is None:
            return False
        neto = (heading - self._h0) * self.sentido
        p, h = self.paredes, self.hueco
        valido = (self.esquinas == 12 and 1080.0 <= neto <= 1115.0
                  and p is not None and p.frontal is not None and h is not None
                  and 0.0 <= ahora - h.timestamp <= 0.3 and h.confianza >= 0.65
                  and abs(p.frontal.distancia_mm - self.origen["frontal_mm"]) < 900.0
                  and abs(gev.normalizar_180(p.frontal.angulo_deg
                          - self.origen["normal_frontal"])) < 25.0)
        self._confirmaciones = self._confirmaciones + 1 if valido else 0
        return self._confirmaciones >= 3

    def parquear(self, heading, ultrasonido_mm, ahora):
        if self.error:
            return self.control._fallar(self.error, ahora)
        if self.paredes is None or ahora - self.paredes.timestamp > 0.3:
            return self.control._fallar("sin barrido reciente en parqueo", ahora)
        # La trasera extrapolada del LiDAR esta ocluida. Nunca se sustituye
        # la medida US por 8000 mm ni por un supuesto espacio libre.
        us_valido = (ultrasonido_mm is not None and math.isfinite(ultrasonido_mm)
                     and self.control.ultrasonido_min_mm <= ultrasonido_mm
                     <= self.control.ultrasonido_max_mm)
        if not us_valido:
            self._sin_ultrasonido += 1
            if self._sin_ultrasonido >= 3:
                return self.control._fallar("sin ultrasonido trasero reciente", ahora)
            return Consigna(0, 0.0, self.control.estado, "esperando ultrasonido trasero")
        self._sin_ultrasonido = 0
        if self.seguidor is None:
            lateral = self.paredes.izquierda if self.lado < 0 else self.paredes.derecha
            if self.hueco is None or lateral is None or self.paredes.frontal is None:
                return self.control._fallar("bahia sin referencias para iniciar", ahora)
            self.seguidor = SeguidorBahia(self, heading, ahora)
            self.control._t_inicio = ahora
            return Consigna(0, 0.0, "APROXIMAR_BAHIA", "referencia de bahia capturada")
        cmd = self.seguidor.procesar(heading, ultrasonido_mm, ahora)
        # _barrido_seguro solo durante APROXIMAR_BAHIA: dentro de la bahia, el
        # planificador ya verifica colisiones SAT en cada sub-paso y las
        # paredes/delimitadores aparecen como puntos validos en el LiDAR.
        if (cmd.velocidad and self.seguidor.estado == "APROXIMAR_BAHIA"
                and not self._barrido_seguro(cmd.velocidad, cmd.angulo)):
            return self.seguidor.detener_para_replanear()
        return cmd

    def _barrido_seguro(self, velocidad, angulo):
        """Barrido del rectangulo completo durante el siguiente margen de reaccion.

Incluye morro, culata y ancho al tope; el ultrasonido cubre la zona ciega.
Solo rechaza puntos inesperados MUY cerca del cuerpo. Los muros y
delimitadores de la bahia ya fueron verificados por el planificador SAT;
un margen excesivo aqui aborta la maniobra normal.
"""
        puntos = self.percepcion._mascarar(self.scan, 0.0)
        puntos = [geo.lidar_a_eje_trasero(d * math.sin(math.radians(a)),
                                         d * math.cos(math.radians(a))) for a, d in puntos]
        radio = gev.radio_de_comando(angulo)
        semiancho = max(140.0, geo.ANCHO_ROBOT) / 2.0 + 6.0
        delante = geo.LARGO_ROBOT - geo.VOLADIZO_TRASERO + 6.0
        detras = -geo.VOLADIZO_TRASERO - 6.0
        # Un barrido de reaccion mas frenado; el plan completo ya verifica
        # los cambios de volante y marcha. No prolongar el arco 0.45 s al
        # otro lado de un punto donde la ruta exige detenerse.
        distancia = max(16.0, abs(velocidad) * 4.0 * 0.2)
        for i in range(9):
            avance = distancia * i / 8.0 * (1 if velocidad > 0 else -1)
            th = 0.0 if math.isinf(radio) else avance / radio * (1 if angulo > 0 else -1)
            if abs(th) < 1e-6:
                dx, dy = 0.0, avance
            else:
                r = avance / th
                dx, dy = -r * (1.0 - math.cos(th)), r * math.sin(th)
            c, s = math.cos(th), math.sin(th)
            for x, y in puntos:
                xr, yr = (x - dx) * c + (y - dy) * s, -(x - dx) * s + (y - dy) * c
                if abs(xr) < semiancho and detras < yr < delante:
                    return False
        return True

    def instantanea(self):
        return {"esquinas": self.esquinas, "vueltas": self.esquinas // 4,
                "retorno_error": self.error, **self.control.instantanea()}
