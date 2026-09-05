"""Donde esta el robot dentro de la recta, sin SLAM y sin deriva.

LA APUESTA
Un mapa metrico global obligaria a estimar pose con odometria (no hay
encoder), a cerrar bucles y a mantener una nube.  No hace falta: para decidir
por que lado se pasa un pilar y cuando se gira, basta con dos numeros que el
LiDAR mide directo en cada barrido y que NO acumulan error:

* ``avance_mm``   distancia perpendicular al muro que se tiene enfrente.
* ``offset_mm``   distancia perpendicular al muro EXTERIOR de la pista.

El tercero, ``rumbo_error_deg``, sale de la IMU comparada con el rumbo
cardinal del segmento.  La IMU si deriva, pero solo entre esquina y esquina, y
cada esquina la vuelve a anclar.

REDUNDANCIA DELIBERADA
Cada uno de los tres tiene un respaldo geometrico, porque los tres se pierden
de vez en cuando:

* Sin muro frontal: ``largo_recta - trasera``.  El campo mide lo que mide.
* Sin muro exterior: ``ancho_carril - interior``.
* Sin ninguna pared lateral: se propaga el ultimo valor con el rumbo y la
  velocidad, y se marca ``offset_valido = False`` para que el control sepa
  que esta navegando a ciegas y no confie en el como si fuera medido.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from .modelos import MapaParedes, PoseCarril


def _limitar(valor: float, minimo: float, maximo: float) -> float:
    return max(minimo, min(maximo, valor))


def diferencia_angular(actual: float, referencia: float) -> float:
    """Diferencia envuelta a (-180, 180]."""

    return (float(actual) - float(referencia) + 180.0) % 360.0 - 180.0


class Localizador:
    """Mantiene la pose dentro del segmento barrido a barrido."""

    def __init__(self, config: Dict[str, Any]):
        pista = config.get("track", {})
        control = config.get("control", {})

        self.largo_recta_mm = float(pista.get("segment_length_mm", 3000.0))
        self.ancho_carril_mm = float(pista.get("lane_width_mm", 1000.0))
        self.alfa = float(control.get("pose_filter_alpha", 0.55))
        self.salto_max_mm = float(control.get("pose_max_jump_mm", 260.0))
        self.mm_por_pwm = float(control.get("mm_s_per_pwm", 4.0))

        self.segmento = 0
        self.rumbo_cardinal_deg = 0.0
        self._offset_mm = self.ancho_carril_mm / 2.0
        self._avance_mm = self.largo_recta_mm
        self._ultimo_t = 0.0
        self._rechazos_avance = 0
        self._rechazos_offset = 0

    # ------------------------------------------------------------- control

    def reiniciar(self, rumbo_inicial_deg: float = 0.0) -> None:
        self.segmento = 0
        self.rumbo_cardinal_deg = float(rumbo_inicial_deg)
        self._offset_mm = self.ancho_carril_mm / 2.0
        self._avance_mm = self.largo_recta_mm
        self._ultimo_t = 0.0
        self._rechazos_avance = 0
        self._rechazos_offset = 0

    def anotar_esquina(self, sentido: int) -> None:
        """Una esquina terminada: avanza el segmento y ancla el rumbo cardinal.

        ``sentido`` +1 gira a la derecha (horario), -1 a la izquierda.  El
        rumbo de la IMU es positivo hacia la izquierda, asi que un giro a la
        derecha resta 90 grados al cardinal.
        """

        self.segmento = (self.segmento + 1) % 4
        self.rumbo_cardinal_deg -= 90.0 * float(sentido)
        self._avance_mm = self.largo_recta_mm

    # ---------------------------------------------------------- estimacion

    def _avance_medido(self, paredes: MapaParedes) -> Optional[tuple]:
        if paredes.frontal is not None:
            return paredes.frontal.distancia_mm, "FRONTAL"
        if paredes.trasera is not None:
            return self.largo_recta_mm - paredes.trasera.distancia_mm, "TRASERA"
        # NO usar aqui ``frontal_min_mm``.  Se probo en la corrida 8 del 05-09
        # y fue peor: 17 retrocesos frente a 7, y la velocidad media de 24,7 a
        # 10,6 PWM.  La razon es que los avances "clavados en 3000" no eran el
        # fallo sino el SINTOMA: los siete saltos de la corrida 7 cayeron todos
        # entre 0,3 y 0,4 s despues de contar una esquina, con el robot aun
        # cruzado, y lo que hay a 300-800 mm por delante en ese momento no es
        # el muro de la recta nueva.  Darle esa medida al filtro solo hace que
        # adopte antes un valor equivocado.
        return None

    def _offset_medido(self, paredes: MapaParedes, sentido: int) -> Optional[tuple]:
        """Distancia al muro exterior.

        Girando a la derecha (sentido +1) el exterior queda a la izquierda del
        robot; girando a la izquierda, a la derecha.
        """

        if sentido >= 0:
            exterior, interior = paredes.izquierda, paredes.derecha
        else:
            exterior, interior = paredes.derecha, paredes.izquierda

        if exterior is not None:
            return exterior.distancia_mm, "EXTERIOR"
        if interior is not None:
            return self.ancho_carril_mm - interior.distancia_mm, "INTERIOR"
        return None

    def actualizar(
        self,
        paredes: MapaParedes,
        rumbo_imu_deg: float,
        sentido: int,
        velocidad_pwm: float = 0.0,
        timestamp: Optional[float] = None,
    ) -> PoseCarril:
        ahora = float(paredes.timestamp if timestamp is None else timestamp)
        # En el primer ciclo no hay prediccion que valga: los valores semilla
        # son 3000 y medio carril, y compararlos con la medida la rechazaria
        # siempre.  Un valor SEMILLA colandose como medida es exactamente lo
        # que rompio el parqueo del 03-09 (``Der=2000mm`` era el valor de
        # inicializacion, no una lectura), asi que aqui manda la medida.
        primera = self._ultimo_t <= 0.0
        dt = 0.0 if primera else max(0.0, min(0.5, ahora - self._ultimo_t))
        self._ultimo_t = ahora

        rumbo_error = diferencia_angular(rumbo_imu_deg, self.rumbo_cardinal_deg)

        # Propagacion con el modelo de movimiento: sirve de prediccion cuando
        # no hay medida y de referencia para juzgar si la medida es creible.
        avance_delante = math.cos(math.radians(rumbo_error)) * self.mm_por_pwm * velocidad_pwm * dt
        lateral = math.sin(math.radians(rumbo_error)) * self.mm_por_pwm * velocidad_pwm * dt
        prediccion_avance = self._avance_mm - avance_delante
        # rumbo_error positivo = morro hacia la izquierda = el robot se acerca
        # al muro exterior cuando ese muro esta a la izquierda (sentido +1).
        prediccion_offset = self._offset_mm - float(sentido) * lateral

        medido_avance = self._avance_medido(paredes)
        medido_offset = self._offset_medido(paredes, sentido)

        # Cada magnitud lleva su propio contador de rechazos.  Compartirlo era
        # un error sutil: bastaba con que el avance siguiera valido para que el
        # offset no llegara nunca a los tres rechazos y se quedara propagando
        # a ciegas para siempre.
        (
            self._avance_mm,
            avance_valido,
            fuente_avance,
            self._rechazos_avance,
        ) = self._mezclar(
            medido_avance, prediccion_avance, primera, self._rechazos_avance
        )
        (
            self._offset_mm,
            offset_valido,
            fuente_offset,
            self._rechazos_offset,
        ) = self._mezclar(
            medido_offset, prediccion_offset, primera, self._rechazos_offset
        )

        self._avance_mm = _limitar(self._avance_mm, -400.0, self.largo_recta_mm + 400.0)
        self._offset_mm = _limitar(self._offset_mm, -300.0, self.ancho_carril_mm + 300.0)

        return PoseCarril(
            timestamp=ahora,
            segmento=self.segmento,
            avance_mm=self._avance_mm,
            offset_mm=self._offset_mm,
            rumbo_error_deg=rumbo_error,
            avance_valido=avance_valido,
            offset_valido=offset_valido,
            ancho_carril_mm=self.ancho_carril_mm,
            fuente_avance=fuente_avance,
            fuente_offset=fuente_offset,
        )

    def _mezclar(self, medido, prediccion, primera, rechazos):
        """Funde medida y prediccion, o admite que va a ciegas.

        Un rechazo aislado es ruido y se ignora; tres seguidos significan que
        la prediccion es la que esta mal -- el robot choco, alguien lo movio,
        la esquina cambio de recta -- y entonces manda la medida.
        """

        if medido is None:
            return prediccion, False, "PREDICHO", rechazos + 1
        valor, fuente = medido
        creible = abs(valor - prediccion) <= self.salto_max_mm
        if creible:
            return self.alfa * valor + (1.0 - self.alfa) * prediccion, True, fuente, 0
        if primera or rechazos >= 3:
            # Resincronizacion: se ADOPTA la medida entera, no se mezcla.
            # Mezclando, la estimacion se queda a medio camino, el ciclo
            # siguiente vuelve a parecer un salto imposible y el filtro entra
            # en un ciclo de aceptar-rechazar del que no sale.
            return valor, True, fuente, 0
        return prediccion, False, "PREDICHO", rechazos + 1

    # ------------------------------------------------------ marco del mundo

    def offset_de_punto(
        self, x_mm: float, y_mm: float, paredes: MapaParedes, sentido: int
    ) -> Optional[float]:
        """Distancia al muro exterior de un punto cualquiera del marco robot.

        Es la operacion que convierte "un pilar a 250 mm a mi derecha" en "un
        pilar a 650 mm del muro exterior", que es la coordenada con la que el
        mapa recuerda las cosas y la unica que sigue valiendo cuando el robot
        se ha movido.
        """

        exterior = paredes.izquierda if sentido >= 0 else paredes.derecha
        if exterior is not None:
            return exterior.distancia_desde(x_mm, y_mm)
        interior = paredes.derecha if sentido >= 0 else paredes.izquierda
        if interior is not None:
            return self.ancho_carril_mm - interior.distancia_desde(x_mm, y_mm)
        return None

    def avance_de_punto(
        self, x_mm: float, y_mm: float, paredes: MapaParedes
    ) -> Optional[float]:
        """Distancia de un punto al muro frontal del segmento."""

        if paredes.frontal is not None:
            return paredes.frontal.distancia_desde(x_mm, y_mm)
        if paredes.trasera is not None:
            return self.largo_recta_mm - paredes.trasera.distancia_desde(x_mm, y_mm)
        return None
