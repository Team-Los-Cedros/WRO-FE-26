"""Control de ruta puro y ligero para ``ronda_nueva``.

La clase de este modulo no abre puertos, no crea hilos y no depende de
OpenCV. Recibe la geometria ya filtrada y los tracks fusionados, y devuelve
una :class:`~src.pi3B.ronda_nueva.modelos.Consigna` por barrido. Esto permite
usar exactamente la misma FSM en la Pi 3B y en los replays de escritorio.

Convenciones importantes:

* ``sentido`` vale ``+1`` para pista antihoraria/giros a la izquierda y
  ``-1`` para pista horaria/giros a la derecha.
* el servo es positivo hacia la izquierda;
* un pilar verde se pasa por su izquierda y uno rojo por su derecha;
* un timeout siempre termina en ``FAILED``. Nunca sustituye una transicion
  geometrica ni una verificacion de estacionamiento.
"""

import inspect
import math
import time
from typing import Any, Dict, Optional, Sequence

from .estacionamiento import ControlEstacionamiento
from .modelos import (
    Consigna,
    Corredor,
    HuecoParqueo,
    TrackObstaculo,
)


def _limitar(valor: float, minimo: float, maximo: float) -> float:
    return max(minimo, min(maximo, valor))


def _diferencia_angular(actual: float, referencia: float) -> float:
    """Diferencia firmada, incluso si el IMU cruza +/-180 grados."""

    return (float(actual) - float(referencia) + 180.0) % 360.0 - 180.0


def _finito_no_negativo(valor: float) -> bool:
    try:
        return math.isfinite(float(valor)) and float(valor) >= 0.0
    except (TypeError, ValueError):
        return False


def _distancia_lidar_real(valor: float, sin_dato_mm: float) -> bool:
    """Rechaza NaN, cero y el centinela que representa ausencia de eco."""

    return bool(
        _finito_no_negativo(valor)
        and 0.0 < float(valor) < float(sin_dato_mm)
    )


def _cobertura_suficiente(valor: float, minimo: float) -> bool:
    try:
        cobertura = float(valor)
    except (TypeError, ValueError):
        return False
    return bool(math.isfinite(cobertura) and minimo <= cobertura <= 1.0)


class ControlRuta:
    """FSM de carrera, evasion, recuperacion, giros y estacionamiento."""

    ESTADOS = (
        "WAIT_DIRECTION",
        "CRUISE",
        "AVOID_APPROACH",
        "AVOID_PASS",
        "RECENTER",
        "TURN",
        "RECOVERY",
        "FORCED_TURN",
        "PARKING",
        "DONE",
        "FAILED",
    )

    def __init__(self, config: Dict[str, Any]):
        self._config = config
        self._control = config["control"]
        self._parking_cfg = config["parking"]

        self._angulo_izquierda = float(
            self._control["steering_max_left_deg"]
        )
        self._angulo_derecha = float(
            self._control["steering_max_right_deg"]
        )
        self._estacionamiento = ControlEstacionamiento(config)
        # La FSM de estacionamiento puede evolucionar sin romper este modulo.
        # Se inspecciona una sola vez (no en el bucle de control de la Pi).
        self._parametros_estacionamiento = set(
            inspect.signature(self._estacionamiento.procesar).parameters
        )
        self.reiniciar()

    def reiniciar(self) -> None:
        self._estado = "WAIT_DIRECTION"
        self._esquinas = 0
        direccion = str(self._control["turn_direction"]).strip().upper()
        self._sentido = 1 if direccion == "LEFT" else -1 if direccion == "RIGHT" else 0
        self._lado_parqueo_solicitado = 0
        self._t_inicio: Optional[float] = None
        self._t_estado: Optional[float] = None

        self._error_lateral_filtrado: Optional[float] = None
        self._error_rumbo_filtrado: Optional[float] = None
        self._heading_actual = 0.0
        self._heading_referencia: Optional[float] = None
        self._confirmaciones_esquina = 0
        self._confirmaciones_salida_esquina = 0
        self._t_ultima_esquina = -math.inf
        self._heading_inicio_giro = 0.0
        self._kturn_fase = "AVANCE"
        self._kturn_tramos = 0
        self._t_kturn_fase: Optional[float] = None

        self._track_id: Optional[int] = None
        self._track_color: Optional[str] = None
        self._ultimo_track: Optional[TrackObstaculo] = None
        self._track_observado = False
        self._heading_sobrepaso = 0.0
        self._distancia_sobrepaso_mm = 0.0
        self._t_ultimo_sobrepaso: Optional[float] = None
        self._confirmaciones_recentrado = 0

        self._recuperaciones = []
        self._forzar_al_salir = False
        self._heading_inicio_forzado = 0.0
        self._signo_giro_forzado = 0

        self._ultimo_angulo = 0.0
        self._ultima_velocidad = 0
        self._razon_terminal = ""
        self._estacionamiento.reiniciar()

    @property
    def estado(self) -> str:
        return self._estado

    @property
    def esquinas(self) -> int:
        return self._esquinas

    @property
    def sentido(self) -> int:
        return self._sentido

    @property
    def lado_parqueo_solicitado(self) -> int:
        """Lado pedido a percepcion: 0 hasta completar las esquinas."""

        return self._lado_parqueo_solicitado

    @property
    def track_activo(self) -> Optional[TrackObstaculo]:
        """Ultima observacion del pilar unido a la maniobra actual."""

        return self._ultimo_track

    @property
    def track_activo_id(self) -> Optional[int]:
        return self._track_id

    @property
    def track_activo_color(self) -> Optional[str]:
        return self._track_color

    @property
    def track_activo_observado(self) -> bool:
        """Indica si el track activo fue observado en el ciclo actual."""

        return self._track_observado

    @property
    def distancia_sobrepaso_mm(self) -> float:
        return self._distancia_sobrepaso_mm

    def _entrar(self, estado: str, ahora: float) -> None:
        if estado not in self.ESTADOS:
            raise ValueError("estado desconocido: " + estado)
        self._estado = estado
        self._t_estado = float(ahora)

    def _tiempo_estado(self, ahora: float) -> float:
        if self._t_estado is None:
            return 0.0
        return max(0.0, float(ahora) - self._t_estado)

    def _acotar_angulo(self, angulo: float) -> float:
        return _limitar(
            float(angulo), self._angulo_derecha, self._angulo_izquierda
        )

    def _emitir(
        self,
        velocidad: float,
        angulo: float,
        razon: str = "",
        terminado: bool = False,
        verificado: bool = False,
        detener_inmediato: bool = False,
        direccion_neutra: bool = False,
        slew_angulo_deg: Optional[float] = None,
    ) -> Consigna:
        """Aplica limites fisicos y slew en el unico punto de salida."""

        if terminado or detener_inmediato:
            velocidad_salida = 0
        else:
            deseada = int(round(float(velocidad)))
            paso_vel = max(1, int(self._control["speed_slew_pwm_per_scan"]))
            # Nunca conservar avance durante una solicitud de reversa (ni al
            # reves): se intercala un ciclo a cero antes de cambiar el signo.
            if deseada * self._ultima_velocidad < 0:
                velocidad_salida = 0
            else:
                delta_vel = int(
                    _limitar(
                        deseada - self._ultima_velocidad, -paso_vel, paso_vel
                    )
                )
                velocidad_salida = self._ultima_velocidad + delta_vel

        if terminado or direccion_neutra:
            angulo_salida = 0.0
        else:
            deseado = self._acotar_angulo(angulo)
            paso_ang = max(
                0.1,
                float(
                    self._control["steering_slew_deg_per_scan"]
                    if slew_angulo_deg is None
                    else slew_angulo_deg
                ),
            )
            angulo_salida = self._acotar_angulo(
                self._ultimo_angulo
                + _limitar(deseado - self._ultimo_angulo, -paso_ang, paso_ang)
            )

        self._ultima_velocidad = int(velocidad_salida)
        self._ultimo_angulo = float(angulo_salida)
        return Consigna(
            velocidad=int(velocidad_salida),
            angulo=float(angulo_salida),
            estado=self._estado,
            razon=razon,
            terminado=bool(terminado),
            verificado=bool(verificado),
        )

    def _fallar(self, razon: str, ahora: float) -> Consigna:
        self._razon_terminal = razon
        self._entrar("FAILED", ahora)
        return self._emitir(
            0,
            0.0,
            razon,
            terminado=True,
            verificado=False,
            detener_inmediato=True,
            direccion_neutra=True,
        )

    def _resolver_sentido(self, color_piso: Optional[str]) -> bool:
        configurado = str(self._control["turn_direction"]).strip().upper()
        if configurado == "LEFT":
            self._sentido = 1
            return True
        if configurado == "RIGHT":
            self._sentido = -1
            return True

        color = str(color_piso or "").strip().upper()
        if color == str(self._control["floor_color_left"]).strip().upper():
            self._sentido = 1
        elif color == str(self._control["floor_color_right"]).strip().upper():
            self._sentido = -1
        return self._sentido != 0

    def _rumbo_ideal_carril(self) -> Optional[float]:
        # El rumbo del tramo actual es la referencia tomada al arrancar mas
        # 90 grados por esquina contada, con el signo del sentido de vuelta.
        # No depende del ajuste de pared, solo de la IMU y del conteo.
        # No hace falta envolver: _diferencia_angular normaliza cualquier
        # magnitud al comparar contra el heading actual.
        if self._heading_referencia is None or self._sentido == 0:
            return None
        return self._heading_referencia + 90.0 * self._esquinas * self._sentido

    def _angulo_rumbo_carril(self) -> float:
        rumbo = self._rumbo_ideal_carril()
        if rumbo is None:
            return 0.0
        error = _diferencia_angular(rumbo, self._heading_actual)
        kp = float(self._control.get("heading_fallback_kp", 0.5))
        return self._acotar_angulo(error * kp)

    def _angulo_pared(self, corredor: Corredor) -> float:
        """P filtrado por pared; sin pared confiable, endereza por rumbo."""

        calidad_minima = float(self._control.get("wall_min_quality", 0.30))
        if (
            not math.isfinite(float(corredor.calidad_pared))
            or corredor.calidad_pared < calidad_minima
            or not math.isfinite(float(corredor.error_lateral_mm))
            or not math.isfinite(float(corredor.error_rumbo_muro_deg))
        ):
            # Un 0.0 fijo aqui congela el timon con el robot cruzado: en la
            # corrida 20260831_120547 el ajuste de pared perdio calidad tras
            # el sobrepaso (heading +33 grados) y el robot derivo en diagonal
            # hasta el timeout de RECENTER. Volver al rumbo del carril
            # endereza la vista del LiDAR y recupera el propio ajuste.
            return self._angulo_rumbo_carril()

        alfa = _limitar(float(self._control["wall_filter_alpha"]), 0.0, 1.0)
        if self._error_lateral_filtrado is None:
            self._error_lateral_filtrado = float(corredor.error_lateral_mm)
            self._error_rumbo_filtrado = float(corredor.error_rumbo_muro_deg)
        else:
            self._error_lateral_filtrado += alfa * (
                float(corredor.error_lateral_mm) - self._error_lateral_filtrado
            )
            assert self._error_rumbo_filtrado is not None
            self._error_rumbo_filtrado += alfa * (
                float(corredor.error_rumbo_muro_deg)
                - self._error_rumbo_filtrado
            )

        angulo = (
            self._error_lateral_filtrado * float(self._control["wall_kp"])
            - self._error_rumbo_filtrado
            * float(self._control["wall_heading_kp"])
        )
        return self._acotar_angulo(angulo)

    def _con_guardia_pared(self, deseado: float, corredor: Corredor) -> float:
        """Da autoridad progresiva a la pared mas cercana durante evasion."""

        izquierda = float(corredor.izquierda_mm)
        derecha = float(corredor.derecha_mm)
        if not (_finito_no_negativo(izquierda) and _finito_no_negativo(derecha)):
            return self._acotar_angulo(deseado)

        cercana = min(izquierda, derecha)
        inicio = float(self._control["wall_guard_start_mm"])
        completa = float(self._control["wall_guard_full_mm"])
        if cercana >= inicio:
            return self._acotar_angulo(deseado)
        peso = _limitar((inicio - cercana) / max(inicio - completa, 1.0), 0.0, 1.0)

        # El termino lateral de este protector empuja al robot hacia el pilar
        # que esquiva (la pared llego a pesar 0,94 del mando en la corrida
        # 145857). Quitarlo mientras hay un pilar activo se probo en pista y
        # sale peor: sin el, el robot llega al recentrado pegado a la pared y
        # agota su timeout antes de la primera esquina (corrida 152111, cero
        # esquinas frente a las dos de la 151037 desde la misma salida). El
        # termino lateral tambien es lo que impide pegarse, asi que se
        # conserva; el desvio se corrige antes, en el punto de paso.
        if corredor.calidad_pared >= float(
            self._control.get("wall_min_quality", 0.30)
        ):
            protector = self._angulo_pared(corredor)
        elif izquierda < derecha:
            protector = self._angulo_derecha
        elif derecha < izquierda:
            protector = self._angulo_izquierda
        else:
            protector = 0.0
        return self._acotar_angulo(deseado * (1.0 - peso) + protector * peso)

    def _con_frenado(self, velocidad_base: int, frontal_mm: float) -> int:
        inicio = float(self._control["brake_start_mm"])
        completa = float(self._control["brake_full_mm"])
        minima = abs(int(self._control["speed_min_pwm"]))
        base = abs(int(velocidad_base))
        if not _finito_no_negativo(frontal_mm):
            return 0
        if frontal_mm >= inicio:
            return base
        if frontal_mm <= completa:
            return min(minima, base)
        proporcion = (frontal_mm - completa) / max(inicio - completa, 1.0)
        return int(round(minima + proporcion * (base - minima)))

    def _timeout_evasion(self) -> float:
        """Escala la red temporal con la velocidad configurada de evasion."""

        base = float(self._control["obstacle_timeout_s"])
        referencia = max(
            1.0,
            abs(float(self._control.get("obstacle_timeout_reference_pwm", 40.0))),
        )
        velocidad = max(
            1.0, abs(float(self._control["speed_avoid_pwm"]))
        )
        return base * referencia / velocidad

    def _timeout_recentrado(self) -> float:
        """Conserva la misma distancia disponible al reducir el PWM."""

        base = float(self._control["recenter_timeout_s"])
        referencia = max(
            1.0,
            abs(float(self._control.get("obstacle_timeout_reference_pwm", 40.0))),
        )
        velocidad = max(
            1.0, abs(float(self._control["speed_avoid_pwm"]))
        )
        return base * referencia / velocidad

    def _hay_emergencia(self, corredor: Corredor) -> bool:
        return bool(
            corredor.frontal_mm < float(self._control["emergency_front_mm"])
            or corredor.izquierda_mm < float(self._control["emergency_side_mm"])
            or corredor.derecha_mm < float(self._control["emergency_side_mm"])
        )

    def _iniciar_recuperacion(self, ahora: float) -> None:
        ventana = float(self._control["forced_turn_window_s"])
        self._recuperaciones = [
            instante
            for instante in self._recuperaciones
            if ahora - instante <= ventana
        ]
        self._recuperaciones.append(ahora)
        self._forzar_al_salir = len(self._recuperaciones) >= int(
            self._control["forced_turn_after_recoveries"]
        )
        self._track_id = None
        self._track_color = None
        self._ultimo_track = None
        self._track_observado = False
        self._confirmaciones_recentrado = 0
        self._entrar("RECOVERY", ahora)

    def _procesar_recuperacion(
        self, corredor: Corredor, ahora: float
    ) -> Consigna:
        if self._tiempo_estado(ahora) > float(
            self._control["recovery_timeout_s"]
        ):
            return self._fallar("timeout de recuperacion", ahora)

        despejado = (
            corredor.frontal_mm
            > float(self._control["recovery_exit_front_mm"])
            and corredor.izquierda_mm
            > float(self._control["recovery_exit_side_mm"])
            and corredor.derecha_mm
            > float(self._control["recovery_exit_side_mm"])
        )
        if (
            self._tiempo_estado(ahora)
            >= float(self._control["recovery_min_s"])
            and despejado
        ):
            if self._forzar_al_salir:
                self._signo_giro_forzado = self._sentido
                self._heading_inicio_forzado = self._heading_actual
                self._entrar("FORCED_TURN", ahora)
                return self._procesar_giro_forzado(corredor, ahora)
            self._entrar("CRUISE", ahora)
            return self._emitir(
                self._con_frenado(
                    int(self._control["speed_cruise_pwm"]), corredor.frontal_mm
                ),
                self._angulo_pared(corredor),
                "recuperacion despejada",
            )

        trasera_segura = (
            bool(corredor.trasera_valida)
            and _finito_no_negativo(corredor.trasera_mm)
            and corredor.trasera_mm
            > float(self._control["emergency_rear_mm"])
        )
        if not trasera_segura:
            return self._emitir(
                0,
                0.0,
                "reversa bloqueada: trasera ciega o sin holgura",
                detener_inmediato=True,
                direccion_neutra=True,
            )

        diagonales_validas = (
            bool(corredor.trasera_izquierda_valida)
            and bool(corredor.trasera_derecha_valida)
            and _finito_no_negativo(corredor.trasera_izquierda_mm)
            and _finito_no_negativo(corredor.trasera_derecha_mm)
        )
        if diagonales_validas:
            error = (
                corredor.trasera_derecha_mm
                - corredor.trasera_izquierda_mm
            )
            kp_reversa = float(self._control.get("recovery_steering_kp", 0.05))
            angulo = self._acotar_angulo(error * kp_reversa)
            neutra = False
        else:
            angulo = 0.0
            neutra = True
        return self._emitir(
            int(self._control["speed_reverse_pwm"]),
            angulo,
            "retroceso corto de emergencia",
            direccion_neutra=neutra,
        )

    def _procesar_giro_forzado(
        self, corredor: Corredor, ahora: float
    ) -> Consigna:
        if self._tiempo_estado(ahora) > float(
            self._control.get("forced_turn_timeout_s", self._control["corner_timeout_s"])
        ):
            return self._fallar("timeout de giro forzado", ahora)

        delta = (
            _diferencia_angular(self._heading_actual, self._heading_inicio_forzado)
            * self._signo_giro_forzado
        )
        if (
            delta >= float(self._control["corner_min_heading_deg"])
            and corredor.frontal_muro_mm
            >= float(self._control["corner_front_exit_mm"])
        ):
            self._recuperaciones = []
            self._forzar_al_salir = False
            self._entrar("CRUISE", ahora)
            return self._emitir(
                self._con_frenado(
                    int(self._control["speed_cruise_pwm"]), corredor.frontal_mm
                ),
                self._angulo_pared(corredor),
                "giro forzado libero la esquina",
            )

        angulo = (
            self._angulo_izquierda
            if self._signo_giro_forzado > 0
            else self._angulo_derecha
        )
        return self._emitir(
            self._con_frenado(
                int(self._control["speed_turn_pwm"]), corredor.frontal_mm
            ),
            angulo,
            "giro comprometido hacia el sentido de pista",
            slew_angulo_deg=float(
                self._control.get(
                    "corner_steering_slew_deg_per_scan",
                    self._control["steering_slew_deg_per_scan"],
                )
            ),
        )

    @staticmethod
    def _normalizar_color_pilar(color: Optional[str]) -> Optional[str]:
        valor = str(color or "").strip().upper()
        if valor in ("VERDE", "GREEN"):
            return "VERDE"
        if valor in ("ROJO", "RED"):
            return "ROJO"
        return None

    def _seleccionar_track(
        self, tracks: Sequence[TrackObstaculo]
    ) -> Optional[TrackObstaculo]:
        candidatos = [
            track
            for track in tracks
            if track.confirmado
            and self._normalizar_color_pilar(track.color) is not None
            and track.y_mm > 50.0
            and track.distancia_mm
            <= float(self._control["obstacle_trigger_mm"])
        ]
        return min(
            candidatos,
            key=lambda track: (track.distancia_mm, track.track_id),
            default=None,
        )

    def _track_bloqueado(
        self, tracks: Sequence[TrackObstaculo]
    ) -> Optional[TrackObstaculo]:
        edad_maxima = float(
            self._control.get("obstacle_track_max_age_s", 0.45)
        )
        for track in tracks:
            if track.track_id == self._track_id and track.edad_s <= edad_maxima:
                self._ultimo_track = track
                self._track_observado = True
                return track

        # La fusion puede regenerar el ID si el poste se oculta algunos
        # barridos durante una rotacion. Se permite reasociar solo un track
        # confirmado, del mismo color y geometricamente cercano a la ultima
        # observacion; asi no se salta a otro poste del mismo color.
        if self._ultimo_track is not None and self._track_color is not None:
            puerta = float(
                self._control.get("obstacle_relock_gate_mm", 360.0)
            )
            candidatos = []
            for track in tracks:
                if (
                    track.confirmado
                    and track.edad_s <= edad_maxima
                    and self._normalizar_color_pilar(track.color)
                    == self._track_color
                ):
                    distancia = math.hypot(
                        track.x_mm - self._ultimo_track.x_mm,
                        track.y_mm - self._ultimo_track.y_mm,
                    )
                    if distancia <= puerta:
                        candidatos.append((distancia, track.track_id, track))
            if candidatos:
                _, nuevo_id, track = min(candidatos)
                self._track_id = nuevo_id
                self._ultimo_track = track
                self._track_observado = True
                return track

        self._track_observado = False
        return None

    def _procesar_aproximacion(
        self,
        corredor: Corredor,
        tracks: Sequence[TrackObstaculo],
        ahora: float,
    ) -> Consigna:
        if self._tiempo_estado(ahora) > self._timeout_evasion():
            return self._fallar("timeout aproximandose al pilar", ahora)

        track = self._track_bloqueado(tracks)
        observado = track is not None
        if track is None:
            return self._emitir(
                0,
                0.0,
                "track bloqueado perdido; parada para reasociar",
                detener_inmediato=True,
                direccion_neutra=True,
            )

        if observado and track.y_mm <= float(
            self._control["obstacle_pass_y_mm"]
        ):
            self._heading_sobrepaso = self._heading_actual
            self._distancia_sobrepaso_mm = 0.0
            self._t_ultimo_sobrepaso = ahora
            self._entrar("AVOID_PASS", ahora)
            return self._procesar_sobrepaso(corredor, tracks, ahora)

        # Verde: robot a la izquierda del pilar (x objetivo negativo).
        # Rojo: robot a la derecha del pilar (x objetivo positivo).
        separacion = float(self._control["obstacle_lateral_clearance_mm"])
        desplazamiento = -separacion if self._track_color == "VERDE" else separacion
        x_objetivo = float(track.x_mm) + desplazamiento
        y_objetivo = max(150.0, float(track.y_mm))
        bearing = math.degrees(math.atan2(x_objetivo, y_objetivo))
        deseado = -bearing * float(self._control["obstacle_pursuit_kp"])
        deseado = self._con_guardia_pared(deseado, corredor)
        return self._emitir(
            self._con_frenado(
                int(self._control["speed_avoid_pwm"]), corredor.frontal_mm
            ),
            deseado,
            "pure-pursuit al punto de paso del pilar {}".format(
                self._track_id
            ),
        )

    def _procesar_sobrepaso(
        self,
        corredor: Corredor,
        tracks: Sequence[TrackObstaculo],
        ahora: float,
    ) -> Consigna:
        if self._tiempo_estado(ahora) > self._timeout_evasion():
            return self._fallar("timeout sobrepasando el pilar", ahora)

        if self._t_ultimo_sobrepaso is None:
            self._t_ultimo_sobrepaso = ahora
        dt = max(0.0, ahora - self._t_ultimo_sobrepaso)
        self._t_ultimo_sobrepaso = ahora
        mm_s_por_pwm = float(self._config["fusion"]["mm_s_per_pwm"])
        self._distancia_sobrepaso_mm += (
            max(0, int(self._ultima_velocidad)) * mm_s_por_pwm * dt
        )

        track = self._track_bloqueado(tracks)
        superado_por_lidar = bool(
            track is not None
            and track.y_mm <= float(self._control["obstacle_cleared_y_mm"])
        )
        superado_por_distancia = (
            self._distancia_sobrepaso_mm
            >= float(self._control.get("obstacle_pass_distance_mm", 200.0))
        )
        if superado_por_lidar or superado_por_distancia:
            self._confirmaciones_recentrado = 0
            self._entrar("RECENTER", ahora)
            orden = self._procesar_recentrado(corredor, ahora)
            if superado_por_distancia and not superado_por_lidar:
                orden = Consigna(
                    velocidad=orden.velocidad,
                    angulo=orden.angulo,
                    estado=orden.estado,
                    razon="reincorporacion tras {:.0f} mm de sobrepaso estimado".format(
                        self._distancia_sobrepaso_mm
                    ),
                    terminado=orden.terminado,
                    verificado=orden.verificado,
                )
            return orden

        error_heading = _diferencia_angular(
            self._heading_sobrepaso, self._heading_actual
        )
        deseado = error_heading * float(self._control["obstacle_pursuit_kp"])
        deseado = self._con_guardia_pared(deseado, corredor)
        return self._emitir(
            self._con_frenado(
                int(self._control["speed_avoid_pwm"]), corredor.frontal_mm
            ),
            deseado,
            "manteniendo rumbo mientras pasa el pilar {}".format(
                self._track_id
            ),
        )

    def _procesar_recentrado(
        self, corredor: Corredor, ahora: float
    ) -> Consigna:
        # En la pista real el tramo disponible entre un pilar cercano a la
        # esquina y el inicio del giro puede ser menor que la distancia
        # necesaria para obtener un centrado perfecto. La pared frontal es
        # geometria independiente del track: si queda confirmada durante la
        # reincorporacion, se entrega el mando a TURN en vez de prolongar una
        # correccion lateral hasta su timeout.
        fuera_refractario = (
            ahora - self._t_ultima_esquina
            >= float(self._control["corner_refractory_s"])
        )
        frente_de_esquina = (
            _finito_no_negativo(corredor.frontal_muro_mm)
            and corredor.frontal_muro_mm
            <= float(
                self._control.get(
                    "recenter_corner_handoff_mm",
                    self._control["corner_front_exit_mm"],
                )
            )
        )
        if fuera_refractario and frente_de_esquina:
            self._confirmaciones_esquina += 1
        else:
            self._confirmaciones_esquina = 0

        if self._confirmaciones_esquina >= int(
            self._control["corner_confirm_scans"]
        ):
            self._track_id = None
            self._track_color = None
            self._ultimo_track = None
            self._track_observado = False
            self._confirmaciones_recentrado = 0
            self._heading_inicio_giro = self._heading_actual
            self._confirmaciones_salida_esquina = 0
            self._kturn_fase = "AVANCE"
            self._kturn_tramos = 0
            self._t_kturn_fase = ahora
            self._entrar("TURN", ahora)
            return self._procesar_giro(corredor, ahora)

        if self._tiempo_estado(ahora) > self._timeout_recentrado():
            return self._fallar("timeout de reincorporacion", ahora)

        # Estar centrado es una afirmacion sobre distancias medidas, no
        # sobre lo bien que se ajusto la recta de la pared. Exigir la
        # calidad aqui hacia inconfirmable el recentrado justo cuando el
        # ajuste se degrada tras rebasar un pilar: en las corridas 152111 y
        # 152413 el robot llego a 108 mm de error, dentro de la tolerancia
        # de 150, y aun asi agoto el timeout con la calidad en 0,21. La
        # calidad sigue gobernando el mando en _angulo_pared, que es donde
        # importa; para juzgar el centrado basta con laterales validas.
        laterales_ok = (
            bool(corredor.izquierda_valida)
            and bool(corredor.derecha_valida)
            and _finito_no_negativo(corredor.izquierda_mm)
            and _finito_no_negativo(corredor.derecha_mm)
            and math.isfinite(float(corredor.error_lateral_mm))
        )
        centrado = laterales_ok and abs(corredor.error_lateral_mm) <= float(
            self._control["recenter_tolerance_mm"]
        )
        self._confirmaciones_recentrado = (
            self._confirmaciones_recentrado + 1 if centrado else 0
        )
        if self._confirmaciones_recentrado >= int(
            self._control["recenter_confirm_scans"]
        ):
            self._track_id = None
            self._track_color = None
            self._ultimo_track = None
            self._track_observado = False
            self._confirmaciones_esquina = 0
            self._entrar("CRUISE", ahora)
            return self._emitir(
                self._con_frenado(
                    int(self._control["speed_cruise_pwm"]), corredor.frontal_mm
                ),
                self._angulo_pared(corredor),
                "reincorporacion verificada",
            )

        return self._emitir(
            self._con_frenado(
                int(self._control["speed_avoid_pwm"]), corredor.frontal_mm
            ),
            self._angulo_pared(corredor),
            "reincorporando por posicion lateral",
        )

    def _iniciar_parqueo(self, ahora: float) -> Consigna:
        self._lado_parqueo_solicitado = int(
            self._parking_cfg[
                "parking_side_for_left_turn"
                if self._sentido > 0
                else "parking_side_for_right_turn"
            ]
        )
        self._estacionamiento.reiniciar()
        self._entrar("PARKING", ahora)
        # Se espera un barrido nuevo procesado con el lado solicitado; el
        # hueco recibido en este ciclo se calculo todavia con lado 0.
        return self._emitir(
            0,
            0.0,
            "busqueda de parqueo habilitada",
            detener_inmediato=True,
            direccion_neutra=True,
        )

    def _procesar_giro(self, corredor: Corredor, ahora: float) -> Consigna:
        if self._tiempo_estado(ahora) > float(
            self._control["corner_timeout_s"]
        ):
            return self._fallar("timeout completando esquina", ahora)

        delta = (
            _diferencia_angular(self._heading_actual, self._heading_inicio_giro)
            * self._sentido
        )
        salida = (
            delta >= float(self._control["corner_min_heading_deg"])
            and corredor.frontal_muro_mm
            >= float(self._control["corner_front_exit_mm"])
        )
        self._confirmaciones_salida_esquina = (
            self._confirmaciones_salida_esquina + 1 if salida else 0
        )
        if self._confirmaciones_salida_esquina >= int(
            self._control["corner_confirm_scans"]
        ):
            self._esquinas += 1
            self._t_ultima_esquina = ahora
            self._confirmaciones_esquina = 0
            self._confirmaciones_salida_esquina = 0
            if self._esquinas >= int(
                self._control["corners_before_parking"]
            ):
                return self._iniciar_parqueo(ahora)
            self._entrar("CRUISE", ahora)
            return self._emitir(
                self._con_frenado(
                    int(self._control["speed_cruise_pwm"]), corredor.frontal_mm
                ),
                self._angulo_pared(corredor),
                "esquina {} verificada".format(self._esquinas),
            )

        if not bool(self._control.get("corner_kturn_enabled", False)):
            return self._giro_avanzando(corredor)
        if self._kturn_fase == "REVERSA":
            return self._giro_retrocediendo(corredor, ahora)
        return self._giro_avanzando_con_kturn(corredor, ahora)

    def _slew_esquina(self) -> float:
        return float(
            self._control.get(
                "corner_steering_slew_deg_per_scan",
                self._control["steering_slew_deg_per_scan"],
            )
        )

    def _angulo_giro_avance(self) -> float:
        # Avanzando, la rueda apunta hacia el lado al que se quiere rotar.
        return (
            self._angulo_izquierda if self._sentido > 0 else self._angulo_derecha
        )

    def _angulo_giro_reversa(self) -> float:
        # Retrocediendo, el signo se invierte: con Ackermann la rotacion es
        # omega = v*tan(delta)/L, asi que con v negativa hace falta delta del
        # signo contrario para que el morro siga rotando hacia el mismo lado.
        return (
            self._angulo_derecha if self._sentido > 0 else self._angulo_izquierda
        )

    def _giro_avanzando(self, corredor: Corredor) -> Consigna:
        return self._emitir(
            self._con_frenado(
                int(self._control["speed_turn_pwm"]), corredor.frontal_mm
            ),
            self._angulo_giro_avance(),
            "giro de esquina por IMU y reapertura frontal",
            slew_angulo_deg=self._slew_esquina(),
        )

    def _trasera_medida(self, corredor: Corredor) -> bool:
        return bool(corredor.trasera_valida) and _finito_no_negativo(
            corredor.trasera_mm
        )

    def _reversa_de_esquina_segura(self, corredor: Corredor) -> bool:
        """Holgura de confort para *iniciar* un tramo de reversa."""

        return (
            self._trasera_medida(corredor)
            and corredor.trasera_mm
            > float(self._control["corner_kturn_rear_mm"])
        )

    def _reversa_de_esquina_critica(self, corredor: Corredor) -> bool:
        """Limite duro que corta un tramo ya empezado."""

        return not self._trasera_medida(corredor) or corredor.trasera_mm <= float(
            self._control["emergency_rear_mm"]
        )

    def _tiempo_kturn(self, ahora: float) -> float:
        if self._t_kturn_fase is None:
            return math.inf
        return max(0.0, float(ahora) - self._t_kturn_fase)

    def _cambiar_fase_kturn(self, fase: str, ahora: float) -> None:
        self._kturn_fase = fase
        self._t_kturn_fase = float(ahora)

    def _tramo_kturn_maduro(self, ahora: float) -> bool:
        return self._tiempo_kturn(ahora) >= float(
            self._control["corner_kturn_min_tramo_s"]
        )

    def _giro_avanzando_con_kturn(
        self, corredor: Corredor, ahora: float
    ) -> Consigna:
        # El radio del chasis (~600 mm medidos) no cierra una esquina de
        # 1000 mm de carril. Cuando el avance se queda sin frente, la
        # maniobra en tres tiempos gana angulo retrocediendo en vez de
        # seguir empujando contra la pared hasta el timeout.
        sin_frente = (
            _finito_no_negativo(corredor.frontal_mm)
            and corredor.frontal_mm
            <= float(self._control["corner_kturn_front_mm"])
        )
        quedan_tramos = self._kturn_tramos < int(
            self._control["corner_kturn_max_tramos"]
        )
        # El primer tramo entra sin demora: esperar con el morro a 200 mm de
        # la pared solo acerca la emergencia. La madurez amortigua los
        # rebotes posteriores, que son los que oscilaban.
        listo = self._kturn_tramos == 0 or self._tramo_kturn_maduro(ahora)
        if sin_frente and quedan_tramos and listo:
            if self._reversa_de_esquina_segura(corredor):
                self._cambiar_fase_kturn("REVERSA", ahora)
                self._kturn_tramos += 1
                return self._giro_retrocediendo(corredor, ahora)
            return self._emitir(
                0,
                self._angulo_giro_avance(),
                "esquina sin frente y sin trasera fiable para el tramo {}".format(
                    self._kturn_tramos + 1
                ),
                detener_inmediato=True,
                slew_angulo_deg=self._slew_esquina(),
            )
        return self._giro_avanzando(corredor)

    def _giro_retrocediendo(
        self, corredor: Corredor, ahora: float
    ) -> Consigna:
        frente_recuperado = (
            _finito_no_negativo(corredor.frontal_mm)
            and corredor.frontal_mm
            >= float(self._control["corner_kturn_resume_mm"])
        )
        # En la esquina el sector trasero cruza la arista entre dos paredes y
        # la lectura salta entre dos valores reales (medidos: 258 y 690 mm,
        # ambos con cobertura plena). Comparar cada ciclo contra la holgura de
        # confort hacia oscilar la maniobra a 5 Hz sin desplazar el robot, asi
        # que un tramo empezado solo lo corta el limite duro o su madurez.
        agotado = self._tramo_kturn_maduro(ahora) and not (
            self._reversa_de_esquina_segura(corredor)
        )
        if (
            frente_recuperado
            or agotado
            or self._reversa_de_esquina_critica(corredor)
        ):
            self._cambiar_fase_kturn("AVANCE", ahora)
            return self._giro_avanzando(corredor)
        return self._emitir(
            int(self._control["speed_reverse_pwm"]),
            self._angulo_giro_reversa(),
            "tramo {} en reversa para cerrar la esquina".format(
                self._kturn_tramos
            ),
            slew_angulo_deg=self._slew_esquina(),
        )

    def _procesar_crucero(
        self,
        corredor: Corredor,
        tracks: Sequence[TrackObstaculo],
        ahora: float,
    ) -> Consigna:
        fuera_refractario = (
            ahora - self._t_ultima_esquina
            >= float(self._control["corner_refractory_s"])
        )
        frente_cerrado = (
            _finito_no_negativo(corredor.frontal_muro_mm)
            and corredor.frontal_muro_mm
            <= float(self._control["corner_front_trigger_mm"])
        )
        if fuera_refractario and frente_cerrado:
            self._confirmaciones_esquina += 1
        else:
            self._confirmaciones_esquina = 0

        if self._confirmaciones_esquina >= int(
            self._control["corner_confirm_scans"]
        ):
            self._heading_inicio_giro = self._heading_actual
            self._confirmaciones_salida_esquina = 0
            self._kturn_fase = "AVANCE"
            self._kturn_tramos = 0
            self._t_kturn_fase = ahora
            self._entrar("TURN", ahora)
            return self._procesar_giro(corredor, ahora)

        # En la entrada de una esquina, un pilar puede tapar parcialmente el
        # frente y parecer el candidato mas urgente. Mientras la geometria de
        # muro confirma el cierre, no se inicia una evasion que apunte hacia
        # la isla central. El giro de esquina tiene prioridad y exige las
        # mismas confirmaciones consecutivas que antes.
        if not (fuera_refractario and frente_cerrado):
            track = self._seleccionar_track(tracks)
            if track is not None:
                self._track_id = track.track_id
                self._track_color = self._normalizar_color_pilar(track.color)
                self._ultimo_track = track
                self._track_observado = True
                self._entrar("AVOID_APPROACH", ahora)
                return self._procesar_aproximacion(corredor, tracks, ahora)

        return self._emitir(
            self._con_frenado(
                int(self._control["speed_cruise_pwm"]), corredor.frontal_mm
            ),
            self._angulo_pared(corredor),
            "crucero centrado por paredes",
        )

    def _procesar_estacionamiento(
        self,
        corredor: Corredor,
        hueco: Optional[HuecoParqueo],
        ahora: float,
    ) -> Consigna:
        if self._tiempo_estado(ahora) > float(
            self._parking_cfg["total_timeout_s"]
        ):
            return self._fallar("timeout total de estacionamiento", ahora)

        if hueco is not None and hueco.lado != self._lado_parqueo_solicitado:
            hueco = None
        lateral = (
            corredor.derecha_mm
            if self._lado_parqueo_solicitado > 0
            else corredor.izquierda_mm
        )
        bandera_lateral = (
            corredor.derecha_valida
            if self._lado_parqueo_solicitado > 0
            else corredor.izquierda_valida
        )
        sin_dato = float(
            self._config.get("lidar", {}).get("rear_no_data_mm", 8000.0)
        )
        lateral_valida = bool(
            bandera_lateral and _distancia_lidar_real(lateral, sin_dato)
        )

        parametros = self._parametros_estacionamiento
        kwargs = {"ahora": ahora}
        if "trasera_valida" in parametros:
            kwargs["trasera_valida"] = bool(corredor.trasera_valida)
        if "cobertura_trasera" in parametros:
            kwargs["cobertura_trasera"] = float(corredor.cobertura_trasera)
        if "distancia_lateral_mm" in parametros:
            kwargs["distancia_lateral_mm"] = float(lateral)
        if "lateral_mm" in parametros:
            kwargs["lateral_mm"] = float(lateral)
        if "lateral_valida" in parametros:
            kwargs["lateral_valida"] = bool(lateral_valida)
        for nombre, valor in (
            ("trasera_izquierda_mm", corredor.trasera_izquierda_mm),
            ("trasera_derecha_mm", corredor.trasera_derecha_mm),
            (
                "trasera_izquierda_valida",
                bool(corredor.trasera_izquierda_valida),
            ),
            (
                "trasera_derecha_valida",
                bool(corredor.trasera_derecha_valida),
            ),
            (
                "cobertura_trasera_izquierda",
                corredor.cobertura_trasera_izquierda,
            ),
            (
                "cobertura_trasera_derecha",
                corredor.cobertura_trasera_derecha,
            ),
        ):
            if nombre in parametros:
                kwargs[nombre] = valor

        # Con la FSM antigua no se deja que el valor centinela de una trasera
        # ciega parezca espacio libre. Una implementacion ampliada recibe la
        # validez explicitamente y puede seguir buscando hacia delante.
        if not corredor.trasera_valida and "trasera_valida" not in parametros:
            return self._emitir(
                0,
                0.0,
                "estacionamiento pausado: trasera no observable",
                detener_inmediato=True,
                direccion_neutra=True,
            )

        resultado = self._estacionamiento.procesar(
            hueco,
            self._heading_actual,
            corredor.frontal_mm,
            corredor.trasera_mm,
            **kwargs,
        )

        if resultado.estado == "FAILED":
            return self._fallar(
                resultado.razon or "fallo del controlador de estacionamiento",
                ahora,
            )
        if resultado.velocidad < 0:
            cobertura_axial_ok = _cobertura_suficiente(
                corredor.cobertura_trasera,
                float(self._parking_cfg["minimum_rear_coverage"]),
            )
            trasera_segura = bool(
                corredor.trasera_valida
                and cobertura_axial_ok
                and _distancia_lidar_real(corredor.trasera_mm, sin_dato)
                and corredor.trasera_mm
                > float(self._control["emergency_rear_mm"])
            )
            if not trasera_segura:
                return self._fallar(
                    "estacionamiento solicito reversa sin trasera segura", ahora
                )

            minimo_cobertura_diagonal = float(
                self._parking_cfg["minimum_rear_diagonal_coverage"]
            )
            minimo_diagonal = float(
                self._parking_cfg["minimum_rear_diagonal_clearance_mm"]
            )
            diagonales = (
                (
                    "izquierda",
                    corredor.trasera_izquierda_mm,
                    corredor.trasera_izquierda_valida,
                    corredor.cobertura_trasera_izquierda,
                ),
                (
                    "derecha",
                    corredor.trasera_derecha_mm,
                    corredor.trasera_derecha_valida,
                    corredor.cobertura_trasera_derecha,
                ),
            )
            for nombre, distancia, valida, cobertura in diagonales:
                diagonal_segura = bool(
                    valida
                    and _distancia_lidar_real(distancia, sin_dato)
                    and _cobertura_suficiente(
                        cobertura, minimo_cobertura_diagonal
                    )
                    and float(distancia) > minimo_diagonal
                )
                if not diagonal_segura:
                    return self._fallar(
                        "estacionamiento solicito reversa sin diagonal {} segura".format(
                            nombre
                        ),
                        ahora,
                    )

        if resultado.velocidad != 0 and resultado.estado in (
            "ALIGN",
            "ARC_IN",
            "ARC_OUT",
            "CENTER",
        ):
            if (
                not lateral_valida
                or lateral
                <= float(self._parking_cfg["minimum_lateral_clearance_mm"])
            ):
                return self._fallar(
                    "estacionamiento solicito movimiento sin lateral seguro",
                    ahora,
                )

        if resultado.estado == "DONE" or resultado.terminado:
            if not resultado.verificado:
                return self._fallar(
                    resultado.razon
                    or "estacionamiento termino sin verificacion geometrica",
                    ahora,
                )
            if bool(self._parking_cfg.get("require_lateral_verification", True)):
                objetivo = float(
                    self._parking_cfg["target_outer_wall_lidar_mm"]
                )
                tolerancia = float(self._parking_cfg["lateral_tolerance_mm"])
                if not lateral_valida or abs(lateral - objetivo) > tolerancia:
                    return self._fallar(
                        "estacionamiento sin verificacion lateral", ahora
                    )
            self._razon_terminal = resultado.razon
            self._entrar("DONE", ahora)
            return self._emitir(
                0,
                0.0,
                resultado.razon,
                terminado=True,
                verificado=True,
                detener_inmediato=True,
                direccion_neutra=True,
            )

        return self._emitir(
            resultado.velocidad,
            resultado.angulo,
            "parqueo/{}: {}".format(resultado.estado, resultado.razon),
            detener_inmediato=resultado.velocidad == 0,
            direccion_neutra=(
                resultado.velocidad == 0 and abs(resultado.angulo) < 1e-9
            ),
        )

    def procesar(
        self,
        corredor: Corredor,
        tracks: Sequence[TrackObstaculo],
        heading_deg: float,
        color_piso: Optional[str],
        hueco: Optional[HuecoParqueo] = None,
        ahora: Optional[float] = None,
    ) -> Consigna:
        """Avanza exactamente un ciclo, sin realizar efectos laterales."""

        instante = time.monotonic() if ahora is None else float(ahora)
        if self._t_inicio is None:
            self._t_inicio = instante
            self._t_estado = instante

        if self._estado == "DONE":
            return self._emitir(
                0,
                0.0,
                self._razon_terminal,
                terminado=True,
                verificado=True,
                detener_inmediato=True,
                direccion_neutra=True,
            )
        if self._estado == "FAILED":
            return self._emitir(
                0,
                0.0,
                self._razon_terminal,
                terminado=True,
                verificado=False,
                detener_inmediato=True,
                direccion_neutra=True,
            )

        try:
            heading_valido = math.isfinite(float(heading_deg))
        except (TypeError, ValueError):
            heading_valido = False
        if not heading_valido:
            return self._fallar("heading IMU invalido", instante)
        for nombre in ("frontal_mm", "frontal_muro_mm", "izquierda_mm", "derecha_mm"):
            if not _finito_no_negativo(getattr(corredor, nombre)):
                return self._fallar("geometria critica invalida: " + nombre, instante)
        self._heading_actual = float(heading_deg)

        if self._estado == "WAIT_DIRECTION":
            if not self._resolver_sentido(color_piso):
                if instante - self._t_inicio > float(
                    self._control["direction_timeout_s"]
                ):
                    return self._fallar(
                        "timeout esperando color de sentido", instante
                    )
                return self._emitir(
                    0,
                    0.0,
                    "esperando AZUL/NARANJA para fijar sentido",
                    detener_inmediato=True,
                    direccion_neutra=True,
                )
            # El heading del arranque define el rumbo del primer tramo; las
            # esquinas contadas le suman 90 grados por sentido de vuelta.
            self._heading_referencia = self._heading_actual
            self._entrar("CRUISE", instante)

        if self._estado == "PARKING":
            return self._procesar_estacionamiento(corredor, hueco, instante)

        # Prioridad global sobre carrera, evasion y giro. RECOVERY conserva
        # su propio mando hasta despejar o fallar; no cuenta una emergencia
        # nueva por cada barrido del mismo episodio.
        if self._estado != "RECOVERY" and self._hay_emergencia(corredor):
            self._iniciar_recuperacion(instante)
        if self._estado == "RECOVERY":
            return self._procesar_recuperacion(corredor, instante)
        if self._estado == "FORCED_TURN":
            return self._procesar_giro_forzado(corredor, instante)
        if self._estado == "TURN":
            return self._procesar_giro(corredor, instante)
        if self._estado == "AVOID_APPROACH":
            return self._procesar_aproximacion(corredor, tracks, instante)
        if self._estado == "AVOID_PASS":
            return self._procesar_sobrepaso(corredor, tracks, instante)
        if self._estado == "RECENTER":
            return self._procesar_recentrado(corredor, instante)
        if self._estado == "CRUISE":
            return self._procesar_crucero(corredor, tracks, instante)
        return self._fallar("estado de ruta desconocido", instante)


__all__ = ["ControlRuta"]
