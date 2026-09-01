"""Maquina de estados pura para el estacionamiento en paralelo.

El controlador no abre puertos ni conoce drivers. Recibe un hueco que ya fue
persistido por :mod:`percepcion_lidar`, el rumbo acumulado y distancias
filtradas. La convencion de ``HuecoParqueo.lado`` es la del marco LiDAR:
``-1`` izquierda y ``+1`` derecha.

La seguridad es deliberadamente conservadora. Una trasera sin cobertura nunca
se interpreta como espacio libre: se ordena detener el robot y se conserva el
estado hasta recuperar la medida o agotar el timeout. ``DONE`` solo se alcanza
despues de varios barridos geometricamente consistentes.
"""

import math
import time
from typing import Any, Dict, Optional

from .modelos import HuecoParqueo, ResultadoParqueo


def _diferencia_angular(actual: float, referencia: float) -> float:
    """Diferencia firmada en grados, acotada a [-180, 180)."""

    return (actual - referencia + 180.0) % 360.0 - 180.0


class ControlEstacionamiento:
    """Controla busqueda, entrada en dos arcos y verificacion del parqueo."""

    ESTADOS = (
        "SEARCH_GAP",
        "ALIGN",
        "ARC_IN",
        "ARC_OUT",
        "CENTER",
        "VERIFY",
        "DONE",
        "FAILED",
    )

    def __init__(self, config: Dict[str, Any]):
        self._config = config
        self._parking = config["parking"]
        self._control = config["control"]

        self._velocidad = abs(int(self._control["speed_parking_pwm"]))
        self._angulo_izq = float(self._control["steering_max_left_deg"])
        self._angulo_der = float(self._control["steering_max_right_deg"])
        self._confianza_minima = max(
            0.5, float(config.get("fusion", {}).get("min_color_confidence", 0.55))
        )

        # PercepcionLidar ya exige persistencia durante varios barridos. La FSM
        # no vuelve a aplicar ese filtro y bloquea el primer HuecoParqueo valido.
        self._perdidas_hueco_max = max(
            0,
            int(
                self._parking.get(
                    "align_gap_loss_tolerance_scans",
                    self._parking.get("align_lost_scans", 2),
                )
            ),
        )

        self._umbral_reversa_mm = max(
            float(self._parking["center_min_clearance_mm"]),
            float(self._control.get("emergency_rear_mm", 0.0)),
        )
        self._objetivo_align_mm = self._calcular_objetivo_alineacion()

        claves_geometria = (
            "robot_length_mm",
            "rear_overhang_mm",
            "lidar_forward_from_rear_axle_mm",
        )
        if all(clave in self._parking for clave in claves_geometria):
            largo_robot = float(self._parking["robot_length_mm"])
            voladizo_trasero = float(self._parking["rear_overhang_mm"])
            lidar_desde_eje = float(
                self._parking["lidar_forward_from_rear_axle_mm"]
            )
            # Positivo si el LiDAR esta por delante del centro geometrico.
            self._offset_lidar_centro_mm = (
                voladizo_trasero + lidar_desde_eje - largo_robot / 2.0
            )
        else:
            # Las configuraciones anteriores centraban el propio LiDAR.
            self._offset_lidar_centro_mm = 0.0
        self._delta_centrado_objetivo_mm = 2.0 * self._offset_lidar_centro_mm

        largo_hueco = self._parking.get("bay_length_mm")
        self._largo_hueco_mm = (
            float(largo_hueco) if largo_hueco is not None else None
        )
        tolerancia_centro = float(
            self._parking["center_clearance_tolerance_mm"]
        )
        grosor_separador = float(self._parking.get("separator_thickness_mm", 0.0))
        self._tolerancia_largo_hueco_mm = float(
            self._parking.get(
                "bay_length_tolerance_mm",
                max(2.0 * tolerancia_centro, 2.0 * grosor_separador, 40.0),
            )
        )

        self._requiere_lateral = bool(
            self._parking.get("require_lateral_verification", False)
        )
        self._objetivo_lateral_mm = float(
            self._parking.get("target_outer_wall_lidar_mm", 0.0)
        )
        self._tolerancia_lateral_mm = float(
            self._parking.get("lateral_tolerance_mm", 0.0)
        )
        self._sin_dato_mm = float(
            config.get("lidar", {}).get("rear_no_data_mm", 8000.0)
        )
        self._cobertura_trasera_minima = float(
            self._parking["minimum_rear_coverage"]
        )
        self._cobertura_diagonal_minima = float(
            self._parking["minimum_rear_diagonal_coverage"]
        )
        self._despeje_diagonal_minimo_mm = float(
            self._parking["minimum_rear_diagonal_clearance_mm"]
        )
        self._despeje_lateral_minimo_mm = float(
            self._parking["minimum_lateral_clearance_mm"]
        )
        self.reiniciar()

    def _calcular_objetivo_alineacion(self) -> float:
        """Posicion del borde delantero al alinear el eje trasero.

        El borde del separador se expresa respecto al LiDAR. El ajuste
        ``align_edge_trim_mm`` permite adelantar ligeramente el inicio del arco.
        En configuraciones antiguas se infiere ese ajuste a partir de
        ``align_edge_y_mm`` para conservar exactamente el comportamiento previo.
        """

        legado = self._parking.get("align_edge_y_mm")
        if "lidar_forward_from_rear_axle_mm" not in self._parking:
            return float(legado if legado is not None else 0.0)

        lidar_desde_eje = float(self._parking["lidar_forward_from_rear_axle_mm"])
        medio_separador = float(
            self._parking.get("separator_thickness_mm", 0.0)
        ) / 2.0
        if "align_edge_trim_mm" in self._parking:
            recorte = float(self._parking["align_edge_trim_mm"])
        elif legado is not None:
            # -lidar + medio + recorte == objetivo legado.
            recorte = float(legado) + lidar_desde_eje - medio_separador
        else:
            recorte = 0.0
        return -lidar_desde_eje + medio_separador + recorte

    @property
    def hueco_bloqueado(self) -> Optional[HuecoParqueo]:
        return self._hueco

    @property
    def objetivo_alineacion_mm(self) -> float:
        """Objetivo longitudinal usado por ``ALIGN`` (util para telemetria)."""

        return self._objetivo_align_mm

    @property
    def delta_centrado_objetivo_mm(self) -> float:
        """Valor esperado de ``trasera - frontal`` al centrar el robot."""

        return self._delta_centrado_objetivo_mm

    def reiniciar(self) -> None:
        self.estado = "SEARCH_GAP"
        self._t_inicio: Optional[float] = None
        self._t_estado: Optional[float] = None
        self._hueco: Optional[HuecoParqueo] = None
        self._perdidas_hueco = 0
        self._heading_paralelo = 0.0
        self._verificaciones = 0
        self._ultimo_barrido_verificacion: Optional[float] = None
        self._razon_terminal = ""

    def _entrar(self, estado: str, ahora: float) -> None:
        self.estado = estado
        self._t_estado = ahora

    def _resultado(
        self,
        velocidad: int,
        angulo: float,
        razon: str = "",
        terminado: bool = False,
        verificado: bool = False,
    ) -> ResultadoParqueo:
        return ResultadoParqueo(
            velocidad=int(velocidad),
            angulo=float(angulo),
            estado=self.estado,
            razon=razon,
            terminado=terminado,
            verificado=verificado,
        )

    def _fallar(self, razon: str, ahora: float) -> ResultadoParqueo:
        self._razon_terminal = razon
        self._entrar("FAILED", ahora)
        return self._resultado(0, 0.0, razon, terminado=True, verificado=False)

    def _distancia_valida(self, valor: Any) -> bool:
        try:
            valor_float = float(valor)
        except (TypeError, ValueError):
            return False
        # El centinela SIN_DATO (8000 mm en el C1) nunca equivale a libre.
        return bool(
            math.isfinite(valor_float)
            and 0.0 < valor_float < self._sin_dato_mm
        )

    @staticmethod
    def _cobertura_valida(valor: Any, minimo: float) -> bool:
        try:
            cobertura = float(valor)
        except (TypeError, ValueError):
            return False
        return bool(math.isfinite(cobertura) and minimo <= cobertura <= 1.0)

    def _hueco_valido(self, hueco: Optional[HuecoParqueo]) -> bool:
        return bool(
            hueco is not None
            and hueco.lado in (-1, 1)
            and math.isfinite(hueco.centro_y_mm)
            and math.isfinite(hueco.borde_delantero_y_mm)
            and hueco.separacion_mm > 0.0
            and hueco.confianza >= self._confianza_minima
        )

    def _angulo_hacia_hueco(self) -> float:
        assert self._hueco is not None
        return self._angulo_der if self._hueco.lado > 0 else self._angulo_izq

    def _angulo_fuera_hueco(self) -> float:
        assert self._hueco is not None
        return self._angulo_izq if self._hueco.lado > 0 else self._angulo_der

    def _movimiento_seguro(
        self,
        velocidad: int,
        frontal_mm: float,
        trasera_mm: float,
        ahora: float,
        *,
        trasera_valida: bool = True,
        cobertura_trasera: float = 0.0,
        lateral_mm: Optional[float] = None,
        lateral_valida: bool = False,
        trasera_izquierda_mm: Optional[float] = None,
        trasera_derecha_mm: Optional[float] = None,
        trasera_izquierda_valida: bool = False,
        trasera_derecha_valida: bool = False,
        cobertura_trasera_izquierda: float = 0.0,
        cobertura_trasera_derecha: float = 0.0,
        exigir_lateral: bool = False,
        exigir_diagonales: bool = False,
    ) -> Optional[ResultadoParqueo]:
        minimo_frontal = float(self._parking["center_min_clearance_mm"])
        if velocidad != 0 and exigir_lateral:
            if not lateral_valida or not self._distancia_valida(lateral_mm):
                return self._resultado(
                    0,
                    0.0,
                    "movimiento inhibido: distancia lateral no valida",
                )
            assert lateral_mm is not None
            if float(lateral_mm) <= self._despeje_lateral_minimo_mm:
                return self._fallar(
                    "holgura lateral critica durante estacionamiento", ahora
                )

        if velocidad < 0:
            trasera_observable = bool(
                trasera_valida
                and self._distancia_valida(trasera_mm)
                and self._cobertura_valida(
                    cobertura_trasera, self._cobertura_trasera_minima
                )
            )
            if not trasera_observable:
                return self._resultado(
                    0,
                    0.0,
                    "reversa inhibida: cobertura trasera no valida",
                )
            if trasera_mm <= self._umbral_reversa_mm:
                return self._fallar(
                    "sin holgura trasera segura para continuar", ahora
                )

            if exigir_diagonales:
                diagonales = (
                    (
                        "izquierda",
                        trasera_izquierda_mm,
                        trasera_izquierda_valida,
                        cobertura_trasera_izquierda,
                    ),
                    (
                        "derecha",
                        trasera_derecha_mm,
                        trasera_derecha_valida,
                        cobertura_trasera_derecha,
                    ),
                )
                for nombre, distancia, valida, cobertura in diagonales:
                    observable = bool(
                        valida
                        and self._distancia_valida(distancia)
                        and self._cobertura_valida(
                            cobertura, self._cobertura_diagonal_minima
                        )
                    )
                    if not observable:
                        return self._resultado(
                            0,
                            0.0,
                            "reversa inhibida: diagonal trasera {} no valida".format(
                                nombre
                            ),
                        )
                    assert distancia is not None
                    if float(distancia) <= self._despeje_diagonal_minimo_mm:
                        return self._fallar(
                            "holgura diagonal trasera {} critica".format(nombre),
                            ahora,
                        )
        if velocidad > 0 and frontal_mm <= minimo_frontal:
            return self._fallar("sin holgura frontal para continuar", ahora)
        return None

    def _timeout_estado(self, ahora: float, limite: float) -> bool:
        return self._t_estado is not None and ahora - self._t_estado > limite

    def _razon_timeout_estado(self, ahora: float) -> Optional[str]:
        if self.estado == "SEARCH_GAP":
            limite = float(self._parking["search_timeout_s"])
            razon = "timeout buscando hueco"
        elif self.estado == "ALIGN":
            limite = float(self._parking["align_timeout_s"])
            razon = "timeout alineando con el hueco"
        elif self.estado in ("ARC_IN", "ARC_OUT"):
            limite = float(self._parking["arc_timeout_s"])
            razon = "timeout en arco de estacionamiento"
        elif self.estado == "CENTER":
            limite = float(self._parking["center_timeout_s"])
            razon = "timeout centrando dentro del hueco"
        elif self.estado == "VERIFY":
            limite = float(self._parking["center_timeout_s"])
            razon = "timeout verificando estacionamiento"
        else:
            return None
        return razon if self._timeout_estado(ahora, limite) else None

    def _geometria_longitudinal_base(
        self, frontal_mm: float, trasera_mm: float
    ) -> bool:
        minimo = float(self._parking["center_min_clearance_mm"])
        maximo = float(self._parking["center_max_wall_distance_mm"])
        suma_compatible = (
            self._largo_hueco_mm is None
            or abs((frontal_mm + trasera_mm) - self._largo_hueco_mm)
            <= self._tolerancia_largo_hueco_mm
        )
        return bool(
            minimo <= frontal_mm <= maximo
            and minimo <= trasera_mm <= maximo
            and suma_compatible
        )

    def _longitudinal_centrado(self, frontal_mm: float, trasera_mm: float) -> bool:
        tolerancia = float(self._parking["center_clearance_tolerance_mm"])
        diferencia = trasera_mm - frontal_mm
        return self._geometria_longitudinal_base(frontal_mm, trasera_mm) and abs(
            diferencia - self._delta_centrado_objetivo_mm
        ) <= tolerancia

    def _lateral_verificado(
        self, lateral_mm: Optional[float], lateral_valida: bool
    ) -> bool:
        if not self._requiere_lateral:
            return True
        if not lateral_valida or not self._distancia_valida(lateral_mm):
            return False
        assert lateral_mm is not None
        return (
            abs(float(lateral_mm) - self._objetivo_lateral_mm)
            <= self._tolerancia_lateral_mm
        )

    def procesar(
        self,
        hueco: Optional[HuecoParqueo],
        heading_deg: float,
        frontal_mm: float,
        trasera_mm: float,
        ahora: Optional[float] = None,
        *,
        trasera_valida: bool = False,
        cobertura_trasera: float = 0.0,
        lateral_mm: Optional[float] = None,
        lateral_valida: bool = False,
        trasera_izquierda_mm: Optional[float] = None,
        trasera_derecha_mm: Optional[float] = None,
        trasera_izquierda_valida: bool = False,
        trasera_derecha_valida: bool = False,
        cobertura_trasera_izquierda: float = 0.0,
        cobertura_trasera_derecha: float = 0.0,
        distancia_ultrasonido_mm: Optional[float] = None,
    ) -> ResultadoParqueo:
        """Avanza un ciclo de la FSM y devuelve una orden sin efectos laterales.

        Las banderas de validez y coberturas son evidencia independiente del
        numero recibido. Un centinela ``SIN_DATO`` o una cobertura insuficiente
        frena el arco y deja correr su timeout; nunca se interpreta como libre.
        ``lateral_mm`` es la distancia LiDAR al muro exterior de la plaza.
        ``distancia_ultrasonido_mm`` aporta lectura de distancia fisica trasera/lateral
        complementaria e inmune a las oclusiones del mástil LiDAR.
        """

        if ahora is None:
            ahora = time.monotonic()
        ahora = float(ahora)

        if self.estado == "DONE":
            return self._resultado(
                0, 0.0, self._razon_terminal, terminado=True, verificado=True
            )
        if self.estado == "FAILED":
            return self._resultado(
                0, 0.0, self._razon_terminal, terminado=True, verificado=False
            )

        if self._t_inicio is None:
            self._t_inicio = ahora
            self._t_estado = ahora

        if not math.isfinite(heading_deg) or not self._distancia_valida(frontal_mm):
            return self._fallar("medicion de parqueo invalida", ahora)

        if ahora - self._t_inicio > float(self._parking["total_timeout_s"]):
            return self._fallar("timeout total de estacionamiento", ahora)

        razon_timeout = self._razon_timeout_estado(ahora)
        if razon_timeout is not None:
            return self._fallar(razon_timeout, ahora)

        trasera_disponible = bool(
            trasera_valida
            and self._distancia_valida(trasera_mm)
            and self._cobertura_valida(
                cobertura_trasera, self._cobertura_trasera_minima
            )
        )
        us_valido = bool(
            distancia_ultrasonido_mm is not None
            and math.isfinite(distancia_ultrasonido_mm)
            and 20.0 <= float(distancia_ultrasonido_mm) <= 2500.0
        )
        if not trasera_disponible and us_valido:
            trasera_mm = float(distancia_ultrasonido_mm)
            trasera_disponible = True
            cobertura_trasera = max(cobertura_trasera, 0.5)

        if not trasera_disponible:
            # No sumar una verificacion a ambos lados de una perdida de dato.
            self._verificaciones = 0
            self._ultimo_barrido_verificacion = None
        frontal_mm = float(frontal_mm)
        # El numero no se usa para autorizar reversa cuando la bandera es
        # falsa. Mantener un valor finito permite seguir BUSCANDO hacia
        # delante; la ausencia de eco trasero no debe inmovilizar una fase
        # que no retrocede.
        trasera_mm = (
            float(trasera_mm) if self._distancia_valida(trasera_mm) else 0.0
        )

        argumentos_guardia = {
            "trasera_valida": trasera_disponible,
            "cobertura_trasera": cobertura_trasera,
            "lateral_mm": lateral_mm,
            "lateral_valida": lateral_valida,
            "trasera_izquierda_mm": trasera_izquierda_mm,
            "trasera_derecha_mm": trasera_derecha_mm,
            "trasera_izquierda_valida": trasera_izquierda_valida,
            "trasera_derecha_valida": trasera_derecha_valida,
            "cobertura_trasera_izquierda": cobertura_trasera_izquierda,
            "cobertura_trasera_derecha": cobertura_trasera_derecha,
        }

        if self.estado == "SEARCH_GAP":
            if self._hueco_valido(hueco):
                assert hueco is not None
                self._hueco = hueco
                self._perdidas_hueco = 0
                self._entrar("ALIGN", ahora)
                return self._resultado(0, 0.0, "hueco persistido bloqueado")

            inseguro = self._movimiento_seguro(
                self._velocidad,
                frontal_mm,
                trasera_mm,
                ahora,
                **argumentos_guardia,
            )
            return inseguro or self._resultado(
                self._velocidad, 0.0, "buscando separadores de parqueo"
            )

        if self.estado == "ALIGN":
            if not self._hueco_valido(hueco):
                self._perdidas_hueco += 1
                if self._perdidas_hueco <= self._perdidas_hueco_max:
                    return self._resultado(
                        0,
                        0.0,
                        "hueco no visible; conservando ultimo ({}/{})".format(
                            self._perdidas_hueco, self._perdidas_hueco_max
                        ),
                    )
                return self._fallar("hueco perdido durante alineacion", ahora)

            assert hueco is not None
            assert self._hueco is not None
            if hueco.lado != self._hueco.lado:
                return self._fallar(
                    "lado del hueco cambio durante alineacion", ahora
                )
            self._perdidas_hueco = 0
            self._hueco = hueco

            error = hueco.borde_delantero_y_mm - self._objetivo_align_mm
            tolerancia = min(
                35.0, float(self._parking["center_clearance_tolerance_mm"])
            )
            if abs(error) <= tolerancia:
                self._heading_paralelo = heading_deg
                self._entrar("ARC_IN", ahora)
                return self._resultado(0, 0.0, "alineacion longitudinal lista")

            velocidad = self._velocidad if error > 0.0 else -self._velocidad
            inseguro = self._movimiento_seguro(
                velocidad,
                frontal_mm,
                trasera_mm,
                ahora,
                exigir_lateral=True,
                exigir_diagonales=velocidad < 0,
                **argumentos_guardia,
            )
            return inseguro or self._resultado(
                velocidad, 0.0, "alineando eje trasero con borde delantero"
            )

        if self.estado == "ARC_IN":
            inseguro = self._movimiento_seguro(
                -self._velocidad,
                frontal_mm,
                trasera_mm,
                ahora,
                exigir_lateral=True,
                exigir_diagonales=True,
                **argumentos_guardia,
            )
            if inseguro:
                return inseguro

            delta = abs(_diferencia_angular(heading_deg, self._heading_paralelo))
            if delta >= float(self._parking["entry_heading_delta_deg"]):
                self._entrar("ARC_OUT", ahora)
                return self._resultado(
                    -self._velocidad,
                    self._angulo_fuera_hueco(),
                    "delta de entrada alcanzado; contravolante inmediato",
                )
            return self._resultado(
                -self._velocidad,
                self._angulo_hacia_hueco(),
                "arco de entrada en reversa",
            )

        if self.estado == "ARC_OUT":
            inseguro = self._movimiento_seguro(
                -self._velocidad,
                frontal_mm,
                trasera_mm,
                ahora,
                exigir_lateral=True,
                exigir_diagonales=True,
                **argumentos_guardia,
            )
            if inseguro:
                return inseguro

            error_paralelo = abs(
                _diferencia_angular(heading_deg, self._heading_paralelo)
            )
            if error_paralelo <= float(self._parking["parallel_tolerance_deg"]):
                self._entrar("CENTER", ahora)
                return self._resultado(0, 0.0, "robot paralelo dentro del hueco")
            return self._resultado(
                -self._velocidad,
                self._angulo_fuera_hueco(),
                "contravolante en reversa",
            )

        if self.estado == "CENTER":
            if not trasera_disponible:
                return self._resultado(
                    0, 0.0, "detenido: cobertura trasera temporalmente no valida"
                )
            paralelo = abs(
                _diferencia_angular(heading_deg, self._heading_paralelo)
            ) <= float(self._parking["parallel_tolerance_deg"])
            if not paralelo:
                return self._fallar("se perdio paralelismo durante centrado", ahora)

            if not self._geometria_longitudinal_base(frontal_mm, trasera_mm):
                return self._resultado(
                    0,
                    0.0,
                    "esperando geometria longitudinal compatible con el hueco",
                )

            error = trasera_mm - frontal_mm - self._delta_centrado_objetivo_mm
            tolerancia = float(self._parking["center_clearance_tolerance_mm"])
            if abs(error) <= tolerancia:
                self._verificaciones = 0
                self._ultimo_barrido_verificacion = None
                self._entrar("VERIFY", ahora)
                return self._resultado(0, 0.0, "robot centrado; verificando")

            # Diferencia mayor al objetivo => el robot esta adelantado y debe
            # retroceder. Una diferencia menor se corrige hacia delante.
            velocidad_fina = max(8, self._velocidad // 2)
            velocidad = -velocidad_fina if error > 0.0 else velocidad_fina
            inseguro = self._movimiento_seguro(
                velocidad,
                frontal_mm,
                trasera_mm,
                ahora,
                exigir_lateral=True,
                exigir_diagonales=velocidad < 0,
                **argumentos_guardia,
            )
            return inseguro or self._resultado(
                velocidad, 0.0, "centrando el robot, no el LiDAR"
            )

        if self.estado == "VERIFY":
            if not trasera_disponible:
                return self._resultado(
                    0, 0.0, "detenido: cobertura trasera temporalmente no valida"
                )
            paralelo = abs(
                _diferencia_angular(heading_deg, self._heading_paralelo)
            ) <= float(self._parking["parallel_tolerance_deg"])
            longitudinal_ok = self._longitudinal_centrado(frontal_mm, trasera_mm)
            lateral_ok = self._lateral_verificado(lateral_mm, lateral_valida)

            if not paralelo or not longitudinal_ok:
                self._verificaciones = 0
                self._ultimo_barrido_verificacion = None
                if paralelo and self._geometria_longitudinal_base(
                    frontal_mm, trasera_mm
                ):
                    self._entrar("CENTER", ahora)
                    return self._resultado(
                        0, 0.0, "verificacion rechazo el centrado longitudinal"
                    )
                return self._resultado(
                    0, 0.0, "verificacion longitudinal geometricamente incierta"
                )

            if not lateral_ok:
                self._verificaciones = 0
                self._ultimo_barrido_verificacion = None
                if not lateral_valida or not self._distancia_valida(lateral_mm):
                    razon = "esperando medida lateral valida del muro exterior"
                else:
                    razon = "distancia lateral fuera del objetivo de parqueo"
                return self._resultado(0, 0.0, razon)

            if (
                self._ultimo_barrido_verificacion is None
                or ahora > self._ultimo_barrido_verificacion
            ):
                self._verificaciones += 1
                self._ultimo_barrido_verificacion = ahora
            verificaciones_necesarias = max(1, int(self._parking["verify_scans"]))
            if self._verificaciones >= verificaciones_necesarias:
                self._razon_terminal = (
                    "estacionamiento verificado en varios barridos"
                )
                self._entrar("DONE", ahora)
                return self._resultado(
                    0,
                    0.0,
                    self._razon_terminal,
                    terminado=True,
                    verificado=True,
                )
            return self._resultado(
                0,
                0.0,
                "verificacion estable {}/{}".format(
                    self._verificaciones, verificaciones_necesarias
                ),
            )

        return self._fallar("estado de estacionamiento desconocido", ahora)


def maniobra_estacionamiento(
    controlador: ControlEstacionamiento,
    corredor: Any,
    hueco: Optional[HuecoParqueo],
    heading_deg: float,
    ahora: Optional[float] = None,
    *,
    distancia_ultrasonido_mm: Optional[float] = None,
) -> ResultadoParqueo:
    """Función de alto nivel para ejecutar el paso de estacionamiento asistido por ultrasonido."""

    lateral = getattr(corredor, "derecha_mm", None)
    lateral_valida = bool(getattr(corredor, "derecha_valida", False))

    return controlador.procesar(
        hueco=hueco,
        heading_deg=heading_deg,
        frontal_mm=float(corredor.frontal_mm),
        trasera_mm=float(corredor.trasera_mm),
        ahora=ahora,
        trasera_valida=bool(getattr(corredor, "trasera_valida", True)),
        cobertura_trasera=float(getattr(corredor, "cobertura_trasera", 1.0)),
        lateral_mm=lateral,
        lateral_valida=lateral_valida,
        trasera_izquierda_mm=getattr(corredor, "trasera_izquierda_mm", None),
        trasera_derecha_mm=getattr(corredor, "trasera_derecha_mm", None),
        trasera_izquierda_valida=bool(getattr(corredor, "trasera_izquierda_valida", False)),
        trasera_derecha_valida=bool(getattr(corredor, "trasera_derecha_valida", False)),
        cobertura_trasera_izquierda=float(getattr(corredor, "cobertura_trasera_izquierda", 0.0)),
        cobertura_trasera_derecha=float(getattr(corredor, "cobertura_trasera_derecha", 0.0)),
        distancia_ultrasonido_mm=distancia_ultrasonido_mm,
    )


__all__ = ["ControlEstacionamiento", "maniobra_estacionamiento"]
