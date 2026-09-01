"""Percepcion geometrica ligera sobre el ``Medicion`` LiDAR existente.

El modulo no abre puertos ni importa drivers. Recibe el barrido que ya obtuvo
el proceso principal y la instancia de ``comun.lidar_geometria.Medicion``. Las
operaciones deliberadamente evitan NumPy y algoritmos de coste elevado para
mantener un tiempo predecible en la Raspberry Pi 3B.

Convenciones heredadas:

* 0 grados apunta al frente y el angulo crece en sentido horario.
* x positivo apunta a la derecha; y positivo, al frente.
* ``lado_parqueo`` vale +1 para la derecha, -1 para la izquierda y 0 para
  desactivar la busqueda.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from statistics import median
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .config import cargar_configuracion
from .modelos import Corredor, HuecoParqueo, ObjetoLidar, ParedEstimada
from .oclusion_lidar import (
    compensar_oclusion_lidar,
    configuracion_desde_mapa,
)


PuntoPolar = Tuple[float, float]
PuntoXY = Tuple[float, float]


@dataclass(frozen=True)
class ResultadoLidar:
    """Resultado completo de un barrido, sin referencias al hardware."""

    corredor: Corredor
    objetos: Tuple[ObjetoLidar, ...]
    hueco: Optional[HuecoParqueo]


@dataclass(frozen=True)
class _SegmentoLateral:
    x_mm: float
    y_mm: float
    longitud_mm: float
    alineacion: float
    residuo_mm: float
    puntos: int


@dataclass(frozen=True)
class _CandidatoHueco:
    borde_trasero_y_mm: float
    borde_delantero_y_mm: float
    separacion_mm: float
    distancia_lateral_mm: float
    confianza: float

    @property
    def centro_y_mm(self) -> float:
        return 0.5 * (self.borde_trasero_y_mm + self.borde_delantero_y_mm)


def _limitar(valor: float, minimo: float, maximo: float) -> float:
    return max(minimo, min(maximo, valor))


def _es_numero_valido(valor: Any) -> bool:
    try:
        return math.isfinite(float(valor))
    except (TypeError, ValueError):
        return False


def _normalizar_barrido(
    scan: Iterable[Sequence[float]], distancia_max_mm: float
) -> List[PuntoPolar]:
    """Filtra valores invalidos conservando el orden angular del dispositivo."""

    puntos: List[PuntoPolar] = []
    for muestra in scan:
        if len(muestra) < 2:
            continue
        angulo, distancia = muestra[0], muestra[1]
        if not _es_numero_valido(angulo) or not _es_numero_valido(distancia):
            continue
        distancia = float(distancia)
        if distancia <= 0.0 or distancia > distancia_max_mm:
            continue
        puntos.append((float(angulo) % 360.0, distancia))
    return puntos


def _salto_compatible(
    anterior: PuntoPolar,
    actual: PuntoPolar,
    factor_abd: float,
    offset_abd_mm: float,
    max_salto_angular_deg: float,
) -> bool:
    salto_angular = (actual[0] - anterior[0]) % 360.0
    if salto_angular > max_salto_angular_deg:
        return False
    umbral_radial = min(anterior[1], actual[1]) * factor_abd + offset_abd_mm
    return abs(actual[1] - anterior[1]) <= umbral_radial


def _segmentar_normalizado(
    puntos: Sequence[PuntoPolar],
    factor_abd: float,
    offset_abd_mm: float,
    max_salto_angular_deg: float,
    min_puntos: int,
) -> List[List[PuntoPolar]]:
    """Segmenta en O(n) por salto radial ABD y discontinuidad angular."""

    if not puntos:
        return []

    clusters: List[List[PuntoPolar]] = []
    actual: List[PuntoPolar] = [puntos[0]]
    for punto in puntos[1:]:
        if _salto_compatible(
            actual[-1], punto, factor_abd, offset_abd_mm, max_salto_angular_deg
        ):
            actual.append(punto)
        else:
            if len(actual) >= min_puntos:
                clusters.append(actual)
            actual = [punto]
    if len(actual) >= min_puntos:
        clusters.append(actual)

    # Un objeto que cruza 359 -> 0 queda en los extremos de un barrido
    # ascendente. La union circular sigue siendo O(1) tras el unico recorrido.
    if len(clusters) >= 2:
        primero, ultimo = clusters[0], clusters[-1]
        if _salto_compatible(
            ultimo[-1], primero[0], factor_abd, offset_abd_mm, max_salto_angular_deg
        ):
            clusters[0] = ultimo + primero
            clusters.pop()

    return clusters


def segmentar_angular(
    scan: Iterable[Sequence[float]],
    factor_abd: float = 0.04,
    offset_abd_mm: float = 40.0,
    max_salto_angular_deg: float = 4.0,
    min_puntos: int = 2,
    distancia_max_mm: float = 4000.0,
) -> List[List[PuntoPolar]]:
    """API funcional para la segmentacion angular lineal.

    El barrido debe conservar el orden de adquisicion del LiDAR. No se ordena
    internamente porque hacerlo anadiria coste O(n log n) en cada revolucion.
    """

    puntos = _normalizar_barrido(scan, distancia_max_mm)
    return _segmentar_normalizado(
        puntos,
        float(factor_abd),
        float(offset_abd_mm),
        float(max_salto_angular_deg),
        max(1, int(min_puntos)),
    )


def _polar_a_xy(angulo_deg: float, distancia_mm: float) -> PuntoXY:
    angulo_rad = math.radians(angulo_deg)
    return distancia_mm * math.sin(angulo_rad), distancia_mm * math.cos(angulo_rad)


def _en_sector(angulo_deg: float, limites: Sequence[float]) -> bool:
    inicio = float(limites[0]) % 360.0
    fin = float(limites[1]) % 360.0
    angulo = angulo_deg % 360.0
    if inicio <= fin:
        return inicio <= angulo <= fin
    return angulo >= inicio or angulo <= fin


def _ajuste_minimos_cuadrados(puntos: Sequence[PuntoXY]) -> Optional[Tuple[float, float]]:
    """Ajusta x=a*y+b. Retorna ``None`` si no hay extension longitudinal."""

    n = len(puntos)
    if n < 2:
        return None
    media_y = sum(p[1] for p in puntos) / n
    media_x = sum(p[0] for p in puntos) / n
    denominador = sum((p[1] - media_y) ** 2 for p in puntos)
    if denominador < 1.0:
        return None
    pendiente = sum((p[1] - media_y) * (p[0] - media_x) for p in puntos) / denominador
    intercepto = media_x - pendiente * media_y
    return pendiente, intercepto


def _medianas_longitudinales(puntos: Sequence[PuntoXY], cantidad: int) -> List[PuntoXY]:
    """Reduce la influencia de puntos aislados antes del primer ajuste."""

    if not puntos:
        return []
    y_min = min(p[1] for p in puntos)
    y_max = max(p[1] for p in puntos)
    extension = y_max - y_min
    if extension < 1.0:
        return []

    grupos: List[List[PuntoXY]] = [[] for _ in range(cantidad)]
    for punto in puntos:
        indice = int((punto[1] - y_min) * cantidad / extension)
        indice = min(cantidad - 1, max(0, indice))
        grupos[indice].append(punto)

    representantes: List[PuntoXY] = []
    for grupo in grupos:
        if grupo:
            representantes.append(
                (median([p[0] for p in grupo]), median([p[1] for p in grupo]))
            )
    return representantes


class PercepcionLidar:
    """Convierte ``scan + Medicion`` en geometria estable para navegacion."""

    def __init__(self, config: Optional[Mapping[str, Any]] = None):
        self.config: Mapping[str, Any] = config if config is not None else cargar_configuracion()
        self._lidar: Mapping[str, Any] = self.config.get("lidar", self.config)  # type: ignore[arg-type]
        self._config_oclusion = configuracion_desde_mapa(self._lidar)
        self.reiniciar()

    def reiniciar(self) -> None:
        """Borra la persistencia del detector de hueco de parqueo."""

        self._hueco_lado = 0
        self._hueco_conteo = 0
        self._hueco_ultimo_timestamp: Optional[float] = None
        self._hueco_candidato: Optional[_CandidatoHueco] = None
        self._hueco_confirmado: Optional[HuecoParqueo] = None

    def _cfg(self, nombre: str, defecto: Any) -> Any:
        return self._lidar.get(nombre, defecto)

    def segmentar(self, scan: Iterable[Sequence[float]]) -> List[List[PuntoPolar]]:
        """Segmenta un barrido usando la configuracion activa."""

        return segmentar_angular(
            scan,
            factor_abd=float(self._cfg("abd_factor", 0.04)),
            offset_abd_mm=float(self._cfg("abd_offset_mm", 40.0)),
            min_puntos=max(2, int(self._cfg("object_min_points", 3))),
            distancia_max_mm=float(self._cfg("max_distance_mm", 4000.0)),
        )

    def _objetos_desde_medicion(self, medicion: Any, timestamp: float) -> Tuple[ObjetoLidar, ...]:
        objetos: List[ObjetoLidar] = []
        min_puntos = int(self._cfg("object_min_points", 3))
        max_ancho = float(self._cfg("object_max_width_mm", 240.0))
        max_distancia = float(self._cfg("object_max_distance_mm", 1400.0))

        # ``clusters_obstaculo`` ya fue clasificado por el ProcesadorLidar
        # existente. Aqui solo se calcula geometria fisica y se aplica la cota
        # dimensional nueva, independiente de la distancia/angularidad.
        for cluster in getattr(medicion, "clusters_obstaculo", ()) or ():
            validos = [
                (float(p[0]) % 360.0, float(p[1]))
                for p in cluster
                if len(p) >= 2 and _es_numero_valido(p[0]) and _es_numero_valido(p[1]) and float(p[1]) > 0.0
            ]
            if len(validos) < min_puntos:
                continue

            cartesianos = [_polar_a_xy(a, d) for a, d in validos]
            x = sum(p[0] for p in cartesianos) / len(cartesianos)
            y = sum(p[1] for p in cartesianos) / len(cartesianos)
            distancia = math.hypot(x, y)
            if distancia <= 0.0 or distancia > max_distancia:
                continue

            bearing_rad = math.atan2(x, y)
            # Proyeccion sobre la tangente al rayo central: mide el ancho
            # transversal real sin un maximo O(n^2) entre pares de puntos.
            tangente_x = math.cos(bearing_rad)
            tangente_y = -math.sin(bearing_rad)
            proyecciones = [p[0] * tangente_x + p[1] * tangente_y for p in cartesianos]
            ancho = max(proyecciones) - min(proyecciones)
            if ancho > max_ancho:
                continue

            objetos.append(
                ObjetoLidar(
                    timestamp=timestamp,
                    x_mm=x,
                    y_mm=y,
                    distancia_mm=distancia,
                    bearing_deg=math.degrees(bearing_rad),
                    ancho_mm=ancho,
                    puntos=len(validos),
                )
            )

        objetos.sort(key=lambda objeto: objeto.distancia_mm)
        return tuple(objetos)

    @staticmethod
    def _bins_de_objetos(medicion: Any) -> set:
        bins = set()
        for cluster in getattr(medicion, "clusters_obstaculo", ()) or ():
            for punto in cluster:
                if len(punto) < 1 or not _es_numero_valido(punto[0]):
                    continue
                centro = int(float(punto[0])) % 360
                # Un grado de margen evita que los bordes del poste entren al
                # ajuste de pared por cuantizacion angular.
                bins.update(((centro - 1) % 360, centro, (centro + 1) % 360))
        return bins

    def _puntos_pared(
        self,
        scan: Sequence[PuntoPolar],
        sector: Sequence[float],
        lado: int,
        bins_excluidos: set,
    ) -> List[PuntoXY]:
        lateral_min = float(self._cfg("wall_side_min_mm", 80.0))
        lateral_max = float(self._cfg("wall_side_max_mm", 1600.0))
        extension_y = float(self._cfg("wall_forward_extent_mm", 1100.0))
        puntos: List[PuntoXY] = []

        for angulo, distancia in scan:
            if int(angulo) % 360 in bins_excluidos or not _en_sector(angulo, sector):
                continue
            x, y = _polar_a_xy(angulo, distancia)
            lateral = lado * x
            if lateral < lateral_min or lateral > lateral_max or abs(y) > extension_y:
                continue
            puntos.append((x, y))
        return puntos

    def _ajustar_pared(self, puntos: Sequence[PuntoXY], lado: int) -> Optional[ParedEstimada]:
        min_puntos = int(self._cfg("wall_min_points", 8))
        max_residuo = float(self._cfg("wall_max_residual_mm", 75.0))
        extension_objetivo = float(self._cfg("wall_forward_extent_mm", 1100.0))
        if len(puntos) < min_puntos:
            return None
        extension = max(p[1] for p in puntos) - min(p[1] for p in puntos)
        if extension < max(120.0, 0.18 * extension_objetivo):
            return None

        cantidad_grupos = min(9, max(3, len(puntos) // max(2, min_puntos // 2)))
        representantes = _medianas_longitudinales(puntos, cantidad_grupos)
        ajuste = _ajuste_minimos_cuadrados(representantes)
        if ajuste is None:
            return None

        inliers = list(puntos)
        for _ in range(2):
            pendiente, intercepto = ajuste
            norma = math.sqrt(1.0 + pendiente * pendiente)
            residuos_firmados = [
                (x - pendiente * y - intercepto) / norma for x, y in inliers
            ]
            centro = median(residuos_firmados)
            mad = median([abs(r - centro) for r in residuos_firmados])
            umbral = max(18.0, min(max_residuo, 8.0 + 3.0 * 1.4826 * mad))
            nuevos = [
                punto
                for punto, residuo in zip(inliers, residuos_firmados)
                if abs(residuo - centro) <= umbral
            ]
            if len(nuevos) < min_puntos:
                return None
            inliers = nuevos
            ajuste_nuevo = _ajuste_minimos_cuadrados(inliers)
            if ajuste_nuevo is None:
                return None
            ajuste = ajuste_nuevo

        pendiente, intercepto = ajuste
        if lado * intercepto <= 0.0:
            return None
        norma = math.sqrt(1.0 + pendiente * pendiente)
        residuos = [abs(x - pendiente * y - intercepto) / norma for x, y in inliers]
        residuo = sum(residuos) / len(residuos)
        if residuo > max_residuo:
            return None

        distancia = abs(intercepto)
        rumbo_error = math.degrees(math.atan(pendiente))
        proporcion_inliers = len(inliers) / float(len(puntos))
        calidad_residuo = _limitar(1.0 - residuo / max(max_residuo, 1.0), 0.0, 1.0)
        calidad_puntos = _limitar(len(inliers) / float(2 * min_puntos), 0.0, 1.0)
        extension_inliers = max(p[1] for p in inliers) - min(p[1] for p in inliers)
        calidad_extension = _limitar(
            extension_inliers / max(0.75 * extension_objetivo, 1.0), 0.0, 1.0
        )
        calidad = _limitar(
            0.38 * calidad_residuo
            + 0.24 * calidad_puntos
            + 0.23 * calidad_extension
            + 0.15 * proporcion_inliers,
            0.0,
            1.0,
        )

        return ParedEstimada(
            distancia_mm=distancia,
            pendiente=pendiente,
            rumbo_error_deg=rumbo_error,
            residuo_mm=residuo,
            puntos=len(inliers),
            calidad=calidad,
        )

    @staticmethod
    def _valor_medicion(medicion: Any, nombre: str, defecto: float) -> float:
        valor = getattr(medicion, nombre, defecto)
        return float(valor) if _es_numero_valido(valor) else float(defecto)

    @staticmethod
    def _sector_medicion_valido(
        medicion: Any, nombre: str, limite_sin_dato: float
    ) -> bool:
        """Distingue una lectura real de un valor centinela o inexistente."""

        valor = getattr(medicion, nombre, None)
        return bool(
            _es_numero_valido(valor)
            and 0.0 < float(valor) < float(limite_sin_dato)
        )

    def _construir_corredor(
        self, scan: Sequence[PuntoPolar], medicion: Any, timestamp: float
    ) -> Corredor:
        sin_dato = float(self._cfg("rear_no_data_mm", 8000.0))
        excluidos = self._bins_de_objetos(medicion)
        sector_izq = self._cfg("left_sector_deg", (225.0, 315.0))
        sector_der = self._cfg("right_sector_deg", (45.0, 135.0))
        pared_izq = self._ajustar_pared(
            self._puntos_pared(scan, sector_izq, -1, excluidos), -1
        )
        pared_der = self._ajustar_pared(
            self._puntos_pared(scan, sector_der, +1, excluidos), +1
        )

        izquierda = (
            pared_izq.distancia_mm
            if pared_izq is not None
            else self._valor_medicion(medicion, "izquierda", 2000.0)
        )
        derecha = (
            pared_der.distancia_mm
            if pared_der is not None
            else self._valor_medicion(medicion, "derecha", 2000.0)
        )

        # Sin recta ajustada se cae al minimo crudo del sector, que no pasa
        # por ``wall_side_min_mm`` y recoge el propio mecanismo de direccion:
        # al girar, la rueda entra en el barrido lateral y aparecen lecturas
        # por debajo del perimetro del robot, que esta a 61 mm por la
        # izquierda y 45 por la derecha del eje del LiDAR. Medido sobre las
        # corridas del 2026-08-31 y 09-01: 101 lecturas laterales imposibles
        # a la izquierda, con el servo mediano en +17 grados, y 90 a la
        # derecha con -8. Cada una podia disparar una emergencia falsa.
        #
        # Un eco mas cercano que el propio chasis no es una pared. Se marca
        # sin dato en vez de creerselo, y el error lateral deja de ser
        # calculable: ``_angulo_pared`` cae entonces a enderezar por rumbo,
        # que es el comportamiento seguro ya probado en pista.
        # El umbral del fallback es mayor que ``wall_side_min_mm``, que
        # solo filtra puntos para ajustar rectas. Con el servo al tope el
        # eco de la rueda aparece mas lejos: 49-51 mm medidos con el
        # volante a +17 y 91-107 con el a +25. Subirlo es seguro porque
        # una pared de verdad a esa distancia deja puntos de sobra y se
        # ajusta como recta; se cae al minimo crudo justamente cuando lo
        # que se ve no es una pared.
        lateral_min = float(
            self._cfg(
                "lateral_fallback_min_mm",
                float(self._cfg("wall_side_min_mm", 80.0)) * 2.0,
            )
        )
        autoeco_izq = pared_izq is None and izquierda < lateral_min
        autoeco_der = pared_der is None and derecha < lateral_min
        if autoeco_izq:
            izquierda = sin_dato
        if autoeco_der:
            derecha = sin_dato

        izquierda_valida = bool(
            not autoeco_izq
            and (
                pared_izq is not None
                or self._sector_medicion_valido(medicion, "izquierda", sin_dato)
            )
        )
        derecha_valida = bool(
            not autoeco_der
            and (
                pared_der is not None
                or self._sector_medicion_valido(medicion, "derecha", sin_dato)
            )
        )

        paredes = [p for p in (pared_izq, pared_der) if p is not None]
        if paredes:
            peso = sum(max(0.01, p.calidad) for p in paredes)
            error_rumbo = sum(p.rumbo_error_deg * max(0.01, p.calidad) for p in paredes) / peso
            calidad_pared = sum(p.calidad for p in paredes) / len(paredes)
            if len(paredes) == 2:
                diferencia = abs(paredes[0].rumbo_error_deg - paredes[1].rumbo_error_deg)
                calidad_pared *= _limitar(1.0 - diferencia / 18.0, 0.25, 1.0)
            else:
                calidad_pared *= 0.85
        else:
            error_rumbo = self._valor_medicion(medicion, "angulo_muro", 0.0)
            calidad_pared = 0.0

        frontal = self._valor_medicion(medicion, "frontal", 8000.0)
        frontal_muro = self._valor_medicion(medicion, "frontal_muro", frontal)

        # Los minimos traseros de ``Medicion`` estan contaminados por el
        # mastil (eco fijo trasero entre 163 y 191 grados). Se recalculan
        # desde los hombros visibles y, a diferencia del codigo historico,
        # la ausencia de retorno queda marcada como invalida.
        perfil = getattr(medicion, "perfil", None)
        if not isinstance(perfil, Sequence) or len(perfil) != 360:
            perfil_reconstruido = [sin_dato] * 360
            for angulo, distancia in scan:
                indice = int(angulo) % 360
                if distancia < perfil_reconstruido[indice]:
                    perfil_reconstruido[indice] = distancia
            perfil = perfil_reconstruido
        oclusion = compensar_oclusion_lidar(perfil, self._config_oclusion)
        trasera = (
            float(oclusion.trasera_axial_mm)
            if oclusion.trasera_valida
            else sin_dato
        )
        trasera_izq = (
            float(oclusion.diagonal_trasera_izquierda_mm)
            if oclusion.diagonal_izquierda_valida
            else sin_dato
        )
        trasera_der = (
            float(oclusion.diagonal_trasera_derecha_mm)
            if oclusion.diagonal_derecha_valida
            else sin_dato
        )
        return Corredor(
            timestamp=timestamp,
            frontal_mm=frontal,
            frontal_muro_mm=frontal_muro,
            trasera_mm=trasera,
            izquierda_mm=izquierda,
            derecha_mm=derecha,
            trasera_izquierda_mm=trasera_izq,
            trasera_derecha_mm=trasera_der,
            error_lateral_mm=(
                izquierda - derecha
                if not (autoeco_izq or autoeco_der)
                else float("nan")
            ),
            error_rumbo_muro_deg=error_rumbo,
            calidad_pared=_limitar(calidad_pared, 0.0, 1.0),
            pared_izquierda=pared_izq,
            pared_derecha=pared_der,
            izquierda_valida=izquierda_valida,
            derecha_valida=derecha_valida,
            trasera_valida=oclusion.trasera_valida,
            trasera_izquierda_valida=oclusion.diagonal_izquierda_valida,
            trasera_derecha_valida=oclusion.diagonal_derecha_valida,
            cobertura_trasera=oclusion.cobertura.trasera,
            cobertura_trasera_izquierda=oclusion.cobertura.diagonal_izquierda,
            cobertura_trasera_derecha=oclusion.cobertura.diagonal_derecha,
            diagnostico_oclusion=oclusion.diagnostico,
            mascara_oclusion_confirmada=oclusion.mascara_confirmada,
            estructura_fuera_mascara_deg=oclusion.estructura_fuera_mascara_deg,
        )

    @staticmethod
    def _segmento_pca(cluster: Sequence[PuntoPolar]) -> Optional[_SegmentoLateral]:
        if len(cluster) < 2:
            return None
        puntos = [_polar_a_xy(a, d) for a, d in cluster]
        n = len(puntos)
        mx = sum(p[0] for p in puntos) / n
        my = sum(p[1] for p in puntos) / n
        cxx = sum((p[0] - mx) ** 2 for p in puntos) / n
        cyy = sum((p[1] - my) ** 2 for p in puntos) / n
        cxy = sum((p[0] - mx) * (p[1] - my) for p in puntos) / n
        if cxx + cyy < 1.0:
            return None

        angulo = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
        ux, uy = math.cos(angulo), math.sin(angulo)
        proyecciones = [(x - mx) * ux + (y - my) * uy for x, y in puntos]
        longitud = max(proyecciones) - min(proyecciones)
        residuos = [abs(-(x - mx) * uy + (y - my) * ux) for x, y in puntos]
        return _SegmentoLateral(
            x_mm=mx,
            y_mm=my,
            longitud_mm=longitud,
            alineacion=abs(ux),
            residuo_mm=sum(residuos) / n,
            puntos=n,
        )

    def _buscar_hueco(
        self, clusters: Sequence[Sequence[PuntoPolar]], lado: int
    ) -> Optional[_CandidatoHueco]:
        longitud_min = float(self._cfg("bay_separator_min_length_mm", 125.0))
        longitud_max = float(self._cfg("bay_separator_max_length_mm", 290.0))
        separacion_esperada = float(self._cfg("bay_expected_separation_mm", 353.0))
        tolerancia = float(self._cfg("bay_separation_tolerance_mm", 105.0))
        lateral_min = float(self._cfg("bay_lateral_min_mm", 140.0))
        lateral_max = float(self._cfg("bay_lateral_max_mm", 900.0))
        longitudinal_max = float(self._cfg("bay_max_longitudinal_mm", 1100.0))
        min_puntos = max(3, int(self._cfg("object_min_points", 3)))
        cos_max_desvio = math.cos(math.radians(25.0))
        max_residuo = min(55.0, float(self._cfg("wall_max_residual_mm", 75.0)))

        segmentos: List[_SegmentoLateral] = []
        for cluster in clusters:
            if len(cluster) < min_puntos:
                continue
            segmento = self._segmento_pca(cluster)
            if segmento is None:
                continue
            lateral = lado * segmento.x_mm
            if not (longitud_min <= segmento.longitud_mm <= longitud_max):
                continue
            if segmento.alineacion < cos_max_desvio or segmento.residuo_mm > max_residuo:
                continue
            if not (lateral_min <= lateral <= lateral_max):
                continue
            if abs(segmento.y_mm) > longitudinal_max:
                continue
            segmentos.append(segmento)

        mejor: Optional[_CandidatoHueco] = None
        mejor_coste = float("inf")
        for indice, primero in enumerate(segmentos):
            for segundo in segmentos[indice + 1 :]:
                separacion = abs(segundo.y_mm - primero.y_mm)
                error_separacion = abs(separacion - separacion_esperada)
                if error_separacion > tolerancia:
                    continue
                lateral_1 = lado * primero.x_mm
                lateral_2 = lado * segundo.x_mm
                diferencia_lateral = abs(lateral_1 - lateral_2)
                if diferencia_lateral > tolerancia:
                    continue

                trasero = min(primero.y_mm, segundo.y_mm)
                delantero = max(primero.y_mm, segundo.y_mm)
                calidad_sep = 1.0 - error_separacion / max(tolerancia, 1.0)
                calidad_lat = 1.0 - diferencia_lateral / max(tolerancia, 1.0)
                calidad_alineacion = 0.5 * (primero.alineacion + segundo.alineacion)
                calidad_residuo = 1.0 - (
                    primero.residuo_mm + segundo.residuo_mm
                ) / max(2.0 * max_residuo, 1.0)
                confianza = _limitar(
                    0.42 * calidad_sep
                    + 0.23 * calidad_lat
                    + 0.23 * calidad_alineacion
                    + 0.12 * calidad_residuo,
                    0.0,
                    1.0,
                )
                coste = error_separacion + 0.45 * diferencia_lateral - 20.0 * confianza
                if coste < mejor_coste:
                    mejor_coste = coste
                    mejor = _CandidatoHueco(
                        borde_trasero_y_mm=trasero,
                        borde_delantero_y_mm=delantero,
                        separacion_mm=separacion,
                        distancia_lateral_mm=0.5 * (lateral_1 + lateral_2),
                        confianza=confianza,
                    )
        return mejor

    def _coincide_hueco(
        self, anterior: _CandidatoHueco, actual: _CandidatoHueco
    ) -> bool:
        tolerancia = float(self._cfg("bay_separation_tolerance_mm", 105.0))
        return (
            abs(anterior.separacion_mm - actual.separacion_mm) <= 0.65 * tolerancia
            and abs(anterior.distancia_lateral_mm - actual.distancia_lateral_mm) <= tolerancia
            and abs(anterior.centro_y_mm - actual.centro_y_mm) <= max(180.0, 1.6 * tolerancia)
        )

    def _actualizar_hueco(
        self,
        candidato: Optional[_CandidatoHueco],
        timestamp: float,
        lado: int,
    ) -> Optional[HuecoParqueo]:
        requeridos = max(1, int(self._cfg("bay_confirm_scans", 3)))

        if self._hueco_lado != lado:
            self.reiniciar()
            self._hueco_lado = lado

        # Una segunda llamada con el mismo timestamp no representa otro
        # barrido y nunca puede aumentar la persistencia.
        if (
            self._hueco_ultimo_timestamp is not None
            and timestamp <= self._hueco_ultimo_timestamp + 1e-9
        ):
            return self._hueco_confirmado

        self._hueco_ultimo_timestamp = timestamp
        if candidato is None:
            self._hueco_conteo = 0
            self._hueco_candidato = None
            self._hueco_confirmado = None
            return None

        if self._hueco_candidato is not None and self._coincide_hueco(
            self._hueco_candidato, candidato
        ):
            self._hueco_conteo += 1
        else:
            self._hueco_conteo = 1
        self._hueco_candidato = candidato

        if self._hueco_conteo < requeridos:
            self._hueco_confirmado = None
            return None

        confianza_persistencia = min(1.0, self._hueco_conteo / float(requeridos))
        self._hueco_confirmado = HuecoParqueo(
            timestamp=timestamp,
            lado=lado,
            borde_trasero_y_mm=candidato.borde_trasero_y_mm,
            borde_delantero_y_mm=candidato.borde_delantero_y_mm,
            centro_y_mm=candidato.centro_y_mm,
            separacion_mm=candidato.separacion_mm,
            distancia_lateral_mm=candidato.distancia_lateral_mm,
            confianza=_limitar(
                candidato.confianza * (0.7 + 0.3 * confianza_persistencia), 0.0, 1.0
            ),
        )
        return self._hueco_confirmado

    def procesar(
        self,
        scan: Iterable[Sequence[float]],
        medicion: Any,
        timestamp: Optional[float] = None,
        lado_parqueo: int = 0,
    ) -> ResultadoLidar:
        """Procesa un barrido sin efectuar ninguna operacion de hardware.

        Args:
            scan: pares ``(angulo_deg, distancia_mm)`` en orden de adquisicion.
            medicion: instancia actual de ``lidar_geometria.Medicion``.
            timestamp: timestamp del barrido; por defecto usa el de ``medicion``.
            lado_parqueo: +1 derecha, -1 izquierda, 0 desactivado.
        """

        if lado_parqueo not in (-1, 0, 1):
            raise ValueError("lado_parqueo debe ser -1, 0 o 1")
        ts = (
            float(timestamp)
            if timestamp is not None
            else self._valor_medicion(medicion, "timestamp", 0.0)
        )
        puntos = _normalizar_barrido(
            scan, float(self._cfg("max_distance_mm", 4000.0))
        )
        corredor = self._construir_corredor(puntos, medicion, ts)
        objetos = self._objetos_desde_medicion(medicion, ts)

        hueco: Optional[HuecoParqueo] = None
        if lado_parqueo == 0:
            # No arrastrar una confirmacion de una fase de busqueda anterior.
            if self._hueco_lado != 0:
                self.reiniciar()
        else:
            clusters = _segmentar_normalizado(
                puntos,
                float(self._cfg("abd_factor", 0.04)),
                float(self._cfg("abd_offset_mm", 40.0)),
                4.0,
                max(2, int(self._cfg("object_min_points", 3))),
            )
            hueco = self._actualizar_hueco(
                self._buscar_hueco(clusters, lado_parqueo), ts, lado_parqueo
            )

        return ResultadoLidar(corredor=corredor, objetos=objetos, hueco=hueco)


__all__ = ["PercepcionLidar", "ResultadoLidar", "segmentar_angular"]
