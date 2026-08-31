"""Compensacion pura de la oclusion trasera causada por el mastil.

El perfil de entrada conserva la convencion del C1: 360 bins, 0 grados al
frente y angulos crecientes en sentido horario.  Un bin con ``SIN_DATO`` no
se interpreta como espacio libre.  Las distancias invalidas se devuelven
como ``None`` y tienen una bandera de validez asociada.

Este modulo no modifica la medicion original y no depende de drivers ni de
NumPy.  Los valores por defecto reproducen la mascara medida en
``ronda_camara/lidar_mascara.py``; se agrupan en una dataclass para permitir
recalibrarlos si cambia la posicion fisica de la camara o del mastil.
"""

from dataclasses import dataclass
import math
from typing import Mapping, Optional, Sequence, Tuple


SIN_DATO = 8000.0


@dataclass(frozen=True)
class ConfiguracionOclusionLidar:
    """Geometria y umbrales de la mascara trasera, expresados en grados/mm."""

    arco_ciego_min_deg: int = 163
    arco_ciego_max_deg: int = 191
    eje_trasero_deg: int = 180
    hombro_offset_min_deg: int = 18
    hombro_offset_max_deg: int = 35
    diagonal_offset_max_deg: int = 90
    min_puntos_validos: int = 2
    distancia_estructura_mm: float = 150.0
    sin_dato_mm: float = SIN_DATO

    def __post_init__(self) -> None:
        if not 0 <= self.arco_ciego_min_deg < 360:
            raise ValueError("arco_ciego_min_deg debe estar en [0, 359]")
        if not 0 <= self.arco_ciego_max_deg < 360:
            raise ValueError("arco_ciego_max_deg debe estar en [0, 359]")
        if not 0 <= self.eje_trasero_deg < 360:
            raise ValueError("eje_trasero_deg debe estar en [0, 359]")
        if not 0 < self.hombro_offset_min_deg <= self.hombro_offset_max_deg < 90:
            raise ValueError("los offsets del hombro deben cumplir 0 < min <= max < 90")
        if not self.hombro_offset_max_deg <= self.diagonal_offset_max_deg <= 180:
            raise ValueError("diagonal_offset_max_deg debe cubrir el hombro y ser <= 180")
        if self.min_puntos_validos < 1:
            raise ValueError("min_puntos_validos debe ser positivo")
        if self.distancia_estructura_mm <= 0.0:
            raise ValueError("distancia_estructura_mm debe ser positiva")
        if self.sin_dato_mm <= self.distancia_estructura_mm:
            raise ValueError("sin_dato_mm debe ser mayor que distancia_estructura_mm")


@dataclass(frozen=True)
class CoberturaOclusionLidar:
    """Fraccion de bins visibles que aporto una distancia real, en [0, 1]."""

    hombro_derecho: float
    hombro_izquierdo: float
    trasera: float
    diagonal_derecha: float
    diagonal_izquierda: float


@dataclass(frozen=True)
class ResultadoOclusionLidar:
    """Medidas traseras corregidas y evidencia usada para validarlas."""

    trasera_axial_mm: Optional[float]
    diagonal_trasera_izquierda_mm: Optional[float]
    diagonal_trasera_derecha_mm: Optional[float]
    trasera_valida: bool
    diagonal_izquierda_valida: bool
    diagonal_derecha_valida: bool
    cobertura: CoberturaOclusionLidar
    mascara_confirmada: bool
    diagnostico: str
    estructura_fuera_mascara_deg: Tuple[int, ...]

    @property
    def trasera_izquierda_mm(self) -> Optional[float]:
        """Alias compatible con el nombre historico de la diagonal izquierda."""

        return self.diagonal_trasera_izquierda_mm

    @property
    def trasera_derecha_mm(self) -> Optional[float]:
        """Alias compatible con el nombre historico de la diagonal derecha."""

        return self.diagonal_trasera_derecha_mm


CONFIGURACION_MEDIDA = ConfiguracionOclusionLidar()


def configuracion_desde_mapa(
    lidar: Mapping[str, object],
) -> ConfiguracionOclusionLidar:
    """Construye la mascara desde la seccion ``lidar`` del JSON.

    El montaje actual tiene una sola cuña trasera. Se rechazan configuraciones
    con cero o mas de un sector para no aplicar silenciosamente una geometria
    distinta a la que este compensador sabe proyectar.
    """

    sectores = lidar.get("blind_sectors_deg", ((163.0, 191.0),))
    if not isinstance(sectores, (list, tuple)) or len(sectores) != 1:
        raise ValueError("se requiere exactamente un sector ciego trasero")
    sector = sectores[0]
    hombros = lidar.get("rear_shoulder_offset_deg", (18.0, 35.0))
    if not isinstance(sector, (list, tuple)) or len(sector) != 2:
        raise ValueError("blind_sectors_deg debe contener pares")
    if not isinstance(hombros, (list, tuple)) or len(hombros) != 2:
        raise ValueError("rear_shoulder_offset_deg debe contener min/max")

    return ConfiguracionOclusionLidar(
        arco_ciego_min_deg=int(round(float(sector[0]))),
        arco_ciego_max_deg=int(round(float(sector[1]))),
        eje_trasero_deg=int(round(float(lidar.get("rear_axis_deg", 180.0)))),
        hombro_offset_min_deg=int(round(float(hombros[0]))),
        hombro_offset_max_deg=int(round(float(hombros[1]))),
        diagonal_offset_max_deg=90,
        min_puntos_validos=int(lidar.get("rear_min_valid_points", 2)),
        distancia_estructura_mm=float(lidar.get("self_echo_max_mm", 150.0)),
        sin_dato_mm=float(lidar.get("rear_no_data_mm", SIN_DATO)),
    )


def _indices_inclusivos(inicio: int, fin: int) -> Tuple[int, ...]:
    inicio %= 360
    fin %= 360
    if inicio <= fin:
        return tuple(range(inicio, fin + 1))
    return tuple(range(inicio, 360)) + tuple(range(0, fin + 1))


def _es_dato(distancia: float, configuracion: ConfiguracionOclusionLidar) -> bool:
    return math.isfinite(distancia) and 0.0 < distancia < configuracion.sin_dato_mm


def _datos_sector(
    perfil: Sequence[float],
    indices: Sequence[int],
    mascara: frozenset,
    configuracion: ConfiguracionOclusionLidar,
) -> Tuple[Tuple[Tuple[int, float], ...], float]:
    visibles = tuple(indice for indice in indices if indice not in mascara)
    datos = tuple(
        (indice, float(perfil[indice]))
        for indice in visibles
        if _es_dato(float(perfil[indice]), configuracion)
    )
    cobertura = len(datos) / float(len(visibles)) if visibles else 0.0
    return datos, cobertura


def compensar_oclusion_lidar(
    perfil: Sequence[float],
    configuracion: ConfiguracionOclusionLidar = CONFIGURACION_MEDIDA,
) -> ResultadoOclusionLidar:
    """Calcula distancias traseras sin alterar ``perfil``.

    La distancia axial se obtiene de los hombros simetricos situados entre
    18 y 35 grados a ambos lados del eje trasero. Cada haz se proyecta con
    ``d * cos(offset)`` y se conserva el minimo por seguridad. Las diagonales
    abarcan, tambien simetricamente, desde 18 hasta 90 grados del eje.

    Un sector que no contiene ningun retorno real produce ``None`` y su
    bandera ``*_valida`` queda en falso. Esto evita convertir una zona ciega
    en una falsa via libre de 8000 mm.
    """

    if len(perfil) != 360:
        raise ValueError("el perfil LiDAR debe contener exactamente 360 bins")

    eje = configuracion.eje_trasero_deg
    offset_min = configuracion.hombro_offset_min_deg
    offset_max = configuracion.hombro_offset_max_deg
    offset_diagonal = configuracion.diagonal_offset_max_deg

    mascara = frozenset(
        _indices_inclusivos(
            configuracion.arco_ciego_min_deg,
            configuracion.arco_ciego_max_deg,
        )
    )
    hombro_derecho = _indices_inclusivos(eje - offset_max, eje - offset_min)
    hombro_izquierdo = _indices_inclusivos(eje + offset_min, eje + offset_max)
    diagonal_derecha = _indices_inclusivos(eje - offset_diagonal, eje - offset_min)
    diagonal_izquierda = _indices_inclusivos(eje + offset_min, eje + offset_diagonal)

    datos_hd, cobertura_hd = _datos_sector(
        perfil, hombro_derecho, mascara, configuracion
    )
    datos_hi, cobertura_hi = _datos_sector(
        perfil, hombro_izquierdo, mascara, configuracion
    )
    datos_dd, cobertura_dd = _datos_sector(
        perfil, diagonal_derecha, mascara, configuracion
    )
    datos_di, cobertura_di = _datos_sector(
        perfil, diagonal_izquierda, mascara, configuracion
    )

    datos_hombros = datos_hd + datos_hi
    proyecciones = tuple(
        distancia
        * math.cos(
            math.radians(abs(((indice - eje + 180) % 360) - 180))
        )
        for indice, distancia in datos_hombros
    )
    minimo = configuracion.min_puntos_validos
    trasera_axial = min(proyecciones) if len(proyecciones) >= minimo else None
    trasera_derecha = (
        min(distancia for _, distancia in datos_dd)
        if len(datos_dd) >= minimo
        else None
    )
    trasera_izquierda = (
        min(distancia for _, distancia in datos_di)
        if len(datos_di) >= minimo
        else None
    )

    bins_hombros_visibles = sum(
        1
        for indice in hombro_derecho + hombro_izquierdo
        if indice not in mascara
    )
    cobertura_trasera = (
        len(datos_hombros) / float(bins_hombros_visibles)
        if bins_hombros_visibles
        else 0.0
    )

    estructura_dentro = tuple(
        indice
        for indice in sorted(mascara)
        if _es_dato(float(perfil[indice]), configuracion)
        and float(perfil[indice]) < configuracion.distancia_estructura_mm
    )
    estructura_fuera = tuple(
        indice
        for indice in range(360)
        if indice not in mascara
        and _es_dato(float(perfil[indice]), configuracion)
        and float(perfil[indice]) < configuracion.distancia_estructura_mm
    )

    if estructura_fuera:
        mascara_confirmada = False
        diagnostico = (
            "estructura fuera de la mascara en grados "
            f"{estructura_fuera}; recalibrar la oclusion"
        )
    elif not estructura_dentro:
        mascara_confirmada = False
        diagnostico = (
            "no se confirmo el mastil dentro de la mascara; "
            "verificar montaje o recalibrar"
        )
    else:
        mascara_confirmada = True
        diagnostico = (
            f"mascara confirmada por {len(estructura_dentro)} bins de estructura"
        )

    return ResultadoOclusionLidar(
        trasera_axial_mm=trasera_axial,
        diagonal_trasera_izquierda_mm=trasera_izquierda,
        diagonal_trasera_derecha_mm=trasera_derecha,
        trasera_valida=trasera_axial is not None,
        diagonal_izquierda_valida=trasera_izquierda is not None,
        diagonal_derecha_valida=trasera_derecha is not None,
        cobertura=CoberturaOclusionLidar(
            hombro_derecho=cobertura_hd,
            hombro_izquierdo=cobertura_hi,
            trasera=cobertura_trasera,
            diagonal_derecha=cobertura_dd,
            diagonal_izquierda=cobertura_di,
        ),
        mascara_confirmada=mascara_confirmada,
        diagnostico=diagnostico,
        estructura_fuera_mascara_deg=estructura_fuera,
    )


# Nombre corto para consumidores que ya trabajan con un perfil de 360 bins.
analizar_perfil = compensar_oclusion_lidar


__all__ = (
    "CONFIGURACION_MEDIDA",
    "SIN_DATO",
    "CoberturaOclusionLidar",
    "ConfiguracionOclusionLidar",
    "ResultadoOclusionLidar",
    "analizar_perfil",
    "compensar_oclusion_lidar",
    "configuracion_desde_mapa",
)
