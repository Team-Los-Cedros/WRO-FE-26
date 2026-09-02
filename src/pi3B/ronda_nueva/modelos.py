"""Tipos compartidos de la ronda nueva.

Las convenciones son las mismas del LiDAR existente:

* x positivo: derecha del robot.
* y positivo: frente del robot.
* bearing positivo: hacia la derecha.
* direccion positiva: giro de las ruedas hacia la izquierda.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple


BBox = Tuple[int, int, int, int]
Point2D = Tuple[float, float]
MuestraLidar = Tuple[float, float]      # (angulo_deg, distancia_mm)


@dataclass(frozen=True)
class BarridoLidar:
    """Barrido crudo con el instante real en que se termino de recibir.

    El timestamp lo pone el driver al leer del puerto, no el consumidor: es lo
    unico que permite saber si el control esta decidiendo sobre una foto
    reciente de la pista o sobre una que ya envejecio en una cola.
    """

    timestamp: float
    muestras: Sequence[MuestraLidar]


@dataclass(frozen=True)
class DeteccionVisual:
    timestamp: float
    color: str
    bearing_deg: float
    centro_px: Point2D
    bbox: BBox
    area_ratio: float
    bottom_ratio: float
    confianza: float
    soporte_suelo: float


@dataclass(frozen=True)
class PaqueteVision:
    timestamp: float
    detecciones: Tuple[DeteccionVisual, ...]
    duracion_ms: float = 0.0


@dataclass(frozen=True)
class ObjetoLidar:
    timestamp: float
    x_mm: float
    y_mm: float
    distancia_mm: float
    bearing_deg: float
    ancho_mm: float
    puntos: int


@dataclass(frozen=True)
class ParedEstimada:
    distancia_mm: float
    pendiente: float
    rumbo_error_deg: float
    residuo_mm: float
    puntos: int
    calidad: float


@dataclass(frozen=True)
class Corredor:
    timestamp: float
    frontal_mm: float
    frontal_muro_mm: float
    trasera_mm: float
    izquierda_mm: float
    derecha_mm: float
    trasera_izquierda_mm: float
    trasera_derecha_mm: float
    error_lateral_mm: float
    error_rumbo_muro_deg: float
    calidad_pared: float
    pared_izquierda: Optional[ParedEstimada] = None
    pared_derecha: Optional[ParedEstimada] = None
    izquierda_valida: bool = True
    derecha_valida: bool = True
    trasera_valida: bool = True
    trasera_izquierda_valida: bool = True
    trasera_derecha_valida: bool = True
    cobertura_trasera: float = 1.0
    cobertura_trasera_izquierda: float = 1.0
    cobertura_trasera_derecha: float = 1.0
    diagnostico_oclusion: str = ""
    mascara_oclusion_confirmada: bool = False
    estructura_fuera_mascara_deg: Tuple[int, ...] = ()


@dataclass(frozen=True)
class HuecoParqueo:
    timestamp: float
    lado: int
    borde_trasero_y_mm: float
    borde_delantero_y_mm: float
    centro_y_mm: float
    separacion_mm: float
    distancia_lateral_mm: float
    confianza: float


@dataclass(frozen=True)
class MedidasParqueo:
    """Todo lo que la FSM de parqueo necesita medir, en un solo paquete.

    Antes estos valores viajaban como una docena de argumentos sueltos que
    ``control_ruta`` armaba a mano barrido a barrido. Agruparlos permite que
    ``ejecutar_estacionamiento`` sea la unica frontera entre el robot y la
    maquina de estados, y que anadir una fuente nueva (el ultrasonido) no
    obligue a tocar la firma de nadie mas.

    Cada distancia viaja con su bandera de validez porque un numero solo no
    dice si se midio algo: el centinela SIN_DATO del LiDAR y una lectura real
    de 8 m son el mismo float.
    """

    frontal_mm: float
    trasera_mm: float
    trasera_valida: bool = False
    cobertura_trasera: float = 0.0
    lateral_mm: Optional[float] = None
    lateral_valida: bool = False
    trasera_izquierda_mm: Optional[float] = None
    trasera_derecha_mm: Optional[float] = None
    trasera_izquierda_valida: bool = False
    trasera_derecha_valida: bool = False
    cobertura_trasera_izquierda: float = 0.0
    cobertura_trasera_derecha: float = 0.0
    # None significa "sin evidencia", nunca "libre". Ver EnlacePicoNuevo.
    ultrasonido_trasero_mm: Optional[float] = None


@dataclass(frozen=True)
class TrackObstaculo:
    track_id: int
    timestamp: float
    x_mm: float
    y_mm: float
    distancia_mm: float
    bearing_deg: float
    color: Optional[str]
    confianza_color: float
    impactos_lidar: int
    impactos_color: int
    edad_s: float
    confirmado: bool


@dataclass(frozen=True)
class Consigna:
    velocidad: int
    angulo: float
    estado: str
    razon: str = ""
    terminado: bool = False
    verificado: bool = False


@dataclass(frozen=True)
class ResultadoParqueo:
    velocidad: int
    angulo: float
    estado: str
    razon: str = ""
    terminado: bool = False
    verificado: bool = False
