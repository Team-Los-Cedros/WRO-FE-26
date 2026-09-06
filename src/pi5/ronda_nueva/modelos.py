"""Tipos compartidos de la ronda del Pi 5.

CONVENCIONES (las mismas que ya usan el LiDAR y la Pico del robot, para que
todos los numeros medidos en las sesiones anteriores sigan valiendo):

* ``x`` positivo: a la derecha del robot.  ``y`` positivo: hacia adelante.
* Todas las distancias en milimetros; todos los angulos en grados.
* ``bearing`` positivo: hacia la derecha.
* ``angulo`` de direccion positivo: ruedas hacia la IZQUIERDA.
* ``rumbo`` de la IMU positivo: giro a la izquierda (acumulado, sin envolver).

MARCO DEL SEGMENTO
La pista se recorre como cuatro rectas encadenadas.  En vez de construir un
mapa metrico global -- que exigiria SLAM y no cabe en el presupuesto de la
ronda -- cada recta se describe con dos numeros que el LiDAR mide directo:

* ``avance_mm``: distancia perpendicular al muro que se tiene enfrente.  Va
  bajando conforme el robot avanza por la recta.
* ``offset_mm``: distancia perpendicular al muro EXTERIOR de la pista.  Es la
  unica variable que el control necesita mover para trazar el recorrido.

Esa pareja identifica una posicion dentro de la recta con precision de
centimetros y sin acumular deriva, que es todo lo que hace falta para decidir
por que lado se pasa cada pilar.
"""

from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple


Punto2D = Tuple[float, float]
MuestraLidar = Tuple[float, float]      # (angulo_deg, distancia_mm)

ROJO = "ROJO"
VERDE = "VERDE"
MAGENTA = "MAGENTA"

EXTERIOR = "EXTERIOR"
INTERIOR = "INTERIOR"

IZQUIERDA = -1
DERECHA = 1


@dataclass(frozen=True)
class BarridoLidar:
    """Barrido crudo con el instante en que el driver termino de recibirlo.

    El timestamp lo pone el driver al leer del puerto, no el consumidor: es lo
    unico que permite saber si el control esta decidiendo sobre una foto
    reciente de la pista o sobre una que envejecio en una cola.
    """

    timestamp: float
    muestras: Sequence[MuestraLidar]


@dataclass(frozen=True)
class Recta:
    """Una pared ajustada como recta en el marco del robot.

    ``distancia_mm`` es la perpendicular desde el origen del robot hasta la
    recta y ``angulo_deg`` el angulo de esa perpendicular respecto al eje del
    robot.  Guardar la recta entera -- y no solo el minimo del sector -- es lo
    que permite medir la distancia desde un punto cualquiera (por ejemplo, un
    pilar) hasta la pared sin volver a mirar el barrido.
    """

    distancia_mm: float
    angulo_deg: float
    residuo_mm: float
    puntos: int
    calidad: float

    def distancia_desde(self, x_mm: float, y_mm: float) -> float:
        """Perpendicular desde un punto del marco del robot hasta la recta."""

        import math

        rad = math.radians(self.angulo_deg)
        return self.distancia_mm - (x_mm * math.sin(rad) + y_mm * math.cos(rad))


@dataclass(frozen=True)
class MapaParedes:
    """Las cuatro paredes vistas desde el robot en un barrido."""

    timestamp: float
    frontal: Optional[Recta] = None
    trasera: Optional[Recta] = None
    izquierda: Optional[Recta] = None
    derecha: Optional[Recta] = None
    # Las minimas de cada sector NO incluyen los puntos de los objetos: son
    # distancias a ESTRUCTURA (muros).  Un pilar que el robot va a rebasar por
    # el costado no puede disparar una emergencia lateral.
    frontal_min_mm: float = float("inf")
    trasera_min_mm: float = float("inf")
    izquierda_min_mm: float = float("inf")
    derecha_min_mm: float = float("inf")
    # Espacio libre en linea recta dentro del ancho del robot, contando TODO
    # (muros y objetos).  Es lo que hay que mirar para frenar: dice si se
    # puede seguir avanzando, no si hay algo cerca.
    corredor_mm: float = float("inf")
    # Rumbo del punto que CIERRA el corredor.  Es diagnostico puro: si el
    # corredor se cierra a 150 mm con los dos muros a 900, el eco no es de la
    # pista y el rumbo dice de que parte del propio robot viene.
    corredor_deg: float = float("nan")
    # El MISMO corredor pero contando solo ESTRUCTURA.  Existe porque las dos
    # preguntas que se le hacian al corredor no son la misma: para FRENAR hay
    # que ver el pilar que se va a rebasar, pero para declarar una EMERGENCIA
    # y retroceder no, porque rebasar un pilar de cerca es la maniobra normal
    # de la ronda, no un atasco.  Medido el 06-09 sobre 13 corridas: 111 de
    # los 112 retrocesos los disparo el corredor cerrandose a 138 mm de
    # mediana mientras la estructura estaba a 552.
    corredor_estructura_mm: float = float("inf")
    corredor_estructura_deg: float = float("nan")
    puntos_totales: int = 0


@dataclass(frozen=True)
class ObjetoLidar:
    """Grupo compacto de puntos que no pertenece a ninguna pared."""

    timestamp: float
    x_mm: float
    y_mm: float
    distancia_mm: float
    bearing_deg: float
    ancho_mm: float
    puntos: int


@dataclass(frozen=True)
class DeteccionPilar:
    """Un pilar visto en un instante, ya en milimetros del marco del robot.

    ``fuente`` dice de donde salio la posicion: ``CAMARA`` (punto de contacto
    con el suelo proyectado por la homografia), ``LIDAR`` (centroide del
    cluster) o ``FUSION`` (los dos coincidieron).  Se guarda porque las dos
    fuentes fallan en rangos distintos y saber cual hablo cambia cuanto hay
    que fiarse del numero.
    """

    timestamp: float
    color: str
    x_mm: float
    y_mm: float
    fuente: str
    confianza: float
    ancho_px: int = 0
    alto_px: int = 0
    distancia_por_altura_mm: float = 0.0
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)

    @property
    def distancia_mm(self) -> float:
        import math

        return math.hypot(self.x_mm, self.y_mm)


@dataclass(frozen=True)
class ParedMagenta:
    """Uno de los muros del cajon de parqueo visto por la camara."""

    timestamp: float
    x_mm: float
    y_mm: float
    ancho_px: int
    alto_px: int
    confianza: float


@dataclass(frozen=True)
class LineaPiso:
    """Linea azul o naranja del piso, proyectada al suelo."""

    timestamp: float
    color: str
    y_mm: float
    x_mm: float
    area_px: int


@dataclass(frozen=True)
class PaqueteVision:
    timestamp: float
    pilares: Tuple[DeteccionPilar, ...] = ()
    magenta: Tuple[ParedMagenta, ...] = ()
    lineas: Tuple[LineaPiso, ...] = ()
    duracion_ms: float = 0.0
    suelo_px: int = 0


@dataclass(frozen=True)
class PoseCarril:
    """Donde esta el robot dentro de la recta que recorre ahora mismo."""

    timestamp: float
    segmento: int
    avance_mm: float
    offset_mm: float
    rumbo_error_deg: float
    avance_valido: bool = False
    offset_valido: bool = False
    ancho_carril_mm: float = 1000.0
    fuente_avance: str = ""
    fuente_offset: str = ""


@dataclass(frozen=True)
class Casilla:
    """Una de las doce posiciones de pilar de la pista.

    ``segmento`` 0..3 y ``indice`` 0..2, donde 0 es la posicion mas cercana al
    muro frontal de esa recta (o sea, la primera que el robot encuentra).
    """

    segmento: int
    indice: int


@dataclass
class EntradaMapa:
    """Lo que el mapa recuerda de una casilla, con sus votos."""

    votos: Dict[str, float] = field(default_factory=dict)
    lado: Optional[str] = None
    offset_mm: float = 0.0
    avance_mm: float = 0.0
    observaciones: int = 0
    ultima_vista_s: float = 0.0
    vuelta_ultima_vista: int = -1

    @property
    def color(self) -> Optional[str]:
        if not self.votos:
            return None
        color, peso = max(self.votos.items(), key=lambda par: par[1])
        return color if peso > 0.0 else None

    @property
    def confianza(self) -> float:
        total = sum(self.votos.values())
        if total <= 0.0:
            return 0.0
        return max(self.votos.values()) / total


@dataclass(frozen=True)
class NodoRuta:
    """Un punto del carril deseado: a que distancia del muro exterior hay que
    ir cuando queden ``avance_mm`` hasta el muro de enfrente."""

    avance_mm: float
    offset_mm: float
    motivo: str = ""


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
class Consigna:
    """Lo unico que sale del cerebro hacia el hardware."""

    velocidad: int
    angulo: float
    estado: str
    razon: str = ""
    terminado: bool = False
    verificado: bool = False
