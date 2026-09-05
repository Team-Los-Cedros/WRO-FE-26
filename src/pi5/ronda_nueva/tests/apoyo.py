"""Fixtures sinteticas: una pista de juguete con la que probar sin robot.

Todo lo que hay aqui es geometria pura.  El objetivo es poder ejercitar la
percepcion, el mapa y el planificador en el escritorio, porque el robot es un
recurso escaso y compartido y porque un fallo que se reproduce en un test se
arregla en minutos en vez de en una tarde de pista.
"""

from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple


# --- La pista, en el marco del mundo -------------------------------------
# Origen en el centro del campo.  X a la derecha, Y hacia el fondo.
LADO_EXTERIOR_MM = 3000.0
LADO_INTERIOR_MM = 1000.0
ANCHO_CARRIL_MM = (LADO_EXTERIOR_MM - LADO_INTERIOR_MM) / 2.0   # 1000 mm


def _interseccion_rayo_segmento(
    origen: Tuple[float, float],
    direccion: Tuple[float, float],
    a: Tuple[float, float],
    b: Tuple[float, float],
) -> float:
    """Distancia del rayo al segmento a-b, o inf si no lo corta."""

    dx, dy = direccion
    ax, ay = a
    bx, by = b
    ex, ey = bx - ax, by - ay
    denominador = dx * ey - dy * ex
    if abs(denominador) < 1e-9:
        return float("inf")
    ox, oy = origen
    t = ((ax - ox) * ey - (ay - oy) * ex) / denominador
    u = ((ax - ox) * dy - (ay - oy) * dx) / denominador
    if t <= 1e-6 or not -1e-9 <= u <= 1.0 + 1e-9:
        return float("inf")
    return t


def _segmentos_rectangulo(
    cx: float, cy: float, ancho: float, alto: float
) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    hx, hy = ancho / 2.0, alto / 2.0
    esquinas = [
        (cx - hx, cy - hy),
        (cx + hx, cy - hy),
        (cx + hx, cy + hy),
        (cx - hx, cy + hy),
    ]
    return [(esquinas[i], esquinas[(i + 1) % 4]) for i in range(4)]


def paredes_pista() -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Muro exterior de 3000 mm y bloque interior de 1000 mm."""

    return _segmentos_rectangulo(0.0, 0.0, LADO_EXTERIOR_MM, LADO_EXTERIOR_MM) + (
        _segmentos_rectangulo(0.0, 0.0, LADO_INTERIOR_MM, LADO_INTERIOR_MM)
    )


def barrido_sintetico(
    x_mm: float,
    y_mm: float,
    rumbo_deg: float,
    pilares: Sequence[Tuple[float, float, float]] = (),
    paso_deg: float = 0.8,
    alcance_mm: float = 4000.0,
    ruido_mm: float = 0.0,
    semilla: int = 0,
) -> List[Tuple[float, float]]:
    """Barrido del LiDAR desde una pose del mundo, con oclusion correcta.

    ``pilares`` son tuplas (x, y, lado) en el marco del mundo.  El rayo se
    queda con el primer objeto que encuentra, asi que un pilar tapa la pared
    que tiene detras -- que es justo el efecto que rompia la version anterior
    de esta fixture.
    """

    import random

    aleatorio = random.Random(semilla)
    obstaculos = list(paredes_pista())
    for px, py, lado in pilares:
        obstaculos.extend(_segmentos_rectangulo(px, py, lado, lado))

    muestras: List[Tuple[float, float]] = []
    angulo = 0.0
    while angulo < 360.0:
        # El barrido esta en el marco del robot: 0 grados es hacia adelante.
        rad = math.radians(angulo + rumbo_deg)
        direccion = (math.sin(rad), math.cos(rad))
        mejor = float("inf")
        for a, b in obstaculos:
            distancia = _interseccion_rayo_segmento((x_mm, y_mm), direccion, a, b)
            if distancia < mejor:
                mejor = distancia
        if mejor <= alcance_mm:
            if ruido_mm > 0.0:
                mejor += aleatorio.gauss(0.0, ruido_mm)
            muestras.append((angulo, mejor))
        angulo += paso_deg
    return muestras


def pose_en_carril(
    segmento: int, avance_mm: float, offset_mm: float, error_rumbo_deg: float = 0.0
) -> Tuple[float, float, float]:
    """Convierte (segmento, avance, offset) a una pose del mundo.

    ``segmento`` 0 = el robot mira hacia +Y.  Cada incremento gira 90 grados
    en sentido horario visto desde arriba, que es el orden en que el robot
    encadena las rectas al girar a la derecha.  ``avance_mm`` es la distancia
    al muro exterior que tiene enfrente y ``offset_mm`` la distancia al muro
    exterior de su lado izquierdo.
    """

    borde = LADO_EXTERIOR_MM / 2.0
    # En el segmento 0 (mirando a +Y) el muro de enfrente esta en y = +borde y
    # el exterior izquierdo en x = -borde.
    x = -borde + offset_mm
    y = borde - avance_mm
    rumbo = 0.0

    # Girar la escena -90 grados lleva a un robot que miraba a +Y a mirar a
    # +X, que en la convencion del barrido (rumbo positivo = hacia +X) es
    # rumbo +90.
    for _ in range(segmento % 4):
        x, y = y, -x
        rumbo += 90.0
    return x, y, rumbo + error_rumbo_deg


def config_minima() -> Dict:
    """Configuracion suficiente para instanciar la percepcion y el control."""

    return {
        "lidar": {},
        "track": {
            "lane_width_mm": ANCHO_CARRIL_MM,
            "segment_length_mm": LADO_EXTERIOR_MM,
            "pillar_width_mm": 100.0,
            "pillar_height_mm": 100.0,
        },
        "control": {},
        "parking": {},
    }


# --- Disposiciones oficiales de obstaculos --------------------------------
# Sacadas del randomizer oficial de la temporada (las 36 cartas del sorteo,
# de las que 28 son distintas).  Dos datos que cambian el diseño del
# planificador y que no son evidentes leyendo solo el reglamento:
#
# * Cuando una recta lleva DOS señales, son siempre p1 y p3 -- nunca dos
#   adyacentes.  O sea 953 mm entre ellas, no 480.
# * Las dos filas laterales estan a 380 y 574 mm del muro exterior, o sea
#   solo 194 mm una de otra.
#
# Sin esto se dimensiona el planificador para un caso que el sorteo no puede
# producir y se acaba pidiendo maniobras que el chasis no traza.
POS_P1, POS_P2, POS_P3 = 2000.0, 1500.0, 1000.0
FILA_EXTERIOR_MM, FILA_INTERIOR_MM = 380.0, 574.0


def disposiciones_oficiales():
    """(pilares, nombre) para las 28 disposiciones distintas del sorteo."""

    salida = []
    for posicion, etiqueta in ((POS_P1, "p1"), (POS_P2, "p2"), (POS_P3, "p3")):
        for offset, fila in ((FILA_EXTERIOR_MM, "E"), (FILA_INTERIOR_MM, "I")):
            for color in ("ROJO", "VERDE"):
                salida.append(([(posicion, offset, color)], f"1:{etiqueta}{fila}{color[0]}"))
    for offset_a, fila_a in ((FILA_EXTERIOR_MM, "E"), (FILA_INTERIOR_MM, "I")):
        for color_a in ("ROJO", "VERDE"):
            for offset_b, fila_b in ((FILA_EXTERIOR_MM, "E"), (FILA_INTERIOR_MM, "I")):
                for color_b in ("ROJO", "VERDE"):
                    salida.append(
                        (
                            [(POS_P1, offset_a, color_a), (POS_P3, offset_b, color_b)],
                            f"2:{fila_a}{color_a[0]}-{fila_b}{color_b[0]}",
                        )
                    )
    return salida


def _distancia_punto_rectangulo(
    px: float,
    py: float,
    cx: float,
    cy: float,
    rumbo_rad: float,
    largo: float = 222.0,
    ancho: float = 125.0,
) -> float:
    """Distancia de un punto al rectangulo del robot, en el marco de la recta.

    Se mide contra el RECTANGULO y no contra el centro porque el robot cruza
    el carril en diagonal: a 30 grados su silueta ocupa lateralmente casi el
    doble que parado, y esa es justo la situacion en la que roza el poste.
    """

    dx, dy = px - cx, py - cy
    # El avance decrece hacia adelante, de ahi los signos.
    ux, uy = -math.cos(rumbo_rad), math.sin(rumbo_rad)
    vx, vy = -uy, ux
    fuera_largo = max(abs(dx * ux + dy * uy) - largo / 2.0, 0.0)
    fuera_ancho = max(abs(dx * vx + dy * vy) - ancho / 2.0, 0.0)
    return math.hypot(fuera_largo, fuera_ancho)


def simular_recta(
    planificador,
    sentido: int,
    pilares,
    velocidad_pwm: int = 55,
    dt: float = 0.05,
    offset_inicial: float = 500.0,
    avance_inicial: float = 2900.0,
    mm_s_por_pwm: float = 4.0,
    ruido_pilar_mm: float = 0.0,
    ruido_pose_mm: float = 0.0,
    semilla: int = 0,
):
    """Modelo de bicicleta recorriendo una recta con el planificador al mando.

    Devuelve la traza [(avance, offset, rumbo_deg, mando, objetivo)].  Es el
    banco donde se afinaron las ganancias, y donde aparecieron tres errores de
    geometria que en pista habrian costado una tarde cada uno.
    """

    import random

    from ..modelos import PoseCarril

    aleatorio = random.Random(semilla)
    avance, offset, rumbo = avance_inicial, offset_inicial, 0.0
    traza = []
    for _ in range(int(90.0 / dt)):
        vistos = [
            (
                a + aleatorio.gauss(0.0, ruido_pilar_mm),
                o + aleatorio.gauss(0.0, ruido_pilar_mm),
                c,
            )
            for a, o, c in pilares
        ]
        pose = PoseCarril(
            timestamp=0.0,
            segmento=0,
            avance_mm=avance + aleatorio.gauss(0.0, ruido_pose_mm),
            offset_mm=offset + aleatorio.gauss(0.0, ruido_pose_mm),
            rumbo_error_deg=math.degrees(rumbo),
            avance_valido=True,
            offset_valido=True,
        )
        ruta = planificador.construir_ruta(pose, sentido, vistos)
        mando, objetivo, _error = planificador.direccion(
            pose, ruta, sentido, velocidad_pwm
        )
        traza.append((avance, offset, math.degrees(rumbo), mando, objetivo))

        rueda = math.radians(planificador.conversor.a_rueda(mando))
        velocidad = velocidad_pwm * mm_s_por_pwm
        avance -= velocidad * math.cos(rumbo) * dt
        offset -= sentido * velocidad * math.sin(rumbo) * dt
        rumbo += velocidad / 136.0 * math.tan(rueda) * dt
        if avance < 450.0:
            break
    return traza


def evaluar_paso(traza, sentido: int, pilares, medio_pilar_mm: float = 50.0):
    """(holgura minima al poste en mm, lista de fallos) de una traza."""

    peor = float("inf")
    fallos = []
    for avance_pilar, offset_pilar, color in pilares:
        holgura = min(
            _distancia_punto_rectangulo(
                avance_pilar,
                offset_pilar,
                avance,
                offset,
                -sentido * math.radians(rumbo),
            )
            - medio_pilar_mm
            for avance, offset, rumbo, _m, _o in traza
        )
        cercano = min(traza, key=lambda paso: abs(paso[0] - avance_pilar))
        # Rojo se rebasa por su derecha, verde por su izquierda; el offset
        # crece hacia la derecha del robot solo cuando sentido > 0.
        debe_ser_mayor = (color == "ROJO") == (sentido > 0)
        lado_ok = cercano[1] > offset_pilar if debe_ser_mayor else cercano[1] < offset_pilar
        peor = min(peor, holgura)
        if not lado_ok:
            fallos.append((color, int(avance_pilar), "lado incorrecto"))
        elif holgura < 10.0:
            fallos.append((color, int(avance_pilar), f"holgura {holgura:.0f} mm"))
    return peor, fallos
