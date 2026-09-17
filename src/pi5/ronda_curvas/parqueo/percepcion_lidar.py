"""Percepcion LiDAR: las cuatro paredes como rectas, objetos y hueco de bahia.

QUE CAMBIA RESPECTO A LA VERSION DE LA PI 3B
Antes cada lado se resumia en un numero (la distancia minima o la mediana del
sector).  Aqui cada pared se guarda como una RECTA completa, con su
perpendicular y su angulo.  La diferencia importa porque con la recta se puede
preguntar "a que distancia de la pared exterior esta ESE pilar" sin volver a
mirar el barrido, y esa pregunta es la que alimenta el mapa de la pista.

QUE SE CONSERVA, PORQUE COSTO PISTA MEDIRLO
* Segmentacion ABD lineal con union circular en 359 -> 0.
* Mascara de sectores ciegos del mastil (163-195 grados).
* Filtro del eco de la propia rueda: el LiDAR se ve el neumatico al girar y
  eso disparaba emergencias falsas sin parar.  El eco SE ALEJA con el angulo
  del volante (49-51 mm a +17 grados, 91-107 a +25), asi que el umbral no
  puede ser fijo: se enmascara el sector del lado hacia el que apunta el
  servo, en proporcion al angulo.
* ``_partir_en_rectas`` (iterative end-point fit) antes del PCA al buscar el
  hueco de parqueo.  Sin el, la cara del delimitador y el muro llegan fundidos
  en un cluster en L y el detector daba 0 emparejamientos en 22 barridos; con
  el, 9 de 10.  Se aplica SOLO en la busqueda del hueco.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from statistics import median
from typing import Any, Iterable, List, Mapping, Optional, Sequence, Tuple

from .modelos import (
    HuecoParqueo,
    MapaParedes,
    ObjetoLidar,
    Recta,
)


PuntoPolar = Tuple[float, float]
PuntoXY = Tuple[float, float]

SIN_DATO_MM = float("inf")


def _limitar(valor: float, minimo: float, maximo: float) -> float:
    return max(minimo, min(maximo, valor))


def _es_numero_valido(valor: Any) -> bool:
    try:
        numero = float(valor)
    except (TypeError, ValueError):
        return False
    return math.isfinite(numero)


def _polar_a_xy(angulo_deg: float, distancia_mm: float) -> PuntoXY:
    rad = math.radians(angulo_deg)
    return distancia_mm * math.sin(rad), distancia_mm * math.cos(rad)


def _en_sector(angulo_deg: float, limites: Sequence[float]) -> bool:
    inicio = float(limites[0]) % 360.0
    fin = float(limites[1]) % 360.0
    angulo = angulo_deg % 360.0
    if inicio <= fin:
        return inicio <= angulo <= fin
    return angulo >= inicio or angulo <= fin


def normalizar_barrido(
    scan: Iterable[Sequence[float]], distancia_max_mm: float
) -> List[PuntoPolar]:
    """Filtra invalidos conservando el orden angular con que llego el barrido."""

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
    umbral = min(anterior[1], actual[1]) * factor_abd + offset_abd_mm
    return abs(actual[1] - anterior[1]) <= umbral


def segmentar(
    puntos: Sequence[PuntoPolar],
    factor_abd: float = 0.04,
    offset_abd_mm: float = 40.0,
    max_salto_angular_deg: float = 4.0,
    min_puntos: int = 2,
) -> List[List[PuntoPolar]]:
    """Segmentacion angular O(n) por salto radial adaptativo.

    No se ordena el barrido: hacerlo añadiria O(n log n) por revolucion y el
    driver ya lo entrega en orden de adquisicion.
    """

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

    # Un objeto a caballo del cruce 359 -> 0 queda partido en los dos extremos.
    if len(clusters) >= 2:
        primero, ultimo = clusters[0], clusters[-1]
        if _salto_compatible(
            ultimo[-1], primero[0], factor_abd, offset_abd_mm, max_salto_angular_deg
        ):
            clusters[0] = ultimo + primero
            clusters.pop()
    return clusters


@dataclass(frozen=True)
class _Segmento:
    x_mm: float
    y_mm: float
    longitud_mm: float
    alineacion: float
    residuo_mm: float
    puntos: int

    @property
    def centro_y_mm(self) -> float:
        return self.y_mm


def _segmento_pca(cluster: Sequence[PuntoPolar]) -> Optional[_Segmento]:
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
    residuos = [abs(-(x - mx) * uy + (y - my) * ux) for x, y in puntos]
    return _Segmento(
        x_mm=mx,
        y_mm=my,
        longitud_mm=max(proyecciones) - min(proyecciones),
        alineacion=abs(ux),
        residuo_mm=sum(residuos) / n,
        puntos=n,
    )


def ajustar_recta(
    puntos: Sequence[PuntoXY], recorte_residuo: float = 2.5
) -> Optional[Recta]:
    """Ajuste total (PCA) con una ronda de recorte de atipicos.

    Se usa PCA y no minimos cuadrados sobre una variable porque una pared vista
    de frente es casi horizontal en el marco del robot y otra vista de lado es
    casi vertical: cualquier ajuste ``x = a*y + b`` revienta en uno de los dos
    casos.  El PCA no tiene direccion privilegiada.

    Una sola ronda de recorte basta: la contaminacion tipica es un pilar
    pegado a la pared o el reflejo de una esquina, no la mitad del sector.
    """

    if len(puntos) < 2:
        return None
    for _ in range(2):
        n = len(puntos)
        if n < 2:
            return None
        mx = sum(p[0] for p in puntos) / n
        my = sum(p[1] for p in puntos) / n
        cxx = sum((p[0] - mx) ** 2 for p in puntos) / n
        cyy = sum((p[1] - my) ** 2 for p in puntos) / n
        cxy = sum((p[0] - mx) * (p[1] - my) for p in puntos) / n
        if cxx + cyy < 1.0:
            return None
        angulo = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
        ux, uy = math.cos(angulo), math.sin(angulo)
        nx, ny = -uy, ux
        residuos = [abs((p[0] - mx) * nx + (p[1] - my) * ny) for p in puntos]
        residuo_medio = sum(residuos) / n

        corte = recorte_residuo * max(residuo_medio, 1.0)
        conservados = [p for p, r in zip(puntos, residuos) if r <= corte]
        if len(conservados) == n or len(conservados) < max(2, n // 2):
            break
        puntos = conservados

    distancia = mx * nx + my * ny
    if distancia < 0.0:
        nx, ny, distancia = -nx, -ny, -distancia
    calidad = _limitar(1.0 - residuo_medio / 60.0, 0.0, 1.0) * _limitar(n / 12.0, 0.0, 1.0)
    return Recta(
        distancia_mm=float(distancia),
        angulo_deg=float(math.degrees(math.atan2(nx, ny))),
        residuo_mm=float(residuo_medio),
        puntos=int(n),
        calidad=float(calidad),
    )


class PercepcionLidar:
    """Convierte un barrido crudo en paredes, objetos y candidatos de bahia."""

    def __init__(self, config: Optional[Mapping[str, Any]] = None):
        self.config = dict((config or {}).get("lidar", {}))
        self.reiniciar()

    def reiniciar(self) -> None:
        self._hueco_previo = None
        self._hueco_confirmado: Optional[HuecoParqueo] = None
        self._hueco_repeticiones = 0
        self._hueco_lado = 0
        self._hueco_timestamp = None

    def _cfg(self, nombre: str, defecto: Any) -> Any:
        return self.config.get(nombre, defecto)

    # ------------------------------------------------------------ filtrado

    def _mascarar(
        self, puntos: Sequence[PuntoPolar], angulo_servo_deg: float
    ) -> List[PuntoPolar]:
        """Quita el mastil y el eco de la propia rueda.

        El eco se aleja con el volante, asi que el radio que se descarta crece
        con ``|angulo_servo_deg|`` en vez de ser una constante.  Solo se aplica
        al lado hacia el que giran las ruedas, que es el unico donde el
        neumatico entra en el plano del barrido.
        """

        ciegos = [tuple(s) for s in self._cfg("blind_sectors_deg", ()) or ()]
        # Recta ajustada a las dos medidas de pista: 49-51 mm de eco con el
        # servo a 17 grados y 91-107 a 25.  Salen 7,0 mm por grado con -68 mm
        # de ordenada, o sea que por debajo de ~10 grados el neumatico ni
        # siquiera entra en el plano del barrido y no hay nada que enmascarar.
        eco_base = float(self._cfg("self_echo_base_mm", -68.0))
        eco_por_grado = float(self._cfg("self_echo_mm_per_deg", 7.0))
        eco_margen = float(self._cfg("self_echo_margin", 1.3))
        eco_max = float(self._cfg("self_echo_max_mm", 150.0))
        sector_eco_izq = tuple(self._cfg("self_echo_left_sector_deg", (250.0, 330.0)))
        sector_eco_der = tuple(self._cfg("self_echo_right_sector_deg", (30.0, 110.0)))

        radio_eco = _limitar(
            eco_margen * (eco_base + eco_por_grado * abs(float(angulo_servo_deg))),
            0.0,
            eco_max,
        )
        # angulo positivo = ruedas a la izquierda
        sector_eco = sector_eco_izq if angulo_servo_deg > 0.0 else sector_eco_der
        aplicar_eco = radio_eco > 1.0

        salida: List[PuntoPolar] = []
        for angulo, distancia in puntos:
            if any(_en_sector(angulo, sector) for sector in ciegos):
                continue
            if aplicar_eco and distancia <= radio_eco and _en_sector(angulo, sector_eco):
                continue
            salida.append((angulo, distancia))
        return salida

    # -------------------------------------------------------------- paredes

    def _puntos_sector(
        self, puntos: Sequence[PuntoPolar], sector: Sequence[float], max_mm: float
    ) -> List[PuntoXY]:
        return [
            _polar_a_xy(a, d)
            for a, d in puntos
            if d <= max_mm and _en_sector(a, sector)
        ]

    def _trasera_por_hombros(
        self, puntos: Sequence[PuntoPolar]
    ) -> Optional[Recta]:
        """Reconstruye el muro trasero cuando el mastil tapa su eje.

        El mastil ocupa 140..213 grados: no queda ningun haz util en el
        sector trasero central.  Los dos hombros visibles a 40..60 grados del
        eje trasero ven la misma pared oblicuamente; ajustarlos juntos permite
        recuperar su perpendicular sin inventar una lectura dentro de la zona
        ciega.  Si no hay evidencia suficiente se devuelve ``None``.
        """

        eje = float(self._cfg("rear_axis_deg", 180.0))
        offsets = tuple(self._cfg("rear_shoulder_offset_deg", (40.0, 60.0)))
        if len(offsets) != 2:
            return None
        minimo, maximo = (float(offsets[0]), float(offsets[1]))
        if not 0.0 < minimo <= maximo < 90.0:
            return None

        candidatos = []
        for angulo, distancia in puntos:
            diferencia = abs(((angulo - eje + 180.0) % 360.0) - 180.0)
            if minimo <= diferencia <= maximo:
                # Los laterales tambien cruzan estas ventanas cuando el robot
                # esta cerca de un borde.  La pared que queda detras tiene la
                # mayor componente sobre el eje trasero; no se usa la minima,
                # que seria precisamente el lateral proximo.
                axial = distancia * math.cos(math.radians(diferencia))
                candidatos.append((_polar_a_xy(angulo, distancia), axial))

        requeridos = max(2, int(self._cfg("rear_min_valid_points", 2)))
        if len(candidatos) < requeridos:
            return None
        fraccion = float(self._cfg("rear_shoulder_wall_fraction", 0.8))
        if not 0.0 < fraccion <= 1.0:
            return None
        axial_maximo = max(axial for _punto, axial in candidatos)
        hombros = [
            punto
            for punto, axial in candidatos
            if axial >= fraccion * axial_maximo
        ]
        if len(hombros) < requeridos:
            return None
        recta = ajustar_recta(hombros)
        if recta is None:
            return None
        ventana = float(self._cfg("wall_normal_window_deg", 42.0))
        if abs(abs(recta.angulo_deg) - 180.0) > ventana:
            return None
        if recta.residuo_mm > float(self._cfg("wall_max_residual_mm", 75.0)):
            return None
        return recta

    def _largo_minimo_mm(
        self, recta: Recta, lado_parqueo: int, largo_carrera_mm: float
    ) -> float:
        """Cuanto segmento se le exige a UNA recta para aceptarla como pared.

        MEDIDO EL 06-09 CON EL ROBOT COLOCADO A MANO EN LA POSE APARCADA: el
        muro de la bahia deja 38 puntos con 3,0 mm de residuo y una normal de
        -93,4 grados, pero ``paredes.izquierda`` salia ``None`` en los doce
        barridos.  Barriendo el umbral sobre ese mismo barrido: 220, 180, 150,
        120, 100 y 80 mm no encuentran nada; 60 SI, y devuelve 77,9 mm.  Con
        el flanco a 11 mm del muro los dos delimitadores tapan el resto de la
        pared, asi que exigir 220 mm de segmento ahi no es un umbral estricto,
        es una condicion imposible -- y sin lateral ni paralelo VERIFICAR no
        podia cerrar nunca, aunque la maniobra saliera perfecta.

        POR QUE NO SE BAJA EL UMBRAL A SECAS
        Los 220 mm son los que impiden que un pilar (100 mm) o un delimitador
        (200 mm de huella) pasen por pared durante la vuelta.  Aqui se relajan
        con dos candados: solo la pared del LADO de la bahia -- frontal y
        trasera conservan el umbral de carrera, que es donde un delimitador
        visto de canto haria dano durante la APROXIMACION -- y solo en
        proporcion a la distancia, porque el trozo de muro que cabe en un
        sector angular fijo crece con ella.  A 78 mm el segmento real medido
        estaba entre 60 y 80 mm, o sea 0,8-1,0 de la distancia; con 0,75 el
        umbral vuelve al valor de carrera a partir de 293 mm, ya fuera de la
        bahia (200 mm de profundidad).
        """

        if not lado_parqueo:
            return largo_carrera_mm
        ventana = float(self._cfg("wall_normal_window_deg", 42.0))
        referencia = 90.0 if lado_parqueo > 0 else -90.0
        if abs(recta.angulo_deg - referencia) > ventana:
            return largo_carrera_mm
        minimo = float(self._cfg("wall_min_length_parking_mm", 60.0))
        razon = float(self._cfg("wall_min_length_parking_ratio", 0.75))
        return _limitar(recta.distancia_mm * razon, minimo, largo_carrera_mm)

    def rectas_del_barrido(
        self, clusters: Sequence[Sequence[PuntoPolar]], lado_parqueo: int = 0
    ) -> List[Tuple[Recta, float, PuntoXY, PuntoXY]]:
        """Todos los tramos rectos del barrido, con su longitud.

        POR QUE NO SE AJUSTA POR SECTOR ANGULAR FIJO
        La primera version ajustaba cada pared sobre una ventana de angulos
        (frontal = +-40 grados, etc.).  En un carril de 1000 mm eso no
        funciona: con el robot a 400 mm del muro izquierdo y el frontal a
        1200, la ventana frontal se come 500 mm de muro lateral y el ajuste
        sale a 15 grados con 188 mm de residuo, o sea inservible.  Es un
        problema de geometria, no de umbrales: la ventana no sabe donde acaba
        una pared y empieza la otra.

        Partir el barrido en tramos rectos y clasificar CADA TRAMO por la
        direccion de su normal resuelve eso de raiz, y de paso entrega los
        delimitadores de la bahia con el mismo trabajo.
        """

        min_puntos = max(4, int(self._cfg("wall_min_points", 8)))
        max_desvio = float(self._cfg("wall_split_max_deviation_mm", 45.0))
        max_residuo = float(self._cfg("wall_max_residual_mm", 75.0))
        largo_min = float(self._cfg("wall_min_length_mm", 220.0))

        salida: List[Tuple[Recta, float, PuntoXY, PuntoXY]] = []
        for cluster in clusters:
            for pieza in self._partir_en_rectas(cluster, max_desvio, min_puntos):
                if len(pieza) < min_puntos:
                    continue
                puntos_xy = [_polar_a_xy(a, d) for a, d in pieza]
                recta = ajustar_recta(puntos_xy)
                if recta is None or recta.residuo_mm > max_residuo:
                    continue
                inicio, fin = puntos_xy[0], puntos_xy[-1]
                largo = math.hypot(fin[0] - inicio[0], fin[1] - inicio[1])
                if largo < self._largo_minimo_mm(recta, lado_parqueo, largo_min):
                    continue
                salida.append((recta, largo, inicio, fin))
        return salida

    def _corredor_libre_mm(
        self, puntos: Sequence[PuntoPolar]
    ) -> Tuple[float, float]:
        """Cuanto se puede avanzar en linea recta sin tocar nada.

        Se mira solo la banda del ancho del robot: un poste que queda al lado
        no estorba, y confundirlo con un obstaculo frontal es lo que hacia que
        el robot frenara -- y a veces retrocediera -- cada vez que veia un
        pilar que iba a rebasar limpiamente.
        """

        medio_ancho = float(self._cfg("corridor_half_width_mm", 95.0))
        # HUELLA PROPIA.  ``objetos()`` ya descartaba los puntos que caen
        # dentro del robot; el corredor no lo hacia, y se tragaba cualquier
        # eco con y>0 dentro de la banda aunque estuviera fisicamente encima
        # del chasis.  Medido el 06-09 en las corridas fix_01 y fix_02: un eco
        # a 55 mm y 54 grados a la derecha -- x 46, y 33, o sea 15 mm por
        # delante del parachoques y DENTRO del ancho -- cerraba el corredor y
        # disparaba el retroceso.  Que es del robot y no de la pista esta
        # probado por retroceso: en 13 de 13 episodios el corredor seguia en
        # 52-59 mm despues de 1,5 s alejandose, cuando un objeto real ya se
        # habria ido a mas de 150.
        huella_x = float(self._cfg("corridor_self_half_width_mm", 75.0))
        huella_y = float(self._cfg("corridor_self_front_mm", 45.0))
        mejor = SIN_DATO_MM
        rumbo = float("nan")
        for angulo, distancia in puntos:
            x, y = _polar_a_xy(angulo, distancia)
            if y <= 0.0 or abs(x) > medio_ancho:
                continue
            if y <= huella_y and abs(x) <= huella_x:
                continue
            if y < mejor:
                mejor = y
                rumbo = angulo
        return mejor, rumbo

    def paredes(
        self,
        clusters: Sequence[Sequence[PuntoPolar]],
        puntos: Sequence[PuntoPolar],
        timestamp: float,
        puntos_objeto: Optional[set] = None,
        lado_parqueo: int = 0,
    ) -> MapaParedes:
        """Clasifica los tramos rectos en frontal / trasera / izquierda / derecha.

        La clase la decide la direccion de la NORMAL de cada tramo: cerca de 0
        grados apunta al frente, +90 a la derecha, -90 a la izquierda, +-180
        atras.

        Dentro de cada clase gana la MAS CERCANA, no la mas larga.  Se probo al
        reves y estaba mal: desde el carril se ven a la vez el bloque interior
        (1000 mm de lado) y el muro exterior del otro extremo del campo (3000
        mm), y quedarse con el largo elegia una pared a 2,7 m teniendo otra a
        0,7 m.  El filtro de longitud minima ya descarta lo que podria
        confundirse con una pared -- un pilar mide 100 mm y un delimitador de
        bahia 200 --, asi que la cercania es el criterio seguro.
        """

        ventana = float(self._cfg("wall_normal_window_deg", 42.0))
        tolerancia = float(self._cfg("wall_straddle_tolerance_mm", 200.0))
        clases: dict = {"frontal": None, "derecha": None, "izquierda": None, "trasera": None}

        for recta, _largo, inicio, fin in self.rectas_del_barrido(
            clusters, lado_parqueo
        ):
            angulo = recta.angulo_deg
            if abs(angulo) <= ventana:
                nombre = "frontal"
            elif abs(abs(angulo) - 90.0) <= ventana:
                nombre = "derecha" if angulo > 0.0 else "izquierda"
            elif abs(angulo) >= 180.0 - ventana:
                nombre = "trasera"
            else:
                continue

            # El tramo tiene que cruzar el eje del robot para contar como "la
            # pared que tengo delante" o "la pared que tengo al lado".  Sin
            # esto, al empezar una recta la cara trasera del bloque interior
            # -- que esta a un lado, no enfrente -- se clasificaba como muro
            # frontal y daba un avance de 400 mm cuando el real era 2400.
            eje = 0 if nombre in ("frontal", "trasera") else 1
            minimo = min(inicio[eje], fin[eje])
            maximo = max(inicio[eje], fin[eje])
            if minimo > tolerancia or maximo < -tolerancia:
                continue

            actual = clases[nombre]
            if actual is None or recta.distancia_mm < actual.distancia_mm:
                clases[nombre] = recta

        sectores = {
            "frontal": tuple(self._cfg("front_sector_deg", (330.0, 30.0))),
            "trasera": tuple(self._cfg("rear_sector_deg", (150.0, 210.0))),
            "izquierda": tuple(self._cfg("left_sector_deg", (225.0, 315.0))),
            "derecha": tuple(self._cfg("right_sector_deg", (45.0, 135.0))),
        }
        max_mm = float(self._cfg("wall_max_distance_mm", 3200.0))
        # Las minimas describen ESTRUCTURA: se quitan los puntos que ya se
        # explicaron como objeto para que un pilar no cuente como pared.
        estructura = (
            puntos if not puntos_objeto else [p for p in puntos if p not in puntos_objeto]
        )
        minimos = {}
        for nombre, sector in sectores.items():
            muestras = self._puntos_sector(estructura, sector, max_mm)
            minimos[nombre] = min(
                (math.hypot(x, y) for x, y in muestras), default=SIN_DATO_MM
            )

        if clases["trasera"] is None:
            trasera_hombros = self._trasera_por_hombros(estructura)
            if trasera_hombros is not None:
                clases["trasera"] = trasera_hombros
                # El minimo angular central queda intencionadamente vacio.
                # La perpendicular ajustada con los hombros es la medida
                # segura que consume el parqueo y tambien la telemetria.
                minimos["trasera"] = trasera_hombros.distancia_mm

        corredor_mm, corredor_deg = self._corredor_libre_mm(puntos)
        # ``estructura`` ya tiene fuera los puntos que se explicaron como
        # objeto, asi que el segundo corredor sale del mismo recorrido sin
        # volver a mirar el barrido.
        corredor_est_mm, corredor_est_deg = self._corredor_libre_mm(estructura)
        return MapaParedes(
            timestamp=float(timestamp),
            frontal=clases["frontal"],
            trasera=clases["trasera"],
            izquierda=clases["izquierda"],
            derecha=clases["derecha"],
            frontal_min_mm=minimos["frontal"],
            trasera_min_mm=minimos["trasera"],
            izquierda_min_mm=minimos["izquierda"],
            derecha_min_mm=minimos["derecha"],
            corredor_mm=corredor_mm,
            corredor_deg=corredor_deg,
            corredor_estructura_mm=corredor_est_mm,
            corredor_estructura_deg=corredor_est_deg,
            puntos_totales=len(puntos),
        )

    # -------------------------------------------------------------- objetos

    def objetos(
        self,
        clusters: Sequence[Sequence[PuntoPolar]],
        timestamp: float,
        con_puntos: bool = False,
    ):
        """Clusters compactos que pueden ser un pilar.

        Se filtra por ANCHO FISICO, no por arco angular.  La version anterior
        exigia menos de 15 grados y por eso perdia el pilar a 216 mm de
        mediana: un poste de 100 mm ocupa 23 grados a esa distancia.  El ancho
        en milimetros no depende de la distancia, que es justo lo que se quiere
        cuando el objeto se acerca.
        """

        max_distancia = float(self._cfg("object_max_distance_mm", 2200.0))
        min_puntos = int(self._cfg("object_min_points", 3))
        max_ancho = float(self._cfg("object_max_width_mm", 240.0))
        min_ancho = float(self._cfg("object_min_width_mm", 20.0))
        huella_x = float(self._cfg("self_footprint_half_width_mm", 90.0))
        # La huella es ASIMETRICA porque el LiDAR va al ras del parachoques
        # (medido con regla el 04-09): el robot esta todo DETRAS de el.  Con
        # una ventana simetrica de +-250 mm se descartaba como "robot" todo lo
        # que hubiera hasta 250 mm POR DELANTE del morro, que es justo donde
        # esta un poste al que se va a chocar: el corredor libre si veia sus
        # puntos y frenaba, pero el planificador nunca llegaba a saber que
        # habia un objeto.  En las corridas 3 y 4 eso acabo en retroceso nueve
        # veces, siempre contra un bulto de 55 mm -- la anchura de una señal.
        huella_delante = float(self._cfg("self_footprint_front_mm", 0.0))
        huella_detras = float(
            self._cfg(
                "self_footprint_back_mm",
                self._cfg("self_footprint_half_length_mm", 250.0),
            )
        )

        salida: List[ObjetoLidar] = []
        usados: set = set()
        for cluster in clusters:
            if len(cluster) < min_puntos:
                continue
            distancias = [d for _, d in cluster]
            distancia = median(distancias)
            if distancia > max_distancia:
                continue
            segmento = _segmento_pca(cluster)
            if segmento is None:
                continue
            if not min_ancho <= segmento.longitud_mm <= max_ancho:
                continue
            # Lo que cae dentro de la HUELLA del propio robot es el robot: el
            # mastil a 154 grados se cuela por el borde del sector ciego y sale
            # como un objeto a 90 mm.  Ese mastil esta DETRAS, que es donde
            # esta el robot entero; por delante del parachoques no hay nada
            # nuestro que enmascarar.
            if (
                abs(segmento.x_mm) <= huella_x
                and -huella_detras <= segmento.y_mm <= huella_delante
            ):
                continue
            usados.update(cluster)
            bearing = math.degrees(math.atan2(segmento.x_mm, segmento.y_mm))
            salida.append(
                ObjetoLidar(
                    timestamp=float(timestamp),
                    x_mm=float(segmento.x_mm),
                    y_mm=float(segmento.y_mm),
                    distancia_mm=float(math.hypot(segmento.x_mm, segmento.y_mm)),
                    bearing_deg=float(bearing),
                    ancho_mm=float(segmento.longitud_mm),
                    puntos=int(segmento.puntos),
                )
            )
        salida.sort(key=lambda obj: obj.distancia_mm)
        if con_puntos:
            return tuple(salida), usados
        return tuple(salida)

    def filtrar_pegados_a_pared(
        self, objetos: Sequence[ObjetoLidar], paredes: MapaParedes
    ) -> Tuple[ObjetoLidar, ...]:
        """Quita los "objetos" que en realidad son un bulto de la pared.

        Medido con el robot quieto: 19 falsos positivos en 358 ciclos, casi
        todos trozos del muro izquierdo a 615 mm cuando el muro estaba a 540, y
        uno de ellos llego a mover el carril objetivo.
        """

        margen = float(self._cfg("object_min_wall_distance_mm", 70.0))
        rectas = [
            recta
            for recta in (
                paredes.frontal, paredes.trasera, paredes.izquierda, paredes.derecha
            )
            if recta is not None
        ]
        if not rectas:
            return tuple(objetos)
        return tuple(
            objeto
            for objeto in objetos
            if all(
                abs(recta.distancia_desde(objeto.x_mm, objeto.y_mm)) >= margen
                for recta in rectas
            )
        )

    # ---------------------------------------------------------- bahia

    def _partir_en_rectas(
        self,
        cluster: Sequence[PuntoPolar],
        max_desvio_mm: float,
        min_puntos: int,
        profundidad: int = 0,
    ) -> List[Sequence[PuntoPolar]]:
        """Parte un cluster por donde deja de ser una recta.

        El delimitador de la bahia y el muro que tiene detras llegan fundidos:
        entre la punta del delimitador y la pared no hay salto radial sino una
        rampa continua que el ABD no puede cortar.  Partir por la desviacion
        maxima al segmento extremo-extremo los separa limpios.
        """

        if len(cluster) < 2 * min_puntos or profundidad >= 4:
            return [cluster]

        inicio = _polar_a_xy(*cluster[0])
        fin = _polar_a_xy(*cluster[-1])
        dx, dy = fin[0] - inicio[0], fin[1] - inicio[1]
        largo = math.hypot(dx, dy)
        if largo < 1.0:
            return [cluster]
        ux, uy = dx / largo, dy / largo

        peor_indice, peor_desvio = -1, 0.0
        for indice in range(1, len(cluster) - 1):
            x, y = _polar_a_xy(*cluster[indice])
            desvio = abs(-(x - inicio[0]) * uy + (y - inicio[1]) * ux)
            if desvio > peor_desvio:
                peor_desvio, peor_indice = desvio, indice

        if peor_indice < 0 or peor_desvio <= max_desvio_mm:
            return [cluster]

        izquierda = cluster[: peor_indice + 1]
        derecha = cluster[peor_indice:]
        if len(izquierda) < min_puntos or len(derecha) < min_puntos:
            return [cluster]
        return self._partir_en_rectas(
            izquierda, max_desvio_mm, min_puntos, profundidad + 1
        ) + self._partir_en_rectas(derecha, max_desvio_mm, min_puntos, profundidad + 1)

    def buscar_hueco(
        self,
        clusters: Sequence[Sequence[PuntoPolar]],
        lado: int,
        timestamp: float,
    ) -> Optional[HuecoParqueo]:
        """Dos delimitadores paralelos al muro separados lo que mide la bahia."""

        longitud_min = float(self._cfg("bay_separator_min_length_mm", 125.0))
        longitud_max = float(self._cfg("bay_separator_max_length_mm", 290.0))
        separacion_esperada = float(self._cfg("bay_expected_separation_mm", 390.0))
        tolerancia = float(self._cfg("bay_separation_tolerance_mm", 105.0))
        lateral_min = float(self._cfg("bay_lateral_min_mm", 140.0))
        lateral_max = float(self._cfg("bay_lateral_max_mm", 900.0))
        longitudinal_max = float(self._cfg("bay_max_longitudinal_mm", 1100.0))
        min_puntos = max(3, int(self._cfg("object_min_points", 3)))
        max_desvio = float(self._cfg("bay_split_max_deviation_mm", 30.0))
        cos_max_desvio = math.cos(math.radians(float(self._cfg("bay_max_tilt_deg", 25.0))))
        max_residuo = min(55.0, float(self._cfg("wall_max_residual_mm", 75.0)))

        partidos: List[Sequence[PuntoPolar]] = []
        for cluster in clusters:
            if max_desvio > 0.0:
                partidos.extend(self._partir_en_rectas(cluster, max_desvio, min_puntos))
            else:
                partidos.append(cluster)

        segmentos: List[_Segmento] = []
        for cluster in partidos:
            if len(cluster) < min_puntos:
                continue
            segmento = _segmento_pca(cluster)
            if segmento is None:
                continue
            lateral = lado * segmento.x_mm
            if not longitud_min <= segmento.longitud_mm <= longitud_max:
                continue
            if segmento.alineacion < cos_max_desvio or segmento.residuo_mm > max_residuo:
                continue
            if not lateral_min <= lateral <= lateral_max:
                continue
            if abs(segmento.y_mm) > longitudinal_max:
                continue
            segmentos.append(segmento)

        mejor = None
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

                calidad_sep = 1.0 - error_separacion / max(tolerancia, 1.0)
                calidad_lat = 1.0 - diferencia_lateral / max(tolerancia, 1.0)
                calidad_alin = 0.5 * (primero.alineacion + segundo.alineacion)
                calidad_res = 1.0 - (primero.residuo_mm + segundo.residuo_mm) / max(
                    2.0 * max_residuo, 1.0
                )
                confianza = _limitar(
                    0.42 * calidad_sep
                    + 0.23 * calidad_lat
                    + 0.23 * calidad_alin
                    + 0.12 * calidad_res,
                    0.0,
                    1.0,
                )
                coste = error_separacion + 0.45 * diferencia_lateral - 20.0 * confianza
                if coste < mejor_coste:
                    mejor_coste = coste
                    mejor = HuecoParqueo(
                        timestamp=float(timestamp),
                        lado=int(lado),
                        borde_trasero_y_mm=min(primero.y_mm, segundo.y_mm),
                        borde_delantero_y_mm=max(primero.y_mm, segundo.y_mm),
                        centro_y_mm=0.5 * (primero.y_mm + segundo.y_mm),
                        separacion_mm=separacion,
                        distancia_lateral_mm=0.5 * (lateral_1 + lateral_2),
                        confianza=confianza,
                    )
        return mejor

    def confirmar_hueco(
        self, candidato: Optional[HuecoParqueo], lado: int
    ) -> Optional[HuecoParqueo]:
        """Exige varios barridos seguidos viendo el mismo hueco.

        Un solo barrido puede emparejar dos cosas cualquiera; tres seguidos con
        la misma separacion y el mismo lateral, no.
        """

        requeridos = max(1, int(self._cfg("bay_confirm_scans", 3)))
        tolerancia = float(self._cfg("bay_separation_tolerance_mm", 105.0))

        if self._hueco_lado != lado:
            self.reiniciar()
            self._hueco_lado = lado

        if candidato is None:
            self._hueco_repeticiones = 0
            self._hueco_previo = None
            return self._hueco_confirmado

        anterior = self._hueco_previo
        coincide = anterior is not None and (
            abs(anterior.separacion_mm - candidato.separacion_mm) <= 0.65 * tolerancia
            and abs(anterior.distancia_lateral_mm - candidato.distancia_lateral_mm)
            <= tolerancia
            and abs(anterior.centro_y_mm - candidato.centro_y_mm)
            <= max(180.0, 1.6 * tolerancia)
        )
        self._hueco_repeticiones = self._hueco_repeticiones + 1 if coincide else 1
        self._hueco_previo = candidato
        if self._hueco_repeticiones >= requeridos:
            self._hueco_confirmado = candidato
        return self._hueco_confirmado

    # ------------------------------------------------------------------ api

    def procesar(
        self,
        scan: Iterable[Sequence[float]],
        timestamp: float,
        angulo_servo_deg: float = 0.0,
        lado_parqueo: int = 0,
    ):
        """Todo el trabajo de un barrido en una pasada.

        Devuelve ``(paredes, objetos, hueco)``.  El hueco solo se busca cuando
        ``lado_parqueo`` es distinto de cero, porque partir clusters en rectas
        cuesta y no hace falta durante las vueltas.

        ``lado_parqueo`` hace ademas otra cosa: relaja el minimo de segmento
        de la pared de ESE lado (ver ``_largo_minimo_mm``), que es lo unico
        que permite ver el muro de la bahia con el flanco a 11 mm de el.
        """

        max_mm = float(self._cfg("max_distance_mm", 4000.0))
        puntos = self._mascarar(normalizar_barrido(scan, max_mm), angulo_servo_deg)

        clusters = segmentar(
            puntos,
            factor_abd=float(self._cfg("abd_factor", 0.04)),
            offset_abd_mm=float(self._cfg("abd_offset_mm", 40.0)),
            max_salto_angular_deg=float(self._cfg("abd_max_angular_gap_deg", 4.0)),
            min_puntos=max(2, int(self._cfg("object_min_points", 3))),
        )
        objetos, puntos_objeto = self.objetos(clusters, timestamp, con_puntos=True)
        paredes = self.paredes(
            clusters, puntos, timestamp, puntos_objeto, lado_parqueo
        )
        # El descarte por cercania a la pared va DESPUES y sobre la lista ya
        # hecha: solo necesita el centro de cada objeto, asi que no hay que
        # repetir el PCA.  Hacerlo con una segunda pasada completa costaba 27
        # barridos descartados de 333 en el robot.
        objetos = self.filtrar_pegados_a_pared(objetos, paredes)

        hueco = None
        if lado_parqueo:
            hueco = self.confirmar_hueco(
                self.buscar_hueco(clusters, lado_parqueo, timestamp), lado_parqueo
            )
        return paredes, objetos, hueco
