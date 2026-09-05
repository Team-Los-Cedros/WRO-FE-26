"""Mapa de la pista: doce casillas donde se vota el color de cada pilar.

LA IDEA, DEL SEGUNDO PUESTO DE 2025
Los pilares no estan en cualquier sitio: caen en posiciones marcadas de la
lona.  Cuatro rectas por tres posiciones son doce casillas, y la identidad de
un pilar es SU CASILLA, no un ``track_id`` que se pierde en cuanto el poste
sale del campo de vision.

Esto cambia el problema de sitio:

* Perder de vista un pilar deja de importar.  El esquema de tracks anterior lo
  apagaba a 216 mm de mediana y desde ahi el robot maniobraba a ciegas.
* La segunda y la tercera vuelta se corren SABIENDO lo que viene.  El carril
  se puede empezar a mover mucho antes de ver el poste, que es de donde sale
  la mayor parte del tiempo que se gana.
* Una deteccion mala no manda: la casilla acumula votos, y un cuadro con el
  color equivocado no le gana a diez buenos.

Y una diferencia deliberada con la referencia: alli, un pilar que no encajaba
en ninguna banda se DESCARTABA.  Aqui solo se descarta para la MEMORIA.  El
planificador esquiva ademas con las detecciones vivas del instante, asi que un
poste mal colocado o mal medido se sigue esquivando aunque no entre en el mapa.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .modelos import Casilla, DeteccionPilar, EntradaMapa, EXTERIOR, INTERIOR, MapaParedes


class MapaPista:
    """Doce casillas con su color votado y su posicion medida."""

    def __init__(self, config: Dict[str, Any]):
        pista = config.get("track", {})
        self.largo_recta_mm = float(pista.get("segment_length_mm", 3000.0))
        self.ancho_carril_mm = float(pista.get("lane_width_mm", 1000.0))
        self.posiciones_mm: Tuple[float, ...] = tuple(
            float(v) for v in pista.get("pillar_positions_mm", (2000.0, 1500.0, 1000.0))
        )
        self.tolerancia_mm = float(pista.get("pillar_position_tolerance_mm", 200.0))
        self.filas_mm: Tuple[float, ...] = (
            float(pista.get("pillar_offset_outer_mm", 380.0)),
            float(pista.get("pillar_offset_inner_mm", 574.0)),
        )
        self.margen_carril_mm = float(pista.get("lane_margin_mm", 120.0))
        self.peso_min_confianza = float(pista.get("min_vote_confidence", 0.25))
        self.entradas: Dict[Tuple[int, int], EntradaMapa] = {}

    # ------------------------------------------------------------ escritura

    def reiniciar(self) -> None:
        self.entradas.clear()

    def _indice_por_avance(self, avance_mm: float) -> Optional[int]:
        mejor, mejor_error = None, self.tolerancia_mm
        for indice, posicion in enumerate(self.posiciones_mm):
            error = abs(avance_mm - posicion)
            if error <= mejor_error:
                mejor, mejor_error = indice, error
        return mejor

    def coordenadas_de_pilar(
        self, avance_mm: float, offset_mm: float
    ) -> Optional[Tuple[int, float, float]]:
        """(salto de segmento, avance, offset) del pilar en SU recta.

        Un pilar que cae fuera del carril actual esta, casi siempre, en la
        recta siguiente vista por encima del bloque interior.  Cambiar de
        recta es intercambiar las dos coordenadas, porque el muro frontal de
        esta recta es el muro exterior de la que viene::

            avance_siguiente = largo - offset_actual
            offset_siguiente = avance_actual

        Es la misma relacion que usa la referencia, escrita como el cambio de
        marco que es en vez de como dos tablas de bandas.
        """

        margen = self.margen_carril_mm
        if -margen <= offset_mm <= self.ancho_carril_mm + margen:
            return 0, avance_mm, offset_mm

        siguiente_avance = self.largo_recta_mm - offset_mm
        siguiente_offset = avance_mm
        if not -margen <= siguiente_offset <= self.ancho_carril_mm + margen:
            return None
        if not 0.0 <= siguiente_avance <= self.largo_recta_mm:
            return None
        return 1, siguiente_avance, siguiente_offset

    def _distancia_a_fila(self, offset_mm: float) -> float:
        return min(abs(offset_mm - fila) for fila in self.filas_mm)

    def pertenece_a_esta_recta(self, avance_mm: float, offset_mm: float) -> bool:
        """Decide si un poste medido esta en la recta actual o en la siguiente.

        Con la camara alta se ven los postes de la recta de despues POR ENCIMA
        del bloque interior, y en el marco de la recta actual caen en sitios
        perfectamente creibles: uno que esta en la siguiente a 2000 mm del
        muro aparece aqui con offset 1000, justo en el borde del carril.
        Filtrar solo por "cabe en el carril" no los distingue, y colarlos
        deforma la ruta con un obstaculo que el robot no tiene delante.

        El desempate usa la geometria del sorteo: los postes solo caen en dos
        filas (380 y 574 mm del muro exterior).  Se prueban las dos lecturas y
        gana la que deja el poste mas cerca de una fila real.
        """

        largo = self.largo_recta_mm
        actual = (
            self._distancia_a_fila(offset_mm)
            if -self.margen_carril_mm <= offset_mm <= self.ancho_carril_mm + self.margen_carril_mm
            else float("inf")
        )
        avance_siguiente = largo - offset_mm
        siguiente = (
            self._distancia_a_fila(avance_mm)
            if 0.0 <= avance_siguiente <= largo
            else float("inf")
        )
        if actual == float("inf") and siguiente == float("inf"):
            return False
        return actual <= siguiente

    def observar(
        self,
        pilar: DeteccionPilar,
        segmento: int,
        avance_mm: float,
        offset_mm: float,
        vuelta: int = 0,
    ) -> Optional[Casilla]:
        """Vota una deteccion en su casilla.  Devuelve la casilla o ``None``."""

        if not pilar.color:
            return None
        if pilar.confianza < self.peso_min_confianza:
            return None

        coordenadas = self.coordenadas_de_pilar(avance_mm, offset_mm)
        if coordenadas is None:
            return None
        salto, avance_recta, offset_recta = coordenadas

        indice = self._indice_por_avance(avance_recta)
        if indice is None:
            return None

        casilla = Casilla(segmento=(segmento + salto) % 4, indice=indice)
        entrada = self.entradas.get((casilla.segmento, casilla.indice))
        if entrada is None:
            entrada = EntradaMapa()
            self.entradas[(casilla.segmento, casilla.indice)] = entrada

        entrada.votos[pilar.color] = entrada.votos.get(pilar.color, 0.0) + pilar.confianza
        entrada.observaciones += 1
        entrada.ultima_vista_s = pilar.timestamp
        entrada.vuelta_ultima_vista = int(vuelta)
        # La posicion se promedia porque cada medida tiene su ruido, pero se
        # pondera hacia la ultima: si el juez movio el poste entre vueltas, el
        # mapa tiene que seguirlo en vez de promediar dos sitios distintos.
        peso = 0.35
        entrada.avance_mm = (1.0 - peso) * entrada.avance_mm + peso * avance_recta if entrada.observaciones > 1 else avance_recta
        entrada.offset_mm = (1.0 - peso) * entrada.offset_mm + peso * offset_recta if entrada.observaciones > 1 else offset_recta
        entrada.lado = EXTERIOR if entrada.offset_mm < self.ancho_carril_mm / 2.0 else INTERIOR
        return casilla

    # ------------------------------------------------------------- lectura

    def entrada(self, segmento: int, indice: int) -> Optional[EntradaMapa]:
        return self.entradas.get((segmento % 4, indice))

    def color_recordado(
        self,
        segmento: int,
        avance_mm: float,
        offset_mm: float,
        tolerancia_offset_mm: float = 220.0,
    ) -> Optional[str]:
        """El color que el mapa ya sabe de la casilla donde cae este punto.

        Existe porque la camara clasifica bien de LEJOS y deja de hacerlo justo
        cuando hace falta.  Medido sobre las tres corridas del 05-09, la tasa
        de color del poste que usa el planificador::

            0-250 mm     13 %   <-- aqui es donde se decide el lado
            250-500 mm   62 %
            750-1000 mm  75 %
            1000-1250 mm 73 %

        Sin color, el planificador no aplica la regla (rojo por la derecha,
        verde por la izquierda): elige lado por el hueco mayor.  Y como el
        color aparece y desaparece ciclo a ciclo, el carril objetivo oscila --
        medidos saltos de mas de 100 mm entre ciclos consecutivos, hasta 500,
        con 9 a 27 idas y vueltas por corrida.

        El robot averigua el color a tiempo y lo tira a la basura justo antes
        de usarlo.  Esto lo recupera.
        """

        indice = self._indice_por_avance(avance_mm)
        if indice is None:
            return None
        entrada = self.entrada(segmento, indice)
        if entrada is None or entrada.observaciones == 0:
            return None
        # La casilla tiene dos filas (380 y 574 mm del muro exterior).  Sin
        # esta comprobacion, un poste de la fila interior heredaria el color
        # del exterior de la misma banda longitudinal, que es justo el error
        # que mandaria al robot por el lado contrario.
        if abs(entrada.offset_mm - offset_mm) > tolerancia_offset_mm:
            return None
        return entrada.color

    def pilares_del_segmento(
        self, segmento: int, con_color: bool = True
    ) -> List[Tuple[Casilla, EntradaMapa]]:
        """Casillas ocupadas de una recta, en orden de encuentro."""

        salida = []
        for indice in range(len(self.posiciones_mm)):
            entrada = self.entrada(segmento, indice)
            if entrada is None or entrada.observaciones == 0:
                continue
            if con_color and not entrada.color:
                continue
            salida.append((Casilla(segmento % 4, indice), entrada))
        salida.sort(key=lambda par: -par[1].avance_mm)
        return salida

    def pilares_delante(
        self, segmento: int, avance_mm: float, margen_mm: float = 150.0
    ) -> List[Tuple[Casilla, EntradaMapa]]:
        """Lo que queda por delante en la recta actual, mas la recta siguiente.

        Mirar ya a la recta siguiente es lo que permite elegir el radio del
        giro: si tras la esquina hay un pilar rojo pegado al muro exterior, el
        giro tiene que salir mas abierto, y eso hay que decidirlo ANTES de
        empezar a girar.
        """

        actuales = [
            (casilla, entrada)
            for casilla, entrada in self.pilares_del_segmento(segmento)
            if entrada.avance_mm < avance_mm + margen_mm
        ]
        siguientes = self.pilares_del_segmento((segmento + 1) % 4)
        return actuales + siguientes

    def resumen(self) -> str:
        """Una linea por recta, para el CSV y el panel: ``R0:.RV R1:V..``."""

        partes = []
        for segmento in range(4):
            celdas = ""
            for indice in range(len(self.posiciones_mm)):
                entrada = self.entrada(segmento, indice)
                color = entrada.color if entrada else None
                celdas += {"ROJO": "R", "VERDE": "V"}.get(color or "", ".")
            partes.append(f"R{segmento}:{celdas}")
        return " ".join(partes)

    def casillas_conocidas(self) -> int:
        return sum(1 for entrada in self.entradas.values() if entrada.color)
