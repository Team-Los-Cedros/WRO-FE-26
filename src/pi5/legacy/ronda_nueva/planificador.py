"""Planificador de carril: convierte el mapa en una trayectoria y en direccion.

EL CAMBIO DE FONDO RESPECTO A LA VERSION ANTERIOR
Antes, un pilar INTERRUMPIA la conduccion: la maquina de estados saltaba a
APROXIMACION, luego a SOBREPASO, luego a RECENTRADO, y el seguimiento de pared
se apagaba mientras tanto.  Medido sobre los CSV, evadir se llevaba el 45-52 %
de la ronda -- mas que girar esquinas -- y la eficiencia de rumbo caia al
6-23 % en cuanto habia pilares.

Aqui un pilar no interrumpe nada: solo MUEVE EL CARRIL.  El robot siempre esta
haciendo lo mismo, seguir una linea de offset objetivo dentro de la recta, y
los pilares son nodos que deforman esa linea.  Es la idea del segundo puesto
de 2025 (que controla un unico ``targetOuterWallDistance``), con dos anadidos:

1. El offset objetivo se calcula desde la posicion MEDIDA del pilar, no de una
   tabla de cuatro constantes.  Con la camara dando milimetros no hace falta
   suponer donde esta el poste: se sabe.
2. Entre nodos hay una rampa, no un escalon.  La referencia salta de 0,43 m a
   0,76 m de golpe y deja que el PID se pelee; aqui el escalon se convierte en
   un tramo recto de la ruta y la persecucion pura lo sigue sin sobrepico.

LA LEY DE DIRECCION
Tipo Stanley: rumbo de la ruta (pre-alimentado) mas correccion del error
lateral.  Devuelve un angulo de RUEDA con significado geometrico, y la
conversion a unidades de servo sale de los radios de giro medidos en el chasis
(228 mm a la izquierda, 260 a la derecha), no de una constante inventada.  Por
eso el mando es asimetrico: el varillaje lo es.

VALIDACION
Los valores por defecto salen de simular las 28 disposiciones de obstaculos
del sorteo oficial (las 36 cartas, de las que 28 son distintas) en los dos
sentidos, con un modelo de bicicleta y colision de rectangulo contra poste.
Las 56 corridas pasan con 33,8 mm de holgura minima, y 24,9 mm añadiendo 25 mm
de ruido gaussiano a la posicion medida de cada pilar y 12 mm a la pose.
Ninguna rebasa por el lado equivocado.  El banco esta en
``tests/test_planificador.py``.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .modelos import DeteccionPilar, NodoRuta, PoseCarril, ROJO, VERDE


def _limitar(valor: float, minimo: float, maximo: float) -> float:
    return max(minimo, min(maximo, valor))


class ConversorDireccion:
    """Traduce un angulo de rueda a la unidad que entiende la Pico.

    El mando del servo no es el angulo de la rueda: hay un varillaje de por
    medio y ademas no es simetrico.  Se calibra con lo unico que se puede
    medir sin desmontar nada -- el radio de giro a tope de cada lado -- y de
    ahi sale el angulo de rueda maximo real por bicicleta de Ackermann::

        rueda_max = atan(batalla / radio_eje)

    Con batalla 136 mm: 30,8 grados a la izquierda y 27,6 a la derecha, contra
    mandos de +25 y -20.  Suponer que mando y rueda son lo mismo metia un 23 %
    de error en un lado y un 38 % en el otro.
    """

    def __init__(self, config: Dict[str, Any]):
        control = config.get("control", {})
        chasis = config.get("chassis", {})
        self.batalla_mm = float(chasis.get("wheelbase_mm", 136.0))
        self.radio_izq_mm = float(chasis.get("turn_radius_left_mm", 228.0))
        self.radio_der_mm = float(chasis.get("turn_radius_right_mm", 260.0))
        self.mando_max_izq = abs(float(control.get("steering_max_left_deg", 25.0)))
        self.mando_max_der = abs(float(control.get("steering_max_right_deg", 20.0)))

        self.rueda_max_izq = math.degrees(
            math.atan2(self.batalla_mm, max(1.0, self.radio_izq_mm))
        )
        self.rueda_max_der = math.degrees(
            math.atan2(self.batalla_mm, max(1.0, self.radio_der_mm))
        )

    def a_mando(self, rueda_deg: float) -> float:
        """Angulo de rueda (positivo a la izquierda) -> mando del servo."""

        if rueda_deg >= 0.0:
            rueda = min(rueda_deg, self.rueda_max_izq)
            return rueda / max(1e-6, self.rueda_max_izq) * self.mando_max_izq
        rueda = max(rueda_deg, -self.rueda_max_der)
        return rueda / max(1e-6, self.rueda_max_der) * self.mando_max_der

    def a_rueda(self, mando_deg: float) -> float:
        if mando_deg >= 0.0:
            return mando_deg / max(1e-6, self.mando_max_izq) * self.rueda_max_izq
        return mando_deg / max(1e-6, self.mando_max_der) * self.rueda_max_der


class PlanificadorCarril:
    """Construye la ruta de offsets y la sigue con persecucion pura."""

    def __init__(self, config: Dict[str, Any]):
        pista = config.get("track", {})
        control = config.get("control", {})
        chasis = config.get("chassis", {})

        self.ancho_carril_mm = float(pista.get("lane_width_mm", 1000.0))
        self.medio_pilar_mm = float(pista.get("pillar_width_mm", 100.0)) / 2.0
        self.medio_robot_mm = float(chasis.get("width_mm", 125.0)) / 2.0

        self.holgura_pilar_mm = float(control.get("pillar_clearance_mm", 70.0))
        self.holgura_min_mm = float(control.get("pillar_min_clearance_mm", 18.0))
        self.holgura_pared_mm = float(control.get("wall_clearance_mm", 55.0))
        self.offset_crucero_mm = float(
            control.get("cruise_offset_mm", self.ancho_carril_mm / 2.0)
        )
        self.encaje_mm = float(control.get("pillar_setup_distance_mm", 900.0))
        self.encaje_min_mm = float(control.get("pillar_setup_min_mm", 300.0))
        self.salida_mm = float(control.get("pillar_release_distance_mm", 420.0))
        self.asentamiento_mm = float(control.get("pillar_settle_mm", 0.0))
        self.rumbo_max_ruta_deg = float(control.get("plan_max_heading_deg", 32.0))
        self.radio_giro_mm = max(
            float(chasis.get("turn_radius_left_mm", 228.0)),
            float(chasis.get("turn_radius_right_mm", 260.0)),
        )
        self.alcance_planificacion_mm = float(control.get("plan_range_mm", 2400.0))
        # Entrada de esquina: a que offset y a que avance hay que estar ya
        # abierto hacia el muro EXTERIOR antes de tirar el volante a tope.
        # Con 0 se desactiva y la ruta termina en crucero, como antes.
        self.offset_esquina_mm = float(control.get("corner_entry_offset_mm", 0.0))
        self.entrada_esquina_mm = float(control.get("corner_entry_avance_mm", 900.0))
        self.margen_detras_mm = float(control.get("plan_behind_mm", 500.0))
        self.medio_largo_robot_mm = float(chasis.get("length_mm", 222.0)) / 2.0
        # Meseta: cuanto antes y cuanto despues del poste se mantiene el offset.
        # Con el modelo de anchura barrida ya no hace falta pasar recto, asi
        # que basta con la mitad del robot; una meseta mayor solo le quita
        # sitio al cambio de carril.
        self.meseta_mm = float(control.get("pillar_plateau_mm", 300.0))

        self.lookahead_min_mm = float(control.get("lookahead_min_mm", 240.0))
        self.lookahead_por_pwm = float(control.get("lookahead_mm_per_pwm", 3.2))
        self.lookahead_max_mm = float(control.get("lookahead_max_mm", 620.0))
        self.ganancia_rumbo = float(control.get("heading_gain", 0.8))
        self.ganancia_lateral = float(control.get("crosstrack_gain", 1.6))
        self.batalla_mm = float(chasis.get("wheelbase_mm", 136.0))
        self.conversor = ConversorDireccion(config)
        self.ultimo_plan_cedido = False

        self.velocidad_crucero = int(control.get("speed_cruise_pwm", 55))
        self.velocidad_min = int(control.get("speed_min_pwm", 25))
        self.frenar_desde_mm = float(control.get("brake_start_mm", 900.0))
        self.frenar_hasta_mm = float(control.get("brake_full_mm", 320.0))
        self.penalizacion_error_mm = float(control.get("speed_error_scale_mm", 260.0))

    # ------------------------------------------------------------ la ruta

    def lado_de_paso(self, color: str, offset_pilar_mm: float, sentido: int) -> int:
        """+1 si hay que ir a MAS offset que el pilar, -1 si a menos.

        Regla WRO: el rojo se rebasa por su derecha y el verde por su
        izquierda.  Traducirlo a offsets depende del sentido de la vuelta,
        porque el offset se mide desde el muro EXTERIOR y ese muro cambia de
        lado: girando a la derecha queda a la izquierda del robot, y al reves.
        """

        if color == ROJO:
            return int(sentido)
        if color == VERDE:
            return -int(sentido)
        # Sin color no se puede elegir lado; se rodea por el hueco mayor.
        return 1 if offset_pilar_mm < self.ancho_carril_mm / 2.0 else -1

    def semiancho_barrido_mm(self, pendiente: float) -> float:
        """Media anchura que el robot BARRE cuando cruza el carril en diagonal.

        Un rectangulo de 222 x 125 mm que avanza cruzado ``psi`` grados ocupa
        lateralmente ``(ancho*cos psi + largo*sin psi) / 2``.  A 30 grados eso
        son 108 mm en vez de 62,5: casi el doble.

        Ignorarlo fue el error de la primera version.  Se compensaba exigiendo
        una meseta plana de 420 mm alrededor de cada poste para pasar SIEMPRE
        recto, y esa meseta se comia el sitio que hacia falta para cambiar de
        carril entre dos postes.  Contando la diagonal de verdad, la meseta se
        queda en lo que mide el robot y el cambio de carril tiene sitio.
        """

        psi = math.atan(abs(pendiente))
        return 0.5 * (
            2.0 * self.medio_robot_mm * math.cos(psi)
            + 2.0 * self.medio_largo_robot_mm * math.sin(psi)
        )

    def _limites_de_paso(
        self, color: str, offset_pilar_mm: float, sentido: int, semiancho_mm: Optional[float] = None
    ) -> Tuple[float, float, int]:
        """(offset preferido, offset minimo admisible, lado).

        El minimo admisible es el que roza: por debajo de el se toca el poste
        y la corrida se acaba.  El preferido lleva la holgura de diseño.  Tener
        los dos permite CEDER holgura cuando la geometria no da para mas, en
        vez de pedir una trayectoria que el chasis no puede trazar.
        """

        lado = self.lado_de_paso(color, offset_pilar_mm, sentido)
        semiancho = self.medio_robot_mm if semiancho_mm is None else float(semiancho_mm)
        base = semiancho + self.medio_pilar_mm
        margen = semiancho + self.holgura_pared_mm
        techo = self.ancho_carril_mm - margen
        preferido = _limitar(
            offset_pilar_mm + lado * (base + self.holgura_pilar_mm), margen, techo
        )
        limite = _limitar(
            offset_pilar_mm + lado * (base + self.holgura_min_mm), margen, techo
        )
        return preferido, limite, lado

    def offset_de_paso(self, color: str, offset_pilar_mm: float, sentido: int) -> float:
        """Offset preferido para rebasar el pilar, sin considerar vecinos."""

        return self._limites_de_paso(color, offset_pilar_mm, sentido)[0]

    def _relajar(
        self, objetivos: List[float], limites: List[Tuple[float, int]], tramos: List[float]
    ) -> List[float]:
        """Acerca objetivos vecinos hasta que el tramo entre ellos sea trazable.

        Dos pilares de colores opuestos separados 500 mm piden un
        desplazamiento lateral de ~400 mm en 500 mm de recorrido, o sea 38
        grados de cruce sostenido: este chasis no lo hace, y pedirlo solo
        consigue que la direccion sature y el robot llegue tarde a los dos.

        La salida honesta es CEDER: se recorta la holgura de los dos, a partes
        iguales, hasta que la pendiente cabe -- pero nunca por debajo del
        limite que roza el poste.  Si aun asi no cabe, el plan se queda en el
        limite y el freno por pendiente (ver ``velocidad``) hace el resto.
        """

        cedidos = [False] * len(objetivos)

        for _ in range(6):
            cambiado = False
            for i in range(len(objetivos) - 1):
                exceso = abs(objetivos[i + 1] - objetivos[i]) - self.desplazamiento_maximo(
                    tramos[i]
                )
                if exceso <= 0.5:
                    continue
                for indice, vecino in ((i, i + 1), (i + 1, i)):
                    limite, lado = limites[indice]
                    destino = objetivos[indice] + math.copysign(
                        exceso / 2.0, objetivos[vecino] - objetivos[indice]
                    )
                    # No cruzar nunca el limite que roza el poste.
                    destino = max(destino, limite) if lado > 0 else min(destino, limite)
                    if abs(destino - objetivos[indice]) > 0.5:
                        objetivos[indice] = destino
                        cambiado = True
            if not cambiado:
                break

        # Si tras ceder toda la holgura el tramo SIGUE sin caber, la fisica no
        # da y no hay plan que lo arregle: dos postes de colores opuestos en la
        # misma posicion lateral y a 500 mm uno de otro piden 365 mm de
        # desplazamiento y el chasis traza 240 en esa distancia.
        #
        # Aqui se hizo una version que "cedia" moviendo el objetivo del segundo
        # hasta donde si llegaba, y estaba MAL: el objetivo resultante caia
        # ENCIMA del poste.  Apuntar a un choque nunca es mejor que apuntar a
        # rozar.  Se dejan los dos en su limite de rozadura y se levanta la
        # bandera, que el piloto usa para frenar: mas lento no acorta la
        # distancia, pero si multiplica los ciclos de control por milimetro
        # recorrido y con eso el seguimiento mejora.
        for i in range(len(objetivos) - 1):
            if abs(objetivos[i + 1] - objetivos[i]) > self.desplazamiento_maximo(tramos[i]) + 0.5:
                cedidos[i] = cedidos[i + 1] = True
        self.ultimo_plan_cedido = any(cedidos)
        return objetivos

    def desplazamiento_maximo(self, tramo_mm: float) -> float:
        """Cuanto se puede mover el robot de lado en ``tramo_mm`` de recorrido.

        Dos limites, y manda el menor:

        * El cruce sostenido: ``tramo * tan(rumbo_max)``.  Es el que domina en
          tramos largos, donde sobra sitio para girar.
        * La curva en S con el radio minimo del chasis: ``tramo^2 / (4 R)``.
          Es el que domina en tramos cortos, y es el que faltaba: con radio
          260 mm, en 500 mm de recorrido no se pueden ganar mas de 240 mm de
          lado por mucho que se pise el volante.
        """

        tramo = max(1.0, float(tramo_mm))
        por_cruce = tramo * math.tan(math.radians(self.rumbo_max_ruta_deg))
        por_radio = tramo * tramo / (4.0 * max(1.0, self.radio_giro_mm))
        return min(por_cruce, por_radio)

    def construir_ruta(
        self,
        pose: PoseCarril,
        sentido: int,
        obstaculos: Sequence[Tuple[float, float, str]],
    ) -> List[NodoRuta]:
        """Ruta como lista de nodos (avance, offset), del mas lejano al mas cercano.

        ``obstaculos`` son tuplas (avance_mm, offset_mm, color) ya en el marco
        de la recta actual.  Se ordenan por avance decreciente, que es el orden
        en que el robot los va a encontrar.
        """

        # El avance BAJA conforme el robot progresa, asi que lo que tiene
        # delante son avances MENORES que el suyo.  Se conserva tambien un
        # margen por detras, porque un pilar que el eje ya paso todavia esta
        # al costado del robot.
        relevantes = [
            (avance, offset, color)
            for avance, offset, color in obstaculos
            if pose.avance_mm - self.alcance_planificacion_mm
            <= avance
            <= pose.avance_mm + self.margen_detras_mm
        ]
        relevantes.sort(key=lambda item: -item[0])

        if not relevantes:
            base = [NodoRuta(pose.avance_mm + 4000.0, self.offset_crucero_mm, "crucero")]
            return base + self._nodos_de_esquina(base[-1])

        # Una MESETA por pilar en vez de un punto: el robot mide 222 mm de
        # largo, asi que tiene que mantener el offset desde antes de que el
        # morro llegue al poste hasta que la culata lo deja atras.  Y la
        # meseta empieza ``asentamiento_mm`` antes, porque el robot no puede
        # doblar la esquina del plan sin redondearla: medido en simulacion,
        # ese redondeo se comia 41 mm de holgura justo encima del poste.
        meseta = self.meseta_mm
        avances = [avance for avance, _, _ in relevantes]
        ventanas = self._ventanas(avances, meseta)

        # Dos pasadas: la primera supone paso recto y da una pendiente; la
        # segunda recalcula con la anchura barrida que esa pendiente implica.
        # Dos bastan -- la tercera mueve menos de un milimetro.
        objetivos: List[float] = []
        limites: List[Tuple[float, int]] = []
        semianchos = [self.medio_robot_mm] * len(relevantes)
        for _pasada in range(2):
            objetivos = []
            limites = []
            for indice, (avance, offset, color) in enumerate(relevantes):
                preferido, limite, lado = self._limites_de_paso(
                    color, offset, sentido, semianchos[indice]
                )
                objetivos.append(preferido)
                limites.append((limite, lado))
            semianchos = [
                self.semiancho_barrido_mm(
                    self._pendiente_local(objetivos, avances, indice)
                )
                for indice in range(len(relevantes))
            ]

        # El tramo disponible para cambiar de carril es la distancia entre
        # pilares.  Se probo medirlo entre mesetas y era DEMASIADO pesimista:
        # con dos postes a 1000 mm las mesetas dejan 400 y el modelo declaraba
        # infactible un desplazamiento de 171 mm que el robot hace de sobra.
        # El efecto no era academico -- la bandera de "plan cedido" frenaba al
        # 60 % durante toda la ronda.  La meseta es una PREFERENCIA (pasar lo
        # mas paralelo posible), no una prohibicion de moverse.
        tramos = [
            max(1.0, avances[i] - avances[i + 1]) for i in range(len(avances) - 1)
        ]
        objetivos = self._relajar(objetivos, limites, tramos)

        nodos: List[NodoRuta] = []
        for indice, (avance, _offset, color) in enumerate(relevantes):
            superior, inferior = ventanas[indice]
            motivo = f"{color or 'SIN_COLOR'}@{int(avance)}"
            nodos.append(NodoRuta(superior, objetivos[indice], motivo))
            nodos.append(NodoRuta(inferior, objetivos[indice], motivo))

        # Rampas a crucero por los dos extremos, con la longitud que el cruce
        # maximo permite: asi el plan nunca pide un desplazamiento imposible.
        primero, ultimo = nodos[0], nodos[-1]
        nodos.insert(
            0,
            NodoRuta(
                primero.avance_mm + self._rampa(primero.offset_mm),
                self.offset_crucero_mm,
                "crucero:antes",
            ),
        )
        nodos.append(
            NodoRuta(
                ultimo.avance_mm - self._rampa(ultimo.offset_mm),
                self.offset_crucero_mm,
                "crucero:despues",
            )
        )
        nodos.sort(key=lambda nodo: -nodo.avance_mm)
        nodos.extend(self._nodos_de_esquina(nodos[-1]))
        return nodos

    def _nodos_de_esquina(self, ultimo: NodoRuta) -> List[NodoRuta]:
        """Cierre de la ruta abriendo hacia el muro exterior antes de la esquina.

        El giro se hace a tope de volante, y a tope el radio es 228/260 mm.
        Entrando desde el centro del carril (offset 500) el centro de giro cae
        a 760 del muro exterior y la esquina delantera interior barre hasta
        367 mm de ese centro: raspa el bloque interior.  Medido en la corrida 2
        del 04-09, el robot llego a 150 mm del bloque en las cuatro esquinas y
        cada una acabo en retroceso de emergencia.

        Abrirse hacia fuera ANTES de girar es lo que le da sitio al barrido.
        No se acredita nada aqui: es solo un nodo mas de la ruta, asi que un
        pilar cercano sigue mandando por la interpolacion normal.
        """

        if self.offset_esquina_mm <= 0.0:
            return []
        entrada = self.entrada_esquina_mm
        if ultimo.avance_mm <= entrada:
            return [NodoRuta(entrada, self.offset_esquina_mm, "esquina")]
        inicio = min(ultimo.avance_mm, entrada + self._rampa(self.offset_esquina_mm))
        return [
            NodoRuta(inicio, ultimo.offset_mm, "esquina:rampa"),
            NodoRuta(entrada, self.offset_esquina_mm, "esquina"),
        ]

    def _pendiente_local(
        self, objetivos: Sequence[float], avances: Sequence[float], indice: int
    ) -> float:
        """Pendiente del plan alrededor de un pilar, en mm de offset por mm."""

        anterior = objetivos[indice - 1] if indice > 0 else self.offset_crucero_mm
        avance_anterior = (
            avances[indice - 1] if indice > 0 else avances[indice] + self.encaje_mm
        )
        siguiente = (
            objetivos[indice + 1]
            if indice + 1 < len(objetivos)
            else self.offset_crucero_mm
        )
        avance_siguiente = (
            avances[indice + 1]
            if indice + 1 < len(avances)
            else avances[indice] - self.salida_mm
        )
        tramo = max(1.0, avance_anterior - avance_siguiente)
        return abs(siguiente - anterior) / tramo

    def _ventanas(
        self, avances: Sequence[float], meseta: float
    ) -> List[Tuple[float, float]]:
        """Ventana (inicio, fin) en la que cada pilar impone su offset.

        Se reparten los solapes por el punto medio entre pilares vecinos, y si
        ni asi caben se colapsan a un punto.  Nunca se permite que la ventana
        de uno empiece antes de que acabe la del anterior: eso creaba dos
        nodos al mismo avance con offsets distintos, o sea un escalon vertical
        que la interpolacion no sabe resolver.
        """

        ventanas: List[Tuple[float, float]] = []
        for indice, avance in enumerate(avances):
            superior = avance + meseta + self.asentamiento_mm
            inferior = avance - meseta
            if indice > 0:
                superior = min(superior, (avances[indice - 1] + avance) / 2.0)
            if indice + 1 < len(avances):
                inferior = max(inferior, (avance + avances[indice + 1]) / 2.0)
            if superior <= inferior:
                superior = inferior = avance
            ventanas.append((superior, inferior))

        for indice in range(1, len(ventanas)):
            anterior_inferior = ventanas[indice - 1][1]
            superior, inferior = ventanas[indice]
            if superior >= anterior_inferior:
                superior = anterior_inferior - 1.0
                if superior <= inferior:
                    inferior = superior
            ventanas[indice] = (superior, inferior)
        return ventanas

    def _rampa(self, offset_mm: float) -> float:
        """Longitud de la rampa desde crucero hasta ``offset_mm``.

        Se busca el tramo mas corto en el que ese desplazamiento es trazable,
        con un margen: pedir exactamente el minimo deja al control sin sitio
        para el transitorio y llega tarde.
        """

        salto = abs(offset_mm - self.offset_crucero_mm)
        if salto < 1.0:
            return self.encaje_min_mm
        # Se usa la rampa MAS LARGA que cabe, no la mas corta que basta: en la
        # recta sobra sitio, y una rampa larga es siempre mas suave y deja
        # margen para el transitorio.  Acortarla al minimo factible costaba
        # 54 mm de holgura en simulacion.
        return self.encaje_mm

    @staticmethod
    def offset_en(ruta: Sequence[NodoRuta], avance_mm: float) -> float:
        """Interpola el offset objetivo de la ruta a una distancia dada.

        La ruta esta ordenada por avance DECRECIENTE porque el avance baja
        conforme el robot progresa; interpolar entre los dos nodos que rodean
        ``avance_mm`` es lo que convierte una lista de consignas en una
        trayectoria continua.
        """

        if not ruta:
            return 0.0
        if avance_mm >= ruta[0].avance_mm:
            return ruta[0].offset_mm
        if avance_mm <= ruta[-1].avance_mm:
            return ruta[-1].offset_mm
        for anterior, siguiente in zip(ruta, ruta[1:]):
            if siguiente.avance_mm <= avance_mm <= anterior.avance_mm:
                tramo = anterior.avance_mm - siguiente.avance_mm
                if tramo <= 1e-6:
                    return siguiente.offset_mm
                peso = (anterior.avance_mm - avance_mm) / tramo
                return anterior.offset_mm + peso * (siguiente.offset_mm - anterior.offset_mm)
        return ruta[-1].offset_mm

    # ------------------------------------------------------------ direccion

    def lookahead_mm(self, velocidad_pwm: float) -> float:
        return _limitar(
            self.lookahead_min_mm + self.lookahead_por_pwm * abs(float(velocidad_pwm)),
            self.lookahead_min_mm,
            self.lookahead_max_mm,
        )

    def direccion(
        self,
        pose: PoseCarril,
        ruta: Sequence[NodoRuta],
        sentido: int,
        velocidad_pwm: float,
    ) -> Tuple[float, float, float]:
        """Devuelve (mando_servo_deg, offset_objetivo_mm, error_lateral_mm).

        Ley tipo Stanley: rumbo de la ruta menos rumbo del robot, mas un
        termino que corrige el error lateral instantaneo.

        POR QUE NO PERSECUCION PURA A SECAS
        Se probo primero y en simulacion se quedaba corta: la persecucion pura
        apunta a un punto situado ``L`` mm mas adelante, asi que en una rampa
        recorta y llega al pilar con ~160 mm de retraso lateral -- suficiente
        para comerse toda la holgura.  El termino de rumbo de la ruta es un
        pre-alimentado: le dice al robot cuanto tiene que cruzarse ANTES de
        acumular error, y con el el retraso en rampa es cero.
        """

        L = self.lookahead_mm(velocidad_pwm)
        objetivo_adelante = self.offset_en(ruta, pose.avance_mm - L)
        objetivo_aqui = self.offset_en(ruta, pose.avance_mm)
        error_offset = objetivo_aqui - pose.offset_mm

        # El offset crece hacia la derecha del robot cuando el muro exterior
        # queda a su izquierda (sentido +1), y al reves con sentido -1.
        rumbo_ruta_deg = math.degrees(
            math.atan2(-float(sentido) * (objetivo_adelante - objetivo_aqui), L)
        )
        error_izquierda = -float(sentido) * error_offset

        rueda = self.ganancia_rumbo * (rumbo_ruta_deg - pose.rumbo_error_deg)
        rueda += self.ganancia_lateral * math.degrees(
            math.atan2(error_izquierda, max(1.0, L))
        )
        return self.conversor.a_mando(rueda), objetivo_adelante, error_offset

    # ------------------------------------------------------------ velocidad

    def velocidad(
        self,
        pose: PoseCarril,
        error_lateral_mm: float,
        frontal_mm: float,
        velocidad_max_pwm: Optional[int] = None,
    ) -> int:
        """Cuanto se puede correr aqui.

        Tres frenos independientes, y manda el mas exigente: la pared de
        enfrente, el error lateral que queda por corregir y el rumbo cruzado.
        Frenar por error lateral es lo que evita el zigzag a alta velocidad:
        el robot solo acelera cuando ya esta donde queria estar.
        """

        techo = int(self.velocidad_crucero if velocidad_max_pwm is None else velocidad_max_pwm)
        factor = 1.0

        if math.isfinite(frontal_mm) and frontal_mm < self.frenar_desde_mm:
            tramo = max(1.0, self.frenar_desde_mm - self.frenar_hasta_mm)
            factor = min(factor, _limitar((frontal_mm - self.frenar_hasta_mm) / tramo, 0.0, 1.0))

        error = abs(error_lateral_mm) / max(1.0, self.penalizacion_error_mm)
        factor = min(factor, _limitar(1.0 - 0.45 * error, 0.4, 1.0))

        cruzado = abs(pose.rumbo_error_deg) / 30.0
        factor = min(factor, _limitar(1.0 - 0.4 * cruzado, 0.5, 1.0))

        return int(max(self.velocidad_min, round(techo * factor)))
