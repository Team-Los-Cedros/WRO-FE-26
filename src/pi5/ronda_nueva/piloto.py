"""Piloto: la maquina de estados que arbitra toda la ronda.

Es deliberadamente PEQUEÑA.  La version anterior tenia 1900 lineas en un solo
modulo porque la evasion vivia dentro de la FSM: cada pilar abria tres estados
(aproximacion, sobrepaso, recentrado) con sus timeouts, sus relocks y sus
salidas de emergencia.  Aqui la evasion no es un estado -- es el planificador
moviendo el carril -- asi que la FSM solo tiene que decidir cosas que de
verdad son modos de conduccion distintos:

    ESPERA -> ORIENTACION -> RECTA <-> GIRO -> ... -> APROXIMACION -> PARQUEO

mas dos estados de excepcion (RETROCESO y FIN) que existen porque el robot
puede quedarse encajado y hay que sacarlo.

REGLA DE ORO
Nada aqui hace I/O.  Entran medidas, sale una ``Consigna``.  Eso es lo que
permite correr la ronda entera en el escritorio contra barridos sinteticos o
grabados, que es como se encontraron los tres errores de geometria del
planificador sin gastar bateria.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .localizacion import Localizador, diferencia_angular
from .mapa_pista import MapaPista
from .modelos import (
    Consigna,
    NodoRuta,
    DeteccionPilar,
    HuecoParqueo,
    LineaPiso,
    MapaParedes,
    ParedMagenta,
    PoseCarril,
    ROJO,
    VERDE,
)
from .planificador import PlanificadorCarril


ESPERA = "ESPERA"
ORIENTACION = "ORIENTACION"
SALIDA_BAHIA = "SALIDA_BAHIA"
RECTA = "RECTA"
GIRO = "GIRO"
APROXIMACION = "APROXIMACION"
PARQUEO = "PARQUEO"
RETROCESO = "RETROCESO"
FIN = "FIN"


def _limitar(valor: float, minimo: float, maximo: float) -> float:
    return max(minimo, min(maximo, valor))


def _mediana(valores: Sequence[float]) -> float:
    ordenados = sorted(valores)
    mitad = len(ordenados) // 2
    if len(ordenados) % 2:
        return float(ordenados[mitad])
    return (ordenados[mitad - 1] + ordenados[mitad]) / 2.0


class Piloto:
    """Arbitra estados y devuelve la consigna de cada ciclo."""

    def __init__(self, config: Dict[str, Any], parqueo=None):
        self.config = config
        control = config.get("control", {})
        pista = config.get("track", {})

        self.localizador = Localizador(config)
        self.mapa = MapaPista(config)
        self.plan = PlanificadorCarril(config)
        self.parqueo = parqueo

        self.sentido_configurado = str(control.get("turn_direction", "AUTO")).upper()
        self.color_izquierda = str(control.get("floor_color_left", "AZUL")).upper()
        self.color_derecha = str(control.get("floor_color_right", "NARANJA")).upper()
        self.esquinas_objetivo = int(control.get("corners_before_parking", 12))
        # La primera vuelta MAPEA; a partir de la segunda el mapa ya dice lo
        # que viene antes de verlo, asi que se puede subir el techo de
        # velocidad.  Con 1.0 se desactiva y las tres vueltas van igual.
        self.ganancia_vuelta_conocida = float(
            control.get("speed_known_lap_gain", 1.0)
        )
        self.arranca_en_bahia = bool(control.get("start_in_bay", False))
        self.permitir_parqueo = bool(control.get("parking_enabled", True))
        self.plan_con_sin_color = bool(control.get("plan_uses_colorless", False))
        # Un bulto sin color lejos se ignora (puede aparecer su color antes de
        # llegar), pero uno CERCA hay que esquivarlo aunque no se sepa de que
        # color es: no desviarse no es "seguir recto", es chocar.
        self.sin_color_cerca_mm = float(control.get("plan_colorless_below_mm", 0.0))

        chasis = config.get("chassis", {})
        self.radio_izq_mm = float(chasis.get("turn_radius_left_mm", 228.0))
        self.radio_der_mm = float(chasis.get("turn_radius_right_mm", 260.0))
        self.giro_disparo_mm = float(control.get("corner_trigger_mm", 780.0))
        # 680 y no 520: con el disparo por geometria, apurar mas hace que el
        # propio giro pase a menos de 300 mm del muro de enfrente y la vuelta
        # cargada pasa de 0 a 3 retrocesos.  Es el suelo que impone la esquina,
        # no una preferencia.
        self.giro_disparo_min_mm = float(control.get("corner_trigger_min_mm", 680.0))
        self.giro_disparo_max_mm = float(control.get("corner_trigger_max_mm", 1050.0))
        self.giro_rumbo_min_deg = float(control.get("corner_min_heading_deg", 72.0))
        self.giro_timeout_s = float(control.get("corner_timeout_s", 6.0))
        self.giro_refractario_s = float(control.get("corner_refractory_s", 1.2))
        self.giro_salida_mm = float(control.get("corner_exit_front_mm", 700.0))
        self.giro_rumbo_max_deg = float(control.get("corner_max_heading_deg", 105.0))

        self.velocidad_giro = int(control.get("speed_turn_pwm", 45))
        self.velocidad_retroceso = int(control.get("speed_reverse_pwm", -32))
        self.velocidad_aprox = int(control.get("speed_parking_approach_pwm", 30))

        self.emergencia_frontal_mm = float(control.get("emergency_front_mm", 145.0))
        self.emergencia_lateral_mm = float(control.get("emergency_side_mm", 85.0))
        self.retroceso_min_s = float(control.get("recovery_min_s", 0.6))
        self.retroceso_max_s = float(control.get("recovery_timeout_s", 3.0))
        self.retroceso_salida_mm = float(control.get("recovery_exit_front_mm", 300.0))

        # Vigilante de atascos.  La corrida del 05-09 14:18 se paso 21 de sus
        # 68 s INMOVIL con el motor mandado a 25 PWM: paredes clavadas a +-1 mm
        # y rumbo a 0,1 grados durante 21 s, con el LiDAR fresco (0,3 ms), el
        # watchdog de la Pico en OK y 495 mm de sitio libre por delante.  Nadie
        # se entero: no hay emergencia (no habia nada cerca), no hay retroceso
        # y el estado seguia siendo RECTA.  En una ronda de verdad se habria
        # quedado ahi los tres minutos.
        self.atasco_ventana_s = float(control.get("stall_window_s", 2.0))
        self.atasco_tolerancia_mm = float(control.get("stall_tolerance_mm", 15.0))
        self.atasco_tolerancia_deg = float(control.get("stall_tolerance_deg", 2.0))
        self.atasco_empujon_pwm = int(control.get("stall_kick_pwm", 55))
        self.atasco_empujon_s = float(control.get("stall_kick_s", 0.8))
        self.atasco_empujones_max = int(control.get("stall_max_kicks", 2))
        # Empujar es acelerar hacia delante: solo con sitio de sobra.  El
        # atasco medido tenia 2017 mm de corredor libre.
        self.atasco_empujon_libre_mm = float(
            control.get("stall_kick_min_corridor_mm", 400.0)
        )

        self.slew_direccion = float(control.get("steering_slew_deg_per_scan", 9.0))
        self.slew_velocidad = int(control.get("speed_slew_pwm_per_scan", 12))
        self.orientacion_timeout_s = float(control.get("direction_timeout_s", 10.0))
        self.linea_max_lateral_mm = float(control.get("line_max_lateral_mm", 320.0))
        # La ventana longitudinal tiene que llegar donde llega la camara: con
        # 900 mm y el filtro lateral a la vez, en el robot no quedaba NINGUNA
        # linea candidata y el sentido salia por defecto tras 10 s.
        self.linea_max_avance_mm = float(control.get("line_max_forward_mm", 1600.0))
        self.duracion_max_s = float(control.get("run_timeout_s", 175.0))

        self.reiniciar()

    # --------------------------------------------------------------- estado

    def reiniciar(self) -> None:
        self.estado = ESPERA
        self.sentido = 0
        self.esquinas = 0
        self.pose: Optional[PoseCarril] = None
        self.ruta: List = []
        self.localizador.reiniciar()
        self.mapa.reiniciar()
        self._t_estado = 0.0
        self._t_inicio = 0.0
        self._rumbo_giro_ref = 0.0
        self._estado_interrumpido = ""
        self._angulo_retroceso = 0.0
        self._ultima_esquina_s = -99.0
        self._ultimo_angulo = 0.0
        self._ultima_velocidad = 0
        self._rumbo_absoluto = 0.0
        self._razon = ""
        self._objetivo_offset = 0.0
        self._error_lateral = 0.0
        self._retrocesos = 0
        self._historial_atasco: List[Tuple[float, ...]] = []
        self._t_empujon = -99.0
        self._empujones = 0
        self._atascos = 0
        self._empujando = False
        if self.parqueo is not None:
            self.parqueo.reiniciar()

        if self.sentido_configurado == "LEFT":
            self.sentido = -1
        elif self.sentido_configurado == "RIGHT":
            self.sentido = 1

    def _entrar(self, estado: str, ahora: float) -> None:
        if estado != self.estado:
            self.estado = estado
            self._t_estado = ahora

    def _tiempo(self, ahora: float) -> float:
        return max(0.0, ahora - self._t_estado)

    # ---------------------------------------------------------- utilidades

    def _suavizar(self, velocidad: int, angulo: float) -> Tuple[int, float]:
        """Limita cuanto pueden cambiar direccion y velocidad en un ciclo.

        El servo y el motor son fisicos: pedirles un salto de 40 grados en un
        barrido solo consigue que la mecanica se retuerza y que el LiDAR vea la
        propia rueda.  El limite se salta en RETROCESO y GIRO, donde el salto
        es intencionado.
        """

        if self.estado in (GIRO, RETROCESO) or self._empujando:
            self._ultimo_angulo = angulo
            self._ultima_velocidad = velocidad
            return velocidad, angulo

        angulo = _limitar(
            angulo,
            self._ultimo_angulo - self.slew_direccion,
            self._ultimo_angulo + self.slew_direccion,
        )
        velocidad = int(
            _limitar(
                velocidad,
                self._ultima_velocidad - self.slew_velocidad,
                self._ultima_velocidad + self.slew_velocidad,
            )
        )
        self._ultimo_angulo = angulo
        self._ultima_velocidad = velocidad
        return velocidad, angulo

    def _emitir(self, velocidad: int, angulo: float, razon: str = "") -> Consigna:
        velocidad, angulo = self._suavizar(int(velocidad), float(angulo))
        self._razon = razon or self._razon
        return Consigna(
            velocidad=velocidad,
            angulo=angulo,
            estado=self.estado,
            razon=razon,
        )

    # ----------------------------------------------------------- atascos

    def _olvidar_atasco(self) -> None:
        self._historial_atasco = []

    def _medidas_de_atasco(self, paredes: MapaParedes, rumbo_deg: float):
        """Las magnitudes que TIENEN que cambiar si el robot se mueve."""

        longitudinales = [paredes.frontal_min_mm, paredes.corredor_mm]
        laterales = [paredes.izquierda_min_mm, paredes.derecha_min_mm]
        return longitudinales, laterales, float(rumbo_deg)

    def _detectar_atasco(
        self, paredes: MapaParedes, rumbo_deg: float, ahora: float
    ) -> bool:
        """Se manda avanzar y el mundo no cambia.

        La prueba se hace sobre lo que MIDEN los sensores, nunca sobre el
        avance estimado: el localizador puede quedarse clavado el solo, y de
        hecho fue lo que enmascaro el problema durante toda la sesion del
        05-09.  Aqui se comparan paredes y rumbo entre los dos extremos de una
        ventana de ``stall_window_s``.

        Hace falta al menos una medida LONGITUDINAL finita (la pared frontal o
        el corredor libre).  Sin ella, un robot que baja una recta paralelo a
        los muros tendria los dos laterales y el rumbo constantes y se
        declararia atascado sin estarlo.
        """

        largo, lados, rumbo = self._medidas_de_atasco(paredes, rumbo_deg)
        if not any(math.isfinite(v) for v in largo):
            self._olvidar_atasco()
            return False

        self._historial_atasco.append((ahora, largo[0], largo[1], lados[0], lados[1], rumbo))
        # Se poda mirando el SEGUNDO, no el primero.  Podando por el primero se
        # tira justo la muestra que completa la ventana: la edad del historial
        # se queda pegada por debajo del umbral y la deteccion no salta nunca.
        # Medido en el robot el 05-09: edad 1,90 s ciclo tras ciclo con la
        # ventana en 2,0, y cero detecciones en 18 s de robot inmovil.
        while (
            len(self._historial_atasco) > 2
            and ahora - self._historial_atasco[1][0] >= self.atasco_ventana_s
        ):
            self._historial_atasco.pop(0)

        if ahora - self._historial_atasco[0][0] < self.atasco_ventana_s:
            return False
        if len(self._historial_atasco) < 6:
            return False

        # Se comparan MEDIANAS de los dos extremos de la ventana, no las dos
        # muestras sueltas.  Medido con el robot inmovil en el banco, la pared
        # frontal oscila 37 mm de amplitud (desviacion 8,4) mientras los dos
        # laterales se mueven 2 mm: comparando muestras sueltas, un solo
        # barrido ruidoso bastaba para declarar que el robot se movia, y el
        # vigilante no disparo ni una vez en 18 s de robot parado.
        tercio = max(3, len(self._historial_atasco) // 3)
        primeras = self._historial_atasco[:tercio]
        ultimas = self._historial_atasco[-tercio:]

        def mediana(muestras, indice):
            valores = [m[indice] for m in muestras if math.isfinite(m[indice])]
            return _mediana(valores) if valores else None

        antes_rumbo, despues_rumbo = mediana(primeras, 5), mediana(ultimas, 5)
        if antes_rumbo is None or despues_rumbo is None:
            return False
        if abs(despues_rumbo - antes_rumbo) > self.atasco_tolerancia_deg:
            self._empujones = 0
            return False

        for indice in (1, 2, 3, 4):
            antes, despues = mediana(primeras, indice), mediana(ultimas, indice)
            if antes is None or despues is None:
                continue
            if abs(despues - antes) > self.atasco_tolerancia_mm:
                # El mundo cambia: el robot se mueve.  Se perdona la cuenta
                # de empujones, o dos atascos lejanos entre si escalarian a
                # retroceso como si fueran el mismo.
                self._empujones = 0
                return False
        return True

    def _vigilar_atasco(
        self, paredes: MapaParedes, rumbo_deg: float, ahora: float
    ) -> Optional[Consigna]:
        """Devuelve una consigna si hay que sacar al robot de un atasco.

        Escala en dos peldaños, del mas barato al mas caro:

        1. **Empujon**: la misma direccion, pero a ``stall_kick_pwm`` durante
           ``stall_kick_s``.  El atasco medido ocurria con el motor en 25 PWM,
           que es ``speed_min_pwm``; lo mas probable es que a esa consigna no
           haya par para vencer el rozamiento estatico desde segun que pose.
           Se salta el limitador de rampa a proposito: rampar de 25 a 55 en
           tres ciclos es justo lo que NO rompe un rozamiento estatico.
        2. Si tras ``stall_max_kicks`` empujones el mundo sigue quieto, ya no
           es par: hay algo trabado.  Se cae al RETROCESO de siempre, que es la
           maniobra de escape ya probada, y se contabiliza como tal.

        Durante el empujon NO se vuelve a medir: la ventana se olvida al
        empezar y se vuelve a llenar al terminar, para no encadenar empujones
        con las medidas del empujon anterior dentro de la ventana.
        """

        if self._empujando:
            if ahora - self._t_empujon < self.atasco_empujon_s:
                return self._emitir(
                    self.atasco_empujon_pwm, self._ultimo_angulo, "empujon de atasco"
                )
            self._empujando = False
            self._olvidar_atasco()
            return None

        if self._ultima_velocidad <= 0:
            # Parado a proposito (o retrocediendo): no hay atasco que detectar.
            self._olvidar_atasco()
            return None

        if not self._detectar_atasco(paredes, rumbo_deg, ahora):
            return None

        self._atascos += 1
        self._olvidar_atasco()

        # Un empujon es acelerar hacia delante.  Solo se hace con sitio de
        # sobra: con el corredor justo, la salida buena es la de siempre.
        hay_sitio = paredes.corredor_mm > self.atasco_empujon_libre_mm
        if hay_sitio and self._empujones < self.atasco_empujones_max:
            self._empujones += 1
            self._empujando = True
            self._t_empujon = ahora
            return self._emitir(
                self.atasco_empujon_pwm, self._ultimo_angulo, "empujon de atasco"
            )

        self._empujones = 0
        self._retrocesos += 1
        self._estado_interrumpido = self.estado
        self._angulo_retroceso = -self._ultimo_angulo
        self._entrar(RETROCESO, ahora)
        return self._procesar_retroceso(paredes, ahora)

    def _hay_emergencia(self, paredes: MapaParedes) -> bool:
        """Solo la ESTRUCTURA dispara emergencias, nunca un pilar.

        Las minimas laterales excluyen los puntos de objeto y el frente se
        juzga por el corredor libre del ancho del robot.  Con la version
        anterior -- minimos crudos de sector -- un poste que se iba a rebasar
        limpiamente por el costado disparaba el retroceso, y en simulacion el
        robot se quedaba en 1 esquina de 12 con la velocidad media en 6 PWM.
        """

        if paredes.corredor_mm < self.emergencia_frontal_mm:
            return True
        return (
            min(paredes.izquierda_min_mm, paredes.derecha_min_mm)
            < self.emergencia_lateral_mm
        )

    # -------------------------------------------------------- orientacion

    def _resolver_sentido(
        self, paredes: MapaParedes, lineas: Sequence[LineaPiso]
    ) -> bool:
        """Fija el sentido de la vuelta.  Devuelve True cuando ya esta claro.

        Dos evidencias, y basta con una:

        1. La LINEA DE PISO que se cruza primero.  Es la señal oficial y la
           usan los dos finalistas de 2025.  En la Pi 5 el sensor de color
           de la Pico responde ``SIN_SENSOR``, asi que se lee con la camara:
           un blob azul o naranja proyectado al suelo por delante del robot.
        2. La ASIMETRIA de las paredes.  El bloque interior siempre esta mas
           cerca que el muro exterior, y se gira HACIA el interior.  Sirve de
           respaldo y no depende de la calibracion de color.
        """

        if self.sentido:
            return True

        # Solo cuentan las lineas que el robot va a CRUZAR: una franja a 650 mm
        # de lado no es la suya.  Sin este filtro, con las dos lineas pintadas
        # cerca de una esquina se elegia la de al lado.
        candidatas = [
            linea
            for linea in lineas
            if linea.y_mm < self.linea_max_avance_mm
            and abs(linea.x_mm) < self.linea_max_lateral_mm
        ]
        if candidatas:
            primera = min(candidatas, key=lambda linea: linea.y_mm)
            if primera.color == self.color_izquierda:
                self.sentido = -1
                return True
            if primera.color == self.color_derecha:
                self.sentido = 1
                return True

        izquierda = paredes.izquierda.distancia_mm if paredes.izquierda else None
        derecha = paredes.derecha.distancia_mm if paredes.derecha else None
        if izquierda is not None and derecha is not None:
            if max(izquierda, derecha) > 1.6 * min(izquierda, derecha):
                self.sentido = 1 if derecha < izquierda else -1
                return True
        return False

    # ------------------------------------------------------------ obstaculos

    def _obstaculos_del_plan(
        self, pilares: Sequence[DeteccionPilar], paredes: MapaParedes
    ) -> List[Tuple[float, float, str]]:
        """Lo que el planificador tiene que esquivar, en el marco de la recta.

        Se mezclan dos fuentes y en este orden de prioridad:

        * Lo que se VE ahora mismo.  Manda siempre: si hay un poste delante, da
          igual lo que diga el mapa.
        * Lo que el MAPA recuerda de esta recta y de la siguiente.  Es lo que
          permite empezar a cambiar de carril antes de tener el poste a la
          vista, y lo que hace que la segunda y la tercera vuelta sean mas
          rapidas que la primera.

        Una casilla recordada se descarta si ya hay una deteccion viva cerca:
        no tiene sentido planificar dos veces el mismo poste.
        """

        salida: List[Tuple[float, float, str]] = []
        vivos: List[Tuple[float, float]] = []

        for pilar in pilares:
            avance = self.localizador.avance_de_punto(pilar.x_mm, pilar.y_mm, paredes)
            offset = self.localizador.offset_de_punto(
                pilar.x_mm, pilar.y_mm, paredes, self.sentido
            )
            if avance is None or offset is None:
                continue
            # Un poste de la recta SIGUIENTE, visto por encima del bloque
            # interior, cae en este marco en un sitio creible y deforma la
            # ruta con un obstaculo que el robot no tiene delante.
            if not self.mapa.pertenece_a_esta_recta(avance, offset):
                continue
            # Si la deteccion viva perdio el color, se HEREDA el de la casilla
            # del mapa donde cae.  De cerca la camara deja de clasificar (13 %
            # de los ciclos por debajo de 250 mm, contra 73-75 % entre 0,75 y
            # 1,25 m), y como la deteccion viva manda sobre el mapa, un poste
            # perfectamente identificado a un metro llegaba al rebase sin
            # color: el planificador elegia lado por el hueco mayor en vez de
            # por la regla.  Medido el 05-09: de 12 rebases, 4 por el lado
            # equivocado, uno con el poste a 6 mm del eje, o sea de frente.
            color = pilar.color
            if not color and self.pose is not None:
                color = self.mapa.color_recordado(self.pose.segmento, avance, offset) or ""

            # Un objeto SIN COLOR lejano no mueve el carril: puede aparecer su
            # color antes de llegar, y desviarse por un bulto sin identificar
            # tiene su propio riesgo.  Pero de cerca hay que esquivarlo igual.
            # "Frenar y seguir recto" sonaba prudente y en pista era chocar:
            # las corridas 3, 4 y 5 del 04/05-09 acumularon 24 retrocesos,
            # casi todos contra un bulto de 55 mm a 150 mm del morro.  El
            # planificador ya sabe rodear sin color, por el hueco mayor.
            if not color and not self.plan_con_sin_color:
                distancia = math.hypot(pilar.x_mm, pilar.y_mm)
                if distancia > self.sin_color_cerca_mm:
                    continue
            salida.append((avance, offset, color))
            vivos.append((avance, offset))

        if self.pose is None:
            return salida

        # Solo las casillas de ESTA recta.  Las de la siguiente estan al otro
        # lado de la esquina: intentar meterlas en el mismo perfil de offset
        # no tiene sentido geometrico, porque el marco cambia al girar.  Se
        # usan igualmente, pero en ``_disparo_de_giro_mm``, que es donde de
        # verdad importan.
        for _casilla, entrada in self.mapa.pilares_del_segmento(self.pose.segmento):
            if not entrada.color:
                continue
            avance, offset = entrada.avance_mm, entrada.offset_mm
            if avance > self.pose.avance_mm + self.plan.margen_detras_mm:
                continue
            if any(
                abs(avance - va) < 300.0 and abs(offset - vo) < 220.0
                for va, vo in vivos
            ):
                continue
            salida.append((avance, offset, entrada.color))
        return salida

    def _memorizar(self, pilares: Sequence[DeteccionPilar], paredes: MapaParedes) -> None:
        if self.pose is None:
            return
        for pilar in pilares:
            if not pilar.color:
                continue
            avance = self.localizador.avance_de_punto(pilar.x_mm, pilar.y_mm, paredes)
            offset = self.localizador.offset_de_punto(
                pilar.x_mm, pilar.y_mm, paredes, self.sentido
            )
            if avance is None or offset is None:
                continue
            self.mapa.observar(
                pilar,
                self.pose.segmento,
                avance,
                offset,
                vuelta=self.esquinas // 4,
            )

    # ------------------------------------------------------------- esquinas

    def _disparo_de_giro_mm(self) -> float:
        """A que distancia del muro de enfrente empezar a girar.

        PROBADO Y DESCARTADO (05-09): disparar TAMBIEN cuando el muro frontal
        ajustado baja del umbral, como red de seguridad frente a un `avance`
        estimado poco fiable.  Barrido determinista sobre la vuelta de ocho
        pilares, inyectando saltos en el avance con probabilidad ``p``::

            p       solo avance        + muro medido
            0,00    0 retrocesos       14 retrocesos, 16 giros para 12 esquinas
            0,10    8-10               6-7
            0,25    8-10               6-7

        Ayuda cuando el localizador esta roto y ARRUINA el caso sano: dispara
        esquinas que no existen -- 16 entradas en GIRO para 12 esquinas -- y la
        ronda se da por terminada en 75 s en vez de 131, a mitad de recorrido.
        `frontal_min` no distingue "el muro que cierra esta recta" de "un muro
        que se ve de frente"; el avance del localizador, con lo malo que es,
        al menos sabe en que recta va.

        SEGUNDO INTENTO, TAMBIEN DESCARTADO (05-09): la idea del 1.er puesto de
        2025 (`ejm22`), disparar por la DERIVADA -- muro frontal cerca Y
        acercandose de forma sostenida 120 mm en 0,6 s.  Con el localizador
        sano da **3 esquinas y 57 retrocesos** contra 12 y 0.  Peor que la
        version ingenua, y la razon es de ARQUITECTURA, no de umbrales:

        `ejm22` no tiene odometria ni marco de recta.  Su detector de esquina
        puede equivocarse porque debajo no hay nada que corromper: son
        reactivos de arriba abajo.  Aqui, contar una esquina gira el rumbo
        cardinal y reinicia el marco a 3000 mm, asi que UNA esquina disparada
        de mas deja el marco desplazado 90 grados para el resto de la vuelta.
        Por eso 3 esquinas y 57 retrocesos: no dispara de mas muchas veces,
        basta con una.

        Su deteccion de esquina no es portable sin su arquitectura.  El valor
        de su enfoque esta en la AUSENCIA del marco, no en el detector.

        Depende del PRIMER pilar de la recta siguiente, que ya esta en el mapa
        aunque todavia no se vea: si toca salir pegado al muro exterior, el
        giro tiene que cerrarse antes; si toca salir por dentro, se abre.  Es
        la idea del segundo puesto de 2025, resuelta con una interpolacion en
        vez de con ocho constantes.
        """

        base = self.giro_disparo_mm
        if self.pose is None:
            return base
        siguientes = self.mapa.pilares_del_segmento((self.pose.segmento + 1) % 4)
        if not siguientes:
            return base
        _casilla, entrada = siguientes[0]
        if not entrada.color:
            return base
        objetivo = self.plan.offset_de_paso(entrada.color, entrada.offset_mm, self.sentido)
        # GEOMETRIA, no interpolacion.  El muro de enfrente de esta recta es el
        # muro EXTERIOR de la siguiente, asi que un giro de 90 grados a radio R
        # convierte la distancia de disparo D en un offset de salida D - R.
        # Para salir donde hay que pasar el poste basta disparar en objetivo+R.
        #
        # La interpolacion anterior se quedaba a medias: para un rojo pegado al
        # muro exterior (objetivo 223) salia a offset 370, con 147 mm todavia
        # por corregir en el primer metro de la recta nueva.  Daniel lo vio en
        # pista antes que yo en el codigo: "al tardar en tomar el giro dificulta
        # que pase con facilidad el bloque".
        radio = self.radio_der_mm if self.sentido > 0 else self.radio_izq_mm
        return _limitar(
            objetivo + radio, self.giro_disparo_min_mm, self.giro_disparo_max_mm
        )

    # ----------------------------------------------------------------- ciclo

    def procesar(
        self,
        paredes: MapaParedes,
        pilares: Sequence[DeteccionPilar] = (),
        lineas: Sequence[LineaPiso] = (),
        magenta: Sequence[ParedMagenta] = (),
        hueco: Optional[HuecoParqueo] = None,
        rumbo_deg: float = 0.0,
        ultrasonido_mm: Optional[float] = None,
        ahora: float = 0.0,
    ) -> Consigna:
        if self.estado == ESPERA:
            self._t_inicio = ahora
            self._entrar(
                SALIDA_BAHIA if self.arranca_en_bahia else ORIENTACION, ahora
            )

        if self.estado not in (FIN,) and ahora - self._t_inicio > self.duracion_max_s:
            self._entrar(FIN, ahora)
            return Consigna(0, 0.0, FIN, "tiempo agotado")

        if self.estado == ORIENTACION:
            return self._procesar_orientacion(paredes, lineas, ahora)

        if self.estado == SALIDA_BAHIA:
            return self._procesar_salida_bahia(paredes, lineas, ahora)

        self._rumbo_absoluto = float(rumbo_deg)
        self.pose = self.localizador.actualizar(
            paredes, rumbo_deg, self.sentido, self._ultima_velocidad, ahora
        )
        self._memorizar(pilares, paredes)

        if self.estado == RETROCESO:
            self._olvidar_atasco()
            return self._procesar_retroceso(paredes, ahora)

        if self.estado in (RECTA, GIRO, APROXIMACION) and self._hay_emergencia(paredes):
            self._retrocesos += 1
            self._estado_interrumpido = self.estado
            # Se congela el volante del retroceso AL ENTRAR.  Calcularlo cada
            # ciclo como ``-self._ultimo_angulo`` lo hacia oscilar a tope de un
            # lado a otro a 10 Hz (visible en la corrida 2 del 04-09: +20/-20/
            # +20 fila tras fila), que ni deshace el atasco ni deja una lectura
            # de LiDAR estable.
            self._angulo_retroceso = -self._ultimo_angulo
            self._entrar(RETROCESO, ahora)
            return self._procesar_retroceso(paredes, ahora)

        # El vigilante va DESPUES de la emergencia a proposito: si el robot
        # esta quieto porque tiene algo encima, lo que toca es retroceder, no
        # empujar mas fuerte contra ello.
        if self.estado in (RECTA, GIRO, APROXIMACION):
            respuesta = self._vigilar_atasco(paredes, rumbo_deg, ahora)
            if respuesta is not None:
                return respuesta

        if self.estado == RECTA:
            return self._procesar_recta(paredes, pilares, ahora)
        if self.estado == GIRO:
            return self._procesar_giro(paredes, rumbo_deg, ahora)
        if self.estado == APROXIMACION:
            return self._procesar_aproximacion(paredes, pilares, magenta, hueco, ahora)
        if self.estado == PARQUEO:
            return self._procesar_parqueo(paredes, hueco, ultrasonido_mm, ahora)
        return Consigna(0, 0.0, self.estado, self._razon, terminado=self.estado == FIN)

    # ------------------------------------------------------------- estados

    def _procesar_orientacion(
        self, paredes: MapaParedes, lineas: Sequence[LineaPiso], ahora: float
    ) -> Consigna:
        if self._resolver_sentido(paredes, lineas):
            self.localizador.reiniciar()
            self._entrar(RECTA, ahora)
            return self._emitir(self.velocidad_giro, 0.0, f"sentido {self.sentido:+d}")
        if self._tiempo(ahora) > self.orientacion_timeout_s:
            # Sin evidencia, horario: es la mitad de las veces y seguir parado
            # es cero puntos seguro.
            self.sentido = 1
            self.localizador.reiniciar()
            self._entrar(RECTA, ahora)
            return self._emitir(self.velocidad_giro, 0.0, "sentido por defecto")
        return self._emitir(self.velocidad_aprox, 0.0, "buscando sentido")

    def _procesar_salida_bahia(
        self, paredes: MapaParedes, lineas: Sequence[LineaPiso], ahora: float
    ) -> Consigna:
        """Sacar el robot del cajon al empezar la ronda de obstaculos.

        Se sale en dos tiempos y por sensores, no por tiempo: primero se
        avanza girando hacia el carril hasta que el muro de al lado se abre, y
        despues se endereza.  El sentido se resuelve en cuanto hay carril.
        """

        if self.sentido == 0:
            self._resolver_sentido(paredes, lineas)
        giro = -float(self.sentido or 1) * self.plan.conversor.mando_max_izq
        lateral = min(paredes.izquierda_min_mm, paredes.derecha_min_mm)
        if self._tiempo(ahora) > 0.6 and lateral > 260.0:
            self.localizador.reiniciar()
            self._entrar(ORIENTACION, ahora)
            return self._emitir(self.velocidad_aprox, 0.0, "fuera de la bahia")
        if self._tiempo(ahora) > 4.0:
            self._entrar(ORIENTACION, ahora)
            return self._emitir(self.velocidad_aprox, 0.0, "salida por tiempo")
        return self._emitir(self.velocidad_aprox, giro, "saliendo de la bahia")

    def _procesar_recta(
        self,
        paredes: MapaParedes,
        pilares: Sequence[DeteccionPilar],
        ahora: float,
    ) -> Consigna:
        assert self.pose is not None
        obstaculos = self._obstaculos_del_plan(pilares, paredes)
        self.ruta = self.plan.construir_ruta(self.pose, self.sentido, obstaculos)
        angulo, objetivo, error = self.plan.direccion(
            self.pose, self.ruta, self.sentido, self._ultima_velocidad
        )
        self._objetivo_offset = objetivo
        self._error_lateral = error

        techo = self._techo_de_vuelta()
        if self.plan.ultimo_plan_cedido:
            # El plan no cabe en la geometria: mas lento no acorta la
            # distancia, pero multiplica los ciclos de control por milimetro y
            # con eso el seguimiento mejora.
            cedido = max(self.plan.velocidad_min, int(self.plan.velocidad_crucero * 0.6))
            techo = cedido if techo is None else min(techo, cedido)
        velocidad = self.plan.velocidad(self.pose, error, paredes.corredor_mm, techo)

        disparo = self._disparo_de_giro_mm()
        refractario = ahora - self._ultima_esquina_s > self.giro_refractario_s
        if self.pose.avance_valido and self.pose.avance_mm < disparo and refractario:
            self._rumbo_giro_ref = self._rumbo_absoluto
            self._entrar(GIRO, ahora)
            return self._emitir(self.velocidad_giro, self._angulo_de_giro(), "entrando en giro")

        return self._emitir(velocidad, angulo, f"carril {objetivo:.0f}mm")

    def _techo_de_vuelta(self) -> Optional[int]:
        """Techo de velocidad segun lo que ya se sabe de la pista.

        La vuelta 1 se corre para mapear.  De la 2 en adelante el mapa ya sabe
        que hay en cada casilla antes de verlo, el carril se prepara con mas
        antelacion y se puede correr mas.  Los tres frenos de
        ``PlanificadorCarril.velocidad`` siguen mandando por debajo: esto solo
        sube el techo, asi que solo corre mas donde el frente esta libre, el
        error lateral es pequeño y el rumbo va derecho.
        """

        if self.ganancia_vuelta_conocida <= 1.0:
            return None
        if self.esquinas // 4 < 1:
            return None
        return int(round(self.plan.velocidad_crucero * self.ganancia_vuelta_conocida))

    def _angulo_de_giro(self) -> float:
        """Volante a tope hacia el interior de la pista."""

        return (
            -self.plan.conversor.mando_max_der
            if self.sentido > 0
            else self.plan.conversor.mando_max_izq
        )

    def _procesar_giro(
        self, paredes: MapaParedes, rumbo_deg: float, ahora: float
    ) -> Consigna:
        """Giro de esquina: volante a tope y salida por rumbo de la IMU.

        Se sale cuando el rumbo ha cambiado lo suficiente Y el frente se ha
        vuelto a abrir.  Exigir las dos cosas evita el fallo clasico de contar
        una esquina que no se llego a completar: la bitacora del equipo tiene
        corridas donde el conteo iba por delante de la realidad y el parqueo
        se disparaba en la recta equivocada.
        """

        girado = abs(rumbo_deg - self._rumbo_giro_ref)
        angulo = self._angulo_de_giro()

        frente_abierto = (
            not math.isfinite(paredes.corredor_mm)
            or paredes.corredor_mm > self.giro_salida_mm
        )
        if girado >= self.giro_rumbo_min_deg and frente_abierto:
            return self._terminar_giro(ahora, "esquina completa")
        # Una esquina son 90 grados.  Si ya se paso de largo, el frente cerrado
        # no puede seguir mandando: en la corrida 2 del 04-09 un eco espurio a
        # 150 mm mantuvo ``frente_abierto`` en falso y el robot encadeno 155
        # grados en un solo giro.  Pasado el tope se cierra la esquina y que la
        # recta corrija el rumbo, que para eso tiene realimentacion.
        if girado >= self.giro_rumbo_max_deg:
            return self._terminar_giro(ahora, "esquina pasada de rumbo")
        if self._tiempo(ahora) > self.giro_timeout_s:
            return self._terminar_giro(ahora, "esquina por tiempo")
        return self._emitir(self.velocidad_giro, angulo, f"girando {girado:.0f}deg")

    def _terminar_giro(self, ahora: float, razon: str) -> Consigna:
        self.esquinas += 1
        self._ultima_esquina_s = ahora
        self.localizador.anotar_esquina(self.sentido)
        if (
            self.permitir_parqueo
            and self.parqueo is not None
            and self.esquinas >= self.esquinas_objetivo
        ):
            self._entrar(APROXIMACION, ahora)
            return self._emitir(self.velocidad_aprox, 0.0, "buscando la bahia")
        if not self.permitir_parqueo and self.esquinas >= self.esquinas_objetivo:
            self._entrar(FIN, ahora)
            return Consigna(0, 0.0, FIN, "vueltas completas", terminado=True)
        self._entrar(RECTA, ahora)
        return self._emitir(self.velocidad_giro, 0.0, razon)

    def _procesar_aproximacion(
        self,
        paredes: MapaParedes,
        pilares: Sequence[DeteccionPilar],
        magenta: Sequence[ParedMagenta],
        hueco: Optional[HuecoParqueo],
        ahora: float,
    ) -> Consigna:
        """Acercarse al cajon pegado al muro exterior, listo para entrar.

        La aproximacion NO usa el detector LiDAR del hueco para decidir por
        donde ir: usa los muros magenta que ve la camara, que aparecen mucho
        antes.  El LiDAR entra despues, para confirmar el hueco con precision
        de milimetros cuando ya se esta al lado.
        """

        assert self.pose is not None
        lado = self._lado_de_bahia()
        objetivo = (
            self.plan.medio_robot_mm + self.plan.holgura_pared_mm + 60.0
            if lado * self.sentido > 0
            else self.plan.ancho_carril_mm
            - (self.plan.medio_robot_mm + self.plan.holgura_pared_mm + 60.0)
        )
        pose_objetivo = PoseCarril(
            timestamp=self.pose.timestamp,
            segmento=self.pose.segmento,
            avance_mm=self.pose.avance_mm,
            offset_mm=self.pose.offset_mm,
            rumbo_error_deg=self.pose.rumbo_error_deg,
        )
        ruta = [NodoRuta(self.pose.avance_mm + 4000.0, objetivo, "bahia")]
        angulo, _objetivo, error = self.plan.direccion(
            pose_objetivo, ruta, self.sentido, self.velocidad_aprox
        )
        self._objetivo_offset = objetivo
        self._error_lateral = error

        if hueco is not None and abs(error) < 90.0:
            self._entrar(PARQUEO, ahora)
            return self._procesar_parqueo(paredes, hueco, None, ahora)

        if self.pose.avance_valido and self.pose.avance_mm < self.giro_disparo_min_mm:
            self._rumbo_giro_ref = self._rumbo_absoluto
            self._entrar(GIRO, ahora)
            return self._emitir(self.velocidad_giro, self._angulo_de_giro(), "entrando en giro")

        return self._emitir(self.velocidad_aprox, angulo, "aproximando a la bahia")

    def _lado_de_bahia(self) -> int:
        """La bahia esta siempre contra el muro EXTERIOR de su recta."""

        return 1 if self.sentido < 0 else -1

    def _procesar_parqueo(
        self,
        paredes: MapaParedes,
        hueco: Optional[HuecoParqueo],
        ultrasonido_mm: Optional[float],
        ahora: float,
    ) -> Consigna:
        if self.parqueo is None:
            self._entrar(FIN, ahora)
            return Consigna(0, 0.0, FIN, "sin modulo de parqueo", terminado=True)
        resultado = self.parqueo.procesar(
            paredes=paredes,
            hueco=hueco,
            ultrasonido_mm=ultrasonido_mm,
            lado=self._lado_de_bahia(),
            ahora=ahora,
        )
        if resultado.terminado:
            self._entrar(FIN, ahora)
        return Consigna(
            velocidad=resultado.velocidad,
            angulo=resultado.angulo,
            estado=f"{PARQUEO}:{resultado.estado}",
            razon=resultado.razon,
            terminado=resultado.terminado,
            verificado=resultado.verificado,
        )

    def _procesar_retroceso(self, paredes: MapaParedes, ahora: float) -> Consigna:
        """Sacar el robot de un encajonamiento y devolverlo a la conduccion.

        Se retrocede girando AL CONTRARIO de como se entro, que es lo que
        deshace el atasco en vez de repetirlo.  La salida exige que el frente
        se haya abierto durante un tiempo minimo, para no salir y volver a
        entrar en el mismo ciclo.
        """

        angulo = self._angulo_retroceso
        transcurrido = self._tiempo(ahora)
        libre = paredes.corredor_mm > self.retroceso_salida_mm
        if transcurrido > self.retroceso_min_s and libre:
            return self._salir_de_retroceso(ahora, "recuperado")
        if transcurrido > self.retroceso_max_s:
            return self._salir_de_retroceso(ahora, "retroceso agotado")
        return self._emitir(self.velocidad_retroceso, angulo, "retrocediendo")

    def _salir_de_retroceso(self, ahora: float, razon: str) -> Consigna:
        """Devolver el control al estado que la emergencia interrumpio.

        Salir SIEMPRE a RECTA perdia la esquina: en la primera corrida con
        motores (04-09) el robot giro los 90 grados, un retroceso lo saco del
        rincon y la esquina nunca se conto.  Con el contador congelado el
        localizador seguia creyendose en el segmento anterior, el rumbo de
        referencia quedo 90 grados equivocado durante toda la recta siguiente
        y el robot la hizo pegado al muro interior (148 mm).

        No se acredita la esquina aqui: se REANUDA el giro con la misma
        referencia de rumbo, y la acredita ``_procesar_giro`` cuando de
        verdad se cumplan sus dos condiciones.
        """

        destino = GIRO if self._estado_interrumpido == GIRO else RECTA
        self._estado_interrumpido = ""
        self._entrar(destino, ahora)
        if destino == GIRO:
            return self._emitir(
                self.velocidad_giro, self._angulo_de_giro(), f"{razon}: sigue el giro"
            )
        return self._emitir(self.plan.velocidad_min, 0.0, razon)

    # ------------------------------------------------------------ telemetria

    def instantanea(self) -> Dict[str, Any]:
        """Todo lo que el CSV y el panel quieren saber, sin tocar internos."""

        pose = self.pose
        return {
            "estado": self.estado,
            "razon": self._razon,
            "sentido": self.sentido,
            "esquinas": self.esquinas,
            "segmento": pose.segmento if pose else -1,
            "avance": round(pose.avance_mm, 1) if pose else "",
            "offset": round(pose.offset_mm, 1) if pose else "",
            "rumbo_error": round(pose.rumbo_error_deg, 2) if pose else "",
            "avance_valido": int(pose.avance_valido) if pose else 0,
            "offset_valido": int(pose.offset_valido) if pose else 0,
            "objetivo_offset": round(self._objetivo_offset, 1),
            "error_lateral": round(self._error_lateral, 1),
            "plan_cedido": int(self.plan.ultimo_plan_cedido),
            "mapa": self.mapa.resumen(),
            "casillas": self.mapa.casillas_conocidas(),
            "retrocesos": self._retrocesos,
            "atascos": self._atascos,
        }
