"""Estacionamiento en la bahia magenta, en varios tiempos y por sensores.

QUE SE SABE YA DE ESTE PROBLEMA, MEDIDO EN PISTA
* El detector del hueco FUNCIONA desde el 03-09: partir los clusters en rectas
  antes del PCA subio de 0 emparejamientos en 22 barridos a 9 de 10, con la
  separacion estable en 389-391 mm.  Ese detector se conserva tal cual en
  ``percepcion_lidar.buscar_hueco``.
* La maniobra de DOS ARCOS no cabe.  Rehecha la simulacion el 06-09 con las
  medidas de regla (bahia util 330 x 200, robot 210 x 130, 140 a tope de
  volante, radios 228 a la izquierda y 260 a la derecha) y barriendo todas las
  aproximaciones, posiciones finales, rumbos pico y puntos de arranque: faltan
  18 mm en el mejor caso, y no cabe con ningun voladizo entre 40 y 80 mm.  El
  techo geometrico es r <= 180 mm y este chasis no llega.
* El lado de entrada NO es indiferente.  El corrimiento lateral si es
  simetrico en r_entrada + r_salida, pero la envolvente barrida no: el radio
  grande en el arco de ENTRADA pide 348 mm de hueco y en el de enderezado 363.

LA CONSECUENCIA DE DISENO
No se intenta clavar la maniobra de una vez.  Se entra en VARIOS TIEMPOS, como
un coche aparca en linea de verdad: arco de entrada, arco de enderezado, y si
falta sitio, tantos vaivenes cortos como haga falta hasta quedar dentro y
paralelo.  Cada tramo se corta por MEDIDA -- rumbo alcanzado o holgura minima
--, nunca por tiempo, asi que la maniobra no depende de acertar el radio: se
adapta al que el chasis tenga ese dia.  Y como solo hay que recuperar 18 mm,
un par de vaivenes sobra.

DE DONDE SALE CADA MEDIDA, Y CUAL NO EXISTE
* Paralelismo: la recta de la pared del lado de la bahia si el LiDAR la ve, y
  si no el rumbo de la IMU contra la referencia tomada al salir de ALINEAR.
* Holgura trasera: SOLO el ultrasonido.  La pared trasera del LiDAR es una
  extrapolacion de los hombros y en la pose aparcada del 06-09 daba 1777 mm
  contra 44 del ultrasonido (ver ``_trasera_mm``).
* Distancia recorrida: NO EXISTE.  No hay encoders.  Por eso ningun tramo se
  corta por "ya he avanzado tanto", y los relojes que quedan son red de
  seguridad hacia FALLO, nunca criterio de avance.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

from .modelos import Consigna, HuecoParqueo, MapaParedes


BUSCAR = "BUSCAR"
ALINEAR = "ALINEAR"
ARCO_ENTRADA = "ARCO_ENTRADA"
ARCO_ENDEREZA = "ARCO_ENDEREZA"
VAIVEN_ATRAS = "VAIVEN_ATRAS"
VAIVEN_ADELANTE = "VAIVEN_ADELANTE"
CENTRAR = "CENTRAR"
VERIFICAR = "VERIFICAR"
LISTO = "LISTO"
FALLO = "FALLO"


def _limitar(valor: float, minimo: float, maximo: float) -> float:
    return max(minimo, min(maximo, valor))


def _valida(valor: Any) -> bool:
    try:
        numero = float(valor)
    except (TypeError, ValueError):
        return False
    return math.isfinite(numero) and 0.0 < numero < 6000.0


class ControlEstacionamiento:
    """FSM del parqueo.  Entra medida, sale consigna; ningun I/O."""

    def __init__(self, config: Dict[str, Any]):
        parqueo = config.get("parking", {})
        control = config.get("control", {})
        chasis = config.get("chassis", {})

        self.largo_bahia_mm = float(parqueo.get("bay_length_mm", 330.0))
        self.largo_robot_mm = float(chasis.get("length_mm", 210.0))
        self.ancho_robot_mm = float(chasis.get("width_mm", 130.0))
        self.voladizo_mm = float(chasis.get("rear_overhang_mm", 55.0))
        self.lidar_a_derecha_mm = float(chasis.get("lidar_to_right_edge_mm", 66.0))
        self.lidar_a_izquierda_mm = float(chasis.get("lidar_to_left_edge_mm", 63.0))
        # El ultrasonido no va en el paragolpes: esta 34 mm POR DELANTE del
        # punto mas atrasado del robot (regla, 06-09).  Su lectura no es la
        # holgura de la culata hasta que se descuentan esos 34 mm; en la pose
        # aparcada, los 44 de lectura eran 10 mm de sitio real.
        self.ultrasonido_a_culata_mm = float(
            chasis.get("ultrasound_rear_to_tail_mm", 34.0)
        )
        self.radio_izq_mm = float(chasis.get("turn_radius_left_mm", 228.0))
        self.radio_der_mm = float(chasis.get("turn_radius_right_mm", 260.0))

        self.velocidad = int(parqueo.get("speed_pwm", 22))
        self.velocidad_reversa = -abs(int(parqueo.get("speed_reverse_pwm", 22)))
        self.mando_max_izq = abs(float(control.get("steering_max_left_deg", 25.0)))
        self.mando_max_der = abs(float(control.get("steering_max_right_deg", 20.0)))

        self.lateral_objetivo_mm = float(parqueo.get("approach_lateral_mm", 270.0))
        self.alineacion_objetivo_mm = float(parqueo.get("align_target_mm", 40.0))
        self.rumbo_entrada_deg = float(parqueo.get("entry_heading_delta_deg", 42.0))
        self.tolerancia_paralelo_deg = float(parqueo.get("parallel_tolerance_deg", 6.0))
        self.holgura_trasera_min_mm = float(parqueo.get("min_rear_clearance_mm", 70.0))
        self.holgura_frontal_min_mm = float(parqueo.get("min_front_clearance_mm", 70.0))
        self.lateral_dentro_mm = float(parqueo.get("inside_lateral_mm", 140.0))
        self.tolerancia_lateral_mm = float(parqueo.get("lateral_tolerance_mm", 45.0))
        self.rumbo_vaiven_deg = float(parqueo.get("shuffle_heading_deg", 12.0))
        self.radio_arco_entrada_mm = float(parqueo.get("entry_arc_radius_mm", 260.0))
        self.max_vaivenes = int(parqueo.get("max_shuffles", 6))
        self.barridos_verificacion = int(parqueo.get("verify_scans", 3))
        self.barridos_sin_trasera = max(1, int(parqueo.get("rear_evidence_scans", 3)))
        self.edad_hueco_max_s = float(parqueo.get("bay_max_age_s", 0.5))

        self.timeout_busqueda_s = float(parqueo.get("search_timeout_s", 14.0))
        self.timeout_tramo_s = float(parqueo.get("leg_timeout_s", 6.0))
        self.timeout_verificacion_s = float(parqueo.get("verify_timeout_s", 3.0))
        self.timeout_total_s = float(parqueo.get("total_timeout_s", 40.0))

        self.usar_ultrasonido = bool(parqueo.get("ultrasound_rear_enabled", True))
        self.ultrasonido_min_mm = float(parqueo.get("ultrasound_rear_min_mm", 20.0))
        self.ultrasonido_max_mm = float(parqueo.get("ultrasound_rear_max_mm", 1200.0))

        self.reiniciar()

    # --------------------------------------------------------------- estado

    def reiniciar(self) -> None:
        self.estado = BUSCAR
        self.hueco: Optional[HuecoParqueo] = None
        self._t_estado = 0.0
        # None y no 0.0: el reloj monotono puede empezar en cualquier valor, y
        # con 0.0 de centinela la primera llamada en t=0 no armaba el timeout.
        self._t_inicio = None
        self._rumbo = 0.0
        self._rumbo_ref = 0.0
        # Rumbo con el robot paralelo al muro, tomado al salir de ALINEAR.  Es
        # la referencia que deja medir el paralelismo con la IMU cuando el
        # LiDAR pierde la pared, que dentro de la bahia es casi siempre.
        self._rumbo_paralelo: Optional[float] = None
        self._vaivenes = 0
        self._confirmaciones = 0
        self._recentrados = 0
        self._ciegos_atras = 0
        self._fuente_paralelo = ""
        self._razon = ""
        self._lado = -1

    def _entrar(self, estado: str, ahora: float, rumbo: Optional[float] = None) -> None:
        """Cambia de tramo y CONGELA el rumbo con el que empieza.

        Sin encoders, ``rumbo - self._rumbo_ref`` es lo unico que dice cuanto
        ha progresado un tramo, asi que la referencia se toma sola en cada
        transicion en vez de depender de que cada llamada se acuerde de
        pasarla.
        """

        if estado != self.estado:
            self.estado = estado
            self._t_estado = ahora
            self._rumbo_ref = self._rumbo if rumbo is None else float(rumbo)
            # Las confirmaciones son de ESTA visita a VERIFICAR.  Heredar las
            # de la anterior deja cerrar el parqueo con un solo barrido bueno
            # despues de haberse movido, que es justo lo que la verificacion
            # existe para impedir.
            if estado == VERIFICAR:
                self._confirmaciones = 0

    def _tiempo(self, ahora: float) -> float:
        return max(0.0, ahora - self._t_estado)

    def _girado_deg(self) -> float:
        """Rumbo ganado desde que empezo el tramo actual, en grados."""

        return abs(self._rumbo - self._rumbo_ref)

    # ------------------------------------------------------------ geometria

    def _lateral_mm(self, paredes: MapaParedes) -> Optional[float]:
        """Distancia al muro exterior, que es contra el que esta la bahia."""

        pared = paredes.izquierda if self._lado < 0 else paredes.derecha
        if pared is None:
            return None
        return pared.distancia_mm

    def _trasera_mm(
        self, paredes: MapaParedes, ultrasonido_mm: Optional[float]
    ) -> Optional[float]:
        """Holgura entre la culata y lo que haya detras.  SOLO ultrasonido.

        MEDIDO EL 06-09 EN LA POSE APARCADA: el ultrasonido leia 44 mm y la
        "pared trasera" del LiDAR 1777, con calidad 0,95 y 3,0 mm de residuo.
        No es ruido y no se arregla con un umbral.  El LiDAR esta ciego justo
        hacia atras -- el soporte del propio ultrasonido ocupa 140..213 grados
        y ``blind_sectors_deg`` lo enmascara --, asi que ``paredes.trasera``
        se reconstruye ajustando los hombros en oblicuo: es una extrapolacion
        limpia, convincente y equivocada por metro y medio, y se equivoca en
        el sentido peligroso, diciendo LIBRE.  Por eso aqui no participa, ni
        siquiera para bajar la medida buena; ``paredes`` se sigue recibiendo
        para dejar constancia de que se miro y se descarto a proposito.

        Sin ultrasonido valido se devuelve ``None``, que en todo este modulo
        significa SIN EVIDENCIA y jamas "hay sitio": los tramos que dependen
        de la culata tienen que cortar por otra cosa o fallar (ver
        ``_sin_evidencia_atras``), nunca seguir retrocediendo a ciegas.
        """

        if not self.usar_ultrasonido or not _valida(ultrasonido_mm):
            return None
        lectura = float(ultrasonido_mm)
        if not self.ultrasonido_min_mm <= lectura <= self.ultrasonido_max_mm:
            return None
        return max(0.0, lectura - self.ultrasonido_a_culata_mm)

    def _paralelo(self, paredes: MapaParedes) -> Optional[float]:
        """Error de paralelismo contra el muro exterior, en grados."""

        pared = paredes.izquierda if self._lado < 0 else paredes.derecha
        if pared is None:
            return None
        # La normal de un muro lateral perfecto vale -90 o +90 grados.
        referencia = -90.0 if self._lado < 0 else 90.0
        return pared.angulo_deg - referencia

    def _paralelo_medido(self, paredes: MapaParedes) -> Tuple[Optional[float], str]:
        """Paralelismo con la fuente que lo dio: LiDAR primero, IMU despues.

        El LiDAR manda cuando ve la pared, porque mide contra el muro de
        verdad y no acumula deriva.  Pero con el robot cruzado 42 grados
        dentro de un hueco de 330 mm no la ve casi nunca, y ese es justo el
        momento en que hay que decidir si el arco ya termino.  Entonces entra
        la IMU: girar a la IZQUIERDA es rumbo POSITIVO, y eso lleva la normal
        del muro izquierdo de -90 a -90-delta, o sea que el paralelismo es
        MENOS el rumbo ganado desde que el robot estaba paralelo.  Con las dos
        fuentes cada arco tiene siempre un criterio medible; sin la segunda
        solo le quedaba el reloj, que es lo que hacia la maniobra irrepetible.
        """

        del_lidar = self._paralelo(paredes)
        if del_lidar is not None:
            return del_lidar, "lidar"
        if self._rumbo_paralelo is None:
            return None, ""
        return -(self._rumbo - self._rumbo_paralelo), "imu"

    def _hueco_fresco(self, paredes: MapaParedes) -> bool:
        """El hueco guardado se vio hace poco, no es un recuerdo.

        ``confirmar_hueco`` devuelve el ultimo hueco confirmado mientras no
        aparezca otro, asi que ``self.hueco`` no se pone a None solo.  Un
        hueco viejo mueve el robot contra una referencia que ya no esta
        delante, y eso es conducir a ciegas con cara de medida.
        """

        if self.hueco is None:
            return False
        return (paredes.timestamp - self.hueco.timestamp) <= self.edad_hueco_max_s

    def _sin_evidencia_atras(self, trasera: Optional[float]) -> bool:
        """Cuenta barridos seguidos sin lectura trasera y dice si ya sobran.

        Un barrido suelto sin eco es normal -- el ultrasonido pierde tramas --
        y abortar por uno solo dejaria la maniobra a merced del ruido.  Tres
        seguidos ya no son ruido: son retroceder sin saber que hay detras, y
        tocar un delimitador es la ronda entera (regla 13.7).
        """

        if trasera is None:
            self._ciegos_atras += 1
        else:
            self._ciegos_atras = 0
        return self._ciegos_atras >= self.barridos_sin_trasera

    def _mando(self, hacia_bahia: bool, reversa: bool) -> float:
        """Volante a tope, con el signo que toca.

        Marcha atras invierte el efecto del volante sobre la trayectoria del
        eje trasero: para meter la culata en la bahia hay que girar las ruedas
        hacia el lado CONTRARIO al que se quiere ir.
        """

        hacia_izquierda = (self._lado < 0) == (not reversa)
        if not hacia_bahia:
            hacia_izquierda = not hacia_izquierda
        return self.mando_max_izq if hacia_izquierda else -self.mando_max_der

    def _mando_arco_entrada(self) -> float:
        """Volante del primer arco, forzado al radio GRANDE.

        Medido el 06-09: el arco de entrada trazado con el radio grande pide
        348 mm de hueco y con el chico 363, sobre una bahia de 330.  Son 15 mm
        gratis, y el lado no se elige: lo fija contra que muro esta la bahia.
        Con la bahia a la IZQUIERDA el tope ya da el radio grande (260, ruedas
        a la derecha) y esto no cambia nada.  Con la bahia a la DERECHA el
        tope daria 228, asi que se rebaja el volante en proporcion inversa al
        radio -- 25 * 228/260 = 21,9 grados --, que es el unico modelo que
        casa con las dos medidas del chasis; el Ackermann teorico no, porque
        con batalla 140 predice 31,5 grados donde el chasis real da 25.

        PENDIENTE DEL 06-09: 228 y 260 son de marcha ADELANTE.  La inferencia
        de la IMU da ~306 en reversa, un 34 % peor, pero es una inferencia y
        no una medida.  Cuando se cierre con cinta hay que rehacer esto.
        """

        angulo = self._mando(hacia_bahia=True, reversa=True)
        radio_tope = self.radio_izq_mm if angulo > 0.0 else self.radio_der_mm
        objetivo = max(self.radio_arco_entrada_mm, radio_tope)
        if radio_tope <= 0.0 or objetivo <= 0.0:
            return angulo
        return angulo * (radio_tope / objetivo)

    def _resultado(
        self, velocidad: int, angulo: float, razon: str, **extra
    ) -> Consigna:
        self._razon = razon
        return Consigna(
            velocidad=int(velocidad),
            angulo=float(angulo),
            estado=self.estado,
            razon=razon,
            **extra,
        )

    def _fallar(self, razon: str, ahora: float) -> Consigna:
        """Terminar la maniobra parado y con las ruedas rectas.

        Un FALLO tiene que dejar el robot inocuo: velocidad 0 y volante
        centrado.  Salir de un vaiven a medias, con el volante a tope y el
        motor todavia mandado, es como se toca un delimitador justo despues de
        haber decidido rendirse.
        """

        self._entrar(FALLO, ahora)
        self._razon = razon
        return Consigna(0, 0.0, FALLO, razon, terminado=True, verificado=False)

    # ---------------------------------------------------------------- ciclo

    def procesar(
        self,
        paredes: MapaParedes,
        hueco: Optional[HuecoParqueo],
        ultrasonido_mm: Optional[float],
        lado: int,
        ahora: float,
        rumbo_deg: float = 0.0,
    ) -> Consigna:
        self._lado = int(lado) or -1
        self._rumbo = float(rumbo_deg)
        if self._t_inicio is None:
            self._t_inicio = ahora
            self._t_estado = ahora
            self._rumbo_ref = self._rumbo

        if self.estado in (LISTO, FALLO):
            return Consigna(
                0,
                0.0,
                self.estado,
                self._razon,
                terminado=True,
                verificado=self.estado == LISTO,
            )

        if ahora - self._t_inicio > self.timeout_total_s:
            return self._fallar("parqueo agotado", ahora)

        if hueco is not None:
            self.hueco = hueco

        lateral = self._lateral_mm(paredes)
        trasera = self._trasera_mm(paredes, ultrasonido_mm)
        paralelo, self._fuente_paralelo = self._paralelo_medido(paredes)

        if self.estado == BUSCAR:
            return self._buscar(paredes, ahora)
        if self.estado == ALINEAR:
            return self._alinear(paredes, lateral, ahora)
        if self.estado == ARCO_ENTRADA:
            return self._arco_entrada(paralelo, trasera, ahora)
        if self.estado == ARCO_ENDEREZA:
            return self._arco_endereza(paralelo, lateral, trasera, ahora)
        if self.estado == VAIVEN_ADELANTE:
            return self._vaiven_adelante(paredes, ahora)
        if self.estado == VAIVEN_ATRAS:
            return self._vaiven_atras(paralelo, lateral, trasera, ahora)
        if self.estado == CENTRAR:
            return self._centrar(trasera, ahora)
        if self.estado == VERIFICAR:
            return self._verificar(paredes, lateral, paralelo, trasera, ahora)
        return self._fallar(f"estado desconocido {self.estado}", ahora)

    # --------------------------------------------------------------- tramos

    def _buscar(self, paredes: MapaParedes, ahora: float) -> Consigna:
        if self._hueco_fresco(paredes):
            self._entrar(ALINEAR, ahora)
            return self._resultado(self.velocidad, 0.0, "hueco confirmado")
        if self._tiempo(ahora) > self.timeout_busqueda_s:
            return self._fallar("timeout buscando hueco", ahora)
        return self._resultado(self.velocidad, 0.0, "buscando hueco")

    def _alinear(
        self, paredes: MapaParedes, lateral: Optional[float], ahora: float
    ) -> Consigna:
        """Avanzar hasta que el eje trasero quede a la altura de la bahia.

        La referencia es ``borde_delantero_y_mm`` del hueco, medido por el
        LiDAR en el marco del robot: cuando ese borde queda ligeramente por
        delante del LiDAR, el eje trasero esta en el sitio desde el que el
        arco entra.  No hay ningun tiempo ni ninguna cuenta de encoder de por
        medio, que es lo que hacia la maniobra irrepetible.

        Es ademas el ultimo instante en que el robot esta paralelo al muro con
        el hueco a la vista, asi que aqui se toma la referencia de rumbo que
        van a usar los dos arcos.
        """

        if not self._hueco_fresco(paredes):
            self._entrar(BUSCAR, ahora)
            return self._resultado(self.velocidad, 0.0, "hueco perdido")

        objetivo = self.alineacion_objetivo_mm
        error = self.hueco.borde_delantero_y_mm - objetivo

        correccion = 0.0
        if lateral is not None:
            # Mantenerse a distancia constante del muro mientras se alinea:
            # entrar torcido cuesta un vaiven de mas.  El signo es
            # ``* self._lado`` y no ``* -self._lado``: con la bahia a la
            # IZQUIERDA (lado -1) y el robot mas cerca del muro de lo que pide
            # approach_lateral_mm hay que apartarse, o sea girar a la DERECHA,
            # o sea angulo NEGATIVO -- el positivo son las ruedas a la
            # izquierda.  Al reves era realimentacion positiva: cuanto mas
            # cerca del muro, mas se giraba hacia el.
            correccion = _limitar(
                (self.lateral_objetivo_mm - lateral) * 0.06 * self._lado, -8.0, 8.0
            )

        if abs(error) <= 35.0:
            self._rumbo_paralelo = self._rumbo
            self._entrar(ARCO_ENTRADA, ahora)
            return self._resultado(0, 0.0, "alineado con la bahia")
        if self._tiempo(ahora) > self.timeout_tramo_s:
            return self._fallar("alineacion sin cerrar por medida", ahora)
        velocidad = self.velocidad if error > 0 else self.velocidad_reversa
        return self._resultado(velocidad, correccion, f"alineando {error:+.0f}mm")

    def _arco_entrada(
        self, paralelo: Optional[float], trasera: Optional[float], ahora: float
    ) -> Consigna:
        """Primer arco en reversa, metiendo la culata en la bahia.

        QUE PASA SI SE PIERDE EL HUECO A MITAD DE ESTE TRAMO: nada, y es a
        proposito.  Al cruzarse dentro de un hueco de 330 mm los dos
        delimitadores dejan de emparejarse y el detector se cae; si el arco
        dependiera de el, la maniobra se romperia justo en el unico sitio
        donde no se puede parar.  El arco corta por la culata y por el rumbo,
        que son las dos cosas que siguen midiendose con el robot cruzado.
        """

        if self._sin_evidencia_atras(trasera):
            return self._fallar("sin evidencia trasera en el arco", ahora)
        if trasera is not None and trasera < self.holgura_trasera_min_mm:
            self._entrar(VAIVEN_ADELANTE, ahora)
            return self._resultado(0, 0.0, "culata cerca, vaiven")
        if paralelo is not None and abs(paralelo) >= self.rumbo_entrada_deg:
            self._entrar(ARCO_ENDEREZA, ahora)
            return self._resultado(0, 0.0, "angulo de entrada alcanzado")
        if self._tiempo(ahora) > self.timeout_tramo_s:
            return self._fallar("arco de entrada sin corte por medida", ahora)
        return self._resultado(
            self.velocidad_reversa,
            self._mando_arco_entrada(),
            f"arco de entrada ({self._fuente_paralelo or 'a ciegas'})",
        )

    def _arco_endereza(
        self,
        paralelo: Optional[float],
        lateral: Optional[float],
        trasera: Optional[float],
        ahora: float,
    ) -> Consigna:
        """Segundo arco en reversa, enderezando dentro de la bahia."""

        if self._sin_evidencia_atras(trasera):
            return self._fallar("sin evidencia trasera enderezando", ahora)
        if trasera is not None and trasera < self.holgura_trasera_min_mm:
            if self._dentro(lateral, paralelo):
                self._entrar(CENTRAR, ahora)
                return self._resultado(0, 0.0, "dentro, a centrar")
            self._entrar(VAIVEN_ADELANTE, ahora)
            return self._resultado(0, 0.0, "culata cerca, vaiven")
        if paralelo is not None and abs(paralelo) <= self.tolerancia_paralelo_deg:
            if self._dentro(lateral, paralelo):
                self._entrar(CENTRAR, ahora)
                return self._resultado(0, 0.0, "paralelo y dentro")
            self._entrar(VAIVEN_ADELANTE, ahora)
            return self._resultado(0, 0.0, "paralelo pero fuera")
        if self._tiempo(ahora) > self.timeout_tramo_s:
            return self._fallar("enderezado sin corte por medida", ahora)
        return self._resultado(
            self.velocidad_reversa,
            self._mando(hacia_bahia=False, reversa=True),
            "enderezando",
        )

    def _dentro(self, lateral: Optional[float], paralelo: Optional[float]) -> bool:
        if lateral is None or paralelo is None:
            return False
        return (
            lateral <= self.lateral_dentro_mm + self.tolerancia_lateral_mm
            and abs(paralelo) <= self.tolerancia_paralelo_deg
        )

    def _vaiven_adelante(self, paredes: MapaParedes, ahora: float) -> Consigna:
        """Tramo corto hacia adelante girando para ganar angulo.

        Es la mitad del vaiven que hace que la maniobra no dependa del radio.
        Cada pareja adelante/atras gana unos milimetros de penetracion; se
        repite hasta entrar o hasta agotar ``max_shuffles``.

        EL TRAMO SE CORTA POR RUMBO, NO POR RELOJ.  Sin encoders no hay forma
        de medir cuanto se ha avanzado, pero si cuanto se ha GIRADO, y en un
        vaiven a tope de volante las dos cosas son la misma: 12 grados son
        unos 0,8 s a los 15,7 deg/s medidos el 06-09.  La diferencia es que un
        tramo cortado por rumbo se adapta solo a la bateria, al restregado y
        al lado, y el cortado por reloj hay que reajustarlo cada sesion.
        """

        if self._vaivenes >= self.max_vaivenes:
            self._entrar(VERIFICAR, ahora)
            return self._resultado(0, 0.0, "vaivenes agotados")
        frontal = paredes.frontal_min_mm
        if math.isfinite(frontal) and frontal < self.holgura_frontal_min_mm:
            self._vaivenes += 1
            self._entrar(VAIVEN_ATRAS, ahora)
            return self._resultado(0, 0.0, "morro cerca")
        if self._girado_deg() >= self.rumbo_vaiven_deg:
            self._vaivenes += 1
            self._entrar(VAIVEN_ATRAS, ahora)
            return self._resultado(0, 0.0, "tramo adelante hecho")
        if self._tiempo(ahora) > self.timeout_tramo_s * 0.5:
            return self._fallar("vaiven adelante sin corte por medida", ahora)
        return self._resultado(
            self.velocidad,
            self._mando(hacia_bahia=False, reversa=False),
            f"vaiven adelante {self._vaivenes + 1}",
        )

    def _vaiven_atras(
        self,
        paralelo: Optional[float],
        lateral: Optional[float],
        trasera: Optional[float],
        ahora: float,
    ) -> Consigna:
        if self._dentro(lateral, paralelo):
            self._entrar(CENTRAR, ahora)
            return self._resultado(0, 0.0, "dentro tras el vaiven")
        if self._sin_evidencia_atras(trasera):
            return self._fallar("sin evidencia trasera en el vaiven", ahora)
        if trasera is not None and trasera < self.holgura_trasera_min_mm:
            self._entrar(VAIVEN_ADELANTE, ahora)
            return self._resultado(0, 0.0, "culata cerca")
        if self._girado_deg() >= self.rumbo_vaiven_deg:
            self._entrar(VAIVEN_ADELANTE, ahora)
            return self._resultado(0, 0.0, "tramo atras hecho")
        if self._tiempo(ahora) > self.timeout_tramo_s * 0.5:
            return self._fallar("vaiven atras sin corte por medida", ahora)
        return self._resultado(
            self.velocidad_reversa,
            self._mando(hacia_bahia=True, reversa=True),
            f"vaiven atras {self._vaivenes + 1}",
        )

    def _centrar(self, trasera: Optional[float], ahora: float) -> Consigna:
        """Repartir el hueco sobrante entre morro y culata.

        La bahia mide ``bay_length_mm`` y el robot ``length_mm``: lo que sobra
        se parte por la mitad.  Con los 330 y 210 medidos el 06-09 quedan 60
        mm por lado, no los 84 que salian con el 390 viejo.

        El corte por tiempo de aqui SI puede avanzar, y es la unica excepcion
        del modulo: lleva a VERIFICAR, que deja el robot QUIETO.  Rendirse
        centrando no mueve nada a ciegas, solo deja de empujar y pasa a mirar.
        """

        sobra = max(0.0, self.largo_bahia_mm - self.largo_robot_mm)
        objetivo = sobra / 2.0
        if trasera is None:
            self._entrar(VERIFICAR, ahora)
            return self._resultado(0, 0.0, "sin medida trasera")
        error = trasera - objetivo
        if abs(error) <= 25.0 or self._tiempo(ahora) > self.timeout_tramo_s * 0.5:
            self._entrar(VERIFICAR, ahora)
            return self._resultado(0, 0.0, "centrado")
        velocidad = self.velocidad_reversa if error > 0 else self.velocidad
        return self._resultado(velocidad, 0.0, f"centrando {error:+.0f}mm")

    def _verificar(
        self,
        paredes: MapaParedes,
        lateral: Optional[float],
        paralelo: Optional[float],
        trasera: Optional[float],
        ahora: float,
    ) -> Consigna:
        """Confirmar el resultado con el robot QUIETO, varios barridos.

        Terminar sin verificar es lo mismo que no saber si se aparco.  Se
        exige que las tres medidas se repitan, no que coincidan una vez.

        ESTE ES EL ESTADO QUE NO PODIA CERRAR NUNCA.  El 06-09, con el robot
        colocado a mano en la pose buena, ``_lateral_mm`` y ``_paralelo``
        salian None en los doce barridos porque el ajuste de pared exigia 220
        mm de segmento y ahi solo hay entre 60 y 80: la maniobra podia salir
        perfecta y la ronda terminaba igual en FALLO al agotar el timeout.
        Con el minimo de segmento relajado para la pared del lado de la bahia
        (``percepcion_lidar._largo_minimo_mm``) ese mismo barrido da 77,9 mm y
        3,4 grados de paralelo, o sea LISTO.  El paralelismo se acepta ademas
        de la IMU: quedarse sin verificar por no ver la pared con el robot ya
        dentro seria repetir el mismo error por otra puerta.
        """

        frontal = paredes.frontal_min_mm
        bien = (
            lateral is not None
            and paralelo is not None
            and lateral <= self.lateral_dentro_mm + self.tolerancia_lateral_mm
            and abs(paralelo) <= self.tolerancia_paralelo_deg
            and (trasera is None or trasera >= self.holgura_trasera_min_mm * 0.5)
            and (not math.isfinite(frontal) or frontal >= self.holgura_frontal_min_mm * 0.5)
        )
        self._confirmaciones = self._confirmaciones + 1 if bien else 0
        if self._confirmaciones >= self.barridos_verificacion:
            self._entrar(LISTO, ahora)
            self._razon = "aparcado y verificado"
            return Consigna(0, 0.0, LISTO, self._razon, True, True)
        if self._tiempo(ahora) > self.timeout_verificacion_s:
            # Dentro y paralelo pero sin cerrar solo puede ser una cosa: el
            # robot esta bien puesto y corrido hacia un extremo.  Eso se
            # arregla centrando, no meciendose.  Es exactamente la pose que se
            # midio el 06-09: lateral 77,9 mm y 3,4 grados de paralelo -- las
            # dos buenas -- con la culata a 10 mm del delimitador.  Un solo
            # reintento: si centrar no lo arregla, repetirlo tampoco.
            if self._dentro(lateral, paralelo) and self._recentrados == 0:
                self._recentrados += 1
                self._entrar(CENTRAR, ahora)
                return self._resultado(0, 0.0, "dentro pero corrido, a centrar")
            if self._vaivenes < self.max_vaivenes:
                self._entrar(VAIVEN_ADELANTE, ahora)
                return self._resultado(0, 0.0, "verificacion fallida, otro vaiven")
            return self._fallar("no se pudo verificar el parqueo", ahora)
        return self._resultado(0, 0.0, f"verificando {self._confirmaciones}")

    # ------------------------------------------------------------ telemetria

    def instantanea(self) -> Dict[str, Any]:
        return {
            "parqueo_estado": self.estado,
            "parqueo_razon": self._razon,
            "parqueo_vaivenes": self._vaivenes,
            "parqueo_paralelo_fuente": self._fuente_paralelo,
            "parqueo_ciegos_atras": self._ciegos_atras,
            "hueco_confianza": round(self.hueco.confianza, 3) if self.hueco else "",
            "hueco_separacion": round(self.hueco.separacion_mm, 1) if self.hueco else "",
            "hueco_lateral": round(self.hueco.distancia_lateral_mm, 1) if self.hueco else "",
        }
