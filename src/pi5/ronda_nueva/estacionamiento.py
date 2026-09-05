"""Estacionamiento en la bahia magenta, en varios tiempos y por sensores.

QUE SE SABE YA DE ESTE PROBLEMA, MEDIDO EN PISTA
* El detector del hueco FUNCIONA desde el 03-09: partir los clusters en rectas
  antes del PCA subio de 0 emparejamientos en 22 barridos a 9 de 10, con la
  separacion estable en 389-391 mm.  Ese detector se conserva tal cual en
  ``percepcion_lidar.buscar_hueco``.
* La maniobra de DOS ARCOS no cabe.  La simulacion con los radios reales
  (228 mm a la izquierda, 260 a la derecha) se pasa 9 mm sobre un delimitador,
  y da igual por que lado se entre: lo que ata la maniobra no es el corrimiento
  lateral sino que la culata barre contra el delimitador durante el arco.  El
  techo geometrico de dos arcos es r ~ 180 mm, y ningun Ackermann con batalla
  de 136 mm llega ahi.

LA CONSECUENCIA DE DISEÑO
No se intenta clavar la maniobra de una vez.  Se entra en VARIOS TIEMPOS, como
un coche aparca en linea de verdad: arco de entrada, arco de enderezado, y si
falta sitio, tantos vaivenes cortos como haga falta hasta quedar dentro y
paralelo.  Cada tramo se corta por MEDIDA -- rumbo alcanzado o holgura minima
--, nunca por tiempo, asi que la maniobra no depende de acertar el radio: se
adapta al que el chasis tenga ese dia.

Y una consecuencia practica: los 9 mm dejan de importar.  Un vaiven que se
queda corto se corrige en el siguiente en vez de terminar la ronda contra el
delimitador.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

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

        self.largo_bahia_mm = float(parqueo.get("bay_length_mm", 390.0))
        self.largo_robot_mm = float(chasis.get("length_mm", 222.0))
        self.ancho_robot_mm = float(chasis.get("width_mm", 125.0))
        self.voladizo_mm = float(chasis.get("rear_overhang_mm", 60.0))
        self.lidar_a_derecha_mm = float(chasis.get("lidar_to_right_edge_mm", 45.0))
        self.lidar_a_izquierda_mm = float(chasis.get("lidar_to_left_edge_mm", 61.0))

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
        self.max_vaivenes = int(parqueo.get("max_shuffles", 6))
        self.barridos_verificacion = int(parqueo.get("verify_scans", 3))

        self.timeout_busqueda_s = float(parqueo.get("search_timeout_s", 14.0))
        self.timeout_tramo_s = float(parqueo.get("leg_timeout_s", 3.5))
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
        self._rumbo_ref = 0.0
        self._vaivenes = 0
        self._confirmaciones = 0
        self._razon = ""
        self._lado = -1

    def _entrar(self, estado: str, ahora: float, rumbo: float = 0.0) -> None:
        if estado != self.estado:
            self.estado = estado
            self._t_estado = ahora
            self._rumbo_ref = rumbo

    def _tiempo(self, ahora: float) -> float:
        return max(0.0, ahora - self._t_estado)

    # ------------------------------------------------------------ geometria

    def _lateral_mm(self, paredes: MapaParedes) -> Optional[float]:
        """Distancia al muro exterior, que es contra el que esta la bahia."""

        pared = paredes.izquierda if self._lado < 0 else paredes.derecha
        if pared is None:
            return None
        return pared.distancia_mm

    def _trasera_mm(self, paredes: MapaParedes, ultrasonido_mm: Optional[float]) -> Optional[float]:
        """Holgura por detras, fusionando LiDAR y ultrasonido.

        Se usa la MENOR de las dos: el LiDAR tiene el mastil tapandole un
        sector por detras y el ultrasonido tiene un cono ancho que puede ver el
        delimitador de al lado.  Quedarse con la menor hace que un fallo de
        cualquiera de los dos frene la maniobra, que es el sentido seguro.
        """

        candidatos = []
        if paredes.trasera is not None:
            candidatos.append(paredes.trasera.distancia_mm - self.voladizo_mm)
        if math.isfinite(paredes.trasera_min_mm):
            candidatos.append(paredes.trasera_min_mm - self.voladizo_mm)
        if (
            self.usar_ultrasonido
            and ultrasonido_mm is not None
            and self.ultrasonido_min_mm <= float(ultrasonido_mm) <= self.ultrasonido_max_mm
        ):
            candidatos.append(float(ultrasonido_mm))
        if not candidatos:
            return None
        return min(candidatos)

    def _paralelo(self, paredes: MapaParedes) -> Optional[float]:
        """Error de paralelismo contra el muro exterior, en grados."""

        pared = paredes.izquierda if self._lado < 0 else paredes.derecha
        if pared is None:
            return None
        # La normal de un muro lateral perfecto vale -90 o +90 grados.
        referencia = -90.0 if self._lado < 0 else 90.0
        return pared.angulo_deg - referencia

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
        self._entrar(FALLO, ahora)
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
        if self._t_inicio is None:
            self._t_inicio = ahora
            self._t_estado = ahora
        self._lado = int(lado) or -1

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
        paralelo = self._paralelo(paredes)

        if self.estado == BUSCAR:
            return self._buscar(ahora)
        if self.estado == ALINEAR:
            return self._alinear(lateral, ahora)
        if self.estado == ARCO_ENTRADA:
            return self._arco_entrada(paralelo, trasera, ahora)
        if self.estado == ARCO_ENDEREZA:
            return self._arco_endereza(paralelo, lateral, trasera, ahora)
        if self.estado == VAIVEN_ADELANTE:
            return self._vaiven_adelante(paredes, paralelo, lateral, ahora)
        if self.estado == VAIVEN_ATRAS:
            return self._vaiven_atras(paralelo, lateral, trasera, ahora)
        if self.estado == CENTRAR:
            return self._centrar(paredes, trasera, ahora)
        if self.estado == VERIFICAR:
            return self._verificar(paredes, lateral, paralelo, trasera, ahora)
        return self._fallar(f"estado desconocido {self.estado}", ahora)

    # --------------------------------------------------------------- tramos

    def _buscar(self, ahora: float) -> Consigna:
        if self.hueco is not None:
            self._entrar(ALINEAR, ahora)
            return self._resultado(self.velocidad, 0.0, "hueco confirmado")
        if self._tiempo(ahora) > self.timeout_busqueda_s:
            return self._fallar("timeout buscando hueco", ahora)
        return self._resultado(self.velocidad, 0.0, "buscando hueco")

    def _alinear(self, lateral: Optional[float], ahora: float) -> Consigna:
        """Avanzar hasta que el eje trasero quede a la altura de la bahia.

        La referencia es ``borde_delantero_y_mm`` del hueco, medido por el
        LiDAR en el marco del robot: cuando ese borde queda ligeramente por
        delante del LiDAR, el eje trasero esta en el sitio desde el que el
        arco entra.  No hay ningun tiempo ni ninguna cuenta de encoder de por
        medio, que es lo que hacia la maniobra irrepetible.
        """

        if self.hueco is None:
            self._entrar(BUSCAR, ahora)
            return self._resultado(self.velocidad, 0.0, "hueco perdido")

        objetivo = self.alineacion_objetivo_mm
        error = self.hueco.borde_delantero_y_mm - objetivo

        correccion = 0.0
        if lateral is not None:
            # Mantenerse a distancia constante del muro mientras se alinea:
            # entrar torcido cuesta un vaiven de mas.
            correccion = _limitar(
                (self.lateral_objetivo_mm - lateral) * 0.06 * -self._lado, -8.0, 8.0
            )

        if abs(error) <= 35.0:
            self._entrar(ARCO_ENTRADA, ahora)
            return self._resultado(0, 0.0, "alineado con la bahia")
        if self._tiempo(ahora) > self.timeout_tramo_s * 2.0:
            self._entrar(ARCO_ENTRADA, ahora)
            return self._resultado(0, 0.0, "alineacion por tiempo")
        velocidad = self.velocidad if error > 0 else self.velocidad_reversa
        return self._resultado(velocidad, correccion, f"alineando {error:+.0f}mm")

    def _arco_entrada(
        self, paralelo: Optional[float], trasera: Optional[float], ahora: float
    ) -> Consigna:
        """Primer arco en reversa, metiendo la culata en la bahia."""

        if trasera is not None and trasera < self.holgura_trasera_min_mm:
            self._entrar(VAIVEN_ADELANTE, ahora)
            return self._resultado(0, 0.0, "culata cerca, vaiven")
        if paralelo is not None and abs(paralelo) >= self.rumbo_entrada_deg:
            self._entrar(ARCO_ENDEREZA, ahora)
            return self._resultado(0, 0.0, "angulo de entrada alcanzado")
        if self._tiempo(ahora) > self.timeout_tramo_s:
            self._entrar(ARCO_ENDEREZA, ahora)
            return self._resultado(0, 0.0, "arco de entrada por tiempo")
        return self._resultado(
            self.velocidad_reversa,
            self._mando(hacia_bahia=True, reversa=True),
            "arco de entrada",
        )

    def _arco_endereza(
        self,
        paralelo: Optional[float],
        lateral: Optional[float],
        trasera: Optional[float],
        ahora: float,
    ) -> Consigna:
        """Segundo arco en reversa, enderezando dentro de la bahia."""

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
            self._entrar(VAIVEN_ADELANTE, ahora)
            return self._resultado(0, 0.0, "enderezado por tiempo")
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

    def _vaiven_adelante(
        self,
        paredes: MapaParedes,
        paralelo: Optional[float],
        lateral: Optional[float],
        ahora: float,
    ) -> Consigna:
        """Tramo corto hacia adelante girando para ganar angulo.

        Es la mitad del vaiven que hace que la maniobra no dependa del radio.
        Cada pareja adelante/atras gana unos milimetros de penetracion; se
        repite hasta entrar o hasta agotar ``max_shuffles``.
        """

        if self._vaivenes >= self.max_vaivenes:
            self._entrar(VERIFICAR, ahora)
            return self._resultado(0, 0.0, "vaivenes agotados")
        frontal = paredes.frontal_min_mm
        if math.isfinite(frontal) and frontal < self.holgura_frontal_min_mm:
            self._vaivenes += 1
            self._entrar(VAIVEN_ATRAS, ahora)
            return self._resultado(0, 0.0, "morro cerca")
        if self._tiempo(ahora) > self.timeout_tramo_s * 0.45:
            self._vaivenes += 1
            self._entrar(VAIVEN_ATRAS, ahora)
            return self._resultado(0, 0.0, "tramo adelante hecho")
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
        if trasera is not None and trasera < self.holgura_trasera_min_mm:
            self._entrar(VAIVEN_ADELANTE, ahora)
            return self._resultado(0, 0.0, "culata cerca")
        if self._tiempo(ahora) > self.timeout_tramo_s * 0.6:
            self._entrar(VAIVEN_ADELANTE, ahora)
            return self._resultado(0, 0.0, "tramo atras hecho")
        return self._resultado(
            self.velocidad_reversa,
            self._mando(hacia_bahia=True, reversa=True),
            f"vaiven atras {self._vaivenes + 1}",
        )

    def _centrar(
        self, paredes: MapaParedes, trasera: Optional[float], ahora: float
    ) -> Consigna:
        """Repartir el hueco sobrante entre morro y culata.

        La bahia mide ``bay_length_mm`` y el robot ``length_mm``: lo que sobra
        se parte por la mitad.  Con 390 y 222 quedan 84 mm por lado, que es
        poco pero medible.
        """

        sobra = max(0.0, self.largo_bahia_mm - self.largo_robot_mm)
        objetivo = sobra / 2.0
        if trasera is None:
            self._entrar(VERIFICAR, ahora)
            return self._resultado(0, 0.0, "sin medida trasera")
        error = trasera - objetivo
        if abs(error) <= 25.0 or self._tiempo(ahora) > self.timeout_tramo_s:
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
            return Consigna(0, 0.0, LISTO, "aparcado y verificado", True, True)
        if self._tiempo(ahora) > self.timeout_tramo_s * 2.0:
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
            "hueco_confianza": round(self.hueco.confianza, 3) if self.hueco else "",
            "hueco_separacion": round(self.hueco.separacion_mm, 1) if self.hueco else "",
            "hueco_lateral": round(self.hueco.distancia_lateral_mm, 1) if self.hueco else "",
        }
