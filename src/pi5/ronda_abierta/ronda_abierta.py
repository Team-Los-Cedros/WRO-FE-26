"""Ronda Abierta (Open Challenge) para la Pi 5.

Tres vueltas siguiendo las paredes y parada dentro de la seccion de salida.
Sin pilares, sin camara y sin mapa: la Ronda Abierta no los necesita, y
mantenerlo en un solo archivo auditable es una virtud, no un descuido.

PORTADO DESDE `src/pi3B/ronda_abierta/ronda_abierta.py`. Tres cosas cambian, y
ninguna es opcional:

1. **El conteo de vueltas ya no puede salir del sensor de color de piso.** En
   este robot la Pico responde ``COLOR:SIN_SENSOR``: el TCS3472 no esta
   montado. Ese sensor era quien contaba las doce lineas naranjas, o sea todo
   el criterio de "ya son tres vueltas". Aqui las esquinas se cuentan por el
   rumbo acumulado de la IMU, que la Pico ya publica y que en esta placa
   deriva -0,00 deg/s medidos. Si algun dia vuelve el sensor, el contador por
   color sigue implementado y se activa solo.

2. **El LiDAR se lee por bloques**, reutilizando ``comun.lidar_driver``. El
   original hacia read(1)+read(4) por muestra; medido en la Pi con carga, esa
   lectura se queda atras y entrega barridos viejos.

3. **Se enmascara la estructura del propio robot.** El mastil y su soporte
   devuelven eco permanente en 28..54 y 141..212 grados, y el primero cae
   DENTRO del sector derecho (30-90): sin enmascararlo, la distancia derecha
   se queda clavada en ~142 mm y el centrado entre paredes deja de funcionar.

Uso tipico::

    python3 -m ronda_abierta.ronda_abierta --arranque-inmediato --velocidad 45
"""

import argparse
import math
import signal
import sys
import threading
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


# --- Puertos y enlace ----------------------------------------------------
PUERTO_LIDAR = "/dev/ttyUSB0"
PUERTO_PICO = "/dev/ttyACM0"
BAUDRATE_LIDAR = 460800
BAUDRATE_PICO = 115200
PIN_BOTON = 21

# --- Navegacion ----------------------------------------------------------
KP_LATERAL = 0.14
VELOCIDAD_CRUCERO = 90          # valor heredado de la 3B; ver --velocidad
VELOCIDAD_META = 55
ANGULO_MAX_DEG = 25.0           # el firmware acota igual, pero no se le pide de mas

# --- Fin de ronda --------------------------------------------------------
VUELTAS_OBJETIVO = 3
ESQUINAS_POR_VUELTA = 4
GRADOS_POR_ESQUINA = 90.0
# Margen en grados que se le regala al contador. Va a CERO a proposito:
#
# - Exigir de mas (n*90 + margen) es un fallo grave, no una precaucion: tras
#   la ultima esquina el robot enfila la recta final y deja de girar, asi que
#   esos grados de mas no llegan nunca y la ronda no terminaria.
# - Exigir de menos tampoco hace falta. El contador es monotono y el bamboleo
#   del centrado oscila alrededor de un valor fijo en vez de acumular (la
#   Pico integra el giroscopio con zona muerta), asi que no puede inventar
#   una esquina: como mucho la cuenta un par de grados antes de rematarla.
#
# Queda como parametro por si la pista dice otra cosa, no porque hoy sirva.
MARGEN_ESQUINA_DEG = 0.0
TIEMPO_AVANCE_META_S = 1.8      # avance final dentro de la seccion de salida
TOLERANCIA_FIRMA_MM = 80.0      # parecido a la firma geometrica de la salida

# --- Sectores del LiDAR --------------------------------------------------
SECTOR_DERECHA = (30.0, 90.0)
SECTOR_IZQUIERDA = (270.0, 330.0)

# Arcos con eco permanente de la propia estructura, MEDIDOS con
# `herramientas/diag_mastil.py` el 2026-09-05: 28..54 grados a 142-172 mm
# (soporte) y 141..212 a 35-68 mm (mastil). Se remide cada vez que se toca el
# montaje de la camara: la mascara es del MONTAJE, no del robot.
SECTORES_CIEGOS = ((27.0, 55.0), (140.0, 213.0))

DISTANCIA_MAX_MM = 6000.0
SIN_DATO_MM = 8000.0
# Un sector sin ningun eco no es "camino libre". Se sustituye por este valor
# solo para poder seguir centrando; nunca autoriza nada.
FALLBACK_LATERAL_MM = 2000.0


# =========================================================================
# Logica pura (probada sin hardware en tests/)
# =========================================================================

def en_sector(angulo: float, sector: Tuple[float, float]) -> bool:
    """Pertenencia a un sector angular, soportando el cruce por 0 grados."""

    inicio, fin = sector
    angulo %= 360.0
    inicio %= 360.0
    fin %= 360.0
    if inicio <= fin:
        return inicio <= angulo <= fin
    return angulo >= inicio or angulo <= fin


def esta_ciego(angulo: float, ciegos: Iterable[Tuple[float, float]]) -> bool:
    return any(en_sector(angulo, sector) for sector in ciegos)


def minimos_laterales(
    barrido: Sequence[Tuple[float, float]],
    ciegos: Iterable[Tuple[float, float]] = SECTORES_CIEGOS,
    sector_derecha: Tuple[float, float] = SECTOR_DERECHA,
    sector_izquierda: Tuple[float, float] = SECTOR_IZQUIERDA,
) -> Tuple[float, float]:
    """Distancia minima a cada lado, ignorando la estructura del robot.

    Devuelve ``(izquierda_mm, derecha_mm)``. Un lado sin ecos utiles devuelve
    ``inf``; quien llama decide que hacer con eso.
    """

    izquierda = math.inf
    derecha = math.inf
    for angulo, distancia in barrido:
        if not (0.0 < distancia < DISTANCIA_MAX_MM):
            continue
        if esta_ciego(angulo, ciegos):
            continue
        if en_sector(angulo, sector_derecha):
            if distancia < derecha:
                derecha = distancia
        elif en_sector(angulo, sector_izquierda):
            if distancia < izquierda:
                izquierda = distancia
    return izquierda, derecha


class ContadorEsquinas:
    """Cuenta esquinas por rumbo acumulado de la IMU.

    Sustituye al conteo de lineas naranjas, que en esta placa no es posible:
    la Pico responde ``COLOR:SIN_SENSOR``. Una vuelta son cuatro esquinas de
    90 grados, asi que tres vueltas son 1080 grados de rumbo acumulado.

    El contador es MONOTONO a proposito: una maniobra de rescate que deshaga
    parte del giro no puede descontar una esquina ya recorrida.
    """

    def __init__(
        self,
        grados_por_esquina: float = GRADOS_POR_ESQUINA,
        margen_deg: float = MARGEN_ESQUINA_DEG,
    ):
        self._grados = float(grados_por_esquina)
        self._margen = float(margen_deg)
        self._referencia: Optional[float] = None
        self.esquinas = 0
        self.giro_total_deg = 0.0

    def fijar_referencia(self, rumbo_deg: float) -> None:
        self._referencia = float(rumbo_deg)
        self.esquinas = 0
        self.giro_total_deg = 0.0

    @property
    def sentido(self) -> int:
        """+1 antihorario, -1 horario, 0 mientras no se sepa."""

        if abs(self.giro_total_deg) < 30.0:
            return 0
        return 1 if self.giro_total_deg > 0 else -1

    def actualizar(self, rumbo_deg: float) -> int:
        if self._referencia is None:
            self.fijar_referencia(rumbo_deg)
            return 0
        self.giro_total_deg = float(rumbo_deg) - self._referencia
        recorrido = abs(self.giro_total_deg)
        candidatas = int((recorrido + self._margen) // self._grados)
        if candidatas > self.esquinas:
            self.esquinas = candidatas
        return self.esquinas


class ContadorLineas:
    """Conteo historico por lineas naranjas del sensor de piso.

    Se conserva para cuando el TCS3472 vuelva a estar montado. Mientras la
    Pico responda ``SIN_SENSOR`` nunca se arma, y manda ContadorEsquinas.
    """

    def __init__(self, refractario_s: float = 1.2, salida_s: float = 0.3):
        self._refractario = float(refractario_s)
        self._salida = float(salida_s)
        self.lineas = 0
        self._sobre_linea = False
        self._t_ultima = 0.0
        self._t_fuera = 0.0

    def actualizar(self, color: str, ahora: float) -> int:
        if color == "NARANJA":
            if not self._sobre_linea and (ahora - self._t_ultima) > self._refractario:
                self._sobre_linea = True
                self.lineas += 1
                self._t_ultima = ahora
                self._t_fuera = 0.0
        elif color == "PISTA":
            if self._sobre_linea:
                if self._t_fuera == 0.0:
                    self._t_fuera = ahora
                elif (ahora - self._t_fuera) > self._salida:
                    self._sobre_linea = False
                    self._t_fuera = 0.0
        else:
            self._t_fuera = 0.0
        return self.lineas


def angulo_centrado(izquierda_mm: float, derecha_mm: float, kp: float = KP_LATERAL) -> float:
    """Mando de direccion para quedarse a medio camino entre las dos paredes.

    Positivo es hacia la izquierda, igual que en el resto del robot.
    """

    error = float(izquierda_mm) - float(derecha_mm)
    return max(-ANGULO_MAX_DEG, min(ANGULO_MAX_DEG, error * kp))


def coincide_firma(
    izquierda_mm: float,
    derecha_mm: float,
    firma_izquierda_mm: float,
    firma_derecha_mm: float,
    tolerancia_mm: float = TOLERANCIA_FIRMA_MM,
) -> bool:
    """Si el corredor se parece al de la salida, el robot volvio a su seccion."""

    return (
        abs(izquierda_mm - firma_izquierda_mm) < tolerancia_mm
        and abs(derecha_mm - firma_derecha_mm) < tolerancia_mm
    )


# =========================================================================
# Enlace con la Pico
# =========================================================================

class EnlacePico:
    """Lee la telemetria en un hilo y envia consignas ``velocidad,angulo``."""

    def __init__(self, puerto: str = PUERTO_PICO, baudrate: int = BAUDRATE_PICO):
        import serial

        self._serial = serial
        self._ser = serial.Serial(puerto, baudrate=baudrate, timeout=0.05)
        self._lock = threading.Lock()
        self._rumbo = 0.0
        self._color = "DESCONOCIDO"
        self._ultrasonido: Optional[float] = None
        self._t_telemetria = 0.0
        self._corriendo = True
        self._hilo = threading.Thread(target=self._leer, name="pico", daemon=True)
        self._hilo.start()

    @staticmethod
    def parsear(linea: str) -> Optional[Dict[str, object]]:
        """``IMU:-12.5,COLOR:PISTA,US:185,WD:OK`` -> dict, o None si no es trama.

        Se recorre por NOMBRE de campo, no por posicion: el firmware fue
        creciendo (COLOR, luego US, luego WD) y el orden no es contrato.
        """

        if not linea.startswith("IMU:"):
            return None
        datos: Dict[str, object] = {"color": None, "ultrasonido": None, "watchdog": None}
        rumbo = None
        for campo in linea.split(","):
            if ":" not in campo:
                continue
            clave, _, valor = campo.partition(":")
            clave = clave.strip().upper()
            valor = valor.strip()
            if clave == "IMU":
                try:
                    rumbo = float(valor)
                except ValueError:
                    return None
            elif clave == "COLOR":
                datos["color"] = valor.upper()
            elif clave == "US":
                try:
                    medida = float(valor)
                except ValueError:
                    medida = -1.0
                datos["ultrasonido"] = medida if medida > 0 else None
            elif clave == "WD":
                datos["watchdog"] = valor.upper()
        if rumbo is None:
            return None
        datos["rumbo"] = rumbo
        return datos

    def _leer(self) -> None:
        while self._corriendo:
            try:
                cruda = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if not cruda:
                    continue
                trama = self.parsear(cruda)
                if trama is None:
                    continue
                with self._lock:
                    self._rumbo = float(trama["rumbo"])
                    if trama["color"]:
                        self._color = str(trama["color"])
                    self._ultrasonido = trama["ultrasonido"]
                    self._t_telemetria = time.monotonic()
            except (ValueError, IndexError):
                continue
            except self._serial.SerialException:
                time.sleep(0.05)

    def rumbo(self) -> float:
        with self._lock:
            return self._rumbo

    def color(self) -> str:
        with self._lock:
            return self._color

    def ultrasonido_mm(self) -> Optional[float]:
        with self._lock:
            return self._ultrasonido

    def telemetria_valida(self, timeout_s: float = 0.5) -> bool:
        with self._lock:
            return (
                self._t_telemetria > 0.0
                and (time.monotonic() - self._t_telemetria) <= timeout_s
            )

    def enviar(self, velocidad: int, angulo: float) -> bool:
        try:
            self._ser.write(
                "{:d},{:.2f}\n".format(int(velocidad), float(angulo)).encode("ascii")
            )
            return True
        except self._serial.SerialException:
            return False

    def detener(self) -> None:
        for _ in range(8):
            self.enviar(0, 0.0)
            time.sleep(0.01)

    def cerrar(self) -> None:
        self._corriendo = False
        try:
            self.detener()
            self._ser.close()
        except Exception:
            pass


# =========================================================================
# Orquestacion
# =========================================================================

class RondaAbierta:
    """Fases: ESPERA -> FIRMA -> CARRERA -> META -> FIN."""

    def __init__(self, args):
        self.args = args
        self.fase = "ESPERA"
        self.enlace: Optional[EnlacePico] = None
        self.lidar = None
        self._seguir = threading.Event()
        self._seguir.set()
        self._gpio = None
        self._hilo_lidar: Optional[threading.Thread] = None

        self.contador = ContadorEsquinas()
        self.lineas = ContadorLineas()
        self.esquinas_objetivo = args.vueltas * ESQUINAS_POR_VUELTA

        self.firma_izquierda = 0.0
        self.firma_derecha = 0.0
        self._t_meta = 0.0
        self._motivo = ""
        self._barridos = 0
        self._ultimo_barrido = 0.0

    @property
    def corriendo(self) -> bool:
        return self._seguir.is_set()

    def _terminar(self, motivo: str) -> None:
        if not self._motivo:
            self._motivo = motivo
            print("[FIN] " + motivo)
        if self.enlace is not None:
            self.enlace.detener()
        self._seguir.clear()

    def _al_barrido(self, barrido, timestamp: float) -> None:
        """Un ciclo de decision por barrido. Corre en el hilo del LiDAR."""

        if not self.corriendo or self.enlace is None:
            return
        self._barridos += 1
        self._ultimo_barrido = timestamp

        izquierda, derecha = minimos_laterales(barrido, self.args.sectores_ciegos)
        # Un lado sin eco no es camino libre, pero tampoco puede paralizar el
        # centrado: se sustituye por un valor neutro y se sigue.
        if not math.isfinite(izquierda):
            izquierda = FALLBACK_LATERAL_MM
        if not math.isfinite(derecha):
            derecha = FALLBACK_LATERAL_MM

        if self.fase == "FIRMA":
            self.firma_izquierda = izquierda
            self.firma_derecha = derecha
            self.contador.fijar_referencia(self.enlace.rumbo())
            self.fase = "CARRERA"
            print(
                "[i] Firma de la salida -> izq {:.0f} mm | der {:.0f} mm".format(
                    izquierda, derecha
                )
            )
            print("[+] Rodando. Ctrl-C para parar.")
            return

        if not self.enlace.telemetria_valida():
            self._terminar("watchdog de la Pico vencido")
            return

        angulo = angulo_centrado(izquierda, derecha, self.args.kp)

        if self.fase == "CARRERA":
            esquinas = self.contador.actualizar(self.enlace.rumbo())
            # Si el sensor de piso volviera a existir, su cuenta manda.
            if self.enlace.color() not in ("SIN_SENSOR", "DESCONOCIDO"):
                self.lineas.actualizar(self.enlace.color(), time.monotonic())
            if esquinas >= self.esquinas_objetivo:
                self.fase = "META"
                self._t_meta = time.monotonic()
                print(
                    "[i] {} esquinas ({:.0f} deg). Buscando la seccion de salida.".format(
                        esquinas, self.contador.giro_total_deg
                    )
                )
                self.enlace.enviar(self.args.velocidad_meta, angulo)
                return
            self.enlace.enviar(self.args.velocidad, angulo)
            return

        if self.fase == "META":
            self.enlace.enviar(self.args.velocidad_meta, angulo)
            transcurrido = time.monotonic() - self._t_meta
            en_la_salida = coincide_firma(
                izquierda, derecha, self.firma_izquierda, self.firma_derecha
            )
            if transcurrido >= self.args.avance_meta or en_la_salida:
                causa = "firma geometrica" if en_la_salida else "tiempo de avance"
                self.enlace.detener()
                self.fase = "FIN"
                self._terminar(
                    "ronda completa: {} vueltas, parada por {} ({:.2f} s)".format(
                        self.args.vueltas, causa, transcurrido
                    )
                )

    def _esperar_boton(self) -> bool:
        assert self.enlace is not None
        if self.args.arranque_inmediato:
            print("[LISTO] Arranque inmediato; no se espera GP{}.".format(PIN_BOTON))
            limite = time.monotonic() + 1.0
            while self.corriendo and time.monotonic() < limite:
                self.enlace.enviar(0, 0.0)
                time.sleep(0.05)
            return self.corriendo
        print("[LISTO] Coloca el robot en la salida y pulsa GP{}.".format(PIN_BOTON))
        while self.corriendo and self._gpio.input(PIN_BOTON) == self._gpio.HIGH:
            self.enlace.enviar(0, 0.0)
            time.sleep(0.05)
        return self.corriendo

    def ejecutar(self) -> int:
        try:
            from ..comun.lidar_driver import LidarDriver
        except (ImportError, ValueError):
            from comun.lidar_driver import LidarDriver
        import RPi.GPIO as GPIO

        self._gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.enlace = EnlacePico(self.args.puerto_pico)
        self.enlace.detener()

        limite = time.monotonic() + 3.0
        while self.corriendo and not self.enlace.telemetria_valida():
            if time.monotonic() > limite:
                self._terminar("no llega telemetria de la Pico")
                return 2
            time.sleep(0.05)

        if not self._esperar_boton():
            return 1

        self.fase = "FIRMA"
        self.lidar = LidarDriver(self.args.puerto_lidar, self.args.baudrate_lidar)
        self._hilo_lidar = threading.Thread(
            target=self.lidar.hilo_lectura,
            args=(lambda: self.corriendo, self._al_barrido),
            name="lidar",
            daemon=True,
        )
        self._hilo_lidar.start()

        limite = time.monotonic() + self.args.timeout_total
        while self.corriendo:
            ahora = time.monotonic()
            if ahora > limite:
                self._terminar("timeout total de la ronda")
                break
            if self._barridos and (ahora - self._ultimo_barrido) > 1.0:
                self._terminar("el LiDAR dejo de entregar barridos")
                break
            time.sleep(0.05)

        return 0 if self.fase == "FIN" else 1

    def cerrar(self) -> None:
        self._seguir.clear()
        if self.enlace is not None:
            self.enlace.detener()
        if self.lidar is not None:
            self.lidar.cerrar()
        if self._hilo_lidar is not None and self._hilo_lidar.is_alive():
            self._hilo_lidar.join(timeout=2.0)
        if self.enlace is not None:
            self.enlace.cerrar()
        if self._gpio is not None:
            try:
                self._gpio.cleanup()
            except Exception:
                pass


def _sectores(texto: str) -> Tuple[Tuple[float, float], ...]:
    """``27-55,140-213`` -> ((27.0, 55.0), (140.0, 213.0))."""

    if not texto.strip():
        return ()
    sectores: List[Tuple[float, float]] = []
    for trozo in texto.split(","):
        inicio, _, fin = trozo.partition("-")
        sectores.append((float(inicio), float(fin)))
    return tuple(sectores)


def _argumentos(argv=None):
    parser = argparse.ArgumentParser(description="Ronda Abierta WRO FE (Pi 5)")
    parser.add_argument("--velocidad", type=int, default=VELOCIDAD_CRUCERO,
                        help="PWM de crucero (heredado de la 3B: %(default)s)")
    parser.add_argument("--velocidad-meta", type=int, default=VELOCIDAD_META,
                        help="PWM al buscar la seccion de salida")
    parser.add_argument("--vueltas", type=int, default=VUELTAS_OBJETIVO)
    parser.add_argument("--kp", type=float, default=KP_LATERAL)
    parser.add_argument("--avance-meta", type=float, default=TIEMPO_AVANCE_META_S,
                        help="segundos de avance dentro de la seccion de salida")
    parser.add_argument("--timeout-total", type=float, default=210.0)
    parser.add_argument("--arranque-inmediato", action="store_true",
                        help="no espera GP21 (para lanzar por SSH)")
    parser.add_argument("--puerto-pico", default=PUERTO_PICO)
    parser.add_argument("--puerto-lidar", default=PUERTO_LIDAR)
    parser.add_argument("--baudrate-lidar", type=int, default=BAUDRATE_LIDAR)
    parser.add_argument("--sectores-ciegos", type=_sectores, default=SECTORES_CIEGOS,
                        help="arcos de la propia estructura, p.ej. 27-55,140-213")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _argumentos(argv)
    ronda = RondaAbierta(args)

    def cerrar(_sig, _frame):
        ronda._terminar("interrupcion solicitada")

    signal.signal(signal.SIGINT, cerrar)
    signal.signal(signal.SIGTERM, cerrar)
    try:
        return ronda.ejecutar()
    finally:
        ronda.cerrar()


if __name__ == "__main__":
    raise SystemExit(main())
