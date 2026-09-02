"""Drivers exclusivos de la ronda nueva.

Los imports dependientes de Raspberry se hacen al instanciar/arrancar cada
driver, por lo que importar este modulo en Windows sigue siendo seguro.
"""

import math
import threading
import time
from typing import Callable, Dict, NamedTuple, Optional, Tuple


class TramaPico(NamedTuple):
    """Trama de telemetria completa tal y como la emite la Pico 2."""

    yaw: float
    color: Optional[str]
    watchdog: Optional[str]
    ultrasonido_mm: Optional[float]


class EnlacePicoNuevo:
    """USB CDC con heading, color de piso, ultrasonido y edad de telemetria."""

    def __init__(self, puerto: str, baudrate: int = 115200, timeout_s: float = 0.5):
        import serial

        self._serial_mod = serial
        self._ser = serial.Serial(puerto, baudrate=baudrate, timeout=0.05)
        self._timeout_s = float(timeout_s)
        self._lock = threading.Lock()
        self._yaw_crudo = 0.0
        self._cero_yaw = None
        self._color_piso = "DESCONOCIDO"
        self._ultrasonido_mm: Optional[float] = None
        self._watchdog_comando = None
        self._t_telemetria = 0.0
        self._corriendo = True
        self._hilo = threading.Thread(target=self._leer, name="pico-ronda-nueva", daemon=True)
        self._hilo.start()

    @staticmethod
    def parsear_telemetria(linea: str) -> Optional[Tuple[float, Optional[str]]]:
        """Parser historico; conserva exactamente el retorno ``(yaw, color)``."""

        extendida = EnlacePicoNuevo.parsear_telemetria_extendida(linea)
        if extendida is None:
            return None
        yaw, color, _watchdog = extendida
        return yaw, color

    @staticmethod
    def parsear_telemetria_extendida(
        linea: str,
    ) -> Optional[Tuple[float, Optional[str], Optional[str]]]:
        """Devuelve yaw, color y estado anunciado del watchdog de comandos.

        ``watchdog`` es ``None`` para firmware historico, ``OK``/``STOP`` para
        el firmware seguro e ``INVALIDO`` si la trama anuncia otro valor. Un
        valor invalido no inutiliza la IMU, pero nunca permite armar motores.

        Conserva a proposito la tupla de tres campos: es el contrato que ya
        usan las pruebas y el resto del arranque. El campo nuevo del
        ultrasonido se lee en :meth:`parsear_telemetria_completa`.
        """

        trama = EnlacePicoNuevo.parsear_telemetria_completa(linea)
        if trama is None:
            return None
        return trama.yaw, trama.color, trama.watchdog

    @staticmethod
    def parsear_telemetria_completa(linea: str) -> Optional[TramaPico]:
        """Lee la trama entera, incluido el ultrasonido trasero (``US:``).

        ``ultrasonido_mm`` es ``None`` cuando el firmware no anuncia el campo
        (version anterior), cuando el sensor no vio eco (la Pico manda -1) o
        cuando el numero llega corrupto. Un valor ausente NUNCA se confunde
        con "camino libre": quien lo consume debe tratar ``None`` como falta
        de evidencia, no como distancia grande.

        Un ``US:`` ilegible no invalida la trama: la IMU y el watchdog son
        criticos para la ronda y el ultrasonido solo es una ayuda.
        """

        if not linea.startswith("IMU:"):
            return None
        yaw = None
        color = None
        watchdog = None
        ultrasonido = None
        for campo in linea.split(","):
            if campo.startswith("IMU:"):
                yaw = float(campo.split(":", 1)[1])
            elif campo.startswith("COLOR:"):
                color = campo.split(":", 1)[1].strip().upper()
            elif campo.startswith("WD:"):
                anunciado = campo.split(":", 1)[1].strip().upper()
                watchdog = anunciado if anunciado in ("OK", "STOP") else "INVALIDO"
            elif campo.startswith("US:"):
                try:
                    medida = float(campo.split(":", 1)[1])
                except (TypeError, ValueError):
                    medida = None
                if medida is not None and math.isfinite(medida) and medida > 0.0:
                    ultrasonido = medida
        if yaw is None:
            return None
        return TramaPico(yaw, color, watchdog, ultrasonido)

    def _leer(self) -> None:
        while self._corriendo:
            try:
                linea = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if not linea:
                    continue
                trama = self.parsear_telemetria_completa(linea)
                if trama is None:
                    continue
                ahora = time.monotonic()
                with self._lock:
                    self._yaw_crudo = trama.yaw
                    if self._cero_yaw is None:
                        self._cero_yaw = trama.yaw
                    if trama.color:
                        self._color_piso = trama.color
                    if trama.watchdog is not None:
                        self._watchdog_comando = trama.watchdog
                    # Se guarda tal cual, incluido el None: una trama sin eco
                    # tiene que borrar la medida anterior, no dejarla vigente.
                    self._ultrasonido_mm = trama.ultrasonido_mm
                    self._t_telemetria = ahora
            except (ValueError, IndexError):
                continue
            except self._serial_mod.SerialException:
                time.sleep(0.05)

    def fijar_cero(self) -> None:
        with self._lock:
            self._cero_yaw = self._yaw_crudo

    def heading(self) -> float:
        with self._lock:
            cero = self._cero_yaw
            return 0.0 if cero is None else self._yaw_crudo - cero

    def color_piso(self) -> str:
        with self._lock:
            return self._color_piso

    def distancia_ultrasonido_mm(
        self, ahora: Optional[float] = None
    ) -> Optional[float]:
        """Distancia trasera medida por ultrasonido, o ``None``.

        Devuelve ``None`` en los tres casos que significan lo mismo -- no hay
        evidencia -- para que ningun consumidor tenga que distinguirlos: el
        firmware no anuncia el campo, el sensor no vio eco, o la telemetria
        entera caduco. La caducidad se comparte con el resto de la trama
        porque la Pico manda el ultrasonido en cada envio: si la trama esta
        fresca, la medida tambien.
        """

        instante = time.monotonic() if ahora is None else float(ahora)
        with self._lock:
            if self._t_telemetria <= 0.0:
                return None
            if (instante - self._t_telemetria) > self._timeout_s:
                return None
            return self._ultrasonido_mm

    def estado_watchdog_comando(self) -> Optional[str]:
        """Estado mas reciente anunciado por la Pico, o ``None`` si no existe."""

        with self._lock:
            return self._watchdog_comando

    def watchdog_comando_ok(self) -> bool:
        with self._lock:
            return self._watchdog_comando == "OK"

    def telemetria_valida(self, ahora: Optional[float] = None) -> bool:
        instante = time.monotonic() if ahora is None else ahora
        with self._lock:
            return self._t_telemetria > 0.0 and (instante - self._t_telemetria) <= self._timeout_s

    def enviar(self, velocidad: int, angulo: float) -> bool:
        try:
            self._ser.write(f"{int(velocidad)},{float(angulo):.2f}\n".encode("ascii"))
            return True
        except self._serial_mod.SerialException:
            return False

    def detener(self) -> None:
        for _ in range(5):
            self.enviar(0, 0.0)
            time.sleep(0.01)

    def cerrar(self) -> None:
        self._corriendo = False
        try:
            self.detener()
            self._ser.close()
        except Exception:
            pass


class FuenteCamara:
    """Entrega el frame mas reciente; vision decide rotacion, ROI y color.

    ``picamera_format`` es el nombre solicitado a libcamera. No describe por
    si solo el orden del ``ndarray``: en el montaje probado ``RGB888`` llega
    a OpenCV como BGR, dato separado en ``array_color_order``.
    """

    def __init__(self, config_camara: Dict):
        self._cfg = config_camara
        self._camara = None
        # ``ultimo_error`` queda reservado para fallos que hacen imposible
        # seguir capturando. No mezclar aqui avisos recuperables: el arranque
        # usa esta propiedad como una condicion de parada de seguridad.
        self._ultimo_error = None
        self._ultima_advertencia = None
        self._lista = threading.Event()

    @property
    def lista(self) -> bool:
        return self._lista.is_set()

    @property
    def ultimo_error(self):
        """Ultimo fallo fatal del hilo, o ``None`` si puede capturar."""

        return self._ultimo_error

    @property
    def ultima_advertencia(self):
        """Ultimo problema recuperable (por ejemplo, AE/AWB no bloqueable)."""

        return self._ultima_advertencia

    def _bloquear_automaticos(self) -> None:
        if not self._cfg.get("lock_auto_exposure", True):
            return
        try:
            meta = self._camara.capture_metadata()
            controles = {"AeEnable": False, "AwbEnable": False}
            for clave in ("ExposureTime", "AnalogueGain", "ColourGains"):
                if clave in meta:
                    controles[clave] = meta[clave]
            self._camara.set_controls(controles)
        except Exception as exc:
            # Algunas versiones antiguas de Picamera2 no exponen todos los
            # controles. La captura puede continuar; se registra por separado
            # para que el supervisor no la confunda con un fallo fatal.
            self._ultima_advertencia = f"No se pudo bloquear AE/AWB: {exc}"

    def bucle(self, seguir: Callable[[], bool], al_frame: Callable) -> None:
        try:
            from picamera2 import Picamera2

            self._camara = Picamera2()
            fps = max(1.0, float(self._cfg["fps"]))
            duracion_us = int(1_000_000.0 / fps)
            argumentos_config = {
                "main": {
                    "size": (int(self._cfg["width"]), int(self._cfg["height"])),
                    "format": self._cfg.get("picamera_format", "RGB888"),
                },
                "controls": {"FrameDurationLimits": (duracion_us, duracion_us)},
                "buffer_count": 2,
            }
            # El montaje del mastil fue calibrado usando la salida 16:9 y el
            # modo raw 2304x1296. Sin declarar ``raw`` libcamera puede elegir
            # otro recorte y dejan de ser validos HFOV, focal y centro optico.
            modo_raw = self._cfg.get("raw_sensor_size")
            if modo_raw:
                argumentos_config["raw"] = {
                    "size": (int(modo_raw[0]), int(modo_raw[1]))
                }
            config = self._camara.create_video_configuration(**argumentos_config)
            self._camara.configure(config)
            self._camara.start()
            time.sleep(float(self._cfg.get("lock_auto_after_s", 2.0)))
            self._bloquear_automaticos()
            self._lista.set()

            latencia = float(self._cfg.get("latency_s", 0.0))
            while seguir():
                frame = self._camara.capture_array("main")[:, :, :3]
                # Este timestamp ya representa el instante estimado de
                # captura. La fusion no debe volver a restar la latencia.
                al_frame(frame, time.monotonic() - latencia)
        except Exception as exc:
            self._ultimo_error = str(exc)
        finally:
            self._lista.clear()
            if self._camara is not None:
                try:
                    self._camara.stop()
                except Exception:
                    pass
