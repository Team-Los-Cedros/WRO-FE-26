"""Drivers exclusivos de la ronda nueva.

Los imports dependientes de Raspberry se hacen al instanciar/arrancar cada
driver, por lo que importar este modulo en Windows sigue siendo seguro.
"""

import threading
import time
from typing import Callable, Dict, Optional, Tuple


class EnlacePicoNuevo:
    """USB CDC con heading, color de piso y edad de telemetria."""

    def __init__(self, puerto: str, baudrate: int = 115200, timeout_s: float = 0.5):
        import serial

        self._serial_mod = serial
        self._ser = serial.Serial(puerto, baudrate=baudrate, timeout=0.05)
        self._timeout_s = float(timeout_s)
        self._lock = threading.Lock()
        self._yaw_crudo = 0.0
        self._cero_yaw = None
        self._color_piso = "DESCONOCIDO"
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
        """

        if not linea.startswith("IMU:"):
            return None
        yaw = None
        color = None
        watchdog = None
        for campo in linea.split(","):
            if campo.startswith("IMU:"):
                yaw = float(campo.split(":", 1)[1])
            elif campo.startswith("COLOR:"):
                color = campo.split(":", 1)[1].strip().upper()
            elif campo.startswith("WD:"):
                anunciado = campo.split(":", 1)[1].strip().upper()
                watchdog = anunciado if anunciado in ("OK", "STOP") else "INVALIDO"
        if yaw is None:
            return None
        return yaw, color, watchdog

    def _leer(self) -> None:
        while self._corriendo:
            try:
                linea = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if not linea:
                    continue
                parsed = self.parsear_telemetria_extendida(linea)
                if parsed is None:
                    continue
                yaw, color, watchdog = parsed
                ahora = time.monotonic()
                with self._lock:
                    self._yaw_crudo = yaw
                    if self._cero_yaw is None:
                        self._cero_yaw = yaw
                    if color:
                        self._color_piso = color
                    if watchdog is not None:
                        self._watchdog_comando = watchdog
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
