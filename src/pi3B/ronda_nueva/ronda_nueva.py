"""Punto de entrada seguro para la ronda nueva.

Se ejecuta conservando la estructura de paquete::

    cd /home/pi/wro_nueva
    python3 -m ronda_nueva.ronda_nueva --config ronda_nueva/configuracion.json

Los imports de Raspberry y los puertos se abren solo dentro de ``ejecutar``.
Importar este modulo o usar ``--validar-config`` es seguro en una laptop.
"""

import argparse
import copy
import signal
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

from .config import (
    ErrorConfiguracion,
    calibraciones_pendientes,
    cargar_configuracion,
    exigir_listo_para_mover,
)
from .control_ruta import ControlRuta
from .fusion import FusionLigera
from .hardware import EnlacePicoNuevo, FuenteCamara
from .percepcion_lidar import PercepcionLidar
from .sincronizacion import BuzonVision
from .telemetria import TelemetriaAsincrona
from .vision_ligera import VisionLigera


CAMPOS_TELEMETRIA = (
    "t",
    "estado",
    "razon",
    "velocidad",
    "angulo",
    "heading",
    "color_piso",
    "watchdog_pico",
    "esquinas",
    "frontal",
    "frontal_muro",
    "izquierda",
    "derecha",
    "trasera",
    "trasera_valida",
    "cobertura_trasera",
    "trasera_izquierda",
    "trasera_izquierda_valida",
    "cobertura_trasera_izquierda",
    "trasera_derecha",
    "trasera_derecha_valida",
    "cobertura_trasera_derecha",
    "calidad_pared",
    "tracks",
    "tracks_confirmados",
    "track_activo_id",
    "track_activo_color",
    "track_activo_x",
    "track_activo_y",
    "track_activo_edad",
    "track_activo_observado",
    "distancia_sobrepaso",
    "hueco_confianza",
    "vision_edad_ms",
)


def _drivers_comunes():
    """Importa los drivers probados tanto desde repo como desde despliegue."""

    try:
        from ..comun.lidar_driver import LidarDriver
        from ..comun.lidar_geometria import ProcesadorLidar
    except (ImportError, ValueError):  # paquete copiado junto a ``comun``
        from comun.lidar_driver import LidarDriver
        from comun.lidar_geometria import ProcesadorLidar
    return LidarDriver, ProcesadorLidar


class AplicacionRondaNueva:
    """Conecta adquisicion, percepcion y control sin poner I/O en la logica."""

    def __init__(
        self,
        config: Dict[str, Any],
        permitir_parqueo: bool = True,
        esperar_boton: bool = True,
    ):
        self.config = config
        self.permitir_parqueo = bool(permitir_parqueo)
        self.esperar_boton = bool(esperar_boton)
        self._seguir = threading.Event()
        self._seguir.set()
        self._armado = threading.Event()
        self._barrido_recibido = threading.Event()
        self._oclusion_validada = threading.Event()

        self.vision = VisionLigera(config)
        self.buzon_vision = BuzonVision(4)
        self.percepcion = PercepcionLidar(config)
        self.fusion = FusionLigera(config)
        self.control = ControlRuta(config)

        self.enlace: Optional[EnlacePicoNuevo] = None
        self.fuente_camara: Optional[FuenteCamara] = None
        self.lidar_driver = None
        self.lidar_geo = None
        self.registro: Optional[TelemetriaAsincrona] = None
        self._gpio = None
        self._hilo_camara: Optional[threading.Thread] = None
        self._hilo_lidar: Optional[threading.Thread] = None

        self._ultimo_barrido = 0.0
        self._ultima_velocidad = 0
        self._oclusion_ok_consecutivos = 0
        self._error_oclusion = ""
        self._motivo_fin = ""
        self._terminado_verificado = False

    @property
    def corriendo(self) -> bool:
        return self._seguir.is_set()

    def _detener_por_fallo(self, razon: str) -> None:
        if not self._motivo_fin:
            self._motivo_fin = razon
            print("[-] " + razon)
        if self.enlace is not None:
            self.enlace.detener()
        self._seguir.clear()

    def _al_frame(self, frame, timestamp: float) -> None:
        try:
            self.buzon_vision.publicar(self.vision.procesar(frame, timestamp))
        except Exception as exc:
            self._detener_por_fallo("fallo procesando camara: {}".format(exc))

    def _validar_oclusion_inicial(self, corredor) -> None:
        requeridos = max(
            1, int(self.config["hardware"].get("occlusion_validation_scans", 3))
        )
        if corredor.estructura_fuera_mascara_deg:
            self._error_oclusion = corredor.diagnostico_oclusion
            self._oclusion_ok_consecutivos = 0
            return
        if corredor.mascara_oclusion_confirmada:
            self._oclusion_ok_consecutivos += 1
        else:
            self._error_oclusion = corredor.diagnostico_oclusion
            self._oclusion_ok_consecutivos = 0
        if self._oclusion_ok_consecutivos >= requeridos:
            self._oclusion_validada.set()

    def _registrar(self, ahora, corredor, tracks, hueco, consigna, paquete) -> None:
        if self.registro is None or self.enlace is None:
            return
        edad_vision = (
            float("inf") if paquete is None else max(0.0, ahora - paquete.timestamp)
        )
        track_activo = self.control.track_activo
        self.registro.registrar(
            {
                "t": "{:.6f}".format(ahora),
                "estado": consigna.estado,
                "razon": consigna.razon,
                "velocidad": consigna.velocidad,
                "angulo": "{:.2f}".format(consigna.angulo),
                "heading": "{:.2f}".format(self.enlace.heading()),
                "color_piso": self.enlace.color_piso(),
                "watchdog_pico": self.enlace.estado_watchdog_comando() or "NO_ANUNCIADO",
                "esquinas": self.control.esquinas,
                "frontal": "{:.1f}".format(corredor.frontal_mm),
                "frontal_muro": "{:.1f}".format(corredor.frontal_muro_mm),
                "izquierda": "{:.1f}".format(corredor.izquierda_mm),
                "derecha": "{:.1f}".format(corredor.derecha_mm),
                "trasera": "{:.1f}".format(corredor.trasera_mm),
                "trasera_valida": int(corredor.trasera_valida),
                "cobertura_trasera": "{:.3f}".format(corredor.cobertura_trasera),
                "trasera_izquierda": "{:.1f}".format(
                    corredor.trasera_izquierda_mm
                ),
                "trasera_izquierda_valida": int(
                    corredor.trasera_izquierda_valida
                ),
                "cobertura_trasera_izquierda": "{:.3f}".format(
                    corredor.cobertura_trasera_izquierda
                ),
                "trasera_derecha": "{:.1f}".format(
                    corredor.trasera_derecha_mm
                ),
                "trasera_derecha_valida": int(
                    corredor.trasera_derecha_valida
                ),
                "cobertura_trasera_derecha": "{:.3f}".format(
                    corredor.cobertura_trasera_derecha
                ),
                "calidad_pared": "{:.3f}".format(corredor.calidad_pared),
                "tracks": len(tracks),
                "tracks_confirmados": sum(1 for track in tracks if track.confirmado),
                "track_activo_id": (
                    "" if self.control.track_activo_id is None
                    else self.control.track_activo_id
                ),
                "track_activo_color": self.control.track_activo_color or "",
                "track_activo_x": (
                    "" if track_activo is None
                    else "{:.1f}".format(track_activo.x_mm)
                ),
                "track_activo_y": (
                    "" if track_activo is None
                    else "{:.1f}".format(track_activo.y_mm)
                ),
                "track_activo_edad": (
                    "" if track_activo is None
                    else "{:.3f}".format(track_activo.edad_s)
                ),
                "track_activo_observado": int(
                    self.control.track_activo_observado
                ),
                "distancia_sobrepaso": "{:.1f}".format(
                    self.control.distancia_sobrepaso_mm
                ),
                "hueco_confianza": "" if hueco is None else "{:.3f}".format(hueco.confianza),
                "vision_edad_ms": (
                    "" if paquete is None else "{:.1f}".format(1000.0 * edad_vision)
                ),
            }
        )

    def _al_barrido(self, scan) -> None:
        """Un ciclo completo de decision; corre dentro del hilo LiDAR."""

        if not self.corriendo or self.enlace is None or self.lidar_geo is None:
            return
        ahora = time.monotonic()
        try:
            medicion = self.lidar_geo.procesar(scan)
            lado_parqueo = (
                self.control.lado_parqueo_solicitado
                if self._armado.is_set() and self.permitir_parqueo
                else 0
            )
            resultado = self.percepcion.procesar(
                scan,
                medicion,
                timestamp=ahora,
                lado_parqueo=lado_parqueo,
            )
            self._ultimo_barrido = ahora
            self._barrido_recibido.set()

            if not self._armado.is_set():
                self._validar_oclusion_inicial(resultado.corredor)
                self.enlace.enviar(0, 0.0)
                return

            paquete = self.buzon_vision.mas_cercano(
                ahora, float(self.config["fusion"]["max_camera_lidar_age_s"])
            )
            tracks = self.fusion.actualizar(
                resultado.objetos,
                paquete,
                self.enlace.heading(),
                self._ultima_velocidad,
                timestamp=ahora,
            )
            consigna = self.control.procesar(
                resultado.corredor,
                tracks,
                self.enlace.heading(),
                self.enlace.color_piso(),
                hueco=resultado.hueco,
                ahora=ahora,
            )

            # Ninguna FSM puede autorizar movimiento si un sensor esencial
            # caduco entre dos callbacks.
            if not self.enlace.telemetria_valida(ahora):
                self._detener_por_fallo("watchdog IMU/Pico vencido")
                return
            if not self._comprobar_watchdog_pico_en_ejecucion():
                return
            if self.buzon_vision.edad_ultimo(ahora) > float(
                self.config["hardware"]["camera_watchdog_s"]
            ):
                self._detener_por_fallo("watchdog de camara vencido")
                return

            if not self.enlace.enviar(consigna.velocidad, consigna.angulo):
                self._detener_por_fallo("no se pudo enviar la consigna a la Pico")
                return
            self._ultima_velocidad = consigna.velocidad
            self._registrar(
                ahora, resultado.corredor, tracks, resultado.hueco, consigna, paquete
            )

            if consigna.terminado:
                self._terminado_verificado = consigna.verificado
                self._motivo_fin = consigna.razon or consigna.estado
                self.enlace.detener()
                self._seguir.clear()
        except Exception as exc:
            self._detener_por_fallo("fallo en ciclo LiDAR/control: {}".format(exc))

    def _esperar_boton(self, GPIO) -> bool:
        pin = int(self.config["hardware"]["start_button_bcm"])
        if not self.esperar_boton:
            # Banco de pruebas remoto (SSH): no hay quien presione GP21.
            # El bucle del boton es tambien la fuente del heartbeat 0,0 que
            # saca a la Pico de WD:STOP, asi que se envia un segundo de
            # heartbeats antes de continuar o el armado se negaria.
            print(
                "[LISTO] Arranque inmediato solicitado; "
                "no se espera GP{}.".format(pin)
            )
            limite = time.monotonic() + 1.0
            while self.corriendo and time.monotonic() < limite:
                assert self.enlace is not None
                self.enlace.enviar(0, 0.0)
                time.sleep(0.05)
            return self.corriendo
        print("[LISTO] Robot detenido; presiona GP{} para iniciar.".format(pin))
        while self.corriendo and GPIO.input(pin) == GPIO.HIGH:
            assert self.enlace is not None
            self.enlace.enviar(0, 0.0)
            if self.fuente_camara is not None and self.fuente_camara.ultimo_error:
                self._detener_por_fallo(
                    "camara no disponible: {}".format(
                        self.fuente_camara.ultimo_error
                    )
                )
                return False
            time.sleep(0.05)
        return self.corriendo

    def _watchdog_pico_requerido(self) -> bool:
        return bool(
            self.config["hardware"].get("require_pico_command_watchdog", True)
        )

    def _comprobar_watchdog_pico_en_ejecucion(self) -> bool:
        """Detiene si el firmware requerido deja de anunciar estado seguro."""

        if not self._watchdog_pico_requerido():
            return True
        assert self.enlace is not None
        if self.enlace.watchdog_comando_ok():
            return True
        estado = self.enlace.estado_watchdog_comando() or "NO_ANUNCIADO"
        self._detener_por_fallo(
            "watchdog autonomo de la Pico no esta OK: {}".format(estado)
        )
        return False

    def _esperar_watchdog_pico_listo(self) -> bool:
        """Mantiene freno/heartbeat hasta que la Pico anuncie ``WD:OK``."""

        if not self._watchdog_pico_requerido():
            return True
        assert self.enlace is not None
        espera_s = max(
            1.0, 2.0 * float(self.config["hardware"]["imu_watchdog_s"])
        )
        limite = time.monotonic() + espera_s
        while self.corriendo and time.monotonic() <= limite:
            if not self.enlace.enviar(0, 0.0):
                self._detener_por_fallo(
                    "no se pudo enviar heartbeat seguro a la Pico"
                )
                return False
            if self.enlace.watchdog_comando_ok():
                return True
            time.sleep(0.02)
        estado = self.enlace.estado_watchdog_comando() or "NO_ANUNCIADO"
        self._detener_por_fallo(
            "la Pico no confirmo WD:OK antes de armar: {}".format(estado)
        )
        return False

    def ejecutar(self) -> bool:
        """Abre hardware, espera el boton y no arma motores sin diagnostico."""

        LidarDriver, ProcesadorLidar = _drivers_comunes()
        import RPi.GPIO as GPIO

        self._gpio = GPIO
        hardware = self.config["hardware"]
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(
            int(hardware["start_button_bcm"]),
            GPIO.IN,
            pull_up_down=GPIO.PUD_UP,
        )

        instante_log = time.strftime("%Y%m%d_%H%M%S")
        ruta_log = (
            Path(self.config["runtime"]["log_directory"])
            / ("ronda_nueva_{}.csv".format(instante_log))
        )
        self.registro = TelemetriaAsincrona(
            ruta_log,
            CAMPOS_TELEMETRIA,
            int(self.config["runtime"]["telemetry_queue_size"]),
        )

        self.fuente_camara = FuenteCamara(self.config["camera"])
        self._hilo_camara = threading.Thread(
            target=self.fuente_camara.bucle,
            args=(lambda: self.corriendo, self._al_frame),
            name="camara-ronda-nueva",
            daemon=True,
        )
        self._hilo_camara.start()

        self.enlace = EnlacePicoNuevo(
            hardware["pico_port"],
            int(hardware["pico_baudrate"]),
            float(hardware["imu_watchdog_s"]),
        )
        self.enlace.detener()

        if not self._esperar_boton(GPIO):
            return False
        limite_preparacion = time.monotonic() + 5.0
        while self.corriendo and not self.fuente_camara.lista:
            if self.fuente_camara.ultimo_error:
                self._detener_por_fallo(
                    "camara no disponible: {}".format(
                        self.fuente_camara.ultimo_error
                    )
                )
                return False
            if time.monotonic() > limite_preparacion:
                self._detener_por_fallo("la camara no quedo lista en 5 s")
                return False
            time.sleep(0.05)
        if self.fuente_camara.ultima_advertencia:
            print(
                "[!] Advertencia de camara (captura activa): {}".format(
                    self.fuente_camara.ultima_advertencia
                )
            )
        if not self.enlace.telemetria_valida():
            self._detener_por_fallo("no hay telemetria valida de la Pico")
            return False
        if not self._esperar_watchdog_pico_listo():
            return False

        self.enlace.fijar_cero()
        self.lidar_geo = ProcesadorLidar()
        self.lidar_driver = LidarDriver(
            hardware["lidar_port"], int(hardware["lidar_baudrate"])
        )
        self._hilo_lidar = threading.Thread(
            target=self.lidar_driver.hilo_lectura,
            args=(lambda: self.corriendo, self._al_barrido),
            name="lidar-ronda-nueva",
            daemon=True,
        )
        self._hilo_lidar.start()

        timeout_inicio = float(hardware.get("startup_scan_timeout_s", 6.0))
        limite = time.monotonic() + timeout_inicio
        while self.corriendo and not self._oclusion_validada.is_set():
            if time.monotonic() > limite:
                detalle = self._error_oclusion or "no llegaron barridos suficientes"
                self._detener_por_fallo(
                    "no se valido la mascara del mastil: " + detalle
                )
                return False
            time.sleep(0.05)

        print("[OK] Mascara trasera validada; traccion armada.")
        self._armado.set()

        while self.corriendo:
            ahora = time.monotonic()
            edad_lidar = ahora - self._ultimo_barrido
            if edad_lidar > float(hardware["lidar_watchdog_s"]):
                self.enlace.detener()
                if edad_lidar > 5.0:
                    self._detener_por_fallo("LiDAR sin barridos durante 5 s")
                    break
            if not self.enlace.telemetria_valida(ahora):
                self._detener_por_fallo("watchdog IMU/Pico vencido")
                break
            if not self._comprobar_watchdog_pico_en_ejecucion():
                break
            if self.buzon_vision.edad_ultimo(ahora) > float(
                hardware["camera_watchdog_s"]
            ):
                self._detener_por_fallo("watchdog de camara vencido")
                break
            time.sleep(0.05)
        return self._terminado_verificado

    def cerrar(self) -> None:
        self._seguir.clear()
        self._armado.clear()
        if self.enlace is not None:
            self.enlace.detener()
        if self.lidar_driver is not None:
            self.lidar_driver.cerrar()
        for hilo in (self._hilo_lidar, self._hilo_camara):
            if hilo is not None and hilo.is_alive():
                hilo.join(timeout=2.0)
        if self.enlace is not None:
            self.enlace.cerrar()
        if self.registro is not None:
            self.registro.cerrar()
        if self._gpio is not None:
            try:
                self._gpio.cleanup()
            except Exception:
                pass
        if self._motivo_fin:
            print("[FIN] " + self._motivo_fin)


def _argumentos(argv=None):
    parser = argparse.ArgumentParser(description="Ronda nueva WRO FE")
    parser.add_argument("--config", help="JSON alternativo")
    parser.add_argument(
        "--validar-config",
        action="store_true",
        help="valida y muestra pendientes sin abrir hardware",
    )
    parser.add_argument(
        "--sin-parqueo",
        action="store_true",
        help="prueba de recorrido: no entra a parqueo ni exige su calibracion",
    )
    parser.add_argument(
        "--arranque-inmediato",
        action="store_true",
        help=(
            "banco de pruebas remoto: no espera el boton GP21; "
            "la ronda oficial no usa esta opcion"
        ),
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _argumentos(argv)
    try:
        config = cargar_configuracion(args.config)
        if args.validar_config:
            pendientes = calibraciones_pendientes(
                config, incluir_estacionamiento=not args.sin_parqueo
            )
            print("Configuracion valida: {}".format(config["_ruta"]))
            print("Calibraciones pendientes: {}".format(
                ", ".join(pendientes) if pendientes else "ninguna"
            ))
            print("Movimiento habilitado: {}".format(
                bool(config["runtime"]["motion_enabled"])
            ))
            return 0

        exigir_listo_para_mover(
            config, incluir_estacionamiento=not args.sin_parqueo
        )
        if args.sin_parqueo:
            config = copy.deepcopy(config)
            config["control"]["corners_before_parking"] = 1_000_000
        aplicacion = AplicacionRondaNueva(
            config,
            permitir_parqueo=not args.sin_parqueo,
            esperar_boton=not args.arranque_inmediato,
        )

        def solicitar_cierre(_sig, _frame):
            aplicacion._motivo_fin = "interrupcion solicitada"
            aplicacion._seguir.clear()

        signal.signal(signal.SIGINT, solicitar_cierre)
        signal.signal(signal.SIGTERM, solicitar_cierre)
        try:
            return 0 if aplicacion.ejecutar() else 1
        finally:
            aplicacion.cerrar()
    except (ErrorConfiguracion, OSError, ValueError) as exc:
        print("[-] {}".format(exc))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
