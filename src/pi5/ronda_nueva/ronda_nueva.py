"""Punto de entrada de la ronda de obstaculos en la Raspberry Pi 5.

Se ejecuta conservando la estructura de paquete::

    cd /home/pi/wro_pi5
    python3 -m ronda_nueva.ronda_nueva --config ronda_nueva/configuracion.json

Los imports de Raspberry y los puertos se abren solo dentro de ``ejecutar``,
asi que importar este modulo o usar ``--validar-config`` es seguro en el PC.

ARQUITECTURA DE HILOS
Tres hilos y ningun bloqueo entre ellos:

* **camara**: captura, procesa el cuadro y publica un ``PaqueteVision`` en un
  buzon pequeño.  Nunca se encolan imagenes; si el control va lento, lo que se
  pierde son resultados viejos, que es lo correcto.
* **lidar**: lee el puerto por bloques y publica el barrido en un buzon de UN
  hueco.  Un barrido nuevo pisa al que no se consumio, y el contador de
  descartados queda en el CSV.
* **principal**: toma el barrido, le busca la vision mas cercana en el tiempo,
  decide y envia la consigna.

El ciclo de decision va al ritmo del LiDAR (~10 Hz) y no del de la camara
(30 Hz), porque el LiDAR es quien mide las paredes y las paredes son quienes
mandan en la direccion.
"""

from __future__ import annotations

import argparse
import math
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
from .estacionamiento import ControlEstacionamiento
from .fusion import FusionPilares
from .hardware import EnlacePicoNuevo, FuenteCamara
from .percepcion_lidar import PercepcionLidar
from .piloto import Piloto
from .servidor_web import EstadoRobot, ServidorPanel
from .sincronizacion import BuzonBarridosLidar, BuzonVision
from .telemetria import TelemetriaAsincrona
from .vision_pista import VisionPista


CAMPOS_TELEMETRIA = (
    "t",
    "estado",
    "razon",
    "velocidad",
    "angulo",
    "heading",
    "watchdog_pico",
    "sentido",
    "esquinas",
    "segmento",
    "avance",
    "offset",
    "rumbo_error",
    "avance_valido",
    "offset_valido",
    "objetivo_offset",
    "error_lateral",
    "plan_cedido",
    "mapa",
    "casillas",
    "retrocesos",
    "atascos",
    "frontal_min",
    "izquierda_min",
    "derecha_min",
    "trasera_min",
    # ``corredor`` es lo que dispara emergencias y frenado, y hasta la corrida
    # 2 no estaba en el CSV: se analizaba con ``frontal_min``, que es otra cosa.
    "corredor",
    "corredor_deg",
    "pilares",
    # Desglose por origen.  Sin esto solo se veia ``pilar1_fuente``, que es el
    # PRIMERO de una lista ordenada por parejas de fusion, no el mas cercano:
    # sirve para sospechar, no para concluir.
    "pilares_camara",
    "pilares_lidar",
    "pilares_fusion",
    "pilar1_x",
    "pilar1_y",
    "pilar1_color",
    "pilar1_fuente",
    "vision_edad_ms",
    "lidar_edad_ms",
    "ciclo_ms",
    "percepcion_ms",
    "barridos_descartados",
    "ultrasonido_mm",
    "parqueo_estado",
    "parqueo_razon",
    "parqueo_vaivenes",
    "hueco_confianza",
    "hueco_separacion",
    "hueco_lateral",
)


def _driver_lidar():
    """Importa el driver probado tanto desde el repo como desde el despliegue."""

    try:
        from ..comun.lidar_driver import LidarDriver
    except (ImportError, ValueError):  # paquete copiado junto a ``comun``
        from comun.lidar_driver import LidarDriver
    return LidarDriver


class AplicacionRonda:
    """Conecta adquisicion, percepcion y control sin poner I/O en la logica."""

    def __init__(
        self,
        config: Dict[str, Any],
        esperar_boton: bool = True,
        permitir_parqueo: bool = True,
        simulacro: bool = False,
        puerto_panel: Optional[int] = None,
    ):
        self.config = config
        # El panel vive en hilos aparte, solo copia y comprime el cuadro si hay
        # alguien mirando, y se traga sus excepciones: no puede tumbar la ronda.
        self.estado_panel = EstadoRobot() if puerto_panel else None
        self.panel = (
            ServidorPanel(self.estado_panel, puerto_panel) if puerto_panel else None
        )
        self.esperar_boton = bool(esperar_boton)
        # En simulacro se monta la tuberia entera y se decide de verdad, pero
        # NUNCA se llama a ``enviar``: el robot no recibe ni una consigna.
        # Sirve para medir cadencia, edad del barrido y comportamiento de la
        # FSM sobre sensores reales sin arriesgar el chasis.
        self.simulacro = bool(simulacro)
        self._seguir = threading.Event()
        self._seguir.set()

        self.vision = VisionPista(config)
        self.percepcion = PercepcionLidar(config)
        self.fusion = FusionPilares(config)
        self.parqueo = ControlEstacionamiento(config) if permitir_parqueo else None
        self.piloto = Piloto(config, parqueo=self.parqueo)

        self.buzon_vision = BuzonVision(4)
        self.buzon_barridos = BuzonBarridosLidar()

        self.enlace: Optional[EnlacePicoNuevo] = None
        self.fuente_camara: Optional[FuenteCamara] = None
        self.lidar_driver = None
        self.registro: Optional[TelemetriaAsincrona] = None
        self._hilo_camara: Optional[threading.Thread] = None
        self._hilo_lidar: Optional[threading.Thread] = None
        self._motivo_fin = ""
        self._terminado_verificado = False
        self._ultimo_angulo = 0.0

    @property
    def corriendo(self) -> bool:
        return self._seguir.is_set()

    def detener(self, motivo: str = "") -> None:
        if motivo and not self._motivo_fin:
            self._motivo_fin = motivo
        self._seguir.clear()

    # ------------------------------------------------------------- callbacks

    def _al_frame(self, frame, timestamp: float) -> None:
        try:
            self.buzon_vision.publicar(self.vision.procesar(frame, timestamp))
            if self.estado_panel is not None and self.estado_panel.quiere_video:
                self.estado_panel.publicar_frame(self.vision.orientar(frame))
        except Exception as exc:  # una excepcion aqui no puede tumbar la ronda
            self.detener(f"vision: {exc}")

    def _al_barrido(self, scan, timestamp: float) -> None:
        self.buzon_barridos.publicar(scan, timestamp)

    # ----------------------------------------------------------------- ciclo

    def _ciclo(self, barrido, ahora: float) -> None:
        inicio_ciclo = time.perf_counter()
        edad_lidar_ms = max(0.0, (ahora - barrido.timestamp) * 1000.0)

        lado_parqueo = 0
        if self.piloto.estado.startswith("PARQUEO") or self.piloto.estado == "APROXIMACION":
            lado_parqueo = self.piloto._lado_de_bahia()

        if self.estado_panel is not None:
            self.estado_panel.publicar_barrido(barrido.muestras)

        paredes, objetos, hueco = self.percepcion.procesar(
            barrido.muestras,
            barrido.timestamp,
            angulo_servo_deg=self._ultimo_angulo,
            lado_parqueo=lado_parqueo,
        )

        percepcion_ms = (time.perf_counter() - inicio_ciclo) * 1000.0

        paquete = self.buzon_vision.mas_cercano(
            barrido.timestamp, self.fusion.edad_max_s
        )
        visuales = paquete.pilares if paquete else ()
        lineas = paquete.lineas if paquete else ()
        magenta = paquete.magenta if paquete else ()
        pilares = self.fusion.asociar(visuales, objetos, barrido.timestamp)

        heading = self.enlace.heading() if self.enlace else 0.0
        ultrasonido = (
            self.enlace.distancia_ultrasonido_mm() if self.enlace else None
        )

        consigna = self.piloto.procesar(
            paredes=paredes,
            pilares=pilares,
            lineas=lineas,
            magenta=magenta,
            hueco=hueco,
            rumbo_deg=heading,
            ultrasonido_mm=ultrasonido,
            ahora=ahora,
        )
        self._ultimo_angulo = consigna.angulo

        if self.enlace is not None and not self.simulacro:
            self.enlace.enviar(consigna.velocidad, consigna.angulo)

        self._registrar(
            ahora,
            consigna,
            paredes,
            pilares,
            paquete,
            edad_lidar_ms,
            ultrasonido,
            (time.perf_counter() - inicio_ciclo) * 1000.0,
            percepcion_ms,
        )

        if consigna.terminado:
            self._terminado_verificado = bool(consigna.verificado)
            self.detener(consigna.razon or "ronda terminada")

    def _registrar(
        self,
        ahora,
        consigna,
        paredes,
        pilares,
        paquete,
        edad_lidar_ms,
        ultrasonido,
        ciclo_ms=0.0,
        percepcion_ms=0.0,
    ) -> None:
        if self.registro is None:
            return
        fila: Dict[str, Any] = {
            "t": round(ahora, 3),
            "velocidad": consigna.velocidad,
            "angulo": round(consigna.angulo, 2),
            "heading": round(self.enlace.heading(), 2) if self.enlace else "",
            "watchdog_pico": self.enlace.estado_watchdog_comando() if self.enlace else "",
            "frontal_min": round(paredes.frontal_min_mm, 1),
            "izquierda_min": round(paredes.izquierda_min_mm, 1),
            "derecha_min": round(paredes.derecha_min_mm, 1),
            "trasera_min": round(paredes.trasera_min_mm, 1),
            "corredor": round(paredes.corredor_mm, 1)
            if math.isfinite(paredes.corredor_mm)
            else "",
            # Diagnostico: de que rumbo viene el eco que cierra el corredor.
            "corredor_deg": round(paredes.corredor_deg, 1)
            if math.isfinite(paredes.corredor_deg)
            else "",
            "izquierda": round(paredes.izquierda_min_mm, 1)
            if math.isfinite(paredes.izquierda_min_mm)
            else "",
            "derecha": round(paredes.derecha_min_mm, 1)
            if math.isfinite(paredes.derecha_min_mm)
            else "",
            "trasera": round(paredes.trasera_min_mm, 1)
            if math.isfinite(paredes.trasera_min_mm)
            else "",
            "pilares": len(pilares),
            # Los objetos del LiDAR salen como LIDAR (sin pareja) o FUSION
            # (con ella), asi que la suma es cuantos vio el LiDAR.
            "pilares_camara": len(paquete.pilares) if paquete else 0,
            "pilares_lidar": sum(
                1 for p in pilares if p.fuente in ("LIDAR", "FUSION")
            ),
            "pilares_fusion": sum(1 for p in pilares if p.fuente == "FUSION"),
            "vision_edad_ms": round(
                self.buzon_vision.edad_ultimo(ahora) * 1000.0, 1
            ),
            # Microsegundos, no decimas. En la 3B esta edad eran 16 ms y una
            # decima sobraba; en la Pi 5 la mediana es 0,3 ms, asi que
            # redondear a 0,1 convierte la columna en doce valores sueltos y
            # el cuanto pasa a ser un tercio de la magnitud que se mide.
            "lidar_edad_ms": round(edad_lidar_ms, 3),
            "ciclo_ms": round(ciclo_ms, 2),
            "percepcion_ms": round(percepcion_ms, 2),
            "barridos_descartados": self.buzon_barridos.descartados,
            "ultrasonido_mm": "" if ultrasonido is None else round(ultrasonido, 1),
        }
        if pilares:
            primero = pilares[0]
            fila.update(
                {
                    "pilar1_x": round(primero.x_mm, 1),
                    "pilar1_y": round(primero.y_mm, 1),
                    "pilar1_color": primero.color,
                    "pilar1_fuente": primero.fuente,
                }
            )
        fila.update(self.piloto.instantanea())
        if self.parqueo is not None:
            fila.update(self.parqueo.instantanea())
        self.registro.registrar(fila)
        if self.estado_panel is not None:
            self.estado_panel.publicar(**fila)

    # ------------------------------------------------------------- arranque

    def _esperar_boton(self) -> bool:
        pin = int(self.config["hardware"].get("start_button_bcm", 21))
        try:
            import RPi.GPIO as GPIO
        except Exception as exc:
            print(f"[-] Sin GPIO ({exc}); usa --arranque-inmediato")
            return False
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print(f"[i] Esperando GP{pin}...")
        try:
            while self.corriendo:
                if GPIO.input(pin) == 0:
                    time.sleep(0.05)
                    if GPIO.input(pin) == 0:
                        return True
                time.sleep(0.02)
        finally:
            GPIO.cleanup(pin)
        return False

    def _esperar_telemetria(self, segundos: float) -> bool:
        """Espera a que la Pico empiece a hablar, sin bloquear el Ctrl-C."""

        limite = time.monotonic() + float(segundos)
        while self.corriendo and time.monotonic() < limite:
            if self.enlace.telemetria_valida():
                return True
            time.sleep(0.05)
        return self.enlace.telemetria_valida()

    def ejecutar(self) -> bool:
        LidarDriver = _driver_lidar()
        hardware = self.config["hardware"]
        runtime = self.config["runtime"]

        directorio = Path(runtime.get("log_directory", "logs"))
        marca = time.strftime("%Y%m%d_%H%M%S")
        self.registro = TelemetriaAsincrona(
            directorio / f"ronda_{marca}.csv",
            CAMPOS_TELEMETRIA,
            int(runtime.get("telemetry_queue_size", 256)),
        )

        try:
            self.enlace = EnlacePicoNuevo(
                hardware["pico_port"], int(hardware.get("pico_baudrate", 115200))
            )
        except Exception as exc:
            print(f"[-] No se pudo abrir la Pico: {exc}")
            return False

        self.fuente_camara = FuenteCamara(self.config["camera"])
        self._hilo_camara = threading.Thread(
            target=self.fuente_camara.bucle,
            args=(lambda: self.corriendo, self._al_frame),
            name="camara",
            daemon=True,
        )
        self._hilo_camara.start()

        try:
            self.lidar_driver = LidarDriver(
                hardware["lidar_port"], int(hardware.get("lidar_baudrate", 460800))
            )
        except Exception as exc:
            print(f"[-] No se pudo abrir el LiDAR: {exc}")
            self.cerrar()
            return False
        self._hilo_lidar = threading.Thread(
            target=self.lidar_driver.hilo_lectura,
            args=(lambda: self.corriendo, self._al_barrido),
            name="lidar",
            daemon=True,
        )
        self._hilo_lidar.start()

        limite = time.monotonic() + float(hardware.get("startup_scan_timeout_s", 6.0))
        while self.corriendo and time.monotonic() < limite:
            if self.buzon_barridos.recibidos > 0 and self.fuente_camara.lista:
                break
            time.sleep(0.05)
        else:
            if self.corriendo:
                print("[-] No llegaron barridos o la camara no arranco")
                self.cerrar()
                return False

        # La IMU tiene que estar hablando ANTES de armar la traccion: sin
        # telemetria el rumbo es cero constante y el robot conduce a ciegas
        # creyendo que va recto.
        if not self._esperar_telemetria(4.0):
            # Abrir el CDC de una MicroPython la REINICIA y la hace volver a
            # enumerar, asi que el primer descriptor puede quedarse mudo para
            # siempre.  Paso el 05-09 en el primer arranque tras reiniciar la
            # Pi: leyendo el puerto a mano habia 116 tramas en 6 s, pero la
            # ronda no veia ninguna.  Un reintento lo resuelve; si tampoco asi,
            # entonces si es la Pico.
            print("[i] La Pico no hablaba al abrir; reabriendo el puerto.")
            try:
                self.enlace.cerrar()
            except Exception:
                pass
            time.sleep(2.0)
            try:
                self.enlace = EnlacePicoNuevo(
                    hardware["pico_port"], int(hardware.get("pico_baudrate", 115200))
                )
            except Exception as exc:
                print(f"[-] No se pudo reabrir la Pico: {exc}")
                self.cerrar()
                return False
            if not self._esperar_telemetria(4.0):
                print(
                    "[-] La Pico no envia telemetria. Suele arreglarse con "
                    "'cd /home/pi/pico_nuevo && bash deploy_pico.sh --reiniciar'"
                )
                self.cerrar()
                return False
        # La deriva del giroscopio se mide DESDE AQUI, con todo encendido y el
        # robot quieto.  Ver EnlacePicoNuevo.calibrar_deriva: el 04-09 el
        # firmware dejo pasar un sesgo de 20,9 grados/s y sin esto cada esquina
        # habria salido con 42 grados de error.
        deriva = self.enlace.calibrar_deriva(
            float(hardware.get("gyro_drift_seconds", 2.5))
        )
        limite_deriva = float(hardware.get("gyro_drift_warn_deg_s", 1.0))
        if abs(deriva) > limite_deriva:
            print(
                f"[!] Deriva del giroscopio {deriva:+.2f} deg/s: se compensa por"
                " software, pero conviene reiniciar la Pico para que rehaga su"
                " calibracion ('cd /home/pi/pico_nuevo && bash deploy_pico.sh"
                " --reiniciar')."
            )
        else:
            print(f"[i] Deriva del giroscopio {deriva:+.2f} deg/s.")

        if self.panel is not None and self.panel.arrancar():
            print(f"[+] Panel web en http://<ip>:{self.panel.puerto}/")

        if self.esperar_boton and not self._esperar_boton():
            self.cerrar()
            return False

        if self.simulacro:
            print("[+] SIMULACRO: se decide todo pero no se envia nada al motor.")
        else:
            print("[+] Rodando. Ctrl-C para parar.")
        while self.corriendo:
            barrido = self.buzon_barridos.tomar(timeout_s=0.5)
            if barrido is None:
                continue
            try:
                self._ciclo(barrido, time.monotonic())
            except Exception as exc:
                self.detener(f"ciclo: {exc}")
                raise
        self.cerrar()
        return self._terminado_verificado

    def cerrar(self) -> None:
        self._seguir.clear()
        if self.enlace is not None:
            try:
                self.enlace.detener()
                self.enlace.cerrar()
            except Exception:
                pass
        if self.lidar_driver is not None:
            try:
                self.lidar_driver.cerrar()
            except Exception:
                pass
        for hilo in (self._hilo_camara, self._hilo_lidar):
            if hilo is not None and hilo.is_alive():
                hilo.join(timeout=1.5)
        if self.panel is not None:
            try:
                self.panel.detener()
            except Exception:
                pass
        if self.registro is not None:
            self.registro.cerrar()
        if self._motivo_fin:
            print(f"[i] Fin: {self._motivo_fin}")


def _argumentos(argv=None):
    parser = argparse.ArgumentParser(description="Ronda de obstaculos (Pi 5)")
    parser.add_argument("--config", default=None, help="ruta al JSON de configuracion")
    parser.add_argument(
        "--validar-config",
        action="store_true",
        help="valida y sale, sin abrir ningun puerto",
    )
    parser.add_argument(
        "--arranque-inmediato",
        action="store_true",
        help="no esperar el boton GP21 (para lanzar por SSH)",
    )
    parser.add_argument(
        "--sin-parqueo",
        action="store_true",
        help="terminar al completar las esquinas, sin entrar en la bahia",
    )
    parser.add_argument(
        "--panel-web",
        nargs="?",
        type=int,
        const=8080,
        default=None,
        metavar="PUERTO",
        help="sirve el panel en vivo (por defecto en el 8080)",
    )
    parser.add_argument(
        "--simulacro",
        action="store_true",
        help=(
            "monta la ronda entera sobre los sensores reales pero NO envia"
            " ninguna consigna al motor: el robot no se mueve"
        ),
    )
    parser.add_argument(
        "--solo-parqueo",
        action="store_true",
        help="entrar en la bahia en cuanto haya sentido, sin recorrer la pista",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _argumentos(argv)
    if args.sin_parqueo and args.solo_parqueo:
        print("[-] --sin-parqueo y --solo-parqueo son excluyentes")
        return 2

    try:
        config = cargar_configuracion(args.config)
    except ErrorConfiguracion as exc:
        print(f"[-] Configuracion invalida: {exc}")
        return 2

    if args.sin_parqueo:
        config["control"]["parking_enabled"] = False
    if args.solo_parqueo:
        config["control"]["corners_before_parking"] = 0
        if str(config["control"].get("turn_direction", "AUTO")).upper() == "AUTO":
            print(
                "[-] --solo-parqueo exige turn_direction LEFT o RIGHT: arrancando "
                "dentro de la bahia no se cruza ninguna linea de sentido"
            )
            return 2

    # Sin parqueo no se entra en la bahia, asi que exigir ``parking_ready``
    # solo impide rodar por algo que la corrida no va a hacer.
    omitir = ("parking_ready",) if (args.solo_parqueo or args.sin_parqueo) else ()
    pendientes = calibraciones_pendientes(config, omitir=omitir)
    if args.validar_config:
        print("[OK] Configuracion valida.")
        print(
            "[i] Calibraciones pendientes: "
            + (", ".join(pendientes) if pendientes else "ninguna")
        )
        print(f"[i] motion_enabled={config['runtime'].get('motion_enabled')}")
        return 0

    if args.simulacro:
        # No se comprueban las calibraciones porque no se va a mover nada; lo
        # que se esta probando es precisamente si la tuberia funciona antes de
        # tenerlas todas.
        if pendientes:
            print(f"[i] Simulacro con calibraciones pendientes: {', '.join(pendientes)}")
    else:
        try:
            exigir_listo_para_mover(config, omitir=omitir)
        except ErrorConfiguracion as exc:
            print(f"[-] {exc}")
            return 3

    aplicacion = AplicacionRonda(
        config,
        esperar_boton=not args.arranque_inmediato,
        permitir_parqueo=not args.sin_parqueo,
        simulacro=args.simulacro,
        puerto_panel=args.panel_web,
    )

    def _al_sigint(_signum, _frame):
        aplicacion.detener("interrumpido")

    signal.signal(signal.SIGINT, _al_sigint)
    verificado = aplicacion.ejecutar()
    return 0 if verificado else 1


if __name__ == "__main__":
    raise SystemExit(main())
