"""Captura estatica y no destructiva de camara/LiDAR para calibracion.

Esta utilidad NO importa ni abre ``EnlacePicoNuevo`` y por tanto no puede
enviar consignas al motor o al servo. Cada ejecucion exige una ruta que no
exista y crea dentro el formato consumido por :mod:`replay_captura`.

La IMU se registra deliberadamente como cero sintetico: basta para inspeccion
estatica y deja una marca explicita en ``meta.json``. No autoriza a marcar como
lista la calibracion temporal en movimiento.
"""

import argparse
import csv
import json
import queue
import signal
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2

from .config import cargar_configuracion
from .hardware import FuenteCamara


EventoEscritura = Tuple[str, float, Any]


def _driver_lidar():
    try:
        from ..comun.lidar_driver import LidarDriver
    except (ImportError, ValueError):
        from comun.lidar_driver import LidarDriver
    return LidarDriver


def crear_directorio_captura(destino: Path) -> Path:
    """Crea una captura nueva; se niega a reutilizar cualquier ruta."""

    destino = Path(destino).expanduser().resolve()
    destino.mkdir(parents=True, exist_ok=False)
    (destino / "frames").mkdir(exist_ok=False)
    return destino


class CapturadorEstatico:
    def __init__(self, config: Dict[str, Any], destino: Path, usar_lidar: bool):
        self.config = config
        self.destino = destino
        self.usar_lidar = bool(usar_lidar)
        self.seguir = threading.Event()
        self.seguir.set()
        self.cola: "queue.Queue[Optional[EventoEscritura]]" = queue.Queue(maxsize=96)
        self.inicio = time.monotonic()
        self.frames = 0
        self.barridos = 0
        self.imu = 0
        self.descartados = {"frame": 0, "lidar": 0, "imu": 0}
        self.error_escritura: Optional[str] = None
        self.fuente = FuenteCamara(config["camera"])
        self.lidar = None
        self.hilos = []

    def _relativo(self, timestamp: Optional[float] = None) -> float:
        actual = time.monotonic() if timestamp is None else float(timestamp)
        return max(0.0, actual - self.inicio)

    def _publicar(self, tipo: str, timestamp: float, dato: Any) -> None:
        try:
            self.cola.put_nowait((tipo, timestamp, dato))
        except queue.Full:
            self.descartados[tipo] += 1

    def _al_frame(self, frame, timestamp: float) -> None:
        # ``imencode`` desacopla el buffer reutilizable de Picamera2 y reduce
        # mucho la RAM retenida por la cola en una Pi 3B.
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if ok:
            self._publicar("frame", self._relativo(timestamp), jpg.tobytes())
        else:
            self.descartados["frame"] += 1

    def _al_barrido(self, scan) -> None:
        self._publicar("lidar", self._relativo(), list(scan))

    def _escritor(self) -> None:
        try:
            with (self.destino / "imu.csv").open(
                "w", newline="", encoding="utf-8"
            ) as archivo_imu, (self.destino / "lidar.jsonl").open(
                "w", encoding="utf-8"
            ) as archivo_lidar:
                csv_imu = csv.writer(archivo_imu)
                csv_imu.writerow(("t", "valor_crudo_imu"))
                while True:
                    evento = self.cola.get()
                    if evento is None:
                        break
                    tipo, timestamp, dato = evento
                    if tipo == "frame":
                        nombre = "{:.6f}.jpg".format(timestamp)
                        (self.destino / "frames" / nombre).write_bytes(dato)
                        self.frames += 1
                    elif tipo == "lidar":
                        archivo_lidar.write(
                            json.dumps(
                                {"t": round(timestamp, 6), "scan": dato},
                                separators=(",", ":"),
                            )
                            + "\n"
                        )
                        self.barridos += 1
                    elif tipo == "imu":
                        csv_imu.writerow(("{:.6f}".format(timestamp), "0.0"))
                        self.imu += 1
        except Exception as exc:  # la captura principal lo reporta y falla
            self.error_escritura = str(exc)
            self.seguir.clear()

    def ejecutar(self, duracion_s: float) -> bool:
        hilo_escritor = threading.Thread(
            target=self._escritor, name="escritor-calibracion", daemon=True
        )
        hilo_escritor.start()
        self.hilos.append(hilo_escritor)

        hilo_camara = threading.Thread(
            target=self.fuente.bucle,
            args=(self.seguir.is_set, self._al_frame),
            name="camara-calibracion",
            daemon=True,
        )
        hilo_camara.start()
        self.hilos.append(hilo_camara)

        if self.usar_lidar:
            hardware = self.config["hardware"]
            self.lidar = _driver_lidar()(
                hardware["lidar_port"], int(hardware["lidar_baudrate"])
            )
            hilo_lidar = threading.Thread(
                target=self.lidar.hilo_lectura,
                args=(self.seguir.is_set, self._al_barrido),
                name="lidar-calibracion",
                daemon=True,
            )
            hilo_lidar.start()
            self.hilos.append(hilo_lidar)

        fin = self.inicio + float(duracion_s)
        # Primera muestra causal disponible desde t=0 para el replay.
        self._publicar("imu", 0.0, 0.0)
        proxima_imu = self.inicio
        while self.seguir.is_set() and time.monotonic() < fin:
            ahora = time.monotonic()
            if ahora >= proxima_imu:
                self._publicar("imu", self._relativo(ahora), 0.0)
                proxima_imu = ahora + 0.05
            if self.fuente.ultimo_error:
                self.seguir.clear()
                break
            time.sleep(0.01)

        self.seguir.clear()
        if self.lidar is not None:
            self.lidar.cerrar()
        for hilo in self.hilos[1:]:
            hilo.join(timeout=3.0)
        self.cola.put(None)
        hilo_escritor.join(timeout=8.0)

        duracion_real = self._relativo()
        meta = {
            "formato": "ronda_nueva_captura_v1",
            "duracion_pedida_s": float(duracion_s),
            "duracion_real_s": round(duracion_real, 3),
            "barridos_lidar": self.barridos,
            "lecturas_imu": self.imu,
            "frames_camara": self.frames,
            "imu_source": "synthetic_zero_no_pico",
            "pico_abierto": False,
            "movimiento_enviado": False,
            "lidar_habilitado": self.usar_lidar,
            "descartados_por_cola": self.descartados,
            "camera": {
                clave: self.config["camera"].get(clave)
                for clave in (
                    "width",
                    "height",
                    "fps",
                    "picamera_format",
                    "raw_sensor_size",
                    "rotation_deg",
                    "hfov_deg",
                    "principal_x_px",
                )
            },
            "advertencia_camara": self.fuente.ultima_advertencia,
            "error_camara": self.fuente.ultimo_error,
            "error_escritura": self.error_escritura,
        }
        (self.destino / "meta.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        return (
            self.error_escritura is None
            and self.fuente.ultimo_error is None
            and self.frames > 0
            and (not self.usar_lidar or self.barridos > 0)
        )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Captura estatica sin abrir la Pico ni mover el robot"
    )
    parser.add_argument("destino", type=Path, help="ruta nueva que no debe existir")
    parser.add_argument("--duracion", type=float, default=30.0)
    parser.add_argument("--config", default=None)
    parser.add_argument(
        "--solo-camara",
        action="store_true",
        help="no abre ni gira el LiDAR; genera lidar.jsonl vacio",
    )
    args = parser.parse_args(argv)
    if not 1.0 <= args.duracion <= 180.0:
        parser.error("--duracion debe estar entre 1 y 180 segundos")

    config = cargar_configuracion(args.config)
    try:
        destino = crear_directorio_captura(args.destino)
    except FileExistsError:
        print("[-] El destino ya existe; no se escribio nada: {}".format(args.destino))
        return 2

    capturador = CapturadorEstatico(config, destino, usar_lidar=not args.solo_camara)

    def detener(_senal, _frame):
        capturador.seguir.clear()

    signal.signal(signal.SIGINT, detener)
    signal.signal(signal.SIGTERM, detener)
    print("[SEGURO] Pico no abierto; motores/servo no reciben consignas.")
    print("[*] Captura nueva: {}".format(destino))
    correcto = capturador.ejecutar(args.duracion)
    print(
        "[*] frames={} barridos={} imu_sintetica={} descartados={}".format(
            capturador.frames,
            capturador.barridos,
            capturador.imu,
            capturador.descartados,
        )
    )
    if not correcto:
        print("[-] Captura incompleta; revisa meta.json")
        return 1
    print("[+] Captura completa; ningun comando de movimiento fue enviado.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
