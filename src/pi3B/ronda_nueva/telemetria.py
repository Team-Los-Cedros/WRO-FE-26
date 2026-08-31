"""CSV asincrono: ninguna escritura o flush ocurre en el callback LiDAR."""

import csv
import queue
import threading
import time
from pathlib import Path
from typing import Dict, Iterable, Optional


class TelemetriaAsincrona:
    def __init__(self, ruta: Path, campos: Iterable[str], capacidad: int = 256):
        self.ruta = Path(ruta)
        self.ruta.parent.mkdir(parents=True, exist_ok=True)
        self._campos = tuple(campos)
        self._cola = queue.Queue(maxsize=max(16, int(capacidad)))
        self._cerrando = threading.Event()
        self.descartadas = 0
        self._hilo = threading.Thread(target=self._escribir, name="log-ronda-nueva", daemon=True)
        self._hilo.start()

    def registrar(self, fila: Dict) -> None:
        try:
            self._cola.put_nowait({k: fila.get(k, "") for k in self._campos})
        except queue.Full:
            self.descartadas += 1

    def _escribir(self) -> None:
        with self.ruta.open("w", newline="", encoding="utf-8") as archivo:
            writer = csv.DictWriter(archivo, fieldnames=self._campos)
            writer.writeheader()
            ultimo_flush = time.monotonic()
            while not self._cerrando.is_set() or not self._cola.empty():
                try:
                    fila = self._cola.get(timeout=0.1)
                    writer.writerow(fila)
                except queue.Empty:
                    pass
                ahora = time.monotonic()
                if ahora - ultimo_flush >= 1.0:
                    archivo.flush()
                    ultimo_flush = ahora
            archivo.flush()

    def cerrar(self, timeout_s: float = 2.0) -> None:
        self._cerrando.set()
        self._hilo.join(timeout=max(0.1, timeout_s))

