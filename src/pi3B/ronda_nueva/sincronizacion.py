"""Buzones temporales pequenos; nunca se acumulan frames de camara."""

import threading
from collections import deque
from typing import Optional

from .modelos import PaqueteVision


class BuzonVision:
    """Conserva unos pocos resultados, no las imagenes completas."""

    def __init__(self, capacidad: int = 4):
        self._datos = deque(maxlen=max(2, int(capacidad)))
        self._lock = threading.Lock()

    def publicar(self, paquete: PaqueteVision) -> None:
        with self._lock:
            # Un timestamp solo puede aportar un voto de color. Sustituir el
            # paquete evita confirmar un poste al reprocesar el mismo frame.
            if self._datos and self._datos[-1].timestamp == paquete.timestamp:
                self._datos[-1] = paquete
            else:
                self._datos.append(paquete)

    def mas_cercano(self, timestamp: float, edad_maxima_s: float) -> Optional[PaqueteVision]:
        with self._lock:
            if not self._datos:
                return None
            mejor = min(self._datos, key=lambda p: abs(p.timestamp - timestamp))
        if abs(mejor.timestamp - timestamp) > edad_maxima_s:
            return None
        return mejor

    def edad_ultimo(self, ahora: float) -> float:
        with self._lock:
            if not self._datos:
                return float("inf")
            return max(0.0, ahora - self._datos[-1].timestamp)


class BuzonUltimo:
    def __init__(self):
        self._valor = None
        self._timestamp = 0.0
        self._lock = threading.Lock()

    def publicar(self, valor, timestamp: float) -> None:
        with self._lock:
            self._valor = valor
            self._timestamp = timestamp

    def leer(self):
        with self._lock:
            return self._valor, self._timestamp

