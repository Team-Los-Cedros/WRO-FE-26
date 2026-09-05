"""Buzones temporales pequenos; nunca se acumulan frames de camara."""

import threading
from collections import deque
from typing import Optional, Sequence

from .modelos import BarridoLidar, MuestraLidar, PaqueteVision


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


class BuzonBarridosLidar:
    """Casilla de un solo hueco entre el hilo del LiDAR y el de control.

    Deliberadamente NO es una cola. Si el ciclo de decision tarda mas que un
    barrido (una pausa del recolector de basura, un frame de vision lento), la
    alternativa seria encolar y acabar controlando con datos viejos: el robot
    esquiva donde el pilar estaba, no donde esta. Aqui el barrido nuevo pisa al
    que no llego a consumirse y el contador ``descartados`` deja constancia en
    la telemetria de cuantas veces paso.
    """

    def __init__(self):
        self._condicion = threading.Condition()
        self._barrido: Optional[BarridoLidar] = None
        self._descartados = 0
        self._recibidos = 0

    def publicar(self, muestras: Sequence[MuestraLidar], timestamp: float) -> None:
        with self._condicion:
            if self._barrido is not None:
                self._descartados += 1
            self._recibidos += 1
            self._barrido = BarridoLidar(timestamp=float(timestamp), muestras=muestras)
            self._condicion.notify()

    def tomar(self, timeout_s: Optional[float] = None) -> Optional[BarridoLidar]:
        """Devuelve el barrido mas reciente y deja la casilla vacia."""

        with self._condicion:
            if self._barrido is None and timeout_s:
                self._condicion.wait(timeout_s)
            barrido = self._barrido
            self._barrido = None
            return barrido

    @property
    def descartados(self) -> int:
        with self._condicion:
            return self._descartados

    @property
    def recibidos(self) -> int:
        with self._condicion:
            return self._recibidos


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

