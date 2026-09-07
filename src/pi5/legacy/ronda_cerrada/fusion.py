"""Fusion ligera, temporal y uno-a-uno de camara y LiDAR.

No contiene imports de hardware. La API principal es::

    FusionLigera(config).actualizar(
        objetos_lidar, paquete_vision, heading_deg, velocidad_pwm, timestamp
    )

``heading_deg`` sigue la convencion del IMU existente: positivo al girar el
robot a la izquierda. ``velocidad_pwm`` positiva significa avance. La
prediccion usa el cambio de heading y ``fusion.mm_s_per_pwm``; cada medicion
LiDAR corrige despues esa prediccion.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

try:  # Permite paquete normal y despliegue plano en la Pi.
    from .modelos import DeteccionVisual, ObjetoLidar, PaqueteVision, TrackObstaculo
except ImportError:  # pragma: no cover - compatibilidad con despliegue plano
    from modelos import DeteccionVisual, ObjetoLidar, PaqueteVision, TrackObstaculo


def _limitar(valor: float, minimo: float, maximo: float) -> float:
    return max(minimo, min(maximo, valor))


def _normalizar_angulo(angulo_deg: float) -> float:
    return (float(angulo_deg) + 180.0) % 360.0 - 180.0


def _clave_timestamp(timestamp: float) -> int:
    """Clave estable en microsegundos para reconocer paquetes repetidos."""

    return int(round(float(timestamp) * 1_000_000.0))


@dataclass(frozen=True)
class AsociacionVisualLidar:
    """Pareja uno-a-uno aceptada por el gate angular."""

    deteccion: DeteccionVisual
    objeto: ObjetoLidar
    residuo_deg: float


@dataclass
class _TrackInterno:
    track_id: int
    timestamp: float
    x_mm: float
    y_mm: float
    timestamps_lidar: Set[int] = field(default_factory=set)
    votos_color: Dict[str, float] = field(default_factory=dict)
    frames_color: Dict[str, Set[int]] = field(default_factory=dict)
    frames_vistos: Set[int] = field(default_factory=set)
    ultimo_frame_color: float = -math.inf


class FusionLigera:
    """Asocia observaciones y mantiene una cantidad acotada de tracks.

    La asociacion camara-LiDAR calcula primero el bearing que *deberia* ver la
    camara desde su traslacion y yaw respecto al LiDAR. Todos los pares dentro
    del gate se ordenan por residuo angular y se asignan greedy, sin permitir
    que dos blobs voten por el mismo objeto ni viceversa.

    Los impactos LiDAR y los votos de color solo cuentan una vez por timestamp.
    Los votos decaen al llegar cuadros nuevos, por lo que varios cuadros de un
    color diferente pueden corregir una etiqueta anterior.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        camara = config["camera"]
        fusion = config["fusion"]

        self.camara_adelante_mm = float(camara["forward_from_lidar_mm"])
        self.camara_derecha_mm = float(camara["right_from_lidar_mm"])
        self.camara_yaw_deg = float(camara["yaw_from_lidar_deg"])
        self.max_edad_camara_lidar_s = float(fusion["max_camera_lidar_age_s"])
        self.gate_camara_lidar_deg = float(fusion["camera_lidar_gate_deg"])
        self.gate_track_mm = float(fusion["track_gate_mm"])
        self.ttl_track_s = float(fusion["track_ttl_s"])
        self.max_tracks = max(1, int(fusion["track_max_count"]))
        self.confirmar_lidar = max(1, int(fusion["confirm_lidar_hits"]))
        self.confirmar_color = max(1, int(fusion["confirm_color_hits"]))
        self.min_confianza_color = float(fusion["min_color_confidence"])
        self.decaimiento_color = _limitar(float(fusion["color_decay"]), 0.0, 1.0)
        self.mm_s_por_pwm = float(fusion["mm_s_per_pwm"])

        self.reiniciar()

    def reiniciar(self) -> None:
        """Elimina tracks e historial temporal sin cambiar calibraciones."""

        self._tracks: List[_TrackInterno] = []
        self._siguiente_id = 1
        self._ultimo_timestamp: Optional[float] = None
        self._ultimo_heading: Optional[float] = None

    def bearing_camara_predicho(self, objeto: ObjetoLidar) -> float:
        """Bearing del objeto visto desde la extrinseca de la camara."""

        x_camara = float(objeto.x_mm) - self.camara_derecha_mm
        y_camara = float(objeto.y_mm) - self.camara_adelante_mm
        bearing_lidar = math.degrees(math.atan2(x_camara, y_camara))
        return _normalizar_angulo(bearing_lidar - self.camara_yaw_deg)

    def _indices_asociados(
        self,
        objetos: Sequence[ObjetoLidar],
        paquete_vision: Optional[PaqueteVision],
    ) -> List[Tuple[int, int, float]]:
        if paquete_vision is None or not objetos or not paquete_vision.detecciones:
            return []

        pares: List[Tuple[float, int, int]] = []
        for indice_visual, deteccion in enumerate(paquete_vision.detecciones):
            # FuenteCamara publica el instante de captura ya compensado. Los
            # replays tambien deben entregar timestamps de captura; restar la
            # latencia aqui por segunda vez desplazaria todo el apareo.
            tiempo_camara = float(deteccion.timestamp)
            for indice_lidar, objeto in enumerate(objetos):
                if abs(tiempo_camara - float(objeto.timestamp)) > self.max_edad_camara_lidar_s:
                    continue
                residuo = abs(
                    _normalizar_angulo(
                        float(deteccion.bearing_deg)
                        - self.bearing_camara_predicho(objeto)
                    )
                )
                if residuo <= self.gate_camara_lidar_deg:
                    pares.append((residuo, indice_visual, indice_lidar))

        pares.sort(key=lambda par: par[0])
        visuales_usadas: Set[int] = set()
        lidar_usados: Set[int] = set()
        aceptadas: List[Tuple[int, int, float]] = []
        for residuo, indice_visual, indice_lidar in pares:
            if indice_visual in visuales_usadas or indice_lidar in lidar_usados:
                continue
            visuales_usadas.add(indice_visual)
            lidar_usados.add(indice_lidar)
            aceptadas.append((indice_visual, indice_lidar, residuo))
        return aceptadas

    def asociar(
        self,
        objetos: Sequence[ObjetoLidar],
        paquete_vision: Optional[PaqueteVision],
    ) -> Tuple[AsociacionVisualLidar, ...]:
        """Devuelve asociaciones puras, sin modificar los tracks."""

        if paquete_vision is None:
            return ()
        return tuple(
            AsociacionVisualLidar(
                deteccion=paquete_vision.detecciones[indice_visual],
                objeto=objetos[indice_lidar],
                residuo_deg=residuo,
            )
            for indice_visual, indice_lidar, residuo in self._indices_asociados(
                objetos, paquete_vision
            )
        )

    def _predecir_movimiento(
        self, timestamp: float, heading_deg: float, velocidad_pwm: float
    ) -> None:
        if self._ultimo_timestamp is None or self._ultimo_heading is None:
            self._ultimo_timestamp = timestamp
            self._ultimo_heading = float(heading_deg)
            return

        dt = max(0.0, float(timestamp) - self._ultimo_timestamp)
        dt = min(dt, self.ttl_track_s)
        delta_heading = _normalizar_angulo(float(heading_deg) - self._ultimo_heading)
        delta = math.radians(delta_heading)
        coseno = math.cos(-delta)
        seno = math.sin(-delta)
        avance_mm = float(velocidad_pwm) * self.mm_s_por_pwm * dt

        for track in self._tracks:
            x_anterior, y_anterior = track.x_mm, track.y_mm
            track.x_mm = x_anterior * coseno - y_anterior * seno
            track.y_mm = x_anterior * seno + y_anterior * coseno - avance_mm

        self._ultimo_timestamp = timestamp
        self._ultimo_heading = float(heading_deg)

    def _eliminar_caducados(self, timestamp: float) -> None:
        self._tracks = [
            track
            for track in self._tracks
            if -1e-6 <= timestamp - track.timestamp <= self.ttl_track_s
        ]

    def _asociar_tracks(
        self, objetos: Sequence[ObjetoLidar]
    ) -> Tuple[Dict[int, int], Set[int]]:
        pares: List[Tuple[float, int, int]] = []
        for indice_track, track in enumerate(self._tracks):
            for indice_objeto, objeto in enumerate(objetos):
                distancia = math.hypot(
                    float(objeto.x_mm) - track.x_mm,
                    float(objeto.y_mm) - track.y_mm,
                )
                if distancia <= self.gate_track_mm:
                    pares.append((distancia, indice_track, indice_objeto))
        pares.sort(key=lambda par: par[0])

        tracks_usados: Set[int] = set()
        objetos_usados: Set[int] = set()
        objeto_a_track: Dict[int, int] = {}
        for _, indice_track, indice_objeto in pares:
            if indice_track in tracks_usados or indice_objeto in objetos_usados:
                continue
            tracks_usados.add(indice_track)
            objetos_usados.add(indice_objeto)
            objeto_a_track[indice_objeto] = indice_track
        return objeto_a_track, objetos_usados

    def _actualizar_lidar(self, track: _TrackInterno, objeto: ObjetoLidar) -> None:
        clave = _clave_timestamp(objeto.timestamp)
        if clave in track.timestamps_lidar:
            return
        # Un barrido fuera de orden no debe confirmar ni arrastrar un track.
        if float(objeto.timestamp) + 1e-9 < track.timestamp:
            return
        track.timestamps_lidar.add(clave)
        track.timestamp = float(objeto.timestamp)
        track.x_mm = float(objeto.x_mm)
        track.y_mm = float(objeto.y_mm)

    def _votar_color(self, track: _TrackInterno, deteccion: DeteccionVisual) -> None:
        timestamp = float(deteccion.timestamp)
        clave = _clave_timestamp(timestamp)
        if clave in track.frames_vistos or timestamp + 1e-9 < track.ultimo_frame_color:
            return
        track.frames_vistos.add(clave)
        track.ultimo_frame_color = timestamp

        for color in tuple(track.votos_color):
            track.votos_color[color] *= self.decaimiento_color
        color = str(deteccion.color).upper()
        peso = _limitar(float(deteccion.confianza), 0.0, 1.0)
        track.votos_color[color] = track.votos_color.get(color, 0.0) + peso
        track.frames_color.setdefault(color, set()).add(clave)

    def _nuevo_track(self, objeto: ObjetoLidar) -> _TrackInterno:
        track = _TrackInterno(
            track_id=self._siguiente_id,
            timestamp=float(objeto.timestamp),
            x_mm=float(objeto.x_mm),
            y_mm=float(objeto.y_mm),
        )
        track.timestamps_lidar.add(_clave_timestamp(objeto.timestamp))
        self._siguiente_id += 1
        self._tracks.append(track)
        return track

    @staticmethod
    def _color_track(track: _TrackInterno) -> Tuple[Optional[str], float, int]:
        if not track.votos_color:
            return None, 0.0, 0
        color = max(track.votos_color, key=track.votos_color.get)
        total = sum(max(0.0, voto) for voto in track.votos_color.values())
        confianza = track.votos_color[color] / total if total > 1e-12 else 0.0
        impactos = len(track.frames_color.get(color, ()))
        return color, _limitar(confianza, 0.0, 1.0), impactos

    def _esta_confirmado(self, track: _TrackInterno) -> bool:
        _, confianza, impactos_color = self._color_track(track)
        return (
            len(track.timestamps_lidar) >= self.confirmar_lidar
            and impactos_color >= self.confirmar_color
            and confianza >= self.min_confianza_color
        )

    def _acotar_tracks(self) -> None:
        if len(self._tracks) <= self.max_tracks:
            return
        # Conserva primero tracks confirmados, con mas impactos, mas recientes
        # y finalmente mas cercanos.
        self._tracks.sort(
            key=lambda track: (
                self._esta_confirmado(track),
                len(track.timestamps_lidar),
                track.timestamp,
                -math.hypot(track.x_mm, track.y_mm),
            ),
            reverse=True,
        )
        del self._tracks[self.max_tracks :]

    def _exportar(self, timestamp: float) -> Tuple[TrackObstaculo, ...]:
        salida: List[TrackObstaculo] = []
        for track in self._tracks:
            color, confianza_color, impactos_color = self._color_track(track)
            distancia = math.hypot(track.x_mm, track.y_mm)
            salida.append(
                TrackObstaculo(
                    track_id=track.track_id,
                    timestamp=track.timestamp,
                    x_mm=track.x_mm,
                    y_mm=track.y_mm,
                    distancia_mm=distancia,
                    bearing_deg=math.degrees(math.atan2(track.x_mm, track.y_mm)),
                    color=color,
                    confianza_color=confianza_color,
                    impactos_lidar=len(track.timestamps_lidar),
                    impactos_color=impactos_color,
                    edad_s=max(0.0, float(timestamp) - track.timestamp),
                    confirmado=self._esta_confirmado(track),
                )
            )
        salida.sort(key=lambda track: (not track.confirmado, track.distancia_mm, track.track_id))
        return tuple(salida)

    def tracks(self, timestamp: Optional[float] = None) -> Tuple[TrackObstaculo, ...]:
        """Instantanea inmutable de los tracks actuales."""

        if timestamp is None:
            timestamp = self._ultimo_timestamp
        if timestamp is None:
            timestamp = time.monotonic()
        self._eliminar_caducados(float(timestamp))
        return self._exportar(float(timestamp))

    def actualizar(
        self,
        objetos: Sequence[ObjetoLidar],
        paquete_vision: Optional[PaqueteVision],
        heading_deg: float,
        velocidad_pwm: float,
        timestamp: Optional[float] = None,
    ) -> Tuple[TrackObstaculo, ...]:
        """Predice, asocia y devuelve una instantanea de los tracks.

        Los objetos de un mismo barrido deben compartir timestamp. Si
        ``timestamp`` se omite se toma el instante mas nuevo de las entradas;
        solo se usa ``time.monotonic`` cuando ambas estan vacias.
        """

        candidatos_tiempo = [float(obj.timestamp) for obj in objetos]
        if paquete_vision is not None:
            candidatos_tiempo.append(float(paquete_vision.timestamp))
        ahora = (
            float(timestamp)
            if timestamp is not None
            else max(candidatos_tiempo) if candidatos_tiempo else time.monotonic()
        )

        # No se permite que un paquete fuera de orden rebobine el estado.
        if self._ultimo_timestamp is not None and ahora + 1e-9 < self._ultimo_timestamp:
            return self._exportar(self._ultimo_timestamp)

        self._predecir_movimiento(ahora, heading_deg, velocidad_pwm)
        self._eliminar_caducados(ahora)

        # Rechaza barridos demasiado antiguos o futuros respecto al ciclo.
        objetos_frescos = tuple(
            objeto
            for objeto in objetos
            if abs(ahora - float(objeto.timestamp)) <= self.ttl_track_s
        )
        asociaciones_visuales = {
            indice_lidar: paquete_vision.detecciones[indice_visual]
            for indice_visual, indice_lidar, _ in self._indices_asociados(
                objetos_frescos, paquete_vision
            )
        } if paquete_vision is not None else {}

        objeto_a_track, objetos_usados = self._asociar_tracks(objetos_frescos)
        for indice_objeto, objeto in enumerate(objetos_frescos):
            if indice_objeto in objetos_usados:
                track = self._tracks[objeto_a_track[indice_objeto]]
                self._actualizar_lidar(track, objeto)
            else:
                track = self._nuevo_track(objeto)
            deteccion = asociaciones_visuales.get(indice_objeto)
            if deteccion is not None:
                self._votar_color(track, deteccion)

        self._acotar_tracks()
        return self._exportar(ahora)
