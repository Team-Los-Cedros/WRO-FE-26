"""Une lo que ve la camara con lo que ve el LiDAR, en milimetros.

POR QUE ES TAN CORTO AHORA
La version anterior necesitaba 389 lineas: predecia el bearing de cada objeto
LiDAR en la camara, abria una puerta angular de 10 grados, resolvia una
asignacion voraz y votaba color con decaimiento.  Todo eso existia porque la
camara solo daba una DIRECCION y habia que adivinar la distancia.

Con la homografia del suelo la camara da (x, y) en milimetros, o sea el mismo
espacio que el LiDAR.  Asociar pasa a ser "el vecino mas cercano dentro de una
puerta en milimetros", que es una linea de codigo y no tiene el punto ciego de
la puerta angular: a 250 mm, 10 grados son 44 mm y a 1500 mm son 260, asi que
la puerta angular era demasiado estrecha de cerca y demasiado ancha de lejos.

QUIEN MANDA EN CADA COSA
* Color: siempre la camara.  El LiDAR no tiene color.
* Posicion: el LiDAR cuando lo ve, porque mide de verdad; la camara cuando no.
  El LiDAR se queda ciego con los pilares lejanos (su plano les pasa por
  encima) y la camara con los muy cercanos (se salen del cuadro por abajo):
  se tapan los agujeros el uno al otro.

EL SESGO CONOCIDO
El LiDAR ve la CARA frontal del pilar y la camara la silueta completa, asi que
sus centroides no coinciden a menos de 600 mm.  Es un sesgo sistematico, no
ruido: se corrige empujando el centroide LiDAR media anchura de pilar hacia
adelante en vez de ensanchar la puerta hasta que todo empareje con todo.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .modelos import DeteccionPilar, ObjetoLidar


def _distancia(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class FusionPilares:
    """Combina detecciones visuales y objetos LiDAR del mismo instante."""

    def __init__(self, config: Dict[str, Any]):
        fusion = config.get("fusion", {})
        pista = config.get("track", {})
        self.puerta_mm = float(fusion.get("gate_mm", 220.0))
        self.edad_max_s = float(fusion.get("max_camera_lidar_age_s", 0.15))
        self.preferir_lidar_hasta_mm = float(
            fusion.get("prefer_lidar_below_mm", 1400.0)
        )
        self.confianza_solo_lidar = float(fusion.get("lidar_only_confidence", 0.30))
        self.medio_pilar_mm = float(pista.get("pillar_width_mm", 100.0)) / 2.0
        self.aceptar_solo_lidar = bool(fusion.get("accept_lidar_only", True))

    def _centro_corregido(self, objeto: ObjetoLidar) -> Tuple[float, float]:
        """Empuja el centroide LiDAR al centro real del pilar.

        El barrido solo toca la cara que mira al robot, asi que el centroide
        del cluster queda media anchura mas cerca de lo que esta el centro.
        """

        if objeto.y_mm <= 0.0:
            # Detras del LiDAR la correccion no significa nada: empujar "mas
            # lejos por el rayo" aleja el objeto hacia atras, y en el robot eso
            # convertia un artefacto a -52 mm en uno a -88.
            return objeto.x_mm, objeto.y_mm
        distancia = max(1.0, objeto.distancia_mm)
        factor = (distancia + self.medio_pilar_mm) / distancia
        return objeto.x_mm * factor, objeto.y_mm * factor

    def asociar(
        self,
        visuales: Sequence[DeteccionPilar],
        objetos: Sequence[ObjetoLidar],
        timestamp: Optional[float] = None,
    ) -> Tuple[DeteccionPilar, ...]:
        """Devuelve los pilares del instante, con la mejor posicion y color.

        La asignacion es voraz sobre las parejas ordenadas por distancia: con
        cuatro objetos como maximo por lado, el optimo hungaro no compra nada
        y cuesta explicarlo cuando algo va mal en pista.
        """

        instante = float(timestamp if timestamp is not None else 0.0)
        centros = [self._centro_corregido(objeto) for objeto in objetos]

        parejas: List[Tuple[float, int, int]] = []
        for i, visual in enumerate(visuales):
            for j, centro in enumerate(centros):
                distancia = _distancia((visual.x_mm, visual.y_mm), centro)
                if distancia <= self.puerta_mm:
                    parejas.append((distancia, i, j))
        parejas.sort()

        visual_usada = set()
        objeto_usado = set()
        salida: List[DeteccionPilar] = []

        for distancia, i, j in parejas:
            if i in visual_usada or j in objeto_usado:
                continue
            visual_usada.add(i)
            objeto_usado.add(j)
            visual = visuales[i]
            objeto = objetos[j]
            cerca = objeto.distancia_mm <= self.preferir_lidar_hasta_mm
            x_mm, y_mm = centros[j] if cerca else (visual.x_mm, visual.y_mm)
            acuerdo = 1.0 - min(1.0, distancia / max(self.puerta_mm, 1.0))
            salida.append(
                DeteccionPilar(
                    timestamp=instante or visual.timestamp,
                    color=visual.color,
                    x_mm=float(x_mm),
                    y_mm=float(y_mm),
                    fuente="FUSION",
                    confianza=min(0.99, 0.6 + 0.4 * acuerdo),
                    ancho_px=visual.ancho_px,
                    alto_px=visual.alto_px,
                    distancia_por_altura_mm=visual.distancia_por_altura_mm,
                    bbox=visual.bbox,
                )
            )

        for i, visual in enumerate(visuales):
            if i in visual_usada:
                continue
            salida.append(
                DeteccionPilar(
                    timestamp=instante or visual.timestamp,
                    color=visual.color,
                    x_mm=visual.x_mm,
                    y_mm=visual.y_mm,
                    fuente=visual.fuente,
                    confianza=visual.confianza,
                    ancho_px=visual.ancho_px,
                    alto_px=visual.alto_px,
                    distancia_por_altura_mm=visual.distancia_por_altura_mm,
                    bbox=visual.bbox,
                )
            )

        if self.aceptar_solo_lidar:
            for j, objeto in enumerate(objetos):
                if j in objeto_usado:
                    continue
                # Sin color no se puede decidir el lado, pero SI se puede
                # frenar y no arrollarlo.  Se emite con color None y confianza
                # baja para que el planificador lo trate como estorbo.
                x_mm, y_mm = centros[j]
                salida.append(
                    DeteccionPilar(
                        timestamp=instante or objeto.timestamp,
                        color="",
                        x_mm=float(x_mm),
                        y_mm=float(y_mm),
                        fuente="LIDAR",
                        confianza=self.confianza_solo_lidar,
                    )
                )

        salida.sort(key=lambda pilar: pilar.y_mm)
        return tuple(salida)
