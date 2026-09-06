"""Vision de color ligera y reproducible para la ronda nueva.

El modulo no abre camaras ni importa controladores de hardware. Recibe un
``numpy.ndarray`` y devuelve tipos de :mod:`modelos`, por lo que la misma ruta
de codigo sirve para la Raspberry, videos grabados y pruebas sinteticas.

Todas las medidas geometricas se expresan como proporciones de la ROI. Los
unicos tamanos en pixeles son los kernels morfologicos configurables, que son
operaciones locales y no representan dimensiones fisicas de un pilar.
"""

import math
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:  # Permite tanto imports de paquete como ejecucion desde la carpeta.
    from .modelos import DeteccionVisual, PaqueteVision
except ImportError:  # pragma: no cover - compatibilidad con despliegue plano
    from modelos import DeteccionVisual, PaqueteVision


RangoHSV = Tuple[np.ndarray, np.ndarray]


def _limitar(valor: float, minimo: float, maximo: float) -> float:
    return max(minimo, min(maximo, valor))


class VisionLigera:
    """Extrae todos los pilares rojos y verdes geometricamente validos.

    La imagen de entrada debe respetar ``camera.array_color_order``. Este dato
    esta separado de ``camera.picamera_format`` porque Picamera2 entrega el
    ``RGB888`` usado por el robot como un array BGR para OpenCV. ``rotation_deg``
    se interpreta en sentido horario y el espejo horizontal se aplica despues
    de la rotacion. El resultado siempre queda en ``camera.width`` x
    ``camera.height`` para que la calibracion de ``c0`` y HFOV tenga una unica
    referencia.
    """

    COLORES = ("ROJO", "VERDE")

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        camara = config["camera"]
        vision = config["vision"]

        self.ancho = int(camara["width"])
        self.alto = int(camara["height"])
        self.orden_color = str(camara.get("array_color_order", "BGR")).upper()
        self.rotacion = int(camara.get("rotation_deg", 0))
        self.espejo_horizontal = bool(camara.get("flip_horizontal", False))
        self.roi_top_ratio = float(camara["roi_top_ratio"])
        self.roi_bottom_ratio = float(camara["roi_bottom_ratio"])
        self.semilla_suelo = tuple(camara.get("floor_seed_norm", (0.5, 0.82)))
        self.poligonos_robot = tuple(camara.get("robot_mask_polygons_norm", ()))

        self.hfov_deg = float(camara["hfov_deg"])
        self.centro_optico_x = float(camara["principal_x_px"])
        self.focal_px = self.ancho / (
            2.0 * math.tan(math.radians(self.hfov_deg) / 2.0)
        )

        self.rangos_rojo = self._preparar_rangos(vision["red_ranges"])
        self.rangos_verde = self._preparar_rangos(vision["green_ranges"])
        self.rangos_suelo = self._preparar_rangos(vision["floor_ranges"])

        self.min_area_ratio = float(vision["min_area_ratio"])
        self.max_area_ratio = float(vision["max_area_ratio"])
        self.min_height_ratio = float(vision["min_height_ratio"])
        self.min_aspecto = float(vision["min_aspect_height_width"])
        self.min_relleno = float(vision["min_fill_ratio"])
        self.min_soporte = float(vision["min_ground_support"])
        self.peso_soporte = _limitar(
            float(vision.get("ground_support_weight", 0.25)), 0.0, 1.0
        )
        self.max_blobs_por_color = int(vision.get("max_blobs_per_color", 0))

        self.kernel_apertura = self._crear_kernel(vision.get("morph_open_px", 0))
        self.kernel_cierre = self._crear_kernel(vision.get("morph_close_px", 0))

    @staticmethod
    def _preparar_rangos(rangos: Iterable[Sequence[Sequence[int]]]) -> Tuple[RangoHSV, ...]:
        preparados: List[RangoHSV] = []
        for inferior, superior in rangos:
            preparados.append(
                (np.asarray(inferior, dtype=np.uint8), np.asarray(superior, dtype=np.uint8))
            )
        return tuple(preparados)

    @staticmethod
    def _crear_kernel(tamano: Any) -> Optional[np.ndarray]:
        lado = int(tamano)
        if lado <= 1:
            return None
        if lado % 2 == 0:
            lado += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (lado, lado))

    def orientar(self, imagen: np.ndarray) -> np.ndarray:
        """Aplica orientacion configurada y devuelve la resolucion canonica."""

        if imagen is None or imagen.ndim != 3 or imagen.shape[2] != 3:
            raise ValueError("La imagen debe tener forma HxWx3")
        if imagen.dtype != np.uint8:
            imagen = np.clip(imagen, 0, 255).astype(np.uint8)

        if self.rotacion == 90:
            orientada = cv2.rotate(imagen, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotacion == 180:
            orientada = cv2.rotate(imagen, cv2.ROTATE_180)
        elif self.rotacion == 270:
            orientada = cv2.rotate(imagen, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotacion == 0:
            orientada = imagen
        else:
            raise ValueError("rotation_deg debe ser 0, 90, 180 o 270")

        if self.espejo_horizontal:
            orientada = cv2.flip(orientada, 1)
        if orientada.shape[1] != self.ancho or orientada.shape[0] != self.alto:
            orientada = cv2.resize(
                orientada, (self.ancho, self.alto), interpolation=cv2.INTER_AREA
            )
        return orientada

    def bearing_para_x(self, x_px: float) -> float:
        """Convierte una abscisa al bearing estenopeico; derecha es positiva."""

        return math.degrees(
            math.atan2(float(x_px) - self.centro_optico_x, self.focal_px)
        )

    def _a_hsv(self, imagen: np.ndarray) -> np.ndarray:
        if self.orden_color == "RGB":
            return cv2.cvtColor(imagen, cv2.COLOR_RGB2HSV)
        if self.orden_color == "BGR":
            return cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
        raise ValueError("camera.array_color_order debe ser RGB o BGR")

    @staticmethod
    def _mascara_rangos(hsv: np.ndarray, rangos: Tuple[RangoHSV, ...]) -> np.ndarray:
        mascara = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for inferior, superior in rangos:
            cv2.bitwise_or(mascara, cv2.inRange(hsv, inferior, superior), dst=mascara)
        return mascara

    def _morfologia(self, mascara: np.ndarray) -> np.ndarray:
        if self.kernel_apertura is not None:
            mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, self.kernel_apertura)
        if self.kernel_cierre is not None:
            mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, self.kernel_cierre)
        return mascara

    def _aplicar_roi_y_robot(self, mascara: np.ndarray, y0: int, y1: int) -> None:
        mascara[:y0, :] = 0
        mascara[y1:, :] = 0
        for poligono in self.poligonos_robot:
            if len(poligono) < 3:
                continue
            puntos = np.asarray(
                [
                    (
                        int(round(_limitar(float(x), 0.0, 1.0) * (self.ancho - 1))),
                        int(round(_limitar(float(y), 0.0, 1.0) * (self.alto - 1))),
                    )
                    for x, y in poligono
                ],
                dtype=np.int32,
            )
            cv2.fillPoly(mascara, [puntos], 0)

    def _suelo_conectado(self, mascara_suelo: np.ndarray) -> np.ndarray:
        binaria = (mascara_suelo > 0).astype(np.uint8)
        cantidad, etiquetas, estadisticas, _ = cv2.connectedComponentsWithStats(
            binaria, connectivity=8
        )
        if cantidad <= 1:
            return np.zeros_like(mascara_suelo)

        sx = int(round(_limitar(float(self.semilla_suelo[0]), 0.0, 1.0) * (self.ancho - 1)))
        sy = int(round(_limitar(float(self.semilla_suelo[1]), 0.0, 1.0) * (self.alto - 1)))
        etiqueta = int(etiquetas[sy, sx])
        if etiqueta == 0:
            areas = estadisticas[1:, cv2.CC_STAT_AREA]
            etiqueta = int(np.argmax(areas)) + 1
        return np.where(etiquetas == etiqueta, 255, 0).astype(np.uint8)

    def _soporte_suelo(
        self,
        suelo: np.ndarray,
        bbox: Tuple[int, int, int, int],
        y_roi_fin: int,
        alto_roi: int,
    ) -> float:
        x, y, ancho, alto = bbox
        borde_inferior = y + alto
        if borde_inferior >= y_roi_fin:
            return 0.0

        # Banda proporcional al propio blob y a la ROI, nunca una distancia
        # absoluta ligada a 320x240.
        alto_banda = max(alto * 0.35, alto_roi * 0.025)
        y_fin = min(y_roi_fin, int(math.ceil(borde_inferior + alto_banda)))
        margen_x = ancho * 0.20
        x_inicio = max(0, int(math.floor(x - margen_x)))
        x_fin = min(self.ancho, int(math.ceil(x + ancho + margen_x)))
        banda = suelo[borde_inferior:y_fin, x_inicio:x_fin]
        if banda.size == 0:
            return 0.0
        return float(np.count_nonzero(banda)) / float(banda.size)

    def _extraer_color(
        self,
        mascara: np.ndarray,
        color: str,
        timestamp: float,
        suelo: np.ndarray,
        y_roi_fin: int,
        alto_roi: int,
        area_roi: float,
    ) -> List[DeteccionVisual]:
        contornos, _ = cv2.findContours(
            mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        candidatas: List[DeteccionVisual] = []

        for contorno in contornos:
            area = float(cv2.contourArea(contorno))
            area_ratio = area / area_roi
            if not self.min_area_ratio <= area_ratio <= self.max_area_ratio:
                continue

            x, y, ancho, alto = cv2.boundingRect(contorno)
            height_ratio = alto / float(alto_roi)
            aspecto = alto / float(max(1, ancho))
            relleno = area / float(max(1, ancho * alto))
            if height_ratio < self.min_height_ratio:
                continue
            if aspecto < self.min_aspecto or relleno < self.min_relleno:
                continue

            soporte = self._soporte_suelo(
                suelo, (x, y, ancho, alto), y_roi_fin, alto_roi
            )
            if soporte < self.min_soporte:
                continue

            momentos = cv2.moments(contorno)
            if momentos["m00"] > 1e-9:
                cx = float(momentos["m10"] / momentos["m00"])
                cy = float(momentos["m01"] / momentos["m00"])
            else:
                cx = x + ancho / 2.0
                cy = y + alto / 2.0

            area_score = min(1.0, area_ratio / max(self.min_area_ratio * 4.0, 1e-9))
            altura_score = min(
                1.0, height_ratio / max(self.min_height_ratio * 2.0, 1e-9)
            )
            relleno_score = _limitar(
                (relleno - self.min_relleno) / max(1.0 - self.min_relleno, 1e-9),
                0.0,
                1.0,
            )
            geometria = (area_score + altura_score + relleno_score) / 3.0
            confianza = (1.0 - self.peso_soporte) * geometria + self.peso_soporte * soporte

            candidatas.append(
                DeteccionVisual(
                    timestamp=float(timestamp),
                    color=color,
                    bearing_deg=self.bearing_para_x(cx),
                    centro_px=(cx, cy),
                    bbox=(int(x), int(y), int(ancho), int(alto)),
                    area_ratio=area_ratio,
                    bottom_ratio=(y + alto) / float(self.alto),
                    confianza=_limitar(confianza, 0.0, 1.0),
                    soporte_suelo=soporte,
                )
            )

        candidatas.sort(
            key=lambda det: (det.bottom_ratio, det.confianza, det.area_ratio),
            reverse=True,
        )
        if self.max_blobs_por_color > 0:
            return candidatas[: self.max_blobs_por_color]
        return candidatas

    def procesar(
        self, imagen: np.ndarray, timestamp: Optional[float] = None
    ) -> PaqueteVision:
        """Procesa un cuadro y devuelve todas las detecciones validas.

        ``timestamp`` debe representar el instante de captura, no el instante
        posterior al procesamiento. Si se omite se usa ``time.monotonic()``.
        """

        inicio = time.perf_counter()
        instante = time.monotonic() if timestamp is None else float(timestamp)
        orientada = self.orientar(imagen)
        hsv = self._a_hsv(orientada)

        y0 = int(round(self.roi_top_ratio * self.alto))
        y1 = int(round(self.roi_bottom_ratio * self.alto))
        y0 = max(0, min(self.alto - 1, y0))
        y1 = max(y0 + 1, min(self.alto, y1))
        alto_roi = y1 - y0
        area_roi = float(self.ancho * alto_roi)

        suelo = self._morfologia(self._mascara_rangos(hsv, self.rangos_suelo))
        rojo = self._morfologia(self._mascara_rangos(hsv, self.rangos_rojo))
        verde = self._morfologia(self._mascara_rangos(hsv, self.rangos_verde))
        for mascara in (suelo, rojo, verde):
            self._aplicar_roi_y_robot(mascara, y0, y1)
        suelo = self._suelo_conectado(suelo)

        detecciones = self._extraer_color(
            rojo, "ROJO", instante, suelo, y1, alto_roi, area_roi
        )
        detecciones.extend(
            self._extraer_color(
                verde, "VERDE", instante, suelo, y1, alto_roi, area_roi
            )
        )
        detecciones.sort(
            key=lambda det: (det.bottom_ratio, det.confianza, det.area_ratio),
            reverse=True,
        )

        duracion_ms = (time.perf_counter() - inicio) * 1000.0
        return PaqueteVision(
            timestamp=instante,
            detecciones=tuple(detecciones),
            duracion_ms=duracion_ms,
        )
