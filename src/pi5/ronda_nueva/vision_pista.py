"""Vision de la pista: espacio libre, pilares, cajon de parqueo y lineas.

LA IDEA CENTRAL: EL POLIGONO DE ESPACIO LIBRE
Se toma la mascara de "suelo" (baja saturacion, alto valor), se queda el
componente conectado que contiene el punto de siembra del borde inferior -- el
trozo de lona sobre el que el robot esta parado -- y se RELLENA su contorno.
Rellenarlo mete dentro del poligono a los objetos que estan apoyados en ese
suelo, porque son agujeros del componente.

A partir de ahi, "¿este blob rojo esta apoyado en la pista?" es un AND de dos
mascaras.  Es la prueba de soporte de suelo del campeon de 2025, pero hecha de
una sola pasada en vez de contando pixeles blancos en una banda debajo de la
caja.  Descarta de golpe reflejos en la pared, la camiseta de alguien y
cualquier cosa por encima del horizonte, sin un solo umbral de altura.

LA SEGUNDA IDEA: EL PUNTO DE CONTACTO
De cada blob interesa el pixel donde toca el suelo, no su centroide.  Ese
pixel se proyecta con la homografia y da (x, y) en milimetros.  El centroide
sube con la altura del objeto y por eso no sirve; el contacto esta en el plano
que la homografia describe.

DEGRADACION SIN CALIBRAR
Si todavia no hay homografia (la camara acaba de montarse), el modulo sigue
funcionando: estima la distancia por la altura aparente del pilar -- que es
independiente de la homografia -- y el bearing por la columna.  Los numeros
son peores pero el formato de salida es el mismo, asi que el resto del sistema
no se entera.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .geometria_suelo import ProyectorSuelo
from .modelos import (
    DeteccionPilar,
    LineaPiso,
    PaqueteVision,
    ParedMagenta,
    ROJO,
    VERDE,
)


RangoHSV = Tuple[np.ndarray, np.ndarray]


def _limitar(valor: float, minimo: float, maximo: float) -> float:
    return max(minimo, min(maximo, valor))


class VisionPista:
    """Un cuadro entra, un ``PaqueteVision`` en milimetros sale."""

    def __init__(self, config: Dict[str, Any]):
        camara = config["camera"]
        vision = config["vision"]
        pista = config.get("track", {})

        self.ancho = int(camara["width"])
        self.alto = int(camara["height"])
        # El color y la morfologia se hacen a MENOS resolucion.  Medido en la
        # Pi 5, el cuadro completo a 1280x720 costaba 49 ms por las seis
        # mascaras HSV y sus doce pasadas de morfologia; a la mitad de lado
        # baja a la cuarta parte de pixeles.  La homografia y el bearing
        # siguen usando la resolucion COMPLETA: los pixeles se reescalan antes
        # de proyectar, asi que la calibracion no cambia.
        self.escala = _limitar(float(vision.get("process_scale", 1.0)), 0.2, 1.0)
        self.ancho_p = max(32, int(round(self.ancho * self.escala)))
        self.alto_p = max(32, int(round(self.alto * self.escala)))
        self._a_completa = self.ancho / float(self.ancho_p)
        self.rotar_180 = int(camara.get("rotation_deg", 0)) == 180
        self.voltear_h = bool(camara.get("flip_horizontal", False))
        self.orden_bgr = str(camara.get("array_color_order", "BGR")).upper() == "BGR"
        self.hfov_deg = float(camara["hfov_deg"])
        self.principal_x = float(camara.get("principal_x_px", self.ancho / 2.0))
        self.focal_px = (self.ancho / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

        self.proyector: Optional[ProyectorSuelo] = ProyectorSuelo.desde_config(camara)

        self.roi_top = _limitar(float(camara.get("roi_top_ratio", 0.0)), 0.0, 0.95)
        self.roi_bottom = _limitar(float(camara.get("roi_bottom_ratio", 1.0)), 0.05, 1.0)
        self.semilla = tuple(camara.get("floor_seed_norm", (0.5, 0.95)))
        self.poligonos_robot = tuple(
            tuple(map(tuple, poligono))
            for poligono in camara.get("robot_mask_polygons_norm", ())
        )

        self.rangos = {
            ROJO: self._preparar(vision["red_ranges"]),
            VERDE: self._preparar(vision["green_ranges"]),
            "MAGENTA": self._preparar(vision.get("magenta_ranges", ())),
            "AZUL": self._preparar(vision.get("blue_ranges", ())),
            "NARANJA": self._preparar(vision.get("orange_ranges", ())),
        }
        self.rangos_suelo = self._preparar(vision["floor_ranges"])

        # Los umbrales de la configuracion estan en pixeles del cuadro
        # COMPLETO: aqui se traducen a la resolucion de proceso.  Las areas van
        # con el cuadrado de la escala y las longitudes con la escala.
        self.kernel_abrir = self._kernel(
            round(float(vision.get("morph_open_px", 5)) * self.escala)
        )
        self.kernel_cerrar = self._kernel(
            round(float(vision.get("morph_close_px", 5)) * self.escala)
        )

        area = self.escala * self.escala
        self.min_area_px = max(8, int(vision.get("min_area_px", 90) * area))
        self.min_alto_px = max(4, int(vision.get("min_height_px", 12) * self.escala))
        self.min_aspecto = float(vision.get("min_aspect_height_width", 0.7))
        self.min_relleno = float(vision.get("min_fill_ratio", 0.35))
        self.max_pilares = int(vision.get("max_blobs_per_color", 4))
        self.tolerancia_altura = float(vision.get("height_range_tolerance", 0.45))
        self.min_area_magenta_px = max(12, int(vision.get("magenta_min_area_px", 250) * area))
        # Las lineas se miden en pixeles del cuadro completo, no reducido.
        self.min_area_linea_px = max(40, int(vision.get("line_min_area_px", 400)))
        self.min_aspecto_linea = float(vision.get("line_min_aspect_width_height", 1.6))
        self.max_profundidad_linea = float(vision.get("line_max_depth_ratio", 2.2))
        self.min_soporte_suelo = float(vision.get("min_ground_support", 0.35))
        self._kernel_borde = self._kernel(vision.get("line_polygon_dilate_px", 11))
        self.epsilon_poligono = float(vision.get("polygon_epsilon_ratio", 0.002))

        self.altura_pilar_mm = float(pista.get("pillar_height_mm", 100.0))
        self.ancho_pilar_mm = float(pista.get("pillar_width_mm", 50.0))
        self.alcance_max_mm = float(vision.get("max_range_mm", 3200.0))

        # Buffers reutilizados: la ronda procesa 30 cuadros por segundo y
        # reservar mascaras nuevas en cada uno le regala trabajo al recolector
        # justo en el hilo que no debe atrasarse.
        self._suelo = np.zeros((self.alto_p, self.ancho_p), dtype=np.uint8)
        self._mascara_robot = self._construir_mascara_robot(self.ancho_p, self.alto_p)
        # Las lineas del piso se buscan a resolucion COMPLETA aunque el resto
        # vaya reducido: son franjas finas y al reducir se promedian con la
        # lona blanca de al lado.  Medido en la lona real, la naranja a 1,5 m
        # pasaba de 2074 pixeles saturados a 30 al reducir a la mitad.
        self._mascara_robot_completa = (
            self._construir_mascara_robot(self.ancho, self.alto)
            if self.escala < 0.999
            else self._mascara_robot
        )
        self.ultimo_poligono: Optional[np.ndarray] = None
        # Se expone para las herramientas de calibracion, que necesitan el
        # borde suelo/pared. La ronda no lo usa.
        self.ultimo_espacio_libre: Optional[np.ndarray] = None
        self._componente_suelo: Optional[np.ndarray] = None

    # ---------------------------------------------------------------- setup

    @staticmethod
    def _preparar(rangos: Iterable[Sequence[Sequence[int]]]) -> Tuple[RangoHSV, ...]:
        preparados = []
        for bajo, alto in rangos or ():
            preparados.append(
                (
                    np.asarray(bajo, dtype=np.uint8),
                    np.asarray(alto, dtype=np.uint8),
                )
            )
        return tuple(preparados)

    @staticmethod
    def _kernel(tamano: Any) -> Optional[np.ndarray]:
        lado = int(tamano or 0)
        if lado <= 1:
            return None
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (lado, lado))

    def _construir_mascara_robot(self, ancho: int, alto: int) -> Optional[np.ndarray]:
        """Zonas del cuadro ocupadas por el propio robot (mastil, guardabarros).

        Se calcula una vez porque no cambia: la camara esta atornillada.
        """

        if not self.poligonos_robot:
            return None
        mascara = np.full((alto, ancho), 255, dtype=np.uint8)
        for poligono in self.poligonos_robot:
            if len(poligono) < 3:
                continue
            puntos = np.asarray(
                [
                    (
                        int(round(_limitar(float(x), 0.0, 1.0) * (ancho - 1))),
                        int(round(_limitar(float(y), 0.0, 1.0) * (alto - 1))),
                    )
                    for x, y in poligono
                ],
                dtype=np.int32,
            )
            cv2.fillPoly(mascara, [puntos], 0)
        return mascara

    # ------------------------------------------------------------- utilidad

    def orientar(self, imagen: np.ndarray) -> np.ndarray:
        if self.rotar_180:
            imagen = cv2.rotate(imagen, cv2.ROTATE_180)
        if self.voltear_h:
            imagen = cv2.flip(imagen, 1)
        return imagen

    def bearing_para_x(self, x_px: float) -> float:
        """Bearing en grados de una columna del cuadro (positivo a la derecha)."""

        return math.degrees(math.atan2(float(x_px) - self.principal_x, self.focal_px))

    def _a_hsv(self, imagen: np.ndarray) -> np.ndarray:
        codigo = cv2.COLOR_BGR2HSV if self.orden_bgr else cv2.COLOR_RGB2HSV
        return cv2.cvtColor(imagen, codigo)

    def _mascara(self, hsv: np.ndarray, rangos: Tuple[RangoHSV, ...]) -> np.ndarray:
        salida = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for bajo, alto in rangos:
            cv2.bitwise_or(salida, cv2.inRange(hsv, bajo, alto), dst=salida)
        return salida

    def _morfologia(self, mascara: np.ndarray) -> np.ndarray:
        if self.kernel_abrir is not None:
            mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, self.kernel_abrir)
        if self.kernel_cerrar is not None:
            mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, self.kernel_cerrar)
        return mascara

    # ----------------------------------------------------- espacio libre

    def _espacio_libre(self, mascara_suelo: np.ndarray) -> Optional[np.ndarray]:
        """Componente de suelo bajo el robot, con sus agujeros rellenos.

        Devuelve la mascara rellena y guarda el contorno simplificado en
        ``ultimo_poligono`` para el panel web y las herramientas de diagnostico.
        """

        binaria = (mascara_suelo > 0).astype(np.uint8)
        cantidad, etiquetas, estadisticas, _ = cv2.connectedComponentsWithStats(
            binaria, connectivity=8
        )
        if cantidad <= 1:
            self.ultimo_poligono = None
            self._componente_suelo = None
            return None

        sx = int(round(_limitar(float(self.semilla[0]), 0.0, 1.0) * (self.ancho_p - 1)))
        sy = int(round(_limitar(float(self.semilla[1]), 0.0, 1.0) * (self.alto_p - 1)))
        etiqueta = int(etiquetas[sy, sx])
        if etiqueta == 0:
            # El punto de siembra cayo sobre una linea del piso o sobre una
            # sombra.  Quedarse con el componente mayor es la eleccion menos
            # mala; abandonar dejaria al robot ciego por un pixel.
            areas = estadisticas[1:, cv2.CC_STAT_AREA]
            if len(areas) == 0:
                self.ultimo_poligono = None
                self._componente_suelo = None
                return None
            etiqueta = int(np.argmax(areas)) + 1

        componente = np.where(etiquetas == etiqueta, 255, 0).astype(np.uint8)
        # El componente SIN rellenar: es la lona de verdad, la que esta
        # conectada con el trozo sobre el que el robot esta parado.  La
        # mascara de color cruda no vale para nada que dependa de "esto es
        # pista": la pared beige de la sala es igual de clara y desaturada que
        # la lona, y pasaba el umbral sin problema.
        self._componente_suelo = componente
        contornos, _ = cv2.findContours(
            componente, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contornos:
            self.ultimo_poligono = None
            return None
        contorno = max(contornos, key=cv2.contourArea)
        epsilon = self.epsilon_poligono * cv2.arcLength(contorno, True)
        poligono = cv2.approxPolyDP(contorno, epsilon, True)
        self.ultimo_poligono = poligono

        relleno = np.zeros_like(componente)
        cv2.fillPoly(relleno, [poligono], 255)
        return relleno

    # ---------------------------------------------------------- detecciones

    def _contactos(
        self, mascara: np.ndarray, min_area_px: int
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int], float]]:
        contornos, _ = cv2.findContours(
            mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        salida = []
        for contorno in contornos:
            area = float(cv2.contourArea(contorno))
            if area < min_area_px:
                continue
            salida.append((contorno, cv2.boundingRect(contorno), area))
        return salida

    def _posicion_desde_contacto(
        self, x_px: float, y_px: float
    ) -> Optional[Tuple[float, float]]:
        """Pixel del cuadro COMPLETO -> milimetros.

        CONVENCION: todos los ayudantes de posicion de este modulo reciben
        pixeles del cuadro completo.  Quien lee una caja del mundo reducido la
        multiplica por ``_a_completa`` ANTES de llamar aqui.  La primera
        version escalaba dentro y ademas fuera, y la linea naranja salia a
        245 mm cuando estaba a 960: un factor de dos exacto, que es la firma
        inconfundible de este error.
        """

        if self.proyector is None:
            return None
        return self.proyector.punto_suelo(x_px, y_px)

    def _posicion_por_altura(
        self, x_px: float, alto_px: float, altura_real_mm: float
    ) -> Optional[Tuple[float, float]]:
        """Respaldo geometrico cuando no hay homografia o el pie esta tapado.

        La altura aparente da la distancia a lo largo del rayo; el bearing da
        su direccion.  No necesita conocer la pose de la camara, solo la
        optica, asi que sirve el primer dia de montaje.
        """

        if alto_px <= 0.0:
            return None
        distancia = self.focal_px * altura_real_mm / float(alto_px)
        if not 0.0 < distancia <= self.alcance_max_mm:
            return None
        bearing = math.radians(self.bearing_para_x(x_px))
        return distancia * math.sin(bearing), distancia * math.cos(bearing)

    def _detectar_pilares(
        self,
        hsv: np.ndarray,
        libre: Optional[np.ndarray],
        timestamp: float,
        y_roi_fin: int,
    ) -> List[DeteccionPilar]:
        detecciones: List[DeteccionPilar] = []
        for color in (ROJO, VERDE):
            mascara = self._morfologia(self._mascara(hsv, self.rangos[color]))
            self._recortar(mascara, y_roi_fin)

            candidatos = []
            for contorno, (x, y, ancho, alto), area in self._contactos(
                mascara, self.min_area_px
            ):
                if alto < self.min_alto_px:
                    continue
                if alto / float(max(1, ancho)) < self.min_aspecto:
                    continue
                if area / float(max(1, ancho * alto)) < self.min_relleno:
                    continue
                if libre is not None and (
                    self._soporte_suelo(libre, (x, y, ancho, alto))
                    < self.min_soporte_suelo
                ):
                    continue

                deteccion = self._construir_pilar(
                    color, (x, y, ancho, alto), area, timestamp, y_roi_fin
                )
                if deteccion is not None:
                    candidatos.append(deteccion)

            candidatos.sort(key=lambda det: (det.y_mm, -det.confianza))
            detecciones.extend(candidatos[: self.max_pilares])
        return detecciones

    def _construir_pilar(
        self,
        color: str,
        bbox: Tuple[int, int, int, int],
        area: float,
        timestamp: float,
        y_roi_fin: int,
    ) -> Optional[DeteccionPilar]:
        x, y, ancho, alto = bbox
        pie_cortado = (y + alto) >= y_roi_fin - 2 or (y + alto) >= self.alto_p - 2
        # A partir de aqui se trabaja en pixeles del cuadro completo.
        escala = self._a_completa
        x, y, ancho, alto = x * escala, y * escala, ancho * escala, alto * escala
        pie_x = x + ancho / 2.0
        pie_y = y + alto

        por_altura = self._posicion_por_altura(pie_x, alto, self.altura_pilar_mm)
        por_contacto = None if pie_cortado else self._posicion_desde_contacto(pie_x, pie_y)

        if por_contacto is not None:
            x_mm, y_mm = por_contacto
            fuente = "CAMARA"
            confianza = 0.85
            # El acuerdo entre dos medidas independientes es la unica prueba
            # barata de que el blob es de verdad un pilar apoyado en la lona.
            if por_altura is not None:
                distancia_contacto = math.hypot(x_mm, y_mm)
                distancia_altura = math.hypot(*por_altura)
                if distancia_contacto > 1.0:
                    desvio = abs(distancia_altura - distancia_contacto) / distancia_contacto
                    if desvio > self.tolerancia_altura:
                        return None
                    confianza = _limitar(1.0 - desvio, 0.35, 0.98)
        elif por_altura is not None:
            x_mm, y_mm = por_altura
            fuente = "ALTURA"
            confianza = 0.45
        else:
            return None

        if not 0.0 < y_mm <= self.alcance_max_mm:
            return None

        distancia_altura_mm = 0.0
        if por_altura is not None:
            distancia_altura_mm = math.hypot(*por_altura)

        return DeteccionPilar(
            timestamp=timestamp,
            color=color,
            x_mm=float(x_mm),
            y_mm=float(y_mm),
            fuente=fuente,
            confianza=float(confianza),
            ancho_px=int(ancho),
            alto_px=int(alto),
            distancia_por_altura_mm=float(distancia_altura_mm),
            bbox=(int(x), int(y), int(ancho), int(alto)),
        )

    def _detectar_magenta(
        self,
        hsv: np.ndarray,
        libre: Optional[np.ndarray],
        timestamp: float,
        y_roi_fin: int,
    ) -> List[ParedMagenta]:
        """Los delimitadores del cajon de parqueo.

        No se les exige forma de poste: son piezas bajas y anchas.  Interesa
        donde tocan el suelo, porque eso los pone en el mismo mapa que el
        hueco que el LiDAR mide entre ellos.
        """

        if not self.rangos["MAGENTA"]:
            return []
        mascara = self._morfologia(self._mascara(hsv, self.rangos["MAGENTA"]))
        self._recortar(mascara, y_roi_fin)

        salida = []
        for _, (x, y, ancho, alto), area in self._contactos(
            mascara, self.min_area_magenta_px
        ):
            # Base cortada por el borde del cuadro: no hay punto de contacto,
            # asi que no hay posicion fiable.  Pasa cuando el delimitador esta
            # pegado al robot, y ahi manda el LiDAR de todas formas: el magenta
            # sirve para la aproximacion larga, no para el ultimo palmo.
            if (y + alto) >= y_roi_fin - 2:
                continue
            if libre is not None and (
                self._soporte_suelo(libre, (x, y, ancho, alto)) < self.min_soporte_suelo
            ):
                continue
            escala = self._a_completa
            posicion = self._posicion_desde_contacto(
                (x + ancho / 2.0) * escala, (y + alto) * escala
            )
            if posicion is None:
                continue
            salida.append(
                ParedMagenta(
                    timestamp=timestamp,
                    x_mm=float(posicion[0]),
                    y_mm=float(posicion[1]),
                    ancho_px=int(ancho * self._a_completa),
                    alto_px=int(alto * self._a_completa),
                    confianza=_limitar(area / float(max(1, ancho * alto)), 0.0, 1.0),
                )
            )
        salida.sort(key=lambda pared: pared.y_mm)
        return salida

    def _detectar_lineas(
        self,
        hsv: np.ndarray,
        libre: Optional[np.ndarray],
        timestamp: float,
        y_roi_fin: int,
        escala: float = 1.0,
        suelo: Optional[np.ndarray] = None,
    ) -> List[LineaPiso]:
        """Lineas azul y naranja del piso.

        Importan por dos razones: dan el sentido de giro al cruzar la primera
        y cuentan las vueltas.  Leerlas con la camara sustituye al sensor de
        color de piso, que en la Pi 5 responde ``SIN_SENSOR``.
        """

        salida = []
        for color in ("AZUL", "NARANJA"):
            if not self.rangos[color]:
                continue
            mascara = self._mascara(hsv, self.rangos[color])
            mascara[: int(round(self.roi_top * hsv.shape[0])), :] = 0
            mascara[y_roi_fin:, :] = 0
            robot = (
                self._mascara_robot_completa if escala >= 0.999 else self._mascara_robot
            )
            if robot is not None:
                cv2.bitwise_and(mascara, robot, dst=mascara)
            if libre is not None:
                # El poligono se DILATA un poco solo para las lineas: una linea
                # lejana cae justo sobre el borde suelo/pared, que ya viene
                # simplificado por approxPolyDP, y el AND estricto se la comia.
                # Los pilares no necesitan esto -- estan bien dentro -- y ahi
                # interesa que el filtro siga siendo estricto.
                cv2.bitwise_and(
                    mascara, self._dilatar_libre(libre), dst=mascara
                )

            mejor = None
            for _, (x, y, ancho, alto), area in self._contactos(
                mascara, self.min_area_linea_px
            ):
                # Una linea del piso es una franja ANCHA y baja.  Sin este
                # filtro, un pilar rojo cae dentro del rango de matiz de la
                # naranja y se cuenta como linea de sentido, que es el peor
                # error posible: manda al robot a dar la vuelta al reves.
                if ancho < self.min_aspecto_linea * max(1, alto):
                    continue
                factor = 1.0 / max(1e-6, escala) if escala < 0.999 else 1.0
                if not self._es_plano(
                    (
                        int(x * factor),
                        int(y * factor),
                        int(ancho * factor),
                        int(alto * factor),
                    )
                ):
                    continue
                if mejor is not None and area <= mejor[2]:
                    continue
                mejor = ((x, y, ancho, alto), (x + ancho / 2.0, y + alto / 2.0), area)
            if mejor is None:
                continue

            (_, _, _, _), (cx, cy), area = mejor
            posicion = self._posicion_desde_contacto(
                cx * (self._a_completa if escala < 0.999 else 1.0),
                cy * (self._a_completa if escala < 0.999 else 1.0),
            )
            if posicion is None:
                continue
            salida.append(
                LineaPiso(
                    timestamp=timestamp,
                    color=color,
                    x_mm=float(posicion[0]),
                    y_mm=float(posicion[1]),
                    area_px=int(area / max(1e-6, escala * escala)),
                )
            )
        return salida

    def _soporte_suelo(
        self, libre: np.ndarray, bbox: Tuple[int, int, int, int]
    ) -> float:
        """Fraccion de suelo en la banda justo DEBAJO de la base del blob.

        POR QUE NO BASTA CON CRUZARLO CON EL POLIGONO
        La primera version hacia un AND entre la mascara de color y el poligono
        de espacio libre.  Funciona para un poste en medio de la lona -- es un
        agujero del componente y al rellenar el contorno queda dentro -- pero
        FALLA para un poste pegado al muro del fondo: ahi el poste no es un
        agujero, es una muesca del borde superior, y queda fuera del relleno.

        Medido en el robot el 04-09: un pilar rojo perfectamente visible a
        1,4 m pasaba todos los filtros de forma y las dos estimaciones de
        distancia coincidian al 9 %, y aun asi el AND lo dejaba en 7 pixeles de
        725.  Preguntar por el suelo que hay DEBAJO cubre los dos casos, y es
        ademas la prueba original del campeon de 2025.
        """

        x, y, ancho, alto = bbox
        base = y + alto
        if base >= libre.shape[0] - 2:
            return 0.0
        # Banda FINA y pegada a la base.  Con una banda ancha, una mancha roja
        # pintada en el muro a unos pocos pixeles por encima de la lona
        # tambien la alcanzaba y pasaba el filtro; lo que distingue a un poste
        # es que su base ES el borde del suelo, no que haya suelo cerca.
        y0 = min(libre.shape[0] - 1, base + 1)
        alto_banda = max(3, int(alto * 0.18))
        y1 = min(libre.shape[0], y0 + alto_banda)
        margen = max(1, int(ancho * 0.15))
        x0 = max(0, x + margen)
        x1 = min(libre.shape[1], x + ancho - margen)
        if x1 <= x0:
            return 0.0
        if y1 <= y0:
            return 0.0
        banda = libre[y0:y1, x0:x1]
        if banda.size == 0:
            return 0.0
        return float(np.count_nonzero(banda)) / float(banda.size)

    def _es_plano(self, bbox: Tuple[int, int, int, int]) -> bool:
        """¿La mancha esta TUMBADA en el suelo, o de pie?

        Se proyectan al suelo su borde inferior y su borde superior.  Si esta
        tumbada -- una linea pintada --, los dos caen a distancias parecidas.
        Si esta de pie -- el delimitador del cajon, un pilar, un mueble --, el
        borde superior se va muchisimo mas lejos o directamente por encima del
        horizonte, porque la homografia supone que todo pixel es suelo.

        Hizo falta porque el delimitador violeta del banco (H 127-131) cae
        dentro del rango de la linea azul (H 125-132) y ningun umbral de color
        los separa con margen.  Antes se probaron dos filtros peores: exigir
        que la mancha fuera mucho mas ancha que alta -- descartaba tambien la
        linea azul de verdad cuando solo se ve un trozo -- y mirar si habia
        lona por encima, que falla porque por encima del delimitador se ve la
        lona del otro lado del muro.

        Sin homografia no se puede decidir, y entonces se acepta: mas vale una
        linea de mas que quedarse sin sentido de giro.
        """

        if self.proyector is None:
            return True
        x, y, ancho, alto = bbox
        base = self.proyector.punto_suelo(x + ancho / 2.0, float(y + alto))
        techo = self.proyector.punto_suelo(x + ancho / 2.0, float(y))
        if base is None:
            return False
        if techo is None:
            return False        # el borde superior pasa del horizonte: esta de pie
        return techo[1] <= self.max_profundidad_linea * base[1]

    def _dilatar_libre(self, libre: np.ndarray) -> np.ndarray:
        if self._kernel_borde is None:
            return libre
        return cv2.dilate(libre, self._kernel_borde)

    def _recortar(self, mascara: np.ndarray, y_roi_fin: int) -> None:
        y0 = int(round(self.roi_top * self.alto_p))
        mascara[:y0, :] = 0
        mascara[y_roi_fin:, :] = 0
        if self._mascara_robot is not None:
            cv2.bitwise_and(mascara, self._mascara_robot, dst=mascara)

    # ---------------------------------------------------------------- api

    def procesar(
        self, imagen: np.ndarray, timestamp: Optional[float] = None
    ) -> PaqueteVision:
        inicio = time.perf_counter()
        instante = time.monotonic() if timestamp is None else float(timestamp)

        orientada = self.orientar(imagen)
        if self.escala < 0.999:
            orientada = cv2.resize(
                orientada, (self.ancho_p, self.alto_p), interpolation=cv2.INTER_AREA
            )
        hsv = self._a_hsv(orientada)
        y_roi_fin = max(1, min(self.alto_p, int(round(self.roi_bottom * self.alto_p))))

        suelo = self._morfologia(self._mascara(hsv, self.rangos_suelo))
        self._recortar(suelo, y_roi_fin)
        libre = self._espacio_libre(suelo)
        self.ultimo_espacio_libre = libre

        pilares = self._detectar_pilares(hsv, libre, instante, y_roi_fin)
        magenta = self._detectar_magenta(hsv, libre, instante, y_roi_fin)

        if self.escala < 0.999 and (self.rangos["AZUL"] or self.rangos["NARANJA"]):
            hsv_completa = self._a_hsv(self.orientar(imagen))
            libre_completa = (
                None
                if libre is None
                else cv2.resize(
                    libre, (self.ancho, self.alto), interpolation=cv2.INTER_NEAREST
                )
            )
            lineas = self._detectar_lineas(
                hsv_completa,
                libre_completa,
                instante,
                int(round(self.roi_bottom * self.alto)),
                escala=1.0,
                suelo=self._componente_suelo,
            )
        else:
            lineas = self._detectar_lineas(
                hsv,
                libre,
                instante,
                y_roi_fin,
                escala=self.escala,
                suelo=self._componente_suelo,
            )

        pilares.sort(key=lambda det: det.y_mm)
        return PaqueteVision(
            timestamp=instante,
            pilares=tuple(pilares),
            magenta=tuple(magenta),
            lineas=tuple(lineas),
            duracion_ms=(time.perf_counter() - inicio) * 1000.0,
            suelo_px=int(np.count_nonzero(libre) / max(1e-6, self.escala ** 2))
            if libre is not None
            else 0,
        )
