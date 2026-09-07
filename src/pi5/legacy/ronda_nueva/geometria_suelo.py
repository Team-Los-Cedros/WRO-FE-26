"""Proyeccion de la imagen al plano del suelo.

POR QUE ESTE MODULO EXISTE
Con la camara en el mastil trasero y alta, casi todo lo que importa de la
pista -- pilares, lineas de sentido, muros del cajon -- toca el suelo dentro
del cuadro.  Un punto del suelo visto por una camara fija se convierte en una
posicion metrica con una simple homografia 3x3, asi que la vision puede
entregar milimetros del marco del robot en vez de un bearing.

Eso cambia tres cosas de raiz respecto al esquema anterior:

* El pilar se mide a 200 mm y a 2500 mm por igual.  No depende de que el plano
  del LiDAR le pegue al poste, que era la causa del "760 -> 30 mm en un solo
  barrido" al salir de las esquinas.
* Camara y LiDAR hablan el mismo idioma, asi que asociarlos es buscar el
  vecino mas cercano en milimetros.  Desaparecen la puerta angular y la
  sensibilidad al sesgo de guiñada.
* La calibracion se puede comprobar: proyectar un pilar que el LiDAR tambien
  ve y restar da un error en milimetros, no una corazonada.

DOS CAMINOS PARA OBTENER LA HOMOGRAFIA
1. ``desde_montaje``: se calcula a partir de altura, cabeceo, guiñada y optica
   medidos con regla.  Sirve para arrancar el mismo dia que se monta la
   camara, y su error es el error de la regla.
2. ``desde_correspondencias``: DLT sobre pares (pixel del suelo, punto en mm).
   Es la buena.  Los pares los da ``herramientas/calibrar_suelo.py`` usando
   los pilares que el LiDAR ve, o puntos marcados en la lona.

CONVENCION DE LA IMAGEN
``u`` crece hacia la derecha del cuadro y ``v`` hacia abajo, como en OpenCV.
El resultado esta en el marco del robot: ``x`` a la derecha, ``y`` adelante.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np


ParCalibracion = Tuple[Tuple[float, float], Tuple[float, float]]


class ErrorHomografia(ValueError):
    """La homografia pedida no se puede construir con los datos dados."""


def matriz_intrinseca(
    ancho_px: int,
    alto_px: int,
    hfov_deg: float,
    principal_x_px: Optional[float] = None,
    principal_y_px: Optional[float] = None,
) -> np.ndarray:
    """Intrinseca de un pinhole con pixeles cuadrados.

    ``hfov_deg`` es el campo horizontal REAL medido, no el de catalogo: en la
    sesion del 29-08 la diferencia entre ambos era de 2,3x y se llevo por
    delante toda la geometria de la evasion.
    """

    if ancho_px <= 0 or alto_px <= 0:
        raise ErrorHomografia("tamaño de imagen invalido")
    if not 1.0 < float(hfov_deg) < 179.0:
        raise ErrorHomografia("hfov_deg fuera de rango")

    fx = (ancho_px / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    cx = ancho_px / 2.0 if principal_x_px is None else float(principal_x_px)
    cy = alto_px / 2.0 if principal_y_px is None else float(principal_y_px)
    return np.array([[fx, 0.0, cx], [0.0, fx, cy], [0.0, 0.0, 1.0]], dtype=float)


def _rotacion_camara(cabeceo_deg: float, guinada_deg: float) -> np.ndarray:
    """Columnas = ejes de la camara expresados en el marco del robot.

    Marco del robot: X derecha, Y adelante, Z arriba.
    Marco de la camara: x derecha, y abajo, z hacia donde mira.

    ``cabeceo_deg`` positivo significa que la camara mira hacia el suelo, que
    es como esta montada en el mastil.
    """

    t = math.radians(cabeceo_deg)
    p = math.radians(guinada_deg)
    eje_z = np.array(
        [math.sin(p) * math.cos(t), math.cos(p) * math.cos(t), -math.sin(t)]
    )
    eje_x = np.array([math.cos(p), -math.sin(p), 0.0])
    eje_y = np.cross(eje_z, eje_x)
    return np.column_stack((eje_x, eje_y, eje_z))


def desde_montaje(
    ancho_px: int,
    alto_px: int,
    hfov_deg: float,
    altura_mm: float,
    cabeceo_deg: float,
    guinada_deg: float = 0.0,
    adelante_mm: float = 0.0,
    derecha_mm: float = 0.0,
    principal_x_px: Optional[float] = None,
    principal_y_px: Optional[float] = None,
) -> np.ndarray:
    """Homografia imagen -> suelo a partir de las medidas del montaje."""

    if altura_mm <= 0.0:
        raise ErrorHomografia("altura_mm debe ser positiva")
    if not 0.0 < float(cabeceo_deg) < 89.0:
        raise ErrorHomografia("cabeceo_deg debe mirar al suelo (0 < c < 89)")

    K = matriz_intrinseca(ancho_px, alto_px, hfov_deg, principal_x_px, principal_y_px)
    R = _rotacion_camara(cabeceo_deg, guinada_deg)
    centro = np.array([float(derecha_mm), float(adelante_mm), float(altura_mm)])

    # p_camara = R^T (P - C).  Con Z = 0 en el suelo, la dependencia de (X, Y)
    # es lineal, asi que suelo -> imagen es exactamente una homografia.
    Rt = R.T
    M = np.column_stack((Rt[:, 0], Rt[:, 1], -Rt @ centro))
    suelo_a_imagen = K @ M
    if abs(np.linalg.det(suelo_a_imagen)) < 1e-9:
        raise ErrorHomografia("montaje degenerado: la camara no ve el suelo")
    return np.linalg.inv(suelo_a_imagen)


def _casi_alineados(puntos: np.ndarray, umbral: float = 0.06) -> bool:
    """True si la nube no tiene area util.

    Se compara el segundo valor singular con el primero: en una fila de puntos
    el segundo es casi cero.  Sin esta comprobacion el DLT devuelve una matriz
    que reproyecta bien los propios puntos y mal todo lo demas, que es la peor
    forma de fallar porque parece que funciona.
    """

    centrados = puntos - puntos.mean(axis=0)
    valores = np.linalg.svd(centrados, compute_uv=False)
    if valores[0] < 1e-9:
        return True
    return bool(valores[1] / valores[0] < umbral)


def _normalizar(puntos: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    centro = puntos.mean(axis=0)
    desplazados = puntos - centro
    escala_media = float(np.sqrt((desplazados ** 2).sum(axis=1)).mean())
    if escala_media < 1e-9:
        raise ErrorHomografia("todos los puntos coinciden")
    s = math.sqrt(2.0) / escala_media
    T = np.array([[s, 0.0, -s * centro[0]], [0.0, s, -s * centro[1]], [0.0, 0.0, 1.0]])
    homogeneos = np.column_stack((puntos, np.ones(len(puntos))))
    return T, (homogeneos @ T.T)[:, :2]


def desde_correspondencias(pares: Sequence[ParCalibracion]) -> np.ndarray:
    """DLT normalizado sobre pares (pixel, punto del suelo en mm).

    Hacen falta al menos cuatro pares y que no esten alineados.  La
    normalizacion de Hartley no es cosmetica: sin ella, mezclar coordenadas de
    pixel (centenas) con milimetros (millares) deja la matriz mal
    condicionada y la solucion depende del orden de los puntos.
    """

    if len(pares) < 4:
        raise ErrorHomografia("hacen falta al menos 4 correspondencias")

    origen = np.array([[float(u), float(v)] for (u, v), _ in pares])
    destino = np.array([[float(x), float(y)] for _, (x, y) in pares])

    for nombre, puntos in (("pixeles", origen), ("puntos del suelo", destino)):
        if _casi_alineados(puntos):
            raise ErrorHomografia(
                f"los {nombre} estan casi alineados; una homografia necesita "
                "cuatro puntos que formen area, no una fila"
            )

    T_origen, origen_n = _normalizar(origen)
    T_destino, destino_n = _normalizar(destino)

    filas = []
    for (u, v), (x, y) in zip(origen_n, destino_n):
        filas.append([-u, -v, -1.0, 0.0, 0.0, 0.0, u * x, v * x, x])
        filas.append([0.0, 0.0, 0.0, -u, -v, -1.0, u * y, v * y, y])
    A = np.asarray(filas, dtype=float)

    # Este umbral NO detecta alineacion -- de eso se encarga _casi_alineados,
    # diez lineas mas arriba, y ese si es el test correcto.  Lo que mide el
    # menor valor singular de A es cuanto se DESVIAN las correspondencias de
    # una unica homografia, o sea ruido y parejas cruzadas.  Medido el 05-09
    # sobre correspondencias sinteticas del montaje real:
    #
    #     exactas                       1e-17
    #     1 px de ruido en el pixel     6,3e-04
    #     3 px de ruido en el pixel     2,1e-03
    #     cinco puntos EN LINEA         2,8e-19   <- pasaria, no es este test
    #
    # Con el umbral en 1e-3 se rechazaba cualquier dato real: las siete
    # correspondencias camara-LiDAR del 05-09 daban 3,8e-03 con un error de
    # reproyeccion de 8,6 mm de media y 18,3 mm el peor, que es una
    # calibracion perfectamente utilizable.  El liston que de verdad importa
    # esta aguas abajo y en milimetros (``error_de_reproyeccion``), no aqui.
    _, valores, Vt = np.linalg.svd(A)
    if valores[-1] > valores[0] * 1e-2:
        raise ErrorHomografia(
            "las correspondencias no encajan en una sola homografia: ruido "
            "excesivo o parejas cruzadas entre si"
        )
    H_n = Vt[-1].reshape(3, 3)
    H = np.linalg.inv(T_destino) @ H_n @ T_origen
    if abs(H[2, 2]) < 1e-12:
        raise ErrorHomografia("homografia degenerada")
    H = H / H[2, 2]

    # El signo global de una homografia es libre -- H y -H son el mismo mapa --
    # pero ProyectorSuelo usa ``w > 0`` como prueba del horizonte, asi que el
    # signo NO es libre para el, y dividir por H[2,2] no lo fija: con pixeles
    # en la mitad baja del cuadro el termino H[2,1]*v domina sobre el 1.
    #
    # Paso el 05-09 con las siete correspondencias camara-LiDAR reales: la
    # matriz reproducia los puntos al milimetro pero salia con w negativo, y
    # el proyector daba TODOS los pixeles por encima del horizonte.  El error
    # de reproyeccion salia ``inf`` con una homografia perfectamente buena.
    #
    # Las correspondencias son, por construccion, puntos del suelo visibles,
    # asi que su w tiene que ser positivo: eso fija el signo.
    w = np.column_stack((origen, np.ones(len(origen)))) @ H[2]
    if np.median(w) < 0.0:
        H = -H
    return H


class ProyectorSuelo:
    """Convierte pixeles del suelo en milimetros y al reves.

    La instancia es barata de usar: proyectar un punto son nueve
    multiplicaciones.  ``proyectar_muchos`` existe para no pagar el bucle de
    Python cuando hay que proyectar el contorno entero de un blob.
    """

    def __init__(
        self,
        homografia: np.ndarray,
        ancho_px: int,
        alto_px: int,
        alcance_max_mm: float = 3500.0,
        avance_min_mm: float = 40.0,
        focal_px: Optional[float] = None,
    ):
        H = np.asarray(homografia, dtype=float)
        if H.shape != (3, 3):
            raise ErrorHomografia("la homografia debe ser 3x3")
        if abs(np.linalg.det(H)) < 1e-12:
            raise ErrorHomografia("homografia singular")
        self.homografia = H
        self.inversa = np.linalg.inv(H)
        self.ancho_px = int(ancho_px)
        self.alto_px = int(alto_px)
        self.alcance_max_mm = float(alcance_max_mm)
        self.avance_min_mm = float(avance_min_mm)
        self.focal_px = None if focal_px is None else float(focal_px)

    @classmethod
    def desde_config(cls, camara: dict) -> Optional["ProyectorSuelo"]:
        """Construye el proyector desde el bloque ``camera`` de la config.

        Devuelve ``None`` -- y no lanza -- cuando el bloque dice que el suelo
        no esta calibrado todavia, porque ese es el estado normal mientras la
        camara esta desmontada.  Quien lo use decide como degradar.
        """

        suelo = camara.get("ground_homography") or {}
        if not suelo.get("ready"):
            return None

        ancho = int(camara["width"])
        alto = int(camara["height"])
        focal = None
        hfov = camara.get("hfov_deg")
        if hfov:
            focal = (ancho / 2.0) / math.tan(math.radians(float(hfov)) / 2.0)

        matriz = suelo.get("matrix")
        if matriz:
            H = np.asarray(matriz, dtype=float).reshape(3, 3)
        else:
            H = desde_montaje(
                ancho_px=ancho,
                alto_px=alto,
                hfov_deg=float(camara["hfov_deg"]),
                altura_mm=float(suelo["height_mm"]),
                cabeceo_deg=float(suelo["pitch_deg"]),
                guinada_deg=float(suelo.get("yaw_deg", 0.0)),
                adelante_mm=float(suelo.get("forward_mm", 0.0)),
                derecha_mm=float(suelo.get("right_mm", 0.0)),
                principal_x_px=camara.get("principal_x_px"),
                principal_y_px=camara.get("principal_y_px"),
            )
        return cls(
            H,
            ancho,
            alto,
            alcance_max_mm=float(suelo.get("max_range_mm", 3500.0)),
            avance_min_mm=float(suelo.get("min_forward_mm", 40.0)),
            focal_px=focal,
        )

    def punto_suelo(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        """Pixel -> (x, y) en mm, o ``None`` si el pixel no cae en el suelo.

        Un pixel por encima del horizonte da un tercer componente <= 0: no es
        que la cuenta salga grande, es que la recta de vision nunca corta el
        plano.  Devolver ``None`` en vez de un numero enorme evita que un
        reflejo en la pared se convierta en un pilar a cinco metros.
        """

        vector = self.homografia @ np.array([float(u), float(v), 1.0])
        w = vector[2]
        if w <= 1e-9:
            return None
        x = float(vector[0] / w)
        y = float(vector[1] / w)
        if y < self.avance_min_mm:
            return None
        if math.hypot(x, y) > self.alcance_max_mm:
            return None
        return x, y

    def proyectar_muchos(self, pixeles) -> np.ndarray:
        """Version vectorizada.  Devuelve NaN donde el pixel no toca el suelo."""

        pts = np.asarray(pixeles, dtype=float).reshape(-1, 2)
        homogeneos = np.column_stack((pts, np.ones(len(pts))))
        proyectados = homogeneos @ self.homografia.T
        w = proyectados[:, 2]
        valido = w > 1e-9
        salida = np.full((len(pts), 2), np.nan)
        salida[valido, 0] = proyectados[valido, 0] / w[valido]
        salida[valido, 1] = proyectados[valido, 1] / w[valido]
        return salida

    def punto_imagen(self, x_mm: float, y_mm: float) -> Optional[Tuple[float, float]]:
        """(x, y) en mm -> pixel, para dibujar sobre el cuadro."""

        vector = self.inversa @ np.array([float(x_mm), float(y_mm), 1.0])
        w = vector[2]
        if abs(w) < 1e-9:
            return None
        return float(vector[0] / w), float(vector[1] / w)

    def distancia_por_altura_mm(
        self, alto_px: float, altura_real_mm: float
    ) -> Optional[float]:
        """Segunda estimacion de distancia, independiente de la homografia.

        Un pilar de altura conocida ocupa ``f * H / d`` pixeles.  Sirve de
        contraste: si esta y la del punto de contacto discrepan mucho, el blob
        no esta apoyado en el suelo y hay que desconfiar de el.
        """

        if self.focal_px is None or alto_px <= 0.0 or altura_real_mm <= 0.0:
            return None
        return float(self.focal_px) * float(altura_real_mm) / float(alto_px)


def error_de_reproyeccion(
    proyector: ProyectorSuelo, pares: Sequence[ParCalibracion]
) -> Tuple[float, float]:
    """(error medio, error maximo) en mm sobre los pares dados.

    Es el numero que decide si una calibracion vale.  Se calcula sobre pares
    que NO se usaron para ajustar cuando se quiere una cifra honesta.
    """

    errores = []
    for (u, v), (x, y) in pares:
        estimado = proyector.punto_suelo(u, v)
        if estimado is None:
            errores.append(float("inf"))
            continue
        errores.append(math.hypot(estimado[0] - x, estimado[1] - y))
    if not errores:
        return 0.0, 0.0
    return float(sum(errores) / len(errores)), float(max(errores))
