# -*- coding: utf-8 -*-
"""
Modelo pixel -> rumbo de la camara, MEDIDO en vez de sacado del catalogo,
y traslado de un cluster del LiDAR al origen de la camara.

navegacion.py usa hoy:

    HFOV_CAMARA = 102.0                # catalogo de la Module 3 Wide
    FOCAL_PX    = 160 / tan(51 grados) = 129.6 px
    CX_CENTRO_OPTICO = 160.0           # centro geometrico, supuesto

y su propio comentario avisa de que nunca se calibro. Los tres numeros
estan mal, y ademas el montaje en mastil añade un cuarto problema que
antes era despreciable (el paralaje). Todo esto se midio el 2026-08-28.

1. FOCAL. El FOV de catalogo no es el que llega al frame: el sensor
   entrega 1536x864 (16:9) y camara_driver pide 4:3, asi que libcamera
   recorta un 25% del ancho antes de escalar. Ademas la camara responde
   al FOV estandar, no al Wide. medir_fov.py emparejo una esquina que el
   LiDAR situa en -21.5 grados con su borde negro/blanco en la columna
   144 de 1280:

       f = (144 - 640) / tan(-21.5) = 1261 px a 1280 de ancho
       HFOV efectivo = 53.8 grados   (con el recorte de 3072 px)

   Ese 53.8 es el que fija FOCAL_SENSOR_PX, y por eso sigue siendo la
   medida de referencia aunque ahora se use el sensor entero.

   Concuerda con los 51.9 que predice la Module 3 ESTANDAR recortada a
   4:3, no con los 85.6 de la Wide. El modelo viejo infla el rumbo 2.3x:
   verificado con un pilar real a +9.2 grados, que calculaba a +21.1.

2. CENTRO OPTICO. Del barrido de test_pilar.py (90 muestras, pilar
   empujado de 428 a 215 mm) sale un sesgo constante de +2.78 grados
   respecto de la posicion que da el LiDAR, o sea el centro optico esta
   en cx = 175.1, no en 160. Son 15 px: montaje, no lente.

3. PARALAJE. La camara va en el mastil, ~100mm DETRAS del LiDAR (el
   propio barrido lo situa en el rumbo 176 a esa distancia, ver
   mapa_oclusion.py). A 900mm da igual, pero a 215mm el mismo objeto se
   ve 5 grados distinto desde cada sensor. Con la camara vieja, pegada
   al LiDAR, esto no importaba; ahora si.

Los puntos 2 y 3 estan ACOPLADOS y hay que corregir los dos o ninguno:
el sesgo del centro optico estaba tapando parte del paralaje, asi que
arreglar solo el centro empeora el resultado. Medido sobre las 82
muestras limpias entre 215 y 428 mm, error contra el rumbo real:

    modelo con c0=160 y sin paralaje  ->  [-6.0, +0.8]  desv 1.91
    solo centro optico corregido      ->  [-8.6, -1.9]  desv 1.89   (peor)
    centro optico + paralaje          ->  [-0.9, +0.4]  desv 0.30

Por eso el apareo no debe comparar dos rumbos medidos desde origenes
distintos: hay que trasladar el cluster del LiDAR al origen de la camara
con rumbo_camara_de_cluster() y comparar ahi.
"""
import math

# El frame ya no es 320x240 (4:3) sino 640x360 (16:9) desde el modo de
# sensor 2304x1296: ver la cabecera de camara_driver.py. El recorte pasa
# de 3072 a 4608 px de ancho de sensor, asi que TODO lo de aqui abajo
# cambia de valor -- lo que NO cambia es el metodo con que se obtuvo.
ANCHO_FRAME = 640.0
ALTO_FRAME  = 360.0

# Focal del sensor, MEDIDA. De aqui sale el HFOV de cualquier recorte
# por geometria, sin volver a la pista.
#
# CERRADO el 2026-08-29 con calib_fov.py y DOS pilares, que es lo unico
# que separa la focal del centro optico. Un pilar rojo a -27.5 grados y
# 462mm, uno verde a +19.2 y 1006mm: base angular de 47.2 grados, 44
# puntos, residuo de 0.21 grados de desviacion (maximo 0.61, o 2.2 px).
#
# Lo que sustituye: FOCAL_SENSOR_PX valia 3028, deducido de UNA esquina
# suponiendo el centro optico en el centro geometrico. Con el centro ya
# despejado sale 3405, un 12% mas, y el sesgo del centro es de 3.88
# grados y no de 2.78. Aquel 2.78 se habia medido con el barrido del
# pilar dando la focal vieja por buena, asi que arrastraba su error: con
# rumbos que solo cubren 7.7 grados, f y c0 se compensan y el barrido no
# podia distinguirlos.
#
# Aviso sobre el residuo: con dos pilares hay dos rumbos distintos y dos
# incognitas, asi que el ajuste queda exactamente determinado en la
# media. Ese 0.21 mide la repetibilidad de cada poste, NO valida la
# forma del modelo -- para eso haria falta un tercer rumbo. Lo que si
# queda bien condicionado, que es lo que fallaba antes, es la separacion
# entre focal y centro.
FOCAL_SENSOR_PX = 3405.0
ANCHO_RECORTE   = 4608.0          # leido de la metadata del frame
HFOV_EFECTIVO   = 2.0 * math.degrees(math.atan((ANCHO_RECORTE / 2.0) / FOCAL_SENSOR_PX))
FOCAL_PX        = (ANCHO_FRAME / 2.0) / math.tan(math.radians(HFOV_EFECTIVO / 2.0))

# El sesgo del centro optico es un ANGULO, asi que sobrevive a un cambio
# de modo de camara; lo que cambia es a cuantos pixeles equivale.
SESGO_CENTRO_GRADOS = 3.88
CX_CENTRO_OPTICO = ANCHO_FRAME / 2.0 + FOCAL_PX * math.tan(math.radians(SESGO_CENTRO_GRADOS))

# Posicion de la camara respecto del LiDAR, deducida del propio barrido:
# el mastil corta el plano del C1 en el rumbo 176 grados a ~100mm. Esto
# es geometria del chasis y no depende del modo de camara.
RUMBO_MASTIL_DEG = 176.0
DIST_MASTIL_MM   = 100.0
CAM_X = DIST_MASTIL_MM * math.sin(math.radians(RUMBO_MASTIL_DEG))
CAM_Y = DIST_MASTIL_MM * math.cos(math.radians(RUMBO_MASTIL_DEG))

# Con la focal, el centro y el paralaje corregidos el residuo medido fue
# de +-0.9 grados. Los 20 de navegacion.py se eligieron "generosos a
# proposito" para absorber un modelo sin calibrar; 10 deja margen de
# sobra. No bajar mas sin medirlo en movimiento: el barrido del pilar se
# tomo con el robot quieto y no incluye el desfase entre frame y barrido.
TOLERANCIA_APAREO_GRADOS = 10.0


def rumbo_de_cx(cx):
    """Rumbo en grados del pixel cx, desde el origen de la CAMARA.

    Positivo = a la derecha del eje optico.
    """
    return math.degrees(math.atan2(cx - CX_CENTRO_OPTICO, FOCAL_PX))


def cx_de_rumbo(rumbo_deg):
    """Inversa de rumbo_de_cx, util para dibujar diagnosticos."""
    return CX_CENTRO_OPTICO + FOCAL_PX * math.tan(math.radians(rumbo_deg))


def rumbo_camara_de_cluster(x_mm, y_mm):
    """Rumbo con que la CAMARA ve un cluster que el LiDAR situa en (x, y).

    x+ = derecha, y+ = frente, en mm y en el origen del LiDAR, que es lo
    que devuelve lidar_geometria.centroide_xy_cluster(). Es la funcion
    que hay que usar para aparear con rumbo_de_cx(): comparar contra
    atan2(x, y) mide el rumbo desde el LiDAR y mete hasta 5 grados de
    error de paralaje a distancia de evasion corta.
    """
    return math.degrees(math.atan2(x_mm - CAM_X, y_mm - CAM_Y))
