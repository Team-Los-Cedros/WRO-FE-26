# Fuente unica de verdad de la geometria fisica del robot y de la pista.
# Todo numero en milimetros y grados. Ningun otro modulo debe volver a
# escribir una constante fisica a mano: si aparece un 136, un 222 o un
# offset de sensor en otro archivo, es un bug de duplicacion.
#
# Convencion de marcos (la misma de lidar_geometria.py):
#   x+ = derecha del robot, y+ = frente del robot, angulos en grados
#   0 grados = frente, crecen en sentido horario.
#
# Hay TRES marcos distintos y confundirlos es el error de rumbo que se
# quiere eliminar:
#   MARCO LIDAR       origen en el eje de rotacion del RPLIDAR C1.
#                     Es lo que devuelve centroide_xy_cluster().
#   MARCO CAMARA      origen en el lente. Solo importa su offset en x
#                     respecto al LiDAR (el paralaje).
#   MARCO EJE TRASERO origen en el punto medio del eje trasero. Es el
#                     marco donde la cinematica de bicicleta (Ackermann)
#                     es valida y donde se debe calcular el pure pursuit.
#
# Estado de cada constante:
#   [MEDIDO]     verificado en el robot fisico
#   [ESTIMADO]   valor provisional, ver seccion de estimacion abajo
#   [PENDIENTE]  todavia no hay dato, el placeholder es una suposicion
import math

# ==========================================
# CHASIS  [MEDIDO] - README seccion 7.3
# ==========================================
BATALLA        = 136.0   # distancia entre ejes (l)
VIA            = 115.0   # ancho de via (w)
LARGO_ROBOT    = 222.0   # largo total, el que usa el juez para el parqueo
ANCHO_ROBOT    = 125.0   # ancho total
RADIO_RUEDA    = 18.0    # neumatico LEGO de 36mm de diametro

# Voladizo trasero: cuanto sobresale la cola por detras del eje trasero.
# tracker.DISTANCIA_SUPERADO (280mm) lo asume implicitamente ~200mm.
# [PENDIENTE] medir con la misma regla que los offsets de sensores.
VOLADIZO_TRASERO = 60.0

# ==========================================
# OFFSET DEL LIDAR RESPECTO AL EJE TRASERO  [ESTIMADO]
# ==========================================
# LIDAR_X: cuanto ADELANTE del eje trasero esta el eje de rotacion del
#          LiDAR. Positivo = hacia el frente.
# LIDAR_Y: cuanto a la DERECHA de la linea central esta. Negativo = a la
#          izquierda.
# LIDAR_Z: altura del plano de barrido sobre el piso (README: 90mm).
#
# Estos valores salen de fotogrametria sobre v-photos/Topview.jpeg,
# v-photos/Rightview.jpeg y v-photos/frontview.jpeg, corrigiendo el
# paralaje por altura (ver MEDICIONES.md, seccion 1). Incertidumbre
# +-10mm en x y +-5mm en y. SUSTITUIR por la medicion con regla.
LIDAR_X = 128.0
LIDAR_Y = -4.0
LIDAR_Z = 90.0

# ==========================================
# CAMARA -- LA FUENTE DE VERDAD NO ESTA AQUI
# ==========================================
# Aqui vivian CAMARA_X_REL_LIDAR (47.0, "el lente por DELANTE del eje del
# LiDAR"), ANCHO_FRAME/ALTO_FRAME (320x240) y FOV_H_CAMARA (66.0), y las
# tres cosas eran del montaje ANTERIOR:
#
#   * la camara ya no va delante: va en un mastil ~100 mm DETRAS del
#     LiDAR (optica.RUMBO_MASTIL_DEG = 176, DIST_MASTIL_MM = 100). El
#     signo del paralaje estaba invertido.
#   * el frame es 640x360 desde el modo 2304x1296 (camara_driver.py), no
#     320x240. Los umbrales en pixeles de vision.py estan reescalados a
#     ese tamaño.
#   * el HFOV efectivo se calcula de la focal medida (optica.py):
#     68.2 grados, no los 66 nominales.
#
# Nadie las usaba -- la carrera resuelve toda la optica con optica.py --
# pero eran tres numeros del montaje viejo con nombres creibles. El
# simulador del repo ya avisaba de esta contradiccion en un comentario.
#
# FUENTE UNICA DE LA OPTICA: **optica.py**.

# ==========================================
# DIRECCION -- LA FUENTE DE VERDAD NO ESTA AQUI
# ==========================================
# Este bloque tenia SERVO_TRIM, RADIO_MIN_IZQ/DER y COMANDO_MAX_IZQ/DER,
# los cuatro marcados [PENDIENTE] y los cuatro equivocados:
#
#   RADIO_MIN_IZQ/DER = 400/400   frente a 260/360 en geometria_evasion
#                                 (y 228/260 medidos en el banco del 06-09)
#   COMANDO_MAX_DER   = -20.0     con el signo CAMBIADO respecto al +20.0
#                                 de geometria_evasion, que es el que usa
#                                 la carrera
#
# No los usaba nadie -- la carrera importa los suyos de geometria_evasion
# --, asi que eran sesenta lineas de numeros equivocados esperando a que
# alguien los importara por error. El del signo es el peor: un
# _clamp_servo escrito contra este modulo recortaria al reves.
#
# La fuente unica de los topes y los radios es **geometria_evasion.py**.
# No se reexportan desde aqui a proposito: geometria_evasion importa este
# modulo, y hacerlo al reves crearia un import circular.
#
# PENDIENTE DE MEDIR (no se pierde con el borrado):
#   * radio de giro real por lado, con kd=0 en la consigna a la Pico
#     (`enlace_pico.enviar(v, ang, kd=0.0)`): la amortiguacion por
#     giroscopio del firmware desvia el servo ~2 grados en giro sostenido
#     y falsea el angulo, y los 260/360 se tomaron CON ella activa.
#     Protocolo: girar 90 grados a tope, medir la cuerda, R = cuerda/raiz(2).
#   * lo mismo EN REVERSA, que es la unica entrada geometrica del parqueo
#     que sigue sin medir (inferida ~306 mm, un 34% peor que adelante).
#   * SERVO_TRIM: los limites calibrados (70/90/115) son asimetricos, lo
#     que sugiere que el centro mecanico esta corrido.

# ==========================================
# TRACCION -- LA FUENTE DE VERDAD NO ESTA AQUI
# ==========================================
# Aqui vivian VELOCIDAD_MM_S y VELOCIDAD_MM_S_BATERIA_BAJA (dos tablas
# con TODOS sus valores a None), TAU_ACELERACION (None) y velocidad_mm_s(),
# que devolvia None siempre porque las tablas estaban vacias. Nadie las
# llamaba.
#
# Lo que la carrera usa de verdad es `tracker.MM_POR_SEG_A_PWM100 = 400`,
# coherente con los 3,85 mm/s por punto de PWM medidos en el banco del
# 06-09 (385 mm/s al 100%) y con los 150-200 mm/s observados en video.
#
# PENDIENTE: girando se pierde un 26% de velocidad por el restregado de
# las ruedas (62 mm/s a tope frente a 84 en recta, banco del 06-09), y el
# modelo no lo tiene en cuenta en ningun sitio.

# ==========================================
# TRANSFORMADAS ENTRE MARCOS
# ==========================================
def lidar_a_eje_trasero(x, y):
    # Punto medido por el LiDAR -> marco del eje trasero.
    # Es una traslacion pura: ambos marcos comparten orientacion.
    return (x + LIDAR_Y, y + LIDAR_X)


def eje_trasero_a_lidar(x, y):
    return (x - LIDAR_Y, y - LIDAR_X)


# ==========================================
# PISTA Y PARQUEO  [MEDIDO] - Reglamento WRO FE 2026
# ==========================================
ANCHO_CARRIL = 1000.0    # entre muro exterior e interior
ALTO_MURO    = 100.0
LADO_POSTE   = 50.0      # los traffic signs son 50x50x100mm
DIAM_CIRCULO_POSTE = 200.0

# Regla 13.25: cada limitacion de parqueo es 200 x 20 x 100 mm, magenta
# RGB(255,0,255). Van con su lado de 200mm perpendicular al muro
# exterior, asi que sobresalen 200mm dentro del carril.
LARGO_MURO_PARQUEO      = 200.0
ESPESOR_MURO_PARQUEO    = 20.0
PROFUNDIDAD_PARQUEO     = 200.0   # "fixed width: 20 cm", desde el muro exterior

# Seccion 5 del reglamento + Figura 4: el hueco util se mide entre las
# CARAS INTERNAS de los dos muros magenta y vale 1.5 * largo del robot.
# Como el juez mide TU robot, la holgura longitudinal siempre es
# 0.5 * largo -- un robot mas corto no gana margen absoluto.
LARGO_PARQUEO           = 1.5 * LARGO_ROBOT              # 333.0 mm
HOLGURA_LONGITUDINAL    = (LARGO_PARQUEO - LARGO_ROBOT) / 2.0   # 55.5 mm por extremo
HOLGURA_LATERAL         = (PROFUNDIDAD_PARQUEO - ANCHO_ROBOT) / 2.0  # 37.5 mm por lado

# Apendice A seccion 6: se considera paralelo si la diferencia entre las
# distancias de las dos ruedas de un mismo lado al muro no pasa de 20mm.
# Sobre la batalla eso es un cono de guiñada de +-8.4 grados.
TOLERANCIA_PARALELO_MM  = 20.0
TOLERANCIA_PARALELO_DEG = math.degrees(math.atan(TOLERANCIA_PARALELO_MM / BATALLA))

# Margen de diseño para la maniobra: nunca apuntar al centro exacto, sino
# dejar este colchon contra cada muro magenta. Tocarlos termina la ronda
# (regla 9.24.7), asi que el colchon se paga con puntos, no con tiempo.
MARGEN_MURO_PARQUEO = 25.0


if __name__ == "__main__":
    print("=== Geometria del robot ===")
    print(f"Batalla {BATALLA:.0f}mm | via {VIA:.0f}mm | "
          f"robot {LARGO_ROBOT:.0f}x{ANCHO_ROBOT:.0f}mm")
    print(f"LiDAR   x=+{LIDAR_X:.0f}mm del eje trasero, y={LIDAR_Y:+.0f}mm, z={LIDAR_Z:.0f}mm")
    print(f"Camara  x=+{CAMARA_X_REL_LIDAR:.0f}mm del LiDAR "
          f"(= +{CAMARA_X:.0f}mm del eje trasero)")

    print("\n=== Impacto del offset del LiDAR en el pure pursuit ===")
    print("distancia frontal -> error de rumbo por ignorar LIDAR_X")
    for y in (150, 200, 300, 450, 600, 900):
        print(f"  y={y:4d}mm  ->  {error_bearing_pure_pursuit(y):5.2f} grados por cada "
              f"mm de desplazamiento lateral")
    print("  (para un objetivo a 260mm de lado, multiplicar por 260)")
    for y in (200, 300, 450):
        b_sin = math.degrees(math.atan2(260.0, y))
        b_con = math.degrees(math.atan2(260.0, y + LIDAR_X))
        print(f"  poste a y={y}mm, paso lateral 260mm: "
              f"sin corregir {b_sin:.1f} grados, corregido {b_con:.1f} grados, "
              f"delta {b_sin - b_con:.1f}")

    print("\n=== Parqueo (reglamento 2026) ===")
    print(f"Hueco util entre caras internas: {LARGO_PARQUEO:.1f}mm "
          f"(1.5 x {LARGO_ROBOT:.0f})")
    print(f"Holgura longitudinal por extremo: {HOLGURA_LONGITUDINAL:.1f}mm")
    print(f"Holgura lateral por lado:         {HOLGURA_LATERAL:.1f}mm")
    print(f"Tolerancia de paralelismo:        +-{TOLERANCIA_PARALELO_DEG:.1f} grados "
          f"({TOLERANCIA_PARALELO_MM:.0f}mm sobre la batalla)")
    print(f"Desalineacion maxima que todavia cabe: "
          f"{desalineacion_maxima_admisible():.1f} grados")
    print(f"Firma esperada en el LiDAR: {firma_lidar_parqueo()}")
