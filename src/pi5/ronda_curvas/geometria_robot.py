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
# OFFSET DE LA CAMARA RESPECTO AL LIDAR  [ESTIMADO]
# ==========================================
# El paralaje: el lente esta por DELANTE del eje del LiDAR, asi que un
# poste que la camara ve centrado NO esta en el angulo 0 del LiDAR.
# Incertidumbre +-10mm. SUSTITUIR por la medicion con regla.
CAMARA_X_REL_LIDAR = 47.0
CAMARA_Y_REL_LIDAR = 0.0
CAMARA_Z           = 15.0    # el lente va colgado del beam amarillo, muy bajo

CAMARA_X = LIDAR_X + CAMARA_X_REL_LIDAR
CAMARA_Y = LIDAR_Y + CAMARA_Y_REL_LIDAR

# Camara: Pi Camera Module 3 con ScalerCrop de sensor completo
ANCHO_FRAME = 320
ALTO_FRAME  = 240
FOV_H_CAMARA = 66.0      # [PENDIENTE] grados, nominal del Module 3 (no gran angular)

# ==========================================
# DIRECCION  [PENDIENTE] - protocolo en MEDICIONES.md seccion 4
# ==========================================
# SERVO_CENTRO_REAL: el valor de angulo (en la escala de la Pico, donde
# 90 = CENTRO nominal) que de verdad produce marcha recta. Que los
# limites calibrados sean LIMITE_DER=70 / LIMITE_IZQ=115 (asimetricos
# respecto a 90) sugiere que el centro mecanico esta corrido hacia la
# izquierda; el valor de abajo es la correccion a aplicar sobre 90.
SERVO_TRIM = 0.0         # grados a sumar al comando para ir recto

# Radio de giro minimo medido rueda a rueda, con el servo al tope.
# Sirve para dos cosas: saturar el pure pursuit en algo fisicamente
# alcanzable, y convertir un radio deseado en un comando de servo.
RADIO_MIN_IZQ = 400.0    # [PENDIENTE]
RADIO_MIN_DER = 400.0    # [PENDIENTE]

# Tope util del servo en grados de COMANDO (lo que se le manda a la Pico
# sobre el centro). El README dice -20/+25 reales.
COMANDO_MAX_IZQ =  25.0
COMANDO_MAX_DER = -20.0

# ==========================================
# TRACCION  [PENDIENTE] - protocolo en MEDICIONES.md seccion 5
# ==========================================
# Velocidad de crucero estable en mm/s por cada consigna de PWM, con
# bateria llena. tracker.MM_POR_SEG_A_PWM100 = 900.0 es una suposicion
# no medida: esta tabla la reemplaza.
VELOCIDAD_MM_S = {
    40: None,
    55: None,
    70: None,
    90: None,
}
VELOCIDAD_MM_S_BATERIA_BAJA = {
    40: None,
    55: None,
    70: None,
    90: None,
}

# Tiempo en alcanzar el 90% de la velocidad de regimen tras un escalon
# de PWM. Lo necesita el modelo de prediccion del tracker.
TAU_ACELERACION = None   # segundos


def velocidad_mm_s(pwm, bateria_baja=False):
    # Interpolacion lineal sobre la tabla medida. Devuelve None mientras
    # la tabla siga sin llenarse -- el llamador decide que hacer, en vez
    # de propagar un numero inventado.
    tabla = VELOCIDAD_MM_S_BATERIA_BAJA if bateria_baja else VELOCIDAD_MM_S
    puntos = sorted((p, v) for p, v in tabla.items() if v is not None)
    if not puntos:
        return None

    signo = 1.0 if pwm >= 0 else -1.0
    p = abs(pwm)
    if p <= puntos[0][0]:
        return signo * puntos[0][1] * p / puntos[0][0]
    if p >= puntos[-1][0]:
        return signo * puntos[-1][1]
    for (p0, v0), (p1, v1) in zip(puntos, puntos[1:]):
        if p0 <= p <= p1:
            return signo * (v0 + (v1 - v0) * (p - p0) / (p1 - p0))
    return signo * puntos[-1][1]


# ==========================================
# TRANSFORMADAS ENTRE MARCOS
# ==========================================
def lidar_a_eje_trasero(x, y):
    # Punto medido por el LiDAR -> marco del eje trasero.
    # Es una traslacion pura: ambos marcos comparten orientacion.
    return (x + LIDAR_Y, y + LIDAR_X)


def eje_trasero_a_lidar(x, y):
    return (x - LIDAR_Y, y - LIDAR_X)


def bearing_desde_eje_trasero(x_lidar, y_lidar):
    # Rumbo (grados, positivo = a la derecha) hacia un punto que el LiDAR
    # reporto en (x_lidar, y_lidar), visto DESDE EL EJE TRASERO.
    #
    # Esta es la correccion que faltaba en el pure pursuit: navegacion.py
    # calcula atan2(x_obj, y_obj) con las coordenadas crudas del LiDAR,
    # que estan adelantadas LIDAR_X mm. A 300mm de distancia eso son
    # varios grados de error de rumbo, y el error crece cuanto mas cerca
    # esta el poste -- justo cuando mas importa.
    xr, yr = lidar_a_eje_trasero(x_lidar, y_lidar)
    return math.degrees(math.atan2(xr, max(1.0, yr)))


def error_bearing_pure_pursuit(y_lidar):
    # Cuanto se equivoca el pure pursuit si ignora LIDAR_X, para un
    # objetivo con desplazamiento lateral unitario a distancia y_lidar.
    # Util para justificar la correccion con numeros en la bitacora.
    if y_lidar <= 0:
        return float("nan")
    sin_correccion = math.degrees(math.atan2(1.0, y_lidar))
    con_correccion = math.degrees(math.atan2(1.0, y_lidar + LIDAR_X))
    return sin_correccion - con_correccion


def camara_cx_a_bearing(cx):
    # Centroide horizontal de un contorno (px) -> rumbo en grados en el
    # MARCO DE LA CAMARA. vision.poste_cx entrega este cx.
    f_px = (ANCHO_FRAME / 2.0) / math.tan(math.radians(FOV_H_CAMARA / 2.0))
    return math.degrees(math.atan2(cx - ANCHO_FRAME / 2.0, f_px))


def camara_bearing_a_lidar(bearing_cam, distancia_mm):
    # Corrige el paralaje: un poste visto a bearing_cam grados por la
    # camara, a distancia_mm del lente, esta en este otro angulo para el
    # LiDAR. Sin esto la asociacion camara<->cluster falla en corto.
    ang = math.radians(bearing_cam)
    x = distancia_mm * math.sin(ang) + CAMARA_Y_REL_LIDAR
    y = distancia_mm * math.cos(ang) + CAMARA_X_REL_LIDAR
    return math.degrees(math.atan2(x, max(1.0, y)))


# ==========================================
# CINEMATICA DE BICICLETA (Ackermann)
# ==========================================
def radio_a_angulo_rueda(radio_mm):
    # Angulo de la rueda virtual del modelo de bicicleta para un radio de
    # giro dado, medido en el eje trasero: tan(delta) = BATALLA / R
    if radio_mm is None or abs(radio_mm) < 1.0:
        return 0.0
    return math.degrees(math.atan(BATALLA / radio_mm))


def angulo_rueda_a_radio(delta_grados):
    if abs(delta_grados) < 0.05:
        return float("inf")
    return BATALLA / math.tan(math.radians(delta_grados))


def angulo_rueda_max_medido(lado="IZQ"):
    # Angulo de rueda equivalente al radio minimo medido en pista. Es el
    # tope fisico real, no el tope de comando del servo.
    r = RADIO_MIN_IZQ if lado == "IZQ" else RADIO_MIN_DER
    return radio_a_angulo_rueda(r)


def curvatura_maxima():
    return 1.0 / min(RADIO_MIN_IZQ, RADIO_MIN_DER)


def velocidad_angular_esperada(pwm, delta_grados, bateria_baja=False):
    # Grados/s de guiñada que deberia reportar la IMU si el modelo es
    # correcto. Comparar contra la IMU real es la validacion cruzada mas
    # barata de todo el modelo cinematico.
    v = velocidad_mm_s(pwm, bateria_baja)
    if v is None:
        return None
    r = angulo_rueda_a_radio(delta_grados)
    if r == float("inf"):
        return 0.0
    return math.degrees(v / r)


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


def firma_lidar_parqueo():
    # Como se ve el parqueo desde el LiDAR cuando el robot pasa por el
    # carril: dos muros de 200mm perpendiculares al muro exterior,
    # separados LARGO_PARQUEO entre caras internas. A ese lado la
    # distancia lateral cae de ANCHO_CARRIL a (ANCHO_CARRIL - 200) en dos
    # escalones, con un valle de LARGO_PARQUEO mm entre ellos.
    #
    # Es una firma mucho mas especifica que el "match de firma de pared"
    # actual (navegacion.TOLERANCIA_FIRMA), que solo compara dos
    # distancias laterales contra las del arranque.
    return {
        "salto_lateral_mm":      LARGO_MURO_PARQUEO,
        "separacion_valle_mm":   LARGO_PARQUEO,
        "distancia_al_muro_mm":  ANCHO_CARRIL - LARGO_MURO_PARQUEO,
    }


def cabe_el_robot_parqueado(desalineacion_grados):
    # Proyeccion del robot sobre el mat cuando queda con esa desviacion
    # angular respecto al muro. Si excede el hueco disponible, la
    # maniobra no puede aceptar esa desalineacion.
    a = abs(math.radians(desalineacion_grados))
    largo_proyectado = LARGO_ROBOT * math.cos(a) + ANCHO_ROBOT * math.sin(a)
    ancho_proyectado = LARGO_ROBOT * math.sin(a) + ANCHO_ROBOT * math.cos(a)
    return (largo_proyectado <= LARGO_PARQUEO
            and ancho_proyectado <= PROFUNDIDAD_PARQUEO), largo_proyectado, ancho_proyectado


def desalineacion_maxima_admisible():
    # Barrido de 0 a 45 grados en pasos de 0.1: el ultimo angulo con el
    # que la proyeccion todavia cabe entre los dos muros magenta.
    peor = 0.0
    for i in range(451):
        ang = i * 0.1
        cabe, _, _ = cabe_el_robot_parqueado(ang)
        if not cabe:
            break
        peor = ang
    return peor


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
