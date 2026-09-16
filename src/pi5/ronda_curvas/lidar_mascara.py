# -*- coding: utf-8 -*-
"""
Mascara de oclusion del LiDAR y sectores traseros.

El robot se ve a si mismo por detras: el mastil de la camara y el soporte
del ultrasonido se meten en el barrido del C1 y lo ciegan en un arco fijo.
Remedido con `diag_lidar_360.py` el 10-09-2026 (25 barridos, robot quieto):

    grados 136-198  ->  eco a 32-79 mm, tasa 25-100%, dispersion 2-9 mm

Esos 63 grados no son entorno: son el robot mirandose a si mismo. Y el
problema no es perder resolucion trasera, es que los sectores de
lidar_geometria.py toman el MINIMO del rango, asi que un eco fijo de
~45mm gana siempre.

Lo que costo tenerlo mal medido (la version anterior decia 165-187, del
montaje de antes del mastilfix del 05-09), en los CSV del 09-09:

  - `med.trasera` valio 35-50 mm en el 99,9% de 1634 ciclos. Como
    RETROCESO sale en cuanto `med.trasera < EMERGENCIA_TRASERA (250mm)`,
    los 447 episodios de retroceso duraron TODOS 1 ciclo: 16 mm de marcha
    atras por intento. El robot no podia desatascarse ni en principio.
  - Las dos diagonales traseras quedaban clavadas tambien, asi que el
    control P del retroceso (`trasera_derecha - trasera_izquierda`) veia
    un error de cero y retrocedia siempre recto.
  - Los grados sin eco NO son mejores: construir_perfil_360 los rellena
    con 8000.0, que significa "via libre". Un sector ciego que se reporta
    despejado es peor que uno que se reporta ocupado.

La conclusion honesta tras remedirlo es que con este montaje **el LiDAR
no puede medir hacia atras**: no queda ni un grado util a menos de 46
grados del eje trasero. La distancia trasera sale del ULTRASONIDO de la
Pico (campo US de su telemetria), que `enlace_pico.py` descartaba.

Convenciones heredadas de lidar_geometria: 0 = frente, horario,
perfil = 360 floats (distancia minima por grado).
"""
import math

# ==========================================
# ARCO CIEGO
# ==========================================
# REMEDIDO EL 10-09-2026 con `diag_lidar_360.py` (25 barridos, robot
# quieto). Lo que ciega el LiDAR por detras NO es solo el mastil de la
# camara: es el conjunto mastil + soporte del ultrasonido, y ocupa
#
#     grados 136-198  ->  eco a 32-79 mm, tasa 25-100%, dispersion 2-9 mm
#
# El valor anterior (163-189, eco a 90-107 mm) describia el montaje de
# antes del `mastilfix` del 05-09 y dejaba 136-162 y 190-198 SIN TAPAR.
# Como los sectores toman el MINIMO del rango, esos 35 grados de
# estructura a ~45 mm mandaban sobre todo lo demas.
#
# Lo que costo, medido en los CSV del 09-09 (tres corridas, 1634 ciclos):
# `med.trasera` valio 35-50 mm en el 99,9% de los ciclos, y los 447
# episodios de RETROCESO duraron TODOS exactamente 1 ciclo -- salian por
# `trasera < EMERGENCIA_TRASERA` antes de retroceder nada. A -35 PWM y
# 0,1 s eso son 16 mm de marcha atras por intento: el robot no podia
# desatascarse ni en principio.
#
# Se guarda con 1 grado de margen a cada lado: los bordes (135 y 199)
# salieron con tasa de eco parcial, que es el caso que peor se comporta
# (alterna entre 45 mm y 8000 mm en ciclos seguidos).
MASTIL_MIN = 135
MASTIL_MAX = 199
BINS_CIEGOS = frozenset(range(MASTIL_MIN, MASTIL_MAX + 1))

# Distancia por debajo de la cual un eco es estructura del robot, no pista.
DIST_ESTRUCTURA = 150.0

# Igual que lidar_geometria.SIN_PARED_FRONTAL: "no hay nada medible ahi",
# que no es lo mismo que "hay algo encima".
SIN_DATO = 8000.0

# ==========================================
# SECTORES TRASEROS RECALCULADOS
# ==========================================
# El arco ciego llega a 45 grados del eje trasero por el lado derecho
# (180-135) y a 19 por el izquierdo (199-180). Se recorta el MISMO margen
# de 45 en ambos lados aunque por la izquierda sobre sitio: el control P
# del retroceso resta un sector del otro, y dos sectores de ancho
# distinto meten un sesgo constante hacia el lado mas ancho -- que es
# justo el fallo que se persigue.
MARGEN_MASTIL = 45

# Diagonales traseras (eran [90,170] y [190,270] en lidar_geometria)
TRASDER_MIN, TRASDER_MAX = 90, 180 - MARGEN_MASTIL          # [ 90, 135]
TRASIZQ_MIN, TRASIZQ_MAX = 180 + MARGEN_MASTIL, 270         # [225, 270]

# NO HAY VENTANA TRASERA. Con el arco ciego real (135-199), los hombros
# que la version anterior usaba -- [145,162] y [198,215] -- caen DENTRO
# de la estructura: eran los que devolvian 47 mm en todos los ciclos.
#
# La conclusion honesta es que este LiDAR, con este montaje, NO PUEDE
# medir hacia atras: los 45 grados a la derecha del eje trasero estan
# tapados por completo. Inventar una "trasera" a partir de haces
# oblicuos de +-46 grados o mas seria repetir el error con otro numero.
#
# La medida buena existe y esta en el otro sensor: el HC-SR04 que la
# Pico publica en el campo US de su telemetria. Comprobado el 10-09 en
# la misma pose: LiDAR 47 mm, ultrasonido 1085 mm, verdad ~1 m.
# `ronda_camara.py` sobrescribe `med.trasera` con ese valor.



def _indices(ang_min, ang_max):
    i_min, i_max = int(ang_min) % 360, int(ang_max) % 360
    if i_min <= i_max:
        return range(i_min, i_max + 1)
    return list(range(i_min, 360)) + list(range(0, i_max + 1))


def distancia_util(perfil, ang_min, ang_max):
    """Minimo del rango ignorando el arco ciego del mastil.

    Devuelve SIN_DATO si el rango entero cae dentro del arco ciego, para
    no confundir "no lo puedo ver" con "esta libre".
    """
    vals = [perfil[i] for i in _indices(ang_min, ang_max) if i not in BINS_CIEGOS]
    return min(vals) if vals else SIN_DATO


def distancia_trasera(perfil):
    """SIN_DATO, siempre y a proposito: este LiDAR no ve hacia atras.

    Con el arco ciego real (135-199) no queda ni un grado util a menos de
    46 grados del eje trasero, asi que no hay ninguna ventana con la que
    reconstruir la distancia axial. Devolver SIN_DATO es decir "no lo
    puedo ver", que es la verdad; devolver un numero sacado de haces muy
    oblicuos seria repetir el fallo de la version anterior con otra
    cifra.

    Quien mide por detras es el ultrasonido de la Pico (campo US de la
    telemetria). `ronda_camara.py` pisa `med.trasera` con el, y si el
    ultrasonido no responde deja este SIN_DATO -- que la FSM interpreta
    como "no hay obstaculo confirmado detras", no como "hay uno encima".
    """
    return SIN_DATO


def distancia_trasera_derecha(perfil):
    return distancia_util(perfil, TRASDER_MIN, TRASDER_MAX)


def distancia_trasera_izquierda(perfil):
    return distancia_util(perfil, TRASIZQ_MIN, TRASIZQ_MAX)


def aplicar(med):
    """Corrige en sitio los tres campos traseros de una Medicion.

    Pensado para llamarse justo despues de ProcesadorLidar.procesar(),
    de modo que navegacion.py siga leyendo `med.trasera` y compania sin
    enterarse de que el mastil existe. El resto de campos (frontal,
    laterales, clusters) no se tocan: el mastil no los alcanza -- el
    clustering ya descarta 120-240 y los sectores frontal/laterales
    salieron con 0% de grados tapados en la medicion.
    """
    med.trasera           = distancia_trasera(med.perfil)
    med.trasera_derecha   = distancia_trasera_derecha(med.perfil)
    med.trasera_izquierda = distancia_trasera_izquierda(med.perfil)
    return med


def diagnosticar(perfil):
    """Comprueba que el mastil sigue donde se midio.

    La mascara son constantes calibradas contra una pieza fisica: si el
    mastil se afloja, se reorienta o se cambia de sitio, la mascara tapa
    entorno bueno y deja pasar entorno malo, y nada en el sistema se
    entera. Devuelve (ok, mensaje) para poder gritarlo al arrancar.
    """
    dentro = [i for i in BINS_CIEGOS if perfil[i] < DIST_ESTRUCTURA]
    fuera  = [i for i in range(360)
              if i not in BINS_CIEGOS and perfil[i] < DIST_ESTRUCTURA]
    if not dentro:
        return False, ("mascara sospechosa: ningun grado de "
                       f"[{MASTIL_MIN},{MASTIL_MAX}] ve estructura a <{DIST_ESTRUCTURA:.0f}mm. "
                       "Se movio el mastil? Recorrer mapa_oclusion.py.")
    if fuera:
        return False, (f"estructura FUERA de la mascara en los grados {sorted(fuera)} "
                       "-- hay algo mas metido en el barrido. Recorrer mapa_oclusion.py.")
    return True, f"mascara ok: {len(dentro)}/{len(BINS_CIEGOS)} grados del mastil confirmados."
