# -*- coding: utf-8 -*-
"""
Mascara de oclusion del LiDAR y sectores traseros recalculados.

Desde que la camara se monto en un mastil trasero, el mastil se mete en
el barrido del C1 y lo ciega en un arco fijo. Medido con mapa_oclusion.py
(40 barridos, robot quieto):

    grados 165-187  ->  eco constante a 90-107 mm, dispersion 3-8 mm

Esos 23 grados no son entorno: son el robot mirandose a si mismo. El
problema no es perder resolucion trasera, es que los sectores de
lidar_geometria.py toman el MINIMO del rango, asi que un eco fijo de
~92mm gana siempre y deja tres cosas rotas en navegacion.py:

  - `med.trasera` (rango [170,190], 18 de sus 21 grados tapados) vale
    ~92mm de forma permanente. Como el estado RETROCESO sale en cuanto
    `med.trasera < EMERGENCIA_TRASERA (250mm)`, el retroceso se aborta
    en su primer ciclo pase lo que pase: el robot pierde la maniobra de
    desatasco entera.
  - `med.trasera_derecha` (rango [90,170]) incluye 165-170 y tambien se
    queda clavado en ~104mm. El control P del retroceso usa
    `trasera_derecha - trasera_izquierda`, asi que su error queda
    dominado por una constante del chasis: gira siempre al mismo lado,
    saturado, sin relacion con el espacio libre real.
  - Los grados sin eco NO son mejores: construir_perfil_360 los rellena
    con 8000.0, que significa "via libre". Un sector ciego que se
    reporta despejado es peor que uno que se reporta ocupado.

La correccion no es tapar los bins y ya: hay que recuperar la medida
trasera desde los grados que SI ven. Este modulo hace las dos cosas y
mantiene la simetria izquierda/derecha, que es de lo que depende el
control P del retroceso.

Convenciones heredadas de lidar_geometria: 0 = frente, horario,
perfil = 360 floats (distancia minima por grado).
"""
import math

# ==========================================
# ARCO CIEGO
# ==========================================
# Medido 165-187. Se guarda con 2 grados de margen a cada lado porque los
# bordes (165 y 188) salieron con tasa de eco parcial: el haz roza el
# canto del mastil y unas veces vuelve y otras no, que es el caso que peor
# se comporta (alterna entre 100mm y 8000mm en ciclos seguidos).
MASTIL_MIN = 163
MASTIL_MAX = 189
BINS_CIEGOS = frozenset(range(MASTIL_MIN, MASTIL_MAX + 1))

# Distancia por debajo de la cual un eco es estructura del robot, no pista.
DIST_ESTRUCTURA = 150.0

# Igual que lidar_geometria.SIN_PARED_FRONTAL: "no hay nada medible ahi",
# que no es lo mismo que "hay algo encima".
SIN_DATO = 8000.0

# ==========================================
# SECTORES TRASEROS RECALCULADOS
# ==========================================
# El arco ciego llega hasta 18 grados del eje trasero por el lado derecho
# (180-162) y hasta 9 por el izquierdo (189-180). Se recorta el MISMO
# margen de 18 grados en ambos lados aunque por la izquierda sobre sitio:
# el control P del retroceso resta un sector del otro, y dos sectores de
# ancho distinto meten un sesgo constante hacia el lado mas ancho.
MARGEN_MASTIL = 18

# Diagonales traseras (eran [90,170] y [190,270] en lidar_geometria)
TRASDER_MIN, TRASDER_MAX = 90, 180 - MARGEN_MASTIL          # [ 90, 162]
TRASIZQ_MIN, TRASIZQ_MAX = 180 + MARGEN_MASTIL, 270         # [198, 270]

# Ventanas para la distancia trasera "de frente". El sector original
# [170,190] esta tapado casi entero, asi que se mide por los dos hombros
# que quedan: +-18 a +-35 grados del eje. Se proyecta cada haz sobre el
# eje trasero (d * cos(offset)) para que el numero siga siendo "cuanto
# me falta para tocar la pared de atras" y no la distancia oblicua, que
# es mayor y haria creer que hay mas sitio del que hay.
SEMIANCHO_TRASERA = 35
HOMBRO_DER = (180 - SEMIANCHO_TRASERA, 180 - MARGEN_MASTIL)  # [145, 162]
HOMBRO_IZQ = (180 + MARGEN_MASTIL, 180 + SEMIANCHO_TRASERA)  # [198, 215]


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
    """Distancia a lo que haya detras, proyectada sobre el eje trasero.

    Sustituye a `Medicion.trasera` (rango [170,190]), que con el mastil
    montado vale ~92mm siempre. Mide por los dos hombros que quedan
    visibles y se queda con el minimo proyectado, que es el criterio
    conservador: si un haz oblicuo ve algo a 300mm a 30 grados, lo que
    falta por detras son 300*cos(30) = 260mm, no 300.
    """
    mejor = SIN_DATO
    for a_min, a_max in (HOMBRO_DER, HOMBRO_IZQ):
        for i in _indices(a_min, a_max):
            if i in BINS_CIEGOS:
                continue
            d = perfil[i]
            if d >= SIN_DATO:
                continue
            axial = d * math.cos(math.radians(i - 180))
            if axial < mejor:
                mejor = axial
    return mejor


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
