# Interpretacion geometrica de un barrido crudo del LiDAR (ver lidar_driver.py
# para el protocolo/hilo). Construye un perfil de distancia minima en los
# 360 grados completos en cada ciclo (1 bin por grado) y todo lo demas
# (sectores de pared, diagonales traseras, modo Inercial) se deriva de ese
# perfil -- no hay sectores calculados por separado con su propio loop.
# Tambien hace clustering ABD para separar postes de paredes. No sabe nada
# del puerto serial ni del protocolo binario del C1.
#
# Convenciones: 0 grados = frente, los angulos crecen en sentido horario.
# Cartesianas: x+ = derecha, y+ = frente (en mm).
import time
import math
import threading

# ==========================================
# PERFIL 360 GRADOS (1 bin por grado)
# ==========================================
NUM_BINS       = 360
GRADOS_POR_BIN = 360.0 / NUM_BINS

# ==========================================
# SECTORES DE PARED Y DIAGONALES TRASERAS (grados)
# Las diagonales traseras cubren el hueco entre "derecha"/"izquierda" y
# "trasera" -- las usa el retroceso de emergencia para saber de que lado
# hay mas espacio libre en vivo (ver navegacion.py, estado RETROCESO).
# ==========================================
ANGULO_MIN_DER = 30
ANGULO_MAX_DER = 90
ANGULO_MIN_IZQ = 270
ANGULO_MAX_IZQ = 330
ANGULO_MIN_TRAS = 170
ANGULO_MAX_TRAS = 190

ANGULO_MIN_TRASDER = 90
ANGULO_MAX_TRASDER = 170
ANGULO_MIN_TRASIZQ = 190
ANGULO_MAX_TRASIZQ = 270

# Haces perpendiculares y diagonales para calculo cinematico de guiñada
ANGULO_MIN_PERP_DER = 80
ANGULO_MAX_PERP_DER = 100
ANGULO_MIN_PERP_IZQ = 260
ANGULO_MAX_PERP_IZQ = 280

ANGULO_MIN_DIAG_DER = 40
ANGULO_MAX_DIAG_DER = 50
ANGULO_MIN_DIAG_IZQ = 310
ANGULO_MAX_DIAG_IZQ = 320

# Sector frontal por defecto (350 -> 10, cruza el 0). La navegacion lo
# ensancha durante la evasion para no perder el poste al girar.
SECTOR_FRONTAL_NORMAL = (350.0, 10.0)

# Por encima de esto la pared se da por perdida y se sostiene el ultimo
# valor valido (modo Inercial). Saltar a un valor fijo daba giros bruscos
# en las curvas cerradas.
DIST_PARED_VALIDA_MAX = 4000.0

# Valor de "no hay pared frontal medible" (ver distancia_en_rango_sin_bins).
# Deliberadamente grande: significa via libre, no pared lejana.
SIN_PARED_FRONTAL = 8000.0

# ==========================================
# CLUSTERING ABD (Adaptive Breakpoint Detection)
# Si el salto radial entre dos puntos seguidos supera r*FACTOR + OFFSET,
# ahi se corta el cluster. Calibrado para el C1 (~15mm de ruido).
# ==========================================
ABD_FACTOR           = 0.04
ABD_OFFSET           = 40.0    # mm
MIN_PUNTOS_CLUSTER   = 3
MAX_PUNTOS_OBSTACULO = 30      # un poste de 10cm no genera mas de 30 puntos

DIST_MAX_OBSTACULO    = 1200.0  # mm, postes mas lejos no interesan todavia
EXT_ANG_MAX_OBSTACULO = 15.0    # grados, arco maximo de un poste de 10cm

# Ancho fisico maximo (longitud de arco, mm) para dar un cluster por
# "objeto estrecho" y no por pared. Un poste del reglamento mide 100mm;
# 260mm deja margen de sobra para el ruido del C1 y para el ensanche por
# el propio haz, sin llegar a admitir un tramo de muro (que a 500mm ya
# pasa de 500mm de arco en cuanto ocupa 60 grados). Ver es_objeto_estrecho.
ANCHO_MAX_OBJETO_MM = 260.0

# Tope ANGULAR para es_objeto_estrecho, ademas del tope de ancho fisico.
#
# Sin esto el filtro se rompe de cerca, y de la peor manera posible. El
# ancho se mide como arco = radianes * distancia, asi que el mismo arco
# en milimetros cubre cada vez mas GRADOS segun te acercas: a 130mm, un
# cluster de 114 grados sigue midiendo menos de 260mm de arco y pasa por
# "poste". O sea que pegado a una pared, la pared entera se clasifica
# como objeto estrecho.
#
# Y eso no es un detalle cosmetico: los bins de los objetos estrechos se
# EXCLUYEN de frontal_muro, asi que cuando la pared entera cuenta como
# poste no queda ningun bin y frontal_muro devuelve SIN_PARED_FRONTAL
# (8000) = "via libre". Medido en la corrida del 2026-08-29: los 27
# ciclos en que frontal_muro dijo 8000 tenian la pared a 112-197mm
# (mediana 134). Nunca dijo "libre" habiendo sitio: solo lo decia justo
# antes de chocar, y ahi apaga la asistencia de esquina y el escape
# frontal, que son los dos que consumen frontal_muro.
#
# 30 grados deja de sobra para un poste real: uno de 50mm de ancho
# subtiende 14 grados a 200mm y 19 a 150mm. Y rechaza la pared de cerca,
# que es lo que se busca.
EXT_ANG_MAX_OBJETO = 30.0

# Arco que se excluye del clustering: solo lo que el propio robot tapa.
# Se toma de lidar_mascara (135-199) con un grado de holgura a cada lado,
# escrito aqui como numero para no crear un import circular -- si se toca
# alla, hay que tocarlo aqui, y el test de sectores lo comprueba.
LIMITE_CLUSTER_MIN = 134.0
LIMITE_CLUSTER_MAX = 200.0

# ==========================================
# ANGULO DE PARED POR AJUSTE DE RECTA
# ==========================================
# Sustituye a la triangulacion perp+diag (dos MINIMOS de sector por un
# 0.7071 fijo). Por que se cambio, medido el 10-09 sobre barridos
# sinteticos con la pose CONOCIDA:
#
#   robot paralelo y centrado en el carril  ->  la formula vieja daba -18.0
#   yaw real  0 -> media -17.97 | yaw real +5 -> +4.50   (salto de 22 grados
#                                                         justo en el cero)
#
# Y en las tres corridas del 09-09, |angulo_muro| tuvo mediana 28.5 grados
# y saturo el limite de credibilidad (+-12) en el 91% de los ciclos. El
# numero entra en `lateral_predicho`, que es la restriccion DURA de pared:
# con la pared a 100mm, +12 deja 16 comandos admisibles y -12 deja CERO.
#
# Las dos causas de la formula vieja:
#   1. Aplicaba cos(45) al minimo de un sector de 40-50 grados, que en una
#      pared plana cae en 50, no en 45. Sesgo fijo de 4.8 grados por lado
#      (se cancelan solo si las DOS paredes son planas y validas).
#   2. No comprobaba que los dos haces tocaran LA MISMA pared. En un
#      carril de 1000mm el haz diagonal se pasa de largo el final del muro
#      interior bastante antes de la esquina y aterriza en el muro de
#      enfrente, a 3536mm -- que DIST_PARED_VALIDA_MAX (4000) acepta.
#
# El ajuste de recta no tiene ninguno de los dos problemas: usa TODOS los
# puntos del sector, mide la direccion real de la pared y trae su propio
# criterio de "esto no es una pared" (el residuo). Es el mismo metodo que
# ya usaba `ronda_nueva/percepcion_lidar.ajustar_recta`.
SECTOR_MURO_DER = (35, 115)
SECTOR_MURO_IZQ = (245, 325)

# Un muro del carril no esta a mas de esto. Por encima el eco es de otra
# pared del anillo y no dice nada del pasillo que se esta siguiendo.
DIST_MURO_MAX = 1600.0

# Menos puntos que esto no definen una recta con el ruido del C1.
MIN_PUNTOS_MURO = 8

# Residuo medio por encima del cual el conjunto NO es una pared plana:
# es una esquina, un pilar sin filtrar, o dos paredes a la vez. Se
# devuelve None -- sin dato -- en vez de un angulo inventado.
RESIDUO_MAX_MURO = 25.0

# Recorte de atipicos: se descartan los puntos a mas de este multiplo del
# residuo medio y se reajusta una vez.
RECORTE_RESIDUO = 2.5

# Tope del angulo que se acepta como "pared del carril".
#
# El residuo comprueba que los puntos formen una RECTA, pero no que esa
# recta sea la pared de al lado. El sector derecho (35-115) mira tambien
# hacia delante-derecha, asi que acercandose a una esquina puede quedar
# dominado por la pared FRONTAL -- que es igual de plana, da residuo
# bajo, y su direccion es perpendicular al carril: sale a +-90 grados.
#
# Medido en la corrida del 10-09: con el robot centrado en un carril de
# 1040 mm (izq 522, der 518), el ajuste daba mediana 39 grados y p90
# 86.7. No era ruido: era el ajuste enganchandose a la pared de enfrente.
# En la pose estatica, donde el sector derecho SI ve pared lateral, el
# mismo codigo daba +0.01 con residuo 2.8 mm.
#
# 45 grados separa las dos cosas sin discutir: una guiñada real dentro
# del carril no pasa de ahi (y el consumidor la acota a 12 de todas
# formas), mientras que una pared frontal cae siempre cerca de 90.
# Por encima del tope se devuelve None -- sin dato -- que es lo que
# apaga la deriva y la asistencia en vez de meter un numero grande en
# una restriccion dura.
ANGULO_MURO_MAX = 45.0


class Medicion:
    # Resultado de un barrido completo
    __slots__ = ("frontal", "frontal_muro", "izquierda", "derecha", "trasera",
                 "trasera_derecha", "trasera_izquierda",
                 "clusters_obstaculo", "perfil", "timestamp",
                 "d_perp_izq", "d_perp_der", "d_diag_izq", "d_diag_der", "angulo_muro",
                 "muro_valido", "angulo_muro_viejo", "clusters_estrechos")

    def __init__(self, frontal, izquierda, derecha, trasera,
                 trasera_derecha, trasera_izquierda, clusters, perfil,
                 d_perp_izq=2000.0, d_perp_der=2000.0, d_diag_izq=2000.0, d_diag_der=2000.0,
                 angulo_muro=0.0, frontal_muro=None, muro_valido=False,
                 angulo_muro_viejo=0.0, clusters_estrechos=None):
        self.frontal   = frontal
        # Distancia a la PARED de enfrente, ignorando los postes que
        # tapan el sector (ver distancia_en_rango_sin_bins). Sin este
        # dato la navegacion cae a `frontal`, que es lo que hacia antes.
        self.frontal_muro = frontal if frontal_muro is None else frontal_muro
        self.izquierda = izquierda
        self.derecha   = derecha
        self.trasera   = trasera
        self.trasera_derecha   = trasera_derecha
        self.trasera_izquierda = trasera_izquierda
        self.clusters_obstaculo = clusters
        # Clusters que pasan el filtro por ANCHO FISICO (es_objeto_estrecho).
        # Es una lista distinta de clusters_obstaculo a proposito: aquella
        # acota el arco a 15 grados, y un poste de 50mm supera esos 15
        # grados a partir de ~250mm (por su diagonal, ~260mm), o sea que
        # DEJA DE EXISTIR justo cuando la maniobra lo esta rodeando.
        # Medido por el equipo en su dia: el pilar se apagaba para la
        # percepcion a una mediana de 216mm. En las corridas del 10-09,
        # 3 de 4 pilares acabaron en ABORTO por "estimacion perdida" con
        # sigma de 159-169mm -- o sea por quedarse 3s sin asociar, no por
        # incertidumbre. El tracker se alimenta de esta lista.
        self.clusters_estrechos = clusters_estrechos or []
        self.perfil = perfil    # 360 floats, perfil[i] = distancia min en el grado i
        self.timestamp = time.time()
        self.d_perp_izq = d_perp_izq
        self.d_perp_der = d_perp_der
        self.d_diag_izq = d_diag_izq
        self.d_diag_der = d_diag_der
        self.angulo_muro = angulo_muro
        # False = ningun sector lateral es una pared plana ajustable, asi
        # que `angulo_muro` vale 0.0 por FALTA DE DATO, no porque el
        # robot este alineado. Quien lo consume tiene que distinguirlo:
        # la asistencia de esquina se apaga y la deriva del predictor se
        # va a cero, en vez de creerse un numero que nadie midio.
        self.muro_valido = muro_valido
        # SOLO DIAGNOSTICO, no lo consume nadie: la triangulacion vieja
        # (perp+diag por cos45) calculada sobre el MISMO barrido, para
        # poder comparar las dos formulas en cada pose de una corrida
        # real. Con las dos paredes planas y visibles las dos coinciden
        # -- los sesgos de +-4.8 grados se cancelan -- asi que el caso
        # que las separa es la aproximacion a esquina, y ese solo se
        # muestrea conduciendo. Quitar esta columna cuando este zanjado.
        self.angulo_muro_viejo = angulo_muro_viejo


def construir_perfil_360(scan):
    # Distancia minima por cada grado del circulo completo
    perfil = [8000.0] * NUM_BINS
    for ang, dist in scan:
        i = int(ang / GRADOS_POR_BIN) % NUM_BINS
        if dist < perfil[i]:
            perfil[i] = dist
    return perfil


def _indices_de_rango(ang_min, ang_max):
    # Bins que cubre un rango angular, soportando el cruce por el 0
    # (ej 350 -> 10, como el sector frontal por defecto).
    i_min = int(ang_min / GRADOS_POR_BIN) % NUM_BINS
    i_max = int(ang_max / GRADOS_POR_BIN) % NUM_BINS
    if i_min <= i_max:
        return range(i_min, i_max + 1)
    return list(range(i_min, NUM_BINS)) + list(range(0, i_max + 1))


def distancia_en_rango(perfil, ang_min, ang_max):
    # Minima distancia entre ang_min y ang_max.
    i_min = int(ang_min / GRADOS_POR_BIN) % NUM_BINS
    i_max = int(ang_max / GRADOS_POR_BIN) % NUM_BINS
    if i_min <= i_max:
        return min(perfil[i_min:i_max + 1])
    return min(min(perfil[i_min:]), min(perfil[:i_max + 1]))


def bins_de_clusters(clusters):
    # Bins del perfil ocupados por los clusters dados
    ocupados = set()
    for cluster in clusters:
        for ang_deg, _ in cluster:
            ocupados.add(int(ang_deg / GRADOS_POR_BIN) % NUM_BINS)
    return ocupados


def extension_angular(cluster):
    # Arco que ocupa el cluster, en grados, tolerando el cruce por el 0.
    ext = cluster[-1][0] - cluster[0][0]
    if ext < 0:
        ext += 360.0
    return ext


def ancho_cluster(cluster):
    # Ancho FISICO del cluster en mm: arco_en_radianes * distancia. Da
    # ~100mm para un poste a cualquier distancia y varios cientos para un
    # tramo de muro. Lo usa el tracker como puerta de asociacion (un
    # objeto de 50mm no se convierte en una esquina de muro de un ciclo
    # al siguiente) ademas de es_objeto_estrecho.
    return math.radians(extension_angular(cluster)) * min(p[1] for p in cluster)


def es_objeto_estrecho(cluster):
    # Filtro por ancho FISICO (longitud de arco), no angular.
    #
    # es_cluster_obstaculo() acota el arco en grados (EXT_ANG_MAX_OBSTACULO
    # = 15), y eso solo vale de lejos: un poste de 100mm subtiende 15
    # grados a 380mm, pero 25.6 grados a 220mm. O sea que el poste deja
    # de reconocerse justo cuando esta encima -- que es cuando tapa el
    # sector frontal. Por eso este filtro es aparte y mide milimetros:
    # ancho = arco_en_radianes * distancia, que da ~100mm para un poste
    # a cualquier distancia y varios cientos para un tramo de muro.
    #
    # No se toca es_cluster_obstaculo(): la evasion depende de ese
    # criterio y hoy funciona (8 evasiones correctas en la corrida del
    # README 8.5). Este filtro solo decide que bins ignora el control de
    # pared, no que persigue la FSM.
    ext_ang = extension_angular(cluster)
    d_min = min(p[1] for p in cluster)
    ancho_mm = ancho_cluster(cluster)
    return (ancho_mm <= ANCHO_MAX_OBJETO_MM
            and d_min < DIST_MAX_OBSTACULO
            and ext_ang <= EXT_ANG_MAX_OBJETO)


def distancia_en_rango_sin_bins(perfil, ang_min, ang_max, excluidos):
    # Como distancia_en_rango pero ignorando los bins indicados. Se usa
    # para separar "pared de frente" de "poste de frente": el perfil
    # guarda el minimo por bin, asi que un poste tapa la pared que tiene
    # detras y ese bin no dice nada de donde esta la pared.
    #
    # Si TODOS los bins del sector estan tapados por postes no hay pared
    # medible, que no es lo mismo que tenerla encima: se devuelve el
    # valor de "sin pared a la vista" para que el control no reaccione a
    # un poste como si fuera un muro (los postes los maneja la FSM de
    # evasion, ver navegacion.py).
    vals = [perfil[i] for i in _indices_de_rango(ang_min, ang_max)
            if i not in excluidos]
    return min(vals) if vals else SIN_PARED_FRONTAL


def puntos_de_sector(perfil, ang_min, ang_max, excluidos=frozenset(),
                     d_max=DIST_MURO_MAX):
    # (x, y) en mm de los bins con eco util del sector. x+ = derecha,
    # y+ = frente. `excluidos` son los bins de objetos estrechos: la
    # misma proteccion que ya tiene frontal_muro, que hasta ahora el
    # angulo de pared no tenia -- un pilar en la diagonal se ajustaba
    # como si fuera el muro.
    pts = []
    for i in _indices_de_rango(ang_min, ang_max):
        if i in excluidos:
            continue
        d = perfil[i]
        if d <= 1.0 or d >= d_max:
            continue
        r = math.radians(i)
        pts.append((d * math.sin(r), d * math.cos(r)))
    return pts


def _pca(pts):
    # Direccion principal de la nube y residuo medio a esa recta.
    # PCA y no minimos cuadrados sobre una variable: una pared vista de
    # costado es casi vertical en el marco del robot y cualquier ajuste
    # y = a*x + b revienta ahi.
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = syy = sxy = 0.0
    for x, y in pts:
        dx, dy = x - mx, y - my
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    # Autovector mayor de [[sxx,sxy],[sxy,syy]], en forma cerrada
    tr, det = sxx + syy, sxx * syy - sxy * sxy
    disc = max(0.0, tr * tr / 4.0 - det)
    lam = tr / 2.0 + math.sqrt(disc)
    if abs(sxy) > 1e-9:
        ux, uy = lam - syy, sxy
    elif sxx >= syy:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = 0.0, 1.0
    norma = math.hypot(ux, uy)
    if norma < 1e-9:
        return None
    ux, uy = ux / norma, uy / norma
    nx, ny = -uy, ux                      # normal a la recta
    residuos = [abs((x - mx) * nx + (y - my) * ny) for x, y in pts]
    return ux, uy, sum(residuos) / n, residuos


def angulo_de_pared(pts):
    """Guiñada del chasis respecto a una pared, en grados, o None.

    Positivo = el robot esta girado a la IZQUIERDA respecto a la pared,
    que es el mismo convenio que tenia la triangulacion anterior (lo usa
    _rumbo_nominal con el signo cambiado, y lateral_predicho como deriva).

    Devuelve None cuando el conjunto no es una pared plana. Ese None es
    la mitad del arreglo: antes, un sector que miraba a una esquina
    producia un numero con la misma pinta que uno bueno.
    """
    if len(pts) < MIN_PUNTOS_MURO:
        return None
    r = _pca(pts)
    if r is None:
        return None
    ux, uy, residuo, residuos = r

    # Un recorte de atipicos y reajuste. Salva el caso comun de una pared
    # buena con dos o tres ecos sueltos de otra cosa.
    if residuo > 1.0:
        corte = RECORTE_RESIDUO * residuo
        limpios = [p for p, res in zip(pts, residuos) if res <= corte]
        if len(limpios) >= MIN_PUNTOS_MURO and len(limpios) < len(pts):
            r2 = _pca(limpios)
            if r2 is not None:
                ux, uy, residuo, _ = r2

    if residuo > RESIDUO_MAX_MURO:
        return None
    if uy < 0.0:                          # direccion siempre hacia delante
        ux, uy = -ux, -uy
    ang = math.degrees(math.atan2(ux, uy))
    if abs(ang) > ANGULO_MURO_MAX:
        # Recta plana pero transversal al carril: es la pared de enfrente
        # metida en el sector lateral, no la de al lado. Ver ANGULO_MURO_MAX.
        return None
    return ang


def _angulo_muro_triangulado(d_perp_der, d_perp_izq, d_diag_der, d_diag_izq):
    # La formula ANTERIOR, conservada solo para poder compararla contra
    # el ajuste de recta sobre los mismos barridos. No la usa el control.
    ad = ai = 0.0
    if d_perp_der < DIST_PARED_VALIDA_MAX and d_diag_der < DIST_PARED_VALIDA_MAX:
        dx, dy = d_diag_der * 0.7071 - d_perp_der, d_diag_der * 0.7071
        if dy > 1.0:
            ad = math.degrees(math.atan2(dx, dy))
    if d_perp_izq < DIST_PARED_VALIDA_MAX and d_diag_izq < DIST_PARED_VALIDA_MAX:
        dx, dy = d_perp_izq - d_diag_izq * 0.7071, d_diag_izq * 0.7071
        if dy > 1.0:
            ai = math.degrees(math.atan2(dx, dy))
    if d_perp_der < DIST_PARED_VALIDA_MAX and d_perp_izq < DIST_PARED_VALIDA_MAX:
        return (ad + ai) / 2.0
    if d_perp_der < DIST_PARED_VALIDA_MAX:
        return ad
    if d_perp_izq < DIST_PARED_VALIDA_MAX:
        return ai
    return 0.0


def centroide_xy_cluster(cluster):
    # Centroide cartesiano del cluster. x+ = derecha, y+ = frente (mm)
    sx, sy = 0.0, 0.0
    for ang_deg, dist_mm in cluster:
        ang_rad = math.radians(ang_deg)
        sx += dist_mm * math.sin(ang_rad)
        sy += dist_mm * math.cos(ang_rad)
    n = len(cluster)
    return sx / n, sy / n


# Hueco angular maximo entre el final y el principio del barrido para
# darlos por el mismo objeto al cerrar el circulo. El C1 entrega ~1 punto
# por grado; 3 grados tolera algun punto perdido sin unir objetos
# distintos.
HUECO_CIERRE_GRADOS = 3.0


def segmentar_clusters_abd(scan):
    # ATENCION AL CIERRE DEL CIRCULO. lidar_driver.py corta el barrido
    # justo en el wrap-around del angulo, asi que el scan SIEMPRE empieza
    # cerca de 0 grados y termina cerca de 360. Un objeto centrado en el
    # frente cae partido en dos trozos, uno al final de la lista y otro
    # al principio, y ninguno de los dos llega a MIN_PUNTOS_CLUSTER.
    #
    # Sin cerrar el circulo, el caso que se rompe es el MAS importante:
    # el pilar que se tiene justo delante. Deja de reconocerse como
    # obstaculo (el tracker no lo puede capturar) y deja de excluirse de
    # frontal_muro (el control de pared lo trata como un muro, y con el
    # se disparan la asistencia de esquina y el limite frontal). El
    # comentario de es_cluster_obstaculo ya daba por hecho que existian
    # clusters cruzando el 0; con la segmentacion lineal no existia
    # ninguno.
    if len(scan) < 2:
        return []
    bruto, actual = [], [scan[0]]
    for i in range(1, len(scan)):
        r_prev, r_curr = scan[i - 1][1], scan[i][1]
        if abs(r_curr - r_prev) <= r_prev * ABD_FACTOR + ABD_OFFSET:
            actual.append(scan[i])
        else:
            bruto.append(actual)
            actual = [scan[i]]
    bruto.append(actual)

    # Cierre del circulo: se une la cola con la cabeza si son contiguas
    # en angulo y compatibles en radio. El orden resultante (cola y luego
    # cabeza) es el que hace que extension_angular() de el arco correcto.
    if len(bruto) >= 2:
        cola, cabeza = bruto[-1], bruto[0]
        hueco = (cabeza[0][0] + 360.0) - cola[-1][0]
        r_a, r_b = cola[-1][1], cabeza[0][1]
        if (hueco <= HUECO_CIERRE_GRADOS
                and abs(r_b - r_a) <= r_a * ABD_FACTOR + ABD_OFFSET):
            bruto[0] = cola + cabeza
            bruto.pop()

    return [c for c in bruto if len(c) >= MIN_PUNTOS_CLUSTER]


def es_cluster_obstaculo(cluster):
    # Firma geometrica de un poste de ~10cm de diametro
    n = len(cluster)
    ext_ang = cluster[-1][0] - cluster[0][0]
    if ext_ang < 0:                       # cluster que cruza el 0 (355 -> 5)
        ext_ang += 360.0
    dist_min = min(p[1] for p in cluster)
    return (MIN_PUNTOS_CLUSTER <= n <= MAX_PUNTOS_OBSTACULO
            and ext_ang < EXT_ANG_MAX_OBSTACULO
            and dist_min < DIST_MAX_OBSTACULO)


class ProcesadorLidar:
    # Convierte barridos crudos (de lidar_driver.LidarDriver) en Medicion.
    # Mantiene el estado de interpretacion: sector frontal vigente (lo
    # reconfigura la FSM de evasion) y el modo Inercial de cada pared.
    def __init__(self):
        self._lock_sector    = threading.Lock()
        self._sector_frontal = SECTOR_FRONTAL_NORMAL

        # Ultimo valor valido de cada pared para el modo Inercial
        self._ultima_der = 2000.0
        self._ultima_izq = 2000.0

    def fijar_sector_frontal(self, a_min, a_max):
        with self._lock_sector:
            self._sector_frontal = (float(a_min), float(a_max))

    def sector_frontal_normal(self):
        self.fijar_sector_frontal(*SECTOR_FRONTAL_NORMAL)

    def procesar(self, scan):
        with self._lock_sector:
            sector_frontal = self._sector_frontal

        perfil = construir_perfil_360(scan)

        d_front     = distancia_en_rango(perfil, *sector_frontal)
        d_der       = distancia_en_rango(perfil, ANGULO_MIN_DER, ANGULO_MAX_DER)
        d_izq       = distancia_en_rango(perfil, ANGULO_MIN_IZQ, ANGULO_MAX_IZQ)
        d_tras      = distancia_en_rango(perfil, ANGULO_MIN_TRAS, ANGULO_MAX_TRAS)
        d_tras_der  = distancia_en_rango(perfil, ANGULO_MIN_TRASDER, ANGULO_MAX_TRASDER)
        d_tras_izq  = distancia_en_rango(perfil, ANGULO_MIN_TRASIZQ, ANGULO_MAX_TRASIZQ)

        d_perp_der  = distancia_en_rango(perfil, ANGULO_MIN_PERP_DER, ANGULO_MAX_PERP_DER)
        d_perp_izq  = distancia_en_rango(perfil, ANGULO_MIN_PERP_IZQ, ANGULO_MAX_PERP_IZQ)
        d_diag_der  = distancia_en_rango(perfil, ANGULO_MIN_DIAG_DER, ANGULO_MAX_DIAG_DER)
        d_diag_izq  = distancia_en_rango(perfil, ANGULO_MIN_DIAG_IZQ, ANGULO_MAX_DIAG_IZQ)

        # Modo Inercial en las paredes laterales
        if d_der < DIST_PARED_VALIDA_MAX:
            self._ultima_der = d_der
        else:
            d_der = self._ultima_der
        if d_izq < DIST_PARED_VALIDA_MAX:
            self._ultima_izq = d_izq
        else:
            d_izq = self._ultima_izq

        # Clustering en todo lo que NO esta fisicamente ciego. El arco
        # excluido era 120-240 -- "la trasera no hace falta y ahorra CPU
        # en la Pi 3B" -- y eso dejaba fuera 45 grados de barrido bueno a
        # cada lado del arco ciego real, que son 135-199 (lidar_mascara,
        # remedido el 10-09).
        #
        # No era gratis: el tracker se alimenta de estos clusters, asi que
        # un pilar dejaba de tener candidatos en cuanto su rumbo pasaba de
        # 120 grados -- con el poste todavia al costado, no detras. En la
        # corrida de las 10:23, los 74 ciclos con `sin_candidatos` tenian
        # TODOS el pilar entre 121 y 170 grados. En una Pi 5 el coste de
        # esos 45 grados no se nota.
        scan_relevante = [p for p in scan
                          if not (LIMITE_CLUSTER_MIN < p[0] < LIMITE_CLUSTER_MAX)]
        todos = segmentar_clusters_abd(scan_relevante)
        clusters = [c for c in todos if es_cluster_obstaculo(c)]
        estrechos = [c for c in todos if es_objeto_estrecho(c)]
        bins_estrechos = bins_de_clusters(estrechos)

        # Guiñada respecto a las paredes, por AJUSTE DE RECTA sobre todos
        # los puntos del sector y con los pilares descontados. La
        # triangulacion perp+diag que habia aqui marcaba -18 grados con
        # el robot perfectamente paralelo; ver el bloque ANGULO DE PARED
        # POR AJUSTE DE RECTA arriba para las medidas.
        #
        # Se promedian los dos lados cuando los dos son paredes planas
        # -- promediar cancela el error de un lado con el del otro -- y
        # si solo uno lo es, manda ese. Si ninguno, angulo_muro queda en
        # 0.0 con muro_valido=False: sin dato, no "estoy alineado".
        angulo_muro_viejo = _angulo_muro_triangulado(
            d_perp_der, d_perp_izq, d_diag_der, d_diag_izq)

        ang_der = angulo_de_pared(
            puntos_de_sector(perfil, *SECTOR_MURO_DER, excluidos=bins_estrechos))
        ang_izq = angulo_de_pared(
            puntos_de_sector(perfil, *SECTOR_MURO_IZQ, excluidos=bins_estrechos))
        if ang_der is not None and ang_izq is not None:
            angulo_muro, muro_valido = (ang_der + ang_izq) / 2.0, True
        elif ang_der is not None:
            angulo_muro, muro_valido = ang_der, True
        elif ang_izq is not None:
            angulo_muro, muro_valido = ang_izq, True
        else:
            angulo_muro, muro_valido = 0.0, False

        # Pared frontal con los postes descontados. Medido en pista
        # (README 8.5): un poste de 10cm es mas estrecho que el sector
        # frontal de 20 grados, asi que al avanzar entra y sale del
        # sector y el minimo salta entre el poste (~200mm) y el pasillo
        # de detras (~3000mm) en ciclos seguidos -- 24 saltos de factor
        # >=3x en una corrida, el 83% con un poste confirmado delante.
        # `frontal` sigue incluyendolos porque la emergencia anti-choque
        # SI tiene que ver los postes; el control de pared, no.
        d_front_muro = distancia_en_rango_sin_bins(
            perfil, sector_frontal[0], sector_frontal[1], bins_estrechos)

        return Medicion(d_front, d_izq, d_der, d_tras,
                         d_tras_der, d_tras_izq, clusters, perfil,
                         d_perp_izq=d_perp_izq, d_perp_der=d_perp_der,
                         d_diag_izq=d_diag_izq, d_diag_der=d_diag_der,
                         angulo_muro=angulo_muro, frontal_muro=d_front_muro,
                         muro_valido=muro_valido,
                         angulo_muro_viejo=angulo_muro_viejo,
                         clusters_estrechos=estrechos)
