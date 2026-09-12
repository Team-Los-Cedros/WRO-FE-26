# Geometria pura de la maniobra alrededor de un pilar. Sin estado, sin
# hilos, sin sensores: entra la posicion del pilar en el marco del robot
# y sale un radio, un comando de servo o una holgura en milimetros.
#
# Existe porque el fallo de las curvas no era de ganancias sino de
# geometria: la evasion anterior perseguia "un punto 260mm al lado del
# poste" con un control P sobre el bearing, y ese objetivo no dice nada
# sobre si el robot PUEDE rodear el pilar desde donde esta. Aqui esa
# pregunta se responde de forma cerrada y verificable.
#
# CONVENCIONES (identicas a lidar_geometria.py; no se repiten en ningun
# otro archivo de esta carpeta):
#   x+ = derecha del robot,  y+ = frente,  milimetros
#   rumbo = atan2(x, y) en grados, POSITIVO A LA DERECHA
#   comando de servo en grados, POSITIVO A LA IZQUIERDA (ver la Pico:
#       angulo_servo = CENTRO + angulo_objetivo, con LIMITE_IZQ=115 > 90)
#   yaw de la IMU en grados, POSITIVO A LA IZQUIERDA (antihorario)
#   s_lado = +1  el pilar debe quedar a la DERECHA del robot  (VERDE)
#            -1  el pilar debe quedar a la IZQUIERDA del robot (ROJO)
#
# Las dos ultimas son las que hay que verificar en pista antes de
# creerse nada de este archivo: ver "Verificacion de signos" en
# DISENO_CURVAS.md. "Pasar por la izquierda del pilar" y "girar a la
# izquierda" son cosas distintas, y de signo OPUESTO durante la mitad de
# la maniobra; por eso en este modulo no hay ni un solo sitio donde una
# se derive de la otra: s_lado viaja explicito.
import math

import geometria_robot as geo

# ==========================================
# CUERPO DEL ROBOT
# ==========================================
SEMIANCHO = geo.ANCHO_ROBOT / 2.0                  # 62.5 mm
VOLADIZO_DELANTERO = geo.LARGO_ROBOT - geo.BATALLA - geo.VOLADIZO_TRASERO
# Distancia del eje trasero a la esquina delantera, en y. El barrido de
# una curva lo definen las esquinas del chasis, no su punto medio.
X_MORRO = geo.BATALLA + VOLADIZO_DELANTERO         # 162 mm

# Radio de un poste del reglamento (50x50x100mm) tratado como circulo
# circunscrito: contra el chasis golpea la esquina, no la cara.
RADIO_POSTE = geo.LADO_POSTE * math.sqrt(2.0) / 2.0   # 35.4 mm

# ==========================================
# DIRECCION: RADIO MINIMO POR LADO
# ==========================================
# [MEDIDO CON CINTA el 10-09-2026] Radio del CENTRO DEL EJE TRASERO,
# que es el marco en el que trabaja todo este modulo.
#
# Metodo: girar a tope hasta 90 grados de guiñada y medir con cinta el
# radio de la circunferencia que traza cada rueda. El radio del eje es el
# promedio de la trasera interior y la trasera exterior.
#
#                       trasera int   trasera ext   -> EJE TRASERO
#   giro DERECHO             180.0         307.5         243.8
#   giro IZQUIERDO           195.0         290.0         242.5
#
# LO QUE ESTO CORRIGE, Y ES GRANDE. Los valores anteriores eran 260 y
# 360, derivados de dividir una velocidad de rumbo de la IMU por una
# velocidad lineal estimada, y tomados ADEMAS con la amortiguacion por
# giroscopio del firmware activa. El de la derecha estaba un 48% alto:
# a tope de volante el planificador dibujaba un arco de 360 mm y el
# robot trazaba 244. El equipo lo describio en pista como "gira de mas",
# y era literalmente eso.
#
# Y LA ASIMETRIA ERA FICTICIA. El robot gira practicamente igual a los
# dos lados (243.8 contra 242.5, medio punto porcentual). La diferencia
# 260/360 que habia aqui metia un sesgo estructural hacia la izquierda en
# toda situacion apretada: como `alcance_frontal` crece con el radio,
# girar a la derecha parecia alargar mas el morro, y con una pared
# cerca los unicos comandos admisibles salian siempre a la izquierda.
#
# COHERENCIA DE LAS MEDIDAS: la via DELANTERA que implican (112 y 115 mm)
# cuadra con VIA=115. La via TRASERA no (127.5 y 95), asi que las dos
# medidas traseras individuales traen unos +-16 mm de dispersion -- razon
# de mas para usar su PROMEDIO, que es lo que hace falta y que si
# concuerda entre lados. La batalla implicada por los radios delanteros
# sale larga (165 y 197 contra 136-140 reales): los delanteros parecen
# tomados a un punto mas adelantado que el centro de la huella. No se
# usan aqui.
#
# El margen de seguridad ya NO debe venir de inflar estos numeros -- esa
# era la practica anterior y es la que rompio la geometria. Viene de
# FACTOR_RADIO_PESIMISTA, que existe justo para eso.
RADIO_MIN_IZQ = 242.5
RADIO_MIN_DER = 243.8

# Topes reales del servo, medidos en la Pico (CENTRO=90, comando
# recortado a [70, 115]). Fuente unica: navegacion.py los importa de
# aqui en vez de volver a escribirlos.
COMANDO_MAX_IZQ = 25.0
COMANDO_MAX_DER = 20.0

# Batalla EFECTIVA por lado: la que hace que el modelo de bicicleta
# reproduzca el radio minimo medido justo en el tope del servo,
# tan(cmd_max) = L_ef / R_min. Difiere de la batalla real (136mm) por el
# deslizamiento de las ruedas y porque un "grado de comando" no es
# exactamente un grado de rueda virtual. Interpolar con ella deja el
# modelo exacto en los dos extremos, que es donde vive la maniobra.
BATALLA_EF_IZQ = RADIO_MIN_IZQ * math.tan(math.radians(COMANDO_MAX_IZQ))
BATALLA_EF_DER = RADIO_MIN_DER * math.tan(math.radians(COMANDO_MAX_DER))

# Radio por encima del cual "envolver" el pilar ya no es envolver sino
# seguir casi recto. Sale de la pista: una curva de 90 grados de radio R
# necesita R de ancho y R de largo para completarse, y el carril mide
# ANCHO_CARRIL (1000mm). Con 700mm queda sitio para el cuerpo del robot
# y para no comerse el muro exterior al salir.
RADIO_ENVOLVENTE_MAX = 0.7 * geo.ANCHO_CARRIL      # 700 mm

# Al predecir CUANTO se desplaza el robot de lado se supone un radio mas
# CERRADO que el comandado. No es un factor de ajuste: RADIO_MIN_IZQ y
# RADIO_MIN_DER estan redondeados hacia arriba a proposito, asi que el
# robot real gira algo mas de lo que se le pide y se desplaza de lado
# mas de lo que dice el modelo. Para no chocar hay que equivocarse por
# el lado de suponer MAS excursion, no menos.
#
# QUEDA EN 1.0 A PROPOSITO, con el mecanismo montado y sin usar. Se
# probo en 0.85 (el 8% que separa los radios medidos 239/341 de los
# asumidos 260/360) y EMPEORO: al recortar mas comandos, la envolvente
# de seguridad se queda sin salida mas a menudo, y las marchas atras
# encadenadas que eso provoca son peores que la excursion lateral que
# evitan (en la simulacion, un choque contra el muro que sin el factor
# no ocurria). Es el mismo patron que ya documento el repositorio con
# los angulos y las velocidades: apretar el margen no es gratis.
#
# El sitio donde arreglar esto es la medicion de los radios con kd=0,
# no este factor. Cuando esten medidos, subirlo a ~0.95 cuesta poco y
# cubre el error residual.
FACTOR_RADIO_PESIMISTA = 1.0


def normalizar_180(grados):
    # Reduce un angulo al intervalo (-180, 180]
    a = math.fmod(grados + 180.0, 360.0)
    if a <= 0.0:
        a += 360.0
    return a - 180.0


def rumbo(x, y):
    # Rumbo en grados hacia (x, y). Positivo = a la derecha.
    return math.degrees(math.atan2(x, y))


def radio_de_comando(cmd):
    # Radio absoluto de la trayectoria del EJE TRASERO para un comando de
    # servo. float('inf') para el comando recto.
    if abs(cmd) < 0.15:
        return float("inf")
    batalla = BATALLA_EF_IZQ if cmd > 0 else BATALLA_EF_DER
    return batalla / math.tan(math.radians(abs(cmd)))


def comando_de_radio(radio, hacia_izquierda):
    # Inversa de radio_de_comando, ya recortada al tope fisico del lado.
    if radio is None or radio > 1e6:
        return 0.0
    batalla = BATALLA_EF_IZQ if hacia_izquierda else BATALLA_EF_DER
    tope = COMANDO_MAX_IZQ if hacia_izquierda else COMANDO_MAX_DER
    cmd = math.degrees(math.atan(batalla / max(1.0, radio)))
    cmd = min(cmd, tope)
    return cmd if hacia_izquierda else -cmd


def radio_minimo(hacia_izquierda):
    return RADIO_MIN_IZQ if hacia_izquierda else RADIO_MIN_DER


def radio_interior(radio):
    # Radio mas pequeño que barre el cuerpo al girar: el costado interior
    # a la altura del eje. Cota conservadora (la esquina trasera interior
    # queda algo mas lejos del centro de giro que el costado).
    return radio - SEMIANCHO


def radio_exterior(radio):
    # Radio mas grande que barre el cuerpo: la esquina delantera exterior.
    return math.hypot(radio + SEMIANCHO, X_MORRO)


def alcance_frontal(radio, arco):
    """Cuanto se adelanta el punto mas avanzado del robot al recorrer un
    arco de longitud `arco` con radio `radio`.

    Es la version de HORIZONTE FINITO de radio_exterior(). La diferencia
    importa mucho: radio_exterior es lo que el morro alcanza si el arco
    se mantuviera INDEFINIDAMENTE, y usar eso como restriccion por ciclo
    es absurdamente conservador. Medido en pista (corrida 195623), con
    frontal_muro de mediana 370mm el criterio de circunferencia completa
    dejaba el conjunto de comandos admisibles VACIO en el 30% de los
    ciclos, y el robot retrocedia con 350mm de pared delante y los dos
    laterales por encima de 200mm.

    A angulo de arco phi el eje trasero esta en y = R sin(phi) y el
    cuerpo añade X_MORRO cos(phi) + SEMIANCHO |sin(phi)|. La suma crece
    hasta phi_pico = atan2(R + SEMIANCHO, X_MORRO), donde vale
    exactamente radio_exterior(R): o sea que esta funcion coincide con la
    otra cuando el horizonte es largo, y solo relaja el caso corto.

    Reparto de responsabilidades que esto deja claro, y que hay que
    respetar: el criterio de circunferencia COMPLETA
    (radio_maximo_para_frente) es la CONSIGNA -- dice cuando empezar a
    girar para que la curva quepa. El de horizonte finito es la
    RESTRICCION -- dice que se puede ejecutar en este ciclo. Hacen falta
    los dos y no son intercambiables.
    """
    if radio == float("inf"):
        return arco + X_MORRO
    phi_pico = math.atan2(radio + SEMIANCHO, X_MORRO)
    th = min(arco / radio, phi_pico)
    return (radio * math.sin(th) + X_MORRO * math.cos(th)
            + SEMIANCHO * math.sin(th))


def radio_maximo_para_frente(alcance):
    """Radio mas ABIERTO que todavia dobla antes de una pared frontal.

    En un arco sostenido, el punto del robot que mas se adelanta es la
    esquina delantera exterior, y su alcance maximo es
    radio_exterior(R) = hypot(R + SEMIANCHO, X_MORRO). Igualando eso a lo
    que queda hasta la pared y despejando:

        R = sqrt(alcance^2 - X_MORRO^2) - SEMIANCHO

    Devuelve None si no cabe ningun arco (la pared esta mas cerca que el
    propio morro): ahi la unica accion es la marcha atras.

    Es la que hace que la curva se empiece a tiempo. Un limite sobre los
    comandos admisibles solo prohibe llegar tarde; esto dice CUANDO hay
    que empezar a girar y con que radio, y lo hace solo mas cerrado segun
    la pared se acerca.
    """
    if alcance <= X_MORRO:
        return None
    return math.sqrt(alcance * alcance - X_MORRO * X_MORRO) - SEMIANCHO


def holgura_arco(x_r, y_r, cmd):
    """Holgura en mm entre el pilar y el cuerpo si se ejecuta `cmd` y se
    mantiene indefinidamente.

    (x_r, y_r) es el pilar en el MARCO DEL EJE TRASERO. Positivo = el
    arco no toca el pilar; negativo = lo barre, y el valor dice cuanto
    falta para librarlo.

    Girando, el cuerpo barre un anillo entre radio_interior y
    radio_exterior alrededor del centro instantaneo de giro. El pilar
    esta a salvo si queda DENTRO del agujero del anillo (caso de una
    envolvente: el robot orbita a su alrededor) o FUERA del anillo
    entero (el robot se aleja).
    """
    if abs(cmd) < 0.15:
        # Recto: solo cuenta el desplazamiento lateral, y solo si el
        # pilar esta por delante de la cola.
        if y_r <= -geo.VOLADIZO_TRASERO:
            return 1e4
        return abs(x_r) - SEMIANCHO - RADIO_POSTE

    radio = radio_de_comando(cmd)
    # Comando positivo = giro a la izquierda = centro de giro a la
    # izquierda del robot, o sea en x negativa.
    centro_x = -radio if cmd > 0 else radio
    d = math.hypot(x_r - centro_x, y_r)
    por_dentro = radio_interior(radio) - d
    por_fuera = d - radio_exterior(radio)
    return max(por_dentro, por_fuera) - RADIO_POSTE


def preserva_lado(x_r, y_r, s_lado, cmd, holgura):
    """¿Ejecutar `cmd` deja el pilar en su lado con holgura?

    Es LA pregunta de la maniobra, y no es la misma que "¿choco?".
    Sostener `cmd` indefinidamente deja el pilar del lado bueno en dos
    casos, y solo en esos dos:

    1. ENVOLVENTE: el pilar queda DENTRO del circulo de giro, con el
       centro de giro de su mismo lado. El robot lo orbita; el pilar no
       puede cruzar al otro lado porque esta dentro.
    2. ALEJARSE O SEGUIR RECTO: el arco no lo toca (queda fuera del
       anillo barrido) y ademas curva HACIA EL LADO CONTRARIO al pilar,
       o va recto. El pilar se queda atras por su lado.

    El caso que hay que rechazar, y que el simple "no choco" deja pasar,
    es curvar HACIA el pilar sin llegar a meterlo dentro del circulo: el
    robot no lo toca, pero su rumbo barre por encima y el pilar acaba
    cambiando de lado. Ese es exactamente el "cambio de lado silencioso"
    que se veia en pista.

    Nota importante de diseño: si ALGUN comando admisible preserva el
    lado, NO hace falta envolver. Exigir siempre la envolvente es lo que
    provocaba excursiones inutiles hacia la concavidad de la esquina con
    el pilar ya del lado bueno.
    """
    u = s_lado * x_r
    if u < SEMIANCHO + RADIO_POSTE + holgura and y_r > -geo.VOLADIZO_TRASERO:
        return False

    radio = radio_de_comando(cmd)
    if radio == float("inf"):
        return True

    centro_x = -radio if cmd > 0 else radio
    d = math.hypot(x_r - centro_x, y_r)
    margen = RADIO_POSTE + holgura

    if (radio_interior(radio) - d) >= margen:
        # Dentro del circulo: solo vale si el centro esta del lado del
        # pilar, o sea si se gira HACIA el.
        return (centro_x * s_lado) > 0
    if (d - radio_exterior(radio)) >= margen:
        # Fuera del anillo: vale si el giro se aleja del pilar (o es
        # practicamente recto).
        return (cmd * s_lado) >= 0
    return False


def radio_envolvente(x_r, y_r, s_lado, holgura):
    """Radio mas cerrado que envuelve el pilar dejandolo en su lado.

    Devuelve (factible, radio). `factible` es False cuando ningun radio
    ejecutable (hasta RADIO_ENVOLVENTE_MAX) mete el pilar dentro del
    circulo con la holgura pedida: toca seguir abriendo.

    DEMOSTRACION. Se envuelve girando HACIA el lado del pilar, asi que
    el centro de giro esta en (s_lado*R, 0). Llamando
    k = SEMIANCHO + RADIO_POSTE + holgura   y   u = s_lado * x_r
    (la separacion lateral medida por el lado bueno), el pilar queda
    dentro del circulo con holgura si

        d = hypot(u - R, y_r) <= R - k                        (1)

    Si (1) se cumple, se cumple durante TODO el arco: el anillo barrido
    es el mismo en cada punto de la circunferencia, asi que el pilar
    sigue dentro del agujero. De ahi salen los dos invariantes que la
    maniobra necesita -- el pilar no puede cambiar de lado (esta del
    lado del centro de giro, que es s_lado) y la separacion minima vale
    (R - SEMIANCHO) - d - RADIO_POSTE >= holgura.

    g(R) = (R - k) - hypot(u - R, y_r) es creciente en R, porque
    dg/dR = 1 + (u - R)/hypot(...) >= 0. Basta entonces evaluar g en
    RADIO_ENVOLVENTE_MAX para saber si hay solucion, y su raiz da el
    radio mas cerrado que sirve:

        (R - k)^2 = (u - R)^2 + y_r^2
        R = (u^2 + y_r^2 - k^2) / (2 (u - k))                 (2)

    Con u <= k no hay solucion para ningun R, porque
    g(R) <= (R - k) - (R - u) = u - k <= 0. Es el caso "no tengo
    separacion lateral suficiente ni pegandome al pilar", y la respuesta
    correcta es abrir, no girar mas fuerte.
    """
    k = SEMIANCHO + RADIO_POSTE + holgura
    u = s_lado * x_r

    if u <= k:
        return (False, None)

    g_max = (RADIO_ENVOLVENTE_MAX - k) - math.hypot(u - RADIO_ENVOLVENTE_MAX, y_r)
    if g_max < 0.0:
        return (False, None)

    r_sol = (u * u + y_r * y_r - k * k) / (2.0 * (u - k))
    return (True, max(radio_minimo(s_lado < 0), r_sol))


def separacion_requerida(y_r, holgura, radio_max=RADIO_ENVOLVENTE_MAX):
    """Separacion lateral minima (mm) para que la envolvente sea posible
    con el pilar a y_r por delante.

    Es la ecuacion (1) despejada en u con R = radio_max:

        u_req = radio_max - sqrt((radio_max - k)^2 - y_r^2)

    o sea un lugar geometrico circular: el pilar tiene que caer dentro
    del disco de radio (radio_max - k) centrado en el centro de giro del
    radio_max. Devuelve None cuando y_r > radio_max - k, es decir cuando
    el pilar esta tan por delante que ninguna separacion sirve todavia:
    lo unico que arregla ese caso es avanzar.
    """
    k = SEMIANCHO + RADIO_POSTE + holgura
    resto = (radio_max - k) ** 2 - y_r * y_r
    if resto <= 0.0:
        return None
    return radio_max - math.sqrt(resto)


def radio_apertura(x_r, y_r, s_lado, separacion_objetivo):
    """Radio del arco de APERTURA: el que gana la separacion que falta
    justo cuando el pilar llegue al travez.

    Alejandose en arco de radio R, el desplazamiento lateral tras
    recorrer s vale R(1 - cos(s/R)) ~= s^2/(2R). Pidiendo que en los y_r
    que quedan hasta el travez se gane (objetivo - u):

        R = y_r^2 / (2 (objetivo - u))

    Devuelve None si ya no hace falta abrir. Sale recortado al radio
    minimo del lado por el que se abre, que es el CONTRARIO al pilar.
    """
    u = s_lado * x_r
    falta = separacion_objetivo - u
    if falta <= 0.0:
        return None
    if y_r <= 0.0:
        return radio_minimo(s_lado > 0)
    return max(radio_minimo(s_lado > 0), (y_r * y_r) / (2.0 * falta))


def comando_apertura(x_r, y_r, s_lado, separacion_objetivo):
    # Abrir es girar hacia el lado CONTRARIO al del pilar: si el pilar
    # debe quedar a la derecha (s_lado=+1) se abre a la izquierda, que es
    # comando positivo. De ahi que el signo sea +s_lado y no -s_lado.
    radio = radio_apertura(x_r, y_r, s_lado, separacion_objetivo)
    if radio is None:
        return 0.0
    return comando_de_radio(radio, hacia_izquierda=(s_lado > 0))


def comando_envolvente(x_r, y_r, s_lado, holgura):
    # Envolver es girar HACIA el pilar (signo -s_lado). Devuelve None si
    # la envolvente todavia no es factible desde esta pose.
    factible, radio = radio_envolvente(x_r, y_r, s_lado, holgura)
    if not factible:
        return None
    return comando_de_radio(radio, hacia_izquierda=(s_lado < 0))


def predecir_pilar(x_r, y_r, avance_mm, giro_grados):
    """Donde estara el pilar en el marco del robot tras recorrer un arco
    de `avance_mm` girando `giro_grados` (positivo = izquierda).

    Mismo modelo que usa el tracker, expuesto aparte para poder mirar un
    ciclo de servo por delante. El limitador mueve el comando 12 grados
    por ciclo, asi que invertir el volante de un tope al otro cuesta ~4
    ciclos (0.45 s a 8.5 Hz), y hay decisiones que conviene tomar sobre
    donde estara el pilar al final de ese movimiento y no sobre donde
    esta ahora. Lo usa el sesgo de readquisicion por camara.

    OJO: NO se usa para la salida de APERTURA. Se probo y sacaba de la
    fase antes de tener la separacion real (135mm cuando el minimo
    geometrico son 183), y la fase siguiente se quedaba sin ningun
    comando compatible desde el primer ciclo.
    """
    th = math.radians(giro_grados)
    if abs(th) < 1e-4:
        dx, dy = 0.0, avance_mm
    else:
        radio = avance_mm / th
        dx = -radio * (1.0 - math.cos(th))
        dy = radio * math.sin(th)
    xa, ya = x_r - dx, y_r - dy
    c, s = math.cos(th), math.sin(th)
    return (xa * c + ya * s, -xa * s + ya * c)


def lateral_predicho(cmd, avance_mm, angulo_muro_grados):
    """Desplazamiento lateral (mm, +x = derecha) tras recorrer
    `avance_mm` con el comando `cmd`, contando ADEMAS la deriva por
    llevar el morro torcido respecto del pasillo.

    El segundo termino es el que le faltaba al control anterior: en
    Ackermann girar no separa de la pared al instante, hace falta
    avanzar, y mientras tanto el robot sigue derivando hacia donde
    apunta. lidar_geometria mide ese angulo (angulo_muro), negativo
    cuando el morro apunta a la derecha.
    """
    deriva = avance_mm * math.sin(math.radians(-angulo_muro_grados))
    radio = radio_de_comando(cmd) * FACTOR_RADIO_PESIMISTA
    if radio == float("inf"):
        return deriva
    theta = avance_mm / radio
    curva = radio * (1.0 - math.cos(theta))
    return deriva + (-curva if cmd > 0 else curva)


def esquinas_barridas(cmd, arco, lidar_x, fracciones=(0.5, 1.0)):
    """Donde acaban las esquinas DELANTERAS al recorrer `arco` con `cmd`.

    Devuelve [(x, y), ...] en el marco del LiDAR -- x+ = derecha,
    y+ = frente -- que es el marco en el que viene el perfil, para poder
    preguntar "que hay EN EL RUMBO por el que va a pasar mi esquina".

    POR QUE EXISTE. `alcance_frontal()` da UN numero, el adelanto maximo
    del cuerpo, y quien lo usaba lo comparaba contra `frontal_muro`, que
    es el minimo del sector de +-10 grados. Son dos direcciones
    distintas. Al girar, la esquina delantera EXTERIOR se abre hacia el
    costado a la vez que avanza, asi que su adelanto CRECE con el angulo:
    medido con la pared a 122 mm, el filtro aceptaba cmd 0 y +2 y
    rechazaba todo lo que pasara de 10 grados a los dos lados. O sea que
    cuanto mas cerca estaba la pared, mas obligaba a ir RECTO contra
    ella. En la corrida de las 19:22 las cuatro entradas en RETROCESO
    llegaron asi: seis ciclos con el volante en 0.0 exacto y el frente
    bajando 157->122, mientras el rumbo deseado pedia girar 12 grados.

    De una pared de frente se sale girando. La esquina que se abre lo
    hace hacia un costado donde, en un pasillo de 1000 mm, casi siempre
    hay sitio -- y si no lo hay, quien tiene que decirlo es la distancia
    A ESE RUMBO, no la del sector frontal.

    Geometria: el eje trasero describe el arco de radio R; el cuerpo gira
    con el. Con s = +1 girando a la izquierda, tras un angulo th el eje
    esta en (-s R (1 - cos th), R sin th) y el marco del robot ha rotado
    s*th. Un punto (bx, by) del cuerpo -- bx+ = derecha, by+ = frente --
    acaba en eje + Rot(s*th) (bx, by). El comando recto es el caso limite
    y se trata aparte para no dividir por infinito.
    """
    radio = radio_de_comando(cmd)
    s = 1.0 if cmd > 0 else -1.0
    puntos = []
    for frac in fracciones:
        d = arco * frac
        if radio == float("inf"):
            x_eje, y_eje, giro = 0.0, d, 0.0
        else:
            th = d / radio
            x_eje = -s * radio * (1.0 - math.cos(th))
            y_eje = radio * math.sin(th)
            giro = s * th
        cg, sg = math.cos(giro), math.sin(giro)
        for bx in (-SEMIANCHO, SEMIANCHO):
            puntos.append((x_eje + bx * cg - X_MORRO * sg,
                           y_eje + bx * sg + X_MORRO * cg - lidar_x))
    return puntos
