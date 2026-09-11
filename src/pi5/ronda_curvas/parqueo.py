# Salida y entrada del estacionamiento. SCRIPT APARTE a proposito: el
# codigo de carrera va 12/12 y no se toca hasta que esto funcione solo.
#
#   python3 parqueo.py salida          maniobra de verdad
#   python3 parqueo.py salida --seco   lee sensores y dice que haria, SIN MOVERSE
#
# POR QUE UNA MANIOBRA EN VARIOS TIEMPOS Y NO UN ARCO
#
# Medido el 10-09 con el robot dentro de la plaza: muro exterior a 78mm
# del LiDAR (flanco izquierdo a 16mm de el), cola a 14mm del muro magenta
# trasero, morro a 119mm del delantero, y 902mm de carril abierto al otro
# lado. La plaza real mide 355mm de largo, 22 mas que los 333 que da el
# reglamento para un robot de 222.
#
# Con esa geometria NO EXISTE salida de un solo arco, desde ninguna
# colocacion. La cuenta: para escapar de la sombra de los muros magenta
# (200mm de fondo) el robot tiene que desplazarse al menos 134mm de lado
# en el caso mas favorable; con radio minimo 243.8 eso pide 63 grados de
# giro y 218mm de avance longitudinal, y la plaza solo deja 133mm de
# holgura. No es cuestion de afinar: no cabe.
#
# La que si cabe es la de siempre: adelante con el volante HACIA la
# salida, atras con el volante AL CONTRARIO. Los dos sentidos rotan el
# chasis en el MISMO sentido, asi que cada par gana angulo neto. En
# simulacion sale al decimo tiempo con 82 grados acumulados.
#
# Es el mismo principio que rompio el ciclo limite de las esquinas en la
# corrida de las 22:41, y por la misma razon: una marcha atras sin
# intencion devuelve el angulo que el avance habia ganado.
#
# CUANTO MARGEN TIENE LA MANIOBRA, con el robot MEDIDO (240 x 140)
#
# La colocacion LATERAL no se elige: el reglamento obliga a dejar 30 mm
# entre el flanco y el muro negro exterior, o sea el centro del robot a
# 100 mm de el. Ahi el margen maximo de la maniobra es de 14-16 mm, y
# depende mucho de donde quede a lo LARGO de la plaza:
#
#   plaza 352    cola:    0    15    30    45    60    75    90   105
#   margen 10 mm          -     -    6t    4t    4t    4t    4t    -
#   margen 14 mm          -     -    6t    6t    6t    6t   10t    -
#   margen 16 mm          -     -   10t    8t    8t    8t     -    -
#
# Pegado al muro magenta trasero (cola 0-15) NO SALE, y pegado al
# delantero tampoco. El punto bueno es cola 45-75: cuatro tiempos con
# 12-14 mm de holgura.
#
# Y hay que tenerlo presente: las esquinas criticas -- la trasera del lado
# del muro exterior contra el magenta, y el flanco contra el exterior --
# NO LAS VE NINGUN SENSOR. El LiDAR mira desde el centro del robot y el
# ultrasonido es un solo haz recto hacia atras. Los cortes por sensor de
# este script protegen el frente y la trasera CENTRADA, no esas esquinas:
# el margen sale de la COLOCACION, no del control.
import collections
import math
import signal
import sys
import threading
import time

sys.path.insert(0, "/home/pi/ronda_curvas")

import geometria_evasion as gev          # noqa: E402
import geometria_robot as geo            # noqa: E402
import lidar_geometria as lg             # noqa: E402
from lidar_mascara import MASTIL_MAX, MASTIL_MIN   # noqa: E402
from enlace_pico import EnlacePico       # noqa: E402
from lidar_driver import LidarDriver     # noqa: E402

# ---------------------------------------------------------- el robot REAL
#
# MEDIDO CON CINTA el 10-09: 240 x 140 mm. `geometria_robot` dice 222 x 125
# y esta corto -- es la cuarta vez que un valor de montaje sale desviado.
#
# Aqui se usan los reales y NO se corrige geometria_robot: ese modulo lo
# consume el arbitraje de la carrera, que acaba de hacer 12/12, y cambiarle
# el semiancho y el morro mueve todas sus envolventes. Eso merece su propia
# corrida medida, no un cambio a ciegas a la 1 de la madrugada.
LARGO_REAL   = 242.0
ANCHO_REAL   = 138.0
SEMIANCHO_REAL = ANCHO_REAL / 2.0

# EL LiDAR NO ESTA EN EL EJE CENTRAL. Medido el 11-09 con regla: el flanco
# del lado del muro exterior estaba a 30 mm de el y el LiDAR leia 74. O
# sea que de sensor a ese flanco hay 44 mm, no los 70 del semiancho: el
# LiDAR va 26 mm desplazado hacia ese costado.
#
# Importa aqui y no en el resto: los numeros de colocacion de la cabecera
# estan calculados sobre el CENTRO REAL del robot, que no cambia. Lo que
# cambia es como se traduce una lectura del LiDAR a una distancia de
# flanco, que es justo lo que comprueba el candado de abajo.
#
# SIN CONFIRMAR de que lado va el desplazamiento en el chasis. Se dedujo
# de una sola pose, con el muro a un lado. Si algun dia se coloca el robot
# mirando al reves -- y el sorteo puede obligar a ello -- el costado
# cercano seria el otro y este numero seria 96, no 44. Medirlo con regla a
# los dos lados antes de fiarse.
# MEDIDO CON REGLA (Daniel, 11/09): 68 mm del eje del LiDAR a CADA
# costado. El robot es simetrico. Antes aqui habia 44 y 94, y esa
# asimetria falsa de 50 mm era la que estrangulaba la salida: con 94 mm
# de chasis supuesto al costado derecho, el muro exterior a 76 mm daba
# holgura NEGATIVA (-17 mm) con el robot PARADO, y los doce primeros
# tiempos del vaiven cortaban al primer ciclo avanzando 1 o 2 grados.
# Concuerda ademas con geometria_robot.LIDAR_Y = -4: casi centrado.
LIDAR_A_FLANCO_CERCA = 68.0
LIDAR_A_FLANCO_LEJOS = 68.0

# DERIVADOS, no medidos: salen de LIDAR_X=128 (eje trasero al LiDAR) y
# VOLADIZO_TRASERO=60, ninguno de los dos comprobado con regla en este
# chasis. Si alguno esta mal, la silueta de abajo esta mal y con ella
# todo el margen. Es lo primero que hay que medir.
LIDAR_A_MORRO_REAL = LARGO_REAL - 60.0 - 128.0 + 128.0 - (LARGO_REAL - 60.0 - 128.0) + 54.0
LIDAR_A_MORRO_REAL = 54.0      # 242 - 60 (voladizo) - 128 (eje->LiDAR)
LIDAR_A_COLA_REAL  = 188.0     # 128 + 60


def _silueta():
    """Distancia del LiDAR al borde del propio robot, por cada rumbo.

    El LiDAR no esta ni centrado ni en mitad del chasis: tiene 44 mm de
    carroceria a un costado, 94 al otro, 54 al morro y 188 a la cola. Asi
    que una lectura de 200 mm significa cosas muy distintas segun hacia
    donde mire, y compararlas todas contra un mismo umbral es comparar
    magnitudes distintas -- el error que ya costo una sesion entera en el
    codigo de carrera.

    Con esta tabla, `perfil[a] - silueta[a]` es la holgura REAL de chapa a
    obstaculo en ese rumbo, y eso si se puede comparar con un margen.
    """
    tabla = []
    for a in range(360):
        r = math.radians(a)
        sx, sy = math.sin(r), math.cos(r)      # 0=frente, 90=derecha
        # distancia al rectangulo [-cerca, +lejos] x [-cola, +morro]
        t = float("inf")
        for lim, comp in ((LIDAR_A_FLANCO_LEJOS, sx), (-LIDAR_A_FLANCO_CERCA, sx),
                          (LIDAR_A_MORRO_REAL, sy), (-LIDAR_A_COLA_REAL, sy)):
            if abs(comp) > 1e-6:
                d = lim / comp
                if d > 0:
                    px, py = d * sx, d * sy
                    if (-LIDAR_A_FLANCO_CERCA - 0.5 <= px <= LIDAR_A_FLANCO_LEJOS + 0.5
                            and -LIDAR_A_COLA_REAL - 0.5 <= py <= LIDAR_A_MORRO_REAL + 0.5):
                        t = min(t, d)
        tabla.append(t)
    return tabla


SILUETA = _silueta()

# ---------------------------------------------------------------- ajustes
VEL_MANIOBRA   = 22      # % PWM. Despacio: aqui no se gana tiempo, se
VEL_REVERSA    = -22     # gana no tocar un muro magenta (regla 9.24.7).
# NO SE USA EL TOPE FISICO ENTERO. En pista la cola rozaba la pila
# contra el muro al girar a fondo: el voladizo trasero barre tanto mas
# cuanto mas cerrado es el giro. Un 12% menos de angulo abre el radio
# (243 -> ~270 mm) y a cambio baja el barrido de la cola, que es lo que
# tocaba. Cuesta algun tramo mas, y eso aqui no importa.
RECORTE_TOPE   = 0.88
TOPE_IZQ       = gev.COMANDO_MAX_IZQ * RECORTE_TOPE     # 22.0
TOPE_DER       = gev.COMANDO_MAX_DER * RECORTE_TOPE     # 17.6

# Margenes de parada de cada tramo. Contra los muros magenta el colchon
# es caro de perder: tocarlos TERMINA el recorrido, asi que se paga con
# tiempo, no con puntos.
# EL MARGEN DELANTERO SE MIDE EN EL MORRO, NO EN LA LECTURA DEL LiDAR.
# El sensor va LIDAR_A_MORRO por detras de la punta, asi que una lectura
# de 55 mm son 3 mm de morro: el margen "de 55" no existia.
LIDAR_A_MORRO  = 52.0    # 180 (eje->morro, robot de 240) - 128 (eje->LiDAR)
# 10 mm, no 18. El margen MAXIMO que admite esta maniobra es 14-16 mm
# (ver la tabla de la cabecera): pedir 18 la hacia imposible por
# construccion, y se vio en pista -- 14 tiempos cortando todos por
# "morro a 10-12 mm" con la guiñada oscilando entre -5,5 y -6,5. Neto
# cero. La plaza da 113 mm de holgura para un robot de 240: el margen
# no es una decision, es lo que sobra.
HUECO_MORRO_MIN = 10.0   # mm de chapa a muro
MARGEN_FRENTE  = LIDAR_A_MORRO + HUECO_MORRO_MIN     # 62 mm de lectura
# EL ULTRASONIDO MANDA POR DETRAS. Es el unico sensor que ve el sector
# ciego del mastil (135-199 grados), justo donde queda el muro magenta
# trasero durante media maniobra. 18 mm y no menos: el LiDAR no puede
# cubrir ahi y no hay segunda linea de defensa.
MARGEN_TRASERA = 18.0    # mm de ultrasonido (ya descuenta el voladizo)

# CON LA COLA PEGADA NO SE PUEDE GIRAR: hay que avanzar RECTO primero.
#
# La colocacion de competencia deja la cola casi tocando el muro magenta
# trasero. Girando hacia la salida la cola barre HACIA ATRAS, asi que el
# primer tiempo se bloquea antes de ganar un grado. En recto no barre
# ninguna esquina, y el morro tiene los otros 98 mm de la plaza: avanzar
# recto convierte ese hueco delantero en hueco trasero, que es el que
# hace falta para empezar a girar.
#
# Medido en simulacion con la cola a 15 mm:
#   sin avanzar recto primero -> NO SALE, en ningun margen
#   avanzando recto primero   -> 6 tiempos con 10 mm
#
# No es un paso fijo al principio: se decide en cada ciclo, asi que el
# robot va recto solo mientras le falte hueco y empieza a girar en cuanto
# lo tiene. Si lo colocan con la cola ya suelta, gira desde el primer
# instante y no pierde nada.
# SECUENCIA DEL EQUIPO, probada en pista: volante al tope hacia la
# salida desde el PRIMER instante -- nada de avanzar recto antes.
#
#   1. ruedas a la salida, adelante girando hasta que no pueda mas
#   2. ruedas al contrario, atras todo lo que pueda
#   3. ruedas a la salida otra vez, y sale
#
# Arrancar recto parecia prudente (con la cola pegada, girar la barre
# hacia el muro magenta) pero se come el hueco delantero: de los 98 mm de
# morro gastaba 33 en enderezar la cola y dejaba 65 para girar, que no
# bastan. Girando desde el principio cada avance rinde 25 grados.
#
# Lo que protege la cola no es ir recto: es el corte por holgura de
# cuerpo, que mira los 360 grados contra la silueta real.
ARRANCAR_GIRANDO = True

# EL VOLANTE PRIMERO, EL ROBOT DESPUES (idea del equipo).
#
# Mandar velocidad y angulo en la misma consigna hace que el robot
# arranque mientras el servo todavia va de camino al tope: los primeros
# centimetros de cada tramo los recorre con un angulo intermedio, o sea
# describiendo un arco MAS ABIERTO del que se le pide. Dentro de la plaza
# eso es carisimo -- se gasta el hueco delantero sin cobrar el giro, y es
# parte de por que cada tramo rinde 5-8 grados donde la geometria dice
# 22.
#
# Asi que cada tramo empieza con el robot QUIETO y el volante puesto, y
# solo cuando el servo ha llegado se le da velocidad.
ESPERA_SERVO = 0.55      # s con el robot parado y el volante al tope
PAUSA_ENTRE_TRAMOS = 0.35

# CUANTO SE PUEDE RETROCEDER: SE DECIDE ANTES DE ARRANCAR, NO DURANTE.
#
# El ultrasonido apunta recto hacia atras. Mientras el robot esta
# alineado con la plaza eso vale, pero a 40-50 grados de guiñada el haz
# cruza la plaza en diagonal y mide el fondo del pasillo -- reporta sitio
# de sobra mientras la ESQUINA trasera esta pegada al muro magenta. Y esa
# esquina cae en el sector ciego del mastil (135-199), asi que el LiDAR
# tampoco la ve. Resultado en pista: los retrocesos corrian sus 3 s
# enteros ("corta por tiempo maximo del tramo") y tocaban la pared.
#
# Asi que el hueco se mide UNA VEZ, con el robot quieto y el volante ya
# puesto, y se convierte en un limite de recorrido. Es un tope duro que
# no depende de hacia donde apunte el haz mientras el robot gira.
#
# 400 mm/s a 100% de PWM, curva medida en el banco.
MM_POR_SEG = 400.0 * abs(VEL_REVERSA) / 100.0
# 15 mm, no 25. En pista se vio que a las maniobras "les falta
# retroceder un poco mas": cada tramo se quedaba con 10 mm de hueco sin
# usar, y en una maniobra de 8 tramos eso son 80 mm de recorrido regalado.
MARGEN_RECORRIDO_ATRAS = 15.0    # mm que NO se gastan del hueco medido

# CUANTO SE PUEDE GIRAR HACIA ADELANTE: LO DICE EL HUECO DE LA COLA.
#
# Girando en marcha adelante la esquina trasera se desplaza HACIA ATRAS
# aproximadamente `semiancho * sen(giro)` -- con 69 mm de semiancho, 10
# grados la mueven 12 mm. Si la cola tiene 15 mm de hueco, pedir mas de
# unos 6 grados la estampa contra el muro magenta.
#
# Y eso NO lo puede salvar ningun sensor: esa esquina cae en el sector
# ciego del mastil (135-199) y el haz del ultrasonido, con el robot ya
# torcido, apunta en diagonal a traves de la plaza. Se vio en el video
# del 11-09: la trasera acercandose al muro en los primeros tramos
# mientras el log no decia nada. La unica defensa es NO PEDIR el giro.
#
# Se despeja de semiancho*sen(giro) <= hueco - margen.
MARGEN_ESQUINA_TRASERA = 8.0
COLA_PARA_GIRAR = 45.0
REVERSA_RECTA = False

# Margen de chapa a obstaculo en CUALQUIER rumbo. Es el corte que de
# verdad protege la maniobra: los otros dos solo miran al frente y atras.
# 8 mm, y son 8 mm DE VERDAD: este corte compara cada rumbo contra la
# silueta real del robot, y se valido contra la regla (dijo 29 donde la
# cinta decia 30). No es un umbral prudente sobre una medida dudosa.
HOLGURA_MINIMA_CUERPO = 8.0

# UN ECO DENTRO DE MI PROPIA SILUETA SOY YO.
#
# El LiDAR ve estructura del robot en rumbos que la mascara no cubre. La
# corrida de las 13:23 lo aviso al arrancar ("estructura FUERA de la
# mascara en los grados [44..99]") y ahi mismo se atasco el vaiven: los
# 24 tiempos cortaron por "chapa a -16 mm en el rumbo 51" contra ecos
# PLANOS de 61-66 mm entre los rumbos 36 y 46. Una pared a esa distancia
# subiria de 65 a 80 mm en ese arco; estos no se mueven. Distancia
# constante a lo largo de 10 grados = algo a radio fijo del LiDAR.
#
# Un obstaculo no puede estar DENTRO del contorno del chasis sin que el
# chasis lo este tocando, y tocandolo leeria justo la silueta, no 30 mm
# por dentro. Asi que por debajo de este margen no es un obstaculo: es
# estructura propia, y ese rumbo no cuenta.
#
# Esto no tapa un contacto real: un muro rozando el flanco lee la silueta
# con unos pocos mm de diferencia, y sigue contando.
MARGEN_ESTRUCTURA = 12.0

# LA RUEDA DELANTERA A TOPE, QUE EL LIDAR SE VE A SI MISMO.
#
# El vaiven va SIEMPRE con el volante al tope, y ahi la rueda asoma
# dentro del barrido. Medido en dos corridas:
#
#   13:23  rumbos 36-46, ecos PLANOS de 61-66 mm
#   13:44  rumbos 25-39 y 65-75, ecos PLANOS de 49-56 mm
#
# Planos: una pared a esa distancia subiria 15 mm a lo largo de ese arco.
# Y el radio CAMBIA entre corridas porque cambia el angulo de la rueda,
# que es lo que descarta que sea un muro y lo que confirma que es la
# rueda. Esta en las notas del proyecto: el LiDAR ve la propia rueda al
# girar.
#
# Nada ajeno al robot puede estar a menos de 72 mm en estos sectores sin
# que el chasis (68 mm de semiancho) lo este tocando ya. El muro exterior
# con el robot aparcado lee 80-81 mm: queda fuera y sigue contando.
#
# El frente NO entra aqui: ahi la silueta son 54 mm y un obstaculo a 64
# es real. Del frente se encarga el sector frontal de _hueco_para_avanzar.
SECTORES_RUEDA = ((20, 110), (250, 340))
RADIO_RUEDA = 72.0


def _es_rueda(a, d):
    return d < RADIO_RUEDA and any(lo <= a <= hi for lo, hi in SECTORES_RUEDA)
# Grados seguidos que tiene que ocupar algo para contar como obstaculo.
ANCHO_OBSTACULO = 7
SECTOR_AVANCE  = 45.0    # +-grados que se miran al avanzar
SECTOR_BUSQUEDA = 15.0   # +-grados mientras se busca el carril: ver
                         # _hueco_para_avanzar

# Un tramo termina tambien por angulo: pasado esto ya no gana nada util y
# lo que hace falta es invertir, que es lo que hace crecer la guiñada.
GIRO_MAX_TRAMO = 25.0    # grados por tramo
TRAMOS_MAX     = 24

# TOPE DE GUIÑADA DE LA SALIDA, en grados.
#
# El vaiven solo hace la MITAD de salir: gira el morro hacia el carril.
# Enderezar es trabajo de `alinear`. Si el vaiven sigue hasta que el LiDAR
# ve todo despejado, se pasa: en la corrida de las 12:57 acumulo +97
# grados, o sea que salio casi PERPENDICULAR al carril, apuntando a la
# isla. Y asi llego al primer pilar: separacion -781 mm, por el lado
# prohibido.
#
# Con 90 el tramo 17 (+97,3) ya es el ultimo, que es justo quitar los dos
# ultimos vaivenes de esa corrida.
GUIÑADA_MAX_SALIDA = 90.0
TIMEOUT_TRAMO  = 3.0     # s

# Fuera de la plaza: el costado que era el muro exterior se abre, y en el
# sentido de avance ya no hay muro magenta.
SALIDA_COSTADO = 320.0   # mm en el costado del muro exterior
SALIDA_LIBRE   = 450.0   # mm en el sentido de avance

# DISTANCIA MINIMA AL MURO EXTERIOR PARA QUE LA SALIDA SEA POSIBLE.
#
# Esto no es un margen de comodidad: por debajo de aqui la maniobra NO
# EXISTE. Con el robot pegado al muro exterior cualquier rotacion mete
# una esquina contra el, y las dos unicas direcciones utiles (adelante
# girando hacia la salida, atras girando al contrario) quedan las dos
# bloqueadas.
#
# Barrido en simulacion sobre la plaza medida (355 mm), variando donde se
# coloca el robot y contando en cuantos tiempos sale:
#
#   centro a  78 mm (flanco a 16)  -> solo desde UNA posicion, 7 tiempos
#   centro a  90 mm (flanco a 28)  -> sale siempre, 3-9 tiempos
#   centro a 100 mm (flanco a 38)  -> sale siempre, 2-5 tiempos
#   centro a 112+ mm               -> sale siempre, 2-3 tiempos
#
# Centrado en el fondo de la plaza el flanco queda a 37.5 mm de cada
# lado, que es EXACTAMENTE la holgura para la que el reglamento
# dimensiona la plaza (geometria_robot.HOLGURA_LATERAL). No es
# casualidad: la plaza esta dimensionada para que quepa la maniobra.
#
# Se mide desde el LiDAR, que va en el eje central, asi que el flanco
# esta a la lectura menos el semiancho.
# El reglamento obliga a 30 mm de flanco al muro negro. Se admite desde
# 22 para tolerar el ruido del LiDAR, pero por debajo la maniobra no sale.
# 5 mm, no 22. Con el flanco real (68) la lectura de 76 mm que da el
# robot aparcado son 8 mm de chapa a muro, no 32: o sea que la plaza se
# ocupa mucho mas pegado al muro exterior de lo que se creia. Exigir 22
# abortaria una colocacion que lleva toda la sesion saliendo. El margen
# de verdad lo pone HOLGURA_MINIMA_CUERPO, que vigila la maniobra entera
# por rumbo y contra la silueta.
MIN_COSTADO = LIDAR_A_FLANCO_CERCA + 5.0       # 73 mm de lectura

# Hueco a la COLA, medido por el ultrasonido. Ver la tabla de la cabecera:
# pegado al muro magenta trasero la maniobra no existe, y pegado al
# delantero tampoco.
# La colocacion de competencia deja la cola casi pegada, asi que el
# minimo es lo que hace falta para que exista maniobra (medido: con 5 mm
# no sale ni avanzando recto; con 10 si), no una posicion comoda.
COLA_MIN, COLA_MAX = 8.0, 95.0

# Alineacion final con el carril, mismo criterio que rompio el ciclo
# limite de las esquinas: se termina POR RUMBO, no por reloj.
# 6 grados Y SOSTENIDOS. Antes bastaba UN ciclo dentro de tolerancia, y
# el robot salia del bucle en mitad de un giro: cruzaba el umbral con
# velocidad angular, seguia rotando por inercia y entraba en la carrera
# atravesado en el carril. En video se ve saliendo de la plaza en
# diagonal cuando el log decia "a rumbo".
#
# Pidiendo CICLOS_ALINEADO seguidos, el bucle no puede terminar mientras
# el robot siga girando: la unica forma de encadenarlos es haber parado
# de rotar.
TOL_ALINEADO   = 6.0     # grados
CICLOS_ALINEADO = 6      # ciclos seguidos dentro de tolerancia
# Subida de 0.9 a 1.6 y con suelo: con la ganancia baja el ultimo tramo
# se arrastra -- la corrida buena acabo en 11.3 grados tras los 14 s, a
# solo 3 de la tolerancia, porque cerca del objetivo el comando se hace
# diminuto y el robot casi no gira.
KP_ALINEAR     = 1.6
MANDO_MIN_ALINEAR = 10.0   # por debajo de esto no gira, solo gasta pista
# 6 s no bastaban: la maniobra sale con ~86 grados acumulados y en la
# primera corrida buena la alineacion solo recupero 34. Girar 86 grados a
# radio 243 son ~365 mm de recorrido, y a esta velocidad eso es tiempo.
TIMEOUT_ALINEAR = 20.0

# ANCHO QUE DELATA QUE YA SE ESTA EN EL CARRIL. Los dos sectores laterales
# sumados, en mm. Medido: dentro de la plaza da 252 (sonda con el robot
# aparcado) y entre 400 y 630 mientras sale; en el carril de verdad da 818
# (log de las 12:21, t=7,59). El carril mide 1000 y el robot 138, asi que
# el valor sano ronda los 860. 700 separa las dos poblaciones con holgura.
ANCHO_CARRIL_MINIMO = 700.0

# El ajuste de pared salta: seis lecturas seguidas del robot quieto dieron
# 5,6 / 6,3 / 5,3 / 1,0 / 3,9 / 5,8 grados. Con TOL_ALINEADO=6 un solo
# barrido flojo declara "paralelo" en falso, asi que se decide sobre la
# MEDIANA de las ultimas lecturas, no sobre la ultima.
MUESTRAS_MURO = 5

HZ = 10.0


class Sensores:
    """LiDAR + Pico, con el ultimo perfil de 360 siempre fresco."""

    def __init__(self):
        self.enlace = EnlacePico()
        time.sleep(2.2)
        self._barridos = []
        self._corriendo = True
        self._drv = LidarDriver()
        threading.Thread(target=self._drv.hilo_lectura,
                         args=(lambda: self._corriendo, self._barridos.append),
                         daemon=True).start()

    def esperar(self, n=3, t_max=8.0):
        t0 = time.time()
        while len(self._barridos) < n and time.time() - t0 < t_max:
            time.sleep(0.05)
        return len(self._barridos) >= 1

    def perfil(self):
        if not self._barridos:
            return None
        return lg.construir_perfil_360(self._barridos[-1])

    def rango(self, centro, media_ventana):
        """Minimo en un sector, en grados y con 0 = frente."""
        p = self.perfil()
        if p is None:
            return None
        a = (centro - media_ventana) % 360.0
        b = (centro + media_ventana) % 360.0
        return lg.distancia_en_rango(p, a, b)

    def holgura_minima(self):
        """La chapa mas cerca de algo, en TODO el barrido. Devuelve
        (mm, rumbo) o (None, None).

        Esto es lo que faltaba. Los cortes por sector frontal y por
        ultrasonido solo miran dos direcciones, y dentro de la plaza el
        robot GIRA: a 30-70 grados de guiñada los muros magenta ya no
        estan ni delante ni detras, sino en diagonal, donde no miraba
        nadie. En la primera corrida que salio, los tiempos 4, 5 y 6
        cortaron por "tiempo maximo" -- o sea que corrieron 3 s enteros,
        unos 264 mm cada uno, sin que ningun sensor los vigilara. Ahi fue
        donde el robot movio el muro.

        Se descarta el sector del mastil, que es estructura propia; de esa
        zona se encarga el ultrasonido.
        """
        p = self.perfil()
        if p is None:
            return None, None
        libre = float("inf")
        h = []
        for a in range(360):
            d = p[a]
            if (MASTIL_MIN <= a <= MASTIL_MAX or d >= 4000.0
                    or d <= SILUETA[a] - MARGEN_ESTRUCTURA
                    or _es_rueda(a, d)):
                h.append(libre)
            else:
                h.append(d - SILUETA[a])

        # UN OBSTACULO OCUPA VARIOS GRADOS SEGUIDOS. Tomar el minimo crudo sobre 360
        # rumbos de UN barrido es tan sensible que un unico eco espurio
        # en toda la vuelta para el tramo -- y eso paso: 14 tiempos
        # cortados por "chapa a 9 mm en el rumbo 321" cuando ese rumbo, en
        # reposo y con mediana de 10 barridos, mide 179 mm y deja 110 de
        # holgura.
        #
        # Un muro magenta de 20 mm visto a 100 abarca unos 11 grados, o
        # sea 11 bins; un fantasma, uno. Asi que se toma el MAXIMO de cada
        # terna de bins vecinos antes de buscar el minimo: un bin solo
        # queda tapado por sus vecinos y un objeto de verdad sobrevive.
        # Se exige ANCHO_OBSTACULO bins seguidos por debajo del umbral,
        # no uno ni tres. Un muro magenta de 20 mm visto a 100 abarca
        # unos 11 grados; la franja 315-322 de este montaje es marginal
        # -- en reposo unas veces lee 179 mm y otras nada -- y ahi
        # aparecen retornos cortos sueltos en cuanto el robot se mueve.
        # Con 3 bins no bastaba: la corrida se paraba 14 veces seguidas
        # en el rumbo 321 por algo que la medida estatica dice que no
        # esta (se comprobo con el volante recto y a los dos topes: no es
        # la rueda).
        peor, rumbo = libre, None
        for a in range(360):
            v = max(h[(a + k) % 360] for k in range(ANCHO_OBSTACULO))
            if v < peor:
                peor, rumbo = v, (a + ANCHO_OBSTACULO // 2) % 360
        return (None, None) if rumbo is None or peor >= libre else (peor, rumbo)

    def trasera(self):
        return self.enlace.ultrasonido_mm()

    def rumbo(self):
        return self.enlace.heading()

    def angulo_muro(self, p=None):
        """Guiñada del chasis respecto a las paredes del carril, en
        grados, o None si ninguna de las dos es una pared plana.

        ESTO es "paralelo a las dos paredes", y es una medida ABSOLUTA:
        sale del LiDAR en este instante. El rumbo del giroscopio no
        sirve para esto -- la meta de la alineacion era volver al yaw con
        el que se aparco, y eso solo es paralelo al carril si el yaw NO
        ha derivado. Entre medias hay ocho tramos de adelante y atras con
        90 grados acumulados, arranques y frenadas: deriva.

        Se queda con el ajuste de MAS puntos, que es el mas fiable; el
        otro costado suele ser el hueco de la plaza o la esquina.
        """
        if p is None:
            p = self.perfil()
        if p is None:
            return None
        mejor, n_mejor = None, 0
        for sector in (lg.SECTOR_MURO_DER, lg.SECTOR_MURO_IZQ):
            pts = lg.puntos_de_sector(p, sector[0], sector[1])
            if len(pts) < n_mejor:
                continue
            ang = lg.angulo_de_pared(pts)
            if ang is not None:
                mejor, n_mejor = ang, len(pts)
        return mejor

    def parar(self):
        for _ in range(3):
            self.enlace.enviar(0, 0.0)
            time.sleep(0.02)

    def cerrar(self):
        """Suelta el hardware. IMPORTANTE para el encadenado: si el
        puerto del LiDAR no se cierra, `ronda_camara.py` no lo puede
        abrir despues y la carrera no arranca."""
        self._corriendo = False
        time.sleep(0.3)
        for cerrar_algo in (self.enlace.detener, self.enlace.cerrar,
                            self._drv.cerrar):
            try:
                cerrar_algo()
            except Exception:
                pass
        time.sleep(0.4)


def _mm(v):
    return "sin medida" if v is None else "%5.0f mm" % v


def describir(sen):
    """Lo que ve el robot ahora mismo, y por donde deduce la salida."""
    izq = sen.rango(270.0, 30.0)
    der = sen.rango(90.0, 30.0)
    fre = sen.rango(0.0, 20.0)
    tra = sen.trasera()
    print("  costado izquierdo %s | derecho %s" % (_mm(izq), _mm(der)))
    print("  frente %s | trasera (ultrasonido) %s" % (_mm(fre), _mm(tra)))
    h, r = sen.holgura_minima()
    if h is not None:
        print("  chapa mas cerca de algo: %.0f mm, en el rumbo %d "
              "(margen exigido %.0f)" % (h, r, HOLGURA_MINIMA_CUERPO))
    if izq is None or der is None:
        return None
    # El muro exterior es el costado CERRADO; la salida, el abierto. No
    # hace falta saber como quedo colocado el robot ni hacia donde se
    # sorteo la vuelta: 902 contra 78 no admite duda.
    s = 1.0 if der > izq else -1.0
    print("  -> muro exterior a la %s, SALIDA por la %s (relacion %.1f:1)"
          % ("IZQUIERDA" if s > 0 else "DERECHA",
             "DERECHA" if s > 0 else "IZQUIERDA",
             max(izq, der) / max(1.0, min(izq, der))))
    if tra is not None and not (COLA_MIN <= tra <= COLA_MAX):
        print()
        print("  [!] MAL COLOCADO A LO LARGO: %.0f mm de hueco por detras." % tra)
        print("      Tiene que quedar entre %.0f y %.0f. Pegado al muro magenta"
              % (COLA_MIN, COLA_MAX))
        print("      trasero la maniobra NO EXISTE, y pegado al delantero")
        print("      tampoco: hace falta sitio para el primer tiempo.")
        print("      Muevelo %s unos %.0f mm."
              % ("ADELANTE" if tra < COLA_MIN else "ATRAS",
                 abs((COLA_MIN + COLA_MAX) / 2.0 - tra)))
        return None
    cerca = min(izq, der)
    if cerca < MIN_COSTADO:
        print()
        print("  [!] DEMASIADO PEGADO AL MURO EXTERIOR: %.0f mm de lectura,"
              % cerca)
        print("      o sea el flanco a %.0f mm. Hacen falta %.0f de lectura"
              % (cerca - LIDAR_A_FLANCO_CERCA, MIN_COSTADO))
        print("      (flanco a 22 mm) para que la maniobra tenga margen.")
        print("      Ver MIN_COSTADO: no es comodidad, es que pegado al muro")
        print("      cualquier rotacion mete una esquina contra el y los dos")
        print("      sentidos utiles quedan bloqueados.")
        print("      Centra el robot en el fondo de la plaza y repite.")
        return None
    return s


def fuera_de_la_plaza(sen, s_salida):
    """Ya no hay muro exterior pegado ni muro magenta por delante."""
    costado = sen.rango(270.0 if s_salida > 0 else 90.0, 30.0)
    if costado is None or costado < SALIDA_COSTADO:
        return False
    fre = sen.rango(0.0, 30.0)
    return fre is not None and fre > SALIDA_LIBRE


def vaiven(sen, s_salida, seco):
    """Los tiempos de la maniobra.

    Adelante con el volante HACIA la salida, atras con el volante AL
    CONTRARIO: los dos rotan el chasis en el mismo sentido y cada par
    gana angulo neto. Cada tramo se corta por DISTANCIA MEDIDA -- nunca
    por reloj -- y ademas por angulo, porque pasado cierto giro el tramo
    deja de aportar y lo que hace falta es invertir el sentido.
    """
    rumbo0 = sen.rumbo()
    for t in range(1, TRAMOS_MAX + 1):
        adelante = (t % 2 == 1)
        # Positivo = izquierda. s_salida +1 = el carril esta a la
        # DERECHA, y a la derecha se va con comando NEGATIVO.
        if adelante:
            cmd, vel = (-TOPE_DER, VEL_MANIOBRA) if s_salida > 0 else (TOPE_IZQ, VEL_MANIOBRA)
        elif REVERSA_RECTA:
            # IDEA DEL EQUIPO: retroceder EN RECTO en vez de con el
            # volante al contrario. El razonamiento es solido -- en recto
            # la reversa no puede DESHACER la guiñada que el avance
            # gano, solo devolver sitio -- y ataca justo lo que se ve en
            # pista: la alternancia con volante contrario gana 5-8 grados
            # por par cuando la geometria dice 22.
            #
            # En simulacion se bloquea al segundo tiempo (el chasis queda
            # en diagonal y la cola se mete contra el muro), pero esa
            # misma simulacion predice 22 grados donde el robot hace 8,
            # asi que no refuta nada. Se prueba en pista, que es quien
            # manda.
            cmd, vel = 0.0, VEL_REVERSA
        else:
            cmd, vel = (TOPE_IZQ, VEL_REVERSA) if s_salida > 0 else (-TOPE_DER, VEL_REVERSA)
        giro_pleno = cmd
        r_ini = sen.rumbo()
        print("[TIEMPO %d] %-8s volante %+5.1f" %
              (t, "adelante" if adelante else "atras", cmd))
        if seco:
            print("           modo seco: no se manda nada")
            continue

        # VOLANTE PRIMERO, CON EL ROBOT PARADO. Ver ESPERA_SERVO.
        t_servo = time.time()
        while time.time() - t_servo < ESPERA_SERVO:
            sen.enlace.enviar(0, cmd)
            time.sleep(1.0 / HZ)

        # TOPE DE GIRO HACIA ADELANTE, que lo fija el hueco de la cola.
        giro_tope = GIRO_MAX_TRAMO
        if adelante:
            hueco = sen.trasera()
            if hueco is not None:
                util = max(0.0, hueco - MARGEN_ESQUINA_TRASERA)
                giro_tope = min(GIRO_MAX_TRAMO,
                                math.degrees(math.asin(min(1.0, util / SEMIANCHO_REAL))))
                print("           cola %.0f mm -> como mucho %.1f grados en este tramo"
                      % (hueco, giro_tope))

        # TOPE DE RECORRIDO PARA LA REVERSA, medido con el robot quieto.
        limite = TIMEOUT_TRAMO
        if not adelante:
            hueco = sen.trasera()
            if hueco is not None:
                util = max(0.0, hueco - MARGEN_RECORRIDO_ATRAS)
                limite = min(TIMEOUT_TRAMO, util / MM_POR_SEG)
                print("           hueco medido atras %.0f mm -> %.0f mm utiles, "
                      "%.1f s como mucho" % (hueco, util, limite))

        t0 = time.time()
        motivo = "tiempo maximo del tramo"
        recto = False
        while time.time() - t0 < limite:
            # RECTO MIENTRAS LA COLA NO TENGA SITIO. Ver COLA_PARA_GIRAR:
            # girando hacia la salida la cola barre hacia atras, asi que
            # con el poste magenta encima el tramo se bloquea antes de
            # ganar un grado. Se reevalua cada ciclo: en cuanto hay hueco,
            # entra el volante.
            tra = sen.trasera()
            if (not ARRANCAR_GIRANDO and adelante
                    and tra is not None and tra < COLA_PARA_GIRAR):
                cmd = 0.0
                if not recto:
                    print("           recto: la cola solo tiene %.0f mm" % tra)
                    recto = True
            else:
                if recto:
                    print("           ya hay %.0f mm detras, entra el volante" % (tra or 0.0))
                    recto = False
                cmd = giro_pleno
            sen.enlace.enviar(vel, cmd)
            time.sleep(1.0 / HZ)
            # CADA EXTREMO SE VIGILA CUANDO SE ACERCA A ALGO, NO SIEMPRE.
            #
            # Girando hacia adelante la COLA barre hacia atras, y girando
            # en reversa el MORRO barre hacia adelante: por eso no basta
            # con mirar el sentido de la marcha. Pero EN RECTO no barre
            # nada -- la cola sigue al morro -- y su hueco solo puede
            # CRECER.
            #
            # Vigilar la cola tambien en recto fue un error que costo una
            # corrida entera: con el poste magenta a 10 mm y el margen en
            # 12, el corte saltaba en el PRIMER ciclo y el tramo no
            # llegaba a moverse nunca. Se bloqueaba justo el movimiento
            # que resuelve el problema. 14 tiempos cortando todos por
            # "cola a 9-11 mm" con la guiñada en -2 grados.
            # LA CHAPA MAS CERCA DE ALGO, MIRE DONDE MIRE. Ver
            # Sensores.holgura_minima: es el unico corte que sigue
            # valiendo cuando el robot esta cruzado dentro de la plaza.
            h, rumbo_h = sen.holgura_minima()
            if h is not None and h < HOLGURA_MINIMA_CUERPO:
                pf = sen.perfil()
                vecindad = " ".join(
                    "%d:%s" % (b % 360,
                               "%.0f" % pf[b % 360] if pf[b % 360] < 4000 else ".")
                    for b in range(rumbo_h - 5, rumbo_h + 6))
                motivo = ("chapa a %.0f mm en el rumbo %d" % (h, rumbo_h)
                          + chr(10) + "           perfil: " + vecindad)
                break

            # EL CORTE POR SECTOR FRONTAL SE QUITA. Tomaba el MINIMO de
            # todo el cono de +-45 grados, asi que veia el muro magenta
            # aunque el robot fuera a pasar por su lado -- y es el que
            # estaba frenando los tramos a 5 grados cuando la geometria
            # daba 22. El corte de cuerpo de arriba ya cubre el frente, y
            # lo hace por rumbo contra la silueta, que es lo correcto.
            # Queda el ultrasonido para el sector ciego del mastil, que es
            # el unico sitio donde el LiDAR no puede ayudar.
            girando = abs(cmd) > 0.15
            if not adelante or girando:
                d = sen.trasera()
                if d is not None and d < MARGEN_TRASERA:
                    motivo = "cola a %.0f mm del muro" % d
                    break
            if abs(sen.rumbo() - r_ini) > giro_tope:
                motivo = ("el tramo ya giro %.0f grados (tope %.0f)"
                          % (abs(sen.rumbo() - r_ini), giro_tope))
                break
            if fuera_de_la_plaza(sen, s_salida):
                sen.parar()
                print("           FUERA de la plaza")
                return True
        sen.parar()
        time.sleep(PAUSA_ENTRE_TRAMOS)
        if motivo == "tiempo maximo del tramo" and not adelante:
            motivo = "agotado el hueco medido"
        acumulada = abs(sen.rumbo() - rumbo0)
        print("           corta por %s | guiñada acumulada %+.1f grados"
              % (motivo, sen.rumbo() - rumbo0))
        if acumulada >= GUIÑADA_MAX_SALIDA:
            print("           %.0f grados de guiñada: BASTA de vaiven. "
                  "Enderezar es trabajo de alinear(); seguir aqui saca el "
                  "robot perpendicular al carril." % acumulada)
            return True
        if fuera_de_la_plaza(sen, s_salida):
            print("           FUERA de la plaza")
            return True
    return False


def _anuncio(anterior, clave, texto):
    """Un aviso por CAMBIO de situacion, no uno por ciclo. La clave es
    estable; el texto lleva los numeros, que cambian siempre."""
    if clave != anterior:
        print("          %s" % texto)
    return clave


def _hueco_para_avanzar(sen, que, sector=SECTOR_AVANCE):
    """Vigilancia del avance: sector frontal y silueta completa.

    El corte por sector frontal solo mira una direccion, y aqui el robot
    va girado: la isla o un muro magenta aparecen en diagonal, donde ese
    sector no llega. Es la misma leccion que el vaiven.

    `sector` se estrecha mientras se busca el carril. Con los +-45 de
    siempre el guardian se disparaba en el primer ciclo SIN que hubiera
    nada delante: dentro de la plaza la pared exterior esta a 79 mm de
    costado, y a 45 grados eso son 79/sin45 = 112 mm, por debajo del
    umbral. Estaba midiendo el costado y llamandolo frente -- el mismo
    error de comparar magnitudes tomadas en rumbos distintos que ya
    aparecio tres veces en la carrera. La vigilancia real ahi es la
    silueta, que compara cada rumbo contra la chapa que hay en ESE rumbo.
    """
    d = sen.rango(0.0, sector)
    if d is not None and d < MARGEN_FRENTE + 60.0:
        sen.parar()
        print("          %s: pared a %.0f mm, se corta" % (que, d))
        return False
    h, rumbo = sen.holgura_minima()
    if h is not None and h < HOLGURA_MINIMA_CUERPO:
        sen.parar()
        print("          %s: chapa a %.0f mm en el rumbo %d, se corta"
              % (que, h, rumbo))
        return False
    return True


def alinear(sen, rumbo_meta, s_salida, seco):
    """Enderezarse con el carril: PARALELO A LAS PAREDES.

    El criterio ya no es el yaw sino `angulo_muro`, que sale del LiDAR y
    no deriva. Y no termina hasta que HAY una pared que ajustar: eso es
    lo que faltaba.

    MEDIDO en la corrida de las 12:21. `fuera_de_la_plaza` daba por
    terminada la salida con solo un costado y el frente despejados, y eso
    se cumple con el morro fuera y el cuerpo todavia dentro. El log de la
    carrera arranca asi:

        t=4,17  izq 374  der 2000   sin ajuste de pared valido
        t=4,81  izq 330  der   71   suman 401 sobre un carril de 1000
        t=7,59  izq 578  der  240   suman 818: ESTO ya es el carril

    El primer `muro_ok` no llega hasta t=7,29, o sea 3,1 s despues de
    entregar el mando -- y en esos 3 s la FSM de carrera se mete en
    COMPROMISO, APERTURA, RETROCESO y REINTENTO peleando por salir de un
    bolsillo que ella no sabe que existe. De ahi sale cruzada, y cruzada
    llega al primer pilar.

    Asi que mientras no haya pared no se endereza: se SALE. El yaw ya no
    decide nada, solo se informa.
    """
    print("[ALINEAR] meta: paralelo a las paredes (respaldo: rumbo %.1f)"
          % rumbo_meta)
    if seco:
        print("          modo seco: no se manda nada")
        return True
    t0 = time.time()
    dentro = 0
    busca = None
    hist = collections.deque(maxlen=MUESTRAS_MURO)
    while time.time() - t0 < TIMEOUT_ALINEAR:
        perfil = sen.perfil()
        izq = der = None
        if perfil is not None:
            izq = lg.distancia_en_rango(perfil, *lg.SECTOR_MURO_IZQ)
            der = lg.distancia_en_rango(perfil, *lg.SECTOR_MURO_DER)
        ancho = (izq + der) if (izq and der) else 0.0
        hist.append(sen.angulo_muro(perfil))
        buenas = sorted(a for a in hist if a is not None)
        amuro = buenas[len(buenas) // 2] if len(buenas) >= 3 else None

        # ESTAR PARALELO NO ES ESTAR EN EL CARRIL, y hacen falta las dos.
        # Aparcado, el ajuste de pared da unos 5 grados perfectamente
        # validos -- es la pared exterior, que el robot tiene a 79 mm --
        # mientras los costados suman 252. Mirando solo el angulo, la
        # salida se daba por buena sin haber salido.
        en_carril = ancho >= ANCHO_CARRIL_MINIMO
        if not _hueco_para_avanzar(sen, "saliendo" if not en_carril else "enderezando",
                                   SECTOR_AVANCE if en_carril else SECTOR_BUSQUEDA):
            return False

        if amuro is None:
            # A CIEGAS: no hay ninguna recta que ajustar, asi que no se
            # sabe hacia donde se apunta. Se gira al hueco para asomarse.
            # Los primeros ciclos la mediana aun no tiene muestras; girar
            # ahi seria decidir sin haber medido, asi que se va recto.
            dentro = 0
            # A CIEGAS SE VA RECTO, NO SE GIRA.
            #
            # Antes esta rama giraba al tope hasta que apareciera una
            # pared. Medido en la corrida de las 13:13: el vaiven termino
            # con +91,7 grados de guiñada y `alinear` entrego con yaw
            # 184,6 -- o sea que se comio OTROS 93 grados girando a
            # ciegas, 22 de los 42 segundos de la salida, y dejo al robot
            # mirando al reves del carril. En el video de Daniel se ve:
            # a los 19,5 s el robot ya esta fuera y apuntando bien, y
            # despues da vueltas por el tapete hasta los 42.
            #
            # Girar a ciegas es decidir sin medir. Recto es la unica
            # accion que no empeora el rumbo, y ademas es la que saca del
            # bolsillo: la plaza tiene 200 mm de fondo. En cuanto asome
            # al carril habra pared que ajustar y manda el servo de
            # abajo, que si mide.
            busca = _anuncio(busca, "ciego",
                             "sin pared fiable (%d de %d barridos): recto"
                             % (len(hist), MUESTRAS_MURO))
            sen.enlace.enviar(VEL_MANIOBRA, 0.0)
            time.sleep(1.0 / HZ)
            continue

        # `angulo_muro` positivo = chasis girado a la IZQUIERDA respecto a
        # la pared, y hacia adelante se corrige con el signo cambiado
        # (mismo convenio que _rumbo_nominal en la carrera).
        err = -amuro
        if abs(err) < TOL_ALINEADO:
            if en_carril:
                dentro += 1
                if dentro >= CICLOS_ALINEADO:
                    sen.parar()
                    print("          PARALELO: error %.1f grados sostenido %d "
                          "ciclos, costados %.0f mm (yaw %.1f, meta vieja %.1f)"
                          % (err, dentro, ancho, sen.rumbo(), rumbo_meta))
                    return True
                if busca is not None:
                    busca = None
                    print("          en el carril: costados %.0f mm, pared a "
                          "%.1f grados" % (ancho, amuro))
            else:
                # PARALELO PERO TODAVIA DENTRO. Aqui girar es justo lo que
                # no toca: la plaza tiene 200 mm de fondo contra la pared
                # exterior, asi que se sale de ella avanzando RECTO a lo
                # largo de esa pared. Girar al tope para "buscar" costaba
                # 45 grados de guiñada que luego habia que deshacer
                # (medido en simulacion).
                dentro = 0
                busca = _anuncio(busca, "recto",
                                 "paralelo pero aun dentro (costados %.0f mm): "
                                 "saliendo recto" % ancho)
            sen.enlace.enviar(VEL_MANIOBRA, 0.0)
            time.sleep(1.0 / HZ)
            continue

        dentro = 0
        cmd = err * KP_ALINEAR
        if abs(cmd) < MANDO_MIN_ALINEAR:
            # Por debajo de esto el servo no mueve la rueda, solo gasta
            # pista. Es un suelo fisico, y fija el error residual: a
            # VEL_MANIOBRA un ciclo con 10 grados de mando son 2,4 grados
            # de guiñada, asi que TOL_ALINEADO no puede bajar de ahi.
            cmd = MANDO_MIN_ALINEAR if cmd > 0 else -MANDO_MIN_ALINEAR
        cmd = max(-TOPE_DER, min(TOPE_IZQ, cmd))
        sen.enlace.enviar(VEL_MANIOBRA, cmd)
        time.sleep(1.0 / HZ)
    sen.parar()
    print("          tiempo maximo SIN quedar paralelo (la carrera va a "
          "arrancar torcida)")
    return False


def main():
    fase = sys.argv[1] if len(sys.argv) > 1 else "salida"
    seco = "--seco" in sys.argv
    if fase != "salida":
        print("por ahora solo esta hecha la fase 'salida'")
        return 1

    sen = Sensores()

    def al_cortar(*_):
        sen.parar()
        sen.cerrar()
        sys.exit(0)
    signal.signal(signal.SIGINT, al_cortar)

    if not sen.esperar():
        print("[-] sin barridos del LiDAR")
        sen.cerrar()
        return 1

    print("=== LO QUE VE AHORA ===")
    s_salida = describir(sen)
    if s_salida is None:
        print("[-] no puedo decidir el lado de salida")
        sen.cerrar()
        return 1
    rumbo0 = sen.rumbo()
    print("  rumbo de aparcado %.1f grados: esa es la meta de la alineacion,"
          % rumbo0)
    print("  porque el robot aparca PARALELO al carril.")
    print("  robot %.0fx%.0f mm (medido) | radios minimos %.1f izq / %.1f der"
          % (LARGO_REAL, ANCHO_REAL, gev.RADIO_MIN_IZQ, gev.RADIO_MIN_DER))
    print()

    print("=== %s ===" % ("PLAN (modo seco: el robot NO se mueve)" if seco
                          else "MANIOBRA"))
    ok = vaiven(sen, s_salida, seco)
    print()
    if ok or seco:
        recto = alinear(sen, rumbo0, s_salida, seco)
        print()
        if recto or seco:
            print("[LISTO] en el carril y paralelo a las paredes. "
                  "Aqui entra la FSM de carrera.")
        else:
            print("[OJO] se entrega el mando SIN estar paralelo al carril.")
    else:
        print("[-] no consiguio salir en %d tiempos" % TRAMOS_MAX)
    sen.parar()
    sen.cerrar()
    return 0


if __name__ == "__main__":
    sys.exit(main())
