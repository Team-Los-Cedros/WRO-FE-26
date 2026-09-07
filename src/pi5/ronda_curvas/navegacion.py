# Cerebro de la Ronda Cerrada. La clase Navegador recibe por cada barrido
# del LiDAR la medicion, el color de la camara y el yaw de la IMU, y
# devuelve la consigna (velocidad, angulo) para la Pico. No abre puertos
# ni hilos, asi que se puede probar fuera del robot con barridos grabados.
#
# Fases: CAPTURA_FIRMA -> CARRERA -> PARQUEO -> FIN
#
# ==========================================================
# POR QUE ESTA VERSION NO SE PARECE A LA ANTERIOR
# ==========================================================
# La evasion anterior (CRUCERO -> APROXIMACION -> SOBREPASO ->
# REINCORPORACION) funcionaba en recta y fallaba en las esquinas con dos
# pilares. El fallo no era de ganancias. Eran cinco cosas, y todas se
# arreglan con geometria, no subiendo angulos ni tiempos:
#
#   1. SOBREPASO congelaba el rumbo del instante en que el poste llegaba
#      al costado (`_heading_sobrepaso = heading`) y lo mantenia con un
#      lazo P. En recta ese rumbo es aproximadamente la tangente del
#      carril; en una esquina apunta a la concavidad del muro. O sea que
#      el estado que debia rebasar el pilar conducia activamente hacia la
#      esquina durante TIMEOUT_SOBREPASO entero.
#   2. "Poste superado" era `y < -280mm` en marco CHASIS. En una curva
#      cerrada el robot rota mucho y avanza poco, y una rotacion pura
#      basta para llevar y por debajo de -280 sin haber rebasado nada.
#      Lo mismo `al_costado()` y `trk.y < Y_POSTE_EN_PASO`, que ademas
#      no comprobaba de que lado quedaba el pilar.
#   3. La emergencia hacia `tracker.desactivar("emergencia")`. Como en la
#      esquina el anticolision es justo lo que dispara, cada excursion
#      terminaba borrando color, lado y fase: el robot salia del
#      RETROCESO sin ninguna memoria de que estaba rodeando un pilar.
#   4. El tracker asociaba con UNA puerta de 250mm en distancia euclidea
#      sobre cualquier cluster de obstaculo. La quilla de una esquina
#      pasa ese filtro, y el objetivo saltaba del pilar al muro sin que
#      nada lo registrara.
#   5. `_con_seguridad_pared()` MEZCLABA el comando de evasion con el
#      centrado de paredes, y a 120mm el centrado mandaba del todo. En
#      una concavidad las dos paredes estan cerca, asi que la mezcla
#      borraba la intencion de rodear el pilar y podia invertir su signo
#      sin dejar rastro.
#
# Las tres responsabilidades ahora estan separadas y NO se suman:
#
#   TRAYECTORIA  seguir el carril             -> _rumbo_nominal()
#   REGLA        el pilar por su lado         -> geometria_evasion.py
#   SEGURIDAD    no tocar paredes ni pilar    -> _arbitrar()
#
# _arbitrar() no corrige el comando deseado: construye el CONJUNTO de
# comandos que las paredes permiten, se queda con los que ademas libran
# el pilar, y elige el mas parecido a lo que la maniobra pedia. Si el
# conjunto compatible con el pilar queda vacio, la maniobra se SUSPENDE
# de forma explicita (y se registra), en vez de disolverse en una mezcla.
#
# Estados de CARRERA:
#   CRUCERO         centrado entre paredes
#   CONFIRMACION    hay un pilar candidato; todavia no hay compromiso
#   COMPROMISO      un ciclo: congela identidad, color, lado y rumbo
#   APERTURA        ganar el radio que hace posible la envolvente
#   CONTRAGIRO      invertir el volante y meter el pilar en el circulo
#   PASO_LATERAL    sostener la envolvente hasta cumplir el invariante
#   SALIDA_PILAR    recuperar la tangente de salida de la curva
#   RECUPERACION    volver al centro del carril por POSICION
#   ABORTO          la maniobra no se puede terminar: salida definida
#   RETROCESO       emergencia anti-choque (suspende, no borra)
#   GIRO_FORZADO    desempate de esquina simetrica
#
# Angulo positivo = giro a la izquierda EN MARCHA ADELANTE. En reversa el
# mismo angulo de rueda gira el chasis al sentido contrario (geometria
# Ackermann), por eso RETROCESO mide en vivo que diagonal trasera tiene
# mas espacio en vez de usar un signo fijo.
import math
import time

import geometria_evasion as gev
import geometria_robot as geo
import optica
import sentido_vuelta
import tracker as tracker_mod
from lidar_geometria import ancho_cluster, centroide_xy_cluster, es_objeto_estrecho

# ==========================================
# VELOCIDADES (% PWM)
# ==========================================
VELOCIDAD_CRUCERO  = 55
VELOCIDAD_EVASION  = 40
VELOCIDAD_PARQUEO  = 20
VELOCIDAD_REVERSA  = -35
# PROBADO Y REVERTIDO el 2026-08-29 (corrida 153206). Se bajo a 18 (72
# mm/s) para dar mas tiempo de reaccion en las curvas. NO SIRVIO, y la
# corrida lo demuestra de forma limpia: la velocidad angular al tope
# derecho paso de 16.8 a 11.9 grados/s, y 16.8 * 18/25 = 12.1. O sea que
# el giro escala EXACTAMENTE con la velocidad: el radio es el mismo y el
# robot recorre la misma circunferencia, solo que mas despacio.
#
# Ese resultado es la base de geometria_evasion.py: FRENAR NO CAMBIA LA
# GEOMETRIA. Lo unico que compra frenar son ciclos de control por
# milimetro recorrido y distancia de parada.
VELOCIDAD_MINIMA   = 25      # piso del frenado progresivo

# ==========================================
# SEGUIMIENTO DE PARED Y FRENADO
# ==========================================
KP_LATERAL = 0.14            # calibrado en pista, mismo valor que la Ronda Abierta

# Tope del termino de POSICION dentro del control de pista. El error
# (izquierda - derecha) no esta acotado: en una esquina el carril se abre
# hasta 1400mm y con KP_LATERAL eso pide del orden de 200 grados de
# servo. El repositorio ya habia medido el windup que eso produce
# (comando de -78 grados mientras el error ya bajaba, 60% de una corrida
# en saturacion), pero el recorte era al tope del servo, y recortar ahi
# no arregla nada: el termino de posicion se come TODA la autoridad y no
# queda ninguna para el termino de alineacion.
#
# El sintoma, medido en simulacion: con el robot desalineado, izq-der se
# queda grande de forma sostenida, el comando se clava en +25 y el robot
# describe circulos completos dentro del pasillo (heading de 26 a 473
# grados sin dejar de girar a la izquierda). Recortando la posicion a 12
# grados, la asistencia por angulo_muro (que llega a 15) puede ganarle y
# reorientar el chasis en vez de seguir persiguiendo un centro que con
# ese rumbo no significa nada.
#
# 12 grados equivalen a 86mm de error lateral a plena autoridad: de sobra
# para centrarse en un carril de 1000mm.
MAX_APORTE_POSICION = 12.0

# Asistencia de esquina por angulo_muro (ver _rumbo_nominal). Con la
# lectura estable medida en pista (-22 grados apuntando a una esquina
# real) aporta 13-15 grados hacia el lado que se abre.
KP_ANGULO_MURO = 0.65
DIST_ASISTENCIA_ESQUINA = 900.0
MAX_APORTE_ANGULO_MURO = 15.0

DIST_FRENADO_INICIO = 900.0  # mm, empieza a bajar velocidad
DIST_FRENADO_MIN    = 300.0  # mm, velocidad minima alcanzada

# ==========================================
# ENVOLVENTE DE SEGURIDAD (paredes)
# ==========================================
# Margen del CUERPO a la pared. Los sectores del LiDAR miden desde el
# sensor, no desde el costado: con `izquierda` = 80mm (EMERGENCIA_LATERAL)
# el flanco esta ya a 17mm de la pared. Por eso la envolvente predictiva
# trabaja con 110mm, que deja ~48mm de chasis, y la emergencia se queda
# como ultimo recurso por debajo.
MARGEN_PARED = 110.0

# Suelo duro. Cuando cumplir el lado obligatorio del pilar es imposible
# con el margen preferido, se APURA hasta este otro antes de dar el
# conflicto por irresoluble. Es la unica concesion ordenada entre las dos
# restricciones: primero se intenta con holgura comoda, luego se aprieta,
# y por debajo de aqui no se baja nunca (queda por encima de
# EMERGENCIA_LATERAL, que es el ultimo recurso reactivo).
#
# Hace falta porque hay geometrias legales en las que el pilar deja un
# hueco estrecho contra el muro exterior de una esquina: exigir 110mm ahi
# convierte una maniobra posible en un aborto, y un aborto deja al robot
# pasando por el lado prohibido.
MARGEN_PARED_DURO = 85.0

# El termino de deriva de lateral_predicho usa angulo_muro, y esa señal
# NO es la guiñada real: es fiable apuntando a una esquina, y ruido el
# resto del tiempo. Medido en la corrida 132009, en los ciclos sin salida
# su mediana era 22.2 grados y su p90 35.3 -- o sea que el predictor
# estaba dando por hecho un desvio permanente de 20 grados que no
# existia, y con el horizonte viejo eso son 85mm de deriva inventada por
# ciclo. Con 12 grados el termino sigue capturando un morro realmente
# torcido sin poder por si solo vaciar el conjunto admisible.
MAX_DERIVA_MURO = 12.0

# Cuanto se mira hacia delante al comprobar un comando. Es distancia, no
# tiempo, porque la trayectoria no depende de la velocidad; el tiempo
# solo entra para no mirar mas lejos de lo que se puede corregir.
# El horizonte de la RESTRICCION es una distancia de REACCION, no de
# planificacion: hasta donde llega el robot antes de poder cambiar el
# comando de forma efectiva (unos 3 ciclos de servo, 0.35s, mas margen).
# Quien anticipa es el termino de esquina de _rumbo_nominal, que usa el
# criterio de circunferencia completa.
#
# Estaba en 250mm / 1.5s, que es horizonte de planificacion, y con eso la
# restriccion se comia el pasillo entero: medido en la corrida 132009,
# 160 de 664 ciclos (24%) sin ningun comando admisible, con las paredes a
# 310mm y el frente a 294. En un pasillo de 726mm de mediana -- el de
# este banco de pruebas, no los 1000mm nominales -- pedir que a 250mm
# vista queden 110mm libres a los dos lados no lo cumple ninguna
# trayectoria.
HORIZONTE_MIN = 180.0        # mm
HORIZONTE_SEG = 0.9          # s de marcha a la velocidad actual

# El horizonte LATERAL y el FRONTAL no son la misma magnitud, y usar uno
# solo para los dos es lo que vaciaba el conjunto admisible.
#
# El lateral mide DERIVA: cuanto se desplaza el costado mientras el
# comando esta en vigor. 180mm es correcto ahi -- con 120mm la matriz de
# esquinas detecta choque contra el muro en la diagonal concava.
#
# El frontal mide otra cosa: si el MORRO cruza la pared antes de poder
# cambiar de comando. Y ahi el 180 no es conservador, es imposible.
# La cuenta: alcance_frontal(inf, h) = h + X_MORRO, y lo disponible es
# frontal_muro + LIDAR_X - MARGEN_FRENTE, o sea que existe algun comando
# admisible solo si frontal_muro >= h + X_MORRO - LIDAR_X + MARGEN_FRENTE
# = h + 74. Con h=180 eso son 242.6mm de pared frontal POR DEBAJO DE LOS
# CUALES EL CONJUNTO ESTA VACIO POR CONSTRUCCION, gire como gire.
#
# Medido en la corrida 132913: las 12 rachas de SIN_SALIDA entraron con
# frontal_muro entre 213 y 242mm -- todas pegadas a ese umbral de 242.6,
# que no es casualidad sino la definicion de la restriccion. En 10 de
# las 12 quedaban entre 12 y 19 comandos admisibles POR LOS LADOS (los
# 19 de la rejilla completa en cinco de ellas): el robot retrocedia con
# el pasillo entero libre porque el frente pedia avanzar 180mm sin tocar
# cuando solo se compromete durante la reaccion.
#
# El horizonte frontal es distancia de REACCION: 3 ciclos de servo
# (0.35s) mas colchon. A 100mm/s reales son 35mm, asi que manda el piso.
#
# El piso lo acotan las dos fuentes por los dos lados, y 150 es el unico
# valor que satisface a las dos:
#   - por abajo lo fija la matriz de esquinas, no una preferencia. Con
#     120 y con 135 el robot mete el morro en el muro exterior de la
#     esquina superior derecha (esquina del cuerpo en y=1504 con el muro
#     en 1500) durante el GIRO_FORZADO. Con 150 la misma pista se
#     termina en CRUCERO sin tocar nada.
#   - por arriba lo fija la pista: 180 pone el umbral en 242.6mm, que es
#     exactamente donde entraron las 12 rachas de SIN_SALIDA.
# Se probo 150 en la corrida 133838 y la hipotesis quedo FALSIFICADA.
# El umbral bajo a 223.7mm exactamente como predecia la cuenta, y las
# rachas bajaron con el: entraron con frontal_muro entre 190 y 223 en
# vez de entre 213 y 242. Pero fueron 10 rachas en vez de 12 y
# SIN_SALIDA subio del 26.9% al 32.6%: parar mas cerca de la pared
# alarga la recuperacion (22 ciclos por racha contra 14).
#
# O sea que el umbral no es la causa: el robot avanza contra la pared
# hasta que ALGO le para, y mover el punto donde le para solo mueve
# donde ocurre el sintoma. La causa esta aguas arriba, en el rumbo
# nominal (ver MAX_DERIVA_MURO). El horizonte frontal se deja en 180,
# que es el valor medido menos malo y el que mas margen le deja a la
# matriz de esquinas; la SEPARACION de los dos horizontes se conserva
# porque miden magnitudes distintas y el dia que el rumbo nominal deje
# de empotrarse habra que volver a bajarlo.
HORIZONTE_FRENTE_MIN = 180.0   # mm
HORIZONTE_FRENTE_SEG = 0.35    # s: 3 ciclos de servo, la reaccion real

# Con una pared de frente mas cerca que esto, el comando tiene que poder
# doblar ANTES de llegar a ella. La condicion exacta la da el barrido del
# cuerpo: en un arco sostenido de radio R, el punto del robot que mas se
# adelanta es la esquina delantera exterior, y su alcance maximo es
# radio_exterior(R) = hypot(R + semiancho, morro). Si eso cabe en lo que
# queda hasta la pared, el arco dobla a tiempo.
#
# Es la version geometrica del "escape frontal" anterior, que era una
# rampa de urgencia MEZCLADA sobre el centrado. La diferencia practica:
# aqui el frente no corrige el comando, RECORTA el conjunto de comandos
# admisibles, asi que no puede tapar la intencion de la maniobra sin que
# se note (queda como recorte en el log).
#
# Consecuencia medible y util: por debajo de HORIZONTE_FRENTE_MIN + 74mm
# de pared frontal (223.7mm con el piso actual) no cabe ningun comando y
# la unica accion segura es la marcha atras, que el sistema DICE en vez
# de quedarse empujando contra la pared. Ese umbral es la definicion de
# la restriccion, no una heuristica: sale de igualar alcance_frontal al
# hueco disponible. Ver HORIZONTE_FRENTE_MIN para por que no puede
# compartir horizonte con la envolvente lateral.
DIST_FRENTE_EXIGE_GIRO = 650.0
MARGEN_FRENTE = 40.0         # colchon contra la pared frontal

# Colchon EXTRA que se pide al termino de esquina de la trayectoria
# nominal, por encima del que exige la envolvente de seguridad. Asi el
# arco que se comanda no va pegado al limite de lo admisible: la
# restriccion queda como red, no como consigna.
MARGEN_ESQUINA = 60.0

# Rejilla de comandos candidatos que se evalua cada ciclo. 2.5 grados es
# la quinta parte del salto que el limitador del servo permite por ciclo
# (MAX_DELTA_ANGULO=12), asi que la rejilla nunca es el factor limitante.
# 19 evaluaciones de hypot por ciclo a 8.5 Hz no se notan en la Pi 3B.
PASO_REJILLA = 2.5

# ==========================================
# MANIOBRA DE PILAR
# ==========================================
# Holgura de diseño entre el CUERPO del robot y la superficie del pilar.
# geometria_evasion suma ademas el semiancho del chasis y el radio del
# poste, asi que esto es margen limpio.
HOLGURA_BASE = 40.0

# La holgura crece con la incertidumbre del tracker: una estimacion peor
# obliga a una envolvente mas ancha. Es el acoplamiento explicito entre
# confianza y geometria que faltaba -- antes, una estimacion mala producia
# exactamente la misma trayectoria que una buena.
HOLGURA_POR_SIGMA = 1.5

# Separacion lateral minima que hay que poder DEMOSTRAR para dar el paso
# por bueno. Es el minimo geometrico puro: medio chasis + medio poste +
# holgura. Por debajo de esto el robot y el pilar se tocan.
SEPARACION_MIN_VALIDA = gev.SEMIANCHO + gev.RADIO_POSTE + HOLGURA_BASE   # 137.9 mm

# Objetivo de apertura: lo que la geometria exige (separacion_requerida)
# mas este colchon, para no quedarse justo en el limite de factibilidad.
MARGEN_APERTURA = 60.0

# Barrido minimo del rumbo del pilar en marco MUNDO para considerarlo
# rodeado. Empezando con el pilar al frente (barrido 0) y terminando con
# el detras, el barrido geometrico completo son 180 grados; 100 deja el
# paso cerrado sin exigir que el pilar quede exactamente en la cola.
#
# Esta es la magnitud clave del diseño: girar sobre el sitio NO la mueve
# (ver tracker._refrescar_progreso), asi que ningun giro de chasis puede
# fingir un rebase.
PROGRESO_PASO = 100.0

# Ciclos consecutivos cumpliendo el invariante antes de declararlo. Evita
# que un ciclo con una asociacion mala cierre la maniobra antes de tiempo.
CICLOS_PASO = 3

DIST_CONFIRMACION = 900.0    # mm, a partir de aqui el pilar interesa
CICLOS_CONFIRMACION = 3      # barridos coherentes antes de comprometerse

# Adelanto con el que se evaluan las condiciones de transicion. El
# limitador del servo mueve el comando 12 grados por ciclo, asi que
# invertir el volante de +25 a -20 cuesta ~4 ciclos (0.45 s a 8.5 Hz).
# Disparar el contragiro cuando la condicion ya se cumple llega tarde por
# esos 4 ciclos; se dispara cuando se VA a cumplir.
T_ADELANTO_SERVO = 0.45

# Una pared de frente a menos de un ancho de carril es una esquina: el
# pasillo mide ANCHO_CARRIL (1000mm) y en recta no hay nada de frente a
# esa distancia. Marca la maniobra como "en esquina" y con eso la salida
# sabe que la tangente gira 90 grados.
DIST_ESQUINA = geo.ANCHO_CARRIL

# Sin progreso geometrico durante este tiempo la maniobra se aborta. NO
# es un timeout de fase: es la ausencia medida de avance hacia el
# objetivo (ni se acerca el travez ni crece la separacion), que es lo que
# distingue "me estoy apartando para ganar radio" de "he perdido el hilo".
SIN_PROGRESO_MAX = 1.5

# Distancia a la que la excursion deja de ser ganar radio y es abandono.
# Un pilar a mas de esto ya no se puede envolver dentro del carril.
DIST_EXCURSION_MAX = 950.0

# Ciclos seguidos con la intencion suspendida por las paredes antes de
# abortar. ~1.4 s a 8.5 Hz.
CICLOS_SUSPENSION_MAX = 12

# Redes de seguridad. Ninguna es la ruta normal de salida de su fase.
TIMEOUT_CONFIRMACION = 2.5
TIMEOUT_FASE         = 6.0
TIMEOUT_MANIOBRA     = 14.0
TIMEOUT_SALIDA       = 3.0
TIMEOUT_RECUPERACION = 2.5

# Tras un aborto no se vuelve a capturar durante este tiempo, escalado
# por el numero de abortos seguidos: rompe el bucle capturar-abortar sin
# impedir que un pilar real se vuelva a enganchar enseguida.
REFRACTARIO_ABORTO = 1.2
REFRACTARIO_MAX    = 4.0

# Tras un aborto, el pilar se conserva como OBSTACULO (ya no como regla)
# y mientras dure eso no se captura ningun objetivo nuevo. Es lo que
# rompe el bucle capturar-abortar-recapturar sobre el mismo pilar:
# medido en simulacion, sin esto el robot daba tres vueltas completas
# sobre si mismo reenganchando el mismo poste (#4, #5, #6...) porque cada
# captura nueva reiniciaba el progreso geometrico desde cero.
#
# El obstaculo se suelta antes si queda detras o si se aleja demasiado
# para ser creible; el tope temporal es la red.
VIDA_OBSTACULO_ABORTADO = 3.0

ERROR_LATERAL_OK = 120.0     # mm, |izq-der| para dar la recuperacion por buena
ERROR_RUMBO_SALIDA_OK = 20.0 # grados contra la tangente de salida

# Cuanto puede desviar la tangente de salida al control de pista. Es un
# SESGO acotado, no una consigna: el rumbo absoluto es la señal mas debil
# que hay (deriva de la IMU, geometria de la esquina supuesta) y las
# paredes son la mas fuerte. Persiguiendolo a fondo, el robot recien
# salido de un pilar se clava contra el muro exterior de la esquina
# intentando cuadrar un rumbo -- que es la version nueva del error que
# cometia SOBREPASO congelando el rumbo de entrada.
SESGO_SALIDA_MAX = 8.0

# Distancia a la que el pilar rebasado deja de ser una restriccion. Por
# debajo, la cola todavia lo puede barrer al enderezar.
DIST_PILAR_LIBRE = 450.0

# ==========================================
# APAREO COLOR <-> CLUSTER (que pilar es cual)
# ==========================================
# La camara dice QUE color hay delante y el LiDAR DONDE hay postes. Se
# aparean por RUMBO, no por cercania: el pixel cx se convierte a rumbo con
# el modelo estenopeico medido (optica.py) y se elige el cluster que este
# en ese mismo rumbo. Con dos pilares en el frame, "el color del blob mas
# grande" y "el cluster mas cercano" pueden caer en pilares distintos.
ANCHO_FRAME_CAM   = optica.ANCHO_FRAME
CX_CENTRO_OPTICO  = optica.CX_CENTRO_OPTICO
HFOV_CAMARA       = optica.HFOV_EFECTIVO
FOCAL_PX          = optica.FOCAL_PX
TOLERANCIA_APAREO_GRADOS = optica.TOLERANCIA_APAREO_GRADOS
SECTOR_BUSQUEDA_POSTE = 50.0   # grados a cada lado del frente

# Sigma inicial cuando el objetivo se siembra SOLO con la camara y la
# distancia frontal, sin cluster apareado. Es una estimacion mucho peor
# que un centroide de LiDAR y el resto del sistema lo tiene en cuenta
# solo: mas sigma -> mas holgura exigida -> envolvente mas ancha.
SIGMA_SIEMBRA_VISION = 120.0
DIST_SIEMBRA_VISION  = 700.0

# ==========================================
# EMERGENCIA ANTI-CHOQUE
# ==========================================
EMERGENCIA_FRONTAL  = 120.0
EMERGENCIA_LATERAL  = 80.0
EMERGENCIA_TRASERA  = 250.0
TIMEOUT_RETROCESO   = 3.5

# Margenes para DAR POR TERMINADO el retroceso, mas holgados que los de
# entrada: saliendo justo en el umbral, el primer ciclo de marcha vuelve
# a disparar la emergencia y el robot rebota sin salir del sitio.
SALIDA_RETROCESO_FRONTAL = 300.0
SALIDA_RETROCESO_LATERAL = 160.0
TIEMPO_MIN_RETROCESO = 0.6

KP_RETROCESO         = 0.05
MAX_ANGULO_RETROCESO = 25.0

# ==========================================
# DESEMPATE DE ESQUINA SIMETRICA (GIRO_FORZADO)
# ==========================================
RACHA_RETROCESO_PARA_FORZAR = 4
VENTANA_ATASCO = 10.0        # s
# Lectura lateral por encima de la cual esa pared YA NO ES la del carril:
# se acabo. En un anillo de pasillo ANCHO_CARRIL, la unica cosa que abre
# un sector lateral muy por encima de la anchura del pasillo es el final
# del muro interior, y ese es el lado hacia el que dobla la curva.
#
# Sustituye a "gira hacia el lado con mas espacio", que era la regla
# anterior y es FALSA en recta: (izquierda - derecha) mide lo descentrado
# que va el robot, no hacia donde gira la pista. Medido en simulacion,
# saliendo de esquivar un pilar el robot lee izq=576 der=424 -- 152mm de
# diferencia que solo dicen que va pegado al muro interior -- y con la
# regla vieja doblaba hacia FUERA, se metia en la concavidad y acababa
# dando media vuelta y recorriendo la pista al reves.
UMBRAL_APERTURA_ESQUINA = 1.3 * geo.ANCHO_CARRIL
LADO_POR_DEFECTO = 1.0       # +1.0 = izquierda
ANGULO_GIRO_FORZADO    = 25.0
VELOCIDAD_GIRO_FORZADO = VELOCIDAD_EVASION
TIMEOUT_GIRO_FORZADO   = 2.5
SALIDA_GIRO_FORZADO_ASIMETRIA = 150.0

# ==========================================
# CARRERA / PARQUEO
# ==========================================
UMBRAL_VUELTAS       = 1010.0  # grados de yaw neto, ~3 vueltas
TOLERANCIA_FIRMA     = 80.0    # mm contra la firma de pared inicial
TIMEOUT_PARQUEO      = 6.0     # s

# Rate limiter del servo en marcha normal (la emergencia lo salta).
# Subido de 6 a 12 el 2026-08-29: con 6, invertir el volante de -19 a +11
# tardaba 5 ciclos (0.5 s) y a 100 mm/s el robot se comia los 145mm que
# le quedaban. Confirmado en la corrida 153206: de sus 8 emergencias,
# CERO tienen la firma del limitador (antes eran 2 de 5).
#
# Este limitador es la razon de que las transiciones se evaluen con
# T_ADELANTO_SERVO de adelanto.
MAX_DELTA_ANGULO = 12.0

# Topes reales del servo, medidos en la Pico: CENTRO=90 con el comando
# recortado a [70, 115]. Son asimetricos y el tope derecho ademas es
# MECANICO (subirlo a 25 no aumento la velocidad angular). Fuente unica
# en geometria_evasion.py, que los necesita para el modelo de radios.
ANGULO_MAX_IZQ = gev.COMANDO_MAX_IZQ
ANGULO_MAX_DER = gev.COMANDO_MAX_DER

ESTADOS_MANIOBRA = ("COMPROMISO", "APERTURA", "CONTRAGIRO",
                    "PASO_LATERAL", "SALIDA_PILAR")

# Rejilla de comandos, de tope derecho a tope izquierdo.
CANDIDATOS = []
_c = -ANGULO_MAX_DER
while _c <= ANGULO_MAX_IZQ + 1e-6:
    CANDIDATOS.append(round(_c, 3))
    _c += PASO_REJILLA
if abs(CANDIDATOS[-1] - ANGULO_MAX_IZQ) > 1e-6:
    CANDIDATOS.append(ANGULO_MAX_IZQ)


def _clamp(v, lim):
    return max(-lim, min(lim, v))


def _clamp_servo(v):
    # Recorte al recorrido fisico real (positivo = izquierda)
    return max(-ANGULO_MAX_DER, min(ANGULO_MAX_IZQ, v))


def _vel_mm_s(pwm):
    return tracker_mod.MM_POR_SEG_A_PWM100 * abs(pwm) / 100.0


def lado_obligatorio(color):
    """Lado del ROBOT en el que tiene que quedar el pilar.

    Regla WRO FE: el pilar rojo se pasa por su DERECHA y el verde por su
    IZQUIERDA. Pasar por la derecha de un pilar significa que el pilar
    queda a la IZQUIERDA del robot.

        ROJO  -> s_lado = -1  (pilar a la izquierda del robot)
        VERDE -> s_lado = +1  (pilar a la derecha del robot)

    Esta funcion es el UNICO sitio del sistema donde el color se traduce
    a geometria. En ningun otro lado se deriva un sentido de giro de un
    color: el sentido de giro sale de donde esta el pilar, y durante una
    envolvente cambia de signo una vez. Ver "Verificacion de signos" en
    DISENO_CURVAS.md antes de dar por buena esta tabla.
    """
    return -1 if color == "ROJO" else 1


class Navegador:
    def __init__(self, control_sector):
        # control_sector: objeto con fijar_sector_frontal() y
        # sector_frontal_normal(), normalmente el ProcesadorLidar.
        #
        # Ya NO se ensancha el sector frontal durante la evasion. Lo hacia
        # la version anterior "para no perder el poste", pero (a) el pilar
        # se sigue con el cluster y el tracker, no con el sector frontal, y
        # (b) los sectores que usaba estaban del lado CONTRARIO al pilar:
        # SECTOR_EVASION_IZQ abria 30 grados a la izquierda cuando
        # "evadir por la izquierda" significa pasar por la izquierda DEL
        # PILAR, o sea con el pilar a la derecha. Y como `frontal` alimenta
        # la emergencia y el frenado, ensancharlo tambien hacia frenar por
        # el propio pilar que se estaba rodeando.
        self._sector = control_sector
        self._sector.sector_frontal_normal()
        self.tracker = tracker_mod.TrackerObstaculo()
        self.sentido = sentido_vuelta.SentidoVuelta()

        self.fase   = "CAPTURA_FIRMA"
        self.estado = "CRUCERO"

        self._firma_izq = 0.0
        self._firma_der = 0.0

        self._t_estado  = 0.0
        self._t_parqueo = 0.0

        self._ultimo_angulo = 0.0
        self._ultima_vel    = 0
        self._t_ultimo_ciclo = None
        self._cx_cam = None

        # --- memoria de la maniobra, congelada en COMPROMISO ---
        self._heading_commit = 0.0
        self._t_commit       = None
        self._en_esquina     = False
        self._rumbo_salida   = None
        self._ciclos_paso    = 0
        self._paso_validado  = False
        self._ciclos_suspension = 0
        self._ciclos_no_factible = 0
        self._ciclos_confirmacion = 0
        self._t_ultimo_progreso = 0.0
        self._mejor_y_r = None
        self._mejor_u   = None
        self._mejor_progreso = -1e9
        self._estado_suspendido = None
        self._abortos = 0
        self._t_refractario = 0.0
        self._solo_obstaculo = False
        self._t_obstaculo = None

        # Desempate de esquina simetrica
        self._racha_retroceso = 0
        self._t_ultima_emergencia = None
        self._signo_memoria_asimetria = None
        self._signo_giro_forzado = 0.0
        self._esquina_latch = False
        self._signo_esquina = LADO_POR_DEFECTO

        # --- diagnostico para el CSV ---
        self.rama_evasion   = ""
        self.rumbo_poste_cam = 0.0
        self.objetivo_id    = 0
        self.holgura_pilar  = 0.0
        self.radio_maniobra = 0.0
        self.separacion_lat = 0.0
        self.seguridad      = "LIBRE"
        self.cmd_deseado    = 0.0
        self.n_seguros      = 0
        self.n_compatibles  = 0
        self.solo_envolvente = False

    # ==========================================
    # ENTRADA
    # ==========================================
    def procesar(self, med, color_cam, heading, ahora=None, cx_cam=None):
        # Una llamada por barrido completo. Devuelve (velocidad, angulo)
        # o None cuando la carrera termino.
        if ahora is None:
            ahora = time.time()

        dt = 0.0 if self._t_ultimo_ciclo is None else min(0.3, ahora - self._t_ultimo_ciclo)
        self._t_ultimo_ciclo = ahora

        if self.fase == "CAPTURA_FIRMA":
            self._firma_izq, self._firma_der = med.izquierda, med.derecha
            self.fase = "CARRERA"
            self._t_estado = ahora
            self._t_ultimo_progreso = ahora
            print("[+] Firma de parqueo: Izq=%.0f Der=%.0fmm"
                  % (self._firma_izq, self._firma_der))
            print("[INICIO] Carrera con obstaculos iniciada!")
            return (0, 0.0)

        if self.fase == "CARRERA":
            return self._ciclo_carrera(med, color_cam, heading, ahora, dt, cx_cam)

        if self.fase == "PARQUEO":
            return self._ciclo_parqueo(med, ahora)

        return None    # FIN

    # ==========================================
    # FASE CARRERA
    # ==========================================
    def _ciclo_carrera(self, med, color_cam, heading, ahora, dt, cx_cam=None):
        self._cx_cam = cx_cam
        en_maniobra = self.estado in ESTADOS_MANIOBRA

        # 0. Percepcion del objetivo: prediccion, asociacion y, si no hay
        #    objetivo vivo ni maniobra en curso, captura.
        avance_mm = (self._ultima_vel / 100.0) * tracker_mod.MM_POR_SEG_A_PWM100 * dt
        self.tracker.predecir(heading, avance_mm, ahora)
        if self.tracker.activo:
            self.tracker.asociar(self._candidatos_pilar(med))
            self._readquirir_con_camara(color_cam, cx_cam)
            if self.tracker.perdido(ahora):
                # No se suelta aqui: el estado decide que hacer con la
                # perdida. Soltar en el bloque de percepcion es
                # exactamente como se perdia la intencion antes.
                pass
        elif not en_maniobra and ahora >= self._t_refractario:
            self._intentar_capturar(med, color_cam, heading, cx_cam, ahora)

        # 0b. Sentido de la vuelta. Se congela durante la maniobra: el yaw
        #     de una envolvente es de la maniobra, no de la pista, y
        #     meterlo aqui es la forma tipica de fijar un sentido falso.
        self.sentido.actualizar(heading, ahora, congelado=en_maniobra)

        # 0c. Memoria de la ultima asimetria REAL entre paredes. Vive
        #     fuera de cualquier estado: es la pista con la que se
        #     desempata una esquina que, cuando por fin dispara la
        #     emergencia, ya se ve perfectamente simetrica.
        # 0d. Deteccion del final de muro, enclavamiento del lado de la
        #     esquina y memoria del ultimo lado visto
        self._actualizar_esquina(med)

        # 1. Emergencia anti-choque, prioridad sobre cualquier estado.
        #    SUSPENDE la maniobra, no la borra: el objetivo sigue vivo y
        #    el estado al que hay que volver queda guardado.
        if (med.frontal < EMERGENCIA_FRONTAL
                or med.izquierda < EMERGENCIA_LATERAL
                or med.derecha < EMERGENCIA_LATERAL):
            if self.estado not in ("RETROCESO", "GIRO_FORZADO"):
                if (self._t_ultima_emergencia is not None and
                        (ahora - self._t_ultima_emergencia) <= VENTANA_ATASCO):
                    self._racha_retroceso += 1
                else:
                    self._racha_retroceso = 1
                self._t_ultima_emergencia = ahora
                if self.estado in ESTADOS_MANIOBRA:
                    self._estado_suspendido = self.estado
                self._entrar("RETROCESO", ahora)
                print("[EMERGENCIA] F:%.0f I:%.0f D:%.0fmm -> RETROCESO "
                      "(racha %d, suspende %s)"
                      % (med.frontal, med.izquierda, med.derecha,
                         self._racha_retroceso, self._estado_suspendido or "nada"))

        # 2. Vueltas completas -> parqueo. Solo desde CRUCERO para no
        #    abandonar una maniobra a medias con un pilar al lado.
        if abs(heading) >= UMBRAL_VUELTAS and self.estado == "CRUCERO":
            self.fase = "PARQUEO"
            self._t_parqueo = ahora
            print("[!] %.0f grados acumulados. Modo Parqueo." % heading)
            return (VELOCIDAD_PARQUEO, self._nominal_arbitrado(med, VELOCIDAD_PARQUEO))

        # 3. Despacho
        manejador = {
            "CRUCERO":      self._est_crucero,
            "CONFIRMACION": self._est_confirmacion,
            "COMPROMISO":   self._est_compromiso,
            "APERTURA":     self._est_apertura,
            "CONTRAGIRO":   self._est_contragiro,
            "PASO_LATERAL": self._est_paso_lateral,
            "SALIDA_PILAR": self._est_salida_pilar,
            "RECUPERACION": self._est_recuperacion,
            "ABORTO":       self._est_aborto,
            "RETROCESO":    self._est_retroceso,
            "GIRO_FORZADO": self._est_giro_forzado,
        }[self.estado]
        velocidad, angulo = manejador(med, color_cam, heading, ahora)

        # 4. Rate limiter del servo. En emergencia no se aplica: ahi el
        #    giro completo tiene que entrar de una.
        if self.estado == "RETROCESO":
            self._ultimo_angulo = _clamp_servo(angulo)
            angulo = self._ultimo_angulo
        else:
            delta = _clamp(angulo - self._ultimo_angulo, MAX_DELTA_ANGULO)
            angulo = _clamp_servo(self._ultimo_angulo + delta)
            self._ultimo_angulo = angulo

        self._ultima_vel = velocidad
        self._publicar_diagnostico()
        return (velocidad, angulo)

    # ==========================================
    # PERCEPCION DEL OBJETIVO
    # ==========================================
    def _candidatos_pilar(self, med):
        # (x, y, ancho_mm) de los clusters que pueden ser un poste. El
        # ancho es la puerta que rechaza esquinas de muro; sin el, la
        # asociacion del tracker se conforma con "esta cerca".
        cands = []
        for clust in med.clusters_obstaculo:
            if not es_objeto_estrecho(clust):
                continue
            cx, cy = centroide_xy_cluster(clust)
            cands.append((cx, cy, ancho_cluster(clust)))
        return cands

    def _readquirir_con_camara(self, color_cam, cx_cam):
        # La camara solo da RUMBO, y solo se usa si el color coincide con
        # el CONGELADO. Un color distinto no reasigna nada: como mucho
        # deja la estimacion como esta. Ademas solo corrige cuando la
        # estimacion ya esta degradada -- con el tracker sano, el LiDAR
        # es mejor que la camara en las dos coordenadas.
        trk = self.tracker
        if not trk.activo or cx_cam is None or color_cam is None:
            return
        if color_cam != trk.color or not trk.degradado():
            return
        rumbo_cam = optica.rumbo_de_cx(cx_cam)
        pred = optica.rumbo_camara_de_cluster(trk.x, trk.y)
        delta = gev.normalizar_180(rumbo_cam - pred)
        if abs(delta) <= TOLERANCIA_APAREO_GRADOS:
            trk.corregir_rumbo(delta)
            self.rama_evasion = "READQUISICION_CAM"

    def _intentar_capturar(self, med, color_cam, heading, cx_cam, ahora):
        # Crea el objetivo cuando camara y LiDAR coinciden en un pilar
        # frontal. El apareo va por RUMBO: asi el color se pega al pilar
        # que la camara esta viendo, no al que casualmente esta mas cerca.
        if color_cam is None:
            return
        s_lado = lado_obligatorio(color_cam)
        rumbo_cam = optica.rumbo_de_cx(cx_cam) if cx_cam is not None else None

        candidatos = []
        for clust in med.clusters_obstaculo:
            if not es_objeto_estrecho(clust):
                continue
            cx, cy = centroide_xy_cluster(clust)
            r = math.degrees(math.atan2(cx, cy))
            if cy > 80.0 and abs(r) <= SECTOR_BUSQUEDA_POSTE:
                candidatos.append((cx, cy, math.hypot(cx, cy), r,
                                   optica.rumbo_camara_de_cluster(cx, cy)))

        if candidatos and rumbo_cam is not None:
            en_rumbo = [c for c in candidatos
                        if abs(c[4] - rumbo_cam) <= TOLERANCIA_APAREO_GRADOS]
            if en_rumbo:
                el = min(en_rumbo, key=lambda c: abs(c[4] - rumbo_cam))
                self.tracker.iniciar(color_cam, s_lado, el[0], el[1], heading, ahora)
                return
        elif candidatos:
            el = min(candidatos, key=lambda c: c[2])
            self.tracker.iniciar(color_cam, s_lado, el[0], el[1], heading, ahora)
            return

        # Siembra solo por vision: hay color y hay un OBJETO ESTRECHO de
        # frente, pero el LiDAR no da un cluster apareable (ocluido,
        # fuera del sector, o el pilar todavia no rompe el muro de
        # fondo). Se siembra la posicion con el rumbo de la camara y la
        # distancia frontal, con sigma alta. No es un atajo: el resto del
        # sistema la trata como lo que es, y una sigma alta exige mas
        # holgura y una envolvente mas ancha.
        estrecho_delante = med.frontal_muro > med.frontal * 1.5
        if (rumbo_cam is not None and estrecho_delante
                and 50.0 < med.frontal < DIST_SIEMBRA_VISION):
            d = med.frontal
            x = d * math.sin(math.radians(rumbo_cam))
            y = d * math.cos(math.radians(rumbo_cam))
            self.tracker.iniciar(color_cam, s_lado, x, y, heading, ahora,
                                 sigma=SIGMA_SIEMBRA_VISION, sembrado=True)
            print("[OBJETIVO] sembrado solo con vision (sin cluster apareado)")

    # ==========================================
    # ARBITRAJE: TRAYECTORIA vs REGLA vs SEGURIDAD
    # ==========================================
    def _comandos_seguros(self, med, velocidad, margen=MARGEN_PARED):
        """Conjunto de comandos que las PAREDES permiten ejecutar.

        No devuelve una correccion sino un conjunto. Esa es la diferencia
        con `_con_seguridad_pared()`, que mezclaba el comando deseado con
        el centrado y podia invertirle el signo sin dejar rastro.
        """
        horizonte = max(HORIZONTE_MIN, _vel_mm_s(velocidad) * HORIZONTE_SEG)
        h_frente = max(HORIZONTE_FRENTE_MIN,
                       _vel_mm_s(velocidad) * HORIZONTE_FRENTE_SEG)
        exige_giro = med.frontal_muro < DIST_FRENTE_EXIGE_GIRO
        # El sector frontal se mide desde el LiDAR; el arco se calcula
        # desde el eje trasero, LIDAR_X mm mas atras.
        alcance = med.frontal_muro + geo.LIDAR_X - MARGEN_FRENTE
        deriva = _clamp(med.angulo_muro, MAX_DERIVA_MURO)

        seguros = []
        for cmd in CANDIDATOS:
            ok = True
            for frac in (0.5, 1.0):
                dx = gev.lateral_predicho(cmd, horizonte * frac, deriva)
                if med.derecha - dx < margen:
                    ok = False
                    break
                if med.izquierda + dx < margen:
                    ok = False
                    break
            if ok and exige_giro:
                # HORIZONTE FINITO Y PROPIO, no circunferencia completa
                # y no el horizonte lateral. Ver gev.alcance_frontal:
                # exigir que quepa el arco entero dejaba el conjunto
                # admisible VACIO en el 30% de los ciclos de la corrida
                # 195623; reutilizar el horizonte lateral de 180mm lo
                # vaciaba por debajo de 242mm de pared frontal, que es
                # justo donde entraron las 12 rachas de SIN_SALIDA de la
                # corrida 132913. Quien hace empezar la curva a tiempo es
                # el termino de esquina de _rumbo_nominal, que si usa el
                # criterio completo.
                if gev.alcance_frontal(gev.radio_de_comando(cmd),
                                       h_frente) > alcance:
                    ok = False
            if ok:
                seguros.append(cmd)
        return seguros, horizonte

    def _decidir_lado_esquina(self, med):
        # Hacia donde dobla esta esquina, en orden de fiabilidad:
        #   1. el sentido de la vuelta, si ya se conoce (una vuelta no
        #      cambia de sentido a mitad de carrera);
        #   2. el muro que SE ACABA en este barrido: un sector lateral
        #      muy por encima de la anchura del carril es el final del
        #      muro interior, y por ahi dobla la pista;
        #   3. el ultimo final de muro memorizado.
        # Devuelve None cuando no hay ninguna evidencia: preferimos no
        # tener lado a inventarnos uno.
        if self.sentido.conocido:
            return float(self.sentido.sentido)
        abre_izq = med.izquierda > UMBRAL_APERTURA_ESQUINA
        abre_der = med.derecha > UMBRAL_APERTURA_ESQUINA
        if abre_izq != abre_der:
            return 1.0 if abre_izq else -1.0
        return self._signo_memoria_asimetria

    def _actualizar_esquina(self, med):
        """Enclava el lado de la curva al DETECTARLA, y lo mantiene.

        (izquierda - derecha) es una medida de POSICION, no de direccion
        de giro. Sirve para decidir el lado mientras el robot esta
        centrado y todavia en la recta -- ahi el lado que se abre es el
        interior de la curva -- y deja de servir en cuanto el robot entra
        a la esquina descentrado, que es justo como sale de una evasion.
        Medido en simulacion: saliendo de esquivar un pilar por el lado
        interior, el robot queda 200mm hacia dentro, lee izq>der y dobla
        hacia FUERA, se mete en la concavidad y se atasca.
        """
        # La memoria del lado solo guarda finales de muro, no
        # descentramientos: ese era el ruido que se colaba antes.
        abre_izq = med.izquierda > UMBRAL_APERTURA_ESQUINA
        abre_der = med.derecha > UMBRAL_APERTURA_ESQUINA
        if abre_izq != abre_der:
            self._signo_memoria_asimetria = 1.0 if abre_izq else -1.0

        if med.frontal_muro < DIST_ESQUINA:
            if not self._esquina_latch:
                lado = self._decidir_lado_esquina(med)
                if lado is not None:
                    self._esquina_latch = True
                    self._signo_esquina = lado
        elif med.frontal_muro > DIST_ESQUINA * 1.4:
            self._esquina_latch = False

    def _preferencia_esquina(self, med):
        # None = todavia no hay evidencia de hacia donde dobla.
        if self.sentido.conocido:
            return float(self.sentido.sentido)
        if self._esquina_latch:
            return self._signo_esquina
        return self._decidir_lado_esquina(med)

    def _compatibles(self, seguros, pilar, holgura, criterio):
        # `criterio` separa las dos preguntas que antes iban juntas:
        #   "lado"   -> ¿el pilar queda de su lado reglamentario?
        #   "choque" -> solo ¿lo toco? (durante APERTURA la regla todavia
        #               no se puede cumplir, pero chocar sigue prohibido)
        x_r, y_r = pilar
        if criterio == "lado":
            return [c for c in seguros
                    if gev.preserva_lado(x_r, y_r, self.tracker.s_lado, c, holgura)]
        return [c for c in seguros if gev.holgura_arco(x_r, y_r, c) >= holgura]

    def _arbitrar(self, deseado, med, velocidad, pilar=None, holgura=0.0,
                  preferir_camara=False, criterio="lado"):
        """Elige el comando ejecutable mas parecido al deseado.

        Prioridad: paredes (dura) > pilar (regla) > trayectoria deseada.
        Si el conjunto compatible con el pilar queda vacio, no se
        abandona la intencion en silencio: se marca SUSPENDE y el estado
        decide (esperar, o abortar si se prolonga).

        Devuelve (comando, hay_salida).
        """
        seguros, _ = self._comandos_seguros(med, velocidad)
        self.n_seguros = len(seguros)
        if not seguros:
            self.seguridad = "SIN_SALIDA"
            return (0.0, False)

        candidatos = seguros
        self.seguridad = "LIBRE"

        if pilar is not None:
            x_r, y_r = pilar
            compat = self._compatibles(seguros, pilar, holgura, criterio)
            if not compat:
                # Con el margen comodo no cabe. Antes de declarar el
                # conflicto se APURA hasta el suelo duro: hay huecos
                # legales (un pilar contra el muro exterior de una
                # esquina) donde exigir 110mm convierte una maniobra
                # posible en un aborto, y abortar deja al robot pasando
                # por el lado prohibido, que es peor que ir apretado.
                apurados, _ = self._comandos_seguros(med, velocidad,
                                                     margen=MARGEN_PARED_DURO)
                compat = self._compatibles(apurados, pilar, holgura, criterio)
                if compat:
                    self.seguridad = "APURA"
                    seguros = apurados
            self.n_compatibles = len(compat)
            self.solo_envolvente = bool(compat) and all(
                c * self.tracker.s_lado < 0 for c in compat)
            if compat:
                candidatos = compat
            else:
                # Las paredes y el pilar piden cosas incompatibles ni
                # apretando. Gana la pared, pero se elige el comando que
                # MENOS invade al pilar, y el estado se entera.
                self.seguridad = "SUSPENDE"
                self.n_compatibles = 0
                self.solo_envolvente = False
                mejor = max(seguros, key=lambda c: gev.holgura_arco(x_r, y_r, c))
                self.holgura_pilar = gev.holgura_arco(x_r, y_r, mejor)
                return (mejor, True)

        if preferir_camara and pilar is not None and len(candidatos) > 1:
            # Readquisicion: con la estimacion degradada interesa volver a
            # VER el pilar, no solo predecirlo. Entre los comandos
            # admisibles se prefiere el que lo deja mas centrado en la
            # camara dentro de un ciclo de servo, con la distancia al
            # comando deseado como desempate ponderado -- la maniobra no
            # se sacrifica por mirar, solo se sesga.
            elegido = min(candidatos,
                          key=lambda c: (abs(self._rumbo_camara_previsto(
                              pilar, c, velocidad))
                              + 0.5 * abs(c - deseado)))
        else:
            elegido = min(candidatos, key=lambda c: abs(c - deseado))

        if abs(elegido - deseado) > 0.6 and self.seguridad == "LIBRE":
            self.seguridad = "RECORTA"
            if deseado * elegido < 0 and abs(deseado) > 5.0:
                # El recorte invirtio el signo del comando. Es legitimo
                # (la pared manda), pero es exactamente el suceso que
                # antes ocurria en silencio y hacia perder la maniobra.
                self.seguridad = "INVIERTE"
        return (elegido, True)

    def _rumbo_camara_previsto(self, pilar, cmd, velocidad):
        # Rumbo con el que la CAMARA veria el pilar tras un ciclo de servo
        # ejecutando `cmd`. Se usa solo para sesgar la readquisicion.
        x_r, y_r = pilar
        avance = _vel_mm_s(velocidad) * T_ADELANTO_SERVO
        radio = gev.radio_de_comando(cmd)
        if radio == float("inf"):
            giro = 0.0
        else:
            giro = math.degrees(avance / radio) * (1.0 if cmd > 0 else -1.0)
        xf, yf = gev.predecir_pilar(x_r, y_r, avance, giro)
        xl, yl = geo.eje_trasero_a_lidar(xf, yf)
        return optica.rumbo_camara_de_cluster(xl, yl)

    def _rumbo_nominal(self, med):
        # Trayectoria nominal: centrado entre paredes por POSICION mas la
        # asistencia de esquina por angulo_muro. Es la responsabilidad
        # numero 1 y sale de aqui SIN mezclarse con nada mas: el recorte
        # por paredes y por pilar lo hace _arbitrar().
        #
        # El recorte de este termino no es cosmetico: en las esquinas el
        # carril se abre hasta 1400mm y con KP_LATERAL eso pide del orden
        # de 200 grados de servo, que con el rate limiter se convierte en
        # windup puro (medido: comando de -78 grados mientras el error ya
        # bajaba).
        # Centrado por POSICION, pero solo cuando las dos lecturas son de
        # verdad las dos paredes del carril. En una esquina un sector se
        # va a 1300mm mirando por encima de la isla, y (izq - der) pasa a
        # valer 900mm: con KP_LATERAL eso pide 130 grados de servo, el
        # comando satura al tope y el robot da vueltas sobre si mismo
        # persiguiendo un "centro" que no existe. Medido en simulacion:
        # 60 ciclos girando en el sitio con izq=1299 y der=356.
        #
        # El carril mide ANCHO_CARRIL. Si un lado lee mucho mas que eso,
        # ese lado no es la pared del carril y centrarse entre los dos no
        # significa nada: se sigue la pared que SI es valida, a media
        # anchura de ella, que es un objetivo acotado y con sentido.
        if max(med.izquierda, med.derecha) > 1.5 * geo.ANCHO_CARRIL:
            objetivo = geo.ANCHO_CARRIL / 2.0
            # Mismo signo que la formula de dos paredes: la pared que
            # falta se sustituye por la distancia nominal.
            if med.izquierda < med.derecha:
                ang = (med.izquierda - objetivo) * KP_LATERAL
            else:
                ang = (objetivo - med.derecha) * KP_LATERAL
        else:
            ang = (med.izquierda - med.derecha) * KP_LATERAL
        ang = _clamp(ang, MAX_APORTE_POSICION)
        if med.frontal_muro < DIST_ASISTENCIA_ESQUINA:
            # MISMA CREDIBILIDAD QUE LA ENVOLVENTE. `angulo_muro` tiene
            # dos consumidores y hasta la corrida 133838 creian cosas
            # distintas del mismo numero: la envolvente lo acotaba a
            # MAX_DERIVA_MURO (12 grados, porque fuera de una esquina
            # real es ruido) y esta asistencia lo usaba CRUDO con
            # ganancia 0.65.
            #
            # La factura, medida en 133838: en las cuatro aproximaciones
            # que acaban en RETROCESO la lectura era -37/-38 grados y
            # oscilaba de signo cada pocos ciclos (-37 <-> +24). Con el
            # numero crudo la asistencia saturaba en +15 y el rumbo
            # deseado salia +17.5, mientras el comando EJECUTADO era 0-10
            # porque la envolvente solo se creia -12 de esos -37. Entre 5
            # y 15 grados de cada ciclo eran demanda que la envolvente no
            # podia conceder nunca: RECORTA 21.5% + SUSPENDE 6.3% +
            # INVIERTE 1.2% de los ciclos, y un ciclo limite
            # PASO_LATERAL -> RETROCESO -> PASO_LATERAL repetido 4 veces
            # en 100 ciclos mientras la pared pasaba de 388 a 219mm.
            #
            # Acotar aqui tambien no baja la ganancia "a ver si mejora":
            # hace que las dos mitades del sistema opinen sobre el mismo
            # numero. El tope efectivo pasa a ser 12*0.65 = 7.8 grados,
            # asi que MAX_APORTE_ANGULO_MURO deja de morder y quien manda
            # es el limite de credibilidad, que es lo que se midio.
            fiable = _clamp(med.angulo_muro, MAX_DERIVA_MURO)
            ang += _clamp(-fiable * KP_ANGULO_MURO, MAX_APORTE_ANGULO_MURO)
        ang = _clamp_servo(ang)

        # Termino de esquina. Los dos terminos de arriba miden cosas
        # distintas -- posicion entre paredes y orientacion respecto al
        # muro -- y acercandose a una esquina apuntan a lados OPUESTOS y
        # se anulan: medido en el repo, 274 de 394 ciclos con el frente
        # por debajo de 400mm dejaban un comando mediano de 1.5 grados
        # con un servo que da 20-25.
        #
        # Esto no es una rampa de urgencia mezclada encima (eso era el
        # escape frontal anterior, y podia invertirle el signo a una
        # evasion sin dejar rastro): es el arco MAS ABIERTO que todavia
        # dobla antes de la pared. Se enciende solo cuando ese arco baja
        # de RADIO_ENVOLVENTE_MAX -- o sea cuando de verdad hay una
        # esquina -- y se cierra solo segun la pared se acerca. Y como
        # solo puede pedir MAS giro hacia el lado de la curva, nunca
        # cancela el centrado, solo lo supera cuando hace falta.
        alcance = (med.frontal_muro + geo.LIDAR_X - MARGEN_FRENTE
                   - MARGEN_ESQUINA)
        radio = gev.radio_maximo_para_frente(alcance)
        lado = self._preferencia_esquina(med)
        if radio is not None and radio < gev.RADIO_ENVOLVENTE_MAX and lado is not None:
            hacia_izq = lado > 0
            cmd_esquina = gev.comando_de_radio(radio, hacia_izquierda=hacia_izq)
            ang = max(ang, cmd_esquina) if hacia_izq else min(ang, cmd_esquina)
        return ang

    def _nominal_arbitrado(self, med, velocidad):
        # La trayectoria nominal tambien respeta la holgura de un pilar
        # abortado: dejar de perseguir la regla no es dejar de verlo.
        deseado = self._rumbo_nominal(med)
        self.cmd_deseado = deseado
        cmd, hay = self._arbitrar(deseado, med, velocidad,
                                  pilar=self._obstaculo_pasivo(),
                                  holgura=HOLGURA_BASE)
        if not hay:
            return self._sin_salida(med, self._t_ultimo_ciclo or 0.0)[1]
        return cmd

    def _con_frenado(self, velocidad_base, frontal):
        if frontal >= DIST_FRENADO_INICIO:
            return velocidad_base
        if frontal <= DIST_FRENADO_MIN:
            return VELOCIDAD_MINIMA
        p = (frontal - DIST_FRENADO_MIN) / (DIST_FRENADO_INICIO - DIST_FRENADO_MIN)
        return int(VELOCIDAD_MINIMA + p * (velocidad_base - VELOCIDAD_MINIMA))

    # ==========================================
    # GEOMETRIA DE LA MANIOBRA
    # ==========================================
    def _holgura_exigida(self):
        # La holgura crece con la incertidumbre: una estimacion peor
        # obliga a rodear mas ancho. Sin esto, una estimacion mala
        # produce exactamente la misma trayectoria que una buena.
        return HOLGURA_BASE + HOLGURA_POR_SIGMA * min(self.tracker.sigma, 200.0)

    def _envolvente_factible(self, adelantado=None):
        x_r, y_r = adelantado if adelantado else self.tracker.xy_eje()
        return gev.radio_envolvente(x_r, y_r, self.tracker.s_lado,
                                    self._holgura_exigida())

    def _hay_progreso(self, ahora):
        """Distingue 'me aparto para ganar radio' de 'perdi el objetivo'.

        Hay progreso si el pilar se acerca al travez (y_r baja) O si la
        separacion por el lado bueno crece. Apartarse es progreso mientras
        una de las dos mejore; si ninguna mejora durante SIN_PROGRESO_MAX,
        la excursion dejo de servir a la maniobra.
        """
        x_r, y_r = self.tracker.xy_eje()
        u = self.tracker.s_lado * x_r
        mejora = False
        # Rodear tambien es progresar. Ya al costado del pilar, y_r deja
        # de bajar y u deja de crecer, pero el barrido sigue subiendo:
        # sin este termino la propia fase de paso se declaraba estancada.
        if self.tracker.progreso > self._mejor_progreso + 2.0:
            self._mejor_progreso = self.tracker.progreso
            mejora = True
        if self._mejor_y_r is None or y_r < self._mejor_y_r - 5.0:
            self._mejor_y_r = y_r if self._mejor_y_r is None else min(self._mejor_y_r, y_r)
            mejora = True
        if self._mejor_u is None or u > self._mejor_u + 5.0:
            self._mejor_u = u if self._mejor_u is None else max(self._mejor_u, u)
            mejora = True
        if mejora:
            self._t_ultimo_progreso = ahora
        return (ahora - self._t_ultimo_progreso) <= SIN_PROGRESO_MAX

    def _invariante_de_paso(self, ahora):
        """Condicion geometrica para declarar el pilar superado.

        Tres cosas a la vez, y ninguna sirve sola:

        1. PROGRESO >= PROGRESO_PASO. Barrido del rumbo del pilar en
           marco MUNDO, con el signo del lado obligatorio. Inmune a la
           rotacion del chasis, que es lo que falsificaba el criterio
           anterior (`y < -280mm`).
        2. SEPARACION demostrable. La aproximacion maxima registrada,
           descontada la incertidumbre que tenia la estimacion en ese
           instante, por encima del minimo geometrico. Si la estimacion
           era mala, se exige mas separacion real.
        3. DISTANCIA actual suficiente: el pilar ya no esta encima.

        Mas histeresis: CICLOS_PASO barridos seguidos. Una vez declarado
        queda enclavado -- el paso no se "des-valida".
        """
        trk = self.tracker
        if self._paso_validado:
            return True
        ok = (trk.progreso >= PROGRESO_PASO
              and trk.separacion_minima_garantizada() >= SEPARACION_MIN_VALIDA
              and trk.distancia() >= SEPARACION_MIN_VALIDA + 60.0)
        if ok:
            self._ciclos_paso += 1
        else:
            self._ciclos_paso = 0
        if self._ciclos_paso >= CICLOS_PASO:
            self._paso_validado = True
            print("[PASO] pilar #%d superado: progreso %.0f grados, "
                  "separacion garantizada %.0fmm"
                  % (trk.id, trk.progreso, trk.separacion_minima_garantizada()))
        return self._paso_validado

    # ==========================================
    # ESTADOS
    # ==========================================
    def _est_crucero(self, med, color_cam, heading, ahora):
        trk = self.tracker
        if trk.activo and not self._solo_obstaculo:
            x_r, y_r = trk.xy_eje()
            if trk.perdido(ahora):
                trk.soltar("candidato sin confirmar (sigma %.0fmm)" % trk.sigma)
            elif y_r < 0.0:
                # Se quedo atras sin llegar a comprometerse: no estorba.
                trk.soltar("el candidato quedo detras")
            elif 50.0 < trk.distancia() < DIST_CONFIRMACION:
                self._ciclos_confirmacion = 0
                self._entrar("CONFIRMACION", ahora)
        velocidad = self._con_frenado(VELOCIDAD_CRUCERO, med.frontal)
        return (velocidad, self._nominal_arbitrado(med, velocidad))

    def _est_confirmacion(self, med, color_cam, heading, ahora):
        # Un pilar candidato ya localizado, pero sin comprometerse. Aqui
        # no se congela nada todavia: si resulta ser ruido o una esquina
        # de muro, se vuelve a CRUCERO sin haber tocado la trayectoria.
        trk = self.tracker
        if trk.perdido(ahora) or not trk.activo:
            trk.soltar("candidato descartado")
            self._entrar("CRUCERO", ahora)
            return self._est_crucero(med, color_cam, heading, ahora)

        if color_cam is not None and color_cam != trk.color:
            # Otro color en el cuadro. NO cambia el objetivo: solo
            # significa que hay mas de un pilar a la vista.
            pass

        self._ciclos_confirmacion += 1
        t_en = ahora - self._t_estado
        # Un objetivo sembrado solo con vision no puede exigir dos
        # asociaciones de LiDAR (por definicion no tiene ninguna), pero
        # tampoco se le regala el compromiso: si en TIMEOUT_CONFIRMACION
        # el LiDAR no lo respalda, se suelta.
        respaldado = trk.confirmado() or (trk.sembrado and color_cam == trk.color)
        if self._ciclos_confirmacion >= CICLOS_CONFIRMACION and respaldado:
            self._entrar("COMPROMISO", ahora)
        elif t_en > TIMEOUT_CONFIRMACION:
            trk.soltar("no se confirmo a tiempo")
            self._entrar("CRUCERO", ahora)

        velocidad = self._con_frenado(VELOCIDAD_EVASION, med.frontal)
        return (velocidad, self._nominal_arbitrado(med, velocidad))

    def _est_compromiso(self, med, color_cam, heading, ahora):
        # Un solo ciclo. Congela todo lo que la maniobra no puede volver a
        # decidir: identidad, color, lado obligatorio (ya en el tracker),
        # rumbo de entrada, si esto es una esquina y cual es la tangente
        # de salida.
        trk = self.tracker
        self._heading_commit = heading
        self._t_commit = ahora
        self._en_esquina = med.frontal_muro < DIST_ESQUINA
        self._rumbo_salida = self.sentido.rumbo_salida(heading, self._en_esquina)
        self._paso_validado = False
        self._ciclos_paso = 0
        self._ciclos_suspension = 0
        self._ciclos_no_factible = 0
        self._mejor_y_r = None
        self._mejor_u = None
        self._mejor_progreso = -1e9
        self._t_ultimo_progreso = ahora
        trk.marcar_compromiso()

        # Que hace falta desde aqui: nada (el carril ya deja el pilar de
        # su lado), envolver, o abrir primero.
        pilar = trk.xy_eje()
        holgura = self._holgura_exigida()
        seguros, _ = self._comandos_seguros(med, VELOCIDAD_EVASION)
        compat = self._compatibles(seguros, pilar, holgura, "lado")
        if not compat:
            apurados, _ = self._comandos_seguros(med, VELOCIDAD_EVASION,
                                                 margen=MARGEN_PARED_DURO)
            compat = self._compatibles(apurados, pilar, holgura, "lado")

        if not compat:
            destino, plan = "APERTURA", "hace falta abrir"
        elif all(c * trk.s_lado < 0 for c in compat):
            destino, plan = "CONTRAGIRO", "solo cabe envolviendo"
        else:
            destino, plan = "PASO_LATERAL", "el carril ya deja el pilar de su lado"
        self._ciclos_no_factible = 0
        print("[FSM] COMPROMISO pilar #%d %s | lado %s | %s | esquina=%s | "
              "salida=%s -> %s"
              % (trk.id, trk.color,
                 "DERECHA" if trk.s_lado > 0 else "IZQUIERDA", plan,
                 self._en_esquina,
                 ("%.0f" % self._rumbo_salida) if self._rumbo_salida is not None
                 else "por paredes",
                 destino))
        self._entrar(destino, ahora)
        return self._despachar_maniobra(med, color_cam, heading, ahora)

    def _despachar_maniobra(self, med, color_cam, heading, ahora):
        if self.estado == "APERTURA":
            return self._est_apertura(med, color_cam, heading, ahora)
        if self.estado == "PASO_LATERAL":
            return self._est_paso_lateral(med, color_cam, heading, ahora)
        return self._est_contragiro(med, color_cam, heading, ahora)

    def _vigilar_maniobra(self, med, ahora):
        """Comprobaciones comunes a todas las fases con objetivo.

        Devuelve un motivo de aborto o None. Ninguna es un timeout de
        fase: son perdidas geometricas reales. El unico reloj es
        TIMEOUT_MANIOBRA, y esta puesto muy por encima de la duracion
        fisica de la maniobra.
        """
        trk = self.tracker
        if not trk.activo:
            return "objetivo soltado"
        if trk.perdido(ahora):
            return "estimacion perdida (sigma %.0fmm)" % trk.sigma
        if not self._hay_progreso(ahora):
            return "sin progreso geometrico"
        if trk.distancia() > DIST_EXCURSION_MAX:
            return "excursion de %.0fmm: ya no cabe la envolvente" % trk.distancia()
        if self._t_commit is not None and (ahora - self._t_commit) > TIMEOUT_MANIOBRA:
            return "maniobra demasiado larga"
        if self.seguridad == "SUSPENDE":
            self._ciclos_suspension += 1
            if self._ciclos_suspension > CICLOS_SUSPENSION_MAX:
                return "las paredes no dejan cumplir el lado obligatorio"
        else:
            self._ciclos_suspension = 0
        return None

    def _existe_paso_legal(self, med, velocidad, pilar, holgura):
        # ¿Hay ALGUN comando ejecutable que deje el pilar de su lado?
        # Es la condicion de salida de APERTURA, y se evalua sobre el
        # pilar ADELANTADO un ciclo de servo (ver T_ADELANTO_SERVO).
        seguros, _ = self._comandos_seguros(med, velocidad)
        if self._compatibles(seguros, pilar, holgura, "lado"):
            return True
        apurados, _ = self._comandos_seguros(med, velocidad,
                                             margen=MARGEN_PARED_DURO)
        return bool(self._compatibles(apurados, pilar, holgura, "lado"))

    def _est_apertura(self, med, color_cam, heading, ahora):
        """Ganar el espacio que hace falta para poder cumplir la regla.

        Se entra aqui SOLO cuando ningun comando ejecutable deja el pilar
        de su lado: o esta demasiado centrado, o las paredes cierran el
        unico arco que servia. Ir hacia la concavidad de la esquina no es
        un error en esta fase, es el remedio; lo que no puede pasar es
        que la excursion no tenga final. Tiene tres, y ninguno es un
        reloj: aparece un paso legal (salida normal), deja de haber
        progreso geometrico, o la distancia se sale del carril.

        Mientras se abre, el pilar sigue siendo un obstaculo: el criterio
        de arbitraje baja a "no chocar" porque la regla todavia no se
        puede cumplir, pero no desaparece.
        """
        trk = self.tracker
        velocidad = max(VELOCIDAD_MINIMA, self._con_frenado(VELOCIDAD_EVASION, med.frontal))
        x_r, y_r = trk.xy_eje()
        holgura = self._holgura_exigida()

        req = gev.separacion_requerida(y_r, holgura)
        objetivo = (SEPARACION_MIN_VALIDA + MARGEN_APERTURA if req is None
                    else req + MARGEN_APERTURA)
        deseado = gev.comando_apertura(x_r, y_r, trk.s_lado, objetivo)
        self.cmd_deseado = deseado
        self.rama_evasion = "APERTURA"

        cmd, hay = self._arbitrar(deseado, med, velocidad, pilar=(x_r, y_r),
                                  holgura=HOLGURA_BASE, criterio="choque")
        if not hay:
            return self._sin_salida(med, ahora)

        # La salida se evalua sobre la posicion ACTUAL, no sobre la
        # adelantada. El adelanto existe para que el contragiro se
        # comprometa a tiempo pese al limitador del servo, pero aplicado
        # aqui deja salir de la apertura antes de tener la separacion:
        # medido en simulacion, salia con 135mm cuando el minimo
        # geometrico son 183, y la fase siguiente se quedaba sin ningun
        # comando compatible desde el primer ciclo.
        if self._existe_paso_legal(med, velocidad, (x_r, y_r), holgura):
            factible, radio = self._envolvente_factible()
            destino = "CONTRAGIRO" if factible else "PASO_LATERAL"
            self.radio_maniobra = radio or 0.0
            print("[FSM] APERTURA -> %s | separacion %.0fmm, pilar a %.0fmm"
                  % (destino, trk.separacion_lateral(), trk.distancia()))
            self._entrar(destino, ahora)
            self._ciclos_no_factible = 0

        motivo = self._vigilar_maniobra(med, ahora)
        if motivo:
            return self._abortar(med, ahora, motivo)
        return (velocidad, cmd)

    def _est_contragiro(self, med, color_cam, heading, ahora):
        """Invertir el volante hasta meter el pilar dentro del circulo.

        Se llega aqui cuando los UNICOS comandos que dejan el pilar de su
        lado son los que giran HACIA el: la envolvente. Es una fase corta
        y explicita a proposito, porque es donde el limitador del servo
        cuesta 4 ciclos y donde antes se perdia la intencion. Si la
        estimacion esta degradada, entre los admisibles se prefiere el
        que devuelve el pilar al campo de la camara.
        """
        trk = self.tracker
        velocidad = max(VELOCIDAD_MINIMA, self._con_frenado(VELOCIDAD_EVASION, med.frontal))
        x_r, y_r = trk.xy_eje()
        holgura = self._holgura_exigida()

        # Preferido: el arco mas cerrado que envuelve. Si ya no cabe, se
        # pide el rumbo de pista y que el arbitraje recorte.
        deseado = gev.comando_envolvente(x_r, y_r, trk.s_lado, holgura)
        if deseado is None:
            deseado = self._rumbo_nominal(med)
        else:
            self.radio_maniobra = gev.radio_de_comando(deseado)
        self.cmd_deseado = deseado
        self.rama_evasion = "CONTRAGIRO"

        cmd, hay = self._arbitrar(deseado, med, velocidad, pilar=(x_r, y_r),
                                  holgura=holgura,
                                  preferir_camara=trk.degradado())
        if not hay:
            return self._sin_salida(med, ahora)

        if self.n_compatibles == 0:
            # Se quedo sin paso legal: hay que volver a abrir. Con
            # histeresis, un ciclo suelto no cambia de fase.
            self._ciclos_no_factible += 1
            if self._ciclos_no_factible >= 3:
                print("[FSM] CONTRAGIRO -> APERTURA | sin paso legal")
                self._entrar("APERTURA", ahora)
        else:
            self._ciclos_no_factible = 0
            # Salida normal: o el volante ya llego al lado de la
            # envolvente, o han aparecido pasos legales que no exigen
            # envolver (el pilar ya esta rebasandose por su lado).
            if (not self.solo_envolvente) or (cmd * trk.s_lado < 0):
                print("[FSM] CONTRAGIRO -> PASO_LATERAL | volante en %.1f, "
                      "separacion %.0fmm" % (cmd, trk.separacion_lateral()))
                self._entrar("PASO_LATERAL", ahora)

        motivo = self._vigilar_maniobra(med, ahora)
        if motivo:
            return self._abortar(med, ahora, motivo)
        return (velocidad, cmd)

    def _est_paso_lateral(self, med, color_cam, heading, ahora):
        """Pasar de verdad por el lado obligatorio.

        Aqui la consigna deseada vuelve a ser la TRAYECTORIA DE PISTA: el
        pilar actua como restriccion, no como objetivo. Es la diferencia
        practica con perseguir siempre una orbita -- si el pilar ya queda
        de su lado siguiendo el carril, no se hace ninguna excursion; y
        si no, el conjunto compatible se reduce a los arcos que envuelven
        y el comando sale de ahi solo.
        """
        trk = self.tracker
        velocidad = max(VELOCIDAD_MINIMA, self._con_frenado(VELOCIDAD_EVASION, med.frontal))
        x_r, y_r = trk.xy_eje()
        holgura = self._holgura_exigida()

        deseado = self._rumbo_nominal(med)
        self.cmd_deseado = deseado
        self.rama_evasion = "PASO_LATERAL"

        cmd, hay = self._arbitrar(deseado, med, velocidad, pilar=(x_r, y_r),
                                  holgura=holgura)
        if not hay:
            return self._sin_salida(med, ahora)

        if self._invariante_de_paso(ahora):
            self._entrar("SALIDA_PILAR", ahora)
            print("[FSM] PASO_LATERAL -> SALIDA_PILAR")
            return (velocidad, cmd)

        motivo = self._vigilar_maniobra(med, ahora)
        if motivo:
            return self._abortar(med, ahora, motivo)
        return (velocidad, cmd)

    def _est_salida_pilar(self, med, color_cam, heading, ahora):
        """Recuperar la tangente de salida.

        Este es el estado que sustituye a SOBREPASO, y es lo contrario de
        el: SOBREPASO congelaba el rumbo del instante en que el poste
        llegaba al costado -- en una esquina, el rumbo que apunta a la
        concavidad -- y lo sostenia. Aqui la referencia es la tangente de
        SALIDA: el rumbo de entrada girado 90 grados hacia el sentido de
        la vuelta cuando la maniobra ocurrio en una esquina.

        Si el sentido de la vuelta todavia no se conoce, no se inventa
        una referencia: se sigue el pasillo con el control nominal, que
        es lo que hace CRUCERO y no necesita saber el sentido.
        """
        trk = self.tracker
        velocidad = max(VELOCIDAD_MINIMA, self._con_frenado(VELOCIDAD_EVASION, med.frontal))
        t_en = ahora - self._t_estado

        deseado = self._rumbo_nominal(med)
        error = None
        if self._rumbo_salida is not None:
            error = gev.normalizar_180(self._rumbo_salida - heading)
            # La tangente entra como sesgo acotado sobre el control de
            # pista, no como objetivo que perseguir a fondo.
            arco = max(150.0, _vel_mm_s(velocidad) * 1.0)
            if abs(error) > 0.5:
                radio = arco / math.radians(abs(error))
                sesgo = gev.comando_de_radio(radio, hacia_izquierda=(error > 0))
                deseado = _clamp_servo(deseado + _clamp(sesgo, SESGO_SALIDA_MAX))
        # El pilar sigue siendo una restriccion hasta estar lejos: la
        # cola barre hacia fuera al enderezar.
        pilar = None
        if trk.activo and trk.distancia() < DIST_PILAR_LIBRE:
            pilar = trk.xy_eje()
        cmd, hay = self._arbitrar(deseado, med, velocidad, pilar=pilar,
                                  holgura=HOLGURA_BASE)
        if not hay:
            return self._sin_salida(med, ahora)

        # Salida en cuanto el pilar deja de restringir: la tangente
        # termina de recuperarla el control de pista, que si mira las
        # paredes. Quedarse aqui cuadrando un rumbo es lo que metia al
        # robot en bucles de SALIDA_PILAR <-> RETROCESO contra el muro.
        alineado = (error is not None and abs(error) < ERROR_RUMBO_SALIDA_OK)
        lejos = (not trk.activo) or trk.distancia() > DIST_PILAR_LIBRE
        if lejos or alineado or t_en > TIMEOUT_SALIDA:
            razon = ("pilar libre" if lejos else
                     ("tangente recuperada" if alineado else "tiempo maximo"))
            trk.soltar("maniobra completada (%s)" % razon)
            self._abortos = 0
            self._solo_obstaculo = False
            self._entrar("RECUPERACION", ahora)
            print("[FSM] SALIDA_PILAR -> RECUPERACION | %s" % razon)
        return (velocidad, cmd)

    def _est_recuperacion(self, med, color_cam, heading, ahora):
        # Volver al centro del carril por POSICION, no por rumbo. El rumbo
        # no dice donde esta el robot dentro del pasillo: se puede cumplir
        # el objetivo entero y acabar pegado a un muro. El error de
        # centrado (izquierda - derecha) si es una medida de posicion y se
        # anula sola al llegar al medio.
        error_lat = med.izquierda - med.derecha
        t_en = ahora - self._t_estado
        velocidad = self._con_frenado(VELOCIDAD_EVASION, med.frontal)

        if abs(error_lat) < ERROR_LATERAL_OK or t_en > TIMEOUT_RECUPERACION:
            razon = "centrado" if abs(error_lat) < ERROR_LATERAL_OK else "tiempo maximo"
            self._entrar("CRUCERO", ahora)
            print("[FSM] RECUPERACION -> CRUCERO | %s (error lateral %+.0fmm)"
                  % (razon, error_lat))
            return (VELOCIDAD_CRUCERO,
                    self._nominal_arbitrado(med, VELOCIDAD_CRUCERO))
        return (velocidad, self._nominal_arbitrado(med, velocidad))

    def _est_aborto(self, med, color_cam, heading, ahora):
        # Salida definida cuando la maniobra no se puede terminar. Se
        # conduce al centro del carril despacio y no se captura nada
        # durante el refractario, para no entrar en un bucle de
        # capturar-abortar sobre el mismo pilar.
        velocidad = self._con_frenado(VELOCIDAD_EVASION, med.frontal)
        if ahora >= self._t_refractario:
            self._entrar("CRUCERO", ahora)
            print("[FSM] ABORTO -> CRUCERO")
        return (velocidad, self._nominal_arbitrado(med, velocidad))

    def _abortar(self, med, ahora, motivo):
        """Cierre definido de una maniobra que no se puede completar.

        Se abandona la REGLA (el lado obligatorio ya no se persigue) pero
        NO la estimacion: el pilar sigue ahi y sigue siendo algo con lo
        que se puede chocar. Se conserva como obstaculo hasta que quede
        atras o la estimacion muera. Sin esto, abortar delante de un
        pilar equivale a dejar de verlo, y el robot lo raspa.
        """
        self._abortos += 1
        espera = min(REFRACTARIO_MAX, REFRACTARIO_ABORTO * self._abortos)
        self._t_refractario = ahora + espera
        self._solo_obstaculo = self.tracker.activo
        self._t_obstaculo = ahora
        if self._solo_obstaculo:
            print("[OBJETIVO] #%d degradado a obstaculo: %s"
                  % (self.tracker.id, motivo))
        self._entrar("ABORTO", ahora)
        print("[FSM] ABORTO (%s) | %d seguidos, refractario %.1fs"
              % (motivo, self._abortos, espera))
        velocidad = self._con_frenado(VELOCIDAD_EVASION, med.frontal)
        return (velocidad, self._nominal_arbitrado(med, velocidad))

    def _obstaculo_pasivo(self, ahora=None):
        # El pilar de una maniobra abortada, solo para holgura. Ya no se
        # persigue la regla, pero sigue siendo algo con lo que chocar --
        # y mientras siga aqui no se captura nada nuevo, que es lo que
        # impide reenganchar el mismo poste una y otra vez.
        #
        # OJO: no se suelta por `perdido()`. Justo el aborto tipico es
        # "estimacion perdida", asi que soltarlo por eso lo devolveria al
        # bucle en el ciclo siguiente. Se suelta cuando queda detras, o
        # cuando la estimacion se va tan lejos que ya no puede estorbar,
        # o por tiempo.
        if not self._solo_obstaculo or not self.tracker.activo:
            return None
        ahora = self._t_ultimo_ciclo if ahora is None else ahora
        x_r, y_r = self.tracker.xy_eje()
        vencido = (ahora is not None and self._t_obstaculo is not None
                   and (ahora - self._t_obstaculo) > VIDA_OBSTACULO_ABORTADO)
        if y_r < -geo.VOLADIZO_TRASERO or math.hypot(x_r, y_r) > 1200.0 or vencido:
            self.tracker.soltar("obstaculo superado")
            self._solo_obstaculo = False
            return None
        return (x_r, y_r)

    def _sin_salida(self, med, ahora):
        """Ningun comando de marcha adelante es seguro.

        Pararse y esperar NO es una salida: la situacion no cambia sola,
        y si la emergencia no llega a dispararse (pared a 300mm, por
        ejemplo, con el radio minimo sin caber) el robot se queda
        empujando contra la pared para siempre. La unica accion que
        cambia la geometria es la marcha atras, asi que se entra en
        RETROCESO por la misma via que una emergencia -- suspendiendo la
        maniobra, no borrandola.
        """
        if self.estado in ESTADOS_MANIOBRA:
            self._estado_suspendido = self.estado
        if (self._t_ultima_emergencia is not None and
                (ahora - self._t_ultima_emergencia) <= VENTANA_ATASCO):
            self._racha_retroceso += 1
        else:
            self._racha_retroceso = 1
        self._t_ultima_emergencia = ahora
        self._entrar("RETROCESO", ahora)
        print("[SEGURIDAD] sin comando seguro de avance "
              "(F:%.0f I:%.0f D:%.0f) -> RETROCESO (racha %d, suspende %s)"
              % (med.frontal_muro, med.izquierda, med.derecha,
                 self._racha_retroceso, self._estado_suspendido or "nada"))
        return (VELOCIDAD_REVERSA, 0.0)

    def _est_retroceso(self, med, color_cam, heading, ahora):
        t_en = ahora - self._t_estado

        despejado = (med.frontal    > SALIDA_RETROCESO_FRONTAL and
                     med.izquierda  > SALIDA_RETROCESO_LATERAL and
                     med.derecha    > SALIDA_RETROCESO_LATERAL)

        if (med.trasera < EMERGENCIA_TRASERA
                or (t_en > TIEMPO_MIN_RETROCESO and despejado)
                or t_en > TIMEOUT_RETROCESO):
            if med.trasera < EMERGENCIA_TRASERA:
                razon = "obstaculo trasero"
            elif despejado:
                razon = "despejado en %.1fs" % t_en
            else:
                razon = "tiempo maximo"

            if self._racha_retroceso >= RACHA_RETROCESO_PARA_FORZAR:
                # Aqui SI hace falta un lado si o si: es el desempate de
                # ultimo recurso, y su valor esta en terminar el bucle.
                self._signo_giro_forzado = (self._preferencia_esquina(med)
                                            or LADO_POR_DEFECTO)
                self._entrar("GIRO_FORZADO", ahora)
                print("[FSM] RETROCESO -> GIRO_FORZADO (%s) | %d emergencias "
                      "encadenadas -> giro hacia %s"
                      % (razon, self._racha_retroceso,
                         "IZQUIERDA" if self._signo_giro_forzado > 0 else "DERECHA"))
            elif self._estado_suspendido:
                # La memoria del pilar sobrevivio a la emergencia. Si la
                # estimacion sigue viva se retoma la MISMA maniobra, con
                # el mismo id, color y lado.
                if self.tracker.activo and not self.tracker.perdido(ahora):
                    destino = self._estado_suspendido
                    if self._paso_validado:
                        # La regla ya esta cumplida: no hay nada que
                        # proteger y volver a SALIDA_PILAR solo reabre el
                        # bucle emergencia-salida-emergencia.
                        destino = "RECUPERACION"
                    self._estado_suspendido = None
                    self._t_ultimo_progreso = ahora
                    self._entrar(destino, ahora)
                    print("[FSM] RETROCESO -> %s (%s) | se retoma el pilar #%d %s"
                          % (destino, razon, self.tracker.id, self.tracker.color))
                else:
                    self._estado_suspendido = None
                    return self._abortar(med, ahora,
                                         "el objetivo no sobrevivio a la emergencia")
            else:
                self._entrar("CRUCERO", ahora)
                print("[FSM] RETROCESO -> CRUCERO (%s)" % razon)

        # Control P en vivo sobre las diagonales traseras: gira hacia el
        # lado con mas espacio medido en ESTE ciclo. En reversa el mismo
        # angulo de rueda gira el chasis al reves, por eso no vale un
        # signo precalculado.
        error  = med.trasera_derecha - med.trasera_izquierda
        angulo = _clamp(error * KP_RETROCESO, MAX_ANGULO_RETROCESO)
        return (VELOCIDAD_REVERSA, angulo)

    def _est_giro_forzado(self, med, color_cam, heading, ahora):
        # Desempate de esquina simetrica. Unico estado que NO recalcula su
        # decision cada ciclo: recalcularla con la misma señal simetrica
        # que causo el atasco la volveria a poner en 0.
        t_en = ahora - self._t_estado
        diff = med.izquierda - med.derecha
        asimetria = (diff * self._signo_giro_forzado) > SALIDA_GIRO_FORZADO_ASIMETRIA

        if asimetria or t_en > TIMEOUT_GIRO_FORZADO:
            razon = "asimetria recuperada" if asimetria else "tiempo maximo"
            self._racha_retroceso = 0
            self._t_ultima_emergencia = None
            if self._estado_suspendido or self.tracker.activo:
                # El giro forzado destruyo la geometria de la maniobra:
                # se cierra de forma explicita, no se retoma a ciegas.
                self._estado_suspendido = None
                print("[FSM] GIRO_FORZADO -> ABORTO (%s)" % razon)
                return self._abortar(med, ahora, "giro forzado durante la maniobra")
            self._entrar("CRUCERO", ahora)
            print("[FSM] GIRO_FORZADO -> CRUCERO (%s, %.1fs)" % (razon, t_en))
            return (VELOCIDAD_CRUCERO, self._nominal_arbitrado(med, VELOCIDAD_CRUCERO))

        angulo = self._signo_giro_forzado * ANGULO_GIRO_FORZADO
        velocidad = self._con_frenado(VELOCIDAD_GIRO_FORZADO, med.frontal)
        return (max(VELOCIDAD_MINIMA, velocidad), angulo)

    # ==========================================
    # FASE PARQUEO
    # ==========================================
    def _ciclo_parqueo(self, med, ahora):
        match_firma = (abs(med.derecha - self._firma_der) < TOLERANCIA_FIRMA and
                       abs(med.izquierda - self._firma_izq) < TOLERANCIA_FIRMA)
        timeout = (ahora - self._t_parqueo) > TIMEOUT_PARQUEO

        if match_firma or timeout:
            self.fase = "FIN"
            print("[PARQUEO] " + ("Firma detectada! Estacionando..." if match_firma
                                  else "Timeout. Deteniendo en zona segura."))
            return None

        return (VELOCIDAD_PARQUEO, self._nominal_arbitrado(med, VELOCIDAD_PARQUEO))

    # ==========================================
    # AUXILIARES
    # ==========================================
    def _entrar(self, estado, ahora):
        self.estado    = estado
        self._t_estado = ahora

    def _publicar_diagnostico(self):
        trk = self.tracker
        self.objetivo_id = trk.id if trk.activo else 0
        if trk.activo:
            x_r, y_r = trk.xy_eje()
            self.holgura_pilar = gev.holgura_arco(x_r, y_r, self._ultimo_angulo)
            self.separacion_lat = trk.separacion_lateral()
        else:
            self.holgura_pilar = 0.0
            self.separacion_lat = 0.0
