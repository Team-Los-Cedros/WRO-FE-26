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
from lidar_geometria import (ancho_cluster, bins_de_clusters,
                             centroide_xy_cluster, distancia_en_rango,
                             distancia_en_rango_sin_bins, es_objeto_estrecho)

# ==========================================
# VELOCIDADES (% PWM)
# ==========================================
VELOCIDAD_CRUCERO  = 100
# ES LA VELOCIDAD QUE MANDA EN LA CORRIDA, no la de crucero. Medido en
# la corrida de las 18:07 (5 bloques): los estados de maniobra
# -- APERTURA, CONTRAGIRO, PASO_LATERAL, SALIDA_PILAR, RECUPERACION --
# ocupan el 71,8% de los ciclos y van a 132 mm/s, mientras que CRUCERO es
# solo el 21,5% a 206 mm/s. Subir el crucero otra vez casi no se nota;
# esto si.
#
# Se sube de 40 a 55, el mismo valor que tenia el crucero antes. No es
# gratis y hay que mirarlo: mas velocidad en maniobra es menos distancia
# de reaccion, y el horizonte de la envolvente crece con la velocidad.
# Pero el precedente apunta al otro lado: subir el crucero de 55 a 70
# BAJO las emergencias (RETROCESO de 4,5% a 0,6%), porque el robot dejo
# de perder maniobras y de meterse en sitios de los que habia que salir.
# `_con_frenado` sigue aplicandose encima, asi que cerca de una pared
# esto no cambia nada.
# PROBADO Y REVERTIDO el 10-09 (corrida 18:14). Se subio a 55 porque los
# estados de maniobra son el 72% de los ciclos y parecia el mayor freno.
# El resultado fue el CONTRARIO: la maniobra si fue mas rapida (132 ->
# 148 mm/s) pero RETROCESO paso del 3,1% al 16,3% y ULTIMO_RECURSO del
# 4,3% al 17,5%, y la velocidad NETA cayo de 141 a 121 mm/s. Proyeccion
# de 3 vueltas: de 170 s a 198.
#
# La leccion, y vale para el resto: en maniobra la velocidad no compra
# tiempo, porque el tiempo se lo lleva desatascarse, no avanzar. Lo que
# hay que atacar es la DURACION de la maniobra, no su velocidad --
# PASO_LATERAL ocupa el 36% de los ciclos y APERTURA el 23% para solo
# cinco pilares, o sea unos 10 s de maniobra por poste.
VELOCIDAD_EVASION  = 55
# FRENAR PARA LEER UN POSTE QUE EL LiDAR VE Y LA CAMARA NO.
#
# El LiDAR detecta el pilar mucho antes que la camara: medido el 11-09,
# la camara deja de dar color mas alla de 1026 mm y la FSM usa pilares
# hasta 1400. En esos 370 mm el robot SABE que hay un poste pero no de
# que color es -- y sin color no hay lado obligatorio, asi que no lo
# ficha y acaba pasandolo por donde salga. En competencia eso termina el
# recorrido.
#
# Es el fallo que el equipo describio como "hay bloques que no los lee
# pero los ve con el LiDAR, por lo que no los pasa bien", y es tambien
# un punto ciego de la MEDIDA: un poste que nunca se ficha no aparece en
# el marcador de lado, asi que esas infracciones no se estaban contando.
#
# Lo unico que hace falta es acercarse mas despacio: la camara funciona
# de 1026 mm hacia dentro, asi que llegar mas lento da mas fotogramas por
# milimetro recorrido y, sobre todo, deja mas pista por delante cuando
# por fin lo lee. No se para -- eso seria regalar tiempo -- solo se baja
# a VELOCIDAD_LECTURA mientras haya un poste sin color a la vista.
# ENCARARSE AL POSTE QUE EL LiDAR VE Y LA CAMARA NO SABE COLOREAR.
#
# Sin color no hay lado obligatorio, y pasarlo "por donde salga" termina
# el recorrido. Asi que no se pasa: se apunta el morro hacia el y se va a
# por el, que es lo que pone el blob en el CENTRO del frame -- donde la
# camara lee mejor -- en vez de en el borde del cono de 68 grados.
#
# NO es lo mismo que frenar, que ya se probo y salio mal (el alcance del
# color no mejoro y la corrida cayo de 8 pasos y 0 infracciones a 3 y 2).
# Aquello daba mas fotogramas del mismo blob mal colocado; esto cambia
# DONDE cae el blob.
#
# Se suelta al acercarse mas de DIST_ENCARE_MIN: a esa distancia o se ha
# leido ya, o insistir es empotrarse contra el.
VELOCIDAD_LECTURA  = 45
DIST_ENCARE_MAX    = 1400.0
DIST_ENCARE_MIN    = 420.0
DIST_LECTURA_MAX   = 1400.0   # mismo alcance que usa el arbitraje
DIST_LECTURA_MIN   = 250.0    # mas cerca ya no sirve frenar (sin uso)

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

# EMPIEZA A FRENAR AQUI. Estaba en 900 mm, y en este anillo eso es
# frenar SIEMPRE: el pasillo mide ANCHO_CARRIL (1000 mm) y siempre hay una
# esquina delante, asi que `frontal` casi nunca pasa de 900. Medido en la
# corrida de las 16:01: el frenado estaba activo en el 66,6% de los ciclos
# y `frontal` tenia mediana 682 mm, con lo que CRUCERO promediaba 45,7 PWM
# en vez de los 55 comandados. El robot no iba a su velocidad de crucero
# practicamente nunca.
#
# Y frenar por geometria no hace falta: con el radio minimo real (243 mm,
# medido con cinta el 10-09) el robot cierra cualquier esquina del carril
# a cualquier velocidad. El propio repositorio ya lo habia demostrado --
# "FRENAR NO CAMBIA LA GEOMETRIA" -- asi que lo unico que compra frenar
# son ciclos de control por milimetro recorrido y distancia de parada.
#
# 600 mm deja ~3 s de rampa a velocidad de crucero, que son 30 ciclos de
# control: de sobra para reaccionar. Por debajo de DIST_FRENADO_MIN el
# suelo sigue siendo VELOCIDAD_MINIMA.
DIST_FRENADO_INICIO = 600.0  # mm, empieza a bajar velocidad
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

# Piso absoluto del horizonte frontal cuando el hueco obliga a encogerlo
# (ver _comandos_seguros). Es lo que el robot recorre en un ciclo de
# control a velocidad de evasion -- 40% de PWM son 160 mm/s, y a 10 Hz
# eso son 16 mm -- con holgura. Por debajo de esto el arco ya no se
# comprueba de verdad, asi que aqui se deja de encoger y el conjunto
# puede quedar vacio con razon: no hay sitio ni para un ciclo.
HORIZONTE_FRENTE_PISO = 45.0   # mm

# ==========================================
# RELAJACION DE LA ENVOLVENTE
# ==========================================
# Con esto en True, _comandos_seguros hace dos concesiones que quitan el
# suelo estructural de 254 mm de pared frontal (ver HORIZONTE_FRENTE_MIN):
# encoge el horizonte frontal segun se cierra el hueco, y deja de exigir
# el margen lateral cuando el robot YA esta mas cerca que el margen
# (pasa a exigir solo "no acercarte mas").
#
# Funciona: en simulacion SIN_SALIDA cae del 6-12% al 0% en cinco de seis
# escenarios. PERO no sale gratis, y esto se midio antes de encenderlo:
# aparecen choques contra muro (0 -> 7 en un escenario, 0 -> 120 en otro)
# y un roce de -8 mm contra un pilar, o sea contacto. Una ablacion
# separada confirmo que no lo causa el peldaño de ultimo recurso.
#
# La decision es medir primero sin esto. El 82% de SIN_SALIDA de la
# corrida del 09-09 se produjo con la trasera clavada en 47 mm y con
# angulo_muro saturado en el 91% de los ciclos: las dos cosas ya estan
# arregladas aguas arriba, y puede que el conjunto admisible deje de
# vaciarse solo. Encender esto sin comprobarlo seria cambiar un modo de
# fallo (se para) por otro peor (choca), que en WRO cuesta puntos.
#
# COMO DECIDIR: correr con esto en False y pasar el CSV por
# `analizar_corrida.py`. Si SIN_SALIDA sigue por encima del 10% y las
# rachas entran con un lado por debajo de 115 mm, encenderlo y repetir.
# Se separa en DOS porque no cuestan lo mismo:
#
# ENCENDIDO el 10-09 por la tarde. Estuvo apagado toda la sesion por una
# simulacion quedio 7 choques y un contacto de -8 mm con un pilar --
# pero aquella simulacion corria con RADIO_MIN_DER = 360, un 48% por
# encima del valor real (244, medido con cinta). Repetida con la
# geometria correcta: CERO choques y cero contactos con las dos
# posiciones de la bandera, en los seis escenarios.
#
# Y en pista el suelo si muerde: 15,5% de los ciclos con frontal_muro por
# debajo de 254 mm, contra un 0,8% por debajo del umbral de emergencia.
# De 7 entradas en RETROCESO, 4 fueron por envolvente vacia y 3 por
# emergencia. O sea que el suelo estructural es la causa dominante de las
# marchas atras, y la emergencia es su consecuencia.
#
#   RELAJAR_FRENTE  encoge el horizonte frontal segun se cierra el hueco.
#       Quita el suelo de 254 mm. En las dos corridas del 10-09 este es
#       el 100% de lo que queda: 40 de 60 rachas con frontal_muro < 260 y
#       CERO por un lado. Solo afecta a comandos que ya iban a chocar de
#       morro, y el que sobrevive es el que mas dobla.
#
#   RELAJAR_LATERAL deja de exigir el margen cuando el robot YA esta mas
#       cerca que el margen. Esta es la que en simulacion cambiaba
#       paradas por choques (0 -> 7 contra muro y un roce de -8 mm contra
#       un pilar). Y en pista ya no hace falta: con la trasera y el
#       angulo de pared arreglados, la causa lateral de SIN_SALIDA paso
#       de 791 de 818 ciclos a 0 de 60.
RELAJAR_FRENTE  = True
RELAJAR_LATERAL = False

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
# El mismo peldaño que MARGEN_PARED_DURO, pero de frente. Faltaba: la
# escalera de concesiones aflojaba el lateral y dejaba el frontal clavado
# en 40, asi que una pared frontal cerrada vaciaba el conjunto en los dos
# peldaños a la vez y la unica respuesta era marcha atras. Queda muy por
# encima de EMERGENCIA_FRONTAL, que sigue siendo el reflejo de verdad.
MARGEN_FRENTE_DURO = 12.0

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
# Distancia de anticipacion de la persecucion pura: el punto de paso
# nunca se pone mas cerca que esto por delante del eje trasero. Es lo que
# evita que la consigna se vuelva una curva cerrada cuando el poste ya
# esta al costado -- y lo que mantiene viva la persecucion en la parte de
# la maniobra que de verdad importa.
#
# 300 mm es algo mas que el radio minimo medido (243): el arco que sale
# de perseguir a esa distancia siempre es ejecutable.
ADELANTO_MINIMO_PERSECUCION = 300.0

PROGRESO_PASO = 100.0

# Ciclos consecutivos cumpliendo el invariante antes de declararlo. Evita
# que un ciclo con una asociacion mala cierre la maniobra antes de tiempo.
CICLOS_PASO = 3

# Margen por el que un pilar puede estar mas lejos que la pared de
# enfrente y seguir siendo de ESTA seccion. Cubre que el poste este
# lateral (su `y` puede pasar de `frontal_muro` sin estar tras la
# esquina) y el ruido del sector frontal. Ver _est_crucero.
MARGEN_SECCION = 250.0

# Medio ancho del sector con el que se mide la pared que hay DETRAS del
# poste, en su mismo rumbo. Un poste de 50 mm subtiende 14 grados a 200
# mm, asi que +-12 coge pared a los dos lados de el sin irse a otra cosa.
# Media ventana, en grados, con la que se pregunta al perfil que hay en
# el rumbo de una esquina barrida. Estrecha para que siga siendo "ESE
# rumbo" y no un sector, pero mas ancha que un bin para no depender de
# donde caiga el redondeo.
VENTANA_RUMBO_BARRIDO = 8.0

VENTANA_RUMBO_SECCION = 12.0

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

# Por delante de esta coordenada (marco del eje trasero) el pilar todavia
# manda el LADO; por detras, solo manda no tocarlo. Ver _compatibles.
#
# El valor es el travez del eje delantero: con el poste por detras de las
# ruedas directrices ya no hay arco que cambie por que lado se pasa -- eso
# quedo decidido. Exigir el lado mas alla de aqui no es conservador, es
# pedir una curva que no termina nunca.
Y_PILAR_MANDA_LA_REGLA = geo.BATALLA

# ==========================================
# APAREO COLOR <-> CLUSTER (que pilar es cual)
# ==========================================
# La camara dice QUE color hay delante y el LiDAR DONDE hay postes. Se
# aparean por RUMBO, no por cercania: el pixel cx se convierte a rumbo con
# el modelo estenopeico medido (optica.py) y se elige el cluster que este
# en ese mismo rumbo. Con dos pilares en el frame, "el color del blob mas
# grande" y "el cluster mas cercano" pueden caer en pilares distintos.
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

# Sector dentro del cual la distancia frontal y el rumbo de camara hablan
# del MISMO objeto. Es el sector frontal del LiDAR, de donde sale
# `med.frontal`; sembrar con un rumbo de fuera de aqui es mezclar dos
# medidas de direcciones distintas. Ver _intentar_capturar.
SECTOR_FRONTAL_SIEMBRA = (-10.0, 10.0)

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

# Retroceso que ademas RECOLOCA: dentro del margen que deja la seguridad
# trasera, se gira para dejar el pilar centrado en la camara, de forma
# que el avance siguiente empiece encarandolo. Ver _est_retroceso.
KP_ENCARE_REVERSA = 0.35     # grados de servo por grado de rumbo
FRACCION_ENCARE   = 0.7      # cuanto del margen libre se le cede

# ==========================================
# DESEMPATE DE ESQUINA SIMETRICA (GIRO_FORZADO)
# ==========================================
RACHA_RETROCESO_PARA_FORZAR = 4

# ENCAJONADO: LAS TRES DISTANCIAS PEQUEÑAS A LA VEZ.
#
# El desempate se disparaba SOLO tras cuatro emergencias encadenadas, y
# eso deja fuera el caso que mas duele: el robot metido de morro en una
# esquina, avanzando y retrocediendo sin encadenar la racha. Medido en la
# corrida 120004: el 19,1% de los ciclos con frente, izquierda y derecha
# por debajo de 200 mm -- un quinto de la vuelta encajonado -- y el
# desempate solo entro TRES veces, y las tres con 300 mm libres, o sea
# cuando no hacia falta.
#
# Estar en un hueco de 200 mm por los tres lados no es una situacion de
# la que se salga conduciendo: hay que PONERSE A RUMBO con el cuadrante,
# que es lo que hace GIRO_FORZADO. Se pide medio segundo seguido para no
# dispararlo con un barrido suelto.
# DURACION MINIMA DE CADA TRAMO DEL DESEMPATE.
#
# Sin esto la decision "adelante o atras" es un umbral desnudo
# (`frontal > SALIDA_GIRO_FORZADO_FRENTE`) y la lectura del LiDAR
# fluctua justo alrededor de el: el robot cambia de sentido CADA CICLO y
# se queda clavado. Medido en la corrida 121145: cinco episodios de 9,1 s
# con velocidad media de +-1 -- o sea parado -- teniendo 3 metros libres
# detras, y girando 6 grados en nueve segundos.
#
# La bitacora del proyecto ya lo traia escrito del K-turn del 31-08: "un
# umbral fijo en medio hacia oscilar la maniobra a 5 Hz sin que el robot
# se moviera; se resolvio con duracion minima de tramo". Mismo error.
TRAMO_MIN_GIRO     = 0.8      # s antes de poder invertir el sentido
ENCAJONADO_MM      = 200.0
CICLOS_ENCAJONADO  = 5

# LA MARCHA ATRAS DEL DESEMPATE SUMA ROTACION (maniobra en tres tiempos).
#
# El ciclo limite, medido dos veces con el mismo codigo (corridas 221357
# y 223336) y ausente en la unica corrida que llevaba esto (222018):
#
#   CRUCERO   frontal 320 -> 130, volante -20 (derecha), heading -30 -> -45
#   RETROCESO frontal vuelve a 300,                      heading -45 ->  +4
#   CRUCERO   otra vez lo mismo,                         heading  +4 -> -28
#
# Gana 49 grados retrocediendo y los devuelve enteros al avanzar: neto
# CERO. En 221357 fueron 68 segundos de los 110; en 223336, tres
# episodios encadenados. GIRO_FORZADO sale siempre por "tiempo maximo",
# nunca por recuperar la asimetria: gira 2,6 s, no consigue nada, y
# CRUCERO lo devuelve al mismo rincon (F:117 I:101 D:115).
#
# La marcha atras no llevaba intencion: se guiaba solo por las diagonales
# traseras, asi que la rotacion que ganaba era casualidad. Un conductor
# atascado en una esquina no retrocede "hacia donde quepa": retrocede
# girando el volante al lado que le deja ENCARADO para salir.
#
# En reversa el chasis gira al reves para el mismo angulo de rueda, asi
# que para seguir rotando en el sentido del desempate hay que pedir el
# signo CONTRARIO. Con eso cada ida y vuelta gana angulo neto.
#
# Solo mientras el desempate esta activo (`_signo_giro_forzado` distinto
# de cero): fuera de un atasco, la marcha atras normal no tiene ninguna
# razon para preferir un lado.
FRACCION_ROTA_REVERSA = 0.8

# ALINEARSE CON EL CUADRANTE QUE IBA A CRUZAR (idea del equipo).
#
# El desempate no tenia META: giraba a ciegas y salia por reloj a los
# 2,6 s -- "GIRO_FORZADO -> CRUCERO (tiempo maximo)" tres veces seguidas
# en la corrida 223336 -- sin haber conseguido nada, y CRUCERO lo
# devolvia al mismo rincon (F:117 I:101 D:115).
#
# La pista es un anillo cuadrado: los tramos rectos estan a 90 grados
# unos de otros. Asi que cuando el robot se mete de morro en una esquina
# no hay nada que "decidir", hay que PONERSE A RUMBO: el siguiente
# multiplo de 90 grados en el sentido de la vuelta. Eso es una meta
# medible con el yaw que ya tenemos, y convierte un giro a ciegas en una
# maniobra que se sabe cuando ha terminado.
#
# Y si no cabe hacia adelante, se hace en tres tiempos DENTRO del propio
# estado: adelante girando hasta la pared, atras girando al reves --que
# en reversa rota el chasis en el MISMO sentido-- y otra vez. Antes eso
# salia del estado y lo perdia todo.
TOLERANCIA_ALINEACION   = 15.0   # grados; el control de pista hace el resto
TRASERA_PARA_TRES_TIEMPOS = 300.0

# Retrocesos encadenados antes de permitir el peldaño de ultimo recurso.
# Con 1 (que era el efecto de la version anterior) el modo se convertia en
# el normal en cuanto habia emergencias.
RETROCESOS_PARA_ULTIMO_RECURSO = 1
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
# LADO_POR_DEFECTO ELIMINADO. Valia 1.0 (izquierda) y era el origen del
# bucle que se autoconfirma: sin evidencia el robot giraba a la izquierda,
# eso generaba yaw positivo, el estimador por yaw declaraba ANTIHORARIA y
# desde ahi toda esquina se tomaba a la izquierda -- con la pista montada
# en sentido horario. En competicion el sentido se SORTEA, asi que
# cualquier valor por defecto es una apuesta a cara o cruz.
# Ahora, sin evidencia, `_preferencia_esquina` devuelve None y el termino
# de esquina se apaga. Ver SentidoPorGeometria en sentido_vuelta.py.

# Razon minima entre las diagonales frontales (10-60 contra 300-350) para
# creerse que el pasillo dobla hacia un lado. Por debajo, sin dato.
# Se pide mas que en SentidoPorGeometria (1.15) porque alli el voto se
# ACUMULA sobre muchos barridos y aqui decide un ciclo el solo.
RAZON_DIAGONAL_ESQUINA = 1.25   # sin unidades
ANGULO_GIRO_FORZADO    = 25.0
VELOCIDAD_GIRO_FORZADO = VELOCIDAD_EVASION
# 9 s, no 2,5. La meta del desempate es ponerse a rumbo con el cuadrante
# siguiente -- 90 grados -- y con 2,5 s no le daba tiempo ni de acercarse:
# medido en la corrida 120004, los tres episodios salieron TODOS por
# "tiempo maximo" habiendo girado 24, 25 y 2 grados. Giraba, sonaba el
# reloj, y CRUCERO lo devolvia al mismo rincon.
#
# El estado ya sabe terminarse solo: sale al llegar a rumbo, al
# recuperarse la asimetria, o si no hay sitio ni delante ni detras. El
# reloj es la red de seguridad, no el criterio, y tenerlo por debajo de
# lo que dura la maniobra lo convertia en el criterio.
TIMEOUT_GIRO_FORZADO   = 9.0
SALIDA_GIRO_FORZADO_ASIMETRIA = 150.0
# Suelo de pared del giro forzado. Por encima de EMERGENCIA_FRONTAL
# para salir ANTES de que salte el reflejo, no despues: asi la salida es
# ordenada (CRUCERO o ABORTO con su arbitraje) en vez de un rebote
# GIRO_FORZADO -> RETROCESO -> GIRO_FORZADO.
SALIDA_GIRO_FORZADO_FRENTE = 160.0

# ==========================================
# CARRERA / PARQUEO
# ==========================================
# ===== CUANTAS VUELTAS: SE CUENTAN LINEAS, NO GRADOS NI SEGUNDOS =====
#
# Cada esquina de la pista lleva una linea naranja y una azul. Dando la
# vuelta se cruzan las cuatro esquinas, o sea CUATRO lineas de un mismo
# color por vuelta. Doce cruces del color que manda = tres vueltas.
#
# Y hay una propiedad que el yaw no tiene: contar tramos devuelve al robot
# al MISMO TRAMO del que salio, sin importar en que punto del tramo
# arrancara. Que es justo lo que pide el reglamento -- parar en el
# cuadrante del estacionamiento.
#
# Se cuenta un solo color: el que va DELANTE en el sentido de la vuelta
# (naranja si es horaria, azul si es antihoraria), que es el mismo
# criterio con el que se fija el sentido.
VUELTAS_OBJETIVO     = 3
LINEAS_POR_VUELTA    = 4
# Refractario entre cruces.
#
# 5 s, no 2. MEDIDO en la corrida de las 13:23: el contador dio 12 lineas
# (o sea 3 vueltas) con solo 506 grados de guiñada, que son vuelta y
# media. Exactamente el DOBLE, y 2 por esquina: la misma linea se vuelve
# a ver despues de que el robot gire los 90 grados de la esquina, y con
# 2 s de refractario eso cuenta otra vez.
#
# A 221 mm/s un tramo dura unos 12 s, asi que 5 no puede tapar un cruce
# de verdad y si tapa la segunda vista de uno.
REFRACTARIO_LINEA    = 5.0     # s

# Y ADEMAS, GUIÑADA MINIMA ENTRE CRUCES.
#
# Dos lineas del mismo color estan separadas por una esquina, o sea por
# unos 90 grados de giro. Si entre dos cuentas el robot apenas ha girado,
# la segunda no es otra esquina: es la misma linea vista dos veces, o una
# mancha. El reloj solo no basta -- medido en la corrida de las 13:44:
# 871 grados de guiñada (2,42 vueltas) contados como 12 lineas (3), con
# huecos de 5,7 a 7,9 s que el refractario no tapa.
#
# Esto es independiente de la velocidad, que es justo lo que le faltaba
# al refractario.
GIRO_MIN_ENTRE_LINEAS = 55.0   # grados
# Solo cuenta si la linea esta ya por debajo de esta fraccion del alto:
# arriba del frame la linea esta lejos y todavia no se ha cruzado.
LINEA_CY_MINIMA      = 0.55

# RED, NO CRITERIO. Si el contador de lineas se quedara ciego (camara
# tapada, lineas gastadas) esto acaba la ronda igual. Va MUY por encima
# de las 3 vueltas reales (~1010 grados) a proposito: si saltara antes que
# el contador, seria el criterio de verdad, y no lo es.
UMBRAL_VUELTAS       = 1350.0  # grados de yaw neto; red de seguridad
TOLERANCIA_FIRMA     = 80.0    # mm contra la firma de pared inicial
# Una lectura fuera de esta banda no es el carril: o es el valor semilla
# de un sector sin eco (2000 mm) o es una diagonal de esquina.
DIST_FIRMA_MIN       = 60.0
DIST_FIRMA_MAX       = 1500.0
MUESTRAS_FIRMA       = 8       # barridos validos antes de fijar la firma
TIMEOUT_FIRMA        = 3.0     # s; pasado esto se arranca con lo que haya
# 12 s, no 6. Este reloj NO es el criterio de parada: la fase de parqueo
# arranca al cumplir UMBRAL_VUELTAS (yaw, o sea vueltas de verdad) y para
# al reconocer la FIRMA DE PAREDES que se capturo al arrancar la carrera
# -- que desde que el arranque va encadenado al `parqueo.py` es el sitio
# justo al lado del estacionamiento, fuera de el. El reloj solo cubre el
# caso de que la firma no aparezca, y con 6 s apenas daba un 15% de
# vuelta para encontrarla.
TIMEOUT_PARQUEO      = 12.0    # s

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
# RENOMBRADOS. Se llamaban SERVO_MAX_IZQ/DER, y `lidar_geometria` define
# esos MISMOS nombres con otro significado: alli son bordes de sector del
# barrido (30 y 90 grados de rumbo), aqui son topes del servo (25 y 20
# grados de comando). Este modulo importa de aquel, asi que bastaba una
# linea de mas en el import para que el tope del servo pasara a valer 90.
SERVO_MAX_IZQ = gev.COMANDO_MAX_IZQ
SERVO_MAX_DER = gev.COMANDO_MAX_DER

ESTADOS_MANIOBRA = ("COMPROMISO", "APERTURA", "CONTRAGIRO",
                    "PASO_LATERAL", "SALIDA_PILAR")

# REINTENTO: la marcha atras que evita pasar por el lado prohibido.
#
# Pasar un bloque por el lado que no toca TERMINA el recorrido en
# competicion. Eso convierte a `_abortar` -- que por diseño "abandona la
# REGLA" y sigue hacia adelante -- en la accion mas cara del programa:
# medido en las cuatro corridas del 10-09, los 3 abortos de la corrida
# 202956 son EXACTAMENTE los 3 pasos por el lado prohibido, y el primero
# llega a los 42 s. La carrera no duraba 12 pilares: duraba 4.
#
# Retroceder cuesta segundos; abortar cuesta la carrera entera. Asi que
# mientras el pilar siga DELANTE y del lado malo, la respuesta correcta
# no es rendirse sino echarse atras y volver a entrar. El aborto de
# verdad queda como ultimo recurso, cuando ni retrocediendo se puede.
# Sigma maxima con la que se admite la clausula 0 del invariante de paso
# (el pilar "ya esta detras"). Esa clausula es la unica que NO descuenta
# la incertidumbre, y sin este tope una estimacion muerta fabrica pasos
# que no ocurrieron: corrida 204651, pilar #3 ROJO. Estuvo en el lado
# PROHIBIDO los 11 segundos de maniobra -- separacion de -161 a -351 mm,
# cruzo el eje a -236 -- y al perderlo el tracker (fuera_de_puerta y
# luego sin_candidatos) la estimacion derivo con sigma 30 -> 184 hasta
# cambiar de signo, y la clausula 0 declaro "rebasado a 166mm por su
# lado" con el poste 770 mm por detras. Un hecho geometrico no se puede
# afirmar con una medida que ya no existe.
#
# El corte separa limpiamente los dos regimenes medidos: sigma 30 con el
# tracker sano, y 36-184 en la deriva.
SIGMA_MAX_PARA_PASO  = 60.0

REINTENTOS_MAX       = 2      # por pilar; luego si se aborta de verdad
TIMEOUT_REINTENTO    = 2.5    # s
TIEMPO_MIN_REINTENTO = 0.5    # s antes de admitir que ya hay paso legal

# Ganancia del enderezado contra las paredes durante el reintento. La
# entrada va acotada a MAX_DERIVA_MURO, que es el limite de CREDIBILIDAD
# del angulo de pared, no una ganancia bajada a ojo: mas alla de ahi el
# ajuste por PCA ya no describe una pared plana.
KP_PARALELO_REVERSA  = 1.0

# SESGO DE CARRIL HACIA EL SIGUIENTE BLOQUE ("ver el bloque y ver el
# siguiente, y trazar una diagonal que pase los dos").
#
# El problema medido, corrida 212600: los dos unicos fallos son el MISMO
# pilar fisico en dos vueltas, y los dos son el ROJO que viene justo
# DESPUES de un verde. Se captura en (349, 416) y (353, 421): cruzar 350
# mm de lado en 420 de avance con radio minimo 243 no cabe. `sep_lat` es
# negativa del primer ciclo al ultimo -- nunca cruza.
#
# La camara YA lo esta viendo (color_cam = ROJO desde el ciclo 0 del
# objetivo anterior). Lo que falta no es VERLO antes sino ACTUAR antes:
# con un solo hueco en el tracker el rojo no existe hasta que se suelta
# el verde, y para entonces la maniobra ya es imposible.
#
# Esto no captura dos pilares -- eso pide un segundo tracker -- sino que
# usa la unica informacion que ya esta disponible y se estaba tirando: el
# COLOR del siguiente, que da su lado obligatorio. Con el lado basta para
# salir del bloque actual ya desplazado hacia el carril que el siguiente
# necesita, en vez de volver al centro y tener que cruzar entero.
#
# Desplazamiento respecto al centro del carril, en mm. Acotado a mucho
# menos de la media anchura util (~437 mm con carril de 1000 y robot de
# 125): deja mas de 300 mm a la pared, muy por encima de MARGEN_PARED.
SESGO_SIGUIENTE_MM   = 120.0
# Cuanto vive la observacion. Si el siguiente deja de verse, el carril
# vuelve al centro: un sesgo sostenido con informacion vieja es peor que
# ninguno.
VIDA_SIGUIENTE       = 2.5

# "La separacion al lado obligatorio dejo de mejorar y empieza a
# empeorar": el disparador de REINTENTO que faltaba.
#
# Medido en la corrida 215330, pilar #7 ROJO -- el unico fallo de esa
# corrida, y el mismo poste que falla vuelta tras vuelta:
#
#     t=0,00 sep=-346   abre bien
#     t=1,20 sep= -89
#     t=1,40 sep= -48   frontal 1054 -> 247   <- aparece pared de frente
#     t=1,60 sep= -14   frontal 214, n_comp 19 -> 2
#     t=1,80 sep= -21   t=2,40 sep=-68   t=3,00 sep=-166
#
# Estuvo a 14 mm de cruzar al lado bueno y una pared frontal le vacio el
# conjunto compatible: tuvo que apartarse del pilar para no chocar y
# devolvio toda la separacion ganada. Ese rojo esta justo antes de una
# esquina y abrir hacia su lado mete al robot en ella.
#
# Nada abortaba ahi. El robot seguia moliendo hacia adelante con
# n_comp=1 hasta que el tracker moria medio segundo despues, y para
# entonces ya habia pasado por el lado prohibido. `_hay_progreso` no lo
# ve porque mide el BARRIDO DEL RUMBO, que si avanzaba: el robot
# progresaba, solo que hacia el lado equivocado.
#
# El aborto que dispara esto no termina la maniobra: entra por
# `_quiere_reintentar`, o sea marcha atras y volver a entrar, que es
# exactamente lo que hace falta cuando el intento hacia adelante se ha
# quedado sin sitio.
MARGEN_SEP_EMPEORA   = 25.0   # mm por debajo del mejor registrado
CICLOS_SEP_EMPEORA   = 5      # medio segundo a 10 Hz

# Rejilla de comandos, de tope derecho a tope izquierdo.
CANDIDATOS = []
_c = -SERVO_MAX_DER
while _c <= SERVO_MAX_IZQ + 1e-6:
    CANDIDATOS.append(round(_c, 3))
    _c += PASO_REJILLA
if abs(CANDIDATOS[-1] - SERVO_MAX_IZQ) > 1e-6:
    CANDIDATOS.append(SERVO_MAX_IZQ)


def _clamp(v, lim):
    return max(-lim, min(lim, v))


def _clamp_servo(v):
    # Recorte al recorrido fisico real (positivo = izquierda)
    return max(-SERVO_MAX_DER, min(SERVO_MAX_IZQ, v))


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
        # El sentido de la vuelta se sortea en competicion, asi que no
        # puede salir de una constante -- y tampoco del yaw del propio
        # robot, que se autoconfirma (ver SentidoPorGeometria). Este mira
        # la PISTA: que muro se acaba y hacia donde llegan mas lejos las
        # diagonales frontales. `self.sentido` (por yaw) se conserva como
        # contraste y para rumbo_salida.
        self.pista = sentido_vuelta.SentidoPorGeometria()

        self.fase   = "CAPTURA_FIRMA"
        self.estado = "CRUCERO"

        self._firma_izq = 0.0
        self._firma_der = 0.0
        self._muestras_firma = []

        self._t_estado  = 0.0
        self._t_parqueo = 0.0
        # Conteo de vueltas por lineas del suelo. Ver VUELTAS_OBJETIVO.
        self._lineas_cruzadas = 0
        self._color_contado = None
        # None, no 0.0: con 0.0 el refractario se come el PRIMER cruce
        # si el reloj de la carrera empieza cerca de cero.
        self._t_ultima_linea = None
        self._giro_ultima_linea = None
        self._linea_encima = False

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
        self._t_aviso_lectura = None
        self._ciclos_encajonado = 0
        self._t_tramo_giro = None
        self._sentido_tramo_giro = None
        self._id_objetivo    = 0
        self._siguiente_lado = 0
        self._t_siguiente    = None
        self._ciego_previo   = False
        self._mejor_sep      = None
        self._ciclos_empeora = 0
        self._ciclos_suspension = 0
        self._reintentos = 0
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
        self._heading_objetivo   = None
        self._esquina_latch = False
        self._signo_esquina = 0.0   # 0 = sin lado; el latch solo se arma con evidencia

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
        # Quien decidio hacia donde dobla la esquina en el ultimo ciclo:
        # MEDIDO (barrido de ahora) / LATCH / YAW (estado interno) /
        # DEFECTO. Si en una corrida manda YAW, el sentido se esta
        # imponiendo en vez de medirse -- ver _preferencia_esquina.
        self.fuente_lado = ""

    # ==========================================
    # ENTRADA
    # ==========================================
    def procesar(self, med, color_cam, heading, ahora=None, cx_cam=None,
                 linea_cam=None,
                 color_piso=None):
        """Una llamada por barrido. Devuelve (velocidad, angulo) o None.

        El limitador del servo se aplica AQUI, a la salida de cualquier
        fase. Antes vivia dentro de `_ciclo_carrera` y habia dos caminos
        que se lo saltaban: el `return` del paso 2 al entrar en parqueo,
        y la fase PARQUEO entera, que se despacha un nivel mas arriba.
        Por esos dos caminos salian saltos de tope a tope sin limitar, y
        ademas `_ultimo_angulo` y `_ultima_vel` se quedaban viejos -- con
        lo que el ciclo siguiente calculaba mal su propio salto y el
        tracker su propio avance.
        """
        if ahora is None:
            ahora = time.time()

        dt = 0.0 if self._t_ultimo_ciclo is None else min(0.3, ahora - self._t_ultimo_ciclo)
        self._t_ultimo_ciclo = ahora

        if self.fase == "CAPTURA_FIRMA":
            # LA FIRMA NO SE TOMA DEL PRIMER BARRIDO. Ese llega parcial
            # -- el LiDAR acaba de arrancar -- y los sectores sin eco
            # salen con el valor SEMILLA de lidar_geometria (2000 mm).
            #
            # Medido en la ronda completa del 11-09: la firma quedo
            # "Izq=384 Der=2000". Los 2000 no son una medida, son la
            # semilla. Con la derecha midiendo 400-930 mm en el pasillo,
            # la tolerancia de +-80 era inalcanzable y el robot NUNCA
            # podia reconocer su sitio: se comio la fase de parqueo
            # entera buscando una pared que no existe.
            #
            # (El mismo fallo estaba anotado desde agosto en el paquete
            # ronda_nueva, corrida 174028. Se repitio aqui porque el
            # bloque se copio tal cual.)
            #
            # Ahora se acumulan lecturas que de verdad lo sean y se toma
            # la mediana, que ademas aguanta un barrido suelto malo.
            if (DIST_FIRMA_MIN < med.izquierda < DIST_FIRMA_MAX
                    and DIST_FIRMA_MIN < med.derecha < DIST_FIRMA_MAX):
                self._muestras_firma.append((med.izquierda, med.derecha))
            if len(self._muestras_firma) < MUESTRAS_FIRMA:
                if (ahora - self._t_estado) < TIMEOUT_FIRMA:
                    return (0, 0.0)
                print("[-] firma de parqueo con solo %d muestras validas: "
                      "el robot puede no reconocer su sitio al volver"
                      % len(self._muestras_firma))
            if self._muestras_firma:
                izq = sorted(m[0] for m in self._muestras_firma)
                der = sorted(m[1] for m in self._muestras_firma)
                n = len(izq) // 2
                self._firma_izq, self._firma_der = izq[n], der[n]
            else:
                self._firma_izq, self._firma_der = med.izquierda, med.derecha
            self.fase = "CARRERA"
            self._t_estado = ahora
            self._t_ultimo_progreso = ahora
            print("[+] Firma de parqueo: Izq=%.0f Der=%.0fmm (%d muestras)"
                  % (self._firma_izq, self._firma_der, len(self._muestras_firma)))
            print("[INICIO] Carrera con obstaculos iniciada!")
            return (0, 0.0)

        if self.fase == "CARRERA":
            consigna = self._ciclo_carrera(med, color_cam, heading, ahora, dt,
                                           cx_cam, color_piso, linea_cam=linea_cam)
        elif self.fase == "PARQUEO":
            consigna = self._ciclo_parqueo(med, ahora)
        else:
            return None    # FIN

        if consigna is None:
            return None
        velocidad, angulo = consigna
        angulo = self._limitar_servo(angulo)
        self._ultima_vel = velocidad
        self._publicar_diagnostico()
        return (velocidad, angulo)

    def _limitar_servo(self, angulo):
        # En emergencia no se limita: ahi el giro completo tiene que
        # entrar de una. Este limitador es la razon de que las
        # transiciones se evaluen con T_ADELANTO_SERVO de adelanto.
        if self.estado in ("RETROCESO", "REINTENTO"):
            # Los dos van marcha atras y los dos necesitan el angulo
            # entero de una: rampar el servo mientras se retrocede gasta
            # el recorrido util en llegar al angulo, no en usarlo.
            self._ultimo_angulo = _clamp_servo(angulo)
        else:
            delta = _clamp(angulo - self._ultimo_angulo, MAX_DELTA_ANGULO)
            self._ultimo_angulo = _clamp_servo(self._ultimo_angulo + delta)
        return self._ultimo_angulo

    # ==========================================
    # FASE CARRERA
    # ==========================================
    def _contar_lineas(self, linea_cam, ahora, heading):
        """Cuenta tramos de pista por las lineas del suelo.

        Un cruce no es "ver una linea", es verla PASAR. Se cuenta el
        flanco de subida -- aparece ya por debajo de LINEA_CY_MINIMA, o
        sea cerca del robot -- y hasta que no desaparece no se cuenta
        otra vez. Con el refractario encima, un parpadeo del detector no
        suma dos cruces.
        """
        if self._color_contado is None:
            # El color que manda es el que va DELANTE en el sentido de la
            # vuelta. Mientras no se sepa el sentido se adopta el primero
            # que se vea, que es el mismo criterio con el que se fija el
            # sentido: no pueden discrepar.
            if linea_cam is None:
                return
            self._color_contado = linea_cam[0]

        encima = (linea_cam is not None
                  and linea_cam[0] == self._color_contado
                  and linea_cam[1] >= LINEA_CY_MINIMA)
        if not encima:
            self._linea_encima = False
            return
        if self._linea_encima:
            return
        self._linea_encima = True
        if (self._t_ultima_linea is not None
                and ahora - self._t_ultima_linea < REFRACTARIO_LINEA):
            return
        if (self._giro_ultima_linea is not None
                and abs(heading - self._giro_ultima_linea) < GIRO_MIN_ENTRE_LINEAS):
            return
        anterior = self._t_ultima_linea
        self._t_ultima_linea = ahora
        self._giro_ultima_linea = heading
        self._lineas_cruzadas += 1
        print("[VUELTA] linea %s #%d de %d  (%.2f vueltas) | t=%.1fs, "
              "%s desde la anterior"
              % (self._color_contado, self._lineas_cruzadas,
                 LINEAS_POR_VUELTA * VUELTAS_OBJETIVO,
                 self._lineas_cruzadas / float(LINEAS_POR_VUELTA), ahora,
                 "primera" if anterior is None else "%.1fs" % (ahora - anterior)))
        print("         yaw %.0f grados" % heading)

    def _ciclo_carrera(self, med, color_cam, heading, ahora, dt, cx_cam=None,
                       color_piso=None, linea_cam=None):
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

        # 0a-bis-2. EL LADO DEL SIGUIENTE BLOQUE, mientras se termina
        #     este. La camara informa del blob mas grande; en cuanto el
        #     pilar actual queda al costado, ese blob ya es el SIGUIENTE
        #     -- y se estaba tirando. Se acepta solo si el rumbo de
        #     camara NO es el del pilar fichado, con la misma tolerancia
        #     de apareo que usa la captura: asi no se confunde el color
        #     del que se esta rodeando con el del que viene.
        self._observar_siguiente(color_cam, cx_cam, ahora)

        # 0a-ter. LAS BANDERAS POR PILAR SE BORRAN AL CAMBIAR DE PILAR.
        #     Aqui, contra la IDENTIDAD del objetivo, y no dentro de
        #     COMPROMISO: hay tres caminos de captura y una readquisicion
        #     por camara, y basta que uno no pase por el bloque de
        #     compromiso para que el pilar nuevo herede el "ya rebasado"
        #     del anterior.
        #
        #     Es lo que pasaba. Medido en la corrida 205339: SIETE de los
        #     diez objetivos nacian con paso_ok = 1, con el poste hasta
        #     1219 mm POR DELANTE y separaciones de -472 mm, o sea del
        #     lado prohibido. Consecuencias reales, no solo de registro:
        #     `_invariante_de_paso` cortocircuita en su primera linea
        #     cuando la bandera esta puesta, y `_est_retroceso` la
        #     consulta para decidir si al salir de una emergencia retoma
        #     la maniobra o la da por terminada -- con la bandera sucia
        #     abandona un pilar que nunca paso. Es el "falso rebasado"
        #     que se vio en video.
        if self.tracker.activo and self.tracker.id != self._id_objetivo:
            self._id_objetivo = self.tracker.id
            self._paso_validado = False
            self._ciclos_paso = 0
            self._reintentos = 0
            # La prevision se consume: el "siguiente" ya es el actual.
            self._siguiente_lado = 0
            self._t_siguiente = None
            self._mejor_sep = None
            self._ciclos_empeora = 0

        # 0a-bis. El invariante de paso es un HECHO GEOMETRICO, no un paso
        #     de la maniobra: si el pilar ya quedo detras y de su lado, esta
        #     rebasado, se este en el estado que se este. Comprobarlo solo
        #     dentro de PASO_LATERAL costaba maniobras enteras.
        #
        #     Medido en la corrida de las 16:01, pilar #3 ROJO: paso por su
        #     lado con 289 mm de holgura y cruzo a y = -61 mm... justo en un
        #     ciclo de RETROCESO. Como el invariante no se evaluaba ahi, no
        #     se declaro; y al retroceder el poste "vuelve" a quedar delante
        #     (y de -61 a +84), asi que el credito se perdio. 17,5 segundos
        #     y un ABORTO por un pilar que ya estaba superado.
        if (self.tracker.activo and not self._solo_obstaculo
                and self.estado in ESTADOS_MANIOBRA + ("RETROCESO", "REINTENTO")):
            self._invariante_de_paso(ahora)

        # 0b. Sentido de la vuelta. Se congela durante la maniobra: el yaw
        #     de una envolvente es de la maniobra, no de la pista, y
        #     meterlo aqui es la forma tipica de fijar un sentido falso.
        self.sentido.actualizar(heading, ahora, congelado=en_maniobra)
        # La geometria de pista se observa SIEMPRE, tambien durante la
        # maniobra: no depende del yaw, asi que rodear un pilar no la
        # contamina. Es justo al reves que el estimador por yaw, que hay
        # que congelar por eso mismo.
        # Las LINEAS del suelo van primero: son la unica evidencia
        # absoluta del sentido (no dependen del yaw ni de interpretar la
        # geometria). Si el sensor de piso esta puesto y calibrado, esto
        # zanja el sentido en la primera esquina.
        # LA CAMARA VE LAS LINEAS MUCHO MEJOR QUE EL SENSOR DE PISO. Se
        # fija el sentido con la que tenga mas cerca, una vez y para
        # siempre. Ver SentidoPorGeometria.fijar_por_camara.
        if linea_cam is not None and not self.pista.por_lineas:
            self.pista.fijar_por_camara(linea_cam[0])
        self._contar_lineas(linea_cam, ahora, heading)
        if color_piso is not None:
            self.pista.observar_linea(
                color_piso, ahora,
                retrocediendo=(self.estado == "RETROCESO" or self._ultima_vel < 0))
        self.pista.observar(med, ahora)
        self.pista.contradice(self.sentido.sentido)

        # 0c. Memoria de la ultima asimetria REAL entre paredes. Vive
        #     fuera de cualquier estado: es la pista con la que se
        #     desempata una esquina que, cuando por fin dispara la
        #     emergencia, ya se ve perfectamente simetrica.
        # 0d. Deteccion del final de muro, enclavamiento del lado de la
        #     esquina y memoria del ultimo lado visto
        self._actualizar_esquina(med)

        # 0e. ENCAJONADO -> DESEMPATE DIRECTO. Ver ENCAJONADO_MM: con las
        #     tres distancias por debajo de 200 mm no hay trayectoria que
        #     valga, y esperar a encadenar cuatro emergencias es esperar
        #     a que se acabe la ronda.
        if (med.frontal < ENCAJONADO_MM and med.izquierda < ENCAJONADO_MM
                and med.derecha < ENCAJONADO_MM):
            self._ciclos_encajonado += 1
        else:
            self._ciclos_encajonado = 0
        if (self._ciclos_encajonado >= CICLOS_ENCAJONADO
                and self.estado not in ("GIRO_FORZADO", "RETROCESO")):
            self._ciclos_encajonado = 0
            # EL SENTIDO DE LA VUELTA MANDA SI ES ABSOLUTO. Cuando lo
            # fijaron las lineas (camara), es la unica evidencia que no
            # se puede discutir: en una vuelta antihoraria TODAS las
            # esquinas giran a la izquierda. Dejar que
            # `_preferencia_esquina` pise eso produjo objetivos absurdos
            # -- "173 -> 90 grados", o sea girar a la DERECHA en una
            # vuelta antihoraria -- en la corrida 121145.
            self._signo_giro_forzado = (
                (float(self.pista.sentido) if self.pista.por_lineas else 0.0)
                or self._preferencia_esquina(med)
                or (float(self.pista.sentido) if self.pista.conocido else 0.0)
                or (1.0 if med.izquierda >= med.derecha else -1.0))
            self._heading_objetivo = None
            self._t_tramo_giro = None
            if self.estado in ESTADOS_MANIOBRA:
                self._estado_suspendido = self.estado
            self._entrar("GIRO_FORZADO", ahora)
            print("[ENCAJONADO] F:%.0f I:%.0f D:%.0fmm -> a rumbo con el cuadrante"
                  % (med.frontal, med.izquierda, med.derecha))

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
        meta = LINEAS_POR_VUELTA * VUELTAS_OBJETIVO
        por_lineas = self._lineas_cruzadas >= meta
        if (por_lineas or abs(heading) >= UMBRAL_VUELTAS) and self.estado == "CRUCERO":
            self.fase = "PARQUEO"
            self._t_parqueo = ahora
            if por_lineas:
                print("[!] %d lineas %s cruzadas = %d vueltas. Modo Parqueo."
                      % (self._lineas_cruzadas, self._color_contado or "?",
                         VUELTAS_OBJETIVO))
            else:
                print("[!] RED DE SEGURIDAD: %.0f grados de yaw con solo %d "
                      "lineas de %d. Modo Parqueo." % (heading,
                                                       self._lineas_cruzadas, meta))
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
            "REINTENTO":    self._est_reintento,
            "RETROCESO":    self._est_retroceso,
            "GIRO_FORZADO": self._est_giro_forzado,
        }[self.estado]
        velocidad, angulo = manejador(med, color_cam, heading, ahora)

        # 4. El limitador del servo y la publicacion del diagnostico se
        #    hacen en procesar(), a la salida de CUALQUIER fase: aqui se
        #    saltaban por el return del paso 2 y por la fase de parqueo.
        return (velocidad, angulo)

    # ==========================================
    # PERCEPCION DEL OBJETIVO
    # ==========================================
    def _candidatos_pilar(self, med):
        # (x, y, ancho_mm) de los clusters que pueden ser un poste. El
        # ancho es la puerta que rechaza esquinas de muro; sin el, la
        # asociacion del tracker se conforma con "esta cerca".
        #
        # SE ALIMENTA DE clusters_estrechos, NO de clusters_obstaculo.
        # Los dos filtros no miden lo mismo: `es_cluster_obstaculo` acota
        # el ARCO a 15 grados, y un poste de 50mm pasa de 15 grados a
        # partir de unos 250mm. O sea que con la lista de antes el pilar
        # desaparecia de los candidatos justo cuando se le estaba
        # rodeando, el tracker se quedaba 3s sin asociar y la maniobra
        # moria por "estimacion perdida" con sigma todavia baja (159-169
        # en las corridas del 10-09, muy por debajo de SIGMA_PERDIDO=300).
        # `es_objeto_estrecho` mide ancho FISICO en mm, que no depende de
        # la distancia, y por eso sigue reconociendo el poste de cerca.
        cands = []
        for clust in med.clusters_estrechos:
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

    def _en_mi_seccion(self, med, cx, cy, rumbo, bins_estrechos):
        """¿El poste esta antes de la pared que tiene detras, o pasada la curva?

        La comparacion se hace POR RUMBO, no contra el sector frontal.
        `frontal_muro` es el minimo de +-10 grados, y un poste a 30
        grados y 600 mm esta a 520 mm de avance -- puede estar
        perfectamente en esta seccion aunque el sector frontal vea la
        esquina a 237 mm porque el robot va encarado hacia ella. La
        version anterior comparaba esas dos cosas y dejaba de capturar
        postes propios: en la corrida de las 18:40, 6 de las 14 entradas
        en RETROCESO ocurrieron con `frontal_muro` entre 232 y 237.

        Lo que si es comparable es el poste contra la PARED QUE HAY EN SU
        MISMA DIRECCION, descontando los objetos estrechos para no medir
        el propio poste. Si la pared de ese rumbo esta mas cerca que el
        poste, el poste esta al otro lado de ella -- o sea pasada la
        curva -- y comprometerse con el es empezar la maniobra en el
        sitio equivocado. Es el "coge bien el primer rojo y al ver el
        segundo gira hacia el creyendo que lo tiene delante".
        """
        a = (rumbo - VENTANA_RUMBO_SECCION) % 360.0
        b = (rumbo + VENTANA_RUMBO_SECCION) % 360.0
        pared = distancia_en_rango_sin_bins(med.perfil, a, b, bins_estrechos)
        return math.hypot(cx, cy) <= pared + MARGEN_SECCION

    def _intentar_capturar(self, med, color_cam, heading, cx_cam, ahora):
        # Crea el objetivo cuando camara y LiDAR coinciden en un pilar
        # frontal. El apareo va por RUMBO: asi el color se pega al pilar
        # que la camara esta viendo, no al que casualmente esta mas cerca.
        if color_cam is None:
            return
        # NO SE CAPTURA LO QUE ESTA AL OTRO LADO DE LA CURVA. Un pilar mas
        # adelantado que la pared que tenemos enfrente no esta en nuestra
        # seccion: se ve "de frente" porque la esquina lo pone en linea,
        # pero para llegar a el hay que doblar primero. Comprometerse ahi
        # es empezar la maniobra en el sitio equivocado -- descrito en
        # pista como "coge bien el primer rojo y al ver el segundo gira
        # hacia el creyendo que lo tiene delante".
        #
        # El filtro va AQUI y no en CRUCERO: alli soltaba el objetivo y lo
        # volvia a capturar al ciclo siguiente, 19 veces seguidas sobre el
        # mismo poste, quemando ids y llenando el log.
        # (el filtro de seccion se aplica por RUMBO, ver _en_mi_seccion)
        bins_estrechos = bins_de_clusters(med.clusters_estrechos)
        s_lado = lado_obligatorio(color_cam)
        rumbo_cam = optica.rumbo_de_cx(cx_cam) if cx_cam is not None else None

        candidatos = []
        for clust in med.clusters_estrechos:
            cx, cy = centroide_xy_cluster(clust)
            r = math.degrees(math.atan2(cx, cy))
            if (cy > 80.0 and abs(r) <= SECTOR_BUSQUEDA_POSTE
                    and self._en_mi_seccion(med, cx, cy, r, bins_estrechos)):
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
        # OJO CON LOS DOS MARCOS: `rumbo_cam` puede venir de cualquier
        # punto del campo de la camara (+-34 grados) y `med.frontal` es
        # el minimo del SECTOR FRONTAL, que son +-10. Sin atar los dos,
        # un pilar visto a +30 grados se sembraba a la distancia de la
        # pared que hay de frente: un objetivo fantasma en un sitio donde
        # no hay nada. Y como `respaldado` acepta los sembrados sin
        # ninguna asociacion de LiDAR, ese fantasma llegaba a COMPROMISO
        # en tres ciclos. Solo se siembra si la camara apunta DENTRO del
        # sector que dio la distancia.
        _, sector_max = SECTOR_FRONTAL_SIEMBRA
        en_sector_frontal = (rumbo_cam is not None and abs(rumbo_cam) <= sector_max)
        estrecho_delante = med.frontal_muro > med.frontal * 1.5
        if (en_sector_frontal and estrecho_delante
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
    def _holgura_barrido(self, med, cmd, arco, bins_estrechos):
        """Sitio que le queda al cuerpo barrido EN SU PROPIO RUMBO, en mm.

        Sustituye a comparar `alcance_frontal(R, h)` contra
        `frontal_muro`. Aquello media el adelanto maximo del cuerpo -- que
        CRECE con el angulo, porque la esquina exterior se abre mientras
        avanza -- contra la distancia del sector frontal de +-10 grados.
        Dos direcciones distintas, el mismo error que el guardia de
        esquina tenia antes de `_en_mi_seccion`. El efecto medido con la
        pared a 122 mm era que sobrevivian cmd 0 y +2 y se rechazaba todo
        lo que pasara de 10 grados: el filtro empujaba a ir RECTO contra
        la pared justo cuando mas falta hacia girar.

        Ahora cada esquina pregunta por la pared de SU rumbo. Girando
        hacia un costado libre la holgura sale grande aunque el adelanto
        sea mayor, que es la verdad fisica; y si ese costado esta ocupado
        lo dice la distancia de ese rumbo, no la del frente.

        Negativa = el cuerpo se mete en lo que hay. Los objetos estrechos
        se descuentan igual que en `frontal_muro`: los pilares los lleva
        la FSM de evasion y `holgura_arco`, no este filtro de paredes.
        """
        peor = float("inf")
        for (x, y) in gev.esquinas_barridas(cmd, arco, geo.LIDAR_X):
            rango = math.hypot(x, y)
            rumbo = math.degrees(math.atan2(x, y)) % 360.0
            pared = distancia_en_rango_sin_bins(
                med.perfil,
                (rumbo - VENTANA_RUMBO_BARRIDO) % 360.0,
                (rumbo + VENTANA_RUMBO_BARRIDO) % 360.0,
                bins_estrechos)
            peor = min(peor, pared - rango)
        return peor

    def _comandos_seguros(self, med, velocidad, margen=MARGEN_PARED,
                          margen_frente=MARGEN_FRENTE):
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

        # HORIZONTE FRONTAL QUE SE ENCOGE CON EL HUECO. Antes era fijo, y
        # eso ponia un suelo por debajo del cual NINGUN comando pasaba,
        # girase como girase: alcance_frontal(R, h) >= h + X_MORRO para
        # cualquier radio, asi que con h fijo en 180 hacia falta
        # frontal_muro >= 180 + 162 - 128 + 40 = 254 mm por construccion.
        # Medido en las corridas del 09-09: 82% de los ciclos sin ningun
        # comando admisible, y 405 de las 818 rachas con el frente por
        # debajo de 260 mm.
        #
        # El horizonte frontal es una distancia de REACCION, y cuando
        # queda menos sitio del que se pensaba reaccionar, lo honesto es
        # reaccionar antes -- no declarar que no hay salida. Encogerlo
        # deja siempre vivo el comando que mas dobla, que es justo el que
        # hace falta ahi. El piso absoluto evita que se anule del todo.
        if exige_giro and RELAJAR_FRENTE:
            h_frente = max(HORIZONTE_FRENTE_PISO,
                           min(h_frente, alcance - gev.X_MORRO))

        deriva = _clamp(med.angulo_muro, MAX_DERIVA_MURO) if med.muro_valido else 0.0

        # MARGEN QUE NO SE PUEDE VIOLAR HACIA ATRAS. Si el robot ya esta
        # mas cerca de la pared que el margen, exigirle el margen es
        # pedirle algo imposible y vaciar el conjunto: la restriccion
        # correcta ahi es "no acercarte mas", no "estate a 110". Medido:
        # 791 de las 818 rachas de SIN_SALIDA del 09-09 tenian un lado
        # por debajo de 115 mm, con mediana de 86.
        if RELAJAR_LATERAL:
            margen_der = min(margen, max(0.0, med.derecha - 1.0))
            margen_izq = min(margen, max(0.0, med.izquierda - 1.0))
        else:
            margen_der = margen_izq = margen

        bins_estrechos = bins_de_clusters(med.clusters_estrechos)

        seguros = []
        for cmd in CANDIDATOS:
            ok = True
            for frac in (0.5, 1.0):
                dx = gev.lateral_predicho(cmd, horizonte * frac, deriva)
                if med.derecha - dx < margen_der:
                    ok = False
                    break
                if med.izquierda + dx < margen_izq:
                    ok = False
                    break
            if ok and exige_giro:
                # HORIZONTE FINITO Y PROPIO, no circunferencia completa
                # y no el horizonte lateral: exigir que quepa el arco
                # entero dejaba el conjunto admisible VACIO en el 30% de
                # los ciclos de la corrida 195623. Quien hace empezar la
                # curva a tiempo es el termino de esquina de
                # _rumbo_nominal, que si usa el criterio completo.
                #
                # Y la comparacion va POR RUMBO (ver _holgura_barrido).
                # Antes era `alcance_frontal(R, h) > frontal_muro`, que
                # mide el adelanto del cuerpo contra la distancia del
                # sector de +-10 grados: al girar, la esquina exterior se
                # abre HACIA UN COSTADO mientras avanza, asi que ese
                # numero crece con el angulo y el filtro acababa dejando
                # vivo solo el comando recto. Cuatro de cuatro entradas
                # en RETROCESO de la corrida 192239 fueron eso: seis
                # ciclos con el volante en 0.0 y el frente cayendo de 157
                # a 122 mm, con el rumbo deseado pidiendo 12 grados.
                if self._holgura_barrido(med, cmd, h_frente,
                                         bins_estrechos) < margen_frente:
                    ok = False
            if ok:
                seguros.append(cmd)
        return seguros, horizonte

    def _retroceso_ya_fallo(self):
        """True cuando retroceder ya se probo y NO sirvio.

        Pedia solo "una emergencia en los ultimos 10 s", y en cuanto las
        emergencias se encadenan eso es siempre: en la corrida de las
        16:44 el ultimo recurso estuvo activo el 32% de los ciclos, que
        no es un ultimo recurso sino el modo normal. Ahora hacen falta
        DOS retrocesos seguidos en la misma ventana -- o sea, retroceder
        una vez, volver al mismo sitio, y retroceder otra.
        """
        ahora = self._t_ultimo_ciclo
        if ahora is None or self._t_ultima_emergencia is None:
            return False
        # REVERTIDO a 1. Se subio a 2 y la corrida siguiente lo
        # falsifico: los pilares completados cayeron de 4 a 1 y
        # SIN_SALIDA subio del 3,8% al 12,8%. Lo que antes era "avanzar
        # con el mejor comando posible" paso a ser "retroceder", y el
        # robot dejo de progresar. El problema del ultimo recurso no era
        # que se usara mucho: era que no miraba el pilar, y eso ya esta
        # arreglado.
        if self._racha_retroceso < RETROCESOS_PARA_ULTIMO_RECURSO:
            return False
        return (ahora - self._t_ultima_emergencia) <= VENTANA_ATASCO

    def _mejor_esfuerzo(self, med, velocidad, pilar=None, holgura=0.0):
        """El comando menos malo cuando ninguno es admisible.

        No relaja la seguridad: la emergencia anti-choque
        (EMERGENCIA_FRONTAL/LATERAL) sigue por delante en el paso 1 del
        ciclo y se lleva los casos de verdad criticos. Esto cubre solo la
        banda intermedia -- pared entre el umbral de emergencia y el
        margen de la envolvente -- donde la version anterior contestaba
        marcha atras y volvia a entrar en el mismo sitio un ciclo
        despues.

        Criterio: maximizar la holgura MINIMA prevista dentro de un ciclo
        de control, contando los dos costados y el morro. Es la misma
        magnitud que la restriccion, solo que como objetivo continuo en
        vez de como puerta de si/no.
        """
        horizonte = max(HORIZONTE_MIN, _vel_mm_s(velocidad) * HORIZONTE_SEG)
        deriva = _clamp(med.angulo_muro, MAX_DERIVA_MURO) if med.muro_valido else 0.0
        bins_estrechos = bins_de_clusters(med.clusters_estrechos)

        mejor, mejor_puntos = None, -1e9
        for cmd in CANDIDATOS:
            dx = gev.lateral_predicho(cmd, horizonte, deriva)
            lateral = min(med.derecha - dx, med.izquierda + dx)
            # MISMA MEDIDA QUE LA RESTRICCION, por rumbo. Con
            # `alcance_frontal` el ultimo recurso heredaba el sesgo a ir
            # recto y elegia justo el comando que no sale de la pared.
            # SIN restar MARGEN_FRENTE: aqui la pregunta es "¿toca?",
            # no "¿le queda el colchon comodo?". Restandolo, el ultimo
            # recurso devolvia None -- o sea marcha atras -- en cuanto la
            # holgura bajaba de 40 mm, con el cuerpo todavia a 34 mm de
            # la pared y sitio de sobra para salir girando.
            frontal = self._holgura_barrido(med, cmd, HORIZONTE_FRENTE_PISO,
                                            bins_estrechos)
            puntos = min(lateral, frontal)
            # EL PILAR TAMBIEN CUENTA. Sin esto, el ultimo recurso elegia
            # el rumbo como si el poste no existiera, y en la corrida de
            # las 16:44 eso fue el 32% de los ciclos: el robot se le
            # echaba encima, saltaba la emergencia a ~115 mm, cuatro
            # seguidas, y el GIRO_FORZADO acababa dandole la vuelta. Es
            # el "tenia sitio de sobra para pasar y no paso".
            if pilar is not None:
                puntos = min(puntos, gev.holgura_arco(pilar[0], pilar[1], cmd))
            if puntos > mejor_puntos:
                mejor, mejor_puntos = cmd, puntos
        # Si ni el mejor deja el morro fuera de la pared, no hay comando
        # de avance que valga y SI toca retroceder de verdad.
        return mejor if mejor_puntos > 0.0 else None

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
        """Hacia donde dobla la pista, con la MEDIDA por delante.

        El orden estaba invertido y eso creaba un bucle que se
        autoconfirma: `sentido_vuelta` deduce el sentido del yaw del
        PROPIO robot, asi que si el robot gira a un lado por cualquier
        motivo, el estimador lo declara "el sentido de la vuelta" y
        entonces el termino de esquina lo manda girar mas hacia ese lado.
        Como `LADO_POR_DEFECTO` es izquierda, la primera esquina sin
        evidencia arranca el bucle y ya no se sale de el.
        
        Medido en las cinco corridas del 10-09: `sentido` = ANTIHORARIO
        en el 76-93% de los ciclos y una relacion de giro izquierda/derecha
        de 4:1 a 6:1 en TODAS, con el angulo medio siempre positivo
        (+8.8 a +11.7 grados) -- incluso con la pista montada en sentido
        horario. El robot no estaba midiendo el sentido: lo estaba
        imponiendo.

        Ahora manda lo que se MIDE en este barrido (que muro se acaba, y
        las diagonales frontales), y `sentido_vuelta` solo desempata
        cuando no hay medida. Es la misma jerarquia que ya usa el resto
        del sistema: dato de sensor por encima de estado interno.
        """
        lado = self._decidir_lado_esquina_medido(med)
        if lado is not None:
            self.fuente_lado = "MEDIDO"
            return lado
        if self.pista.conocido:
            self.fuente_lado = "PISTA"
            return float(self.pista.sentido)
        if self._esquina_latch:
            self.fuente_lado = "LATCH"
            return self._signo_esquina
        # NO HAY VALOR POR DEFECTO, y es deliberado. Antes se devolvia
        # LADO_POR_DEFECTO (izquierda) y eso arrancaba el bucle: girar a
        # la izquierda sin evidencia genera yaw positivo, el estimador por
        # yaw declara ANTIHORARIA, y a partir de ahi toda esquina se toma
        # a la izquierda aunque la pista vaya al reves. Devolver None
        # apaga el termino de esquina: el robot se centra entre paredes y
        # frena contra la pared frontal, que es seguro -- y es justo la
        # situacion en la que la evidencia buena (el muro interior que se
        # acaba) aparece sola.
        self.fuente_lado = "SIN_DATO"
        return None

    def _decidir_lado_esquina_medido(self, med):
        # Solo evidencia de ESTE barrido, sin memoria ni yaw propio:
        #   1. un sector lateral muy por encima de la anchura del carril
        #      es el final del muro interior, y por ahi dobla la pista;
        #   2. si no, las diagonales frontales, cuando difieren claro.
        abre_izq = med.izquierda > UMBRAL_APERTURA_ESQUINA
        abre_der = med.derecha > UMBRAL_APERTURA_ESQUINA
        if abre_izq != abre_der:
            return 1.0 if abre_izq else -1.0
        d_izq = distancia_en_rango(med.perfil, 300, 350)
        d_der = distancia_en_rango(med.perfil, 10, 60)
        # Razon y no diferencia: la diferencia absoluta depende de lo
        # lejos que este la esquina. Medido en pista encarando una curva a
        # 686 mm, la razon buena era 812/669 = 1.21 y la diferencia solo
        # 143 mm -- por debajo del umbral absoluto de 200 que habia aqui,
        # que es como esta medida acabo cayendo al valor por defecto y
        # contestando justo al reves.
        mayor, menor = max(d_izq, d_der), min(d_izq, d_der)
        if menor > 1.0 and mayor / menor >= RAZON_DIAGONAL_ESQUINA:
            return 1.0 if d_izq > d_der else -1.0
        return None


    def _compatibles(self, seguros, pilar, holgura, criterio):
        # `criterio` separa las dos preguntas que antes iban juntas:
        #   "lado"   -> ¿el pilar queda de su lado reglamentario?
        #   "choque" -> solo ¿lo toco? (durante APERTURA la regla todavia
        #               no se puede cumplir, pero chocar sigue prohibido)
        x_r, y_r = pilar
        # LA REGLA SOLO MANDA MIENTRAS EL PILAR ESTA DELANTE.
        #
        # `preserva_lado` pregunta si el ARCO comandado deja el poste de su
        # lado. Con el poste ya al costado esa pregunta deja de tener
        # respuesta conduciendo recto: la unica forma de que un arco siga
        # "dejandolo a la derecha" es curvar hacia el otro lado, y hacerlo
        # cada ciclo es curvar para siempre.
        #
        # Es lo que se veia en pista, descrito por el equipo como "en vez
        # de rodearlo por la izquierda, el robot solo gira a la izquierda
        # pensando que lo tiene delante". Medido en la corrida de las
        # 10:09: PASO_LATERAL con un verde mantuvo +12.2 grados de media
        # durante 144 ciclos seguidos -- 14 segundos girando al mismo
        # lado. Con un pilar en esquina eso destruye el sentido de la
        # vuelta, que es justo donde fallaba.
        #
        # Cuando el poste queda por detras del travez, lo unico que sigue
        # importando es no tocarlo. El lado por el que se paso ya esta
        # decidido: lo mide `_invariante_de_paso` con el barrido en marco
        # mundo, que no se puede falsificar girando el chasis.
        if criterio == "lado" and y_r > Y_PILAR_MANDA_LA_REGLA:
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
        self.seguridad = "LIBRE"

        # ESCALERA DE CONCESIONES, antes de dar la situacion por perdida.
        # Antes solo existia el primer peldaño, y su fallo era declarar
        # SIN_SALIDA -- o sea marcha atras -- en el 82% de los ciclos de
        # la corrida del 09-09. Retroceder es una accion valida cuando de
        # verdad no cabe nada; usarla como respuesta por defecto a "el
        # margen comodo no cabe" es lo que produjo el ciclo limite.
        if not seguros:
            seguros, _ = self._comandos_seguros(
                med, velocidad, margen=MARGEN_PARED_DURO,
                margen_frente=MARGEN_FRENTE_DURO)
            if seguros:
                self.seguridad = "APURA"
        if not seguros and self._retroceso_ya_fallo():
            # Ultimo peldaño, y SOLO si retroceder ya se intento y no
            # sirvio. El orden importa: la marcha atras es la accion
            # limpia cuando no cabe nada, y con la trasera arreglada
            # (ver lidar_mascara + el ultrasonido en ronda_camara) vuelve
            # a funcionar de verdad. Medido en simulacion, ofrecer este
            # peldaño SIEMPRE -- con el retroceso ya sano -- cambia
            # SIN_SALIDA por choques: 0 -> 7 contra muro y un roce de
            # -8 mm contra un pilar en dos de los seis escenarios.
            # Aqui solo cubre el caso que motivo el arreglo: el robot ya
            # retrocedio, volvio al mismo sitio, y repetirlo es el bucle.
            mejor = self._mejor_esfuerzo(med, velocidad, pilar, holgura)
            if mejor is not None:
                self.seguridad = "ULTIMO_RECURSO"
                seguros = [mejor]

        self.n_seguros = len(seguros)
        if not seguros:
            self.seguridad = "SIN_SALIDA"
            return (0.0, False)

        candidatos = seguros

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
                apurados, _ = self._comandos_seguros(
                    med, velocidad, margen=MARGEN_PARED_DURO,
                    margen_frente=MARGEN_FRENTE_DURO)
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

    def _rumbo_a_punto_de_paso(self, x_r, y_r, s_lado, holgura):
        """Persecucion pura hacia el punto por el que hay que pasar.

        El punto es el pilar desplazado LATERALMENTE hacia el lado por el
        que el robot tiene que pasar: si el poste debe quedar a la derecha
        (s_lado=+1), se apunta a un punto a su izquierda. La separacion
        pedida es la minima geometrica mas la holgura exigida, que ya
        crece con la incertidumbre de la estimacion.
        """
        objetivo_x = x_r - s_lado * (gev.SEMIANCHO + gev.RADIO_POSTE + holgura)
        # EL PUNTO VA POR DELANTE, a una distancia de anticipacion, no
        # pegado al poste. Poniendolo a la altura del pilar (`y_r`) la
        # persecucion se apaga justo cuando hace falta: en cuanto el poste
        # se acerca, `y_r` baja del minimo y la funcion se rendia. Medido
        # en la corrida de las 19:0x: la persecucion solo actuo en 195 de
        # los 734 ciclos de PASO_LATERAL, el 27%, y siempre con el poste
        # todavia lejos -- que es cuando el centrado ya bastaba.
        #
        # Con el suelo de anticipacion, el punto se mantiene delante y al
        # costado del poste, que es la trayectoria de paso: el robot sigue
        # conduciendo hacia el hueco hasta haberlo cruzado.
        objetivo_y = max(y_r, ADELANTO_MINIMO_PERSECUCION)
        # Curvatura de persecucion pura desde el eje trasero:
        # k = 2x / L^2, con (x, y) el objetivo en ese marco.
        dist2 = objetivo_x * objetivo_x + objetivo_y * objetivo_y
        if dist2 < 1.0:
            return None
        curv = 2.0 * objetivo_x / dist2
        if abs(curv) < 1e-6:
            return 0.0
        radio = abs(1.0 / curv)
        return gev.comando_de_radio(radio, hacia_izquierda=(objetivo_x < 0.0))

    def _observar_siguiente(self, color_cam, cx_cam, ahora):
        """Anota el lado obligatorio del pilar que viene DESPUES de este.

        No es una captura: no hay segundo hueco en el tracker y no se
        estima su posicion. Lo unico que se guarda es el LADO, que es lo
        que hace falta para salir del bloque actual por el carril
        correcto en vez de volver al centro. Ver SESGO_SIGUIENTE_MM.

        Solo se mira cuando el pilar actual ya esta rebasado: antes, el
        color de la camara es suyo. Y aun asi se exige que el rumbo de
        camara SE SEPARE del rumbo del pilar fichado mas que la
        tolerancia de apareo, que es el mismo criterio con el que
        `_intentar_capturar` decide a quien pertenece un color.
        """
        if color_cam is None or cx_cam is None:
            return
        trk = self.tracker
        if trk.activo and not (self._paso_validado or self._solo_obstaculo):
            return
        rumbo_cam = optica.rumbo_de_cx(cx_cam)
        if trk.activo:
            x_r, y_r = trk.xy_eje()
            xl, yl = geo.eje_trasero_a_lidar(x_r, y_r)
            if abs(rumbo_cam - optica.rumbo_camara_de_cluster(xl, yl))                     <= TOLERANCIA_APAREO_GRADOS:
                return          # ese color es del pilar que ya llevamos
        lado = lado_obligatorio(color_cam)
        if lado == 0:
            return
        if lado != self._siguiente_lado:
            print("[SIGUIENTE] %s a la vista: el carril se desplaza hacia la %s"
                  % (color_cam, "IZQUIERDA" if lado > 0 else "DERECHA"))
        self._siguiente_lado = lado
        self._t_siguiente = ahora

    def _sesgo_de_carril(self):
        """Desplazamiento del centro de carril, en mm, hacia donde hace
        falta para el SIGUIENTE bloque. Positivo = hacia la izquierda.

        El pilar debe quedar del lado `s_lado`, asi que el robot tiene que
        pasar por el lado CONTRARIO: verde (s_lado +1, pilar a la
        derecha) -> el robot por la izquierda; rojo -> por la derecha.
        Ese es justo el signo de `s_lado`.
        """
        if not self._siguiente_lado or self._t_siguiente is None:
            return 0.0
        ahora = self._t_ultimo_ciclo
        if ahora is not None and (ahora - self._t_siguiente) > VIDA_SIGUIENTE:
            self._siguiente_lado = 0
            return 0.0
        return self._siguiente_lado * SESGO_SIGUIENTE_MM

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
        # El centro de carril se desplaza hacia el lado que pedira el
        # SIGUIENTE bloque (0.0 si no hay ninguno a la vista). Va aqui,
        # en la TRAYECTORIA, y no en el arbitraje: es una preferencia,
        # asi que las paredes y el pilar siguen recortandola igual.
        desvio = self._sesgo_de_carril()
        if max(med.izquierda, med.derecha) > 1.5 * geo.ANCHO_CARRIL:
            objetivo = geo.ANCHO_CARRIL / 2.0
            # Mismo signo que la formula de dos paredes: la pared que
            # falta se sustituye por la distancia nominal.
            if med.izquierda < med.derecha:
                ang = (med.izquierda - objetivo + desvio) * KP_LATERAL
            else:
                ang = (objetivo - med.derecha + desvio) * KP_LATERAL
        else:
            ang = (med.izquierda - med.derecha + 2.0 * desvio) * KP_LATERAL
        ang = _clamp(ang, MAX_APORTE_POSICION)
        if med.frontal_muro < DIST_ASISTENCIA_ESQUINA and med.muro_valido:
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

        # 0. EL PILAR YA ESTA DETRAS. Es el criterio mas fuerte de todos y
        #    faltaba: si el poste quedo por detras del eje trasero y del
        #    lado que le tocaba, la maniobra esta hecha, se cumpla o no el
        #    resto del invariante.
        #
        #    Sin esta clausula el sistema se quedaba "rodeando" un pilar ya
        #    rebasado hasta morir de hambre. Medido en la corrida de las
        #    10:23: el clustering descarta el sector 120-240 grados (la
        #    trasera), asi que en cuanto el pilar cruza los 120 grados de
        #    rumbo el tracker deja de tener candidatos -- 74 ciclos con
        #    `sin_candidatos`, TODOS con el pilar entre 121 y 170 grados y
        #    con y negativa (hasta -718mm). A los 3 segundos saltaba
        #    "estimacion perdida" con sigma 162-170, o sea starvation pura,
        #    y la maniobra terminaba en ABORTO con su refractario -- una
        #    racha llego a 228 ciclos, 27 segundos persiguiendo un fantasma.
        #    Los tres pilares verdes de esa corrida murieron asi.
        x_r, y_r = trk.xy_eje()
        sep = trk.s_lado * x_r
        if (y_r < 0.0 and sep >= SEPARACION_MIN_VALIDA
                and trk.sigma <= SIGMA_MAX_PARA_PASO):
            self._paso_validado = True
            print("[PASO] pilar #%d rebasado: quedo detras del eje (y=%.0f) y a "
                  "%.0fmm por su lado" % (trk.id, y_r, sep))
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
    def _poste_sin_color(self, med, color_cam):
        """Distancia al poste mas cercano que el LiDAR ve y la camara no
        ha sabido colorear. None si no hay ninguno.

        Solo mira clusters ESTRECHOS y en mi seccion, con los mismos dos
        filtros que usa la captura: sin ellos entraria la quilla de una
        pared o un poste del otro lado de la curva.
        """
        if color_cam is not None or self.tracker.activo:
            return None
        bins = bins_de_clusters(med.clusters_estrechos)
        mejor, d_mejor = None, None
        for clust in med.clusters_estrechos:
            cx, cy = centroide_xy_cluster(clust)
            if cy <= 80.0:
                continue
            r = math.degrees(math.atan2(cx, cy))
            if abs(r) > SECTOR_BUSQUEDA_POSTE:
                continue
            d = math.hypot(cx, cy)
            if not (DIST_ENCARE_MIN < d < DIST_ENCARE_MAX):
                continue
            if not self._en_mi_seccion(med, cx, cy, r, bins):
                continue
            if d_mejor is None or d < d_mejor:
                mejor, d_mejor = (cx, cy), d
        return mejor

    def _rumbo_al_poste(self, cx, cy):
        """Comando que apunta el morro al poste (persecucion pura).

        Se construye con `gev.comando_de_radio`, que ya sabe de que lado
        cae cada signo y recorta al tope fisico -- no se inventa aqui un
        convenio de signos, que es donde se cuelan los errores.
        """
        x_r, y_r = geo.lidar_a_eje_trasero(cx, cy)
        d2 = x_r * x_r + y_r * y_r
        if abs(x_r) < 1.0 or d2 < 1.0:
            return 0.0
        radio = d2 / (2.0 * abs(x_r))
        return gev.comando_de_radio(radio, x_r < 0.0)

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
        # Poste a la vista del LiDAR y sin color: se llega mas despacio
        # para darle a la camara mas fotogramas y mas pista por delante.
        # Ver VELOCIDAD_LECTURA.
        # DOS INTENTOS DE AYUDAR A LA CAMARA, LOS DOS REFUTADOS EN PISTA.
        # No repetirlos sin leer esto.
        #
        # El problema real: el LiDAR ve el poste hasta 1400 mm y la
        # camara solo le pone color desde ~1100 hacia dentro. Sin color no
        # hay lado obligatorio, el poste no se ficha, y se pasa por donde
        # salga -- que en competencia termina el recorrido.
        #
        # 1. FRENAR al verlo sin color, para dar mas fotogramas por
        #    milimetro. Medido: el alcance del color NO mejoro (1128 ->
        #    1056 mm) y la corrida cayo de 8 pasos y 0 infracciones a 3 y
        #    2, con cinco emergencias mas.
        #
        # 2. ENCARARLO, para poner el blob en el centro del frame en vez
        #    de en el borde del cono. Peor todavia: el color bajo del
        #    39,2% al 26,8% de los ciclos, el alcance de 1128 a 1025, y la
        #    corrida a 4 pasos con 2 infracciones. Ademas disparaba en el
        #    33% de los ciclos, o sea que secuestraba la trayectoria: el
        #    robot se salia del carril a mirar postes, se metia en sitios
        #    estrechos y -- girando tanto -- CORROMPIA EL ESTIMADOR DE
        #    SENTIDO ("el yaw dice ANTIHORARIA y la pista dice HORARIA").
        #
        # Conclusion: el corte a ~1100 mm no depende de como se acerque el
        # robot. Esta en la cadena HSV, y se arregla mirando la mascara,
        # no cambiando el control. `_poste_sin_color` y `_rumbo_al_poste`
        # se conservan por si sirven cuando el color llegue mas lejos.
        # REFUTADO Y QUITADO: frenar al ver un poste sin color.
        #
        # La idea era que llegando mas despacio la camara tendria mas
        # fotogramas por milimetro y lo leeria antes. Medido el 11-09 con
        # la misma pista: el alcance del color NO mejoro (1128 -> 1056 mm,
        # incluso peor), la distancia de captura apenas se movio (223 ->
        # 271 mm) y la corrida empeoro de 8 pasos y 0 infracciones a 3 y
        # 2, con cinco emergencias mas. `_poste_sin_color` se conserva
        # por si sirve para otra cosa, pero el frenado se quita.
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
        self._reintentos = 0
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
        _x_r, y_r = trk.xy_eje()
        if trk.perdido(ahora):
            return "estimacion perdida (sigma %.0fmm)" % trk.sigma
        if not self._hay_progreso(ahora):
            return "sin progreso geometrico"
        if trk.distancia() > DIST_EXCURSION_MAX:
            return "excursion de %.0fmm: ya no cabe la envolvente" % trk.distancia()
        if self._t_commit is not None and (ahora - self._t_commit) > TIMEOUT_MANIOBRA:
            return "maniobra demasiado larga"
        # LA SEPARACION DEJO DE MEJORAR. Con el pilar todavia delante y
        # la regla sin cumplir, que la mejor separacion conseguida se
        # pierda significa que el intento hacia adelante se quedo sin
        # sitio -- tipicamente una pared frontal que vacia el conjunto
        # compatible justo cuando quedaba poco para cruzar. Seguir
        # empujando desde ahi solo lleva al lado prohibido. Ver
        # MARGEN_SEP_EMPEORA.
        #
        # Se mide contra el MEJOR valor registrado, no contra el ciclo
        # anterior: la separacion oscila con el barrido del LiDAR y una
        # comparacion ciclo a ciclo disparia con ruido.
        if y_r > 0.0 and not self._paso_validado:
            sep = trk.s_lado * _x_r
            if self._mejor_sep is None or sep > self._mejor_sep:
                self._mejor_sep = sep
                self._ciclos_empeora = 0
            elif sep < self._mejor_sep - MARGEN_SEP_EMPEORA:
                self._ciclos_empeora += 1
                if (self._ciclos_empeora >= CICLOS_SEP_EMPEORA
                        and sep < SEPARACION_MIN_VALIDA):
                    return ("la separacion se pierde (%.0f -> %.0fmm)"
                            % (self._mejor_sep, sep))
            else:
                self._ciclos_empeora = 0

        # LA SUSPENSION SOLO CUENTA MIENTRAS LA REGLA MANDA DE VERDAD.
        # SUSPENDE significa "las paredes y el pilar piden cosas
        # incompatibles", y eso es motivo de aborto mientras el lado
        # todavia se pueda perder. Pero por detras de
        # Y_PILAR_MANDA_LA_REGLA `_compatibles` YA NO exige el lado --
        # solo holgura -- asi que contar esos ciclos abortaba por algo
        # que nadie estaba pidiendo, y con un motivo falso.
        #
        # Es lo que mato a cuatro de los cinco rojos de la corrida
        # 194413. El #2 estaba al costado, a 154-187 mm por su lado
        # obligatorio, encajado entre el pilar y la esquina de la isla:
        # la holgura comoda (55 mm) no cabia, medida 50-58. Tres ciclos
        # de SUSPENDE y ABORTO, medio segundo antes de que el invariante
        # lo declarara rebasado. Abandonar ahi no salva nada -- el lado
        # ya esta decidido -- y cuesta el refractario, que crece con cada
        # aborto encadenado (1,2 -> 2,4 -> 3,6 s).
        #
        # Sin holgura comoda no se queda a ciegas: _arbitrar ya elige en
        # SUSPENDE el comando que MENOS invade al pilar, y la emergencia
        # anti-choque sigue por delante.
        if self.seguridad == "SUSPENDE" and y_r > Y_PILAR_MANDA_LA_REGLA:
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

        # CONDUCIR HACIA EL PUNTO POR DONDE HAY QUE PASAR, no centrarse
        # en el carril y dejar que el pilar recorte.
        #
        # Aqui la consigna deseada era `_rumbo_nominal`, o sea el centrado
        # entre paredes, con el pilar entrando solo como filtro. Eso
        # significa que NADIE conducia hacia el hueco legal: el control
        # pedia el centro del carril y la regla del pilar tiraba del
        # volante al otro lado, y la maniobra duraba hasta que un
        # invariante geometrico se daba por satisfecho. Medido: 36% de los
        # ciclos en este estado y 7,3 s por pilar.
        #
        # El robot de referencia que trajo el equipo hace lo contrario y
        # no se para nunca: apunta a un punto al costado del bloque y
        # conduce hacia el. Es lo que se hace aqui -- persecucion pura
        # hacia el punto de paso -- dejando la envolvente como filtro,
        # que es donde tiene que estar.
        # PROBADO Y REVERTIDO el 10-09 (corrida 19:2x). La idea venia del
        # video de referencia que trajo el equipo: apuntar a un punto al
        # costado del bloque y conducir hacia el, en vez de centrarse en
        # el carril y dejar que el pilar recorte.
        #
        # Con la anticipacion bien puesta la persecucion paso a mandar de
        # verdad -- de 195 a 798 ciclos -- y el resultado EMPEORO: el
        # tiempo por pilar subio de 7,2 a 13,0 s, los pilares completados
        # bajaron de 7 a 4 y RETROCESO subio del 10,2% al 15,0%.
        #
        # Por que, en una frase: perseguir un punto FIJO al costado del
        # poste ignora las paredes, asi que el arbitraje tiene que
        # recortar la consigna casi siempre, y una consigna que se recorta
        # siempre es peor que una que ya nace dentro de lo posible. El
        # centrado de carril, aunque parezca que "no conduce hacia el
        # hueco", nace compatible con el pasillo y el pilar solo lo
        # desvia lo justo.
        #
        # Lo que SI valdria la pena probar de la idea: que el punto de
        # paso no sea fijo al costado del poste, sino la interseccion
        # entre ese costado y el carril -- o sea, respetar las dos cosas
        # a la vez en vez de elegir una. `_rumbo_a_punto_de_paso` se
        # conserva para eso.
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
            # SOLTAR NO PUEDE SIGNIFICAR "DEJAR DE VERLO". Si el pilar
            # sigue DELANTE del eje trasero, la maniobra habra terminado
            # como intencion pero el poste sigue fisicamente al costado,
            # y este estado sale precisamente ENDEREZANDO hacia el
            # carril. Soltarlo ahi lo borra de la arbitracion -- deja de
            # ser `pilar=` en `_nominal_arbitrado` -- y el robot endereza
            # contra el.
            #
            # Medido en la corrida 210600: el objetivo #3 se solto por
            # "tangente recuperada" con y_eje = 0 exacto, o sea con el
            # poste justo a la altura del eje y a 212 mm de costado; el
            # #5 con +24 y el #7 con +14. En la 205339, tres mas, uno de
            # ellos con el poste a 369 mm POR DELANTE. Es el fallo que se
            # vio en video en el tercer bloque.
            #
            # El propio archivo ya lo tenia escrito en DIST_PILAR_LIBRE
            # ("por debajo, la cola todavia lo puede barrer al
            # enderezar"); lo que faltaba era que el camino de soltado lo
            # respetase. Ahora degrada a obstaculo pasivo -- ya no manda
            # el LADO, que eso quedo decidido, pero sigue contando para
            # la holgura -- y `_obstaculo_pasivo` lo suelta de verdad
            # cuando queda por detras de la culata.
            x_r, y_r = trk.xy_eje()
            if y_r >= 0.0 and trk.distancia() < DIST_PILAR_LIBRE:
                self._solo_obstaculo = True
                self._t_obstaculo = ahora
                print("[OBJETIVO] #%d pasa a obstaculo: maniobra completada "
                      "(%s) pero sigue al costado (y=%.0f, %.0fmm)"
                      % (trk.id, razon, y_r, trk.s_lado * x_r))
            else:
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
        """Cierre de una maniobra que no se puede completar.

        Se abandona la REGLA (el lado obligatorio ya no se persigue) pero
        NO la estimacion: el pilar sigue ahi y sigue siendo algo con lo
        que se puede chocar. Se conserva como obstaculo hasta que quede
        atras o la estimacion muera. Sin esto, abortar delante de un
        pilar equivale a dejar de verlo, y el robot lo raspa.

        ESTO ES EL ULTIMO RECURSO, NO LA SALIDA NORMAL. Abandonar la
        regla con el pilar todavia delante significa pasarlo por donde
        salga, y en competicion pasar por el lado prohibido TERMINA el
        recorrido. Antes de llegar aqui se intenta REINTENTO, que es la
        misma situacion resuelta hacia atras.
        """
        reintento = self._quiere_reintentar(med, ahora, motivo)
        if reintento is not None:
            return reintento
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

    def _quiere_reintentar(self, med, ahora, motivo):
        """¿Se puede resolver echandose atras en vez de rindiendose?

        Condiciones, todas necesarias:
          - hay estimacion viva del pilar (si no, no hay a que volver);
          - el pilar sigue DELANTE del eje trasero: por detras ya no hay
            nada que reintentar, el lado quedo decidido;
          - todavia NO cumple la separacion del lado obligatorio, que es
            justo lo que haria perder el recorrido;
          - queda sitio detras (el ultrasonido, que es la unica medida
            trasera fiable -- ver lidar_mascara);
          - no se ha agotado el cupo de reintentos de ESTE pilar.

        El cupo existe para que el reintento no sea un bucle: si dos
        marchas atras no han abierto el paso, el problema no es el sitio
        y se aborta de verdad.
        """
        trk = self.tracker
        if not trk.activo or self._solo_obstaculo:
            return None
        # LA ESTIMACION PERDIDA NO ES UNA RAZON PARA NO REINTENTAR: es
        # una razon MAS. Antes se exigia `not trk.perdido(ahora)` y eso
        # dejaba fuera justo el caso que mas se repite. Corrida 214747:
        # el mismo pilar fisico, dos vueltas seguidas -- en la primera
        # (#3) llego a REINTENTO y salio bien, de -98 a +188 mm; en la
        # segunda (#6) murio con sigma 209 y se abandono sin intentarlo.
        #
        # Retroceder es precisamente lo que recupera la estimacion: aleja
        # el poste del sector ciego del mastil (134-200 grados), lo
        # devuelve al cono de la camara y le da al tracker ciclos con
        # candidatos. Quedarse quieto no recupera nada y el paso por el
        # lado prohibido es seguro.
        #
        # Con la estimacion muerta no se conduce POR ella: ver
        # _est_reintento, que en ese caso solo endereza contra las
        # paredes y retrocede. La seguridad trasera manda igual.
        if self._reintentos >= REINTENTOS_MAX:
            return None
        x_r, y_r = trk.xy_eje()
        if y_r <= 0.0:
            return None
        if trk.s_lado * x_r >= SEPARACION_MIN_VALIDA:
            return None
        if med.trasera < EMERGENCIA_TRASERA:
            return None
        self._reintentos += 1
        self._estado_suspendido = (self.estado if self.estado in ESTADOS_MANIOBRA
                                   else "COMPROMISO")
        self._entrar("REINTENTO", ahora)
        print("[FSM] %s -> REINTENTO #%d (%s) | pilar #%d %s a (%.0f, %.0f), "
              "separacion %+.0fmm del lado obligatorio"
              % (self._estado_suspendido, self._reintentos, motivo, trk.id,
                 trk.color, x_r, y_r, trk.s_lado * x_r))
        return (VELOCIDAD_REVERSA, 0.0)

    def _est_reintento(self, med, color_cam, heading, ahora):
        """Marcha atras para volver a entrar al pilar por el lado bueno.

        No es el RETROCESO de emergencia: alli se huye de una pared y el
        pilar es un recuerdo; aqui el pilar es el OBJETIVO y la marcha
        atras es parte de su maniobra. Por eso la salida no es "ya hay
        hueco" sino "ya existe un comando de avance que cumple la regla"
        (`_existe_paso_legal`), que es la misma condicion con la que
        APERTURA da por terminada su excursion.

        El volante: en reversa el chasis gira al reves que hacia
        adelante, asi que para ABRIR separacion hay que pedir el signo
        contrario al comando de apertura. Es el mismo razonamiento -- y
        el mismo signo -- que el encare por camara de _est_retroceso.

        La seguridad no se mezcla: el hueco de detras manda y define el
        margen; dentro de ese margen se elige el angulo que mas abre. Si
        no hay margen, gana la seguridad.
        """
        trk = self.tracker
        t_en = ahora - self._t_estado
        holgura = self._holgura_exigida()

        if not trk.activo or self._solo_obstaculo:
            self._estado_suspendido = None
            return self._abortar(med, ahora, "el objetivo no sobrevivio al reintento")

        x_r, y_r = trk.xy_eje()

        # DOS REGIMENES, segun se pueda o no confiar en la estimacion.
        #   viva   -> el objetivo es ABRIR el lado, y se sale cuando
        #             existe un comando de avance que cumple la regla.
        #   muerta -> el objetivo es RECUPERAR LA PERCEPCION, y se sale
        #             cuando el tracker vuelve a tener el poste. Aqui no
        #             se conduce por la estimacion: preguntar
        #             `_existe_paso_legal` con sigma 200 es preguntarle a
        #             un numero inventado.
        ciego = trk.perdido(ahora)

        # Salidas, en orden de prioridad.
        if med.trasera < EMERGENCIA_TRASERA:
            razon, ok = "sin sitio detras (%.0fmm)" % med.trasera, False
        elif t_en > TIMEOUT_REINTENTO:
            razon, ok = "tiempo maximo", False
        elif t_en <= TIEMPO_MIN_REINTENTO:
            razon, ok = None, None
        elif ciego:
            razon, ok = None, None
        elif self._ciego_previo:
            razon, ok = "estimacion recuperada (sigma %.0fmm)" % trk.sigma, True
        elif self._existe_paso_legal(med, VELOCIDAD_EVASION,
                                     (x_r, y_r), holgura):
            razon, ok = "ya hay paso legal", True
        else:
            razon, ok = None, None
        self._ciego_previo = ciego

        if razon is not None:
            destino = self._estado_suspendido or "COMPROMISO"
            self._estado_suspendido = None
            if ok:
                self._t_ultimo_progreso = ahora
                self._ciclos_suspension = 0
                self._entrar(destino, ahora)
                print("[FSM] REINTENTO -> %s (%s) | pilar #%d %s, separacion "
                      "%+.0fmm" % (destino, razon, trk.id, trk.color,
                                   trk.s_lado * x_r))
                velocidad = max(VELOCIDAD_MINIMA,
                                self._con_frenado(VELOCIDAD_EVASION, med.frontal))
                return (velocidad, self._nominal_arbitrado(med, velocidad))
            # Ni retrocediendo. Ahora si es un aborto de verdad, y el
            # cupo gastado impide que se vuelva a intentar con este pilar.
            self._reintentos = REINTENTOS_MAX
            return self._abortar(med, ahora, "no hay paso legal ni atras (%s)" % razon)

        # Seguridad trasera primero, igual que en RETROCESO: el hueco
        # medido detras manda y define el margen.
        error  = med.trasera_derecha - med.trasera_izquierda
        angulo = _clamp(error * KP_RETROCESO, MAX_ANGULO_RETROCESO)

        # PRIORIDADES EN REVERSA, cada una dentro del margen que deja la
        # anterior. No se mezclan: es la misma disciplina que el resto
        # del archivo -- seguridad, luego regla, luego preferencia.
        #
        #   1. hueco trasero medido   (ya calculado arriba: manda)
        #   2. ABRIR el lado obligatorio, que es a lo que se vino aqui
        #   3. quedar paralelo a las paredes
        #
        # EL ORDEN ES EL ARREGLO, y esta medido. La version anterior puso
        # el paralelo de PRIMERO -- solo enderezar -- y no es que no
        # abriera: CERRABA. Corrida 212004, pilar #7 ROJO, los 2,5 s de
        # reintento:
        #
        #     t=3,40  entra con sep=+96 (necesita 138)
        #     t=4,50  sep=+74      t=5,00  sep=+44
        #     t=5,50  sep=+15      t=5,70  sep=-2   <- lado PROHIBIDO
        #
        # La razon es geometrica: el poste esta DELANTE, asi que rotar el
        # chasis para alinearlo con la pared lo barre lateralmente, y con
        # el chasis torcido hacia el lado malo ese barrido va justo en la
        # direccion que no toca. Ponerse paralelo es una preferencia
        # buena -- deja la maniobra empezando desde una pose limpia -- y
        # de hecho aqui se conserva; lo que no puede es decidir por
        # encima de la regla que define la carrera.
        #
        # Signos: `comando_apertura` es hacia ADELANTE y en reversa el
        # chasis gira al reves, de ahi el menos. `angulo_muro` positivo =
        # chasis girado a la IZQUIERDA respecto a la pared; adelante se
        # corrige con el signo cambiado (ver _rumbo_nominal), asi que en
        # reversa va directo.
        abrir = 0.0
        if not ciego:
            req = gev.separacion_requerida(y_r, holgura)
            objetivo = (SEPARACION_MIN_VALIDA + MARGEN_APERTURA if req is None
                        else req + MARGEN_APERTURA)
            abrir = _clamp(-gev.comando_apertura(x_r, y_r, trk.s_lado, objetivo),
                           MAX_ANGULO_RETROCESO)
            margen = max(0.0, MAX_ANGULO_RETROCESO - abs(angulo))
            angulo += _clamp(abrir - angulo, margen * FRACCION_ENCARE)
        self.rama_evasion = "REINTENTO_CIEGO" if ciego else "REINTENTO"

        # El paralelo solo cuando NO contradice a la apertura. Si los dos
        # piden el mismo sentido de giro se suma lo que quepa; si piden
        # sentidos opuestos, gana la regla y el enderezado se deja para
        # el avance siguiente, que si puede hacerlo sin coste.
        if med.muro_valido:
            fiable = _clamp(med.angulo_muro, MAX_DERIVA_MURO)
            paralelo = _clamp(fiable * KP_PARALELO_REVERSA,
                              MAX_ANGULO_RETROCESO)
            if ciego or paralelo * abrir > 0.0:
                margen = max(0.0, MAX_ANGULO_RETROCESO - abs(angulo))
                angulo += _clamp(paralelo - angulo, margen * FRACCION_ENCARE)
                self.rama_evasion = "REINTENTO_PARALELO"
        self.cmd_deseado = abrir
        return (VELOCIDAD_REVERSA, angulo)

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
                # Aqui SI hace falta un lado si o si -- es el desempate
                # de ultimo recurso y su valor esta en romper el bucle --
                # pero se pide en orden: medida, pista, y solo si no hay
                # nada, el lado hacia el que mas espacio queda AHORA.
                self._signo_giro_forzado = (
                    self._preferencia_esquina(med)
                    or (float(self.pista.sentido) if self.pista.conocido else 0.0)
                    or (1.0 if med.izquierda >= med.derecha else -1.0))
                self._heading_objetivo = None
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

        # RETROCEDER RECOLOCANDOSE FRENTE AL PILAR (idea del equipo).
        #
        # Retroceder recto deja al robot mirando adonde miraba, o sea
        # encarado igual de mal que cuando se atasco: vuelve a avanzar y
        # vuelve al mismo sitio. Si en vez de eso la marcha atras lo deja
        # ENCARANDO el poste, el avance siguiente empieza con la
        # geometria que la maniobra necesita.
        #
        # No se MEZCLA con la seguridad, que es el error que este archivo
        # lleva todo el dia quitando: el espacio libre de detras sigue
        # mandando y define el margen; dentro de ese margen se elige el
        # angulo que mas centra el pilar en la camara. Si no hay margen,
        # gana la seguridad y no pasa nada.
        # LA MARCHA ATRAS DEL DESEMPATE SUMA ROTACION. Ver
        # FRACCION_ROTA_REVERSA: sin esto, el avance devuelve exactamente
        # el angulo que la reversa habia ganado y el atasco no termina
        # nunca. Va por delante del encare por camara porque cuando el
        # desempate esta activo no hay maniobra que preservar -- se esta
        # rompiendo un bucle -- y por detras de la seguridad trasera,
        # como todo lo demas en este estado.
        if self._signo_giro_forzado:
            deseado = -self._signo_giro_forzado * MAX_ANGULO_RETROCESO
            margen = max(0.0, MAX_ANGULO_RETROCESO - abs(angulo))
            angulo += _clamp(deseado - angulo, margen * FRACCION_ROTA_REVERSA)
            self.rama_evasion = "REVERSA_ROTA"
            return (VELOCIDAD_REVERSA, angulo)

        trk = self.tracker
        if trk.activo and not self._solo_obstaculo:
            x_r, y_r = trk.xy_eje()
            xl, yl = geo.eje_trasero_a_lidar(x_r, y_r)
            rumbo = optica.rumbo_camara_de_cluster(xl, yl)
            # En reversa el chasis gira al reves que en marcha adelante,
            # asi que para llevar el pilar hacia el centro hay que pedir
            # el signo contrario al del rumbo.
            deseado = _clamp(-rumbo * KP_ENCARE_REVERSA, MAX_ANGULO_RETROCESO)
            margen = max(0.0, MAX_ANGULO_RETROCESO - abs(angulo))
            angulo += _clamp(deseado - angulo, margen * FRACCION_ENCARE)
            self.rama_evasion = "REVERSA_ENCARA"
        return (VELOCIDAD_REVERSA, angulo)

    def _est_giro_forzado(self, med, color_cam, heading, ahora):
        # Desempate de esquina simetrica. Unico estado que NO recalcula su
        # decision cada ciclo: recalcularla con la misma señal simetrica
        # que causo el atasco la volveria a poner en 0.
        t_en = ahora - self._t_estado
        diff = med.izquierda - med.derecha
        asimetria = (diff * self._signo_giro_forzado) > SALIDA_GIRO_FORZADO_ASIMETRIA

        # META DE RUMBO: el siguiente tramo recto. Se fija UNA vez al
        # entrar -- como el resto de este estado, que es el unico que no
        # recalcula su decision cada ciclo -- porque recalcularla con la
        # misma señal simetrica que causo el atasco la volveria a poner
        # en cero. Ver TOLERANCIA_ALINEACION.
        if self._heading_objetivo is None:
            if self._signo_giro_forzado > 0:
                self._heading_objetivo = (math.floor(heading / 90.0) + 1.0) * 90.0
            else:
                self._heading_objetivo = (math.ceil(heading / 90.0) - 1.0) * 90.0
            print("[GIRO] alinearse con el cuadrante: %.0f -> %.0f grados"
                  % (heading, self._heading_objetivo))
        error = self._heading_objetivo - heading
        alineado = abs(error) < TOLERANCIA_ALINEACION
        # TERCERA SALIDA: LA PARED. Este estado se salta la envolvente a
        # proposito -- para eso existe -- y la emergencia del paso 1
        # tampoco lo mira, asi que sin esto no tenia NINGUN suelo: en el
        # escenario del pilar en la diagonal se comio 17 ciclos seguidos
        # a +25 y velocidad de giro con el frente cayendo de 317 a 47 mm,
        # y choco. Un giro forzado que se empotra no desatasca nada.
        #
        # Salir aqui no reabre el bucle que el estado rompe: los ~48
        # grados de yaw que ya gano no se pierden al salir, y el contador
        # de rachas se pone a cero, asi que lo que venga despues --
        # incluida otra marcha atras -- arranca de limpio y desde otra
        # orientacion.
        pared_encima = med.frontal < SALIDA_GIRO_FORZADO_FRENTE

        # LA PARED YA NO SACA DEL ESTADO: pasa a ser el momento de meter
        # marcha atras. Sacarlo ahi era lo que perdia el giro a medias --
        # y lo que convertia el suelo de pared, puesto para que no
        # chocara, en la jaula que producia el ciclo limite.
        puede_avanzar = not pared_encima
        puede_retroceder = med.trasera > TRASERA_PARA_TRES_TIEMPOS
        sin_sitio = not puede_avanzar and not puede_retroceder

        if alineado or asimetria or sin_sitio or t_en > TIMEOUT_GIRO_FORZADO:
            razon = ("a rumbo (%.0f grados)" % heading if alineado
                     else "asimetria recuperada" if asimetria
                     else "sin sitio ni delante ni detras" if sin_sitio
                     else "tiempo maximo")
            self._heading_objetivo = None
            self._racha_retroceso = 0
            self._t_ultima_emergencia = None
            # Se apaga aqui, no al salir a RETROCESO: mientras el
            # desempate siga vivo, la marcha atras sigue sumando angulo
            # en su sentido y el par avanzar/retroceder gana rotacion
            # neta. Apagarlo antes era volver al neto cero.
            if asimetria or t_en > TIMEOUT_GIRO_FORZADO:
                self._signo_giro_forzado = 0.0
            if self._estado_suspendido or self.tracker.activo:
                # El giro forzado destruyo la geometria de la maniobra:
                # se cierra de forma explicita, no se retoma a ciegas.
                self._estado_suspendido = None
                print("[FSM] GIRO_FORZADO -> ABORTO (%s)" % razon)
                return self._abortar(med, ahora, "giro forzado durante la maniobra")
            self._entrar("CRUCERO", ahora)
            print("[FSM] GIRO_FORZADO -> CRUCERO (%s, %.1fs)" % (razon, t_en))
            return (VELOCIDAD_CRUCERO, self._nominal_arbitrado(med, VELOCIDAD_CRUCERO))

        # TRES TIEMPOS DENTRO DEL ESTADO. El sentido de giro lo marca el
        # error de rumbo, no un signo congelado: asi la maniobra se cierra
        # sobre su meta en vez de girar hasta que suene el reloj.
        signo = 1.0 if error > 0.0 else -1.0

        # HISTERESIS POR TIEMPO, no por distancia. Ver TRAMO_MIN_GIRO: un
        # tramo empezado se termina, aunque la lectura cruce el umbral.
        if (self._t_tramo_giro is None
                or self._sentido_tramo_giro is None
                or (ahora - self._t_tramo_giro) >= TRAMO_MIN_GIRO):
            self._sentido_tramo_giro = 1 if puede_avanzar else -1
            self._t_tramo_giro = ahora
        puede_avanzar = (self._sentido_tramo_giro > 0)

        if puede_avanzar:
            velocidad = self._con_frenado(VELOCIDAD_GIRO_FORZADO, med.frontal)
            return (max(VELOCIDAD_MINIMA, velocidad), signo * ANGULO_GIRO_FORZADO)
        # Sin sitio delante: atras con el volante al REVES, que en
        # reversa hace girar el chasis en el MISMO sentido.
        self.rama_evasion = "GIRO_TRES_TIEMPOS"
        return (VELOCIDAD_REVERSA, -signo * ANGULO_GIRO_FORZADO)

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
