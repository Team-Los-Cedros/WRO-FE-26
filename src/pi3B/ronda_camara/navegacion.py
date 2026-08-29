# Cerebro de la Ronda Cerrada. La clase Navegador recibe por cada barrido
# del LiDAR la medicion, el color de la camara y el yaw de la IMU, y
# devuelve la consigna (velocidad, angulo) para la Pico. No abre puertos
# ni hilos, asi que se puede probar fuera del robot con barridos grabados.
#
# Fases: CAPTURA_FIRMA -> CARRERA -> PARQUEO -> FIN
#
# Estados de evasion dentro de CARRERA:
#   CRUCERO          centrado entre paredes (P sobre el error lateral)
#   APROXIMACION     pure pursuit hacia el punto de paso al lado del poste
#                    (ROJO -> por la derecha, VERDE -> por la izquierda)
#   SOBREPASO        rumbo paralelo al pasillo hasta dejar el poste atras
#   REINCORPORACION  volver al rumbo original con P sobre la IMU
#
# Estos tres persiguen al poste o al rumbo sin saber donde esta la pared
# -- _con_seguridad_pared() mezcla el comando con el centrado de pared
# normal cuando la pared del lado del giro se acerca (bug real de pista:
# la evasion no "veia" las paredes y podia clavarse contra ellas).
#   RETROCESO        emergencia anti-choque: reversa con control P sobre
#                    las diagonales traseras del LiDAR (no un signo fijo,
#                    ver nota abajo)
#   GIRO_FORZADO     desempate de esquina simetrica: unico estado que NO
#                    recalcula su decision cada ciclo (ver nota abajo)
#
# La emergencia se chequea en todos los ciclos sin importar el estado.
# Angulo positivo = giro a la izquierda EN MARCHA ADELANTE. En reversa el
# mismo angulo de rueda gira el chasis al sentido contrario (geometria
# Ackermann: para el mismo angulo, la tasa de giro del rumbo cambia de
# signo si la velocidad cambia de signo). Por eso RETROCESO no usa un
# signo fijo -- mide en vivo que diagonal trasera tiene mas espacio
# (med.trasera_derecha vs med.trasera_izquierda) y gira hacia ahi cada
# ciclo, autocorrigiendose sin importar el sentido de giro de la pista.
# El servo solo da -20/+25 grados reales, de ahi todos los clamps.
#
# GIRO_FORZADO existe por un caso limite medido en pista (README 8.4):
# aproximandose a una esquina perfectamente simetrica (izquierda~derecha
# en cada ciclo) ni angulo_muro ni el propio RETROCESO tienen ninguna
# señal instantanea para preferir un lado -- las dos paredes se ven igual
# de cerca todo el tiempo, asi que ambos deciden ~0 y el robot repite
# emergencia-retroceso-reintento indefinidamente (medido: 133s, 51
# episodios seguidos). Ningun control reactivo puro puede resolver esto:
# hace falta memoria ENTRE ciclos. _racha_retroceso cuenta emergencias
# encadenadas (cada una poco despues de la anterior, ver VENTANA_ATASCO);
# al llegar al umbral se entra a GIRO_FORZADO con un lado decidido UNA
# VEZ (la ultima asimetria real vista, o un lado por defecto si nunca
# hubo ninguna) y mantenido hasta romper el empate, en vez de
# recalcularse cada ciclo como todo lo demas en este archivo.
#
# GIRO_FORZADO es la ultima red, no la primera: el caso que de verdad se
# midio en pista (README 8.5) lo resuelve antes _con_escape_frontal, que
# ataca la causa -- el centrado se anulaba a si mismo y nadie miraba el
# frente. Ver la nota de DIST_ESCAPE_FRONTAL.
import math
import time

import tracker as tracker_mod
import optica
from lidar_geometria import centroide_xy_cluster

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
# Ademas empeoro lo demas: 233 grados de rumbo acumulado en 133 s contra
# 398 en 122 s, 12% del tiempo en RETROCESO contra 10%, y 71 ciclos con
# velocidad comandada y el frontal congelado -- sintoma de que a 18 de
# PWM el motor se cala contra la friccion.
#
# Vuelve a 25, que es el valor con el que no se calaba. 20 (la velocidad
# de parqueo) queda como terreno intermedio sin probar.
VELOCIDAD_MINIMA   = 25      # piso del frenado progresivo

# ==========================================
# SEGUIMIENTO DE PARED Y FRENADO
# ==========================================
KP_LATERAL = 0.14            # calibrado en pista, mismo valor que la Ronda Abierta

# Ganancia de la asistencia de esquina por angulo_muro (ver _centrado_paredes).
# Con la lectura estable medida en pista (-22 grados apuntando a una esquina
# real) esto aporta unos 13-15 grados de giro extra hacia el lado que se abre,
# encima de lo que ya de por si de izq-der. En el baseline lejos de cualquier
# esquina (angulo_muro ~ -3 grados) el aporte es de 1-2 grados, despreciable.
KP_ANGULO_MURO = 0.65

# La asistencia de esquina se calibro (README 8.4) suponiendo que lejos
# de una esquina angulo_muro ronda los -3 grados y aporta "1-2 grados,
# despreciable", con -22 grados estables al apuntar a una esquina real.
# Esa validacion se hizo en una corrida limpia SIN pilares. Con la pista
# completa la suposicion no se cumple: medido en la corrida del README
# 8.6, |angulo_muro| tiene mediana 18-22 grados TODO el rato (7 veces el
# baseline supuesto) y rango -59..+45, y su aporte satura el servo el
# solo en el 22% de los ciclos y domina sobre el termino de posicion en
# el 32%. El robot se iba a un lado y al otro sin acumular rumbo: 20
# inversiones de sentido de giro, recorrido estancado en 367 grados
# durante 42s.
#
# Dos condiciones, las dos conservadoras: preservan intacto el caso que
# SI se valido (esquina real con el frente cerrandose, -22 grados ->
# 14.3 de aporte) y recortan lo que va mas alla.

# 1. Una esquina de verdad tiene algo delante. Con el pasillo despejado
#    no hay esquina que asistir, y ahi es donde la señal era pura basura:
#    206 ciclos (25% de las lecturas fuertes) con el frente a mas de
#    900mm, el robot centrado (|izq-der| mediana 200mm) y la asistencia
#    inyectando 20 grados de media sin ningun motivo.
DIST_ASISTENCIA_ESQUINA = 900.0

# 2. Asistencia quiere decir asistir, no mandar. Con lecturas de +-40/50
#    grados el aporte llegaba a 26-32 grados y saturaba el servo por si
#    solo, tapando por completo el control de posicion. El tope deja
#    pasar entero el caso validado (14.3) y corta el resto.
MAX_APORTE_ANGULO_MURO = 15.0

DIST_FRENADO_INICIO = 900.0  # mm, empieza a bajar velocidad
DIST_FRENADO_MIN    = 300.0  # mm, velocidad minima alcanzada

# ==========================================
# ESCAPE FRONTAL
# ==========================================
# Los dos terminos de _centrado_paredes miden cosas distintas -- posicion
# entre paredes (izq-der) y orientacion respecto al muro (angulo_muro) --
# y acercandose a una esquina apuntan a lados OPUESTOS y se anulan.
# Medido en la corrida del 8.5, ciclo a ciclo:
#
#   t=44.96 front=278 izq-der=-38 a_muro=-6.4 | T_pos=-5.32 T_muro=+4.17 -> -1.15
#   t=45.96 front=208 izq-der=-30 a_muro=-5.9 | T_pos=-4.20 T_muro=+3.82 -> -0.38
#
# 274 de los 394 ciclos con el frente por debajo de 400mm (70%) tienen
# los dos terminos cancelandose, dejando un comando mediano de 1.5
# grados con un servo que da 20-25. El robot entra recto contra la pared
# con la direccion practicamente centrada, dispara EMERGENCIA a 120mm,
# retrocede y repite: 12 episodios, uno cada 4.7-5.8s.
#
# Ademas ninguno de los dos terminos mira el frente: (izq-der) dice donde
# esta el robot ENTRE las paredes, no cuanto espacio total queda, y en un
# pasillo que se cierra a 200mm de ancho vale casi cero aunque el robot
# este a punto de chocar contra las dos.
#
# El escape mete la unica pregunta que faltaba -- "hay pared delante, hacia
# donde salgo" -- y le da autoridad creciente segun se cierra el frente,
# mezclandose sobre el centrado normal en vez de sumarse (sumar dejaria
# que la cancelacion se lo siguiera comiendo).
DIST_ESCAPE_FRONTAL = 500.0   # mm, por encima de esto no interviene
ANGULO_ESCAPE_MAX   = 22.0    # grados a plena urgencia

# ==========================================
# EVASION
# ==========================================
SEPARACION_LATERAL   = 260.0  # mm entre el centro del poste y el punto de paso
KP_PURSUIT           = 1.0    # grados de comando por grado de bearing
KP_HEADING           = 1.0    # P de rumbo en SOBREPASO / REINCORPORACION
MAX_ANGULO_EVASION   = 25.0   # tope fisico util del servo
ANGULO_EVASION_CIEGA = 18.0   # tope de la evasion a ciegas (magnitud, no signo)

# Cuanto hay que apuntar POR FUERA del poste para pasarlo limpio, en
# grados. Es el equivalente angular de SEPARACION_LATERAL: 260mm a la
# distancia tipica de una evasion a ciegas (el color entra a
# DIST_INICIO_EVASION_CAM = 700mm y el poste suele estar entre 400 y 700)
# son atan(260/550) = 25 grados. Con el poste centrado da el mismo
# comando de siempre (se satura en ANGULO_EVASION_CIEGA); lo que cambia
# es el caso en que el poste ya esta a un lado.
MARGEN_PASO_GRADOS = 25.0

# ==========================================
# APAREO COLOR <-> CLUSTER (que poste es cual)
# ==========================================
# La camara dice QUE color hay delante y el LiDAR DONDE hay postes, pero
# hasta ahora se juntaban con dos criterios independientes: el color del
# blob de mayor area y el cluster mas cercano. Con dos postes en el mismo
# frame pueden caer en postes distintos, y el color acaba pegado a la
# posicion equivocada -- el robot esquiva hacia el lado contrario al que
# manda el reglamento (rojo por la derecha, verde por la izquierda).
# Estaba anotado como pendiente en el README 8.3.
#
# Con el cx del blob (vision.get_deteccion) se puede aparear por ANGULO:
# se convierte el pixel a rumbo con el modelo estenopeico
#   rumbo = atan((cx - c0) / f)
# y se elige el cluster que este en ese mismo rumbo, no el mas cercano.
#
# c0 y f DEBERIAN salir de calib_fov.py (que ya existe y ajusta justo
# este modelo contra postes reales), pero nunca se ha corrido: no hay
# resultados guardados. Mientras tanto se derivan del FOV de catalogo de
# la Camera Module 3 Wide documentado en README 4.2 (~102 grados), con
# el centro optico en el centro geometrico del frame. Correr calib_fov.py
# y sustituir estos dos numeros es la forma de afinarlo.
# MONTAJE EN MASTIL: los tres numeros de abajo ya no se derivan del
# catalogo, se importan medidos de optica.py (ver ronda_camara/README.md
# secciones 3 y 5). El de catalogo inflaba el rumbo 2.3x.
ANCHO_FRAME_CAM   = optica.ANCHO_FRAME
CX_CENTRO_OPTICO  = optica.CX_CENTRO_OPTICO        # c0 medido, no 160
HFOV_CAMARA       = optica.HFOV_EFECTIVO           # medido: 53.8, no 102
FOCAL_PX          = optica.FOCAL_PX

# Cuanto puede discrepar el rumbo de la camara del rumbo del cluster para
# darlos por el mismo poste. Generoso a proposito: camara y LiDAR estan a
# alturas distintas del chasis (paralaje) y la focal es de catalogo, sin
# calibrar. Si en pista se ve que rechaza apareos buenos, el arreglo es
# correr calib_fov.py, no abrir mas la tolerancia.
TOLERANCIA_APAREO_GRADOS = optica.TOLERANCIA_APAREO_GRADOS   # 10, ver optica.py

# Hasta donde se buscan postes candidatos, EN ANGULO. Antes la puerta era
# rectangular (abs(cx) < 450mm), y un filtro en milimetros laterales se
# cierra en angulo segun te alejas: 450mm de lado son 48 grados a 400mm
# pero solo 26.6 a 900mm, justo donde el poste deberia engancharse para
# que la evasion tenga margen. La camara ve +-51 grados (HFOV 102), asi
# que veia postes que este filtro rechazaba -- y precisamente los
# LEJANOS. El mismo poste acababa entrando al acercarse, cuando la
# puerta ya se habia abierto en angulo, pero para entonces estaba encima.
#
# Medido en la corrida 6: el color se detecta a 858mm de mediana pero el
# tracker no engancha hasta los 676mm, y el 70% de los enganches ocurren
# con el poste ya a menos de 700mm. Son 224mm por debajo de
# DIST_INICIO_EVASION_TRK (900mm), o sea 1.4s de maniobra perdidos a la
# velocidad de evasion.
#
# Con una puerta angular el criterio deja de depender de la distancia y
# se alinea con lo que la camara puede ver. El apareo por rumbo ya acota
# de verdad cual es el poste bueno (TOLERANCIA_APAREO_GRADOS); esta
# puerta solo marca el borde del campo de busqueda.
SECTOR_BUSQUEDA_POSTE = 50.0   # grados a cada lado del frente

DIST_INICIO_EVASION_TRK = 900.0   # mm, poste confirmado por tracker
DIST_INICIO_EVASION_CAM = 700.0   # mm, frontal LiDAR + color de camara
Y_POSTE_EN_PASO         = 180.0   # mm, el poste ya esta a la altura del morro

# Debajo de esto, la pared del lado hacia donde se esta girando empieza a
# mezclarse con el comando deseado (ver _con_seguridad_pared). El pure
# pursuit y el rumbo paralelo de la evasion persiguen al poste sin saber
# donde esta la pared -- esto evita que la persecucion mande al robot
# contra la pared cuando el carril es mas angosto de lo esperado.
DIST_ALERTA_PARED = 220.0

# Distancia a la que el centrado toma el mando POR COMPLETO, dejando de
# perseguir el poste. Antes no existia: el peso era 1 - pared/220, que
# vale 0.25 a 164mm y solo 0.64 a 80mm (el umbral de emergencia), o sea
# que la persecucion del poste conservaba el 36% del comando incluso
# pegado a la pared. Medido en la corrida del README 8.6, ciclo a ciclo:
#
#   t=74.16 izq=164 der=807 ang=+25.0   pared izquierda cerca,
#   t=74.86 izq=136 der=858 ang= +7.7   pasillo abierto a la derecha,
#   t=75.46 izq=100 der=946 ang= +0.5   y el robot girando A LA IZQUIERDA
#   t=75.96 izq= 76         EMERGENCIA
#
# Con izq-der=-672, _centrado_paredes pedia -20 (todo a la derecha), pero
# la mezcla al 25% lo convertia en +14.7: giro hacia la pared. Con la
# rampa saturando a 120mm el centrado manda entero con 40mm de margen
# antes de la emergencia, que a 100mm/s es medio segundo de reaccion.
DIST_PARED_CRITICA = 120.0

# Velocidad de avance en funcion del PWM, a partir de la curva medida en
# pista (ver la nota de tracker.MM_POR_SEG_A_PWM100). Sale practicamente
# proporcional, asi que basta escalar la constante.
def _vel_mm_s(pwm):
    return tracker_mod.MM_POR_SEG_A_PWM100 * abs(pwm) / 100.0


# Los timeouts de la evasion se derivan de la velocidad a la que se corre
# ESA fase, no de la de crucero. Con la curva medida, VELOCIDAD_EVASION=40
# son unos 160 mm/s, no los 215 de crucero: derivarlos de la velocidad de
# crucero los dejaba un 35% cortos justo en la fase donde se aplican.
VELOCIDAD_EVASION_MMS = _vel_mm_s(VELOCIDAD_EVASION)

# Margen sobre el tiempo teorico. Estos timeouts son RED DE SEGURIDAD:
# la transicion normal es geometrica (el tracker dice donde esta el
# poste). Si el timeout es mas corto que la fisica deja de ser una red y
# pasa a ser la ruta principal, que es justo lo que estaba pasando.
MARGEN_TIMEOUT = 1.3

# APROXIMACION: desde que se confirma el poste hasta tenerlo al costado.
# Con 1.5s el robot solo recorria 320mm de los 720 necesarios, asi que
# saltaba a SOBREPASO con el poste todavia a 460mm por delante, cuando
# SOBREPASO todavia enderezaba hacia el rumbo previo a la evasion y por
# tanto deshacia la esquiva justo delante del poste.
TIMEOUT_APROXIMACION = round(
    (DIST_INICIO_EVASION_TRK - Y_POSTE_EN_PASO) / VELOCIDAD_EVASION_MMS * MARGEN_TIMEOUT, 1)

# SOBREPASO: avance necesario para dejar el poste atras antes de volver
# al carril.
#
# El limite no lo pone el poste sino la pared. En SOBREPASO el robot
# mantiene el rumbo de la esquiva, que apunta ligeramente hacia la pared
# del lado por el que paso, asi que cada segundo de mas en este estado es
# excursion lateral acumulada. Medido en la corrida 7, con el servo casi
# recto todo el tramo:
#
#   t=2.61 SOBREPASO der=366    t=3.61 der=287    t=4.81 der=186
#   t=5.50 REINCORPORACION der=130
#
# Son 81 mm/s de cierre lateral sostenido. Con 350mm el estado duraba
# 2.9s, el robot entraba a REINCORPORACION ya a 130mm de la pared y bajo
# hasta 84mm con el servo saturado al tope contrario -- a 4mm del umbral
# de emergencia (80mm). La direccion Ackermann necesita avance para
# desplazarse de lado, asi que llegar saturado no basta: hay que no
# llegar tan cerca.
#
# Con 200mm el estado dura ~1.6s y la excursion prevista es de unos
# 130mm, dejando la salida de SOBREPASO cerca de 235mm de la pared. El
# poste no corre riesgo: cuando este estado empieza ya esta a ~294mm de
# separacion lateral, muy por encima del medio ancho del robot.
DIST_SOBREPASO_MM = 200.0
TIMEOUT_SOBREPASO = round(DIST_SOBREPASO_MM / VELOCIDAD_EVASION_MMS * MARGEN_TIMEOUT, 1)

TIMEOUT_REINCORPORACION = 2.5
# Margen de centrado para dar la reincorporacion por terminada. El pasillo
# de la pista ronda los 1000mm, asi que 120mm de diferencia entre paredes
# es estar practicamente en el eje. Se compara contra (izquierda-derecha),
# que es una medida de POSICION y por eso no puede sobrepasar el objetivo
# como si lo hacia el lazo de rumbo que habia antes.
ERROR_LATERAL_OK       = 120.0

# ==========================================
# EMERGENCIA ANTI-CHOQUE
# ==========================================
EMERGENCIA_FRONTAL  = 120.0
EMERGENCIA_LATERAL  = 80.0
EMERGENCIA_TRASERA  = 250.0
TIMEOUT_RETROCESO   = 3.5

# Margenes para DAR POR TERMINADO el retroceso. Son mas holgados que los
# de entrada a proposito: saliendo justo en el umbral, el primer ciclo de
# CRUCERO vuelve a disparar la emergencia y el robot se queda rebotando
# entre avanzar y retroceder sin salir del sitio.
SALIDA_RETROCESO_FRONTAL = 300.0
SALIDA_RETROCESO_LATERAL = 160.0

# Minimo de retroceso antes de siquiera evaluar si ya esta despejado:
# hace falta despegarse de verdad del obstaculo, no solo dejar de tocarlo.
TIEMPO_MIN_RETROCESO = 0.6

# Control P del retroceso: error = espacio diagonal-trasero derecho menos
# izquierdo (mm), medido en vivo cada ciclo con el perfil 360 del LiDAR.
# KP_RETROCESO=0.05 satura al tope (25 grados) con una diferencia de
# ~500mm entre ambas diagonales -- valor de partida, ajustar en pista.
KP_RETROCESO         = 0.05
MAX_ANGULO_RETROCESO = 25.0

# ==========================================
# DESEMPATE DE ESQUINA SIMETRICA (GIRO_FORZADO)
# Unico bloque de estado de este archivo que persiste MAS ALLA de un solo
# episodio de RETROCESO -- ver la nota de cabecera y README 8.4.
# ==========================================

# Reintentos de RETROCESO seguidos antes de forzar el giro. En 4 cabe una
# esquina con algo de asimetria real (caso normal, README 8.4-1)
# resolviendose sola antes de que esto intervenga -- no es la primera
# linea de defensa, es la red de debajo.
RACHA_RETROCESO_PARA_FORZAR = 4

# Que cuenta como "seguidos": una emergencia nueva dentro de esta ventana
# desde la anterior. Primer intento de esto uso el avance de rumbo entre
# episodios y NO funciono en pista (corrida del 8.5): el robot giraba
# 5-7 grados por ciclo sin escapar de la esquina, asi que la racha se
# reiniciaba cada dos episodios y nunca llegaba al umbral. El rumbo se
# mueve sin que el robot progrese -- no sirve como medida de escape.
# La cadencia si distingue los dos casos sin ambiguedad: atascado, las
# emergencias caen cada 4.7-5.8s como un reloj (12 episodios medidos);
# en una corrida sana no hay ninguna (corridas 6, 7 y 8 de README 8.3).
VENTANA_ATASCO = 10.0   # s

# Por debajo de esto en |izquierda-derecha| no hay pista real, es ruido
# tipico del C1 (~15mm). Por encima, el signo se guarda como la ultima
# preferencia de lado observada -- la esquina rara vez es simetrica
# perfecta desde lejos, asi que normalmente hay un sesgo minusculo pero
# real que capturar antes de que se cierre del todo.
UMBRAL_MEMORIA_ASIMETRIA = 30.0

# Lado por defecto cuando nunca hubo ninguna asimetria que memorizar (la
# esquina fue simetrica desde el primer ciclo, caso de la corrida del
# 8.4-3). No hay ninguna pista fisica para elegir aqui -- es arbitrario a
# proposito, y el punto es que sea consistente y termine el bucle, no que
# acierte el lado "correcto" (no lo hay).
LADO_POR_DEFECTO = 1.0     # +1.0 = izquierda

ANGULO_GIRO_FORZADO    = MAX_ANGULO_EVASION
VELOCIDAD_GIRO_FORZADO = VELOCIDAD_EVASION
TIMEOUT_GIRO_FORZADO   = 2.5   # s, tope de seguridad si nunca se desatasca

# Salida por geometria: la pared del lado hacia el que se fuerza el giro
# se abre de verdad (la esquina dejo de ser simetrica), no solo ruido.
SALIDA_GIRO_FORZADO_ASIMETRIA = 150.0

# ==========================================
# CARRERA / PARQUEO
# ==========================================
UMBRAL_VUELTAS       = 1010.0  # grados de yaw neto, ~3 vueltas
TOLERANCIA_FIRMA     = 80.0    # mm contra la firma de pared inicial
TIMEOUT_PARQUEO      = 6.0     # s

# Sectores frontales ensanchados durante la evasion para no perder el poste
SECTOR_EVASION_IZQ = (330.0, 20.0)
SECTOR_EVASION_DER = (340.0, 30.0)

# Rate limiter del servo en marcha normal (la emergencia lo salta).
#
# Subido de 6 a 12 el 2026-08-29. Medido en la corrida del 145548: dos de
# las cinco emergencias son el limitador en accion, con el angulo
# moviendose exactamente +6.00 por ciclo mientras la pared se acercaba:
#
#   21.8  ang -18.83     21.9  ang -12.83     22.0  ang -6.83
#   22.1  ang  -0.83     22.2  EMERGENCIA
#
# Invertir el volante de -19 a +11 tardaba 5 ciclos (0.5 s) y a 100 mm/s
# el robot se comia los 145mm que le quedaban antes de llegar al angulo.
# Con 12 el mismo recorrido son 3 ciclos.
#
# CONFIRMADO en la corrida 153206: de sus 8 emergencias, CERO tienen la
# firma del limitador (antes eran 2 de 5). El cuello de botella que
# quitaba este cambio era real y ya no aparece. Se queda en 12.
#
# El limitador existe para que el servo no de tirones que desestabilicen
# la marcha: si en pista se ve que el coche colea o que el rumbo se
# vuelve ruidoso en recta, bajarlo.
MAX_DELTA_ANGULO = 12.0

# Topes reales del servo, medidos en la Pico: CENTRO=90 con el comando
# recortado a [70, 115], o sea -20 grados a la derecha y +25 a la
# izquierda. Son asimetricos, asi que un unico tope simetrico no sirve:
# con +-25 los comandos a la derecha piden 5 grados que el servo no tiene
# y la Pico los recorta sin avisar, dejando el giro a la derecha mas
# debil que el de la izquierda sin que se note en el log.
ANGULO_MAX_IZQ = 25.0
# PROBADO Y REVERTIDO el 2026-08-29. Se subio a 25 (con LIMITE_DER 70->65
# en el firmware) para igualar los dos lados, porque el robot lograba 17
# grados/s girando a la derecha contra 24 a la izquierda. NO FUNCIONO: con
# el servo pidiendo 25 la velocidad angular a la derecha se quedo en 16.8
# grados/s, igual que con 20 (17.6 y 17.0 en las dos corridas anteriores).
#
# O sea que el tope real no es el firmware: es MECANICO, la direccion no
# da mas de si antes de la posicion 65 del servo. Pedir mas solo deja el
# servo forzando contra el tope, que consume corriente sin girar. Los dos
# numeros vuelven a su sitio (aqui 20, LIMITE_DER 70 en la Pico).
#
# La asimetria sigue ahi y sigue siendo la causa de que las curvas a
# derechas se queden largas, pero se arregla en la varilla de direccion,
# no en software.
ANGULO_MAX_DER = 20.0


def _clamp(v, lim):
    return max(-lim, min(lim, v))


def _clamp_servo(v):
    # Recorte al recorrido fisico real (positivo = izquierda)
    return max(-ANGULO_MAX_DER, min(ANGULO_MAX_IZQ, v))


class Navegador:
    def __init__(self, control_sector):
        # control_sector: objeto con fijar_sector_frontal() y
        # sector_frontal_normal(), normalmente el ProcesadorLidar
        self._sector = control_sector
        self.tracker = tracker_mod.TrackerObstaculo()

        self.fase   = "CAPTURA_FIRMA"
        self.estado = "CRUCERO"

        self._firma_izq = 0.0
        self._firma_der = 0.0

        self._evadir_por_izquierda = True
        self._heading_sobrepaso = 0.0          # rumbo al empezar a rebasar el poste
        self._t_estado      = 0.0
        self._t_parqueo     = 0.0

        self._ultimo_angulo = 0.0              # para el rate limiter
        self._ultima_vel    = 0
        self._t_ultimo_ciclo = None

        # Desempate de esquina simetrica (GIRO_FORZADO), ver constantes
        # arriba y la nota de cabecera del archivo
        self._racha_retroceso       = 0
        self._t_ultima_emergencia   = None
        self._signo_memoria_asimetria = LADO_POR_DEFECTO
        # Ultimo cx de la camara, guardado para que _est_aproximacion pueda
        # usarlo: los manejadores de estado reciben (med, color_cam,
        # heading, ahora) y el cx no viajaba hasta ellos, que es la razon
        # de raiz de que la evasion a ciegas no supiera DONDE esta el poste.
        self._cx_cam = None
        # Diagnostico de la ultima decision de evasion, para el CSV
        self.rama_evasion     = ""
        self.rumbo_poste_cam  = 0.0
        self.angulo_ciego_viejo = 0.0
        self._signo_giro_forzado      = 0.0

    def procesar(self, med, color_cam, heading, ahora=None, cx_cam=None):
        # Una llamada por barrido completo. Devuelve (velocidad, angulo)
        # o None cuando la carrera termino.
        if ahora is None:
            ahora = time.time()

        # dt del ciclo para la odometria del tracker
        dt = 0.0 if self._t_ultimo_ciclo is None else min(0.3, ahora - self._t_ultimo_ciclo)
        self._t_ultimo_ciclo = ahora

        if self.fase == "CAPTURA_FIRMA":
            self._firma_izq, self._firma_der = med.izquierda, med.derecha
            self.fase = "CARRERA"
            print(f"[+] Firma de parqueo: Izq={self._firma_izq:.0f} Der={self._firma_der:.0f}mm")
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
        # 0. Odometria del tracker (rotacion IMU + avance estimado)
        avance_mm = (self._ultima_vel / 100.0) * tracker_mod.MM_POR_SEG_A_PWM100 * dt
        self.tracker.predecir(heading, avance_mm)
        if self.tracker.activo:
            self.tracker.asociar(med.clusters_obstaculo, centroide_xy_cluster)
        else:
            self._intentar_capturar_poste(med, color_cam, heading, cx_cam)

        # 0b. Memoria de la ultima asimetria REAL entre paredes (por
        # encima del ruido del LiDAR). Vive fuera de cualquier estado a
        # proposito: es la pista que GIRO_FORZADO usa para desempatar una
        # esquina que, cuando por fin dispara la emergencia, puede que ya
        # se vea perfectamente simetrica -- pero rara vez lo fue desde
        # lejos. Sin esto, la unica alternativa es un lado fijo siempre.
        diff_paredes = med.izquierda - med.derecha
        if abs(diff_paredes) > UMBRAL_MEMORIA_ASIMETRIA:
            self._signo_memoria_asimetria = 1.0 if diff_paredes > 0 else -1.0

        # 1. Emergencia anti-choque, prioridad sobre cualquier estado
        if (med.frontal < EMERGENCIA_FRONTAL
                or med.izquierda < EMERGENCIA_LATERAL
                or med.derecha < EMERGENCIA_LATERAL):
            if self.estado not in ("RETROCESO", "GIRO_FORZADO"):
                # Racha por CADENCIA, no por rumbo: una emergencia nueva
                # poco despues de la anterior es un atasco; una aislada
                # despues de mucho rato es un incidente normal.
                if (self._t_ultima_emergencia is not None and
                        (ahora - self._t_ultima_emergencia) <= VENTANA_ATASCO):
                    self._racha_retroceso += 1
                else:
                    self._racha_retroceso = 1
                self._t_ultima_emergencia = ahora
                self._entrar("RETROCESO", ahora)
                self.tracker.desactivar("emergencia")
                self._sector.sector_frontal_normal()
                print(f"[EMERGENCIA] F:{med.frontal:.0f} I:{med.izquierda:.0f} "
                      f"D:{med.derecha:.0f}mm -> RETROCESO (racha {self._racha_retroceso})")

        # 2. Vueltas completas -> parqueo. Solo desde CRUCERO para no
        #    abandonar una evasion a medias con un poste al lado
        if abs(heading) >= UMBRAL_VUELTAS and self.estado == "CRUCERO":
            self.fase = "PARQUEO"
            self._t_parqueo = ahora
            print(f"[!] {heading:.0f} grados acumulados. Modo Parqueo.")
            return (VELOCIDAD_PARQUEO, self._centrado_paredes(med))

        # 3. Despacho por estado
        manejador = {
            "CRUCERO":         self._est_crucero,
            "APROXIMACION":    self._est_aproximacion,
            "SOBREPASO":       self._est_sobrepaso,
            "REINCORPORACION": self._est_reincorporacion,
            "RETROCESO":       self._est_retroceso,
            "GIRO_FORZADO":    self._est_giro_forzado,
        }[self.estado]
        velocidad, angulo = manejador(med, color_cam, heading, ahora)

        # 4. Rate limiter del servo. En emergencia no se aplica, ahi el
        #    giro completo tiene que entrar de una
        if self.estado == "RETROCESO":
            self._ultimo_angulo = _clamp_servo(angulo)
            angulo = self._ultimo_angulo
        else:
            delta = _clamp(angulo - self._ultimo_angulo, MAX_DELTA_ANGULO)
            # Recorte final al recorrido real del servo: aunque cada estado
            # ya acota lo suyo, es esta la unica salida hacia la Pico y
            # aqui se corta cualquier windup antes de que se acumule en
            # _ultimo_angulo y haya que desenrollarlo despues.
            angulo = _clamp_servo(self._ultimo_angulo + delta)
            self._ultimo_angulo = angulo

        self._ultima_vel = velocidad
        return (velocidad, angulo)

    def _est_crucero(self, med, color_cam, heading, ahora):
        trk = self.tracker
        poste_en_rango = trk.confirmado() and 50.0 < trk.y < DIST_INICIO_EVASION_TRK
        # La via de camara exige, ademas del color, que lo que bloquea el
        # frente sea un OBJETO ESTRECHO y no la pared.
        #
        # Antes bastaba "hay un color en el cuadro" + "hay algo a menos de
        # 700mm delante", sin comprobar que fueran el mismo objeto. Con un
        # color latcheado, cualquier pared que se acercara disparaba la
        # evasion y el robot se iba hacia el lado que manda ese color aunque
        # no hubiera ningun poste que rodear. Medido en la corrida del
        # 2026-08-29 (t=70.1s): entro en APROXIMACION con la pared a 386mm y
        # el rojo latcheado, y se quedo 2 segundos girando a la derecha
        # mientras la distancia frontal CRECIA de 386 a 1224mm -- o sea que
        # nunca hubo nada que esquivar. Encima el color cambio a VERDE a
        # mitad del episodio, pero el lado ya estaba fijado por el ROJO de
        # la entrada.
        #
        # frontal_muro es el mismo sector con los postes descontados, asi
        # que "frontal_muro >> frontal" significa que lo cercano es
        # estrecho. Esta comprobacion solo es fiable desde que
        # es_objeto_estrecho tiene tope angular: antes, de cerca la pared
        # entera contaba como poste y frontal_muro valia 8000.
        estrecho_delante = med.frontal_muro > med.frontal * 1.5
        vision_en_rango = (color_cam is not None
                           and 50.0 < med.frontal < DIST_INICIO_EVASION_CAM
                           and estrecho_delante)

        if poste_en_rango or vision_en_rango:
            color = trk.color if trk.activo and trk.color else color_cam
            self._evadir_por_izquierda = (color == "VERDE")
            self._entrar("APROXIMACION", ahora)
            if self._evadir_por_izquierda:
                self._sector.fijar_sector_frontal(*SECTOR_EVASION_IZQ)
            else:
                self._sector.fijar_sector_frontal(*SECTOR_EVASION_DER)
            lado = "IZQUIERDA" if self._evadir_por_izquierda else "DERECHA"
            print(f"[FSM] CRUCERO -> APROXIMACION | {color} | paso por {lado} | "
                  f"F:{med.frontal:.0f}mm")

        velocidad = self._con_frenado(VELOCIDAD_CRUCERO, med.frontal)
        return (velocidad, self._centrado_paredes(med))

    def _est_aproximacion(self, med, color_cam, heading, ahora):
        trk = self.tracker
        t_en_estado = ahora - self._t_estado

        # Evadiendo por la izquierda el poste queda a la derecha del robot
        lado_poste = "DER" if self._evadir_por_izquierda else "IZQ"
        if trk.activo and (trk.y < Y_POSTE_EN_PASO or trk.al_costado(lado_poste)
                           or trk.superado()):
            self._entrar("SOBREPASO", ahora)
            self._heading_sobrepaso = heading
            print(f"[FSM] APROXIMACION -> SOBREPASO | poste en ({trk.x:.0f},{trk.y:.0f})mm")
        elif t_en_estado > TIMEOUT_APROXIMACION:
            # Sin tracker resuelto asumimos que ya avanzamos lo suficiente
            self._entrar("SOBREPASO", ahora)
            self._heading_sobrepaso = heading
            print("[FSM] APROXIMACION -> SOBREPASO | timeout")

        # Pure pursuit hacia el punto de paso al lado del poste
        if trk.activo:
            signo = -1.0 if self._evadir_por_izquierda else 1.0
            x_obj = trk.x + signo * SEPARACION_LATERAL
            y_obj = max(150.0, trk.y)          # evita el atan2 degenerado de cerca
            bearing = math.degrees(math.atan2(x_obj, y_obj))
            angulo = _clamp(-bearing * KP_PURSUIT, MAX_ANGULO_EVASION)
            self.rama_evasion = "PURSUIT"
            self.rumbo_poste_cam = math.degrees(math.atan2(trk.x, max(1.0, trk.y)))
            self.angulo_ciego_viejo = 0.0
        else:
            # Evasion a ciegas: hay color pero el LiDAR todavia no da un
            # cluster con el que aparearlo, asi que no hay (x, y) del poste.
            #
            # Lo que SI hay es el cx de la camara, o sea su RUMBO. Antes
            # esta rama no lo usaba: aplicaba un sesgo fijo cuyo signo
            # salia solo del color (rojo -> siempre derecha, verde ->
            # siempre izquierda). Eso es incorrecto, porque el color dice
            # por que LADO DEL POSTE hay que pasar, no hacia donde hay que
            # girar. Con el poste ya a un lado, el giro correcto puede ser
            # el contrario: para pasar por la derecha de un rojo que esta
            # a 40 grados a la izquierda hay que ir casi recto, no torcer
            # a la derecha.
            #
            # La regla geometrica es la misma que la del pure pursuit de
            # arriba, pero en angulo en vez de en milimetros: se apunta a
            # un rumbo desplazado del poste hacia el lado por el que toca
            # pasarlo.
            if self._cx_cam is not None:
                self.rumbo_poste_cam = optica.rumbo_de_cx(self._cx_cam)
                signo = -1.0 if self._evadir_por_izquierda else 1.0
                rumbo_obj = self.rumbo_poste_cam + signo * MARGEN_PASO_GRADOS
                angulo = _clamp(-rumbo_obj * KP_PURSUIT, ANGULO_EVASION_CIEGA)
                self.rama_evasion = "CIEGA_RUMBO"
            else:
                # Sin cx no queda mas que el sesgo fijo de siempre
                signo = 1.0 if self._evadir_por_izquierda else -1.0
                angulo = signo * ANGULO_EVASION_CIEGA
                self.rama_evasion = "CIEGA_FIJA"
            # Lo que habria mandado la regla vieja, para poder medir en el
            # CSV cuantas veces y cuanto cambia la decision.
            self.angulo_ciego_viejo = (1.0 if self._evadir_por_izquierda else -1.0) * ANGULO_EVASION_CIEGA

        angulo = self._con_seguridad_pared(angulo, med)
        velocidad = self._con_frenado(VELOCIDAD_EVASION, med.frontal)
        return (max(VELOCIDAD_MINIMA, velocidad), angulo)

    def _est_sobrepaso(self, med, color_cam, heading, ahora):
        t_en_estado = ahora - self._t_estado

        if self.tracker.superado() or t_en_estado > TIMEOUT_SOBREPASO:
            razon = "geometrico" if self.tracker.superado() else "timeout"
            self.tracker.desactivar(f"superado ({razon})")
            self._entrar("REINCORPORACION", ahora)
            self._sector.sector_frontal_normal()
            print(f"[FSM] SOBREPASO -> REINCORPORACION | {razon}")

        # Seguir RECTO mientras el poste pasa por el costado, manteniendo el
        # rumbo con el que se entro aqui.
        #
        # Antes se apuntaba al rumbo ANTERIOR a la evasion, o sea que este
        # estado deshacia el giro de esquiva justo mientras el robot estaba
        # a la altura del poste. Medido en la corrida 3:
        #
        #   t=1.54 APROXIMACION ang= -5.4 trk_x=-215   esquiva progresando
        #   t=2.04 SOBREPASO    ang=+21.3 trk_x=-272   servo al tope opuesto
        #   t=2.44 SOBREPASO    ang=+22.0 trk_x=-195   el poste vuelve al centro
        #
        # Los 278mm de separacion ganados se perdian enteros y el robot se
        # volvia a meter encima del poste. Manteniendo el rumbo de entrada
        # la separacion se conserva y el poste queda atras de verdad; el
        # regreso al carril es trabajo de REINCORPORACION, que ya lo hace
        # por posicion.
        error_h = self._heading_sobrepaso - heading
        angulo = _clamp(error_h * KP_HEADING, 22.0)
        angulo = self._con_seguridad_pared(angulo, med)
        return (VELOCIDAD_EVASION, angulo)

    def _est_reincorporacion(self, med, color_cam, heading, ahora):
        # Volver al centro del carril por POSICION, no por rumbo.
        #
        # Antes este estado anulaba el error contra el rumbo previo a la
        # evasion. El rumbo no dice nada de donde esta el robot
        # dentro del pasillo: se puede cumplir el objetivo entero y acabar
        # pegado a un muro, porque enderezar estando desplazado deja el
        # desplazamiento intacto. Peor aun, el giro de vuelta se hace
        # mientras el robot avanza, asi que la correccion de rumbo se paga
        # con MAS desplazamiento en la direccion contraria. En la corrida 4
        # eso salio sistematico: la mediana de izquierda cae de 561mm en
        # APROXIMACION a 310mm en SOBREPASO, con minimos de 80mm, mientras
        # que derecha no bajo de 257mm en toda la corrida. Siempre se
        # pasaba al mismo lado, y de las seis emergencias, tres fueron por
        # el lateral izquierdo con el frente despejado (una con el frente a
        # 1032mm).
        #
        # El error de centrado (izquierda - derecha) si es una medida de
        # posicion y se anula sola al llegar al medio: no puede sobrepasar
        # como lo hace un lazo de rumbo. Es ademas el mismo control que usa
        # CRUCERO, asi que la salida de la evasion entrega el robot en el
        # estado en que CRUCERO espera recibirlo.
        error_lat = med.izquierda - med.derecha
        t_en_estado = ahora - self._t_estado

        if abs(error_lat) < ERROR_LATERAL_OK or t_en_estado > TIMEOUT_REINCORPORACION:
            razon = ("centrado" if abs(error_lat) < ERROR_LATERAL_OK else "timeout")
            self._entrar("CRUCERO", ahora)
            print(f"[FSM] REINCORPORACION -> CRUCERO | {razon} "
                  f"(error lateral {error_lat:+.0f}mm)")
            return (VELOCIDAD_CRUCERO, self._centrado_paredes(med))

        angulo = self._centrado_paredes(med)
        angulo = self._con_seguridad_pared(angulo, med)
        return (VELOCIDAD_EVASION, angulo)

    def _est_retroceso(self, med, color_cam, heading, ahora):
        t_en_estado = ahora - self._t_estado

        # Salir en cuanto el peligro se despeja. Antes la unica salida era
        # el obstaculo trasero o el timeout, asi que el retroceso corria
        # SIEMPRE los 3.5s enteros: en la corrida 4 los seis episodios
        # salieron por "tiempo maximo", ninguno por otra cosa. Medido en
        # ese CSV, el frente y los laterales quedaban libres a los 1.5-2.4s
        # y el robot seguia retrocediendo con el servo puesto hasta los
        # 3.5s. Ese sobrante es lo que lo reorientaba 54-63 grados por
        # episodio, 324 grados en total, y lo dejaba encarando cualquier
        # cosa al volver a CRUCERO.
        despejado = (med.frontal    > SALIDA_RETROCESO_FRONTAL and
                     med.izquierda  > SALIDA_RETROCESO_LATERAL and
                     med.derecha    > SALIDA_RETROCESO_LATERAL)

        if (med.trasera < EMERGENCIA_TRASERA
                or (t_en_estado > TIEMPO_MIN_RETROCESO and despejado)
                or t_en_estado > TIMEOUT_RETROCESO):
            if med.trasera < EMERGENCIA_TRASERA:
                razon = "obstaculo trasero"
            elif despejado:
                razon = f"despejado en {t_en_estado:.1f}s"
            else:
                razon = "tiempo maximo"

            if self._racha_retroceso >= RACHA_RETROCESO_PARA_FORZAR:
                self._signo_giro_forzado = self._signo_memoria_asimetria
                self._entrar("GIRO_FORZADO", ahora)
                lado = "IZQUIERDA" if self._signo_giro_forzado > 0 else "DERECHA"
                print(f"[FSM] RETROCESO -> GIRO_FORZADO ({razon}) | "
                      f"{self._racha_retroceso} emergencias encadenadas "
                      f"-> giro forzado hacia {lado}")
            else:
                self._entrar("CRUCERO", ahora)
                print(f"[FSM] RETROCESO -> CRUCERO ({razon})")

        # Control P en vivo sobre las diagonales traseras: gira hacia el
        # lado con mas espacio libre medido en ESTE ciclo, no un signo
        # precalculado. Si en pista se ve que gira para el lado
        # equivocado, el arreglo es invertir el signo de KP_RETROCESO,
        # no rediseñar esto -- ver nota al inicio del archivo.
        error  = med.trasera_derecha - med.trasera_izquierda
        angulo = _clamp(error * KP_RETROCESO, MAX_ANGULO_RETROCESO)
        return (VELOCIDAD_REVERSA, angulo)

    def _est_giro_forzado(self, med, color_cam, heading, ahora):
        # Desempate de esquina simetrica -- ver README 8.4 y la nota de
        # cabecera del archivo. A diferencia de todos los demas estados,
        # NO recalcula su decision cada ciclo: el lado (self._signo_giro_
        # forzado) se fijo una sola vez al entrar, en _est_retroceso.
        # Recalcularlo aqui con la misma señal simetrica que causo el
        # atasco lo volveria a poner en 0 y deshace el punto entero de
        # este estado.
        t_en_estado = ahora - self._t_estado

        # Salida geometrica: la pared del lado hacia el que se esta
        # forzando el giro se abrio de verdad (diff a favor de ese lado
        # por encima del ruido), es decir, la esquina dejo de ser
        # simetrica y ya hay una pared real que seguir.
        diff = med.izquierda - med.derecha
        asimetria_recuperada = (diff * self._signo_giro_forzado) > SALIDA_GIRO_FORZADO_ASIMETRIA

        if asimetria_recuperada or t_en_estado > TIMEOUT_GIRO_FORZADO:
            razon = "asimetria recuperada" if asimetria_recuperada else "tiempo maximo"
            # Cuenta como intento de desatascarse, exitoso o no: la
            # proxima racha de RETROCESO (si la hay) empieza de cero, no
            # arrastra los reintentos de este atasco.
            self._racha_retroceso     = 0
            self._t_ultima_emergencia = None
            self._entrar("CRUCERO", ahora)
            print(f"[FSM] GIRO_FORZADO -> CRUCERO ({razon}, {t_en_estado:.1f}s)")
            return (VELOCIDAD_CRUCERO, self._centrado_paredes(med))

        angulo    = self._signo_giro_forzado * ANGULO_GIRO_FORZADO
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

        return (VELOCIDAD_PARQUEO, self._centrado_paredes(med))

    # ==========================================
    # AUXILIARES
    # ==========================================
    def _entrar(self, estado, ahora):
        self.estado    = estado
        self._t_estado = ahora

    def _centrado_paredes(self, med):
        # Control P clasico de centrado entre las dos paredes.
        #
        # El recorte NO es cosmetico. El error es (izq - der) sin acotar, y
        # en las esquinas el carril se abre de verdad hasta 1400mm o mas:
        # con KP_LATERAL eso pide del orden de 200 grados de servo. Como el
        # rate limiter de mas abajo solo mueve el comando 6 grados por
        # ciclo, el valor se pone a rampar hacia ese objetivo imposible y
        # sigue creciendo aunque el error ya este bajando -- windup puro.
        # Medido en pista (corrida 2): el comando llego a -78 grados
        # mientras el error caia de 1456 a 818mm, y a 8.6Hz devolverlo al
        # rango util cuesta unos 2 segundos en los que el servo esta
        # clavado en el tope y el robot no responde. El 60% de la corrida
        # se fue en saturacion por esto.
        ang = (med.izquierda - med.derecha) * KP_LATERAL

        # Asistencia de esquina por angulo_muro (triangulacion perp+diag,
        # lidar_geometria.py). izq-der por si solo puede dar señal casi
        # nula acercandose a una esquina si ambos lados se cierran
        # parejo -- exactamente lo que paso en la sesion del dia 2: el
        # robot fue derecho a una esquina con frontal/izq/der cayendo
        # juntos de ~500 a 110mm en 5s sin apenas girar, hasta disparar
        # EMERGENCIA. angulo_muro detecta la esquina antes y mas fuerte:
        # medido en pista con el robot centrado apuntando a esa misma
        # esquina, perp_izq=234mm pero diag_izq=3000mm (el muro
        # izquierdo se "abrio" -- el haz diagonal ya no lo encuentra),
        # dando angulo_muro=-22 grados estable, mientras que izq-der solo
        # (234-153mm) hubiera pedido apenas +11 grados: la mitad de
        # fuerte y mas tarde. El signo se verifico con esa misma medicion:
        # pared abierta a la izquierda -> angulo_muro negativo -> con el
        # signo invertido da giro positivo (izquierda), hacia donde el
        # pasillo realmente se abre. No hace falta saber el sentido de
        # giro de la pista (sigue la seccion 5.3-C): la señal sale fresca
        # del barrido de cada ciclo, sea cual sea el lado que se abra.
        # Solo cuando hay algo delante que pueda ser una esquina, y sin
        # dejar que la asistencia mande por encima del control de
        # posicion (ver DIST_ASISTENCIA_ESQUINA y MAX_APORTE_ANGULO_MURO).
        if med.frontal_muro < DIST_ASISTENCIA_ESQUINA:
            ang += _clamp(-med.angulo_muro * KP_ANGULO_MURO, MAX_APORTE_ANGULO_MURO)

        return _clamp_servo(self._con_escape_frontal(ang, med))

    def _con_escape_frontal(self, ang, med):
        # Ver la nota de DIST_ESCAPE_FRONTAL. Los dos terminos de arriba se
        # cancelan justo cuando mas falta hacen; esto pone un giro
        # comprometido hacia el lado con mas espacio, con peso creciente
        # segun el frente se cierra, y a plena urgencia manda del todo.
        # Contra la PARED, no contra los postes: un poste de 10cm es mas
        # estrecho que el sector frontal, asi que entra y sale de el al
        # avanzar y hace saltar `frontal` entre ~200 y ~3000mm en ciclos
        # seguidos (24 saltos medidos en una corrida, 83% con un poste
        # confirmado delante). Con `frontal` este escape se encendia y
        # apagaba 17 veces por minuto y metia un 50% mas de temblor en la
        # direccion. Los postes ya los rodea la FSM de evasion; aqui
        # estorban. Ver README 8.5.
        frontal = med.frontal_muro

        if frontal >= DIST_ESCAPE_FRONTAL:
            return ang

        # 0 al empezar a ver la pared, 1 justo en el umbral de emergencia
        urgencia = ((DIST_ESCAPE_FRONTAL - frontal) /
                    (DIST_ESCAPE_FRONTAL - EMERGENCIA_FRONTAL))
        urgencia = max(0.0, min(1.0, urgencia))

        # Hacia el lado con mas espacio. Si las dos paredes estan dentro
        # del ruido del LiDAR el escape no tiene a quien preferir (la
        # esquina simetrica de README 8.4-3): ahi tira de la misma memoria
        # persistente que usa GIRO_FORZADO, para que las dos defensas
        # elijan el MISMO lado y no se peleen entre si.
        diff = med.izquierda - med.derecha
        if abs(diff) > UMBRAL_MEMORIA_ASIMETRIA:
            signo = 1.0 if diff > 0 else -1.0
        else:
            signo = self._signo_memoria_asimetria

        objetivo = signo * ANGULO_ESCAPE_MAX
        return ang * (1.0 - urgencia) + objetivo * urgencia

    def _con_seguridad_pared(self, angulo_deseado, med):
        # La evasion (pure pursuit al poste, rumbo paralelo) no sabe donde
        # esta la pared -- persigue al poste sin mirar el LiDAR lateral.
        # Segun se acerca una pared, este mezclador le va quitando mando a
        # la persecucion y se lo da al centrado normal.
        #
        # Se mira la pared MAS CERCANA, no la del lado hacia el que se
        # gira. Antes era `med.derecha if angulo_deseado < 0 else
        # med.izquierda`, y eso deja un hueco: con el comando ya girando
        # para alejarse, la proteccion se apagaba justo mientras el robot
        # seguia trasladandose hacia la pared por inercia. Medido en la
        # corrida del README 8.6, con la pared izquierda cerrandose:
        #
        #   t=75.46 izq=100 der=946 ang=+0.5   protege (mira izquierda)
        #   t=75.56 izq= 95 der=954 ang=-0.6   deja de proteger (mira derecha)
        #   t=75.86 izq= 80 der=981 ang=-3.6   EMERGENCIA
        #
        # En Ackermann girar no te separa de la pared al instante: hace
        # falta avanzar. Por eso la pared cercana importa aunque ya estes
        # girando para el otro lado.
        pared = min(med.izquierda, med.derecha)
        if pared >= DIST_ALERTA_PARED:
            return angulo_deseado

        # Rampa que SATURA (ver DIST_PARED_CRITICA): a 120mm el centrado
        # manda del todo. La anterior (1 - pared/220) nunca llegaba a 1 y
        # dejaba a la persecucion del poste un 36% del comando incluso en
        # el umbral de emergencia.
        peso_pared = ((DIST_ALERTA_PARED - pared) /
                      (DIST_ALERTA_PARED - DIST_PARED_CRITICA))
        peso_pared = max(0.0, min(1.0, peso_pared))

        mezcla = angulo_deseado * (1.0 - peso_pared) + self._centrado_paredes(med) * peso_pared
        return _clamp(mezcla, MAX_ANGULO_EVASION)

    def _con_frenado(self, velocidad_base, frontal):
        # Rampa lineal de velocidad segun la distancia frontal libre
        if frontal >= DIST_FRENADO_INICIO:
            return velocidad_base
        if frontal <= DIST_FRENADO_MIN:
            return VELOCIDAD_MINIMA
        proporcion = (frontal - DIST_FRENADO_MIN) / (DIST_FRENADO_INICIO - DIST_FRENADO_MIN)
        return int(VELOCIDAD_MINIMA + proporcion * (velocidad_base - VELOCIDAD_MINIMA))

    def _intentar_capturar_poste(self, med, color_cam, heading, cx_cam=None):
        # Crea el tracker cuando camara y LiDAR coinciden en un poste
        # frontal. El apareo va por RUMBO cuando la camara da la posicion
        # del blob (ver el bloque APAREO COLOR <-> CLUSTER): asi el color
        # se pega al poste que la camara realmente esta viendo, no al que
        # casualmente esta mas cerca.
        if color_cam is None or not med.clusters_obstaculo:
            return

        rumbo_cam = None
        if cx_cam is not None:
            rumbo_cam = optica.rumbo_de_cx(cx_cam)

        # Cada candidato lleva DOS rumbos. c[3] es el rumbo desde el
        # LiDAR, que es el que define el sector de busqueda del robot.
        # c[4] es el mismo poste visto desde la CAMARA, que va 100mm mas
        # atras en el mastil: es el unico que se puede comparar contra
        # rumbo_cam. Compararlo contra c[3] mete hasta 5 grados de
        # paralaje a distancia corta (README seccion 5).
        candidatos = []
        for clust in med.clusters_obstaculo:
            cx, cy = centroide_xy_cluster(clust)
            rumbo = math.degrees(math.atan2(cx, cy))
            if cy > 80.0 and abs(rumbo) <= SECTOR_BUSQUEDA_POSTE:
                candidatos.append((cx, cy, math.hypot(cx, cy), rumbo,
                                   optica.rumbo_camara_de_cluster(cx, cy)))
        if not candidatos:
            return

        # El mas cercano, que es lo que se usaba antes; sirve de
        # referencia para avisar cuando el apareo por rumbo cambia la
        # decision (o sea, cuando esto acaba de evitar un error).
        cercano = min(candidatos, key=lambda c: c[2])

        if rumbo_cam is None:
            elegido = cercano
        else:
            en_rumbo = [c for c in candidatos
                        if abs(c[4] - rumbo_cam) <= TOLERANCIA_APAREO_GRADOS]
            if not en_rumbo:
                # La camara ve un color donde el LiDAR no tiene ningun
                # poste. Antes se le encajaba al cluster mas cercano
                # igualmente; ahora no se inventa el apareo. La evasion
                # sigue disponible por vision (_est_crucero mira el color
                # con la distancia frontal), solo que sin tracker.
                return
            elegido = min(en_rumbo, key=lambda c: abs(c[4] - rumbo_cam))

            if elegido is not cercano:
                print(f"[APAREO] camara en {rumbo_cam:+.0f}deg -> poste en "
                      f"{elegido[3]:+.0f}deg ({elegido[2]:.0f}mm); el mas cercano "
                      f"era otro en {cercano[3]:+.0f}deg ({cercano[2]:.0f}mm)")

        self.tracker.iniciar(color_cam, elegido[0], elegido[1], heading)
