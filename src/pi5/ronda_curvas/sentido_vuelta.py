# Sentido de la vuelta (horario / antihorario) a partir del yaw.
#
# La navegacion anterior no tenia ninguna representacion de esto: usaba
# abs(heading) para contar vueltas y nada mas. Sin sentido de vuelta no
# se puede saber cual es la tangente de SALIDA de una curva, y sin
# tangente de salida el robot que acaba de rodear un pilar en una esquina
# no tiene a que rumbo volver: se queda a merced del centrado de paredes,
# que en la concavidad de una esquina no describe ningun pasillo.
#
# Convencion: yaw de la IMU POSITIVO A LA IZQUIERDA (antihorario).
#   sentido = +1  ->  vuelta ANTIHORARIA (se gira a la izquierda)
#   sentido = -1  ->  vuelta HORARIA     (se gira a la derecha)
#   sentido =  0  ->  todavia sin evidencia
#
# El estimador es deliberadamente lento y con memoria: equivocarse de
# sentido es peor que no saberlo, porque el sistema esta diseñado para
# funcionar sin el (ver `rumbo_salida` mas abajo y el estado
# SALIDA_PILAR de navegacion.py).
import time

from geometria_evasion import normalizar_180

# Yaw neto acumulado que se considera "esto ya no es una evasion". Una
# esquiva de pilar mueve el rumbo 25-40 grados y lo devuelve; una esquina
# de la pista son 90 grados que NO se devuelven. 60 separa los dos casos
# con margen por los dos lados.
UMBRAL_YAW = 60.0

# Ademas del umbral hay que ver el rumbo CRECIENDO en el mismo sentido
# durante este tiempo. Evita fijar el sentido con un pico de ruido o con
# el yaw que deja una emergencia (el RETROCESO reorientaba 54-63 grados
# por episodio en las corridas viejas: justo el orden del umbral).
PERSISTENCIA = 1.5      # s
CRECIMIENTO_MINIMO = 25.0   # grados dentro de la ventana

# Para desdecirse hace falta el doble de evidencia. En una carrera valida
# esto no deberia ocurrir nunca; si ocurre, sale por consola.
FACTOR_CAMBIO = 2.0

# --- Pista de refuerzo por sensor de piso (NO ACTIVADA) ---------------
# El firmware de la Pico ya publica "COLOR:<nombre>" con el TCS3472 en la
# misma trama de telemetria que el yaw, pero enlace_pico.py descarta ese
# campo al parsear (solo se queda con lo anterior a la coma). Las lineas
# de esquina de la pista dan una señal ABSOLUTA del sentido: el orden en
# que se cruzan naranja y azul lo determina sin depender de la IMU.
#
# No se cablea aqui un mapeo inventado. Para activarlo:
#   1. exponer el color en enlace_pico.py (campo COLOR de la trama),
#   2. empujar el robot a mano una vuelta en sentido conocido anotando el
#      orden real de los cruces sobre el tapete de competencia,
#   3. rellenar ORDEN_LINEAS con lo observado y poner USAR_LINEAS = True.
# Mientras tanto el estimador funciona solo con yaw, que es lo que hay
# medido hoy.
# CALIBRADO EN PISTA el 10-09-2026 con `calibrar_lineas.py`: empujando el
# robot a mano en sentido ANTIHORARIO, las lineas de esquina se cruzan en
# el orden AZUL -> NARANJA (dos esquinas, 2.1 s y 0.5 s entre lineas).
# El convenio de `observar_linea` es que ORDEN_LINEAS corresponde a
# sentido +1, y +1 es antihorario.
#
# Esto NO es una constante de diseño: es una observacion del tapete y del
# montaje del sensor. Si se gira el sensor o se cambia de tapete, se
# vuelve a correr `calibrar_lineas.py`.
USAR_LINEAS = True
ORDEN_LINEAS = ("AZUL", "NARANJA")     # cruzar en este orden -> sentido +1


class SentidoVuelta:
    def __init__(self):
        self.sentido = 0
        self.confianza = 0.0
        self._muestras = []          # [(t, heading)]
        self._t_signo = None
        self._signo_visto = 0
        self._ultima_linea = None

    def actualizar(self, heading, ahora=None, congelado=False):
        """Una llamada por ciclo. `congelado` la salta sin borrar nada.

        La navegacion congela el estimador mientras hay una maniobra de
        pilar en curso: el yaw de una envolvente es de la maniobra, no de
        la pista, y meterlo aqui es justo como se cuela un sentido falso.
        """
        ahora = time.time() if ahora is None else ahora
        if congelado:
            return self.sentido

        self._muestras.append((ahora, heading))
        while self._muestras and ahora - self._muestras[0][0] > PERSISTENCIA:
            self._muestras.pop(0)

        signo = 1 if heading > 0 else (-1 if heading < 0 else 0)
        if signo != self._signo_visto:
            self._signo_visto = signo
            self._t_signo = ahora

        if abs(heading) < self._umbral_actual():
            return self.sentido
        if self._t_signo is None or (ahora - self._t_signo) < PERSISTENCIA:
            return self.sentido
        if len(self._muestras) < 3:
            return self.sentido

        crecimiento = (abs(heading) - abs(self._muestras[0][1]))
        if crecimiento < CRECIMIENTO_MINIMO:
            return self.sentido

        if self.sentido == 0:
            self.sentido = signo
            self.confianza = 1.0
            print("[SENTIDO] vuelta %s (yaw %.0f grados)"
                  % ("ANTIHORARIA" if signo > 0 else "HORARIA", heading))
        elif signo != self.sentido:
            print("[SENTIDO] !! cambio de sentido a %s con yaw %.0f. "
                  "En una carrera valida esto no deberia pasar."
                  % ("ANTIHORARIA" if signo > 0 else "HORARIA", heading))
            self.sentido = signo
        return self.sentido

    def observar_linea(self, color):
        # Gancho para el sensor de piso. Inactivo mientras USAR_LINEAS
        # sea False (ver la nota de arriba): prefiero no tener dato a
        # tener uno inventado.
        if not USAR_LINEAS or color is None or ORDEN_LINEAS is None:
            return self.sentido
        if color == self._ultima_linea:
            return self.sentido
        anterior, self._ultima_linea = self._ultima_linea, color
        if anterior is None:
            return self.sentido
        if (anterior, color) == tuple(ORDEN_LINEAS):
            self.sentido = 1
        elif (color, anterior) == tuple(ORDEN_LINEAS):
            self.sentido = -1
        return self.sentido

    def _umbral_actual(self):
        return UMBRAL_YAW * (FACTOR_CAMBIO if self.sentido != 0 else 1.0)

    @property
    def conocido(self):
        return self.sentido != 0

    def rumbo_salida(self, heading_entrada, en_esquina):
        """Rumbo al que hay que volver despues de rodear el pilar.

        En recta es el rumbo con el que se entro a la maniobra. En una
        esquina el pasillo gira 90 grados HACIA el sentido de la vuelta,
        asi que la tangente de salida es la de entrada mas 90 grados con
        el signo del sentido.

        Devuelve None cuando el sentido no se conoce todavia y hay
        esquina: ahi no se inventa una referencia -- SALIDA_PILAR cae a
        seguir las paredes, que es lo que hace CRUCERO y no depende de
        saber el sentido.
        """
        if not en_esquina:
            return heading_entrada
        if self.sentido == 0:
            return None
        return normalizar_180(heading_entrada + self.sentido * 90.0)


# ==========================================================
# DETECCION DEL SENTIDO POR GEOMETRIA DE PISTA
# ==========================================================
# El sentido de la vuelta se SORTEA en la competicion, asi que no puede
# salir de ninguna constante. Y tampoco puede salir del yaw del propio
# robot: `SentidoVuelta.actualizar` deduce el sentido de cuanto ha girado
# el robot, asi que si el robot gira a un lado por el motivo que sea, el
# estimador lo bendice y el control de esquina lo manda girar mas hacia
# ese lado. Medido el 10-09 en cinco corridas seguidas: `sentido` salio
# ANTIHORARIO en el 76-93% de los ciclos y la relacion de giro
# izquierda/derecha fue de 4:1 a 6:1 en todas, con la pista montada en
# sentido HORARIO. El robot no medía el sentido: lo imponia.
#
# Lo que SI es de la pista: en un anillo, el pasillo dobla siempre hacia
# el mismo lado, y ese lado es donde esta la ISLA central. Dos evidencias
# lo delatan, y las dos son del barrido de este ciclo:
#
#   FUERTE  un sector lateral se dispara muy por encima de la anchura del
#           carril: ese muro SE ACABO, y el unico muro que se acaba en un
#           anillo es el interior. Por ahi dobla.
#   DEBIL   de las dos diagonales frontales, el pasillo continua por donde
#           el haz llega mas lejos. Medido en pista con el robot encarando
#           una curva a 686 mm: 669 mm por la izquierda contra 812 por la
#           derecha -- ordenacion correcta, pero solo un 21% de diferencia,
#           asi que como voto suelto no basta y hay que acumular.
#
# Por eso esto no decide por ciclo: acumula votos con peso y se COMPROMETE
# una sola vez, cuando la evidencia es concluyente. Despues queda
# enclavado: el sentido de una vuelta no cambia a mitad de carrera.
#
# Y mientras no haya compromiso NO SE INVENTA UN LADO. El termino de
# esquina se queda apagado, el robot se centra entre paredes y frena
# contra la pared frontal -- que es seguro y ademas es justo la situacion
# en la que la evidencia FUERTE aparece sola, porque acercarse a la
# esquina es lo que hace que el muro interior se acabe dentro del sector.

# Por encima de esto, ese sector lateral ya no es la pared del carril:
# es su final. Sale de la FISICA de la pista, no de copiar otra constante:
# el pasillo mide ANCHO_CARRIL (1000 mm) y el sector lateral toma el
# MINIMO, que es esencialmente la distancia perpendicular. Un minimo por
# encima de la anchura del pasillo solo lo produce un muro que se acaba.
# Y es inmune a los pilares por construccion: un poste solo puede ACORTAR
# una lectura, nunca alargarla.
#
# Estaba en 1300 (copiado de navegacion.UMBRAL_APERTURA_ESQUINA) y medido
# sobre las tres corridas del 10-09 eso se superaba en el 0,1% de los
# ciclos: el detector no se comprometia nunca por esta via. Con 1000 se
# supera en el 2-5%, que es la cola que corresponde a estar en una
# esquina. p90 = 900 mm, p99 = 1021-1182.
UMBRAL_MURO_ACABADO = 1000.0

# Voto que aporta cada evidencia. El final de muro es una observacion
# geometrica inequivoca; la diagonal es una tendencia.
VOTO_MURO_ACABADO = 1.0
# LA DIAGONAL YA NO VOTA (era 0.25). Un poste ACORTA el sector en el que
# esta, asi que la razon entre diagonales se puede invertir con un solo
# pilar bien puesto. Medido en simulacion con 8 pilares: el detector se
# comprometia al sentido EQUIVOCADO, y con un poste ademas justo delante
# no se comprometia en 45 s -- lo que deja el termino de esquina apagado
# toda la corrida, que es igual de inutil.
#
# El final de muro no tiene ese problema, y la razon es asimetrica: un
# poste solo puede ACORTAR una lectura, nunca alargarla. Un sector que
# se dispara por encima de 1300 mm no lo puede fabricar un pilar; solo
# lo produce el muro interior acabandose. Es la unica evidencia
# geometrica que se sostiene con la pista llena.
VOTO_DIAGONAL     = 0.0

# Razon minima entre las dos diagonales para que la debil vote. 1.15 es
# holgado frente al ruido del C1 (2-9 mm de dispersion) y por debajo de
# la razon medida en pista encarando una curva (812/669 = 1.21).
RAZON_DIAGONAL_MIN = 1.15

# Evidencia acumulada necesaria para comprometerse. Con la evidencia
# fuerte bastan tres barridos; solo con diagonales hacen falta doce.
# Tres barridos con evidencia FUERTE. Ya no hay evidencia debil que
# pueda acumularse hasta el umbral por su cuenta.
EVIDENCIA_PARA_COMPROMISO = 3.0

# Cuanto se espera a las lineas antes de dejar que la geometria decida.
# Es una red de seguridad para el caso de sensor de piso caido: en una
# vuelta normal se cruza una pareja de lineas en la primera esquina,
# mucho antes de esto.
SEGUNDOS_SIN_LINEA = 20.0

# Las dos lineas de UNA esquina se cruzan seguidas. Por encima de esto
# son lineas de esquinas distintas y no forman pareja.
#
# Estaba en 3,0 s, calibrado empujando el robot A MANO (0,5 y 2,1 s entre
# lineas) -- y conduciendo solo va mas despacio, sobre todo maniobrando.
# En la corrida de 5 bloques la primera pareja llego con 4,2 s de
# separacion y se rechazo por 1,2 s; el sentido acabo decidiendolo la red
# geometrica a los 20 s, y mal.
#
# Medido sobre esa corrida, los dos casos se separan solos:
#   misma esquina      1,1  2,1  4,2  4,6 s
#   esquinas distintas 12,6 13,7 15,2 15,3 26,0 s
# 6,0 parte el hueco por la mitad con margen por los dos lados.
VENTANA_PAREJA = 6.0

# Parejas seguidas que hacen falta para desdecir un sentido que YA se
# fijo por lineas.
PAREJAS_PARA_CAMBIAR = 2


# La evidencia vieja pesa menos: si el robot cambia de zona, lo que
# observo hace veinte segundos no deberia decidir.
DECAIMIENTO_POR_CICLO = 0.98


class SentidoPorGeometria:
    """Decide el sentido de la vuelta mirando la PISTA, no el robot.

    Una sola decision por carrera, y enclavada. `sentido` vale 0 mientras
    no haya compromiso, y quien pregunta tiene que saber tratar ese 0
    como "todavia no lo se" -- nunca como un lado.
    """

    def __init__(self):
        self.sentido = 0
        self.evidencia = 0.0
        self.barridos = 0
        self._contradicciones = 0
        self._ultima_linea = None
        self.por_lineas = False
        self._t_inicio = None
        self._t_ultima_linea = None
        self._contra_lineas = 0

    def observar_linea(self, color, ahora=None, retrocediendo=False):
        """Evidencia ABSOLUTA: el orden de las lineas de esquina.

        Es la unica fuente que no depende ni del yaw del robot ni de
        interpretar la geometria: las lineas naranja y azul estan pintadas
        en la pista y su ORDEN al cruzarlas determina el sentido sin
        ambiguedad. Por eso, cuando existe, se compromete de golpe.

        Requiere calibrar `ORDEN_LINEAS` una vez con `calibrar_lineas.py`
        (empujar el robot una vuelta a mano en sentido conocido). Sin esa
        calibracion no se inventa nada: se ignora.
        """
        # OJO: aqui NO se sale si ya hay sentido. Las lineas son la unica
        # evidencia absoluta, asi que pueden CORREGIR un compromiso
        # geometrico equivocado -- que es justo lo que paso con 8 bloques
        # y lo que el contraste por yaw solo podia denunciar, no arreglar.
        if not USAR_LINEAS or ORDEN_LINEAS is None:
            return self.sentido
        # 1. RETROCEDIENDO NO SE MIRA. Marcha atras sobre una linea la
        #    cruza en el orden inverso, o sea que dice el sentido
        #    contrario del que lleva la pista.
        if retrocediendo:
            return self.sentido
        if color not in ("NARANJA", "AZUL") or color == self._ultima_linea:
            return self.sentido
        ahora = time.time() if ahora is None else ahora
        anterior, t_ant = self._ultima_linea, self._t_ultima_linea
        self._ultima_linea, self._t_ultima_linea = color, ahora
        if anterior is None:
            return self.sentido
        # 2. LA PAREJA TIENE QUE SER DE LA MISMA ESQUINA. Sin esto se
        #    emparejaba la naranja de una esquina con la azul de la
        #    SIGUIENTE, y el sentido oscilaba: medido en pista, cinco
        #    cambios en una sola corrida.
        if t_ant is None or (ahora - t_ant) > VENTANA_PAREJA:
            return self.sentido
        if (anterior, color) == tuple(ORDEN_LINEAS):
            nuevo = 1
        elif (color, anterior) == tuple(ORDEN_LINEAS):
            nuevo = -1
        else:
            return self.sentido
        if nuevo == self.sentido:
            self._contra_lineas = 0
            self.por_lineas = True
            return self.sentido
        # 3. CAMBIAR UN COMPROMISO YA HECHO POR LINEAS EXIGE DOS PAREJAS
        #    SEGUIDAS. Corregir a la geometria es inmediato (la linea es
        #    mejor evidencia); desdecir a otra LINEA no, porque entonces
        #    cualquier cruce raro -- una maniobra sobre una esquina, una
        #    linea rozada de lado -- vuelve a mover el sentido.
        if self.por_lineas:
            self._contra_lineas += 1
            if self._contra_lineas < PAREJAS_PARA_CAMBIAR:
                print("[SENTIDO] pareja %s->%s contradice; hacen falta %d seguidas"
                      % (anterior, color, PAREJAS_PARA_CAMBIAR))
                return self.sentido
        self._contra_lineas = 0
        print("[SENTIDO] pista %s por LINEAS del suelo (%s -> %s)%s"
              % ("ANTIHORARIA" if nuevo > 0 else "HORARIA", anterior, color,
                 "  << CORRIGE a la geometria" if not self.por_lineas
                 and self.sentido != 0 else ""))
        self.sentido = nuevo
        self.por_lineas = True
        return self.sentido

    def observar(self, med, ahora=None):
        """Evidencia geometrica. RED DE SEGURIDAD, no fuente principal.

        Se intento cuatro veces hacer de esto la fuente principal y no
        converge, y conviene que quede escrito por que en vez de seguir
        moviendo umbrales:

          * la razon entre diagonales frontales la invierte UN pilar bien
            puesto -- con 8 bloques el detector se comprometio al sentido
            contrario en 1,4 s;
          * el "final de muro" es inmune a los pilares (un poste solo
            acorta), pero los sectores laterales son el MINIMO sobre 60
            grados, y con la isla en medio ese minimo casi nunca supera la
            anchura del pasillo aunque el muro se haya acabado. Medido
            sobre tres corridas reales: max(izq,der) supera 1300 mm en el
            0,1% de los ciclos y 1000 mm en el 2-5%.

        La señal ABSOLUTA existe y funciona: las lineas naranja y azul del
        tapete, que son el marcador de sentido del propio reglamento.
        Comprobado el 10-09: el TCS3472 da NARANJA en 156 de 156 lecturas
        quieto sobre la linea, y cruzo 6 naranjas y 6 azules en una
        corrida. Asi que las lineas mandan, y esto solo entra si tras
        SEGUNDOS_SIN_LINEA no se ha cruzado ninguna pareja -- sensor
        sucio, mal montado o desconectado.
        """
        if self.sentido != 0:
            return self.sentido
        ahora = time.time() if ahora is None else ahora
        if self._t_inicio is None:
            self._t_inicio = ahora
        if (ahora - self._t_inicio) < SEGUNDOS_SIN_LINEA:
            return 0
        # NO se filtra por "hay algo delante". Se intento con
        # `frontal < 900` para proteger el voto de las diagonales, y era
        # un error de bulto: en un anillo de pasillo 1000 mm siempre hay
        # una esquina delante -- `frontal` tuvo mediana 682 mm en la
        # corrida real -- asi que descartaba casi todos los barridos y el
        # detector no se comprometia nunca, ni con la pista vacia.
        # Sin voto de diagonal, la unica evidencia es el final de muro,
        # que un pilar no puede falsificar.
        self.barridos += 1
        self.evidencia *= DECAIMIENTO_POR_CICLO

        voto = 0.0
        abre_izq = med.izquierda > UMBRAL_MURO_ACABADO
        abre_der = med.derecha > UMBRAL_MURO_ACABADO
        if abre_izq != abre_der:
            voto += VOTO_MURO_ACABADO * (1.0 if abre_izq else -1.0)

        d_izq, d_der = self._diagonales(med)
        if d_izq is not None and d_der is not None:
            mayor, menor = max(d_izq, d_der), min(d_izq, d_der)
            if menor > 1.0 and mayor / menor >= RAZON_DIAGONAL_MIN:
                voto += VOTO_DIAGONAL * (1.0 if d_izq > d_der else -1.0)

        self.evidencia += voto
        if abs(self.evidencia) >= EVIDENCIA_PARA_COMPROMISO:
            self.sentido = 1 if self.evidencia > 0 else -1
            print("[SENTIDO] pista %s por geometria (evidencia %+.1f en %d barridos)"
                  % ("ANTIHORARIA" if self.sentido > 0 else "HORARIA",
                     self.evidencia, self.barridos))
        return self.sentido

    @staticmethod
    def _diagonales(med):
        """Diagonales frontales CON LOS PILARES DESCONTADOS.

        Este era el agujero: `frontal_muro` y `angulo_muro` ya descuentan
        los objetos estrechos de sus sectores, y esta medida no lo hacia.
        Un poste delante mete su eco en una de las dos diagonales, la
        acorta, y el detector concluye que el pasillo sigue por el otro
        lado.

        Medido en la corrida de 8 bloques del 10-09: con el pilar #1 a
        347 mm justo delante, el detector se comprometio en 14 barridos
        (1,4 s) a ANTIHORARIA en una pista HORARIA. Como el compromiso
        queda enclavado, el 63% de los ciclos de esa corrida tomaron las
        esquinas hacia el lado equivocado: la distancia recorrida cayo de
        16,0 a 8,3 m y RETROCESO subio del 0,6% al 16,6%.
        """
        from lidar_geometria import (distancia_en_rango_sin_bins,
                                     bins_de_clusters)
        if not getattr(med, "perfil", None):
            return None, None
        excl = bins_de_clusters(getattr(med, "clusters_estrechos", []) or [])
        return (distancia_en_rango_sin_bins(med.perfil, 300, 350, excl),
                distancia_en_rango_sin_bins(med.perfil, 10, 60, excl))

    def contradice(self, sentido_yaw):
        # El yaw pasa a ser CONTRASTE, no fuente. Si dice lo contrario que
        # la geometria de forma sostenida, hay algo mal y conviene verlo
        # en la consola -- pero no se le hace caso.
        if self.sentido == 0 or sentido_yaw == 0 or sentido_yaw == self.sentido:
            self._contradicciones = 0
            return False
        self._contradicciones += 1
        if self._contradicciones == 30:
            print("[SENTIDO] !! el yaw dice %s y la pista dice %s. Se hace caso "
                  "a la pista." % ("ANTIHORARIA" if sentido_yaw > 0 else "HORARIA",
                                   "ANTIHORARIA" if self.sentido > 0 else "HORARIA"))
        return True

    def fijar_por_camara(self, color):
        """La linea de suelo que la camara ve MAS CERCA fija el sentido.

        Idea del equipo, y es la fuente mas fuerte que tenemos: las lineas
        estan pintadas en la pista y cual se encuentra primero depende
        UNICAMENTE del sentido de la vuelta. No depende del yaw (que se
        corrompe con cada maniobra) ni de interpretar la geometria (que es
        lo que venia fallando en pista).

        Hace falta porque el sensor de piso apenas las ve: medido el
        11-09, 35 lecturas AZUL y 13 NARANJA en 1310 ciclos, y en otra
        corrida UNA sola. La camara, en cambio, las ve cruzando el frame
        entero (2500 px de blob) y distingue cual esta mas abajo -- o sea
        mas cerca -- sin ambiguedad.

        Se fija UNA vez y no se revisa: es evidencia absoluta.
        """
        if self.por_lineas or color not in ("NARANJA", "AZUL"):
            return False
        self.sentido = -1 if color == "NARANJA" else 1
        self.evidencia = 99.0
        self.por_lineas = True
        print("[SENTIDO] %s por CAMARA: la linea %s es la mas cercana"
              % ("HORARIA" if self.sentido < 0 else "ANTIHORARIA", color))
        return True

    @property
    def conocido(self):
        return self.sentido != 0
