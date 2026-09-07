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
USAR_LINEAS = False
ORDEN_LINEAS = None     # p.ej. ("NARANJA", "AZUL") -> sentido +1


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
