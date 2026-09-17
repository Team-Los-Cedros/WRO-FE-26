"""Seguimiento del SIGUIENTE pilar por LiDAR cuando la camara ya no lo ve.

POR QUE `EDAD_COLOR_MAX` VALE 20 s Y NO 5. El reloj del color media el
tiempo desde la ultima vez que la camara confirmo el pilar, y eso castiga
justo el caso para el que existe este modulo. Medido en la corrida
031133: el rojo arrancaba a 24,5 grados, con su caja centrada en cx=567
de 640, y el giro a la izquierda del verde la empujo contra el borde --
la ultima deteccion sale recortada en x1=640, a 0,4 s de arrancar -- y no
volvio a entrar en 20 s. El seguimiento lo llevo medido 3,7 s seguidos
-- sigma 30 mm, barrido de 1 ms -- y en t=5,97 lo tiro por haber cumplido
5,12 s sin re-ver un color que la camara no podia ver.

La continuidad de la identidad NO la garantiza la camara: la garantiza el
cluster. `EDAD_LIDAR_MAX` mata la identidad en cuanto se pierde el
cluster mas de 1,5 s, y ademas cada ciclo exige candidato unico dentro de
la puerta, ancho de poste, pertenencia a la seccion y rechazo de
ambiguedad, con la sigma creciendo mientras solo se predice. El reloj del
color queda como tope duro para que una identidad olvidada no viva para
siempre, no como prueba de frescura.

La caja de camara del pilar lejano desaparece mucho antes que su cluster:
en las cuatro capturas del 16-09 el cluster seguia ahi 38 ciclos mas por
corrida. El observador de `dos_pilares` necesita caja para actualizar, asi
que en esos ciclos solo predecia, y una prediccion no autoriza control.

Aqui el color viaja con la identidad: se fija SOLO con una confirmacion
conjunta camara/LiDAR y a partir de ahi la posicion se mantiene con el
cluster, que no tiene color. Por eso la identidad se protege por otras
vias -- un unico candidato dentro de la puerta, lejos del pilar actual,
dentro de la seccion propia y con ancho de poste.
"""
import math

import dos_pilares as dp
import geometria_robot as geo
import geometria_evasion as gev

PUERTA_MM = 160.0           # radio de asociacion alrededor de la prediccion
SEPARACION_ACTUAL = 220.0   # no se le adjudica el cluster del pilar actual
ANCHO_MIN, ANCHO_MAX = 10.0, 180.0
EDAD_BARRIDO_MAX = 0.30     # un barrido mas viejo que esto no es "ahora"
EDAD_LIDAR_MAX = 1.5        # sin cluster propio, la identidad caduca
EDAD_COLOR_MAX = 20.0       # tope duro de acarreo sin volver a ver el color
SIGMA_MAX = 300.0
RACHA_MINIMA = 3            # asociaciones seguidas antes de fiarse
DISTANCIA_MINIMA = 500.0    # mas cerca lo lleva la FSM, no la anticipacion


class SeguimientoLidar:
    def __init__(self):
        self.color = None
        self.id_actual = None
        self.t_color = None     # cuando camara y LiDAR coincidieron
        self.t_lidar = None     # cuando lo midio el ULTIMO barrido propio
        self.x = self.y = 0.0
        self.sigma = SIGMA_MAX
        self.racha = 0
        self.fuente = None
        self.motivo = "sin_identidad"
        self.en_seccion = False
        self._barrido = None
        self._firma = None

    def _olvidar(self, motivo):
        identidad = self.id_actual
        self.__init__()
        self.id_actual = identidad
        self.motivo = motivo

    def _barrido_nuevo(self, clusters, barrido, ahora):
        """¿Traen los clusters informacion de un barrido que no habia visto?

        Con `barrido` = (secuencia, monotonico) lo decide el sensor. Los
        registros anteriores al 16-09 no lo llevan; en ese caso solo se
        puede distinguir una lista identica, que no prueba frescura y por
        eso no vale para control (ver `autoriza_control`).
        """
        if barrido is None:
            firma = tuple(tuple(c) for c in clusters)
            if firma == self._firma:
                return False
            self._firma = firma
            self._barrido = None
            return True
        secuencia, mono = barrido
        if secuencia == self._barrido:
            return False
        if not 0 <= ahora - mono <= EDAD_BARRIDO_MAX:
            return False
        self._barrido = secuencia
        return True

    def actualizar(self, referencia, clusters, actual, giro, avance, ahora,
                   en_seccion, barrido=None):
        identidad = actual.id if actual.activo else None
        if identidad is not None and identidad != self.id_actual:
            # El pilar de la FSM cambio: la identidad anterior ya no dice
            # cual es "el siguiente". Se empieza de cero.
            self._olvidar("cambio_actual")
            self.id_actual = identidad
        if self.color is not None:
            x, y = geo.lidar_a_eje_trasero(self.x, self.y)
            self.x, self.y = geo.eje_trasero_a_lidar(
                *gev.predecir_pilar(x, y, avance, giro))
            self.sigma += (2.0 + .12 * abs(avance)
                           + .10 * abs(math.radians(giro)) * math.hypot(x, y))
        # Solo una confirmacion conjunta camara/LiDAR puede fijar el color.
        if (referencia.fuente == "lidar" and referencia.confirmaciones >= dp.CONFIRMACIONES
                and referencia.en_seccion and referencia.t is not None
                and 0 <= ahora - referencia.t <= EDAD_BARRIDO_MAX
                and (self.t_color is None or referencia.t > self.t_color)):
            self.color = referencia.color
            self.x, self.y = referencia.x, referencia.y
            self.t_color = referencia.t
            self.t_lidar = ahora if barrido is None else barrido[1]
            self.sigma = referencia.sigma
            self.racha = RACHA_MINIMA
            self.fuente = "camara_lidar"
            self.motivo = "color_confirmado"
            self.en_seccion = True
            self._barrido_nuevo(clusters, barrido, ahora)
            return
        if self.color is None:
            return
        if (not 0 <= ahora - self.t_color <= EDAD_COLOR_MAX or self.sigma >= SIGMA_MAX
                or not 0 <= ahora - self.t_lidar <= EDAD_LIDAR_MAX):
            self._olvidar("caducado")
            return
        self.fuente = "prediccion"
        if not self._barrido_nuevo(clusters, barrido, ahora):
            self.motivo = "sin_barrido_nuevo"
            return
        candidatos = [(math.hypot(x - self.x, y - self.y), x, y)
                      for x, y, w in clusters
                      if all(math.isfinite(v) for v in (x, y, w))
                      and ANCHO_MIN <= w <= ANCHO_MAX
                      and math.hypot(x - self.x, y - self.y) <= PUERTA_MM
                      and (not actual.activo
                           or math.hypot(x - actual.x, y - actual.y) > SEPARACION_ACTUAL)]
        if len(candidatos) != 1:
            self.racha = 0
            self.motivo = "ambiguo" if candidatos else "sin_cluster"
            return
        error, x, y = candidatos[0]
        if not en_seccion(x, y):
            self.racha = 0
            self.motivo = "otra_seccion"
            return
        self.x, self.y = x, y
        self.sigma = 30.0 + .5 * error
        self.t_lidar = ahora if barrido is None else barrido[1]
        self.en_seccion = True
        self.racha += 1
        self.fuente = "lidar_seguimiento"
        self.motivo = "medido" if self.racha >= RACHA_MINIMA else "reconfirmando"

    def proponer(self, candidatos, base, velocidad_mm_s, holgura, ahora,
                 s_lado, anterior=None):
        """Misma eleccion que el observador, con la posicion medida aqui."""
        if self.color is None or velocidad_mm_s <= 0:
            return base
        x, y = geo.lidar_a_eje_trasero(self.x, self.y)
        return dp.preparar_paso(candidatos, base, x, y, s_lado,
                                holgura + self.sigma, anterior, sesgo=None)

    def _identidad_firme(self, ahora):
        """Medida propia, fresca, de un barrido identificado y sin ambiguedad.

        Sin `barrido` real no se sabe si el cluster es de este ciclo o del
        anterior, asi que la reproduccion de capturas viejas mira pero no
        manda. Tampoco vale una prediccion: para eso ya estaba la memoria
        de `dos_pilares`, y lo que faltaba era medir, no recordar mejor.
        """
        return (ahora is not None and self._barrido is not None
                and self.color is not None
                and self.fuente == "lidar_seguimiento"
                and self.racha >= RACHA_MINIMA and self.en_seccion
                and self.sigma < dp.SIGMA_CONTROL_MAX
                and self.t_lidar is not None
                and 0 <= ahora - self.t_lidar <= EDAD_BARRIDO_MAX
                and self.t_color is not None
                and 0 <= ahora - self.t_color <= EDAD_COLOR_MAX
                and self.y > 0)

    def autoriza_control(self, ahora=None):
        """Anticipar el paso del SIGUIENTE pilar, que aun esta lejos."""
        return (self._identidad_firme(ahora)
                and math.hypot(self.x, self.y) >= DISTANCIA_MINIMA)

    def identidad_para_captura(self, ahora=None):
        """(color, x, y) con que fichar un pilar que la camara ya no ve.

        La misma identidad que autoriza la anticipacion, sin el suelo de
        `DISTANCIA_MINIMA`: hace falta justo cuando el pilar ya esta cerca,
        que es donde el seguimiento suelta el testigo confiando en que la
        FSM lo recoja. Si la camara no puede verlo, no lo recoge nadie.

        Medido en la corrida 040743: el rojo vuelve a entrar en el sector
        de busqueda solo en t=4,51 y t=4,62 -- dos decimas -- con el
        tracker libre, identidad firme y un unico cluster en la puerta.
        Sin esto el robot siguio en CRUCERO a plena velocidad y lo dejo
        atras por el lado prohibido.

        El color no se inventa: salio de una confirmacion conjunta
        camara/LiDAR y el cluster lo ha mantenido sin romperse.
        """
        if not self._identidad_firme(ahora):
            return None
        return (self.color, self.x, self.y)
