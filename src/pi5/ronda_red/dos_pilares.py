"""Observacion del siguiente pilar y eleccion acotada de la salida.

Las distancias de control proceden del LiDAR. Una caja lejana sin cluster
puede estimarse respecto a la cercana, suponiendo pilares de igual altura;
esa estimacion se registra en sombra y no autoriza cambios de direccion.
No importa ni abre dispositivos.
"""
from dataclasses import dataclass
import math

import geometria_robot as geo
import geometria_evasion as gev
import optica

EDAD_MAX = 0.30
CONFIRMACIONES = 3
PUERTA_MM = 160.0
PUERTA_GRADOS = optica.TOLERANCIA_APAREO_GRADOS
AMBIGUEDAD_GRADOS = 2.0
SEPARACION_IDENTIDADES = 220.0
SESGO_MAX = 12.0
MEJORA_MINIMA_MM = 20.0   # cambiar de arco tiene que ganar al menos esto
EDAD_ESCALA_MAX = 5.0
MEMORIA_MAX = 5.0
SIGMA_MAX = 300.0
MEMORIA_CONTROL_MAX = 1.5
SIGMA_CONTROL_MAX = 150.0


@dataclass(frozen=True)
class Caja:
    color: str
    x0: float
    y0: float
    x1: float
    y1: float
    score: float

    @property
    def cx(self):
        return (self.x0 + self.x1) / 2.0

    @property
    def alto(self):
        return self.y1 - self.y0


@dataclass(frozen=True)
class Cuadro:
    secuencia: int
    captura_mono: float
    captura_unix: float
    cajas: tuple


def decodificar(salida, escala, dx, dy, ancho, alto, umbral):
    """Deshace las bandas y descarta cajas invalidas o fuera de imagen."""
    cajas = []
    for clase, filas in enumerate(salida):
        if clase >= 2:
            continue
        for fila in filas:
            if len(fila) < 5:
                continue
            y0, x0, y1, x1, score = map(float, fila[:5])
            if not all(math.isfinite(v) for v in (x0, y0, x1, y1, score)):
                continue
            if score < umbral or not 0 <= score <= 1:
                continue
            x0, x1 = ((x0 * 640 - dx) / escala, (x1 * 640 - dx) / escala)
            y0, y1 = ((y0 * 640 - dy) / escala, (y1 * 640 - dy) / escala)
            x0, x1 = max(0., x0), min(float(ancho), x1)
            y0, y1 = max(0., y0), min(float(alto), y1)
            if x1 <= x0 or y1 <= y0:
                continue
            cajas.append(Caja(("ROJO", "VERDE")[clase], x0, y0, x1, y1, score))
    return tuple(sorted(cajas, key=lambda c: c.alto, reverse=True))


def asociar(cajas, clusters):
    """Parejas univocas caja-cluster; ante ambiguedad no asigna color.

    Ambos rumbos se comparan desde la camara, incluido el paralaje.
    Dos cajas nunca reciben el mismo cluster, ni una caja dos clusters.
    """
    aristas = []
    for i, caja in enumerate(cajas):
        for j, (x, y, ancho) in enumerate(clusters):
            if not (10 <= ancho <= 180 and y > 0):
                continue
            error = abs(gev.normalizar_180(
                optica.rumbo_de_cx(caja.cx) - optica.rumbo_camara_de_cluster(x, y)))
            if error <= PUERTA_GRADOS:
                aristas.append((error, i, j))
    parejas = []
    for error, i, j in aristas:
        rivales = [e for e, a, b in aristas if (a == i or b == j) and (a, b) != (i, j)]
        if rivales and min(rivales) - error < AMBIGUEDAD_GRADOS:
            continue
        x, y, _ = clusters[j]
        parejas.append((cajas[i], x, y))
    return parejas


def preparar_paso(candidatos, base, x_eje, y_eje, s_lado, holgura,
                  anterior=None, sesgo=SESGO_MAX, mejora=None):
    """Comando, de entre los admitidos, que cruza el siguiente pilar a su lado.

    `candidatos` ya paso las paredes y el pilar actual en el arbitraje, asi
    que elegir dentro de ese conjunto no relaja ninguna de las dos
    garantias: cambia cual de los comandos YA permitidos se toma.

    Por que el criterio es `cruza_al_lado` y no un coste suave. En la
    corrida 004228 el rojo estaba a 1013mm y 382mm a la derecha con el
    verde todavia por rodear. Reconstruida la pista en marco mundo, entre
    t=3,1 y t=6,0 habia una banda ancha de comandos sostenidos -- de -12 a
    -20 grados -- que dejaba AMBOS pilares de su lado con mas de 40mm. El
    robot solo estuvo dentro de esa banda medio segundo: al quedar el
    verde al costado la regla dejo de mandar, volvio al seguimiento de
    pared y salio de la banda 1,9s antes de necesitarlo. Cuando el rojo
    llego a ser el objetivo, ya no quedaba ningun arco legal.

    Por que el de MAYOR margen y no el mas parecido a `base`. Probado
    sobre la trayectoria reconstruida, quedarse en el borde de la banda
    falla: el conjunto parpadea en el umbral y la consigna persigue a
    `base` de vuelta. El borde mas alejado de `base` es ademas el que
    sobrevive a un error de estimacion del tamaño de sigma.

    Por que `anterior` gana. El limitador del servo mueve 12 grados por
    ciclo; si la eleccion cambia cada ciclo, lo aplicado no es ninguna de
    las dos. Eso es literalmente lo que se registro en APERTURA del rojo:
    -4,5 -> -16,5 -> -5,0 -> -7,5 en cuatro ciclos seguidos.

    Pero conservar `anterior` contra viento y marea tiene su propio coste,
    y se midio en la corrida 025702. El cruce arranco en -15 porque con el
    servo en -5 el limitador no daba mas; al ciclo siguiente -20 ya estaba
    disponible con 183mm de margen contra los 109mm del -15, y la regla lo
    bloqueo tres ciclos seguidos. El margen cayo de 126mm a 67mm y hubo que
    retroceder. Apretar el mismo giro NO es oscilar: es la misma intencion
    con mas margen. Con `mejora` se conserva `anterior` solo mientras
    ningun arco le gane por ese numero de milimetros, de modo que la regla
    sigue frenando los cambios de idea y deja pasar los de grado.

    Y por que lo primero que se mira es `base`: si la consigna que ya
    eligio el arbitraje cruza, no hay nada que preparar. Intervenir ahi
    solo empeoraria la maniobra del pilar actual.

    `sesgo` acota cuanto puede alejarse la eleccion de `base`, y con el
    pilar SIGUIENTE tiene sentido: la maniobra del actual manda y
    anticipar es un extra. Con el pilar ACTUAL del lado prohibido se
    invierte -- no hay nada mas importante que cruzarlo, y `base` viene
    del seguimiento de pared, que no sabe que existe. Ahi se pasa None.
    Quitar el tope no relaja ninguna garantia fisica: el limitador del
    servo ya esta en `candidatos` (el llamador pasa los alcanzables) y
    las paredes tambien. Medido en 015816: con el tope de 12 grados sobre
    una base de +7,5 no se alcanza el arco -15 que cruzaba con 140mm, que
    es exactamente el ciclo que decidia la corrida.
    """
    if gev.cruza_al_lado(x_eje, y_eje, s_lado, base, holgura):
        return base         # la consigna que ya hay sirve: no se toca nada
    admisibles = [c for c in candidatos
                  if (sesgo is None or abs(c - base) <= sesgo)
                  and gev.cruza_al_lado(x_eje, y_eje, s_lado, c, holgura)]
    if not admisibles:
        return base
    mejor = max(admisibles, key=lambda c: (gev.margen_cruce(x_eje, y_eje, s_lado, c),
                                           -abs(c - base)))
    if anterior in admisibles:
        if mejora is None:
            return anterior
        if (gev.margen_cruce(x_eje, y_eje, s_lado, anterior)
                >= gev.margen_cruce(x_eje, y_eje, s_lado, mejor) - mejora):
            return anterior
    return mejor


class SiguientePilar:
    def __init__(self):
        self.secuencia = -1
        self.color = None
        self.x = self.y = 0.0
        self.t = None
        self.confirmaciones = 0
        self.id_actual = None
        self.motivo = "sin_cuadro"
        self.fuente = None
        self.en_seccion = False
        self._escala_visual = None
        self._sesgo_visual = 0.0
        self._t_escala = None
        self.sigma = SIGMA_MAX
        self._origen_memoria = None
        self._seccion_medida = False

    def _vaciar(self, motivo):
        self.color = None
        self.confirmaciones = 0
        self.t = None
        self.motivo = motivo
        self.fuente = None
        self.en_seccion = False
        self._escala_visual = None
        self._t_escala = None
        self.sigma = SIGMA_MAX
        self._origen_memoria = None
        self._seccion_medida = False

    def _conservar_prediccion(self, ahora, motivo):
        if (self.confirmaciones >= CONFIRMACIONES and self.t is not None
                and 0 <= ahora - self.t <= MEMORIA_MAX and self.sigma < SIGMA_MAX):
            self.fuente = "prediccion"
            self.en_seccion = False
            self.motivo = "predicho_" + motivo
            return
        self._vaciar(motivo)

    def _mantener_siguiente(self, cuadro, clusters, ahora, en_seccion):
        """Actualiza una identidad confirmada aunque el actual este degradado.

        Exige nueva evidencia de la misma identidad. No prolonga la vida de
        una prediccion sin caja y no convierte una memoria visual en LiDAR.
        """
        if self.confirmaciones < CONFIRMACIONES or self.color is None:
            return False
        parejas = [(c, x, y) for c, x, y in asociar(cuadro.cajas, clusters)
                   if c.color == self.color and math.hypot(x - self.x, y - self.y) <= PUERTA_MM]
        if len(parejas) == 1:
            _, self.x, self.y = parejas[0]
            self.fuente = "lidar"
            self.en_seccion = en_seccion(self.x, self.y)
            self.sigma = 30.0
        else:
            if (parejas or self._escala_visual is None or self._t_escala is None
                    or not 0 <= ahora - self._t_escala <= EDAD_ESCALA_MAX):
                return False
            candidatos = []
            for c in cuadro.cajas:
                if (c.color != self.color or c.alto < 8 or c.y0 <= 2
                        or c.y1 >= optica.ALTO_FRAME - 2 or c.x0 <= 2
                        or c.x1 >= optica.ANCHO_FRAME - 2):
                    continue
                profundidad = self._escala_visual / c.alto
                rumbo = optica.rumbo_de_cx(c.cx) + self._sesgo_visual
                x = optica.CAM_X + profundidad * math.tan(math.radians(rumbo))
                y = optica.CAM_Y + profundidad
                if math.hypot(x - self.x, y - self.y) <= PUERTA_MM:
                    candidatos.append((x, y))
            if len(candidatos) != 1:
                return False
            self.x, self.y = candidatos[0]
            self.fuente = "visual_memoria"
            self.en_seccion = False
            self.sigma = max(60., .15 * math.hypot(self.x, self.y)) + 10 * (ahora - self._t_escala)
        self.t = cuadro.captura_mono
        self._origen_memoria = self.fuente
        self._seccion_medida = self.en_seccion
        self.confirmaciones += 1
        self.motivo = "confirmado_independiente"
        return True

    def actualizar(self, cuadro, clusters, actual, giro, avance, ahora, en_seccion):
        # Predice incluso si no llega un cuadro nuevo, sin rejuvenecerlo.
        if self.color is not None:
            self.sigma += (2.0 + .12 * abs(avance)
                           + .10 * abs(math.radians(giro)) * math.hypot(self.x, self.y))
            x, y = geo.lidar_a_eje_trasero(self.x, self.y)
            x, y = gev.predecir_pilar(x, y, avance, giro)
            self.x, self.y = geo.eje_trasero_a_lidar(x, y)
        identidad = actual.id if actual.activo else None
        if identidad is not None and identidad != self.id_actual:
            self._vaciar("cambio_actual")
            self.id_actual = identidad
        if self.t is not None and (ahora - self.t > MEMORIA_MAX or self.sigma >= SIGMA_MAX):
            self._vaciar("caducado")
        if cuadro is None or not 0 <= ahora - cuadro.captura_mono <= EDAD_MAX:
            self._conservar_prediccion(ahora, "cuadro_caducado")
            return
        if cuadro.secuencia == self.secuencia:
            return
        self.secuencia = cuadro.secuencia
        if not actual.activo or actual.sigma > 100 or actual.ambiguo:
            if self._mantener_siguiente(cuadro, clusters, ahora, en_seccion):
                return
            self._conservar_prediccion(ahora, "actual_incierto")
            return
        # Ver mas alla de la esquina no autoriza atravesar el muro interior.
        candidatos = []
        # Primero reserva la identidad ya confirmada. Un apareo global por
        # rumbo puede adjudicarle la caja del siguiente si la optica tiene
        # sesgo; eso no debe invalidar la identidad congelada de la FSM.
        cercanos = [(x, y, w) for x, y, w in clusters
                    if math.hypot(x - actual.x, y - actual.y) < PUERTA_MM]
        referencias = asociar(tuple(c for c in cuadro.cajas if c.color == actual.color), cercanos)
        if len(referencias) == 1:
            ref, rx, ry = referencias[0]
            restantes = [(x, y, w) for x, y, w in clusters if (x, y) != (rx, ry)]
            pares = referencias + asociar(tuple(c for c in cuadro.cajas if c is not ref), restantes)
        else:
            pares = asociar(cuadro.cajas, clusters)
        conflicto_actual = any(c.color != actual.color and
                               math.hypot(x - actual.x, y - actual.y) < PUERTA_MM
                               for c, x, y in pares)
        if conflicto_actual:
            self._conservar_prediccion(ahora, "conflicto_color_actual")
            return
        for caja, x, y in pares:
            if math.hypot(x - actual.x, y - actual.y) < SEPARACION_IDENTIDADES:
                continue
            if y < max(200., actual.y + 150.):
                continue
            candidatos.append((y, caja.color, x, y, "lidar", en_seccion(x, y)))
        # Escala relativa medida en ESTE cuadro; nunca una distancia fija
        # inventada para el pilar que el LiDAR no alcanza. El sesgo angular
        # de la referencia se conserva en la extrapolacion y en el registro.
        if len(referencias) == 1:
            ref, rx, ry = referencias[0]
            completa = lambda c: (c.y0 > 2 and c.y1 < optica.ALTO_FRAME - 2
                                  and c.x0 > 2 and c.x1 < optica.ANCHO_FRAME - 2)
            if completa(ref):
                k = (ry - optica.CAM_Y) * ref.alto
                sesgo = optica.rumbo_camara_de_cluster(rx, ry) - optica.rumbo_de_cx(ref.cx)
                self._escala_visual, self._sesgo_visual = k, sesgo
                self._t_escala = cuadro.captura_mono
                for caja in cuadro.cajas:
                    if (any(caja is c for c, _, _ in pares) or not completa(caja)
                            or caja.alto >= .8 * ref.alto or caja.alto < 8):
                        continue
                    # Cajas solapadas no demuestran una segunda identidad.
                    if min(caja.x1, ref.x1) > max(caja.x0, ref.x0):
                        continue
                    profundidad = k / caja.alto
                    rumbo = optica.rumbo_de_cx(caja.cx) + sesgo
                    x = optica.CAM_X + profundidad * math.tan(math.radians(rumbo))
                    y = optica.CAM_Y + profundidad
                    if y >= max(200., actual.y + 150.):
                        candidatos.append((y, caja.color, x, y, "visual_relativa", False))
        candidatos.sort()
        if not candidatos:
            if self._mantener_siguiente(cuadro, clusters, ahora, en_seccion):
                return
            self._conservar_prediccion(ahora, "sin_siguiente_univoco")
            return
        if len(candidatos) > 1 and candidatos[1][0] - candidatos[0][0] < 150:
            self._vaciar("orden_ambiguo")
            return
        _, color, x, y, fuente, seccion = candidatos[0]
        mismo = (color == self.color and math.hypot(x - self.x, y - self.y) <= PUERTA_MM)
        self.confirmaciones = self.confirmaciones + 1 if mismo else 1
        self.color, self.x, self.y, self.t = color, x, y, cuadro.captura_mono
        self.fuente, self.en_seccion = fuente, seccion
        self._origen_memoria, self._seccion_medida = fuente, seccion
        self.sigma = 30.0 if fuente == "lidar" else max(60., .15 * math.hypot(x, y))
        self.motivo = "confirmado" if self.confirmaciones >= CONFIRMACIONES else "confirmando"

    def proponer(self, candidatos, base, velocidad_mm_s, holgura, ahora,
                 s_lado, anterior=None):
        """Comando que prepara el paso del siguiente, si hace falta y cabe.

        `s_lado` lo traduce el llamador con navegacion.lado_obligatorio, que
        es el unico sitio donde el color se convierte en lado.

        Sin tope respecto a `base`, y por lo mismo que en el pilar actual.
        El tope se puso para que anticipar el siguiente no estropease la
        maniobra del que se esta rodeando, pero esa maniobra NO depende de
        el: las paredes y el pilar actual ya estan garantizados porque el
        llamador solo pasa comandos que el arbitraje admitio, y
        `anticipacion_habilitada` ademas exige tener el actual al traves o
        detras. El tope solo era una heuristica de "no te desvies mucho".

        Y costaba la ventana entera. Corrida 031133, con el rojo a 600 mm
        y RECUPERACION ya en marcha: en t=3,68 la anticipacion aplicaba
        -17,5, y en t=3,77 la base bajo a -5,0 y el -17,5 quedo a 12,5
        grados -- fuera por medio grado. Los arcos que cruzaban seguian
        ahi, -20,0 y -17,5, y se soltaron. El robot paso de largo.
        """
        edad_max = MEMORIA_MAX if self.fuente == "prediccion" else EDAD_MAX
        if (self.confirmaciones < CONFIRMACIONES or self.t is None or self.sigma >= SIGMA_MAX
                or not 0 <= ahora - self.t <= edad_max or velocidad_mm_s <= 0):
            return base
        x, y = geo.lidar_a_eje_trasero(self.x, self.y)
        return preparar_paso(candidatos, base, x, y, s_lado,
                             holgura + self.sigma, anterior, sesgo=None)

    def autoriza_control(self, ahora=None, memoria=False):
        if self.fuente == "lidar" and self.en_seccion:
            return True
        return (memoria and ahora is not None and self.t is not None
                and self.fuente == "prediccion" and self._origen_memoria == "lidar"
                and self._seccion_medida and self.confirmaciones >= CONFIRMACIONES
                and 0 <= ahora - self.t <= MEMORIA_CONTROL_MAX
                and self.sigma < SIGMA_CONTROL_MAX
                and math.hypot(self.x, self.y) >= 500.0 and self.y > 0)
