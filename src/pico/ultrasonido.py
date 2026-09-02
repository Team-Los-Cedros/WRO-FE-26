"""Conversion y filtrado de las medidas del ultrasonido trasero.

Como ``protocolo_seguro.py``, este modulo no importa ``machine`` ni ``time``
del hardware: es logica pura y se puede probar en CPython desde la laptop.
``main.py`` aporta el pulso y el cronometraje por interrupcion; aqui solo se
convierte el ancho del eco a milimetros y se limpia el ruido.

Por que hace falta filtrar: el HC-SR04 devuelve de vez en cuando un eco
espurio (rebote en el suelo, en la propia estructura del robot o en una
pared oblicua que le llega tarde). Una sola lectura mala no puede autorizar
ni prohibir una reversa, asi que se publica la mediana de una ventana corta.
"""

# 343 m/s a 20 C, expresado en mm/us. El error por temperatura es del orden
# del 0,2 % por grado; a 300 mm son menos de 1 mm por cada 2 C, muy por
# debajo de la tolerancia de centrado del parqueo (35 mm).
VELOCIDAD_SONIDO_MM_US = 0.343

# Valor unico que viaja en la trama cuando no hay medida fiable. Es negativo
# a proposito: cualquier consumidor que lo trate como distancia lo descarta
# en la primera comprobacion en vez de creerse un cero "muy cerca".
SIN_MEDIDA = -1


def distancia_mm(ancho_pulso_us):
    """Convierte el ancho del eco a distancia. El sonido va y vuelve."""

    try:
        ancho = float(ancho_pulso_us)
    except (TypeError, ValueError):
        return SIN_MEDIDA
    if not ancho > 0.0:
        return SIN_MEDIDA
    return int(ancho * VELOCIDAD_SONIDO_MM_US / 2.0)


def mediana(valores):
    """Mediana de una lista corta; devuelve SIN_MEDIDA si esta vacia."""

    if not valores:
        return SIN_MEDIDA
    ordenados = sorted(valores)
    return ordenados[len(ordenados) // 2]


class FiltroUltrasonido:
    """Ventana movil con mediana y caducidad explicita de la medida.

    ``fallos_max`` evita el peor comportamiento posible de un sensor de
    distancia: seguir publicando la ultima lectura buena cuando en realidad
    ya no ve nada. Tras esa cantidad de intentos sin eco la ventana se vacia
    y se publica SIN_MEDIDA, que aguas arriba significa "no autorizo nada".
    """

    def __init__(self, ventana=3, minima_mm=20, maxima_mm=4000, fallos_max=3):
        self._ventana = max(1, int(ventana))
        self._minima = float(minima_mm)
        self._maxima = float(maxima_mm)
        self._fallos_max = max(1, int(fallos_max))
        self._muestras = []
        self._fallos = 0

    def valor(self):
        return mediana(self._muestras)

    @property
    def fallos_seguidos(self):
        return self._fallos

    def actualizar(self, ancho_pulso_us):
        """Incorpora un eco (o su ausencia) y devuelve la distancia vigente."""

        distancia = distancia_mm(ancho_pulso_us)
        if distancia == SIN_MEDIDA or not (
            self._minima <= distancia <= self._maxima
        ):
            self._fallos += 1
            if self._fallos >= self._fallos_max:
                self._muestras = []
            return self.valor()

        self._fallos = 0
        self._muestras.append(distancia)
        if len(self._muestras) > self._ventana:
            self._muestras.pop(0)
        return self.valor()

    def reiniciar(self):
        self._muestras = []
        self._fallos = 0


__all__ = [
    "FiltroUltrasonido",
    "SIN_MEDIDA",
    "VELOCIDAD_SONIDO_MM_US",
    "distancia_mm",
    "mediana",
]
