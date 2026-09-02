"""Pruebas del parser por bloques del RPLIDAR C1.

El driver es I/O puro, asi que se prueba con un puerto serie simulado. La
prueba central compara el resultado con una reimplementacion literal del
lector byte a byte anterior: sobre un flujo bien formado ambos tienen que
entregar exactamente los mismos barridos.
"""

import unittest
from collections import deque
from unittest.mock import patch

from src.pi3B.ronda_nueva.tests.apoyo import asegurar_pyserial

asegurar_pyserial()

from src.pi3B.comun import lidar_driver  # noqa: E402
from src.pi3B.comun.lidar_driver import LidarDriver  # noqa: E402


def empaquetar(angulo_deg, distancia_mm, inicio=False):
    """Construye un paquete de 5 bytes del modo de barrido estandar."""

    angulo_q6 = int(round(angulo_deg * 64.0)) & 0x7FFF
    distancia_q2 = int(round(distancia_mm * 4.0)) & 0xFFFF
    byte0 = (0x3F << 2) | ((0 if inicio else 1) << 1) | (1 if inicio else 0)
    byte1 = ((angulo_q6 & 0x7F) << 1) | 0x01
    byte2 = (angulo_q6 >> 7) & 0xFF
    return bytes(
        (byte0, byte1, byte2, distancia_q2 & 0xFF, (distancia_q2 >> 8) & 0xFF)
    )


def flujo_de_barridos(barridos):
    partes = []
    for barrido in barridos:
        for indice, (angulo, distancia) in enumerate(barrido):
            partes.append(empaquetar(angulo, distancia, inicio=indice == 0))
    return b"".join(partes)


def barrido_sintetico(muestras=90, distancia_base=1000.0):
    # Los angulos se generan ya sobre la rejilla q6 (1/64 de grado) y las
    # distancias sobre la rejilla q2 (1/4 de mm): asi el barrido esperado es
    # exactamente lo que el protocolo puede transportar y la comparacion no
    # falla por el redondeo del propio formato.
    paso = 360.0 / muestras
    return [
        (
            round(k * paso * 64.0) / 64.0,
            distancia_base + (k % 17) * 10.0,
        )
        for k in range(muestras)
    ]


def barridos_legado(flujo):
    """Reimplementacion literal del lector byte a byte que existia antes."""

    barridos = []
    previo = 0.0
    actual = []
    indice = 0
    total = len(flujo)
    while indice < total:
        byte0 = flujo[indice]
        indice += 1
        if (byte0 & 0x01) == ((byte0 >> 1) & 0x01):
            continue
        if indice + 4 > total:
            break
        byte1, byte2, byte3, byte4 = flujo[indice:indice + 4]
        indice += 4
        if (byte1 & 0x01) != 1:
            continue
        angulo = ((byte2 << 7) | (byte1 >> 1)) / 64.0
        distancia = ((byte4 << 8) | byte3) / 4.0
        if not (0 < distancia < 6000):
            continue
        if angulo < previo and (previo - angulo) > 300.0:
            if actual:
                barridos.append(actual)
                actual = []
        previo = angulo
        actual.append((angulo, distancia))
    return barridos


class _SerialFalso:
    """Puerto simulado: entrega los bloques pedidos y luego se queda mudo."""

    def __init__(self, bloques):
        self.is_open = True
        self._cola = deque(bloques)
        self._buffer = bytearray()
        self.escritos = []
        self.lecturas = 0

    @property
    def in_waiting(self):
        if not self._buffer and self._cola:
            self._buffer.extend(self._cola.popleft())
        return len(self._buffer)

    def read(self, cantidad=1):
        self.lecturas += 1
        if not self._buffer and self._cola:
            self._buffer.extend(self._cola.popleft())
        datos = bytes(self._buffer[:cantidad])
        del self._buffer[:cantidad]
        return datos

    def write(self, datos):
        self.escritos.append(datos)
        return len(datos)

    def reset_input_buffer(self):
        self._buffer.clear()

    def close(self):
        self.is_open = False


def recolectar(driver, bloques):
    """Pasa los bloques por _consumir y devuelve los barridos entregados."""

    entregados = []
    for bloque in bloques:
        entregados.extend(driver._consumir(bloque))
    return entregados


def trocear(datos, tamano):
    return [datos[i:i + tamano] for i in range(0, len(datos), tamano)]


class ParserBloquesTests(unittest.TestCase):
    def test_equivale_al_lector_byte_a_byte_sobre_un_flujo_limpio(self):
        barridos = [barrido_sintetico(120), barrido_sintetico(97, 800.0),
                    barrido_sintetico(150, 2500.0)]
        flujo = flujo_de_barridos(barridos)
        esperados = barridos_legado(flujo)

        # El troceo no debe influir: el corte cae en medio de paquetes.
        for tamano in (7, 64, 513, len(flujo)):
            with self.subTest(tamano=tamano):
                driver = LidarDriver()
                obtenidos = recolectar(driver, trocear(flujo, tamano))
                self.assertEqual(obtenidos, esperados)
                # Los dos primeros barridos se cierran; el tercero queda en la
                # cola parcial hasta que llegue el siguiente wrap-around.
                self.assertEqual(len(obtenidos), 2)
                self.assertEqual(obtenidos[0], barridos[0])
                self.assertEqual(obtenidos[1], barridos[1])

    def test_el_parser_sin_numpy_da_el_mismo_resultado(self):
        # La rama de respaldo se usa si numpy no esta instalado; tiene que ser
        # equivalente, no solo "parecida".
        barridos = [barrido_sintetico(110), barrido_sintetico(83, 900.0),
                    barrido_sintetico(64, 2100.0)]
        bloques = trocear(flujo_de_barridos(barridos), 97)
        con_numpy = recolectar(LidarDriver(), bloques)
        with patch.object(lidar_driver, "_np", None):
            sin_numpy = recolectar(LidarDriver(), bloques)
        self.assertEqual(sin_numpy, con_numpy)
        self.assertEqual(sin_numpy, barridos[:2])

    def test_arrastra_la_cola_entre_bloques_sin_perder_muestras(self):
        barridos = [barrido_sintetico(60), barrido_sintetico(60, 1500.0)]
        flujo = flujo_de_barridos(barridos)
        driver = LidarDriver()
        obtenidos = recolectar(driver, trocear(flujo, 3))
        self.assertEqual(obtenidos, [barridos[0]])
        self.assertEqual(driver._parcial, barridos[1])

    def test_descarta_distancias_y_angulos_imposibles(self):
        limpio = barrido_sintetico(40)
        sucio = list(limpio)
        sucio.insert(10, (12.0, 0.0))         # sin eco
        sucio.insert(20, (30.0, 7000.0))      # fuera del alcance del C1
        sucio.insert(30, (400.0, 900.0))      # angulo imposible (campo de 15 bits)
        flujo = flujo_de_barridos([sucio, barrido_sintetico(10, 1200.0)])
        driver = LidarDriver()
        obtenidos = recolectar(driver, [flujo])
        self.assertEqual(len(obtenidos), 1)
        self.assertEqual(obtenidos[0], limpio)

    def test_resincroniza_tras_ruido_intercalado(self):
        primero = barrido_sintetico(80)
        segundo = barrido_sintetico(80, 1800.0)
        tercero = barrido_sintetico(20, 2200.0)
        flujo = (
            flujo_de_barridos([primero])
            + b"\x00\x00\x00\x00\x00\x00\x00"     # bytes que no son paquetes
            + flujo_de_barridos([segundo, tercero])
        )
        driver = LidarDriver()
        obtenidos = recolectar(driver, [flujo])
        self.assertGreaterEqual(driver.resincronizaciones, 1)
        # El ruido puede llevarse por delante el final del primer barrido, pero
        # el segundo tiene que reaparecer completo y sin muestras inventadas.
        self.assertIn(segundo, obtenidos)
        for barrido in obtenidos:
            for angulo, distancia in barrido:
                self.assertTrue(0.0 <= angulo < 360.0)
                self.assertTrue(0.0 < distancia < 6000.0)

    def test_ruido_inicial_no_impide_encontrar_el_primer_barrido(self):
        barridos = [barrido_sintetico(100), barrido_sintetico(100, 1300.0)]
        flujo = b"\x11\x22\x33" + flujo_de_barridos(barridos)
        driver = LidarDriver()
        obtenidos = recolectar(driver, trocear(flujo, 128))
        self.assertEqual(len(obtenidos), 1)
        self.assertEqual(obtenidos[0][-1], barridos[0][-1])

    def test_el_residuo_no_crece_sin_limite_con_basura_continua(self):
        driver = LidarDriver()
        for _ in range(40):
            self.assertEqual(driver._consumir(b"\x00" * 4096), [])
        self.assertLessEqual(len(driver._residuo), lidar_driver._LIMITE_RESIDUO)


class HiloLecturaTests(unittest.TestCase):
    # Descriptor de respuesta que el C1 envia antes del primer paquete y que
    # el driver descarta al arrancar. Sin el, la prueba perderia las primeras
    # muestras por un artefacto del simulador, no por el parser.
    CABECERA = b"\xa5\x5a\x05\x00\x00\x40\x81"

    def _ejecutar(self, al_barrido, bloques):
        bloques = [self.CABECERA + bloques[0]] + list(bloques[1:])
        puerto = _SerialFalso(bloques)
        driver = LidarDriver()
        pendientes = [len(bloques) + 2]

        def seguir():
            pendientes[0] -= 1
            return pendientes[0] > 0

        with patch.object(
            lidar_driver.serial, "Serial", return_value=puerto
        ), patch.object(lidar_driver.time, "sleep", return_value=None):
            driver.hilo_lectura(seguir, al_barrido)
        return driver, puerto

    def test_entrega_timestamp_solo_a_quien_lo_declara(self):
        flujo = flujo_de_barridos(
            [barrido_sintetico(60), barrido_sintetico(60, 1400.0)]
        )
        recibidos = []

        def con_timestamp(scan, timestamp):
            recibidos.append((len(scan), timestamp))

        driver, _puerto = self._ejecutar(con_timestamp, trocear(flujo, 200))
        self.assertEqual(driver.barridos_entregados, 1)
        self.assertEqual(recibidos[0][0], 60)
        self.assertGreater(recibidos[0][1], 0.0)

        historicos = []
        driver, _puerto = self._ejecutar(
            lambda scan: historicos.append(len(scan)), trocear(flujo, 200)
        )
        self.assertEqual(historicos, [60])

    def test_drena_el_puerto_con_pocas_lecturas(self):
        # La regresion que se esta evitando: una llamada al sistema por byte.
        flujo = flujo_de_barridos(
            [barrido_sintetico(180), barrido_sintetico(180, 1600.0)]
        )
        driver, puerto = self._ejecutar(lambda _scan: None, [flujo])
        self.assertEqual(driver.barridos_entregados, 1)
        self.assertLess(puerto.lecturas, 10)


if __name__ == "__main__":
    unittest.main()
