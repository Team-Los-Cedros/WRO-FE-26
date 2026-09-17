"""Rayos contra paredes y delimitadores reales del simulador; sin hardware."""
import math
import unittest
from simulador import Mundo, Pista, Robot, _segmentos_rect, _corta_rayo
from retorno_parqueo import RetornoParqueo


def escenario(lado=-1, x=-300, lateral=300):
    pista = Pista()
    # Hueco util 330: caras internas x=-165/+165, delimitadores 20x200.
    if lado == -1:
        banda = (1300, 1500)
        robot = Robot(x, 1500 - lateral, 0)
    else:
        banda = (-1500, -1300)
        robot = Robot(x, -1500 + lateral, 0)
    for a, b in ((-185, -165), (165, 185)):
        pista.segmentos.extend(_segmentos_rect(a, banda[0], b, banda[1]))
    return Mundo(pista, robot)


class GeometriaBahia(unittest.TestCase):
    def test_detector_en_ambos_lados(self):
        for lado in (-1, 1):
            with self.subTest(lado=lado):
                mundo = escenario(lado, x=-400)
                r = RetornoParqueo()
                for i in range(5):
                    _, _, hueco = r.percepcion.procesar(mundo.barrido(), i * .1,
                                                        lado_parqueo=lado)
                self.assertIsNotNone(hueco)
                self.assertAlmostEqual(hueco.separacion_mm, 350, delta=25)
                self.assertEqual(hueco.lado, lado)

    def test_un_separador_oculto_se_sigue_por_paredes_medidas(self):
        mundo = escenario(x=-400)
        r = RetornoParqueo()
        r.lado = -1
        r._t_busqueda = 0
        for i in range(5):
            r.observar(mundo.medicion(), 0, i * .1, 0, 0, .1)
        previo = r.hueco
        self.assertIsNotNone(previo)
        mundo.robot.x += 100
        r.observar(mundo.medicion(), 0, .6, 0, 0, .1)
        self.assertAlmostEqual(r.hueco.borde_delantero_y_mm,
                               previo.borde_delantero_y_mm - 100, delta=15)
        self.assertEqual(r.hueco.timestamp, .6)

    def test_captura_origen_con_cinco_barridos(self):
        mundo = escenario()
        r = RetornoParqueo()
        for i in range(5):
            med = mundo.medicion()
            r.observar(med, 0, i * .1, 0, 0, .1)
            capturado = r.capturar_origen(med, 0, i * .1)
            self.assertEqual(capturado, i == 4)
        self.assertIsNotNone(r.origen)

    def test_sensor_frontal_ve_delimitador_dentro(self):
        for lado in (-1, 1):
            mundo = escenario(lado, x=-51, lateral=100)
            r = RetornoParqueo()
            paredes, _, _ = r.percepcion.procesar(mundo.barrido(), 1,
                                                  lado_parqueo=lado)
            self.assertLess(paredes.frontal_min_mm, 120)
            pared = paredes.izquierda if lado < 0 else paredes.derecha
            self.assertIsNotNone(pared)
            self.assertAlmostEqual(pared.distancia_mm, 100, delta=10)


if __name__ == "__main__":
    unittest.main()
