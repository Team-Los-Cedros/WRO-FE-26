import math
import sys
import unittest
from pathlib import Path


RAIZ_REPO = Path(__file__).resolve().parents[4]
if str(RAIZ_REPO) not in sys.path:
    sys.path.insert(0, str(RAIZ_REPO))

from src.pi3B.comun.lidar_geometria import Medicion, construir_perfil_360
from src.pi3B.ronda_nueva.config import cargar_configuracion
from src.pi3B.ronda_nueva.percepcion_lidar import PercepcionLidar


def polar(x_mm, y_mm):
    return math.degrees(math.atan2(x_mm, y_mm)) % 360.0, math.hypot(x_mm, y_mm)


def ordenar_scan(puntos_xy):
    return sorted((polar(x, y) for x, y in puntos_xy), key=lambda punto: punto[0])


def pared_vertical(x_mm, y_min=-900.0, y_max=1050.0, cantidad=40):
    paso = (y_max - y_min) / float(cantidad - 1)
    return [(x_mm, y_min + i * paso) for i in range(cantidad)]


def crear_medicion(scan, clusters=(), derecha=500.0, izquierda=500.0, timestamp=1.0):
    medicion = Medicion(
        frontal=1200.0,
        izquierda=izquierda,
        derecha=derecha,
        trasera=850.0,
        trasera_derecha=900.0,
        trasera_izquierda=910.0,
        clusters=list(clusters),
        perfil=construir_perfil_360(scan),
        angulo_muro=1.5,
        frontal_muro=1450.0,
    )
    medicion.timestamp = float(timestamp)
    return medicion


class PercepcionLidarTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()

    def test_corredor_y_objeto_estrecho_con_ancho_fisico(self):
        puntos = pared_vertical(-510.0) + pared_vertical(490.0)
        scan = ordenar_scan(puntos)
        cluster_poste = [((angulo % 360.0), 700.0) for angulo in range(-4, 5)]
        medicion = crear_medicion(
            scan, clusters=[cluster_poste], derecha=475.0, izquierda=525.0, timestamp=10.0
        )

        resultado = PercepcionLidar(self.config).procesar(scan, medicion)

        self.assertIsNotNone(resultado.corredor.pared_izquierda)
        self.assertIsNotNone(resultado.corredor.pared_derecha)
        self.assertAlmostEqual(resultado.corredor.izquierda_mm, 510.0, delta=8.0)
        self.assertAlmostEqual(resultado.corredor.derecha_mm, 490.0, delta=8.0)
        self.assertAlmostEqual(resultado.corredor.error_lateral_mm, 20.0, delta=12.0)
        self.assertGreater(resultado.corredor.calidad_pared, 0.75)
        self.assertEqual(resultado.corredor.frontal_muro_mm, 1450.0)

        self.assertEqual(len(resultado.objetos), 1)
        objeto = resultado.objetos[0]
        self.assertEqual(objeto.puntos, len(cluster_poste))
        self.assertAlmostEqual(objeto.distancia_mm, 698.6, delta=5.0)
        self.assertGreater(objeto.ancho_mm, 85.0)
        self.assertLess(objeto.ancho_mm, 115.0)

    def test_ajuste_robusto_descarta_outlier_y_no_usa_el_minimo(self):
        puntos = pared_vertical(-600.0) + pared_vertical(600.0)
        puntos.append((120.0, 0.0))  # minimo espurio dentro del sector derecho
        scan = ordenar_scan(puntos)
        medicion = crear_medicion(scan, derecha=120.0, izquierda=600.0, timestamp=20.0)

        resultado = PercepcionLidar(self.config).procesar(scan, medicion)
        pared = resultado.corredor.pared_derecha

        self.assertIsNotNone(pared)
        self.assertGreaterEqual(pared.puntos, self.config["lidar"]["wall_min_points"])
        self.assertAlmostEqual(pared.distancia_mm, 600.0, delta=12.0)
        self.assertAlmostEqual(resultado.corredor.derecha_mm, 600.0, delta=12.0)
        self.assertGreater(pared.calidad, 0.70)

    def test_dos_separadores_magenta_requieren_tres_barridos_distintos(self):
        # Las paredes magenta de la plaza son laterales (paralelas al eje x),
        # miden 200 mm y sus centros estan separados los 353 mm configurados.
        pared_trasera = [(x, -100.0) for x in range(260, 461, 20)]
        pared_delantera = [(x, 253.0) for x in range(260, 461, 20)]
        scan = ordenar_scan(pared_trasera + pared_delantera)
        percepcion = PercepcionLidar(self.config)

        self.assertEqual(len(percepcion.segmentar(scan)), 2)

        m1 = crear_medicion(scan, timestamp=30.0)
        self.assertIsNone(percepcion.procesar(scan, m1, lado_parqueo=1).hueco)
        # Reprocesar el mismo timestamp no puede sumar persistencia.
        self.assertIsNone(percepcion.procesar(scan, m1, lado_parqueo=1).hueco)

        m2 = crear_medicion(scan, timestamp=31.0)
        self.assertIsNone(percepcion.procesar(scan, m2, lado_parqueo=1).hueco)
        m3 = crear_medicion(scan, timestamp=32.0)
        hueco = percepcion.procesar(scan, m3, lado_parqueo=1).hueco

        self.assertIsNotNone(hueco)
        self.assertEqual(hueco.lado, 1)
        self.assertAlmostEqual(hueco.borde_trasero_y_mm, -100.0, delta=8.0)
        self.assertAlmostEqual(hueco.borde_delantero_y_mm, 253.0, delta=8.0)
        self.assertAlmostEqual(hueco.separacion_mm, 353.0, delta=10.0)
        self.assertAlmostEqual(hueco.distancia_lateral_mm, 360.0, delta=12.0)
        self.assertGreater(hueco.confianza, 0.80)

        percepcion.reiniciar()
        self.assertIsNone(
            percepcion.procesar(scan, crear_medicion(scan, timestamp=33.0), lado_parqueo=-1).hueco
        )

    def test_mastil_no_contamina_reversa_y_sin_eco_no_es_via_libre(self):
        scan = [(float(a), 92.0) for a in range(163, 192)]
        scan += [(150.0, 620.0), (151.0, 625.0), (209.0, 640.0), (210.0, 635.0)]
        medicion = crear_medicion(scan, timestamp=40.0)

        corredor = PercepcionLidar(self.config).procesar(scan, medicion).corredor
        self.assertTrue(corredor.trasera_valida)
        self.assertGreater(corredor.trasera_mm, 500.0)
        self.assertNotAlmostEqual(corredor.trasera_mm, 92.0)
        self.assertTrue(corredor.trasera_izquierda_valida)
        self.assertTrue(corredor.trasera_derecha_valida)
        self.assertGreater(corredor.cobertura_trasera_izquierda, 0.0)
        self.assertGreater(corredor.cobertura_trasera_derecha, 0.0)

        vacia = crear_medicion(
            [], timestamp=41.0, izquierda=8000.0, derecha=8000.0
        )
        corredor_vacio = PercepcionLidar(self.config).procesar([], vacia).corredor
        self.assertFalse(corredor_vacio.trasera_valida)
        self.assertFalse(corredor_vacio.izquierda_valida)
        self.assertFalse(corredor_vacio.derecha_valida)
        self.assertEqual(
            corredor_vacio.trasera_mm,
            self.config["lidar"]["rear_no_data_mm"],
        )

    def test_descarta_el_eco_de_la_direccion_como_pared_lateral(self):
        """La rueda entra en el barrido al girar y no es una pared.

        Sin recta ajustada, el lateral cae al minimo crudo del sector, que
        no pasa por ``wall_side_min_mm``. Al girar, el mecanismo de
        direccion aparece por debajo del perimetro del robot: 61 mm por la
        izquierda y 45 por la derecha del eje del LiDAR. Medido en las
        corridas del 2026-08-31 y 09-01, 101 lecturas imposibles a la
        izquierda con el servo mediano en +17 grados y 90 a la derecha con
        -8, cada una capaz de disparar una emergencia falsa."""

        # Barrido sin puntos suficientes para ajustar ninguna recta lateral.
        scan = ordenar_scan([(float(a), 1500.0) for a in range(0, 360, 24)])
        medicion = crear_medicion(scan, derecha=450.0, izquierda=51.0)

        corredor = PercepcionLidar(self.config).procesar(scan, medicion).corredor

        self.assertIsNone(corredor.pared_izquierda)
        self.assertFalse(corredor.izquierda_valida)
        self.assertGreater(corredor.izquierda_mm, 1000.0)
        self.assertTrue(math.isnan(corredor.error_lateral_mm))

    def test_el_umbral_del_fallback_cubre_el_eco_con_el_servo_al_tope(self):
        """El eco de la rueda se aleja segun cuanto gire el volante.

        Medido: 49-51 mm con el servo en +17 grados y 91-107 con el a +25.
        Un umbral de 80 dejaba pasar el segundo caso, que fue el que mato la
        corrida 130404. Y el discriminante es nitido: con recta ajustada la
        izquierda daba 783-830 mm; sin ella, 91-265."""

        scan = ordenar_scan([(float(a), 1500.0) for a in range(0, 360, 24)])
        percepcion = PercepcionLidar(self.config)
        for eco in (51.0, 99.0, 155.0):
            corredor = percepcion.procesar(
                scan, crear_medicion(scan, derecha=450.0, izquierda=eco)
            ).corredor
            self.assertFalse(
                corredor.izquierda_valida,
                "un eco de %.0f mm sin recta no es una pared" % eco,
            )
            self.assertTrue(math.isnan(corredor.error_lateral_mm))

    def test_descarta_la_pared_espuria_mas_cercana_que_el_chasis(self):
        """Ni siquiera valiendose de una recta ajustada.

        En la corrida 134145 la lateral izquierda alternaba 709 mm con
        calidad 0,83 y 58 mm con calidad 0,23 en ciclos consecutivos. El
        segundo valor venia de una recta, no del minimo crudo, asi que el
        filtro anterior lo dejaba pasar y disparaba la emergencia. Una
        pared mas cercana que el propio chasis -61 mm a la izquierda del
        eje- no existe."""

        percepcion = PercepcionLidar(self.config)
        puntos = pared_vertical(-58.0) + pared_vertical(470.0)
        scan = ordenar_scan(puntos)
        corredor = percepcion.procesar(
            scan, crear_medicion(scan, izquierda=58.0, derecha=470.0)
        ).corredor
        self.assertFalse(corredor.izquierda_valida)
        self.assertGreater(corredor.izquierda_mm, 1000.0)
        self.assertTrue(math.isnan(corredor.error_lateral_mm))
