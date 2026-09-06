"""Percepcion LiDAR contra una pista sintetica con oclusion correcta."""

import math
import unittest

from ..percepcion_lidar import PercepcionLidar, ajustar_recta, segmentar
from .apoyo import barrido_sintetico, config_minima, pose_en_carril


class PruebaParedes(unittest.TestCase):
    def setUp(self):
        self.percepcion = PercepcionLidar(config_minima())

    def test_las_cuatro_paredes_en_los_cuatro_segmentos(self):
        """Barrido de todas las poses razonables, en los dos ejes.

        Este es el test que atrapo dos errores de diseño: ajustar por sector
        angular fijo (la ventana frontal se comia el muro lateral en un carril
        de 1000 mm) y quedarse con el tramo mas largo en vez del mas cercano
        (elegia el muro exterior del otro extremo del campo, a 2,7 m, teniendo
        el bloque interior a 0,7).
        """

        casos = [(1200, 400, 0.0), (900, 300, 7.0), (2400, 500, -5.0),
                 (700, 700, 0.0), (2000, 850, 3.0), (500, 200, -10.0)]
        for segmento in range(4):
            for avance, offset, error in casos:
                x, y, rumbo = pose_en_carril(segmento, avance, offset, error)
                scan = barrido_sintetico(x, y, rumbo, ruido_mm=5.0, semilla=segmento)
                paredes, _objetos, _hueco = self.percepcion.procesar(scan, 1.0)
                with self.subTest(segmento=segmento, avance=avance, offset=offset):
                    self.assertIsNotNone(paredes.frontal)
                    self.assertIsNotNone(paredes.izquierda)
                    self.assertAlmostEqual(paredes.frontal.distancia_mm, avance, delta=30.0)
                    self.assertAlmostEqual(paredes.izquierda.distancia_mm, offset, delta=30.0)
                    self.assertAlmostEqual(paredes.frontal.angulo_deg, -error, delta=3.0)

    def test_al_empezar_la_recta_el_frontal_no_es_el_bloque_interior(self):
        """A 2400 mm del muro, la cara del bloque interior queda a 400.

        Sin exigir que el tramo cruce el eje del robot, esa cara se clasificaba
        como muro frontal y el avance salia 400 en vez de 2400: el robot
        habria disparado el giro nada mas entrar en la recta.
        """

        x, y, rumbo = pose_en_carril(0, 2400.0, 500.0)
        paredes, _o, _h = self.percepcion.procesar(barrido_sintetico(x, y, rumbo), 1.0)
        self.assertAlmostEqual(paredes.frontal.distancia_mm, 2400.0, delta=30.0)

    def test_la_distancia_de_un_punto_a_la_pared_usa_la_recta(self):
        x, y, rumbo = pose_en_carril(0, 1200.0, 400.0)
        paredes, _o, _h = self.percepcion.procesar(barrido_sintetico(x, y, rumbo), 1.0)
        # Un punto 250 mm a la derecha del robot esta a 650 del muro exterior.
        self.assertAlmostEqual(
            paredes.izquierda.distancia_desde(250.0, 800.0), 650.0, delta=25.0
        )

    def test_los_hombros_reconstruyen_la_trasera_tapada_por_el_mastil(self):
        """La mascara 140..213 deja la trasera solo en sus hombros.

        Es la geometria medida del Pi 5.  Con los antiguos offsets 18..35 los
        dos hombros caian en la mascara; 40..60 usa los haces visibles y no
        convierte el mastil de 50 mm en un obstaculo trasero.
        """

        distancia = 1120.0
        barrido = []
        for angulo in range(115, 246, 2):
            coseno = math.cos(math.radians(angulo))
            if coseno < 0.0:
                barrido.append((float(angulo), -distancia / coseno))
        # Los laterales cercanos entran tambien por los bordes de los hombros.
        # Su proyeccion axial es menor y no pueden convertirse en la trasera.
        barrido.extend([(121.0, 540.0), (123.0, 550.0), (237.0, 620.0)])
        config = config_minima()
        config["lidar"] = {
            "blind_sectors_deg": [[140.0, 213.0]],
            "rear_axis_deg": 180.0,
            "rear_shoulder_offset_deg": [40.0, 60.0],
            "rear_min_valid_points": 2,
            "rear_shoulder_wall_fraction": 0.8,
            "wall_normal_window_deg": 42.0,
            "wall_max_residual_mm": 75.0,
        }
        paredes, _objetos, _hueco = PercepcionLidar(config).procesar(barrido, 1.0)

        self.assertIsNotNone(paredes.trasera)
        self.assertAlmostEqual(paredes.trasera.distancia_mm, distancia, delta=10.0)
        self.assertAlmostEqual(paredes.trasera_min_mm, distancia, delta=10.0)


class PruebaObjetos(unittest.TestCase):
    def setUp(self):
        self.percepcion = PercepcionLidar(config_minima())

    def test_un_pilar_aparece_como_objeto_estrecho(self):
        x, y, rumbo = pose_en_carril(0, 1200.0, 400.0)
        px, py, _ = pose_en_carril(0, 400.0, 650.0)
        scan = barrido_sintetico(x, y, rumbo, pilares=[(px, py, 100.0)])
        _paredes, objetos, _hueco = self.percepcion.procesar(scan, 1.0)
        self.assertTrue(objetos)
        pilar = objetos[0]
        self.assertLess(abs(pilar.x_mm - 250.0), 60.0)
        self.assertLess(abs(pilar.y_mm - 800.0), 80.0)
        self.assertLess(pilar.ancho_mm, 240.0)

    def test_el_ancho_fisico_no_depende_de_la_distancia(self):
        """La puerta angular de la version anterior perdia el poste a 216 mm.

        Un poste de 100 mm subtiende 23 grados a 250 mm y 15 a 383, asi que un
        umbral de 15 grados lo descartaba justo cuando mas importaba.  Con el
        criterio de ancho fisico se detecta igual de cerca que de lejos.
        """

        x, y, rumbo = pose_en_carril(0, 1200.0, 400.0)
        for avance_pilar in (300.0, 600.0, 950.0):
            px, py, _ = pose_en_carril(0, avance_pilar, 650.0)
            scan = barrido_sintetico(x, y, rumbo, pilares=[(px, py, 100.0)])
            _p, objetos, _h = self.percepcion.procesar(scan, 1.0)
            with self.subTest(avance=avance_pilar):
                self.assertTrue(objetos, "el pilar deberia verse")
                self.assertLess(objetos[0].ancho_mm, 240.0)


class PruebaHuellaPropia(unittest.TestCase):
    """La huella del robot va DETRAS del LiDAR, que esta al ras del parachoques.

    Con la ventana simetrica de +-250 mm que habia antes, un poste a punto de
    ser embestido caia dentro de la huella y se descartaba como "robot": el
    corredor libre si veia sus puntos y frenaba, pero el planificador nunca
    sabia que habia un objeto.  En las corridas 3 y 4 del 04-09 eso acabo en
    retroceso nueve veces, siempre contra un bulto de 55 mm.
    """

    def setUp(self):
        self.percepcion = PercepcionLidar(config_minima())

    def _objetos(self, avance_pilar):
        x, y, rumbo = pose_en_carril(0, 1200.0, 400.0)
        px, py, _ = pose_en_carril(0, avance_pilar, 420.0)
        scan = barrido_sintetico(x, y, rumbo, pilares=[(px, py, 100.0)])
        _p, objetos, _h = self.percepcion.procesar(scan, 1.0)
        return objetos

    def test_un_poste_pegado_al_morro_sigue_siendo_un_objeto(self):
        objetos = self._objetos(1050.0)  # ~150 mm por delante del LiDAR
        self.assertTrue(objetos, "un poste a 150 mm no puede pasar por robot")
        self.assertLess(objetos[0].y_mm, 260.0)

    def test_detras_del_lidar_se_sigue_descartando(self):
        """El mastil sale por el borde del sector ciego a unos 90 mm."""

        percepcion = PercepcionLidar(config_minima())
        # Arco de ~13 grados a 90 mm: unos 20 mm de ancho fisico, suficiente
        # para pasar el filtro de tamaño y llegar a la prueba de huella.
        cluster = [(grados, 90.0) for grados in (148.0, 151.0, 154.0, 157.0, 161.0)]
        self.assertFalse(percepcion.objetos([cluster], 1.0))
        # Y el mismo cluster POR DELANTE del morro si es un objeto.
        delante = [(grados, 90.0) for grados in (-13.0, -6.0, 0.0, 6.0, 13.0)]
        self.assertTrue(percepcion.objetos([delante], 1.0))


class PruebaEcoPropio(unittest.TestCase):
    def test_el_radio_enmascarado_crece_con_el_volante(self):
        """El eco de la rueda se ALEJA al girar: 49-51 mm a 17 grados, 91-107 a 25.

        Un umbral fijo no lo cubre; por eso el filtro escala con el mando.
        """

        percepcion = PercepcionLidar(config_minima())
        eco_17 = (290.0, 50.0)    # medido: 49-51 mm con el servo a 17 grados
        eco_25 = (290.0, 100.0)   # medido: 91-107 mm con el servo a 25
        pared = (0.0, 1500.0)

        a_17 = percepcion._mascarar([eco_17, eco_25, pared], angulo_servo_deg=17.0)
        a_25 = percepcion._mascarar([eco_17, eco_25, pared], angulo_servo_deg=25.0)

        self.assertNotIn(eco_17, a_17, "el eco de 17 grados tiene que filtrarse")
        self.assertIn(eco_25, a_17, "a 17 grados no hay que borrar hasta 100 mm")
        self.assertNotIn(eco_25, a_25, "a 25 grados el eco llega a ~107 mm")
        self.assertIn(pared, a_17)
        self.assertIn(pared, a_25)

    def test_el_lado_enmascarado_es_el_del_volante(self):
        percepcion = PercepcionLidar(config_minima())
        izquierdo, derecho = (290.0, 50.0), (70.0, 50.0)
        a_izquierda = percepcion._mascarar([izquierdo, derecho], angulo_servo_deg=20.0)
        self.assertNotIn(izquierdo, a_izquierda)
        self.assertIn(derecho, a_izquierda)

    def test_con_las_ruedas_rectas_no_se_filtra_nada(self):
        """Por debajo de ~10 grados el neumatico no entra en el plano."""

        percepcion = PercepcionLidar(config_minima())
        self.assertEqual(
            percepcion._mascarar([(290.0, 40.0)], angulo_servo_deg=0.0),
            [(290.0, 40.0)],
        )
        self.assertEqual(
            percepcion._mascarar([(290.0, 40.0)], angulo_servo_deg=8.0),
            [(290.0, 40.0)],
        )


class PruebaAjuste(unittest.TestCase):
    def test_pca_funciona_igual_en_horizontal_y_en_vertical(self):
        """Un ajuste x = a*y + b revienta con la pared vista de frente."""

        horizontal = [(float(x), 1000.0) for x in range(-400, 401, 50)]
        vertical = [(500.0, float(y)) for y in range(-400, 401, 50)]
        recta_h = ajustar_recta(horizontal)
        recta_v = ajustar_recta(vertical)
        self.assertAlmostEqual(recta_h.distancia_mm, 1000.0, delta=1.0)
        self.assertAlmostEqual(recta_h.angulo_deg, 0.0, delta=1.0)
        self.assertAlmostEqual(recta_v.distancia_mm, 500.0, delta=1.0)
        self.assertAlmostEqual(abs(recta_v.angulo_deg), 90.0, delta=1.0)

    def test_un_atipico_no_arrastra_la_recta(self):
        puntos = [(float(x), 1000.0) for x in range(-400, 401, 50)]
        puntos.append((0.0, 1600.0))
        recta = ajustar_recta(puntos)
        self.assertAlmostEqual(recta.distancia_mm, 1000.0, delta=15.0)


class PruebaSegmentacion(unittest.TestCase):
    def test_une_el_cruce_de_359_a_0(self):
        puntos = [(358.0, 500.0), (359.0, 501.0), (0.5, 502.0), (1.5, 503.0)]
        clusters = segmentar(puntos, min_puntos=2)
        self.assertEqual(len(clusters), 1)
        self.assertEqual(len(clusters[0]), 4)


if __name__ == "__main__":
    unittest.main()
