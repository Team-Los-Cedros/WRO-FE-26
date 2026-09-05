"""La homografia del suelo: si esta mal, todo lo demas mide donde no es."""

import math
import unittest

import numpy as np

from ..geometria_suelo import (
    ErrorHomografia,
    ProyectorSuelo,
    desde_correspondencias,
    desde_montaje,
    error_de_reproyeccion,
    matriz_intrinseca,
)


ANCHO, ALTO, HFOV = 1280, 720, 68.16865


def _proyector(altura=300.0, cabeceo=30.0, guinada=0.0, adelante=-80.0):
    H = desde_montaje(
        ANCHO, ALTO, HFOV, altura, cabeceo, guinada, adelante_mm=adelante
    )
    focal = (ANCHO / 2.0) / math.tan(math.radians(HFOV) / 2.0)
    return ProyectorSuelo(H, ANCHO, ALTO, focal_px=focal)


class PruebaMontaje(unittest.TestCase):
    def test_centro_del_cuadro_cae_donde_dice_la_trigonometria(self):
        proyector = _proyector()
        punto = proyector.punto_suelo(ANCHO / 2.0, ALTO / 2.0)
        self.assertIsNotNone(punto)
        esperado_y = -80.0 + 300.0 / math.tan(math.radians(30.0))
        self.assertAlmostEqual(punto[0], 0.0, places=6)
        self.assertAlmostEqual(punto[1], esperado_y, places=6)

    def test_ida_y_vuelta_suelo_imagen(self):
        proyector = _proyector()
        for x, y in ((0.0, 600.0), (300.0, 1200.0), (-450.0, 900.0)):
            pixel = proyector.punto_imagen(x, y)
            self.assertIsNotNone(pixel)
            vuelta = proyector.punto_suelo(*pixel)
            self.assertIsNotNone(vuelta)
            self.assertAlmostEqual(vuelta[0], x, places=4)
            self.assertAlmostEqual(vuelta[1], y, places=4)

    def test_la_guinada_corre_el_punto_hacia_ese_lado(self):
        """Un sesgo de guiñada como el de +3,57 grados medido en el mastil."""

        recto = _proyector().punto_suelo(ANCHO / 2.0, ALTO / 2.0)
        girado = _proyector(guinada=3.57).punto_suelo(ANCHO / 2.0, ALTO / 2.0)
        self.assertGreater(girado[0], recto[0] + 20.0)

    def test_por_encima_del_horizonte_devuelve_none(self):
        """Un pixel que no corta el suelo no puede dar una distancia enorme.

        Devolver un numero convertiria un reflejo en la pared en un pilar a
        cinco metros; devolver None lo descarta.
        """

        proyector = _proyector(cabeceo=8.0)
        self.assertIsNone(proyector.punto_suelo(ANCHO / 2.0, 0.0))

    def test_montaje_invalido_falla_pronto(self):
        with self.assertRaises(ErrorHomografia):
            desde_montaje(ANCHO, ALTO, HFOV, altura_mm=-1.0, cabeceo_deg=30.0)
        with self.assertRaises(ErrorHomografia):
            desde_montaje(ANCHO, ALTO, HFOV, altura_mm=300.0, cabeceo_deg=0.0)
        with self.assertRaises(ErrorHomografia):
            matriz_intrinseca(0, ALTO, HFOV)


class PruebaCorrespondencias(unittest.TestCase):
    def test_el_dlt_deja_las_correspondencias_delante_del_horizonte(self):
        """El signo global de H no es libre: el proyector usa w > 0.

        Con las siete correspondencias camara-LiDAR reales del 05-09 el DLT
        devolvia una matriz que reproducia los puntos al milimetro pero con w
        negativo, y ``ProyectorSuelo`` daba TODOS los pixeles por encima del
        horizonte: ``error_de_reproyeccion`` salia ``inf`` con una calibracion
        buena.  Estas son esas mismas siete correspondencias.
        """

        pares = [
            ((704.0, 368.0), (79.4, 715.6)),
            ((201.0, 382.0), (-341.5, 685.7)),
            ((273.0, 338.0), (-362.7, 912.0)),
            ((969.0, 356.0), (329.1, 752.6)),
            ((598.0, 370.0), (-18.5, 685.3)),
            ((720.0, 332.0), (115.7, 904.2)),
            ((581.0, 432.0), (-22.7, 479.1)),
        ]
        H = desde_correspondencias(pares)
        proyector = ProyectorSuelo(H, 1280, 720)
        for (u, v), _ in pares:
            with self.subTest(pixel=(u, v)):
                self.assertIsNotNone(
                    proyector.punto_suelo(u, v),
                    "el pixel de una correspondencia no puede caer sobre el horizonte",
                )
        medio, maximo = error_de_reproyeccion(proyector, pares)
        self.assertLess(medio, 20.0, "el ajuste sobre estos siete puntos daba 8,6 mm")
        self.assertLess(maximo, 40.0, "el peor de los siete daba 18,3 mm")

    def test_dlt_recupera_la_homografia_exacta(self):
        proyector = _proyector()
        pares = []
        for x, y in ((-400, 500), (400, 500), (-600, 1500), (600, 1500), (0, 2200), (200, 900)):
            pares.append((proyector.punto_imagen(x, y), (float(x), float(y))))
        H = desde_correspondencias(pares)
        recuperado = ProyectorSuelo(H, ANCHO, ALTO)
        medio, maximo = error_de_reproyeccion(recuperado, pares)
        self.assertLess(medio, 1e-6)
        self.assertLess(maximo, 1e-6)

    def test_pocas_o_degeneradas_fallan_con_mensaje(self):
        with self.assertRaises(ErrorHomografia):
            desde_correspondencias([((0, 0), (0, 0)), ((1, 1), (1, 1))])
        alineadas = [((float(i), 10.0), (float(i) * 10.0, 100.0)) for i in range(5)]
        with self.assertRaises(ErrorHomografia):
            desde_correspondencias(alineadas)

    def test_distancia_por_altura_es_independiente_de_la_homografia(self):
        proyector = _proyector()
        # Un pilar de 100 mm que ocupa 47 px con focal 940 px esta a ~2 m.
        estimada = proyector.distancia_por_altura_mm(47.0, 100.0)
        self.assertIsNotNone(estimada)
        self.assertAlmostEqual(estimada, proyector.focal_px * 100.0 / 47.0, places=6)


class PruebaConfig(unittest.TestCase):
    def test_sin_calibrar_devuelve_none_en_vez_de_lanzar(self):
        """Es el estado normal mientras la camara esta desmontada."""

        self.assertIsNone(
            ProyectorSuelo.desde_config(
                {"width": ANCHO, "height": ALTO, "hfov_deg": HFOV,
                 "ground_homography": {"ready": False}}
            )
        )

    def test_con_matriz_explicita_la_usa(self):
        H = desde_montaje(ANCHO, ALTO, HFOV, 300.0, 30.0)
        proyector = ProyectorSuelo.desde_config(
            {
                "width": ANCHO,
                "height": ALTO,
                "hfov_deg": HFOV,
                "ground_homography": {"ready": True, "matrix": H.tolist()},
            }
        )
        self.assertIsNotNone(proyector)
        self.assertTrue(np.allclose(proyector.homografia, H))


if __name__ == "__main__":
    unittest.main()
