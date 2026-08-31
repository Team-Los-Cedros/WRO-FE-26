import copy
import unittest

import cv2
import numpy as np

from src.pi3B.ronda_nueva.config import cargar_configuracion
from src.pi3B.ronda_nueva.vision_ligera import VisionLigera


def escena_base():
    imagen = np.zeros((240, 320, 3), dtype=np.uint8)
    imagen[165:, :] = (235, 235, 235)  # piso blanco en orden BGR
    return imagen


class VisionLigeraTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()
        self.vision = VisionLigera(self.config)

    def test_rgb888_de_picamera_se_interpreta_como_array_bgr(self):
        imagen = escena_base()
        cv2.rectangle(imagen, (160, 70), (190, 164), (0, 0, 255), -1)

        paquete = self.vision.procesar(imagen, timestamp=10.0)

        self.assertEqual(len(paquete.detecciones), 1)
        deteccion = paquete.detecciones[0]
        self.assertEqual(deteccion.color, "ROJO")
        self.assertAlmostEqual(deteccion.timestamp, 10.0)
        self.assertAlmostEqual(deteccion.bearing_deg, 0.0, delta=0.5)
        self.assertGreater(deteccion.soporte_suelo, 0.8)

    def test_azul_bgr_no_se_confunde_con_rojo(self):
        imagen = escena_base()
        cv2.rectangle(imagen, (160, 70), (190, 164), (255, 0, 0), -1)

        paquete = self.vision.procesar(imagen, timestamp=11.0)

        self.assertEqual(paquete.detecciones, ())

    def test_devuelve_varios_pilares_y_descarta_blob_sin_suelo(self):
        imagen = escena_base()
        cv2.rectangle(imagen, (45, 85), (70, 164), (0, 255, 0), -1)
        cv2.rectangle(imagen, (235, 60), (265, 164), (0, 0, 255), -1)
        # Blob flotante: la banda inmediatamente inferior sigue negra.
        cv2.rectangle(imagen, (120, 60), (140, 115), (0, 255, 0), -1)

        paquete = self.vision.procesar(imagen, timestamp=12.0)

        self.assertEqual({d.color for d in paquete.detecciones}, {"ROJO", "VERDE"})
        self.assertEqual(len(paquete.detecciones), 2)
        self.assertTrue(all(d.soporte_suelo >= 0.8 for d in paquete.detecciones))

    def test_orden_rgb_solo_se_usa_si_se_configura_expresamente(self):
        config = copy.deepcopy(self.config)
        config["camera"]["array_color_order"] = "RGB"
        vision_rgb = VisionLigera(config)
        imagen_rgb = escena_base()
        cv2.rectangle(imagen_rgb, (160, 70), (190, 164), (255, 0, 0), -1)

        paquete = vision_rgb.procesar(imagen_rgb, timestamp=13.0)

        self.assertEqual(len(paquete.detecciones), 1)
        self.assertEqual(paquete.detecciones[0].color, "ROJO")


if __name__ == "__main__":
    unittest.main()
