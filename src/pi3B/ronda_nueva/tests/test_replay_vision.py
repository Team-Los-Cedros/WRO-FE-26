import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from src.pi3B.ronda_nueva.config import cargar_configuracion
from src.pi3B.ronda_nueva.replay_vision import ejecutar_replay


class ReplayVisionTests(unittest.TestCase):
    def test_directorio_de_frames_se_procesa_sin_hardware(self):
        with tempfile.TemporaryDirectory() as temporal:
            carpeta = Path(temporal)
            imagen = np.zeros((240, 320, 3), dtype=np.uint8)
            imagen[165:, :] = (235, 235, 235)
            cv2.rectangle(imagen, (160, 70), (190, 164), (0, 0, 255), -1)
            self.assertTrue(cv2.imwrite(str(carpeta / "1.000.jpg"), imagen))
            self.assertTrue(cv2.imwrite(str(carpeta / "2.000.jpg"), imagen))

            resumen = ejecutar_replay(carpeta, cargar_configuracion())

        self.assertEqual(resumen["cuadros"], 2)
        self.assertEqual(resumen["rojo"], 2)
        self.assertEqual(resumen["verde"], 0)
        self.assertGreaterEqual(resumen["media_ms"], 0.0)


if __name__ == "__main__":
    unittest.main()
