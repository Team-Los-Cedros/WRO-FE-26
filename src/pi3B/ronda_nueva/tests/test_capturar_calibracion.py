import tempfile
import unittest
from pathlib import Path

from src.pi3B.ronda_nueva.capturar_calibracion import crear_directorio_captura


class CapturaCalibracionTests(unittest.TestCase):
    def test_solo_crea_destino_nuevo_y_no_reutiliza(self):
        with tempfile.TemporaryDirectory() as temporal:
            destino = Path(temporal) / "captura_nueva"
            creada = crear_directorio_captura(destino)
            self.assertTrue((creada / "frames").is_dir())
            testigo = creada / "no_tocar.txt"
            testigo.write_text("usuario", encoding="utf-8")

            with self.assertRaises(FileExistsError):
                crear_directorio_captura(destino)
            self.assertEqual(testigo.read_text(encoding="utf-8"), "usuario")


if __name__ == "__main__":
    unittest.main()
