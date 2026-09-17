from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sentido_vuelta


class SembrarTests(unittest.TestCase):
    def test_siembra_el_sentido_de_la_salida(self):
        p = sentido_vuelta.SentidoPorGeometria()
        self.assertTrue(p.sembrar(-1))
        self.assertEqual(p.sentido, -1)
        # Pista, no certeza: una linea posterior lo puede corregir.
        self.assertFalse(p.por_lineas)

    def test_no_pisa_un_sentido_ya_decidido_ni_acepta_basura(self):
        p = sentido_vuelta.SentidoPorGeometria()
        self.assertFalse(p.sembrar(0))
        self.assertFalse(p.sembrar(5))
        p.sembrar(1)
        self.assertFalse(p.sembrar(-1))
        self.assertEqual(p.sentido, 1)


if __name__ == '__main__':
    unittest.main()
