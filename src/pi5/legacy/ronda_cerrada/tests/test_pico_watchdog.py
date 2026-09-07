import unittest

from src.pico.protocolo_seguro import parsear_consigna, watchdog_vencido


class ProtocoloSeguroPicoTests(unittest.TestCase):
    def test_acepta_las_dos_tramas_historicas(self):
        self.assertEqual(parsear_consigna("55,12.5"), (55, 12.5, 1.0))
        self.assertEqual(parsear_consigna("-22,-20,0"), (-22, -20.0, 0.0))

    def test_rechaza_tramas_malformadas_o_fuera_de_limites(self):
        for linea in (
            "",
            "55",
            "55,12,1,extra",
            "101,0",
            "0,46",
            "0,nan",
            "0,0,3",
            "texto,0",
        ):
            with self.subTest(linea=linea):
                self.assertIsNone(parsear_consigna(linea))

    def test_watchdog_arranca_y_vence_en_parada(self):
        diferencia = lambda actual, anterior: actual - anterior
        self.assertTrue(watchdog_vencido(100, None, 500, diferencia))
        self.assertFalse(watchdog_vencido(599, 100, 500, diferencia))
        self.assertFalse(watchdog_vencido(600, 100, 500, diferencia))
        self.assertTrue(watchdog_vencido(601, 100, 500, diferencia))


if __name__ == "__main__":
    unittest.main()
