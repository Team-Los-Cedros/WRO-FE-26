import math
import unittest

from src.pi3B.ronda_nueva.oclusion_lidar import (
    SIN_DATO,
    compensar_oclusion_lidar,
)


def perfil_vacio():
    return [SIN_DATO] * 360


class OclusionLidarTests(unittest.TestCase):
    def test_eco_del_mastil_no_contamina_las_medidas(self):
        perfil = perfil_vacio()
        for angulo in range(163, 192):
            perfil[angulo] = 92.0
        perfil[150] = 600.0
        perfil[151] = 605.0
        perfil[210] = 610.0
        perfil[209] = 615.0

        resultado = compensar_oclusion_lidar(perfil)

        self.assertTrue(resultado.trasera_valida)
        self.assertGreater(resultado.trasera_axial_mm, 500.0)
        self.assertNotAlmostEqual(resultado.trasera_axial_mm, 92.0)
        self.assertAlmostEqual(resultado.diagonal_trasera_derecha_mm, 600.0)
        self.assertAlmostEqual(resultado.diagonal_trasera_izquierda_mm, 610.0)
        self.assertTrue(resultado.mascara_confirmada)

    def test_proyeccion_axial_correcta(self):
        perfil = perfil_vacio()
        perfil[150] = 1000.0  # offset de 30 grados respecto al eje trasero
        perfil[210] = 1000.0  # segundo hombro, mismo offset axial

        resultado = compensar_oclusion_lidar(perfil)

        self.assertTrue(resultado.trasera_valida)
        self.assertAlmostEqual(
            resultado.trasera_axial_mm,
            1000.0 * math.cos(math.radians(30.0)),
            places=7,
        )

    def test_sin_eco_en_ambos_hombros_es_invalido_no_via_libre(self):
        resultado = compensar_oclusion_lidar(perfil_vacio())

        self.assertFalse(resultado.trasera_valida)
        self.assertIsNone(resultado.trasera_axial_mm)
        self.assertNotEqual(resultado.trasera_axial_mm, SIN_DATO)
        self.assertEqual(resultado.cobertura.trasera, 0.0)

    def test_diagonales_son_simetricas(self):
        perfil = perfil_vacio()
        perfil[120] = 730.0  # eje - 60
        perfil[121] = 730.0
        perfil[240] = 730.0  # eje + 60
        perfil[239] = 730.0

        resultado = compensar_oclusion_lidar(perfil)

        self.assertTrue(resultado.diagonal_derecha_valida)
        self.assertTrue(resultado.diagonal_izquierda_valida)
        self.assertEqual(
            resultado.diagonal_trasera_derecha_mm,
            resultado.diagonal_trasera_izquierda_mm,
        )
        self.assertEqual(
            resultado.cobertura.diagonal_derecha,
            resultado.cobertura.diagonal_izquierda,
        )

    def test_estructura_fuera_de_mascara_se_senala(self):
        perfil = perfil_vacio()
        perfil[180] = 92.0
        perfil[140] = 110.0

        resultado = compensar_oclusion_lidar(perfil)

        self.assertFalse(resultado.mascara_confirmada)
        self.assertEqual(resultado.estructura_fuera_mascara_deg, (140,))
        self.assertIn("fuera de la mascara", resultado.diagnostico)


if __name__ == "__main__":
    unittest.main()
