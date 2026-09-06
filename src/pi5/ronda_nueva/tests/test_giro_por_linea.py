# -*- coding: utf-8 -*-
"""El giro disparado por la linea pintada de la esquina.

Lo que estas pruebas protegen es sobre todo lo que NO debe pasar. Cambiar el
disparo del giro ya salio mal dos veces (ver `_disparo_de_giro_mm`): una
esquina contada de mas gira el rumbo cardinal y deja el marco desplazado 90
grados el resto de la vuelta, asi que un falso positivo no cuesta un giro,
cuesta la ronda entera.
"""

import unittest

from ..modelos import LineaPiso
from ..piloto import Piloto
from .test_piloto import config


def linea(color="NARANJA", y_mm=800.0, x_mm=0.0, area_px=2400):
    return LineaPiso(
        timestamp=0.0, color=color, y_mm=y_mm, x_mm=x_mm, area_px=area_px
    )


class DisparoPorLineaTests(unittest.TestCase):
    def _piloto(self, **control):
        cfg = config()
        cfg["control"].update(control)
        return Piloto(cfg)

    def test_una_sola_deteccion_no_basta(self):
        # Un reflejo suelto de la lona no puede meter una esquina que no
        # existe: se exigen confirmaciones consecutivas.
        piloto = self._piloto(corner_line_confirm_scans=2)
        self.assertFalse(piloto._linea_dispara_giro([linea()]))
        self.assertTrue(piloto._linea_dispara_giro([linea()]))

    def test_el_contador_se_reinicia_si_la_linea_desaparece(self):
        piloto = self._piloto(corner_line_confirm_scans=3)
        piloto._linea_dispara_giro([linea()])
        piloto._linea_dispara_giro([linea()])
        piloto._linea_dispara_giro([])          # se perdio un cuadro
        self.assertFalse(piloto._linea_dispara_giro([linea()]))
        self.assertFalse(piloto._linea_dispara_giro([linea()]))
        self.assertTrue(piloto._linea_dispara_giro([linea()]))

    def test_la_linea_de_la_esquina_de_al_lado_no_cuenta(self):
        # Es el mismo filtro lateral que ya usa _resolver_sentido: una franja
        # a 650 mm de lado es de otra esquina.
        piloto = self._piloto(corner_line_confirm_scans=1, line_max_lateral_mm=320.0)
        self.assertFalse(piloto._linea_dispara_giro([linea(x_mm=650.0)]))
        self.assertFalse(piloto._linea_dispara_giro([linea(x_mm=-650.0)]))
        self.assertTrue(piloto._linea_dispara_giro([linea(x_mm=100.0)]))

    def test_una_linea_lejana_todavia_no_dispara(self):
        # Medido en pista: la naranja se estabiliza en 875 mm y la azul en
        # 1109. Con el umbral en 900 la naranja abre el giro y la azul, que
        # es la misma esquina vista mas lejos, todavia no.
        piloto = self._piloto(corner_line_confirm_scans=1, corner_line_trigger_mm=900.0)
        self.assertFalse(piloto._linea_dispara_giro([linea(y_mm=1109.0)]))
        self.assertTrue(piloto._linea_dispara_giro([linea(y_mm=875.0)]))

    def test_una_linea_detras_no_cuenta(self):
        # y negativo es una linea ya cruzada; girar por ella seria girar dos
        # veces en la misma esquina.
        piloto = self._piloto(corner_line_confirm_scans=1)
        self.assertFalse(piloto._linea_dispara_giro([linea(y_mm=-200.0)]))

    def test_el_color_da_igual_para_disparar(self):
        # Las dos lineas se cruzan en cada esquina. El color decide el SENTIDO
        # de la vuelta, no si hay esquina delante.
        piloto = self._piloto(corner_line_confirm_scans=1)
        self.assertTrue(piloto._linea_dispara_giro([linea(color="AZUL")]))

    def test_por_defecto_manda_el_avance_y_la_linea_solo_se_anota(self):
        # El cambio no se activa solo: hasta que no haya tres corridas
        # comparadas, "linea" es una hipotesis.
        piloto = self._piloto()
        self.assertEqual(piloto.giro_fuente, "avance")

    def test_se_puede_conmutar_la_fuente(self):
        piloto = self._piloto(corner_trigger_source="linea")
        self.assertEqual(piloto.giro_fuente, "linea")

    def test_la_telemetria_expone_las_dos_columnas(self):
        piloto = self._piloto(corner_trigger_source="linea")
        instantanea = piloto.instantanea()
        self.assertEqual(instantanea["giro_fuente"], "linea")
        self.assertIn("linea_lista", instantanea)


class VueltaCompletaConLineaTests(unittest.TestCase):
    """La vuelta simulada tiene que seguir cerrando 12 esquinas.

    El mundo sintetico no pinta lineas, asi que con la fuente en "linea" el
    disparo cae en la red del avance. Es justo la garantia que importa: si la
    camara no ve la linea, el robot gira igual y nunca queda peor que hoy.
    """

    def _correr(self, fuente):
        from .test_piloto import Mundo

        cfg = config()
        cfg["control"]["corner_trigger_source"] = fuente
        mundo = Mundo(cfg)
        mundo.correr(180.0)
        return mundo.piloto

    def test_sin_lineas_visibles_la_red_del_avance_cierra_la_vuelta(self):
        piloto = self._correr("linea")
        self.assertGreaterEqual(piloto.esquinas, 12)

    def test_el_modo_avance_no_cambia_de_comportamiento(self):
        piloto = self._correr("avance")
        self.assertGreaterEqual(piloto.esquinas, 12)


if __name__ == "__main__":
    unittest.main()
