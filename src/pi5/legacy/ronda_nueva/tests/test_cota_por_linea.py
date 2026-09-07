# -*- coding: utf-8 -*-
"""La linea de piso como TECHO del avance, no como disparo del giro.

Disparar el giro con la linea se probo en pista el 05-09 y salio mal -- 4
esquinas y 26 retrocesos contra 11 y 14 -- porque la linea es diagonal y se ve
mucho antes de llegar a la esquina. Lo que si aporta es una cota: si la camara
ve la linea de la esquina, NO quedan 3000 mm de recta.

Lo que estas pruebas fijan es la asimetria. El techo solo puede BAJAR el
avance. Si alguna vez pudiera subirlo estaria inventando recta donde no la
hay, y eso si podria empujar al robot contra un muro.
"""

import unittest

from ..localizacion import Localizador
from ..modelos import LineaPiso, MapaParedes, Recta
from ..piloto import RECTA, Piloto
from .test_piloto import config


def recta(distancia_mm, angulo_deg=0.0):
    return Recta(float(distancia_mm), float(angulo_deg), 1.0, 40, 1.0)


def paredes(frontal=None, izquierda=None, derecha=None, t=1.0):
    return MapaParedes(
        timestamp=t,
        frontal=recta(frontal) if frontal is not None else None,
        izquierda=recta(izquierda) if izquierda is not None else None,
        derecha=recta(derecha) if derecha is not None else None,
        trasera=None,
    )


def linea(y_mm=900.0, x_mm=0.0, color="NARANJA"):
    return LineaPiso(timestamp=0.0, color=color, y_mm=y_mm, x_mm=x_mm, area_px=2400)


class TechoEnElLocalizadorTests(unittest.TestCase):
    def _localizador(self):
        cfg = config()
        return Localizador(cfg)

    def test_el_techo_recorta_un_avance_inflado(self):
        loc = self._localizador()
        # Sin muro frontal medido, el avance se queda en la semilla de 3000:
        # es el fallo conocido de "clavado en 3000".
        pose = loc.actualizar(paredes(izquierda=300.0), 0.0, 1, 0.0, 1.0)
        self.assertGreater(pose.avance_mm, 2500.0)

        pose = loc.actualizar(
            paredes(izquierda=300.0), 0.0, 1, 0.0, 1.1, cota_avance_mm=2100.0
        )
        self.assertAlmostEqual(pose.avance_mm, 2100.0, places=1)

    def test_el_techo_NO_puede_subir_el_avance(self):
        # La garantia que hace seguro todo esto.
        loc = self._localizador()
        loc.actualizar(paredes(frontal=800.0), 0.0, 1, 0.0, 1.0)
        pose = loc.actualizar(
            paredes(frontal=800.0), 0.0, 1, 0.0, 1.1, cota_avance_mm=2500.0
        )
        self.assertLess(pose.avance_mm, 1200.0)

    def test_sin_techo_se_comporta_igual_que_antes(self):
        loc_a, loc_b = self._localizador(), self._localizador()
        for i in range(5):
            t = 1.0 + 0.1 * i
            a = loc_a.actualizar(paredes(frontal=1500.0 - 50 * i), 0.0, 1, 20.0, t)
            b = loc_b.actualizar(
                paredes(frontal=1500.0 - 50 * i), 0.0, 1, 20.0, t,
                cota_avance_mm=None,
            )
            self.assertAlmostEqual(a.avance_mm, b.avance_mm, places=6)

    def test_cuenta_los_recortes_para_poder_vigilarlos(self):
        loc = self._localizador()
        loc.actualizar(paredes(izquierda=300.0), 0.0, 1, 0.0, 1.0)
        self.assertEqual(loc._recortes_por_referencia, 0)
        loc.actualizar(paredes(izquierda=300.0), 0.0, 1, 0.0, 1.1, cota_avance_mm=2000.0)
        self.assertEqual(loc._recortes_por_referencia, 1)


class TechoEnElPilotoTests(unittest.TestCase):
    def _piloto(self, **control):
        cfg = config()
        cfg["control"].update(control)
        return Piloto(cfg)

    def test_el_techo_sale_de_la_linea_mas_cercana_mas_el_margen(self):
        piloto = self._piloto(line_avance_ceiling_margin_mm=1200.0)
        cota = piloto._cota_de_avance([linea(y_mm=900.0), linea(y_mm=1400.0)])
        self.assertAlmostEqual(cota, 2100.0)

    def test_la_linea_de_al_lado_no_pone_techo(self):
        piloto = self._piloto(line_max_lateral_mm=320.0)
        self.assertIsNone(piloto._cota_de_avance([linea(x_mm=650.0)]))

    def test_una_linea_ya_cruzada_no_pone_techo(self):
        piloto = self._piloto()
        self.assertIsNone(piloto._cota_de_avance([linea(y_mm=-100.0)]))

    def test_sin_lineas_no_hay_techo(self):
        self.assertIsNone(self._piloto()._cota_de_avance([]))

    def test_se_puede_desactivar(self):
        piloto = self._piloto(line_avance_ceiling_enabled=False)
        self.assertIsNone(piloto._cota_de_avance([linea()]))

    def test_el_techo_nunca_cae_en_la_banda_de_disparo(self):
        # Si el techo pudiera bajar hasta el umbral de giro, estaria
        # disparando esquinas por la puerta de atras, que es justo lo que se
        # descarto en pista.
        piloto = self._piloto(line_avance_ceiling_margin_mm=1200.0)
        cota = piloto._cota_de_avance([linea(y_mm=1.0)])
        self.assertGreater(cota, piloto.giro_disparo_max_mm)


class VueltaCompletaConTechoTests(unittest.TestCase):
    def _correr(self, activo):
        from .test_piloto import Mundo

        cfg = config()
        cfg["control"]["line_avance_ceiling_enabled"] = activo
        mundo = Mundo(cfg)
        mundo.correr(180.0)
        return mundo.piloto

    def test_la_vuelta_sigue_cerrando_doce_esquinas(self):
        # El mundo sintetico no pinta lineas, asi que el techo nunca entra:
        # lo que se comprueba es que anadirlo no rompe nada.
        self.assertGreaterEqual(self._correr(True).esquinas, 12)
        self.assertGreaterEqual(self._correr(False).esquinas, 12)


if __name__ == "__main__":
    unittest.main()
