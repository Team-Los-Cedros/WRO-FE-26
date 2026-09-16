import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from seguimiento_lidar import SeguimientoLidar


class SeguimientoTests(unittest.TestCase):
    def setUp(self):
        self.s = SeguimientoLidar()
        self.actual = SimpleNamespace(id=1, activo=True, x=0., y=300.)
        self.ref = SimpleNamespace(fuente='lidar', confirmaciones=3, en_seccion=True,
            t=10., color='ROJO', x=400., y=1000., sigma=30.)
        self.s.actualizar(self.ref, [(400., 1000., 50.)], self.actual, 0, 0, 10., lambda x,y: True)
        self.ref.fuente = 'prediccion'

    def actualizar(self, clusters, t=10.1, seccion=True, barrido=None):
        self.s.actualizar(self.ref, clusters, self.actual, 0, 0, t,
                          lambda x, y: seccion, barrido)

    def test_sigue_sin_camara_y_conserva_color(self):
        self.actualizar([(402., 1000., 50.)])
        self.assertEqual((self.s.color, self.s.x, self.s.fuente), ('ROJO', 402., 'lidar_seguimiento'))
        self.assertEqual(self.s.t_color, 10.)
        self.assertFalse(self.s.autoriza_control())

    def test_rechaza_dos_candidatos_y_otro_sector(self):
        self.actualizar([(402., 1000., 50.), (405., 1020., 50.)])
        self.assertEqual(self.s.motivo, 'ambiguo')
        self.assertEqual(self.s.t_lidar, 10.)
        self.actualizar([(402., 1000., 50.)], 10.2, False)
        self.assertEqual(self.s.motivo, 'otra_seccion')

    def test_barrido_repetido_no_rejuvenece(self):
        self.actualizar([(400., 1000., 50.)])
        self.assertEqual(self.s.t_lidar, 10.)
        self.actualizar([(400., 1000., 50.)], 11.6)
        self.assertIsNone(self.s.color)

    def test_cambio_actual_descarta_identidad_anterior(self):
        self.actual.id = 2
        self.actualizar([(402., 1000., 50.)])
        self.assertIsNone(self.s.color)

    def test_el_mismo_barrido_no_vuelve_a_medir(self):
        self.actualizar([(402., 1000., 50.)], 10.1, barrido=(7, 10.05))
        self.assertEqual(self.s.t_lidar, 10.05)
        # Mismo numero de barrido: no hay dato nuevo aunque cambie la lista.
        self.actualizar([(404., 1000., 50.)], 10.2, barrido=(7, 10.05))
        self.assertEqual((self.s.x, self.s.motivo), (402., 'sin_barrido_nuevo'))

    def test_un_barrido_viejo_no_cuenta_como_ahora(self):
        self.actualizar([(402., 1000., 50.)], 10.6, barrido=(8, 10.05))
        self.assertEqual(self.s.motivo, 'sin_barrido_nuevo')

    def test_solo_manda_con_barrido_identificado_y_medida_fresca(self):
        self.actualizar([(402., 1000., 50.)], 10.1, barrido=(7, 10.05))
        self.assertTrue(self.s.autoriza_control(10.1))
        # La misma medida vista 0,4s despues ya no es de este ciclo.
        self.assertFalse(self.s.autoriza_control(10.5))

    def test_sin_barrido_mira_pero_no_manda(self):
        self.actualizar([(402., 1000., 50.)])
        self.assertEqual(self.s.fuente, 'lidar_seguimiento')
        self.assertFalse(self.s.autoriza_control(10.1))

    def test_demasiado_cerca_lo_lleva_la_fsm(self):
        self.s = SeguimientoLidar()
        self.ref.fuente, self.ref.x, self.ref.y = 'lidar', 300., 350.
        self.actualizar([(300., 350., 50.)], 10., barrido=(6, 9.95))
        self.ref.fuente = 'prediccion'
        self.actualizar([(302., 350., 50.)], 10.1, barrido=(7, 10.05))
        self.assertEqual(self.s.fuente, 'lidar_seguimiento')
        self.assertLess(math.hypot(self.s.x, self.s.y), 500.)
        self.assertFalse(self.s.autoriza_control(10.1))

    def test_visual_no_inicia_seguimiento(self):
        self.s = SeguimientoLidar()
        self.ref.fuente = 'visual_relativa'
        self.actualizar([(400., 1000., 50.)])
        self.assertIsNone(self.s.color)


class IdentidadSinCamaraTests(unittest.TestCase):
    """El pilar sale del encuadre al empezar la maniobra del anterior y no
    vuelve. Corrida 031133: 24,5 grados en un campo efectivo de 26,9."""

    def sembrar(self, x=400., y=1000., activo_actual=True):
        s = SeguimientoLidar()
        actual = SimpleNamespace(id=1, activo=activo_actual, x=0., y=300.)
        ref = SimpleNamespace(fuente='lidar', confirmaciones=3, en_seccion=True,
                              t=10., color='ROJO', x=x, y=y, sigma=30.)
        s.actualizar(ref, [(x, y, 50.)], actual, 0, 0, 10., lambda a, b: True)
        ref.fuente = 'prediccion'
        return s, ref, actual

    def medir(self, s, ref, actual, hasta, x=400., y=1000.):
        """Mide sin parar hasta `hasta`; devuelve el instante del ultimo ciclo."""
        t, n = 10.1, 7
        while t <= hasta + 1e-9:
            s.actualizar(ref, [(x, y, 50.)], actual, 0, 0, t,
                         lambda a, b: True, (n, t - 0.02))
            ultimo = t
            t, n = round(t + 0.1, 3), n + 1
        return ultimo

    def test_la_identidad_sobrevive_al_reloj_viejo_de_cinco_segundos(self):
        s, ref, actual = self.sembrar()
        ultimo = self.medir(s, ref, actual, 16.5)   # 6,5 s sin ver el color
        self.assertEqual((s.color, s.fuente), ('ROJO', 'lidar_seguimiento'))
        self.assertEqual(s.t_color, 10.)
        self.assertTrue(s.autoriza_control(ultimo))

    def test_perder_el_cluster_si_la_mata(self):
        s, ref, actual = self.sembrar()
        self.medir(s, ref, actual, 11.0)
        # 1,6 s sin candidato: la continuidad se rompe y la identidad cae.
        t, n = 11.1, 40
        while t <= 12.7:
            s.actualizar(ref, [], actual, 0, 0, t, lambda a, b: True, (n, t - 0.02))
            t, n = round(t + 0.1, 3), n + 1
        self.assertIsNone(s.color)
        self.assertEqual(s.motivo, 'caducado')

    def test_sirve_para_fichar_lo_que_ya_no_autoriza_anticipar(self):
        # Cerca del pilar la anticipacion se apaga a proposito, pero la
        # FSM no puede recogerlo sin camara: ahi hace falta la identidad.
        s, ref, actual = self.sembrar(x=300., y=300., activo_actual=False)
        ultimo = self.medir(s, ref, actual, 10.5, x=300., y=300.)
        self.assertLess(math.hypot(s.x, s.y), 500.)
        self.assertFalse(s.autoriza_control(ultimo))
        self.assertEqual(s.identidad_para_captura(ultimo), ('ROJO', 300., 300.))

    def test_una_prediccion_no_sirve_para_fichar(self):
        s, ref, actual = self.sembrar()
        ultimo = self.medir(s, ref, actual, 10.1)
        self.assertIsNotNone(s.identidad_para_captura(ultimo))
        s.actualizar(ref, [], actual, 0, 0, 10.2, lambda a, b: True, (99, 10.18))
        self.assertEqual(s.fuente, 'prediccion')
        self.assertIsNone(s.identidad_para_captura(10.2))

    def test_el_tope_duro_sigue_existiendo(self):
        s, ref, actual = self.sembrar()
        self.medir(s, ref, actual, 31.0)            # 21 s de acarreo
        self.assertIsNone(s.color)


if __name__ == '__main__':
    unittest.main()
