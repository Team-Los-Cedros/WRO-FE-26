"""Regresiones del objetivo #7, arbitraje y cierre de carrera. Sin motores."""
import math
import os
import sys
import unittest
from dataclasses import replace
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from test_geometria import MedicionFalsa, SectorFalso
import navegacion
import tracker
from retorno_parqueo import RetornoParqueo
from parqueo.modelos import HuecoParqueo, MapaParedes, Recta
from parqueo.estacionamiento import VERIFICAR, ARCO_ENTRADA, FALLO, LISTO


class RegresionesPilares(unittest.TestCase):
    def setUp(self):
        self.nav = navegacion.Navegador(SectorFalso())
        self.nav.fase = "CARRERA"
        self.med = MedicionFalsa()

    def test_no_captura_mientras_recupera_aborta_o_retrocede(self):
        for estado in ("RECUPERACION", "ABORTO", "RETROCESO", "GIRO_FORZADO"):
            with self.subTest(estado=estado):
                self.nav.estado = estado
                self.nav._t_estado = 0
                with patch.object(self.nav, "_intentar_capturar") as captura:
                    self.nav.procesar(self.med, "VERDE", 0, ahora=0.1)
                    captura.assert_not_called()

    def test_captura_en_crucero(self):
        with patch.object(self.nav, "_intentar_capturar") as captura:
            self.nav.procesar(self.med, "VERDE", 0, ahora=1)
            captura.assert_called_once()

    def test_objetivo_siete_no_se_confirma_tarde(self):
        # Coordenadas DEL LIDAR del log 76.143: +210/-81. Desde el eje
        # trasero y sigue siendo +47, pero ya no queda espacio para abrir.
        for estado in ("CRUCERO", "CONFIRMACION", "COMPROMISO"):
            with self.subTest(estado=estado):
                nav = navegacion.Navegador(SectorFalso())
                nav.fase, nav.estado = "CARRERA", estado
                nav.tracker.iniciar("VERDE", 1, 210, -81, -547, ahora=10)
                nav.tracker.asociaciones = 4
                nav._ciclos_confirmacion = 3
                nav.procesar(self.med, "VERDE", -547, ahora=10)
                self.assertFalse(nav.tracker.activo)
                self.assertEqual(nav.estado, "CRUCERO")

    def test_reloj_replay_no_se_mezcla_con_reloj_real(self):
        trk = tracker.TrackerObstaculo()
        trk.iniciar("ROJO", -1, 0, 400, 0, ahora=0)
        self.assertTrue(trk.asociar([(0, 400, 50)], ahora=1))
        self.assertFalse(trk.perdido(3.9))
        self.assertTrue(trk.perdido(4.1))

    def test_sin_salida_no_emite_velocidad_positiva(self):
        med = MedicionFalsa(frontal=220, izquierda=820, derecha=194, trasera=700)
        vel, _ = self.nav.procesar(med, None, -625, ahora=0)
        self.assertLess(vel, 0)
        self.assertEqual(self.nav.estado, "RETROCESO")

    def test_no_retrocede_con_trasera_ciega_o_ocupada(self):
        for distancia in (43, 200, 8000, float("nan")):
            with self.subTest(distancia=distancia):
                nav = navegacion.Navegador(SectorFalso())
                nav.fase = "CARRERA"
                med = MedicionFalsa(frontal=60, trasera=distancia)
                self.assertEqual(nav.procesar(med, None, 0, ahora=0)[0], 0)
                self.assertEqual(nav.procesar(med, None, 0, ahora=4)[0], 0)
                self.assertEqual(nav.fase, "FALLO")

    def test_giro_forzado_respeta_emergencia(self):
        self.nav.estado = "GIRO_FORZADO"
        self.nav._signo_giro_forzado = -1
        med = MedicionFalsa(frontal=60, derecha=65, trasera=44)
        self.assertEqual(self.nav.procesar(med, None, 0, ahora=0)[0], 0)
        self.assertEqual(self.nav.estado, "RETROCESO")

    def test_giro_forzado_respeta_envolvente_antes_de_emergencia(self):
        self.nav.estado = "GIRO_FORZADO"
        self.nav._signo_giro_forzado = -1
        med = MedicionFalsa(frontal=210, izquierda=815, derecha=176, trasera=700)
        self.assertLessEqual(self.nav.procesar(med, None, 0, ahora=0)[0], 0)


def paredes(t=1, frontal=1000, lateral=100):
    return MapaParedes(t, frontal=Recta(frontal, 0, 2, 20, 0.95),
                       izquierda=Recta(lateral, -90, 2, 20, 0.95),
                       frontal_min_mm=80)


class RegresionesRetorno(unittest.TestCase):
    def retorno_listo(self):
        r = RetornoParqueo()
        r.origen = {"frontal_mm": 1000, "normal_frontal": 0}
        r._h0, r.sentido, r.lado, r.esquinas = 0, -1, -1, 12
        r.paredes = paredes()
        r.hueco = HuecoParqueo(1, -1, -310, 40, -135, 350, 200, 0.95)
        return r

    def test_dos_vueltas_y_1010_no_parquean(self):
        r = self.retorno_listo()
        for yaw in (-720, -1010):
            self.assertFalse(any(r.listo_para_parquear(yaw, 1) for _ in range(6)))

    def test_tres_vueltas_misma_bahia_persistente(self):
        r = self.retorno_listo()
        self.assertFalse(r.listo_para_parquear(-1080, 1))
        self.assertFalse(r.listo_para_parquear(-1080, 1))
        self.assertTrue(r.listo_para_parquear(-1080, 1))

    def test_firma_lateral_repetida_no_es_origen(self):
        r = self.retorno_listo()
        r.paredes = paredes(frontal=2200)
        self.assertFalse(any(r.listo_para_parquear(-1080, 1) for _ in range(6)))

    def test_hueco_antiguo_y_esquinas_incompletas_no_cierran(self):
        r = self.retorno_listo()
        self.assertFalse(r.listo_para_parquear(-1080, 2))
        r.esquinas = 8
        self.assertFalse(r.listo_para_parquear(-1080, 1))

    def test_salto_de_imu_es_fallo(self):
        r = RetornoParqueo()
        r.observar(MedicionFalsa(), 0, 1, 0, 0, 0)
        r.observar(MedicionFalsa(), -360, 1.1, 0, 0, 0.1)
        self.assertIn("salto", r.error)

    def test_sin_ultrasonido_no_retrocede_ni_declara_exito(self):
        r = self.retorno_listo()
        r.control.estado = ARCO_ENTRADA
        for _ in range(3):
            cmd = r.parquear(-1080, None, 1)
            self.assertEqual(cmd.velocidad, 0)
            self.assertFalse(cmd.verificado)
        self.assertEqual(cmd.estado, FALLO)

    def test_verificacion_exige_robot_entero_dentro(self):
        # El seguidor debe exigir que la pose caiga dentro de la meta
        # volumetrica Y que el ultrasonido confirme espacio trasero razonable.
        # Sin ultrasonido valido, el retorno ya falla en las guardas previas;
        # aqui se comprueba que un US fuera de rango no produce LISTO.
        r = self.retorno_listo()
        r.control._t_inicio = 0
        r.paredes = paredes(t=1)
        # US < min_mm (20) -> no es valido:
        cmd = r.parquear(-1080, 15, 1)
        self.assertFalse(cmd.verificado)
        # Con US valido pero sin seguidor todavia: primer ciclo crea el seguidor
        # y devuelve APROXIMAR_BAHIA, que no es verificado.
        r2 = self.retorno_listo()
        r2.control._t_inicio = 0
        r2.paredes = paredes(t=1)
        cmd2 = r2.parquear(-1080, 88, 1)
        self.assertFalse(cmd2.verificado)

    def test_timeout_no_es_estacionado(self):
        r = self.retorno_listo()
        r.control._t_inicio = 0
        r.paredes = paredes(t=100)
        # Avanzar el reloj mucho mas que el timeout total (90 s del seguidor):
        cmd = r.parquear(-1080, 88, 100)
        self.assertEqual(cmd.velocidad, 0)
        self.assertFalse(cmd.verificado)

    def test_envolvente_detecta_delimitador_y_libera_muro_lejano(self):
        r = RetornoParqueo()
        r.scan = [(0, 40), (90, 500), (270, 500)]
        self.assertFalse(r._barrido_seguro(20, 0))
        r.scan = [(0, 1000), (90, 500), (270, 500)]
        self.assertTrue(r._barrido_seguro(20, 0))


if __name__ == "__main__":
    unittest.main()
