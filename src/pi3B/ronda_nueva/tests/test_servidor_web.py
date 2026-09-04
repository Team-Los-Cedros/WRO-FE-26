import json
import sys
import threading
import unittest
import urllib.request
from pathlib import Path


RAIZ_REPO = Path(__file__).resolve().parents[4]
if str(RAIZ_REPO) not in sys.path:
    sys.path.insert(0, str(RAIZ_REPO))

from src.pi3B.ronda_nueva.servidor_web import EstadoRobot, ServidorPanel


class EstadoRobotTests(unittest.TestCase):
    def test_publicar_y_leer_es_un_snapshot_independiente(self):
        estado = EstadoRobot()
        estado.publicar(estado_fsm="CRUISE", frontal=1200.0)
        copia = estado.leer()
        copia["frontal"] = 0.0
        self.assertEqual(estado.leer()["frontal"], 1200.0)

    def test_el_barrido_se_submuestrea_y_descarta_distancias_nulas(self):
        estado = EstadoRobot(submuestreo_lidar=2)
        estado.publicar_barrido([(0.0, 100.0), (1.0, 0.0), (2.0, 300.0), (3.0, 400.0)])
        puntos = estado.leer()["lidar"]
        # Se toma uno de cada dos y el de distancia 0 no habria entrado igual.
        self.assertEqual(puntos, [(0.0, 100.0), (2.0, 300.0)])

    def test_sin_clientes_de_video_no_se_codifica_nada(self):
        estado = EstadoRobot()
        self.assertFalse(estado.quiere_video)

        class FrameQueExplota:
            @property
            def shape(self):
                raise AssertionError("no debe tocarse el frame sin clientes")

        estado.publicar_frame(FrameQueExplota())
        self.assertIsNone(estado.leer_frame()[0])

    def test_un_barrido_ilegible_no_rompe_al_que_publica(self):
        estado = EstadoRobot()
        estado.publicar_barrido([("no", "es")])
        self.assertEqual(estado.leer()["lidar"], [])


class ServidorPanelTests(unittest.TestCase):
    def setUp(self):
        self.estado = EstadoRobot()
        self.panel = ServidorPanel(self.estado, puerto=0)
        self.assertTrue(self.panel.arrancar(), self.panel.error)
        self.puerto = self.panel._servidor.server_address[1]
        self.addCleanup(self.panel.detener)

    def _pedir(self, ruta):
        url = "http://127.0.0.1:%d%s" % (self.puerto, ruta)
        with urllib.request.urlopen(url, timeout=5) as respuesta:
            return respuesta.status, respuesta.read()

    def test_sirve_la_pagina_y_el_estado_en_json(self):
        codigo, cuerpo = self._pedir("/")
        self.assertEqual(codigo, 200)
        self.assertIn(b"WRO-FE-26", cuerpo)

        self.estado.publicar(estado="PARKING", frontal=640.5)
        self.estado.publicar_barrido([(10.0, 500.0)])
        codigo, cuerpo = self._pedir("/estado.json")
        self.assertEqual(codigo, 200)
        datos = json.loads(cuerpo)
        self.assertEqual(datos["estado"], "PARKING")
        self.assertEqual(datos["frontal"], 640.5)
        self.assertEqual(datos["lidar"], [[10.0, 500.0]])

    def test_una_ruta_desconocida_no_tumba_el_servidor(self):
        with self.assertRaises(urllib.error.HTTPError) as caja:
            self._pedir("/no-existe")
        self.assertEqual(caja.exception.code, 404)
        self.assertEqual(self._pedir("/estado.json")[0], 200)


if __name__ == "__main__":
    unittest.main()
