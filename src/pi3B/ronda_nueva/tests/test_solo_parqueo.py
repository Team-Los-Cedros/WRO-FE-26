"""Puerta directa al parqueo para el banco de pruebas (``--solo-parqueo``).

El unico acceso normal al estacionamiento esta despues de verificar una
esquina, y completar una esquina es lo que bloquea el radio de giro real.
Estas pruebas fijan las dos mitades del compromiso: que la puerta funcione
cuando el banco la pide, y que nada la pueda abrir por accidente.
"""

import copy
import io
import unittest
from contextlib import redirect_stderr, redirect_stdout

from src.pi3B.ronda_nueva.config import (
    ErrorConfiguracion,
    cargar_configuracion,
    validar_configuracion,
)
from src.pi3B.ronda_nueva.control_ruta import ControlRuta
from src.pi3B.ronda_nueva.ronda_nueva import _argumentos, main
from src.pi3B.ronda_nueva.tests.test_control_ruta import corredor


class PuertaDirectaAlParqueoTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()

    def test_con_cero_esquinas_entra_a_parqueo_al_fijar_el_sentido(self):
        config = copy.deepcopy(self.config)
        config["control"]["corners_before_parking"] = 0
        control = ControlRuta(config)

        orden = control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)

        self.assertEqual(control.estado, "PARKING")
        self.assertEqual(control.sentido, 1)
        # Arranca quieto y con la direccion al centro, igual que la entrada
        # normal al parqueo: el hueco de este barrido se calculo con lado 0.
        self.assertEqual(orden.velocidad, 0)
        self.assertEqual(orden.angulo, 0.0)
        self.assertNotEqual(control.lado_parqueo_solicitado, 0)

    def test_sin_sentido_resuelto_no_entra_al_parqueo(self):
        # El lado de la bahia se deriva del sentido de carrera; entrar antes
        # de conocerlo aparcaria contra el lado equivocado.
        config = copy.deepcopy(self.config)
        config["control"]["corners_before_parking"] = 0
        config["control"]["turn_direction"] = "AUTO"
        control = ControlRuta(config)

        control.procesar(corredor(), (), 0.0, "PISTA", ahora=0.0)

        self.assertEqual(control.estado, "WAIT_DIRECTION")
        self.assertEqual(control.lado_parqueo_solicitado, 0)

    def test_la_configuracion_normal_sigue_yendo_a_crucero(self):
        # La regresion que importa: con el valor de carrera nada cambia.
        self.assertGreaterEqual(
            int(self.config["control"]["corners_before_parking"]), 1
        )
        control = ControlRuta(self.config)
        orden = control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)
        self.assertEqual(control.estado, "CRUISE")
        self.assertGreater(orden.velocidad, 0)

    def test_ningun_json_puede_abrir_la_puerta_por_si_solo(self):
        # La puerta solo la abre el flag, que muta la config ya validada.
        for valor in (0, -1):
            with self.subTest(valor=valor):
                config = copy.deepcopy(self.config)
                config["control"]["corners_before_parking"] = valor
                with self.assertRaises(ErrorConfiguracion):
                    validar_configuracion(config)


class ArgumentosTests(unittest.TestCase):
    def test_las_dos_opciones_de_alcance_son_excluyentes(self):
        self.assertTrue(_argumentos(["--solo-parqueo"]).solo_parqueo)
        self.assertTrue(_argumentos(["--sin-parqueo"]).sin_parqueo)
        # argparse escribe el uso en stderr antes de salir; se silencia para
        # no ensuciar la salida de la suite.
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            _argumentos(["--sin-parqueo", "--solo-parqueo"])

    def test_por_defecto_ninguna_esta_activa(self):
        args = _argumentos([])
        self.assertFalse(args.solo_parqueo)
        self.assertFalse(args.sin_parqueo)


class ValidarConfigTests(unittest.TestCase):
    def _pendientes(self, argv):
        salida = io.StringIO()
        with redirect_stdout(salida):
            self.assertEqual(main(argv), 0)
        for linea in salida.getvalue().splitlines():
            if linea.startswith("Calibraciones pendientes:"):
                return linea.split(":", 1)[1].strip()
        self.fail("no se reporto la linea de calibraciones pendientes")

    def test_solo_parqueo_omite_parking_ready_pero_no_las_demas(self):
        completo = self._pendientes(["--validar-config"])
        self.assertIn("parking_ready", completo)

        acotado = self._pendientes(["--validar-config", "--solo-parqueo"])
        self.assertNotIn("parking_ready", acotado)
        # Las otras calibraciones fisicas se siguen exigiendo: el flag acota
        # el alcance de la prueba, no desarma el resto de salvaguardas.
        for pendiente in completo.split(", "):
            if pendiente != "parking_ready":
                self.assertIn(pendiente, acotado)

    def test_el_reporte_no_altera_la_configuracion_en_disco(self):
        self._pendientes(["--validar-config", "--solo-parqueo"])
        config = cargar_configuracion()
        self.assertFalse(config["calibration"]["parking_ready"])
        self.assertGreaterEqual(
            int(config["control"]["corners_before_parking"]), 1
        )


if __name__ == "__main__":
    unittest.main()
