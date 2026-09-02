"""Reactividad de la evasion: slew propio y ganancia dependiente de distancia.

Las dos comprobaciones que importan aqui no son "el numero salio distinto"
sino que la direccion pueda llegar al angulo pedido dentro del tramo que
queda hasta el pilar, y que la respuesta sea mas viva de cerca que de lejos.
"""

import copy
import unittest

from src.pi3B.ronda_nueva.config import cargar_configuracion
from src.pi3B.ronda_nueva.control_ruta import ControlRuta
from src.pi3B.ronda_nueva.tests.test_control_ruta import corredor, track


class SlewDeEvasionTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()

    def _hasta_aproximacion(self, control, color="VERDE", y=600.0, x=-260.0):
        control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)
        orden = control.procesar(
            corredor(), (track(70, color, x=x, y=y),), 0.0, "PISTA", ahora=0.1
        )
        self.assertEqual(control.estado, "AVOID_APPROACH")
        return orden

    def test_la_evasion_usa_su_propio_slew_y_no_el_de_carril(self):
        control = ControlRuta(self.config)
        esperado = float(self.config["control"]["avoid_steering_slew_deg_per_scan"])
        carril = float(self.config["control"]["steering_slew_deg_per_scan"])
        self.assertGreater(esperado, carril)
        self.assertEqual(control._slew_evasion(), esperado)

        # Primer barrido de evasion, saliendo del centro: el pilar pide mucho
        # mas angulo del que cabe en un barrido, asi que la orden queda
        # exactamente en el tope del slew. Ese tope tiene que ser el de
        # evasion, no el de carril.
        orden = self._hasta_aproximacion(control)
        self.assertGreater(abs(orden.angulo), carril)
        self.assertAlmostEqual(abs(orden.angulo), esperado)

    def test_sin_la_clave_nueva_el_slew_de_evasion_es_el_historico(self):
        config = copy.deepcopy(self.config)
        del config["control"]["avoid_steering_slew_deg_per_scan"]
        control = ControlRuta(config)
        self.assertEqual(
            control._slew_evasion(),
            float(config["control"]["steering_slew_deg_per_scan"]),
        )

    def test_alcanza_el_tope_de_direccion_en_menos_barridos(self):
        # El pilar dispara a 950 mm y a 40 PWM (160 mm/s) quedan unos 4 s,
        # pero la maniobra util son los primeros barridos. Con el slew de
        # carril se necesitaban 5 barridos para llegar al tope; se comprueba
        # que ahora bastan 2.
        rapido = ControlRuta(self.config)
        self._hasta_aproximacion(rapido)
        tope = float(self.config["control"]["steering_max_left_deg"])
        barridos = 0
        angulo = 0.0
        while barridos < 10 and abs(angulo) < tope - 0.01:
            barridos += 1
            angulo = rapido.procesar(
                corredor(), (track(70, "VERDE", x=-400.0, y=400.0),),
                0.0, "PISTA", ahora=0.2 + 0.1 * barridos,
            ).angulo
        self.assertLessEqual(barridos, 2)


class GananciaPersecucionTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()

    def test_la_ganancia_crece_al_acercarse_y_se_satura_en_los_extremos(self):
        control = ControlRuta(self.config)
        lejos = float(self.config["control"]["obstacle_pursuit_far_mm"])
        cerca = float(self.config["control"]["obstacle_pursuit_near_mm"])
        kp_lejos = float(self.config["control"]["obstacle_pursuit_kp"])
        kp_cerca = float(self.config["control"]["obstacle_pursuit_kp_near"])

        self.assertAlmostEqual(control._ganancia_persecucion(lejos), kp_lejos)
        self.assertAlmostEqual(control._ganancia_persecucion(cerca), kp_cerca)
        # Fuera del tramo no extrapola: satura en los dos extremos.
        self.assertAlmostEqual(control._ganancia_persecucion(5000.0), kp_lejos)
        self.assertAlmostEqual(control._ganancia_persecucion(10.0), kp_cerca)
        # Y en medio es monotona.
        medio = control._ganancia_persecucion((lejos + cerca) / 2.0)
        self.assertGreater(medio, kp_lejos)
        self.assertLess(medio, kp_cerca)

    def test_sin_las_claves_nuevas_la_ganancia_es_la_de_siempre(self):
        config = copy.deepcopy(self.config)
        for clave in (
            "obstacle_pursuit_kp_near",
            "obstacle_pursuit_far_mm",
            "obstacle_pursuit_near_mm",
        ):
            del config["control"][clave]
        control = ControlRuta(config)
        kp = float(config["control"]["obstacle_pursuit_kp"])
        for distancia in (100.0, 500.0, 900.0, 3000.0):
            self.assertAlmostEqual(control._ganancia_persecucion(distancia), kp)

    def test_una_ganancia_cercana_menor_no_reduce_la_reactividad(self):
        # Proteccion contra una calibracion equivocada: si alguien deja la
        # ganancia de cerca por debajo de la de lejos, se ignora en vez de
        # volver el robot mas lento justo donde hace falta reaccionar.
        config = copy.deepcopy(self.config)
        config["control"]["obstacle_pursuit_kp_near"] = 0.4
        control = ControlRuta(config)
        kp = float(config["control"]["obstacle_pursuit_kp"])
        self.assertAlmostEqual(control._ganancia_persecucion(300.0), kp)

    def test_la_ganancia_cercana_se_nota_en_la_orden_emitida(self):
        # Misma geometria exacta, unica diferencia la ganancia de cerca. Se
        # elige un pilar que pida un angulo pequeno para que la comparacion
        # no quede escondida detras del tope de direccion, y se suelta el
        # slew para medir el angulo deseado y no la rampa.
        base = copy.deepcopy(self.config)
        base["control"]["avoid_steering_slew_deg_per_scan"] = 100.0
        base["control"]["speed_slew_pwm_per_scan"] = 100

        sin_anticipacion = copy.deepcopy(base)
        del sin_anticipacion["control"]["obstacle_pursuit_kp_near"]

        angulos = {}
        for etiqueta, config in (
            ("con", base), ("sin", sin_anticipacion)
        ):
            control = ControlRuta(copy.deepcopy(config))
            control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)
            orden = control.procesar(
                corredor(), (track(70, "VERDE", x=119.0, y=340.0),),
                0.0, "PISTA", ahora=0.1,
            )
            self.assertEqual(control.estado, "AVOID_APPROACH")
            angulos[etiqueta] = orden.angulo

        tope = float(self.config["control"]["steering_max_left_deg"])
        self.assertLess(abs(angulos["con"]), tope)
        self.assertGreater(abs(angulos["con"]), abs(angulos["sin"]))


if __name__ == "__main__":
    unittest.main()
