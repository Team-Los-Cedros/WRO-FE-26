"""Enlace con la Pico: parseo de telemetria y compensacion de deriva.

El objeto real abre un puerto serie en el constructor, asi que aqui se
construye sin llamarlo y se le ponen los campos a mano.  Lo que se prueba es
la aritmetica, que es donde estaba el problema.
"""

import threading
import time
import unittest

from ..hardware import EnlacePicoNuevo


def _enlace_falso():
    """Un enlace sin puerto: solo los campos que usa la aritmetica del rumbo."""

    enlace = EnlacePicoNuevo.__new__(EnlacePicoNuevo)
    enlace._lock = threading.Lock()
    enlace._yaw_crudo = 0.0
    enlace._cero_yaw = None
    enlace._deriva_deg_s = 0.0
    enlace._t_deriva = 0.0
    enlace._color_piso = "DESCONOCIDO"
    enlace._ultrasonido_mm = None
    enlace._watchdog_comando = None
    enlace._t_telemetria = 0.0
    return enlace


class PruebaRumbo(unittest.TestCase):
    def test_sin_cero_el_rumbo_es_cero(self):
        enlace = _enlace_falso()
        enlace._yaw_crudo = 123.0
        self.assertEqual(enlace.heading(), 0.0)

    def test_el_cero_resta_el_valor_del_momento(self):
        enlace = _enlace_falso()
        enlace._yaw_crudo = 123.0
        enlace.fijar_cero()
        enlace._yaw_crudo = 133.0
        self.assertAlmostEqual(enlace.heading(), 10.0, places=3)

    def test_la_deriva_medida_se_resta_linealmente(self):
        """El caso real: +20,9 grados/s de sesgo con el robot quieto.

        El firmware de la Pico promedia 100 muestras al arrancar para quitarlo,
        pero se traga sus excepciones: si el I2C no responde todavia, el offset
        se queda en cero y el sesgo entra entero.  Medido el 04-09 en el robot:
        +20,94 deg/s, perfectamente lineal, o sea 42 grados de error en cada
        esquina de dos segundos.
        """

        enlace = _enlace_falso()
        enlace._cero_yaw = 0.0
        enlace._deriva_deg_s = 20.94
        enlace._t_deriva = time.monotonic()
        # Tres segundos despues, el crudo habria subido 62,8 grados sin girar.
        enlace._yaw_crudo = 20.94 * 3.0
        enlace._t_deriva -= 3.0
        self.assertAlmostEqual(enlace.heading(), 0.0, delta=0.15)

    def test_un_giro_real_sobrevive_a_la_compensacion(self):
        """Compensar la deriva no puede comerse el giro de verdad."""

        enlace = _enlace_falso()
        enlace._cero_yaw = 0.0
        enlace._deriva_deg_s = 20.94
        enlace._t_deriva = time.monotonic() - 2.0
        # En dos segundos: 41,9 de deriva mas 90 de giro real.
        enlace._yaw_crudo = 20.94 * 2.0 + 90.0
        self.assertAlmostEqual(enlace.heading(), 90.0, delta=0.15)

    def test_fijar_cero_borra_la_deriva_anterior(self):
        enlace = _enlace_falso()
        enlace._deriva_deg_s = 5.0
        enlace.fijar_cero()
        self.assertEqual(enlace.deriva_deg_s, 0.0)


class PruebaTelemetria(unittest.TestCase):
    def test_trama_completa(self):
        trama = EnlacePicoNuevo.parsear_telemetria_completa(
            "IMU:-12.50,COLOR:PISTA,US:842,WD:OK"
        )
        self.assertIsNotNone(trama)
        self.assertAlmostEqual(trama.yaw, -12.5)
        self.assertEqual(trama.color, "PISTA")
        self.assertEqual(trama.watchdog, "OK")
        self.assertAlmostEqual(trama.ultrasonido_mm, 842.0)

    def test_sin_sensor_de_color_la_trama_sigue_valiendo(self):
        """En la Pi 5 el TCS3472 responde SIN_SENSOR y no debe invalidar nada.

        El sentido de giro y el conteo de vueltas ya no dependen de el: las
        lineas del piso se leen con la camara.
        """

        trama = EnlacePicoNuevo.parsear_telemetria_completa(
            "IMU:3.00,COLOR:SIN_SENSOR,US:-1,WD:STOP"
        )
        self.assertIsNotNone(trama)
        self.assertEqual(trama.color, "SIN_SENSOR")
        self.assertIsNone(trama.ultrasonido_mm)

    def test_basura_no_pasa(self):
        self.assertIsNone(EnlacePicoNuevo.parsear_telemetria_completa("hola"))
        self.assertIsNone(EnlacePicoNuevo.parsear_telemetria_completa(""))


if __name__ == "__main__":
    unittest.main()
