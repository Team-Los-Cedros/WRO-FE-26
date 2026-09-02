"""Pruebas del desacople entre la adquisicion LiDAR y el ciclo de control."""

import threading
import time
import unittest

from src.pi3B.ronda_nueva.tests.apoyo import asegurar_pyserial

asegurar_pyserial()

from src.pi3B.comun.lidar_driver import LidarDriver  # noqa: E402
from src.pi3B.ronda_nueva.config import cargar_configuracion  # noqa: E402
from src.pi3B.ronda_nueva.ronda_nueva import AplicacionRondaNueva
from src.pi3B.ronda_nueva.sincronizacion import BuzonBarridosLidar


class BuzonBarridosLidarTests(unittest.TestCase):
    def test_conserva_solo_el_ultimo_y_cuenta_lo_descartado(self):
        buzon = BuzonBarridosLidar()
        buzon.publicar([(0.0, 100.0)], 1.0)
        buzon.publicar([(1.0, 200.0)], 2.0)
        buzon.publicar([(2.0, 300.0)], 3.0)

        barrido = buzon.tomar()
        self.assertEqual(barrido.timestamp, 3.0)
        self.assertEqual(barrido.muestras, [(2.0, 300.0)])
        # Dos barridos quedaron obsoletos antes de poder consumirse: eso es
        # exactamente lo que no debe llegar al control.
        self.assertEqual(buzon.descartados, 2)
        self.assertEqual(buzon.recibidos, 3)

    def test_la_casilla_queda_vacia_tras_tomar(self):
        buzon = BuzonBarridosLidar()
        buzon.publicar([(0.0, 100.0)], 1.0)
        self.assertIsNotNone(buzon.tomar())
        self.assertIsNone(buzon.tomar())
        self.assertEqual(buzon.descartados, 0)

    def test_tomar_espera_y_despierta_al_publicar(self):
        buzon = BuzonBarridosLidar()
        recibido = []

        def consumir():
            recibido.append(buzon.tomar(2.0))

        hilo = threading.Thread(target=consumir)
        hilo.start()
        time.sleep(0.05)
        buzon.publicar([(3.0, 400.0)], 9.0)
        hilo.join(2.0)

        self.assertFalse(hilo.is_alive())
        self.assertEqual(recibido[0].timestamp, 9.0)

    def test_tomar_vacio_devuelve_none_sin_colgarse(self):
        buzon = BuzonBarridosLidar()
        inicio = time.monotonic()
        self.assertIsNone(buzon.tomar(0.05))
        self.assertLess(time.monotonic() - inicio, 1.0)


class AcoplamientoConElDriverTests(unittest.TestCase):
    def setUp(self):
        self.app = AplicacionRondaNueva(cargar_configuracion())

    def test_el_driver_le_pasa_el_timestamp_al_callback_de_la_aplicacion(self):
        # Si esta deteccion fallara, el driver llamaria con un solo argumento y
        # el barrido llegaria sin instante de captura: la edad medida en
        # telemetria dejaria de significar nada.
        self.assertTrue(LidarDriver._acepta_timestamp(self.app._al_barrido))

    def test_el_callback_del_hilo_lidar_solo_publica(self):
        # No hay enlace ni geometria abiertos: si el callback intentara decidir
        # aqui mismo, esta llamada reventaria o dejaria motivo de fallo.
        self.app._al_barrido([(10.0, 500.0)], 42.0)

        self.assertEqual(self.app._motivo_fin, "")
        barrido = self.app.buzon_barridos.tomar()
        self.assertEqual(barrido.timestamp, 42.0)
        self.assertEqual(barrido.muestras, [(10.0, 500.0)])

    def test_procesar_barrido_sin_hardware_no_arranca_el_ciclo(self):
        self.app._al_barrido([(10.0, 500.0)], 42.0)
        self.app._procesar_barrido(self.app.buzon_barridos.tomar())
        self.assertEqual(self.app._motivo_fin, "")


if __name__ == "__main__":
    unittest.main()
