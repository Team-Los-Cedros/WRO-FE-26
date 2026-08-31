import copy
import sys
import types
import unittest
from unittest.mock import patch

from src.pi3B.ronda_nueva.config import (
    ErrorConfiguracion,
    cargar_configuracion,
    validar_configuracion,
)
from src.pi3B.ronda_nueva.hardware import EnlacePicoNuevo, FuenteCamara
from src.pi3B.ronda_nueva.ronda_nueva import AplicacionRondaNueva


class _FrameFalso:
    def __getitem__(self, _indice):
        return self


def _modulo_picamera_falso(fallar_controles=False, fallar_inicio=False):
    class Picamera2Falsa:
        ultima_instancia = None

        def __init__(self):
            self.detenida = False
            self.argumentos_config = None
            self.__class__.ultima_instancia = self

        def create_video_configuration(self, **kwargs):
            self.argumentos_config = kwargs
            return {"config": "falsa"}

        def configure(self, _config):
            pass

        def start(self):
            if fallar_inicio:
                raise RuntimeError("sensor ausente")

        def capture_metadata(self):
            return {"ExposureTime": 1200, "AnalogueGain": 1.5}

        def set_controls(self, _controles):
            if fallar_controles:
                raise RuntimeError("control no soportado")

        def capture_array(self, _flujo):
            return _FrameFalso()

        def stop(self):
            self.detenida = True

    return types.SimpleNamespace(Picamera2=Picamera2Falsa), Picamera2Falsa


class FuenteCamaraTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(cargar_configuracion()["camera"])
        self.config["lock_auto_after_s"] = 0.0

    def test_ae_awb_no_soportado_es_advertencia_y_captura_continua(self):
        modulo, clase_camara = _modulo_picamera_falso(fallar_controles=True)
        fuente = FuenteCamara(self.config)
        llamadas = iter((True, False))
        frames = []

        def al_frame(frame, _timestamp):
            frames.append((frame, fuente.lista))

        with patch.dict(sys.modules, {"picamera2": modulo}), patch(
            "src.pi3B.ronda_nueva.hardware.time.sleep", return_value=None
        ):
            fuente.bucle(lambda: next(llamadas), al_frame)

        self.assertIsNone(fuente.ultimo_error)
        self.assertIn("AE/AWB", fuente.ultima_advertencia)
        self.assertEqual(len(frames), 1)
        self.assertTrue(frames[0][1])
        self.assertTrue(clase_camara.ultima_instancia.detenida)
        self.assertEqual(
            clase_camara.ultima_instancia.argumentos_config["raw"]["size"],
            (2304, 1296),
        )

    def test_fallo_de_inicio_si_es_error_fatal(self):
        modulo, _clase_camara = _modulo_picamera_falso(fallar_inicio=True)
        fuente = FuenteCamara(self.config)
        with patch.dict(sys.modules, {"picamera2": modulo}), patch(
            "src.pi3B.ronda_nueva.hardware.time.sleep", return_value=None
        ):
            fuente.bucle(lambda: False, lambda *_args: None)

        self.assertIn("sensor ausente", fuente.ultimo_error)
        self.assertIsNone(fuente.ultima_advertencia)
        self.assertFalse(fuente.lista)


class _EnlacePicoFalso:
    def __init__(self, estado=None, confirmar_al_enviar=False):
        self.estado = estado
        self.confirmar_al_enviar = confirmar_al_enviar
        self.envios = []
        self.detenciones = 0

    def enviar(self, velocidad, angulo):
        self.envios.append((velocidad, angulo))
        if self.confirmar_al_enviar:
            self.estado = "OK"
        return True

    def watchdog_comando_ok(self):
        return self.estado == "OK"

    def estado_watchdog_comando(self):
        return self.estado

    def detener(self):
        self.detenciones += 1


class WatchdogPicoTests(unittest.TestCase):
    def test_parser_historico_conserva_retorno_y_extendido_lee_wd(self):
        linea = "IMU:-12.5,COLOR:azul,WD:OK"
        self.assertEqual(
            EnlacePicoNuevo.parsear_telemetria(linea), (-12.5, "AZUL")
        )
        self.assertEqual(
            EnlacePicoNuevo.parsear_telemetria_extendida(linea),
            (-12.5, "AZUL", "OK"),
        )
        self.assertEqual(
            EnlacePicoNuevo.parsear_telemetria_extendida("IMU:1.0,WD:otro"),
            (1.0, None, "INVALIDO"),
        )
        self.assertEqual(
            EnlacePicoNuevo.parsear_telemetria_extendida("IMU:1.0"),
            (1.0, None, None),
        )

    def test_config_exige_booleano_para_watchdog_autonomo(self):
        config = cargar_configuracion()
        self.assertIs(config["hardware"]["require_pico_command_watchdog"], True)
        config["hardware"]["require_pico_command_watchdog"] = "true"
        with self.assertRaises(ErrorConfiguracion):
            validar_configuracion(config)

    def test_arranque_envia_freno_hasta_recibir_wd_ok(self):
        app = AplicacionRondaNueva(cargar_configuracion())
        enlace = _EnlacePicoFalso(estado="STOP", confirmar_al_enviar=True)
        app.enlace = enlace
        with patch(
            "src.pi3B.ronda_nueva.ronda_nueva.time.sleep", return_value=None
        ):
            self.assertTrue(app._esperar_watchdog_pico_listo())
        self.assertEqual(enlace.envios, [(0, 0.0)])
        self.assertTrue(app.corriendo)

    def test_firmware_sin_wd_no_puede_armar(self):
        app = AplicacionRondaNueva(cargar_configuracion())
        enlace = _EnlacePicoFalso(estado=None)
        app.enlace = enlace
        with patch(
            "src.pi3B.ronda_nueva.ronda_nueva.time.monotonic",
            side_effect=(10.0, 12.0),
        ):
            self.assertFalse(app._esperar_watchdog_pico_listo())
        self.assertFalse(app.corriendo)
        self.assertIn("NO_ANUNCIADO", app._motivo_fin)
        self.assertEqual(enlace.detenciones, 1)

    def test_wd_stop_en_ejecucion_ordena_parada(self):
        app = AplicacionRondaNueva(cargar_configuracion())
        enlace = _EnlacePicoFalso(estado="STOP")
        app.enlace = enlace
        self.assertFalse(app._comprobar_watchdog_pico_en_ejecucion())
        self.assertFalse(app.corriendo)
        self.assertEqual(enlace.detenciones, 1)


if __name__ == "__main__":
    unittest.main()
