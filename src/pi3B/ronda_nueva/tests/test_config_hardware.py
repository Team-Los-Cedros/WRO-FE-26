import copy
import unittest

from src.pi3B.ronda_nueva.config import (
    ErrorConfiguracion,
    calibraciones_pendientes,
    cargar_configuracion,
    exigir_listo_para_mover,
    validar_configuracion,
)
from src.pi3B.ronda_nueva.hardware import EnlacePicoNuevo


class ConfiguracionTests(unittest.TestCase):
    def test_config_porta_las_mediciones_del_mastil(self):
        config = cargar_configuracion()
        camara = config["camera"]
        self.assertEqual((camara["width"], camara["height"]), (640, 360))
        self.assertEqual(camara["picamera_format"], "RGB888")
        self.assertEqual(camara["array_color_order"], "BGR")
        self.assertEqual(camara["raw_sensor_size"], [2304, 1296])
        self.assertAlmostEqual(camara["hfov_deg"], 68.16865, places=4)
        self.assertAlmostEqual(camara["principal_x_px"], 352.074, places=3)
        self.assertAlmostEqual(camara["forward_from_lidar_mm"], -99.76, places=2)
        self.assertEqual(config["lidar"]["blind_sectors_deg"], [[163.0, 191.0]])

    def test_movimiento_completo_sigue_bloqueado_hasta_calibrar_parqueo(self):
        config = cargar_configuracion()
        config["runtime"]["motion_enabled"] = True
        self.assertEqual(
            calibraciones_pendientes(config),
            [
                "camera_lidar_timing_ready",
                "parking_ready",
                "vision_ground_support_ready",
            ],
        )
        with self.assertRaises(ErrorConfiguracion):
            exigir_listo_para_mover(config)

        # Tras validar vision en movimiento, una prueba expresamente sin
        # parqueo puede armarse; el JSON entregado sigue bloqueado por defecto.
        config["calibration"]["camera_lidar_timing_ready"] = True
        config["calibration"]["vision_ground_support_ready"] = True
        exigir_listo_para_mover(config, incluir_estacionamiento=False)

    def test_orden_de_array_invalido_se_rechaza(self):
        config = cargar_configuracion()
        config["camera"]["array_color_order"] = "XYZ"
        with self.assertRaises(ErrorConfiguracion):
            validar_configuracion(config)

    def test_modo_raw_invalido_se_rechaza(self):
        config = cargar_configuracion()
        config["camera"]["raw_sensor_size"] = [2304]
        with self.assertRaises(ErrorConfiguracion):
            validar_configuracion(config)

    def test_umbrales_de_seguridad_de_parqueo_se_validan(self):
        config = cargar_configuracion()
        self.assertGreater(
            config["parking"]["minimum_lateral_clearance_mm"],
            config["parking"]["robot_width_mm"] / 2.0,
        )
        config["parking"]["minimum_rear_diagonal_coverage"] = 0.0
        with self.assertRaises(ErrorConfiguracion):
            validar_configuracion(config)


class EnlacePicoTests(unittest.TestCase):
    def test_parsea_imu_con_y_sin_color(self):
        self.assertEqual(
            EnlacePicoNuevo.parsear_telemetria("IMU:-123.5,COLOR:azul"),
            (-123.5, "AZUL"),
        )
        self.assertEqual(
            EnlacePicoNuevo.parsear_telemetria("IMU:8.25"),
            (8.25, None),
        )
        self.assertIsNone(EnlacePicoNuevo.parsear_telemetria("COLOR:ROJO"))


if __name__ == "__main__":
    unittest.main()
