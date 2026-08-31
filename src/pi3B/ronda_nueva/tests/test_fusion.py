import copy
import unittest

from src.pi3B.ronda_nueva.config import cargar_configuracion
from src.pi3B.ronda_nueva.fusion import FusionLigera
from src.pi3B.ronda_nueva.modelos import (
    DeteccionVisual,
    ObjetoLidar,
    PaqueteVision,
)


def objeto(timestamp, x, y):
    return ObjetoLidar(
        timestamp=timestamp,
        x_mm=x,
        y_mm=y,
        distancia_mm=(x * x + y * y) ** 0.5,
        bearing_deg=0.0,
        ancho_mm=95.0,
        puntos=8,
    )


def deteccion(timestamp, bearing, color="ROJO", confianza=0.9, x=175.1):
    return DeteccionVisual(
        timestamp=timestamp,
        color=color,
        bearing_deg=bearing,
        centro_px=(x, 130.0),
        bbox=(160, 70, 30, 95),
        area_ratio=0.05,
        bottom_ratio=0.69,
        confianza=confianza,
        soporte_suelo=1.0,
    )


class FusionLigeraTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()

    def test_extrinseca_medida_asocia_desde_el_origen_de_camara(self):
        fusion = FusionLigera(self.config)
        poste = objeto(100.0, 70.0, 420.0)
        rumbo_camara = fusion.bearing_camara_predicho(poste)
        paquete = PaqueteVision(100.0, (deteccion(100.0, rumbo_camara),))

        asociaciones = fusion.asociar((poste,), paquete)

        self.assertEqual(len(asociaciones), 1)
        self.assertAlmostEqual(asociaciones[0].residuo_deg, 0.0, places=7)

    def test_un_frame_y_un_objeto_solo_se_usan_una_vez(self):
        fusion = FusionLigera(self.config)
        p1 = objeto(10.0, -150.0, 650.0)
        p2 = objeto(10.0, 160.0, 650.0)
        d1 = deteccion(10.0, fusion.bearing_camara_predicho(p1), "VERDE", x=100)
        d2 = deteccion(10.0, fusion.bearing_camara_predicho(p2), "ROJO", x=240)
        paquete = PaqueteVision(10.0, (d1, d2))

        asociaciones = fusion.asociar((p1, p2), paquete)

        self.assertEqual(len(asociaciones), 2)
        self.assertEqual(len({a.objeto.x_mm for a in asociaciones}), 2)
        self.assertEqual(len({a.deteccion.centro_px for a in asociaciones}), 2)

    def test_confirmacion_requiere_dos_barridos_y_dos_frames_distintos(self):
        fusion = FusionLigera(self.config)
        p1 = objeto(20.0, 0.0, 700.0)
        b1 = fusion.bearing_camara_predicho(p1)
        paquete1 = PaqueteVision(20.0, (deteccion(20.0, b1),))
        tracks = fusion.actualizar((p1,), paquete1, 0.0, 0.0, timestamp=20.0)
        self.assertFalse(tracks[0].confirmado)

        # Repetir exactamente la misma observacion no suma impactos.
        tracks = fusion.actualizar((p1,), paquete1, 0.0, 0.0, timestamp=20.0)
        self.assertEqual(tracks[0].impactos_lidar, 1)
        self.assertEqual(tracks[0].impactos_color, 1)

        p2 = objeto(20.1, 0.0, 700.0)
        paquete2 = PaqueteVision(
            20.1, (deteccion(20.1, fusion.bearing_camara_predicho(p2)),)
        )
        tracks = fusion.actualizar((p2,), paquete2, 0.0, 0.0, timestamp=20.1)
        self.assertTrue(tracks[0].confirmado)
        self.assertEqual(tracks[0].color, "ROJO")

    def test_timestamp_de_captura_no_resta_latencia_dos_veces(self):
        config = copy.deepcopy(self.config)
        config["camera"]["latency_s"] = 0.08
        config["fusion"]["max_camera_lidar_age_s"] = 0.05
        fusion = FusionLigera(config)
        poste = objeto(30.0, 0.0, 600.0)
        paquete = PaqueteVision(
            30.0,
            (deteccion(30.0, fusion.bearing_camara_predicho(poste)),),
        )

        self.assertEqual(len(fusion.asociar((poste,), paquete)), 1)

    def test_paquete_fuera_de_ventana_temporal_se_rechaza(self):
        fusion = FusionLigera(self.config)
        poste = objeto(40.0, 0.0, 600.0)
        paquete = PaqueteVision(
            41.0,
            (deteccion(41.0, fusion.bearing_camara_predicho(poste)),),
        )

        self.assertEqual(fusion.asociar((poste,), paquete), ())


if __name__ == "__main__":
    unittest.main()
