import copy
import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from src.pi3B.ronda_nueva.config import cargar_configuracion
from src.pi3B.ronda_nueva.replay_captura import (
    ErrorFormatoCaptura,
    ejecutar_replay,
)


def scan_rectangular():
    """Pasillo centrado con eco del mastil dentro de la mascara medida."""

    puntos = []
    for angulo in range(360):
        radianes = math.radians(float(angulo))
        dx = math.sin(radianes)
        dy = math.cos(radianes)
        candidatos = []
        if dx > 1e-9:
            candidatos.append(500.0 / dx)
        elif dx < -1e-9:
            candidatos.append(-500.0 / dx)
        if dy > 1e-9:
            candidatos.append(1600.0 / dy)
        elif dy < -1e-9:
            candidatos.append(-1400.0 / dy)
        puntos.append([float(angulo), min(c for c in candidatos if c > 0.0)])
    for angulo in range(163, 192):
        puntos.append([float(angulo) + 0.01, 92.0])
    puntos.sort(key=lambda punto: punto[0])
    return puntos


def imagen_con_pilar_rojo():
    imagen = np.zeros((240, 320, 3), dtype=np.uint8)
    imagen[165:, :] = (235, 235, 235)
    cv2.rectangle(imagen, (160, 70), (190, 164), (0, 0, 255), -1)
    return imagen


def crear_captura(carpeta):
    frames = carpeta / "frames"
    frames.mkdir()
    imagen = imagen_con_pilar_rojo()
    assert cv2.imwrite(str(frames / "0.900.jpg"), imagen)
    assert cv2.imwrite(str(frames / "1.100.jpg"), imagen)

    with (carpeta / "imu.csv").open("w", newline="", encoding="utf-8") as archivo:
        writer = csv.writer(archivo)
        writer.writerow(("t", "valor_crudo_imu"))
        writer.writerows(((0.0, 10.0), (0.95, 12.5), (1.15, 13.0)))

    scan = scan_rectangular()
    with (carpeta / "lidar.jsonl").open("w", encoding="utf-8") as archivo:
        for timestamp in (1.0, 1.2, 1.4):
            archivo.write(json.dumps({"t": timestamp, "scan": scan}) + "\n")

    (carpeta / "meta.json").write_text(
        json.dumps(
            {
                "duracion_pedida_s": 2.0,
                "duracion_real_s": 1.4,
                "barridos_lidar": 3,
                "lecturas_imu": 3,
                "frames_camara": 2,
            }
        ),
        encoding="utf-8",
    )


class ReplayCapturaTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(cargar_configuracion())
        self.config["hardware"]["occlusion_validation_scans"] = 3

    def test_cadena_completa_sincroniza_causalmente_y_exporta_csv(self):
        with tempfile.TemporaryDirectory() as temporal:
            captura = Path(temporal) / "captura_20260804_115704"
            captura.mkdir()
            crear_captura(captura)
            salida = Path(temporal) / "replay.csv"

            resumen = ejecutar_replay(
                captura,
                self.config,
                sentido="LEFT",
                salida_csv=salida,
            )
            with salida.open("r", newline="", encoding="utf-8") as archivo:
                filas = list(csv.DictReader(archivo))

        self.assertEqual(resumen["lidar_barridos_procesados"], 3)
        self.assertEqual(resumen["frames_decodificados"], 2)
        self.assertTrue(resumen["meta_consistente"])
        self.assertTrue(resumen["oclusion_validada"])
        self.assertEqual(resumen["comandos_hardware_enviados"], 0)
        self.assertGreater(resumen["propuestas_movimiento"], 0)
        self.assertEqual(len(filas), 3)
        # A t=1.0 el frame 1.100 aun no ocurrio, aunque sea igual de cercano.
        self.assertEqual(filas[0]["vision_t"], "0.900000")
        self.assertAlmostEqual(float(filas[0]["heading"]), 2.5, places=3)
        self.assertEqual(filas[0]["estado"], "CRUISE")

    def test_auto_sin_color_no_inventa_el_sentido(self):
        with tempfile.TemporaryDirectory() as temporal:
            captura = Path(temporal) / "captura_auto"
            captura.mkdir()
            crear_captura(captura)

            resumen = ejecutar_replay(
                captura,
                self.config,
                sentido="AUTO",
                max_barridos=1,
            )

        self.assertEqual(resumen["estado_final"], "WAIT_DIRECTION")
        self.assertEqual(resumen["propuestas_movimiento"], 0)
        self.assertEqual(resumen["comandos_hardware_enviados"], 0)

    def test_rechaza_carpeta_incompleta_con_error_descriptivo(self):
        with tempfile.TemporaryDirectory() as temporal:
            captura = Path(temporal)
            (captura / "meta.json").write_text("{}", encoding="utf-8")

            with self.assertRaisesRegex(
                ErrorFormatoCaptura, "imu.csv.*lidar.jsonl.*frames"
            ):
                ejecutar_replay(captura, self.config, sentido="LEFT")


if __name__ == "__main__":
    unittest.main()
