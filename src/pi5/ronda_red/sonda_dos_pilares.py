"""Observacion estatica de las cajas: no abre el enlace de traccion."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
from picamera2 import Picamera2
from hailo_platform import (VDevice, HEF, ConfigureParams, HailoStreamInterface,
                            InputVStreamParams, OutputVStreamParams, InferVStreams,
                            FormatType)
from dos_pilares import decodificar
from vision_red import _preparar, RUTA_HEF, UMBRAL
from camara_driver import ANCHO_FRAME, ALTO_FRAME, MODO_SENSOR


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--salida', required=True)
    parser.add_argument('--cuadros', type=int, default=12)
    args = parser.parse_args()
    carpeta = Path(args.salida)
    carpeta.mkdir(parents=True, exist_ok=True)
    hef = HEF(RUTA_HEF)
    entrada = hef.get_input_vstream_infos()[0]
    salida = hef.get_output_vstream_infos()[0]
    registros = []
    with Picamera2() as camara, VDevice() as dispositivo:
        camara.configure(camara.create_video_configuration(
            main={'size': (ANCHO_FRAME, ALTO_FRAME), 'format': 'RGB888'},
            raw={'size': MODO_SENSOR}))
        camara.start()
        time.sleep(2)
        params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        red = dispositivo.configure(hef, params)[0]
        ivp = InputVStreamParams.make(red, format_type=FormatType.UINT8)
        ovp = OutputVStreamParams.make(red, format_type=FormatType.FLOAT32)
        with red.activate(red.create_params()), InferVStreams(red, ivp, ovp) as tuberia:
            for i in range(args.cuadros):
                frame = camara.capture_array()[:, :, :3]
                t = time.time()
                entrada_rgb, escala, dx, dy = _preparar(frame)
                inferida = tuberia.infer({entrada.name: np.expand_dims(entrada_rgb, 0)})
                cajas = decodificar(inferida[salida.name][0], escala, dx, dy,
                                    ANCHO_FRAME, ALTO_FRAME, UMBRAL)
                registros.append({'t_unix': t, 'cajas': [c.__dict__ for c in cajas]})
                cv2.imwrite(str(carpeta / ('%03d_crudo.jpg' % i)), frame)
                vista = frame.copy()
                for c in cajas:
                    tinta = (0, 255, 255) if c.color == 'ROJO' else (255, 255, 0)
                    cv2.rectangle(vista, (round(c.x0), round(c.y0)), (round(c.x1), round(c.y1)), tinta, 2)
                    cv2.putText(vista, '%s %.2f' % (c.color, c.score),
                                (round(c.x0), max(15, round(c.y0) - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, tinta, 1)
                cv2.imwrite(str(carpeta / ('%03d_cajas.jpg' % i)), vista)
                print(i, [(c.color, round(c.score, 3), round(c.cx), round(c.alto)) for c in cajas], flush=True)
                time.sleep(.125)
        camara.stop()
    (carpeta / 'cajas.json').write_text(json.dumps(registros, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
