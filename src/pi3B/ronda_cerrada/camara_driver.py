# Adquisicion de frames de la Pi Camera Module 3 (picamera2). No hace
# ningun procesamiento de color -- eso es responsabilidad de vision.py,
# que recibe cada frame por callback.
import time

from picamera2 import Picamera2

ANCHO_FRAME = 320
ALTO_FRAME  = 240


class CamaraDriver:
    def __init__(self):
        self._picam2 = None

    def hilo_captura(self, obtener_corriendo, al_frame):
        # al_frame(frame) se llama una vez por cada captura (~30 fps).
        # frame es un array BGR (ver nota en vision.py sobre el formato
        # RGB888 de picamera2).
        try:
            self._picam2 = Picamera2()
            config = self._picam2.create_video_configuration(
                main={"size": (ANCHO_FRAME, ALTO_FRAME), "format": "RGB888"},
                controls={"ScalerCrop": (0, 0, 4608, 2592)}
            )
            self._picam2.configure(config)
            self._picam2.start()
            time.sleep(1.0)
            print("[+] Camara Pi Cam Module 3 inicializada (FOV Full-Sensor).")
        except Exception as e:
            print(f"[-] Error inicializando camara: {e}")
            return

        while obtener_corriendo():
            try:
                frame = self._picam2.capture_array()
                al_frame(frame)
            except Exception as e:
                print(f"[-] Falla en hilo camara: {e}")
                time.sleep(0.1)
            time.sleep(0.03)  # ~30 fps
