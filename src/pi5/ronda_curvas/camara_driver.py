# Adquisicion de frames para el montaje NUEVO de la camara (mastil
# trasero). Mismo contrato que ronda_cerrada/camara_driver.py -- entrega
# frames BGR por callback y no procesa color -- con dos diferencias, que
# son las que obligan a tener el archivo aparte.
#
# 1. NO SE ROTA 180 GRADOS.
#
# El driver de la ronda cerrada rota porque el modulo estaba atornillado
# invertido. En el mastil quedo derecho. Medido el 2026-08-28 con el
# robot quieto en pista: el frame crudo tiene muros arriba (V=50 en las
# primeras filas) y suelo blanco abajo (V=142-150); el rotado, al reves.
# Mantener la rotacion espeja el cx (la evasion sale por el lado
# contrario al del reglamento) e invierte el filtro cy de vision.py.
#
# 2. MODO DE SENSOR 2304x1296, SALIDA 16:9.
#
# El driver viejo pedia 320x240 (4:3) y ponia ScalerCrop al sensor
# entero. No sirve de nada: al pedir 4:3 de un sensor 16:9, libcamera
# recorta los lados igualmente. Leido de la metadata del propio frame,
# el recorte que aplicaba era (768, 432, 3072, 1728) -- o sea 3072 de
# los 4608 px de ancho, un tercio del campo tirado a la basura, y de
# paso los pixeles quedaban anamorficos (3072/320 = 9.6 horizontal
# contra 1728/240 = 7.2 vertical, un estirado vertical de 1.33x).
#
# Fijando el modo raw a 2304x1296 y pidiendo una salida 16:9, el recorte
# pasa a ser (0, 0, 4608, 2592) -- el sensor completo:
#
#     HFOV  53.8 -> 74.5 grados        VFOV  31.9 -> 46.3 grados
#     pixeles anamorficos -> cuadrados (f_h = f_v = 420 px)
#
# La salida se queda pequeña a proposito. En una Pi 3B el pipeline de
# vision.py (HSV + dos morfologias) cuesta, medido:
#
#     320x240    13.5 ms/frame     384x216     5.5 ms/frame
#     640x360    13.9 ms/frame    2304x1296  165.9 ms/frame
#
# Sacar el frame a 2304x1296 dejaria la vision en 6 fps, con 166ms de
# latencia por deteccion: inservible para decidir una evasion. 640x360
# cuesta lo mismo que los 320x240 de antes y da el doble de resolucion
# lineal con el campo completo.
import time

from picamera2 import Picamera2

# 16:9, para que el recorte no se coma los lados. Cambiar esto obliga a
# revisar optica.py (focal y centro) y vision.py (area minima y filtro
# de altura): todos sus umbrales estan en pixeles de ESTE tamaño.
ANCHO_FRAME = 640
ALTO_FRAME  = 360

# Modo nativo del imx708 que cubre el sensor entero.
MODO_SENSOR = (2304, 1296)


class CamaraDriver:
    def __init__(self):
        self._picam2 = None

    def hilo_captura(self, obtener_corriendo, al_frame):
        # al_frame(frame) se llama una vez por captura, con un array BGR
        # ya utilizable: sin rotacion, ver nota de cabecera.
        try:
            self._picam2 = Picamera2()
            config = self._picam2.create_video_configuration(
                main={"size": (ANCHO_FRAME, ALTO_FRAME), "format": "RGB888"},
                raw={"size": MODO_SENSOR},
            )
            self._picam2.configure(config)
            self._picam2.start()
            time.sleep(1.0)
            print(f"[+] Camara lista: {ANCHO_FRAME}x{ALTO_FRAME} desde el modo "
                  f"{MODO_SENSOR[0]}x{MODO_SENSOR[1]} (sensor completo, sin rotacion).")
        except Exception as e:
            print(f"[-] Error inicializando camara: {e}")
            return

        while obtener_corriendo():
            try:
                al_frame(self._picam2.capture_array()[:, :, :3])
            except Exception as e:
                print(f"[-] Falla en hilo camara: {e}")
                time.sleep(0.1)
            time.sleep(0.03)

    def cerrar(self):
        if self._picam2 is not None:
            try:
                self._picam2.stop()
            except Exception:
                pass
