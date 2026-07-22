# Driver del RPLIDAR C1: conexion serial, protocolo binario y deteccion
# de barrido completo (wrap-around del angulo). No interpreta geometria
# -- entrega el barrido crudo (lista de (angulo, distancia_mm)) a quien
# lo pida; la interpretacion (paredes, clustering) vive en lidar_geometria.py.
import time

import serial

PUERTO_LIDAR   = '/dev/ttyUSB0'
BAUDRATE_LIDAR = 460800

START_MOTOR_CMD = b'\xa5\xf0\x02\x94\x02\xc1\x02'
START_SCAN_CMD  = b'\xa5\x20'
STOP_CMD        = b'\xa5\x25'


class LidarDriver:
    def __init__(self, puerto=PUERTO_LIDAR, baudrate=BAUDRATE_LIDAR):
        self._puerto   = puerto
        self._baudrate = baudrate
        self._ser      = None

    def hilo_lectura(self, obtener_corriendo, al_barrido):
        # al_barrido(scan) se llama una vez por barrido completo, con
        # scan = lista de (angulo_deg, distancia_mm)
        try:
            self._ser = serial.Serial(self._puerto, baudrate=self._baudrate, timeout=1)
            time.sleep(0.5)
            self._ser.write(START_MOTOR_CMD)
            time.sleep(1.5)
            self._ser.reset_input_buffer()
            self._ser.write(START_SCAN_CMD)
            time.sleep(0.5)
            if self._ser.in_waiting >= 7:          # descartar cabecera de respuesta
                self._ser.read(7)
            print("[+] Telemetria LiDAR activa.")

            angulo_previo = 0.0
            buffer_barrido = []

            while obtener_corriendo():
                b0 = self._ser.read(1)
                if not b0:
                    continue
                byte0 = b0[0]
                # En un paquete valido el bit de start y su inverso difieren
                if (byte0 & 0x01) == ((byte0 >> 1) & 0x01):
                    continue

                resto = self._ser.read(4)
                if len(resto) < 4:
                    continue
                byte1, byte2, byte3, byte4 = resto

                if (byte1 & 0x01) != 1:            # check bit del campo angulo
                    continue

                angle       = ((byte2 << 7) | (byte1 >> 1)) / 64.0
                distance_mm = ((byte4 << 8) | byte3) / 4.0

                if not (0 < distance_mm < 6000):
                    continue

                # Wrap-around del angulo = barrido completo listo
                if angle < angulo_previo and (angulo_previo - angle) > 300.0:
                    if buffer_barrido:
                        al_barrido(buffer_barrido)
                    buffer_barrido = []
                angulo_previo = angle
                buffer_barrido.append((angle, distance_mm))

        except Exception as e:
            if obtener_corriendo():
                print(f"[-] Falla en hilo LiDAR: {e}")

    def cerrar(self):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(STOP_CMD)
                self._ser.close()
            except Exception:
                pass
