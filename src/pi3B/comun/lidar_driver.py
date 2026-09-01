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
            self._ser = serial.Serial(self._puerto, baudrate=self._baudrate, timeout=0.1)
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
            raw_buf = bytearray()

            while obtener_corriendo():
                en_espera = self._ser.in_waiting
                if en_espera > 0:
                    chunk = self._ser.read(max(en_espera, 500))
                else:
                    chunk = self._ser.read(100)

                if not chunk:
                    continue

                raw_buf.extend(chunk)

                # Si el buffer se atrasa por sobrecarga (> 4000 bytes ~ 2 barridos),
                # conservar solo el ultimo bloque para evitar latencia acumulada
                if len(raw_buf) > 4000:
                    raw_buf = raw_buf[-2000:]
                    buffer_barrido = []

                idx = 0
                n = len(raw_buf)
                while idx + 5 <= n:
                    b0 = raw_buf[idx]
                    # En un paquete valido el bit de start y su inverso difieren
                    if (b0 & 0x01) == ((b0 >> 1) & 0x01):
                        idx += 1
                        continue

                    b1 = raw_buf[idx + 1]
                    # check bit del campo angulo
                    if (b1 & 0x01) != 1:
                        idx += 1
                        continue

                    b2 = raw_buf[idx + 2]
                    b3 = raw_buf[idx + 3]
                    b4 = raw_buf[idx + 4]

                    angle = ((b2 << 7) | (b1 >> 1)) / 64.0
                    distance_mm = ((b4 << 8) | b3) / 4.0

                    idx += 5

                    if 0 < distance_mm < 6000:
                        # Wrap-around del angulo = barrido completo listo
                        if angle < angulo_previo and (angulo_previo - angle) > 300.0:
                            if buffer_barrido:
                                al_barrido(buffer_barrido)
                            buffer_barrido = []
                        angulo_previo = angle
                        buffer_barrido.append((angle, distance_mm))

                if idx > 0:
                    del raw_buf[:idx]

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
