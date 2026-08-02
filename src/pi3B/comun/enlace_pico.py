# Comunicacion serial con la Pico 2.
# Manda consignas "velocidad,angulo\n" (velocidad en % PWM con signo,
# angulo en grados sobre el centro del servo, positivo = izquierda) y
# lee la telemetria "IMU:<grados>\n" en un hilo aparte.
import time
import threading

import serial

PUERTO_PICO   = '/dev/ttyACM0'
BAUDRATE_PICO = 115200

# Si la Pico no reporta IMU en este tiempo damos la telemetria por caida
TIMEOUT_TELEMETRIA = 0.5


class EnlacePico:
    def __init__(self, puerto=PUERTO_PICO, baudrate=BAUDRATE_PICO):
        self._ser  = serial.Serial(puerto, baudrate=baudrate, timeout=0.05)
        self._lock = threading.Lock()

        self._yaw_crudo    = 0.0
        self._cero_yaw     = None
        self._t_ultima_imu = 0.0

        self._corriendo = True
        self._hilo = threading.Thread(target=self._hilo_lectura, daemon=True)
        self._hilo.start()

    def _hilo_lectura(self):
        while self._corriendo:
            try:
                if self._ser.in_waiting > 0:
                    linea = self._ser.readline().decode('utf-8', errors='ignore').strip()
                    if linea.startswith("IMU:"):
                        valor = float(linea.split(":")[1])
                        with self._lock:
                            self._yaw_crudo    = valor
                            self._t_ultima_imu = time.time()
                            if self._cero_yaw is None:
                                self._cero_yaw = valor
            except (ValueError, IndexError):
                pass
            except serial.SerialException:
                time.sleep(0.2)
            time.sleep(0.005)

    def fijar_cero(self):
        # Se llama al presionar el boton: el yaw de ese momento pasa a ser 0
        with self._lock:
            self._cero_yaw = self._yaw_crudo

    def heading(self):
        # Yaw acumulado en grados relativo al arranque de la carrera
        with self._lock:
            if self._cero_yaw is None:
                return 0.0
            return self._yaw_crudo - self._cero_yaw

    def heading_valido(self):
        with self._lock:
            return (time.time() - self._t_ultima_imu) < TIMEOUT_TELEMETRIA

    def enviar(self, velocidad, angulo):
        try:
            self._ser.write(f"{int(velocidad)},{angulo:.2f}\n".encode())
        except serial.SerialException:
            pass

    def detener(self):
        # Mandamos el freno varias veces por si se pierde alguna linea
        for _ in range(5):
            self.enviar(0, 0.0)
            time.sleep(0.01)

    def cerrar(self):
        self._corriendo = False
        try:
            self.detener()
            self._ser.close()
        except Exception:
            pass
