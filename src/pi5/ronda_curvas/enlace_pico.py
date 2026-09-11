# Comunicacion serial con la Pico 2.
# Manda consignas "velocidad,angulo\n" (velocidad en % PWM con signo,
# angulo en grados sobre el centro del servo, positivo = izquierda) y
# lee la telemetria de la Pico en un hilo aparte. Segun el firmware que
# tenga flasheada, la trama es "IMU:<grados>\n" (sin sensor de piso) o
# "IMU:<grados>,COLOR:<nombre>\n" (con TCS3472). Se aceptan las dos.
#
# Hay una tercera forma opcional, "velocidad,angulo,kd\n", que ademas
# fija la ganancia de amortiguacion por giroscopio del firmware para ese
# instante. Ningun script de carrera la usa actualmente (medir el radio
# de giro real exige kd=0, si no el angulo de rueda real no es el
# comandado). Cualquier consigna de dos campos restaura el kd por
# defecto en la Pico, asi que ningun script de carrera puede heredar un
# kd alterado.
import time
import threading

import serial

PUERTO_PICO   = '/dev/ttyACM0'
BAUDRATE_PICO = 115200

# Si la Pico no reporta IMU en este tiempo damos la telemetria por caida
TIMEOUT_TELEMETRIA = 0.5

# Rango util del HC-SR04 trasero. Fuera de el la lectura no significa
# nada: el firmware ya acota con US_MINIMA_MM/US_MAXIMA_MM, esto es el
# candado del lado de la Pi.
US_MIN_VALIDO = 20.0
US_MAX_VALIDO = 4000.0

# Del sensor al tope trasero del chasis, medido en el banco del 06-09
# (`chassis.ultrasound_rear_to_tail_mm`). Lo que le queda al robot antes
# de tocar es la lectura MENOS esto.
US_SENSOR_A_CULATA = 34.0

# Valores que el firmware puede emitir en COLOR. Cualquier otra cosa es
# una linea truncada y se descarta: ver el candado en _parsear.
COLORES_PISO = ("NARANJA", "AZUL", "PISTA", "NINGUNO", "ERROR", "SIN_SENSOR")


class EnlacePico:
    def __init__(self, puerto=PUERTO_PICO, baudrate=BAUDRATE_PICO):
        self._ser  = serial.Serial(puerto, baudrate=baudrate, timeout=0.05)
        self._lock = threading.Lock()

        self._yaw_crudo    = 0.0
        self._cero_yaw     = None
        self._t_ultima_imu = 0.0

        # Campos que la Pico ya emitia y este modulo tiraba al hacer
        # split(",")[0]. Medido el 10-09 con el LiDAR y el ultrasonido a
        # la vez, en la misma pose: el LiDAR reportaba 47mm por detras
        # (era su propia estructura, ver lidar_mascara) y el ultrasonido
        # 1085mm, que era la verdad. Sin este campo el estado RETROCESO
        # sale en su primer ciclo SIEMPRE, y el robot no puede desatascarse.
        self._us_mm        = None
        self._t_ultimo_us  = 0.0
        self._color_piso   = None
        self._watchdog_ok  = False

        self._corriendo = True
        self._hilo = threading.Thread(target=self._hilo_lectura, daemon=True)
        self._hilo.start()

    def _hilo_lectura(self):
        while self._corriendo:
            try:
                if self._ser.in_waiting > 0:
                    linea = self._ser.readline().decode('utf-8', errors='ignore').strip()
                    if linea.startswith("IMU:"):
                        self._parsear(linea)
            except (ValueError, IndexError):
                pass
            except serial.SerialException:
                time.sleep(0.2)
            time.sleep(0.005)

    def _parsear(self, linea):
        """Trama completa: "IMU:<g>,COLOR:<n>,US:<mm>,WD:<OK|STOP>".

        Se recorre POR NOMBRE, no por posicion, y cada campo se aisla en
        su propio try: una trama vieja de tres campos, o un campo
        corrupto, no puede tumbar a los demas. El firmware de tres campos
        ("IMU:<g>,COLOR:<n>") y el de dos siguen funcionando igual.

        El bug que esto sustituye era `linea.split(",")[0]`: se quedaba
        con el yaw y descartaba los otros tres campos sin que nadie lo
        supiera. US es la unica medida trasera fiable que tiene el robot
        (ver lidar_mascara), y WD dice si la Pico se paro sola.
        """
        ahora = time.time()
        campos = {}
        for trozo in linea.split(","):
            if ":" in trozo:
                clave, _, valor = trozo.partition(":")
                campos[clave.strip()] = valor.strip()

        with self._lock:
            if "IMU" in campos:
                try:
                    self._yaw_crudo    = float(campos["IMU"])
                    self._t_ultima_imu = ahora
                    if self._cero_yaw is None:
                        self._cero_yaw = self._yaw_crudo
                except ValueError:
                    pass
            if "US" in campos:
                try:
                    # El firmware manda SIN_MEDIDA (un centinela) cuando el
                    # disparo se perdio. Fuera del rango util del HC-SR04 la
                    # lectura no significa nada: mejor None que un numero.
                    us = float(campos["US"])
                    if US_MIN_VALIDO <= us <= US_MAX_VALIDO:
                        self._us_mm = us
                        self._t_ultimo_us = ahora
                    else:
                        self._us_mm = None
                except ValueError:
                    self._us_mm = None
            if "COLOR" in campos:
                # SOLO valores conocidos. `readline()` puede devolver una
                # linea cortada a mitad cuando el timeout vence, y sin
                # este candado "COLOR:PIST" se acepta como un color
                # valido. Medido el 10-09: 1 de cada 120 lecturas llegaba
                # truncada. El parser anterior no lo sufria porque
                # descartaba el campo entero.
                c = campos["COLOR"]
                if c in COLORES_PISO:
                    self._color_piso = None if c in ("SIN_SENSOR", "PISTA",
                                                     "NINGUNO", "ERROR") else c
            if "WD" in campos and campos["WD"] in ("OK", "STOP"):
                self._watchdog_ok = (campos["WD"] == "OK")

    def ultrasonido_mm(self):
        """Hueco REAL por detras del parachoques, en mm, o None.

        Devuelve None -- SIN EVIDENCIA -- si la lectura es vieja o esta
        fuera de rango, para que el llamador decida. Se descuenta la
        distancia del sensor al tope trasero (medida el 06-09), asi que
        el numero es "cuanto me falta para tocar", no "que lee el sensor".
        """
        with self._lock:
            if self._us_mm is None:
                return None
            if (time.time() - self._t_ultimo_us) > TIMEOUT_TELEMETRIA:
                return None
            return max(0.0, self._us_mm - US_SENSOR_A_CULATA)

    def color_piso(self):
        with self._lock:
            return self._color_piso

    def watchdog_ok(self):
        # False = la Pico dejo de recibir consignas y se paro sola.
        with self._lock:
            return self._watchdog_ok

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

    def enviar(self, velocidad, angulo, kd=None):
        # kd es el tercer campo opcional: FACTOR sobre la amortiguacion por
        # giroscopio de la Pico (1.0 = normal, 0.0 = desactivada). En carrera
        # va siempre en None (dos campos, comportamiento de siempre), y una
        # consigna de dos campos devuelve el factor a 1.0 en el firmware.
        # Actualmente ningun script de carrera lo usa: solo tiene sentido al
        # medir el radio de giro real, donde esa amortiguacion lo falsearia.
        linea = f"{int(velocidad)},{angulo:.2f}"
        if kd is not None:
            linea += f",{kd:.2f}"
        try:
            self._ser.write((linea + "\n").encode())
        except serial.SerialException:
            pass

    def led(self, modo):
        """Enciende o apaga el parpadeo del LED de la Pico.

        `modo` es "BLINK" (parpadear) o "OFF". El firmware acepta el
        comando desde hace tiempo (LED:BLINK / LED:OFF); no hacia falta
        reflashear nada para esto.

        Un firmware antiguo que no lo conozca simplemente ignora la linea:
        su parser descarta lo que no sabe leer, asi que esto no puede
        dejar el robot sin consignas.
        """
        try:
            self._ser.write(("LED:" + modo + "\n").encode())
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
