import sys
import select
from machine import Pin, I2C, PWM
import time

# Configurar el Poller para lectura serial asincrona desde la Pi 3B
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

# --- CONFIGURACION DE BUSES I2C INDEPENDIENTES ---
# Bus 0: IMU MPU6050 en GP16 (SDA) y GP17 (SCL)
i2c_imu = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000)

# Bus 1: Sensor de color TCS3472 en GP18 (SDA) y GP19 (SCL). Lee la linea
# del piso para el sentido de carrera (AZUL/NARANJA, ver TCS3472 abajo).
i2c_tcs = I2C(1, sda=Pin(18), scl=Pin(19), freq=100000)

# --- HARDWARE DE CONTROL (SERVO Y MOTOR) ---
servo = PWM(Pin(12))
servo.freq(50)

# Controlador de Motores TB6612FNG
stby = Pin(28, Pin.OUT)
bin2 = Pin(26, Pin.OUT)
bin1 = Pin(27, Pin.OUT)
pwmb = PWM(Pin(22))
pwmb.freq(2000)

stby.value(1)

# Limites de giro del servo calibrados
CENTRO = 90
LIMITE_DER = 70    # Maximo giro a la derecha
LIMITE_IZQ = 115   # Maximo giro a la izquierda

def mover_servo(angulo):
    # Limita al rango calibrado en vez de 0-180 (protege el servo)
    angulo = max(LIMITE_DER, min(LIMITE_IZQ, angulo))
    duty = int(1638 + (angulo / 180.0) * (8192 - 1638))
    servo.duty_u16(duty)

def controlar_motor(velocidad_porcentaje):
    if velocidad_porcentaje > 0:
        bin1.value(1)
        bin2.value(0)
        vel = max(0, min(100, velocidad_porcentaje))
    elif velocidad_porcentaje < 0:
        bin1.value(0)
        bin2.value(1)
        vel = max(0, min(100, abs(velocidad_porcentaje)))
    else:
        bin1.value(1)
        bin2.value(1)
        vel = 0

    duty_u16 = int((vel / 100.0) * 65535)
    pwmb.duty_u16(duty_u16)

class MPU6050:
    def __init__(self, i2c_bus, addr=0x68):
        self.i2c = i2c_bus
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00') # Activar sensor
        # Configura el giroscopio a +-2000 grados/s para evitar saturacion
        self.i2c.writeto_mem(self.addr, 0x1B, b'\x18')

    def get_gyro_z(self):
        data = self.i2c.readfrom_mem(self.addr, 0x47, 2)
        val = (data[0] << 8) | data[1]
        if val >= 32768: val -= 65536
        return val / 16.4 # Factor de escala para +-2000 grados/s

# --- SENSOR DE COLOR DE PISO (TCS3472), CON CALIBRACION DINAMICA EN HSV ---
# Lee la linea de color del piso una vez por vuelta al arranque para fijar
# el sentido de carrera (AZUL=Antihorario, NARANJA=Horario segun la
# convencion del equipo). No hay libreria oficial para MicroPython: se
# escribe/lee directo sobre los registros del chip por I2C.
class TCS3472:
    def __init__(self, i2c_bus, addr=0x29):
        self.i2c = i2c_bus
        self.addr = addr
        self.escribir(0x00, 0x01)       # PON (Power On)
        time.sleep_ms(10)
        self.escribir(0x00, 0x03)       # PON + AEN (Activar lectura RGBC)
        self.escribir(0x01, 0xF6)       # Integration time 24ms
        self.escribir(0x0F, 0x02)       # Ganancia 16x

        self.historial_h = []
        self.historial_s = []
        self.tamano_ventana = 4
        self.saturacion_base_pista = 0.20

    def escribir(self, reg, valor):
        self.i2c.writeto(self.addr, bytearray([0x80 | reg, valor]))

    def leer_16(self, reg):
        self.i2c.writeto(self.addr, bytearray([0x80 | 0x20 | reg]), False)
        datos = self.i2c.readfrom(self.addr, 2)
        return datos[0] | (datos[1] << 8)

    def calibrar_suelo_inicial(self):
        # Se llama una vez al arrancar: promedia la saturacion del piso
        # blanco bajo la iluminacion real de la pista para fijar el
        # umbral que distingue "pista" de "linea de color", en vez de un
        # valor fijo que se desajusta con cada cambio de luz.
        muestras_s = []
        for _ in range(25):
            try:
                c = self.leer_16(0x14)
                r = self.leer_16(0x16)
                g = self.leer_16(0x18)
                b = self.leer_16(0x1A)
                if c > 0:
                    max_c = max(r, g, b)
                    min_c = min(r, g, b)
                    df = max_c - min_c
                    s = 0.0 if max_c == 0 else df / max_c
                    muestras_s.append(s)
            except:
                pass
            time.sleep_ms(10)

        if muestras_s:
            # Asigna umbral de saturacion de pista con margen de tolerancia
            self.saturacion_base_pista = (sum(muestras_s) / len(muestras_s)) + 0.12

    def obtener_color(self):
        try:
            c = self.leer_16(0x14)
            r = self.leer_16(0x16)
            g = self.leer_16(0x18)
            b = self.leer_16(0x1A)

            if c < 40:
                return "NINGUNO"

            # Conversion de RGB a espacio HSV (Hue, Saturation, Value)
            max_c = max(r, g, b)
            min_c = min(r, g, b)
            df = max_c - min_c

            s = 0.0 if max_c == 0 else df / max_c

            if df == 0:
                h = 0.0
            elif max_c == r:
                h = (60 * ((g - b) / df) + 360) % 360
            elif max_c == g:
                h = (60 * ((b - r) / df) + 120) % 360
            elif max_c == b:
                h = (60 * ((r - g) / df) + 240) % 360

            # Filtro de promedio movil para mitigar ruido y destellos
            self.historial_h.append(h)
            self.historial_s.append(s)

            if len(self.historial_h) > self.tamano_ventana:
                self.historial_h.pop(0)
                self.historial_s.pop(0)

            h_prom = sum(self.historial_h) / len(self.historial_h)
            s_prom = sum(self.historial_s) / len(self.historial_s)

            # Descarte dinamico de pista por umbral de saturacion
            if s_prom < self.saturacion_base_pista:
                return "PISTA"

            # Evaluacion del tono (Hue) con tolerancias de movimiento
            if 5 <= h_prom <= 42:
                return "NARANJA"
            elif 180 <= h_prom <= 245:
                return "AZUL"
            else:
                return "PISTA"

        except:
            return "ERROR"

try:
    sensor_imu = MPU6050(i2c_imu)
    sensor_color = TCS3472(i2c_tcs)
    mover_servo(CENTRO) # Arranca alineado al centro calibrado
    controlar_motor(0)
except Exception as e:
    pass

# Calibracion del giroscopio
giro_z_offset = 0.0
for _ in range(100):
    try:
        giro_z_offset += sensor_imu.get_gyro_z()
    except: pass
    time.sleep(0.01)
giro_z_offset /= 100.0

# Calibracion de saturacion base del suelo (piso blanco bajo la luz real)
try:
    sensor_color.calibrar_suelo_inicial()
except:
    pass

angulo_acumulado = 0.0
angulo_objetivo = 0.0
velocidad_comandada = 0
color_detectado = "PISTA"

# Constante de Amortiguacion: Evita que el coche devane o curve de golpe
KD_ESTABILIDAD = 0.12

# Interruptor de la amortiguacion, controlado por un tercer campo opcional
# en la consigna ("velocidad,angulo" sigue funcionando igual y deja esto en
# 1.0). Solo lo usa calibracion/medir_direccion.py: en un giro sostenido la
# velocidad angular es constante y no nula, asi que el termino KD desvia el
# servo varios grados del angulo comandado -- con la amortiguacion activa el
# radio de giro que se mida NO corresponde al comando que se mando.
kd_activo = 1.0

ultima_lectura = time.ticks_ms()
ultimo_envio_telemetria = time.ticks_ms()

# Bucle principal de control
while True:
    try:
        tiempo_actual = time.ticks_ms()
        dt = time.ticks_diff(tiempo_actual, ultima_lectura) / 1000.0
        ultima_lectura = tiempo_actual

        # 1. Integracion de angulo con IMU
        try:
            velocidad_z = sensor_imu.get_gyro_z() - giro_z_offset
        except:
            velocidad_z = 0.0

        if abs(velocidad_z) > 0.15:
            angulo_acumulado += velocidad_z * dt

        # 2. Lectura del sensor de color (TCS3472 con filtro HSV)
        color_detectado = sensor_color.obtener_color()

        # 3. Lectura de comandos desde la Pi 3B
        if poller.poll(0):
            linea = sys.stdin.readline().strip()
            if linea:
                try:
                    partes = linea.split(',')
                    if len(partes) >= 2:
                        velocidad_comandada = int(partes[0])
                        angulo_objetivo = float(partes[1])
                        # Una consigna de dos campos devuelve la
                        # amortiguacion a su valor normal. Sin esto un
                        # kd=0 de calibracion quedaria pegado hasta
                        # reiniciar la Pico y la siguiente ronda correria
                        # sin amortiguacion sin que nadie lo note.
                        kd_activo = 1.0
                    if len(partes) == 3:
                        kd_activo = float(partes[2])
                except:
                    pass

        # 4. Angulo objetivo (de la Pi) sobre el centro, con amortiguacion por gyro
        angulo_servo = CENTRO + angulo_objetivo - (velocidad_z * KD_ESTABILIDAD * kd_activo)
        angulo_servo = max(LIMITE_DER, min(LIMITE_IZQ, angulo_servo))
        mover_servo(angulo_servo)

        # 5. Ajustar velocidad de motores
        if velocidad_comandada == 0:
            controlar_motor(0)
        else:
            controlar_motor(velocidad_comandada)

        # 6. Telemetria a la Pi 3B: angulo acumulado + color de piso
        if time.ticks_diff(tiempo_actual, ultimo_envio_telemetria) > 50:
            sys.stdout.write(f"IMU:{angulo_acumulado:.2f},COLOR:{color_detectado}\n")
            ultimo_envio_telemetria = tiempo_actual

        time.sleep(0.005)

    except KeyboardInterrupt:
        controlar_motor(0)
        stby.value(0)
        mover_servo(CENTRO)
        break
