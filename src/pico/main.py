import sys
import select
from machine import Pin, I2C, PWM
import time

from protocolo_seguro import parsear_consigna, watchdog_vencido

# El ultrasonido es una ayuda, no un requisito de seguridad: si el modulo no
# se copio a la Pico el robot tiene que seguir corriendo la ronda. Se anuncia
# la ausencia por telemetria (US:-1), que es la senal que la Pi ya sabe leer.
try:
    from ultrasonido import SIN_MEDIDA, FiltroUltrasonido
except ImportError:
    SIN_MEDIDA = -1
    FiltroUltrasonido = None

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

# --- ULTRASONIDO TRASERO (HC-SR04 / US-100) ---
# Trigger en GP14, Echo en GP15. Van libres: GP12 es el servo, GP16-GP19 los
# dos buses I2C, GP22 el PWM del motor y GP26-GP28 el TB6612FNG.
#
# AVISO DE CABLEADO: el Echo del HC-SR04 sale a 5 V y los GPIO de la Pico NO
# toleran 5 V. Hay que bajarlo con un divisor (1 k en serie desde Echo y 2 k
# a GND da 3,3 V) o usar un US-100 alimentado a 3,3 V. Sin eso se dana la
# entrada de la Pico.
#
# POR QUE NO SE USA machine.time_pulse_us: esa llamada bloquea hasta que
# vuelve el eco, hasta 30 ms con el timeout tipico. Este bucle corre cada
# 5 ms y es el que sostiene el servo, el motor y el watchdog de comandos:
# pararlo 30 ms de cada 60 seria peor que no tener sensor. En su lugar se
# lanza el pulso y se cronometra el flanco del eco por interrupcion, de modo
# que el control sigue corriendo mientras el sonido viaja.
US_PIN_TRIGGER = 14
US_PIN_ECHO = 15
US_PERIODO_MS = 60      # el HC-SR04 pide >=60 ms entre disparos (ecos fantasma)
US_ESPERA_MAX_MS = 40   # sin eco en este plazo se da el disparo por perdido
US_MINIMA_MM = 20       # por debajo de su zona muerta la lectura no significa nada
US_MAXIMA_MM = 4000


class SensorUltrasonido:
    """Disparo periodico y medida del eco por interrupcion, sin bloquear."""

    def __init__(self, pin_trigger, pin_echo, filtro):
        self._trigger = Pin(pin_trigger, Pin.OUT, value=0)
        self._echo = Pin(pin_echo, Pin.IN)
        self._filtro = filtro
        self._ancho_us = 0
        self._t_subida = 0
        self._midiendo = False
        self._listo = False
        self._esperando = False
        self._t_disparo = time.ticks_ms()
        self.distancia_mm = SIN_MEDIDA
        self._echo.irq(
            handler=self._al_flanco,
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
        )

    def _al_flanco(self, pin):
        # Rutina de interrupcion: solo enteros y booleanos sobre atributos que
        # ya existen. Nada que reserve memoria (ni floats, ni listas, ni
        # formateo), porque en MicroPython eso puede fallar dentro de una ISR.
        if pin.value():
            self._t_subida = time.ticks_us()
            self._midiendo = True
        elif self._midiendo:
            self._ancho_us = time.ticks_diff(time.ticks_us(), self._t_subida)
            self._midiendo = False
            self._listo = True

    def _disparar(self, ahora_ms):
        self._t_disparo = ahora_ms
        self._esperando = True
        self._midiendo = False
        self._trigger.value(0)
        time.sleep_us(2)
        self._trigger.value(1)
        time.sleep_us(10)       # el unico bloqueo: 10 us, no 30 ms
        self._trigger.value(0)

    def actualizar(self, ahora_ms):
        """Se llama en cada vuelta del bucle; devuelve la distancia vigente."""

        if self._listo:
            self._listo = False
            self._esperando = False
            self.distancia_mm = self._filtro.actualizar(self._ancho_us)
        elif self._esperando and time.ticks_diff(
            ahora_ms, self._t_disparo
        ) > US_ESPERA_MAX_MS:
            # Sin eco: pared oblicua, fuera de alcance o sensor desconectado.
            # Cuenta como fallo para que la medida caduque en vez de quedarse
            # congelada en el ultimo valor bueno.
            self._esperando = False
            self.distancia_mm = self._filtro.actualizar(0)

        if not self._esperando and time.ticks_diff(
            ahora_ms, self._t_disparo
        ) >= US_PERIODO_MS:
            self._disparar(ahora_ms)
        return self.distancia_mm

    def apagar(self):
        """Desarma la interrupcion y deja el trigger en reposo."""

        self._echo.irq(handler=None)
        self._trigger.value(0)
        self._esperando = False
        self._midiendo = False


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

# El ultrasonido se monta aparte: si no esta cableado o falta el modulo, el
# resto del firmware arranca igual y la trama sale con US:-1.
sensor_ultrasonido = None
if FiltroUltrasonido is not None:
    try:
        sensor_ultrasonido = SensorUltrasonido(
            US_PIN_TRIGGER,
            US_PIN_ECHO,
            FiltroUltrasonido(
                ventana=3, minima_mm=US_MINIMA_MM, maxima_mm=US_MAXIMA_MM
            ),
        )
    except Exception:
        sensor_ultrasonido = None

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
distancia_ultrasonido = SIN_MEDIDA

# Constante de Amortiguacion: Evita que el coche devane o curve de golpe
KD_ESTABILIDAD = 0.12

# Interruptor de la amortiguacion, controlado por un tercer campo opcional
# en la consigna ("velocidad,angulo" sigue funcionando igual y deja esto en
# 1.0). Actualmente ningun script de carrera lo usa: solo tiene sentido al
# medir el radio de giro real -- en un giro sostenido la velocidad angular
# es constante y no nula, asi que el termino KD desvia el servo varios
# grados del angulo comandado, y con la amortiguacion activa el radio de
# giro que se mida NO corresponde al comando que se mando.
kd_activo = 1.0

# Si la Pi deja de enviar consignas validas (cable USB desconectado, proceso
# caido o trama corrupta), la Pico frena y centra por si sola. El timeout es
# holgado frente al ciclo LiDAR normal y no depende del watchdog de Linux.
WATCHDOG_COMANDO_MS = 500
ultimo_comando_valido = None
watchdog_activo = True

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

        # 2b. Ultrasonido trasero. No bloquea: dispara como mucho una vez cada
        # US_PERIODO_MS y recoge lo que la interrupcion haya dejado listo.
        if sensor_ultrasonido is not None:
            try:
                distancia_ultrasonido = sensor_ultrasonido.actualizar(tiempo_actual)
            except:
                distancia_ultrasonido = SIN_MEDIDA

        # 3. Lectura de comandos desde la Pi 3B
        if poller.poll(0):
            linea = sys.stdin.readline().strip()
            if linea:
                consigna = parsear_consigna(linea)
                if consigna is not None:
                    velocidad_comandada, angulo_objetivo, kd_activo = consigna
                    ultimo_comando_valido = tiempo_actual
                    watchdog_activo = False

        if watchdog_vencido(
            tiempo_actual,
            ultimo_comando_valido,
            WATCHDOG_COMANDO_MS,
            time.ticks_diff,
        ):
            velocidad_comandada = 0
            angulo_objetivo = 0.0
            kd_activo = 1.0
            watchdog_activo = True

        # 4. Angulo objetivo (de la Pi) sobre el centro, con amortiguacion por gyro
        angulo_servo = CENTRO + angulo_objetivo - (velocidad_z * KD_ESTABILIDAD * kd_activo)
        angulo_servo = max(LIMITE_DER, min(LIMITE_IZQ, angulo_servo))
        mover_servo(angulo_servo)

        # 5. Ajustar velocidad de motores
        if velocidad_comandada == 0:
            controlar_motor(0)
        else:
            controlar_motor(velocidad_comandada)

        # 6. Telemetria a la Pi 3B: angulo, color de piso y ultrasonido.
        # El campo US va antes de WD para no alterar el orden que ya leia el
        # firmware anterior; los parsers de la Pi recorren campos por nombre,
        # asi que anadirlo no rompe a quien no lo espera.
        if time.ticks_diff(tiempo_actual, ultimo_envio_telemetria) > 50:
            estado_watchdog = "STOP" if watchdog_activo else "OK"
            sys.stdout.write(
                f"IMU:{angulo_acumulado:.2f},COLOR:{color_detectado},"
                f"US:{distancia_ultrasonido},WD:{estado_watchdog}\n"
            )
            ultimo_envio_telemetria = tiempo_actual

        time.sleep(0.005)

    except KeyboardInterrupt:
        controlar_motor(0)
        stby.value(0)
        mover_servo(CENTRO)
        # Desarmar el eco: si no, la interrupcion sigue viva despues de que
        # el bucle termine y dispara sobre un objeto que ya nadie consulta.
        if sensor_ultrasonido is not None:
            try:
                sensor_ultrasonido.apagar()
            except:
                pass
        break
