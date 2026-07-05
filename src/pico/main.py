import sys
import select
from machine import Pin, I2C, PWM
import time

# Configurar el Poller para lectura serial asíncrona
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

# --- CONFIGURACIÓN DE HARDWARE ---
i2c = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000)

servo = PWM(Pin(12))
servo.freq(50)

# Controlador de Motores TB6612FNG
stby = Pin(28, Pin.OUT)
bin2 = Pin(26, Pin.OUT)  
bin1 = Pin(27, Pin.OUT)  
pwmb = PWM(Pin(22))
pwmb.freq(2000)

stby.value(1)

def mover_servo(angulo):
    angulo = max(0, min(180, angulo))
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
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00') # Despertar sensor
        # Configura el giroscopio a +-2000 °/s para evitar saturación
        self.i2c.writeto_mem(self.addr, 0x1B, b'\x18')
        
    def get_gyro_z(self):
        data = self.i2c.readfrom_mem(self.addr, 0x47, 2)
        val = (data[0] << 8) | data[1]
        if val >= 32768: val -= 65536
        return val / 16.4 # Factor de escala para +-2000 °/s

try:
    sensor = MPU6050(i2c)
    mover_servo(90)
    controlar_motor(0)
except Exception as e:
    pass

# Calibración del giroscopio
giro_z_offset = 0.0
for _ in range(100):
    try:
        giro_z_offset += sensor.get_gyro_z()
    except: pass
    time.sleep(0.01)
giro_z_offset /= 100.0

angulo_acumulado = 0.0
angulo_objetivo = 0.0
velocidad_comandada = 0

# Límites Mecánicos de Seguridad (Protección de chasis)
LIMITE_DER = 75  
LIMITE_IZQ = 105  

# Constante de Amortiguación: Evita que el coche devane o curve de golpe
KD_ESTABILIDAD = 0.12  

ultima_lectura = time.ticks_ms()
ultimo_envio_telemetria = time.ticks_ms()

# --- BUCLE DE CONTROL EN TIEMPO REAL ---
while True:
    try:
        tiempo_actual = time.ticks_ms()
        dt = time.ticks_diff(tiempo_actual, ultima_lectura) / 1000.0
        ultima_lectura = tiempo_actual
        
        try:
            velocidad_z = sensor.get_gyro_z() - giro_z_offset
        except:
            velocidad_z = 0.0
            
        if abs(velocidad_z) > 0.15:
            angulo_acumulado += velocidad_z * dt

        # Lectura de comandos desde la Pi 3B
        if poller.poll(0):
            linea = sys.stdin.readline().strip()
            if linea:
                try:
                    partes = linea.split(',')
                    if len(partes) == 2:
                        velocidad_comandada = int(partes[0])
                        angulo_objetivo = float(partes[1])
                except:
                    pass

        # Lógica de dirección (LiDAR Proporcional + Amortiguador Gyro)
        # El comando de la Pi actúa directamente sobre el centro (90°)
        # Restamos la velocidad angular multiplicada por KD para absorber inercias bruscas
        angulo_servo = 90 + angulo_objetivo - (velocidad_z * KD_ESTABILIDAD)
        
        # Limitación física estricta
        angulo_servo = max(LIMITE_DER, min(LIMITE_IZQ, angulo_servo))
        mover_servo(angulo_servo)
        
        # Ajustar velocidad de motores
        if velocidad_comandada == 0:
            controlar_motor(0)
        else:
            controlar_motor(velocidad_comandada)

        # Enviar telemetría de vuelta para conteo de vueltas en la Pi 3B
        if time.ticks_diff(tiempo_actual, ultimo_envio_telemetria) > 50:
            sys.stdout.write(f"IMU:{angulo_acumulado:.2f}\n")
            ultimo_envio_telemetria = tiempo_actual

        time.sleep(0.005)
        
    except KeyboardInterrupt:
        controlar_motor(0)
        stby.value(0)
        mover_servo(90)
        break