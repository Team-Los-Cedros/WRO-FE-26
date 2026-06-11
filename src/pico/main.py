# src/pico2/main.py
import machine
import sys
import uselect

# Configuración de pines de hardware (Ajusta los números de pin según tu chasis)
pin_servo = machine.Pin(15)
pin_motor_pwm = machine.Pin(16)
pin_motor_dir = machine.Pin(17)

# Configuración de PWM (Frecuencias estándar para servos y motores)
servo = machine.PWM(pin_servo)
servo.freq(50)

motor = machine.PWM(pin_motor_pwm)
motor.freq(1000)

def set_servo_angle(angle):
    # Traduce ángulos (50 a 130) a ciclo de trabajo PWM de MicroPython (0-65535)
    # Calibración estándar: 5% a 10% del ciclo de trabajo
    pulse = int(4000 + (angle - 50) * (4000 / 80))
    servo.duty_u16(pulse)

def set_motor(speed):
    # Control de dirección y velocidad del motor principal
    if speed >= 0:
        pin_motor_dir.value(1)
        value = int((speed / 100) * 65535)
    else:
        pin_motor_dir.value(0)
        value = int((abs(speed) / 100) * 65535)
    motor.duty_u16(value)

# Configuración del búfer de lectura serial nativo en MicroPython
spoll = uselect.poll()
spoll.register(sys.stdin, uselect.POLLIN)

print("Pico 2 a la espera de comandos de la Pi 5...")

buffer = ""
while True:
    # Verifica si hay datos frescos en el puerto serial sin bloquear el microcontrolador
    if spoll.poll(10): 
        caracter = sys.stdin.read(1)
        if caracter == '\n':
            # Al llegar el salto de línea, procesamos la trama "V,A"
            try:
                partes = buffer.strip().split(',')
                if len(partes) == 2:
                    vel = int(partes[0])
                    ang = float(partes[1])
                    
                    # Ejecución física en los actuadores
                    set_motor(vel)
                    set_servo_angle(ang)
            except Exception:
                pass # Ignorar tramas corruptas por ruido eléctrico
            buffer = ""
        else:
            buffer += caracter
