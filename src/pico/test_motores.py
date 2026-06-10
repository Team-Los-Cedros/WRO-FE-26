# src/pico/motores.py
from machine import Pin, PWM
import time

class ControlMovimiento:
    # Configuración de Hardware
    FREQ_SERVO = 50
    FREQ_MOTOR = 1000
    
    # Límites físicos del chasis (Evita que el servo fuerce la dirección)
    ANGULO_MAX_GIRO = 45  
    
    # Resolución PWM de la Raspberry Pi Pico (16 bits)
    MAX_DUTY_16BIT = 65535

    def __init__(self):
        # Servo de Dirección en el PIN 15 (Slice 7B) -> ¡Tu nueva configuración!
        self.servo = PWM(Pin(15))
        self.servo.freq(self.FREQ_SERVO)
        
        # Pines del Puente H (TB6612FNG - Canal B)
        self.bin1 = Pin(26, Pin.OUT)
        self.bin2 = Pin(27, Pin.OUT)
        self.stby = Pin(22, Pin.OUT)
        
        # PWM del Motor de Tracción en el PIN 28 (Slice 6A)
        self.pwm_motor = PWM(Pin(28))
        self.pwm_motor.freq(self.FREQ_MOTOR)
        
        # Inicialización segura
        self.frenar()
        self.centrar_direccion()

    def set_direccion(self, angulo):
        """Ajusta el giro de las ruedas delanteras (-45 a 45)"""
        angulo = max(-self.ANGULO_MAX_GIRO, min(self.ANGULO_MAX_GIRO, angulo))
        pulso_ms = 1.5 + (angulo / self.ANGULO_MAX_GIRO) * 0.5  
        duty = int((pulso_ms / 20.0) * self.MAX_DUTY_16BIT)
        self.servo.duty_u16(duty)

    def set_traccion(self, velocidad):
        """Controla velocidad y sentido (-100 a 100)"""
        if velocidad == 0:
            self.frenar()
            return
            
        self.stby.value(1)
        
        if velocidad > 0:
            self.bin1.value(1)
            self.bin2.value(0)
        else:
            self.bin1.value(0)
            self.bin2.value(1)
            velocidad = abs(velocidad)
            
        velocidad_clamped = min(100, velocidad)
        duty_motor = int((velocidad_clamped / 100.0) * self.MAX_DUTY_16BIT)
        self.pwm_motor.duty_u16(duty_motor)

    def frenar(self):
        """Corta la energía de los motores y activa Standby"""
        self.bin1.value(0)
        self.bin2.value(0)
        self.pwm_motor.duty_u16(0)
        self.stby.value(0)  

    def centrar_direccion(self):
        """Alinea las ruedas en línea recta"""
        self.set_direccion(0)


# Creamos el objeto del coche
coche = ControlMovimiento()

print("Prueba Secuencia")
time.sleep(2)

# 1. Probar dirección
print("Girando a la izquierda")
coche.set_direccion(-30)
time.sleep(1)

print("Girando a la derecha")
coche.set_direccion(30)
time.sleep(1)

print("Centrando volante")
coche.centrar_direccion()
time.sleep(1)

# 2. Probar aceleración progresiva
print("Avanzando al 40%")
coche.set_traccion(40)
time.sleep(2)

print("Freno de mano")
coche.frenar()
time.sleep(1)

print("Retrocediendo al 40%")
coche.set_traccion(-40)
time.sleep(2)

# Final seguro
print("Prueba completada Coche detenido.")
coche.frenar()
coche.centrar_direccion()
