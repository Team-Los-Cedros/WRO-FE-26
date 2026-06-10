# src/pico/imu.py
from machine import I2C, Pin
import time
import math

class ControlIMU:
    def __init__(self):
        # Inicializar bus I2C0 usando los pines reales GP16 y GP17
        self.i2c = I2C(0, sda=Pin(16, Pin.PULL_UP), scl=Pin(17, Pin.PULL_UP), freq=400000)
        self.address = 0x68 # Dirección I2C estándar del MPU6050
        
        # Despertar el MPU6050 (Modo sleep a 0)
        self.i2c.writeto_mem(self.address, 0x6B, b'\x00')
        
        # Configurar el Giroscopio a Full Scale Range de +/- 250 grados/seg
        self.i2c.writeto_mem(self.address, 0x1B, b'\x00')
        
        # Variables para la integración del ángulo Yaw (Eje Z)
        self.yaw = 0.0
        self.giro_z_offset = 0.0
        self.last_time = time.ticks_us()
        
        # Calibrar al arrancar
        self.calibrar_giroscopio()

    def calibrar_giroscopio(self):
        """Lee el sensor en reposo para calcular el ruido de fondo (offset)"""
        print("Calibrando IMU")
        suma = 0
        muestras = 200
        for _ in range(muestras):
            suma += self._leer_giro_z_crudo()
            time.sleep_ms(5)
        self.giro_z_offset = suma / muestras
        print("Calibración completada")
        self.last_time = time.ticks_us()

    def _leer_giro_z_crudo(self):
        """Lee los registros de alta y baja velocidad del eje Z del giroscopio"""
        data = self.i2c.readfrom_mem(self.address, 0x47, 2)
        valor = (data[0] << 8) | data[1]
        if valor >= 32768:
            valor -= 65536
        return valor

    def actualizar_yaw(self):
        """Calcula el ángulo Yaw actual basándose en el tiempo transcurrido (dt)"""
        current_time = time.ticks_us()
        dt = time.ticks_diff(current_time, self.last_time) / 1000000.0
        self.last_time = current_time
        
        giro_z_filtrado = self._leer_giro_z_crudo() - self.giro_z_offset
        velocidad_angular_z = giro_z_filtrado / 131.0
        
        # Ignorar micro-vibraciones menores a 0.5 grados por segundo
        if abs(velocidad_angular_z) > 0.5:
            self.yaw += velocidad_angular_z * dt
            
        return self.yaw

    def reset_yaw(self):
        self.yaw = 0.0

# =========================================================================
# BUCLE DE PRUEBA EN TIEMPO REAL (Solo para validar en Thonny)
# =========================================================================
if __name__ == '__main__':
    # Instanciamos la clase de control de la IMU
    imu_sistema = ControlIMU()
    
    print("Iniciando lecturas de telemetría. Gira el coche en la mesa...")
    while True:
        angulo_actual = imu_sistema.actualizar_yaw()
        # Imprime el valor con dos decimales
        print("Ángulo Yaw (Eje Z): {:.2f}°".format(angulo_actual))
        time.sleep_ms(50)
