# src/pi5/lidar_handler.py
import os
import time
import random

# Intentamos importar serial; si estás en laptop sin la librería, no romperá el código
try:
    import serial
except ImportError:
    serial = None

class LidarHandler:
    def __init__(self, puerto='/dev/ttyUSB0'):
        self.puerto = puerto
        self.simulado = False
        self.motor_encendido = False
        
        # QA: Detectar si estamos en la laptop o si el puerto no existe
        if serial is None or not os.path.exists(self.puerto):
            print("[LIDAR QA] ⚠️ Puerto físico no detectado. ¡ACTIVANDO MODO SIMULACIÓN EN LAPTOP!")
            self.simulado = True
            self.motor_encendido = True
        else:
            print(f"[LIDAR] Conectando por Serial al puerto {self.puerto}...")
            self.ser = serial.Serial(self.puerto, 460800, timeout=0.5)
            self.iniciar_motor()

    def iniciar_motor(self):
        if not self.simulado and not self.motor_encendido:
            self.ser.dtr = False
            self.ser.rts = True
            time.sleep(0.1)
            self.ser.write(b'\xa5\x40') # Reset
            time.sleep(0.5)
            self.ser.write(b'\xa5\x82\x05\x00\x00\x00\x00\x00\x22') # Start C1
            time.sleep(0.5)
            self.motor_encendido = True
            print("[LIDAR] ✅ Motor del C1 girando de forma nativa.")

    def obtener_distancias_zonas(self):
        # Si estamos en la laptop, generamos telemetría falsa pero lógica para pruebas
        if self.simulado:
            time.sleep(0.05) # Simular el delay de lectura del sensor (20 Hz)
            return {
                'frontal': round(random.uniform(20.0, 150.0), 1),
                'izquierda': round(random.uniform(10.0, 100.0), 1),
                'derecha': round(random.uniform(10.0, 100.0), 1)
            }
            
        # Código para el carro real (Raspberry Pi 5)
        zonas = {'frontal': 300.0, 'izquierda': 300.0, 'derecha': 300.0}
        try:
            if self.ser.in_waiting > 0:
                registro = self.ser.read(self.ser.in_waiting)
                for i in range(0, len(registro) - 4, 5):
                    distance = (registro[i+3] << 8) | registro[i+2]
                    dist_cm = distance / 10.0
                    
                    if dist_cm <= 1.0 or dist_cm > 300.0: 
                        continue
                    
                    posicion_relativa = i / len(registro)
                    if posicion_relativa < 0.2 or posicion_relativa > 0.8:
                        if dist_cm < zonas['frontal']: zonas['frontal'] = dist_cm
                    elif 0.2 <= posicion_relativa < 0.5:
                        if dist_cm < zonas['derecha']: zonas['derecha'] = dist_cm
                    elif 0.5 <= posicion_relativa <= 0.8:
                        if dist_cm < zonas['izquierda']: zonas['izquierda'] = dist_cm
        except Exception as e:
            print(f"[LIDAR ERROR]: {e}")
        return zonas

    def detener(self):
        if self.motor_encendido and not self.simulado:
            print("[LIDAR] Deteniendo escaneo...")
            self.ser.write(b'\xa5\x25') 
            self.ser.close()
        self.motor_encendido = False
