# src/pi5/lidar_handler.py
import serial
import time
from collections import deque

class ProcesadorLidar:
    def __init__(self, port='/dev/ttyUSB0', baudrate=460800):
        self.port = port
        self.baud = baudrate
        self.ser = None
        self.activo = False
        
        # Rango maxico de cobertura en cm
        self.max_dist_cm = 150.0 
        
        # Filtro de mediana por cada grado angular (3 muestras)
        self.history = {ang: deque(maxlen=3) for ang in range(360)}
        
        # Definicion geométrica de sectores para el carro
        self.zonas = {
            'frontal':   list(range(0, 25)) + list(range(335, 360)),
            'izquierda': list(range(60, 110)),
            'derecha':   list(range(250, 300))
        }
        
        self.conectar()

    def conectar(self):
        print(f"[LIDAR] Conectando a {self.port} con configuracion WRO...")
        try:
            # Esta es exactamente la misma configuracion de tu script funcional
            self.ser = serial.Serial(
                self.port, 
                baudrate=self.baud, 
                timeout=0.5,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # Reset fisico reglamentario para limpiar el canal
            self.ser.write(b'\xA5\x25') # STOP
            time.sleep(0.1)
            self.ser.write(b'\xA5\x40') # RESET
            time.sleep(1.5)
            self.ser.reset_input_buffer()
            
            # Iniciar escaneo estandar
            self.ser.write(b'\xA5\x20')
            
            # Sincronizacion de cabecera estricta
            while True:
                if self.ser.read(1) == b'\xA5' and self.ser.read(1) == b'\x5A':
                    self.ser.read(5) # Ignorar descriptor inicial
                    break
                    
            self.activo = True
            print("[LIDAR] OK - Motor girando y cabecera sincronizada.")
        except Exception as e:
            print(f"[LIDAR] Fallo critico de enlace: {e}")
            self.activo = False

    def procesar_zonas(self):
        """Lee el buffer de Linux en rafaga y devuelve los minimos de cada zona"""
        if not self.activo:
            return {'frontal': self.max_dist_cm, 'izquierda': self.max_dist_cm, 'derecha': self.max_dist_cm}

        try:
            # Lectura en rafaga si hay datos acumulados
            if self.ser.in_waiting >= 100:
                raw_data = serial.Serial.read(self.ser, self.ser.in_waiting)
                i = 0
                length = len(raw_data)
                
                while i + 4 < length:
                    if not (raw_data[i+1] & 0x01): # Validar bit de sincronia
                        i += 1
                        continue
                        
                    b0, b1, b2, b3, b4 = raw_data[i:i+5]
                    i += 5
                    
                    calidad = b0 >> 2
                    if calidad < 15: # Filtrar ruido de baja calidad
                        continue
                        
                    angulo_exacto = ((b2 << 7) | (b1 >> 1)) / 64.0
                    distancia_cm = (((b4 << 8) | b3) / 4.0) / 10.0
                    
                    if 0 < distancia_cm <= self.max_dist_cm:
                        ang_entero = int(angulo_exacto) % 360
                        self.history[ang_entero].append(distancia_cm)
        except Exception as e:
            print(f"[LIDAR] Error de lectura: {e}")

        # Recompilar mapa aplicando filtro de mediana
        distancias_estables = {ang: self.max_dist_cm for ang in range(360)}
        for ang_entero, lecturas in self.history.items():
            if len(lecturas) == 3:
                distancias_estables[ang_entero] = sorted(lecturas)[1]

        # Reducir a las 3 zonas de control del carro
        min_obstaculos = {'frontal': self.max_dist_cm, 'izquierda': self.max_dist_cm, 'derecha': self.max_dist_cm}
        for zona, angulos in self.zonas.items():
            valores_zona = [distancias_estables[a] for a in angulos]
            if valores_zona:
                min_obstaculos[zona] = min(valores_zona)

        return min_obstaculos

    def apagar(self):
        if self.ser and self.ser.is_open:
            print("🔌 Apagando LiDAR y liberando puerto...")
            try:
                self.ser.write(b'\xA5\x25') # STOP
                time.sleep(0.05)
                self.ser.close()
            except:
                pass
            print("[LIDAR] Apagado limpio completo.")

# --- PRUEBA LOCAL ---
if __name__ == '__main__':
    lidar = ProcesadorLidar()
    try:
        print("[DEBUG] Ejecutando lectura de zonas en tiempo real (Ctrl+C para salir)...")
        while True:
            datos = lidar.procesar_zonas()
            print(f"ZONAS(cm) -> F: {datos['frontal']:.1f} | I: {datos['izquierda']:.1f} | D: {datos['derecha']:.1f}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        lidar.apagar()
