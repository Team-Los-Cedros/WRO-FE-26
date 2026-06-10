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
        
        # Limite visual util de pista en cm
        self.max_dist_cm = 150.0 
        
        # Estructura de filtrado por software (Mediana de 3 muestras)
        self.history = {ang: deque(maxlen=3) for ang in range(360)}
        
        # Sectores clave de navegacion (Coordenadas en grados polares)
        # 0 deg es adelante, 90 izquierda, 270 derecha
        self.zonas = {
            'frontal':   list(range(0, 25)) + list(range(335, 360)),
            'izquierda': list(range(60, 110)),
            'derecha':   list(range(250, 300))
        }
        
        self.conectar()

    def conectar(self):
        print(f"[LIDAR] Abriendo canal UART en {self.port} a {self.baud}...")
        try:
            self.ser = serial.Serial(
                self.port, 
                baudrate=self.baud, 
                timeout=0.5,
                xonxoff=False, rtscts=False, dsrdtr=False
            )
            
            # Reset de hardware reglamentario via comandos crudos
            self.ser.write(b'\xA5\x25') # STOP
            time.sleep(0.1)
            self.ser.write(b'\xA5\x40') # RESET
            time.sleep(1.2)
            self.ser.reset_input_buffer()
            
            # Comando de inicio de escaneo estandar
            self.ser.write(b'\xA5\x20')
            
            # Sincronizacion inicial del descriptor de respuesta
            while True:
                if self.ser.read(1) == b'\xA5' and self.ser.read(1) == b'\x5A':
                    self.ser.read(5) # Consumir payload del descriptor
                    break
                    
            self.activo = True
            print("[LIDAR] Sensor en linea. Escaneo iniciado correctamente.")
        except Exception as e:
            print(f"[LIDAR] Fallo de enlace serial: {e}")
            self.activo = False

    def capturar_frame(self):
        """Procesa la rafaga de bytes del buffer de memoria e inyecta al filtro"""
        if not self.activo or self.ser.in_waiting < 100:
            return

        try:
            raw = self.ser.read(self.ser.in_waiting)
            idx = 0
            length = len(raw)
            
            while idx + 4 < length:
                # Validacion del bit de inicio de la trama de 5 bytes
                if not (raw[idx+1] & 0x01):
                    idx += 1
                    continue
                    
                b0, b1, b2, b3, b4 = raw[idx:idx+5]
                idx += 5
                
                # Descartar ecos de baja calidad reflectiva (ruido de pista)
                if (b0 >> 2) < 15:
                    continue
                    
                ang = int(((b2 << 7) | (b1 >> 1)) / 64.0) % 360
                dist = (((b4 << 8) | b3) / 4.0) / 10.0 # Convertido a cm
                
                if 0 < dist <= self.max_dist_cm:
                    self.history[ang].append(dist)
                    
        except Exception as e:
            print(f"[LIDAR] Error en lectura de flujo binario: {e}")

    def procesar_zonas(self):
        """Aplica el filtro de mediana y mapea los sectores de colision"""
        self.capturar_frame()
        
        # Inicializamos los sectores con el rango maximo por defecto (Via despejada)
        distancias_estables = {ang: self.max_dist_cm for ang in range(360)}
        
        # Extraer mediana matematica de los angulos con datos completos
        for ang, deq in self.history.items():
            if len(deq) == 3:
                distancias_estables[ang] = sorted(deq)[1]
                
        # Clasificar lecturas minimas en las zonas de control del carro
        min_obstaculos = {'frontal': self.max_dist_cm, 'izquierda': self.max_dist_cm, 'derecha': self.max_dist_cm}
        
        for zona, angulos in self.zonas.items():
            valores_zona = [distancias_estables[a] for a in angulos]
            if valores_zona:
                min_obstaculos[zona] = min(valores_zona)
                
        return min_obstaculos

    def apagar(self):
        if self.ser and self.ser.is_open:
            print("[LIDAR] Enviando comando STOP y cerrando puerto...")
            try:
                self.ser.write(b'\xA5\x25')
                time.sleep(0.05)
                self.ser.close()
            except:
                pass
            print("[LIDAR] Canal liberado.")

# --- UNIT TEST ---
if __name__ == '__main__':
    lidar = ProcesadorLidar()
    try:
        print("[DEBUG] Loop de telemetria de zonas activo (Ctrl+C para salir)...")
        while True:
            datos = lidar.procesar_zonas()
            print(f"ZONAS(cm) -> F: {datos['frontal']:.1f} | I: {datos['izquierda']:.1f} | D: {datos['derecha']:.1f}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        lidar.apagar()
        print("[DEBUG] Pruebas terminadas.")
