# src/pi5/serial_sender.py
import serial
import time

class EmisorSerial:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        # /dev/ttyACM0 suele ser el mapeo por defecto de la Pico conectada por USB a la Pi
        self.port = port
        self.baud = baudrate
        self.ser = None
        self.conectado = False
        
        self.conectar_uart()

    def conectar_uart(self):
        print(f"[UART] Estableciendo enlace con el ejecutor en {self.port}...")
        try:
            self.ser = serial.Serial(
                self.port,
                baudrate=self.baud,
                timeout=0.1,
                write_timeout=0.1
            )
            time.sleep(1.0) # Delay tipico de enumeracion de hardware
            self.conectado = True
            print("[UART] Enlace de datos listo.")
        except Exception as e:
            print(f"[UART] Error de conexion con la Pico: {e}")
            self.conectado = False

    def enviar_telemetria_pista(self, datos_zonas):
        """
        Empaqueta el diccionario en una trama plana: 'F_dist$I_dist$D_dist\n'
        Ejemplo: '120.5$150.0$45.2\n'
        """
        if not self.conectado or not self.ser:
            return False
            
        try:
            # Extraemos los flotantes formateados a un decimal para ahorrar ancho de banda
            f = f"{datos_zonas.get('frontal', 150.0):.1f}"
            i = f"{datos_zonas.get('izquierda', 150.0):.1f}"
            d = f"{datos_zonas.get('derecha', 150.0):.1f}"
            
            # Construccion del paquete crudo
            payload = f"{f}${i}${d}\n"
            
            # Envio directo convirtiendo a bytes
            self.ser.write(payload.encode('utf-8'))
            return True
        except Exception as e:
            print(f"[UART] Error en transmision de rafaga: {e}")
            self.conectado = False
            return False

    def cerrar(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[UART] Puerto serial liberado.")

# --- MOCK TEST ---
if __name__ == '__main__':
    # Prueba rapida simulando datos falsos del Lidar
    emisor = EmisorSerial()
    if emisor.conectado:
        try:
            print("[DEBUG] Enviando rafagas de prueba cada 100ms... (Ctrl+C para parar)")
            mock_dist = 120.0
            while True:
                # Simulamos que las distancias cambian ligeramente
                mock_data = {
                    'frontal': mock_dist,
                    'izquierda': mock_dist + 5,
                    'derecha': mock_dist - 10
                }
                emisor.enviar_telemetria_pista(mock_data)
                print(f"[TX] {mock_data}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            emisor.cerrar()
            print("[DEBUG] Test finalizado.")