# src/pi5/main_control.py
import time
from lidar_handler import LidarHandler

# Intentar importar serial para la comunicación con la Pico 2
try:
    import serial
except ImportError:
    serial = None

# Configuración de umbrales de proximidad (en cm)
DIST_CRITICA_FRENTE = 40.0  
DIST_MIN_LATERAL = 20.0     

# Parámetros de calibración del control proporcional y servo
KP = 1.5                    
SERVO_CENTRO = 90           
SERVO_MAX_IZQ = 50          
SERVO_MAX_DER = 130         

class CarController:
    def __init__(self):
        print("Inicializando controlador de navegación...")
        self.lidar = LidarHandler()
        self.estado_actual = "INIT"
        self.corriendo = True
        self.pico = None
        
        # Configuración del puerto serial hacia la Raspberry Pi Pico 2
        # Cambiar '/dev/ttyACM0' si el puerto de la Pico cambia en la Pi 5
        try:
            if serial and int(time.time()) % 1 == 0: # Simulación flexible de puerto
                import os
                if os.path.exists('/dev/ttyACM0'):
                    self.pico = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
                    print("Conectado a la Raspberry Pi Pico 2.")
                else:
                    print("Pico 2 física no detectada. Comandos seriales en modo simulación.")
        except Exception as e:
            print(f"Error al inicializar puerto de la Pico: {e}")

    def determinar_estado(self, frontal, izquierda, derecha):
        if frontal < DIST_CRITICA_FRENTE and izquierda < DIST_MIN_LATERAL and derecha < DIST_MIN_LATERAL:
            return "BLOQUEADO"
        if frontal < DIST_CRITICA_FRENTE:
            return "GIRO_EVASIVO"
        return "CRUCERO_CENTRADO"

    def calcular_angulo_servo(self, izq, der):
        error = izq - der
        ajuste = error * KP
        angulo = SERVO_CENTRO - ajuste
        angulo = max(SERVO_MAX_IZQ, min(angulo, SERVO_MAX_DER))
        return round(angulo, 1)

    def enviar_comandos_pico(self, velocidad, angulo):
        """ Envía la telemetría formateada a la Pico 2 por Serial """
        # Formato de trama simple y directo para parsear en MicroPython en la Pico: "V,A\n"
        trama = f"{velocidad},{angulo}\n"
        
        if self.pico and self.pico.is_open:
            try:
                self.pico.write(trama.encode('utf-8'))
            except Exception as e:
                print(f"Error de envío serial: {e}")
        else:
            # Output de depuración en laptop para verificar que la trama es correcta
            pass 

    def procesar_navegacion(self):
        try:
            print("Bucle de control activo.")
            
            while self.corriendo:
                lecturas = self.lidar.obtener_distancias_zonas()
                frente = lecturas['frontal']
                izq = lecturas['izquierda']
                der = lecturas['derecha']
                
                self.estado_actual = self.determinar_estado(frente, izq, der)
                velocidad_motor = 60
                
                if self.estado_actual == "CRUCERO_CENTRADO":
                    angulo_servo = self.calcular_angulo_servo(izq, der)
                    velocidad_motor = 80  
                    modo_manejo = "P_CONTROL"
                        
                elif self.estado_actual == "GIRO_EVASIVO":
                    velocidad_motor = 45  
                    modo_manejo = "EVASION"
                    angulo_servo = SERVO_MAX_IZQ if izq > der else SERVO_MAX_DER
                        
                elif self.estado_actual == "BLOQUEADO":
                    velocidad_motor = -50  
                    angulo_servo = SERVO_CENTRO
                    modo_manejo = "REVERSA"

                # Envío de datos al hardware
                self.enviar_comandos_pico(velocidad_motor, angulo_servo)

                # Salida de telemetría para depuración
                print(f"| F: {frente:5.1f}cm | I: {izq:5.1f}cm | D: {der:5.1f}cm "
                      f"| ESTADO: {self.estado_actual:16} "
                      f"| MTR: {velocidad_motor:4} | SRV: {angulo_servo}° | TRAMA: {velocidad_motor},{angulo_servo}")
                
                time.sleep(0.15) 
                
        except KeyboardInterrupt:
            print("\nDeteniendo controlador...")
            self.enviar_comandos_pico(0, SERVO_CENTRO) # Parar motor antes de salir
            self.lidar.detener()
            self.corriendo = False

if __name__ == "__main__":
    carro = CarController()
    carro.procesar_navegacion()
