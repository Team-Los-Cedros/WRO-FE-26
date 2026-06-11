# src/pi5/main_control.py
import time
from lidar_handler import LidarHandler

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

    def determinar_estado(self, frontal, izquierda, derecha):
        # Monitoreo de colisión o bloqueo en pasillo
        if frontal < DIST_CRITICA_FRENTE and izquierda < DIST_MIN_LATERAL and derecha < DIST_MIN_LATERAL:
            return "BLOQUEADO"
        # Detección de pared al frente (curva)
        if frontal < DIST_CRITICA_FRENTE:
            return "GIRO_EVASIVO"
        # Comportamiento estándar en recta
        return "CRUCERO_CENTRADO"

    def calcular_angulo_servo(self, izq, der):
        # Cálculo del error de centrado respecto a las paredes laterales
        error = izq - der
        
        # Aplicación del control P para corregir la trayectoria
        ajuste = error * KP
        angulo = SERVO_CENTRO - ajuste
        
        # Restricción de límites para protección mecánica del servo
        angulo = max(SERVO_MAX_IZQ, min(angulo, SERVO_MAX_DER))
        return round(angulo, 1)

    def procesar_navegacion(self):
        try:
            print("Bucle de control activo.")
            
            while self.corriendo:
                # Lectura de datos desde el manejador del Lidar
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
                    # Asignación de giro al lado con mayor espacio libre
                    angulo_servo = SERVO_MAX_IZQ if izq > der else SERVO_MAX_DER
                        
                elif self.estado_actual == "BLOQUEADO":
                    velocidad_motor = -50  
                    angulo_servo = SERVO_CENTRO
                    modo_manejo = "REVERSA"

                # Salida de telemetría para depuración
                print(f"| F: {frente:5.1f}cm | I: {izq:5.1f}cm | D: {der:5.1f}cm "
                      f"| ESTADO: {self.estado_actual:16} "
                      f"| MODO: {modo_manejo:10} "
                      f"| MTR: {velocidad_motor:4} | SRV: {angulo_servo}°")
                
                time.sleep(0.15) # Frecuencia del ciclo de control a ~6.6 Hz
                
        except KeyboardInterrupt:
            print("\nDeteniendo controlador...")
            self.lidar.detener()
            self.corriendo = False

if __name__ == "__main__":
    carro = CarController()
    carro.procesar_navegacion()
