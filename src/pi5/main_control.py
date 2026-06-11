# src/pi5/main_control.py
import time
from lidar_handler import LidarHandler

# CONFIGURACIÓN DE UMBRALES DE COMPETENCIA (QA CALIBRATION)
DIST_CRITICA_FRENTE = 40.0  # Menos de 40cm al frente significa: ¡FRENAR O GIRAR YA!
DIST_MIN_LATERAL = 20.0     # Si se acerca a menos de 20cm de una pared lateral, corregir
UMBRAL_PASILLO_CENTRO = 10.0 # Tolerancia de centrado (en cm)

class CarController:
    def __init__(self):
        print("[SISTEMA] Inicializando cerebro del vehículo WRO 2026...")
        self.lidar = LidarHandler()
        self.estado_actual = "INICIALIZANDO"
        self.corriendo = True

    def determinar_estado(self, frontal, izquierda, derecha):
        """ Máquina de Estados: Decide qué debe hacer el coche """
        # Estado 1: Emergencia / Bloqueo total
        if frontal < DIST_CRITICA_FRENTE and izquierda < DIST_MIN_LATERAL and derecha < DIST_MIN_LATERAL:
            return "BLOQUEADO"
        
        # Estado 2: Curva inminente (Pared al frente, buscar escape)
        if frontal < DIST_CRITICA_FRENTE:
            return "GIRO_EVASIVO"
        
        # Estado 3: Demasiado pegado a la izquierda
        if izquierda < DIST_MIN_LATERAL:
            return "CORRECCION_DERECHA"
            
        # Estado 4: Demasiado pegado a la derecha
        if derecha < DIST_MIN_LATERAL:
            return "CORRECCION_IZQUIERDA"
            
        # Estado 5: Camino libre, centrarse en el pasillo
        return "CRUCERO_CENTRADO"

    def procesar_navegacion(self):
        try:
            print("\n[QA] ARRANCANDO BUCLE DE PRUEBAS EN LAPTOP ")
            self.estado_actual = "CRUCERO_CENTRADO"
            
            while self.corriendo:
                # 1. Leer distancias del Lidar (Físico o Simulador)
                lecturas = self.lidar.obtener_distancias_zonas()
                frente = lecturas['frontal']
                izq = lecturas['izquierda']
                der = lecturas['derecha']
                
                # 2. Evaluar la situación con la máquina de estados
                nuevo_estado = self.determinar_estado(frente, izq, der)
                self.estado_actual = nuevo_estado
                
                # 3. Lógica de control de motores según el estado
                accion_servo = "CENTRADO (0°)"
                accion_motor = "AVANCE CONSTANTE"
                
                if self.estado_actual == "CRUCERO_CENTRADO":
                    # Intentar mantenerse en medio del pasillo (Control Proporcional Simple)
                    error_centro = izq - der
                    if abs(error_centro) > UMBRAL_PASILLO_CENTRO:
                        if error_centro > 0:
                            accion_servo = "SUAVE A LA DERECHA"
                        else:
                            accion_servo = "SUAVE A LA IZQUIERDA"
                    else:
                        accion_servo = "RECTO (Mantener Centro)"
                        
                elif self.estado_actual == "GIRO_EVASIVO":
                    accion_motor = "VELOCIDAD REDUCIDA"
                    # Girar hacia el lado donde haya más espacio libre en la pista
                    if izq > der:
                        accion_servo = "GIRO MÁXIMO IZQUIERDA (Evitando pared)"
                    else:
                        accion_servo = "GIRO MÁXIMO DERECHA (Evitando pared)"
                        
                elif self.estado_actual == "CORRECCION_DERECHA":
                    accion_servo = "ALERTA: Girar a la derecha para alejarse de pared izq"
                    
                elif self.estado_actual == "CORRECCION_IZQUIERDA":
                    accion_servo = "ALERTA: Girar a la izquierda para alejarse de pared der"
                    
                elif self.estado_actual == "BLOQUEADO":
                    accion_motor = "¡FRENADO DE EMERGENCIA / REVERSA!"
                    accion_servo = "RECTO"

                # 4. Telemetría por pantalla (Monitoreo de QA en vivo)
                print(f"| Lidar -> F: {frente:5.1f}cm | I: {izq:5.1f}cm | D: {der:5.1f}cm "
                      f"| ESTADO: {self.estado_actual:18} "
                      f"| MTR: {accion_motor:18} | SRV: {accion_servo}")
                
                # Frecuencia de actualización similar al coche real
                time.sleep(0.2)
                
        except KeyboardInterrupt:
            print("\n[SISTEMA] Deteniendo el controlador de forma segura...")
            self.lidar.detener()
            self.corriendo = False

if __name__ == "__main__":
    carro = CarController()
    carro.procesar_navegacion()
