# src/pico/serial_receiver.py
from machine import UART, Pin
import select
import sys
import time

class ReceptorSerial:
    def __init__(self):
        # En MicroPython, sys.stdin lee directo el puerto USB serial nativo (VCP)
        # configurado cuando conectas la Pico por USB a la Pi 5.
        self.buffer = ""
        print("[UART] Escuchando puerto USB serial nativo...")

    def leer_trama_pista(self):
        """
        Monitorea el buffer serial sin congelar el procesador (Non-blocking).
        Retorna una tupla (frontal, izquierda, derecha) si hay datos validos.
        """
        # select comprueba si hay bytes listos en el buffer de entrada de stdin
        r, _, _ = select.select([sys.stdin], [], [], 0)
        
        if r:
            char = sys.stdin.read(1)
            if char == '\n':
                trama = self.buffer
                self.buffer = "" # Vaciar buffer para la siguiente linea
                
                # Procesar paquete crudo
                try:
                    if "$" in trama:
                        partes = trama.split("$")
                        if len(partes) == 3:
                            f = float(partes[0])
                            i = float(partes[1])
                            d = float(partes[2])
                            return f, i, d
                except ValueError:
                    pass # Ignorar tramas corruptas o incompletas
            else:
                self.buffer += char
                
        return None

# TESTING LOCAL
if __name__ == '__main__':
    rx = ReceptorSerial()
    while True:
        datos = rx.leer_trama_pista()
        if datos:
            front, izq, der = datos
            print(f"[RX OK] F: {front}cm | I: {izq}cm | D: {der}cm")
        time.sleep_ms(10)
