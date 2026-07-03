# /home/pi/controlador_inicio.py
import RPi.GPIO as GPIO
import time
import subprocess
import os

# Configuración de Botones (BCM)
BOTON_OPEN = 21
BOTON_CLOSE = 20

RUTA_BASE = "/home/pi"
SCRIPT_OPEN = os.path.join(RUTA_BASE, "open_round.py")
SCRIPT_CLOSE = os.path.join(RUTA_BASE, "close_round.py")

def main():
    # Configuración limpia de pines para los botones con resistencia Pull-Up
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(BOTON_OPEN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BOTON_CLOSE, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    print("Controlador iniciado, Esperando Señal de botones (GP21 y GP20)")
    
    try:
        while True:
            # Leer el botón OPEN
            if GPIO.input(BOTON_OPEN) == GPIO.LOW:
                print("Botón Open, Ejecutando script")
                subprocess.run(["python3", SCRIPT_OPEN])
                time.sleep(1) # Anti-rebote para evitar que se ejecute dos veces seguidas
                
            # Leer el botón CLOSE
            elif GPIO.input(BOTON_CLOSE) == GPIO.LOW:
                print("¡Botón CLOSE detectado! Ejecutando script...")
                subprocess.run(["python3", SCRIPT_CLOSE])
                time.sleep(1) # Anti-rebote
                
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nPrograma terminado manualmente.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()