# Guía de Usuario: Sistema de Control de Acceso (Solo Botones)

Esta guía describe el funcionamiento, la arquitectura y el mantenimiento del script orquestador simplificado para la Raspberry Pi, diseñado exclusivamente para escuchar pulsaciones de botones físicos y ejecutar scripts secundarios de automatización (`open_round.py` y `close_round.py`).

---

## 1. Vista General del Sistema

Para garantizar la máxima velocidad, estabilidad y compatibilidad con sistemas Linux (Raspberry Pi OS Lite 64 bits). El sistema opera de manera pasiva y ultra ligera, dedicando el 100% de los recursos a la escucha inmediata de los eventos de entrada.

### Componentes Principales
1. **`controlador_inicio.py`**: El script orquestador en Python que corre de fondo en bucle infinito de alta velocidad.
2. **`controlador.service`**: El (`systemd`) que asegura que el script inicie automáticamente con la Raspberry Pi y se autorecupere ante fallos.
3. **Scripts Externos**: `open_round.py` y `close_round.py`, encargados de las acciones físicas finales.

---

## 2. Mapa de Conexión de Hardware (Pines GPIO)

El script utiliza la numeración estándar **BCM (Broadcom)**. Las conexiones físicas deben realizarse referenciando los pines del conector de la placa tal como se detalla a continuación:

| Función | Identificador BCM (Código) | Pin Físico en la Placa | Tipo de Señal |
| :--- | :---: | :---: | :--- |
| **Botón de Apertura (OPEN)** | `GPIO 21` | **Pin 29** | Entrada digital con Pull-Up interno |
| **Botón de Cierre (CLOSE)** | `GPIO 20` | **Pin 38** o **Pin 40** (Según cableado) | Entrada digital con Pull-Up interno |
| **Referencia de Tierra** | `GND` | **Pin 39** | Tierra Común Eléctrica |

> **Nota Crítica de Hardware:** Al utilizar la resistencia interna de acoplamiento `PULL_UP`, los botones deben conectarse de tal forma que al ser presionados **unan directamente el pin GPIO con un pin de Tierra (GND)**. El script detecta el estado `LOW` (0V) como una pulsación válida.

---

## 3. Código Fuente del Orquestador (`/home/pi/controlador_inicio.py`)

```python
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
```

---

## 4. Gestión del Servicio del Sistema (`systemd`)

El orquestador está integrado en las capas del sistema operativo como un servicio nativo, lo que permite su ejecución sin intervención del usuario.

### Archivo de Configuración (`/etc/systemd/system/controlador.service`)

```ini
[Unit]
Description=Controlador de Botones y Rutinas de Inicio
After=multi-user.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/pi
ExecStart=/usr/bin/python3 /home/pi/controlador_inicio.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

### Comandos Esenciales de Control de Terminal (SSH)

* **Iniciar el servicio:**
```
sudo systemctl start controlador.service
```


* **Detener el servicio (Para realizar tareas de mantenimiento):**

```
sudo systemctl stop controlador.service
```


* **Reiniciar el servicio (Aplica cambios realizados en el código):**

```
sudo systemctl restart controlador.service
```


* **Verificar el estado y los logs de ejecución en tiempo real:**
```
sudo systemctl status controlador.service
```

---

## 5. Diagnóstico de Problemas 

### El botón se presiona pero no pasa nada

1. Ejecute `sudo systemctl status controlador.service` y observe las últimas líneas de salida en la terminal. Debería ver impreso `Boton OPEN detectado` o `Boton CLOSE detectado`.
2. Si los logs muestran la detección del botón pero los motores/mecanismos no se mueven, el problema está localizado dentro de `open_round.py` o `close_round.py` (ej. rutas de archivos, permisos, o llamadas bloqueantes de GPIO heredadas).
3. Si los logs no muestran la pulsación, verifique con un multímetro o un cable directo que el pin físico correspondiente (GP21 o GP20) esté haciendo contacto sólido con un pin GND al cerrarse el interruptor.
