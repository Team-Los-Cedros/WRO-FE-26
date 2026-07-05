# Módulo de Alto Nivel y Orquestación (Raspberry Pi 3B)

Este submódulo contiene la lógica de alto nivel encargada de la percepción espacial, procesamiento de visión artificial y la orquestación síncrona de las fases de carrera para el vehículo autónomo del **Team Los Cedros (WRO 2026)**.

El componente central es el script maestro `controlador_inicio.py`, diseñado para operar de manera pasiva y ultra-ligera en segundo plano dentro de **Raspberry Pi OS Lite (64-bit)**, dedicando el 100% de los recursos lógicos a la escucha inmediata de eventos de hardware.

---

## 1. Arquitectura de Orquestación

Para garantizar que el vehículo sea 100% autónomo desde el momento en que se conecta la batería en la pista (requisito estricto de las regulaciones de la WRO), el sistema se divide en tres capas asíncronas:

1. **`controlador_inicio.py`**: Script demonio en Python que corre en un bucle infinito de alta frecuencia monitoreando los pines de entrada.
2. **`wro_start.service`**: Unidad de servicio nativa de Linux (`systemd`) que fuerza el auto-arranque del script maestro inmediatamente después de inicializar el kernel.
3. **Scripts de Carrera**: `Open_round.py` (Control proporcional guiado por RPLIDAR C1) y `Close_round.py` (Procesamiento matricial OpenCV HSV para evasión de pilares).

---

## 2. Mapa de Conexión de Hardware (Pines GPIO)

El script utiliza la asignación estándar de numeración **BCM (Broadcom)**. Las conexiones físicas deben realizarse referenciando los pines del conector de 40 pines de la Raspberry Pi 3B:

| Componente de Carrera | Identificador BCM (Código) | Pin Físico en la Placa | Tipo de Señal Lógica | Evento Asociado |
| :--- | :---: | :---: | :--- | :--- |
| **Botón de Ronda Abierta (OPEN)** | `GPIO 21` | **Pin 40** | Entrada Digital con Pull-Up | Ejecuta `Open_round.py` |
| **Botón de Ronda Cerrada (CLOSE)** | `GPIO 20` | **Pin 38** | Entrada Digital con Pull-Up | Ejecuta `Close_round.py` |
| **Referencia Electrónica** | `GND` | **Pin 39** | Tierra Común (Estrella) | Cierre de circuito de disparo |

> **Nota de Seguridad Eléctrica:** Al configurar internamente el acoplamiento `pull_up_down=GPIO.PUD_UP`, los interruptores físicos deben conmutar directamente a la línea de masa (`GND`). El procesador interpreta la caída de tensión a un estado lógico `LOW` ($0\,\text{V}$) como una pulsación válida, eliminando interferencias por ruido electromagnético parásito.

---

## 3. Demonio de Arranque Autónomo (`systemd`)

La integración en las capas del sistema operativo como un demonio de Linux se realiza mediante el archivo de configuración localizado en `/etc/systemd/system/wro_start.service`:

```ini
[Unit]
Description=Servicio Maestro de Inicio - Team Los Cedros WRO
After=multi-user.target serial-getty@ttyAMA0.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
ExecStart=/usr/bin/python3 /home/pi/controlador_inicio.py
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target

```

### Comandos Esenciales de Control de Terminal (Vía SSH)

Para realizar tareas de depuración en los boxes o mantenimiento de código, se utilizan los siguientes comandos nativos de Linux:

* **Iniciar el entorno autónomo manualmente:**
```bash
sudo systemctl start wro_start.service

```


* **Detener el demonio para edición de código:**
```bash
sudo systemctl stop wro_start.service

```


* **Recargar el script tras aplicar optimizaciones:**
```bash
sudo systemctl restart wro_start.service

```


* **Inspeccionar logs de ejecución y telemetría en tiempo real:**
```bash
sudo systemctl status wro_start.service

```



---

## 4. Diagnóstico de Problemas en Pista (Failsafe y Triage)

### El botón físico es pulsado pero el vehículo no inicia la marcha

1. **Validación de Capa de Software:** Ejecute `sudo journalctl -u wro_start.service -n 20` en la terminal. Verifique si el kernel de Linux imprimió en los logs las cadenas de texto `Botón Open, Ejecutando script` o `¡Botón CLOSE detectado!`.
2. **Aislamiento de Errores Hijos:** Si los logs del sistema operativo confirman la pulsación pero el vehículo permanece estático, el fallo no se encuentra en este submódulo, sino en los scripts de carrera (`Open_round.py`/`Close_round.py`), provocado por excepciones lógicas en los drivers de comunicación UART con la Pico 2 o bloqueos de lectura en el buffer del RPLIDAR C1.
3. **Aislamiento de Capa Física:** Si el demonio de Linux no registra ningún evento, verifique con un multímetro la continuidad física del interruptor o reemplace los jumpers conectados a los pines lógicos `GP21`/`GP20` y al común `GND`.
