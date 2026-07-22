# Módulo de Alto Nivel y Orquestación (Raspberry Pi 3B)

Este submódulo contiene la lógica de alto nivel encargada de la percepción espacial, procesamiento de visión artificial y la orquestación síncrona de las fases de carrera para el vehículo autónomo del **Team Los Cedros (WRO 2026)**.

El componente central es el script maestro `controlador_inicio.py`, diseñado para operar de manera pasiva y ultra-ligera en segundo plano dentro de **Raspberry Pi OS Lite (64-bit)**, dedicando el 100% de los recursos lógicos a la escucha inmediata de eventos de hardware.

---

## 1. Arquitectura de Orquestación

Para garantizar que el vehículo sea 100% autónomo desde el momento en que se conecta la batería en la pista (requisito estricto de las regulaciones de la WRO), el sistema se divide en tres capas asíncronas:

1. **`controlador_inicio.py`**: Script demonio en Python que corre en un bucle infinito de alta frecuencia monitoreando los pines de entrada.
2. **`wro_start.service`**: Unidad de servicio nativa de Linux (`systemd`) que fuerza el auto-arranque del script maestro inmediatamente después de inicializar el kernel.
3. **Scripts de Carrera**: `ronda_abierta/ronda_abierta.py` (Ronda Abierta, centrado proporcional guiado por RPLiDAR C1) y `ronda_cerrada/ronda_cerrada.py` (Ronda Cerrada, fusión de visión OpenCV + LiDAR para evasión de pilares — ver estructura modular abajo).
4. **`calibracion/calibrar_hsv.py`**: Herramienta de calibración interactiva (no se ejecuta en carrera). Levanta un servidor TCP en el puerto `5000` que recibe el streaming JPEG de la Pi Camera y expone sliders de OpenCV (`H/S/V Min/Max` por color) en la laptop del equipo para ajustar en vivo los umbrales de segmentación de los bloques verde y rojo antes de cada ronda.
5. **`calibracion/capturar_hsv.py`**: Herramienta de diagnóstico HSV sin GUI — corre 100% en la Pi y guarda a disco el frame crudo y las máscaras rojo/verde, para revisar la calibración sin necesitar una laptop con pantalla conectada al streaming.
6. **`requirements.txt`**: Dependencias Python del entorno de la Raspberry Pi 3B (OpenCV, pyserial, RPi.GPIO, numpy) — instalar con `pip install -r requirements.txt` para garantizar reproducibilidad del entorno de ejecución. `picamera2` se instala aparte por `apt` (ver [`INSTALACION.md`](../../INSTALACION.md)).
7. **`wro_start.service`**: Copia real del archivo de unidad `systemd`. Para reproducir el arranque autónomo en una Pi nueva: `sudo cp wro_start.service /etc/systemd/system/ && sudo systemctl enable wro_start.service`.

### Organización de `src/pi3B/`

Los scripts están agrupados en subcarpetas por responsabilidad. Esto es **solo organización del repositorio**: al desplegar, todos los `.py` de carrera se copian sin subcarpetas a `/home/pi/` (ver sección 5 de [`INSTALACION.md`](../../INSTALACION.md)), porque Python los importa por nombre de archivo entre sí (`import vision`, `from lidar_geometria import Medicion`, etc.) y no como paquete.

Dentro de `ronda_cerrada/` cada archivo cae en una de dos capas: **driver** (habla con el hardware, no interpreta nada) o **procesador** (interpreta datos, no toca hardware). Esta separación es deliberada — permite probar la interpretación (geometría del LiDAR, máquina de estados) sin el robot conectado.

```
src/pi3B/
├── ronda_cerrada/          # Todo lo que solo usa la Ronda Cerrada
│   ├── ronda_cerrada.py    # Punto de entrada
│   ├── navegacion.py       # Procesador: FSM de carrera/evasión/parqueo
│   ├── camara_driver.py    # Driver: adquisición de frames (Picamera2)
│   ├── vision.py           # Procesador: HSV rojo/verde + histéresis
│   ├── lidar_driver.py     # Driver: protocolo binario RPLIDAR C1
│   ├── lidar_geometria.py  # Procesador: paredes + clustering ABD
│   ├── tracker.py          # Procesador: persistencia del poste activo
│   ├── enlace_pico.py      # Driver: canal serial con la Pico 2
│   └── legacy/             # Versiones superadas, NO desplegar
├── ronda_abierta/
│   └── ronda_abierta.py    # Standalone: no comparte módulos con ronda_cerrada/
├── calibracion/             # Herramientas offline, no corren en carrera
│   ├── calibrar_hsv.py
│   └── capturar_hsv.py
├── controlador_inicio.py    # Orquestador: decide qué ronda lanzar según el botón
├── wro_start.service
└── requirements.txt
```

#### Módulos de `ronda_cerrada/`

| Archivo | Capa | Responsabilidad |
| :--- | :--- | :--- |
| `ronda_cerrada.py` | — | Punto de entrada y orquestador delgado: cablea los hilos, espera el botón, fija el cero IMU y vigila con un *watchdog* que la percepción siga viva. No contiene lógica de navegación. |
| `navegacion.py` | Procesador | Cerebro de la ronda: máquina de estados de carrera/evasión/parqueo como **lógica pura sin I/O** (todo el hardware se inyecta), lo que permite probarla fuera del robot con barridos sintéticos. |
| `camara_driver.py` | Driver | Adquisición de frames de la Pi Camera Module 3 (`picamera2`). No procesa color — entrega cada frame por callback (clase `CamaraDriver`). |
| `vision.py` | Procesador | Recibe cada frame de `camara_driver.py` y hace la detección HSV de postes rojo/verde con su histéresis de estabilización. |
| `lidar_driver.py` | Driver | Protocolo binario del RPLIDAR C1 y detección de barrido completo (clase `LidarDriver`). Entrega el barrido crudo (lista de ángulo/distancia), sin interpretar nada. |
| `lidar_geometria.py` | Procesador | Interpreta el barrido crudo: distancias por sector (con modo "Inercial"), sector frontal reconfigurable en caliente y clustering ABD para separar postes de paredes (clase `ProcesadorLidar`). Entrega un objeto `Medicion` por barrido. |
| `tracker.py` | Procesador | *Object persistence tracker* (clase `TrackerObstaculo`): posición estimada del poste activo, predicha por **rotación IMU + traslación por odometría de velocidad comandada**, y re-anclada con los clusters reales del LiDAR en cada barrido. |
| `enlace_pico.py` | Driver | Canal serial con la Pico 2 (clase `EnlacePico`): envío de consignas, lectura de telemetría IMU en hilo propio, cero de carrera ajustable y detección de telemetría caída. |

> **`ronda_cerrada/legacy/`** conserva `Close_round.py` y `Close2_round_Prueba1.py`, versiones superadas de la Ronda Cerrada que **no** deben desplegarse (ver el `README.md` de esa carpeta para el detalle de por qué se archivaron).

> **Nota:** `ronda_abierta/ronda_abierta.py` es autocontenido — reimplementa su propio parseo de LiDAR y protocolo serial en vez de reutilizar `lidar_driver.py`/`enlace_pico.py`. Es deuda técnica conocida, no un error de organización.

---

## 2. Mapa de Conexión de Hardware (Pines GPIO)

El script utiliza la asignación estándar de numeración **BCM (Broadcom)**. Las conexiones físicas deben realizarse referenciando los pines del conector de 40 pines de la Raspberry Pi 3B:

| Componente de Carrera | Identificador BCM (Código) | Pin Físico en la Placa | Tipo de Señal Lógica | Evento Asociado |
| :--- | :---: | :---: | :--- | :--- |
| **Botón de Ronda Abierta (OPEN)** | `GPIO 21` | **Pin 40** | Entrada Digital con Pull-Up | Ejecuta `ronda_abierta.py` |
| **Botón de Ronda Cerrada (CLOSE)** | `GPIO 20` | **Pin 38** | Entrada Digital con Pull-Up | Ejecuta `ronda_cerrada.py` |
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
2. **Aislamiento de Errores Hijos:** Si los logs del sistema operativo confirman la pulsación pero el vehículo permanece estático, el fallo no se encuentra en este submódulo, sino en los scripts de carrera (`ronda_abierta.py`/`ronda_cerrada.py`), provocado por excepciones lógicas en los drivers de comunicación UART con la Pico 2 o bloqueos de lectura en el buffer del RPLIDAR C1.
3. **Aislamiento de Capa Física:** Si el demonio de Linux no registra ningún evento, verifique con un multímetro la continuidad física del interruptor o reemplace los jumpers conectados a los pines lógicos `GP21`/`GP20` y al común `GND`.
