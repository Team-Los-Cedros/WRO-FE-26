# Proyecto Future Engineers - Team Los Cedros (WRO 2026)

Bienvenidos al repositorio oficial del Team Los Cedros, integrado por 3 estudiantes del Colegio Los Cedros en Valera, Estado Trujillo, Venezuela. Aquí compartimos el código fuente, los diagramas eléctricos y la documentación técnica de nuestro vehículo autónomo para la World Robot Olympiad (WRO) 2026, en la categoría Future Engineers.

Este proyecto es el resultado de muchas horas de diseño, pruebas en pista y, sobre todo, pasión por la robótica. Para la competencia de este año, desarrollamos un vehículo basado en una arquitectura de procesamiento dual: un "Cerebro" de alto rendimiento acelerado por Hardware (Raspberry Pi 5 + AI Hat+ de 26 TOPS) encargado del procesamiento del entorno y toma de decisiones, y un "Actuador" (Raspberry Pi Pico 2) dedicado al control de movimiento de baja latencia y estabilidad en tiempo real.

---

## Integrantes del Equipo y Roles

El Team Los Cedros está conformado por tres estudiantes apasionados por la tecnología, donde cada uno aportó desde su área de especialización para dar vida al vehículo:

* **CARLOS DAVID DIAZ RIVAS** — *Estratega de Software (AI & Lidar)*: Encargado del desarrollo del código en la Raspberry Pi 5, la implementación de modelos de Inteligencia Artificial aprovechando el acelerador de hardware, algoritmos de visión/Lidar (Clustering y Segmentación) y la lógica de navegación autónoma.
* **DANIEL DAVID DIAZ RIVAS** — *Ingeniero de Hardware, Control y QA*: Responsable del diseño del mapa eléctrico, la calibración de la IMU MPU6050, la programación de los sistemas de bajo nivel en la Raspberry Pi Pico 2, además del plan de pruebas en pista y gestión de la documentación técnica.
* **CARLOS SANTIAGO PINTO ABREU** — *Diseñador Mecánico*: Encargado del ensamblaje del chasis, la distribución física de los componentes y la optimización de los sistemas de dirección por servomotor y tracción trasera.

---

## Arquitectura de Sistema Distribuida

Para cumplir con las demandas dinámicas de la competencia, implementamos una **arquitectura distribuida de dos capas**. Esta topología desacopla el procesamiento perceptivo de alto nivel de las tareas de control síncrono en tiempo real, optimizando el ancho de banda y eliminando cuellos de botella informáticos.

1. **Raspberry Pi 5 + AI Hat+ (Capa Coorporativa/Estratégica):** Unidad central de análisis. La Raspberry Pi 5 se apoya en un módulo de aceleración Raspberry Pi AI Hat+ de 26 TOPS para la inferencia de modelos neuronales. Su tarea principal es conectarse al sensor **RPLIDAR C1** mediante una comunicación serial binaria cruda (UART a USB a 460,800 bps) para capturar nubes de puntos del entorno. En lugar de transmitir cientos de lecturas geométricas crudas al actuador, la Pi 5 procesa el flujo en memoria mediante filtros de mediana móvil y segmenta la pista en tres vectores espaciales críticos: *Frontal*, *Izquierda* y *Derecha*. Esto reduce drásticamente el ruido informático y la latencia del canal.
2. **Raspberry Pi Pico 2 (Capa Ejecutora de Tiempo Real):** Unidad de control síncrono basada en microcontrolador. Al estar libre del procesamiento pesado del Lidar, la Pico 2 ejecuta un bucle de control de alta frecuencia que procesa de forma no bloqueante la telemetría inercial de la IMU (extrayendo el ángulo Y/Z integrado en tiempo real) y recibe las directrices de proximidad de las zonas de la Pi 5. Controla directamente las señales PWM del servo de dirección MG996R y la etapa de potencia MOSFET del motor de tracción trasera.

---

## Red de Distribución de Energía (Power Delivery) y Robustez Eléctrica

En un prototipo robótico de alta competencia, la estabilidad eléctrica es tan crítica como la optimización del código. Si el servomotor de dirección o el motor de tracción exigen picos elevados de corriente instantánea durante una curva, pueden provocar caídas severas de tensión (*brownouts*) capaces de reiniciar la Raspberry Pi 5, cuyo consumo se eleva al exigir al máximo la placa junto al AI Hat+ de 26 TOPS.

Para blindar el sistema contra estas fluctuaciones, diseñamos una arquitectura de alimentación desacoplada basada en los siguientes pilares técnicos:

### 1. Matriz de Distribución y Capacidad de Corriente
Cada línea de voltaje fue calculada en función de su tensión y del amperaje máximo soportado para asegurar un funcionamiento holgado de los componentes:

| Fuente / Regulador | V. Entrada | V. Salida | Corriente Máx. | Destino Principal y Justificación Técnica |
| :--- | :--- | :--- | :--- | :--- |
| Baterías 21700 (2S) | N/A | 7.4V - 8.4V | 30A (Descarga) | Línea directa a `VM` del Puente H. Soporta los picos de arranque del motor de tracción sin degradar la línea lógica. |
| Regulador XL1509 | 7.4V - 8.4V | 6.0V | 2.0A | `VCC` de potencia del Servo MG996R. Evita que el torque dinámico máximo afecte la electrónica de control. |
| Regulador XL4015 | 7.4V - 8.4V | 5.2V | 5.0A | Pines 5V GPIO de la Raspberry Pi 5 + AI Hat+. Entrega los 5A requeridos para alimentar de forma limpia y constante ambos módulos bajo alta carga de procesamiento y prevenir alertas de under-voltage. |
| Puerto VBUS (Pico 2) | 5.0V (USB) | 5.0V | 500mA | `VCC` Lógico del Puente H TB6612FNG, aislándolo completamente de la potencia bruta de los motores. |
| Pin 3V3_OUT (Pico 2) | 5.0V (Int.) | 3.3V | 300mA | `VCC` de la IMU MPU6050. Proporciona una alimentación ultraestable para obtener lecturas precisas de telemetría. |

### 2. Referencia y Masa Unificada (Anti-Ruido)
* **Tierra Común (GND Común):** Para evitar errores de lectura y asegurar la correcta interpretación de las señales lógicas y PWM entre ambos controladores, todos los puntos de referencia de tierra (GND) del vehículo confluyen en una topología de nodo central en estrella. Esto previene eficazmente los bucles de tierra (*ground loops*) y estabiliza la comunicación serial.

### 3. Sistema de Seguridad y Protecciones por Hardware
* **Aislamiento de Líneas de Potencia:** El circuito de potencia (destinado al consumo del servomotor y el motor de tracción) se encuentra físicamente separado del plano de alimentación lógico (Raspberry Pi 5 / AI Hat+ y Pico 2). De este modo, si un motor experimenta una alta carga de trabajo en pista, el drenaje excesivo de corriente se gestiona directamente desde la batería a través de los reguladores dedicados, impidiendo que afecte o interrumpa el procesamiento del "Cerebro Estratégico".

---

## Mapa de Conexiones (Pinout)

### Conexiones de la Raspberry Pi Pico 2
*Nota de optimización de hardware: La interfaz I2C destinada a la IMU fue migrada de manera física a los pines GP16 y GP17 para liberar canales PWM en los bloques inferiores del microcontrolador y mitigar el ruido electromagnético inducido por los cables de potencia de los motores.*

| Componente | Pin Físico (Pico 2) | ID del Pin | Tipo de Señal | Función |
| :--- | :--- | :--- | :--- | :--- |
| Servo MG996R | Pin 20 | `GP15` | Salida PWM | Control de ángulo de dirección (Giro) |
| Puente H (PWMB) | Pin 34 | `GP28` | Salida PWM | Control de velocidad del motor de tracción (Canal B) |
| Puente H (BIN2) | Pin 32 | `GP27` | Salida Digital | Dirección de rotación del motor |
| Puente H (BIN1) | Pin 31 | `GP26` | Salida Digital | Dirección de rotación del motor |
| Puente H (STBY) | Pin 29 | `GP22` | Salida Digital | Activación del Driver / Freno de seguridad |
| MPU6050 (SDA) | Pin 21 | `GP16` | I2C0 SDA (Pull-Up) | Bus de Datos de la IMU |
| MPU6050 (SCL) | Pin 22 | `GP17` | I2C0 SCL (Pull-Up) | Bus de Reloj de la IMU |

### Conexiones de la Raspberry Pi 5 (El Estratega)
* **AI Hat+ 26TOPS:** Acoplado directamente sobre la interfaz PCIe de la Raspberry Pi 5 para transferencias de datos de baja latencia con el coprocesador neural.
* **RPLIDAR C1:** Conectado a un puerto USB 3.0 dedicado (Puerto Azul) capaz de proveer la corriente de arranque del motor de escaneo y sostener la transferencia binaria a 460,800 baudios.
* **Raspberry Pi Pico 2:** Interconectada mediante la interfaz USB nativa de la Pico hacia un puerto USB de la Pi 5. Actúa como canal UART bidireccional estable operando en modo VCP (Virtual COM Port) a 115,200 bps.
* **Pulsador (Botón Start):** Conectado al pin `GPIO17` (Pin físico 11) configurado con una resistencia Pull-Down externa de 10kOhm hacia tierra para disparar la rutina autónoma en pista.

---

## Protocolo de Comunicación Inter-Procesador (IPC)

Para enlazar ambos sistemas embebidos de manera ligera y determinista, implementamos un protocolo de transmisión de tramas planas empaquetadas por software. 

La Raspberry Pi 5 calcula la reducción geométrica de las distancias del LiDAR en centímetros (aplicando filtros de mediana estables de 3 muestras) y envía una cadena de bytes con codificación UTF-8 bajo la siguiente estructura cíclica a 10Hz:

```text
TRAMA UART -> [Distancia_Frontal]$[Distancia_Izquierda]$[Distancia_Derecha]\n
```
---

## Anatomía del Repositorio

Cumpliendo rigurosamente las regulaciones internacionales de la WRO, organizamos la estructura de archivos de forma modular, limpia y totalmente auditable:

```text
├── src/                      # Código fuente de la arquitectura distribuida
│   ├── pico/                 # Firmware embebido en MicroPython (Raspberry Pi Pico 2)
│   │   ├── main.py           # Bucle principal de control e interrupciones en tiempo real
│   │   ├── motores.py        # Abstracción PWM para servomotor de dirección y puente H
│   │   ├── test_motores.py   # Script crudo de calibración de ángulos del actuador servo
│   │   ├── imu.py            # Inicialización de registros I2C0 e integración del ángulo Yaw (Z)
│   │   └── serial_receiver.py# Parser serial no bloqueante para adquisición de zonas del Lidar
│   └── pi5/                  # Scripts en Python 3 de alto nivel (Raspberry Pi 5)
│       ├── main_control.py   # Orquestador central de navegación, inferencia de IA y estrategia
│       ├── lidar_handler.py  # Driver de adquisición binaria directa y filtrado de mediana para RPLIDAR C1
│       └── serial_sender.py  # Módulo emisor de telemetría espacial compactada hacia la Pico 2
├── t-photos/                 # Registro fotográfico oficial de las jornadas de desarrollo del equipo
├── v-photos/                 # Las 6 capturas reglamentarias obligatorias del coche desde los ángulos rectores
├── video/                    # Contiene el archivo video.md con el hipervínculo a la mejor vuelta en pista
├── schemes/                  # Planos esquemáticos, diagramas de flujo y mapas eléctricos en LaTeX
├── datasheets/               # Hojas técnicas oficiales de los semiconductores y sensores utilizados
└── README.md                 # Documentación técnica principal (este archivo)
