# Proyecto Future Engineers - Team Los Cedros (WRO 2026)

¡Hola! Bienvenidos al repositorio oficial del **Team Los Cedros**, integrado por 3 estudiantes del Colegio Los Cedros en Valera, Estado Trujillo, Venezuela. Aquí compartimos el código fuente, los diagramas eléctricos y la documentación técnica de nuestro vehículo autónomo para la **World Robot Olympiad (WRO) 2026**, en la categoría *Future Engineers*.

Este proyecto es el resultado de muchas horas de diseño, pruebas en pista y, sobre todo, pasión por la robótica. Para la competencia de este año, desarrollamos un vehículo basado en una **arquitectura de procesamiento dual**: un **"Cerebro"** de alto rendimiento acelerado por Hardware (Raspberry Pi 5 + AI Hat+ de 26 TOPS) encargado del procesamiento del entorno y toma de decisiones, y un **"Actuador"** (Raspberry Pi Pico 2) dedicado al control de movimiento de baja latencia y estabilidad en tiempo real.

---

## Integrantes del Equipo y Roles

El **Team Los Cedros** está conformado por tres estudiantes apasionados por la tecnología, donde cada uno aportó desde su área de especialización para dar vida al vehículo:

* **CARLOS DAVID DIAZ RIVAS** — **Estratega de Software (AI & Lidar):** Encargado del desarrollo del código en la Raspberry Pi 5, la implementación de modelos de Inteligencia Artificial aprovechando el acelerador de hardware, algoritmos de visión/Lidar (Clustering y Segmentación) y la lógica de navegación autónoma.
* **DANIEL DAVID DIAZ RIVAS** — **Ingeniero de Hardware, Control y QA:** Responsable del diseño del mapa eléctrico, la calibración de la IMU MPU6050, la programación de los sistemas de bajo nivel en la Raspberry Pi Pico 2, además del plan de **pruebas en pista y gestión de la documentación técnica**.
* **CARLOS SANTIAGO PINTO ABREU** — **Diseñador Mecánico:** Encargado del ensamblaje del chasis, la distribución física de los componentes y la optimización de los sistemas de dirección por servomotor y tracción trasera.

---

## Arquitectura de Hardware

Para garantizar que el vehículo tome decisiones algorítmicas en milisegundos sin perder el control dinámico, dividimos las tareas operativas utilizando dos controladores interconectados:

1. **Raspberry Pi 5 + AI Hat+ (El Estratega):** Es la unidad central de análisis y cómputo avanzado. La Raspberry Pi 5 se apoya en un módulo de aceleración **Raspberry Pi AI Hat+ de 26 TOPS** (Trillones de Operaciones por Segundo), lo que permite ejecutar lógica compleja e inferencia de IA en tiempo real sin ralentizar el sistema. Se encarga de procesar las nubes de puntos capturadas por el sensor **RPLIDAR C1**. Este dispositivo emite ráfagas láser de alta velocidad que rebotan en las paredes de la pista; al medir el tiempo de vuelo de la luz, calcula la distancia exacta de miles de coordenadas $(X, Y)$ por segundo. El software procesa estos datos crudos ejecutando algoritmos de filtrado de ruido, **Clustering** (agrupación de puntos para identificar obstáculos sólidos) y **Segmentación** para trazar la ruta óptima de navegación.
2. **Raspberry Pi Pico 2 (El Ejecutor):** Representa los reflejos y el músculo del sistema. Controla directamente el hardware físico de manera síncrona: la modulación de velocidad de tracción, el ángulo de la dirección asistida y la estabilización del chasis mediante la lectura de sensores inerciales.

### Componentes Clave
* **Procesamiento Base:** Raspberry Pi 5 + Raspberry Pi Pico 2.
* **Aceleración de IA:** Raspberry Pi AI Hat+ (Capacidad de cómputo neural de 26 TOPS).
* **Sensor de Entorno:** RPLIDAR C1 (Lidar de escaneo dinámico de alta velocidad conectado por USB a la Pi 5).
* **Unidad de Medición Inercial (IMU):** MPU6050 (Acelerómetro y Giroscopio de 6 ejes para el control del rumbo).
* **Actuador de Dirección:** Servomotor MG996R (Alto torque con piñonería metálica para giros mecánicos precisos).
* **Driver de Tracción:** Puente H TB6612FNG (Doble canal MOSFET de alta eficiencia).
* **Fuentes de Poder:** 2x Baterías de Ion de Litio 21700 en configuración 2S (7.4V - 8.4V nominales), garantizando alta capacidad de descarga.

---

## Red de Distribución de Energía (Power Delivery) y Robustez Eléctrica

En un prototipo robótico de alta competencia, la estabilidad eléctrica es tan crítica como la optimización del código. Si el servomotor de dirección o el motor de tracción exigen picos elevados de corriente instantánea durante una curva, pueden provocar caídas severas de tensión (*brownouts*) capaces de reiniciar la Raspberry Pi 5, cuyo consumo se eleva al exigir al máximo la placa junto al **AI Hat+ de 26 TOPS**.

Para blindar el sistema contra estas fluctuaciones, diseñamos una arquitectura de alimentación desacoplada basada en los siguientes pilares técnicos:

### 1. Matriz de Distribución y Capacidad de Corriente
Cada línea de voltaje fue calculada en función de su tensión y del amperaje máximo soportado para asegurar un funcionamiento holgado de los componentes:

| Fuente / Regulador | V. Entrada | V. Salida | Corriente Máx. | Destino Principal y Justificación Técnica |
| :--- | :--- | :--- | :--- | :--- |
| **Baterías 21700 (2S)** | N/A | 7.4V - 8.4V | 30A (Descarga) | Línea directa a `VM` del Puente H. Soporta los picos de arranque del motor de tracción sin degradar la línea lógica. |
| **Regulador XL1509** | 7.4V - 8.4V | **6.0V** | 2.0A | `VCC` de potencia del Servo MG996R. Evita que el torque dinámico máximo afecte la electrónica de control. |
| **Regulador XL4015** | 7.4V - 8.4V | **5.2V** | 5.0A | Pines 5V GPIO de la **Raspberry Pi 5 + AI Hat+**. Entrega los 5A requeridos para alimentar de forma limpia y constante ambos módulos bajo alta carga de procesamiento y prevenir alertas de *under-voltage*. |
| **Puerto VBUS (Pico 2)**| 5.0V (USB) | **5.0V** | 500mA | `VCC` Lógico del Puente H TB6612FNG, aislándolo completamente de la potencia bruta de los motores. |
| **Pin 3V3_OUT (Pico 2)**| 5.0V (Int.) | **3.3V** | 300mA | `VCC` de la IMU MPU6050. Proporciona una alimentación ultraestable para obtener lecturas precisas de telemetría. |

### 2. Referencia y Masa Unificada (Anti-Ruido)
* **Tierra Común (GND Común):** Para evitar errores de lectura y asegurar la correcta interpretación de las señales lógicas y PWM entre ambos controladores, todos los puntos de referencia de tierra (GND) del vehículo confluyen en una topología de nodo central en estrella. Esto previene eficazmente los bucles de tierra (*ground loops*) y estabiliza la comunicación serial.

### 3. Sistema de Seguridad y Protecciones por Hardware
* **Aislamiento de Líneas de Potencia:** El circuito de potencia (destinado al consumo del servomotor y el motor de tracción) se encuentra físicamente separado del plano de alimentación lógico (Raspberry Pi 5 / AI Hat+ y Pico 2). De este modo, si un motor experimenta una alta carga de trabajo en pista, el drenaje excesivo de corriente se gestiona directamente desde la batería a través de los reguladores dedicados, impidiendo que afecte o interrumpa el procesamiento del "Cerebro Estratégico".

---

## Mapa de Conexiones (Pinout)

### Conexiones de la Raspberry Pi Pico 2

| Componente | Pin Físico (Pico 2) | ID del Pin | Tipo de Señal | Función |
| :--- | :--- | :--- | :--- | :--- |
| **Servo MG996R** | Pin 16 | `GP12` | Salida PWM | Control de ángulo de dirección (Giro) |
| **Puente H (PWMA)** | Pin 34 | `GP28` | Salida PWM | Control de velocidad del motor de tracción |
| **Puente H (BIN2)** | Pin 32 | `GP27` | Salida Digital | Dirección de rotación del motor |
| **Puente H (BIN1)** | Pin 31 | `GP26` | Salida Digital | Dirección de rotación del motor |
| **Puente H (STBY)** | Pin 29 | `GP22` | Salida Digital | Activación del Driver / Freno de seguridad |
| **MPU6050 (SDA)** | Pin 4 | `GP2` | I2C1 SDA | Bus de Datos de la IMU |
| **MPU6050 (SCL)** | Pin 5 | `GP3` | I2C1 SCL | Bus de Reloj de la IMU |

### Conexiones de la Raspberry Pi 5 (El Estratega)
* **AI Hat+ 26TOPS:** Acoplado directamente sobre la interfaz PCIe de la Raspberry Pi 5 para transferencias de datos de baja latencia con el coprocesador neural.
* **RPLIDAR C1:** Conectado directamente a un puerto USB 3.0 dedicado para sostener la tasa de transferencia de datos requerida por el escaneo dinámico.
* **Raspberry Pi Pico 2:** Interconectada mediante un cable USB de datos para mantener la comunicación serial bidireccional (UART) y recibir alimentación de control.
* **Pulsador (Botón Start):** Conectado al pin `GPIO17` (Pin 11 física) configurado con una resistencia Pull-Down externa de $10\text{k}\Omega$ hacia tierra para disparar la rutina autónoma en pista.

---

## Anatomía del Repositorio

Cumpliendo rigurosamente las regulaciones internacionales de la WRO, organizamos la estructura de archivos de forma modular y auditable:

```text
├── src/                 # Código fuente del sistema embebido
│   ├── pico/            # Scripts en MicroPython ejecutados en la Pico 2
│   │   ├── main.py      # Bucle principal de control e interrupciones en tiempo real
│   │   ├── motores.py   # Controladores PWM para actuadores de tracción y dirección
│   │   └── imu.py       # Algoritmos de lectura y filtrado del MPU6050 (Eje Z)
│   └── pi5/             # Scripts en Python 3 ejecutados en la Raspberry Pi 5
│       ├── main_control.py  # Algoritmo principal de navegación, IA y evasión autónoma
│       └── lidar_handler.py # Interfaz de procesamiento y segmentación del RPLIDAR
├── t-photos/            # Registro fotográfico oficial del equipo
├── v-photos/            # Las 6 capturas obligatorias del coche desde todos los ángulos rectores
├── video/               # Contiene el archivo video.md con el hipervínculo a la mejor vuelta en pista
├── schemes/             # Planos esquemáticos, diagramas de flujo y el mapa eléctrico del hardware
├── datasheets/          # Hojas técnicas de especificaciones de los componentes electrónicos utilizados
└── README.md            # Documentación técnica principal (este archivo)
