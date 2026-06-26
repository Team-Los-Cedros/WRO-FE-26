 # Proyecto Future Engineers - Team Los Cedros (WRO 2026)

Bienvenidos al repositorio oficial del Team Los Cedros, integrado por 3 estudiantes del Colegio Los Cedros en Valera, Estado Trujillo, Venezuela. Aquí compartimos el código fuente, los diagramas eléctricos y la documentación técnica de nuestro vehículo autónomo para la World Robot Olympiad (WRO) 2026, en la categoría Future Engineers.

Este proyecto es el resultado de muchas horas de diseño, pruebas en pista y sobre todo, pasión por la robótica. Para la competencia de este año, desarrollamos un vehículo basado en una arquitectura de procesamiento dual: un "Cerebro" encargado del procesamiento del entorno mediante fusión de sensores y visión artificial (Raspberry Pi 3B), y un "Actuador" (Raspberry Pi Pico 2) dedicado al control de movimiento de baja latencia y estabilidad de variables críticas en tiempo real.

---
## 📌 Índice
1. [Introducción](#1-introducción)
   - 1.1 [Foto del Equipo](#11-foto-del-equipo)
   - 1.2 [Rol e Integrantes del Team](#12-rol-e-integrantes-del-team)
2. [Arquitectura del Sistema](#2-arquitectura-del-sistema)
   - 2.1 [Lista de Componentes y Funciones](#21-lista-de-componentes-y-funciones)
   - 2.2 [Sistema de Dirección Utilizado](#22-sistema-de-dirección-utilizado)
   - 2.3 [Análisis Visual y Ángulos de Dirección](#23-análisis-visual-y-ángulos-de-dirección)
   - 2.4 [Diseño de la Placa PCB](#24-diseño-de-la-placa-pcb)
   - 2.5 [Galería de Componentes](#25-galería-de-componentes)
3. [Software y Lenguajes de Programación](#3-software-y-lenguajes-de-programación)
   - 3.1 [Raspberry Pi 3B: Lenguaje de Programación](#31-raspberry-pi-3b-lenguaje-de-programación)
   - 3.2 [Raspberry Pi 3B: Función en el Robot](#32-raspberry-pi-3b-función-en-el-robot)
   - 3.3 [Raspberry Pi 3B: Lógica de Funcionamiento](#33-raspberry-pi-3b-lógica-de-funcionamiento)
   - 3.4 [Raspberry Pi Pico 2: Lenguaje de Programación](#34-raspberry-pi-pico-2-lenguaje-de-programación)
   - 3.5 [Raspberry Pi Pico 2: Función en el Robot](#35-raspberry-pi-pico-2-función-en-el-robot)
   - 3.6 [Raspberry Pi Pico 2: Lógica de Funcionamiento](#36-raspberry-pi-pico-2-lógica-de-funcionamiento)

---

## 1. Introducción

El Team Los Cedros está conformado por tres estudiantes:

### 1.1 Foto del Equipo
<p align="center">
  <img src="t-photos/Photo_Team.jpeg" alt="Team Los Cedros - WRO 2026" width="600px"/>
</p>

*Pie de foto: Integrantes del equipo de desarrollo.*

### 1.2 Rol e Integrantes del Team
| Integrante | Rol / Especialidad | Contribución Principal |
| :--- | :--- | :--- |
| **Daniel David Díaz Rivas** | Líder de Proyecto / Hardware | Diseño de PCB y ensamblaje mecánico. |
| **Carlos David Díaz Rivas** | Desarrollador de Software | Programación de la lógica en Raspberry Pi 3B. |
| **Carlos Santiago Pinto Abreu** | Especialista en Control | Firmware y calibración de la Raspberry Pi Pico 2. |

---



## 2. Arquitectura del Sistema

### 2.1 Lista de Componentes y Funciones

| Componente | Cantidad | Función Principal | Imagenes |
| :--- | :---: | :--- | :--- |
| Raspberry Pi 3B | 1 | Cerebro central, procesamiento de alto nivel y lógica principal. | <img src="v-photos/Rspr3B.jpg" alt="Raspherry Pi 3B" width="200px"/> |
| Raspberry Pi Pico 2 | 1 | Control de motores en tiempo real y lectura de sensores (Microcontrolador). | <img src="v-photos/Pico2.jpg" alt="Raspherry Pi 3B" width="200px"/>|
| [Nombre de Motor] | X | Propulsión del robot. | |
| [Nombre de Servomotor] | X | Control del sistema de dirección. | |
| [Nombre de Batería] | X | Alimentación del sistema. | |




### 2.2 Sistema de Dirección Utilizado

En nuestro vehículo utilizamos sistema de dirección Ackermann ya que este permite que las ruedas interiores y exteriores giren en ángulos diferentes alrededor de un único centro geométrico común, evitando el deslizamiento lateral de los neumáticos. Esta rodadura pura elimina la fricción innecesaria y proporciona una trayectoria altamente predecible, lo que facilita que los algoritmos de navegación calculen la posición exacta del vehículo mediante odometría precisa, optimizando además el consumo de energía y reduciendo el desgaste mecánico.

### 2.3 Análisis Visual y Ángulos de Dirección
A continuación se muestran los diagramas del sistema de dirección y los límites angulares configurados:

| Vista Frontal / Superior | Diagrama de Ángulos de Giro |
| :---: | :---: |
| ![Sistema de Dirección](ruta/a/foto-direccion.jpg) | ![Ángulos de Giro](ruta/a/foto-angulos.jpg) |

### 2.4 Diseño de la Placa PCB
El circuito electrónico y las conexiones fueron consolidadas en una placa PCB personalizada.

* **Software de diseño utilizado:** [Ej. KiCad / EasyEDA / Altium]
* **Esquema de la PCB:**

![Diseño PCB](ruta/a/foto-pcb.jpg)

### 2.5 Galería de Componentes
Aquí se pueden observar en detalle los componentes principales antes del ensamblaje final:

| Componente A | Componente B |
| :---: | :---: |
| ![Componente A](ruta/a/foto-comp1.jpg) | ![Componente B](ruta/a/foto-comp2.jpg) |

---









## 3. Software y Lenguajes de Programación

###  Sección: Raspberry Pi 3B

#### 3.1 Raspberry Pi 3B: Lenguaje de Programación
* **Lenguaje Principal:** `Python`  (o el que aplique, ej. C++)
* **Entorno/Framework:** 

#### 3.2 Raspberry Pi 3B: Función en el Robot
La Raspberry Pi 3B actúa como la **unidad de procesamiento central (CPU)**. Se encarga de las tareas pesadas que no requieren respuestas en tiempo real estricto, tales como:
* [Tarea 1: Ej. Procesamiento de imágenes con cámara]
* [Tarea 2: Ej. Toma de decisiones y algoritmos de navegación]
* [Tarea 3: Ej. Comunicación inalámbrica (Wi-Fi/Bluetooth)]

#### 3.3 Raspberry Pi 3B: Lógica de Funcionamiento
[Inicio] ➔ [Inicializar Sensores/Cámara] ➔ [Procesar Datos Ambientales]
│
[Enviar Comandos a RPi Pico 2] 🖚 ─── [Calcular Siguiente Movimiento]

*(Puede q cambiemos el diagrama de arriba paro explicar paso a paso el bucle principal del código aquí).*

---






###  Sección: Raspberry Pi Pico 2

#### 3.4 Raspberry Pi Pico 2: Lenguaje de Programación
* **Lenguaje Principal:** `MicroPython` / `C++` (via . . . )

#### 3.5 Raspberry Pi Pico 2: Función en el Robot
La Raspberry Pi Pico 2 actúa como el **controlador de bajo nivel (MCU)**, garantizando la ejecución de tareas críticas en tiempo real:
* Generación de señales PWM para los motores y servos de dirección.
* Lectura directa de sensores de proximidad/encoders.
* Ejecución de paradas de emergencia (Fail-safe).

#### 3.6 Raspberry Pi Pico 2: Lógica de Funcionamiento
Explicación detallada de cómo interactúa con la Pi 3B.






















## Arquitectura de Sistema Distribuida

- **Capa de Percepción (Raspberry Pi 3B)**: Centraliza el procesamiento visual y espacial. Captura datos de un sensor RPLIDAR C1 (UART-USB a 460,800 baudios) y de una cámara Raspherry Pi Cam Modulo 3, 12 MP. Mediante filtros de media móvil, procesa el flujo en memoria y sintetiza la pista en tres vectores geométricos esenciales (Frontal, Izquierdo y Derecho). Esto reduce el ruido informático y minimiza la latencia de transferencia al enviar solo datos sintetizados.



- **Capa de Control (Raspberry Pi Pico 2)**: Ejecuta un bucle síncrono no bloqueante de alta frecuencia. Integra la telemetría de una IMU (extrayendo el ángulo Yaw en tiempo real) con las directrices espaciales de la Pi 3B. Controla directamente, mediante señales PWM, el servo de dirección "   " y la etapa de potencia "   " del motor de tracción.

---

## Lógica de Control y Algoritmo de Navegación

### 1. Funcionamiento del Código de la Raspberry Pi 3B
Este script se divide en dos procesos que corren al mismo tiempo gracias al uso de Hilos (threading):

#### A. El Hilo del Lidar (leer_lidar)
El Lidar gira sobre su propio eje y envía ráfagas de puntos (ángulo y distancia) a una velocidad altísima (460,800 baudios). Si procesáramos punto por punto, el coche se volvería loco por el ruido o el retraso (jitter).

* **Zonificación:** El código toma cada punto recibido y, según su ángulo, lo mete en un contenedor o buffer específico (frente, izq_frontal, izq_lateral, izq_trasera).
* **Suavizado:** Cuando un contenedor junta más de 5 lecturas, calcula el promedio de la distancia y actualiza el diccionario global telemetria. Esto limpia la señal y le da al robot una percepción estable del entorno.

#### B. El Bucle Principal (Algoritmo de Control)
Este bucle corre de forma perpetua cada 20 milisegundos (50 veces por segundo) y ejecuta la lógica matemática:

* **Evaluación de Seguridad (Frente):** Revisa el sensor frontal. Si el espacio libre al frente es menor a 350 mm o la esquina delantera-izquierda se acerca a menos de 200 mm de una pared, la velocidad se clava en 0 para evitar una colisión destructiva. Si la pista está despejada, acelera a MAX_VEL (95%).
* **Cálculo del Error:** Compara la distancia real medida a la izquierda (izq_lat) contra la distancia que tú quieres mantener (DISTANCIA_OBJETIVO).
```text
Error = Distancia Real - Distancia Objetivo
```
* **Control Proporcional (P):** Multiplica ese error por KP.
  * Si el robot se aleja del centro (Distancia Real > Objetivo), el error es positivo. Al multiplicarlo por -1, da un ángulo negativo que hace que el coche gire a la izquierda para acercarse.
  * Si se pega demasiado (Distancia Real < Objetivo), el error es negativo. Al multiplicarlo por -1, da un ángulo positivo que hace que el coche gire a la derecha para alejarse.
* **Protección de Cola:** Si la diagonal trasera izquierda (izq_trasera) detecta que la parte de atrás del coche se está pegando demasiado al cuadro, artificialmente altera el error restándole 80 unidades. Esto obliga al coche a abrirse inmediatamente a la derecha para no golpear la esquina interna con la rueda trasera.
* **Envío Serial:** Empaqueta los valores calculados en una cadena simple de texto (ejemplo: "-15,95\n") y la manda por el puerto serial /dev/ttyACM0.

### 2. Funcionamiento del Código de la Raspberry Pico 2
La Pico 2 corre MicroPython y utiliza un bucle de lectura de bajísima latencia basado en select.select:

* **Lectura Serial no Bloqueante:** En lugar de quedarse congelada esperando datos, la Pico revisa la entrada serial (sys.stdin). Si hay datos frescos de la Pi 3B, los lee; si no, continúa su ciclo de inmediato para mantener los motores activos.
* **Mapeo del Servo (mapear_servo):** El servo de dirección se controla mediante modulación por ancho de pulsos (PWM) a 50Hz. La Pico recibe un ángulo de -35 a 35. Mediante la ecuación:
```text
Duty = 4950 + (10 * 52)
```
  Transforma esos grados en el pulso exacto en microsegundos que el servo necesita para mover las ruedas delanteras.
* **Control del Puente H BTS7960 (controlar_motores):** Este puente H industrial requiere señales de alta frecuencia (20,000Hz para que los motores no silben). El código toma el porcentaje de velocidad (0 a 100), lo escala a una resolución de 16 bits (0 a 65535) y activa el pin RPWM para avanzar. Si en algún momento la velocidad calculada es 0, el firmware pone en estado lógico bajo (LOW) tanto los pines de control (RPWM, LPWM) como los de habilitación (R_EN, L_EN), aplicando un frenado electrónico dinámico e inmediato sobre el motor.

---

## Red de Distribución de Energía (Power Delivery) y Robustez Eléctrica

En un prototipo robótico de alta competencia, la estabilidad eléctrica es tan crítica como la optimización del código. El uso del puente H BTS7960 permite manejar grandes corrientes para el motor de tracción trasera, pero sus picos de arranque pueden provocar caídas severas de tensión (brownouts) capaces de reiniciar la Raspberry Pi 3B si las líneas lógicas y de potencia no estuvieran correctamente aisladas.

Para blindar el sistema contra estas fluctuaciones, diseñamos una arquitectura de alimentación desacoplada basada en los siguientes pilares técnicos:

### 1. Matriz de Distribución y Capacidad de Corriente
Cada línea de voltaje fue calculada en función de su tensión y del amperaje máximo soportado para asegurar un funcionamiento holgado de los componentes:

| Fuente / Regulador | V. Entrada | V. Salida | Corriente Máx. | Destino Principal y Justificación Técnico |
| :--- | :--- | :--- | :--- | :--- |
| Baterías 21700 (2S) | N/A | 7.4V - 8.4V | 30A (Descarga) | Línea directa a los pines de potencia `B+` y `B-` del Puente H BTS7960. Maneja los picos exigidos por la tracción sin degradar la electrónica lógica. |
| Regulador XL1509 | 7.4V - 8.4V | 6.0V | 2.0A | `VCC` de potencia del Servo MG996R. Evita que el torque dinámico máximo afecte la electrónica de control. |
| Regulador XL4015 | 7.4V - 8.4V | 5.1V | 5.0A | Entrada de alimentación microUSB/GPIO de la Raspberry Pi 3B, la cámara Arducam y el sensor RPLIDAR C1. Entrega corriente limpia y constante bajo alta carga de procesamiento. |
| Pin 3V3_OUT (Pico 2) | 5.0V (Int.) | 3.3V | 300mA | `VCC` de la IMU MPU6050 y alimentación lógica (`VCC`) de las entradas del BTS7960, aislándolo por completo del ruido eléctrico de los motores. |

### 2. Referencia y Masa Unificada (Anti-Ruido)
* **Tierra Común (GND Común):** Para evitar errores de lectura y asegurar la correcta interpretación de las señales lógicas y PWM entre ambos controladores y los drivers de potencia, todos los puntos de referencia de tierra (GND) del vehículo confluyen en una topología de nodo central en estrella. Esto previene eficazmente los bucles de tierra (ground loops) y estabiliza la comunicación serial.

### 3. Sistema de Seguridad y Protecciones por Hardware
* **Aislamiento de Líneas de Potencia:** El circuito de alta potencia (destinado al consumo del servomotor y el motor de tracción a través del BTS7960) se encuentra físicamente separado del plano de alimentación lógico (Raspberry Pi 3B y Pico 2). De este modo, si un motor experimenta una alta carga de trabajo en pista, el drenaje excesivo de corriente se gestiona directamente desde la batería a través de los reguladores dedicados, impidiendo que afecte o interrumpa el procesamiento del "Cerebro Estratégico".

---

## Mapa de Conexiones (Pinout)

### Conexiones de la Raspberry Pi Pico 2
*Nota de optimización de hardware: El puente H BTS7960 requiere pines independientes de habilitación (EN) y control de sentido/velocidad por PWM para el lado izquierdo (L_PWM/L_EN) y derecho (R_PWM/R_EN). La interfaz I2C destinada a la IMU se mantiene físicamente aislada en los pines GP16 y GP17 para mitigar el ruido electromagnético.*

| Componente | Pin Físico (Pico 2) | ID del Pin | Tipo de Señal | Función |
| :--- | :--- | :--- | :--- | :--- |
| Servo MG996R | Pin 25 | `GP19` | Salida PWM | Control de ángulo de dirección (Giro) |
| Driver BTS7960 (R_PWM) | Pin 34 | `GP28` | Salida PWM | Control de velocidad hacia adelante (Forward) |
| Driver BTS7960 (L_PWM) | Pin 32 | `GP27` | Salida PWM | Control de velocidad en reversa (Backward) |
| Driver BTS7960 (R_EN) | Pin 31 | `GP26` | Salida Digital | Habilitación de rama derecha (Activa con High) |
| Driver BTS7960 (L_EN) | Pin 29 | `GP22` | Salida Digital | Habilitación de rama izquierda (Activa con High) |
| MPU6050 (SDA) | Pin 21 | `GP16` | I2C0 SDA (Pull-Up) | Bus de Datos de la IMU |
| MPU6050 (SCL) | Pin 22 | `GP17` | I2C0 SCL (Pull-Up) | Bus de Reloj de la IMU |

### Conexiones de la Raspberry Pi 3B (El Estratega)
* **Cámara Arducam 12MP:** Conectada a la interfaz nativa CSI de la placa mediante un cable plano flex de 15 pines para transmisión de video de alta velocidad.
* **RPLIDAR C1:** Conectado a un puerto USB de la Raspberry Pi 3B capaz de proveer la corriente de arranque del motor de escaneo y sostener la transferencia binaria a 460,800 baudios.
* **Raspberry Pi Pico 2:** Interconectada mediante la interfaz USB nativa de la Pico hacia un puerto USB de la Pi 3B. Actúa como canal UART bidireccional estable operando en modo VCP (Virtual COM Port) a 115,200 bps.
* **Pulsador (Botón Start):** Conectado al pin `GPIO17` (Pin físico 11) configurado con una resistencia Pull-Down externa de 10kOhm hacia tierra para disparar la rutina autónoma en pista.

---

## Registro Fotográfico Reglamentario (v-photos)

En cumplimiento con las normas oficiales de inspección técnica de la World Robot Olympiad, se presentan los 6 ángulos visuales obligatorios del vehículo autónomo perfectamente alineados:

| Vista Frontal | Vista Trasera |
| :---: | :---: |
| <img src="v-photos/Frontview.jpeg" width="380px" alt="Vista Frontal"/> | <img src="v-photos/Backview.jpeg" width="380px" alt="Vista Trasera"/> |
| **Vista Superior** | **Vista Inferior (Chasis)** |
| <img src="v-photos/Topview.jpeg" width="380px" alt="Vista Superior"/> | <img src="v-photos/butview.jpeg" width="380px" alt="Vista Inferior"/> |
| **Vista Lateral Izquierda** | **Vista Lateral Derecha** |
| <img src="v-photos/leftview.jpeg" width="380px" alt="Vista Lateral Izquierda"/> | <img src="v-photos/Rightview.jpeg" width="380px" alt="Vista Lateral Derecha"/> |

---

## Anatomía del Repositorio

Cumpliendo rigurosamente las regulaciones internacionales de la WRO, organizamos la estructura de archivos de forma modular, limpia y totalmente auditable:

```text
├── src/                      # Código fuente de la arquitectura distribuida
│   ├── pico/                 # Firmware embebido en MicroPython (Raspberry Pi Pico 2)
│   │   ├── main.py           # Bucle principal de control e interrupciones en tiempo real
│   │   ├── motores.py        # Abstracción PWM y habilitaciones para servo e inversión con BTS7960
│   │   ├── test_motores.py   # Script crudo de calibración de ángulos del actuador servo
│   │   ├── imu.py            # Inicialización de registros I2C0 e integración del ángulo Yaw (Z)
│   │   └── serial_receiver.py# Parser serial no bloqueante para adquisición de zonas del Lidar
│   └── pi3b/                 # Scripts en Python 3 de alto nivel (Raspberry Pi 3B)
│       ├── main_control.py   # Orquestador central de navegación, procesamiento visual de la Arducam y estrategia
│       ├── lidar_handler.py  # Driver de adquisición binaria directa y filtrado de mediana para RPLIDAR C1
│       └── serial_sender.py  # Módulo emisor de telemetría espacial compactada hacia la Pico 2
├── 3d-Models/                # Modelos de piezas mecánicas en formato STL/PNG y su sub-manual
│   ├── README.md             # Manual técnico exclusivo y documentación visual de piezas en 3D
│   ├── Chasis.stl            # Modelado de la estructura central
│   ├── Chasis.png            # Renderizado del chasis
│   └── [Resto de archivos mecánicos .stl y .png...]
├── t-photos/                 # Registro fotográfico oficial de las jornadas de desarrollo del equipo
│   └── Photo_Team.jpeg       # Foto oficial de los integrantes
├── v-photos/                 # Las 6 capturas reglamentarias obligatorias del coche desde los ángulos rectores
├── video/                    # Contiene el archivo video.md con el hipervínculo a la mejor vuelta en pista
├── schemes/                  # Planos esquemáticos, diagramas de flujo y mapas eléctricos
└── README.md                 # Documentación técnica principal (este archivo)

```
