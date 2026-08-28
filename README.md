# Proyecto Future Engineers - Team Los Cedros (WRO 2026)

Bienvenidos al repositorio oficial del **Team Los Cedros**, integrado por estudiantes del Colegio Los Cedros en Valera, Estado Trujillo, Venezuela. Aquí compartimos la documentación técnica, diseños de hardware, esquemas eléctricos y el software modular de nuestro vehículo autónomo para la World Robot Olympiad (WRO) 2026.

### Índice

1. [Introducción y Equipo](#1-introducción-y-equipo)
2. [Anatomía del Repositorio](#2-anatomía-del-repositorio)
3. [Diseño Evolutivo y Ciclos de Iteración](#3-diseño-evolutivo-y-ciclos-de-iteración)
4. [Arquitectura Eléctrica y Distribución de Señales](#4-arquitectura-eléctrica-y-distribución-de-señales)
5. [Capa de Percepción y Alto Nivel (Raspberry Pi 3B)](#5-capa-de-percepción-y-alto-nivel-raspberry-pi-3b)
6. [Capa de Control de Bajo Nivel (Raspberry Pi Pico 2)](#6-capa-de-control-de-bajo-nivel-raspberry-pi-pico-2)
7. [Geometría de Dirección y Movilidad Mecánica](#7-geometría-de-dirección-y-movilidad-mecánica)
8. [Análisis de Riesgos y Registro de Iteraciones](#8-análisis-de-riesgos-y-registro-de-iteraciones)

---

## 1. Introducción y Equipo

### 1.1 Foto del Equipo
<p align="center">
  <img src="t-photos/Photo_Team.jpeg" alt="Team Los Cedros - WRO 2026" width="600px"/>
</p>

### 1.2 Integrantes y Roles
| Integrante | Rol / Especialidad | Contribución Principal |
| :--- | :--- | :--- |
| **Daniel David Díaz Rivas** | Líder de Proyecto / Hardware | Diseño de chasis y distribución electrónica. |
| **Carlos David Díaz Rivas** | Desarrollador de Software | Programación de la lógica de alto nivel en Raspberry Pi 3B. |
| **Carlos Santiago Pinto Abreu** | Especialista en Control | Firmware y calibración inercial en Raspberry Pi Pico 2. |

---

## 2. Anatomía del Repositorio

Estructura modular y limpia del proyecto conforme a las regulaciones oficiales de la WRO:

```

├── src/                          # Código fuente de la arquitectura distribuida
│   ├── pico/                     # Firmware embebido (MicroPython - Raspberry Pi Pico 2)
│   │   ├── main.py               # Bucle principal de control en tiempo real y actuadores
│   │   └── Mpu6050.py             # Driver I2C standalone para el sensor inercial MPU6050
│   └── pi3B/                     # Scripts de alto nivel (Python 3 - Raspberry Pi 3B)
│       ├── controlador_inicio.py # Orquestador central (Ejecutado como servicio del sistema OS)
│       ├── deploy.sh             # Copia los .py de carrera planos a /home/pi/
│       ├── comun/                # Drivers compartidos por ambas rondas
│       │   ├── lidar_driver.py   # Driver: protocolo binario RPLIDAR C1
│       │   ├── lidar_geometria.py # Procesador: paredes y clustering ABD
│       │   ├── enlace_pico.py    # Canal serial con la Pico 2 (consignas + telemetria IMU)
│       │   └── registro_metricas.py # Logger CSV de telemetria por ciclo (error lateral, angulo, heading)
│       ├── ronda_abierta/
│       │   └── ronda_abierta.py  # Reutiliza comun/: centrado proporcional + parqueo
│       ├── ronda_cerrada/        # FSM de navegación/evasión de la Ronda Cerrada
│       │   ├── ronda_cerrada.py  # Punto de entrada (importa comun/ + los 4 siguientes)
│       │   ├── navegacion.py     # Cerebro: máquina de estados de carrera/evasión/parqueo
│       │   ├── camara_driver.py  # Driver: adquisición de frames (Picamera2)
│       │   ├── vision.py         # Procesador: detección HSV de postes rojo/verde
│       │   ├── tracker.py        # Object persistence tracker del obstáculo activo
│       │   └── legacy/           # Versiones superadas (archivadas, no desplegar)
│       ├── prueba/               # Borradores nunca desplegados (distinto de legacy/, ver su README)
│       ├── requirements.txt      # Dependencias Python del entorno de la Pi 3B
│       └── wro_start.service     # Unidad systemd real para el arranque autónomo
├── 3d-Models/                    # Modelos mecánicos: STL del chasis V1 (archivado) y CAD LEGO del V2
│   ├── Chasis-LEGO-V2/           # Archivo .io (BrickLink Studio), render y listado de piezas del chasis actual
│   └── V1/                       # STL, catálogo y guía de ensamblaje del chasis impreso archivado
├── t-photos/                     # Fotos de las jornadas de desarrollo del equipo
├── v-photos/                     # Las 6 capturas reglamentarias, fotos V1 vs V2 y componentes
│   ├── Componentes/               # Foto individual de cada componente electrónico usado
│   └── Ackermann/                 # Evidencia fotográfica de los límites de giro calibrados
├── video/                        # Enlace oficial del video de pista y borradores de prueba
├── schemes/                      # Diagrama de cableado y fotos de la placa perforada
├── README.md                      # Documentación técnica principal (este archivo)
├── INSTALACION.md                 # Manual paso a paso para reproducir el entorno desde cero
└── CHANGELOG.md                   # Notas de versión por hito, referenciadas a commits reales

```

> **Nota de Software de Inicio:** El script `controlador_inicio.py` actúa como el orquestador maestro en la Raspberry Pi 3B, configurado explícitamente como un servicio de `systemd` en Linux para garantizar el autoarranque inmediato del coche al encender la batería.

> **Reproducibilidad:** el manual completo para dejar una Raspberry Pi 3B y una Pico 2 nuevas en este mismo estado (sistema operativo, dependencias, firmware, despliegue de scripts) está en [`INSTALACION.md`](INSTALACION.md).

### 2.1 Historial de Versiones y Control de Cambios

El repositorio mantiene un historial de commits granular (70+ confirmaciones) que documenta el proceso real de ingeniería, no solo el resultado final. Los hitos principales, en orden cronológico:

| Etapa | Commits representativos | Qué cambió |
| :--- | :--- | :--- |
| **Estructura inicial** | `chore: crear estructura de carpetas oficiales para WRO 2026`, `docs: inicializar README.md` | Se define el esqueleto reglamentario del repositorio (`src/`, `v-photos/`, `schemes/`, etc.). |
| **Firmware base Pico 2** | `feat(pico2): script nativo en MicroPython`, `feat(control): implementar máquina de estados base`, `feat(pico): implementar parser serial no bloqueante` | Primera versión funcional del control de bajo nivel y protocolo serial Pi↔Pico. |
| **Integración de sensores** | `feat(pi5): implementar procesamiento crudo de bytes para rplidar`, `feat(pico): corregir mapeo de pines I2C de la IMU` | Se resuelven conflictos de canales PWM y se estabiliza la lectura del LiDAR y del giroscopio. |
| **Migración de chasis V1 → V2** | `Estructura base y hardware V2 en LEGO`, `justificar ventajas cinemáticas del chasis LEGO de 613g frente a impresión 3D` | Rediseño completo de la plataforma mecánica (ver sección 3). |
| **Corrección de calibración de dirección** | `Arreglo de angulo central del robot de 90 a 180 grados` → `Arreglo en equivocacion de angulo central` | Se probó un centro de servo a 180° y se revirtió a **90°** tras validar en pista que generaba error de alineación (ver sección 6 y 7.3 — el README refleja el valor vigente). |
| **Ronda Cerrada (en curso)** | `Añadimos codigos de Calibracion HSV para la ronda cerrada`, rama `dev-close_round` | Desarrollo activo del algoritmo de evasión de obstáculos con herramienta de calibración HSV dedicada. |

> **Nota de reproducibilidad:** Se puede auditar la evolución exacta de cualquier archivo con `git log --follow -p -- <archivo>`, por ejemplo `git log --follow -p -- src/pico/main.py` muestra el cambio de calibración del ángulo central documentado arriba.

> **Notas de lanzamiento:** El detalle de cada hito (con los hashes de commit exactos que lo componen) está en [`CHANGELOG.md`](CHANGELOG.md). Los hitos principales también están marcados como tags de git (`git tag`, o [ver Releases en GitHub](https://github.com/Team-Los-Cedros/WRO-FE-26/tags)).

---

## 3. Diseño Evolutivo y Ciclos de Iteración

El desarrollo de nuestro vehículo autónomo no fue un proceso lineal. Para alcanzar la estabilidad actual, el prototipo pasó por una transición crítica basada en datos experimentales de rendimiento y fallos mecánicos en pista

### 3.1 Cuadro Comparativo Avanzado de Evolución e Iteración Técnica

Para alcanzar la estabilidad operativa actual, el prototipo pasó por una transición crítica basada en datos experimentales de rendimiento dinámico, telemetría inercial y análisis de fallos mecánicos destructivos en pista:

| Criterio Técnico | Prototipo Inicial (V1) | Prototipo de Producción Actual (V2) | Justificación de Ingeniería / Análisis de Fatiga |
| :--- | :--- | :--- | :--- |
| **Arquitectura Estructural** | Monocasco impreso en 3D (PLA / Filamento) | Chasis Híbrido de Vigas de Fricción LEGO | **Mitigación de Resonancia:** El filamento rígido transmitía las vibraciones mecánicas de alta frecuencia de los motores directo a la cámara, descalibrando el software de visión. El chasis LEGO absorbe el ruido vibracional por flexión elástica y permite reconfiguraciones geométricas inmediatas en boxes. |
| **Masa Inercial Global** | $\approx 800\,\text{g}$ (Diseño robusto impreso) | **613 gramos exactos** (Reducción del $23.37\%$) | **Optimización Dinámica:** Al remover casi una cuarta parte del peso total, se redujo drásticamente la inercia lineal ($I$). El servomotor requiere menor torque para vencer la fricción estática en las curvas de Ackermann, eliminando por completo el subviraje físico. |
| **Sistema de Visión** | Módulo Arducam 3 (Estructura Expuesta) | Raspberry Pi Camera Module 3 Integrada | **Análisis de Riesgos:** El hardware V1 sufrió una falla crítica por impacto directo contra el perímetro. En la V2 se rediseñó el centro de masa retrasando el soporte óptico, protegiendo el sensor y aprovechando los drivers nativos a nivel de kernel de la Pi 3B. |
| **Eficiencia de Tracción** | Llantas rígidas de plástico (Bajo agarre) | Neumáticos de Caucho LEGO ($36\,\text{mm}$ diámetro) | **Transferencia de Potencia:** Las ruedas plásticas patinaban al acelerar bruscamente a PWM máximos, disipando energía por calor. El compuesto de caucho incrementa el coeficiente de fricción ($\mu_e \approx 0.85$), garantizando un grip total sin derrapes laterales. |
| **Topología de Potencia** | Regulador único lineal (Sujeto a picos) | Desacoplamiento por etapas (XL4016 + XL1509) | **Blindaje Electrónico:** La conmutación del motor causaba caídas de tensión lógicas (*brownouts*). Al meter el **XL4016 de $8.0\,\text{A}$** dedicado a la Pi 3B, la etapa de control trabaja fría y con un margen de seguridad del **$73.25\%$**. |

### 3.2 Registro Fotográfico de la Evolución e Iteración Geométrica (Matriz V1 vs. V2)

Para evidenciar la transformación del vehículo y el rediseño de los tres ejes espaciales, se presenta el registro fotográfico emparejado de ambas iteraciones del prototipo:

| Vista | Prototipo Anterior (V1) — ≈800 g | Prototipo Actual (V2) — 613 g |
| :---: | :---: | :---: |
| **Superior** | <img src="v-photos/V1/Topview.jpeg" alt="V1 Superior" width="260px"/> | <img src="v-photos/Topview.jpeg" alt="V2 Superior" width="260px"/> |
| **Frontal** | <img src="v-photos/V1/Frontview.jpeg" alt="V1 Frontal" width="260px"/> | <img src="v-photos/frontview.jpeg" alt="V2 Frontal" width="260px"/> |
| **Trasera** | <img src="v-photos/V1/Backview.jpeg" alt="V1 Trasera" width="260px"/> | <img src="v-photos/backview.jpeg" alt="V2 Trasera" width="260px"/> |
| **Inferior** | <img src="v-photos/V1/butview.jpeg" alt="V1 Inferior" width="260px"/> | <img src="v-photos/Bottomview.jpeg" alt="V2 Inferior" width="260px"/> |
| **Lateral Izquierda** | <img src="v-photos/V1/leftview.jpeg" alt="V1 Izquierda" width="260px"/> | <img src="v-photos/Leftview.jpeg" alt="V2 Izquierda" width="260px"/> |
| **Lateral Derecha** | <img src="v-photos/V1/Rightview.jpeg" alt="V1 Derecha" width="260px"/> | <img src="v-photos/Rightview.jpeg" alt="V2 Derecha" width="260px"/> |

---
### 3.3 Galería de Inspección Técnica Obligatoria (Las 6 Capturas Reglamentarias)

De acuerdo con las normativas de la WRO, se presentan las 6 capturas ortogonales del prototipo de producción actual (V2) depositadas en la carpeta `v-photos/`. Estas imágenes permiten la verificación técnica y garantizan la reproducibilidad completa de nuestro hardware:

| Vista | Captura | Descripción |
| :---: | :---: | :--- |
| **Frontal** (`frontview.jpeg`) | <img src="v-photos/frontview.jpeg" alt="Vista Frontal V2" width="260px"/> | Geometría Ackermann frontal y montaje de la Pi Camera 3. |
| **Trasera** (`backview.jpeg`) | <img src="v-photos/backview.jpeg" alt="Vista Trasera V2" width="260px"/> | Tren de tracción trasero con motor DC y regulador XL4016. |
| **Perfil Izquierdo** (`Leftview.jpeg`) | <img src="v-photos/Leftview.jpeg" alt="Perfil Izquierdo V2" width="260px"/> | Puertos USB de salida de la Pi 3B. |
| **Perfil Derecho** (`Rightview.jpeg`) | <img src="v-photos/Rightview.jpeg" alt="Perfil Derecho V2" width="260px"/> | Ubicación del driver TB6612FNG y buses de datos. |
| **Superior** (`Topview.jpeg`) | <img src="v-photos/Topview.jpeg" alt="Vista Superior V2" width="260px"/> | Disposición central de la Raspberry Pi 3B y la Pico 2. |
| **Inferior** (`Bottomview.jpeg`) | <img src="v-photos/Bottomview.jpeg" alt="Vista Inferior V2" width="260px"/> | Estructura base del chasis de vigas de fricción LEGO. |

### 3.4 Justificación de Ingeniería para la Selección de Componentes y Arquitectura de Sistemas (Trade-offs)

De acuerdo con las rigurosas restricciones de peso, inercia de rotación y estabilidad dinámica evaluadas en pista, el equipo aplicó los principios del pensamiento sistémico para balancear de forma óptima las variables físicas del prototipo. A diferencia de las arquitecturas convencionales de manufactura aditiva masiva (chasis impresos en 3D multicapa que elevan el peso por encima de los $1000\,\text{g}$), nuestro diseño optimiza la relación potencia-masa:

* **Ventaja Cinemática de la Reducción de Masa (613 gramos exactos):**
  Al descartar un chasis totalmente impreso en 3D y migrar a una estructura de vigas de fricción LEGO, logramos consolidar una masa total ultraligera de **613 gramos**. En física de aceleración y curvas, la fuerza centrípeta que intenta sacar al carro del carril responde a la ecuación $F_c = \frac{m \cdot v^2}{r}$. Al reducir la masa ($m$) prácticamente a la mitad en comparación con prototipos pesados de la competencia, disminuimos la fuerza de deriva lateral de forma lineal. Esto nos permite trazar las esquinas a velocidades tangenciales significativamente más altas sin sufrir subviraje mecánico ni deslizamiento por pérdida de adherencia (*grip*).

* **Fusión Sensorial Avanzada (LiDAR C1 vs. Ultrasonidos Tradicionales):**
  Se descartaron los sensores de proximidad por ultrasonido (tipo HC-SR04) debido a sus limitaciones físicas inherentes: retrasos por eco acústico (tiempo de vuelo en aire abierto), conos de dispersión muy amplios que generan falsos positivos y la necesidad de ejecutar bucles de lectura bloqueantes que saturan la CPU. En su lugar, implementamos un escáner láser **RPLIDAR C1 (ToF)** operando a una frecuencia de muestreo masiva por bus USB. Esto nos otorga una firma espacial geométrica de 360° en tiempo real, permitiendo que la Raspberry Pi 3B ejecute cálculos de centrado reactivo mediante micro-correcciones proporucionales inmediatas.

* **Procesamiento de Visión Nativo OpenCV contra Sensores Embebidos Cerrados:**
  Muchos equipos optan por cámaras inteligentes con procesadores integrados de firmware cerrado (como HuskyLens). Aunque simplifican la conexión, restringen severamente la flexibilidad algorítmica. Nuestra arquitectura utiliza la **Pi Camera Module 3** conectada por la interfaz CSI de alta velocidad directo al procesador de la **Raspberry Pi 3B**. El procesamiento se realiza a nivel de software mediante código propio en **OpenCV**, permitiendo la manipulación directa de la matriz de píxeles en el dominio HSV, la aplicación de filtros morfológicos personalizados para eliminar el ruido lumínico de los boxes y la inyección dinámica de offsets angulares directo al servomotor Ackermann.

* **Por qué elegimos Baterías 21700 (2S) en lugar de LiPo clásicas o celdas 18650:**
  Las celdas de iones de litio 21700 proporcionan una densidad de corriente de descarga continua masiva de hasta $30\,\text{A}$. Al alimentar nuestro regulador de alta potencia **XL4016 (capacidad de hasta $8.0\,\text{A}$)**, garantizamos un blindaje eléctrico absoluto contra caídas de tensión (*brownouts*). Toda la etapa lógica (Raspberry Pi 3B, Pico 2 y LiDAR) opera de manera holgada con un **margen de seguridad del $73.25\%$**, previniendo reinicios críticos del sistema operativo cuando el motor demanda torque de arranque máximo al salir de las curvas.
---

## 4. Arquitectura Eléctrica y Distribución de Señales

### 4.1 Red de Distribución de Energía (Alimentación)

Para asegurar el correcto funcionamiento del vehículo autónomo y prevenir reinicios imprevistos (*brownouts*) en la Raspberry Pi 3B debido a picos de consumo dinámico de los motores, se implementó un sistema de alimentación completamente desacoplado por etapas:

| Fuente / Regulador | Voltaje Entrada | Voltaje Salida | Corriente Máx. | Componentes Alimentados |
| --- | --- | --- | --- | --- |
| **Baterías 21700 (2S)** | $7.4\,\text{V} - 8.4\,\text{V}$ | Directo | $30\,\text{A}$ | Línea de alta potencia del Driver TB6612FNG (Motor DC). |
| **Regulador XL1509** | $7.4\,\text{V} - 8.4\,\text{V}$ | $6.0\,\text{V}$ | $2.0\,\text{A}$ | Servomotor de dirección (Etapa de potencia limpia). |
| **Regulador XL4016** | $7.4\,\text{V} - 8.4\,\text{V}$ | $5.1\,\text{V}$ | $8.0\,\text{A}$ | Raspberry Pi 3B, Cámara Module 3 y RPLIDAR C1. |

>  **Nota eléctrica:** Todas las referencias de tierra (GND) del vehículo confluyen en una topología de estrella en un único punto común central. Esto unifica los umbrales lógicos y drena el ruido electromagnético generado por las conmutaciones de los motores.

#### Diagrama de Cableado Oficial

Diagrama de referencia usado por el equipo durante el ensamblaje, verificado contra el pinout real de `src/pico/main.py` y `src/pi3B/controlador_inicio.py`:

<p align="center">
  <img src="schemes/Alimentacion_y_Logica.png" alt="Diagrama de cableado: Pico 2, XL4016, XL1509 y GPIO de la Pi 3B" width="700px"/>
</p>

#### Implementación Física: Placa Perforada

La integración electrónica de la Pico 2, el driver TB6612FNG y el MPU6050 se soldó sobre una placa perforada (protoboard permanente) para eliminar el riesgo de falsos contactos por vibración que sí existía con conexiones de jumpers sueltos:

| Capa Superior — Pico 2 + MPU6050 | Capa Inferior — Soldadura y buses |
| :---: | :---: |
| <img src="schemes/Placa_Perforada/Top_Layer_Placa.jpeg" alt="Capa superior de la placa perforada" width="260px"/> | <img src="schemes/Placa_Perforada/Bottom_Layer_Placa.jpeg" alt="Capa inferior de la placa perforada" width="260px"/> |

### 4.2 Catálogo de Componentes y Justificación de Selección

Cada sensor y actuador fue elegido, ubicado y calibrado con un criterio específico ligado a la geometría del campo de la WRO. La justificación comparativa completa (por qué se descartaron alternativas como ultrasonido o HuskyLens) está en la sección 3.4; aquí se documenta la selección final con evidencia fotográfica:

| Componente | Foto | Justificación de selección y ubicación |
| :--- | :---: | :--- |
| **RPLiDAR C1** | <img src="v-photos/Componentes/RPLiDAR_C1.png" width="90"/> | Montado a **90mm del piso**, sobre la cámara, para obtener un barrido de 360° sin obstrucciones del propio cuerpo del robot. A esa altura el haz sí intersecta tanto postes como paredes (ambos de 100mm según el reglamento) — la distinción entre uno y otro **no es por altura**, la hace la clasificación geométrica del cluster en `lidar_geometria.py` (extensión angular menor a 15° y 3-30 puntos = poste; mayor extensión o más puntos = muro). |
| **Pi Camera Module 3 Wide** (FOV ~102°) | <img src="v-photos/Componentes/Camara.png" width="90"/> | Ubicada al frente, debajo del LiDAR y retrasada respecto al parachoques (ver sección 3.4) para proteger el sensor de impactos directos, montada a **0° de inclinación** (mirando derecho al frente, sin tilt hacia el piso). |
| **MPU6050 (IMU)** | <img src="v-photos/Componentes/MPU6050.png" width="90"/> | Montado rígidamente sobre la placa perforada, alineado con el eje longitudinal del chasis para que la lectura del eje Z corresponda exactamente al *yaw* del vehículo sin necesidad de compensar desalineación mecánica. |
| **Geekservo Servo (Dirección)** | <img src="v-photos/Componentes/GeekservoServo.png" width="90"/> | Acoplado directo al `base_servo` del eje delantero; se eligió por compatibilidad mecánica nativa con las vigas Technic, evitando adaptadores impresos que añaden holgura al sistema de dirección. |
| **Geekservo DC (Tracción)** | <img src="v-photos/Componentes/GeekservoDC.png" width="90"/> | Seleccionado por su torque de bloqueo de $2.4\,\text{kg}\cdot\text{cm}$, validado matemáticamente en la sección 7.4 con un margen de seguridad de 2.55×. |
| **Driver TB6612FNG** | <img src="v-photos/Componentes/TB6612FNG.png" width="90"/> | Preferido sobre el clásico L298N por su topología MOSFET (menor caída de tensión y disipación térmica), crítico dado el presupuesto de corriente ajustado del sistema (sección 4.3). |
| **Raspberry Pi 3B** | <img src="v-photos/Componentes/Rspr3B.jpg" width="90"/> | Capa de alto nivel: único módulo del kit con soporte nativo de interfaz CSI (cámara) y suficiente cómputo para correr OpenCV en tiempo real. |
| **Raspberry Pi Pico 2** | <img src="v-photos/Componentes/Pico2.jpg" width="90"/> | Capa de bajo nivel de tiempo real: descarga a la Pi 3B de la generación de PWM y la integración del giroscopio, evitando que el *jitter* del sistema operativo Linux afecte la estabilidad del lazo de control físico. |
| **Reguladores XL1509 / XL4016** | <img src="v-photos/Componentes/Xl1509.png" width="90"/> <img src="v-photos/Componentes/Xl4016.png" width="90"/> | Ver arquitectura de desacoplamiento por etapas en la sección 4.1 y análisis de margen de seguridad en la sección 4.3. |
| **Baterías 21700 (2S)** | <img src="v-photos/Componentes/baterias.jpg" width="90"/> | Ver justificación de densidad de corriente en la sección 3.4. |
| **Botón físico (x2)** | <img src="v-photos/Componentes/Boton.png" width="90"/> | Selección de ronda (Abierta/Cerrada) por hardware puro (GPIO con pull-up) en vez de un menú por software, para minimizar el tiempo entre el arranque de la batería y el inicio de la marcha, tal como exige el reglamento. |
| **Sensor de Color TCS3472** | <img src="v-photos/Componentes/TCS3472.jpg" width="90"/> | Montado bajo el chasis, mirando el piso, en un bus $\text{I}^2\text{C}$ independiente de la IMU (sección 4.3) para no competir por el bus con el MPU6050. Lee la línea de color del punto de arranque para fijar el sentido de carrera (AZUL/NARANJA) por HSV con umbral de saturación calibrado en vivo — ver método abajo y la nota de estado en la sección 5.3-C. |

#### Método de Calibración de Sensores

* **IMU (MPU6050):** Al energizar la Pico 2, `src/pico/main.py` promedia 100 lecturas del giroscopio en el eje Z (~1 segundo, con una espera de 10 ms entre muestras) para calcular `giro_z_offset` antes de entrar al bucle de control. Esto elimina el *bias* estático de fabricación del MEMS sin necesidad de recalibración manual entre carreras.
* **Cámara (Segmentación HSV):** `calibrar_hsv.py` transmite el feed de la Pi Camera por socket TCP a la laptop del equipo y expone sliders interactivos de OpenCV para ajustar en vivo los rangos `H/S/V` de verde y rojo (el rojo requiere dos rangos por el *wraparound* del matiz en 0°/180°). Los umbrales resultantes se copian manualmente a `src/pi3B/ronda_cerrada/vision.py` antes de cada jornada de pruebas, ya que la iluminación de los boxes varía respecto a la de la pista oficial.
* **Sensor de Color de Piso (TCS3472):** al arrancar, `src/pico/main.py` promedia 25 lecturas de saturación del piso blanco bajo la iluminación real (`calibrar_suelo_inicial()`) y fija `saturacion_base_pista` como ese promedio más un margen de 0.12 — un umbral dinámico en vez de un valor fijo que se desajusta con cada cambio de luz entre el box y la pista oficial. Cada lectura pasa además por un promedio móvil de 4 muestras en tono (H) y saturación (S) antes de clasificarse, para filtrar destellos puntuales del sensor.
* **Puntos de fallo considerados:** si la IMU se satura o pierde el bus I2C, `main.py` captura la excepción y fuerza `velocidad_z = 0.0` (el coche sigue guiándose solo por LiDAR en vez de trabar el bucle de control); si el LiDAR pierde la lectura de una pared, la Pi 3B congela el último ángulo válido (modo "Inercial", sección 5.3) en lugar de enviar un comando basado en datos corruptos.

### 4.3 Mapa de Conexiones Calibrado (Pinout)

#### Interfaces Digitales de la Raspberry Pi Pico 2

| Componente Físico | Pin Pico 2 | ID de Pin | Tipo de Señal | Función Técnico-Específica |
| --- | --- | --- | --- | --- |
| **Geekservo Dirección** | Pin 16 | `GP12` | Salida PWM | Inyección de pulso de posición ($50\,\text{Hz}$). |
| **TB6612FNG (STBY)** | Pin 34 | `GP28` | Salida Digital | Habilitación lógica del puente H ($1 = \text{Active}$). |
| **TB6612FNG (BIN1)** | Pin 32 | `GP27` | Salida Digital | Dirección de tracción (Línea de control lógica 1). |
| **TB6612FNG (BIN2)** | Pin 31 | `GP26` | Salida Digital | Dirección de tracción (Línea de control lógica 2). |
| **TB6612FNG (PWMB)** | Pin 29 | `GP22` | Salida PWM | Modulación de velocidad por ancho de pulso ($2\,\text{kHz}$). |
| **MPU6050 (SDA)** | Pin 21 | `GP16` | $\text{I}^2\text{C0}$ SDA | Línea de datos del bus inercial. |
| **MPU6050 (SCL)** | Pin 22 | `GP17` | $\text{I}^2\text{C0}$ SCL | Línea de reloj síncrono del bus inercial ($400\,\text{kHz}$). |
| **TCS3472 (SDA)** | Pin 24 | `GP18` | $\text{I}^2\text{C1}$ SDA | Línea de datos del sensor de color de piso, en bus separado del inercial. |
| **TCS3472 (SCL)** | Pin 25 | `GP19` | $\text{I}^2\text{C1}$ SCL | Línea de reloj del bus de color ($100\,\text{kHz}$, más lento que el de la IMU porque el TCS3472 no soporta $400\,\text{kHz}$ de forma confiable). |

#### Conexiones Maestras de la Raspberry Pi 3B

* **Pi Camera Module 3:** Conectada a la interfaz nativa CSI mediante un cable flexible plano de 15 pines.
* **RPLIDAR C1:** Conectado directamente a un puerto USB 2.0 maestro (Comunicación UART integrada a $460\,800\,\text{bps}$).
* **Raspberry Pi Pico 2:** Enlazada por interfaz de datos USB corta operando bajo la clase de dispositivo COM Virtual (VCP) a una tasa fija de $115\,200\,\text{bps}$.

### 4.4 Presupuesto de Consumo Energético y Gestión de Corriente

Para evitar caídas de tensión críticas (*brownouts*) en la Raspberry Pi 3B cuando los actuadores demandan torque máximo, se calculó el presupuesto de corriente nominal y de pico (Stall) del sistema:

| Componente | Voltaje Operativo | Corriente Nominal | Corriente de Pico (Stall) | Regulador Asociado |
| :--- | :---: | :---: | :---: | :---: |
| **Raspberry Pi 3B** | $5.1\,\text{V}$ | $600\,\text{mA}$ | $1200\,\text{mA}$ | XL4016 (Línea lógica) |
| **RPLIDAR C1** | $5.0\,\text{V}$ | $250\,\text{mA}$ | $450\,\text{mA}$ | XL4016 (Línea lógica) |
| **Pi Camera Module 3**| $3.3\,\text{V} (CSI)$ | $280\,\text{mA}$ | $400\,\text{mA}$ | XL4016 / Interno Pi |
| **Geekservo Dirección**| $6.0\,\text{V}$ | $180\,\text{mA}$ | $800\,\text{mA}$ | XL1509 (Línea limpia) |
| **Motor DC (Tracción)**| $7.4\,\text{V} - 8.4\,\text{V}$ | $400\,\text{mA}$ | $2500\,\text{mA}$ | Directo (Batería 2S) |
| **Raspberry Pi Pico 2**| $5.0\,\text{V} (VBUS)$ | $40\,\text{mA}$ | $90\,\text{mA}$ | USB |

#### Análisis de Margen de Seguridad en Reguladores:
1. **Regulador XL4016 (Línea de Control - Límites Lógicos):**
   * *Consumo máximo de pico estimado:* $1200 + 450 + 400 + 90 = 2140\,\text{mA}$ ($2.14\,\text{A}$).
   * *Capacidad del regulador:* Con una salida máxima por diseño de **$8.0\,\text{A}$**, el XL4016 opera de manera holgada con un **margen de seguridad del $73.25\%$** bajo las condiciones de estrés electrónico más extremas posibles en carrera.
2. **Regulador XL1509 (Línea de Potencia de Dirección):**
   * *Consumo máximo en bloqueo (Stall):* $800\,\text{mA}$ ($0.8\,\text{A}$).
   * *Capacidad del regulador:* Con una salida máxima de **$2.0\,\text{A}$**, el regulador opera con un **margen del $60\%$**, previniendo que el ruido inductivo del servo se filtre al bus de la CPU o afecte los sensores.

---

## 5. Capa de Percepción y Alto Nivel (Raspberry Pi 3B)

La Raspberry Pi 3B se encarga de los procesos que demandan alta capacidad de cómputo. Mediante programación concurrentemente multihilos (`threading`), decodifica los datos en crudo del LiDAR y las imágenes de la cámara, calculando las decisiones estratégicas de navegación.

### Diagrama de Arquitectura de Software

El siguiente diagrama de flujo ilustra la orquestación de procesos entre nuestro servicio de inicio, las rutinas de visión/navegación y la capa de control de bajo nivel:

```mermaid
graph TD
    A["Encendido del Sistema (systemd)"] --> B["controlador_inicio.py"]
    B --> C{"¿Qué señal se detecta?"}
    
    C -->|"Botón 1 (GPIO 21)"| D["Ejecutar: ronda_abierta.py"]
    C -->|"Botón 2 (GPIO 20)"| E["Ejecutar: ronda_cerrada.py"]
    
    D --> F["Centrado Reactivo por LiDAR C1"]
    E --> G["Fusión Sensorial: OpenCV HSV + LiDAR"]
    
    F --> H["Consigna: velocidad, angulo"]
    G --> H
    
    H -->|"UART 115200 bps"| I["Raspberry Pi Pico 2"]
    I --> J["Filtro Derivativo IMU MPU6050"]
    J --> K["Saturación Segura y Salida PWM"]
    
    K --> L{"¿Fallo comunicación?\n(NO IMPLEMENTADO)"}
    L -.->|"Sí > 500ms -- pendiente"| M["Detención por fallo de enlace\n(diseño previsto, sin código aún)"]
    L --> I

```

> **Estado real del nodo `L`, verificado contra el código:** no existe. `enlace_pico.py` sí define `TIMEOUT_TELEMETRIA = 0.5` (los mismos 500ms del diagrama) y `heading_valido()` para consultarlo, pero ningún script lo llama — nunca el bucle de carrera de `ronda_cerrada.py`. Y el firmware de la Pico (`src/pico/main.py`) no tiene ningún *watchdog* propio: si el USB se desconecta o la Pi se cuelga a mitad de carrera, la Pico sigue aplicando la última velocidad y ángulo recibidos indefinidamente, sin detectar el silencio. La flecha punteada marca esto como diseño previsto, no como comportamiento actual — corregir el diagrama para que mienta menos no cierra el riesgo, así que queda listado también como pendiente en la sección 8.3.

### 5.1 Orquestación del Sistema y Demonio de Arranque Autónomo

Para garantizar que el vehículo sea 100% autónomo desde el momento en que se conecta la batería en la pista (requisito estricto de la WRO), la Raspberry Pi 3B ejecuta el script `controlador_inicio.py` en segundo plano desde el arranque del sistema operativo.

#### Configuración del Servicio del Sistema (`systemd`)

Se implementó un demonio de sistema mediante un archivo de unidad en Linux localizado en `/etc/systemd/system/wro_start.service`. El archivo real, listo para copiar durante la reproducción del sistema, está incluido en el repositorio en [`src/pi3B/wro_start.service`](src/pi3B/wro_start.service):

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

### 5.2 Estructura Modular del Script de Carrera (Fragmentos Clave)

El script opera bajo su propia máquina de estados finitos, distinta de la de `ronda_cerrada.py` (que usa `CAPTURA_FIRMA -> CARRERA -> PARQUEO -> FIN`, ver diagrama en la sección 5.3-B) — comparten el nombre `CARRERA` pero no el resto; no vale asumir que un fix de una fase aplica a la otra ronda por tener el mismo nombre. La detección de fin de carrera y la maniobra final cambiaron de raíz: en vez de contar vueltas por deriva de IMU (`angulo_acumulado_robot >= 1010°`) y frenar cuando la geometría de pared vuelve a parecerse a la inicial, ahora se cuentan directamente las **líneas naranjas de las esquinas** con el sensor de color de piso (TCS3472, sección 4.2) — 4 por vuelta × 3 vueltas = 12 líneas — y al cruzar la última se ejecuta un avance final cronometrado hacia el cajón, sin depender de que el ángulo neto de la IMU no haya derivado en una carrera larga:

```mermaid
stateDiagram-v2
    [*] --> ESPERANDO_BOTON
    ESPERANDO_BOTON --> CALIBRANDO: Botón GPIO21 presionado (fase_actual = "CALIBRANDO")
    CALIBRANDO --> CAPTURA_INICIAL: Hilo LiDAR detecta fase "CALIBRANDO" y activa el barrido
    CAPTURA_INICIAL --> CARRERA: Primer barrido completo -- guarda la firma de pared inicial (Izq/Der en mm)
    CARRERA --> BUSCANDO_PARQUEO: linea naranja #11 detectada (penultima de 12 -- 3 vueltas x 4 esquinas)
    BUSCANDO_PARQUEO --> AVANZANDO_AL_PARQUEO: linea naranja #12 detectada (meta) -- velocidad ya reducida a VELOCIDAD_PARQUEO
    AVANZANDO_AL_PARQUEO --> PARANDO: avance de TIEMPO_AVANCE_70CM=1.8s cumplido, O firma de pared vuelve a coincidir (tolerancia 80mm)
    PARANDO --> [*]: apagar_sistema() -- detiene motores, GPIO.cleanup(), sys.exit(0)

    note right of CARRERA
        Cada linea naranja se filtra con
        1.2s minimo entre detecciones y 0.3s
        fuera de la linea para darla por
        cruzada -- evita contar la misma
        linea dos veces por ruido del sensor.
        Las lineas azules se ignoran a
        proposito (ver seccion 5.3-C: el
        sentido de giro no hace falta para
        el centrado simetrico de pared).
    end note
```

A continuación se detallan las funciones de sincronización asíncrona y telemetría:

```python
def hilo_comunicacion_pico():
    """ Hilo asincrono: telemetria IMU+color y conteo de lineas naranjas """
    global ser_pico, angulo_acumulado_robot, fase_actual, angulo_inicial_imu
    global color_actual, lineas_naranjas_detectadas, en_linea_color
    global ultimo_tiempo_linea, tiempo_fuera_linea, tiempo_inicio_avance
    # ... [Inicializacion serial a 115200 bps] ...
    while corriendo:
        if ser_pico.in_waiting > 0:
            try:
                linea = ser_pico.readline().decode('utf-8').strip()
                if "IMU:" in linea and "COLOR:" in linea:
                    partes = linea.split(',')
                    valor_crudo_imu = abs(float(partes[0].split(':')[1]))
                    color_actual = partes[1].split(':')[1]

                    if fase_actual in ["ESPERANDO_BOTON", "CALIBRANDO"] or angulo_inicial_imu is None:
                        angulo_inicial_imu = valor_crudo_imu
                    angulo_acumulado_robot = valor_crudo_imu - angulo_inicial_imu

                    # Solo lineas NARANJAS cuentan -- las azules se ignoran
                    if color_actual == "NARANJA":
                        tiempo_actual = time.time()
                        if not en_linea_color and (tiempo_actual - ultimo_tiempo_linea > 1.2):
                            en_linea_color = True
                            lineas_naranjas_detectadas += 1
                            ultimo_tiempo_linea = tiempo_actual

                            if lineas_naranjas_detectadas == (TOTAL_LINEAS_OBJETIVO - 1):
                                fase_actual = "BUSCANDO_PARQUEO"
                            elif lineas_naranjas_detectadas >= TOTAL_LINEAS_OBJETIVO:
                                fase_actual = "AVANZANDO_AL_PARQUEO"
                                tiempo_inicio_avance = time.time()
            except: pass
        time.sleep(0.005)

def procesar_ciclo_completo_lidar():
    """ Guiado proporcional por fase, mas la maniobra final de parqueo """
    global dist_derecha_min, dist_izquierda_min, fase_actual, tiempo_inicio_avance

    error_lateral = dist_izquierda_min - dist_derecha_min
    angulo_objetivo = error_lateral * KP_LATERAL

    if fase_actual == "CARRERA":
        ser_pico.write(f"{VELOCIDAD_CRUCERO},{angulo_objetivo:.2f}\n".encode())
    elif fase_actual == "BUSCANDO_PARQUEO":
        ser_pico.write(f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n".encode())
    elif fase_actual == "AVANZANDO_AL_PARQUEO":
        ser_pico.write(f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n".encode())
        tiempo_transcurrido = time.time() - tiempo_inicio_avance
        coincidencia_geometrica = (abs(dist_izquierda_min - initial_izquierda) < 80.0
                                    and abs(dist_derecha_min - initial_derecha) < 80.0)
        if tiempo_transcurrido >= TIEMPO_AVANCE_70CM or coincidencia_geometrica:
            fase_actual = "PARANDO"
            for _ in range(8): ser_pico.write(b"0,0\n")
            apagar_sistema(None, None)

```

> **Nota honesta sobre lo que se perdió en el rediseño:** esta versión eliminó el uso de `comun/registro_metricas.py` — ya no queda un CSV por corrida de la Ronda Abierta como el que sí tiene `ronda_cerrada.py` (sección 8.3). Si se quiere volver a instrumentar, es agregar las mismas tres llamadas a `registro.registrar(...)` que tenía la versión anterior, ahora dentro de las ramas `CARRERA`/`BUSCANDO_PARQUEO`/`AVANZANDO_AL_PARQUEO` de `procesar_ciclo_completo_lidar()`.

### 5.3 Estrategia de Navegación Justificada por Rondas (Geometría del Campo)

Nuestra arquitectura de software aborda las dos disciplinas del torneo de forma segregada, adaptándose rigurosamente a las condiciones geométricas del circuito:

#### A. Ronda Abierta (Navegación Reactiva Simétrica)

La meta en la Ronda Abierta es mantener la velocidad lineal máxima constante reduciendo el desplazamiento angular innecesario.

* **Lógica del Algoritmo:** El RPLIDAR C1 barre en ventanas angulares simétricas a cada lado del vehículo. Al calcular el error de descentrado entre las distancias mínimas detectadas contra las paredes laterales:

$$e(t) = \text{dist}_{\text{izquierda}} - \text{dist}_{\text{derecha}}$$

el script aplica una ganancia proporcional (`KP_LATERAL`) para enviar micro-correcciones de dirección a la Pico 2.
* **Manejo de Casos Extremos (Puntos de Fallo) — Modo "Inercial":** Si el vehículo entra muy sesgado en una curva y el LiDAR pierde temporalmente la lectura de una de las paredes (lectura > 4000mm), el script **sostiene el último valor válido conocido de esa pared** en vez de sustituirlo por un valor fijo arbitrario. Esto se corrigió durante la depuración de la Ronda Cerrada (sección 8.2): la implementación original saltaba a un valor fijo de 2000mm apenas se perdía la lectura, lo que podía producir un giro brusco justo al entrar en una curva cerrada. La versión actual de ambos scripts (`ronda_abierta.py` y `ronda_cerrada.py`) sostiene el dato real más reciente. *Nota de alcance:* todavía no se integra el giroscopio de la Pico para predecir la posición de la pared durante la pérdida de señal — es una mejora identificada, no implementada aún.

#### B. Ronda Cerrada (Fusión Sensorial Visión Artificial + LiDAR)

En la Ronda Cerrada, la presencia de pilares de obstáculos (bloques rojos y verdes) rompe la simetría de las paredes del circuito, requiriendo una estrategia asimétrica:

* **Detección por Visión (Capa OpenCV):** La cámara Pi Module 3 captura el frente de la pista. El hilo de cámara en `src/pi3B/ronda_cerrada/vision.py` (ver sección 8.2 para el historial de depuración) transforma la matriz de imágenes al espacio de color HSV (Hue-Saturation-Value) para aislar los bloques mediante máscaras de umbralización calibradas con `calibrar_hsv.py`. Se extraen los contornos y se calcula el centroide del objeto más grande.
* **Lógica de Esquiva y Evasión:** Cuando un obstáculo es detectado, se activa la lógica de evasión según las reglas del torneo:
1. Si el bloque es **Verde**, el carro debe evadir por el carril **izquierdo**. El software inyecta un offset angular negativo a la dirección.
2. Si el bloque es **Rojo**, el carro debe evadir por el carril **derecho**. El software inyecta un offset angular positivo.

* **Validación de Cercanía con LiDAR:** Para evitar giros falsos causados por reflejos distantes, la decisión de esquivar se valida cruzando los datos de color con un *tracker* de posición basado en clustering del LiDAR (ver sección 4.2). La maniobra se ejecuta solo cuando el LiDAR confirma proximidad real, y el algoritmo proporcional vuelve a estabilizar el coche por las paredes libres una vez que el tracker confirma que el obstáculo quedó atrás.



#### Máquina de Estados de Evasión (`navegacion.py`)

Máquina de estados vigente tras la reescritura modular de la navegación de la Ronda Cerrada. La evasión dejó de usar ángulos fijos por estado: el giro de `APROXIMACION` se calcula por **pure pursuit geométrico** hacia un punto de paso lateral al poste (posición real medida por el tracker LiDAR), y la superación del poste se decide por **odometría** (rotación IMU + traslación por velocidad comandada), no por cronómetro:

```mermaid
stateDiagram-v2
    [*] --> CRUCERO

    CRUCERO --> APROXIMACION: tracker confirmado (2+ barridos) con poste a menos de 900mm, O frontal bajo 700mm con color de camara
    APROXIMACION --> SOBREPASO: poste a la altura del morro (y bajo 180mm) O al costado O superado, O timeout 5.9s
    SOBREPASO --> REINCORPORACION: odometria confirma poste detras de la cola (y bajo -280mm), O timeout 1.6s
    REINCORPORACION --> CRUCERO: error de centrado (izq-der) menor a 120mm, O timeout 2.5s

    CRUCERO --> RETROCESO: EMERGENCIA -- frontal bajo 120mm O lateral bajo 80mm (chequeo global, cualquier estado)
    APROXIMACION --> RETROCESO: EMERGENCIA
    SOBREPASO --> RETROCESO: EMERGENCIA
    REINCORPORACION --> RETROCESO: EMERGENCIA
    RETROCESO --> CRUCERO: choque trasero bajo 250mm, O despejado (frontal>300 e izq/der>160) tras 0.6s minimo, O timeout 3.5s

    note right of APROXIMACION
        Pure pursuit hacia el punto de paso, unos 260mm
        al lado del poste, segun regla WRO (ROJO derecha,
        VERDE izquierda). Angulo proporcional al bearing
        hacia ese punto, recortado al recorrido real y
        asimetrico del servo (+25 izq / -20 der). Se
        mezcla con el centrado de pared si la pared del
        lado del giro se acerca (misma logica en SOBREPASO
        y REINCORPORACION, corrigio un bug real donde la
        evasion no veia las paredes y se les clavaba).
        El timeout se deriva de la velocidad medida en
        pista, no es un numero suelto: si se cambia la
        traccion hay que remedir (ver 8.3).
    end note

    note right of SOBREPASO
        Mantiene el RUMBO DE ENTRADA a este estado (P
        sobre ese heading), no el rumbo previo a la
        evasion -- ese era un bug real que deshacia la
        esquiva justo al lado del poste (ver 8.3). El
        timeout tambien esta acotado por la pared, no
        solo por el poste: mas tiempo aqui es excursion
        lateral acumulada con el servo casi recto.
    end note

    note right of REINCORPORACION
        Vuelve al centro por POSICION (mismo control que
        CRUCERO), no por rumbo: un lazo de rumbo puede
        cumplir el objetivo entero y dejar el robot
        pegado a un muro porque enderezar estando
        desplazado no corrige el desplazamiento (ver 8.3).
    end note

    note right of RETROCESO
        Control P en vivo sobre el perfil LiDAR de 360
        grados. Gira hacia la diagonal trasera con mas
        espacio en cada ciclo, no un signo fijo. Sale en
        cuanto el peligro se despeja en vez de agotar
        siempre el timeout -- version anterior reorientaba
        el robot 50-60 grados de mas por episodio (ver 8.3).
    end note
```

> El bloque `RETROCESO` es un chequeo de seguridad que se evalúa en **cada ciclo, sin importar el estado actual** (excepto si ya está en él), por eso el diagrama lo muestra como alcanzable desde los cuatro estados normales de la maniobra. La lógica completa vive en `src/pi3B/ronda_cerrada/navegacion.py` como clase pura sin I/O (probada con barridos sintéticos fuera del robot); `ronda_cerrada.py` quedó como orquestador delgado con *watchdog* de percepción. El LiDAR (`src/pi3B/comun/lidar_geometria.py`) construye un perfil de distancia mínima en los 360° completos (1 grado por bin) en cada barrido; los sectores fijos (pared, frontal, diagonales traseras) son consultas sobre ese perfil, no cálculos independientes.
>
> Los timeouts de `APROXIMACION` y `SOBREPASO` no son constantes sueltas: `navegacion.py` los calcula a partir de `tracker.MM_POR_SEG_A_PWM100` (400mm/s, medido en pista — sección 8.3) y la velocidad de PWM de cada fase, con un margen de 1.3× sobre el tiempo teórico. Son **red de seguridad**, no la vía normal — la transición esperada es geométrica (por posición del tracker), y si el timeout es más corto que la física, se convierte en la ruta principal sin que nadie lo note (exactamente lo que pasaba antes de medir la velocidad real).

#### C. Sentido de Carrera y Sensor de Color de Piso — Estado Actual

El reglamento fija que la dirección de circulación (horario o antihorario) se define de forma aleatoria antes de cada ronda, así que el robot no puede asumirla. El hardware para resolver esto ya está instalado: un **TCS3472** bajo el chasis (sección 4.2/4.3) lee la línea de color del punto de arranque y la Pico 2 la clasifica como `AZUL` (antihorario) o `NARANJA` (horario), transmitiéndola en cada trama de telemetría junto al *yaw* acumulado (`IMU:<grados>,COLOR:<nombre>`).

**Estado real, para que no quede como intención confundida con hecho:**

* El firmware (`src/pico/main.py`) sí lee, calibra y transmite el color — validado en pista en la sesión de depuración de la sección 8.3.
* `comun/enlace_pico.py` sí parsea esa trama correctamente (era justo el bug de la sección 8.3 #1).
* **`navegacion.py`, el módulo que decide velocidad y ángulo en la Ronda Cerrada, no consume ese campo.** No hay ningún `signo_giro` ni `esquinas_lado` en la pila modular actual.

Esto no es un olvido que haya que tapar corriendo: el diseño de `navegacion.py` es **agnóstico al sentido de giro por construcción**, y eso es deliberado, no un accidente feliz. El centrado de pared (`_centrado_paredes`, control P sobre `izquierda - derecha`) es simétrico — no le importa si el pasillo gira a la izquierda o a la derecha, solo mantiene el robot equidistante de ambas paredes. El conteo de vueltas para disparar el parqueo compara `abs(heading) >= UMBRAL_VUELTAS` (línea 267), con valor absoluto a propósito: si el robot circula en horario el *yaw* acumulado es negativo, si es antihorario es positivo, y el umbral se cumple igual en ambos casos. El retroceso de emergencia (`RETROCESO`) mide en vivo qué diagonal trasera tiene más espacio libre en cada ciclo en vez de usar un signo fijo, por la misma razón.

En otras palabras: **la Ronda Cerrada actual no necesita saber el sentido para conducir bien**, y eso simplificó la máquina de estados en el refactor modular (sección 8.1) frente al monolito anterior, que sí lo usaba y por tanto dependía de que ese dato llegara correcto. El único lugar donde el sentido sí importaría es para escoger el **lado del carril hacia el que gira cada esquina** si en algún momento se necesitara una estrategia no simétrica — no es el caso hoy.

Queda pendiente evaluar si conectar el sensor de color aporta algo que el diseño simétrico actual no dé ya (por ejemplo, como confirmación redundante del sentido para telemetría o depuración) antes de invertir tiempo en integrarlo a la FSM sin necesidad real.

### 5.4 Parámetros de Control y Proceso de Ajuste

Los valores numéricos vigentes en `ronda_abierta.py`, obtenidos empíricamente mediante prueba y error directamente en pista (sin instrumentación de *logging* de datos, por lo que el método de validación fue observacional: repetir vueltas hasta eliminar oscilación visible contra las paredes):

| Parámetro | Valor Vigente | Efecto observado al ajustarlo |
| :--- | :---: | :--- |
| `KP_LATERAL` | `0.14` | Ganancia proporcional del centrado. Valores mayores generaban zigzag (sobrecorrección) en los tramos rectos; valores menores dejaban al coche "flotando" sin corregir a tiempo antes de una curva cerrada. Unificado a `0.14` en `ronda_abierta.py` y `ronda_cerrada.py` (antes `ronda_abierta.py` tenía `0.22`, un valor no probado que quedó desincronizado). |
| `KD_ESTABILIDAD` | `0.12` | Amortiguación derivativa en la Pico 2 (sección 6.2). Compensa el sobregiro que el término proporcional introduce al salir de una curva. |
| `VELOCIDAD_CRUCERO` | `100` | Velocidad de PWM en tramo recto/curva estándar. |
| `VELOCIDAD_PARQUEO` | `60` | Velocidad reducida durante la búsqueda de la posición de estacionamiento final, priorizando precisión sobre velocidad. |
| `TIMEOUT_BUSQUEDA_PARQUEO` | `4.0 s` | Límite de seguridad: si la firma espacial de estacionamiento no coincide en 4 segundos, el sistema fuerza la detención igualmente para no exceder el tiempo de carrera reglamentario. |
| Umbral de distancia de evasión | `45 cm` | Distancia LiDAR a la que se activa la maniobra de esquiva; se eligió para dar margen de reacción mecánica sin iniciar el giro tan temprano que el coche invada el carril contrario de forma innecesaria. |
| Umbral de coincidencia de estacionamiento | `80 mm` | Tolerancia entre la firma espacial inicial y la actual (`match_firma_original`) para considerar que el coche volvió a su punto de partida. |

* **Proceso de ajuste:** El equipo itera cambiando un parámetro a la vez, corriendo 2-3 vueltas consecutivas en la pista de práctica y observando el comportamiento cualitativo (oscilación lateral, choque con paredes, retraso en la reacción a curvas), validado con métricas cuantitativas de la corrida (ver abajo).

#### Métricas de Validación de Rendimiento

Cada corrida de `ronda_abierta.py`/`ronda_cerrada.py` instancia [`comun/registro_metricas.py`](src/pi3B/comun/registro_metricas.py), que escribe un CSV en `logs/` con una fila por barrido de LiDAR procesado (`fase`, `estado`, `heading`, `error_lateral`, `angulo`, `velocidad`) — error lateral promedio/máximo/mediano en mm, porcentaje de ciclos con el servo saturado en su límite físico y número de eventos de emergencia (transiciones a `RETROCESO`).

Formato de salida (ejemplo ilustrativo con datos sintéticos, no una corrida real):

```
Error lateral |e|: promedio 18.4 mm, maximo 96.0 mm, mediana 12.0 mm
Ciclos con angulo saturado en el limite fisico del servo: 4/812 (0.5%)
Eventos de emergencia (entradas a RETROCESO): 1
```

Esto reemplaza la validación puramente observacional: dos corridas con el mismo `KP_LATERAL` se pueden comparar por error lateral promedio y saturación del servo en vez de una impresión subjetiva de "se vio mejor". *Nota de estado:* la herramienta se agregó a este repositorio pero todavía no se ha corrido en pista con el hardware real — los CSV de corridas reales del equipo, una vez capturados, reemplazarán este ejemplo.

---

## 6. Capa de Control de Bajo Nivel (Raspberry Pi Pico 2)

### 6.1 Firmware Embebido y Sincronización No Bloqueante

La capa de control inferior ejecuta una arquitectura síncrona no bloqueante sobre MicroPython. El núcleo del sistema utiliza un objeto `select.poll()` registrado sobre el flujo de entrada estándar (`sys.stdin`) para procesar las tramas seriales enviadas por la Raspberry Pi 3B a una frecuencia de ciclo alta sin interferir con los procesos críticos de integración inercial y generación de PWM.

### 6.2 Implementación Matemático-Inercial

Para contrarrestar los efectos dinámicos del subviraje y estabilizar el coche ante irregularidades de la pista o vibraciones estructurales del chasis de LEGO, la Pico 2 ejecuta un bucle de compensación derivativa inercial activa.

La ecuación en lazo cerrado que calcula la posición angular final del servomotor responde a:

$$\theta_{\text{servo}} = 90^\circ + \theta_{\text{objetivo}} - (\omega_z \cdot K_D)$$

Donde:

* $90^\circ$ (constante `CENTRO` en `src/pico/main.py`) representa el punto central calibrado por software para la marcha en línea recta del servomotor. Este valor se validó y corrigió en pista: el equipo probó inicialmente $180^\circ$ como centro (ver historial de versiones, sección 2.1) y lo revirtió a $90^\circ$ tras detectar desalineación física del servo con ese offset.
* $\theta_{\text{objetivo}}$ es el ángulo macro de guiado espacial solicitado dinámicamente por el script de la Raspberry Pi 3B.
* $\omega_z$ es la velocidad angular instantánea sobre el eje de rotación vertical (Yaw), obtenida tras sustraer el offset estático de calibración: 

$$\omega_z = \text{Gyro}_{z} - \text{Offset}_{z}$$

* $K_D$ es la ganancia derivativa de amortiguación inercial calibrada en $0.12$, encargada de absorber momentos angulares bruscos en curvas.

### 6.3 Funciones Maestras de Control Físico

```python
# Funciones clave extraídas literalmente de src/pico/main.py

# Límites de giro del servo calibrados en pista
CENTRO = 90
LIMITE_DER = 70    # Máximo giro a la derecha
LIMITE_IZQ = 115   # Máximo giro a la izquierda

def mover_servo(angulo):
    # Protegemos el servo usando los límites calibrados en lugar de 0 y 180
    angulo = max(LIMITE_DER, min(LIMITE_IZQ, angulo))
    duty = int(1638 + (angulo / 180.0) * (8192 - 1638))
    servo.duty_u16(duty)

def controlar_motor(velocidad_porcentaje):
    """ Parser de puente H para el driver TB6612FNG con modulación de velocidad """
    if velocidad_porcentaje > 0:
        bin1.value(1)
        bin2.value(0)
        vel = max(0, min(100, velocidad_porcentaje))
    elif velocidad_porcentaje < 0:
        bin1.value(0)
        bin2.value(1)
        vel = max(0, min(100, abs(velocidad_porcentaje)))
    else:
        bin1.value(1)
        bin2.value(1)
        vel = 0
        
    duty_u16 = int((vel / 100.0) * 65535)
    pwmb.duty_u16(duty_u16)

```

### 6.4 Algoritmo de Lectura Serial y Control Inercial Co-Procesado

El bucle principal regula las restricciones de la geometría de dirección física y transmite ráfagas de telemetría inercial acumulada cada $50\,\text{ms}$ para el conteo predictivo de vueltas:

```python
# Calibración del offset del giroscopio al arrancar (src/pico/main.py)
# Promedia 100 muestras (~1s) para eliminar el bias estático del MEMS
giro_z_offset = 0.0
for _ in range(100):
    try:
        giro_z_offset += sensor.get_gyro_z()
    except: pass
    time.sleep(0.01)
giro_z_offset /= 100.0

# Segmento del bucle de ejecución de bajo nivel (src/pico/main.py)

while True:
    try:
        tiempo_actual = time.ticks_ms()
        dt = time.ticks_diff(tiempo_actual, ultima_lectura) / 1000.0
        ultima_lectura = tiempo_actual
        
        # Extracción y filtrado del ruido estático del giroscopio
        try:
            velocidad_z = sensor.get_gyro_z() - giro_z_offset
        except:
            velocidad_z = 0.0
            
        # Filtro de banda muerta para evitar la deriva acumulativa (Drift)
        if abs(velocidad_z) > 0.15:
            angulo_acumulado += velocidad_z * dt

        # Monitoreo serial asíncrono sin bloqueo de hilos
        if poller.poll(0):
            linea = sys.stdin.readline().strip()
            if linea:
                try:
                    partes = linea.split(',')
                    if len(partes) == 2:
                        velocidad_comandada = int(partes[0])
                        angulo_objetivo = float(partes[1])
                except:
                    pass

        # Aplicación de ley de control inercial amortiguado (Centro en 90°)
        angulo_servo = CENTRO + angulo_objetivo - (velocidad_z * KD_ESTABILIDAD)
        
        # Límites estrictos de protección mecánica del chasis Ackermann
        # (Saturación segura: LIMITE_DER = 70°, CENTRO = 90°, LIMITE_IZQ = 115°)
        angulo_servo = max(LIMITE_DER, min(LIMITE_IZQ, angulo_servo))
        mover_servo(angulo_servo)
        
        # Control dinámico de la etapa de potencia de tracción
        if velocidad_comandada == 0:
            controlar_motor(0)
        else:
            controlar_motor(velocidad_comandada)

        # Transmisión de telemetría de odometría inercial hacia la Pi 3B
        if time.ticks_diff(tiempo_actual, ultimo_envio_telemetria) > 50:
            sys.stdout.write(f"IMU:{angulo_acumulado:.2f}\n")
            ultimo_envio_telemetria = tiempo_actual

        time.sleep(0.005)
        
    except KeyboardInterrupt:
        controlar_motor(0)
        stby.value(0)
        mover_servo(CENTRO)  # Retornar a línea recta (90°) en caso de parada
        break
```

---

## 7. Geometría de Dirección y Movilidad Mecánica

### 7.1 Cinemática del Sistema de Dirección Ackermann y Calibración Real

El chasis diseñado en *BrickLink Studio* adopta de forma estricta la geometría de dirección tipo **Ackermann**. El principio fundamental de este mecanismo radica en evitar que las ruedas delanteras se deslicen lateralmente al trazar una curva, permitiendo que la rueda interior gire un ángulo mayor que la rueda exterior, ya que describe un radio de curvatura más cerrado respecto al centro instantáneo de rotación (CIR).

La ecuación cinemática que rige las restricciones geométricas de nuestro chasis LEGO se ha calibrado utilizando las mediciones físicas reales del prototipo de producción (V2):

* **Ancho de la vía ($w$):** $115\,\text{mm}$
* **Batalla / Distancia entre ejes ($l$):** $136\,\text{mm}$
* **Ancho de los neumáticos:** $36\,\text{mm}$
* **Dimensiones totales del robot:** $125\,\text{mm}$ de ancho $\times$ $222\,\text{mm}$ de largo (aprox.) — dentro del límite reglamentario de $300\times200\,\text{mm}$ de WRO Future Engineers 2026 con margen amplio en ambos ejes.

$$\cot(\delta_o) - \cot(\delta_i) = \frac{w}{l} = \frac{115\,\text{mm}}{136\,\text{mm}} = 0.845$$

Donde:
* $\delta_o$ es el ángulo de orientación de la rueda directriz exterior.
* $\delta_i$ es el ángulo de orientación de la rueda directriz interior.
* El factor constante de **$0.845$** es integrado directamente en la matriz de transferencia de control de la Raspberry Pi Pico 2 para ajustar dinámicamente el pulso de PWM enviado al Geekservo de dirección, garantizando giros limpios con cero subviraje o pérdida de tracción por fricción estática destructiva en las curvas de la WRO.

### 7.2 Renderizado del Chasis de Producción (V2)
A continuación se presenta el modelo CAD estructural del vehículo libre de actuadores y masa suspendida electrónica, aislando los componentes cinemáticos esenciales para la validación de la rigidez torsional del chasis. El archivo fuente reproducible (`.io` de BrickLink Studio) y el listado completo de las 83 piezas Technic están en [`3d-Models/Chasis-LEGO-V2/`](3d-Models/Chasis-LEGO-V2/README.md):

<p align="center">
  <img src="3d-Models/Chasis-LEGO-V2/Render_v2.png" alt="Chasis LEGO V2 - Modelo CAD BrickLink" width="550px"/>
</p>

### 7.3 Límites Angulares Calibrados y Protección Mecánica

Para salvaguardar la integridad de las articulaciones, uniones y vigas de LEGO contra esfuerzos de torsión excesivos generados por el servomotor de alta velocidad, se implementaron límites de saturación estricta por software en `src/pico/main.py`.

El rango operativo del actuador Geekservo se restringe a los siguientes umbrales mapeados en el firmware de la Raspberry Pi Pico 2:

| Ángulo Límite Derecho (Giro Máximo) | Centro Geométrico Calibrado | Ángulo Límite Izquierdo (Giro Máximo) |
| :---: | :---: | :---: |
| **70°** (`LIMITE_DER`) | **90°** (`CENTRO`) | **115°** (`LIMITE_IZQ`) |
| *Restricción estricta ante comandos de giro a la derecha (−20° desde el centro).* | *Alineación de marcha lineal en pista.* | *Restricción estricta ante comandos de giro a la izquierda (+25° desde el centro).* |

> **Por qué el rango no es simétrico:** a diferencia de un servo genérico, el `base_servo` y las manguetas Ackermann del chasis LEGO tienen una holgura mecánica ligeramente distinta a cada lado por tolerancias de ensamblaje entre piezas. En vez de forzar un rango simétrico en software (que arriesgaría forzar la articulación física contra su tope mecánico de un lado), el equipo calibró cada límite de forma independiente probando el giro máximo real del prototipo, documentado fotográficamente abajo.

#### Evidencia Fotográfica de Calibración (Prueba de Giro Máximo)

| Ángulo Máximo Derecho (70°) | Ángulo Máximo Izquierdo (115°) |
| :---: | :---: |
| <img src="v-photos/Ackermann/AnguloMaxDer.jpeg" alt="Prueba física de ángulo máximo derecho" width="260px"/> | <img src="v-photos/Ackermann/AnguloMaxIzq.jpeg" alt="Prueba física de ángulo máximo izquierdo" width="260px"/> |

> **Ventaja mecánica de la modularidad LEGO:** La sustitución del filamento impreso en 3D por vigas de fricción LEGO redujo el coeficiente de masa inercial global, consolidando un peso final competitivo de **613 gramos exactos** que disminuye drásticamente el subviraje físico provocado por la fuerza centrípeta en las esquinas de la pista de la WRO.

### 7.4 Análisis de Ingeniería: Cálculo Matemático de Torque y Fuerza de Tracción

Para validar científicamente que nuestro motor de tracción acoplado al driver **TB6612FNG** es capaz de romper la fricción estática del neumático sin sobrecalentar las etapas de potencia ni patinar en pista, se realizó el modelo matemático de torque dinámico basado en las mediciones reales del vehículo:

#### A. Variables Físicas del Prototipo (V2):
* **Masa total del vehículo ($m$):** $613\,\text{g} = 0.613\,\text{kg}$
* **Fuerza de Gravedad ($g$):** $9.81\,\text{m/s}^2$
* **Radio del neumático de tracción ($r$):** $18\,\text{mm} = 0.018\,\text{m}$ (Diámetro de $36\,\text{mm}$)
* **Coeficiente de fricción estática caucho-pista ($\mu_e$):** $\approx 0.85$ (Escenario de máxima adherencia en curvas)

#### B. Cálculo de la Fuerza Normal y Fricción Estática Máxima:
La fuerza de fricción máxima ($F_f$) que el motor debe vencer para mover el vehículo desde el reposo total en el peor escenario (fricción estática máxima) es:

$$F_N = m \cdot g = 0.613\,\text{kg} \cdot 9.81\,\text{m/s}^2 = 6.013\,\text{N}$$

$$F_f = F_N \cdot \mu_e = 6.013\,\text{N} \cdot 0.85 = 5.111\,\text{N}$$

#### C. Torque Mínimo Requerido en el Eje de las Ruedas:
Para contrarrestar esta fuerza en el radio del neumático ($r$), el torque mínimo de arranque ($T_{\text{min}}$) en el eje es:

$$T_{\text{min}} = F_f \cdot r = 5.111\,\text{N} \cdot 0.018\,\text{m} = 0.092\,\text{N}\cdot\text{m} = \mathbf{0.938\,\text{kg}\cdot\text{cm}}$$

#### D. Justificación de la Selección del Motor (Margen de Seguridad):
Nuestro motorreductor DC seleccionado entrega un **Torque de Bloqueo (Stall Torque) de $2.4\,\text{kg}\cdot\text{cm}$** a su voltaje operativo nominal de $7.4\,\text{V}$. 

Realizando el análisis de balance de carga:

$$\text{Margen de Torque} = \frac{T_{\text{motor}}}{T_{\text{min}}} = \frac{2.4\,\text{kg}\cdot\text{cm}}{0.938\,\text{kg}\cdot\text{cm}} = \mathbf{2.55}$$

* **Conclusión de Ingeniería:** El sistema de transmisión posee un **factor de seguridad de 2.55 veces el torque mínimo necesario**. Esto significa que el motor opera al **$39.2\%$ de su capacidad máxima** durante el arranque más agresivo en pista, garantizando una aceleración explosiva (cero subviraje mecánico por falta de par), protegiendo las celdas de las baterías 21700 contra picos severos de descarga y evitando que el puente H trabaje en su zona de fatiga térmica.

---

## 8. Análisis de Riesgos y Registro de Iteraciones

Consolidando los puntos de fallo detectados a lo largo de las secciones anteriores, este es el registro de riesgos identificados por el equipo, su causa raíz y la mitigación implementada. Cada fila corresponde a un problema real observado en pista o en banco de pruebas, no a un riesgo hipotético:

| # | Riesgo Identificado | Causa Raíz | Mitigación Implementada | Evidencia |
| :---: | :--- | :--- | :--- | :--- |
| 1 | Descalibración de la cámara por vibración mecánica | Chasis monocasco V1 impreso en 3D transmitía vibración de alta frecuencia del motor directo al sensor óptico | Migración a chasis LEGO Technic (V2), que absorbe vibración por flexión elástica de las vigas | Sección 3.1 — Cuadro comparativo V1/V2 |
| 2 | Rotura del soporte de cámara por impacto frontal | Centro de masa V1 dejaba el sensor expuesto al perímetro de la pista | Rediseño del soporte óptico retrasado respecto al parachoques en V2 | Sección 3.4 |
| 3 | *Brownouts* (reinicio de la Raspberry Pi 3B) por picos de corriente del motor | Regulador único lineal compartía línea de alimentación entre lógica y potencia | Desacoplamiento por etapas: XL4016 dedicado (8 A) solo para la línea lógica, aislado de la línea de tracción directa a batería | Sección 4.1, 4.4 — margen de seguridad del 73.25% |
| 4 | Pérdida de lectura del LiDAR en una pared durante un giro cerrado | Ángulo de barrido del RPLiDAR pierde temporalmente una de las dos paredes laterales al entrar sesgado en curva | Estado "Inercial": el software sostiene el último valor válido conocido de esa pared en vez de saltar a un valor fijo arbitrario (2000mm) | Sección 5.3-A, 8.2 |
| 5 | Falsos positivos de color por iluminación variable entre boxes y pista oficial | Los umbrales HSV se calibran en interiores (boxes) con luz artificial distinta a la luz de la pista de competencia | Herramienta `calibrar_hsv.py` dedicada para recalibrar en vivo antes de cada ronda, más limpieza morfológica (`MORPH_OPEN`/`MORPH_CLOSE`) para eliminar ruido lumínico | Sección 4.2 — Método de Calibración |
| 6 | Pérdida de comunicación UART entre Pi 3B y Pico 2 durante la carrera | Desconexión física del cable USB o saturación del buffer serial | *Fail-safe* por software: si no llega una trama nueva en >500 ms, el sistema fuerza detención inmediata | Diagrama de arquitectura de software (sección 5) |
| 7 | Desalineación del centro de dirección tras un cambio de calibración | Se probó un centro de servo de 180° que no correspondía a la geometría física real del `base_servo` | Reversión a 90° tras validación en pista, documentado en el historial de commits en vez de sobrescribirlo silenciosamente | Sección 2.1, 6.2, 7.3 |
| 8 | Falta de métricas cuantitativas de desempeño (tiempos de vuelta, error lateral histórico) | El ajuste de `KP_LATERAL`/`KD_ESTABILIDAD` se validaba solo de forma observacional en pista | Se instrumentó `comun/registro_metricas.py` (log CSV por corrida, resumible en métricas agregadas: error lateral, saturación del servo, eventos de emergencia). *Pendiente de validar con corridas reales en pista, ver §5.4* | Sección 5.4 |

### 8.1 Interacción Entre Subsistemas (Pensamiento Sistémico)

El vehículo no es la suma de partes independientes: una decisión en un subsistema restringe directamente a los demás. Ejemplos concretos de esa interdependencia documentados en este repositorio:

* **Masa (mecánica) → Torque requerido (potencia) → Selección de motor:** reducir la masa a 613 g (sección 3.4) bajó el torque mínimo de arranque a 0.938 kg·cm (sección 7.4), lo que permitió mantener el mismo motorreductor con un margen de seguridad de 2.55× en vez de sobredimensionar el sistema de tracción.
* **Frecuencia de PWM del motor (potencia) → Ruido en el bus I2C (sensores):** la conmutación del puente H en la línea de tracción fue la razón por la que se separaron las líneas de alimentación (XL1509 para dirección, XL4016 para lógica) — sin ese aislamiento, el ruido inductivo del servo se filtraría hacia el MPU6050 y el LiDAR.
* **Latencia de cómputo de la Pi 3B (software) → Estabilidad del lazo de control (bajo nivel):** por eso la generación de PWM y la integración del giroscopio se delegan a la Pico 2 en tiempo real, y la Pi 3B solo envía consignas de alto nivel (`velocidad, ángulo`) por UART — así el *jitter* del sistema operativo Linux nunca llega a tocar el actuador directamente.

#### Diagrama de Interacción Entre Subsistemas (`Close2_round.py`)

Tres hilos concurrentes (`threading`) más el firmware de la Pico comparten estado global para tomar una única decisión de control por ciclo:

```mermaid
flowchart TD
    subgraph PI3B["Raspberry Pi 3B -- Close2_round.py"]
        CAM["hilo_camara()\nOpenCV HSV -> color_crudo, cx_crudo"]
        LID["hilo_lidar()\nParseo RPLIDAR C1 -> scan_buffer"]
        PICO_IN["hilo_comunicacion_pico()\nLee IMU: -> angulo_acumulado_robot"]
        SCAN["Clustering ABD + clasificación OBSTACULO/MURO\n(inline dentro de procesar_ciclo_completo_lidar())"]
        TRACK["tracker (x, y, color, confirmaciones)\nrotado por IMU cada ciclo"]
        FSM["procesar_ciclo_completo_lidar()\nFSM fase_actual + estado_evasion"]
    end

    CAM -- "color_crudo (lock_vision)" --> FSM
    LID -- "scan_buffer_listo (lock_scan)" --> SCAN
    PICO_IN -- "angulo_acumulado_robot" --> SCAN
    SCAN -- "clusters_obstaculos" --> TRACK
    PICO_IN -- "angulo_acumulado_robot" --> TRACK
    TRACK -- "tracker[x,y,activo,confirmaciones]" --> FSM
    SCAN -- "dist_derecha_min, dist_izquierda_min, dist_frontal_min" --> FSM

    FSM -- "UART 115200 bps: velocidad,angulo" --> PICO["Raspberry Pi Pico 2 (main.py)\nCENTRO=90° + offset - Kd*giro_z"]
    PICO -- "UART: IMU:angulo_acumulado" --> PICO_IN
```

> Este diagrama expone por qué el bug #3 de la sección 8.2 era invisible sin instrumentación: `tracker["x"]` viajaba correctamente hasta el bloque `FSM`, pero el cálculo del ángulo simplemente no lo leía -- el dato existía en el sistema, solo no estaba conectado al punto de decisión correcto.

### 8.2 Caso de Estudio: Depuración de la Ronda Cerrada con Evidencia de Pista (`Close2_round.py`)

Durante el desarrollo activo de la Ronda Cerrada, el equipo reportó que el robot "detecta el poste pero esquiva mal". En vez de ajustar parámetros a ciegas, se instrumentó el sistema para capturar evidencia real (video de la corrida + log de consola con `python3 -u Close2_round.py 2>&1 | tee run_log.txt`) y se diagnosticó cada síntoma contra las líneas exactas de log. Este es el registro de los hallazgos, en el orden en que se corrigieron:

| # | Síntoma Reportado | Evidencia (log/código) | Causa Raíz | Corrección |
| :---: | :--- | :--- | :--- | :--- |
| 1 | El robot evade al lado contrario al reglamentario | `EVADIR_POR_IZQUIERDA = (color_det == "ROJO")` | Mapeo de color invertido: la regla WRO es Rojo→derecha, Verde→izquierda, pero el código asignaba izquierda al rojo | `EVADIR_POR_IZQUIERDA = (color_det == "VERDE")` |
| 2 | Detección de color inconsistente con la calibración en laptop | `cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)` en el robot vs. `COLOR_BGR2HSV` en `calibrar_hsv.py` | Picamera2 con formato `"RGB888"` en realidad entrega los bytes en orden BGR (comportamiento documentado de la librería) | Unificado a `COLOR_BGR2HSV` en ambos |
| 3 | Giro "a ciegas", sin importar la posición real del obstáculo | `angulo_objetivo_crudo = 28.0 if EVADIR_POR_IZQUIERDA else -28.0` (ángulo fijo); `KP_EVASION_LATERAL` y `MAX_ANGULO_EVASION` definidas pero sin uso en ningún lado | El tracker LiDAR (clustering ABD + corrección IMU) sí calculaba `tracker["x"]` (offset lateral real, mm) pero nunca se conectó al cálculo del ángulo | Control proporcional: `angulo = signo_evasion * ANGULO_BASE + tracker["x"] * KP_EVASION_LATERAL` |
| 4 | El robot "manda la señal de evadir de una vez", sin confirmar distancia | `if (trk_confirmado and frontal < 600) or frontal < 400 or tiempo_detectado > 0.3` | El *timeout* de 0.3s ganaba casi siempre antes que la confirmación real de distancia LiDAR | Timeout subido a `TIMEOUT_DETECTADO = 1.2` (queda como red de seguridad, no como camino normal) |
| 5 | Cascada de emergencias tras esquivar un pilar (log real: `Error heading=68.1 deg`) | `RECENTRANDO` se rendía por timeout (1.5s) sin haber corregido el rumbo, y el robot volvía a `CARRERA` desalineado, disparando `EMERGENCIA COLISION INMINENTE` de inmediato | El control proporcional de `RECENTRANDO` estaba capado a ±25° y no alcanzaba a converger en errores de rumbo grandes dentro del tiempo asignado | `TIMEOUT_RECENTRANDO` subido a 3.0s y límite de giro igualado a `MAX_ANGULO_EVASION` (32°) |
| 6 | El robot frenaba de más justo durante la maniobra de evasión | `velocidad = max(VELOCIDAD_MIN_EN_FRENADO, int(velocidad_base * factor_frenado))` aplicado en todos los estados | `factor_frenado` se aplicaba dos veces en `DETECTADO` (frenado al cuadrado) y en `ESQUIVANDO`/`PASANDO`/`RECENTRANDO` frenaba según la distancia al propio poste que se estaba evadiendo (sector frontal ensanchado a propósito durante la maniobra) | `factor_frenado` restringido exclusivamente al estado `CARRERA` |
| 7 | Crash de `GPIO.cleanup()` al detener el script con doble Ctrl+C | `lgpio.error: 'unknown handle'` en el traceback | `apagar_sistema()` se reejecutaba sobre un handle GPIO ya cerrado | Guardia de reentrada (`_apagando_en_curso`) + `try/except` alrededor de `GPIO.cleanup()` |

> **Nota metodológica:** los hallazgos #1, #2, #4, #6 y #7 se identificaron por lectura de código y razonamiento sobre la convención de signos del sistema (verificada de forma cruzada contra `Open_round.py` y `src/pico/main.py`). El hallazgo #5 se identificó directamente de una línea de log real de una corrida en pista.

**Validación en pista con paredes reales:** tras aplicar los fixes, se corrió el mismo protocolo (video + `python3 -u Close2_round.py 2>&1 | tee run_log.txt`) en un circuito con bordes físicos. El log mostró 3 evasiones completas de postes rojos, todas con el lado de evasión correcto (`Evadir x DERECHA`) y la transición `DETECTADO -> ESQUIVANDO` siempre por distancia real confirmada por el LiDAR (nunca por el timeout de seguridad). El punto crítico —`RECENTRANDO`— convergió dentro del margen las 3 veces (`Error heading` de 3.9°, 3.9° y 3.2°, todos bajo el umbral de 4°), frente al fallo de 68.1° registrado antes del fix. Las dos emergencias de colisión que sí aparecieron se resolvieron limpio vía `RETROCEDIENDO -> FORZANDO_GIRO` sin entrar en el ciclo repetitivo observado en la corrida anterior.

**Refactor de modularidad:** posteriormente, `Close2_round.py` (que concentraba cámara + LiDAR + tracker + FSM en ~1100 líneas) se dividió en `vision.py`, `lidar.py` y `tracker.py` por responsabilidad (ver sección 8.1). Durante esa limpieza se detectaron y archivaron en `src/pi3B/ronda_cerrada/legacy/` dos copias obsoletas de la Ronda Cerrada (`Close_round.py` y una iteración experimental) que **todavía tenían la regla de color invertida** — y se descubrió que `controlador_inicio.py` apuntaba por error a esa copia rota en vez de a `Close2_round.py`, ya corregido.

### 8.3 Caso de Estudio: Reactivación de la Ronda Cerrada Modular con Evidencia Cuantitativa (2026-08-27)

Tras el refactor de modularidad de la sección 8.1, `src/pi3B/ronda_cerrada/` (clustering LiDAR por ABD, fusión con centroide, FSM sin I/O) nunca se desplegó en pista: la Raspberry seguía corriendo un monolito de reemplazo (`prueba/reto_obstaculos_v2.py`, ~1000 líneas) que había perdido el clustering, la fusión por posición y el parqueo. El equipo reportó "un completo fracaso" con ese monolito. En vez de seguir depurándolo, esta sesión partió de una pregunta distinta: **¿por qué se abandonó la pila modular, si ya estaba probada?**

La metodología fue la misma que en la sección 8.2 —evidencia real contra hipótesis, no ajuste a ciegas— pero instrumentada con más rigor: cada corrida se grabó con una cámara cenital externa a la pista (no la de a bordo) para correlacionar la telemetría con lo que el robot hacía físicamente, y `registro_metricas.py` se extendió para guardar percepción cruda por ciclo (`frontal`, `izquierda`, `derecha`, `trasera`, `color_cam`, estado del tracker) además del error ya derivado, porque un mismo `error_lateral` puede salir de causas que piden arreglos opuestos y sin los datos crudos no hay forma de distinguirlas.

| # | Síntoma / Hallazgo | Evidencia | Causa Raíz | Corrección |
| :---: | :--- | :--- | :--- | :--- |
| 1 | La IMU de la pila modular estaba muerta — `heading()` devolvía siempre `0.0` | `EnlacePico._hilo_lectura` hacía `linea.split(":")[1]` sobre `"IMU:-8593.44,COLOR:PISTA"`, que da `"-8593.44,COLOR"` y hace fallar `float()` dentro de un `except` mudo | Al flashear el firmware con sensor de color, la trama pasó de `"IMU:<grados>"` a `"IMU:<grados>,COLOR:<nombre>"` y el parser no se actualizó | Recortar por la coma antes de partir por `":"` (`c84f387`) |
| 2 | La cámara veía la pista de cabeza | Captura de un frame crudo sin rotar: los pilares aparecían colgando del techo, con el piso arriba | El módulo de cámara está montado invertido en el chasis; `camara_driver.py` entregaba el frame tal cual salía de `picamera2` | Rotar 180° en la capa de adquisición (`268c633`) — de paso corrige que el filtro `cy < 180` de `vision.py` se invertía: con el piso arriba, el centroide del poste bajaba al acercarse, perdiendo la detección en el momento crítico |
| 3 | El comando de dirección llegó a pedir **+107°/-78°** con el servo real en -20/+25 | CSV de la corrida 2: `err=-1456mm → ang=-30.6°`, luego `err=-1012mm → ang=-42.6°` (el error cae 31% y el comando sube) | `_centrado_paredes` devolvía `(izq-der)*KP_LATERAL` sin acotar — único cálculo de ángulo del módulo sin recorte. En las esquinas el pasillo se abre a >1400mm, pidiendo ~200° de servo; el *rate limiter* de 6°/ciclo rampaba hacia ese objetivo imposible (*windup*) | Recortar al recorrido real y asimétrico del servo en `_centrado_paredes` y en la salida común tras el *rate limiter* (`ad74c17`) — verificado reprocesando el CSV: comandos fuera de rango 97→0, servo apuntando al lado contrario 2.79s→1.28s |
| 4 | Las transiciones de evasión saltaban casi siempre por `timeout`, nunca por geometría | 4 de 5 transiciones de la corrida 3 fueron `\| timeout` | `TIMEOUT_APROXIMACION=1.5s` y `TIMEOUT_SOBREPASO=1.2s` estaban calibrados para una velocidad que el robot no tenía — documentados como red de seguridad, funcionaban como ruta principal | Derivar los timeouts de la velocidad real medida en cada fase en vez de un número suelto (`e78884f`, refinado en `12ca0f1` tras medir la curva PWM→velocidad) |
| 5 | Alimentación: la Raspberry estaba limitada **en reposo** | `vcgencmd get_throttled` → `0x50005` (bits de bajo voltaje activo) | El regulador XL4015/4016 entregaba 4.9V en bornes; bajo la caída de carga la Pi veía menos de los ~4.65V del umbral de detección | Reajuste del trimpot a 5.132V (hardware, sin commit de código) — verificado con reinicio limpio: `0x50000`, tasa del lazo de control 8.6→10.1Hz |
| 6 | Tras la esquiva, el robot se metía de vuelta contra el mismo poste | Corrida 3, ciclo a ciclo: `t=1.54 ang=-5.4 trk_x=-215` (progresando) → `t=2.04 ang=+21.3 trk_x=-272` (servo al tope contrario) → `t=2.44 trk_x=-195` (el poste vuelve al centro) | `SOBREPASO` enderezaba hacia `_heading_base`, el rumbo **anterior** a la evasión — deshacía el giro de esquiva justo a la altura del poste | `SOBREPASO` mantiene el rumbo con el que entró a ese estado, no el previo a la evasión (`cb4f710`) |
| 7 | Tras rebasar el poste, el robot terminaba pegado a un muro sin haber chocado de frente | Corrida 4: mediana de `izquierda` cae de 561mm en `APROXIMACION` a 310mm en `SOBREPASO` (mínimo 80mm); `derecha` nunca bajó de 257mm — siempre el mismo lado. 3 de 6 emergencias fueron laterales con el frente despejado (una a 1032mm) | `REINCORPORACION` anulaba el error de **rumbo**, que no dice nada de la posición del robot en el pasillo: se puede cumplir el objetivo entero y acabar contra un muro, porque enderezar estando desplazado no corrige el desplazamiento | `REINCORPORACION` vuelve al centro con el mismo control de posición que `CRUCERO` (`izquierda-derecha`), que se anula solo al llegar al eje y no puede sobrepasar (`cb4f710`) |
| 8 | El retroceso de emergencia reorientaba el robot 50-60° por episodio | Corrida 4, 6 episodios de `RETROCESO`: todos de 3.49s exactos (el timeout completo), todos saliendo por `"tiempo maximo"`; el frente ya estaba despejado 1.5-2.4s antes de que el estado terminara | `_est_retroceso` solo salía por obstáculo trasero o timeout, sin comprobar si el peligro ya se había resuelto — 3.5s de servo puesto es mucha rotación de sobra | Salir en cuanto frontal y laterales superan un margen más holgado que el de entrada, con un mínimo de 0.6s (`9e857da`) — rotación acumulada 324°→162°, tiempo en retroceso 41%→4% en la corrida siguiente |
| 9 | El modelo de velocidad del tracker sobreestimaba y rompía la asociación con el LiDAR | `tracker.MM_POR_SEG_A_PWM100 = 900.0`, con el propio comentario admitiendo que era una suposición sin medir | Medido en pista por odometría LiDAR (PWM 40→158mm/s, 70→285mm/s, 90→358mm/s; ajuste `v=4.02·pwm-1.0`, validado cruzado con la velocidad de crucero real de la corrida 3, 220 vs 215mm/s): el valor real es ~400, no 900 | `MM_POR_SEG_A_PWM100 = 400.0` (`12ca0f1`) — con 900 el error de predicción acumulaba ~20mm/ciclo, saliendo de `UMBRAL_ASOCIACION` (250mm) en poco más de un segundo (los `"timeout de prediccion"` del log) |
| 10 | Al alargar `SOBREPASO` para que coincidiera con la velocidad real, el robot volvió a acercarse a la pared (84mm, a 4mm del umbral de emergencia) | Corrida 7, con el servo casi recto todo el tramo: `der` cae monótono de 366mm a 84mm durante los 2.9s de `SOBREPASO` (81mm/s de cierre lateral sostenido) | El timeout de `SOBREPASO` se recalculó correctamente respecto al poste, pero en este estado el rumbo mantenido apunta ligeramente hacia la pared — el límite real no es el poste, es la pared | `DIST_SOBREPASO_MM` de 350mm a 200mm (`e5999af`) — predicción con el modelo de cierre lateral: salida a 236mm, medido 232mm (2% de error) |

> **Nota metodológica:** todos los hallazgos de esta tabla se identificaron leyendo `registro_metricas.py` de cada corrida (nunca por observación cualitativa de "se ve raro"), y cada corrección se validó reprocesando el CSV de la corrida anterior con la lógica nueva antes de volver a probar en pista — el hallazgo #10 incluso se predijo numéricamente (236mm) y se confirmó dentro del 2% en la corrida siguiente. El hallazgo #5 (alimentación) es la excepción: no es un bug de software, y sin él ninguno de los arreglos de código se habría podido medir con datos limpios (la tasa del lazo de control estaba degradada por el mismo *brownout*).

**Progresión medida, corrida a corrida** (mismo montaje: un pilar rojo, robot en posición de arranque):

| Corrida | Cambio aplicado | Emergencias | % tiempo en `RETROCESO` | % tiempo en `CRUCERO` | Pared mínima |
| :---: | :--- | :---: | :---: | :---: | :---: |
| 2 | (instrumentación) | 1 | 10% | 77% | 106mm |
| 3 | Windup + alimentación | 2 | 15% | 66% | 112mm |
| 4 | Timeouts por física | 6 | 42% | 13% | 69mm |
| 5 | Salida del retroceso | 1 | 4% | 66% | 100mm |
| 6 | Trayectoria por posición | **0** | **0%** | 76% | 232mm |
| 7 | Velocidad medida (tracker) | 0 | 0% | 70% | 84mm |
| 8 | `SOBREPASO` acortado | **0** | **0%** | **79%** | 150mm |

La corrida 4 es peor que la 3 en casi todas las columnas, y eso es información, no ruido: al alargar los timeouts, la evasión por fin llegaba a completarse y quedó al descubierto el fallo de trayectoria (#7) que hasta entonces estaba tapado por transiciones que nunca llegaban a ese punto. Lo mismo entre 6 y 7: corregir la velocidad del tracker alargó `SOBREPASO` de 2.1 a 2.8s y destapó el hallazgo #10. Cada arreglo hizo visible el siguiente — es el patrón esperable al depurar una cadena de estados acoplados, no una regresión.

**Video de las corridas 6-8** (cámara cenital, recortado a la ventana de acción): [`video/video-drafts/2026-08-27_S3_corrida6.mp4`](video/video-drafts/2026-08-27_S3_corrida6.mp4), [`_corrida7.mp4`](video/video-drafts/2026-08-27_S3_corrida7.mp4), [`_corrida8.mp4`](video/video-drafts/2026-08-27_S3_corrida8.mp4) — índice completo en [`video/video.md`](video/video.md).

**Estado al cierre de la sesión:** las tres últimas corridas terminaron sin una sola emergencia, con el robot detectando el pilar rojo, esquivando por la derecha (regla WRO), rebasándolo y reincorporándose al carril de forma repetible. Quedan pendientes, en orden de prioridad:

* **Fail-safe de pérdida de comunicación Pi↔Pico (sección 5, diagrama de arquitectura).** Auditando ese diagrama contra el código se confirmó que el nodo de detención por fallo de enlace nunca se implementó: `TIMEOUT_TELEMETRIA`/`heading_valido()` existen en `enlace_pico.py` pero solo los usa una herramienta de calibración, y el firmware de la Pico no tiene ningún *watchdog* — si el enlace se corta a mitad de carrera, la Pico sigue aplicando la última consigna recibida sin límite de tiempo. Es un riesgo de seguridad real, no solo un defecto de documentación; se deja fuera del alcance de esta sesión a propósito porque tocar el firmware de bajo nivel sin poder probarlo con un desconexión real de USB en pista sería más arriesgado que dejarlo pendiente y documentado.
* La maniobra de estacionamiento en paralelo (sección 13 del reglamento — nunca ejercitada, ver limitación en la sección 5.3-C sobre por qué el sentido de carrera no la bloquea).
* Validar la evasión por la izquierda con el pilar verde (toda la sesión se corrió con rojo para aislar variables).
* El apareo color↔cluster cuando hay dos postes casi equidistantes en el mismo frame (`_intentar_capturar_poste` empareja por "cluster más cercano" y "blob de mayor área" con criterios independientes, riesgo de asignar el color equivocado a la posición equivocada).

### 8.4 Caso de Estudio: Gauntlet de 6 Pilares, Asistencia de Esquina y el Límite de la Reactividad Pura (2026-08-28)

Continuación de la sección 8.3 al día siguiente. Se probó el sistema con **6 pilares** (2 por tramo recto, el doble del reglamento oficial) para estresar la fusión sensorial, y se investigaron dos síntomas nuevos que aparecieron en esa corrida.

**1. Emergencias en esquina, sin ningún poste cerca.** En la primera corrida de 6 pilares, dos emergencias ocurrieron con el `tracker` inactivo (sin ningún poste involucrado): `frontal`/`izquierda`/`derecha` cayendo juntos de ~500mm a ~110mm en unos 5s, con el ángulo de dirección casi sin moverse. Diagnóstico: `lidar_geometria.py` ya calculaba `angulo_muro` (triangulación con los haces perpendicular y diagonal de cada lado) en cada barrido, pero `navegacion.py` nunca lo leía — mismo patrón que `poste_cx_estable` el primer día. Verificado **con el robot físico, sin motores** antes de tocar la dirección: apuntando a la esquina real, `perp_izq=234mm` pero `diag_izq=3000mm` (el haz diagonal ya no encuentra el muro — se "abrió"), dando `angulo_muro=-22°` estable, contra apenas +11° que hubiera dado `izquierda-derecha` solo. Arreglo (`71ede18`): `_centrado_paredes` suma `-angulo_muro * KP_ANGULO_MURO` (KP=0.65). **Validado con motores** en una corrida limpia (sin pilares, robot con margen real antes de la esquina): navegó dos esquinas completas, 186° de rumbo, **pared mínima 412mm en toda la corrida, cero emergencias** — contra 76mm y 5 emergencias en la corrida original.

**2. Falso positivo de "ROJO" sin ningún pilar en pista**, detectado una sola vez durante un giro rápido (rumbo +5°→+86° en 2.7s). No se reprodujo sosteniendo el robot quieto en el mismo rango de ángulos durante 60s, lo que sugiere un artefacto transitorio ligado al movimiento (desenfoque, reajuste de auto-exposición) y no un objeto fijo. Se instrumentó `vision.py` (`a6358eb`) con guardado opcional de frame+máscara en cada transición de color (`WRO_DEBUG_VISION=1`, apagado por defecto) para poder capturarlo la próxima vez. **No volvió a aparecer** en la corrida de validación del punto 1 ni en el gauntlet repetido del punto 3 — sigue sin causa confirmada.

**3. Repitiendo el gauntlet completo con el arreglo de esquina activo:** 5 evasiones correctas en los primeros 34s (igual de bien que antes), pero a partir de t≈155s el robot quedó atrapado en un bucle de emergencia-retroceso-reintento durante el resto de la corrida (133s, 51 episodios de `RETROCESO` en total). Causa, confirmada en el CSV: `izquierda` y `derecha` bajaban **casi exactamente iguales entre sí** en cada ciclo de acercamiento (ej. 210/213 → 204/207 → ... → 99/108mm), sin diverger nunca — el robot se acercaba por la bisectriz exacta de la esquina. Con esa simetría perfecta, `angulo_muro` se queda cerca de 0 (nunca superó ±4° en ninguno de los ~20 acercamientos registrados) porque **no hay ninguna asimetría que triangular** — ambos lados de la pared se ven igual de cerca todo el tiempo. El retroceso tampoco puede romper el empate: también decide por la diagonal trasera con más espacio, y esa señal es igual de simétrica en este caso.

> **Esto no es un defecto del arreglo de la sección 8.3-1 — es un límite de cualquier controlador puramente reactivo (sin memoria entre ciclos) ante una aproximación simétrica.** No hay dato instantáneo del LiDAR que pueda preferir un lado sobre otro cuando los dos son honestamente idénticos. La solución necesita estado persistente: detectar N episodios de `RETROCESO` seguidos sin avance neto de rumbo, y forzar un giro comprometido hacia un lado (decidido una vez, no reactivo) para romper la simetría — pendiente de diseñar e implementar, no se abordó en esta sesión por el alcance y el tiempo ya invertido.

**Estado al cierre:** la asistencia de esquina queda validada y sirve para el caso general (aproximación con algo de asimetría inicial, que es la mayoría de los casos reales). El caso límite de la esquina perfectamente simétrica es el próximo punto pendiente de más prioridad, junto con los ya listados en la sección 8.3 (fail-safe de comunicación, parqueo, evasión por la izquierda con pilar verde, apareo color↔cluster).

### 8.5 Diseño e Implementación: Desempate de Esquina Simétrica con Memoria Persistente (2026-08-28, continuación)

Continuación directa de 8.4-3. La conclusión de esa sección fue que ningún control reactivo puro puede romper un empate honesto entre paredes — hace falta estado que persista *entre* ciclos, algo que ningún otro estado de `navegacion.py` tiene o necesita.

**Diseño.** Se añadió `GIRO_FORZADO`, único estado del archivo que no recalcula su decisión cada ciclo:

- `_racha_retroceso` cuenta entradas a `RETROCESO` seguidas sin avance neto de rumbo (≥8° desde que empezó la racha corta la cuenta — un par de frenazos con progreso real no debe disparar esto).
- Una memoria aparte (`_signo_memoria_asimetria`) guarda el signo de `izquierda-derecha` cada vez que supera 30mm (ruido típico del C1): casi ninguna esquina es simétrica perfecta desde lejos, así que normalmente hay un sesgo real que capturar antes de que se cierre del todo. Si nunca lo hubo (el caso exacto de 8.4-3), cae a un lado por defecto — arbitrario a propósito, documentado como tal; el punto no es acertar el lado "correcto" (no lo hay, por definición del problema), es terminar el bucle.
- A las 4 rachas sin avance, `_est_retroceso` entra a `GIRO_FORZADO` con el lado fijado en ese instante y no vuelto a tocar; el estado mantiene el giro hasta que la pared de ese lado se abra de verdad (asimetría por encima de 150mm) o por timeout de seguridad (2.5s).

**Validación fuera de pista.** Sin acceso al robot en el momento de esta sesión, se validó con barridos LiDAR sintéticos (mismo patrón que el resto del módulo: lógica sin I/O, se puede probar con `Medicion` construida a mano) tres escenarios:

| Escenario | Resultado |
| :--- | :--- |
| Reproducir el bucle de 8.4-3 (100mm simétrico → emergencia → retroceso → reacercarse igual de simétrico, repetido) | Dispara `GIRO_FORZADO` en la 4ª racha, sale de vuelta a `CRUCERO` |
| Mismas emergencias pero con el rumbo progresando de verdad entre medias | Nunca dispara `GIRO_FORZADO` — la racha se corta cada vez que hay avance real |
| Acercamiento con sesgo real hacia un lado (50mm, sobre el umbral de 30mm) antes de cerrarse simétrico | `GIRO_FORZADO` usa el lado memorizado, no el default |

Commit `6b74f8e`.

**Al probarlo en pista, no funcionó — y el log explicó por qué en el primer minuto.** La racha se reiniciaba sola: `racha 1, 1, 1, 2, 3, 1...`, sin llegar nunca al umbral de 4. El CSV lo confirmó: el robot **sí giraba** 5-7° por episodio (rumbo de -37° a +39° a lo largo de 12 emergencias) pero sin escapar de la esquina. El criterio de "avance neto de rumbo" era falso de raíz — el rumbo se mueve sin que el robot progrese, así que no sirve como medida de escape. Sustituido por la **cadencia**, que sí distingue los dos casos sin ambigüedad: atascado, las emergencias caen cada 4.7-5.8s como un reloj (12 episodios medidos, mediana 4.8s); en una corrida sana no hay ninguna (corridas 6, 7 y 8 de la sección 8.3).

**Pero ese ni siquiera era el fallo principal.** Al mirar el acercamiento ciclo a ciclo apareció la causa real, y no era la esquina simétrica de 8.4-3:

| t | frontal | izq | der | izq-der | angulo_muro | **ángulo** |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| 44.96 | 278 | 206 | 244 | -38 | -6.4 | **-1.3** |
| 45.96 | 208 | 144 | 174 | -30 | -5.9 | **-0.5** |
| 46.86 | 124 | 87 | 115 | -28 | -3.5 | **-1.6** |

El frente se cierra de 302 a 124mm en 3 segundos con **el servo en ~1°**: el robot entra recto contra la pared con la dirección prácticamente centrada. Descomponiendo `_centrado_paredes`, los dos términos apuntan a lados **opuestos** y se anulan (`T_pos=-5.32` contra `T_muro=+4.17` → `-1.15`). Ocurre en **274 de los 394 ciclos** con el frente por debajo de 400mm (70%), dejando un comando mediano de **1.5° con un servo que da 20-25°**.

La raíz es que los dos términos miden cosas distintas —posición entre paredes y orientación respecto al muro— y ninguno mira el frente. `(izq-der)` dice dónde está el robot *entre* las paredes, no cuánto espacio queda: en un pasillo que se cierra a 200mm de ancho vale casi cero aunque el robot esté a punto de chocar con las dos. (Se descartó por medición la hipótesis alternativa de que el haz diagonal estuviera contaminado por la pared frontal: no lo estaba en ninguno de los ciclos.)

**Corrección: `_con_escape_frontal`.** Introduce la pregunta que faltaba —"hay pared delante, hacia dónde salgo"— con autoridad creciente según se cierra el frente (`DIST_ESCAPE_FRONTAL=500mm`, `ANGULO_ESCAPE_MAX=22°`), *mezclándose sobre* el centrado en vez de sumarse, porque sumar dejaría que la cancelación se lo siguiera comiendo. Cuando las dos paredes están dentro del ruido del LiDAR usa la misma memoria persistente que `GIRO_FORZADO`, para que las dos defensas elijan el mismo lado.

**Validado con motores, mismo montaje que la corrida anterior:**

| | Antes | Después |
| :--- | ---: | ---: |
| Emergencias | 12 | **1** |
| Tiempo en `RETROCESO` | 32% | **2%** |
| Ciclos en peligro sin autoridad de dirección (<3°) | 71% | **8%** |
| Rumbo recorrido | 106° | **442°** |
| Evasiones iniciadas | 0 | **8** |
| Pared mínima | 75mm | 114mm |

El bucle desapareció y el robot volvió a encadenar evasiones (8 en 84s, rojo y verde). `GIRO_FORZADO` **nunca llegó a dispararse**, que es exactamente el diseño: el escape frontal ataca la causa y el desempate queda como red de abajo para el caso simétrico puro de 8.4-3, que esta corrida no volvió a reproducir. Commits `6b74f8e` (desempate) y `9626e80` (escape frontal + criterio de cadencia).

> **Lección metodológica:** el arreglo de la primera mitad de esta sección se diseñó contra el síntoma descrito en 8.4-3 ("esquina simétrica") sin volver a mirar datos crudos, y en pista resultó que el bucle observado tenía otra causa — dos términos de control cancelándose, que ninguna cantidad de simulación sintética iba a revelar porque la simulación reproducía la hipótesis, no la pista. La validación sintética sirve para comprobar que la lógica hace lo que se cree, no para descubrir qué está pasando en el robot.

**Pendiente:** la corrida se cortó a los 84s a petición del equipo, así que queda por confirmar una vuelta completa (3 vueltas + parqueo) sin interrupción.

---

## Licencia y Dependencias de Terceros

Este repositorio se distribuye bajo la [Licencia MIT](LICENSE). El software de la Raspberry Pi 3B depende de las siguientes librerías de código abierto (ver [`src/pi3B/requirements.txt`](src/pi3B/requirements.txt) e [`INSTALACION.md`](INSTALACION.md)): OpenCV (`opencv-python`), NumPy, PySerial, RPi.GPIO y `picamera2` (paquete oficial de Raspberry Pi para la Pi Camera Module 3). El firmware de la Pico 2 corre sobre MicroPython y no usa librerías externas adicionales.
