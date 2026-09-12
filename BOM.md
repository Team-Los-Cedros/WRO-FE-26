# Lista de Materiales (BOM) — Team Los Cedros, WRO Future Engineers 2026

Todo lo que hay que comprar y montar para reconstruir el vehículo desde cero, con la cantidad exacta, la función que cumple cada pieza y dónde está la justificación de por qué se eligió esa y no otra.

Esta lista es el índice de compra. El procedimiento de instalación del software está en [`INSTALACION.md`](INSTALACION.md), el cableado pin a pin en la [sección 4.3 del README](README.md#43-mapa-de-conexiones-calibrado-pinout), y la justificación comparativa de cada selección —incluidas las alternativas descartadas— en las [secciones 3.4 y 4.2](README.md#42-catálogo-de-componentes-y-justificación-de-selección).

**Los precios no se incluyen a propósito:** el equipo compra en Venezuela y en tiendas locales, donde el precio de un mismo componente varía tanto que una cifra publicada engañaría a quien intente reproducir el robot en otro país. Lo que sí se da es el modelo exacto, que es lo que permite cotizar.

---

## 1. Cómputo y sensado

| # | Componente | Modelo exacto | Cant. | Función en el vehículo |
| :---: | :--- | :--- | :---: | :--- |
| 1 | Computadora de alto nivel | **Raspberry Pi 5** (4 GB o superior) | 1 | Visión, procesado del LiDAR y máquina de estados de navegación. Sustituyó a la Pi 3B el 03-09-2026 por cómputo medido: el mismo procesado de imagen pasó de 67-72 ms a **4,8 ms** por cuadro. |
| 2 | Microcontrolador de tiempo real | **Raspberry Pi Pico 2** (RP2350) | 1 | Genera el PWM del servo y del motor, integra el giroscopio y aplica el *watchdog* de 500 ms. Aísla el lazo físico del *jitter* de Linux. |
| 3 | Telémetro láser rotativo | **RPLiDAR C1** | 1 | Paredes, esquinas y detección geométrica de los pilares en 360°. Conectado por USB a 460 800 bps. |
| 4 | Cámara | **Raspberry Pi Camera Module 3** (sensor IMX708, versión **estándar**) | 1 | Color de los pilares y lectura de las líneas de pista. Conecta por CSI de 15 pines. |
| 5 | Unidad inercial | **MPU6050** (giroscopio + acelerómetro, I²C) | 1 | *Yaw* del vehículo para contar vueltas y sostener el rumbo. Va soldada a la placa perforada, alineada con el eje longitudinal. |
| 6 | Ultrasonido | **HC-SR04** | 1 | Única medida real hacia atrás: el mástil del LiDAR le tapa el sector 140-213°. Imprescindible para el estacionamiento. |
| 7 | Pulsador de arranque | Pulsador momentáneo NA | 1 | Da la salida en la ronda oficial (`GPIO 21` de la Pi 5, con *pull-up* interno). |
| 8 | Sensor de color de piso | **TCS3472** (I²C) | 1 | Montado **al frente del vehículo**, por delante del eje delantero y mirando el piso, no bajo el chasis: leer la línea antes de pisarla da margen de reacción. Clasifica la línea de esquina como `AZUL` o `NARANJA`; el **orden** en que se cruza la pareja de una misma esquina es la única evidencia absoluta del sentido de carrera. |
| 10 | Almacenamiento | microSD 32 GB clase 10 o superior | 1 | Sistema operativo y registros de telemetría de cada corrida. |

> **Sobre la cámara: es la Module 3 estándar, no la Wide.** Conviene decirlo porque el catálogo de la Wide anuncia 102° y eso induce a comprar la equivocada. La medida propia está en [`optica.py`](src/pi5/ronda_curvas/optica.py): emparejando una esquina que el LiDAR sitúa en −21,5° con su borde en el frame sale un **HFOV efectivo de 53,8°**, que concuerda con los 51,9° previstos para la Module 3 estándar recortada a 4:3 y no con los 85,6° de la Wide.

## 2. Actuación y potencia

| # | Componente | Modelo exacto | Cant. | Función en el vehículo |
| :---: | :--- | :--- | :---: | :--- |
| 9 | Servomotor de dirección | **Geekservo Servo** | 1 | Dirección Ackermann. Se eligió por acople nativo a vigas Technic: cualquier adaptador impreso añade holgura al tren delantero. |
| 11 | Motor de tracción | **Geekservo DC** | 1 | Tracción trasera. Par de bloqueo de 2,4 kg·cm, con margen de **2,18×** sobre la masa del vehículo. |
| 12 | Driver de motor | **TB6612FNG** (puente H doble) | 1 | Etapa de potencia del motor DC. Elegido sobre el L298N por topología MOSFET: menos caída de tensión y menos calor. |

## 3. Alimentación y regulación

| # | Componente | Modelo exacto | Cant. | Alimenta |
| :---: | :--- | :--- | :---: | :--- |
| 13 | Celdas de batería | **Li-ion 21700** en configuración **2S** (7,4-8,4 V) | 2 | Toda la energía del vehículo. |
| 14 | Regulador reductor | **XL4016** (salida 5,1 V / 8 A) | 1 | Raspberry Pi 5, cámara y RPLiDAR C1. |
| 15 | Regulador reductor | **XL1509** (salida 6,0 V / 2 A) | 1 | Servomotor de dirección, en línea limpia y separada de la lógica. |
| 16 | Portapilas 2S | Portaceldas 21700 doble | 1 | Sujeción mecánica y contacto de las celdas. |

> **La separación en tres etapas no es opcional.** Es lo que evita que el pico de corriente del motor provoque un reinicio de la Raspberry a mitad de carrera. El consumo real medido con multímetro el 06-09-2026 es de **1,39 A en marcha**, **0,61 A en reposo** y **0,21 A con la Pi apagada**; restando estados se deduce que la Pi 5 consume 0,40 A y la tracción 0,78 A. El desglose completo está en la [sección 4.4 del README](README.md#44-presupuesto-de-consumo-energético-y-gestión-de-corriente).

## 4. Estructura mecánica

| # | Componente | Detalle | Cant. | Función |
| :---: | :--- | :--- | :---: | :--- |
| 17 | Chasis | **LEGO Technic, 83 piezas** — listado completo con Design ID de BrickLink en [`models/Chasis-LEGO-V2/`](models/Chasis-LEGO-V2/README.md) | 1 | Estructura del vehículo. El archivo CAD `.io` es reproducible pieza por pieza con BrickLink Studio, que es gratuito. |
| 18 | Ruedas y neumáticos | LEGO, Design ID `bl_56145c01` | 4 | Dos motrices traseras y dos directrices delanteras. |
| 19 | Mástil del LiDAR | Estructura Technic incluida en las 83 piezas | 1 | Eleva el plano de barrido a **69 mm del piso**, altura a la que el haz corta tanto los pilares como las paredes (100 mm ambos según reglamento). |
| 20 | Placa perforada | Placa de prototipado para soldadura | 1 | Soporte permanente de Pico 2, TB6612FNG y MPU6050. Se soldó en vez de usar *jumpers* porque la vibración provocaba falsos contactos. |

> El chasis V1 era un monocasco impreso en 3D de ≈800 g. Se sustituyó por el actual de vigas Technic: **23,37 % menos masa**, menos resonancia en el soporte de la cámara y reconfiguración rápida en boxes sin reimprimir. Los STL del V1 se conservan en [`models/V1/`](models/V1/README.md) como registro del proceso de iteración.

## 5. Cableado e integración

| # | Componente | Detalle | Cant. |
| :---: | :--- | :--- | :---: |
| 21 | Cable plano CSI | 15 pines, para Camera Module 3 | 1 |
| 22 | Cable USB-A a micro-USB | Corto, Pi 5 ↔ Pico 2 (VCP a 115 200 bps) | 1 |
| 23 | Cable USB del LiDAR | El que incluye el RPLiDAR C1 | 1 |
| 24 | Cable de conexión | Calibre 22 AWG para señal, 18 AWG para potencia | — |
| 25 | Interruptor general | Conmutador de corte de batería | 1 |

> **Todas las tierras confluyen en estrella en un único punto central.** Esto unifica los umbrales lógicos entre las tres etapas y drena el ruido de conmutación del motor, que de otro modo aparece como falsos flancos en las líneas digitales.

## 6. Herramientas necesarias para el montaje y la puesta a punto

No forman parte del vehículo, pero sin ellas no se puede reproducir ni verificar el resultado.

| Herramienta | Para qué se usa en este proyecto |
| :--- | :--- |
| Soldador y estaño | Placa perforada y líneas de potencia. |
| Multímetro | Presupuesto de corriente de la sección 4.4. El del equipo es un ANENG M118A. |
| Computadora con SSH | Despliegue del código y lectura de la telemetría de cada corrida. |
| Cámara cenital | Grabar cada corrida y correlacionarla con el CSV de telemetría. Es el método de trabajo del equipo: cuando el registro y el vídeo se contradicen, **manda el vídeo**. |
| Regla o calibre | Verificar la altura del plano del LiDAR (69 mm) y la geometría del chasis. Varias medidas del montaje resultaron desviadas respecto a lo supuesto. |

---

## Resumen de cantidades

| Bloque | Piezas distintas | Unidades |
| :--- | :---: | :---: |
| Cómputo y sensado | 9 | 9 |
| Actuación y potencia | 3 | 3 |
| Alimentación | 4 | 5 |
| Estructura mecánica | 4 | 83 piezas Technic + placa |
| Cableado | 5 | — |

El vehículo completo, en su configuración actual (V3), pesa **720 g**. El chasis de vigas Technic es el mismo que el de la V2 —que pesaba 613 g como vehículo completo—; los 107 g de diferencia son la electrónica que se montó encima: Raspberry Pi 5 con su carcasa y ventilador, el mástil del LiDAR y el ultrasonido trasero.
