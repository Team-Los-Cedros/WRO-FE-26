# Proyecto Future Engineers - Team Los Cedros (WRO 2026)

Bienvenidos al repositorio oficial del **Team Los Cedros**, integrado por estudiantes del Colegio Los Cedros en Valera, Estado Trujillo, Venezuela. Aquí compartimos la documentación técnica, diseños de hardware, esquemas eléctricos y el software modular de nuestro vehículo autónomo para la World Robot Olympiad (WRO) 2026.

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

├── src/                        # Código fuente de la arquitectura distribuida
│   ├── pico/                   # Firmware embebido (MicroPython - Raspberry Pi Pico 2)
│   │   ├── main.py             # Bucle principal de control en tiempo real y actuadores
│   │   └── serial_receiver.py  # Parser serial no bloqueante para comandos de dirección
│   └── pi3b/                   # Scripts de alto nivel (Python 3 - Raspberry Pi 3B)
│       ├── controlador_inicio.py # Orquestador central (Ejecutado como servicio del sistema OS)
│       ├── Open_round.py       # Algoritmo secuencial para la Ronda Abierta
│       └── Close_round.py      # Algoritmo de visión y evasión para la Ronda Cerrada
├── 3d-Models/                  # Modelos mecánicos en formato STL y renders PNG
├── t-photos/                   # Fotos de las jornadas de desarrollo del equipo
├── v-photos/                   # Las 6 capturas reglamentarias del coche
├── video/                      # Archivo video.md con el enlace a la vuelta en pista
├── schemes/                    # Diagramas eléctricos y mapas electrónicos
└── README.md                   # Documentación técnica principal (este archivo)

```

> **Nota de Software de Inicio:** El script `controlador_inicio.py` actúa como el orquestador maestro en la Raspberry Pi 3B, configurado explícitamente como un servicio de `systemd` en Linux para garantizar el autoarranque inmediato del coche al encender la batería.

---

## 3. Diseño Evolutivo y Ciclos de Iteración

El desarrollo de nuestro vehículo autónomo no fue un proceso lineal. Para alcanzar la estabilidad actual, el prototipo pasó por una transición crítica basada en datos experimentales de rendimiento y fallos mecánicos en pista.

### 3.1 Cuadro Comparativo de Evolución Técnica

| Criterio Técnico | Prototipo Inicial (V1) | Prototipo de Producción Actual (V2) | Justificación de Ingeniería |
| --- | --- | --- | --- |
| **Material del Chasis** | Impresión 3D (Filamento) | Componentes Estructurales LEGO | Reducción drástica de la masa inercial. El chasis de LEGO absorbe mejor las vibraciones de alta frecuencia y permite iteraciones de geometría rápidas en los boxes. |
| **Sistema de Visión** | Arducam Module 3 | Raspberry Pi Camera Module 3 | El hardware anterior sufrió una falla crítica de hardware (daño por impacto). Se migró al módulo nativo para asegurar compatibilidad total de drivers a nivel de kernel. |
| **Dinámica en Pista** | Mayor subviraje por inercia física | Agilidad de giro y aceleración óptima | Al aligerar el prototipo, el servomotor requiere menos torque para vencer el coeficiente de fricción estática en las curvas Ackermann. |

### 3.2 Registro Fotográfico de la Evolución Histórica

A continuación se documenta el salto técnico entre ambas versiones del robot:

| Versión Anterior (V1) - Chasis Impreso Completo | Versión Actual (V2) - Optimización Estructural |
| --- | --- |
|  <img src="v-photos/V1/Versión Anterior.jpeg" alt="Anterior" height="500px"/> |  <img src="v-photos/Robot_photos/DSC_0126.png" alt="Actual" height="500px"/> |
| *Fallas identificadas: Exceso de peso, rigidez extrema ante impactos y fatiga de material en soportes.* | *Mejoras: Distribución de pesos equilibrada, flexibilidad ante impactos y modularidad total en BrickLink CAD.* |

---

## 4. Arquitectura Eléctrica y Distribución de Señales

### 4.1 Red de Distribución de Energía (Alimentación)

Para asegurar el correcto funcionamiento del vehículo autónomo y prevenir reinicios imprevistos (*brownouts*) en la Raspberry Pi 3B debido a picos de consumo dinámico de los motores, se implementó un sistema de alimentación completamente desacoplado por etapas:

| Fuente / Regulador | Voltaje Entrada | Voltaje Salida | Corriente Máx. | Componentes Alimentados |
| --- | --- | --- | --- | --- |
| **Baterías 21700 (2S)** | $7.4\,\text{V} - 8.4\,\text{V}$ | Directo | $30\,\text{A}$ | Línea de alta potencia del Driver TB6612FNG (Motor DC). |
| **Regulador XL1509** | $7.4\,\text{V} - 8.4\,\text{V}$ | $6.0\,\text{V}$ | $2.0\,\text{A}$ | Servomotor de dirección (Etapa de potencia limpia). |
| **Regulador XL4015** | $7.4\,\text{V} - 8.4\,\text{V}$ | $5.1\,\text{V}$ | $5.0\,\text{A}$ | Raspberry Pi 3B, Cámara Module 3 y RPLIDAR C1. |

>  **Nota eléctrica:** Todas las referencias de tierra (GND) del vehículo confluyen en una topología de estrella en un único punto común central. Esto unifica los umbrales lógicos y drena el ruido electromagnético generado por las conmutaciones de los motores.

### 4.2 Mapa de Conexiones Calibrado (Pinout)

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

#### Conexiones Maestras de la Raspberry Pi 3B

* **Pi Camera Module 3:** Conectada a la interfaz nativa CSI mediante un cable flexible plano de 15 pines.
* **RPLIDAR C1:** Conectado directamente a un puerto USB 2.0 maestro (Comunicación UART integrada a $460\,800\,\text{bps}$).
* **Raspberry Pi Pico 2:** Enlazada por interfaz de datos USB corta operando bajo la clase de dispositivo COM Virtual (VCP) a una tasa fija de $115\,200\,\text{bps}$.

---

## 5. Capa de Percepción y Alto Nivel (Raspberry Pi 3B)

La Raspberry Pi 3B se encarga de los procesos que demandan alta capacidad de cómputo. Mediante programación concurrente multihilos (`threading`), decodifica los datos en crudo del LiDAR y las imágenes de la cámara, calculando las decisiones estratégicas de navegación.

### 5.1 Lógica de Control de la Ronda Abierta

Para completar el reto de carrera de la Ronda Abierta, se diseñó una estrategia reactiva de centrado de carril en tiempo real. El script extrae las distancias mínimas detectadas en sectores geométricos específicos a izquierda y derecha del vehículo para computar un error proporcional.

La aproximación matemática utilizada responde a la siguiente ecuación proporcional de error lateral:

$$e(t) = D_i - D_d$$

$$\theta_{\text{objetivo}} = e(t) \cdot K_P$$

Donde:

* $D_i$ es la distancia mínima detectada hacia la pared izquierda dentro de la ventana de escaneo $[270^\circ, 330^\circ]$.
* $D_d$ es la distancia mínima detectada hacia la pared derecha dentro de la ventana de escaneo $[30^\circ, 90^\circ]$.
* $e(t)$ representa el error de desplazamiento lateral respecto al centro ideal de la pista.
* $K_P$ es la ganancia proporcional de guiado, calibrada empíricamente en $0.22$.
* $\theta_{\text{objetivo}}$ es la consigna angular enviada directamente hacia la subcapa de control de bajo nivel.

### 5.2 Estructura Modular del Script de Carrera (Fragmentos Clave)

El script opera bajo una máquina de estados finitos (`ESPERANDO_BOTON`, `CALIBRANDO`, `CAPTURA_INICIAL`, `CARRERA`, `BUSCANDO_PARQUEO`, `DETENIDO`). A continuación se detallan las funciones de sincronización asíncrona y telemetría:

```python
def hilo_comunicacion_pico():
    """ Hilo asíncrono para telemetría y procesamiento de odometría inercial global """
    global ser_pico, angulo_acumulado_robot, fase_actual, tiempo_inicio_parqueo, angulo_inicial_imu
    # ... [Inicialización serial a 115200 bps] ...
    while corriendo:
        if ser_pico.in_waiting > 0:
            try:
                linea = ser_pico.readline().decode('utf-8').strip()
                if linea.startswith("IMU:"):
                    valor_crudo_imu = abs(float(linea.split(":")[1]))
                    
                    if fase_actual in ["ESPERANDO_BOTON", "CALIBRANDO"] or angulo_inicial_imu is None:
                        angulo_inicial_imu = valor_crudo_imu
                    
                    # Cálculo del ángulo absoluto neto de carrera
                    angulo_acumulado_robot = valor_crudo_imu - angulo_inicial_imu
                    
                    # Transición automática de parada tras completar 3 vueltas completas (~1010 grados netos)
                    if fase_actual == "CARRERA" and angulo_acumulado_robot >= 1010.0:
                        fase_actual = "BUSCANDO_PARQUEO"
                        tiempo_inicio_parqueo = time.time() 
            except: pass
        time.sleep(0.01)

def procesar_ciclo_completo_lidar():
    """ Algoritmo de guiado proporcional y validación de firmas mecánicas de estacionamiento """
    global dist_derecha_min, dist_izquierda_min, fase_actual, initial_derecha, initial_izquierda
    
    # Cálculo del control proporcional lateralizado
    error_lateral = dist_izquierda_min - dist_derecha_min
    angulo_objetivo = error_lateral * KP_LATERAL
    
    if fase_actual == "CARRERA":
        comando = f"{VELOCIDAD_CRUCERO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())
    elif fase_actual == "BUSCANDO_PARQUEO":
        comando = f"{VELOCIDAD_PARQUEO},{angulo_objetivo:.2f}\n"
        ser_pico.write(comando.encode())
        
        # Validación matemática de firma espacial para frenado seguro
        match_firma_original = abs(dist_derecha_min - initial_derecha) < 80.0 and abs(dist_izquierda_min - initial_izquierda) < 80.0
        if match_firma_original or (time.time() - tiempo_inicio_parqueo > TIMEOUT_BUSQUEDA_PARQUEO):
            fase_actual = "DETENIDO"
            for _ in range(5): ser_pico.write(b"0,0\n")
            apagar_sistema(None, None)

```

### 5.3 Lógica de Control de la Ronda Cerrada (Evasión de Obstáculos)

*Esta sección documentará la arquitectura de visión artificial mediante OpenCV (Segmentación en espacio HSV y cálculo de centroides espaciales) una vez concluidas las pruebas dinámicas de evasión de pilares.*

---

## 6. Capa de Control de Bajo Nivel (Raspberry Pi Pico 2)

### 6.1 Firmware Embebido y Sincronización No Bloqueante

La capa de control inferior ejecuta una arquitectura síncrona no bloqueante sobre MicroPython. El núcleo del sistema utiliza un objeto `select.poll()` registrado sobre el flujo de entrada estándar (`sys.stdin`) para procesar las tramas seriales enviadas por la Raspberry Pi 3B a una frecuencia de ciclo alta sin interferir con los procesos críticos de integración inercial y generación de PWM.

### 6.2 Implementación Matemático-Inercial

Para contrarrestar los efectos dinámicos del subviraje y estabilizar el coche ante irregularidades de la pista o vibraciones estructurales del chasis de LEGO, la Pico 2 ejecuta un bucle de compensación derivativa inercial activa.

La ecuación en lazo cerrado que calcula la posición angular final del servomotor responde a:

$$\theta_{\text{servo}} = 180^\circ + \theta_{\text{objetivo}} - (\omega_z \cdot K_D)$$

Donde:

* $180^\circ$ representa el punto central calibrado por software para la marcha en línea recta del servomotor.
* $\theta_{\text{objetivo}}$ es el ángulo macro de guiado espacial solicitado dinámicamente por el script de la Raspberry Pi 3B.
* $\omega_z$ es la velocidad angular instantánea sobre el eje de rotación vertical (Yaw), obtenida tras sustraer el offset estático de calibración: $\omega_z = \text{Gyro\_Z} - \text{Offset}_z$.
* $K_D$ es la ganancia derivativa de amortiguación inercial calibrada en $0.12$, encargada de absorber momentos angulares bruscos en curvas.

### 6.3 Funciones Maestras de Control Físico

```python
# Módulo de funciones clave extraído de src/pico/main.py

def mover_servo(angulo):
    """ Convierte el ángulo geométrico (0-180) a ciclo de trabajo de hardware (16-bit) """
    angulo = max(0, min(180, angulo))
    # Mapeo lineal para generar los tiempos de pulso correctos del actuador
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

        # Aplicación de ley de control inercial amortiguado
        angulo_servo = 90 + angulo_objetivo - (velocidad_z * KD_ESTABILIDAD)
        
        # Límites estrictos de protección mecánica del chasis Ackermann
        # (Saturación segura: LIMITE_DER = 140, LIMITE_IZQ = 240)
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
        mover_servo(90)
        break

```

---

## 7. Geometría de Dirección y Movilidad Mecánica

### 7.1 Cinemática del Sistema de Dirección Ackermann

El chasis diseñado en *BrickLink Studio* adopta de forma estricta la geometría de dirección tipo **Ackermann**. El principio fundamental de este mecanismo radica en evitar que las ruedas delanteras se deslicen lateralmente al trazar una curva, permitiendo que la rueda interior gire un ángulo mayor que la rueda exterior, ya que describe un radio de curvatura más cerrado respecto al centro instantáneo de rotación (CIR).

La ecuación cinemática que rige las restricciones geométricas y la relación de ángulos de nuestro chasis de LEGO responde a:

$$\cot(\delta_o) - \cot(\delta_i) = \frac{w}{l}$$

Donde:

* $\delta_o$ es el ángulo de orientación de la rueda directriz exterior.
* $\delta_i$ es el ángulo de orientación de la rueda directriz interior.
* $w$ representa el ancho de la vía (*track width*), definido como la distancia transversal entre los pivotes de dirección delanteros.
* $l$ representa la batalla del vehículo (*wheelbase*), que mide la distancia longitudinal entre el eje delantero y el eje de tracción trasero.

### 7.2 Renderizado del Chasis de Producción (V2)

A continuación se presenta el modelo CAD estructural del vehículo libre de actuadores y masa suspendida electrónica, aislando los componentes cinemáticos esenciales para la validación de la rigidez torsional del chasis:

### 7.3 Límites Angulares Calibrados y Protección Mecánica

Para salvaguardar la integridad de las articulaciones, uniones y vigas de LEGO contra esfuerzos de torsión excesivos generados por el servomotor de alta velocidad, se implementaron límites simétricos de saturación estricta por software.

El rango operativo del actuador Geekservo se restringe a los siguientes umbrales de PWM mapeados en la Raspberry Pi Pico 2:

| Ángulo Límite Derecho (Saturación Mínima) | Centro Geométrico Calibrado | Ángulo Límite Izquierdo (Saturación Máxima) |
| --- | --- | --- |
| **$140^\circ$** | **$180^\circ$** | **$240^\circ$** |
| *Restricción estricta ante comandos de giro a la derecha.* | *Alineación de marcha lineal en pista ($0.00\,\text{v}$ de error).* | *Restricción estricta ante comandos de giro a la izquierda.* |

> **Ventaja mecánica de la modularidad LEGO:** La sustitución del filamento impreso en 3D por vigas de fricción LEGO redujo el coeficiente de masa inercial global, disminuyendo drásticamente el subviraje físico provocado por la fuerza centrípeta en las esquinas de la pista de la WRO.
