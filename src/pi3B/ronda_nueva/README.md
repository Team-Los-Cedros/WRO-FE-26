# ronda_nueva

Reescritura independiente de la ronda con obstáculos para la Raspberry Pi 3B.
No modifica `ronda_cerrada/` ni `ronda_camara/`: esta última queda como la
referencia funcional del montaje de cámara en mástil.

## Estado honesto

La arquitectura, percepción, fusión, recorrido, estacionamiento y pruebas
offline están implementados, pero **el robot no debe moverse todavía con esta
carpeta**. El JSON entregado conserva `runtime.motion_enabled=false` y tres
calibraciones dinámicas pendientes:

- `vision_ground_support_ready`: falta un frame/video de la cámara de a bordo
  ya instalada para validar ROI, horizonte y soporte de suelo;
- `camera_lidar_timing_ready`: la óptica se midió con el robot quieto; falta
  medir el desfase cámara–LiDAR en movimiento;
- `parking_ready`: faltan radio de giro, signo y posición final medidos en el
  chasis real.

El punto de entrada llama la barrera de configuración antes de importar GPIO o
abrir un puerto. No basta con cambiar `motion_enabled`: todas las calibraciones
del modo solicitado deben estar aprobadas también.

## Qué cambió respecto a `ronda_camara`

| Tema | `ronda_camara` funcional | `ronda_nueva` |
| --- | --- | --- |
| Cámara | un blob HSV, 640×360, sensor completo | varios blobs, 640×360 a 15 FPS, geometría y suelo debajo |
| Óptica | modelo medido en `optica.py` | mismo modo 16:9: HFOV 68,17°, `c0=352,1` y extrínseca en JSON |
| Color | Picamera `RGB888`, array BGR | formato de captura y orden del array son parámetros separados |
| LiDAR trasero | muta `Medicion` con una máscara | resultado explícito con validez y cobertura; `SIN_DATO` no es libre |
| Pilar | tracker único | asociación cámara–LiDAR uno-a-uno y hasta 8 tracks |
| Esquinas | control reactivo + recuperación | giro determinista por sentido, IMU, reapertura y conteo por eventos |
| Final | `abs(yaw)>=1010` | 12 esquinas confirmadas, sin depender de deriva neta de IMU |
| Parqueo | se detiene al reconocer la firma inicial | detecta dos separadores, alinea, hace dos arcos y verifica geometría |
| Watchdogs | LiDAR | LiDAR, cámara, IMU y watchdog autónomo de consignas en la Pico |

Flujo de un ciclo:

```text
Picamera2 -> VisionLigera --timestamp--+
                                        +-> FusionLigera -> tracks --+
RPLIDAR -> ProcesadorLidar ->            |                          |
           PercepcionLidar --------------+--------------------------+-> ControlRuta -> Pico 2
                 |                                                     |
                 +-> paredes / hueco / trasera válida ----------------+
MPU6050 + TCS3472 (Pico) -> heading / color de sentido ----------------+
```

La escritura CSV ocurre en otro hilo y los buffers conservan resultados de
2–4 frames, nunca una cola de imágenes. Las operaciones por barrido son
lineales o sobre un máximo pequeño de tracks; no hay red neuronal, SLAM ni
asignador cúbico.

## Datos ya medidos del chasis nuevo

La carpeta `ronda_camara` midió el montaje el 2026-08-28:

- cámara derecha, sin rotación de 180°;
- Picamera2 solicita `RGB888`, pero el array usado por OpenCV está en BGR;
- modo raw 2304×1296 y salida 640×360 para conservar el sensor completo;
- focal medida 472,9 px, HFOV efectivo 68,17° y centro óptico `cx=352,1`;
- cámara aproximadamente 99,8 mm detrás y 7,0 mm a la derecha del LiDAR;
- el mástil produce eco propio en 165–190°; la máscara usa 163–191°;
- la distancia posterior se recupera con hombros 145–162° y 198–215°.

Las cinco fotos de `ronda_camara/webcam/imagenes` confirman físicamente que el
mástil, soporte y cable plano cruzan el plano trasero del C1. El video externo
`WIN_20260828_21_38_27_Pro.mp4` muestra unas tres vueltas y correcciones en S,
con pasos de poco margen alrededor de 17–19, 37–39, 57–59, 93–95, 107–109 y
163–165 s. Esa grabación es cenital y **no** contiene el feed de la cámara de
a bordo, por lo que no calibra HSV, ROI ni latencia.

## Recorrido y pilares

`ControlRuta` mantiene un único estado activo y da prioridad a la seguridad:

1. fija el sentido por TCS3472 (`AZUL` = izquierda/antihorario, `NARANJA` =
   derecha/horario), o por configuración explícita;
2. centra por rectas laterales robustas y usa el rumbo de pared solo si tiene
   calidad suficiente;
3. entra a una esquina por `frontal_muro`, no por un pilar que tape el frente;
4. cuenta la esquina únicamente tras cambio de heading y reapertura frontal;
5. asocia cada color con un cluster corrigiendo el paralaje de la cámara;
6. para verde pasa por la izquierda y para rojo por la derecha, conserva el
   rumbo durante el sobrepaso y vuelve al centro por posición;
7. una emergencia intenta una reversa corta solo con cobertura trasera real.

El *slew limiter* se aplica a velocidad y dirección, salvo que detenerse sea
urgente. Los límites físicos siguen siendo asimétricos: +25° izquierda y −20°
derecha.

## Estacionamiento

El detector LiDAR busca dos segmentos con la geometría esperada de las paredes
magenta de 200 mm; no intenta inferir su color. Sus centros deben estar a
aproximadamente 353 mm: 333 mm de hueco útil más 20 mm de espesor. La FSM:

```text
SEARCH_GAP -> ALIGN -> ARC_IN -> ARC_OUT -> CENTER -> VERIFY -> DONE
```

Las correcciones geométricas importantes son:

- `ALIGN` coloca el eje trasero, no el LiDAR, respecto al separador;
- el LiDAR está unos 77 mm por delante del centro geométrico del robot, por lo
  que el objetivo centrado es `trasera - frontal = 154 mm`, no cero;
- `frontal + trasera` debe concordar con los 333 mm del hueco;
- la distancia al muro exterior y el paralelismo también deben aprobarse;
- `DONE` necesita tres barridos distintos; un timeout nunca cuenta como éxito;
- un eco de 92 mm o una trasera sin cobertura inhiben toda reversa;
- `ARC_IN` y `ARC_OUT` exigen una distancia lateral real y las dos diagonales
  traseras con cobertura suficiente: un `SIN_DATO` detiene el arco hasta el
  timeout y una holgura crítica termina en `FAILED`, nunca en recuperación.

Los umbrales conservadores están separados en `parking.minimum_*`. Las
coberturas axial y de cada diagonal se guardan también en la telemetría para
que la calibración física no tenga que inferir si un valor grande fue un eco
real o el centinela del C1.

Estos cálculos son verificables offline, pero los arcos aún requieren medir el
radio efectivo del chasis. Por eso `parking_ready` permanece en `false`.

## Pruebas y replay en laptop

Desde la raíz del repositorio:

```bash
python -m compileall -q src/pi3B/ronda_nueva
python -m unittest discover -s src/pi3B/ronda_nueva/tests -v
python -m src.pi3B.ronda_nueva.ronda_nueva --validar-config
```

Replay de imágenes o video. OpenCV decodifica archivos como BGR:

```bash
python -m src.pi3B.ronda_nueva.replay_vision RUTA_A_FRAMES --cada 3
```

Replay sincronizado del formato `captura_*/`:

```bash
python -m src.pi3B.ronda_nueva.replay_captura RUTA_CAPTURA --sentido LEFT
```

Captura estatica en la Raspberry, sin abrir la Pico ni enviar movimiento. La
ruta debe ser nueva; `--solo-camara` tampoco abre ni gira el LiDAR:

```bash
python3 -m ronda_nueva.capturar_calibracion \
  /home/pi/captura_ronda_nueva_YYYYMMDD_HHMMSS --duracion 30
python3 -m ronda_nueva.capturar_calibracion \
  /home/pi/captura_solo_camara_YYYYMMDD_HHMMSS --duracion 5 --solo-camara
```

Para mantener la prueba libre de consignas, `imu.csv` contiene cero sintetico
y `meta.json` lo declara como `synthetic_zero_no_pico`. Sirve para percepcion
estatica y replay en sombra, pero no para aprobar la latencia en movimiento.

El replay estructurado usa únicamente archivos: sincroniza cada barrido con la
última IMU y el último frame que ya existían en ese timestamp, recorre
percepción, fusión, ruta y parqueo en modo sombra y puede exportar las consignas
a CSV. No importa GPIO/Picamera2/serial ni envía comandos a ningún dispositivo.

La captura local de 2026-08-04 pertenece al montaje anterior: sirve para medir
coste y comprobar el reloj relativo, pero debe fallar el diagnóstico del mástil
nuevo. Eso es evidencia de incompatibilidad, no un motivo para relajar la
máscara.

## Despliegue separado

En la Pi, desde un clon del repositorio:

```bash
DESTINO_NUEVO=/home/pi/wro_nueva_20260829
bash src/pi3B/ronda_nueva/deploy.sh --dry-run "$DESTINO_NUEVO"
bash src/pi3B/ronda_nueva/deploy.sh "$DESTINO_NUEVO"
cd "$DESTINO_NUEVO"
python3 -m ronda_nueva.ronda_nueva --validar-config
```

`--dry-run` hace un preflight sin crear archivos. El despliegue real copia
`ronda_nueva/` y `comun/` como paquetes a un staging vecino, comprueba la
sintaxis y solo entonces lo promueve. Si el destino ya existe —incluso vacío o
como enlace— se niega a escribir; para otra versión hay que elegir otro nombre.
No reemplaza el ejecutable funcional ni cambia `controlador_inicio.py`. Cuando
todas las calibraciones estén aprobadas, una prueba de recorrido sin parqueo se
lanza con `--sin-parqueo`; el modo oficial no usa esa opción.

Este despliegue solo prepara la aplicación de la Pi. No flashea ni modifica la
Pico 2; el firmware seguro se instala por separado y debe anunciar `WD:OK` para
que `ronda_nueva` permita armar la tracción.

## Secuencia de validación física pendiente

No realizarla mientras se modifica el chasis.

1. Flashear juntos `src/pico/main.py` y `src/pico/protocolo_seguro.py`; comprobar
   con las ruedas levantadas que al retirar USB la Pico frena y centra en 500 ms.
2. Motores sin armar: capturar 30–60 s de cámara de a bordo y LiDAR, con un
   pilar rojo y uno verde centrados y descentrados.
3. Validar orientación, suelo debajo del blob, FOV y que el diagnóstico de
   163–191° pase varios barridos seguidos.
4. Medir latencia en movimiento lento y ajustar `camera.latency_s`/gate.
5. Habilitar solo recorrido, a velocidad reducida, y revisar el CSV por estado.
6. Medir radio de ambos giros y ejecutar el parqueo con ruedas libres o zona
   despejada antes de probar dentro de los separadores.
7. Solo entonces marcar `parking_ready=true` y `motion_enabled=true` en una
   copia de configuración versionada con la fecha de la medición.

## Sesión de pista 2026-08-31: el radio de giro es el bloqueante

Tres corridas con motores (`--sin-parqueo`, 25 PWM) sobre la pista con los
ocho pilares montados. Los CSV están en el `logs_prueba_*` de cada
despliegue y el video cenital en `video/video-drafts/`.

**Lo que quedó resuelto.** El *fallback* de rumbo funciona: en la corrida
`143903` `RECENTER` terminó por primera vez en «reincorporación
verificada» en vez de agotar su timeout. Las tres corridas mantuvieron
`WD:OK` en el 100 % de los ciclos y la edad de visión se quedó en
67–72 ms de mediana (p95 ≤ 85 ms), así que ni el enlace con la Pico ni la
cámara son cuello de botella.

**La medida que importa.** Con el servo en su tope y 22–23 PWM:

| Magnitud | Valor medido |
| --- | --- |
| Velocidad lineal | 150 mm/s (cierre de `frontal_muro` en recta) |
| Giro máximo a la izquierda | 14,4 °/s |
| Giro máximo a la derecha | 14,1 °/s |
| **Radio de giro mínimo** | **≈ 600 mm, en ambos sentidos** |
| Ángulo de rueda implícito | 12,7° (Ackermann, batalla 136 mm) |

**Esto corrige la hipótesis anterior.** La bitácora del 28-08 atribuía los
fallos a que el robot «no puede hacer las curvas a derechas» por la
asimetría 25°/20° del servo. Los datos dicen otra cosa: izquierda y
derecha giran prácticamente igual (14,4 contra 14,1 °/s, un 2 % de
diferencia). El problema no es la asimetría sino que **ambos lados giran
demasiado poco**: se comandan 20–25° de servo y la rueda solo alcanza
unos 12,7°, es decir, algo más de la mitad del ángulo pedido. La pérdida
está en la relación varilla/horn, no en el firmware ni en el signo.

**Por qué ninguna calibración lo salva.** Un carril WRO mide 1000 mm, así
que una esquina de 90° necesita un radio de unos 400–500 mm. Con 600 mm
el arco no cabe. Se probaron dos calibraciones muy distintas y las dos
fallan por la misma geometría:

- `corner_front_trigger_mm` 650 y `corner_timeout_s` 3,2: el giro se corta
  a los 39° porque el timeout solo da para eso.
- `corner_front_trigger_mm` 900 y `corner_timeout_s` 8,0 (valores que la
  medición justifica): el robot llega a girar 110°, pero `frontal_muro`
  cae de forma monótona de 906 mm a 282 mm y **nunca** vuelve a abrirse a
  los 780 mm que exige la salida. El arco lo lleva contra la pared en vez
  de rodearla; el video cenital muestra al robot cruzando el carril
  entero en lugar de girar dentro de él.

Los valores nuevos (900 mm y 8,0 s) quedan en `configuracion.json` porque
son los que la medición respalda, pero conviene leerlos como «necesarios
y todavía no suficientes».

**Qué desbloquea esto.** Subir el ángulo real de rueda de 12,7° a ~18°
(radio 400 mm) es trabajo mecánico: alargar el brazo del horn del servo o
acortar el del muñón para ganar recorrido, y comprobar que ningún tope
físico esté recortando el giro antes que el firmware. Si el chasis no da
más, la alternativa por software es una maniobra de esquina en tres
tiempos (avanzar girando, retroceder al contrario, avanzar), que el
reglamento no prohíbe pero cuesta segundos de ronda. Conviene medir el
ángulo de rueda con un transportador antes de decidir.

## Cierre real de la sesión: tres corridas seguidas sin fallo terminal

| Corrida | Esquinas | Emerg. | Sueltas | Fin |
| --- | --- | --- | --- | --- |
| 18:22 (soltar pilar) | 5 | 0 | 0 | **no falló** |
| 18:39 (desde la curva) | 4 | 39 | 0 | **no falló** |
| 18:46 (herencia de color) | 4 | 22 | 1 | **no falló** |

Antes de estos dos arreglos, **dos de cada tres corridas morían por
timeout a los 46-50 s**. Ahora ninguna de las tres muere: las tres llegan
al final de la ventana de prueba. El techo bajó de 6 a 4-5 esquinas, pero
el suelo subió, que era exactamente lo que se buscaba.

Dos comprobaciones directas de que los arreglos actúan:

- En la corrida `184623` se ve el ciclo completo en el CSV: a los 39,7 s
  «track bloqueado perdido; parada para reasociar» y a los 40,5 s «pilar
  no reasociado; se suelta y sigue el carril». **0,8 s de espera en vez de
  los 9,7 s** que antes acababan en `FAILED`.
- La última observación directa de un track llegó a **y = 8 mm**, contra
  los 63-70 mm típicos de antes: la herencia de color mantiene el pilar
  identificado hasta pegado al morro.

Lo que sigue abierto es un caso distinto del punto ciego: **tracks que se
pierden lejos**, a 470 y 774 mm en esta misma corrida. Algunos coinciden
con giros fuertes (13-15°/s) pero otros no, así que la hipótesis del error
de predicción durante el giro no los explica todos. Hace falta
instrumentar la fusión para verlo; los CSV de control no bastan.

## Estado al cierre de la sesión del 2026-08-31

Mejor corrida: **6 esquinas, 6 pilares rebasados, cero emergencias**, 86,7 s
hasta que la cortó el timeout externo de la prueba (`161726`). Es media
ronda —12 esquinas son las tres vueltas— con las cuatro reincorporaciones
verificadas y ningún `RECOVERY`.

| | inicio de sesión | cierre |
| --- | --- | --- |
| Esquinas | 0 | **6** |
| Emergencias | 8–196 | **0** |
| Duración antes de fallar | 14 s | 86,7 s (sin fallar) |

Ritmo medido: 13,8 s por esquina, que extrapolado a las doce da **164 s**
contra el límite de 180. Cabe, pero con poco margen: cualquier maniobra de
tres tiempos o recuperación extra se lo come. Ese margen es lo que
compraría bajar el radio de giro.

Aviso de método, que esta misma sesión demostró: **es una sola corrida**.
La repetición de una configuración dio 2 y 0 esquinas por 41 mm de
colocación inicial (más abajo). Antes de dar por buena cualquier cifra
hacen falta tres corridas.

## Por qué la visión sale a 640×360 y no al tamaño del sensor

Pregunta recurrente, medida el 2026-08-31 en la propia Pi 3B con un frame
real de a bordo y el pipeline completo (`VisionLigera.procesar`):

| Modo | ms/frame | fps máximo | coste | pilares detectados |
| --- | --- | --- | --- | --- |
| **640×360** | **34,0** | 29,4 | 1,0× | 2 |
| 1280×720 | 129,5 | 7,7 | 3,8× | 2 |
| 2304×1296 | 405,1 | 2,5 | 11,9× | 2 |

El presupuesto a 15 fps es de 66,7 ms por frame, y ese presupuesto se
comparte con el LiDAR, la fusión y el control. A 640×360 el pipeline usa
la mitad y sobra margen; a 2304×1296 tarda **seis veces** más de lo que
hay, y el ritmo real caería a 2,5 fps. La edad de visión pasaría de los
~70 ms medidos a más de 400: a 150 mm/s eso son 60 mm de desplazamiento
entre frames en vez de 10, con el control ciego entre medias.

Y no se gana nada a cambio: **detecta los mismos dos pilares en las tres
resoluciones**.

Conviene entender por qué no se está perdiendo campo de visión. El sensor
**ya lee a 2304×1296** (`raw_sensor_size`), que es el fotograma completo;
ese modo se eligió el 28-08 justamente para no recortar. El reescalado a
640×360 lo hace el ISP del chip por hardware, sin coste de CPU, y además
promedia píxeles, lo que reduce ruido. Lo único que cambiaría subiendo la
salida es cuántos píxeles recorre OpenCV en la CPU.

Dicho de otro modo: el campo de visión ya es el máximo, y el punto ciego
cercano de la sección siguiente es **geométrico**, no de resolución — con
cuatro veces más píxeles la base del pilar seguiría cayendo exactamente
igual fuera del encuadre.

## Punto ciego a corta distancia: el LiDAR ve y la cámara no

El atasco de la cuarta esquina (corrida `154741`, segundo 67) es un bucle
`TURN` → `RECOVERY` → `CRUISE` → `TURN` repetido cuatro veces. El CSV lo
explica con dos columnas:

- en `TURN`, `frontal == frontal_muro` (596 = 596): lo más cercano es la
  pared;
- en `RECOVERY`, `frontal = 116` pero `frontal_muro = 597`: **hay un
  objeto a 12 cm que no es pared**;
- y a la vez `tracks = 3`, `tracks_confirmados = 0`.

O sea: el robot tiene un pilar pegado al morro, el LiDAR lo ve, y **no
sabe de qué color es**. Sin color no hay regla de evasión que aplicar, así
que el pilar solo existe como disparador de emergencia. `TURN` gira, el
pilar entra en el sector frontal, salta la emergencia, `RECOVERY`
retrocede, el pilar sale, y vuelta a empezar.

### Dónde empieza el punto ciego

Medido sobre un frame real de a bordo (captura `pilar_verde_160314`,
robot quieto, sin abrir la Pico), usando la óptica ya calibrada
(f = 472,9 px) y los dos pilares del encuadre como referencias:

| Pilar | Alto aparente | Distancia al LiDAR | Base en el frame |
| --- | --- | --- | --- |
| verde | 125 px | **657 mm** | 0,556 |
| rojo | 67 px | 1312 mm | 0,410 |

Los 657 mm calculados coinciden con los ~60 cm medidos a mano, lo que
confirma de paso que la óptica está bien. Ajustando `y = 0,241 + 237,8/d`
y despejando en el recorte inferior de la ROI (`roi_bottom_ratio` 0,92):

> **La base del pilar sale del encuadre a unos 251 mm del LiDAR.**

Por debajo de esa distancia no hay suelo bajo el blob, y `min_ground_support`
—la comprobación que se adoptó del campeón 2025 para descartar falsos
positivos— lo rechaza. La comprobación es correcta; simplemente nadie
había medido a partir de qué distancia empieza a rechazar pilares buenos.

A 657 mm la detección es sólida: el replay en sombra confirmó el color en
63 de 64 barridos.

### La incoherencia que esto destapa

`obstacle_pass_y_mm` vale 160 mm: el robot considera que «ya está
rebasando» el pilar cuando lo tiene a 16 cm. Pero **lo pierde de vista a
25 cm**. Hay una franja de 9 cm en la que la maniobra depende de un color
que ya no se está midiendo.

Dos arreglos, complementarios:

1. **Subir el umbral de sobrepaso por encima del punto ciego** (~260 mm),
   para congelar el rumbo mientras el dato todavía es válido en vez de
   perseguir un punto de paso con información que ya no existe. Es
   calibración, no código.
2. **Conservar el color del track cuando el LiDAR lo sigue viendo pero la
   cámara ya no.** La fusión ya acumula votos por track; falta que el
   track no se dé por perdido y se recree en blanco al entrar en la zona
   ciega. El lado de paso se seguiría recalculando con la posición actual,
   no congelado.

## La causa raíz: el punto de paso no cabía en el hueco

Todo lo que sigue en esta bitácora —la guardia de pared secuestrando el
mando, el robot llegando en diagonal a la esquina, la falta de
repetibilidad— sale de un solo parámetro mal calibrado.

El tramo inferior deja **333 mm de la pared al centro del pilar verde**,
medido por el LiDAR (la suma `izquierda + track_activo_x` converge a ese
valor en dos corridas independientes). Descontando el medio pilar quedan
308 mm libres para un robot de 125 mm de ancho.

`obstacle_lateral_clearance_mm` estaba en **255 mm**, que pide 76 mm más
de los que existen:

| Con 255 mm | Con 179 mm |
| --- | --- |
| punto de paso a 78 mm de la pared | a 154 mm |
| borde del robot a **16 mm** — no cabe | a 92 mm, centrado en el hueco |

El robot nunca llegaba a ese punto imposible: la guardia de pared lo
frenaba antes de tocar. **La guardia llevaba toda la sesión compensando
un objetivo que no existía**, y ese rescate era justo lo que lo empujaba
de vuelta hacia el pilar. Eso explica también por qué quitarle el término
lateral (más abajo) salió tan mal: se quitó el parche sin arreglar la
causa que lo hacía necesario.

### Resultado en pista

| | clearance 255 | clearance 255 (rep.) | **clearance 179** |
| --- | --- | --- | --- |
| Paso junto al pilar | 130 mm | 147 mm | **234 mm** |
| Esquinas | 2 | 0 | **3** |
| Fin | timeout | timeout | **no falló** |

La corrida con 179 mm corrió **71,5 s sin fallar**, hasta que la cortó el
timeout externo de la prueba; las anteriores morían entre 14 y 45 s. Las
dos reincorporaciones se verificaron y las tres esquinas se cerraron.

Y hay un efecto de segundo orden que conviene notar: pasando a 234 mm de
la pared, por encima de `wall_guard_start_mm` (230), **la guardia ya no
llega a activarse durante la evasión**. El lado de paso vuelve a decidirlo
el color por sí solo, sin necesidad de tocar la lógica de la guardia.
Arreglada la causa, el síntoma desaparece.

Queda abierto el atasco de la cuarta esquina, hacia el segundo 67: entra
en un ciclo `TURN` → `RECOVERY` del que no sale. Ese caso todavía no
tiene diagnóstico.

## El lado de paso lo decide el color, no la pared

Diagnóstico del equipo durante la sesión, confirmado después con los CSV:
el lado por el que se rebasa un pilar lo acababa decidiendo el LiDAR.

`_con_guardia_pared` mezcla el ángulo de la evasión con un «protector» de
pared cuyo peso crece al acercarse. En la corrida `145857` ese peso llegó
a **0,94**: el robot iba a −18°, girando *hacia* el pilar que estaba
esquivando, mientras la pared izquierda se le acercaba a 131 mm.

No era un caso extremo sino la regla. La geometría lo obliga:

| | |
| --- | --- |
| Carril | 1000 mm |
| Pilar | 50 mm |
| Robot | 125 mm |
| **Margen a la pared al rebasar** | **≈175 mm** |
| `wall_guard_start_mm` | 230 mm |

Rebasar un pilar deja siempre menos margen del que dispara la guardia, así
que ésta intervenía en **toda** evasión normal, no como excepción.

La causa concreta es el término lateral del protector, que devuelve el
robot al centro del carril; durante una evasión ese centro está al otro
lado del pilar. Ahora, mientras hay un pilar activo y la holgura no es
crítica, el protector conserva **solo el término de rumbo**: la pared
corrige la orientación para no chocar, pero no reabre una decisión que es
del color. Por debajo de `wall_guard_full_mm` recupera toda su autoridad,
porque ahí manda no chocar.

Bajar el umbral de la guardia (230 → 165) se probó primero y **no es la
solución**: cierra las esquinas pero el robot pasa a rozar la pared a
111 mm, a 21 mm de la emergencia lateral.

**La pista refutó el cambio y se revirtió.** Repetido desde la línea de
salida, dejar solo el término de rumbo dio **cero esquinas** frente a las
dos de la mejor configuración: sin el término lateral el robot llega al
recentrado pegado a la pared y agota su timeout antes de la primera
esquina. Ese término es también lo que impide pegarse, así que se
conserva; el desvío hay que corregirlo antes, en el punto de paso, no
quitándole autoridad a la pared. Bajar el umbral (230 → 165) tampoco
sirve: cierra esquinas pero roza a 111 mm, a 21 mm de la emergencia.

Queda un test que fija el término lateral en su sitio, para que el
intento no se repita sin leer antes esta bitácora.

### Lo que sí resolvió el caso

Persiguiendo esto apareció la causa real de los timeouts de
reincorporación: **el criterio de «centrado» exigía calidad de ajuste de
pared**. Tras rebasar un pilar esa calidad cae (0,21 medido), y el robot
se quedaba centrado —108 mm de error, dentro de la tolerancia de 150— sin
poder confirmarlo nunca. Estar centrado es una afirmación sobre
distancias medidas, no sobre lo bien que se ajustó una recta; la calidad
sigue gobernando el mando en `_angulo_pared`, que es donde importa.

| Configuración (desde la misma salida) | Esquinas | Margen | Emerg. |
| --- | --- | --- | --- |
| handoff 900, guardia 230/125 | 1 | 131 mm | 0 |
| handoff 700, guardia 165/110 | 2 | 111 mm | 8 |
| handoff 700, guardia solo-rumbo | 0 | 127 mm | 0 |
| handoff 700, guardia 230/125 | 0 | 138 mm | 0 |
| handoff 700, 230/125 + centrado por laterales | 2 | 130 mm | 0 |
| **la misma, repetida sin tocar nada** | **0** | **103 mm** | **90** |

## El resultado no es repetible, y eso manda sobre la tabla

La última fila es la medición más importante de la sesión. Es la **misma
configuración, desde la misma salida, sin cambiar una sola línea**, y da
el resultado opuesto: cero esquinas y noventa ciclos de emergencia frente
a dos esquinas y ninguna.

Lo único que difiere es la colocación manual de partida:

| | corrida A | corrida B |
| --- | --- | --- |
| izquierda | 449,7 mm | 408,1 mm |
| derecha | 530,9 mm | 570,7 mm |
| Resultado | 2 esquinas | 0 esquinas |

**41 mm de desplazamiento lateral inicial** —un 4 % del carril— deciden si
la vuelta progresa o se atasca. Y el margen a la pared tampoco explica
nada: la corrida con **menos** margen (111 mm) cerró dos esquinas y una
con **más** (138 mm) no cerró ninguna.

La conclusión es incómoda pero clara: con un radio de giro de 600 mm en
un carril de 1000, el sistema corre pegado al límite de su envolvente
física, y ahí la varianza domina sobre cualquier ajuste de parámetros.
Seguir afinando umbrales es perseguir ruido. **La siguiente mejora real
es mecánica** —subir el ángulo de rueda de 12,7° a ~18°— y hasta que
llegue, cualquier tabla comparativa de configuraciones que se mida con
una sola corrida por fila estará midiendo azar.

Nota de método: en una versión anterior de esta tabla el margen de la
quinta fila se anotó como 214 mm. Era un error de lectura —ese valor era
la distancia lateral en un instante concreto, no el mínimo del tramo—; el
mínimo real es 130 mm y todas las filas se recalcularon con el mismo
criterio.

## Maniobra de esquina en tres tiempos

Puente por software mientras el chasis no gane ángulo de rueda. Vive
dentro del estado `TURN` (sin estados nuevos en la FSM) y se apaga con
`corner_kturn_enabled`, que es lo que convendrá hacer si el radio baja a
400 mm.

El detalle que la hace funcionar es el signo del volante: con Ackermann
la rotación es `omega = v*tan(delta)/L`, así que **al retroceder el
volante va al lado contrario** para que el morro siga rotando hacia el
mismo lado. Usar el mismo ángulo que en avance desharía el giro. Hay una
prueba que fija ese signo en los dos sentidos de vuelta.

La reversa exige lo mismo que la recuperación: trasera válida y con
holgura. Un `SIN_DATO` detiene la maniobra en vez de ejecutarla a ciegas,
porque el mástil ya ciega ese arco.

### Lo que enseñó la pista

| Corrida | Esquinas | Conmutaciones | Ciclos en reversa | Fin |
| --- | --- | --- | --- | --- |
| `145409` (primera versión) | 2 | 12 | 38 | timeout de recuperación |
| `145857` (con histéresis) | 1 | 2 | 39 | timeout completando esquina |

La primera prueba cerró dos esquinas, pero los tramos 2 al 6 fueron
oscilación pura: conmutaba cada 0,2 s y el robot no llegaba a desplazarse
porque el *slew* de velocidad nunca alcanzaba el PWM pedido. **La causa no
era ruido del sensor**: en la esquina el sector trasero cruza la arista
entre dos paredes y la lectura alterna entre dos valores reales, 258 y
690 mm, los dos con `trasera_valida` y cobertura sobre 0,97. El umbral de
confort de 300 mm caía justo entre ambos modos.

Ahora un tramo empezado solo lo corta el límite duro
(`emergency_rear_mm`, que 258 mm no viola) o su duración mínima; la
holgura de confort decide únicamente si un tramo **nuevo** puede empezar.
Con eso las conmutaciones bajan de 12 a 2 y el tramo útil pasa de rebotes
de 0,2 s a uno continuo de 3,9 s.

### Lo que sigue abierto

La esquina 1 se cierra de una sola pasada en las dos corridas, sin
necesitar la maniobra. La que falla es la **segunda**, y por una causa
distinta del radio: el pilar de esa esquina deja al robot descolocado y
`RECENTER` cede el mando a `TURN` 0,1 s después de entrar, con
`recenter_corner_handoff_mm` en 900 mm. Llega en diagonal y termina
encajonado con unos 250 mm por delante y 240 mm por detrás: ahí ya no
cabe ninguna maniobra. Bajar el *handoff* para que se reincorpore antes
de girar choca con haber subido el disparo de esquina a 900 mm por el
radio; las dos cosas no se pueden tener a la vez con este chasis.

El `corner_timeout_s` de 14 s tampoco es viable en competencia: doce
esquinas no caben en los 3 minutos de ronda. La maniobra sirve para
seguir probando el resto del recorrido, no como configuración de
carrera.

## Watchdog autónomo de la Pico

`src/pico/main.py` usa ahora `src/pico/protocolo_seguro.py`: solo acepta tramas
acotadas y, si pasan 500 ms sin una consigna válida, frena, centra el servo y
publica `WD:STOP`. La aplicación exige observar `WD:OK` antes de armar y durante
la carrera, conservando compatible el parser histórico de IMU/color.

El 2026-08-29 se cargó el firmware en la Pico real y se verificaron los hashes:
con heartbeat `0,0` anunció `WD:OK` y, al dejar de transmitir sin desconectar el
cable, volvió a `WD:STOP` en 465 ms. No se envió velocidad distinta de cero.
Todavía falta la comprobación complementaria desconectando físicamente el USB
con las ruedas levantadas. Copiar `ronda_nueva` a la Raspberry no actualiza
automáticamente el firmware que ya esté en la Pico.

Las fuentes internacionales estudiadas y las decisiones de portabilidad están
en [`REFERENCIAS_2025.md`](REFERENCIAS_2025.md).

## El sentido detectado NO es el que llevábamos configurado

Primera corrida real en `AUTO` (`185856`). El robot avanzó **11,6 s
buscando la línea**, centrado por las paredes, y al cruzarla fijó el
sentido:

| | |
| --- | --- |
| Primer color de línea leído | **AZUL** a los 11,58 s |
| Sentido fijado | **ANTIHORARIO (LEFT)** |
| Giro ejecutado | +25° de servo, heading −4,8° → +67,3° |

Todas las corridas del día se hicieron con `RIGHT` escrito a mano. Si la
lectura es correcta, **íbamos en el sentido contrario al que marca la
pista**, y eso invalida como referencia buena parte de lo medido hoy en
esquinas.

Hay un segundo efecto, y no es menor. Los topes de dirección son
asimétricos —+25° a la izquierda contra −20° a la derecha— así que el
sentido cambia el radio de giro:

| Sentido de giro | Velocidad angular | Radio implicado |
| --- | --- | --- |
| Izquierda (+25°) | 15,4 °/s | **559 mm** |
| Derecha (−20°) | 13,9 °/s | 617 mm |

Girando a la izquierda el robot cierra **58 mm más de radio**. Es la
misma diferencia que llevamos todo el día intentando ganar por software.
Ojo: la cifra de la izquierda sale de **una sola esquina**, así que hay
que repetirla antes de darla por buena.

Queda por confirmar que la convención `AZUL → LEFT` es la correcta para
esta pista; si estuviera invertida se corrige en `floor_color_left` y
`floor_color_right` sin tocar código. Y los 11,6 s de búsqueda son
demasiados: el timeout está en 12, así que faltó poco para fallar, y en
competencia son 11,6 s de los 180 gastados antes de empezar.

## El sentido detectado depende de la pose de arranque, no de la pista

Cinco corridas en `AUTO`, y el patrón no admite dudas:

| Pose de arranque | 1.ª línea | Sentido fijado | Esquinas |
| --- | --- | --- | --- |
| Recta inferior (×4) | AZUL | ANTIHORARIO | **1, 1, 1, 1** |
| Carril izquierdo | NARANJA | HORARIO | **3** |

La detección **funciona y es repetible**: cuatro veces seguidas leyó lo
mismo desde la misma pose. Y en el carril izquierdo el vídeo confirma que
el robot avanzaba hacia arriba, que es sentido horario — exactamente lo
que dedujo de la línea naranja. Sensor y realidad coinciden.

Lo que esto destapa es conceptual: **el robot no detecta el sentido de la
pista, detecta el sentido en el que lo han colocado**. Cruzar primero azul
o naranja depende de hacia dónde apunta al arrancar. Si la orientación
física y el sentido deducido no concuerdan, el robot gira hacia el lado
contrario del que avanza y se mete contra el bloque o la pared.

Esa es la explicación más probable de los cuatro fallos desde la recta
inferior: allí se colocó siempre con la orientación que veníamos usando
para recorrer en horario, pero la línea que cruzaba le decía antihorario.

**Prueba pendiente que lo resolvería:** colocar el robot en la recta
inferior apuntando al lado contrario del habitual, de modo que su marcha
física sea antihoraria. Si entonces lee AZUL y completa varias esquinas,
queda confirmado que el problema era la incoherencia entre pose y sentido,
no la detección ni el control.

## El eco de la dirección era el bloqueante del sentido antihorario

Corriendo en el sentido correcto (antihorario), el robot moría siempre en
la primera esquina. La causa no era el sentido: **al girar, la rueda entra
en el barrido lateral del LiDAR**.

El perímetro está a 61 mm por la izquierda y 45 por la derecha del eje del
LiDAR, así que cualquier lectura por debajo de eso es físicamente
imposible. Contando sobre todas las corridas del 31-08 y el 01-09:

| Lado | Lecturas imposibles | Ángulo mediano del servo |
| --- | --- | --- |
| Izquierda (<80 mm) | 101 | **+17,3°** (girando a la izquierda) |
| Derecha (<65 mm) | 90 | **−8,0°** (girando a la derecha) |

La correlación entre el lado imposible y el lado hacia el que gira el
servo no deja mucho margen de duda. `wall_side_min_mm` ya protegía el
ajuste de rectas, pero cuando no hay recta el lateral cae al mínimo crudo
del sector, que no pasa por ese filtro. En la corrida `124629` la lateral
izquierda marcó 51, 50 y 49 mm en tres `RECOVERY` seguidos, con el robot
girando y sin nada a su lado.

Resultado de descartar como sin dato cualquier eco más cercano que el
propio chasis, en el sentido antihorario:

| Corrida | Esquinas | Duración | Emergencias | Lecturas imposibles |
| --- | --- | --- | --- | --- |
| 18:58 – 12:46 (seis corridas) | 0–1 | 21–46 s | 30–153 | 0–58 |
| **12:52 (con filtro)** | **5** | **143,7 s** | **42** | **0** |

De una esquina a cinco, y de morir a los 40 s a llegar a 144. Once pilares
rebasados por el camino.

**Lo que limita ahora es el ritmo, no los atascos:** 24 s por esquina, que
a doce esquinas son 288 s contra los 180 de una ronda. El tiempo se va en
las evasiones, no en las esquinas, y la velocidad configurada para las
pruebas es de 25 PWM (unos 100 mm/s). Subirla es el siguiente paso, ya en
terreno de afinado y no de depuración.

## Intento de subir la velocidad: fallido, revertido

Con el eco lateral filtrado, el límite pasó a ser el ritmo: 24 s por
esquina son 288 s a doce, contra los 180 de una ronda. Subir el crucero de
25 a 35 PWM no funcionó, y el porqué es instructivo.

**La velocidad no rompió nada nuevo, hizo insostenible algo que ya estaba
justo.** El sector frontal pasaba de 760 a 30 mm en un solo barrido de
0,1 s, cuando a esa velocidad el robot avanza 12: el pilar que hay tras la
esquina no se acercaba, aparecía. Está fuera del campo durante todo el
giro. A 25 PWM había margen de reacción; a 35 la emergencia llegaba con el
pilar tocando el morro.

Se probaron dos arreglos y **ninguno sirvió**:

- *Salida lenta de esquina* (1,5 s a velocidad de evasión tras cada giro).
  No bastó: el siguiente fallo fue lateral, con el eco de la rueda a
  91-107 mm, por encima del umbral de 80 que lo filtraba.
- *Filtro de continuidad lateral*, rechazando saltos físicamente
  imposibles (832 → 101 mm en un barrido). Descartaba la lateral derecha
  desde el primer ciclo y la corrida moría a los 12 s. **Revertido.**

Subir el umbral del fallback de 80 a 160 mm tampoco ayudó: con esa
configuración la corrida hizo las mismas cinco esquinas pero con **136
emergencias frente a 42**, y 30 s por esquina en vez de 24.

La configuración volvió a la de la corrida `125223`, que sigue siendo la
mejor medida: crucero 25 PWM, `lateral_fallback_min_mm` 80, sin salida
lenta de esquina.

**Lo que esto enseña sobre el orden de trabajo:** el pilar oculto tras la
esquina hay que resolverlo *antes* de tocar la velocidad, no después. Y
mientras el eco de la rueda no se distinga de una pared por algo mejor que
un umbral de distancia —lo natural sería enmascarar el sector en función
del ángulo del servo, que el robot conoce— cada subida de velocidad va a
volver a chocar con él.

## Serie de cinco: la primera medida que merece ese nombre

Cinco corridas seguidas, misma configuracion, sin tocar nada entre ellas.
Es lo que faltaba en todo lo anterior, donde cada decision se tomaba sobre
una sola corrida.

| Run | Esquinas | Vueltas | Duración | Emerg. | Fin |
| --- | --- | --- | --- | --- | --- |
| 1 | 7 | 1,8 | 171,6 s | 11 | no falló |
| 2 | 6 | 1,5 | 171,8 s | 172 | no falló |
| 3 | **8** | **2,0** | 176,9 s | 86 | no falló |
| 4 | 4 | 1,0 | 81,7 s | 0 | timeout reincorporación |
| 5 | 6 | 1,5 | 106,5 s | 69 | timeout recuperación |

**Mediana 6 esquinas (vuelta y media), rango 4-8. Tres de cinco llegan al
final de la ventana de 180 s.** Esa es la linea base real, y hay que
compararla con lo que veniamos creyendo: la corrida de 5 esquinas que
tomamos por «la mejor» y la de 1 esquina que nos hizo revertir estaban
las dos dentro de este mismo rango. Ninguna de las dos medía el codigo.

Dos observaciones que solo aparecen con la serie:

- **Las emergencias no predicen el resultado.** La run 1 hizo 7 esquinas
  con 11 y la run 2 hizo 6 con 172. La run 4, con **cero** emergencias,
  es la peor de todas. Como metrica de calidad no sirve; solo cuentan las
  esquinas y la duración.
- **Los dos fallos son de recentrado y recuperación**, no de percepción ni
  de esquina. Ahi esta el trabajo que queda.

Para las tres vueltas faltan 6 esquinas sobre la mediana, y el ritmo de
~21 s por esquina las situa en 250-270 s contra los 180 del limite. No es
cuestion de estabilidad: hace falta velocidad, y subirla choca con el
pilar que aparece tras la esquina.

## Latencia del LiDAR, ultrasonido trasero y parqueo modular

Tres cambios que van juntos porque los tres atacan lo mismo: el control
estaba decidiendo con informacion que ya no describia la pista.

### El retraso del LiDAR era de adquisicion, no de proceso

`comun/lidar_driver.py` leia el puerto **byte a byte**: un `read(1)` para el
byte de cabecera y un `read(4)` para el resto, por cada muestra. A 460800
bps el C1 entrega ~9200 paquetes/s, o sea **~18400 llamadas de sistema por
segundo** solo para recibir. Y el callback del hilo de lectura era el ciclo
de decision completo (geometria, fusion, vision, control, metricas): 15-30
ms en los que el kernel seguia acumulando bytes. Al volver a leer, el driver
procesaba paquetes viejos, y el retraso **crecia solo**, porque cada barrido
atrasado tardaba lo mismo en procesarse que uno fresco.

Lo que se hizo:

- **Lectura en bloques.** Cada vuelta del hilo drena con una sola llamada
  todo lo pendiente (`read(in_waiting)`) y decodifica el lote entero de una
  vez, vectorizado con numpy (hay una rama pura equivalente si numpy falta).
  El `read(1)` bloqueante se conserva solo como espera pasiva cuando el
  LiDAR calla, para no quemar CPU girando en vacio.
- **Sincronizacion explicita.** El parser exige una racha de 12 paquetes
  coherentes antes de fiarse de una alineacion; los dos check bits del
  protocolo dejan pasar 1 de cada 4 posiciones al azar, asi que un solo
  paquete no basta. Ante un byte corrupto se resincroniza y cuenta el
  episodio (`driver.resincronizaciones`).
- **Filtro de angulo imposible.** El campo trae 15 bits, hasta 511,98
  grados. Un paquete corrupto que superara los check bits terminaba aliasado
  dentro de `construir_perfil_360` (`indice % 360`) inventando un obstaculo
  en un sector que nadie miraba. Ahora se descarta.
- **Buzon de un solo hueco.** `BuzonBarridosLidar` no es una cola: el
  barrido nuevo pisa al que no llego a consumirse. Encolar habria sido peor
  que descartar, porque el robot acabaria esquivando donde el pilar *estaba*.
- **El bucle principal es el bucle de control.** El hilo del LiDAR solo
  publica; el ciclo de decision corre en el hilo principal, que ya hacia los
  watchdogs. Perception, fusion y FSM usan el **timestamp de captura**, no el
  de proceso: eso ademas alinea de verdad la fusion con la camara, que
  siempre uso su instante de captura.

Los watchdogs siguen midiendo contra la hora real (`time.monotonic()`), no
contra la del dato: usarla los haria mas indulgentes justo cuanto mas
atrasado fuera el sistema.

Dos columnas nuevas en el CSV para comprobarlo en pista:

- `lidar_edad_ms`: edad del barrido en el instante de emitir la consigna.
  **Tiene que quedarse plana, no crecer.** Si sube monotonamente durante la
  corrida, el buffer se esta volviendo a acumular.
- `barridos_descartados`: cuantas veces un barrido quedo obsoleto sin
  consumirse. Un numero pequeno y estable es normal; uno que crece rapido
  dice que el ciclo de decision no llega a 10 Hz.

### Ultrasonido trasero (HC-SR04 / US-100)

**Cableado: Trigger en GP14, Echo en GP15.** Son los pines libres mas
comodos: GP12 es el servo, GP16-GP19 los dos buses I2C, GP22 el PWM del
motor y GP26-GP28 el TB6612FNG.

> **Aviso.** El Echo del HC-SR04 sale a 5 V y los GPIO de la Pico **no**
> toleran 5 V. Hay que bajarlo con un divisor (1 k en serie desde Echo, 2 k
> a GND) o usar un US-100 alimentado a 3,3 V. Sin eso se dana la entrada.

El firmware **no** usa `machine.time_pulse_us`: esa llamada bloquea hasta 30
ms esperando el eco, y el bucle de la Pico corre cada 5 ms sosteniendo el
servo, el motor y el watchdog de comandos. En su lugar se lanza el pulso
(10 us de bloqueo) y se cronometra el flanco por **interrupcion**, de modo
que el control sigue corriendo mientras el sonido viaja. Periodo de disparo:
60 ms, que es lo que el HC-SR04 pide para no arrastrar ecos fantasma.

La conversion y el filtrado viven en `src/pico/ultrasonido.py`, sin importar
`machine`, para poder probarlos en CPython igual que `protocolo_seguro.py`.
El filtro publica la **mediana de tres** muestras y caduca la medida tras
tres intentos sin eco: lo peor que puede hacer un sensor de distancia es
seguir publicando la ultima lectura buena cuando ya no ve nada.

> **Al desplegar hay que copiar `ultrasonido.py` a la Pico junto a
> `main.py`.** Si falta, el firmware arranca igual y manda `US:-1`; no se
> cae, pero tampoco hay sensor.

Trama de telemetria: `IMU:...,COLOR:...,US:185,WD:OK`. El campo va antes de
`WD` y los parsers recorren campos por nombre, asi que el firmware anterior
y `comun/enlace_pico.py` (que corta por la primera coma) siguen funcionando.
`US:-1` significa *sin medida*, nunca *libre*.

### Parqueo: una sola entrada y trasera fusionada

`ejecutar_estacionamiento(controlador, hueco, heading, medidas, ahora)` es
ahora el unico punto de entrada. Las medidas viajan en un `MedidasParqueo`
en vez de una docena de argumentos sueltos, y la tolerancia a que la FSM
cambie de firma se movio de `control_ruta` a `estacionamiento.py`, junto a
la FSM que la necesita.

**Regla de fusion de la trasera: se toma la MENOR de las dos fuentes.** El
razonamiento es asimetrico a proposito. Los dos modos de fallo conocidos del
LiDAR aqui —el sector ciego del mastil (163-195 grados) y el eco de la
propia rueda— hacen que informe de *mas* espacio del que hay, o de ninguno;
nunca de menos. Quedarse con la menor significa que:

- el ultrasonido manda justo cuando ve algo que el LiDAR no vio, que es para
  lo que se monto;
- nunca puede autorizar una reversa que el LiDAR ya considera peligrosa.

Con el robot en angulo dentro del arco, el sensor puede estar midiendo el
separador y no la pared del fondo. Sigue siendo el numero seguro, y la
comprobacion de que `frontal + trasera` equivale al largo del hueco impide
cerrar el centrado con esa medida.

Lo que el ultrasonido **no** sustituye: las diagonales traseras. Mira recto
hacia atras y no ve las esquinas, asi que esas siguen saliendo del LiDAR y
su ausencia sigue frenando el arco. La comprobacion de cobertura, que es una
metrica del LiDAR, se omite solo cuando el numero lo puso el ultrasonido.

#### Como probar el parqueo aislado: `--solo-parqueo`

El unico acceso normal al estacionamiento esta **despues de verificar una
esquina** (`_procesar_giro`, justo tras incrementar el contador). Como
completar una esquina es precisamente lo que bloquea el radio de giro, sin
una puerta directa la FSM de parqueo no se puede ejercitar en pista.

`--solo-parqueo` pone `corners_before_parking` a 0, y con ese valor la FSM
entra a buscar la bahia en cuanto conoce el sentido de carrera. Omite ademas
la calibracion `parking_ready`, que es justo lo que la prueba viene a
producir; **el resto de calibraciones y `motion_enabled` se siguen
exigiendo**. Lo anuncia por consola al arrancar.

Esa puerta **solo la puede abrir el flag**: `validar_configuracion` rechaza
`corners_before_parking < 1`, asi que ningun JSON la activa por su cuenta.
Es excluyente con `--sin-parqueo`, que pide lo contrario.

Dos avisos de uso:

- Con `turn_direction: AUTO` el robot nunca entra al parqueo si arranca
  dentro de la bahia, porque no cruza ninguna linea de sentido y sin sentido
  no hay lado de bahia. Para esta prueba hay que fijar `LEFT` o `RIGHT`.
- Es opcion de banco, como `--arranque-inmediato`. La ronda oficial no la usa.

Claves nuevas en `configuracion.json` (`parking`):
`ultrasound_rear_enabled`, `ultrasound_rear_min_mm`, `ultrasound_rear_max_mm`.
Poniendo la primera en `false` se vuelve al comportamiento anterior sin
tocar codigo, que es lo que hace falta para comparar dos corridas.

### Reactividad de la evasion

Dos claves nuevas en `control`, ambas con respaldo al valor historico si no
estan:

- `avoid_steering_slew_deg_per_scan` (18,0). El slew de crucero son 6
  grados/barrido; a 10 Hz eso son cinco barridos —medio segundo, 8 cm a 40
  PWM— para llegar al tope de direccion. Contra un pilar a menos de un metro
  esa rampa se comia el margen: el robot pedia el angulo correcto y llegaba
  tarde.
- `obstacle_pursuit_kp_near` (1,6) entre `obstacle_pursuit_far_mm` (950) y
  `obstacle_pursuit_near_mm` (320). Con ganancia unica hay que elegir: si
  sirve de lejos, de cerca se queda corta porque el bearing crece rapido en
  los ultimos 30 cm; si sirve de cerca, de lejos oscila. Se interpola
  linealmente y satura en los dos extremos.

**Los cuatro valores son puntos de partida razonados, no calibraciones.**
Hay que medirlos en pista antes de darlos por buenos.
