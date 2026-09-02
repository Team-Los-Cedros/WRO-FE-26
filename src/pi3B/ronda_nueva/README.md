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
   rumbo durante el sobrepaso y lo suelta cuando el pilar queda detrás o
   despejado de lado —sobre una pose que se propaga con el movimiento propio,
   porque el LiDAR deja de ver el poste por debajo de ~250 mm—, y vuelve al
   centro por posición;
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

> **AVISO (2026-09-02): las conclusiones sobre el radio de esta sección son
> falsas.** El radio no se midió, se dedujo dividiendo una velocidad de recta
> entre una velocidad angular de giro, y salió inflado casi al doble. El radio
> real es 228 mm a la izquierda y 260 a la derecha, y la rueda gira más de lo
> comandado, no la mitad. La sección se conserva porque el resto (fallback de
> rumbo, `WD:OK`, latencia de visión) sigue siendo válido y porque el error de
> método merece quedar registrado. Ver «Sesión 2026-09-02» al final.

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

## Sesión 2026-09-02: el radio de giro se midió, y el bloqueante no existía

La bitácora del 31-08 (sección «el radio de giro es el bloqueante») daba por
medido un **radio mínimo de ~600 mm en ambos sentidos**, concluía que la rueda
solo alcanzaba 12,7° cuando se le mandaban 20-25°, señalaba la relación
varilla/horn como culpable y recomendaba trabajo mecánico. **Nada de eso era
cierto.** Esta sección lo corrige con la medida directa y deja el método por
escrito, porque el error es fácil de repetir.

### El error: dividir dos medidas que no son de la misma maniobra

Aquel radio no se midió, se dedujo como `r = v / ω`, con:

- `v = 150 mm/s`, medida **en recta** por el cierre contra un muro con el LiDAR;
- `ω = 14,4 °/s`, medida **en giro** con el servo al tope.

El robot pierde el **47 % de su velocidad al girar a tope**: girando avanza a
unos 79 mm/s, no a 150. El rozamiento de arrastre de las ruedas delanteras a
27-31° no es despreciable en un chasis LEGO. Al usar la `v` de la recta, el
radio salía inflado casi al doble. La lección es la de siempre en este
proyecto: *medir la magnitud que interesa, no una combinación de otras dos.*

### La medida directa, por dos métodos independientes

Herramienta nueva: `src/pi3B/herramientas/medir_radio_giro.py`. Manda el servo
al tope y traza círculos completos a 22 PWM (el PWM del parqueo). Dos detalles
que la hacen válida:

- **`kd = 0` obligatorio.** El firmware resta al servo un término de
  amortiguación por giroscopio (`angulo_servo = CENTRO + angulo -
  velocidad_z * KD_ESTABILIDAD * kd_activo`). En un giro sostenido `velocidad_z`
  no es cero, así que con `kd = 1` la rueda **no** está en el ángulo comandado y
  se mediría el radio del lazo de estabilidad. Se usa la trama de tres campos
  `velocidad,angulo,kd`. Verificado contra la placa: 58 líneas de telemetría con
  0 `WD:STOP` aceptando la trama, y una trama con `kd = 9,99` (fuera del rango
  legal) rechazada con el watchdog disparado, como debe.
- **Media vuelta mide mejor que la vuelta entera.** Con `--grados 180` la
  distancia en línea recta entre la marca inicial y la final *es* el diámetro:
  medir una recta entre dos puntos es más preciso que estimar el diámetro de una
  curva dibujada. La vuelta entera sirve para comprobar que cierra.

**Método 1 — círculo dibujado.** Un marcador en el chasis traza el círculo y se
mide con cinta. Costó tres intentos: un trozo de tirro en el suelo atascó al
robot 6,4 s en una corrida (rumbo congelado en −253°, tasa por debajo de
6 °/s), y el marcador falló dos veces por no llegar al suelo con presión.

**Método 2 — sinusoide sobre el LiDAR, sin tocar el suelo** (`--con-lidar`).
Girando en círculo, si se compensa el rumbo con la IMU y se mira siempre hacia
la misma dirección **del mundo**, la distancia a una pared plana recorre una
sinusoide `d(ψ) = A − (r / cos α)·cos(ψ − fase)`, donde α es el ángulo entre el
rayo y la normal de la pared. La amplitud es mínima, y vale exactamente `r`,
cuando el rayo apunta perpendicular. No hace falta saber dónde está la pared:
se ajusta `d = A + B·cos ψ + C·sin ψ` en las 360 direcciones y se toma la
amplitud menor entre los ajustes buenos. Validado antes de usarlo contra datos
sintéticos de radio conocido (200-600 mm, ruido de 8 mm): error por debajo del
0,6 %. En el robot dio **3,0 mm de residuo** y direcciones vecinas coherentes.

**Ojo: el LiDAR no está en el eje trasero.** El método mide el círculo del
sensor, que es `R_lidar = sqrt(d² + r_eje²)` con `d` la distancia del eje
trasero al eje de giro del LiDAR. Hay que despejar `r_eje`.

### Resultado

| Lado | Mando al servo | `R_lidar` | **`r` del eje trasero** | Rueda real | Factor |
| --- | --- | --- | --- | --- | --- |
| Izquierda | +25° | 264 mm | **228 mm** | 30,8° | 1,23× |
| Derecha | −20° | 292 mm | **260 mm** | 27,6° | 1,38× |

Los dos métodos coinciden: con el marcador en el morro (162 mm del eje), el
LiDAR predice un círculo de 613 mm y la cinta midió 633 — un 3 % sobre un
círculo dibujado a mano que además no cerraba (22 mm de error de cierre).

**La rueda gira MÁS de lo que se le manda**, entre 1,23× y 1,38×, no la mitad.
Los ángulos de `steering_max_*_deg` son grados de **servo**, no de rueda; la
relación del varillaje amplifica. No hay nada que arreglar en el horn.

### Qué cambia

1. **Las esquinas no están limitadas por la geometría.** Una esquina de 90° en
   un carril de 1000 mm pide 400-500 mm de radio y el chasis da 228-260. Si el
   robot no las cierra, la causa está en el control —el handoff, los timeouts,
   el K-turn—, no en el chasis. La conclusión del 31-08 de que «la siguiente
   mejora real es mecánica» queda **refutada**.
2. **`lidar_forward_from_rear_axle_mm` estaba mal: decía 162, el real es 133**
   (regla, al eje de giro del LiDAR). Corregido en `configuracion.json`. El
   valor fija dónde para `ALIGN` (`−123 mm`, antes `−152`) y el objetivo de
   centrado (`trasera − frontal = 164 mm`, antes `222`): 29 mm de desvío sobre
   una holgura de 55 mm por punta. *Este valor ya había cambiado dos veces
   (128 por fotogrametría, luego 162 «por regla»); los 29 mm de diferencia con
   los 162 son compatibles con haber medido hasta la carcasa en vez de hasta el
   eje de giro. **Conviene reconfirmarlo con regla antes de competir.***
3. **El parqueo de dos arcos no cierra, pero por poco.** Simulado contra la
   geometría real de la bahía —333 × 200 mm, delimitadores de 200 × 20 mm según
   la regla 13.7— con los dos radios reales y asimétricos, la maniobra se pasa
   **9 mm** sobre un delimitador en el mejor caso, y da igual por qué lado
   entre: el corrimiento lateral depende de `r_entrada + r_salida`, que es
   simétrico. Con el radio falso de 600 mm la interferencia era de **468 mm**,
   y eso sí era concluyente; 9 mm está dentro del error del modelo (el robot
   como rectángulo perfecto de 222 × 125, arcos ideales, sin contar con lo que
   `CENTER` pueda corregir). **No se puede afirmar que sea imposible: hay que
   probarlo en pista con `--solo-parqueo`.** Si de verdad no cierra, la salida
   es una maniobra en varios tiempos, la misma idea del K-turn de las esquinas,
   que el reglamento no prohíbe.

`parking_ready` sigue en `false`. De los tres datos que esperaba —radio, signo
y posición final— ya hay dos: el radio, medido por los dos lados, y el signo
(**izquierda = rumbo positivo, derecha = negativo**, confirmado en las trazas).
Falta la posición final verificada dentro de las barreras.

### Extrínseca cámara–LiDAR: la guiñada estaba a 0 y son 3,57°

`camera_lidar_extrinsics_ready` estaba en `true` y `yaw_from_lidar_deg` en
`0.0`. Medido el 02-09 contra pilares reales: la guiñada es **+3,57°**.

**Método** (`herramientas/capturar_extrinseca.py` y `analizar_extrinseca.py`).
Un pilar visto por los dos sensores da dos bearings, y la diferencia es el
error de montaje. Tiene dos trampas, y las dos importan:

- **Asociar por distancia, nunca por bearing.** El bearing es la magnitud que
  se está midiendo; emparejar por él daría por bueno lo que se quiere
  comprobar. La cámara estima la distancia por la altura del blob (el pilar
  mide 100 mm) y con eso se empareja. Cuando las dos distancias no cuadran, la
  pareja se cae sola.
- **Los pilares cercanos no valen.** Por debajo de ~600 mm un pilar subtiende
  varios grados: el LiDAR ve solo la cara frontal y la cámara la silueta
  frontal más la lateral, así que sus centroides no son el mismo punto. En las
  capturas, los dos pilares a menos de 550 mm fallaron la distancia por un 20 %
  y dieron desacuerdos de −4,00 y −4,66 frente a los −3,5 de los lejanos: el
  criterio de distancia los descarta sin que haya que decírselo.

**Guiñada o punto principal.** Un error de guiñada desplaza todos los bearings
por igual; uno de punto principal escala con `cos²` y se nota más en los
bordes. Cuatro parejas fiables, en dos poses y con bearings de −13,8° a +7,3°:

| Captura | Bearing en cámara | Desacuerdo |
| --- | --- | --- |
| 02 | −3,27° | −3,45° |
| 02 | −13,79° | −3,54° |
| 03 | +7,28° | −3,52° |
| 03 | −11,14° | −3,77° |

Media **−3,57°**, dispersión 0,33°. Constante en todo el campo ⇒ guiñada. Las
dos poses por separado dan −3,50 y −3,64: 0,14° entre poses, así que el número
es del montaje y no de dónde esté el robot.

**`forward_from_lidar_mm` también estaba mal: −99,76, y medido con regla son
−80.** La cámara va en el mástil trasero. Corregirlo baja la dispersión de la
guiñada de 0,47° a 0,33°, que es evidencia indirecta de que el valor nuevo es
mejor. Es el tercer valor de montaje que aparece desviado, tras los 162→133 del
LiDAR al eje trasero: **conviene repasar el bloque entero con regla.**

Dos cosas que quedan anotadas y sin sitio en la configuración: la **altura del
lente sobre el piso, 180 mm** (el bloque `camera` guarda `forward`, `right` y
`yaw`, pero no altura; hoy no hace falta porque el soporte de suelo trabaja con
`roi_*_ratio`, pero cualquier estimación por la base del blob la necesitaría), y
que **capturar en el modo de sensor equivocado invalida la comparación**: un
`create_still_configuration` a 1536×864 es un recorte y deja fuera un tercio del
campo. Un pilar que el LiDAR ve a 29° puede no aparecer siquiera en la imagen.
`capturar_extrinseca.py` fuerza el mismo modo que la ronda (640×360 sobre
2304×1296) justo por eso.

**Qué cambia en carrera.** `camera_lidar_gate_deg` vale 10°, así que un sesgo
fijo de 3,57° gastaba **el 36 % de la puerta de asociación** antes de que
entrara ningún ruido real. No rompía la fusión por sí solo, pero le dejaba un
tercio menos de margen. *No* explica el caso abierto de los tracks que se
pierden lejos: la puerta es angular, así que un sesgo constante en grados
afecta igual a todas las distancias.

## Sesión 2026-09-02 (tarde): el sobrepaso no era un lazo cerrado, era un cronómetro

El pendiente venía escrito como «el primer pilar rojo cuesta 6,7 s». Al medirlo
sobre los CSV resultó ser un caso particular de algo más general y bastante peor.

### La medida: `obstacle_cleared_y_mm` no ha disparado nunca

Recorriendo los 74 episodios de `AVOID_PASS` de las ocho corridas del 01-09
(`125223`, `144939`, `151356`, `152424`, `185017`, `111701`, `112827` y las
cinco de `serie5`), **el criterio de salida por LiDAR no decidió ni una sola
vez**. Las 74 salidas dicen `reincorporacion tras ~32X mm de sobrepaso
estimado`, o sea la red de seguridad por distancia muerta. Y como esa red es
`obstacle_pass_distance_mm / (speed_avoid_pwm · mm_s_per_pwm)`, el sobrepaso
dura **siempre lo mismo**: 3,28 s de media, con muy poca dispersión, sin
importar el color del pilar, su geometría ni si el LiDAR lo estaba viendo.

Lo que eso cuesta, en reparto de tiempo por estado:

| Corrida | Duración | Esquinas | `AVOID_PASS` | `AVOID_APPROACH` | `TURN` |
| --- | --- | --- | --- | --- | --- |
| `125223` | 143,7 s | 5 | 45,1 s (31 %) | 26,4 s (18 %) | 40,6 s (28 %) |
| `185017` | 171,7 s | 6 | 37,6 s (22 %) | 43,9 s (26 %) | 45,5 s (27 %) |
| `serie5 run3` | 176,9 s | 8 | 41,7 s (24 %) | 48,0 s (27 %) | 37,3 s (21 %) |

**Evadir pilares se lleva entre el 45 % y el 52 % de la ronda**, más que girar
esquinas. Es la partida más grande del presupuesto de 180 s, y hasta ahora la
mitad de esa partida era un cronómetro a ciegas.

### Por qué el criterio es inalcanzable

El pilar deja de existir para la percepción mucho antes de lo que supone
`obstacle_cleared_y_mm = -280`. Midiendo las **143 pérdidas de track** de esas
mismas corridas —el ciclo en que `track_activo_observado` pasa de 1 a 0— la
distancia a la que se apaga es:

| | mín | p25 | mediana | p75 | p90 |
| --- | --- | --- | --- | --- | --- |
| distancia al perder el pilar | 145 mm | 201 mm | **216 mm** | 249 mm | 407 mm |

El 75 % de las pérdidas ocurre por debajo de 250 mm, que es justo el
`near_blind_spot_mm = 251` que ya estaba anotado en el JSON —y que hasta hoy
no gobernaba nada, solo documentaba el fenómeno—. La causa está en
`comun/lidar_geometria.py`: `es_cluster_obstaculo` exige `ext_ang < 15°` y
`n ≤ 30` puntos, y un poste de 100 mm subtiende 15° a 383 mm, 23° a 250 mm y
33° a 175 mm. El propio comentario de `es_objeto_estrecho` ya lo avisaba:
*«el poste deja de reconocerse justo cuando está encima»*. Para la pared se
resolvió con un filtro por ancho físico; para el obstáculo se dejó la puerta
angular.

Con eso, pedir «ver el pilar 280 mm **por detrás** del LiDAR» es pedir algo que
la geometría del sensor prohíbe dos veces: queda dentro del radio donde el
cluster ya no pasa el filtro, y además cae en la máscara del mástil (163–195°).

Y al perderse, `_ultimo_track` se queda congelado. En la corrida `125223`,
entre t=7032,4 y t=7034,3, hay **19 ciclos seguidos con la misma x, la misma y
y hasta la misma edad** (`-96,8 / 143,1 / 0,400`): 1,9 s decidiendo sobre una
foto vieja.

### El arreglo: la pose del pilar se propaga, y la salida es geométrica

`_actualizar_pose_pilar` mantiene una pose `(x, y)` del pilar activo que se fija
con cada observación y, cuando no la hay, se propaga con el movimiento propio
—el mismo modelo de predicción que ya usa `FusionLigera`: rotación por el cambio
de rumbo del IMU y avance por `velocidad · mm_s_per_pwm · dt`—. Sobre esa pose,
`_procesar_sobrepaso` sale por dos criterios geométricos:

- **detrás**: `y ≤ obstacle_cleared_y_mm` (−280 mm). Ahora sí es alcanzable,
  porque no exige volver a ver el pilar.
- **de lado**: `|x| ≥ obstacle_side_clear_mm` (350 mm) **y** `y ≤ 0`. Un pilar
  que quedó a más de un tercio de metro del eje y ya no está por delante no se
  puede alcanzar reincorporándose, así que sostener el rumbo solo gasta
  segundos. Es el caso de los pilares que entran al sobrepaso ya rebasados: en
  `125223`, el rojo del segundo 6995,9 entró a `x = −393, y = 87` —al costado
  del robot— y se le dedicaron 3,4 s.

La red de seguridad por distancia **se conserva tal cual**, así que el cambio
solo puede acortar un sobrepaso, nunca alargarlo. Reproduciendo los criterios
sobre los CSV de las mismas ocho corridas: 9 de los 74 sobrepasos salen antes
por «de lado», **−15,3 s en total (−6,3 %)**, con ahorros de hasta 3,1 s en un
solo pilar y ningún episodio más largo que hoy.

También se registran dos columnas nuevas, `pilar_estimado_x` y
`pilar_estimado_y`, y la razón de salida dice ahora **cuál** de los tres
criterios decidió. Sin eso la próxima corrida no puede auditar esta decisión.

### Lo que queda abierto (y no se tocó a propósito)

**1. El sobrepaso termina antes de que el pilar esté realmente rebasado.** Con
el LiDAR a 133 mm por delante del eje trasero y la culata 60 mm por detrás de
él, el parachoques trasero está en `y = −193`. La red de distancia recorre 320 mm
desde `obstacle_pass_y_mm = 280`, o sea que suelta el pilar cerca de `y ≈ −40`:
todavía al costado de la mitad trasera del robot, justo cuando `RECENTER` empieza
a girar el volante **hacia ese lado**. La prueba de `test_el_sobrepaso_arranca_antes_del_punto_ciego`
comprueba que `paso − recorrido < 0`, pero eso es «detrás del LiDAR», no «detrás
del robot». Corregirlo es subir `obstacle_pass_distance_mm`, lo que hace al robot
**más lento**, así que es exactamente el tipo de cambio que exige las tres
corridas por configuración; no se hace a ciegas.

**2. La velocidad del modelo y la velocidad de la bitácora no coinciden.**
`fusion.mm_s_per_pwm = 4,0` y la tabla de «Datos ya medidos del chasis nuevo»
dice 150 mm/s a 22–23 PWM, que son 6,7 mm/s por PWM: **discrepan 1,6×**. Hoy esa
constante *es* la geometría de la evasión, porque es lo único que termina un
sobrepaso, y también es lo que propaga la pose nueva. Estimarla desde los CSV por
el cierre del frontal da ~3,8 mm/s por PWM, pero ese estimador es una cota
inferior (mide `v·cos θ`) y se contamina cuando un pilar entra al sector, así que
**no sirve para cerrar el asunto**: hace falta la medida directa, en recta y con
las ruedas rectas, como se hizo con el radio de giro.

**3. La puerta angular de 15° sigue apagando pilares a 250 mm.** Se puede
arreglar en `es_cluster_obstaculo` con el mismo criterio de ancho físico que ya
usa `es_objeto_estrecho`, pero ese módulo lo comparten `ronda_cerrada` y
`ronda_camara`, así que es un cambio con radio de acción propio y merece su
propia sesión. Y aun arreglado **no salvaría el criterio viejo**: el pilar
rebasado cae en la máscara del mástil. Lo que sí mejoraría es la aproximación,
donde `AVOID_APPROACH` llega a gastar 8,5 s en un solo pilar.

### Buscar la línea de sentido se hacía a ciegas

Tres corridas seguidas del 02-09 por la tarde murieron sin llegar a medir nada
del sobrepaso, y las tres por la misma causa. El vídeo cenital de la corrida
`190712` la muestra entera: el robot cruza la recta inferior **en diagonal**,
llega a un pilar rojo, lo empuja y lo **desplaza casi dos anchos de robot**
fuera de su sitio. En competencia eso no es un roce: es un obstáculo movido.

El CSV explica por qué nadie lo frenó. El bloque de `WAIT_DIRECTION` en
`procesar` sale con `return` propio, y la prioridad global de emergencia estaba
**por debajo** de él. Mientras el robot avanza buscando el AZUL/NARANJA no hay
ni emergencia ni evasión: solo centrado por paredes.

| | Corrida 1 | Corrida 2 | Corrida 3 |
| --- | --- | --- | --- |
| Tiempo en `WAIT_DIRECTION` | 11,7 s | 6,6 s | 11,3 s |
| Primer `frontal` < 150 mm | 4,70 s | 4,72 s | ~4,7 s |
| Mínimo de `frontal` | 18,8 mm | 29,5 mm | 22,2 mm |
| Emergencia | a los 11,9 s | nunca | a los 11,35 s |

Los 4,7 s idénticos son la misma colocación contra el mismo bloque. Y 18,8 mm
está **por dentro del perímetro del propio robot** (45–61 mm): no es un eco
espurio ni la rueda propia, es el LiDAR pegado al pilar. La percepción lo veía
perfectamente; la máquina de estados no estaba mirando.

Para comparar, la corrida `141225` de esa misma mañana —misma configuración,
misma percepción— tuvo el 2,9 % de lecturas por debajo de 150 mm y un mínimo de
117 mm. La diferencia no es el robot: es dónde arranca y contra qué.

**El arreglo, en dos piezas.** La comprobación de emergencia pasa a ejecutarse
**antes** de avanzar buscando la línea. Y como una recuperación termina en
`CRUISE`, hizo falta la segunda pieza: si el sentido todavía no está fijado, la
recuperación vuelve a `WAIT_DIRECTION` en vez de entrar a crucero —entrar sin
sentido dejaría la ronda sin saber hacia dónde gira la pista, que es lo que
gobierna el conteo de esquinas y el lado del parqueo—. Por lo mismo, el giro
forzado no se dispara sin sentido: su signo cae en `self._sentido`, que vale 0.

*Lo que esto no arregla:* el robot sigue cruzando el carril en diagonal
mientras busca la línea, porque ahí el mando es el centrado por paredes y en
mitad del tapete eso no lo mantiene en su lado. Ahora al menos frena antes de
llegar. Y sigue gastando 11 s en encontrar el sentido con `turn_direction:
AUTO`, más de la mitad de lo que duraba cada corrida.

### El bucle que esto destapó, y su salida

La corrida siguiente (`191636`) confirmó el arreglo —mínimo de `frontal` de
**125 mm** frente a los 18–29 mm de antes, o sea que ya no llega a tocar el
pilar— y a la vez dejó ver el problema que tapaba: el robot se quedó en bucle
y murió por `timeout esperando color de sentido` a los 13,6 s, con `color_piso`
en `PISTA` durante los 138 barridos.

| t | Qué pasa | `frontal` |
| --- | --- | --- |
| 0,00 | avanza buscando la línea | 667 mm |
| 5,03 | emergencia → retrocede | 135 mm |
| 6,52 | despejado → vuelve a buscar | 276 mm |
| 8,42 | emergencia → retrocede | 139 mm |
| 9,92 | despejado → vuelve a buscar | 286 mm |
| 12,01 | emergencia → retrocede | 131 mm |
| 13,61 | **FALLO**: timeout de sentido | 288 mm |

La causa es aritmética: `emergency_front_mm` vale 140 y
`recovery_exit_front_mm` 250, así que la recuperación retrocede sólo hasta
tener 250 mm libres y vuelve a avanzar contra lo mismo, que sigue a ~150. Son
110 mm de histéresis y ninguna salida lateral, porque el mando en ese estado es
el centrado por paredes, que manda seguir recto.

`_angulo_busqueda_sentido` da esa salida: con el frente por debajo de
`direction_search_avoid_mm` (400 mm) el robot gira hacia el lado con más hueco,
con fuerza proporcional a lo cerca que esté el frente —lejos no toca nada y
sigue mandando la pared; pegado al límite de emergencia llega al tope de
dirección—. No elige sentido de pista con eso: eso lo sigue decidiendo el color
del piso. Sólo despeja el camino para poder encontrarlo.

*Nota de método:* el vídeo de esa corrida se perdió. Al acabar la ronda antes
de tiempo hubo que cortar `ffmpeg`, y un MP4 normal escribe su índice (`moov`)
sólo al cerrarse: quedaron 26 MB de datos ilegibles. Se graba desde entonces en
MP4 fragmentado, comprobado truncando un fichero a la mitad a propósito.

*Método:* esto salió del vídeo, no del CSV. El CSV decía «`frontal` = 22 mm» y
eso admitía varias lecturas —eco de la rueda, mástil, cable—; el vídeo mostraba
un pilar rojo desplazándose por el tapete. Desde ahora toda prueba se graba con
la cenital y se analizan las dos cosas juntas.
