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
