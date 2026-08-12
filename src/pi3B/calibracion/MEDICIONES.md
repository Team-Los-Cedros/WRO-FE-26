# Mediciones pendientes antes de escribir el algoritmo

Este documento es el protocolo de las mediciones que todavía faltan, en el orden
en que conviene hacerlas. Cada sección dice **qué se mide**, **cómo**, **cuánto
tarda** y **dónde va el número** una vez medido.

Todas las constantes físicas viven en un solo archivo:
[`comun/geometria_robot.py`](../comun/geometria_robot.py). Ese archivo se puede
ejecutar directo (`python3 geometria_robot.py`) e imprime el impacto numérico de
cada offset, así que sirve de verificación después de rellenarlo.

| # | Medición | Bloqueante | Herramienta | Tiempo |
| :- | :- | :- | :- | :- |
| 1 | Offset X/Y del LiDAR respecto al eje trasero | **Sí** | regla | 2 min |
| 2 | Offset X de la cámara respecto al LiDAR | **Sí** | regla, o método B | 2 min |
| 3 | Captura en la pista real con los postes puestos | **Sí** | `capturar_pista.py` | ~30 min |
| 4 | Centro mecánico del servo y radio de giro | No | `medir_direccion.py` | 15 min |
| 5 | Velocidad vs PWM, batería llena y baja | No | `medir_velocidad.py` | 20 min |
| 6 | Dos dudas del reglamento 2026 | — | **ya resuelto**, sección 6 | — |

---

## 1. Offset X/Y del LiDAR respecto al eje trasero

### Por qué importa

El pure pursuit de `navegacion.py` calcula el rumbo con
`math.atan2(x_obj, y_obj)` sobre las coordenadas **crudas del LiDAR**, que están
adelantadas respecto al eje trasero. Pero la cinemática de bicicleta
(Ackermann) solo es válida con origen en el eje trasero. Con un offset de
~128 mm el error no es cosmético:

| Poste a esta distancia | Rumbo sin corregir | Rumbo corregido | Error |
| :- | :- | :- | :- |
| 450 mm | 30.0° | 24.2° | **5.8°** |
| 300 mm | 40.9° | 31.3° | **9.6°** |
| 200 mm | 52.4° | 38.4° | **14.0°** |

(punto de paso a 260 mm de lado, que es `SEPARACION_LATERAL`). El error crece
cuanto más cerca está el poste, es decir justo cuando el margen es menor. Como
`MAX_ANGULO_EVASION` es 25°, un error de 14° es más de la mitad del rango útil
del servo.

### Estimación provisional (ya cargada en el código)

Mientras no exista la medición con regla, `geometria_robot.py` trae valores
sacados por fotogrametría de las fotos del propio repo — `v-photos/Topview.jpeg`,
`v-photos/Rightview.jpeg` y `v-photos/frontview.jpeg` — usando la batalla de
136 mm como escala y corrigiendo el paralaje por altura (el LiDAR está a 90 mm
del piso, las ruedas a 18 mm, así que en una foto cenital el LiDAR aparece
desplazado hacia afuera respecto a su posición real):

```
LIDAR_X = 128.0   # mm por delante del eje trasero  (incertidumbre ±10)
LIDAR_Y =  -4.0   # mm, negativo = a la izquierda   (incertidumbre ±5)
LIDAR_Z =  90.0   # altura del plano de barrido, del README
```

Las tres vistas coinciden en lo mismo: **el eje del LiDAR cae prácticamente
sobre el eje delantero**, unos 5-10 mm por detrás. Eso es coherente con la vista
lateral, donde el cuerpo del LiDAR queda justo encima de las ruedas
directrices. Sirve para desbloquear el código, **no** para competir.

### Protocolo con regla (2 minutos)

1. Robot sobre una mesa plana, ruedas rectas.
2. Apoya una regla metálica o una escuadra **cruzada sobre los dos bujes
   traseros**: esa línea es el eje trasero. Marca con cinta dónde queda.
3. **X (adelante/atrás).** No intentes apuntarle al eje de rotación del LiDAR a
   ojo — mide a las dos caras del cuerpo del C1 y promedia:
   - `a` = distancia del eje trasero a la **cara trasera** de la carcasa
   - `b` = distancia del eje trasero a la **cara delantera** de la carcasa
   - `LIDAR_X = (a + b) / 2`
   El eje de rotación está centrado en la carcasa cuadrada del C1, así que el
   promedio elimina el error de apreciación.
4. **Y (lateral).** Mide de la cara **externa** de cada rueda trasera al costado
   correspondiente de la carcasa del LiDAR: `iz` y `de`.
   `LIDAR_Y = (iz - de) / 2`, positivo = el LiDAR está corrido a la derecha.
5. Escribe los tres números en `geometria_robot.py` y cambia el marcador
   `[ESTIMADO]` por `[MEDIDO]`.

> **Sobre el `204 mm` del código experimental:** correcto descartarlo. 204 mm no
> es compatible con una batalla de 136 mm salvo que el LiDAR sobresalga 68 mm
> por delante del eje delantero, y ninguna de las tres fotos muestra eso.

---

## 2. Offset X de la cámara respecto al LiDAR (el paralaje)

### Por qué importa

`_intentar_capturar_poste()` asocia el color que ve la cámara con el cluster que
ve el LiDAR. Los dos sensores miran desde puntos distintos: la cámara está por
delante y mucho más abajo. Un poste que la cámara ve centrado **no** está en el
ángulo 0 del LiDAR, y el desajuste crece a corta distancia igual que en la
sección 1.

### Estimación provisional

```
CAMARA_X_REL_LIDAR = 47.0   # mm por delante del eje del LiDAR (±10)
CAMARA_Y_REL_LIDAR =  0.0   # centrada dentro de la incertidumbre
CAMARA_Z           = 15.0   # mm sobre el piso (el lente cuelga del beam amarillo)
```

Ojo con esto último: la cámara está a ~15 mm del piso y el LiDAR a 90 mm. No
solo hay paralaje horizontal, es que **los dos sensores ven el poste a alturas
muy distintas**. A 15 mm el lente ve la base del poste y el reflejo del mat; es
la causa más probable de que los umbrales HSV de casa no sobrevivan a la pista.

### Protocolo A, con regla (2 minutos)

Mismo montaje que la sección 1:

1. `c` = distancia de la cara **trasera** de la carcasa del LiDAR a la **cara
   frontal del barril del lente**.
2. `d` = profundidad total de la carcasa del C1 (mídela, son ~55 mm).
3. `CAMARA_X_REL_LIDAR = c - d/2`.
4. Lateral: distancia de cada costado de la carcasa del LiDAR al borde
   correspondiente de la placa de la cámara; la diferencia media es
   `CAMARA_Y_REL_LIDAR`.

### Protocolo B, sin regla (sale gratis de la captura de la sección 3)

Las tres poses laterales del checklist (`poste_rojo_600_der250`,
`poste_rojo_450_der120`, `poste_verde_450_izq200`) más las seis centradas dan,
por cada pose, un par de observaciones del **mismo** poste:

- la cámara reporta el centroide horizontal `cx` (px)
- el LiDAR reporta el centroide del cluster `(x_L, y_L)` (mm)

que se relacionan por

```
cx - cx0 = f * (x_L - Δy) / (y_L - Δx)
```

con cuatro incógnitas: la focal en píxeles `f`, el centro óptico `cx0`, y los dos
offsets `Δx`, `Δy`. Con nueve poses de desplazamiento lateral variado el ajuste
por mínimos cuadrados queda sobredeterminado y sale sin tocar una regla — y de
paso calibra la focal real, que ahora mismo es un valor nominal supuesto
(`FOV_H_CAMARA = 66.0`). `revisar_captura.py` ya extrae los dos términos (`cx`
por frame y `centroide_xy` por cluster), así que con el informe basta.

**Haz igual el protocolo A.** El B es una verificación cruzada, y si los dos no
coinciden dentro de ±10 mm hay algo mal en la asociación cámara-LiDAR que
conviene encontrar antes de la competencia, no durante.

---

## 3. Captura en la pista real (la única bloqueante de verdad)

La captura de casa sirve para la IMU y para nada más: sin carriles de 1000 mm ni
muros de 100 mm no se puede calibrar HSV con la iluminación real, ni comprobar
que el clustering separa poste de esquina a las distancias que de verdad
ocurren, ni validar la extracción de rectas de pared.

### En la pista

```bash
python3 capturar_pista.py --checklist
```

imprime las 18 poses a tomar. Una corrida del script por pose, con el robot
**quieto** (una captura en movimiento no sirve: no se sabe dónde estaba el robot
en cada barrido). Cada pose graba:

- `escaneos.jsonl` — el barrido crudo completo del LiDAR, una línea por barrido
- `frames/*.png` — frames BGR 320×240, **crudos**, para poder reprocesar HSV
- `frames.jsonl`, `pose.json` — índice y metadatos

La tracción va apagada durante toda la captura, así que no hay riesgo de que el
robot se mueva.

```bash
python3 capturar_pista.py carril_centrado
python3 capturar_pista.py poste_rojo_600
# ...una por cada pose del checklist
```

Mide con cinta las distancias que pide cada pose y **anótalas**: son la verdad de
terreno contra la que se compara lo que reportó el LiDAR.

### Al terminar

```bash
tar czf capturas_pista.tar.gz capturas_pista/
```

y bájala a la laptop. Ahí:

```bash
python3 revisar_captura.py capturas_pista/
```

Por cada pose imprime el ajuste de recta a cada muro (distancia perpendicular,
ángulo, RMS del residuo), lo compara con el estimador de dos haces que corre en
carrera, lista los clusters con su clasificación, y saca los percentiles H/S/V
del contorno detectado. Genera además `informe_captura.json` con los perfiles de
360 bins completos.

> **Nota ya detectada con datos sintéticos:** `construir_perfil_360()` se queda
> con el **mínimo** de cada bin de un grado. Con el ruido de ~15 mm del C1 y
> varias muestras por bin, eso sesga todas las distancias hacia abajo en
> aproximadamente un sigma de ruido. En una prueba con un pasillo de 1000 mm
> exactos y sigma 8 mm, el ancho reconstruido salió 986 mm. No es un error del
> análisis, es del perfil: si se quiere precisión milimétrica en el parqueo hay
> que usar la mediana del bin, no el mínimo. Para evitar choques el mínimo está
> bien y es conservador — son objetivos distintos.

---

## 4. Centro mecánico del servo y radio de giro

No bloqueante, pero `LIMITE_DER = 70` / `LIMITE_IZQ = 115` sobre un `CENTRO = 90`
es asimétrico, lo que ya sugiere que el centro mecánico real no es 90.

### Centro real (sin cinta métrica, lo resuelve la IMU)

```bash
python3 medir_direccion.py centro
```

Avanza en recta ~2 s por cada comando de servo entre −6° y +6° y mide la
**deriva de guiñada** (°/s) en cada tirada. La deriva es cero en el centro
mecánico verdadero; el script ajusta la recta deriva-vs-comando y despeja el
cruce por cero. Sale un `SERVO_TRIM` directamente utilizable, más la sensibilidad
en °/s de guiñada por grado de comando (que es un dato aparte muy útil: dice
cuánta autoridad real tiene el servo).

Es más preciso que "ver si va recto en 2 m" porque la IMU integra y no depende
del ojo.

### Radio de giro

```bash
python3 medir_direccion.py radio izq
python3 medir_direccion.py radio der
```

Servo al tope, velocidad baja, y la IMU cuenta hasta 360°. Con el tiempo de
vuelta `T` y la velocidad `v` de la sección 5: `R = v·T/(2π)`. Mide también el
diámetro del círculo con cinta y rellena `diametro_medido_mm` en el JSON — dos
métodos independientes que deben coincidir.

> **Detalle que habría falseado esta medición:** `src/pico/main.py` aplica
> `angulo_servo = CENTRO + angulo_objetivo - velocidad_z * KD_ESTABILIDAD`, con
> `KD_ESTABILIDAD = 0.12`. En un giro **sostenido** `velocidad_z` no es cero: a
> 57 °/s ese término resta ~6.9° de servo, casi un tercio del rango útil. El
> ángulo de rueda real en curva **no es** el comandado. Por eso la consigna
> ahora acepta un tercer campo opcional (`"velocidad,angulo,kd"`) y el modo
> `radio` manda `kd=0.0`; cualquier consigna de dos campos —es decir, todo
> script de carrera— devuelve el factor a 1.0 automáticamente, así que no puede
> quedarse pegado. Esto vale la pena mirarlo aparte: si el servo pierde 7° de
> autoridad en cada curva cerrada, es candidato a explicar subviraje en pista.

---

## 5. Velocidad vs PWM

```bash
python3 medir_velocidad.py bateria_llena
# ...15 minutos de uso después...
python3 medir_velocidad.py bateria_baja
```

Robot apuntando de frente a un muro recto con ~2.5 m libres. El LiDAR es la
cinta métrica: se muestrea la distancia frontal contra el tiempo y se ajusta una
recta al tramo de régimen (se descarta la rampa de arranque). Mide 40/55/70/90 %.

Sale además el tiempo hasta el 90 % de la velocidad de régimen, que es lo que
falta para el modelo de predicción del tracker.

Validado con datos sintéticos: recupera la velocidad real con menos del 3 % de
error entre 280 y 950 mm/s. El script imprime el RMS del residuo; si pasa de
25 mm, el robot no fue recto y hay que repetir la tirada.

Los números van a la tabla `VELOCIDAD_MM_S` de `geometria_robot.py` y reemplazan
`tracker.MM_POR_SEG_A_PWM100 = 900.0`, que hoy es una suposición sin medir.

---

## 6. Reglamento WRO Future Engineers 2026 — resuelto

Consultado el PDF oficial completo,
*WRO 2026 Future Engineers Self-Driving Cars General Rules*, versión 1 de
diciembre de 2025, 55 páginas.

### 6.1 ¿Completamente dentro, o basta con estar mayormente dentro?

**Completamente dentro.** No hay término medio, pero sí hay puntos parciales:

> "A robot is considered fully parked, when the projection of the robot on the
> mat is fully inside the rectangle between the two markers of the parking lot
> […] and the robot is parked in parallel to the wall of the game field."
> — Apéndice A, sección 6

- **Parqueo completo** (proyección totalmente dentro **y** paralelo): **15 puntos**
- **Parqueo parcial o no paralelo**: **7 puntos**
- Arrancar dentro del parqueo y completar al menos una vuelta: **7 puntos** extra

O sea, quedarse a medias no anula el parqueo, lo degrada a menos de la mitad.

**Criterio de "paralelo", que es medible:** la diferencia entre las distancias de
las dos ruedas de un mismo lado al muro no puede pasar de **20 mm**. Sobre una
batalla de 136 mm eso es un cono de **±8.4°** de guiñada. Es bastante generoso —
el requisito duro no es el ángulo, es la posición.

### 6.2 ¿Tocar los muros de parqueo penaliza?

**Peor que penalizar: termina la ronda.**

> "The parking lot limitations cannot be touched by the robot. When they are
> touched, the robot is stopped and no points for the parking can be scored."
> — Apéndice A, sección 6

Y está en la lista de condiciones de fin de ronda:

> "9.24.7. In Obstacle Challenge: The robot touches the parking lot limitations."

Esto cambia el diseño de la maniobra por completo: **no hay maniobra de tanteo**.
No se puede "acercarse hasta rozar y corregir". Un roce no cuesta unos puntos,
corta la ronda ahí mismo con el cronómetro parado.

### 6.3 Los números de holgura (calculados para *este* robot)

El hueco no es fijo: el juez mide **tu** robot.

> "The width of the parking lot is always 20 cm. The length is variable and
> calculated: 1,5 * length of the robot" — sección 5

Y la Figura 4 muestra que ese 1,5·L se mide **entre las caras internas** de los
dos bloques magenta (200 × 20 × 100 mm, RGB 255,0,255, regla 13.25).

Con el robot actual (222 × 125 mm):

| Magnitud | Valor |
| :- | :- |
| Hueco útil entre caras internas | **333.0 mm** |
| Holgura longitudinal por extremo | **55.5 mm** |
| Holgura lateral por lado (200 − 125) / 2 | **37.5 mm** |
| Tolerancia de paralelismo | **±8.4°** |
| Desalineación máxima que aún cabe geométricamente | 22.3° |

Los 37 mm que tenías son correctos, pero son la holgura **lateral** (perpendicular
al muro exterior, dentro de los 200 mm de profundidad), y ese eje **no está
limitado por los muros magenta** — de ese lado hay muro exterior por fuera y
campo abierto por dentro. El eje que sí está limitado por magenta es el
longitudinal, y ahí hay **55.5 mm por extremo**, no 37.

Consecuencia de diseño: la holgura longitudinal es siempre `0.5 · largo_robot`,
así que **acortar el robot no gana margen absoluto** — lo reduce. Y como tocar
magenta corta la ronda, `MARGEN_MURO_PARQUEO = 25.0` mm en `geometria_robot.py`
deja el objetivo con ~30 mm de colchón real a cada extremo.

### 6.4 De regalo, dos cosas del reglamento que afectan al algoritmo

- **Firma de parqueo por LiDAR.** Los bloques magenta miden 200 mm y se colocan
  con ese lado perpendicular al muro exterior, o sea que **sobresalen 200 mm
  dentro del carril de 1000 mm**. Desde el LiDAR eso es una firma mucho más
  específica que la que usa hoy `navegacion.py` (`TOLERANCIA_FIRMA`, que solo
  compara dos distancias laterales contra las del arranque): dos escalones
  laterales de 200 mm separados 333 mm entre sí, con la lateral cayendo de
  ~1000 mm a ~800 mm. Está codificada en `geometria_robot.firma_lidar_parqueo()`.
- **Después de las tres vueltas los semáforos dejan de obligar.** "On the
  subsequent route to the parking lot, they can be bypassed to the right or left
  as desired. Moving them is still not permitted." La ruta al parqueo puede
  ignorar la regla de color rojo/verde — solo hay que no tumbarlos. Eso libera
  bastante la trayectoria de aproximación.

---

## 7. Qué mandar de vuelta

1. `LIDAR_X`, `LIDAR_Y` y `CAMARA_X_REL_LIDAR` medidos con regla (secciones 1 y 2).
2. `capturas_pista.tar.gz` completo, **más las distancias medidas con cinta** en
   cada pose.
3. `mediciones/direccion_centro.json`, `direccion_radio_izq.json`,
   `direccion_radio_der.json` (con el `diametro_medido_mm` rellenado).
4. `mediciones/velocidad_bateria_llena.json` y `velocidad_bateria_baja.json`.

Con eso quedan cerradas las seis y el algoritmo se puede escribir sobre números
medidos en vez de supuestos.
