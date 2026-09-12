# Sesión de banco del 06-09-2026 — medidas del chasis, de la bahía y del movimiento

Sesión dedicada a **medir**, no a programar. El punto de partida era que la
maniobra de parqueo no cierra y que las constantes sobre las que se razonaba
—dimensiones del robot, tamaño de la bahía, velocidad por PWM— venían de
fuentes distintas y en varios casos no se habían comprobado nunca con regla.

Todo lo de aquí está volcado en `ronda_nueva/configuracion.json`, con la
justificación al lado de cada valor. Este documento explica **cómo** se midió
y **qué quedó sin cerrar**.

---

## 1. Chasis (regla, sobre mesa)

Referencia: todo se mide desde el **eje del LiDAR** proyectado al suelo, que es
el origen del marco del robot (`modelos.py`: x a la derecha, y adelante).

| Medida | Antes | Ahora | Clave |
|---|---|---|---|
| Largo total | 222 | **210** | `chassis.length_mm` |
| Ancho (ruedas rectas) | 125 | **130** | `chassis.width_mm` |
| Ancho, volante a tope izq. | — | **135** | `chassis.width_full_lock_left_mm` |
| Ancho, volante a tope der. | — | **140** | `chassis.width_full_lock_right_mm` |
| Batalla | 136 | **140** | `chassis.wheelbase_mm` |
| Voladizo trasero | 60 | **55** | `chassis.rear_overhang_mm` |
| LiDAR desde el eje trasero | 133 | **137** | `chassis.lidar_forward_from_rear_axle_mm` |
| LiDAR → borde izquierdo | 61 | **63** | `chassis.lidar_to_left_edge_mm` |
| LiDAR → borde derecho | 45 | **66** | `chassis.lidar_to_right_edge_mm` |
| Altura del plano de barrido | — | **69** | `chassis.lidar_plane_height_mm` |
| Ultrasonido → tope trasero | — | **34** | `chassis.ultrasound_rear_to_tail_mm` |

**Sobre los bordes.** Los 45/61 viejos no eran una medida del chasis: marcaban
el ángulo donde el LiDAR se veía **la propia rueda**. Ese eco ya no existe (ver
sección 3), así que ahora las dos claves son la extensión física real. Que 63+66=129
cuadre con los 130 de ancho es la comprobación de que están bien tomadas.

**El ancho a tope de volante es una medida nueva y ningún modelo la usaba.**
Durante los arcos del parqueo el robot no es un rectángulo de 130: es de 140.

---

## 2. Bahía de parqueo (regla, sobre la pista montada)

| Medida | Valor | Clave |
|---|---|---|
| Hueco útil (cara interior a cara interior) | **330 mm** | `parking.bay_length_mm` |
| Entre centros de delimitadores | 350 mm | — |
| Profundidad | **200 mm** | `parking.bay_depth_mm` |
| Huella del delimitador | **200 × 20 mm** | `parking.delimiter_*_mm` |

Las dos primeras son coherentes: 330 + 20 de un delimitador = 350. El
`bay_length_mm` anterior (390) estaba **60 mm largo**.

### Lo que esto resuelve: la maniobra de dos arcos NO cabe

Se rehízo la simulación con estas medidas, barriendo todas las distancias de
aproximación, posiciones finales, rumbos pico y puntos de arranque:

```
Bahía útil 330 x 200 | robot 210 x 130 (140 a tope)
Holgura estática: 120 mm longitudinal, 70 mm lateral

DOS ARCOS, radios reales 228/260:
  entrando con el radio grande primero -> NO CABE, faltan 18 mm
  entrando con el radio chico primero  -> NO CABE, faltan 33 mm
```

Sensibilidad al voladizo: no cabe con **ningún** valor entre 40 y 80 mm
(faltan de 11 a 31 mm). El techo geométrico sale en **r ≤ 180 mm**, que es
exactamente la cifra que el equipo había calculado por su cuenta en su día —
dos modelos independientes cayendo en el mismo número.

**Conclusión: el diseño de entrada en varios tiempos con vaivenes de
`estacionamiento.py` era la decisión correcta y ahora está confirmada con
números.** Y el vaivén solo tiene que recuperar 18 mm, que es poco: un solo
tramo corto debería bastar, y `max_shuffles = 6` sobra de largo.

**Hallazgo aparte: el lado de entrada NO es indiferente.** Estaba anotado que
daba igual, porque el corrimiento lateral depende de `r_entrada + r_salida` y
es simétrico. Eso es cierto para el corrimiento pero **falso para la envolvente
barrida**: usar el radio grande en el arco de entrada cuesta 348 mm de hueco y
usarlo en el de enderezado cuesta 363. Son 15 mm gratis por elegir bien.

---

## 3. Auto-eco del LiDAR (`diag_eco_volante.py`, el robot no se desplaza)

Nueve ángulos de servo de −25 a +25, vuelta completa, ecos bajo 500 mm.

**El eco de la rueda desapareció.** Los conteos con el volante recto y a tope
son estadísticamente idénticos, y en los sectores laterales que enmascara
`self_echo_*` (30-110° y 250-330°) **no hay un solo punto** bajo 500 mm a
ningún ángulo. Lo arregló el `mastilfix` del 05-09.

> **Pendiente:** `self_echo_base_mm`, `self_echo_mm_per_deg` y sus dos sectores
> probablemente están tapando datos buenos para un problema que ya no existe.
> Recuperar esa cobertura angular es gratis, pero hay que verificarlo antes de
> quitar el enmascarado.

**Lo que sí ciega el LiDAR por detrás es el soporte del ultrasonido.** Todos
los ecos están entre 140° y 200°, a **32-44 mm**, y aparecen igual con el
volante recto. `blind_sectors_deg: [[140, 213]]` ya lo cubre.

**Consecuencia para el parqueo, y es seria.** `diag_arranque` reporta "pared
trasera" con 35 puntos, pero el LiDAR está ciego justo hacia atrás: esos puntos
vienen de los hombros, en oblicuo. La "pared trasera" del LiDAR **no es una
medida, es una extrapolación**, y discrepa ~350 mm del ultrasonido en la misma
pose. `ControlEstacionamiento._trasera_mm()` fusiona hoy las dos fuentes como
si fueran equivalentes, y no lo son.

---

## 4. Velocidad por PWM (`herramientas/medir_velocidad.py`, nuevo)

Método: el LiDAR hace de odómetro contra la pared frontal. Tres duraciones
(2/3/4 s) por sentido y ajuste por mínimos cuadrados, para que la **pendiente**
sea la velocidad limpia y el arranque y la inercia de frenado queden aparte en
el término independiente.

```
tanda 1   adelante v = 83.7 mm/s   reversa v = 83.5 mm/s
tanda 2   adelante v = 86.1 mm/s   reversa v = 84.6 mm/s
```

**`control.mm_s_per_pwm` = 3,85** (era 4,0, que estaba casi bien).

- **El 6,7 que implicaba la tabla del README está mal por un 76 %.** La
  estimación de ~3,8 que se había sacado de los CSV y se descartó por
  "cota inferior contaminada" era en realidad el valor bueno.
- **Adelante y reversa coinciden dentro del 0,2 %**: no hacen falta dos
  constantes.

---

## 5. Velocidad girando, y los radios

A tope de volante el robot **no** va a 84 mm/s. De las velocidades de rumbo de
la IMU (15,70 °/s a tope izquierdo, 13,65 a tope derecho) contra los radios
228/260 salen **62,5 y 61,9 mm/s**. Que dos casos independientes converjan en
62 confirma dos cosas a la vez: que **228/260 son correctos**, y que girando se
pierde un 26 % de velocidad por el restregado de las ruedas.

`control.turn_speed_full_lock_mm_s = 62.0`.

### Lo que quedó SIN cerrar: el radio en REVERSA

La misma cuenta con la reversa a tope izquierdo (11,66 °/s) da **R ≈ 306 mm**,
un 34 % más que los 228 de marcha adelante. **El parqueo entra marcha atrás**,
así que la geometría real de la maniobra sería peor que la simulada.

**Pero es una inferencia, no una medida**: supone que la velocidad girando en
reversa es también 62 mm/s, y eso no se comprobó. Dos intentos de medirlo
automáticamente fallaron:

1. Calcular el arco como `v·t` con la velocidad **en recta** infla el radio un
   34 %, justo porque girando se va a 62 y no a 84.
2. Seguir la distancia perpendicular a la pared frontal contra el rumbo dio
   residuos de 72 a 275 mm y radios negativos: al rotar 50°, el sector de ±8°
   barre paredes distintas y esquinas. Haría falta el ajuste de recta de
   `percepcion_lidar`, no una mediana de rayos.

**Cómo cerrarlo (2 minutos, con cinta):** marcar la posición, girar **en
reversa** a tope hasta 90°, marcar otra vez, medir la cuerda entre marcas.
`R = cuerda / √2`. Repetir al otro lado.

---

## 6. EL HALLAZGO PRINCIPAL: en la pose parqueada, la FSM está ciega

Con el robot **colocado a mano dentro de la bahía**, en la pose final correcta,
se ejercitaron los mismos caminos de código que usa la FSM
(`PercepcionLidar.procesar` y los helpers de `ControlEstacionamiento`):

```
=== LO QUE VERIA LA FSM DE PARQUEO ===
  bahia a la IZQUIERDA:
     _lateral_mm  = None
     _trasera_mm  = 44.0   (ultrasonido crudo 44.0)
     _paralelo    = None deg
     _dentro()    = False   [exige lateral <= 140 y |paralelo| <= 6]
```

El LiDAR **sí tiene puntos** contra ese muro —mínimo izquierdo 74,5 mm, estable
en los 12 barridos— pero `paredes.izquierda` sale `None` en todos. El ajuste de
pared falla, y `VERIFICAR` exige justamente lateral y paralelo.

**Consecuencia: aunque la maniobra saliera perfecta, la FSM nunca declararía
LISTO.** Se quedaría dando vueltas hasta el timeout y saldría por FALLO. Esto
puede ser el bloqueante real del parqueo, y no es un problema de maniobra ni de
geometría: es de percepción.

### La causa, aislada

Barriendo el umbral sobre el mismo barrido guardado:

```
umbral wall_min_length_mm -> pared izquierda encontrada?
   220 -> no      100 -> no
   180 -> no       80 -> no
   150 -> no       60 -> SI: 77.9 mm, normal -93.4, 38 pts, residuo 3.0
```

`lidar.wall_min_length_mm` está en **220** y aquí hace falta **60**. Es un
umbral pensado para la carrera —donde exigir 220 mm de segmento evita confundir
un pilar con una pared— aplicado a una situación donde es geométricamente
imposible cumplirlo: con el flanco a 11 mm del muro, el sector lateral solo
abarca unas decenas de milímetros de pared.

**Y cuando sí la encuentra, la medida es buena**: 77,9 mm con 38 puntos y
residuo 3,0; normal −93,4°, o sea 3,4° de paralelo, dentro de la tolerancia de
6°; y 77,9 < `inside_lateral_mm` (140). Con esa recta, `_dentro()` daría True y
la verificación cerraría.

> **No basta con bajar el umbral a 60 en el JSON**: eso degradaría el
> seguimiento de pared durante toda la ronda. Hace falta un umbral aparte que
> solo aplique en los estados de parqueo, o que `PercepcionLidar` acepte un
> mínimo distinto cuando `lado_parqueo != 0`.

### De paso, la trasera quedó retratada

En esa misma pose, el ultrasonido lee **44 mm** y el LiDAR reporta una "pared
trasera" a **1777 mm con calidad 0,95 y residuo 3,0**. La FSM hace lo correcto
(`_trasera_mm` devolvió 44,0, el valor del ultrasonido), pero cualquier código
que se fíe de `paredes.trasera` recibe un número limpio, convincente y
equivocado por metro y medio. Ver la sección 3.

---

## 7. Pendientes que salen de esta sesión

0. **`wall_min_length_mm` en los estados de parqueo** (sección 6). Es lo primero.
1. **Radio en reversa, con cinta.** Es la única entrada geométrica del parqueo
   que sigue sin medir, y la inferencia dice que es un 34 % peor de lo supuesto.
2. **Los 40 mm de la separación de la bahía.** El detector mide 389-391 y la
   regla dice 350 entre centros. No cuadra con ninguna lectura posible.
   `lidar.bay_expected_separation_mm` se deja en 390 a propósito: bajarlo sin
   entender la discrepancia rompe el único detector que funciona.
3. **`approach_lateral_mm = 270` deja cero holgura.** Con el volante a tope el
   semiancho es 70, y 270 − 70 = 200, exactamente la profundidad de la bahía:
   el borde roza la punta de los delimitadores al pasar. Subirlo pide corridas
   de pista.
4. **`self_echo_*` probablemente sobra** (sección 3).
5. **`_trasera_mm()` mezcla una medida con una extrapolación** (sección 3).
6. **El firmware de la Pico en el robot es más nuevo que el del repo y no está
   commiteado**: `/home/pi/pico_nuevo/main.py` son 17066 bytes del 03-09 contra
   16057 del 01-09 en `src/pico/`. Mil bytes que solo existen en la Pi.

---

## 8. Lo que se cerro en codigo con estas medidas (misma fecha, por la tarde)

Rama `medidas/banco-0609`, paquete `ronda_nueva`. Los pendientes 0 y 5 de la
lista de arriba quedan cerrados; el 1, el 2, el 3 y el 4 siguen abiertos y
siguen pidiendo pista o cinta metrica.

* **Pendiente 0 — `wall_min_length_mm` en el parqueo.** `PercepcionLidar` acepta
  ahora un minimo de segmento distinto, y solo para la pared del LADO de la
  bahia (`_largo_minimo_mm`): escalado por la distancia medida
  (`wall_min_length_parking_ratio` 0,75) con suelo en
  `wall_min_length_parking_mm` (60) y techo en el valor de carrera (220), que
  se vuelve a alcanzar a partir de 293 mm. Frontal y trasera conservan los 220
  a proposito: con un minimo global de 60, un delimitador de 200 mm de huella
  se clasifica como muro FRONTAL a 171 mm teniendo el de verdad a 1499 --
  comprobado sobre pista sintetica, y es el test que fija el candado.
* **Pendiente 5 — `_trasera_mm()`.** La holgura trasera del parqueo sale ahora
  SOLO del ultrasonido, y descontando los 34 mm de `ultrasound_rear_to_tail_mm`
  (los 44 de aquella pose eran 10 mm de sitio real). La trasera del LiDAR no
  participa ni para bajar la medida. Sin ultrasonido valido la respuesta es
  `None` = SIN EVIDENCIA, y retroceder con tres barridos seguidos sin eco
  termina en FALLO en vez de seguir a ciegas.
* **De propina, tres cosas que salieron al hacer lo anterior.** El paralelismo
  tiene segunda fuente (rumbo de la IMU contra la referencia que se toma al
  salir de ALINEAR), asi que los tramos ya no cortan por reloj: los relojes
  pasan a ser red de seguridad hacia FALLO y `leg_timeout_s` sube de 3,5 a 6,0
  porque 42 grados a los 11,66 deg/s de la reversa son 3,6 s y el 3,5 anterior
  habria matado el arco bueno. El arco de entrada fuerza el radio grande
  tambien con la bahia a la derecha. Y la correccion lateral de ALINEAR tenia
  el signo cambiado: cuanto mas cerca del muro, mas giraba hacia el.

Nada de esto se ha probado en pista todavia. La verificacion en el robot es
`herramientas/diag_pose_bahia.py`, que ahora imprime el mismo barrido con y sin
lado de parqueo para que se vea si el umbral relajado es lo que desbloquea la
lateral.
