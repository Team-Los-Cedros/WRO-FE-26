# `src/pi5` — Ronda de obstáculos para la Raspberry Pi 5

Reescritura del cerebro del robot para la Pi 5. **No toca nada de `src/pi3B`**:
es un paquete aparte, con su propia configuración, sus propios drivers y sus
propias pruebas. La Pi 3B sigue arrancando exactamente igual que antes.

```
src/pi5/
  comun/lidar_driver.py        driver del RPLIDAR C1 (copiado tal cual: está probado)
  herramientas/calibrar_suelo.py
  ronda_nueva/
    modelos.py                 tipos y convenciones de signo
    geometria_suelo.py         homografía imagen → suelo
    vision_pista.py            espacio libre, pilares, magenta y líneas
    percepcion_lidar.py        paredes como rectas, objetos, hueco de bahía
    fusion.py                  cámara + LiDAR, en milímetros
    localizacion.py            pose dentro de la recta, sin SLAM
    mapa_pista.py              las doce casillas y sus votos
    planificador.py            la trayectoria y la ley de dirección
    piloto.py                  la máquina de estados
    estacionamiento.py         la maniobra de parqueo
    ronda_nueva.py             punto de entrada
    tests/                     132 pruebas, todas sin robot
  herramientas/
    calibrar_desde_muros.py    homografía con las paredes, sin colocar nada
    calibrar_suelo.py          cobertura del montaje y homografía con pilares
    capturar_pareja.py         una correspondencia pilar-LiDAR por pose
    diag_arranque.py           comprobación de los tres sensores, sin mover
    diag_cerca.py              qué ve el LiDAR a menos de 600 mm, pasivo
    diag_eco_volante.py        si la propia rueda entra en el barrido
    diag_mastil.py             mide la oclusión fija y propone la máscara
    diag_pilares.py            sesgo cámara-LiDAR sobre un poste real
    bench_vision.py            coste por cuadro y cadencia real de la visión
    reescalar_camara.py        cambia la resolución sin perder la calibración
```

---

## 1. El cambio de fondo

En la versión de la Pi 3B, **un pilar interrumpía la conducción**: la máquina
de estados saltaba a `APROXIMACION`, luego a `SOBREPASO`, luego a `RECENTRADO`,
y el seguimiento de pared se apagaba mientras tanto. Medido sobre los CSV del
equipo, evadir se llevaba el **45–52 % de la ronda** —más que girar esquinas— y
la eficiencia de rumbo caía al 6–23 % en cuanto había pilares.

Aquí un pilar **no interrumpe nada: mueve el carril**. El robot siempre está
haciendo lo mismo —seguir un perfil de distancia al muro exterior dentro de la
recta— y los pilares son nodos que deforman ese perfil.

Es la idea del **2.º puesto de 2025** (KMIDS), que controla un único
`targetOuterWallDistance`, con dos añadidos:

1. El offset objetivo se calcula desde la posición **medida** del pilar, no de
   una tabla de cuatro constantes. Con la cámara dando milímetros no hace falta
   suponer dónde está el poste.
2. Entre consignas hay una **rampa**, no un escalón, y la rampa se dimensiona
   con la capacidad real de maniobra del chasis.

Del **1.er puesto** (`ejm22`) se toma la otra idea central: el **polígono de
espacio libre**. Se aísla el trozo de lona sobre el que el robot está parado y
se rellena su contorno; a partir de ahí, «¿este blob está apoyado en la pista?»
es un `AND` de dos máscaras.

---

## 2. Lo que hace la cámara nueva posible

Con la cámara alta y trasera, casi todo lo que importa toca el suelo dentro del
cuadro. Una **homografía 3×3** convierte el punto de contacto de cada blob en
(x, y) en milímetros del marco del robot. Eso cambia tres cosas de raíz:

| Problema conocido | Cómo lo resuelve |
| --- | --- |
| El plano del LiDAR **pasa por encima** del pilar a partir de cierta distancia (el rojo a 1088 mm no se veía) | La cámara lo mide igual de bien a 400 que a 1800 mm |
| El pilar se apagaba a una mediana de **216 mm** por la puerta angular de 15° | El criterio pasa a ser el ancho físico en mm, que no depende de la distancia |
| El sesgo de guiñada de **+3,57°** gastaba el 36 % de la puerta de fusión | Cámara y LiDAR hablan en mm: asociar es el vecino más cercano |
| `COLOR:SIN_SENSOR` en la Pi 5 deja sin sentido de giro ni conteo de vueltas | Las líneas azul y naranja se leen **con la cámara** |

Si todavía no hay homografía, la visión **sigue funcionando**: estima la
distancia por la altura aparente del poste (100 mm) y el bearing por la
columna. Peor, pero utilizable el mismo día que montes la cámara.

---

## 3. Lo que el sorteo oficial obliga a tener en cuenta

Leyendo el randomizer oficial de la temporada (`Future_Engineer_2025_Randomizer`)
aparecen dos hechos que cambian el dimensionado y que no se deducen del
reglamento a secas:

- **Cuando una recta lleva dos señales, son siempre p1 y p3.** Nunca dos
  adyacentes. O sea **953 mm** entre ellas, no 480.
- Las dos filas laterales están a **380 y 574 mm** del muro exterior.
  (380 − 50 = 330 ≈ los 333 mm de `narrowest_gap_wall_to_pillar_mm` del equipo:
  cuadra con la escala de 1 px = 1,5 mm de la imagen del sorteo.)
- Las tres posiciones longitudinales caen a **1000 / 1500 / 2000 mm** del muro
  de enfrente, que son exactamente las bandas que usaba el 2.º puesto.

Sin eso se dimensiona el planificador para un caso que el sorteo no puede
producir, y se acaba pidiendo maniobras que el chasis no traza.

---

## 4. Qué se ha validado, y con qué números

Todo esto corre en el escritorio, sin robot:

| Prueba | Resultado |
| --- | --- |
| Paredes por LiDAR, 4 segmentos × 6 poses, ruido 5 mm | 24/24 dentro de 30 mm; rumbo dentro de 3° |
| Homografía: ida y vuelta, DLT, degeneraciones | exacta a 1e-6 |
| **Las 28 disposiciones del sorteo × 2 sentidos** | 56/56 rebasadas por el lado correcto, **33,8 mm** de holgura mínima |
| Lo mismo con 25 mm de ruido en el pilar y 12 en la pose, 3 semillas | 168/168, **24,9 mm** de holgura mínima |
| Vuelta completa simulada, sin pilares | **12 esquinas en 111,5 s** |
| Vuelta completa simulada, **8 pilares** (2 por recta) | **12 esquinas en 130,0 s**, 0 retrocesos, mapa completo |

Los 130 s con ocho postes contra los 111 s vacía son un **+16 %**, no el doble.
Ese número es la medida de que la evasión dejó de costar tiempo.

> El simulador usa el modelo de bicicleta con la batalla (136 mm) y los radios
> de giro **medidos** en el chasis (228 mm a la izquierda, 260 a la derecha), y
> la colisión se calcula contra el rectángulo del robot, no contra su centro.
> No sustituye a la pista: sustituye a *descubrir errores de geometría en la
> pista*. Encontró cuatro, y cada uno habría costado una tarde.

---

## 4bis. Lo que ya se comprobó EN EL ROBOT (04-09, tarde)

Con la cámara nueva ya montada y la Pi 5 encendida, sin mover el robot:

| Comprobación | Resultado |
| --- | --- |
| LiDAR | 10,0 Hz, 461-470 puntos/barrido, **cero barridos perdidos** |
| Paredes ajustadas | frontal 1692 mm, izquierda 540, derecha 427; residuos **4,4 – 6,8 mm** |
| Cámara | 1280×720, suelo = 50 % del cuadro |
| **Visión por cuadro** | **19,2 ms** de mediana (era 49 ms antes de optimizar) |
| Homografía del suelo | **calibrada**: suelo visible de **210 a 3121 mm** |
| Líneas de piso | azul y naranja **detectadas**, proyectadas a 786 y 964 mm |
| Pico | telemetría OK, ultrasonido 1089 mm, `WD:STOP` (normal en reposo) |
| Sensor de color | `SIN_SENSOR`, como se esperaba — ya no bloquea nada |

**La calibración salió de las propias paredes**, sin colocar un solo pilar:
`herramientas/calibrar_desde_muros.py`. La línea donde el muro toca la lona es
una recta del plano del suelo, el LiDAR la mide en milímetros y la cámara la ve
como el borde superior de la región de suelo. 317 correspondencias repartidas
por todo el ancho del cuadro, ajustadas a 2,3 px de residuo mediano.

Resultado: **altura 177 mm, cabeceo 10,44°, guiñada −3,0°**.

> Dos avisos honestos sobre esos números. (1) El desplazamiento longitudinal de
> la cámara **no es observable** por este método: da igual suponer −220 o +40 mm,
> el residuo apenas cambia. Se dejó en −80. (2) Por eso la altura solo queda
> determinada a **±10 mm**. Medir altura y voladizo con regla cierra las dos
> cosas de golpe; luego `--altura-mm N` reajusta cabeceo y guiñada con la altura
> fija.

La verificación es visual y está en `verif_homografia.jpg`: se proyectan las
tres rectas del LiDAR sobre el cuadro y **caen sobre la base de los muros**.

### La ronda entera, en simulacro sobre sensores reales

`--simulacro` monta la tubería completa —percepción, localización, mapa,
planificador, FSM y CSV— sobre los sensores de verdad y a su ritmo de verdad,
pero **no envía ni una consigna al motor**. Es la prueba más parecida a correr
sin arriesgar el chasis, y se puede lanzar con el robot en el banco:

```bash
timeout -s INT 40 python3 -u -m ronda_nueva.ronda_nueva   --simulacro --arranque-inmediato --sin-parqueo
```

Medido el 04-09 con el robot quieto dentro de la pista:

| | |
| --- | --- |
| Cadencia del ciclo | **10,0 Hz**, la del LiDAR |
| **Ciclo de decisión** | **3,07 ms** de mediana, 4,0 ms máximo |
| — del que percepción LiDAR | 2,98 ms |
| Edad del barrido al decidir | 0,30 ms de mediana, 8,6 ms máximo |
| Sentido de giro | resuelto **en el primer ciclo**, por la línea del piso |
| Falsos positivos de pilar | **0** en 283 ciclos |
| Deriva de rumbo tras compensar | ≤ 5° en 33 s |

`barridos_descartados` marca 27 de 310: no es que el control llegue tarde
—tarda 3 ms— sino que el driver publica algún barrido parcial de más y el buzón
se queda con el nuevo, que es lo correcto.

### Seis cosas que solo aparecieron al conectar el robot

1. **`heading` y `color_piso` del enlace con la Pico son métodos, no
   propiedades**, y el driver del LiDAR expone `hilo_lectura`, no `bucle`. Dos
   errores míos que la simulación no podía ver porque ahí no hay drivers.
2. **La semilla del polígono de suelo caía sobre la cúpula del LiDAR**, que
   asoma por abajo en el cuadro. Movida a `[0.5, 0.72]` y la cúpula enmascarada.
3. **Los umbrales HSV heredados no veían ninguna línea del piso**: pedían
   S ≥ 110 y lo medido sobre la lona real es naranja S 57-88, azul S 52-73.
4. **Reducir el cuadro a la mitad borra las líneas finas.** Al promediar con la
   lona blanca, la naranja pasaba de 2074 píxeles saturados a 30. Por eso los
   pilares van a media resolución y las líneas a resolución completa.
5. **El giroscopio derivaba +20,9 °/s con el robot inmóvil**, perfectamente
   lineal. Ver más abajo: es el hallazgo grave del día.
6. **19 falsos positivos de pilar en 358 ciclos sin ningún pilar en el campo**:
   bultos del muro izquierdo (a 615 mm con el muro a 540) y la propia estructura
   del robot colándose por el borde del sector ciego. Uno llegó a mover el
   carril objetivo. Arreglado con tres filtros: descartar lo que está pegado a
   una pared ajustada, descartar lo que cae dentro de la huella del robot, y no
   dejar que un objeto **sin color** deforme el carril (frena, pero no se le
   elige lado: la regla WRO solo habla de rojo y verde).

### Con la pista montada (04-09, noche)

Con los pilares y el cajón puestos, en dos poses:

| Comprobación | Resultado |
| --- | --- |
| Pilar **rojo** en el carril | **59 detecciones de 59 ciclos**, (208, 1470) mm, **±0 mm** de dispersión |
| — y solo por cámara | el LiDAR **no lo ve** a 1,47 m: su plano le pasa por encima |
| Clasificación en el mapa | lo asigna a la **recta siguiente**, que es donde está (al otro lado de la esquina) |
| Muros del cajón | **59 de 59 ciclos**, (−9, 268) mm |
| Líneas azul y naranja | detectadas en las dos poses |
| Falsos positivos | 0 |

Que el rojo salga **solo por cámara a 1,47 m** es la demostración directa de por
qué se hizo la homografía: es exactamente el agujero del LiDAR que la bitácora
del equipo llevaba documentado.

### Dos trampas de percepción que solo se ven con cosas en el campo

**1. Los delimitadores del cajón del banco son violeta, no magenta.**
Medido: H 127-131, S 85-140, V 34-77. El magenta oficial de WRO (#F702F9) daría
H 150, S 253, V 249 — nada que ver. Peor: ese violeta cae **dentro** del rango
de la línea azul del piso (H 125-132), así que el delimitador se leía como
línea de sentido, y eso habría mandado el robot a dar la vuelta al revés.

Ningún umbral de color los separa con margen. Se distinguen por **geometría**:
se proyectan al suelo el borde de arriba y el de abajo de la mancha, y si el de
arriba se va más de 2,2 veces más lejos (o pasa del horizonte) es que está de
pie, no tumbada. Se probaron antes dos filtros peores —exigir que fuera mucho
más ancha que alta, y mirar si había lona por encima— y los dos fallaban: el
primero descartaba también la línea azul de verdad, y el segundo se colaba
porque por encima del delimitador se ve la lona del otro lado del muro.

Se aceptan los dos rangos de color. **Si en competencia los delimitadores son
los oficiales, hay que revalidarlo allí.**

**2. El `AND` con el polígono de espacio libre perdía los postes pegados al muro.**
Un poste en medio de la lona es un *agujero* del componente de suelo y al
rellenar el contorno queda dentro. Uno pegado al muro del fondo no: es una
*muesca* del borde superior, y el `AND` lo dejaba fuera. Medido en el robot: un
rojo perfectamente visible a 1,4 m, que pasaba todos los filtros de forma y
cuyas dos estimaciones de distancia coincidían al 9 %, se quedaba en **7 píxeles
de 725**.

Ahora se pregunta por la lona que hay **justo debajo de la base**, en una banda
fina y pegada. Cubre los dos casos y es la prueba original del campeón de 2025.

### Lo que queda abierto: la cámara mide un 19 % más lejos que el LiDAR

Sobre el mismo pilar verde, con los dos sensores viéndolo a la vez:

| | |
| --- | --- |
| LiDAR (centroide crudo) | (−137, 694) mm, clúster de **43 mm** |
| Cámara (contacto con el suelo) | (−227, 840) mm |
| Discrepancia | **173 mm de mediana** sobre 49 parejas |

Dos cosas se arreglaron a raíz de esto:

* **`pillar_width_mm` estaba en 100 y son 50.** El clúster del LiDAR mide 43 mm:
  son las señales de 5×5×10 cm del reglamento. Inflaba la corrección de
  centroide de la fusión y gastaba 25 mm de holgura de más en el planificador.
* **La herramienta de calibración ajustaba con el centro óptico nominal (640) y
  la ronda reconstruía con el medido (704,148).** Son 64 px, casi 4° de
  guiñada. Ahora la herramienta escribe la **matriz explícita**, así que no
  pueden desalinearse, y el ajuste mejora a 1,53 px de mediana (era 2,22). La
  guiñada pasa de −3,0° a +0,75°: estaba absorbiendo ese desplazamiento.

Pero el fondo del asunto sigue abierto. Se intentó un **ajuste conjunto**
(paredes + la correspondencia del pilar, con altura, cabeceo, guiñada y los dos
desplazamientos libres) y el resultado **no es de fiar**:

| | residuo de muros | error al pilar | desplazamiento lateral |
| --- | --- | --- | --- |
| Solo paredes | **4,93 px** | 150 mm | 0 (fijado) |
| Conjunto, rejilla gruesa | 8,74 px | 23 mm | 120 mm |
| Conjunto, refinado | 6,78 px | 61 mm | 90 mm |

Para cuadrar el pilar hay que empeorar las paredes y meter 90-120 mm de
desplazamiento lateral entre cámara y LiDAR, que es demasiado para un mástil.
**Con una sola correspondencia de pilar y cinco parámetros libres el problema
está mal condicionado**: el ajuste solo cambia una evidencia por la otra. Por
eso se conserva el ajuste por paredes, que sí está bien condicionado.

**Impacto real, para decidir si urge:** por debajo de 1400 mm la fusión usa la
posición del LiDAR, así que la evasión cercana y la asignación al mapa del poste
que se está rebasando **no se ven afectadas** — se comprobó: el verde a 718 mm
cae en la casilla p3 fila exterior, la correcta. Por encima de 1400 mm manda la
cámara sola, y un pilar a 2 m leería unos 2,4 m: eso sí caería en la casilla
equivocada y estropearía la anticipación, que es de donde sale el tiempo en las
vueltas 2 y 3.

### CERRADO con dos medidas de regla

El equipo midió: **145 mm del lente a la lona**, y el LiDAR a 30 mm del borde
derecho del chasis y 35 del izquierdo — o sea casi centrado, **2,5 mm** del
centro. Eso descarta de plano los 90-120 mm de desplazamiento lateral que pedía
el ajuste conjunto: era ruido de un problema mal condicionado.

Con la altura fijada a 145 y el lateral a −2,5, el ajuste vuelve a estar bien
planteado y solo quedan cabeceo, guiñada y voladizo. Resultado:

| | cámara vs LiDAR (mediana) | confianza de fusión | residuo de muros |
| --- | --- | --- | --- |
| Ajuste inicial (centro óptico nominal) | 198 mm | — | 5,35 px |
| Centro óptico corregido | 173 mm | 0,73 | 4,93 px |
| **Con la altura medida** | **71 mm** | **0,87** | 8,57 px |

**71 mm está por debajo del listón de 80 mm** que la propia herramienta pone
para dar una homografía por buena. Y el suelo pasa a verse desde **74 mm** por
delante, contra 215 antes.

Que el residuo de muros empeore es esperado y se acepta **a propósito**: es un
indicador indirecto — la cámara no mide paredes, eso lo hace el LiDAR — y la
métrica que de verdad importa es dónde pone los pilares. La explicación del
sesgo: la lona justo al pie del muro queda en sombra, cae fuera de
`floor_ranges`, y el borde detectado sale unos píxeles por debajo de la junta
real; el ajuste lo compensaba subiendo la cámara de 145 a 180 mm.

### El montaje, cerrado

Los **tres parámetros de posición están medidos con regla**:

| | |
| --- | --- |
| Altura del lente a la lona | **145 mm** |
| Voladizo (horizontal, del eje del LiDAR al lente, hacia atrás) | **126 mm** |
| Desplazamiento lateral | **−2,5 mm** (el LiDAR a 30 y 35 mm de los bordes del chasis) |

Los **dos de orientación** —cabeceo 9,60° y guiñada 0,50°— se fijaron con las
dos correspondencias cámara-LiDAR directas del mismo instante, y quedan exactas:

* distancia **radial** a un pilar que ven los dos sensores: **1 mm**
* punto de contacto del delimitador magenta contra la recta que el LiDAR mide de
  ese mismo delimitador: **0 mm**

Vale la pena anotar que el voladizo estimado por esas dos evidencias **antes** de
medirlo dio −130, y la regla dijo −126: **4 mm**. Es la validación del método.

Y una regla de método que costó descubrir: **no se ajusta contra el residuo de
muros.** Ese borde tiene un sesgo conocido —la lona al pie del muro queda en
sombra y cae fuera de `floor_ranges`— y por eso el ajuste por paredes pedía
180 mm de altura contra los 145 reales. La cámara no mide paredes; eso lo hace
el LiDAR.

### Lo único que queda, y por qué no es un parámetro

Sobre el pilar quedan ~70 mm de error, **todo lateral**. Ningún parámetro de este
modelo lo corrige, y se comprobó barriendo la guiñada:

| guiñada | error al pilar (rumbo −16°) | error al delimitador (rumbo +22°) |
| --- | --- | --- |
| +1° | 65 mm | 0 mm |
| +5° | **13 mm** | 19 mm |

Un error de guiñada desplazaría los dos por igual. Que corregir uno estropee el
otro significa que el error **depende del rumbo**: es distorsión de lente —el
modelo es pinhole puro con 68° de campo— más el sesgo de centroide de cada
correspondencia.

Se cierra con `calibrar_suelo.py --usar-lidar` y cuatro pilares repartidos: el
DLT ajusta los **ocho** grados de libertad de la homografía y absorbe lo que un
modelo físico de cinco parámetros no sabe expresar. Con 72 mm y la fusión
prefiriendo el LiDAR por debajo de 1,4 m, ya no es urgente.

### El hallazgo grave: la IMU derivaba 20,9 °/s

Con el robot completamente quieto, el rumbo acumulado subía **742° en 36 s**.
Un giro de esquina dura unos dos segundos: la deriva sola metía **42° de error
en cada esquina**. Con eso, ninguna vuelta habría cerrado.

El firmware de la Pico *sí* promedia 100 muestras al arrancar para quitar el
sesgo, pero ese bucle se traga sus excepciones (`except: pass`). Si el I2C aún
no responde —lo normal justo después de alimentar la placa— el offset se queda
en cero y el sesgo crudo entra entero en la integración. La deriva medida es
perfectamente lineal, que es la firma de exactamente eso.

**No se ha tocado el firmware de competición.** La ronda se blinda desde la Pi:
`EnlacePicoNuevo.calibrar_deriva` mide la deriva durante 2,5 s con el robot
quieto, justo antes de arrancar, y la resta linealmente. Verificado en el robot:
de **+20,94 °/s a 0,13 °/s residual**, un factor 160. Si la deriva supera
1 °/s la ronda lo avisa por consola y recomienda reiniciar la Pico.

Merece la pena arreglarlo también en el firmware —contar las muestras que de
verdad se leyeron y reintentar—, pero eso es una decisión tuya: es la placa de
competición.

---

## 4ter. Recalibración del 05-09: máscara del mástil y cámara a 1536×864

Tres cosas, todas medidas en la Pi 5 (`192.168.30.115`, despliegue
`/home/pi/wro_pi5_20260904`) con el robot parado y sin mover un motor.

### La máscara del mástil estaba 22 grados corta, y eso cegaba la parte trasera

`herramientas/diag_mastil.py` (nueva) abre solo el LiDAR y, para cada grado
entero, cuenta **de las veces que ese grado tuvo muestra, en cuántas la
muestra estaba cerca**. El denominador importa: el C1 entrega unos 465 puntos
por vuelta, o sea 1,3 muestras por grado, así que un grado cualquiera se queda
sin muestra en una vuelta de cada cuatro por puro reparto angular. Contra el
total de barridos, hasta una pieza sólida sale al 75 % y parece intermitente.

Con ese criterio, y repetido tres veces:

| | |
| --- | --- |
| arco con eco en el **100 %** de los barridos que lo miraron | **141..194 grados** |
| distancia | 37-72 mm |
| dispersión (p90 − p10) por grado | 2-4 mm |
| el grado 140 | 17 % |
| el grado 195 | 2 % |

Los bordes son limpios, no hay que interpretar nada. Y a 37-72 mm no puede
haber pista: el LiDAR va **al ras del parachoques** y el robot se extiende
222 mm hacia atrás, así que todo lo que devuelve eco ahí es el propio robot.

La máscara heredada de la Pi 3B era `163-195` y **dejaba 141..162 sin tapar**,
justo dentro de `rear_sector_deg` (150-210). Consecuencia, medida en dos
simulacros consecutivos con el robot en el mismo sitio:

| `trasera_min` | máscara 163-195 | máscara 140-195 |
| --- | --- | --- |
| mediana | **40,2 mm** | **1712,9 mm** |
| p10 | 39,0 | 1709,2 |
| máx | 43,2 | 1718,2 |

O sea: el robot creía tener 40 mm detrás **en todos los ciclos de su vida**, y
lo que había era el muro a 1,71 m. `holgura_trasera_min_mm` son 70, así que
cualquier comprobación de seguridad marcha atrás —y el parqueo entero— estaba
leyendo un obstáculo pegado al culo que era su propio mástil. Los demás
sectores no se movieron ni un milímetro (izquierda 562,8; derecha 394,2 →
394,5; corredor 1640,1 → 1640,2), que es la prueba de que el cambio tocó solo
lo que tenía que tocar.

**La máscara es del montaje, no del robot.** En la Pi 3B el mástil estaba a
84-113 mm y ocupaba 163..193; aquí está a 37-72 y ocupa 141..194. Se mide cada
vez que se toca la cámara, y por eso la herramienta existe.

`blind_sectors_deg` se escribe con el **primer y el último grado a tapar**:
`_en_sector` es inclusivo en los dos extremos.

### La cámara a 1536×864, con toda la calibración reescalada

Subir la resolución no es cambiar dos números. Media configuración está en
píxeles del cuadro, y `herramientas/reescalar_camara.py` (nueva) los mueve
todos a la vez:

* `principal_x_px` 704,148 → **844,9776**, `principal_y_px` 360 → **432**. El
  centro óptico está en el mismo punto de la lente; le toca otro píxel.
* **`ground_homography.matrix`**, componiéndola con la escala:
  `H_nueva = H_vieja @ diag(1/s, 1/s, 1)`. Esto es lo que de verdad podía
  arruinar el mapa en silencio: sin tocarla, cada píxel se proyectaría a 1,2
  veces su distancia real. Medido, no razonado: el píxel del centro-abajo pasa
  de proyectarse a (10,6, 121,3) mm a **caer por encima del horizonte** y
  devolver `None`. Hay un test de regresión que mira el mismo punto del mundo a
  dos resoluciones y exige el mismo milímetro.
* los umbrales de visión en píxeles: las **áreas** con `s²` (`min_area_px`
  90 → 130, `magenta_min_area_px` 250 → 360, `line_min_area_px` 400 → 576) y
  las **longitudes** con `s` (`min_height_px` 12 → 14, `morph_open_px` y
  `morph_close_px` 6 → 7, `line_polygon_dilate_px` 11 → 13). Si no, cambia la
  sensibilidad de la detección y los tiempos dejan de ser comparables.

Lo que **no** cambia: `hfov_deg` (es óptica) y `raw_sensor_size` (es el modo
del sensor). Esto último es la trampa: el IMX708 tiene un modo nativo
1536×864 que **recorta a 3072×1728 y tira un tercio del ángulo**. Pidiendo
`main` 1536×864 con `raw` fijado en 2304×1296, el `ScalerCrop` sigue siendo
`(0, 0, 4608, 2592)`, exactamente igual que a 1280×720: **el campo no cambia,
solo la rejilla de píxeles**. Verificado en la Pi antes de tocar nada.

### Lo que cuesta, medido con la tubería real

`herramientas/bench_vision.py` (nueva) procesa **dentro del callback de la
cámara**, igual que la ronda, porque lo que interesa no es el coste por cuadro
sino la cadencia que ve la fusión: si un cuadro pasa del periodo, no se acumula
retraso, se pierden cuadros.

| | 1280×720 | 1536×864 |
| --- | --- | --- |
| mediana | 19,3 ms | 33,1 ms |
| p90 | 19,4 | 33,6 |
| **entrega real** | **30,0 Hz** | **28,8 Hz** |

Pasa del presupuesto de 33,3 ms por 0,3, y el precio son 1,2 Hz: 4 %. Para un
control que decide al ritmo del LiDAR (9 Hz) no es nada.

`process_scale` **no es la palanca**: a 1536×864, bajarlo de 0,5 a 0,35 solo
ahorra 2,3 ms. El coste vive en la pasada a resolución COMPLETA de las líneas
de piso, que es justo la que no se puede reducir (a media resolución la franja
naranja pasaba de 2074 px a 30).

### El LiDAR no tenía latencia que quitar, y la cámara nueva no se la añadió

Medido en el CSV de dos simulacros (`lidar_edad_ms`, la edad real del barrido
cuando el control lo consume):

| | 1280×720 | 1536×864 |
| --- | --- | --- |
| mediana | 0,30 ms | **0,30 ms** |
| media | 0,43 | 0,48 |
| p90 | 0,40 | 0,60 |

Los 0,1 ms que están apuntados en la bitácora de la migración se midieron con
`diag_latencia` —sin Pico y sin lazo de control—; con la ronda entera montada
el número es 0,3 ms de mediana. **No hay latencia que recortar**: son un tercio
de milisegundo contra un periodo de LiDAR de 108 ms, y lo que queda es la
latencia de despertar un hilo. Lo que sí se puede leer aquí es que subir la
cámara **no** ha estropeado el barrido, que era el riesgo real.

`vision_edad_ms` sí sube, de 41,7 a 50,2 ms de mediana (p90 64,6, máx 83,3),
todavía dentro de los 120 ms de `max_camera_lidar_age_s`. A 200 mm/s eso son
10 mm de desplazamiento entre el cuadro y el barrido, contra 8,3 antes:
1,7 mm más, despreciable al lado de los 36 mm de sesgo cámara-LiDAR.

### Un cabo suelto que deja abierto la máscara nueva

Con 140..195 tapado, de `rear_sector_deg` (150-210) solo sobreviven 15 grados,
todos por el lado izquierdo. Eso es la física —el robot no puede verse la
espalda—, pero deja el sector trasero torcido: la ventana honesta son dos
lóbulos, `110..140` y `196..230`, y el bloque solo admite un sector. **No se ha
tocado** porque solo lo usa `estacionamiento.py`, que sigue con
`parking_ready: false` y nunca se ha ejercitado con motores; hay que decidirlo
antes de estrenar el parqueo.

---

## 5. Lo primero que hay que hacer, en orden

### 5.1 Elegir altura y cabeceo **mientras montas la cámara**

```bash
python3 herramientas/calibrar_suelo.py --cobertura --altura-mm 400 --cabeceo-deg 28
```

No necesita robot ni cámara. Dice desde cuántos mm por delante empieza a verse
el suelo y hasta dónde llega. Prueba varias combinaciones antes de apretar los
tornillos.

**Objetivo:** que el alcance en el eje pase de **1,6 m**. Por debajo de eso el
mapa solo ve el pilar inminente y pierde la ventaja de anticipar los otros dos,
que es de donde sale la mayor parte del tiempo que se gana.

### 5.2 Medir el bloque `chassis` con regla

Van **tres valores de montaje comprobados y tres desviados**. Antes de fiarte
de ninguna geometría, repasa con regla: `wheelbase_mm`, `width_mm`,
`length_mm`, `rear_overhang_mm` y `lidar_to_*_edge_mm`. Luego pon
`calibration.chassis_measured: true`.

### 5.3 Calibrar la homografía

**Ya está hecha** (ver 4bis), pero hay que repetirla cada vez que se toque el
mástil. Dos métodos, y el primero no necesita colocar nada:

```bash
# con las paredes: basta con poner el robot en la pista
python3 herramientas/calibrar_desde_muros.py --dibujar /tmp/verif.jpg --escribir
# y, si ya mediste la altura con regla:
python3 herramientas/calibrar_desde_muros.py --altura-mm 180 --escribir

# con pilares, si quieres una segunda opinión independiente
python3 herramientas/calibrar_suelo.py --usar-lidar --poses 4 --escribir
```

Mira siempre el `--dibujar`: si las rectas rojas caen sobre la base de los
muros, la calibración vale. Un residuo se puede discutir, una línea que cae dos
centímetros fuera de la pared se ve.

### 5.4 HSV

Rojo y verde vienen de la calibración del equipo (94 % de detección) y **siguen
sin comprobarse en la pista nueva: no había ningún pilar en el campo.**

Azul y naranja **sí están medidos** sobre la lona real (ver 4bis) y se detectan.
Magenta sigue sin calibrar: hace falta el cajón de parqueo a la vista.

### 5.5 Validar y desarmar la traca

```bash
python3 -m ronda_nueva.ronda_nueva --validar-config
```

`runtime.motion_enabled` viene en `false` y hay cinco calibraciones que se
exigen antes de armar la tracción. Es deliberado.

---

## 6. Desplegar y correr

```bash
bash src/pi5/deploy.sh /home/pi/wro_pi5_$(date +%Y%m%d_%H%M)
cd /home/pi/wro_pi5_...
python3 -m ronda_nueva.ronda_nueva --validar-config
```

Y una corrida, siempre parando con SIGINT (nunca SIGTERM: dejaría el motor
girando a la última consigna):

```bash
timeout -s INT 180 python3 -u -m ronda_nueva.ronda_nueva --arranque-inmediato --sin-parqueo
```

Banderas:

- `--arranque-inmediato` no espera el botón GP21 (para lanzar por SSH).
- `--sin-parqueo` termina al completar las esquinas.
- `--solo-parqueo` entra en la bahía en cuanto hay sentido. **Exige
  `turn_direction` LEFT o RIGHT**: arrancando dentro de la bahía no se cruza
  ninguna línea de sentido y con AUTO no entra nunca.

Los CSV salen en `logs/`. Columnas nuevas que merece la pena mirar:
`segmento`, `avance`, `offset`, `objetivo_offset`, `error_lateral`,
`plan_cedido`, `mapa` (el resumen de las doce casillas, `R0:R.V R1:...`) y
`pilar1_fuente` (`FUSION` / `CAMARA` / `LIDAR` / `ALTURA`).

---

## 7. El parqueo

El detector del hueco es **el mismo que ya funciona**: partir los clústeres en
rectas antes del PCA subió de 0 emparejamientos en 22 barridos a 9 de 10, con
la separación estable en 389–391 mm. Se conserva tal cual.

Lo que cambia es la **maniobra**. La simulación con los radios reales decía que
la de dos arcos se pasa 9 mm sobre un delimitador, y que el techo geométrico de
dos arcos es r ≈ 180 mm —inalcanzable con batalla de 136 mm—. Así que aquí no
se intenta clavar la maniobra de una vez: se entra **en varios tiempos**, como
un coche aparca en línea de verdad, y cada tramo se corta **por medida** (rumbo
alcanzado o holgura mínima), nunca por tiempo. Un vaivén que se queda corto se
corrige en el siguiente en vez de terminar la ronda contra el delimitador.

Además la aproximación usa los **muros magenta que ve la cámara**, que aparecen
mucho antes que los separadores del LiDAR.

---

## 8. Lo que falta, por orden

1. **Poner pilares en el campo y comprobar rojo y verde.** Es lo único que
   queda del bloque de percepción sin verificar, y no se puede hacer sin ellos.
   Con dos postes delante, `herramientas/diag_arranque.py` los lista con su
   color, su posición en milímetros y de qué fuente salió cada una.
2. **Medir con regla la altura de la cámara y el voladizo trasero.** La
   calibración por paredes deja la altura a ±10 mm porque el desplazamiento
   longitudinal no es observable; con la altura medida, `--altura-mm N` cierra
   las dos cosas.
3. **Decidir qué hacer con la deriva del giroscopio.** Ya está compensada por
   software, pero el `except: pass` del firmware puede volver a morder.
4. **Medir `mm_s_per_pwm` en recta y con las ruedas rectas.** El valor 4,0 y la
   tabla del README de la 3B (6,7) discrepan 1,6×. Aquí solo se usa para
   propagar la pose entre barridos, así que el error es menos grave que antes,
   pero conviene cerrarlo.
5. **Primera corrida con motores**, con `--sin-parqueo` y el dedo en el botón.
   Antes hay que poner `runtime.motion_enabled: true` y aprobar las
   calibraciones que falten.
6. **Serie de tres corridas por configuración.** La bitácora del equipo tiene el
   caso de la misma configuración dando 2 y 0 esquinas según 41 mm de colocación
   manual: con una sola corrida no se sabe nada.
7. **Probar `--solo-parqueo`** con el robot dentro de las barreras, ahora que la
   maniobra converge en vez de ser de un solo tiro. Magenta sigue sin calibrar.
8. **Decidir la forma del sector trasero.** Con la máscara medida (140-195), de
   `rear_sector_deg` solo sobreviven 15 grados y todos por la izquierda. La
   ventana honesta son dos lóbulos, `110..140` y `196..230`, y el bloque solo
   admite uno. Solo lo usa `estacionamiento.py`: hay que cerrarlo **antes** de
   estrenar el parqueo, no después.

---

## 9. Pruebas

```bash
cd src/pi5
python3 -m unittest discover -s ronda_nueva/tests -t . -p "test_*.py"
```

Tarda unos dos minutos: la mayor parte es la vuelta completa simulada, que
genera un barrido LiDAR sintético con oclusión correcta en cada ciclo.
