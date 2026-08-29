# ronda_camara — montaje de camara en mastil trasero

Carpeta aparte de `ronda_cerrada/` a proposito: aqui vive lo que cambia
al mover la camara al mastil trasero, medido en pista el **2026-08-28**
con el robot quieto sobre la recta inferior izquierda. Nada de esto toca
`ronda_cerrada/`, que sigue como estaba.

Todas las herramientas corren **sin instanciar `EnlacePico`**: el robot
no se mueve en ninguna.

## Que se midió

### 1. El mastil ciega 23 grados del LiDAR — y rompe el RETROCESO

`mapa_oclusion.py`, 40 barridos:

| grados | eco | dispersion |
|---|---|---|
| 165–187 | 90–107 mm constantes | 3–8 mm |

Son 23 grados en los que el C1 se ve a si mismo. El resto del circulo
mide entorno real: los sectores frontal, derecha, perp_der y las dos
diagonales salieron con **0%** de grados tapados.

El daño no es perder resolucion trasera. Los sectores de
`lidar_geometria.py` toman el **minimo** del rango, asi que un eco fijo
de ~92 mm gana siempre:

| campo de `Medicion` | rango | tapado | valor real medido |
|---|---|---|---|
| `trasera` | [170,190] | 18/21 grados | **92 mm fijos** |
| `trasera_derecha` | [90,170] | 6/81 grados | **102 mm fijos** |
| `trasera_izquierda` | [190,270] | 0 estructurales | 938 mm (correcto) |

Consecuencias en `navegacion.py`, verificadas con `test_sectores_trasera.py`:

- **El estado `RETROCESO` esta muerto.** Sale en cuanto
  `med.trasera < EMERGENCIA_TRASERA (250 mm)`, y `med.trasera` vale 92 mm
  siempre, este donde este el robot. Aborta en su primer ciclo con la
  razon "obstaculo trasero". El robot pierde entera la maniobra de
  desatasco: cada emergencia frontal lo devuelve a `CRUCERO` sin haberse
  despegado, vuelve a disparar la emergencia, y encadena `racha_retroceso`
  hasta caer en `GIRO_FORZADO`.
- **El giro en reversa apunta a un lado fijo.** El control P usa
  `trasera_derecha - trasera_izquierda`; con `trasera_derecha` clavado en
  102 mm el error medido fue **-836 mm** cuando el real es **-463 mm**.
  El signo lo dicta el chasis, no el espacio libre.
- Los grados **sin eco** no son mejores que los tapados:
  `construir_perfil_360` los rellena con `8000.0`, que significa "via
  libre". Un sector ciego que se reporta despejado es peor que uno que se
  reporta ocupado.

`lidar_mascara.py` corrige las tres cosas: excluye 163–189 (2 grados de
margen, los bordes tienen eco intermitente) y **recupera** la medida
trasera desde los hombros que siguen viendo, [145,162] y [198,215],
proyectando cada haz sobre el eje trasero (`d·cos(offset)`). Recorta el
mismo margen a ambos lados aunque por la izquierda sobre sitio: dos
sectores de ancho distinto meterian un sesgo constante en el control P.
Con la mascara, `trasera` pasa de 92 mm a **759 mm** — el entorno real.

`aplicar(med)` reescribe los tres campos justo despues de
`ProcesadorLidar.procesar()`, para que `navegacion.py` no se entere de
que el mastil existe. `diagnosticar(perfil)` avisa si el mastil se movio.

### 2. La camara quedo derecha: sobra la rotacion de 180

`ronda_cerrada/camara_driver.py` rota 180 grados porque el modulo estaba
atornillado invertido. En el mastil quedo derecho. Medido con
`test_camara_mastil.py`:

| | filas de arriba | filas de abajo |
|---|---|---|
| frame crudo | V=50 (muro) | V=142–150 (suelo blanco) |
| frame rotado 180 | V=142 (suelo) | V=50 (muro) |

El crudo es el correcto. Mantener la rotacion espeja el `cx` (la evasion
sale por el lado contrario al que manda el reglamento) e invierte el
filtro `cy < 180`. `camara_driver.py` de esta carpeta es el mismo driver
sin la rotacion.

### 3. El FOV de `navegacion.py` esta al doble del real

`navegacion.py` convierte pixel a rumbo con `HFOV_CAMARA = 102.0`, dato
de catalogo de la Module 3 **Wide**, anotado en su propio comentario como
nunca calibrado. Esta mal por dos motivos que se suman: la camara
responde al FOV **estandar**, y ademas el numero de catalogo no
sobrevive a la configuracion — el sensor entrega 1536x864 (16:9) y
`camara_driver` pide 4:3, asi que libcamera recorta un 25% del ancho
antes de escalar.

`medir_fov.py` lo mide sin pilares, emparejando una esquina de la pista
que el LiDAR situa en el rumbo **-21.5°** con su borde negro/blanco, que
Otsu situa en la **columna 144 de 1280**:

```
f = (144 - 640) / tan(-21.5) = 1261 px      HFOV efectivo = 53.8 grados
```

| hipotesis | catalogo | efectivo en 4:3 |
|---|---|---|
| Module 3 estandar | 66° | **51.9°** ← concuerda |
| Module 3 Wide (la que asume el codigo) | 102° | 85.6° |

El modelo viejo **sobreestima el rumbo ~2.4x**: un poste a 15° reales se
calcula a 36°, y `TOLERANCIA_APAREO_GRADOS = 20` lo descarta. Es decir,
el apareo por rumbo falla justo en los postes descentrados, que son los
que ese apareo venia a arreglar. `optica.py` lleva la focal medida
(315.4 px a 320 de ancho).

Esto es **anterior** al cambio de montaje: la configuracion de camara es
la misma de siempre. El mastil no lo causo, solo obligo a mirarlo.

### 4. Validado con un pilar real: el modelo nuevo acierta, el viejo se va 12 grados

`test_pilar.py` con un pilar verde quieto a 429 mm y rumbo +9.2 grados
segun el LiDAR, 43 muestras seguidas:

| | rumbo calculado | error contra el LiDAR |
|---|---|---|
| `optica.py` (52 grados, medido) | **+9.0** | **-0.8 a +0.4** |
| `navegacion.py` (102 grados, catalogo) | +21.1 | **+11.3 a +12.5** |

Es la confirmacion directa de la medida de FOV con un objeto
independiente: el modelo nuevo clava el rumbo por debajo de un grado y el
viejo lo infla 2.3x, exactamente el factor que predecia la medicion de
`medir_fov.py`.

El modelo viejo aun "pasa" la tolerancia de 20 grados aqui porque el
pilar esta casi al frente. A 20 grados reales calcularia ~46 y la
tolerancia lo rechazaria: el apareo falla en los postes descentrados, que
son justo los que necesitan que la evasion salga por el lado correcto.

La deteccion de color aguanta de sobra a esa distancia: area ~3060 (el
minimo es 350) y `cy = 77`, muy por encima del umbral 180.

### 5. Barrido del pilar: el filtro `cy < 180` NO hay que tocarlo, pero aparece el paralaje

`test_pilar.py` con el pilar verde empujado a mano de 428 mm hasta casi
tocar el parachoques. 230 muestras de camara, 136 con cluster de LiDAR.

**El filtro `cy < 180` no es el cuello de botella.** Medido:

| distancia | 428 | 343 | 293 | 215 | 106 | 81 |
|---|---|---|---|---|---|---|
| `cy` | 76 | 94 | 107 | 138 | 164 | 172 |

De 230 muestras, **4** llegaron a `cy >= 180`, y ninguna de forma
sostenida (maximo 183). El motivo es que por debajo de ~150 mm el blob
ya no cabe en el cuadro: se recorta contra el borde inferior, el
`boundingRect` se trunca y el centroide **satura** en 172-176 en vez de
seguir bajando. O sea que el filtro casi nunca llega a rechazar el pilar,
y cuando lo hace es a distancia de parachoques, muy por dentro de los 900
mm donde arranca la evasion. **No requiere retoque.** El area tampoco es
problema: entre 2962 y 12214, contra un minimo de 350.

**Lo que si aparece es el paralaje.** La camara va ~100 mm detras del
LiDAR, asi que el mismo objeto se ve con distinto rumbo desde cada
sensor, y la diferencia crece al acercarse: 0 grados a 428 mm, 5 a 215
mm. Con la camara vieja, pegada al LiDAR, esto era despreciable.

Del mismo barrido sale ademas que el centro optico no esta en el centro
geometrico del frame: hay un sesgo constante de +2.78 grados, o sea
`cx = 175.1` y no 160. Los dos efectos estan **acoplados** — el sesgo del
centro estaba tapando parte del paralaje — asi que hay que corregir los
dos o ninguno. Error contra el rumbo real, 82 muestras entre 215 y 428 mm:

| modelo | rango de error | desv |
|---|---|---|
| c0=160, sin paralaje | [-6.0, +0.8] | 1.91 |
| solo centro optico corregido | [-8.6, -1.9] | 1.89 (**peor**) |
| centro optico + paralaje | **[-0.9, +0.4]** | **0.30** |

Por eso `optica.py` expone `rumbo_camara_de_cluster(x, y)`: el apareo no
debe comparar dos rumbos medidos desde origenes distintos. Con el modelo
completo el residuo baja a +-0.9 grados, y por eso
`TOLERANCIA_APAREO_GRADOS` puede cerrarse de 20 a 10.

**Aviso sobre la herramienta:** `test_pilar.py` empareja con el cluster
mas cercano, asi que **la mano que empuja se convierte en el cluster
cuando se acerca mas que el pilar** (se ve como filas con rumbo de +74 a
+79 grados). Al leer el barrido hay que cortar esos tramos; el analisis
de arriba lo hace.

### 6. Modo de sensor 2304x1296: el frame pasa a 640x360 y el FOV a 74.5 grados

El driver viejo pedia 320x240 (4:3) con `ScalerCrop` al sensor entero.
No servia de nada: al pedir 4:3 de un sensor 16:9, libcamera recorta los
lados igual. Leido de la metadata del propio frame, el recorte real era
**(768, 432, 3072, 1728)** -- 3072 de los 4608 px de ancho, un tercio del
campo tirado, y de paso pixeles anamorficos (estirados 1.33x en vertical).

Fijando el modo raw a 2304x1296 y pidiendo salida 16:9, el recorte pasa a
ser el sensor completo:

| | recorte de sensor | HFOV | VFOV | pixeles |
|---|---|---|---|---|
| antes, 320x240 | 3072x1728 | 53.8° | 31.9° | anamorficos 1.33x |
| **ahora, 640x360** | **4608x2592** | **74.5°** | **46.3°** | cuadrados |

La salida se queda pequeña a proposito. Coste del pipeline de `vision.py`
en la Pi 3B, medido:

| frame | ms/frame | fps |
|---|---|---|
| 320x240 | 13.5 | 74 |
| 384x216 | 5.5 | 182 |
| **640x360** | **13.9** | **72** |
| 2304x1296 | 165.9 | **6** |

Sacar el frame entero a 2304x1296 dejaria la vision en 6 fps con 166 ms
de latencia: inservible para decidir una evasion. 640x360 cuesta lo mismo
que los 320x240 de antes y da el doble de resolucion lineal con el campo
completo.

**Constantes reescaladas.** Todo lo calibrado estaba en pixeles del frame
viejo. Se reexpresa por ANGULO, que es lo unico que no depende del modo:

| | antes | ahora | como |
|---|---|---|---|
| `FOCAL_PX` | 315.4 | 420.6 | del recorte, con `FOCAL_SENSOR_PX = 3028` |
| `CX_CENTRO_OPTICO` | 175.1 | 340.4 | el sesgo medido (+2.78°) en px nuevos |
| `AREA_MIN_DETECCION` | 350 | 467 | x1.334 horizontal, x1.000 vertical |
| `UMBRAL_CY` | 180 | 240 | los mismos 8.1° por debajo del centro |

**Cuidado:** el 74.5° y las constantes derivadas de el NO se han medido
contra la pista, se deducen de la geometria del recorte partiendo del
53.8° que si se midio. El metodo esta validado (reproduce 53.8 exacto
para la config vieja), pero conviene reconfirmarlo con `test_pilar.py` y
un pilar delante. Y en particular el `UMBRAL_CY`: con el VFOV nuevo el
pilar se recorta MENOS contra el borde inferior, asi que el filtro que
antes casi nunca mordia ahora puede morder de verdad.

### 7. Verificacion del modo nuevo: la del LiDAR en cuadro salio, la del FOV no

Con la pista montada y **sin pilares** (se habian retirado), se comprobo
lo que no los necesita.

**El LiDAR entra en cuadro.** Al pasar el VFOV de 31.9 a 46.3 grados, la
propia cupula del C1 aparece en la parte baja del frame. Medido sobre las
columnas centrales (200-440 de 640):

| filas | oscuro |
|---|---|
| 80-119 | 60-77% (el muro del fondo, es escena) |
| 280-299 | 4% |
| 300-319 | 51% |
| 320-359 | 76-85% (**la cupula del LiDAR**) |

El centro esta despejado por encima de la fila 301. No genera falsos
positivos (es negra, no roja ni verde) y el filtro `cy < 240` ya cae por
encima, asi que no hay que tocar nada; pero se come el 16% inferior del
cuadro, justo donde apareceria un pilar muy cercano. Es la misma
truncatura de blob que antes hacia el borde de la imagen, ahora causada
por el propio robot.

**La medida del FOV en el modo nuevo FALLO.** `medir_fov.py` localiza el
borde negro/blanco recorriendo una banda de filas, y eso solo vale si en
esa banda hay una esquina. En esta escena el muro del fondo cruza la
imagen entera, asi que lo que encontro fue la columna donde el GROSOR del
muro cruza el umbral del 60%, no una esquina. Resultado: 78.1 grados por
la izquierda y 60.9 por la derecha -- 17 grados de discrepancia, que es
la firma de un emparejamiento malo, no de un centro optico desplazado.

Asi que **el 74.5 grados sigue siendo derivado, no medido**. Y hay un
problema de fondo que esto destapo: **la focal y el centro optico nunca
se separaron**. La medida original usa UNA esquina, y con una sola no se
pueden despejar las dos incognitas:

| supuesto | focal (a 1280 px) | HFOV |
|---|---|---|
| c0 en el centro geometrico | 1261 | 53.8° |
| c0 con el sesgo de +2.78° del pilar | 1436 | 48.1° |

El barrido del pilar tampoco los separa: sus rumbos cubren solo 7.7
grados y ahi f y c0 se compensan (residuo 0.30 con una focal, 0.47 con la
otra: indistinguibles).

**Cuanto duele:** la incertidumbre es de +-5 grados de HFOV, ~8% en la
focal. A 20 grados de rumbo son 1.6 grados de error, muy dentro de la
tolerancia de apareo de 10. No bloquea una corrida.

**Cerrado en la seccion 8**, con `calib_fov.py` adaptado y dos pilares.

### 8. CERRADO: calibracion con dos pilares

`calib_fov.py` adaptado (sin rotacion, config de carrera, paralaje
corregido, minimos cuadrados) con un pilar rojo a **-27.5 grados / 462
mm** y uno verde a **+19.2 / 1006 mm**:

| | antes (derivado) | **medido** |
|---|---|---|
| `FOCAL_PX` | 420.6 | **472.9** |
| `CX_CENTRO_OPTICO` | 340.4 | **352.1** |
| `HFOV_EFECTIVO` | 74.5° | **68.2°** |
| sesgo del centro | +2.78° | **+3.88°** |

44 puntos, base angular **47.2 grados**, residuo **0.21 grados** de
desviacion (maximo 0.61, o 2.2 px). `FOCAL_SENSOR_PX` pasa de 3028 a
**3405**, un 12% mas: el valor viejo salia de UNA esquina suponiendo el
centro optico centrado, y el sesgo de 2.78 se habia medido dando esa
focal por buena, asi que arrastraba su error.

**Dos trampas que costaron una corrida cada una, anotadas para no
repetirlas:**

1. *Emparejar por orden no vale.* La primera corrida asigno al pilar
   verde (+23 grados en camara) un cluster en **-22.5** y escupio un
   HFOV de 9.9 grados. `clusters_obstaculo` admite hasta 15 grados de
   arco y con eso se cuelan esquinas de muro: habia 3-5 clusters con
   solo dos pilares en pista. La correccion es doble: filtrar por
   `es_objeto_estrecho` (ancho fisico <=260mm) y emparejar cada color
   con el cluster mas proximo en rumbo usando el modelo actual como
   semilla, no por posicion en la lista.
2. *Los pilares tienen que caber en el cuadro.* Con el borde en +-34
   grados, un pilar a 38 simplemente no existe para la camara. La
   ventana util por pilar es de 20 a 33 grados: por debajo de 20 la base
   se queda corta y f y c0 vuelven a compensarse. `ver_pilares.py` da
   el rumbo y el veredicto de cada color en vivo para colocarlos sin
   adivinar.

**Lo que este residuo NO dice.** Con dos pilares hay dos rumbos y dos
incognitas, asi que el ajuste queda exactamente determinado en la media:
el 0.21 mide la repetibilidad de cada poste, no valida la forma del
modelo. Para eso haria falta un tercer rumbo. Lo que si queda bien
condicionado -- que es lo que fallaba en todas las medidas anteriores --
es la separacion entre focal y centro optico.

## Pendiente

`test_pilar.py` es la unica herramienta que pide accion fisica — acercar
un pilar de ~1200 mm a ~200 mm. Mide lo que no se puede deducir de una
pista vacia:

Lo medido cubre percepcion estatica. Queda por hacer, y ya no depende de
mas mediciones de banco:

1. ~~Cerrar el modelo optico con dos pilares~~ -- hecho, seccion 8.
2. **Integrar en `ronda_cerrada`**: `lidar_mascara.aplicar()` tras
   `procesar()`, quitar el `ROTATE_180` del driver, y sustituir en
   `navegacion.py` las constantes de FOV por `optica.py` con el apareo
   por `rumbo_camara_de_cluster()`.
3. **Volver a medir en movimiento.** Todo este banco se tomo con el robot
   quieto, asi que no incluye el desfase temporal entre el frame de
   camara y el barrido de LiDAR. La tolerancia de apareo de 10 grados es
   el numero a vigilar.
4. **Comprobar que el `RETROCESO` ya retrocede** en pista, que es el
   bloqueante que abrio esta sesion.

## Herramientas

| archivo | que hace | necesita |
|---|---|---|
| `mapa_oclusion.py` | mapa 360 de grados tapados/ciegos y rangos a excluir | nada |
| `test_sectores_trasera.py` | sectores traseros con y sin mascara, y veredicto sobre `RETROCESO` | nada |
| `mapa_pista.py` | vista cenital ASCII del barrido + autochequeo de la mascara | nada |
| `test_camara_mastil.py` | orientacion, encuadre y blobs; guarda capturas | nada |
| `medir_fov.py` | FOV horizontal efectivo contra una esquina del LiDAR | nada |
| `superponer_lidar.py` | dibuja sobre la foto donde caeria cada hipotesis de FOV | nada |
| `test_pilar.py` | `cy`, area y apareo por rumbo en vivo | **un pilar** |

| modulo | |
|---|---|
| `lidar_mascara.py` | arco ciego y sectores traseros recalculados |
| `camara_driver.py` | igual que el de `ronda_cerrada` pero sin rotar 180 |
| `optica.py` | modelo pixel→rumbo con la focal medida |

## Despliegue

Igual que `ronda_cerrada`: los modulos se importan por nombre suelto, asi
que hay que copiarlos planos en un mismo directorio de la Pi. En esta
sesion se uso `~/ronda_camara_test/`:

```
scp comun/lidar_driver.py comun/lidar_geometria.py comun/enlace_pico.py \
    comun/registro_metricas.py ronda_cerrada/vision.py \
    ronda_camara/*.py  pi@<ip>:~/ronda_camara_test/
```

Terminar cualquier corrida con `timeout -s INT`, nunca con SIGTERM.
