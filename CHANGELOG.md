# Registro de Cambios

Notas de versión del proyecto, basadas en los hitos reales del historial de
commits (ver `git log`). Formato inspirado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).
Cada versión referencia los commits representativos de ese hito para
poder auditar el cambio exacto con `git show <hash>`.

## [v0.11.0] — 2026-09-12 — Calibración en pista, selección de ronda y saneamiento de la documentación

Versión orientada a **operar el vehículo en competencia**: recalibrar el color
sin editar código, cambiar de ronda sin editar el servicio, y dejar el
repositorio sin defectos silenciosos.

### Agregado

- `ronda_curvas/calibrador_web.py`: calibrador de color por navegador. Sirve en
  el puerto 8080 la cámara y la máscara lado a lado, con el **área del blob en
  números**, que es el valor que decide si el robot ve el pilar o no. Guarda en
  `calibracion.json`, que `vision.py` lee al arrancar; si el archivo falta o está
  corrupto se usan los valores del código, de modo que el archivo solo puede
  mejorar la calibración y nunca dejar el vehículo sin una. Se calibran los
  cuatro colores, **incluidas las líneas naranja y azul del piso**, de las que
  sale el conteo de vueltas.
- `ronda.sh`, `correr_ronda.sh` y `correr_abierta.sh`: la ronda que corre el
  servicio se elige con una palabra en un archivo, sin tocar la unidad de
  systemd. La Prueba Abierta no añade lógica de control: es la misma ronda sin
  `parqueo.py` y con `WRO_SIN_PILARES=1`, que deja la cámara leyendo las líneas
  del piso —necesarias para contar vueltas— pero saca su color de la máquina de
  estados. En la abierta no hay pilares, así que toda detección de color es un
  falso positivo, y un falso positivo dispara una evasión contra algo que no
  existe.
- `ronda_curvas/esperar_boton.py`: el botón abre ahora la **secuencia entera** y
  no solo la carrera. Antes solo `ronda_camara.py` lo esperaba, de modo que la
  salida del estacionamiento arrancaba sola al lanzar el script. Mientras espera,
  el LED de la Pico parpadea.
- `herramientas/verificar_docs.py`: comprueba toda la documentación en busca de
  caracteres de control invisibles, enlaces e imágenes rotos, anclas internas
  muertas y finales de línea mezclados. Devuelve 1 si encuentra algo.
- `schemes/Alimentacion_y_Senales_v2.svg`: diagrama vigente de alimentación y
  señales, con las tres etapas de regulación, el reparto Pi 5 / Pico 2, el mapa
  de pines completo y el consumo real medido.
- README secciones **3.5** (línea de tiempo del proyecto y punto de inflexión),
  **4.5** (geometría de sensores, zonas ciegas y autoecos) y **7.6** (materiales
  y manufactura del chasis).
- `.gitattributes`, que fija los finales de línea en LF.

### Corregido

- **La fórmula de autonomía no renderizaba.** No era un comando LaTeX mal
  escrito sino un carácter de avance de página (0x0C) incrustado en el archivo:
  la barra invertida de la fracción se interpretó como escape al generar el
  texto y quedó como byte de control invisible. Se barrió el resto del
  repositorio buscando los demás escapes que producen lo mismo.
- **El diagrama de cableado no describía el vehículo.** Mostraba dos botones de
  selección de ronda cuando hoy hay uno solo en `GPIO 21`. Se sustituye por uno
  vigente y el original se conserva marcado como histórico.
- **El documento se contradecía sobre la versión del prototipo.** La sección 3.1
  decía que el actual era la V3 mientras la sección 0 y las seis vistas
  reglamentarias son de la V4.
- `3d-Models/` pasa a llamarse `models/`, que es el nombre de la plantilla
  oficial de la WRO. Era la única de las seis carpetas de primer nivel que no lo
  usaba.

### Medido

- **El LiDAR se ve su propia rueda** con el volante al tope, y eso bloqueaba la
  maniobra de salida del estacionamiento. Lo que lo distingue de un muro es la
  dependencia con el rumbo: un muro plano se lee $D/\sin(\text{rumbo})$ y sube al
  alejarse de los 90°, mientras que una pieza a radio fijo se lee igual en todos
  los rumbos. Medido en dos corridas: ecos planos de 61-66 mm y de 49-56 mm, con
  el radio cambiando entre ellas porque cambiaba el ángulo del volante.
- **El conteo de vueltas por líneas contaba de más.** Verificado contra la
  guiñada acumulada: 871° de giro, que son 2,4 vueltas, se contaban como tres.
  Se añadió una guiñada mínima entre cruces, porque dos líneas del mismo color
  están separadas por una esquina.
- **Flancos del chasis: 68 mm a cada costado**, medidos con regla. El modelo de
  la silueta suponía 44 y 94, y esa asimetría falsa daba holgura negativa con el
  robot parado.

### Pendiente

- Unificar las medidas del chasis entre `geometria_robot.py` (222 × 125 mm) y
  `parqueo.py` (242 × 138 mm medidos). No se toca antes de competir: cambiar el
  largo altera la geometría del parqueo, que está validada en pista con el valor
  actual.
- Validar en pista los dos arreglos de esta versión (autoeco de rueda y guiñada
  mínima entre líneas).

---

## [v0.10.0] — 2026-09-11 — Ronda completa: salir del estacionamiento, tres vueltas contadas por líneas y volver al cuadrante

Primera versión en que el vehículo ejecuta la secuencia entera de la Ronda
de Obstáculos sin intervención: sale del estacionamiento, corre esquivando
los pilares por el lado que marca su color, y para en el cuadrante del que
salió. La salida del estacionamiento se escribió como proceso aparte a
propósito, para no tocar ni una línea del código de carrera ya validado.

Commit: `0889d41` (código y manual de instalación), `e7edad6` (manual de
ensamblaje).

### Agregado
- `ronda_curvas/parqueo.py`: maniobra de salida del estacionamiento en
  vaivén, con la vigilancia por silueta completa —compara cada rumbo
  contra la chapa que hay en ESE rumbo, no contra un umbral único— y
  enderezado final contra la pared medida por el LiDAR.
- `ronda_curvas/esperar_boton.py`: puerta de arranque. El pulsador de
  `GPIO 21` abre ahora la secuencia entera y no solo la carrera; mientras
  espera, el LED de la Pico parpadea como señal visible de "cargado y
  listo". Antes la salida del estacionamiento arrancaba sola.
- `correr_completa.sh`, `wro.service` y `usar.sh`: encadenado de las dos
  fases, arranque autónomo al encender y cambio de versión entre rondas
  sin tocar los registros.
- **Conteo de vueltas por líneas de pista** en `navegacion.py`. Cada
  esquina lleva una línea naranja y una azul: cuatro de un color por
  vuelta, doce para tres. Contar tramos devuelve al robot al mismo tramo
  del que salió, propiedad que el conteo por guiñada no tiene. El yaw
  queda solo como red de seguridad, muy por encima del umbral real.
- `enlace_pico.led()` y `marcador_real.py`, el verificador independiente
  que solo cuenta un pilar como superado cuando el eje trasero cruza su
  posición con separación positiva del lado obligatorio.

### Corregido
- **El modelo del chasis estaba mal.** La silueta suponía 44 mm de
  carrocería a un costado y 94 al otro; medido con regla son **68 y 68**.
  Esa asimetría falsa daba holgura negativa con el robot parado y los doce
  primeros tiempos del vaivén cortaban al primer ciclo, avanzando uno o
  dos grados cada uno.
- **El LiDAR se veía a sí mismo.** Los cortes eran ecos planos de 49-66 mm
  en los sectores donde asoma la rueda delantera girada a tope. Se
  descartan por ser planos y por cambiar de radio con el ángulo de
  volante, que es lo que prueba que no eran un muro.
- El enderezado tenía el signo invertido respecto al vaivén y corregía
  **contra** el muro, cortándose en el primer ciclo. Y giraba a ciegas
  hasta encontrar pared: se comía 93 grados de guiñada, 22 de los 42
  segundos de la salida, y dejaba el robot mirando al revés del carril.
  Ahora, sin pared fiable a la vista, avanza recto.
- El vaivén se pasaba de vueltas (+97 grados acumulados) y sacaba el robot
  casi perpendicular al carril. Con tope de guiñada, enderezar pasa a ser
  trabajo del enderezado.
- `wro.service` tenía `Restart=always`, que relanzaba la ronda entera al
  terminar: el robot volvía a salir del estacionamiento solo, una y otra
  vez.

### Medido
Cinco corridas el 11-09, puntuadas con `marcador_real.py` y no con el
registro interno del robot:

| Corrida | Vueltas reales | Por el lado prohibido | Velocidad |
| :--- | :---: | :---: | :---: |
| 12:47 | 3 | 1 (primer pilar, a los 7,2 s) | 210 mm/s |
| 13:13 | 3 | 1 (cuarto pilar, a los 78 s) | 171 mm/s |
| 13:23 | 1,5 | **0** | 221 mm/s |
| 13:44 | 2,4 | **0** | 202 mm/s |

El fallo del primer pilar rojo, que llevaba toda la sesión, **desapareció
al arreglar la salida**: la primera medida de pared válida pasó de llegar
3,1 s después de arrancar la carrera a llegar en el primer ciclo, y los
costados al entregar el mando pasaron de 394 mm a 944 sobre un carril de
1000.

### Pendiente de validar en pista
Los dos arreglos de la sesión posterior a la última corrida: el descarte
del eco de la rueda propia y la guiñada mínima entre cruces de línea (se
verificó que 871 grados de giro, o sea 2,4 vueltas, se estaban contando
como tres).

## [v0.9.1] — 2026-09-06 a 2026-09-07 — Ronda Abierta, medidas de banco y reorganizacion

Commits representativos: `ad4e7ec`, `f566de2`, `44820da`, `8c19a03`, `bbfe582`.

### Agregado
- Ronda Abierta con el conteo de vueltas por rumbo de la IMU, y las
  herramientas de camara en vivo por HTTP y medida de las lineas de piso.
- `MEDICIONES_20260906.md`: chasis, bahia de parqueo y velocidad, todas
  remedidas con regla en vez de heredadas.

### Corregido
- La masa real de la V3 son **720 g**, no los 613 de la V2, y el margen de
  torque se rehizo con esa cifra: pasa de 2,55x a **2,18x**.
- El consumo del README era estimado por hoja de datos y con la Pi 3B. Se
  sustituye por medida real con multimetro: 1,39 A en marcha, 0,61 en
  reposo y 0,21 con la Pi apagada.
- La FSM de parqueo podia terminar en FALLO con la maniobra perfecta.
- El robot retrocedia contra los pilares que iba a rebasar.

### Reorganizado
- `ronda_cerrada` y `ronda_nueva` pasan a `legacy/`; el codigo vivo queda
  en `ronda_curvas` y `prueba_abierta`. La separacion es deliberada: lo
  archivado se conserva como registro de iteracion, pero no se despliega.

## [v0.9.0] — 2026-09-03 a 2026-09-05 — Migracion a Raspberry Pi 5

El cambio de hardware mas grande del proyecto. La Pi 3B se sustituye por
una Pi 5, y la decision es de computo medido, no de preferencia.

Commits representativos: `0b7de6c`, `840b7d1`, `f9b011f`, `e47884b`.

### Medido antes y despues
| | Pi 3B | Pi 5 |
| :--- | :---: | :---: |
| Vision por cuadro a 640x360 | 67-72 ms | **4,8 ms** |
| Vision por cuadro a 1280x720 | no cabia | **22,2 ms** |
| Edad del barrido del LiDAR | 16,0 ms de media | **0,1 ms** |

La migracion se verifico archivo a archivo: 3325 archivos identicos y los
170 CSV de telemetria coincidiendo por md5. Nada se borro de la 3B.

Eso es lo que permitio subir la camara a **1280x720 a 30 fps**. El LiDAR
dejo de tener latencia que quitar.

### Agregado
- Cerebro nuevo en `src/pi5`: homografia del suelo y vision de pista en
  milimetros, fusion camara-LiDAR de paredes y objetos, localizacion en la
  recta con mapa de doce casillas, planificador y maniobra de parqueo.
- **139 pruebas de la ronda entera que corren sin robot ni dispositivos.**
- Panel web para ver la corrida mientras ocurre, y herramientas de
  calibracion y diagnostico sobre el robot vivo.

### Corregido
- `RPi.GPIO` 0.7.1 no funciona en la Pi 5: la placa cambio al chip RP1 y
  la libreria vieja escribe directo a los registros del SoC. Se desinstala
  para que gane el shim `python3-rpi-lgpio`. Documentado en
  `INSTALACION.md` porque se vuelve a romper cada vez que alguien instala
  Adafruit-Blinka por pip.

## [v0.8.0] — 2026-08-31 a 2026-09-02 — Recorrido determinista, parqueo verificado y watchdog en la Pico

Reescritura del cerebro como `ronda_nueva`: en vez de reaccionar a lo que
aparece, recorre la pista por esquinas conocidas. Incluye la primera
maniobra de parqueo verificada y el watchdog que hace el sistema seguro
por omision.

Commits representativos: `3c96233`, `afdb163`, `b5c75e7`, `4cc12f5`,
`633b4ad`.

### Agregado
- **Watchdog autonomo de 500 ms en la Pico 2** y parser acotado de
  consignas: si la Pi se cuelga o el USB se desconecta, la Pico frena y
  centra sola en vez de seguir con la ultima orden.
- Recorrido determinista por esquinas, parqueo verificado y maniobra de
  esquina en tres tiempos calibrada con el radio de giro medido en pista.
- Ultrasonido trasero medido **por interrupcion** en la Pico, y fusion de
  esa lectura con la trasera del LiDAR para el parqueo.
- **67 pruebas offline** sin robot ni dispositivos, replay de corridas y
  despliegue del firmware de la Pico desde la propia Raspberry.

### Corregido
- El LiDAR esta a **133 mm del eje trasero, no a 162**, y va al ras del
  morro: las dos cosas medidas con regla. Los valores de montaje que se
  venian usando estaban desviados.
- Guiñada camara-LiDAR de **3,57 grados** y camara a 80 mm, calibradas
  contra pilares reales en vez de supuestas alineadas.
- El eco de la propia rueda se aleja segun cuanto gire el volante, lo que
  bloqueaba el sentido antihorario.
- El mecanismo de direccion se estaba tomando por una pared lateral.

### Validado
Tres corridas seguidas sin fallo terminal; seis esquinas y cero
emergencias tras acoplar los umbrales. Tambien se documenta lo que **no**
salio: un intento de subir la velocidad que se revirtio, y una tabla
comparativa de las cinco configuraciones probadas.

## [v0.7.3] — 2026-08-29 — Optica medida y mascara del mastil

Commits representativos: `81f5d18`, `0df7cd7`, `b9b2012`.

### Corregido
- **La optica de la camara estaba supuesta, no medida.** Se calibro con
  dos pilares reales y se dejo de rotar el cuadro, usando el sensor
  completo. El campo de vision de catalogo no es el que llega al cuadro.
- El color de un pilar dice **por que lado pasarlo**, no hacia donde
  girar. Confundir las dos cosas es lo que hacia que el robot girase hacia
  el pilar que debia esquivar.
- Se enmascara el arco que el mastil de la camara le roba al LiDAR C1.

### Agregado
- `ronda_camara`: punto de entrada e instrumentacion de la evasion, con
  cada medicion documentada **incluidas las que fallaron**.

## [v0.7.2] — 2026-08-28 — Escape frontal y desempate de esquina

Continuación directa del pendiente de v0.7.1. Caso de estudio en README
sección 8.5. El desempate se diseñó primero contra el síntoma descrito
en 8.4-3; al probarlo en pista apareció que el bucle tenía otra causa
—dos términos de control cancelándose— que es lo que arregla el escape
frontal. Ambos quedan: el escape ataca la causa medida, `GIRO_FORZADO`
es la red de abajo para el caso simétrico puro.

### Agregado
- `ronda_cerrada/navegacion.py`: nuevo estado `GIRO_FORZADO`, único con
  memoria entre ciclos. Cuenta emergencias encadenadas (cada una dentro
  de `VENTANA_ATASCO` desde la anterior); a las 4, fuerza un giro
  comprometido hacia un lado decidido una vez (última asimetría real
  memorizada entre paredes, o un lado por defecto si nunca hubo
  ninguna) y lo mantiene hasta que la esquina se abra de verdad o por
  timeout de seguridad. Pensado para el bucle del gauntlet de 8.4-3
  (133s, 51 episodios); no llegó a dispararse en la corrida de
  validación porque el escape frontal lo resolvió antes.

### Corregido
- `ronda_cerrada/navegacion.py`: **causa real del bucle**, encontrada al
  probar lo anterior en pista. Los dos términos de `_centrado_paredes`
  (posición `izq-der` y orientación `angulo_muro`) apuntan a lados
  opuestos acercándose a una esquina y se anulan: 274 de 394 ciclos con
  el frente bajo 400mm quedaban con un comando mediano de 1.5° con un
  servo que da 20-25°. El robot entraba recto contra la pared con la
  dirección centrada. Nuevo `_con_escape_frontal`: giro hacia el lado
  libre con autoridad creciente según se cierra el frente, mezclado
  sobre el centrado (no sumado, que la cancelación se lo comería).
- Criterio de racha de `GIRO_FORZADO`: el "avance neto de rumbo" no
  funcionó en pista (el robot giraba 5-7° por episodio sin escapar, así
  que la racha se reiniciaba y nunca alcanzaba el umbral). Sustituido
  por cadencia entre emergencias (`VENTANA_ATASCO=10s`), que sí separa
  atasco de incidente aislado.

### Validado
- Con motores, mismo montaje que la corrida fallida: emergencias 12→1,
  tiempo en `RETROCESO` 32%→2%, ciclos en peligro sin autoridad de
  dirección 71%→8%, rumbo recorrido 106°→442°, evasiones iniciadas 0→8.
  `GIRO_FORZADO` no llegó a dispararse (queda como red de abajo).
  Corrida cortada a los 84s a petición del equipo: falta confirmar una
  vuelta completa sin interrupción.

- `6b74f8e` feat(navegacion): forzar giro tras N retrocesos sin avance de rumbo
- `9626e80` fix(navegacion): escape frontal, el centrado se anulaba a si mismo

## [v0.7.1] — 2026-08-28 — Asistencia de esquina y límite de la reactividad pura

Caso de estudio completo en README sección 8.4. Gauntlet de 6 pilares
(el doble del reglamento) para estresar la fusión sensorial.

### Corregido
- `ronda_cerrada/navegacion.py`: emergencias en esquinas sin ningún poste
  cerca — `_centrado_paredes` no usaba `angulo_muro` (triangulación
  perp+diag ya calculada en `lidar_geometria.py`, nunca leída).
  Verificado sin motores antes de tocar la dirección; validado con
  motores en una corrida limpia: pared mínima 76mm→412mm, 5→0 emergencias.

### Agregado
- `ronda_cerrada/vision.py`: guardado opcional de frame+máscara en cada
  transición de color (`WRO_DEBUG_VISION=1`), para poder auditar un
  falso positivo de "ROJO" detectado una sola vez sin ningún pilar en
  pista, no reproducido todavía.

### Encontrado, pendiente de resolver
- Repitiendo el gauntlet completo con el arreglo activo: el robot quedó
  atrapado 133s en una esquina abordada de forma perfectamente simétrica
  (`izquierda`≈`derecha` en cada ciclo de acercamiento, `angulo_muro`
  nunca superó ±4°). Es un límite de cualquier controlador reactivo sin
  memoria entre ciclos, no un defecto del arreglo de esquina — no hay
  asimetría instantánea que triangular cuando los dos lados son
  honestamente idénticos. Necesita un mecanismo de "esta atascado, romper
  la simetría" con estado persistente entre ciclos, sin diseñar todavía.

- `71ede18` fix(navegacion): asistencia de esquina con angulo_muro, antes sin usar
- `a6358eb` feat(vision): guardar frame+mascara en cada transicion de color, opcional

## [v0.7.0] — 2026-08-27 — Reactivación de la Ronda Cerrada modular en pista

Sesión de depuración en pista de `src/pi3B/ronda_cerrada/` (la pila
modular de v0.5.0, nunca desplegada en la Pi hasta ahora) con evidencia
cuantitativa por corrida — caso de estudio completo en README sección
8.3. De un robot que no completaba una sola evasión a tres corridas
seguidas sin emergencias, esquivando el pilar rojo por la derecha y
reincorporándose al carril.

### Corregido
- `comun/enlace_pico.py`: la telemetría con sensor de color rompía el
  parseo del *heading* en silencio (la IMU quedaba clavada en 0.0).
- `ronda_cerrada/camara_driver.py`: el frame no se rotaba pese a que la
  cámara va montada invertida en el chasis.
- `ronda_cerrada/navegacion.py`: *windup* en `_centrado_paredes` sin
  recorte al servo; timeouts de evasión más cortos que la física real;
  salida de `RETROCESO` que no comprobaba si el peligro ya se había
  despejado; `SOBREPASO`/`REINCORPORACION` corrigiendo por rumbo en vez
  de por posición (podían cumplir el objetivo entero y acabar contra un
  muro); `DIST_SOBREPASO_MM` dimensionado por el poste cuando en la
  práctica lo limita la pared.
- `ronda_cerrada/tracker.py`: `MM_POR_SEG_A_PWM100` sobreestimaba 2.3
  veces la velocidad real (medida en pista: curva PWM→velocidad).
- Hardware: regulador XL4015/4016 entregando 4.9V, la Raspberry en bajo
  voltaje activo incluso en reposo (`vcgencmd get_throttled` = `0x50005`
  → `0x50000` tras reajustar el trimpot a 5.132V).

### Agregado
- `comun/registro_metricas.py`: percepción cruda por ciclo (`frontal`,
  `izquierda`, `derecha`, `trasera`, `color_cam`, estado del tracker),
  necesaria para diagnosticar de dónde sale cada error en vez de solo el
  error ya derivado.
- `src/pico/main.py`: sincronizado con el sensor de color TCS3472 (que
  ya estaba flasheado en el Pico físico pero nunca se había commiteado)
  y con el campo opcional `kd` de la consigna serial (que la versión
  flasheada había perdido al agregar el sensor).
- README secciones 5.3-C (estado real del sentido de carrera: hardware
  instalado, telemetría parseada, no consumido por `navegacion.py` — y
  por qué eso es un diseño deliberado, no una omisión) y 8.3 (caso de
  estudio completo con evidencia por corrida).

- `c84f387` fix(comun): parsear la telemetria de la Pico con sensor de color
- `268c633` fix(ronda_cerrada): enderezar el frame, la camara va montada invertida
- `528aef5` feat(metricas): registrar percepcion cruda por ciclo, no solo el error
- `ad74c17` fix(navegacion): recortar el centrado al servo, elimina el windup
- `e78884f` fix(navegacion): derivar los timeouts de evasion de la velocidad real
- `9e857da` fix(navegacion): terminar el retroceso al despejarse, no por reloj
- `cb4f710` refactor(navegacion): salir de la evasion por posicion, no por rumbo
- `12ca0f1` fix(tracker): medir la velocidad real, el modelo sobreestimaba 2.3 veces
- `e5999af` fix(navegacion): acortar SOBREPASO, lo limita la pared y no el poste
- `c8219c1` feat(pico): sincronizar el firmware con el sensor de color TCS3472

## [v0.6.0] — 2026-07-27 — Métricas cuantitativas de rendimiento

### Agregado
- `src/pi3B/comun/registro_metricas.py`: logger CSV de telemetría por
  ciclo (fase, estado, heading, error lateral, ángulo, velocidad),
  integrado en `ronda_abierta.py` y `ronda_cerrada.py`.
- `src/pi3B/calibracion/analizar_log.py`: resume los CSV de
  `registro_metricas.py` en métricas agregadas (error lateral
  promedio/máximo, saturación del servo, eventos de emergencia) para
  validar el ajuste de `KP_LATERAL`/`KD_ESTABILIDAD` con datos en vez de
  observación cualitativa (README sección 5.4).
- Este `CHANGELOG.md`.

- `26badf2` feat(pi3B): instrumentar métricas cuantitativas de rendimiento en pista

## [v0.5.0] — 2026-07-21 a 2026-07-24 — Navegación modular y reorganización

Reescritura de la máquina de estados de evasión de la Ronda Cerrada como
lógica pura sin I/O, y reorganización de `src/pi3B/` en carpetas por rol
(`comun/`, `ronda_abierta/`, `ronda_cerrada/`, `calibracion/`).

- `88698f0` feat(close): reescritura modular de la navegación de la Ronda Cerrada
- `9aafb59` chore(pi3B): organizar src/pi3B en carpetas por rol
- `7cee7f2` experimento(pi3B): separar driver/procesador en lidar y cámara
- `2f87433` experimento(pi3B): unificar drivers de LiDAR/Pico entre las dos rondas
- `2b88cec` feat(pi3B): agregar deploy.sh y documentar clonar+desplegar en la Pi
- `4b8a06e` fix(ronda_cerrada): retroceso reactivo con perfil LiDAR de 360 grados
- `4e67679` fix(ronda_cerrada): la evasión ahora ve las paredes

## [v0.4.0] — 2026-07-07 a 2026-07-21 — Ronda Cerrada: visión, tracker y depuración en pista

Primera implementación funcional de la Ronda Cerrada (evasión de postes
rojo/verde) con calibración HSV dedicada, tracker LiDAR y el caso de
estudio de depuración documentado en README sección 8.2.

- `248d83e` Códigos de calibración HSV para la ronda cerrada
- `140f553` fix(close_round): corregir lado de evasión invertido y espacio de color de la cámara
- `6c9158f` feat(close_round): usar control proporcional con tracker LiDAR en la evasión
- `5a182d9` fix(close_round): subir el timeout de confirmación DETECTADO->ESQUIVANDO de 0.3s a 1.2s
- `6602a07` fix(close_round): dar más tiempo/ángulo a RECENTRANDO para converger
- `0c559a1` fix: implementar modo Inercial real (sostener última pared válida)
- `f23a8b7` refactor(close_round): dividir Close2_round.py en módulos por responsabilidad
- `3144a3f` docs: agregar manual de instalación (INSTALACION.md)

## [v0.3.0] — 2026-07-03 a 2026-07-05 — Chasis de producción V2 (LEGO, 613 g)

Migración del chasis monocasco impreso en 3D (V1, ~800 g) a la
plataforma LEGO Technic de producción (V2, 613 g exactos). Incluye la
corrección del centro de dirección del servo (probado en 180°, revertido
a 90° tras validación en pista).

- `07cea89` Estructura base y hardware V2 en LEGO
- `4afdab4` justificar ventajas cinemáticas del chasis LEGO de 613g frente a impresión 3D
- `ca1ea85` actualizar README principal con peso de 613g, regulador XL4016 y análisis de torque
- `5e917e1` implementar matriz fotográfica comparativa de 6 ejes para V1 y V2
- `305e46c` Arreglo de ángulo central del robot de 90 a 180 grados
- `dc4f532` Arreglo en equivocación de ángulo central (revertido a 90°)

## [v0.2.0] — 2026-06-18 — Migración a Raspberry Pi 3B

Cambio de la capa de percepción de alto nivel de Raspberry Pi 5 a
Raspberry Pi 3B (hardware disponible al equipo).

- `bf474b5` Arreglo prueba de RPlidar, cambio de Raspberry Pi de 5 a 3B

## [v0.1.0] — 2026-01-06 a 2026-06-10 — Estructura inicial y control serial

Esqueleto reglamentario del repositorio (`src/`, `v-photos/`, `schemes/`,
etc.) y primera versión funcional del protocolo serial Pi↔Pico: máquina
de estados base en la Pico 2 y parser no bloqueante para el LiDAR.

- `07a7bc6` chore: crear estructura de carpetas oficiales para WRO 2026
- `1a6b984` feat(pico2): script nativo en MicroPython para control de actuadores por serial
- `d84f8be` feat(control): implementar máquina de estados base y bucle de telemetría
- `99b7c0f` feat(pico): implementar parser serial no bloqueante para lectura de zonas del lidar
- `99b052e` feat(pico): corregir mapeo de pines I2C de la IMU y habilitar telemetría
