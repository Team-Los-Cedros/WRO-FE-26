# Registro de Cambios

Notas de versión del proyecto, basadas en los hitos reales del historial de
commits (ver `git log`). Formato inspirado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/).
Cada versión referencia los commits representativos de ese hito para
poder auditar el cambio exacto con `git show <hash>`.

## [v0.7.2] — 2026-08-28 — Desempate de esquina simétrica (GIRO_FORZADO)

Continuación directa del pendiente de v0.7.1. Caso de estudio en README
sección 8.5.

### Agregado
- `ronda_cerrada/navegacion.py`: nuevo estado `GIRO_FORZADO`, único con
  memoria entre ciclos. Cuenta reintentos de `RETROCESO` seguidos sin
  avance neto de rumbo; a los 4, fuerza un giro comprometido hacia un
  lado decidido una vez (última asimetría real memorizada entre paredes,
  o un lado por defecto si nunca hubo ninguna) y lo mantiene hasta que
  la esquina se abra de verdad o por timeout de seguridad. Rompe el
  bucle emergencia-retroceso-reintento de la corrida del gauntlet
  (133s, 51 episodios).

### Validado
- Solo fuera de pista: barridos LiDAR sintéticos reproduciendo el bucle
  medido (rompe a la 4ª racha), un caso negativo (no dispara si el
  rumbo sí progresa entre emergencias) y la memoria de asimetría
  (recuerda el lado real en vez del default cuando lo hay). **Pendiente
  de validar con motores en pista** — la simulación confirma la lógica
  de la máquina de estados, no si `ANGULO_GIRO_FORZADO`/`TIMEOUT_GIRO_FORZADO`
  bastan con la geometría real del chasis.

- `6b74f8e` feat(navegacion): forzar giro tras N retrocesos sin avance de rumbo

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
