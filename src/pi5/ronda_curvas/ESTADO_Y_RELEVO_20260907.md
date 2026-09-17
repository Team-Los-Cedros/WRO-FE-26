# Ronda curvas: estado real y relevo — 7 de septiembre de 2026

## Punto de entrega

**Trabajo incompleto. No desplegar ni presentar el parqueo como terminado.**

Las correcciones iniciales de navegación y seguridad pasaron las pruebas.
Después se incorporó un planificador de parqueo y un seguidor por LiDAR/IMU;
esa integración es experimental y la última simulación NO consigue aparcar.
La última ejecución de las 17 regresiones da 15 correctas y 2 fallidas.
Los archivos están guardados en el árbol de trabajo local, SIN commit.
La Pi conserva su versión anterior: no se modificaron sus archivos ni se
activaron motores durante esta sesión.

El usuario pidió detener la implementación para ahorrar su límite de uso y
documentar el relevo. No continuar implementando sin que retome la tarea.

## Objetivo y aclaraciones del usuario

1. Corregir el fallo que parece aparecer después de unos siete bloques.
2. Completar tres vueltas y estacionar en la bahía correspondiente al origen.
3. Aclaración explícita: **el robot sale desde el carril**, no desde dentro
   de la bahía; al final debe entrar entre los separadores magenta.
4. Se autorizó implementar las correcciones.
5. Se autorizó borrar las otras ramas locales y de GitHub y crear una nueva
   desde `main`. Esa limpieza ya está hecha.

## Repositorio, rama y permisos

- Repositorio: `C:\Users\usuaio\Desktop\Claude\repo\WRO-FE-26`.
- Directorio de trabajo: `src/pi5/ronda_curvas`.
- Rama: `fix/ronda-curvas-retorno-parqueo`.
- Base y HEAD actual: `bbfe582bdebb29a1bc764c63278b9f344b620629`.
- Remoto: `https://github.com/Team-Los-Cedros/WRO-FE-26`.
- La rama remota nueva existe pero todavía apunta a la base: **los cambios
  de implementación no están en GitHub**. Continuar en este mismo checkout.
- No se hicieron commits ni se publicaron cambios de código.
- Hay videos/CSV no seguidos preexistentes fuera de esta tarea: conservarlos.

Instrucciones aplicables: `C:\Users\usuaio\Desktop\AGENTS.md`. Exigen editar
primero el repositorio local, no hacer commit/push de código salvo petición
explícita y, para probar moviendo el robot, avisar y esperar confirmación de
que el usuario ha pulsado el botón. Su mención de Pi 3B es anterior a esta
tarea; el destino que indicó el usuario es la Pi accesible abajo.

### Limpieza de ramas ya realizada

Quedan `main` y `fix/ronda-curvas-retorno-parqueo`, localmente y en GitHub.
Se borraron 16 ramas locales y 4 remotas. Las remotas eliminadas fueron:

- `Close_round/estrategia-nueva`
- `medidas/banco-0609`
- `prueba/estacionamiento`
- `prueba/sensor_color`

Se guardó y verificó un bundle completo ANTES de borrar:

`C:\Users\usuaio\Desktop\Claude\repo\WRO-FE-26\.git\ramas-antes-ronda-curvas-20260907.bundle`

Tamaño: 211750657 bytes. Incluye las referencias originales y el historial
completo, incluidos commits que no estaban integrados en `main`. No borrarlo.
Para inspeccionar lo recuperable, usar `git bundle list-heads` sobre ese archivo.
Restaurar una rama, si se solicita, con `git fetch <bundle> refs/heads/<vieja>:refs/heads/<recuperada>`.

El worktree existente
`C:/Users/usuaio/Desktop/Claude/repo/WRO-FE-26.worktrees/wro-fe-2026-robot-src-analysis`
se dejó en HEAD separado en `9aafb59`; estaba limpio y sus archivos se
conservaron. Se podaron registros de cuatro worktrees que ya no existían.

## Acceso a la Pi

```powershell
ssh -i 'C:\Users\usuaio\.ssh\id_ed25519_wro' -o BatchMode=yes pi@192.168.30.115
```

Carpeta remota: `/home/pi/ronda_curvas`. No es un repositorio Git.
Sistema observado: Raspberry Pi, Linux aarch64; Python `/usr/bin/python3`,
NumPy 2.2.4. `rg` no está instalado allí: usar `grep` o copiar lecturas al PC.

El contenido de `main:src/pi5/ronda_curvas` coincide con la copia remota de
partida. Hashes remotos de referencia, antes de cambios:

```text
navegacion.py  b2c7ff04ba3c54c3f516068b654d698b0b9389cccf5c38d282ce9efb9e89baf9
tracker.py     6e46eed7ee00218fa82e08717a09ffd1ed32f23e983f7ae14a211157fbc324e2
ronda_camara.py 262627faff156ea9e5092666b3e0e9bb011c7e13d8a20002a61e2bb6952c4996
```

Por ahora no se abrió el puerto serial para verificar la telemetría real.
Se verificó el formato `US:` contra el firmware del repositorio, no contra
el firmware actualmente flasheado. El próximo agente debe comprobar que la
Pico efectivamente publica ultrasonido fresco antes de una prueba de parqueo.

## Diagnóstico comprobado

Log principal: `/home/pi/ronda_curvas/logs/ronda_camara_20260907_011832.csv`.
Las marcas de tiempo que siguen son segundos del campo `t` del CSV.

| Tiempo | Evidencia |
| --- | --- |
| 73.149 | El objetivo 6 termina y pasa a RECUPERACION. |
| 73.844 | Ya existe objetivo 7 durante RECUPERACION: LiDAR x=290, y=135 mm. |
| 75.844 | CONFIRMACION del 7 con x=209, y=-31 mm. |
| 76.143 | COMPROMISO con x=210, y=-81 mm; se reinicia el progreso. |
| 80.636 | ABORTO, yaw=-604.84 grados, frente=512 mm. |
| 82.242 | RETROCESO con frente=222 mm, pero la consigna registrada conserva velocidad positiva 25. |
| 82.939 | Entra GIRO_FORZADO tras varios intentos breves de retroceso. |
| 85.238 | Sigue GIRO_FORZADO a velocidad +25, con frente y derecha de 23 mm. |

**Corrección de la explicación inicial:** `trk.y` está referido al LiDAR.
Con `geo.LIDAR_X=128`, y=-81 implica y=47 desde el eje trasero. No era
necesariamente un pilar detrás del eje trasero: era un compromiso demasiado
tardío mientras el robot ya estaba rebasándolo. El yaw continúa hasta unos
-653 grados; no hay evidencia en ese log de un salto o reinicio súbito de IMU.
El ID 7 identifica una captura, no demuestra que sea el séptimo objeto físico
distinto. No convertir esa correlación en un contador real de bloques.

Causas de software:

- La captura estaba permitida en cualquier estado fuera de ESTADOS_MANIOBRA,
  incluidos RECUPERACION, ABORTO y emergencias.
- CONFIRMACION y COMPROMISO no volvían a comprobar que quedara espacio frontal
  para iniciar la maniobra.
- `_nominal_arbitrado()` devuelve solo ángulo; llamaba `_sin_salida()[1]`,
  perdiendo la velocidad negativa y conservando el avance del llamador.
- GIRO_FORZADO estaba excluido de la emergencia y no arbitraba su arco.
- Una trasera ocupada hacía abandonar RETROCESO enseguida, produciendo ciclos
  de avance/retroceso sin salir del atasco.
- `heading_valido()` existía, pero la carrera no lo consultaba.
- `tracker.asociar()` registraba `time.time()` incluso durante simulaciones
  cuyo reloj empezaba en cero, ocultando pérdidas reales por timeout.

Parqueo original: se habilitaba a 1010 grados; comparaba dos laterales con la
primera lectura y se detenía por coincidencia o por seis segundos de timeout.
No identificaba la bahía ni ejecutaba una entrada. No equivalía a tres vueltas
completas ni a estar aparcado.

## Cambios guardados

### Correcciones de navegación y seguridad

`navegacion.py`:

- Captura nueva solo en CRUCERO.
- Distancia mínima de captura y rechazo de candidato tardío en CRUCERO,
  CONFIRMACION y COMPROMISO, con coordenadas LiDAR explícitas.
- GIRO_FORZADO vuelve a respetar emergencia y arbitraje de obstáculos.
- Propagación del par completo velocidad/ángulo cuando el arbitraje cambia
  a RETROCESO; una trasera ocupada o sin dato produce parada y eventual FALLO.
- Integración de RetornoParqueo, estados FIN/FALLO y motivo de terminación.
- Se sustituyó la parada por firma lateral por el nuevo módulo de retorno.
- Algunas constantes antiguas de parqueo quedaron sin uso; limpiar al cerrar.

`tracker.py`: `asociar(..., ahora=None)` usa el mismo reloj que navegación.

`enlace_pico.py`: parser puro de IMU/US, rechazo de NaN/infinito y lectura
de US con caducidad. Se acepta rango 20..4000 mm; ese máximo coincide con
`src/pico/ultrasonido.py`, donde FiltroUltrasonido tiene maxima_mm=4000.

`ronda_camara.py`: comprueba IMU, pasa US al controlador y convierte su
distancia a referencia LiDAR para el retroceso. Serializa control/envíos con
RLock, captura excepciones del callback y agrega watchdog y parada terminal.
Revisar con mocks las carreras de hilos y la parada desde callback: todavía
no se añadieron pruebas específicas de esta integración de hardware.

`registro_metricas.py`: agrega US, motivo final, esquinas, vueltas, estado
de parqueo y diagnóstico de hueco.

### Retorno y parqueo: integración experimental

`retorno_parqueo.py`:

- Captura cinco barridos estables del origen, con pared frontal y lateral.
- Usa yaw relativo, persistencia y avance estimado PWM para contar 12
  cuadrantes. El avance NO es odometría real; no existen encoders aquí.
- Inicia búsqueda al final de la tercera vuelta y exige 12 cuadrantes,
  yaw entre 1080 y 1115, geometría de bahía y pared frontal próxima al origen.
- La comparación frontal tolera 900 mm: es una ventana del segmento para
  aproximarse a la bahía, no prueba de posición exacta del punto de salida.
- Mantiene una pareja de separadores anclada a paredes medidas cuando uno
  queda oculto. No conserva simplemente coordenadas locales antiguas.
- Requiere US fresco; falta de datos produce parada y FALLO, nunca éxito.
- Comprueba puntos LiDAR contra la envolvente completa de la siguiente
  consigna de parqueo; no solo el sector frontal.
- Actualmente llama al nuevo `SeguidorBahia`, no a la FSM antigua de arcos.

`parqueo/`:

- `modelos.py`, `percepcion_lidar.py`, `estacionamiento.py` se copiaron mediante
  apply_patch desde `src/pi5/legacy/ronda_nueva` en main. Así el despliegue no
  depende de importar otra ronda. Los originales legacy no se modificaron.
- PercepcionLidar separa clusters en rectas para detectar los delimitadores.
- Se corrigió en la copia de estacionamiento el signo de entrada en reversa,
  la alineación y la verificación del cuerpo completo. **Esa FSM ya no es el
  controlador principal**: se conserva como configuración/diagnóstico y
  contiene lógica duplicada que conviene simplificar después.
- `planificador.py`: búsqueda Ackermann con avance/reversa, arcos de 25 mm,
  discretización x/y de 10 mm y ángulo de 4 grados, prueba SAT del rectángulo
  completo contra delimitadores, margen actual 16 mm. Busca una pose dentro
  y paralela. Búsqueda incremental que cede cada ~35 ms para no bloquear
  los sensores mientras el robot permanece parado.
- `seguidor.py`: aproximación, planificación, seguimiento de ruta y
  verificación. Estima pose con yaw y registro de puntos LiDAR contra muro,
  caras de los separadores y muro frontal. PWM predice entre barridos; las
  medidas corrigen la pose. Esta localización también requiere validación.

`parqueo_config.json`: configuración independiente y conservadora. Combina
la geometría histórica de ronda_curvas (222 mm de largo, 60 de voladizo)
con ancho de envolvente 140 mm. Los documentos del montaje más nuevo indican
210/130/55 y radios distintos: comprobar qué chasis está conectado. No cambiar
medidas para hacer pasar una simulación sin evidencia. Parte de los parámetros
del controlador de arcos dejó de gobernar al usar SeguidorBahia; centralizar.

## Pruebas: resultados y orden temporal

1. Antes de editar: **44 pruebas correctas**, suite original completa,
   unos 16.2 s en Windows.
2. Después de las correcciones iniciales y retorno, antes de SeguidorBahia:
   **61 pruebas correctas**, suite completa, unos 22.7 s.
3. Pruebas de rayos contra bahía: **4 correctas**. Detectan ambos lados,
   captura estable del origen, pared vista dentro y actualización de un
   separador oculto mediante paredes. No equivalen a aparcar en movimiento.
4. La simulación de la FSM archivada reveló signo de reversa incorrecto;
   corregido el signo, chocaba la envolvente prevista con un delimitador.
5. Primer planificador con margen 8 mm: encontró rutas offline. Un ejemplo
   desde (-400,300,0) encontró 43 tramos, unos 63013 nodos y 6.9 s; desde
   (300,300,0), 27 tramos y 0.8 s. **No prueba ejecución segura.**
6. Seguidor con ese margen: aparecieron paradas por holgura durante ejecución,
   con desviación entre radio planificado y radio del simulador.
7. Último estado, margen 16 mm y búsqueda incremental: en ambos lados llega
   a PLANIFICAR hacia t=9.1 s, pero termina hacia t=30.6 s con
   **FALLO: no se encontro plan Ackermann libre**. Pose estimada izquierda
   aproximadamente (280.01,300.69,-0.01 rad); derecha (280.01,299.3,0.01).
   Puede ser discretización/poda/presupuesto o espacio exigido. No se ha
   demostrado que físicamente sea imposible. NO bajar márgenes sin analizar.
8. Última comprobación de `test_regresiones.py`: **17 ejecutadas, 2 fallidas**:
   `test_timeout_no_es_estacionado` y
   `test_verificacion_exige_robot_entero_dentro`. Configuran el controlador
   anterior y ya no gobiernan el flujo nuevo. Adaptar fixtures al SeguidorBahia
   conservando los requisitos de timeout != éxito y cuerpo entero dentro;
   no eliminarlas ni relajar las aserciones para ocultar fallos.
9. No se volvió a ejecutar la suite completa después de la integración final.

## Qué falta, en orden

1. Leer este estado y los diffs. Conservar los cambios; no resetear ni recrear
   la rama. Separar mentalmente los arreglos verificados del parqueo experimental.
2. Resolver la búsqueda de aparcamiento con margen físico y radios válidos.
   Perfilar poda/discretización, criterio de meta y geometría, y comprobar
   trayectorias tanto a izquierda como a derecha. El seguidor debe completar
   la trayectoria y verificar la pose real del simulador sin tocar separadores.
3. Validar localización: marcos LiDAR/eje trasero, offset lateral, selección
   de caras de separadores, observabilidad cuando faltan puntos, ruido y yaw.
   `tests/simulador.py:Robot.pos_lidar()` solo aplica el offset longitudinal;
   el código real también usa `geo.LIDAR_Y=-4`. No ignorar esta diferencia.
4. Adaptar las dos pruebas fallidas al controlador actual. Añadir pruebas
   de timeout, falta de US, pérdida de pose, cambios de sentido y colisiones
   del seguidor; pruebas de parser de telemetría y watchdog con hardware falso.
5. Validar retorno completo en los dos sentidos y con posiciones de salida
   distintas dentro del carril. La ventana yaw 1080..1115 puede perder el origen
   por deriva; el contador usa movimiento estimado y no demuestra vueltas ante
   giros espurios. Evitar atribuirle una precisión que no se ha probado.
6. Revisar si la exclusión de captura durante RECUPERACION hace perder el
   siguiente pilar; el arreglo evita la captura tardía, pero no puede autorizar
   saltarse reglas de paso. Probar obstáculos consecutivos con trayectorias reales.
7. Depurar el código duplicado y parámetros sin uso de la FSM archivada.
   `parqueo/estacionamiento.py` contiene comentarios históricos que ya no son
   descripción fiel del controlador actual.
8. Revisar máscara de oclusión: navegación antigua usa 163..189; el nuevo
   parqueo usa 140..213 tomado del montaje con soporte de US. Calibrar contra
   el montaje real. No suponer que todos los ecos de 40 mm son ruido del chasis.
9. Ejecutar suite completa y validación sintáctica. Generar un informe final
   honesto de qué pasó en simulación y qué sigue siendo prueba física.
10. Antes de copiar, comprobar hashes y cambios remotos, hacer backup recuperable
   de `/home/pi/ronda_curvas` y copiar solo archivos de la tarea desde local.
   Verificar hashes e imports en la Pi sin mover motores.
11. Para mover el robot, seguir `Desktop/AGENTS.md`: avisar y esperar que el
   usuario confirme que pulsó el botón. No usar WRO_ARRANQUE_AUTO para eludirlo.

## Ejecución local reproducible

Python disponible, aunque no está en PATH:

```powershell
Set-Location 'C:\Users\usuaio\Desktop\Claude\repo\WRO-FE-26\src\pi5\ronda_curvas'
& 'C:\Users\usuaio\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -B -m unittest discover -s tests -q
& 'C:\Users\usuaio\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -B -m unittest discover -s tests -p test_regresiones.py -v
& 'C:\Users\usuaio\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -B tests/repro_parqueo.py
```

El último comando reproduce la exploración completa usada al cerrar la sesión.
No toca hardware. El resultado esperado del estado entregado es FALLO de
planificación en ambos lados, no LISTO. El script devuelve código distinto de
cero mientras no se verifique el parqueo de ambos escenarios.
