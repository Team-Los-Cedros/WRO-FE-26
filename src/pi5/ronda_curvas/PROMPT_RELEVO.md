# Prompt para continuar la tarea

Continúa el trabajo de mi robot WRO en
`C:\Users\usuaio\Desktop\Claude\repo\WRO-FE-26`, rama
`fix/ronda-curvas-retorno-parqueo`, carpeta `src/pi5/ronda_curvas`.

Tu primera lectura debe ser el archivo completo
`src/pi5/ronda_curvas/ESTADO_Y_RELEVO_20260907.md` y las instrucciones
`C:\Users\usuaio\Desktop\AGENTS.md`; después revisa `git status` y los diffs.
Los cambios están guardados pero SIN commit. La rama remota solo contiene
la base. Usa ESTE checkout; no crees un worktree desde la rama remota porque
perderías los archivos nuevos no versionados. No restaures ni borres trabajo.

Objetivo autorizado: arreglar el fallo tras aproximadamente siete bloques y
hacer que, después de tres vueltas, el robot entre y estacione en la bahía
correspondiente al lugar del que salió. Sale DESDE EL CARRIL, no desde dentro
de los separadores magenta.

El agente anterior ya corrigió capturas durante recuperación, compromiso
tardío de pilares, pérdida de la velocidad negativa en el arbitraje,
emergencias ignoradas por GIRO_FORZADO, retroceso sin espacio, uso de IMU
caducada y mezcla de relojes en el tracker. Ese conjunto pasó la suite de
61 pruebas en una versión intermedia. El diagnóstico del objetivo 7 se basa
en el log remoto `logs/ronda_camara_20260907_011832.csv`. Ojo con los marcos:
un pilar con y=-81 desde el LiDAR todavía tiene y=47 desde el eje trasero;
el problema era iniciar su maniobra demasiado tarde, no necesariamente estar
detrás del eje trasero.

El parqueo NO está terminado y NO se desplegó. Se añadió detección de bahía,
origen persistente, seguimiento de una pareja de separadores anclada a paredes,
parser de US y un planificador/seguidor experimental. La FSM fija de parqueo
archivada se comprobó insuficiente, por lo que ahora el flujo activo pasa por
`parqueo/seguidor.py` y `parqueo/planificador.py`. Hay código heredado duplicado
que conviene simplificar después de validar el comportamiento.

Pendiente inmediato:

1. Reproduce `tests/repro_parqueo.py`. Con margen 16 mm, el planificador actual
   llega a PLANIFICAR en ambos lados y falla al buscar ruta. Investiga si es
   discretización, presupuesto, criterio de meta o geometría. No reduzcas
   márgenes ni cambies dimensiones para fabricar una prueba verde.
2. Consigue estacionamiento completo en simulación, a izquierda y derecha,
   comprobando el CUERPO real contra muro y separadores y la pose final;
   luego comprueba variaciones de radio, inicio, ruido y oclusiones.
3. Valida la pose por LiDAR/IMU del seguidor, incluyendo offset lateral del
   LiDAR y selección de caras. Comprueba guardas de pérdida de pose/US,
   frenado, cambios de marcha y timeout sin falso éxito.
4. Las 17 regresiones tienen ahora DOS fallos: `test_timeout_no_es_estacionado`
   y `test_verificacion_exige_robot_entero_dentro`. Sus fixtures configuran la
   FSM anterior. Adáptalas al controlador activo conservando los requisitos.
5. Valida tres vueltas y retorno en ambos sentidos. El contador de esquinas
   y su ventana de yaw aún requieren pruebas completas; no asumir que el yaw
   acumulado o dos distancias laterales identifican el punto de salida.
6. Revisa integración de watchdog/hilos con hardware simulado, configura la
   geometría real y limpia duplicación. Corre la suite completa antes de entregar.

Acceso autorizado a la Pi:
`ssh -i C:\Users\usuaio\.ssh\id_ed25519_wro pi@192.168.30.115`.
Carpeta `/home/pi/ronda_curvas`; no es Git. Está intacta desde la versión
original de esta sesión. Antes de modificarla, cambia primero localmente y
crea backup remoto recuperable. Verifica imports y hashes sin accionar motores.
Comprueba que el firmware de la Pico PUBLICA `US:` válido; hasta ahora solo se
revisó el firmware del repositorio, no el flasheado.

Para una prueba con movimiento hay una condición explícita en Desktop/AGENTS.md:
avisarme y esperar mi confirmación de que pulsé el botón. No uses arranque
automático ni reinicies una carrera sin esa confirmación. No hagas commit/push
de código ni envíos a GitHub sin que lo solicite. Las correcciones e implementación
ya están autorizadas; continúa autónomamente con trabajo local y pruebas puras.

La limpieza de ramas ya se hizo por petición expresa: quedan main y la nueva,
en local y GitHub. No repetirla. Hay backup completo verificado en
`.git/ramas-antes-ronda-curvas-20260907.bundle` (aproximadamente 212 MB).
Conservarlo. Los videos/CSV ajenos no seguidos también se conservan.

Python local:
`C:\Users\usuaio\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe`.
Desde ronda_curvas: `python -B -m unittest discover -s tests -q` usando esa ruta.
La Pi tiene Python3 y NumPy; no tiene rg.

Entrega un resultado verificable y distingue claramente pruebas de software,
simulación física y prueba real. No afirmes que el parqueo funciona mientras
el recorrido completo falle. Actualiza ESTADO_Y_RELEVO_20260907.md al finalizar.
