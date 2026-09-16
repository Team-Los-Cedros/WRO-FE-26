# Dos pilares: estado de la prueba

La red conserva todas las cajas del cuadro y sus tiempos de captura. La
navegacion mantiene su objetivo actual y observa un segundo pilar durante
tres cuadros distintos antes de considerarlo confirmado. Una observacion
caduca a los 300 ms. Una identidad confirmada puede conservarse como
prediccion durante un maximo de cinco segundos, con incertidumbre creciente
por avance y giro y descarte al alcanzar 300 mm. La prediccion solo puede autorizar control con la opcion explicita
`WRO_DOS_MEMORIA=1` y las restricciones descritas abajo. Una deteccion tentativa perdida reinicia su confirmacion.

## Modos

- Sin `WRO_DOS_PILARES`, la nueva preferencia de salida queda desactivada.
- `WRO_RED=1 WRO_RED_HZ=8 WRO_DOS_PILARES=sombra`: registra el comando base y
  el propuesto, pero conserva el comando base.
- `WRO_DOS_PILARES=activo`: permite escoger entre comandos admitidos por el
  arbitraje existente, solo al final de PASO_LATERAL o en SALIDA_PILAR.
  Requiere posicion LiDAR y pertenencia comprobada a la seccion actual.
  Este modo todavia no se ha validado en pista.

Con `WRO_DOS_MEMORIA=1`, el modo activo tambien admite un recuerdo
confirmado por LiDAR dentro de la seccion: edad maxima 1,5 s, sigma menor
de 150 mm, distancia de al menos 500 mm y posicion por delante del LiDAR.
Las posiciones exclusivamente visuales siguen sin autorizar control.
La holgura exigida al siguiente incorpora sigma; esta incertidumbre es
modelada y no constituye una cota garantizada del error real.

La preferencia nueva queda limitada a 6 grados respecto al comando base y
a comandos alcanzables por el limitador del servo. No interviene durante
APERTURA, emergencias, readquisicion del actual ni concesiones de seguridad.
La prediccion de salida tiene un horizonte de 0,8 s; no constituye una
garantia de trayectoria libre mas alla del arbitraje existente.

## Lo que falta para anticipar el siguiente cuadrante

La camara puede ver un pilar que el LiDAR no asocia. Se conserva como
`visual_relativa`: su profundidad se estima por la razon de alturas entre
su caja y la caja del actual asociada al LiDAR en el mismo cuadro. Esto
supone pilares de igual altura, completos y sin solapamiento. El sesgo de
rumbo observado en el actual se propaga a esa estimacion, sin modificar la
calibracion optica.

Una posicion `visual_relativa` o situada fuera de la seccion solo produce
propuestas en sombra. Falta verificar la distancia visual con medidas
independientes, validar la memoria del segundo al ocultarse el primero y
validar la salida alrededor del muro interior antes de darle control.
La observacion de dos pilares y la planificacion completa entre cuadrantes
no deben confundirse: esta version es el primer paso experimental.

## Registros y comprobaciones

`red_*.csv` mantiene las columnas originales y agrega secuencia, tiempo
monotono de captura y todas las cajas en JSON. `dos_*.csv` incluye identidad
actual, posicion siguiente, fuente, confirmaciones, comando base, propuesta
y si se aplico. Ambos llevan tiempo Unix; no necesitan alinearse por el final.
Los resultados se contrastan con el video cenital y el marcador independiente.

El 15-09-2026, inmovil en pista, la red detecto verde y rojo en 12/12 cuadros.
Una segunda observacion produjo 48 muestras sincronizadas: el verde tenia
asociacion LiDAR y el rojo no. La reproduccion local confirmo el rojo en
45/48 muestras. Su posicion visual relativa, aun sin validar, fue unos
(466, 1181) mm desde el LiDAR. Se observo un desacuerdo de aproximadamente
5–6 grados entre el rumbo del verde en camara y LiDAR; requiere comprobacion
optica con varios puntos.

Daniel midio ambos pilares: altura 100 mm y distancia horizontal desde el
lente de 760 mm al verde y 1420 mm al rojo. Comparando distancias directas
desde el mismo origen (no profundidad longitudinal), la mediana LiDAR del
verde es 739 mm y la estimacion visual relativa del rojo es 1361 mm: errores
de -21 mm (-2,8 %) y -59 mm (-4,2 %). No se reajusto el estimador con estos
datos. Es un contraste en una sola pose, insuficiente para habilitar el
control visual o corregir la optica.

## Pruebas cortas grabadas

Las corridas se limitaron a veinte segundos desde GP21, con parada por
SIGINT. Ambas terminaron con `throttled=0x0` y ninguna propuesta aplicada.

- `234224`: el verde se capturo en (98, 633) mm. La FSM abandono su
  maniobra; el video muestra al verde por el lado correcto y al rojo por
  el incorrecto. El marcador independiente conto cero, porque no siguio
  los objetivos hasta el cruce. La red vio dos cajas en 32 inferencias de
  marcha, pero el observador perdio al siguiente al degradarse el actual.
  Mediana de ciclo 100 ms, p95 104 ms.
- `235041`: el verde se capturo en (112, 419) mm, una colocacion inicial
  distinta. La FSM completo su maniobra y el video confirma el paso del
  verde, pero el rojo volvio a quedar por el lado incorrecto. La red vio
  ambos en cinco inferencias de marcha y despues perdio el rojo de vista.
  Mediana de ciclo 100 ms, p95 106 ms. La memoria independiente de una caja
  todavia visible no resolvio esta oclusion.

La siguiente revision conserva la prediccion durante la oclusion y registra
su edad y sigma. No se debe atribuir una mejora de trayectoria a codigo que
sigue en sombra ni comparar las dos colocaciones como si fueran iguales.

- `235835`: arranque en (63, 644) mm, aborto de nuevo y ninguna propuesta
  aplicada. Mediana 100 ms, p95 105 ms. Un conflicto entre el color de la
  caja y el objetivo actual seguia descartando la memoria. Se corrigio
  reservando primero el cluster del actual por su identidad congelada,
  antes de asociar las cajas restantes. Esta correccion requiere otra
  captura para contrastar su efecto; los resultados anteriores no la validan.

`dos_*.csv.jsonl` registra ahora las entradas completas del observador por
ciclo, incluidas las fases que no pasan por el arbitraje. Se reproduce sin
dispositivos con `python3 reproduccion_dos.py logs/dos_....csv.jsonl`.
La reproduccion permite comprobar identidad y prediccion; por si sola no
valida las maniobras que habrian cambiado la trayectoria fisica.

La correccion de dimensiones de 222 x 125 a 242 x 138 mm esta separada de
la preferencia de salida: afecta el arbitraje incluso con dos pilares
desactivado. Se basa en las medidas actuales confirmadas por el equipo.
Batalla, voladizo trasero y offsets de sensores conservan sus valores;
no se han vuelto a medir en esta sesion.

Pruebas sin dispositivos:

```bash
python3 -m unittest discover -s tests -v
```

Las sondas `sonda_dos_pilares.py` y `sonda_asociacion.py` observan sensores
con el coche parado y no importan el enlace de la Pico ni envian traccion.

Para detener una corrida se envia SIGINT al PID verificado del proceso.
Nunca usar SIGTERM ni `systemctl stop wro.service` con el unit actual.

## Captura reproducible desde la marca de 540 mm

`000806` completo el paso del verde y conservo al rojo hasta SALIDA_PILAR.
El video muestra el verde por el lado correcto y el rojo por el incorrecto.
Hubo 15 propuestas y ninguna aplicada. Mediana de ciclo 100 ms, p95 105 ms;
126 inferencias durante la ventana registrada, diez con ambos pilares.
La reproduccion contiene 161 ciclos. Siete de las quince propuestas cumplen
la nueva guardia de antiguedad y origen para controlar desde memoria.
Esto no reproduce el arbitraje completo ni demuestra una trayectoria mejor.
La prueba activa requiere video desde la misma marca y nueva validacion.
