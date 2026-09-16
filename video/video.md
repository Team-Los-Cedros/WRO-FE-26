# Video de Desempeño en Pista

## Ronda Abierta

**[https://youtu.be/zwYa40_EVPY](https://youtu.be/zwYa40_EVPY)** (no listado)

Grabación del 06-09-2026, cámara cenital, 1080p, 75 s. El mismo archivo está en el repositorio por si el enlace no estuviera disponible: [`ronda_abierta_20260906.mp4`](ronda_abierta_20260906.mp4).

Recorrido autónomo del vehículo sobre la pista sin obstáculos. El operador coloca el robot, pulsa el botón de `GPIO 21` y se retira del área; a partir de ahí no hay ninguna intervención. El trazado se mantiene por centrado proporcional entre paredes con el RPLiDAR C1 y corrección inercial con la IMU (MPU6050) integrada en la Pico 2.

Esta corrida es la primera grabada con el vehículo ya migrado a la Raspberry Pi 5. La grabación anterior, con la Pi 3B, se conserva como referencia de la evolución del proyecto: [https://youtu.be/69h0BPew7_Y](https://youtu.be/69h0BPew7_Y) (no listado).

## Ronda Cerrada

**Estado: en proceso.** La grabación oficial está pendiente.

El vehículo ya completa la secuencia entera de la ronda —salir del estacionamiento, dar las tres vueltas esquivando los bloques por el lado que marca su color, y volver al cuadrante de salida— pero todavía no en una sola corrida limpia. Lo que falta está medido, no supuesto. De las cinco corridas registradas el 11-09-2026:

| Corrida | Vueltas completadas | Pasos por el lado prohibido | Qué falló |
| :--- | :---: | :---: | :--- |
| 12:47 | 3 | 1 (primer pilar rojo, a los 7,2 s) | El código de carrera arrancaba con el robot aún dentro del estacionamiento |
| 13:13 | 3 | 1 (cuarto pilar rojo, a los 78 s) | Salida del estacionamiento ya correcta; falla un pilar a mitad de recorrido |
| 13:23 | 1,5 | **0** | La maniobra de salida se bloqueó contra un eco de la propia rueda |
| 13:44 | 2,4 | **0** | Misma causa |

El criterio de la tabla no es el registro interno del robot sino un verificador independiente (`marcador_real.py`), que cuenta un pilar como superado solo cuando el eje trasero cruza su posición con separación positiva del lado obligatorio. Pasar un bloque por el lado equivocado termina el recorrido según el reglamento, así que esa columna manda sobre cualquier otra métrica.

Las dos corridas que completan las tres vueltas fallan un pilar; las dos que no fallan ninguno no completan las vueltas. Ambas causas están identificadas y corregidas en el código, pendientes de validación en pista:

1. **La maniobra de salida se bloqueaba contra el propio vehículo.** Los cortes se producían contra ecos planos de 49-66 mm en los sectores donde asoma la rueda delantera girada a tope; el radio de esos ecos cambiaba entre corridas porque cambiaba el ángulo de la rueda, que es lo que descarta que fueran un muro. Ahora se descartan los retornos que caen dentro de la propia silueta.
2. **El conteo de vueltas por líneas de pista contaba de más.** Se verificó contra la guiñada acumulada: 871° de giro (2,4 vueltas) se estaban contando como tres. Se añadió un mínimo de giro entre cruces, porque dos líneas del mismo color están separadas por una esquina.

Mientras tanto, en [`video-drafts/`](video-drafts/) están las grabaciones de desarrollo usadas para depurar la lógica de evasión.

### Evidencia de la sesión de reactivación de la Ronda Cerrada modular (2026-08-27)

Grabaciones con cámara cenital externa a la pista (no la de a bordo), correlacionadas con la telemetría de [`registro_metricas.py`](../src/pi5/ronda_curvas/registro_metricas.py) en el caso de estudio de la sección 8.3 del [README](../README.md#83-caso-de-estudio-reactivación-de-la-ronda-cerrada-modular-con-evidencia-cuantitativa-2026-08-27). Recortadas a la ventana de acción (sin el tiempo muerto de espera del botón), mismo montaje en las tres: un pilar rojo, robot en posición de arranque.

| Archivo | Corrida | Resultado |
| :--- | :---: | :--- |
| [`2026-08-27_S3_corrida6.mp4`](video-drafts/2026-08-27_S3_corrida6.mp4) | 6 | Primera corrida sin emergencias, tras el refactor de trayectoria por posición (hallazgo #7 de la sección 8.3). |
| [`2026-08-27_S3_corrida7.mp4`](video-drafts/2026-08-27_S3_corrida7.mp4) | 7 | Con el modelo de velocidad del tracker corregido (hallazgo #9); destapó el hallazgo #10 (excursión hacia la pared en `SOBREPASO`). |
| [`2026-08-27_S3_corrida8.mp4`](video-drafts/2026-08-27_S3_corrida8.mp4) | 8 | Con `DIST_SOBREPASO_MM` acortado (hallazgo #10); mejor corrida de la sesión en todas las métricas registradas. |
