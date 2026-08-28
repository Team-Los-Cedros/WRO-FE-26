# Video de Desempeño en Pista

## Ronda Abierta

**[https://youtu.be/69h0BPew7_Y](https://youtu.be/69h0BPew7_Y)** (No listado)

Vuelta autónoma completa del vehículo guiada por RPLiDAR C1 con control proporcional y corrección inercial por IMU (MPU6050).

## Ronda Cerrada

**Estado: pendiente de grabación final.** Se está validando en pista la corrección aplicada a la lógica de evasión de obstáculos (ver historial de commits de `src/pi3B/ronda_cerrada/ronda_cerrada.py`) antes de grabar la corrida oficial.

En `video/video-drafts/` se encuentran las grabaciones preliminares de las pruebas de desarrollo (`P1`–`P5`, aperturas y cierres de cada iteración), usadas internamente por el equipo para depurar el algoritmo de evasión antes de la grabación oficial.

### Evidencia de la sesión de reactivación de la Ronda Cerrada modular (2026-08-27)

Grabaciones con cámara cenital externa a la pista (no la de a bordo), correlacionadas con la telemetría de `registro_metricas.py` en el caso de estudio de la sección 8.3 del [README](../README.md#83-caso-de-estudio-reactivación-de-la-ronda-cerrada-modular-con-evidencia-cuantitativa-2026-08-27). Recortadas a la ventana de acción (sin el tiempo muerto de espera del botón), mismo montaje en las tres: un pilar rojo, robot en posición de arranque.

| Archivo | Corrida | Resultado |
| :--- | :---: | :--- |
| [`2026-08-27_S3_corrida6.mp4`](video-drafts/2026-08-27_S3_corrida6.mp4) | 6 | Primera corrida sin emergencias, tras el refactor de trayectoria por posición (hallazgo #7 de la sección 8.3). |
| [`2026-08-27_S3_corrida7.mp4`](video-drafts/2026-08-27_S3_corrida7.mp4) | 7 | Con el modelo de velocidad del tracker corregido (hallazgo #9); destapó el hallazgo #10 (excursión hacia la pared en `SOBREPASO`). |
| [`2026-08-27_S3_corrida8.mp4`](video-drafts/2026-08-27_S3_corrida8.mp4) | 8 | Con `DIST_SOBREPASO_MM` acortado (hallazgo #10); mejor corrida de la sesión en todas las métricas registradas. |
