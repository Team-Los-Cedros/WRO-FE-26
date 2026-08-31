# Qué se rescató de los finalistas internacionales 2025

La implementación de `ronda_nueva` es propia. Este documento registra qué
principios se estudiaron, qué se adaptó a la Pi 3B y qué se descartó para no
confundir una idea útil con parámetros calibrados para otro robot.

## 1.er lugar — WRO-FE-XX

Fuente analizada: commit
[`97a78ea`](https://github.com/ejm22/WRO-FE-XX/tree/97a78eaf08d9bc9fcd3d6ad619eafae4350fa32a).
El repositorio identifica `run.py` como el programa de la Final Internacional
([README, líneas 1–4](https://github.com/ejm22/WRO-FE-XX/blob/97a78eaf08d9bc9fcd3d6ad619eafae4350fa32a/README.md#L1-L4)).

Ideas adoptadas:

- Cámara alta, cerca de la mitad posterior, mirando hacia delante y hacia el
  piso. El código trabaja sobre la parte inferior del frame, donde están el
  suelo y los pilares
  ([camera_manager.py, líneas 65–71 y 121–145](https://github.com/ejm22/WRO-FE-XX/blob/97a78eaf08d9bc9fcd3d6ad619eafae4350fa32a/code/XX_2025_package/classes/camera_manager.py#L65-L71)).
- Extraer la superficie transitable como componente conectado y comprobar que
  exista suelo debajo del blob de color
  ([image_drawing_utils.py, líneas 27–106](https://github.com/ejm22/WRO-FE-XX/blob/97a78eaf08d9bc9fcd3d6ad619eafae4350fa32a/code/XX_2025_package/utils/image_drawing_utils.py#L27-L106)).
- HSV y contornos convencionales, sin una red neuronal; una FSM arbitra pared,
  pilar y seguridad
  ([image_algoriths.py, líneas 313–398](https://github.com/ejm22/WRO-FE-XX/blob/97a78eaf08d9bc9fcd3d6ad619eafae4350fa32a/code/XX_2025_package/classes/image_algoriths.py#L313-L398)).
- Contar eventos de líneas como secuencia y con período refractario, no contar
  frames coloreados
  ([lap_tracker.py, líneas 20–106](https://github.com/ejm22/WRO-FE-XX/blob/97a78eaf08d9bc9fcd3d6ad619eafae4350fa32a/code/XX_2025_package/classes/lap_tracker.py#L20-L106)).

No se copiaron sus coordenadas de píxel, HSV, tiempos, pasos ni ángulos. Su S
final de estacionamiento termina con una secuencia mayormente abierta por pasos
([run.py, líneas 264–296](https://github.com/ejm22/WRO-FE-XX/blob/97a78eaf08d9bc9fcd3d6ad619eafae4350fa32a/code/XX_2025_package/run.py#L264-L296));
sirve como estructura de estados, no como calibración transferible.

## 2.º lugar — KMIDS-GFM-Future-Engineer-2025

Fuente analizada: commit
[`aebc4ca`](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/tree/aebc4ca5bc683049d36abba8b2fe4ec45434b665).

Ideas adoptadas:

- Separar adquisición, procesadores y FSM; cámara, LiDAR y microcontrolador no
  deben bloquearse mutuamente
  ([arquitectura](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/blob/aebc4ca5bc683049d36abba8b2fe4ec45434b665/code/raspberry-pi-5/src/README.md#L9-L19)).
- Representar paredes como rectas y controlar distancia más rumbo, sin SLAM
  ([lidar_processor.cpp, líneas 290–401](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/blob/aebc4ca5bc683049d36abba8b2fe4ec45434b665/code/raspberry-pi-5/src/processors/lidar/lidar_processor.cpp#L290-L401)).
- Asociar cada blob visual con una posición LiDAR usando el rayo de cámara y la
  extrínseca entre sensores
  ([combined_processor.cpp, líneas 99–183](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/blob/aebc4ca5bc683049d36abba8b2fe4ec45434b665/code/raspberry-pi-5/src/processors/combined/combined_processor.cpp#L99-L183)).
- Recorrido determinista `NORMAL → PRE_TURN → TURNING`, rumbos cardinales y
  conteo de 12 esquinas; es mucho más barato que construir un mapa métrico
  completo
  ([obstacle_challenge/main.cpp, líneas 582–604](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/blob/aebc4ca5bc683049d36abba8b2fe4ec45434b665/code/raspberry-pi-5/apps/challenges/obstacle_challenge/main.cpp#L582-L604)).
- Estacionamiento en etapas y control de movimiento delegado al
  microcontrolador.

Se corrigieron dos debilidades antes de adaptar esas ideas:

- La aplicación toma simplemente el último dato de cada sensor, aunque existe
  una función de sincronización
  ([main.cpp, líneas 408–496](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/blob/aebc4ca5bc683049d36abba8b2fe4ec45434b665/code/raspberry-pi-5/apps/challenges/obstacle_challenge/main.cpp#L408-L496)).
  `ronda_nueva` conserva pocos timestamps y rechaza pares demasiado viejos.
- Su localización geométrica del cajón está comentada y la secuencia activa usa
  umbrales frontales y encoder
  ([main.cpp, líneas 1136–1341](https://github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025/blob/aebc4ca5bc683049d36abba8b2fe4ec45434b665/code/raspberry-pi-5/apps/challenges/obstacle_challenge/main.cpp#L1136-L1341)).
  La versión nueva exige detectar dos separadores LiDAR y verifica el resultado.

## Decisiones específicas para la Raspberry Pi 3B

| Problema | Decisión de `ronda_nueva` |
| --- | --- |
| CPU/RAM limitadas | 640×360 a 15 FPS desde el modo raw 2304×1296, ROI, máximo cuatro blobs por color y sin IA. El pipeline HSV medido en la Pi 3B conserva margen holgado. |
| Backlog de imágenes | Se guardan resultados de 2–4 frames, nunca una cola de imágenes. |
| Varios pilares visibles | Asignación angular voraz uno-a-uno y un voto por objeto/timestamp. |
| Vibración/exposición | Mástil rígido; tras calentamiento se intenta fijar AE/AWB. |
| Mínimos LiDAR ruidosos | Recta robusta y mediana; el mínimo queda solo para emergencia. |
| Cámara trasladada atrás/arriba | Rotación, máscara, ROI, intrínseca, latencia y extrínseca configurables. |
| Fin de tres vueltas | Esquinas terminadas por IMU + reapertura frontal; no `abs(yaw)>=1010`. |
| Parqueo | Detección pared–hueco–pared, alineación, dos arcos, centrado y verificación. |
| Pruebas sin robot | Lógica sin I/O, `unittest` y replay de captura timestampada. |
