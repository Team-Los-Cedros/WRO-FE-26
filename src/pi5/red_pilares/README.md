# Red de detección de pilares — YOLOv8n sobre Hailo-8

Esta carpeta contiene el modelo que reconoce los pilares de la Ronda Cerrada, ya convertido para el **Raspberry Pi AI HAT+ de 26 TOPS** que va montado sobre la Raspberry Pi 5.

**Explicado sin tecnicismos:** el robot tiene que saber si el bloque que tiene delante es rojo o verde, porque de eso depende por qué lado debe pasarlo. Antes lo decidía comparando colores contra unos valores fijos, y eso fallaba cuando cambiaba la luz del salón. Ahora lo decide un programa que aprendió a reconocerlos mirando mil fotos de la pista real, tomadas con la luz encendida, apagada y a media luz. Este es ese programa ya entrenado.

> La justificación completa —por qué se hizo, qué sustituye y qué no— está en la [sección 5.5 del README principal](../../../README.md#55-reconocimiento-de-pilares-por-red-neuronal-acelerador-hailo).

---

## Qué hay en esta carpeta

| Archivo | Qué es |
| :--- | :--- |
| [`pilares_v0.hef`](pilares_v0.hef) | **El artefacto que se despliega.** Modelo ya compilado para el Hailo-8. Es el único que el robot necesita en carrera. |
| [`pilares_v0.pt`](pilares_v0.pt) | Los pesos entrenados en PyTorch, antes de convertir. Sirven para reentrenar o para evaluar en una computadora normal. |
| [`nms_config.json`](nms_config.json) | Configuración del posprocesado: umbrales de detección y los tres decodificadores de caja. |
| [`modelo.alls`](modelo.alls) | La receta de cuantización del compilador de Hailo. |
| [`convertir_hailo8.py`](convertir_hailo8.py) | Script que hace la conversión de ONNX a HEF. |
| [`optimizar_modelo.py`](optimizar_modelo.py) | Paso de optimización previo a la compilación. |
| [`dataset.yaml`](dataset.yaml) | Definición de las clases del dataset. |

---

## Ficha del modelo

| Parámetro | Valor |
| :--- | :--- |
| Arquitectura | YOLOv8n |
| Clases | `0 = pilar_rojo`, `1 = pilar_verde` |
| Entrada | RGB, 640 × 640 |
| Preprocesado | Redimensionar **manteniendo proporciones** y rellenar con RGB (114,114,114), centrado |
| Normalización | **Ya incluida en el modelo.** Hay que entregar píxeles de 0 a 255 **sin** dividirlos antes |
| Posprocesado | NMS de YOLOv8 vía HailoRT, en CPU |
| Umbral de puntuación | 0,2 |
| Umbral de IoU | 0,7 |
| Máximo de detecciones | 100 por clase |
| Decodificadores de caja | 3, en *strides* 8 / 16 / 32, `regression_length` = 16 |
| Arquitectura objetivo | **`hailo8`** (26 TOPS) — *no* `hailo8l` |
| Tamaño del HEF | 4,2 MiB |

### Las tres trampas al integrarlo

Están puestas aquí porque las tres producen un fallo silencioso: el modelo carga, no da error, y detecta mal.

1. **No dividas la imagen entre 255.** La normalización está dentro del grafo. Si además la divides fuera, la entrada queda en el rango 0–1 y el modelo trabaja sobre una imagen prácticamente negra.
2. **Respeta el relleno al recuperar coordenadas.** La imagen entra redimensionada con bandas grises; para convertir una caja detectada a coordenadas de la imagen original hay que **deshacer el escalado y el relleno**, o los pilares aparecerán desplazados.
3. **Respeta el orden de las clases.** `0` es rojo y `1` es verde. Invertirlos hace que el robot pase cada pilar por el lado contrario, que es la falta que **termina el recorrido** según el reglamento.

---

## El dataset

| | Imágenes |
| :--- | :---: |
| Capturadas en pista (3 condiciones de luz) | 1034 |
| Etiquetadas en formato YOLO | **758** |
| └ `train` | 624 |
| └ `val` | 134 |

Instancias anotadas: **830** en total — 462 de `pilar_rojo` y 368 de `pilar_verde`.

> **Desequilibrio conocido:** hay un 25 % más de instancias rojas que verdes. No es intencionado y es el tipo de sesgo que puede inclinar al modelo hacia la clase mayoritaria cuando la imagen es ambigua. Conviene vigilarlo en la validación.

Las tres condiciones de luz (`aula_apagada`, `persiana_cerrada`, `aula_encendida`) están documentadas con su evidencia de captura en la sección 5.5 del README principal.

---

## Cómo se convirtió

La conversión corre en WSL con el entorno de Hailo (DFC 3.34.0):

```bash
python -u convertir_hailo8.py
```

Dos decisiones que se tomaron **después de que algo fallara**, y que conviene no deshacer:

* **Se reconstruyen las seis salidas de detección desde el ONNX**, evitando las operaciones finales que provocaban errores de asignación en el HAR anterior. El posprocesado no viaja dentro del grafo: se delega en la estructura NMS de Hailo Model Zoo adaptada a dos clases.
* **La calibración usa 256 imágenes reales de entrenamiento**, con selección reproducible, y **no** las imágenes ficticias de `calib_data`. Cuantizar con imágenes de relleno habría ajustado los rangos numéricos a una distribución de píxeles que el robot nunca ve.

---

## Estado de validación

| | Estado |
| :--- | :--- |
| El acelerador funciona en la Pi 5 | ✅ Banco, 14-09-2026: 310 FPS, 12,8 ms de *pipeline* |
| El modelo compila y produce un HEF válido | ✅ Convertido el 15-09-2026 para `hailo8` |
| El modelo detecta pilares con precisión suficiente en pista | ⬜ **Pendiente** |

**La conversión no demuestra la precisión de detección.** Falta ejecutar este HEF en la Raspberry con una versión compatible de HailoRT y evaluarlo contra las 134 imágenes de validación y contra la cámara en vivo.

Por eso **la ronda sigue corriendo hoy con la vía HSV**, que sí está validada en pista. Este modelo es una capacidad instalada y convertida, no todavía una mejora demostrada.
