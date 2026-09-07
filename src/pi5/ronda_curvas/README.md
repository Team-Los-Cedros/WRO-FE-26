# Ronda de curvas — copia de la Raspberry Pi

Esta carpeta conserva el código que estaba en la Raspi para la ronda con
cámara y curvas. Se ha guardado sin reescribir su lógica para poder recuperar
exactamente esa versión de trabajo.

## Contenido

- `ronda_camara.py`: punto de entrada; arranca cámara, LiDAR, Pico y la
  navegación al pulsar el botón GPIO 21.
- `navegacion.py`, `geometria_evasion.py`, `tracker.py` y `sentido_vuelta.py`:
  control de la maniobra de curvas y seguimiento de pilares.
- `camara_driver.py`, `lidar_driver.py` y `enlace_pico.py`: acceso a la cámara,
  RPLIDAR C1 y Pico 2.
- `test_pilar.py`, `test_sectores_trasera.py`, `superponer_lidar.py` y
  `ver_pilares.py`: herramientas de diagnóstico.

## Dependencias necesarias

La carpeta no es ejecutable por sí sola. El código de la Raspi importa módulos
que deben acompañarla en el mismo `PYTHONPATH`:

`vision`, `lidar_geometria`, `lidar_mascara`, `geometria_robot`, `optica` y
`registro_metricas`.

También necesita el hardware y librerías de Raspberry Pi: `RPi.GPIO`,
`picamera2`, `opencv-python`, `numpy` y `pyserial`. Los puertos configurados
en el código son `/dev/ttyUSB0` para el LiDAR y `/dev/ttyACM0` para la Pico.

Antes de usarlo para mover el robot, verifica los puertos, la máscara del
LiDAR y el montaje de cámara. Para el programa modular de Pi 5 consulta
`../ronda_nueva/`; esta carpeta es una copia documentada del programa de la
Raspi, no un reemplazo de aquel.
