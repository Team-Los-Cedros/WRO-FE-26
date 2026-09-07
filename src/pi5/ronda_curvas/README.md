# Ronda de curvas — copia de la Pi 5

Esta carpeta es una copia fiel del código fuente que vive en
`/home/pi/ronda_curvas` en la Raspberry Pi 5. Se conserva sin reescribir su
lógica para poder recuperar y revisar exactamente la versión que usa el robot.

## Contenido

- `ronda_camara.py`: punto de entrada; arranca cámara, LiDAR, Pico y la
  navegación al pulsar el botón GPIO 21.
- `navegacion.py`, `geometria_evasion.py`, `tracker.py` y `sentido_vuelta.py`:
  control de la maniobra de curvas y seguimiento de pilares.
- `camara_driver.py`, `lidar_driver.py`, `lidar_geometria.py`,
  `lidar_mascara.py` y `enlace_pico.py`: acceso a la cámara, RPLIDAR C1 y
  Pico 2.
- `vision.py`, `optica.py`, `geometria_robot.py`, `mapa_oclusion.py` y
  `mapa_pista.py`: percepción, calibración y geometría de apoyo.
- `calib_fov.py`, `calib_hsv.py`, `medir_fov.py`, `medir_modo_2304.py` y
  `sonda_fsm.py`: calibración y diagnóstico.
- `test_pilar.py`, `test_sectores_trasera.py`, `superponer_lidar.py` y
  `ver_pilares.py`, más `tests/`: herramientas y pruebas.

## Qué no se versiona

Se excluyen deliberadamente los artefactos generados por la Pi:

- `logs/*.csv`: telemetría de cada corrida.
- `__pycache__/` y `tests/__pycache__/`: bytecode creado por Python.

## Ejecución y dependencias

Desde esta carpeta, el punto de entrada es:

```bash
python3 ronda_camara.py
```

Debe ejecutarse en la Pi 5 con el hardware conectado. Requiere `RPi.GPIO`,
`picamera2`, `opencv-python`, `numpy` y `pyserial`. Los puertos configurados
en el código son `/dev/ttyUSB0` para el LiDAR y `/dev/ttyACM0` para la Pico.

Antes de usarlo para mover el robot, verifica los puertos, la máscara del
LiDAR y el montaje de cámara. Para el programa modular de Pi 5 consulta
`../ronda_nueva/`; esta carpeta es una copia documentada del programa de la
Pi 5, no un reemplazo de aquel.
