# Manual de Instalación — Team Los Cedros (WRO-FE 2026)

Esta guía documenta, paso a paso, cómo dejar una Raspberry Pi 3B y una Raspberry Pi Pico 2 nuevas en el mismo estado que las del robot de competencia, para que cualquier persona (juez, mentor, otro equipo) pueda reproducir el sistema completo desde cero.

## Índice

1. [Requisitos previos](#1-requisitos-previos)
2. [Raspberry Pi 3B — Sistema operativo y dependencias](#2-raspberry-pi-3b--sistema-operativo-y-dependencias)
3. [Raspberry Pi 3B — Cámara, LiDAR y permisos de puerto serie](#3-raspberry-pi-3b--cámara-lidar-y-permisos-de-puerto-serie)
4. [Raspberry Pi Pico 2 — Firmware MicroPython](#4-raspberry-pi-pico-2--firmware-micropython)
5. [Desplegar los scripts en la Raspberry Pi 3B](#5-desplegar-los-scripts-en-la-raspberry-pi-3b)
6. [Arranque autónomo con systemd](#6-arranque-autónomo-con-systemd)
7. [Verificación rápida](#7-verificación-rápida)
8. [Problemas comunes](#8-problemas-comunes)

---

## 1. Requisitos previos

* Raspberry Pi 3B con **Raspberry Pi OS Lite (64-bit)**, versión Bookworm o superior, ya flasheada en la microSD (usa [Raspberry Pi Imager](https://www.raspberrypi.com/software/) — en las opciones avanzadas habilita SSH y configura el usuario `pi` antes de grabar).
* Raspberry Pi Pico 2 (RP2350), sin flashear.
* Laptop en la misma red que la Pi 3B, con cliente SSH (o directamente teclado/monitor conectado a la Pi).
* Componentes del hardware ya ensamblados y cableados según la sección 4 del [README principal](README.md#4-arquitectura-eléctrica-y-distribución-de-señales) (RPLiDAR C1 por USB, Pi Camera Module 3 por CSI, Pico 2 por USB).

---

## 2. Raspberry Pi 3B — Sistema operativo y dependencias

Conéctate por SSH (`ssh pi@<ip-de-la-pi>`) y actualiza el sistema:

```bash
sudo apt update && sudo apt full-upgrade -y
```

### 2.1 Dependencias de sistema (vía `apt`)

`picamera2` **no se instala con `pip`** — depende de las librerías nativas `libcamera` del sistema operativo, y la versión de PyPI suele desincronizarse con ellas. En Raspberry Pi OS Bookworm ya viene preinstalada casi siempre, pero para dejarlo explícito:

```bash
sudo apt install -y python3-picamera2 python3-opencv python3-rpi.gpio python3-pip python3-venv --no-install-recommends
```

### 2.2 Dependencias de Python (vía `pip`)

Raspberry Pi OS Bookworm bloquea `pip install` directo al entorno del sistema (PEP 668, error `externally-managed-environment`). Hay dos formas válidas de instalar el resto de dependencias — elige una:

**Opción A — entorno virtual con acceso a los paquetes de sistema (recomendada):**
```bash
python3 -m venv --system-site-packages ~/venv-wro
source ~/venv-wro/bin/activate
pip install -r requirements.txt
```
El flag `--system-site-packages` es clave: permite que el venv vea `picamera2` y `RPi.GPIO` instalados por `apt` en el paso 2.1, sin tener que reinstalarlos por pip (picamera2 por pip no funciona bien de forma aislada).

> Si usas el venv, recuerda activarlo (`source ~/venv-wro/bin/activate`) antes de correr cualquier script, y actualizar `ExecStart` en `wro_start.service` para apuntar al Python del venv (`~/venv-wro/bin/python3`) en vez de `/usr/bin/python3`.

**Opción B — instalar directo al sistema (más simple, menos aislado):**
```bash
pip install --break-system-packages -r requirements.txt
```

El archivo [`src/pi3B/requirements.txt`](src/pi3B/requirements.txt) instala `opencv-python`, `numpy`, `pyserial` y `RPi.GPIO`.

---

## 3. Raspberry Pi 3B — Cámara, LiDAR y permisos de puerto serie

### 3.1 Habilitar la interfaz de cámara

```bash
sudo raspi-config
```
`Interface Options` → `Camera` → habilitar → reiniciar (`sudo reboot`).

Verifica que la Pi Camera Module 3 responda:
```bash
libcamera-hello --list-cameras
```

### 3.2 Permisos de puerto serie (RPLiDAR C1 y Pico 2)

El usuario `pi` necesita pertenecer al grupo `dialout` para leer/escribir `/dev/ttyUSB0` (LiDAR) y `/dev/ttyACM0` (Pico 2) sin `sudo`:

```bash
sudo usermod -aG dialout pi
```
Cierra sesión y vuelve a entrar (o reinicia) para que el cambio de grupo tome efecto. Verifica los dispositivos conectados con:
```bash
ls -l /dev/ttyUSB0 /dev/ttyACM0
```

---

## 4. Raspberry Pi Pico 2 — Firmware MicroPython

1. Descarga el firmware MicroPython para **Pico 2 (RP2350)** desde [micropython.org/download/RPI_PICO2](https://micropython.org/download/RPI_PICO2/) (archivo `.uf2`). No uses el firmware del Pico original (RP2040) — el chip es distinto.
2. Mantén presionado el botón **BOOTSEL** del Pico 2 mientras lo conectas por USB a tu laptop. Aparecerá como una unidad de almacenamiento externa (`RP2350`).
3. Arrastra el archivo `.uf2` a esa unidad. El Pico se reinicia solo y queda con MicroPython instalado.
4. Instala [Thonny IDE](https://thonny.org/) en tu laptop (el editor más simple para subir archivos a un microcontrolador; también sirven `mpremote` o `rshell` si prefieres línea de comandos).
5. En Thonny, selecciona el intérprete `MicroPython (Raspberry Pi Pico)` en la esquina inferior derecha, y copia al Pico (usando `Archivo > Guardar como > Raspberry Pi Pico`) estos dos archivos:
   - [`src/pico/main.py`](src/pico/main.py)
   - [`src/pico/Mpu6050.py`](src/pico/Mpu6050.py)

No hace falta instalar ningún paquete adicional en el Pico — `main.py` solo usa módulos incluidos en MicroPython (`machine`, `time`, `sys`, `select`).

---

## 5. Desplegar los scripts en la Raspberry Pi 3B

En el repositorio, `src/pi3B/` está organizado en subcarpetas por responsabilidad (`ronda_cerrada/`, `ronda_abierta/`, `calibracion/`) para que sea más fácil de navegar. **Esa organización es solo del repositorio.** `controlador_inicio.py` referencia los scripts de carrera directamente en `/home/pi/` sin subcarpetas, y los módulos de la Ronda Cerrada se importan entre sí por nombre de archivo (`import vision`, `from lidar import LidarC1`, etc.), así que **todos los `.py` deben quedar juntos y sin subcarpetas** en `/home/pi/` al copiarlos, manteniendo la capitalización exacta:

```bash
scp src/pi3B/controlador_inicio.py \
    src/pi3B/ronda_abierta/Open_round.py \
    src/pi3B/ronda_cerrada/Close2_round.py \
    src/pi3B/ronda_cerrada/navegacion.py \
    src/pi3B/ronda_cerrada/vision.py \
    src/pi3B/ronda_cerrada/lidar.py \
    src/pi3B/ronda_cerrada/tracker.py \
    src/pi3B/ronda_cerrada/enlace_pico.py \
    src/pi3B/calibracion/calibrar_hsv.py \
    pi@<ip-de-la-pi>:/home/pi/
```

> No copies nada de `src/pi3B/ronda_cerrada/legacy/` — son versiones superadas de la Ronda Cerrada, archivadas solo como referencia histórica (ver el `README.md` de esa carpeta).

---

## 6. Arranque autónomo con systemd

Copia el archivo de unidad real ya incluido en el repositorio y actívalo:

```bash
sudo cp src/pi3B/wro_start.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable wro_start.service
sudo systemctl start wro_start.service
```

Más detalle de este servicio y comandos de diagnóstico en [`src/pi3B/README.md`](src/pi3B/README.md).

---

## 7. Verificación rápida

Con el servicio detenido (`sudo systemctl stop wro_start.service`), corre manualmente para ver la salida en vivo:

```bash
python3 -u /home/pi/Open_round.py
```

Deberías ver `[+] Telemetria LiDAR activa.` y `SISTEMA LISTO...`. Presiona el botón físico (GPIO 21) y confirma que el robot centra la dirección y avanza guiado por las paredes.

---

## 8. Problemas comunes

| Síntoma | Causa probable | Solución |
| :--- | :--- | :--- |
| `ModuleNotFoundError: No module named 'picamera2'` | Se instaló por `pip` en vez de `apt`, o el venv no tiene `--system-site-packages` | `sudo apt install -y python3-picamera2` y recrear el venv con `--system-site-packages` |
| `error: externally-managed-environment` al hacer `pip install` | Protección PEP 668 de Raspberry Pi OS Bookworm | Usa un venv (sección 2.2, Opción A) o `--break-system-packages` (Opción B) |
| `PermissionError` al abrir `/dev/ttyUSB0` o `/dev/ttyACM0` | Usuario `pi` no está en el grupo `dialout` | `sudo usermod -aG dialout pi` y reiniciar sesión |
| El servicio `wro_start.service` no encuentra `Open_round.py`/`Close2_round.py` | Los archivos no están en `/home/pi/`, les falta algún módulo de soporte, o la capitalización no coincide exactamente | Ver sección 5 — `controlador_inicio.py` es sensible a mayúsculas (Linux) y `Close2_round.py` necesita sus 4 módulos en la misma carpeta |
| `lgpio.error: 'unknown handle'` al detener el script | Doble Ctrl+C mientras `apagar_sistema()` seguía en curso | Ya corregido en el código (guardia de reentrada); si persiste, usa un solo Ctrl+C y espera |
