# Manual de Instalación — Team Los Cedros (WRO-FE 2026)

Esta guía documenta, paso a paso, cómo dejar una **Raspberry Pi 5** y una **Raspberry Pi Pico 2** nuevas en el mismo estado que las del robot de competencia, para que cualquier persona (juez, mentor, otro equipo) pueda reproducir el sistema completo desde cero.

> **El robot corre en una Raspberry Pi 5 desde el 03-09-2026.** Antes usaba una Pi 3B. La migración no fue una preferencia sino una decisión medida: el mismo pipeline de visión pasó de 67-72 ms a **4,8 ms** por cuadro a 640x360, y la edad del barrido LiDAR bajó de 16,0 ms de media a **0,1 ms** (ver sección 4.2 del [README principal](README.md#42-catálogo-de-componentes-y-justificación-de-selección)). Esta guía es para la Pi 5. Los pasos de la Pi 3B están en el historial de git si alguien necesita reconstruir aquella.

## Índice

1. [Requisitos previos](#1-requisitos-previos)
2. [Raspberry Pi 5 — Sistema operativo y dependencias](#2-raspberry-pi-5--sistema-operativo-y-dependencias)
3. [Raspberry Pi 5 — Cámara, LiDAR, I²C y permisos](#3-raspberry-pi-5--cámara-lidar-ic-y-permisos)
4. [Raspberry Pi Pico 2 — Firmware MicroPython](#4-raspberry-pi-pico-2--firmware-micropython)
5. [Desplegar los scripts en la Raspberry Pi 5](#5-desplegar-los-scripts-en-la-raspberry-pi-5)
6. [Arranque autónomo con systemd](#6-arranque-autónomo-con-systemd)
7. [Verificación rápida](#7-verificación-rápida)
8. [Cambiar de versión entre rondas](#8-cambiar-de-versión-entre-rondas)
9. [Problemas comunes](#9-problemas-comunes)

---

## 1. Requisitos previos

* **Raspberry Pi 5** con **Raspberry Pi OS (64-bit)**, Bookworm o superior, ya flasheada en la microSD. Usa [Raspberry Pi Imager](https://www.raspberrypi.com/software/) y, en las opciones avanzadas, habilita SSH y configura el usuario `pi` **antes** de grabar.
* **Raspberry Pi Pico 2** (RP2350), sin flashear.
* Laptop en la misma red que la Pi 5, con cliente SSH.
* Hardware ensamblado y cableado según la sección 4 del [README principal](README.md#4-arquitectura-eléctrica-y-distribución-de-señales): RPLiDAR C1 por USB, Pi Camera Module 3 por CSI, Pico 2 por USB, botón de arranque en GPIO 21.

> **Si tienes las dos Raspberry encendidas, cuidado:** ambas responden al hostname `wro-fe`. Distínguelas por IP o por kernel — `uname -r` termina en `-2712` en la Pi 5 y en `-v8` en la Pi 3B. No te confíes del nombre que muestra la terminal al conectarte: es idéntico en las dos placas. Se perdió media jornada de trabajo el 03-09 por estar probando contra la equivocada sin saberlo.

---

## 2. Raspberry Pi 5 — Sistema operativo y dependencias

Conéctate por SSH (`ssh pi@<ip-de-la-pi>`) y actualiza el sistema:

```bash
sudo apt update && sudo apt full-upgrade -y
```

### 2.1 Dependencias de sistema (vía `apt`)

`picamera2` **no se instala con `pip`** — depende de las librerías nativas `libcamera` del sistema operativo, y la versión de PyPI se desincroniza con ellas. En Raspberry Pi OS Bookworm suele venir preinstalada, pero para dejarlo explícito:

```bash
sudo apt install -y python3-picamera2 python3-opencv python3-rpi-lgpio python3-pip python3-venv git --no-install-recommends
```

### 2.2 El paquete `RPi.GPIO` rompe el GPIO en la Pi 5

Este es el tropiezo que más tiempo costó de toda la migración, y es específico de la Pi 5.

**`RPi.GPIO` 0.7.1 (el paquete clásico de PyPI) no funciona en la Pi 5.** Falla con:

```
RuntimeError: Cannot determine SOC peripheral base address
```

El motivo es que la Pi 5 cambió el controlador de GPIO (chip RP1) y la librería vieja escribe directo a los registros del SoC. En la Pi 3B sí funciona, así que **si migras copiando `~/.local/` de una Pi 3B te llevas el problema contigo**.

La solución es el shim `python3-rpi-lgpio`, que ofrece la misma API `RPi.GPIO` por encima de `lgpio`. Ya se instaló en el paso 2.1; lo que hay que asegurar es que **no haya un `RPi.GPIO` de pip ganándole**:

```bash
pip uninstall -y RPi.GPIO        # si estaba instalado por pip, quitarlo
python3 -c "import RPi.GPIO as G; G.setmode(G.BCM); print('GPIO OK')"
```

> **Esto se rompe solo cada cierto tiempo.** Instalar `Adafruit-Blinka` con pip arrastra `RPi.GPIO` como dependencia y vuelve a romper el GPIO. Si el botón de arranque deja de responder después de instalar cualquier paquete, lo primero que hay que comprobar es esto. Blinka (`board`, `busio`) importa perfectamente sin `RPi.GPIO`.

### 2.3 Dependencias de Python (vía `pip`)

Raspberry Pi OS Bookworm bloquea `pip install` directo al entorno del sistema (PEP 668, error `externally-managed-environment`). Hay dos formas válidas — elige una:

**Opción A — entorno virtual con acceso a los paquetes de sistema (recomendada):**
```bash
python3 -m venv --system-site-packages ~/venv-wro
source ~/venv-wro/bin/activate
pip install -r src/pi5/requirements.txt
```

El flag `--system-site-packages` es clave: permite que el venv vea `picamera2` y el shim de GPIO instalados por `apt`, sin reinstalarlos por pip.

> Si usas el venv, actívalo antes de correr cualquier script y apunta `ExecStart` del servicio (sección 6) al Python del venv (`/home/pi/venv-wro/bin/python3`) en vez de `/usr/bin/python3`.

**Opción B — instalar directo al sistema (más simple, menos aislado):**
```bash
pip install --break-system-packages -r src/pi5/requirements.txt
```

[`src/pi5/requirements.txt`](src/pi5/requirements.txt) instala `opencv-python`, `numpy`, `pyserial` y `RPi.GPIO` (que en la Pi 5 lo resuelve el shim del paso 2.2).

---

## 3. Raspberry Pi 5 — Cámara, LiDAR, I²C y permisos

### 3.1 Cámara

En Raspberry Pi OS Bookworm la cámara ya no se habilita por `raspi-config`: `libcamera` la detecta sola. Verifica que responda:

```bash
libcamera-hello --list-cameras
```

Debe aparecer un `imx708` (Camera Module 3). Si no aparece, revisa que el cable plano esté bien insertado y con el contacto hacia el lado correcto.

### 3.2 I²C para la IMU

El MPU6050 cuelga de la Pico 2, no de la Pi, pero el bus I²C de la Pi se habilita igualmente para herramientas de diagnóstico. Añade a `/boot/firmware/config.txt`:

```
dtparam=i2c_arm=on
```

**Hace falta reiniciar** para que aparezca `/dev/i2c-1`. Guarda una copia del archivo antes de tocarlo.

### 3.3 Permisos de puerto serie

El usuario `pi` necesita pertenecer al grupo `dialout` para leer y escribir `/dev/ttyUSB0` (LiDAR) y `/dev/ttyACM0` (Pico 2) sin `sudo`:

```bash
sudo usermod -aG dialout pi
```

Cierra sesión y vuelve a entrar (o reinicia) para que el cambio tome efecto. Comprueba los tres dispositivos:

```bash
ls -l /dev/ttyUSB0 /dev/ttyACM0 /dev/video0
```

---

## 4. Raspberry Pi Pico 2 — Firmware MicroPython

### 4.1 Instalar MicroPython (una sola vez)

1. Descarga el firmware para **Pico 2 (RP2350)** desde [micropython.org/download/RPI_PICO2](https://micropython.org/download/RPI_PICO2/) (archivo `.uf2`). **No uses el del Pico original (RP2040)** — el chip es distinto.
2. Mantén presionado **BOOTSEL** mientras conectas el Pico 2 por USB. Aparecerá como una unidad externa (`RP2350`).
3. Arrastra el `.uf2` a esa unidad. El Pico se reinicia solo con MicroPython instalado.

### 4.2 Subir el firmware del robot, desde la propia Raspberry

No hace falta Thonny ni desconectar el Pico del robot. La Pi ya tiene el Pico en `/dev/ttyACM0` y trae `mpremote`, así que puede flashearlo ella misma por SSH:

```bash
bash src/pico/deploy_pico.sh --dry-run     # valida sin escribir nada
bash src/pico/deploy_pico.sh               # respalda, copia y reinicia
```

El script hace tres cosas que a mano se olvidan:

1. **Respalda lo que hay en el Pico antes de sobrescribirlo.** El firmware vivo se ha desincronizado del repo más de una vez.
2. **Comprueba la sintaxis de cada archivo antes de subirlo.** Un error de sintaxis en `main.py` deja el robot inerte y solo se descubre al reiniciar.
3. **Saca el Pico del raw REPL al terminar** y verifica que vuelve la telemetría (ver 4.3).

Los archivos que suben son [`src/pico/main.py`](src/pico/main.py), [`src/pico/protocolo_seguro.py`](src/pico/protocolo_seguro.py) y [`src/pico/Mpu6050.py`](src/pico/Mpu6050.py). No hace falta instalar paquetes adicionales: `main.py` solo usa módulos incluidos en MicroPython (`machine`, `time`, `sys`, `select`).

### 4.3 El Pico puede quedar «vivo y mudo»

Después de encender el robot, el Pico puede arrancar sin emitir una sola línea de telemetría, **sin que nadie haya tocado `mpremote`**. Sin telemetría la ronda no arma la tracción, así que el robot no se mueve y no dice por qué.

Pasa porque cualquier comando de `mpremote` interrumpe `main.py` y deja la placa en raw REPL, donde MicroPython **no** relanza `main.py`: queda viva, muda y recibiendo consignas que nadie ejecuta. Se arregla en segundos y sin reflashear:

```bash
bash src/pico/deploy_pico.sh --reiniciar
```

> **`WD:STOP` con el robot parado es lo normal, no un fallo.** Nadie le está mandando consignas y su watchdog está en la posición segura. Pasa a `OK` al arrancar la ronda.

---

## 5. Desplegar los scripts en la Raspberry Pi 5

Los módulos se importan entre sí por nombre suelto (`import vision`, `from lidar_driver import LidarDriver`), así que **todos los `.py` de una ronda tienen que quedar juntos en una misma carpeta**, respetando la capitalización exacta (Linux distingue mayúsculas).

La carpeta de trabajo del robot es `/home/pi/ronda_curvas/`.

### Opción A — clonar el repo en la Pi (recomendada para iterar)

Requiere salida a internet en la Pi. Se clona una sola vez:

```bash
ssh pi@<ip-de-la-pi>
git clone https://github.com/Team-Los-Cedros/WRO-FE-26.git
cd WRO-FE-26
cp src/pi5/ronda_curvas/*.py  ~/ronda_curvas/
cp src/pi5/correr_completa.sh ~/
chmod +x ~/correr_completa.sh
```

Para actualizar después de un cambio: `cd ~/WRO-FE-26 && git pull` y repetir los `cp`.

### Opción B — `scp` directo desde la laptop (sin internet en la Pi)

Útil el día de la competencia si el lugar no tiene wifi confiable: solo necesita que la laptop y la Pi estén en la misma red local.

```bash
scp src/pi5/ronda_curvas/*.py pi@<ip-de-la-pi>:/home/pi/ronda_curvas/
scp src/pi5/correr_completa.sh pi@<ip-de-la-pi>:/home/pi/
```

> **Los `.sh` tienen que ir con finales de línea Unix (LF), no Windows (CRLF).** Si se editan en Windows y se copian con CRLF, bash recibe los argumentos con un `\r` pegado y el script falla en silencio —sin mensaje de error— porque `parqueo.py salida` se convierte en `parqueo.py salida\r`. Comprobar con `file ~/correr_completa.sh`: tiene que decir «ASCII text executable», no «with CRLF line terminators».

---

## 6. Arranque autónomo con systemd

El servicio arranca la secuencia completa al encender la Pi: espera el botón, sale del estacionamiento y corre la ronda.

```bash
sudo cp src/pi5/wro.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable wro.service
sudo systemctl start wro.service
```

Comandos de diagnóstico:

```bash
sudo systemctl status wro.service          # estado
journalctl -u wro.service -f               # diario en vivo
sudo systemctl stop wro.service            # pararlo
```

El diario también queda en `~/ronda_curvas/logs/servicio.log`.

Dos detalles de la unidad que no son cosméticos:

* **No lleva `Restart=`.** Una versión anterior tenía `Restart=always`, que relanzaba la ronda entera en cuanto terminaba: el robot volvía a salir del estacionamiento solo, una y otra vez. Una carrera se lanza una vez.
* **`TimeoutStartSec=0`.** El servicio se pasa la mayor parte del tiempo esperando el botón, y el límite de 90 s que systemd pone por defecto a los `oneshot` lo mataría antes de que nadie lo pulsara.

> **Si vas a lanzar a mano por SSH, para el servicio antes** (`sudo systemctl stop wro.service`). Si no, se pelean por el GPIO del botón y el lanzamiento manual muere con `GPIO ocupado`.

---

## 7. Verificación rápida

Con el servicio detenido, corre la secuencia a mano para ver la salida en vivo:

```bash
bash ~/correr_completa.sh
```

Lo que debes ver, en orden:

1. `[LISTO] LED PARPADEANDO = cargado y listo.` — **el LED del Pico parpadea.** Esa es la señal de que todo está cargado y esperando.
2. Al pulsar el botón (GPIO 21): `[START] boton pulsado.` y el LED se apaga.
3. `=== LO QUE VE AHORA ===` con las distancias de los cuatro costados. Si el robot está mal colocado en la plaza, aquí aborta y **te dice cuántos milímetros moverlo y hacia dónde**.
4. Los tiempos del vaivén (`[TIEMPO 1]`, `[TIEMPO 2]`…) y luego `[LISTO] en el carril y paralelo a las paredes.`
5. La carrera, con `[VUELTA] linea NARANJA #n de 12` en cada esquina.

Para probar sin botón (solo banco, nunca en competencia — el reglamento pide una acción física sobre el robot):

```bash
WRO_ARRANQUE_AUTO=1 bash ~/correr_completa.sh
```

> **Para parar una corrida a mitad, usa siempre SIGINT, nunca SIGTERM.** Solo SIGINT está manejado; un SIGTERM mata Python dejando el motor girando a la última consigna. Desde la laptop: `ssh pi@<ip> 'pkill -INT -f ronda_camara'`.

---

## 8. Cambiar de versión entre rondas

En competencia conviene poder volver a una versión conocida sin depender de internet ni de la laptop. Los respaldos viven en la propia Pi, en `~/respaldos/`:

```bash
./usar.sh                 # lista las versiones y dice cuál está puesta
./usar.sh 02              # pone esa versión
```

`usar.sh` copia los `.py` encima de `~/ronda_curvas`, **sin tocar `logs/`** (no se pierde ninguna medida al cambiar), borra los `.pyc` viejos —que si no, se corre una mezcla de dos versiones sin enterarse— y compila todo antes de dar el visto bueno. Guarda lo que había en `~/respaldos/99_ANTES_DEL_CAMBIO`, así que un cambio equivocado se deshace con `./usar.sh 99`.

Se niega a cambiar nada si el servicio está corriendo. Eso es a propósito: cambiar los `.py` bajo un proceso vivo es la forma más rápida de tener un robot corriendo código que nadie escribió.

---

## 9. Problemas comunes

| Síntoma | Causa probable | Solución |
| :--- | :--- | :--- |
| `RuntimeError: Cannot determine SOC peripheral base address` | `RPi.GPIO` de pip ganándole al shim `python3-rpi-lgpio`. **Específico de la Pi 5** | `pip uninstall -y RPi.GPIO`. Ver sección 2.2 — vuelve a pasar cada vez que se instala Adafruit-Blinka |
| El botón de arranque no responde | Lo mismo de arriba, o el servicio ya tiene tomado el GPIO | Comprobar 2.2; si no, `sudo systemctl stop wro.service` |
| `ModuleNotFoundError: No module named 'picamera2'` | Se instaló por `pip` en vez de `apt`, o el venv no tiene `--system-site-packages` | `sudo apt install -y python3-picamera2` y recrear el venv con `--system-site-packages` |
| `error: externally-managed-environment` al hacer `pip install` | Protección PEP 668 de Bookworm | Venv (sección 2.3, opción A) o `--break-system-packages` (opción B) |
| `PermissionError` al abrir `/dev/ttyUSB0` o `/dev/ttyACM0` | El usuario `pi` no está en el grupo `dialout` | `sudo usermod -aG dialout pi` y reiniciar sesión |
| El robot no se mueve y no dice por qué; cero líneas de telemetría | El Pico quedó vivo pero en raw REPL | `bash src/pico/deploy_pico.sh --reiniciar` (sección 4.3) |
| El Pico «parece mudo» al abrir el puerto a pelo para comprobarlo | Abrir `/dev/ttyACM0` puede reiniciar el Pico | Esperar ~2 s tras abrirlo y luego `reset_input_buffer()` antes de leer |
| `~/correr_completa.sh` falla en silencio, sin mensaje | El `.sh` se copió con finales de línea CRLF desde Windows | `file ~/correr_completa.sh`; convertir a LF (sección 5) |
| `/dev/i2c-1` no existe | Falta `dtparam=i2c_arm=on` o falta reiniciar después de añadirlo | Sección 3.2 |
| Estoy tocando la Pi equivocada | Las dos Raspberry se llaman `wro-fe` | `uname -r`: `-2712` es la Pi 5, `-v8` es la 3B |
| El estacionamiento aborta antes de moverse | El robot está mal colocado en la plaza | El propio mensaje dice cuántos mm moverlo y hacia dónde. La banda válida por detrás es 8-95 mm |
