# -*- coding: utf-8 -*-
"""Sirve la camara de a bordo en vivo por HTTP, a resolucion completa.

Para apuntar la camara, revisar el encuadre del mastil o mirar la pista desde
el robot sin tener que lanzar una ronda. Es una herramienta de banco: no la
usa el control y no debe correr a la vez que una ronda -- la camara es
exclusiva y el servidor se niega si hay una en marcha.

DOS COSAS QUE NO SON OBVIAS
---------------------------

**El campo de vision depende del modo RAW, no del tamano que pidas.** El
IMX708 tiene tres modos y solo dos ven el sensor entero::

    1536x864   ->  recorta a (768,432)/3072x1728   -- pierde un tercio
    2304x1296  ->  (0,0)/4608x2592                 -- campo completo
    4608x2592  ->  (0,0)/4608x2592                 -- campo completo

Por eso aqui el flujo ``raw`` se fija SIEMPRE a 4608x2592: pidas el tamano
que pidas, lo que sale es el campo entero, escalado. Si se dejara elegir a
libcamera, pedir 1536x864 devolveria una imagen recortada que NO es lo que
la ronda ve, y serviria para aparentar que se encuadra bien algo que no.

**A resolucion completa el cuello de botella es el JPEG, no la red.** La Pi 5
no tiene codificador JPEG por hardware, asi que 12 Mpixeles se comprimen en
software. El servidor mide y publica la cadencia real en ``/estado``; si no
llega, ``--escala`` reduce lo que se TRANSMITE conservando el campo.

Uso::

    python3 herramientas/camara_web.py                 # 4608x2592, puerto 8080
    python3 herramientas/camara_web.py --escala 0.5    # mismo campo, mitad de lado
    python3 herramientas/camara_web.py --puerto 9000

Y en el navegador: ``http://<ip-de-la-pi>:8080/``
"""
from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# El sensor completo del IMX708. No es configurable a proposito: es lo que
# garantiza que el recorte sea (0,0,4608,2592) y se vea todo el campo.
SENSOR_COMPLETO = (4608, 2592)

LIMITE = b"--marcofoto"


class CamaraEnVivo:
    """Captura en un hilo y guarda SOLO el ultimo JPEG.

    Nunca acumula: si el navegador va mas lento que la camara, se pierden
    frames intermedios en vez de crecer una cola. Para mirar en vivo, un
    frame viejo no vale nada.
    """

    def __init__(self, ancho, alto, calidad, escala, fps_max):
        self.ancho = int(ancho)
        self.alto = int(alto)
        self.calidad = int(calidad)
        self.escala = float(escala)
        self.fps_max = float(fps_max)

        self._lock = threading.Lock()
        self._jpeg = None
        self._seq = 0
        self._nuevo = threading.Condition(self._lock)
        self._corriendo = True
        self._clientes = 0

        # Metricas honestas: lo que tarda de verdad capturar y comprimir.
        self.fps_real = 0.0
        self.ms_captura = 0.0
        self.ms_jpeg = 0.0
        self.bytes_frame = 0
        self.error = None

    @property
    def hay_publico(self):
        with self._lock:
            return self._clientes > 0

    def sumar_cliente(self, delta):
        with self._lock:
            self._clientes = max(0, self._clientes + delta)

    def bucle(self):
        try:
            import cv2
            import numpy as np
            from picamera2 import Picamera2

            camara = Picamera2()
            duracion_us = int(1_000_000.0 / max(1.0, self.fps_max))
            configuracion = camara.create_video_configuration(
                main={"size": (self.ancho, self.alto), "format": "RGB888"},
                # El campo entero, pase lo que pase con `main`.
                raw={"size": SENSOR_COMPLETO},
                controls={"FrameDurationLimits": (duracion_us, duracion_us)},
                buffer_count=2,
            )
            camara.configure(configuracion)
            camara.start()
            time.sleep(1.5)          # que AE/AWB se asienten antes de servir

            destino = None
            if self.escala < 0.999:
                destino = (
                    max(2, int(self.ancho * self.escala)),
                    max(2, int(self.alto * self.escala)),
                )

            ultimo = time.monotonic()
            while self._corriendo:
                if not self.hay_publico:
                    # Coste cero si nadie mira: ni copia ni compresion.
                    time.sleep(0.1)
                    ultimo = time.monotonic()
                    continue

                t0 = time.monotonic()
                marco = camara.capture_array("main")
                t1 = time.monotonic()
                if destino is not None:
                    marco = cv2.resize(marco, destino, interpolation=cv2.INTER_AREA)
                ok, buffer = cv2.imencode(
                    ".jpg", marco, [int(cv2.IMWRITE_JPEG_QUALITY), self.calidad]
                )
                t2 = time.monotonic()
                if not ok:
                    continue

                datos = buffer.tobytes()
                ahora = time.monotonic()
                with self._nuevo:
                    self._jpeg = datos
                    self._seq += 1
                    self.ms_captura = (t1 - t0) * 1000.0
                    self.ms_jpeg = (t2 - t1) * 1000.0
                    self.bytes_frame = len(datos)
                    intervalo = ahora - ultimo
                    if intervalo > 0:
                        # Media movil suave, para que el numero no baile.
                        instantanea = 1.0 / intervalo
                        self.fps_real = (
                            instantanea if self.fps_real == 0.0
                            else 0.8 * self.fps_real + 0.2 * instantanea
                        )
                    self._nuevo.notify_all()
                ultimo = ahora

            camara.stop()
        except Exception as exc:            # una herramienta no puede reventar muda
            self.error = "{}: {}".format(type(exc).__name__, exc)
            print("[-] Fallo en la camara: " + self.error, file=sys.stderr)

    def esperar_frame(self, visto, timeout=5.0):
        with self._nuevo:
            if self._seq == visto:
                self._nuevo.wait(timeout)
            return self._jpeg, self._seq

    def parar(self):
        self._corriendo = False


PAGINA = """<!doctype html>
<title>Camara del robot</title>
<style>
  body {{ margin:0; background:#111; color:#ddd;
         font:14px system-ui,sans-serif; }}
  header {{ padding:8px 12px; background:#000; display:flex; gap:16px;
            align-items:center; flex-wrap:wrap; }}
  img {{ display:block; width:100%; height:auto; }}
  code {{ color:#7cf; }}
</style>
<header>
  <strong>Camara del robot</strong>
  <span>{ancho}&times;{alto}{nota}</span>
  <span id=est>midiendo...</span>
  <a href="/foto.jpg" style="color:#7cf">descargar foto</a>
</header>
<img src="/stream.mjpg" alt="video en vivo">
<script>
setInterval(async () => {{
  try {{
    const r = await fetch('/estado');
    const e = await r.json();
    document.getElementById('est').textContent =
      e.fps.toFixed(1) + ' fps | captura ' + e.ms_captura.toFixed(0) +
      ' ms | jpeg ' + e.ms_jpeg.toFixed(0) + ' ms | ' +
      (e.kb / 1024).toFixed(2) + ' MB/frame';
  }} catch (err) {{}}
}}, 1000);
</script>
"""


def _crear_manejador(camara, nota):
    class Manejador(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_args):
            pass                      # no ensuciar la consola con cada frame

        def _cabecera(self, tipo, largo=None):
            self.send_response(200)
            self.send_header("Content-Type", tipo)
            if largo is not None:
                self.send_header("Content-Length", str(largo))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

        def do_GET(self):
            ruta = self.path.split("?")[0]
            if ruta == "/":
                cuerpo = PAGINA.format(
                    ancho=camara.ancho, alto=camara.alto, nota=nota
                ).encode("utf-8")
                self._cabecera("text/html; charset=utf-8", len(cuerpo))
                self.wfile.write(cuerpo)
            elif ruta == "/estado":
                import json

                cuerpo = json.dumps({
                    "fps": camara.fps_real,
                    "ms_captura": camara.ms_captura,
                    "ms_jpeg": camara.ms_jpeg,
                    "kb": camara.bytes_frame / 1024.0,
                    "error": camara.error,
                }).encode("ascii")
                self._cabecera("application/json", len(cuerpo))
                self.wfile.write(cuerpo)
            elif ruta == "/foto.jpg":
                camara.sumar_cliente(1)
                try:
                    datos, _ = camara.esperar_frame(-1)
                finally:
                    camara.sumar_cliente(-1)
                if not datos:
                    self.send_error(503, "sin imagen")
                    return
                self._cabecera("image/jpeg", len(datos))
                self.wfile.write(datos)
            elif ruta == "/stream.mjpg":
                self._servir_stream()
            else:
                self.send_error(404)

        def _servir_stream(self):
            camara.sumar_cliente(1)
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=" + LIMITE.decode()[2:],
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            visto = -1
            try:
                while True:
                    datos, visto = camara.esperar_frame(visto)
                    if not datos:
                        continue
                    self.wfile.write(LIMITE + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        b"Content-Length: " + str(len(datos)).encode() + b"\r\n\r\n"
                    )
                    self.wfile.write(datos)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass                  # el navegador cerro la pestana; normal
            finally:
                camara.sumar_cliente(-1)

    return Manejador


def _ip_local():
    try:
        con = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        con.connect(("8.8.8.8", 80))
        ip = con.getsockname()[0]
        con.close()
        return ip
    except Exception:
        return "127.0.0.1"


def _ronda_en_marcha():
    """La camara es exclusiva: si hay ronda, este servidor la reventaria."""

    try:
        salida = subprocess.run(
            ["pgrep", "-af", "ronda_nueva|ronda_cerrada|ronda_abierta"],
            capture_output=True, text=True,
        ).stdout.strip()
    except FileNotFoundError:
        return ""
    lineas = [l for l in salida.splitlines() if "camara_web" not in l]
    return "\n".join(lineas)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Camara del robot en vivo por HTTP")
    parser.add_argument("--ancho", type=int, default=SENSOR_COMPLETO[0])
    parser.add_argument("--alto", type=int, default=SENSOR_COMPLETO[1])
    parser.add_argument("--puerto", type=int, default=8080)
    parser.add_argument("--calidad", type=int, default=80,
                        help="calidad JPEG 1-100 (%(default)s)")
    parser.add_argument("--escala", type=float, default=1.0,
                        help="reduce lo TRANSMITIDO sin recortar campo")
    # El tope depende del tamano, y ponerlo fijo es una trampa: 14 fps es el
    # maximo a 4608x2592, pero a 1536x864 deja la captura esperando 63 ms por
    # cuadro y parece que la herramienta va lenta cuando el limite lo pone uno.
    parser.add_argument("--fps", type=float, default=None,
                        help="tope de cadencia (por defecto 14 a sensor completo, 30 si no)")
    args = parser.parse_args(argv)
    if args.fps is None:
        completo = (args.ancho, args.alto) == SENSOR_COMPLETO
        args.fps = 14.0 if completo else 30.0

    ocupada = _ronda_en_marcha()
    if ocupada:
        print("[-] Hay una ronda en marcha y la camara es exclusiva:", file=sys.stderr)
        print(ocupada, file=sys.stderr)
        return 3

    camara = CamaraEnVivo(args.ancho, args.alto, args.calidad, args.escala, args.fps)
    hilo = threading.Thread(target=camara.bucle, name="camara", daemon=True)
    hilo.start()

    nota = ""
    if args.escala < 0.999:
        nota = " (se transmite a {:.0%}, campo completo)".format(args.escala)

    servidor = ThreadingHTTPServer(
        ("0.0.0.0", args.puerto), _crear_manejador(camara, nota)
    )
    print("[+] Camara en http://{}:{}/".format(_ip_local(), args.puerto))
    print("    {}x{} desde el sensor completo 4608x2592 (sin recorte)".format(
        args.ancho, args.alto))
    print("    Ctrl-C para parar.")
    try:
        servidor.serve_forever()
    except KeyboardInterrupt:
        print("\n[i] Parando.")
    finally:
        camara.parar()
        servidor.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
