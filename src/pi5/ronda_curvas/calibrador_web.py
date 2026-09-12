#!/usr/bin/env python3
"""Calibrador de color por navegador. Para usar EN COMPETENCIA.

    python3 calibrador_web.py          (en la Pi)
    http://10.244.181.244:8080         (desde la laptop o el movil)

Por que existe: los umbrales HSV estaban fijos dentro de vision.py. En
otro pabellon la luz es distinta -- focos, ventanas, el color del suelo --
y hasta ahora recalibrar significaba editar Python por SSH a ciegas, sin
ver la mascara que estabas cambiando. Eso no se puede hacer con el
cronometro corriendo.

Aqui se ve el frame y la mascara UNO AL LADO DEL OTRO mientras mueves los
umbrales, y el area del blob mayor en numeros, que es lo que de verdad
decide si el robot lo ve o no (AREA_MIN_DETECCION).

Lo que guarda va a `calibracion.json`, que `vision.py` lee al arrancar.
Si el archivo no esta o esta roto, se usan los valores del codigo: el
archivo solo puede mejorar la calibracion, nunca dejar el robot sin una.

NO mueve el robot. No abre el LiDAR ni la Pico: solo la camara. Por eso
hay que pararlo antes de correr una ronda -- la camara no se puede abrir
dos veces.

    sudo systemctl stop wro.service      antes
    Ctrl-C                               para salir
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import vision                              # noqa: E402
from camara_driver import CamaraDriver     # noqa: E402

PUERTO = 8080
ANCHO_VISTA = 480          # ancho al que se reescalan las dos imagenes

# Los cuatro colores que el robot necesita ver, con los umbrales que le
# corresponden. El rojo lleva DOS rangos porque en HSV el rojo esta
# partido en los dos extremos del circulo de tono.
COLORES = {
    "ROJO":    ["ROJO_BAJO_1", "ROJO_ALTO_1", "ROJO_BAJO_2", "ROJO_ALTO_2"],
    "VERDE":   ["VERDE_BAJO", "VERDE_ALTO"],
    "NARANJA": ["NARANJA_BAJO", "NARANJA_ALTO"],
    "AZUL":    ["AZUL_BAJO", "AZUL_ALTO"],
}
# Para que sirve cada uno, que es lo que se olvida a los cinco minutos.
PARA_QUE = {
    "ROJO":    "pilar rojo: se pasa por su DERECHA",
    "VERDE":   "pilar verde: se pasa por su IZQUIERDA",
    "NARANJA": "linea del suelo: cuenta vueltas y marca sentido horario",
    "AZUL":    "linea del suelo: cuenta vueltas y marca sentido antihorario",
}

_frame = [None]
_corriendo = [True]


def _al_frame(f):
    _frame[0] = f


def _mascara(hsv, pares):
    """OR de los rangos de un color. `pares` son (bajo, alto) ya en array."""
    m = None
    for bajo, alto in pares:
        trozo = cv2.inRange(hsv, bajo, alto)
        m = trozo if m is None else (m | trozo)
    return m


def _blob_mayor(mask):
    cont, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mejor, caja = 0.0, None
    for c in cont:
        a = cv2.contourArea(c)
        if a > mejor:
            mejor, caja = a, cv2.boundingRect(c)
    return mejor, caja


def _jpeg(img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return buf.tobytes() if ok else b""


def _reescalar(img):
    alto, ancho = img.shape[:2]
    if ancho <= ANCHO_VISTA:
        return img
    return cv2.resize(img, (ANCHO_VISTA, int(alto * ANCHO_VISTA / ancho)))


class Manejador(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass                                # sin ruido en la consola

    def _responder(self, codigo, tipo, cuerpo):
        self.send_response(codigo)
        self.send_header("Content-Type", tipo)
        self.send_header("Content-Length", str(len(cuerpo)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(cuerpo)

    def do_GET(self):
        partes = urlparse(self.path)
        ruta, q = partes.path, parse_qs(partes.query)
        if ruta == "/":
            return self._responder(200, "text/html; charset=utf-8",
                                   PAGINA.encode("utf-8"))
        if ruta == "/valores":
            datos = vision.valores_calibrables()
            datos["_guardado"] = os.path.exists(vision.ARCHIVO_CALIBRACION)
            return self._responder(200, "application/json",
                                   json.dumps(datos).encode())
        if ruta in ("/frame.jpg", "/mask.jpg"):
            f = _frame[0]
            if f is None:
                return self._responder(503, "text/plain", b"sin frame")
            if ruta == "/frame.jpg":
                return self._responder(200, "image/jpeg", _jpeg(_reescalar(f)))
            return self._mascara_jpeg(f, q)
        self._responder(404, "text/plain", b"no")

    def _mascara_jpeg(self, f, q):
        """Mascara con los umbrales que manda el navegador AHORA.

        Van por la URL a proposito: asi mover un deslizador se ve al
        instante sin guardar nada ni tocar lo que usa el robot.
        """
        hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
        color = q.get("color", ["ROJO"])[0]
        nombres = COLORES.get(color, COLORES["ROJO"])
        pares = []
        for i in range(0, len(nombres), 2):
            bajo = q.get(nombres[i], [None])[0]
            alto = q.get(nombres[i + 1], [None])[0]
            try:
                b = np.array([int(x) for x in bajo.split(",")])
                a = np.array([int(x) for x in alto.split(",")])
            except (AttributeError, ValueError):
                b = getattr(vision, nombres[i])
                a = getattr(vision, nombres[i + 1])
            pares.append((b, a))
        mask = _mascara(hsv, pares)
        area, caja = _blob_mayor(mask)
        vista = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if caja is not None:
            x, y, w, h = caja
            cv2.rectangle(vista, (x, y), (x + w, y + h), (0, 128, 255), 2)
        vista = _reescalar(vista)
        # El area va en la cabecera, no dibujada encima: asi el navegador
        # la puede poner en texto legible y no se pierde al reescalar.
        ok, buf = cv2.imencode(".jpg", vista,
                               [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        cuerpo = buf.tobytes() if ok else b""
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(cuerpo)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Area", "%d" % area)
        self.send_header("X-Caja", "%d,%d" % (caja[2], caja[3]) if caja else "0,0")
        self.send_header("Access-Control-Expose-Headers", "X-Area, X-Caja")
        self.end_headers()
        self.wfile.write(cuerpo)

    def do_POST(self):
        if urlparse(self.path).path != "/guardar":
            return self._responder(404, "text/plain", b"no")
        n = int(self.headers.get("Content-Length", 0))
        try:
            datos = json.loads(self.rfile.read(n).decode("utf-8"))
        except (ValueError, UnicodeDecodeError) as e:
            return self._responder(400, "text/plain", str(e).encode())
        # Se escribe a un temporal y se renombra: si se corta la luz a
        # mitad, el archivo viejo sigue entero. Un calibracion.json medio
        # escrito es justo lo que vision.py no deberia tener que sortear.
        tmp = vision.ARCHIVO_CALIBRACION + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(datos, f, indent=2, ensure_ascii=False)
            os.replace(tmp, vision.ARCHIVO_CALIBRACION)
        except OSError as e:
            return self._responder(500, "text/plain", str(e).encode())
        print("[+] guardado %s" % vision.ARCHIVO_CALIBRACION)
        self._responder(200, "application/json", b'{"ok":true}')


PAGINA = """<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Calibrador de color</title>
<style>
 :root{--fondo:#14161a;--panel:#1d2026;--borde:#2e333c;--texto:#e8eaed;
       --tenue:#9aa2ad;--ok:#4caf7d;--aviso:#e0a33e}
 *{box-sizing:border-box}
 body{margin:0;background:var(--fondo);color:var(--texto);
      font:14px/1.5 system-ui,-apple-system,Segoe UI,sans-serif}
 header{padding:12px 16px;border-bottom:1px solid var(--borde);
        display:flex;gap:12px;align-items:center;flex-wrap:wrap}
 h1{font-size:16px;margin:0;font-weight:600}
 .tabs{display:flex;gap:6px;flex-wrap:wrap}
 .tab{padding:6px 12px;border:1px solid var(--borde);border-radius:6px;
      background:var(--panel);color:var(--tenue);cursor:pointer;font-size:13px}
 .tab.on{background:#2a3140;color:var(--texto);border-color:#3d4757}
 main{padding:16px;display:grid;gap:16px;
      grid-template-columns:repeat(auto-fit,minmax(320px,1fr))}
 .caja{background:var(--panel);border:1px solid var(--borde);
       border-radius:8px;padding:12px}
 .caja h2{font-size:13px;margin:0 0 8px;color:var(--tenue);font-weight:600;
          text-transform:uppercase;letter-spacing:.04em}
 img{width:100%;border-radius:6px;display:block;background:#000}
 .fila{display:grid;grid-template-columns:34px 1fr 46px;gap:8px;
       align-items:center;margin:6px 0}
 .fila label{color:var(--tenue);font-size:12px}
 .fila output{text-align:right;font-variant-numeric:tabular-nums;font-size:12px}
 input[type=range]{width:100%}
 .area{font-size:22px;font-variant-numeric:tabular-nums;margin:8px 0 2px}
 .nota{color:var(--tenue);font-size:12px}
 .pasa{color:var(--ok)} .nopasa{color:var(--aviso)}
 button{background:#2a3140;color:var(--texto);border:1px solid #3d4757;
        border-radius:6px;padding:8px 14px;font-size:13px;cursor:pointer}
 button:hover{background:#333b4c}
 .grupo{border-top:1px solid var(--borde);margin-top:10px;padding-top:8px}
 .grupo:first-of-type{border-top:0;margin-top:0}
 #estado{font-size:12px;color:var(--tenue)}
</style>
<header>
  <h1>Calibrador de color</h1>
  <div class="tabs" id="tabs"></div>
  <button onclick="guardar()">Guardar</button>
  <span id="estado"></span>
</header>
<main>
  <div class="caja"><h2>Camara</h2><img id="cam"></div>
  <div class="caja"><h2>Mascara</h2><img id="mask">
    <div class="area" id="area">--</div>
    <div class="nota" id="notaArea"></div>
  </div>
  <div class="caja"><h2>Umbrales</h2>
    <div class="nota" id="paraque"></div>
    <div id="ctrl"></div>
  </div>
</main>
<script>
const COLORES = %%COLORES%%, PARA_QUE = %%PARAQUE%%;
const ETQ = ["H (tono)", "S (saturacion)", "V (brillo)"], TOPE = [179, 255, 255];
let vals = {}, activo = "ROJO";

function pintarTabs(){
  tabs.innerHTML = "";
  for (const c of Object.keys(COLORES)) {
    const b = document.createElement("button");
    b.className = "tab" + (c === activo ? " on" : "");
    b.textContent = c;
    b.onclick = () => { activo = c; pintarTabs(); pintarCtrl(); };
    tabs.appendChild(b);
  }
}

function pintarCtrl(){
  paraque.textContent = PARA_QUE[activo];
  ctrl.innerHTML = "";
  for (const nombre of COLORES[activo]) {
    const g = document.createElement("div");
    g.className = "grupo";
    g.innerHTML = "<div class=nota>" + nombre + "</div>";
    vals[nombre].forEach((v, i) => {
      const f = document.createElement("div");
      f.className = "fila";
      f.innerHTML = "<label>" + ETQ[i][0] + "</label>" +
        "<input type=range min=0 max=" + TOPE[i] + " value=" + v + ">" +
        "<output>" + v + "</output>";
      const r = f.querySelector("input"), o = f.querySelector("output");
      r.oninput = () => { vals[nombre][i] = +r.value; o.textContent = r.value; };
      g.appendChild(f);
    });
    ctrl.appendChild(g);
  }
}

function urlMask(){
  const p = new URLSearchParams({color: activo});
  for (const n of COLORES[activo]) p.set(n, vals[n].join(","));
  p.set("_", Date.now());
  return "/mask.jpg?" + p;
}

async function refrescar(){
  cam.src = "/frame.jpg?_=" + Date.now();
  try {
    const r = await fetch(urlMask());
    const a = +r.headers.get("X-Area"), caja = r.headers.get("X-Caja");
    mask.src = URL.createObjectURL(await r.blob());
    area.textContent = a.toLocaleString("es") + " px";
    // El umbral que decide es distinto para pilares y para lineas.
    const esLinea = activo === "NARANJA" || activo === "AZUL";
    const min = esLinea ? vals.LINEA_AREA_MINIMA : vals.AREA_MIN_DETECCION;
    const pasa = a > min;
    area.className = "area " + (pasa ? "pasa" : "nopasa");
    notaArea.textContent = (pasa ? "PASA" : "NO LLEGA") + " el minimo de " +
      min + " px" + (caja !== "0,0" ? "  |  blob " + caja + " px" : "");
  } catch (e) { estado.textContent = "sin camara"; }
}

async function guardar(){
  estado.textContent = "guardando...";
  const r = await fetch("/guardar", {method:"POST", body: JSON.stringify(vals)});
  estado.textContent = r.ok ? "guardado en calibracion.json"
                            : "error al guardar";
  setTimeout(() => estado.textContent = "", 4000);
}

(async () => {
  vals = await (await fetch("/valores")).json();
  pintarTabs(); pintarCtrl();
  setInterval(refrescar, 700);
  refrescar();
})();
</script>
"""
PAGINA = (PAGINA.replace("%%COLORES%%", json.dumps(COLORES))
                .replace("%%PARAQUE%%", json.dumps(PARA_QUE)))


def main():
    cam = CamaraDriver()
    threading.Thread(target=cam.hilo_captura,
                     args=(lambda: _corriendo[0], _al_frame),
                     daemon=True).start()
    t0 = time.time()
    while _frame[0] is None and time.time() - t0 < 15:
        time.sleep(0.1)
    if _frame[0] is None:
        print("[-] la camara no da frames.")
        print("    Si la ronda esta corriendo, la camara esta tomada:")
        print("      sudo systemctl stop wro.service")
        return 1

    alto, ancho = _frame[0].shape[:2]
    print("[+] camara %dx%d" % (ancho, alto))
    if os.path.exists(vision.ARCHIVO_CALIBRACION):
        print("[+] hay calibracion.json: los umbrales que ves salen de ahi")
    else:
        print("[i] no hay calibracion.json todavia: umbrales del codigo")
    print()
    print("    Abre en la laptop o el movil:  http://%s:%d"
          % (os.environ.get("WRO_IP", "10.244.181.244"), PUERTO))
    print("    Ctrl-C para salir.")
    srv = ThreadingHTTPServer(("0.0.0.0", PUERTO), Manejador)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[fin]")
    finally:
        _corriendo[0] = False
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
