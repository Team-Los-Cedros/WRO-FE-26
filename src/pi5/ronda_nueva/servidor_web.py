# -*- coding: utf-8 -*-
"""Panel web para ver en vivo que esta haciendo el robot.

Hasta ahora la unica ventana a una corrida eran el CSV -que dice lo que el
robot CREYO- y el video cenital -que dice lo que PASO-, y siempre a
posteriori. Esto ensena las dos cosas mientras ocurren: el estado de la FSM,
las distancias, el hueco de parqueo, lo que ve la camara y el barrido del
LiDAR en planta.

Reglas de diseno, en orden de importancia:

1. **No tocar el ciclo de control.** El servidor vive en hilos aparte y solo
   lee un snapshot bajo lock. Publicar cuesta un ``dict`` y un ``lock``.
2. **Coste cero si nadie mira.** El frame de camara solo se copia cuando hay
   un cliente de video conectado; sin navegador abierto no se paga la copia
   ni la codificacion JPEG.
3. **Nunca puede tumbar la ronda.** Cualquier excepcion del servidor se traga
   y se anota: un panel roto no puede parar el robot.

Se sirve en la LAN del banco de pruebas, sin autenticacion, igual que el
resto del banco. No exponerlo fuera de la red local.
"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional


class EstadoRobot:
    """Snapshot compartido entre el ciclo de control y el servidor."""

    def __init__(self, submuestreo_lidar: int = 2):
        self._lock = threading.Lock()
        self._datos: Dict[str, Any] = {}
        self._puntos = []
        self._frame_jpeg: Optional[bytes] = None
        self._frame_ts = 0.0
        self._clientes_video = 0
        self._submuestreo = max(1, int(submuestreo_lidar))
        self._arranque = time.monotonic()

    # -- lado del robot ---------------------------------------------------

    def publicar(self, **campos: Any) -> None:
        with self._lock:
            self._datos.update(campos)
            self._datos["t_panel"] = time.monotonic() - self._arranque

    def publicar_barrido(self, muestras) -> None:
        """Guarda el barrido en planta, submuestreado para no inflar el JSON."""

        try:
            puntos = [
                (round(float(a), 1), round(float(d), 1))
                for a, d in muestras[:: self._submuestreo]
                if d and d > 0.0
            ]
        except Exception:
            return
        with self._lock:
            self._puntos = puntos

    @property
    def quiere_video(self) -> bool:
        with self._lock:
            return self._clientes_video > 0

    def publicar_frame(self, frame) -> None:
        """Codifica a JPEG solo si alguien esta mirando."""

        if not self.quiere_video:
            return
        try:
            import cv2

            alto = frame.shape[0]
            if alto > 480:
                escala = 480.0 / alto
                frame = cv2.resize(
                    frame, None, fx=escala, fy=escala, interpolation=cv2.INTER_AREA
                )
            ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ok:
                return
            datos = buffer.tobytes()
        except Exception:
            return
        with self._lock:
            self._frame_jpeg = datos
            self._frame_ts = time.monotonic()

    # -- lado del servidor ------------------------------------------------

    def leer(self) -> Dict[str, Any]:
        with self._lock:
            datos = dict(self._datos)
            datos["lidar"] = list(self._puntos)
            datos["video"] = self._frame_jpeg is not None
            return datos

    def leer_frame(self):
        with self._lock:
            return self._frame_jpeg, self._frame_ts

    def _sumar_cliente(self, delta: int) -> None:
        with self._lock:
            self._clientes_video = max(0, self._clientes_video + delta)


PAGINA = """<!doctype html>
<meta charset="utf-8">
<title>WRO-FE-26 en vivo</title>
<style>
 :root{--bg:#12141a;--panel:#1b1f28;--linea:#2c323f;--txt:#e6e9ef;--suave:#93a0b5;
       --ok:#4ade80;--mal:#f87171;--avi:#fbbf24;--ac:#60a5fa}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--txt);
      font:14px/1.45 ui-monospace,"Cascadia Code",Consolas,monospace}
 header{padding:10px 16px;border-bottom:1px solid var(--linea);
        display:flex;gap:18px;align-items:baseline;flex-wrap:wrap}
 h1{font-size:15px;margin:0;letter-spacing:.06em;text-transform:uppercase}
 #estado{font-size:20px;font-weight:700;color:var(--ac)}
 #razon{color:var(--suave)}
 .envoltura{display:grid;grid-template-columns:minmax(300px,1fr) minmax(300px,1.1fr);
            gap:14px;padding:14px;align-items:start}
 @media(max-width:820px){.envoltura{grid-template-columns:1fr}}
 .caja{background:var(--panel);border:1px solid var(--linea);border-radius:8px;padding:12px}
 .caja h2{font-size:11px;margin:0 0 10px;color:var(--suave);letter-spacing:.12em;
          text-transform:uppercase}
 .rejilla{display:grid;grid-template-columns:repeat(auto-fit,minmax(96px,1fr));gap:8px}
 .dato{background:#232936;border-radius:6px;padding:7px 9px}
 .dato .k{font-size:10px;color:var(--suave);text-transform:uppercase;letter-spacing:.06em}
 .dato .v{font-size:17px;font-weight:600;font-variant-numeric:tabular-nums;
           overflow-wrap:anywhere}
 canvas,img{width:100%;border-radius:6px;background:#0c0e13;display:block}
 .ok{color:var(--ok)} .mal{color:var(--mal)} .avi{color:var(--avi)}
 #corte{padding:6px 16px;color:var(--mal);display:none}
</style>
<header>
  <h1>WRO-FE-26</h1>
  <span id="estado">--</span>
  <span id="razon"></span>
  <span id="reloj" style="margin-left:auto;color:var(--suave)"></span>
</header>
<div id="corte">sin conexion con el robot</div>
<div class="envoltura">
  <div>
    <div class="caja">
      <h2>Marcha</h2>
      <div class="rejilla" id="marcha"></div>
    </div>
    <div class="caja" style="margin-top:14px">
      <h2>Distancias (mm)</h2>
      <div class="rejilla" id="distancias"></div>
    </div>
    <div class="caja" style="margin-top:14px">
      <h2>Carril</h2>
      <div class="rejilla" id="carril"></div>
    </div>
    <div class="caja" style="margin-top:14px">
      <h2>Mapa y tiempos</h2>
      <div class="rejilla" id="extra"></div>
    </div>
  </div>
  <div>
    <div class="caja">
      <h2>LiDAR en planta &mdash; el robot mira hacia arriba</h2>
      <canvas id="plano" width="460" height="460"></canvas>
    </div>
    <div class="caja" style="margin-top:14px">
      <h2>Camara</h2>
      <img id="camara" alt="camara">
    </div>
  </div>
</div>
<script>
const ESCALA = 2200;           // mm representados de borde a centro
function pinta(id, pares){
  document.getElementById(id).innerHTML = pares.map(function(p){
    return '<div class="dato"><div class="k">'+p[0]+'</div><div class="v '+
           (p[2]||'')+'">'+p[1]+'</div></div>';
  }).join('');
}
function num(v, dec){
  if(v===undefined||v===null||v==='') return '--';
  return (typeof v==='number') ? v.toFixed(dec===undefined?0:dec) : v;
}
function dibujaPlano(puntos){
  const c = document.getElementById('plano'), g = c.getContext('2d');
  const w = c.width, h = c.height, cx = w/2, cy = h/2;
  const k = (Math.min(w,h)/2) / ESCALA;
  g.clearRect(0,0,w,h);
  g.strokeStyle = '#2c323f'; g.lineWidth = 1;
  for(let r=500; r<=2000; r+=500){
    g.beginPath(); g.arc(cx,cy,r*k,0,Math.PI*2); g.stroke();
  }
  g.beginPath(); g.moveTo(cx,0); g.lineTo(cx,h); g.moveTo(0,cy); g.lineTo(w,cy); g.stroke();
  g.fillStyle = '#60a5fa';
  (puntos||[]).forEach(function(p){
    // 0 grados al frente, crece en sentido horario; y hacia arriba en pantalla
    const a = p[0]*Math.PI/180, d = p[1];
    const x = cx + Math.sin(a)*d*k, y = cy - Math.cos(a)*d*k;
    g.fillRect(x-1.5, y-1.5, 3, 3);
  });
  g.fillStyle = '#fbbf24';                       // el robot
  g.fillRect(cx-4, cy-6, 8, 12);
}
async function tic(){
  try{
    const r = await fetch('estado.json', {cache:'no-store'});
    const d = await r.json();
    document.getElementById('corte').style.display = 'none';
    document.getElementById('estado').textContent = d.estado || '--';
    document.getElementById('razon').textContent = d.razon || '';
    document.getElementById('reloj').textContent = 't = ' + num(d.t,1) + ' s';
    const wd = d.watchdog_pico || '--';
    pinta('marcha', [
      ['velocidad', num(d.velocidad), ''],
      ['direccion', num(d.angulo,1)+'&deg;', ''],
      ['rumbo err.', num(d.rumbo_error,1)+'&deg;', ''],
      ['esquinas', num(d.esquinas)+'/12', ''],
      ['sentido', (d.sentido>0?'horario':(d.sentido<0?'antihorario':'--')), ''],
      ['watchdog', wd, wd==='OK'?'ok':'mal'],
    ]);
    pinta('distancias', [
      ['corredor', num(d.corredor), (d.corredor<150)?'mal':(d.corredor<320?'avi':'')],
      ['izquierda', num(d.izquierda), ''],
      ['derecha', num(d.derecha), ''],
      ['trasera', num(d.trasera), ''],
      ['ultrasonido', num(d.ultrasonido_mm), ''],
      ['segmento', num(d.segmento), ''],
    ]);
    pinta('carril', [
      ['avance', num(d.avance), d.avance_valido?'':'avi'],
      ['offset', num(d.offset), d.offset_valido?'':'avi'],
      ['objetivo', num(d.objetivo_offset), ''],
      ['error lat.', num(d.error_lateral), (Math.abs(d.error_lateral)>150)?'avi':''],
      ['plan cedido', d.plan_cedido?'SI':'no', d.plan_cedido?'avi':''],
      ['pilares', num(d.pilares), ''],
    ]);
    pinta('extra', [
      ['mapa', d.mapa || '--', ''],
      ['casillas', num(d.casillas)+'/12', ''],
      ['hueco conf.', num(d.hueco_confianza,2), (d.hueco_confianza>0)?'ok':''],
      ['edad LiDAR', num(d.lidar_edad_ms,1)+' ms', ''],
      ['vision', num(d.vision_edad_ms,0)+' ms', ''],
      ['ciclo', num(d.ciclo_ms,1)+' ms', ''],
    ]);
    dibujaPlano(d.lidar);
  }catch(e){
    document.getElementById('corte').style.display = 'block';
  }
}
setInterval(tic, 200); tic();
document.getElementById('camara').src = 'camara.mjpg';
</script>
"""


def _crear_manejador(estado: EstadoRobot):
    class Manejador(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_args):  # el panel no ensucia la consola
            pass

        def _cabecera(self, tipo: str, longitud: int, cache: bool = False) -> None:
            self.send_response(200)
            self.send_header("Content-Type", tipo)
            self.send_header("Content-Length", str(longitud))
            if not cache:
                self.send_header("Cache-Control", "no-store")
            self.end_headers()

        def do_GET(self):  # noqa: N802 (nombre impuesto por la libreria)
            ruta = self.path.split("?")[0]
            try:
                if ruta in ("/", "/index.html"):
                    cuerpo = PAGINA.encode("utf-8")
                    self._cabecera("text/html; charset=utf-8", len(cuerpo))
                    self.wfile.write(cuerpo)
                elif ruta == "/estado.json":
                    cuerpo = json.dumps(estado.leer()).encode("utf-8")
                    self._cabecera("application/json", len(cuerpo))
                    self.wfile.write(cuerpo)
                elif ruta == "/camara.mjpg":
                    self._servir_video()
                else:
                    self.send_error(404)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception:
                # Un panel roto no puede tumbar la ronda.
                try:
                    self.send_error(500)
                except Exception:
                    pass

        def _servir_video(self) -> None:
            estado._sumar_cliente(1)
            try:
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=marco",
                )
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                ultimo = 0.0
                while True:
                    datos, ts = estado.leer_frame()
                    if datos is None or ts == ultimo:
                        time.sleep(0.03)
                        continue
                    ultimo = ts
                    self.wfile.write(b"--marco\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        ("Content-Length: %d\r\n\r\n" % len(datos)).encode()
                    )
                    self.wfile.write(datos)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                estado._sumar_cliente(-1)

    return Manejador


class ServidorPanel:
    """Arranca el panel en un hilo daemon; nunca bloquea al que lo crea."""

    def __init__(self, estado: EstadoRobot, puerto: int = 8080):
        self.estado = estado
        self.puerto = int(puerto)
        self._servidor: Optional[ThreadingHTTPServer] = None
        self._hilo: Optional[threading.Thread] = None
        self.error: Optional[str] = None

    def arrancar(self) -> bool:
        try:
            self._servidor = ThreadingHTTPServer(
                ("0.0.0.0", self.puerto), _crear_manejador(self.estado)
            )
            self._servidor.daemon_threads = True
            self._hilo = threading.Thread(
                target=self._servidor.serve_forever,
                name="panel-web",
                daemon=True,
            )
            self._hilo.start()
            return True
        except Exception as exc:
            self.error = str(exc)
            return False

    def detener(self) -> None:
        if self._servidor is not None:
            try:
                self._servidor.shutdown()
                self._servidor.server_close()
            except Exception:
                pass


__all__ = ["EstadoRobot", "ServidorPanel"]
