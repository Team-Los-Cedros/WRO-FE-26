# Panel de telemetria en vivo: LiDAR, camara y la fila de CSV, a la vez.
#
# Va DENTRO de ronda_camara.py como un hilo, no como proceso aparte: con
# la ronda corriendo, el LiDAR (/dev/ttyUSB0), la camara y la Pico estan
# tomados y nadie mas puede abrirlos.
#
# Se activa con WRO_PANEL=1 y se apaga solo si no. Una corrida de
# competicion no deberia llevarlo: el reglamento no lo prohibe -- no es
# control remoto, es solo mirar -- pero es CPU y un puerto abierto que no
# hacen falta cuando ya no se esta depurando.
#
# Coste: el servidor solo trabaja cuando el navegador pide. El JPEG de la
# camara se codifica en ese momento y a resolucion reducida; el perfil del
# LiDAR son 360 enteros. Nada de esto toca el hilo de control, que se
# limita a dejar el ultimo estado en una variable bajo lock.
import io
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

PUERTO = 8080

_lock = threading.Lock()
_estado = {}
_perfil = []
_frame = None            # ultimo frame BGR de la camara
_arrancado = False


def publicar(medicion, velocidad, angulo, navegador, extra=None):
    """Deja el estado del ciclo actual. Lo llama al_barrido, bajo lock."""
    global _perfil
    trk = navegador.tracker
    d = {
        "t": time.time(),
        "fase": navegador.fase,
        "estado": navegador.estado,
        "seguridad": navegador.seguridad,
        "velocidad": velocidad,
        "angulo": round(angulo, 1),
        "cmd_deseado": round(navegador.cmd_deseado, 1),
        "frontal": round(medicion.frontal),
        "frontal_muro": round(medicion.frontal_muro),
        "izquierda": round(medicion.izquierda),
        "derecha": round(medicion.derecha),
        "trasera": round(medicion.trasera),
        "angulo_muro": round(medicion.angulo_muro, 1),
        "muro_ok": bool(medicion.muro_valido),
        "n_seguros": navegador.n_seguros,
        "n_compatibles": navegador.n_compatibles,
        "fuente_lado": navegador.fuente_lado,
        "sentido": navegador.pista.sentido,
        "por_lineas": getattr(navegador.pista, "por_lineas", False),
        "trk_activo": bool(trk.activo),
        "trk_color": trk.color or "",
        "trk_x": round(trk.x), "trk_y": round(trk.y),
        "trk_sigma": round(trk.sigma),
        "paso_ok": bool(navegador._paso_validado),
        "rama": navegador.rama_evasion,
    }
    # Todos los clusters que pasan el filtro de ancho fisico, o sea los
    # candidatos a poste que ve el LiDAR AHORA. Sirve para ver de un
    # vistazo si el segundo bloque de una pareja se esta detectando, que
    # es lo primero que hay que saber antes de planificar sobre los dos.
    # Radios de los dos arcos que interesan: el que se EJECUTA y el que la
    # trayectoria PEDIA antes del arbitraje. Ver la diferencia dibujada es
    # ver quien manda en cada instante -- si el deseado sale limpio y el
    # ejecutado se pega al pilar, el recorte por paredes es el culpable.
    import geometria_evasion as _gev
    import geometria_robot as _geo
    def _r(a):
        v = _gev.radio_de_comando(a)
        return None if v == float("inf") else round(v)
    d["radio"] = _r(angulo)
    d["radio_deseado"] = _r(navegador.cmd_deseado)
    d["lidar_x"] = _geo.LIDAR_X
    from lidar_geometria import centroide_xy_cluster
    d["clusters"] = [[round(cx), round(cy)] for cx, cy in
                     (centroide_xy_cluster(c) for c in medicion.clusters_estrechos)]
    if extra:
        d.update(extra)
    with _lock:
        _estado.clear()
        _estado.update(d)
        _perfil = list(medicion.perfil)


def publicar_frame(frame):
    """Ultimo frame de la camara. Lo llama vision.procesar_frame."""
    global _frame
    with _lock:
        _frame = frame


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass                       # sin ruido en la consola de la ronda

    def _enviar(self, cuerpo, tipo="application/json"):
        self.send_response(200)
        self.send_header("Content-Type", tipo)
        self.send_header("Content-Length", str(len(cuerpo)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(cuerpo)

    def do_GET(self):
        try:
            if self.path.startswith("/estado"):
                with _lock:
                    cuerpo = json.dumps({"estado": _estado, "perfil": _perfil})
                self._enviar(cuerpo.encode())
            elif self.path.startswith("/camara"):
                with _lock:
                    f = None if _frame is None else _frame.copy()
                if f is None:
                    self._enviar(b"", "image/jpeg")
                    return
                f = cv2.resize(f, (320, 180))
                ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self._enviar(buf.tobytes() if ok else b"", "image/jpeg")
            else:
                self._enviar(PAGINA.encode(), "text/html; charset=utf-8")
        except (BrokenPipeError, ConnectionResetError):
            pass


def arrancar(puerto=PUERTO):
    """Levanta el servidor en un hilo daemon. Devuelve el puerto o None."""
    global _arrancado
    if _arrancado:
        return puerto
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", puerto), _Handler)
    except OSError as e:
        print("[panel] no se pudo abrir el puerto %d: %s" % (puerto, e))
        return None
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    _arrancado = True
    return puerto


PAGINA = """<!doctype html><meta charset="utf-8">
<title>WRO-FE · telemetria</title>
<style>
 :root{--bg:#0d1219;--pa:#151d27;--tx:#e7ecf3;--t2:#94a3b8;--li:#25303d;
       --az:#78a6e8;--na:#e9954f;--ok:#5fbe92;--ma:#f08279}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--tx);
      font:13px/1.5 ui-monospace,Menlo,Consolas,monospace}
 header{padding:10px 16px;border-bottom:1px solid var(--li);display:flex;
        gap:18px;align-items:baseline;flex-wrap:wrap}
 h1{font-size:14px;margin:0;letter-spacing:.12em;text-transform:uppercase}
 .lat{color:var(--t2)}
 main{display:grid;grid-template-columns:minmax(320px,1fr) 340px;gap:14px;padding:14px}
 @media(max-width:900px){main{grid-template-columns:1fr}}
 .caja{background:var(--pa);border:1px solid var(--li);padding:12px}
 .caja h2{font-size:11px;letter-spacing:.14em;text-transform:uppercase;
          color:var(--t2);margin:0 0 10px}
 canvas{width:100%;height:auto;display:block;background:#0a0f15}
 img{width:100%;display:block;background:#0a0f15;image-rendering:pixelated}
 table{width:100%;border-collapse:collapse}
 td{padding:2px 0;border-bottom:1px solid var(--li)}
 td:last-child{text-align:right;font-variant-numeric:tabular-nums}
 .k{color:var(--t2)}
 .pill{padding:1px 7px;border:1px solid currentColor;border-radius:2px;font-size:11px}
 .LIBRE{color:var(--ok)} .RECORTA{color:var(--az)} .APURA{color:var(--na)}
 .SUSPENDE,.INVIERTE{color:var(--na)} .SIN_SALIDA,.ULTIMO_RECURSO{color:var(--ma)}
</style>
<header>
  <h1>WRO-FE · telemetria</h1>
  <span class="lat" id="lat">conectando…</span>
  <span id="fase"></span><span id="estado"></span><span id="seg"></span>
</header>
<main>
  <div class="caja"><h2>LiDAR · marco del robot</h2><canvas id="c" width="620" height="620"></canvas>
    <div style="margin-top:8px;color:#94a3b8;font-size:11px">
      <span style="color:#5fbe92">━</span> trayectoria ejecutada &nbsp;
      <span style="color:#94a3b8">┄</span> deseada antes del arbitraje &nbsp;
      <span style="color:#f0564a">━</span>/<span style="color:#3fd08a">━</span> pilar seguido &nbsp;
      ○ candidatos del LiDAR &nbsp; <span style="color:#f08279">▨</span> arco ciego
    </div></div>
  <div>
    <div class="caja" style="margin-bottom:14px"><h2>Cámara</h2><img id="cam" alt=""></div>
    <div class="caja"><h2>Estado</h2><table id="t"></table></div>
  </div>
</main>
<script>
const ALCANCE=2200, cv=document.getElementById('c'), cx=cv.getContext('2d');
function dibuja(p){
  const W=cv.width,H=cv.height,ox=W/2,oy=H/2,k=(Math.min(W,H)/2-14)/ALCANCE;
  cx.fillStyle='#0a0f15';cx.fillRect(0,0,W,H);
  cx.strokeStyle='#25303d';
  for(const r of [500,1000,1500,2000]){cx.beginPath();cx.arc(ox,oy,r*k,0,7);cx.stroke();}
  cx.beginPath();cx.moveTo(ox,0);cx.lineTo(ox,H);cx.moveTo(0,oy);cx.lineTo(W,oy);cx.stroke();
  // arco ciego del mastil 135-199
  cx.fillStyle='rgba(240,130,121,.13)';cx.beginPath();cx.moveTo(ox,oy);
  cx.arc(ox,oy,Math.min(W,H)/2-14,(135-90)*Math.PI/180,(199-90)*Math.PI/180);cx.fill();
  if(!p) return;
  for(let i=0;i<360;i++){
    const d=p[i]; if(!d||d>=ALCANCE) continue;
    const a=(i-90)*Math.PI/180, x=ox+Math.cos(a)*d*k, y=oy+Math.sin(a)*d*k;
    cx.fillStyle = d<150?'#f08279' : (d<400?'#e9954f':'#78a6e8');
    cx.fillRect(x-1.5,y-1.5,3,3);
  }
  cx.fillStyle='#5fbe92';cx.fillRect(ox-4,oy-7,8,14);           // el robot
  cx.strokeStyle='#5fbe92';cx.beginPath();cx.moveTo(ox,oy);cx.lineTo(ox,oy-22);cx.stroke();
}
// Candidatos a poste que ve el LiDAR ahora mismo: circulos huecos.
function dibujaCandidatos(cl){
  if(!cl) return;
  const W=cv.width,H=cv.height,ox=W/2,oy=H/2,k=(Math.min(W,H)/2-14)/ALCANCE;
  cx.strokeStyle='#94a3b8';cx.lineWidth=1;
  for(const [x,y] of cl){
    cx.beginPath();cx.arc(ox+x*k,oy-y*k,6,0,7);cx.stroke();
  }
}
// LA TRAYECTORIA: el arco que el robot va a describir con el comando
// actual (solido) y el que pedia la trayectoria antes del arbitraje
// (punteado). El arco se traza desde el EJE TRASERO, que esta lidar_x mm
// por detras del origen del barrido, que es donde va el 0,0 del dibujo.
function arco(radio, izquierda, lidarX, largo){
  const pts=[]; const N=26;
  for(let i=0;i<=N;i++){
    const sArc=largo*i/N;
    let x,y;
    if(radio===null||radio===undefined){ x=0; y=sArc; }
    else{ const th=sArc/radio; x=(izquierda?-1:1)*radio*(1-Math.cos(th)); y=radio*Math.sin(th); }
    pts.push([x, y-lidarX]);          // a marco LiDAR
  }
  return pts;
}
function dibujaTrayectoria(e){
  if(!e) return;
  const W=cv.width,H=cv.height,ox=W/2,oy=H/2,k=(Math.min(W,H)/2-14)/ALCANCE;
  const lx=e.lidar_x||128;
  const traza=(r,ang,color,dash,ancho)=>{
    const p=arco(r, ang>0, lx, 900);
    cx.save(); cx.strokeStyle=color; cx.lineWidth=ancho; cx.setLineDash(dash);
    cx.beginPath();
    p.forEach(([x,y],i)=>{const px=ox+x*k, py=oy-y*k; i?cx.lineTo(px,py):cx.moveTo(px,py);});
    cx.stroke(); cx.restore();
  };
  traza(e.radio_deseado, e.cmd_deseado, '#94a3b8', [4,4], 1.5);   // deseado
  traza(e.radio,         e.angulo,      '#5fbe92', [],    2.5);   // ejecutado
}
// LA LINEA AL PILAR: del robot al poste que se esta siguiendo, del color
// que la camara reconocio. Se mantiene mientras el objetivo siga vivo, y
// pasa a discontinua en cuanto el invariante lo da por rebasado.
function dibujaPilar(e){
  if(!e || !e.trk_activo) return;
  const W=cv.width,H=cv.height,ox=W/2,oy=H/2,k=(Math.min(W,H)/2-14)/ALCANCE;
  const x=ox+e.trk_x*k, y=oy-e.trk_y*k;
  const col = e.trk_color==='ROJO' ? '#f0564a' : (e.trk_color==='VERDE' ? '#3fd08a' : '#e9954f');
  cx.save();
  cx.strokeStyle=col; cx.lineWidth=2.5;
  cx.setLineDash(e.paso_ok ? [6,5] : []);
  cx.beginPath(); cx.moveTo(ox,oy); cx.lineTo(x,y); cx.stroke();
  cx.setLineDash([]);
  cx.fillStyle=col; cx.beginPath(); cx.arc(x,y,8,0,7); cx.fill();
  // incertidumbre de la estimacion
  cx.globalAlpha=0.30; cx.beginPath(); cx.arc(x,y,Math.max(6,e.trk_sigma*k),0,7); cx.stroke();
  cx.globalAlpha=1;
  cx.fillStyle=col; cx.font='12px ui-monospace,monospace';
  const d=Math.round(Math.hypot(e.trk_x,e.trk_y));
  cx.fillText((e.trk_color||'?')+'  '+d+' mm'+(e.paso_ok?'  REBASADO':''), x+13, y-9);
  cx.restore();
}
const FILAS=[['estado','estado'],['seguridad','seguridad'],['velocidad','velocidad'],
 ['angulo','ángulo'],['cmd_deseado','deseado'],['frontal','frontal'],
 ['frontal_muro','frontal muro'],['izquierda','izquierda'],['derecha','derecha'],
 ['trasera','trasera (US)'],['angulo_muro','áng. muro'],['muro_ok','muro ok'],
 ['n_seguros','n seguros'],['n_compatibles','n compatibles'],['fuente_lado','fuente lado'],
 ['sentido','sentido'],['por_lineas','por líneas'],['trk_color','pilar'],
 ['trk_x','pilar x'],['trk_y','pilar y'],['trk_sigma','sigma'],['paso_ok','rebasado'],
 ['rama','rama']];
async function tic(){
  try{
    const t0=performance.now();
    const r=await fetch('/estado?'+Date.now()); const j=await r.json();
    document.getElementById('lat').textContent=Math.round(performance.now()-t0)+' ms';
    const e=j.estado||{};
    document.getElementById('fase').textContent=e.fase||'';
    document.getElementById('estado').textContent=e.estado||'';
    const s=document.getElementById('seg');
    s.textContent=e.seguridad||''; s.className='pill '+(e.seguridad||'');
    dibuja(j.perfil);
    dibujaCandidatos(e.clusters);
    dibujaTrayectoria(e);
    dibujaPilar(e);
    document.getElementById('t').innerHTML=FILAS.map(([k,n])=>{
      let v=e[k]; if(v===true)v='sí'; if(v===false)v='no'; if(v===undefined)v='';
      return '<tr><td class="k">'+n+'</td><td>'+v+'</td></tr>';}).join('');
  }catch(err){document.getElementById('lat').textContent='sin conexión';}
  document.getElementById('cam').src='/camara?'+Date.now();
  setTimeout(tic,200);
}
tic();
</script>
"""
