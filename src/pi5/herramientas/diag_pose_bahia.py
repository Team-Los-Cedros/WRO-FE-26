"""C4: que ven los sensores con el robot YA COLOCADO dentro de la bahia.

No mueve nada. Ejercita los MISMOS caminos de codigo que la FSM de parqueo
(PercepcionLidar.procesar y los helpers de ControlEstacionamiento) para poder
contestar la unica pregunta que decide si la maniobra puede cerrar por medida:
en esta pose, la lateral y la trasera que la FSM quiere usar para cortar cada
tramo, EXISTEN?
"""
import sys, time, threading, statistics, json
from pathlib import Path
RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ))
import serial
from comun.lidar_driver import LidarDriver
from ronda_nueva.percepcion_lidar import PercepcionLidar
from ronda_nueva.estacionamiento import ControlEstacionamiento

CFG = json.load(open(RAIZ / "ronda_nueva" / "configuracion.json", encoding="utf-8"))
_lock = threading.Lock(); _ult = None

def cb(scan, ts):
    global _ult
    with _lock: _ult = (list(scan), ts)

class Pico:
    def __init__(self, p="/dev/ttyACM0"):
        self.ser = serial.Serial(p, 115200, timeout=0.2)
        self.us = None; self.yaw = None; self.seguir = True
        threading.Thread(target=self._leer, daemon=True).start()
    def _leer(self):
        while self.seguir:
            try: l = self.ser.readline().decode("ascii","ignore").strip()
            except Exception: continue
            for c in l.split(","):
                if c.startswith("US:"):
                    try: self.us = float(c[3:])
                    except ValueError: pass
                elif c.startswith("IMU:"):
                    try: self.yaw = float(c[4:])
                    except ValueError: pass

def recta(r, nombre):
    if r is None: return f"  {nombre:11s} NO ENCONTRADA"
    return (f"  {nombre:11s} {r.distancia_mm:7.1f} mm  normal {r.angulo_deg:+7.1f} deg  "
            f"residuo {r.residuo_mm:5.1f}  {r.puntos:3d} pts  calidad {r.calidad:.2f}")

def main():
    seguir = threading.Event(); seguir.set()
    lidar = LidarDriver(CFG["hardware"]["lidar_port"],
                        int(CFG["hardware"].get("lidar_baudrate", 460800)))
    threading.Thread(target=lidar.hilo_lectura, args=(seguir.is_set, cb), daemon=True).start()
    t0 = time.monotonic()
    while _ult is None and time.monotonic() - t0 < 25: time.sleep(0.5)
    if _ult is None: print("[-] LiDAR sin datos"); return 1
    pico = Pico(); time.sleep(2.0)

    percep = PercepcionLidar(CFG)
    ctrl = ControlEstacionamiento(CFG)

    # varias muestras: lo que importa es si la medida es ESTABLE, no un valor suelto
    muestras = {"lat_izq": [], "lat_der": [], "trasera_lidar": [], "us": [],
                "frontal": [], "huecos": 0}
    ultima = None
    for i in range(12):
        with _lock: scan, ts = _ult
        paredes, objetos, hueco = percep.procesar(scan, ts, 0.0, lado_parqueo=-1)
        ultima = paredes
        if paredes.izquierda: muestras["lat_izq"].append(paredes.izquierda.distancia_mm)
        if paredes.derecha:   muestras["lat_der"].append(paredes.derecha.distancia_mm)
        if paredes.trasera:   muestras["trasera_lidar"].append(paredes.trasera.distancia_mm)
        if paredes.frontal:   muestras["frontal"].append(paredes.frontal.distancia_mm)
        if pico.us is not None: muestras["us"].append(pico.us)
        if hueco is not None: muestras["huecos"] += 1
        time.sleep(0.35)

    print("\n=== PAREDES EN ESTA POSE (ultimo barrido) ===")
    for nombre, r in (("frontal", ultima.frontal), ("trasera", ultima.trasera),
                      ("izquierda", ultima.izquierda), ("derecha", ultima.derecha)):
        print(recta(r, nombre))
    print(f"  minimos      frontal {ultima.frontal_min_mm:7.1f}  trasera {ultima.trasera_min_mm:7.1f}"
          f"  izq {ultima.izquierda_min_mm:7.1f}  der {ultima.derecha_min_mm:7.1f}")
    print(f"  corredor     {ultima.corredor_mm:.0f} mm a {ultima.corredor_deg:.0f} deg")

    print("\n=== ESTABILIDAD EN 12 BARRIDOS ===")
    for k in ("frontal", "lat_izq", "lat_der", "trasera_lidar", "us"):
        v = muestras[k]
        if not v: print(f"  {k:14s} SIN DATO en los 12 barridos"); continue
        print(f"  {k:14s} n={len(v):2d}/12  mediana {statistics.median(v):7.1f}  "
              f"min {min(v):7.1f}  max {max(v):7.1f}  disp {max(v)-min(v):6.1f}")
    print(f"  hueco detectado en {muestras['huecos']}/12 barridos")

    print("\n=== LO QUE VERIA LA FSM DE PARQUEO ===")
    for lado, etiqueta in ((-1, "bahia a la IZQUIERDA"), (1, "bahia a la DERECHA")):
        ctrl._lado = lado
        lat = ctrl._lateral_mm(ultima)
        tra = ctrl._trasera_mm(ultima, pico.us)
        par = ctrl._paralelo(ultima)
        print(f"  {etiqueta}:")
        print(f"     _lateral_mm  = {lat if lat is None else round(lat,1)}")
        print(f"     _trasera_mm  = {tra if tra is None else round(tra,1)}   "
              f"(ultrasonido crudo {pico.us})")
        print(f"     _paralelo    = {par if par is None else round(par,2)} deg")
        dentro = ctrl._dentro(lat, par)
        print(f"     _dentro()    = {dentro}   "
              f"[exige lateral <= {ctrl.lateral_dentro_mm:.0f} y |paralelo| <= "
              f"{ctrl.tolerancia_paralelo_deg:.0f}]")
    print(f"\n  rumbo IMU {pico.yaw}")
    pico.seguir = False; seguir.clear(); time.sleep(0.3); lidar.cerrar()
    return 0

if __name__ == "__main__":
    sys.exit(main())
