# Radiografia completa de un barrido del C1, con el robot QUIETO.
# No mueve nada: solo abre el LiDAR, acumula N barridos y contesta tres
# preguntas que los CSV del 09-09 dejaron abiertas.
#   1. Que grados ve el robot su PROPIA estructura, y a que distancia.
#   2. Que valores entregan los sectores que consume navegacion.py,
#      antes y despues de lidar_mascara.
#   3. Cuanto vale angulo_muro (si la pose es conocida, es su verdad).
import sys, time, math, statistics
sys.path.insert(0, "/home/pi/ronda_curvas")
import lidar_geometria as lg
import lidar_mascara as msk
from lidar_driver import LidarDriver

N = int(sys.argv[1]) if len(sys.argv) > 1 else 25
barridos = []

def al_barrido(scan):
    barridos.append(scan)

drv = LidarDriver()
import threading
corriendo = [True]
h = threading.Thread(target=drv.hilo_lectura, args=(lambda: corriendo[0], al_barrido), daemon=True)
h.start()
t0 = time.time()
while len(barridos) < N and time.time() - t0 < 25:
    time.sleep(0.05)
corriendo[0] = False
time.sleep(0.4)
try: drv.cerrar()
except Exception: pass

if not barridos:
    print("[-] Sin barridos. LiDAR conectado en /dev/ttyUSB0?"); sys.exit(1)
print("[+] %d barridos, %d puntos de mediana\n" % (len(barridos),
      statistics.median([len(b) for b in barridos])))

perfiles = [lg.construir_perfil_360(b) for b in barridos]

# ---------- 1. estructura propia ----------
print("=== ECOS DE ESTRUCTURA (grados con eco por debajo de 250 mm) ===")
print("  grado  tasa_eco  d_mediana  dispersion")
tramos = []
for i in range(360):
    ds = [p[i] for p in perfiles if p[i] < 250.0]
    tasa = len(ds) / len(perfiles)
    if tasa >= 0.25:
        disp = (max(ds) - min(ds)) if len(ds) > 1 else 0.0
        tramos.append((i, tasa, statistics.median(ds), disp))
if not tramos:
    print("  (ninguno)")
else:
    for i, tasa, med, disp in tramos:
        print("   %3d    %5.0f%%    %6.1f mm   %5.1f mm" % (i, 100 * tasa, med, disp))
    gs = [t[0] for t in tramos]
    # agrupar en rangos contiguos
    rangos, ini, prev = [], gs[0], gs[0]
    for g in gs[1:]:
        if g - prev > 2:
            rangos.append((ini, prev)); ini = g
        prev = g
    rangos.append((ini, prev))
    print("\n  RANGOS A CEGAR: %s" % ", ".join("[%d, %d]" % r for r in rangos))
    print("  lidar_mascara actual: [%d, %d]" % (msk.MASTIL_MIN, msk.MASTIL_MAX))

print("\n=== AUTOCHEQUEO DE LA MASCARA (msk.diagnosticar) ===")
ok, msj = msk.diagnosticar(perfiles[len(perfiles)//2])
print("  [%s] %s" % ("OK" if ok else "X", msj))

# ---------- 2. sectores que consume la navegacion ----------
proc = lg.ProcesadorLidar()
meds = [proc.procesar(b) for b in barridos]
def col(f): return [f(m) for m in meds]
print("\n=== SECTORES, SIN mascara (lo que hace lidar_geometria) ===")
for nom, f in (("frontal", lambda m: m.frontal), ("frontal_muro", lambda m: m.frontal_muro),
               ("izquierda", lambda m: m.izquierda), ("derecha", lambda m: m.derecha),
               ("trasera", lambda m: m.trasera),
               ("trasera_izq", lambda m: m.trasera_izquierda),
               ("trasera_der", lambda m: m.trasera_derecha),
               ("angulo_muro", lambda m: m.angulo_muro)):
    v = col(f)
    print("  %-13s med=%8.1f  min=%8.1f  max=%8.1f" % (nom, statistics.median(v), min(v), max(v)))

print("\n=== SECTORES, CON mascara (lo que ve navegacion via ronda_camara) ===")
meds2 = [msk.aplicar(proc.procesar(b)) for b in barridos]
for nom, f in (("trasera", lambda m: m.trasera),
               ("trasera_izq", lambda m: m.trasera_izquierda),
               ("trasera_der", lambda m: m.trasera_derecha)):
    v = [f(m) for m in meds2]
    print("  %-13s med=%8.1f  min=%8.1f  max=%8.1f" % (nom, statistics.median(v), min(v), max(v)))
print("\n  EMERGENCIA_TRASERA = 250 mm -> RETROCESO sale en su 1er ciclo si trasera < 250")

# ---------- 3. perfil resumido cada 10 grados ----------
print("\n=== PERFIL 360 (mediana por cada 10 grados, mm; '----' = sin eco) ===")
for base in range(0, 360, 60):
    fila = []
    for i in range(base, base + 60, 10):
        ds = [p[i] for p in perfiles if p[i] < 7999.0]
        fila.append("%4.0f" % statistics.median(ds) if ds else "----")
    print("  %3d-%3d: %s" % (base, base + 59, "  ".join(fila)))
