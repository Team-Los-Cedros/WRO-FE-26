# Muestreo continuo mientras se coloca el robot a mano. Imprime un
# resumen cada 3 s para poder ver cuando la pose se estabiliza.
import sys, time, threading, statistics
sys.path.insert(0, "/home/pi/ronda_curvas")
import lidar_geometria as lg, lidar_mascara as msk
from lidar_driver import LidarDriver

SEG = int(sys.argv[1]) if len(sys.argv) > 1 else 45
barridos = []
drv = LidarDriver(); corr = [True]
threading.Thread(target=drv.hilo_lectura, args=(lambda: corr[0], barridos.append),
                 daemon=True).start()
proc = lg.ProcesadorLidar()
print("\n   t    izq    der   izq-der   ang_muro  valido   frontal_muro")
t0 = time.time(); visto = 0
while time.time() - t0 < SEG:
    time.sleep(3.0)
    lote = barridos[visto:]
    visto = len(barridos)
    if not lote:
        print("   -- sin barridos --"); continue
    meds = [proc.procesar(b) for b in lote[-8:]]
    f = lambda g: statistics.median([g(m) for m in meds])
    val = sum(1 for m in meds if m.muro_valido)
    print("%4.0f %6.0f %6.0f %8.0f   %+7.2f   %2d/%-2d   %8.0f"
          % (time.time()-t0, f(lambda m: m.izquierda), f(lambda m: m.derecha),
             f(lambda m: m.izquierda - m.derecha), f(lambda m: m.angulo_muro),
             val, len(meds), f(lambda m: m.frontal_muro)))
corr[0] = False; time.sleep(0.3); drv.cerrar()
