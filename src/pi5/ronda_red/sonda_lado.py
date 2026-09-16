# Que lado de esquina decide el robot AHORA MISMO, y con que fuente.
# No mueve nada. Colocar el robot mirando hacia una esquina.
import sys, time, threading
sys.path.insert(0, "/home/pi/ronda_curvas")
import lidar_geometria as lg, navegacion as nav
from lidar_driver import LidarDriver
b=[]; drv=LidarDriver(); c=[True]
threading.Thread(target=drv.hilo_lectura,args=(lambda:c[0],b.append),daemon=True).start()
proc=lg.ProcesadorLidar(); n=nav.Navegador(control_sector=proc)
print("\n  izq   der   fmuro   diag_izq diag_der   LADO   fuente")
t0=time.time(); visto=0
while time.time()-t0 < 24:
    time.sleep(2.0)
    lote=b[visto:]; visto=len(b)
    if not lote: continue
    med=proc.procesar(lote[-1])
    di=lg.distancia_en_rango(med.perfil,300,350); dd=lg.distancia_en_rango(med.perfil,10,60)
    lado=n._preferencia_esquina(med)
    print("%6.0f %5.0f %7.0f %9.0f %8.0f   %-6s %s"
          % (med.izquierda, med.derecha, med.frontal_muro, di, dd,
             ("IZQUIERDA" if lado and lado>0 else "DERECHA") if lado else "-", n.fuente_lado))
c[0]=False; time.sleep(0.3); drv.cerrar()
