# Que ve la FSM AHORA MISMO, con el robot quieto y sin mover nada.
# No manda consignas al motor: solo abre LiDAR + Pico y mira.
import sys, time, threading, statistics
sys.path.insert(0, "/home/pi/ronda_curvas")
import lidar_geometria as lg, lidar_mascara as msk, navegacion as nav
from lidar_driver import LidarDriver
from enlace_pico import EnlacePico

N = int(sys.argv[1]) if len(sys.argv) > 1 else 15
enlace = EnlacePico()
time.sleep(2.2)

barridos = []
drv = LidarDriver(); corr = [True]
threading.Thread(target=drv.hilo_lectura, args=(lambda: corr[0], barridos.append),
                 daemon=True).start()
t0 = time.time()
while len(barridos) < N and time.time() - t0 < 25:
    time.sleep(0.05)
corr[0] = False; time.sleep(0.4)

if not barridos:
    print("[-] sin barridos"); sys.exit(1)

proc = lg.ProcesadorLidar()
nvg = nav.Navegador(control_sector=proc)
ok, msj = msk.diagnosticar(lg.construir_perfil_360(barridos[len(barridos)//2]))
print("MASCARA: [%s] %s\n" % ("OK" if ok else "X", msj))

print("US de la Pico: %s mm   |   heading %.2f   |   WD %s"
      % (enlace.ultrasonido_mm(), enlace.heading(),
         "OK" if enlace.watchdog_ok() else "STOP"))
print()
print(" fm     izq    der   ang_muro  valido   trasera(US)  n_seguros  seguridad")
for b in barridos[:12]:
    med = msk.aplicar(proc.procesar(b))
    us = enlace.ultrasonido_mm()
    if us is not None:
        med.trasera = us
    seguros, _ = nvg._comandos_seguros(med, 55)
    cmd, hay = nvg._arbitrar(nvg._rumbo_nominal(med), med, 55)
    print("%6.0f %6.0f %6.0f   %+7.2f    %-6s  %9.0f  %6d      %s"
          % (med.frontal_muro, med.izquierda, med.derecha, med.angulo_muro,
             med.muro_valido, med.trasera, len(seguros), nvg.seguridad))
enlace.cerrar(); drv.cerrar()
