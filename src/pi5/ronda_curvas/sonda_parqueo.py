# Que ve el robot DESDE DENTRO de la plaza de parqueo. No manda nada al
# motor: abre LiDAR + Pico y describe la geometria.
#
# Lo que hace falta saber para el parqueo, y que el reglamento NO da
# (porque depende de como quede el robot al colocarlo):
#   - a que distancia esta cada muro magenta y en que rumbo
#   - a que distancia el muro exterior (el fondo de la plaza)
#   - por donde esta el hueco: la salida al carril
#   - cuanto se desvia el chasis de estar paralelo a esos muros
import sys, time, threading, math
sys.path.insert(0, "/home/pi/ronda_curvas")
import lidar_geometria as lg
import geometria_robot as geo
from lidar_driver import LidarDriver
from enlace_pico import EnlacePico

N = int(sys.argv[1]) if len(sys.argv) > 1 else 12
enlace = EnlacePico(); time.sleep(2.2)
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

# Mediana por bin sobre todos los barridos: quita el eco suelto.
perfiles = [lg.construir_perfil_360(b) for b in barridos]
perfil = []
for i in range(360):
    v = sorted(p[i] for p in perfiles)
    perfil.append(v[len(v)//2])

print("=== PERFIL (mm por rumbo; 0=frente, 90=derecha, 270=izquierda) ===")
for base in range(0, 360, 30):
    fila = "  %3d-%3d |" % (base, base+29)
    for a in range(base, base+30, 2):
        d = perfil[a]
        fila += ("%6.0f" % d) if d < 4000 else "     ."
    print(fila)

print()
print("=== RUMBOS CLAVE ===")
for etq, a in (("frente", 0), ("der 45", 45), ("derecha", 90), ("der-tras", 135),
               ("izq-tras", 225), ("izquierda", 270), ("izq 45", 315)):
    print("  %-9s %3d deg -> %s" % (etq, a,
          "%6.0f mm" % perfil[a] if perfil[a] < 4000 else "sin eco"))
us = enlace.ultrasonido_mm()
print("  trasera (ultrasonido)   -> %s" % ("%6.0f mm" % us if us else "sin medida"))

print()
print("=== SALIDA: por donde hay hueco (sectores de 30 grados) ===")
for base in range(0, 360, 30):
    vals = [perfil[(base+k) % 360] for k in range(30)]
    vis = [v for v in vals if v < 4000]
    if not vis:
        print("  %3d-%3d  LIBRE (sin eco en todo el sector)" % (base, base+29))
    else:
        print("  %3d-%3d  min %5.0f  mediana %5.0f  ecos %2d/30" % (
            base, base+29, min(vis), sorted(vis)[len(vis)//2], len(vis)))

print()
print("=== PAREDES por ajuste PCA (angulo del chasis respecto a cada una) ===")
bins_estrechos = set()
for nombre, sector in (("derecha", lg.SECTOR_MURO_DER), ("izquierda", lg.SECTOR_MURO_IZQ)):
    pts = lg.puntos_de_sector(perfil, sector[0], sector[1], excluidos=bins_estrechos)
    ang = lg.angulo_de_pared(pts)
    print("  muro %-9s sector %s  puntos %2d  angulo %s"
          % (nombre, sector, len(pts),
             "%+6.1f deg" % ang if ang is not None else "no es una pared plana"))

print()
print("=== OBJETOS ESTRECHOS (los muros magenta son 200x20mm) ===")
med = lg.procesar(barridos[len(barridos)//2])
for c in med.clusters_estrechos:
    cx, cy = lg.centroide_xy_cluster(c)
    print("  en (%+6.0f, %+6.0f) mm  rumbo %+6.1f  dist %5.0f  ancho %5.0f mm"
          % (cx, cy, math.degrees(math.atan2(cx, cy)), math.hypot(cx, cy),
             lg.ancho_cluster(c)))
print()
print("robot: %.0f x %.0f mm | plaza segun reglamento: %.0f x %.0f mm"
      % (geo.LARGO_ROBOT, geo.ANCHO_ROBOT, geo.LARGO_PARQUEO, geo.PROFUNDIDAD_PARQUEO))
enlace.cerrar()
