"""C1: mm/s por PWM, medido con el propio LiDAR.

DISENO. Una sola corrida no sirve: la distancia recorrida es v*T + c, donde c
mezcla el arranque del motor y la inercia de frenado. Con tres duraciones y una
recta por minimos cuadrados, la PENDIENTE es la velocidad limpia y c queda
aparte. Ademas se alterna adelante/reversa para volver al punto de partida y
medir los dos sentidos, que no tienen por que coincidir.

SEGURIDAD. Aborta si el frontal baja de FRENO_FRONTAL_MM o el ultrasonido
trasero de FRENO_TRASERO_MM, y siempre manda detener() al salir.
"""
import sys, time, threading, statistics, math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import serial
from comun.lidar_driver import LidarDriver

PWM             = 22
DURACIONES      = (2.0, 3.0, 4.0)
FRENO_FRONTAL_MM = 450.0
FRENO_TRASERO_MM = 400.0
SECTOR_DEG      = 8.0
PERIODO_S       = 0.05

_lock = threading.Lock()
_ultimo = []

def al_barrido(scan, timestamp):
    global _ultimo
    with _lock:
        _ultimo = list(scan)

def frontal_mm():
    with _lock:
        pts = list(_ultimo)
    d = [dist for ang, dist in pts
         if dist > 0 and (ang <= SECTOR_DEG or ang >= 360.0 - SECTOR_DEG)]
    return statistics.median(d) if len(d) >= 5 else None

def medir_frontal(seg=1.2):
    t0, muestras = time.monotonic(), []
    while time.monotonic() - t0 < seg:
        v = frontal_mm()
        if v is not None: muestras.append(v)
        time.sleep(0.05)
    return statistics.median(muestras) if len(muestras) >= 5 else None

class Pico:
    def __init__(self, puerto="/dev/ttyACM0"):
        self.ser = serial.Serial(puerto, 115200, timeout=0.2)
        self.us = None
        self.seguir = True
        threading.Thread(target=self._leer, daemon=True).start()
    def _leer(self):
        while self.seguir:
            try:
                linea = self.ser.readline().decode("ascii", "ignore").strip()
            except Exception:
                continue
            for campo in linea.split(","):
                if campo.startswith("US:"):
                    try: self.us = float(campo[3:])
                    except ValueError: pass
    def enviar(self, v, a=0.0):
        self.ser.write(f"{int(v)},{float(a):.2f}\n".encode("ascii"))
    def detener(self):
        for _ in range(5):
            self.enviar(0, 0.0); time.sleep(0.01)

def corrida(pico, pwm, segundos):
    """Devuelve (recorrido_mm, duracion_real_s) o (None, motivo)."""
    d0 = medir_frontal()
    if d0 is None: return None, "sin lectura frontal inicial"
    t_ini = time.monotonic()
    while time.monotonic() - t_ini < segundos:
        f, r = frontal_mm(), pico.us
        if pwm > 0 and f is not None and f < FRENO_FRONTAL_MM:
            pico.detener(); return None, f"ABORTA: frontal {f:.0f} mm"
        if pwm < 0 and r is not None and r < FRENO_TRASERO_MM:
            pico.detener(); return None, f"ABORTA: trasero {r:.0f} mm"
        pico.enviar(pwm, 0.0)
        time.sleep(PERIODO_S)
    t_fin = time.monotonic()
    pico.detener()
    time.sleep(1.2)                      # que termine de frenar
    d1 = medir_frontal()
    if d1 is None: return None, "sin lectura frontal final"
    return (d0 - d1) if pwm > 0 else (d1 - d0), t_fin - t_ini

def ajuste(pares):
    """Minimos cuadrados d = v*T + c. Devuelve (v_mm_s, c_mm)."""
    n = len(pares)
    sx = sum(t for t, _ in pares); sy = sum(d for _, d in pares)
    sxx = sum(t*t for t, _ in pares); sxy = sum(t*d for t, d in pares)
    den = n*sxx - sx*sx
    if abs(den) < 1e-9: return None, None
    v = (n*sxy - sx*sy)/den
    return v, (sy - v*sx)/n

def main():
    seguir = threading.Event(); seguir.set()
    lidar = LidarDriver()
    threading.Thread(target=lidar.hilo_lectura,
                     args=(seguir.is_set, al_barrido), daemon=True).start()
    # El RPLIDAR tarda: 0 barridos a los 4 s, 40 a los 8. Esperar por dato,
    # no por reloj.
    t0 = time.monotonic()
    while frontal_mm() is None and time.monotonic() - t0 < 25.0:
        time.sleep(0.5)
    if frontal_mm() is None:
        print("[-] el LiDAR no entrega frente tras 25 s"); return 1
    print(f"[i] LiDAR listo en {time.monotonic()-t0:.1f} s")
    pico = Pico()
    time.sleep(1.0)
    print(f"[i] frontal inicial {medir_frontal():.0f} mm | trasero(US) {pico.us}")
    print(f"[i] PWM {PWM}, duraciones {DURACIONES}\n")
    resultados = {"adelante": [], "reversa": []}
    try:
        for T in DURACIONES:
            for sentido, pwm in (("adelante", PWM), ("reversa", -PWM)):
                d, info = corrida(pico, pwm, T)
                if d is None:
                    print(f"  {sentido:9s} T={T:.1f}s  ->  {info}")
                    continue
                resultados[sentido].append((info, d))
                print(f"  {sentido:9s} T={T:.1f}s  recorrido {d:7.1f} mm  "
                      f"({d/info:6.1f} mm/s bruto)")
    finally:
        pico.detener(); pico.seguir = False
        seguir.clear(); time.sleep(0.3); lidar.cerrar()
    print()
    for sentido, pares in resultados.items():
        if len(pares) < 2:
            print(f"{sentido}: datos insuficientes"); continue
        v, c = ajuste(pares)
        print(f"{sentido:9s}: v = {v:6.1f} mm/s   offset {c:+6.1f} mm   "
              f"-> mm_s_per_pwm = {v/PWM:.2f}")
    print(f"\n(el JSON tiene fusion.mm_s_per_pwm = 4.0; el README del equipo implica 6.7)")
    return 0

if __name__ == "__main__":
    sys.exit(main())
