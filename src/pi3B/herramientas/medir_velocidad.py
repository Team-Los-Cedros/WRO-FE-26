# Curva velocidad real (mm/s) contra consigna de PWM (tarea 5). Aunque
# el LiDAR sea el odometro en carrera, el tracker necesita un modelo de
# prediccion: tracker.MM_POR_SEG_A_PWM100 = 900.0 es una suposicion sin
# medir, y de ella depende cuanto "retrocede" el poste en el marco del
# robot entre barridos.
#
# Metodo: el propio LiDAR es la cinta metrica. Se coloca el robot
# apuntando de frente a un muro recto a ~2.5m, se manda un escalon de
# PWM y se muestrea la distancia frontal contra el tiempo. La pendiente
# de la recta ajustada sobre el tramo estacionario es la velocidad. Se
# frena solo al llegar a DIST_PARADA.
#
# Ventajas sobre cronometrar 2m a mano: mide tambien la rampa de
# arranque (TAU), no depende del pulso del cronometrista, y da una
# medida por cada barrido en vez de dos puntos.
#
# La bateria importa: haz la tanda completa con bateria recien cargada y
# repitela cuando lleve ~15 min de uso, con etiquetas distintas.
#
# Uso (en la Pi, robot frente a un muro con 2.5m libres):
#   python3 medir_velocidad.py bateria_llena
#   python3 medir_velocidad.py bateria_baja
import json
import os
import sys
import time
import threading

from lidar_driver import LidarDriver
from lidar_geometria import construir_perfil_360, distancia_en_rango

CARPETA_SALIDA = "mediciones"

PWMS          = [40, 55, 70, 90]
DIST_ARRANQUE = 2300.0    # mm, no empieza a medir hasta estar mas lejos que esto
DIST_PARADA   = 500.0     # mm, frena aqui
TIMEOUT_TIRADA = 12.0     # s
SECTOR_FRONTAL = (350.0, 10.0)

# Se descartan los primeros DESCARTE_INICIAL segundos de cada tirada: ahi
# el motor todavia esta acelerando y la pendiente no es la de regimen.
DESCARTE_INICIAL = 0.7
FRACCION_DESCARTE = 0.35   # ademas, el primer 35% de cada tirada


class MedidorVelocidad:
    def __init__(self):
        self.corriendo = True
        self._lock = threading.Lock()
        self._frontal = None
        self._t_frontal = 0.0
        self._grabando = False
        self._muestras = []
        self._t0 = 0.0

    def al_barrido(self, scan):
        perfil = construir_perfil_360(scan)
        d = distancia_en_rango(perfil, *SECTOR_FRONTAL)
        with self._lock:
            self._frontal = d
            self._t_frontal = time.time()
            if self._grabando:
                self._muestras.append([round(time.time() - self._t0, 4), round(d, 1)])

    def frontal(self):
        with self._lock:
            return self._frontal

    def iniciar_grabacion(self):
        with self._lock:
            self._muestras = []
            self._t0 = time.time()
            self._grabando = True

    def terminar_grabacion(self):
        with self._lock:
            self._grabando = False
            return list(self._muestras)


def _ajustar_recta(muestras):
    # Minimos cuadrados sobre (t, distancia). La velocidad es -pendiente
    # porque la distancia al muro DECRECE al avanzar.
    n = len(muestras)
    if n < 4:
        return None, None
    st = sum(m[0] for m in muestras)
    sd = sum(m[1] for m in muestras)
    stt = sum(m[0] ** 2 for m in muestras)
    std = sum(m[0] * m[1] for m in muestras)
    den = n * stt - st * st
    if abs(den) < 1e-9:
        return None, None
    pendiente = (n * std - st * sd) / den
    ordenada = (sd - pendiente * st) / n

    # RMS del residuo: si es alto, o el muro no estaba recto o el robot
    # se fue de lado y el sector frontal engancho otra cosa.
    rms = (sum((d - (pendiente * t + ordenada)) ** 2 for t, d in muestras) / n) ** 0.5
    return -pendiente, rms


def _tau_arranque(muestras, v_regimen):
    # Tiempo hasta alcanzar el 90% de la velocidad de regimen, estimado
    # por diferencias finitas sobre una ventana movil de 3 muestras.
    if v_regimen is None or v_regimen <= 0 or len(muestras) < 6:
        return None
    for i in range(2, len(muestras)):
        dt = muestras[i][0] - muestras[i - 2][0]
        if dt <= 0:
            continue
        v = -(muestras[i][1] - muestras[i - 2][1]) / dt
        if v >= 0.9 * v_regimen:
            return round(muestras[i][0], 3)
    return None


def una_tirada(medidor, enlace, pwm):
    print(f"\n  --- PWM {pwm}% ---")
    print("  Coloca el robot apuntando al muro, a mas de "
          f"{DIST_ARRANQUE:.0f}mm, alineado de frente.")

    while True:
        d = medidor.frontal()
        if d is None:
            print("    esperando barridos del LiDAR...", end="\r")
            time.sleep(0.3)
            continue
        print(f"    frontal = {d:7.0f}mm ", end="\r")
        if d >= DIST_ARRANQUE:
            break
        time.sleep(0.2)

    input(f"\n  Listo a {medidor.frontal():.0f}mm. Enter para lanzar (Ctrl+C aborta)...")

    medidor.iniciar_grabacion()
    enlace.enviar(pwm, 0.0)
    t0 = time.time()
    try:
        while time.time() - t0 < TIMEOUT_TIRADA:
            time.sleep(0.02)
            d = medidor.frontal()
            if d is not None and d <= DIST_PARADA:
                break
    finally:
        enlace.enviar(0, 0.0)
    muestras = medidor.terminar_grabacion()

    # El ajuste solo usa el tramo de regimen: se descarta lo primero de
    # la tirada (rampa del motor). Un descarte fijo no basta porque a PWM
    # bajo la tirada dura mas, asi que se toma tambien una fraccion.
    t_max = muestras[-1][0] if muestras else 0.0
    t_desde = max(DESCARTE_INICIAL, FRACCION_DESCARTE * t_max)
    utiles = [m for m in muestras
              if m[0] >= t_desde and DIST_PARADA <= m[1] <= DIST_ARRANQUE + 200]
    v, rms = _ajustar_recta(utiles)
    tau = _tau_arranque(muestras, v)

    if v is None:
        print("  [-] Muestras insuficientes. Repite la tirada.")
    else:
        print(f"  velocidad = {v:.0f} mm/s   (residuo RMS {rms:.0f}mm sobre "
              f"{len(utiles)} barridos)")
        if rms > 25.0:
            print("  [!] Residuo alto: el robot probablemente no fue recto, o el "
                  "sector frontal engancho una arista. Repite.")
        if tau:
            print(f"  t hasta el 90% de la velocidad de regimen: {tau:.2f}s")

    return {
        "pwm": pwm,
        "velocidad_mm_s": round(v, 1) if v else None,
        "residuo_rms_mm": round(rms, 1) if rms else None,
        "t_90pct_s": tau,
        "n_muestras": len(utiles),
        "muestras_t_distancia": muestras,
    }


def main():
    etiqueta = sys.argv[1] if len(sys.argv) > 1 else "sin_etiqueta"
    if etiqueta not in ("bateria_llena", "bateria_baja"):
        print(f"[!] Etiqueta '{etiqueta}'. Lo normal es bateria_llena o bateria_baja.")

    from enlace_pico import EnlacePico
    try:
        enlace = EnlacePico()
    except Exception as e:
        print(f"[-] No se pudo abrir el enlace con la Pico: {e}")
        sys.exit(1)
    enlace.enviar(0, 0.0)

    medidor = MedidorVelocidad()
    lidar = LidarDriver()
    threading.Thread(target=lidar.hilo_lectura,
                     args=(lambda: medidor.corriendo, medidor.al_barrido),
                     daemon=True).start()

    print(f"=== Velocidad vs PWM ({etiqueta}) ===")
    print("Necesitas un muro recto y ~2.5m libres al frente.")

    resultados = []
    try:
        for pwm in PWMS:
            resultados.append(una_tirada(medidor, enlace, pwm))
    except KeyboardInterrupt:
        print("\n[!] Interrumpido.")
    finally:
        enlace.enviar(0, 0.0)
        medidor.corriendo = False
        time.sleep(0.3)
        lidar.cerrar()
        enlace.cerrar()

    print("\n=== Tabla ===")
    print("  PWM   mm/s    t90%")
    for r in resultados:
        v = f"{r['velocidad_mm_s']:6.0f}" if r["velocidad_mm_s"] else "     -"
        t = f"{r['t_90pct_s']:5.2f}s" if r["t_90pct_s"] else "     -"
        print(f"  {r['pwm']:3d}  {v}  {t}")
    print("\nPega estos numeros en VELOCIDAD_MM_S de comun/geometria_robot.py")

    os.makedirs(CARPETA_SALIDA, exist_ok=True)
    ruta = os.path.join(CARPETA_SALIDA, f"velocidad_{etiqueta}.json")
    with open(ruta, "w") as f:
        json.dump({"etiqueta": etiqueta, "tiradas": resultados}, f, indent=2)
    print(f"[+] Guardado en {ruta}")


if __name__ == "__main__":
    main()
