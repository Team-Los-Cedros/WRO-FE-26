# Caracterizacion de la direccion (tarea 4). Dos mediciones, ninguna
# necesita cinta metrica: la IMU de la Pico las resuelve sola.
#
#   modo centro : barre comandos de servo cerca de 0 y mide la deriva de
#                 guiñada (grados/s) en marcha recta a velocidad baja.
#                 El comando cuyo |deriva| es minimo es el CENTRO
#                 MECANICO REAL. Que LIMITE_DER=70 y LIMITE_IZQ=115 sean
#                 asimetricos respecto a CENTRO=90 ya sugiere que el
#                 centro esta corrido; esto lo mide en vez de suponerlo.
#
#   modo radio  : fija el servo a un tope, arranca a velocidad baja y
#                 espera a que la IMU acumule 360 grados. Con el tiempo
#                 de vuelta T y la velocidad v (de medir_velocidad.py):
#                     R = v * T / (2*pi)
#                 Se cruza con el diametro medido con cinta.
#
# AVISO IMPORTANTE sobre la Pico: src/pico/main.py aplica
#     angulo_servo = CENTRO + angulo_objetivo - velocidad_z * KD_ESTABILIDAD
# con KD_ESTABILIDAD = 0.12. En un giro estacionario velocidad_z NO es
# cero: a 57 grados/s ese termino resta ~6.9 grados de servo, casi un
# tercio del rango util. Es decir, el angulo de rueda real en curva NO es
# el comandado, y el radio medido saldria contaminado.
#
# No hace falta editar el firmware a mano para esto (y no conviene: subir
# un main.py con KD en 0 y olvidar restaurarlo antes de una ronda es un
# fallo silencioso). La consigna acepta un tercer campo opcional
# "vel,ang,kd" y el modo radio manda kd=0.0, que desactiva la
# amortiguacion solo mientras dura la medicion. El modo centro la deja
# activa porque en recta velocidad_z ~ 0 y no molesta.
#
# Uso (en la Pi, sobre una superficie despejada de al menos 2x2 m):
#   python3 medir_direccion.py centro
#   python3 medir_direccion.py radio izq
#   python3 medir_direccion.py radio der
import json
import os
import sys
import time

from enlace_pico import EnlacePico

CARPETA_SALIDA = "mediciones"

# --- modo centro ---
VEL_CENTRO        = 35          # % PWM, lento pero sin quedarse pegado
COMANDOS_BARRIDO  = [-6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0]
SEG_POR_COMANDO   = 2.0
SEG_ESTABILIZAR   = 0.6         # se descarta este tramo inicial de cada tirada

# --- modo radio ---
VEL_RADIO         = 35
TOPE_IZQ          =  25.0
TOPE_DER          = -20.0
TIMEOUT_VUELTA    = 25.0
GRADOS_VUELTA     = 360.0


def _guardar(nombre, datos):
    os.makedirs(CARPETA_SALIDA, exist_ok=True)
    ruta = os.path.join(CARPETA_SALIDA, nombre)
    with open(ruta, "w") as f:
        json.dump(datos, f, indent=2)
    print(f"[+] Guardado en {ruta}")
    return ruta


def _esperar_imu(enlace, segundos=3.0):
    t_lim = time.time() + segundos
    while time.time() < t_lim:
        if enlace.heading_valido():
            return True
        time.sleep(0.05)
    return False


def modo_centro(enlace):
    print("\n=== CENTRO MECANICO DEL SERVO ===")
    print("El robot va a avanzar en linea recta varias veces, unos 2s cada vez.")
    print("Necesitas ~2m libres al frente. Ctrl+C corta y frena.\n")
    input("Enter para empezar...")

    resultados = []
    for comando in COMANDOS_BARRIDO:
        print(f"  comando {comando:+.1f} grados ... ", end="", flush=True)
        enlace.enviar(0, comando)
        time.sleep(0.4)                        # que el servo llegue antes de rodar

        enlace.enviar(VEL_CENTRO, comando)
        time.sleep(SEG_ESTABILIZAR)

        h0, t0 = enlace.heading(), time.time()
        time.sleep(SEG_POR_COMANDO)
        h1, t1 = enlace.heading(), time.time()

        enlace.enviar(0, 0.0)
        deriva = (h1 - h0) / (t1 - t0)
        resultados.append({"comando": comando, "deriva_grados_s": round(deriva, 3),
                           "delta_heading": round(h1 - h0, 2),
                           "dt": round(t1 - t0, 3)})
        print(f"deriva {deriva:+6.2f} grados/s")

        print("    reposiciona el robot al inicio y dale Enter", end="")
        input("")

    # Recta de minimos cuadrados deriva = m*comando + b  ->  centro = -b/m
    n = len(resultados)
    sx = sum(r["comando"] for r in resultados)
    sy = sum(r["deriva_grados_s"] for r in resultados)
    sxx = sum(r["comando"] ** 2 for r in resultados)
    sxy = sum(r["comando"] * r["deriva_grados_s"] for r in resultados)
    den = n * sxx - sx * sx
    trim = None
    pendiente = None
    if abs(den) > 1e-9:
        pendiente = (n * sxy - sx * sy) / den
        ordenada  = (sy - pendiente * sx) / n
        if abs(pendiente) > 1e-6:
            trim = -ordenada / pendiente

    print("\n--- Resultado ---")
    for r in resultados:
        print(f"  comando {r['comando']:+5.1f}  ->  {r['deriva_grados_s']:+6.2f} grados/s")
    if trim is not None:
        print(f"\n  SERVO_TRIM = {trim:+.2f} grados")
        print(f"  (sumar esto a todo comando para ir recto; en la escala de la "
              f"Pico el centro real es {90 + trim:.1f}, no 90)")
        print(f"  sensibilidad: {pendiente:+.2f} (grados/s de guiñada) por grado de comando")
    else:
        print("\n  [-] No se pudo ajustar la recta: revisa que la IMU este reportando.")

    return _guardar("direccion_centro.json", {
        "velocidad_pwm": VEL_CENTRO,
        "muestras": resultados,
        "servo_trim": round(trim, 3) if trim is not None else None,
        "sensibilidad_grados_s_por_grado": round(pendiente, 3) if pendiente else None,
    })


def modo_radio(enlace, lado):
    tope = TOPE_IZQ if lado == "izq" else TOPE_DER
    print(f"\n=== RADIO DE GIRO, TOPE {lado.upper()} (comando {tope:+.1f}) ===")
    print("El robot va a dar un circulo completo. Necesitas ~1.5x1.5 m libres.")
    print("Marca con cinta adhesiva donde arranca una rueda: al cerrar la vuelta")
    print("mides el DIAMETRO del circulo que dejo y lo comparas con lo calculado.")
    print("\nLa amortiguacion por giroscopio va desactivada (kd=0) durante toda")
    print("la vuelta: con ella activa el angulo de rueda real no seria el")
    print("comandado y el radio saldria contaminado.\n")
    input("Enter para empezar...")

    enlace.enviar(0, tope, kd=0.0)
    time.sleep(0.5)

    h0 = enlace.heading()
    t0 = time.time()
    enlace.enviar(VEL_RADIO, tope, kd=0.0)

    muestras = []
    completo = False
    try:
        while time.time() - t0 < TIMEOUT_VUELTA:
            time.sleep(0.05)
            h = enlace.heading()
            t = time.time() - t0
            muestras.append([round(t, 3), round(h - h0, 2)])
            if abs(h - h0) >= GRADOS_VUELTA:
                completo = True
                break
            print(f"    {t:5.1f}s  guiñada acumulada {h - h0:+7.1f} grados", end="\r")
    except KeyboardInterrupt:
        print("\n[!] Interrumpido.")
    finally:
        enlace.enviar(0, 0.0)

    t_vuelta = muestras[-1][0] if muestras else 0.0
    print("\n--- Resultado ---")
    if not completo:
        print(f"  [-] No completo los 360 grados en {TIMEOUT_VUELTA:.0f}s "
              f"(llego a {muestras[-1][1] if muestras else 0:+.0f}). "
              f"Sube VEL_RADIO o revisa que el servo llegue al tope.")
    else:
        print(f"  Vuelta completa en {t_vuelta:.2f}s")

    # Velocidad angular media de la parte estacionaria (se descarta el
    # primer 20% del recorrido, que es la rampa de arranque)
    omega = None
    if len(muestras) > 10:
        i0 = int(len(muestras) * 0.2)
        dt = muestras[-1][0] - muestras[i0][0]
        dh = muestras[-1][1] - muestras[i0][1]
        if dt > 0.2:
            omega = dh / dt

    if omega is None:
        print("  Velocidad angular estacionaria: sin dato (muy pocas muestras)")
    else:
        print(f"  Velocidad angular estacionaria: {omega:+.1f} grados/s")
    print("\n  Para cerrar el numero necesitas la velocidad lineal:")
    print(f"    R = v * T / (2*pi)   con T = {t_vuelta:.2f}s")
    print(f"    o bien  R = v / omega_rad  con omega = {omega or 0.0:.1f} grados/s")
    print("  Corre medir_velocidad.py primero y sustituye v (mm/s) a PWM "
          f"{VEL_RADIO}.")
    print("  Cross-check: mide el diametro D del circulo con cinta -> R = D/2.")
    print("  Con R, el angulo de rueda equivalente es "
          "atan(BATALLA/R) = atan(136/R).")

    return _guardar(f"direccion_radio_{lado}.json", {
        "lado": lado,
        "comando_servo": tope,
        "velocidad_pwm": VEL_RADIO,
        "vuelta_completa": completo,
        "t_vuelta_s": round(t_vuelta, 3),
        "omega_grados_s": round(omega, 2) if omega else None,
        "diametro_medido_mm": None,          # <-- rellenar a mano con la cinta
        "muestras_t_heading": muestras,
        "nota": "medido con kd=0 (amortiguacion por giroscopio desactivada)",
    })


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("centro", "radio"):
        print("Uso:")
        print("  python3 medir_direccion.py centro")
        print("  python3 medir_direccion.py radio izq|der")
        sys.exit(1)

    try:
        enlace = EnlacePico()
    except Exception as e:
        print(f"[-] No se pudo abrir el enlace con la Pico: {e}")
        sys.exit(1)

    enlace.enviar(0, 0.0)
    if not _esperar_imu(enlace):
        print("[-] La Pico no esta reportando IMU. Sin telemetria no hay medicion.")
        enlace.cerrar()
        sys.exit(1)
    enlace.fijar_cero()

    try:
        if sys.argv[1] == "centro":
            modo_centro(enlace)
        else:
            lado = sys.argv[2] if len(sys.argv) > 2 else "izq"
            if lado not in ("izq", "der"):
                print("[-] El lado debe ser izq o der.")
                sys.exit(1)
            modo_radio(enlace, lado)
    finally:
        enlace.cerrar()
        print("[+] Traccion detenida y puerto cerrado.")


if __name__ == "__main__":
    main()
