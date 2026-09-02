#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Radio de giro real del chasis: se mide el circulo, no se deduce.

Hasta ahora el radio salia de dividir dos medidas independientes (velocidad
lineal contra un muro y velocidad angular de la IMU) tomadas en corridas
distintas: r = v / w. Eso arrastra el error de las dos y ademas supone que no
hay deslizamiento. Aqui el robot traza el circulo y la cinta metrica da el
radio directamente.

Protocolo recomendado (media vuelta):

  1. Marcar en el suelo un punto fijo del chasis (por ejemplo el centro del
     eje trasero, con cinta).
  2. Correr con `--grados 180`. Al terminar, marcar el mismo punto otra vez.
  3. La distancia en linea recta entre las dos marcas es el DIAMETRO: r = d/2.

Media vuelta mide mejor que la vuelta entera, porque una recta entre dos
marcas se mide con mucha mas precision que el diametro de una curva dibujada
en el suelo. La vuelta entera (`--grados 360`) sirve para ver el circulo y
para comprobar que el robot vuelve a su sitio: si no cierra, hay
deslizamiento o la IMU esta derivando.

Por que kd=0
------------
El firmware resta al servo un termino de amortiguacion por giroscopio
(`angulo_servo = CENTRO + angulo - velocidad_z * KD_ESTABILIDAD * kd_activo`).
En un giro sostenido `velocidad_z` no es cero, asi que con kd=1 la rueda NO
esta en el angulo comandado y se mediria el radio del lazo de estabilidad, no
el del chasis. Por eso el tercer campo de la consigna va a 0.00 aqui, y solo
aqui. Cualquier consigna de dos campos lo devuelve a 1.0 en la Pico.

Uso (en la Pi, desde /home/pi/wro_nueva_actual):

    python3 medir_radio_giro.py --solo-servo --lado der     # sin traccion
    python3 medir_radio_giro.py --lado der --grados 180
    python3 medir_radio_giro.py --lado der --grados 360
"""
import argparse
import csv
import math
import os
import signal
import sys
import threading
import time

try:                                    # despliegue por paquetes
    from comun.enlace_pico import EnlacePico
    from comun.lidar_driver import LidarDriver
    from comun.lidar_geometria import construir_perfil_360
except ImportError:                     # despliegue plano
    from enlace_pico import EnlacePico
    from lidar_driver import LidarDriver
    from lidar_geometria import construir_perfil_360

BATALLA_MM = 136.0          # entre ejes, para deducir el angulo de rueda
CARPETA_SALIDA = "mediciones"

# Topes mecanicos del servo, los mismos que LIMITE_DER/LIMITE_IZQ del firmware
# (CENTRO=90, 70 a la derecha, 115 a la izquierda). Pedir mas no gira mas: la
# Pico lo acota igual.
ANGULO_TOPE = {"der": -20.0, "izq": 25.0}

PERIODO_S = 0.05            # 20 Hz de consigna; el watchdog de la Pico son 500 ms
RAMPA_S = 0.4               # subida de PWM, para no arrancar patinando
ASENTAR_SERVO_S = 0.8       # el servo llega al tope ANTES de que ruede la rueda


class Abortado(Exception):
    pass


class ColectorLidar:
    """Guarda (rumbo, perfil 360) de cada barrido mientras dura el giro."""

    def __init__(self, leer_rumbo):
        self._leer_rumbo = leer_rumbo
        self.corriendo = True
        self.activo = False
        self.muestras = []

    def al_barrido(self, scan):
        if not self.activo:
            return
        # El rumbo se lee aqui y no despues: emparejar por timestamp anadiria
        # un desfase que en un giro de 15 grados/s se nota.
        self.muestras.append((self._leer_rumbo(), construir_perfil_360(scan)))


def _parsear_argumentos():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--lado", choices=("der", "izq"), default="der")
    p.add_argument("--grados", type=float, default=360.0,
                   help="giro a completar (180 para medir, 360 para ver el circulo)")
    p.add_argument("--pwm", type=int, default=25,
                   help="consigna de traccion; 22 es la del parqueo")
    p.add_argument("--angulo", type=float, default=None,
                   help="grados de servo; por defecto el tope del lado elegido")
    p.add_argument("--kd", type=float, default=0.0,
                   help="factor de amortiguacion por giroscopio (0 para medir)")
    p.add_argument("--timeout", type=float, default=None,
                   help="corte duro en segundos; por defecto se estima del giro")
    p.add_argument("--solo-servo", action="store_true",
                   help="mueve el servo y espera, sin traccion: para el transportador")
    p.add_argument("--espera", type=float, default=0.0,
                   help="segundos de cuenta atras antes de mover, para marcar el suelo")
    p.add_argument("--con-lidar", action="store_true",
                   help="deduce el radio del propio LiDAR, sin marcar el suelo")
    p.add_argument("--etiqueta", default="")
    return p.parse_args()


def _esperar_telemetria(enlace, limite_s=4.0):
    t0 = time.time()
    while time.time() - t0 < limite_s:
        if enlace.heading_valido():
            return True
        time.sleep(0.05)
    return False


def _solo_servo(enlace, angulo, kd, segundos=12.0):
    """Mantiene el servo en el tope sin traccion, para medir con transportador."""
    print("[i] servo a {:+.1f} grados durante {:.0f} s, motor parado.".format(
        angulo, segundos))
    print("    Mide con el transportador el angulo REAL de la rueda delantera.")
    t0 = time.time()
    ultimo_aviso = 0.0
    while time.time() - t0 < segundos:
        enlace.enviar(0, angulo, kd)
        # Un aviso por segundo: por SSH el retorno de carro no borra la linea
        # y el progreso continuo llena la sesion de basura.
        restante = segundos - (time.time() - t0)
        if ultimo_aviso - restante >= 1.0 or ultimo_aviso == 0.0:
            print("    quedan {:4.1f} s".format(restante), flush=True)
            ultimo_aviso = restante
        time.sleep(PERIODO_S)
    print("\n[i] centrando.")
    for _ in range(10):
        enlace.enviar(0, 0.0, kd)
        time.sleep(PERIODO_S)


def _girar(enlace, angulo, pwm, objetivo_deg, kd, timeout_s, espera_s=0.0,
           colector=None):
    """Gira hasta acumular objetivo_deg de rumbo. Devuelve la traza (t, rumbo)."""
    if espera_s > 0.0:
        # El robot esta quieto: es el momento de marcar el suelo. Se sigue
        # mandando el cero para que el watchdog de la Pico no se dispare.
        print("[i] MARCA AHORA EL PUNTO DE PARTIDA EN EL SUELO.")
        t0 = time.time()
        while time.time() - t0 < espera_s:
            enlace.enviar(0, 0.0, kd)
            print("    arranca en {:4.1f} s".format(espera_s - (time.time() - t0)),
                  flush=True)
            time.sleep(1.0)

    print("[i] asentando el servo en {:+.1f} grados ({:.1f} s)...".format(
        angulo, ASENTAR_SERVO_S))
    t0 = time.time()
    while time.time() - t0 < ASENTAR_SERVO_S:
        enlace.enviar(0, angulo, kd)
        time.sleep(PERIODO_S)

    enlace.fijar_cero()
    if colector is not None:
        colector.activo = True
    print("[i] MARCA EL PUNTO DE PARTIDA AHORA. Arrancando a {} PWM.".format(pwm))
    traza = []
    t_inicio = time.time()
    ultimo_valido = t_inicio
    ultimo_aviso = -1.0
    try:
        while True:
            ahora = time.time()
            transcurrido = ahora - t_inicio

            if transcurrido > timeout_s:
                print("\n[!] corte por timeout ({:.1f} s) sin cerrar el giro.".format(
                    timeout_s))
                break

            # La telemetria es la unica evidencia de que el robot gira. Sin
            # ella no se sigue moviendo a ciegas.
            if enlace.heading_valido():
                ultimo_valido = ahora
            elif ahora - ultimo_valido > 0.5:
                raise Abortado("la IMU dejo de reportar")

            rumbo = enlace.heading()
            traza.append((transcurrido, rumbo))
            if abs(rumbo) >= objetivo_deg:
                print("\n[i] {:.0f} grados completados en {:.2f} s.".format(
                    objetivo_deg, transcurrido))
                break

            # Rampa corta: un escalon de PWM patina y el patinaje falsea el radio.
            factor = min(1.0, transcurrido / RAMPA_S) if RAMPA_S > 0 else 1.0
            enlace.enviar(int(round(pwm * factor)), angulo, kd)
            if transcurrido - ultimo_aviso >= 1.0:
                print("    t={:5.2f}s  rumbo={:+7.1f} deg".format(transcurrido, rumbo),
                      flush=True)
                ultimo_aviso = transcurrido
            time.sleep(PERIODO_S)
    finally:
        enlace.detener()
        if colector is not None:
            colector.activo = False
    return traza


# --------------------------------------------------------------------------
# Radio por LiDAR, sin marcar el suelo
#
# Al girar en circulo, el robot describe una circunferencia de radio r. Si se
# mira SIEMPRE hacia la misma direccion del MUNDO (compensando el rumbo con la
# IMU), la distancia a una pared plana recorre una sinusoide:
#
#     d(psi) = A - (r / cos a) * cos(psi - fase)
#
# donde a es el angulo entre el rayo y la normal de la pared. La amplitud es
# minima, y vale exactamente r, cuando el rayo apunta perpendicular a la
# pared. Por eso se prueban todas las direcciones y se busca la de menor
# amplitud entre las que ajustan bien: no hace falta saber donde esta la pared.
# --------------------------------------------------------------------------

DIST_MIN_VALIDA_MM = 150.0   # por debajo es el propio robot (rueda o mastil)
DIST_MAX_VALIDA_MM = 7000.0  # 8000 es el centinela de "sin dato" del C1


def _resolver_3x3(a, b):
    """Gauss con pivoteo parcial. Devuelve None si el sistema es singular."""
    m = [list(fila) + [t] for fila, t in zip(a, b)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda f: abs(m[f][col]))
        if abs(m[piv][col]) < 1e-12:
            return None
        m[col], m[piv] = m[piv], m[col]
        for fila in range(3):
            if fila == col:
                continue
            factor = m[fila][col] / m[col][col]
            for k in range(col, 4):
                m[fila][k] -= factor * m[col][k]
    return [m[i][3] / m[i][i] for i in range(3)]


def _ajustar_sinusoide(psis, ds):
    """Ajusta d = A + B cos(psi) + C sin(psi). Devuelve (amplitud, residuo)."""
    n = len(ds)
    if n < 12:
        return None
    base = [(1.0, math.cos(p), math.sin(p)) for p in psis]
    ata = [[sum(f[i] * f[j] for f in base) for j in range(3)] for i in range(3)]
    atb = [sum(f[i] * d for f, d in zip(base, ds)) for i in range(3)]
    coef = _resolver_3x3(ata, atb)
    if coef is None:
        return None
    residuo = math.sqrt(sum((d - sum(c * f[i] for i, c in enumerate(coef))) ** 2
                            for f, d in zip(base, ds)) / n)
    return math.hypot(coef[1], coef[2]), residuo


def _radio_por_lidar(muestras):
    """Prueba cada direccion del mundo y devuelve las candidatas ordenadas."""
    candidatas = []
    for signo in (1.0, -1.0):
        for beta0 in range(0, 360, 2):
            psis, ds = [], []
            for rumbo, perfil in muestras:
                d = perfil[int(round(beta0 + signo * rumbo)) % 360]
                if DIST_MIN_VALIDA_MM < d < DIST_MAX_VALIDA_MM:
                    psis.append(math.radians(rumbo))
                    ds.append(d)
            if len(ds) < 0.5 * len(muestras):
                continue
            ajuste = _ajustar_sinusoide(psis, ds)
            if ajuste:
                candidatas.append((ajuste[1], ajuste[0], beta0, signo, len(ds)))
    return sorted(candidatas)


def _ajustar_omega(traza, descarte=0.20):
    """Velocidad angular de regimen por minimos cuadrados, sin la rampa."""
    if len(traza) < 8:
        return None
    corte = traza[-1][0] * descarte
    puntos = [(t, h) for t, h in traza if t >= corte]
    if len(puntos) < 5:
        return None
    n = len(puntos)
    st = sum(t for t, _ in puntos)
    sh = sum(h for _, h in puntos)
    stt = sum(t * t for t, _ in puntos)
    sth = sum(t * h for t, h in puntos)
    denominador = n * stt - st * st
    if abs(denominador) < 1e-9:
        return None
    return (n * sth - st * sh) / denominador


def main():
    args = _parsear_argumentos()
    angulo = args.angulo if args.angulo is not None else ANGULO_TOPE[args.lado]
    if not -45.0 <= angulo <= 45.0:
        sys.exit("[-] La Pico rechaza angulos fuera de +-45 grados.")
    if not 0.0 <= args.kd <= 2.0:
        sys.exit("[-] kd valido entre 0 y 2.")

    enlace = EnlacePico()
    if not _esperar_telemetria(enlace):
        enlace.cerrar()
        sys.exit("[-] La Pico no reporta IMU. Prueba deploy_pico.sh --reiniciar.")

    def _al_interrumpir(_signo, _marco):
        raise Abortado("interrumpido por el usuario")

    signal.signal(signal.SIGINT, _al_interrumpir)

    colector = lidar = None
    if args.con_lidar:
        if abs(args.grados) < 350.0:
            enlace.cerrar()
            sys.exit("[-] --con-lidar necesita una vuelta entera: usa --grados 360.")
        colector = ColectorLidar(enlace.heading)
        lidar = LidarDriver()
        threading.Thread(target=lidar.hilo_lectura,
                         args=(lambda: colector.corriendo, colector.al_barrido),
                         daemon=True).start()
        print("[i] LiDAR arrancado; esperando barridos...")
        time.sleep(2.0)

    traza = []
    try:
        if args.solo_servo:
            _solo_servo(enlace, angulo, args.kd)
            return
        # 14 grados/s fue lo medido el 31-08; el timeout deja holgura de sobra
        # pero no deja al robot dando vueltas si algo se atasca.
        timeout = args.timeout or max(20.0, abs(args.grados) / 14.0 * 2.2)
        traza = _girar(enlace, angulo, args.pwm, abs(args.grados), args.kd, timeout,
                       args.espera, colector)
    except Abortado as error:
        print("\n[!] {}".format(error))
    finally:
        # Doble freno: detener() manda el cero cinco veces y ademas se centra
        # el servo antes de soltar el puerto.
        enlace.detener()
        for _ in range(5):
            enlace.enviar(0, 0.0)
            time.sleep(0.01)
        enlace.cerrar()
        if colector is not None:
            colector.corriendo = False
            time.sleep(0.3)
            lidar.cerrar()

    if not traza:
        return

    omega = _ajustar_omega(traza)
    total_t, total_h = traza[-1]
    print("\n=== resultado ===")
    print("  giro acumulado : {:+.1f} grados en {:.2f} s".format(total_h, total_t))
    if omega:
        print("  velocidad angular de regimen: {:.2f} grados/s".format(abs(omega)))
        print("  vuelta completa equivalente : {:.1f} s".format(360.0 / abs(omega)))
    if colector is not None:
        print("\n=== radio deducido del LiDAR ===")
        print("  {} barridos guardados".format(len(colector.muestras)))
        candidatas = _radio_por_lidar(colector.muestras)
        if not candidatas:
            print("  [-] Ningun ajuste valido. Necesita una pared plana a la vista")
            print("      durante toda la vuelta; en campo abierto no funciona.")
        else:
            print("  las 6 direcciones que mejor ajustan una pared:")
            print("    residuo   amplitud   direccion   muestras")
            for residuo, amp, beta0, signo, n in candidatas[:6]:
                print("    {:7.1f}   {:8.1f}   {:4d}({:+.0f})   {:6d}".format(
                    residuo, amp, beta0, signo, n))
            # El rayo perpendicular a la pared da la amplitud MINIMA, y esa
            # amplitud es el radio. Entre los ajustes buenos se busca ese minimo.
            corte = candidatas[0][0] * 2.0
            buenos = [c for c in candidatas if c[0] <= corte]
            radio = min(c[1] for c in buenos)
            print("\n  RADIO = {:.0f} mm   (diametro {:.0f} mm)".format(radio, 2 * radio))
            print("  angulo de rueda implicito: {:.1f} deg".format(
                math.degrees(math.atan(BATALLA_MM / radio))))
            if omega:
                print("  velocidad lineal en el giro: {:.0f} mm/s".format(
                    radio * math.radians(abs(omega))))
    else:
        print("\n  AHORA MIDE CON CINTA METRICA:")
        if abs(abs(args.grados) - 180.0) < 1.0:
            print("   - distancia entre la marca inicial y la final = DIAMETRO")
            print("   - radio = esa distancia / 2")
        else:
            print("   - diametro del circulo trazado")
            print("   - (con --grados 180 la medida sale de una recta y es mas fina)")
        print("   - con el radio r: angulo real de rueda = atan({:.0f}/r)".format(
            BATALLA_MM))
        print("     Referencia: r=400 mm (rueda {:.0f} grados) cierra una esquina.".format(
            math.degrees(math.atan(BATALLA_MM / 400.0))))

    etiqueta = args.etiqueta or "{}_pwm{}_kd{:.1f}".format(args.lado, args.pwm, args.kd)
    os.makedirs(CARPETA_SALIDA, exist_ok=True)
    ruta = os.path.join(CARPETA_SALIDA, "radio_{}.csv".format(etiqueta))
    with open(ruta, "w", newline="") as f:
        escritor = csv.writer(f)
        escritor.writerow(["t_s", "rumbo_deg"])
        escritor.writerows([["{:.3f}".format(t), "{:.2f}".format(h)] for t, h in traza])
    print("\n  traza de rumbo -> {}".format(ruta))


if __name__ == "__main__":
    main()
