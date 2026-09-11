# Bateria de escenarios largos contra el simulador.
#
# No es una prueba unitaria: es la MEDIDA repetible que se usa para
# decidir si un cambio de control mejora o empeora. Los tests dicen si
# un contrato se rompe; esto dice cuanto avanza el robot y a que precio.
#
# Se corre a mano:   python tests/escenarios.py
#
# Columnas:
#   vueltas   yaw acumulado / 360. Es AVANCE, no vueltas de pista.
#   SIN_SAL   % de ciclos sin ningun comando admisible
#   RETRO     % de ciclos en marcha atras
#   ULT_REC   % de ciclos resueltos por el ultimo recurso
#   choques   ciclos con el chasis tocando un muro
#   roce      holgura MINIMA a un pilar en todo el escenario, en mm
#             (negativa = lo toco)
#   LADO MAL  pilares rebasados POR EL LADO PROHIBIDO. Es la columna que
#             manda: en competicion, uno solo TERMINA el recorrido. Un
#             escenario con muchas vueltas y un lado mal vale menos que
#             uno con menos vueltas y cero.
#             Solo cuenta un cruce si el pilar estaba a menos de
#             DIST_PASO_REAL: en un anillo, TODOS los pilares cruzan el
#             eje trasero una vez por vuelta -- tambien los del lado
#             opuesto de la pista, a 2 metros -- y contar aquello daba 65
#             infracciones donde habia 2 por vuelta.
#   |ang|med  mediana de |angulo_muro|, para ver si la pose se degrada
#
# LIMITE CONOCIDO, y hay que tenerlo presente al leer la tabla: estos
# seis escenarios salen con 0% de RETROCESO en las dos versiones del
# criterio frontal (el de +-10 grados y el de rumbo por esquina), asi
# que NO discriminan ese cambio. Sirven como red anti-regresion --
# vueltas, roce a los pilares y choques -- no como evidencia a favor.
# Quien discrimina ahi es la pista: el caso es "pared a ~120mm CON un
# pilar fichado", que aqui no llega a darse.
import math
import os
import sys

# Un pilar mas lejos que esto al cruzar el eje trasero no se ha
# "rebasado": es uno del lado opuesto del anillo que pasa de largo.
DIST_PASO_REAL = 500.0

_AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _AQUI)
sys.path.insert(0, os.path.dirname(_AQUI))

import navegacion                       # noqa: E402
from simulador import DT, Mundo, Pista, Robot   # noqa: E402


class SectorFalso:
    def fijar_sector_frontal(self, a, b):
        pass

    def sector_frontal_normal(self):
        pass


# Los pilares van en el pasillo, repartidos por los cuatro cuadrantes.
#
# TODOS EN TRAMO RECTO, a proposito. Dos de ellos estuvieron un rato en
# la DIAGONAL de una esquina -- (1000, 700) y (-1000, -700) -- y fallaban
# el lado las tres vueltas, sin excepcion. No era el control: a esa
# posicion el pilar aparece a 77 grados de rumbo, o sea FUERA del campo
# de la camara (~66 grados), asi que `_intentar_capturar` no llegaba a
# llamarse nunca -- sin color no hay captura -- y cuando entraba en el
# cono ya estaba al costado, a (397, 89). Un pilar ahi no es evitable con
# esta camara, y dejarlo en la bateria solo enmascara regresiones reales
# con seis infracciones fijas. Si algun dia hace falta cubrir ese caso,
# el arreglo es de PERCEPCION (mas campo, o sembrar por LiDAR sin color),
# no de la maquina de estados.
ESCENARIOS = {
    "vacio":      ([], (-800.0, 1000.0, 0.0)),
    "vacio desc": ([], (-800.0, 1150.0, -12.0)),
    "4pil":       ([(-1000.0, 1000.0, "ROJO"), (1000.0, 900.0, "VERDE"),
                    (1000.0, -1000.0, "ROJO"), (-900.0, -1000.0, "VERDE")],
                   (-800.0, 1000.0, 0.0)),
    "6pil":       ([(-1000.0, 1000.0, "ROJO"), (700.0, 1000.0, "VERDE"),
                    (1000.0, 300.0, "ROJO"), (1000.0, -800.0, "VERDE"),
                    (-700.0, -1000.0, "ROJO"), (-1000.0, -300.0, "VERDE")],
                   (-800.0, 1000.0, 0.0)),
    "6pil desc":  ([(-1000.0, 1000.0, "ROJO"), (700.0, 1000.0, "VERDE"),
                    (1000.0, 300.0, "ROJO"), (1000.0, -800.0, "VERDE"),
                    (-700.0, -1000.0, "ROJO"), (-1000.0, -300.0, "VERDE")],
                   (-750.0, 1150.0, -15.0)),
    # EL CASO QUE FALLA EN PISTA: dos bloques SEGUIDOS en la misma
    # recta, verde y luego rojo. Es el maximo que permite el reglamento
    # por seccion, y es donde murieron los dos unicos fallos de la
    # corrida 212600: al soltar el verde, el rojo ya esta a 420 mm y
    # exige cruzar 350 -- no cabe. Yendo en sentido horario, por la
    # recta de la derecha se baja en y, asi que y=+300 se encuentra
    # antes que y=-200.
    "2 seguidos": ([(1000.0, 300.0, "VERDE"), (1000.0, -200.0, "ROJO")],
                   (-800.0, 1000.0, 0.0)),
    "2 seguidos x2": ([(1000.0, 300.0, "VERDE"), (1000.0, -200.0, "ROJO"),
                       (-1000.0, -300.0, "ROJO"), (-1000.0, 200.0, "VERDE")],
                      (-800.0, 1000.0, 0.0)),
    "6pil desc2": ([(-1000.0, 1000.0, "ROJO"), (700.0, 1000.0, "VERDE"),
                    (1000.0, 300.0, "ROJO"), (1000.0, -800.0, "VERDE"),
                    (-700.0, -1000.0, "ROJO"), (-1000.0, -300.0, "VERDE")],
                   (-1150.0, 850.0, 20.0)),
}


def correr(pilares, pose, ciclos=900):
    pista = Pista(pilares)
    robot = Robot(*pose)
    mundo = Mundo(pista, robot)
    nav = navegacion.Navegador(SectorFalso())
    t = 0.0
    n = sin_sal = retro = ult = choques = 0
    roce = float("inf")
    angs = []
    # Lado por el que cada pilar quedo atras. La regla: ROJO se pasa por
    # su derecha -- o sea que el pilar termina a la IZQUIERDA del robot,
    # x_b < 0 -- y VERDE al reves.
    lado_exigido = [-1 if c == "ROJO" else 1 for (_x, _y, c) in pilares]
    y_previa = [None] * len(pilares)
    lado_mal = 0
    culpables = set()
    for _ in range(ciclos):
        med = mundo.medicion()
        color, cx = mundo.camara()
        consigna = nav.procesar(med, color, robot.heading, ahora=t, cx_cam=cx)
        if consigna is None:
            break
        robot.avanzar(consigna[0], consigna[1])
        t += DT
        n += 1
        if nav.seguridad == "SIN_SALIDA":
            sin_sal += 1
        if nav.seguridad == "ULTIMO_RECURSO":
            ult += 1
        if nav.estado == "RETROCESO":
            retro += 1
        if mundo.choca_con_muro():
            choques += 1
        if pilares:
            roce = min(roce, mundo.distancia_min_a_pilares())
            for i, (px, py, _c) in enumerate(pilares):
                x_b, y_b = robot.a_marco_robot(px, py, origen=(robot.x, robot.y))
                if (y_previa[i] is not None and y_previa[i] >= 0.0 > y_b
                        and math.hypot(x_b, y_b) < DIST_PASO_REAL
                        and (1 if x_b > 0 else -1) != lado_exigido[i]):
                    lado_mal += 1
                    culpables.add(i)
                y_previa[i] = y_b
        if med.muro_valido:
            angs.append(abs(med.angulo_muro))
    angs.sort()
    return {
        "vueltas": robot.heading / 360.0,
        "sin_sal": 100.0 * sin_sal / max(1, n),
        "retro": 100.0 * retro / max(1, n),
        "ult": 100.0 * ult / max(1, n),
        "choques": choques,
        "roce": roce,
        "ang": angs[len(angs) // 2] if angs else float("nan"),
        "lado_mal": lado_mal,
        "culpables": sorted(culpables),
    }


def main():
    print("escenario      vueltas  SIN_SAL  RETRO  ULT_REC  choques  roce  |ang|med  LADO MAL")
    total = 0.0
    total_mal = 0
    for nombre, (pilares, pose) in ESCENARIOS.items():
        r = correr(pilares, pose)
        total += r["vueltas"]
        total_mal += r["lado_mal"]
        roce = "-" if math.isinf(r["roce"]) else "%.0f" % r["roce"]
        print("%-13s %7.2f  %6.1f%% %5.1f%%  %6.1f%%  %7d %5s %9.1f %9d"
              % (nombre, r["vueltas"], r["sin_sal"], r["retro"], r["ult"],
                 r["choques"], roce, r["ang"], r["lado_mal"])
              + ("  pilares %s" % r["culpables"] if r["culpables"] else ""))
    print("MEDIA de vueltas: %.2f   |   LADO PROHIBIDO en total: %d"
          % (total / len(ESCENARIOS), total_mal))


if __name__ == "__main__":
    main()
