"""Reproduce el parqueo pendiente al cerrar la sesion 2026-09-07.

No toca hardware. Mismo escenario de rayos y radios diferentes del modelo
con que se descubrio que el planificador/seguidor aun no estaba terminado.
"""
import math
import sys

from test_parqueo_simulado import escenario, _corta_rayo
from retorno_parqueo import RetornoParqueo


def correr(lado):
    mundo = escenario(lado, x=-400)
    retorno = RetornoParqueo()
    retorno.lado = lado
    retorno._t_busqueda = 0
    anterior = ""
    for i in range(1200):
        t = i * .1
        retorno.observar(mundo.medicion(), mundo.robot.heading, t, 0, 0, .1)
        if i < 5:
            continue
        th = mundo.robot.theta
        # 60 mm de voladizo menos 34 mm de transductor: 26 mm detras del eje.
        ox = mundo.robot.x - 26 * math.cos(th)
        oy = mundo.robot.y - 26 * math.sin(th)
        ecos = [d for seg in mundo.pista.segmentos_totales()
                if (d := _corta_rayo(ox, oy, -math.cos(th), -math.sin(th), seg)) is not None]
        us = min(ecos) if ecos else None
        cmd = retorno.parquear(mundo.robot.heading, us, t)
        if cmd.estado != anterior or cmd.terminado:
            pose = (mundo.robot.x, mundo.robot.y, mundo.robot.heading)
            estimada = retorno.seguidor.pose if retorno.seguidor else None
            print(lado, round(t, 1), cmd.estado, cmd.razon,
                  "pose real", tuple(round(v, 2) for v in pose),
                  "pose estimada", estimada, flush=True)
            anterior = cmd.estado
        if cmd.terminado:
            return cmd.verificado
        mundo.robot.avanzar(cmd.velocidad, cmd.angulo, .1)
    return False


if __name__ == "__main__":
    resultados = [correr(lado) for lado in (-1, 1)]
    # La validacion completa del cuerpo y contactos sigue pendiente; incluso
    # si se llega a LISTO, ampliar este reproducer antes de autorizar pista.
    sys.exit(0 if all(resultados) else 1)
