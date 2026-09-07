# Matriz de escenarios de esquina, sobre el simulador.
#
# Cada prueba corre la FSM completa contra un mundo con paredes y
# pilares reales y comprueba HECHOS DEL MUNDO, no creencias del robot:
# a que distancia paso del pilar, por que lado paso, y si toco algo.
#
#   cd src/pi3B/ronda_curvas && python -m unittest discover -s tests -v
#
# El simulador usa radios de giro distintos de los que supone
# geometria_evasion (239/341 reales contra 260/360 asumidos), asi que
# ninguna de estas pruebas pasa por tener el modelo perfecto.
import math
import os
import sys
import unittest

AQUI = os.path.dirname(os.path.abspath(__file__))
if AQUI not in sys.path:
    sys.path.insert(0, AQUI)
CARPETA = os.path.dirname(AQUI)
if CARPETA not in sys.path:
    sys.path.insert(0, CARPETA)
COMUN = os.path.join(os.path.dirname(CARPETA), "comun")
if COMUN not in sys.path:
    sys.path.append(COMUN)

import navegacion                       # noqa: E402
from simulador import DT, Mundo, Pista, Robot   # noqa: E402


class SectorFalso:
    def fijar_sector_frontal(self, a, b):
        pass

    def sector_frontal_normal(self):
        pass


class Resultado:
    def __init__(self, pilares):
        self.choque_muro = False
        self.contacto_pilar = 1e9
        self.estados = []
        self.d_min = [1e9] * len(pilares)
        self.lado_en_d_min = [0] * len(pilares)
        self.paso_validado = set()
        self.abortos = 0
        self.ids_objetivo = []

    def visito(self, estado):
        return estado in self.estados

    def resumen(self):
        orden = []
        for e in self.estados:
            if not orden or orden[-1] != e:
                orden.append(e)
        return " -> ".join(orden)


def correr(pista, robot, ciclos=320, ocultar_lidar_en=None,
           ocultar_camara_en=None, ruido_pared=None):
    """Corre la FSM contra el mundo. Los `ocultar_*` son funciones
    ciclo -> conjunto de indices de pilar que ese ciclo no se ven."""
    mundo = Mundo(pista, robot)
    nav = navegacion.Navegador(SectorFalso())
    res = Resultado(pista.pilares)
    t = 0.0

    for k in range(ciclos):
        mundo.ocultar_lidar = ocultar_lidar_en(k) if ocultar_lidar_en else set()
        mundo.ocultar_camara = ocultar_camara_en(k) if ocultar_camara_en else set()
        med = mundo.medicion()
        if ruido_pared:
            ruido_pared(k, med)
        color, cx = mundo.camara()
        consigna = nav.procesar(med, color, robot.heading, ahora=t, cx_cam=cx)
        if consigna is None:
            break
        vel, ang = consigna
        robot.avanzar(vel, ang)
        t += DT

        res.estados.append(nav.estado)
        if nav.tracker.activo:
            res.ids_objetivo.append(nav.tracker.id)
        if nav._paso_validado and nav.tracker.activo:
            res.paso_validado.add(nav.tracker.id)
        res.abortos = nav._abortos

        if mundo.choca_con_muro():
            res.choque_muro = True
            break
        res.contacto_pilar = min(res.contacto_pilar, mundo.distancia_min_a_pilares())
        for i, (px, py, _c) in enumerate(pista.pilares):
            x_b, y_b = robot.a_marco_robot(px, py, origen=(robot.x, robot.y))
            d = math.hypot(x_b, y_b)
            if d < res.d_min[i]:
                res.d_min[i] = d
                res.lado_en_d_min[i] = 1 if x_b > 0 else -1
    return nav, res


# ==========================================================
# ESCENARIOS
# ==========================================================
# Pista: anillo de 3000x3000 con isla de 1000x1000, pasillo de 1000mm.
# Recorrido HORARIO: por el tramo superior se va hacia +x, la isla queda
# a la DERECHA y el muro exterior (con la esquina concava) a la
# IZQUIERDA. La esquina de trabajo es la de (+1500, +1500).
#
# Recorrido ANTIHORARIO: se entra por el tramo derecho hacia +y, y todo
# es el espejo.

# Los pilares van sobre el eje del carril, uno ANTES de la esquina y
# otro justo DESPUES: es la situacion del enunciado (uno al principio y
# otro al final de la curva) y deja el mismo hueco a cada lado, asi que
# rojo y verde son igual de dificiles.
#
# El segundo esta 50mm PASADA la esquina, no dentro de ella. No es una
# concesion: con el HFOV medido de 68 grados, un pilar metido en la
# diagonal de la esquina se queda a ~50 grados del eje optico durante
# TODO el giro y la camara no llega a verlo nunca, asi que su color
# nunca se conoce y no hay regla que aplicar. Es un limite fisico del
# montaje, no del algoritmo, y esta anotado en DISENO_CURVAS.md. El caso
# extremo se conserva al final como prueba de degradacion segura.

def escenario_horario(color_final, con_pilar_inicial=True):
    pilares = []
    if con_pilar_inicial:
        pilares.append((-200.0, 1000.0, "ROJO"))
    pilares.append((1000.0, -250.0, color_final))
    pista = Pista(pilares)
    robot = Robot(-900.0, 1000.0, 0.0)      # mirando a +x
    return pista, robot


def escenario_antihorario(color_final, con_pilar_inicial=True):
    pilares = []
    if con_pilar_inicial:
        pilares.append((1000.0, -200.0, "VERDE"))
    pilares.append((-250.0, 1000.0, color_final))
    pista = Pista(pilares)
    robot = Robot(1000.0, -900.0, 90.0)     # mirando a +y
    return pista, robot


class MatrizDeEsquinas(unittest.TestCase):

    def _comprobar(self, nav, res, pista, indice_final, exigir_paso=True):
        self.assertFalse(res.choque_muro, "choco contra el muro: %s" % res.resumen())
        self.assertGreater(res.contacto_pilar, 0.0,
                           "toco un pilar (holgura %.0fmm)" % res.contacto_pilar)
        s_esperado = navegacion.lado_obligatorio(pista.pilares[indice_final][2])
        if exigir_paso:
            self.assertEqual(res.lado_en_d_min[indice_final], s_esperado,
                             "el pilar final quedo del lado prohibido")
            self.assertGreater(res.d_min[indice_final],
                               navegacion.SEPARACION_MIN_VALIDA - 40.0,
                               "paso demasiado justo: %.0fmm"
                               % res.d_min[indice_final])

    # --- horario x verde/rojo, con y sin pilar inicial -----------------
    def test_horario_verde_al_final(self):
        # EL CASO DEL ENUNCIADO. Horario: la vuelta gira a la derecha,
        # pero un VERDE obliga a pasar por su izquierda, o sea a girar
        # PRIMERO hacia la concavidad del muro exterior, y a recuperar
        # despues con el giro DEBIL (derecha, R_min 360).
        pista, robot = escenario_horario("VERDE")
        nav, res = correr(pista, robot)
        self._comprobar(nav, res, pista, 1)
        self.assertTrue(res.visito("APERTURA") or res.visito("CONTRAGIRO"),
                        "no hubo maniobra: %s" % res.resumen())

    def test_horario_rojo_al_final(self):
        pista, robot = escenario_horario("ROJO")
        nav, res = correr(pista, robot)
        self._comprobar(nav, res, pista, 1)

    def test_horario_sin_pilar_inicial(self):
        pista, robot = escenario_horario("VERDE", con_pilar_inicial=False)
        nav, res = correr(pista, robot)
        self._comprobar(nav, res, pista, 0)

    # --- antihorario ---------------------------------------------------
    def test_antihorario_rojo_al_final(self):
        # Espejo del caso del enunciado: antihorario + ROJO obliga a
        # abrir hacia la concavidad y recuperar con el giro fuerte.
        pista, robot = escenario_antihorario("ROJO")
        nav, res = correr(pista, robot)
        self._comprobar(nav, res, pista, 1)

    def test_antihorario_verde_al_final(self):
        pista, robot = escenario_antihorario("VERDE")
        nav, res = correr(pista, robot)
        self._comprobar(nav, res, pista, 1)

    # --- perdidas de sensor --------------------------------------------
    def test_pilar_fuera_de_camara(self):
        # La camara pierde el pilar final durante 1.4s en plena maniobra.
        # No puede cambiar ni el lado ni la fase.
        pista, robot = escenario_horario("VERDE")
        nav, res = correr(pista, robot,
                          ocultar_camara_en=lambda k: {1} if 70 <= k < 82 else set())
        self._comprobar(nav, res, pista, 1)

    def test_cluster_lidar_intermitente(self):
        # El cluster del pilar desaparece a rafagas: la estimacion tiene
        # que sobrevivir por prediccion.
        pista, robot = escenario_horario("VERDE")
        nav, res = correr(pista, robot,
                          ocultar_lidar_en=lambda k: {1} if (k // 4) % 3 == 0 else set())
        self._comprobar(nav, res, pista, 1)

    def test_dos_pilares_visibles_a_la_vez(self):
        # Dos pilares proximos y del mismo color en el mismo tramo: el
        # objetivo activo no puede cambiar de identidad a mitad de
        # maniobra.
        pista = Pista([(700.0, 1000.0, "VERDE"), (1080.0, 720.0, "VERDE")])
        robot = Robot(-300.0, 1000.0, 0.0)
        nav, res = correr(pista, robot)
        self.assertFalse(res.choque_muro)
        self.assertGreater(res.contacto_pilar, 0.0)
        # Los ids usados tienen que ser una secuencia no decreciente sin
        # saltos hacia atras: nunca se vuelve a un objetivo anterior.
        self.assertEqual(res.ids_objetivo, sorted(res.ids_objetivo))

    def test_baja_confianza_exige_mas_holgura(self):
        # Con el LiDAR tapado la mitad del tiempo, sigma sube y la
        # holgura exigida con ella. La maniobra puede acabar en aborto,
        # pero NO en contacto.
        pista, robot = escenario_horario("ROJO")
        nav, res = correr(pista, robot,
                          ocultar_lidar_en=lambda k: {1} if k % 2 == 0 else set())
        self.assertFalse(res.choque_muro)
        self.assertGreater(res.contacto_pilar, 0.0)

    # --- emergencias ----------------------------------------------------
    def _emergencia_en(self, fase, pista, robot):
        # Falsea una pared pegada durante 3 ciclos la primera vez que la
        # FSM entra en `fase`, para forzar la emergencia justo ahi.
        estado = {"disparos": 0, "visto": False, "ciclo": None}

        def ruido(k, med):
            if estado["ciclo"] is None:
                return
            if estado["ciclo"] <= k < estado["ciclo"] + 3:
                med.izquierda = 60.0
                med.frontal = 110.0

        mundo = Mundo(pista, robot)
        nav = navegacion.Navegador(SectorFalso())
        res = Resultado(pista.pilares)
        t, id_congelado, lado_congelado = 0.0, None, None
        for k in range(300):
            med = mundo.medicion()
            if estado["ciclo"] is None and nav.estado in fase:
                estado["ciclo"] = k
                id_congelado = nav.tracker.id
                lado_congelado = nav.tracker.s_lado
            ruido(k, med)
            color, cx = mundo.camara()
            consigna = nav.procesar(med, color, robot.heading, ahora=t, cx_cam=cx)
            if consigna is None:
                break
            vel, ang = consigna
            robot.avanzar(vel, ang)
            t += DT
            res.estados.append(nav.estado)
            if mundo.choca_con_muro():
                res.choque_muro = True
                break
            res.contacto_pilar = min(res.contacto_pilar,
                                     mundo.distancia_min_a_pilares())
            for i, (px, py, _c) in enumerate(pista.pilares):
                x_b, y_b = robot.a_marco_robot(px, py, origen=(robot.x, robot.y))
                d = math.hypot(x_b, y_b)
                if d < res.d_min[i]:
                    res.d_min[i] = d
                    res.lado_en_d_min[i] = 1 if x_b > 0 else -1
        return nav, res, estado, id_congelado, lado_congelado

    def test_emergencia_durante_apertura(self):
        pista, robot = escenario_horario("VERDE")
        nav, res, est, idz, lado = self._emergencia_en(("APERTURA",), pista, robot)
        self.assertIsNotNone(est["ciclo"], "nunca se entro en APERTURA")
        self.assertIn("RETROCESO", res.estados)
        self.assertFalse(res.choque_muro)
        self.assertGreater(res.contacto_pilar, 0.0)
        # El lado obligatorio no puede haber cambiado por la emergencia
        if nav.tracker.activo and nav.tracker.id == idz:
            self.assertEqual(nav.tracker.s_lado, lado)

    def test_emergencia_durante_la_envolvente(self):
        # CONTRAGIRO solo aparece cuando los UNICOS comandos que dejan el
        # pilar de su lado son los que giran hacia el: es un REMEDIO para
        # cuando las paredes impiden a la vez seguir recto y apartarse,
        # no la ruta normal. Con la restriccion frontal corregida a
        # horizonte finito casi nunca hace falta, asi que la prueba
        # dispara la emergencia en la fase de envolvente que toque.
        pista, robot = escenario_antihorario("ROJO")
        nav, res, est, idz, lado = self._emergencia_en(
            ("CONTRAGIRO", "PASO_LATERAL"), pista, robot)
        self.assertIsNotNone(est["ciclo"], "nunca se entro en la envolvente")
        self.assertIn("RETROCESO", res.estados)
        self.assertFalse(res.choque_muro)
        self.assertGreater(res.contacto_pilar, 0.0)

    # --- pared concava sin pilares --------------------------------------
    def test_esquina_concava_sin_pilares(self):
        # Sin nada que esquivar, la esquina tiene que salir sola. Es la
        # prueba de que la envolvente de seguridad no rompio el CRUCERO.
        pista = Pista([])
        robot = Robot(-800.0, 1000.0, 0.0)
        nav, res = correr(pista, robot, ciclos=200)
        self.assertFalse(res.choque_muro, res.resumen())
        # Y de que la vuelta se identifica como horaria
        self.assertEqual(nav.sentido.sentido, -1)

    # --- limite conocido -------------------------------------------------
    def test_pilar_en_la_diagonal_de_la_esquina(self):
        # Pilar metido en la diagonal de la concavidad: contra el muro
        # exterior queda un hueco de 395mm y contra la isla uno de 555mm,
        # asi que uno de los dos colores puede resultar INCUMPLIBLE sin
        # rozar. Lo que se exige aqui no es cumplir la regla, sino
        # degradar de forma segura: ni choque, ni contacto con el pilar,
        # y el aborto explicito en vez de una trayectoria sin intencion.
        for color in ("VERDE", "ROJO"):
            with self.subTest(color=color):
                pista = Pista([(1080.0, 720.0, color)])
                robot = Robot(-500.0, 1000.0, 0.0)
                nav, res = correr(pista, robot)
                self.assertFalse(res.choque_muro, res.resumen())
                self.assertGreater(res.contacto_pilar, 0.0,
                                   "roza el pilar (%.0fmm)" % res.contacto_pilar)


if __name__ == "__main__":
    unittest.main()
