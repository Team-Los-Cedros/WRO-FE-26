# Pruebas de la geometria, del objetivo persistente y del arbitraje.
# No mueven el robot: comprueban las AFIRMACIONES en que se apoya el
# diseño, una por una. Correr con:
#
#   cd src/pi3B/ronda_curvas && python -m unittest discover -s tests -v
import math
import os
import sys
import unittest

CARPETA = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if CARPETA not in sys.path:
    sys.path.insert(0, CARPETA)
COMUN = os.path.join(os.path.dirname(CARPETA), "comun")
if COMUN not in sys.path:
    sys.path.append(COMUN)

import geometria_evasion as gev        # noqa: E402
import geometria_robot as geo          # noqa: E402
import navegacion                      # noqa: E402
import sentido_vuelta                  # noqa: E402
import tracker as tracker_mod          # noqa: E402


class MedicionFalsa:
    def __init__(self, frontal=2000.0, izquierda=500.0, derecha=500.0,
                 trasera=1000.0, frontal_muro=None, angulo_muro=0.0,
                 clusters=None):
        self.frontal = frontal
        self.frontal_muro = frontal if frontal_muro is None else frontal_muro
        self.izquierda = izquierda
        self.derecha = derecha
        self.trasera = trasera
        self.trasera_derecha = 800.0
        self.trasera_izquierda = 800.0
        self.clusters_obstaculo = clusters or []
        self.angulo_muro = angulo_muro


class SectorFalso:
    def fijar_sector_frontal(self, a, b):
        pass

    def sector_frontal_normal(self):
        pass


def cluster_falso(x, y, ancho_mm=55.0):
    # Genera un cluster de LiDAR sintetico centrado en (x, y) con el
    # ancho fisico pedido, para que ancho_cluster() y es_objeto_estrecho()
    # lo vean como lo verian en pista.
    d = math.hypot(x, y)
    b = math.degrees(math.atan2(x, y))
    ext = math.degrees(ancho_mm / d)
    n = max(3, int(ext) + 2)
    pts = []
    for i in range(n):
        a = b - ext / 2.0 + ext * i / (n - 1)
        pts.append(((a + 360.0) % 360.0, d))
    return pts


# ==========================================================
# 1. CONVENCIONES Y SIGNOS
# ==========================================================
class Convenciones(unittest.TestCase):
    def test_color_a_lado(self):
        # ROJO se pasa por su derecha -> queda a la IZQUIERDA del robot
        self.assertEqual(navegacion.lado_obligatorio("ROJO"), -1)
        self.assertEqual(navegacion.lado_obligatorio("VERDE"), 1)

    def test_apertura_y_envolvente_tienen_signos_OPUESTOS(self):
        # El nucleo del enunciado: "pasar por la izquierda del pilar" no
        # es "girar a la izquierda". Con un VERDE (pilar a la derecha) se
        # abre a la IZQUIERDA (comando positivo) y luego se envuelve a la
        # DERECHA (comando negativo).
        s = navegacion.lado_obligatorio("VERDE")     # +1
        apertura = gev.comando_apertura(0.0, 600.0, s, 300.0)
        self.assertGreater(apertura, 0.0, "abrir un verde es girar a la izquierda")

        envolvente = gev.comando_envolvente(260.0, 0.0, s, 40.0)
        self.assertIsNotNone(envolvente)
        self.assertLess(envolvente, 0.0, "envolver un verde es girar a la derecha")

    def test_rojo_es_el_espejo(self):
        s = navegacion.lado_obligatorio("ROJO")      # -1
        self.assertLess(gev.comando_apertura(0.0, 600.0, s, 300.0), 0.0)
        self.assertGreater(gev.comando_envolvente(-260.0, 0.0, s, 40.0), 0.0)


# ==========================================================
# 2. ENVOLVENTE
# ==========================================================
class Envolvente(unittest.TestCase):
    def test_g_es_creciente_en_R(self):
        # La demostracion de radio_envolvente se apoya en que
        # g(R) = (R-k) - hypot(u-R, y) crece con R. Si no, el criterio de
        # factibilidad (evaluar solo en RADIO_ENVOLVENTE_MAX) no vale.
        k = gev.SEMIANCHO + gev.RADIO_POSTE + 40.0
        for u in (120.0, 250.0, 400.0, 800.0):
            for y in (0.0, 200.0, 500.0):
                prev = -1e9
                for R in range(100, 1400, 25):
                    g = (R - k) - math.hypot(u - R, y)
                    self.assertGreaterEqual(g + 1e-9, prev)
                    prev = g

    def test_sin_separacion_no_hay_envolvente(self):
        # Con el pilar practicamente encima del eje no existe radio que
        # lo rodee: la respuesta correcta es abrir, no girar mas.
        factible, _ = gev.radio_envolvente(60.0, 300.0, 1, 40.0)
        self.assertFalse(factible)

    def test_el_pilar_queda_dentro_del_circulo(self):
        # Invariante de paso: si la envolvente es factible, el pilar esta
        # DENTRO del circulo de giro con la holgura pedida, y por tanto
        # no puede cambiar de lado durante todo el arco.
        holgura = 40.0
        factible, R = gev.radio_envolvente(300.0, 150.0, 1, holgura)
        self.assertTrue(factible)
        cmd = gev.comando_de_radio(R, hacia_izquierda=False)
        self.assertGreaterEqual(gev.holgura_arco(300.0, 150.0, cmd), holgura - 1.0)

    def test_holgura_negativa_si_el_arco_barre_el_pilar(self):
        # Pilar justo sobre la circunferencia que barre el costado
        cmd = -20.0
        R = gev.radio_de_comando(cmd)
        # (2R, 0) esta justo sobre la circunferencia que describe el eje
        # trasero: el costado del robot lo barre.
        self.assertLess(gev.holgura_arco(2.0 * R, 0.0, cmd), 0.0)
        # y el centro del circulo es el punto MAS libre de todos
        self.assertGreater(gev.holgura_arco(R, 0.0, cmd), 200.0)

    def test_separacion_ideal_es_el_radio_minimo_del_lado(self):
        # La separacion lateral que MAXIMIZA la holgura al tope de servo
        # es el radio minimo de ese lado. De ahi sale que envolver por la
        # derecha (R_min 360) exija ~100mm mas de apertura que por la
        # izquierda (260): la asimetria mecanica no es un detalle de
        # ajuste, cambia el objetivo geometrico.
        for cmd, esperado in ((-gev.COMANDO_MAX_DER, gev.RADIO_MIN_DER),
                              (gev.COMANDO_MAX_IZQ, gev.RADIO_MIN_IZQ)):
            s = 1 if cmd < 0 else -1
            mejor_u, mejor_h = None, -1e9
            for u in range(60, 700, 5):
                h = gev.holgura_arco(s * u, 0.0, cmd)
                if h > mejor_h:
                    mejor_u, mejor_h = u, h
            self.assertAlmostEqual(mejor_u, esperado, delta=12.0)

    def test_separacion_requerida_crece_con_la_distancia(self):
        # Cuanto mas por delante esta el pilar, mas separacion hace falta
        # para poder envolverlo dentro del carril.
        a = gev.separacion_requerida(100.0, 40.0)
        b = gev.separacion_requerida(400.0, 40.0)
        self.assertLess(a, b)
        self.assertIsNone(gev.separacion_requerida(900.0, 40.0))


# ==========================================================
# 3. OBJETIVO PERSISTENTE
# ==========================================================
class Objetivo(unittest.TestCase):
    def test_rotar_sobre_el_sitio_no_es_progreso(self):
        # REGRESION del fallo de raiz: el criterio anterior
        # (superado() = y < -280mm en marco chasis) lo cumplia una
        # rotacion pura, sin haber rebasado nada.
        trk = tracker_mod.TrackerObstaculo()
        trk.iniciar("VERDE", 1, 0.0, 500.0, 0.0, ahora=0.0)
        trk.marcar_compromiso()
        for i in range(1, 37):
            trk.predecir(heading=i * 5.0, avance_mm=0.0, ahora=i * 0.1)
        self.assertLess(trk.y, -280.0, "la rotacion pura si mueve y (por eso fallaba)")
        self.assertLess(abs(trk.progreso), 5.0,
                        "pero el barrido en marco mundo no se mueve")

    def test_trasladarse_alrededor_si_es_progreso(self):
        trk = tracker_mod.TrackerObstaculo()
        trk.iniciar("VERDE", 1, 300.0, 0.0, 0.0, ahora=0.0)
        trk.marcar_compromiso()
        for i in range(1, 20):
            trk.predecir(heading=0.0, avance_mm=30.0, ahora=i * 0.1)
        self.assertGreater(trk.progreso, 45.0)

    def test_dos_candidatos_iguales_no_asocian(self):
        # "Dos pilares visibles simultaneamente": preferimos predecir un
        # ciclo mas antes que cambiar de objetivo.
        trk = tracker_mod.TrackerObstaculo()
        trk.iniciar("VERDE", 1, 0.0, 500.0, 0.0, ahora=0.0)
        trk.sigma = 60.0
        ok = trk.asociar([(-60.0, 500.0, 55.0), (60.0, 500.0, 55.0)])
        self.assertFalse(ok)
        self.assertTrue(trk.ambiguo)
        self.assertEqual((trk.x, trk.y), (0.0, 500.0))

    def test_un_candidato_claro_si_asocia(self):
        trk = tracker_mod.TrackerObstaculo()
        trk.iniciar("VERDE", 1, 0.0, 500.0, 0.0, ahora=0.0)
        self.assertTrue(trk.asociar([(20.0, 505.0, 55.0)]))
        self.assertAlmostEqual(trk.x, 20.0)

    def test_el_ancho_rechaza_una_esquina_de_muro(self):
        # Una quilla de pared cae dentro de la puerta de posicion pero no
        # de la de ancho. Sin esta puerta el objetivo saltaba al muro.
        trk = tracker_mod.TrackerObstaculo()
        trk.iniciar("VERDE", 1, 0.0, 500.0, 0.0, ahora=0.0)
        self.assertFalse(trk.asociar([(30.0, 500.0, 420.0)]))

    def test_la_confianza_se_degrada_prediciendo(self):
        trk = tracker_mod.TrackerObstaculo()
        trk.iniciar("VERDE", 1, 0.0, 600.0, 0.0, ahora=0.0)
        c0 = trk.confianza
        for i in range(1, 12):
            trk.predecir(heading=i * 2.0, avance_mm=20.0, ahora=i * 0.12)
        self.assertLess(trk.confianza, c0)
        self.assertTrue(trk.degradado() or trk.sigma > 100.0)

    def test_la_separacion_garantizada_descuenta_la_incertidumbre(self):
        trk = tracker_mod.TrackerObstaculo()
        trk.iniciar("VERDE", 1, 200.0, 0.0, 0.0, ahora=0.0)
        trk.marcar_compromiso()
        limpia = trk.separacion_minima_garantizada()
        trk.sigma = 100.0
        trk.marcar_compromiso()
        sucia = trk.separacion_minima_garantizada()
        self.assertLess(sucia, limpia)


# ==========================================================
# 4. ARBITRAJE: SEGURIDAD COMO RESTRICCION
# ==========================================================
class Arbitraje(unittest.TestCase):
    def setUp(self):
        self.nav = navegacion.Navegador(SectorFalso())

    def test_pasillo_libre_no_toca_el_comando(self):
        med = MedicionFalsa(izquierda=500.0, derecha=500.0, frontal=2000.0)
        cmd, hay = self.nav._arbitrar(12.0, med, 40)
        self.assertTrue(hay)
        self.assertAlmostEqual(cmd, 12.5, delta=1.5)
        self.assertEqual(self.nav.seguridad, "LIBRE")

    def test_pared_cercana_recorta_pero_no_mezcla(self):
        # La pared izquierda cerca: el comando deseado a la izquierda se
        # RECORTA al mas parecido que sea seguro. Lo que no puede pasar es
        # que se sustituya por el centrado (que es lo que hacia la mezcla
        # anterior) ni que se invierta en silencio.
        med = MedicionFalsa(izquierda=140.0, derecha=900.0, frontal=2000.0)
        cmd, hay = self.nav._arbitrar(25.0, med, 40)
        self.assertTrue(hay)
        self.assertLess(cmd, 25.0)
        self.assertIn(self.nav.seguridad, ("RECORTA", "INVIERTE"))

    def test_callejon_sin_salida_se_detecta(self):
        med = MedicionFalsa(izquierda=95.0, derecha=95.0, frontal=180.0,
                            frontal_muro=180.0)
        cmd, hay = self.nav._arbitrar(0.0, med, 40)
        self.assertFalse(hay)
        self.assertEqual(self.nav.seguridad, "SIN_SALIDA")

    def test_los_dos_consumidores_de_angulo_muro_se_creen_lo_mismo(self):
        # Corrida 133838: la asistencia de esquina usaba `angulo_muro`
        # crudo y la envolvente lo acotaba a MAX_DERIVA_MURO. Con la
        # lectura de -37.4 grados del ciclo 46 la asistencia saturaba en
        # +15 y pedia un rumbo de +17.5 que la envolvente jamas concedia
        # (ejecutado: 0-10). Aqui se exige que el aporte no pueda pasar
        # de lo que la envolvente se cree: 12 * 0.65 = 7.8 grados.
        tope = navegacion.MAX_DERIVA_MURO * navegacion.KP_ANGULO_MURO
        self.assertAlmostEqual(tope, 7.8, delta=0.1)
        # Pasillo simetrico: el centrado por posicion aporta 0, asi que
        # lo que quede es exactamente el termino de angulo_muro.
        for lectura in (-37.4, -60.0, 37.4):
            with self.subTest(angulo_muro=lectura):
                med = MedicionFalsa(izquierda=300.0, derecha=300.0,
                                    frontal=800.0, frontal_muro=800.0,
                                    angulo_muro=lectura)
                self.assertLessEqual(abs(self.nav._rumbo_nominal(med)),
                                     tope + 0.1)
        # Y por debajo del tope la asistencia sigue siendo proporcional:
        # acotar la credibilidad no es apagar la señal.
        med = MedicionFalsa(izquierda=300.0, derecha=300.0,
                            frontal=800.0, frontal_muro=800.0,
                            angulo_muro=-6.0)
        self.assertAlmostEqual(self.nav._rumbo_nominal(med),
                               6.0 * navegacion.KP_ANGULO_MURO, delta=0.1)

    def test_el_umbral_frontal_es_el_horizonte_mas_74mm(self):
        # El hallazgo de las corridas 132913 y 133838, que nadie habia
        # escrito: la restriccion frontal tiene un umbral ANALITICO por
        # debajo del cual el conjunto admisible esta vacio POR
        # CONSTRUCCION, gire como gire el robot.
        #
        #   alcance_frontal(inf, h) = h + X_MORRO
        #   disponible = frontal_muro + LIDAR_X - MARGEN_FRENTE
        #   => existe algun comando  <=>  frontal_muro >= h + 74mm
        #
        # No es una heuristica ni un margen: es la definicion. Y explica
        # las dos corridas sin tocar un solo parametro. Con h=180 el
        # umbral vale 254mm para el comando recto y 242.6 para el mejor
        # arco, y las 12 rachas de SIN_SALIDA de 132913 entraron con
        # frontal_muro entre 213 y 242. Con h=150 el umbral bajo a 223.7
        # y las 10 rachas de 133838 entraron entre 190 y 223: la
        # distribucion se desplazo exactamente lo que se movio el umbral.
        #
        # Esta prueba fija la RELACION, no el valor, para que el umbral
        # siga siendo calculable cuando se cambie el horizonte.
        umbral_recto = (navegacion.HORIZONTE_FRENTE_MIN + gev.X_MORRO
                        - geo.LIDAR_X + navegacion.MARGEN_FRENTE)
        self.assertAlmostEqual(
            umbral_recto, navegacion.HORIZONTE_FRENTE_MIN + 74.0, delta=1.0)

        # Por debajo del umbral: vacio aunque los lados esten despejados.
        med = MedicionFalsa(izquierda=600.0, derecha=600.0,
                            frontal=umbral_recto - 40.0,
                            frontal_muro=umbral_recto - 40.0)
        self.assertFalse(self.nav._comandos_seguros(med, 25)[0])

        # Por encima: con el pasillo libre tiene que haber salida. Si
        # esto falla, el frente esta cerrando donde no le toca.
        med = MedicionFalsa(izquierda=600.0, derecha=600.0,
                            frontal=umbral_recto + 40.0,
                            frontal_muro=umbral_recto + 40.0)
        self.assertTrue(self.nav._comandos_seguros(med, 25)[0])

    def test_el_frente_y_no_los_lados_es_quien_vacia_el_conjunto(self):
        # La otra mitad del hallazgo: en 9 de las 10 rachas de 133838 los
        # laterales admitian la rejilla ENTERA (19 comandos) y el frente
        # los tumbaba todos. Se reproduce una de esas lecturas -- la del
        # ciclo 443, F=211 I=232 D=286 -- para dejar fijado que el
        # culpable es el frente. El dia que el rumbo nominal deje de
        # empotrarse, esta lectura no deberia darse; mientras se de, la
        # prueba dice a quien mirar.
        med = MedicionFalsa(izquierda=232.0, derecha=286.0,
                            frontal=211.0, frontal_muro=211.0)
        solo_lados = MedicionFalsa(izquierda=232.0, derecha=286.0,
                                   frontal=2000.0, frontal_muro=2000.0)
        self.assertEqual(len(self.nav._comandos_seguros(solo_lados, 25)[0]),
                         len(navegacion.CANDIDATOS))
        self.assertFalse(self.nav._comandos_seguros(med, 25)[0])

    def test_el_pilar_recorta_dentro_de_lo_seguro(self):
        # Con el pilar delante-derecha y pasillo libre, el arbitraje no
        # puede devolver un comando que lo barra.
        med = MedicionFalsa(izquierda=500.0, derecha=500.0, frontal=2000.0)
        pilar = (150.0, 260.0)
        cmd, hay = self.nav._arbitrar(-20.0, med, 40, pilar=pilar, holgura=40.0)
        self.assertTrue(hay)
        self.assertGreaterEqual(gev.holgura_arco(pilar[0], pilar[1], cmd), 0.0)

    def test_conflicto_pared_pilar_suspende_de_forma_explicita(self):
        # Si ningun comando seguro libra el pilar, gana la pared PERO
        # queda registrado. Antes esto ocurria en silencio dentro de la
        # mezcla y la maniobra se disolvia.
        med = MedicionFalsa(izquierda=130.0, derecha=130.0, frontal=1500.0)
        cmd, hay = self.nav._arbitrar(0.0, med, 40, pilar=(0.0, 150.0), holgura=40.0)
        self.assertTrue(hay)
        self.assertEqual(self.nav.seguridad, "SUSPENDE")


# ==========================================================
# 5. LA EMERGENCIA NO BORRA LA MEMORIA
# ==========================================================
class EmergenciaYMemoria(unittest.TestCase):
    def _nav_en_maniobra(self, estado="APERTURA"):
        nav = navegacion.Navegador(SectorFalso())
        nav.fase = "CARRERA"
        nav.estado = estado
        nav.tracker.iniciar("VERDE", 1, 150.0, 400.0, 0.0, ahora=0.0)
        nav.tracker.marcar_compromiso()
        nav._t_commit = 0.0
        nav._t_ultimo_progreso = 0.0
        return nav

    def test_emergencia_suspende_y_conserva_el_objetivo(self):
        for estado in ("APERTURA", "CONTRAGIRO", "PASO_LATERAL"):
            nav = self._nav_en_maniobra(estado)
            med = MedicionFalsa(frontal=90.0, izquierda=400.0, derecha=400.0)
            nav.procesar(med, "VERDE", 0.0, ahora=0.1)
            self.assertEqual(nav.estado, "RETROCESO")
            self.assertEqual(nav._estado_suspendido, estado)
            self.assertTrue(nav.tracker.activo,
                            "la emergencia no puede borrar el objetivo")
            self.assertEqual(nav.tracker.color, "VERDE")
            self.assertEqual(nav.tracker.s_lado, 1)

    def test_tras_la_emergencia_se_retoma_el_mismo_pilar(self):
        nav = self._nav_en_maniobra("CONTRAGIRO")
        idz = nav.tracker.id
        nav.procesar(MedicionFalsa(frontal=90.0), "VERDE", 0.0, ahora=0.1)
        self.assertEqual(nav.estado, "RETROCESO")
        t = 0.2
        for _ in range(30):
            med = MedicionFalsa(frontal=800.0, izquierda=500.0, derecha=500.0,
                                clusters=[cluster_falso(150.0, 380.0)])
            nav.procesar(med, "VERDE", 0.0, ahora=t)
            t += 0.12
            if nav.estado != "RETROCESO":
                break
        self.assertEqual(nav.estado, "CONTRAGIRO")
        self.assertTrue(nav.tracker.activo)
        self.assertEqual(nav.tracker.id, idz, "es el MISMO pilar, no uno nuevo")

    def test_otro_color_no_secuestra_la_maniobra(self):
        nav = self._nav_en_maniobra("PASO_LATERAL")
        idz, color = nav.tracker.id, nav.tracker.color
        t = 0.1
        for _ in range(6):
            med = MedicionFalsa(frontal=900.0, izquierda=450.0, derecha=450.0,
                                clusters=[cluster_falso(150.0, 300.0)])
            nav.procesar(med, "ROJO", 0.0, ahora=t)   # aparece un rojo
            t += 0.12
        self.assertEqual(nav.tracker.id, idz)
        self.assertEqual(nav.tracker.color, color)
        self.assertEqual(nav.tracker.s_lado, 1)


# ==========================================================
# 6. SENTIDO DE LA VUELTA
# ==========================================================
class Sentido(unittest.TestCase):
    def test_una_esquina_fija_el_sentido(self):
        s = sentido_vuelta.SentidoVuelta()
        t, h = 0.0, 0.0
        while h > -100.0:
            h -= 3.0
            t += 0.12
            s.actualizar(h, ahora=t)
        self.assertEqual(s.sentido, -1)      # horario

    def test_una_esquiva_no_fija_el_sentido(self):
        # +-35 grados de ida y vuelta: es una evasion, no una curva.
        s = sentido_vuelta.SentidoVuelta()
        t = 0.0
        for h in list(range(0, 36, 3)) + list(range(35, -1, -3)):
            t += 0.12
            s.actualizar(float(h), ahora=t)
        self.assertEqual(s.sentido, 0)

    def test_la_tangente_de_salida_usa_el_sentido(self):
        s = sentido_vuelta.SentidoVuelta()
        s.sentido = -1                                # horario
        self.assertAlmostEqual(s.rumbo_salida(10.0, en_esquina=True), -80.0)
        self.assertAlmostEqual(s.rumbo_salida(10.0, en_esquina=False), 10.0)

    def test_sin_sentido_no_se_inventa_tangente(self):
        s = sentido_vuelta.SentidoVuelta()
        self.assertIsNone(s.rumbo_salida(10.0, en_esquina=True))


if __name__ == "__main__":
    unittest.main()
