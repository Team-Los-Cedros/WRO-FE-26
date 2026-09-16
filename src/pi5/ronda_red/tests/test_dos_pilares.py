import math
import csv
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import dos_pilares as dp
import geometria_evasion as gev
import optica


def caja(color, x, y):
    cx = optica.cx_de_rumbo(optica.rumbo_camara_de_cluster(x, y))
    return dp.Caja(color, cx - 12, 90, cx + 12, 180, .9)


class DosPilaresTests(unittest.TestCase):
    def setUp(self):
        self.actual = SimpleNamespace(id=1, activo=True, color='VERDE', x=150., y=300., sigma=30., ambiguo=False)
        self.clusters = [(150., 300., 50.), (-250., 1050., 50.)]
        self.cajas = (caja('VERDE', 150, 300), caja('ROJO', -250, 1050))
        self.obs = dp.SiguientePilar()

    def alimentar(self, n=1, cajas=None, edad=0., en_seccion=lambda x, y: True):
        ahora = 10 + n * .125
        cuadro = dp.Cuadro(n, ahora - edad, 1000 + ahora, self.cajas if cajas is None else cajas)
        self.obs.actualizar(cuadro, self.clusters, self.actual, 0, 0, ahora, en_seccion)
        return cuadro, ahora

    def confirmar(self):
        for i in range(1, 4):
            self.alimentar(i)

    def test_band_aspect_ratio(self):
        cajas = dp.decodificar([[[.3, .2, .5, .3, .9]], []], 1, 0, 140, 640, 360, .35)
        self.assertEqual(len(cajas), 1)
        self.assertAlmostEqual(cajas[0].y0, 52)
        self.assertAlmostEqual(cajas[0].y1, 180)

    def test_descarta_bandas_nan_y_clases_desconocidas(self):
        filas = [[0, .2, .1, .3, .9], [.3, .2, .4, .3, float('nan')]]
        self.assertFalse(dp.decodificar([filas, [], [[.3, .2, .4, .3, .9]]], 1, 0, 140, 640, 360, .35))

    def test_dos_colores_y_paralaje(self):
        pares = dp.asociar(self.cajas, self.clusters)
        self.assertEqual([(c.color, x, y) for c, x, y in pares],
                         [('VERDE', 150., 300.), ('ROJO', -250., 1050.)])

    def test_dos_del_mismo_color(self):
        cajas = (self.cajas[0], caja('VERDE', -250, 1050))
        for i in range(1, 4):
            self.alimentar(i, cajas)
        self.assertEqual(self.obs.color, 'VERDE')
        self.assertEqual(self.obs.x, -250)
        self.assertEqual(self.obs.confirmaciones, 3)

    def test_un_cluster_no_recibe_dos_colores(self):
        cajas = (caja('ROJO', 150, 300), caja('VERDE', 150, 300))
        self.assertFalse(dp.asociar(cajas, [self.clusters[0]]))

    def test_identidad_actual_reserva_su_cluster_ante_sesgo_optico(self):
        rumbo = optica.rumbo_camara_de_cluster(self.actual.x, self.actual.y)
        cv = optica.cx_de_rumbo(rumbo - 8)
        cr = optica.cx_de_rumbo(rumbo - 1)
        cajas = (dp.Caja('VERDE', cv - 7, 80, cv + 7, 170, .95),
                 dp.Caja('ROJO', cr - 7, 100, cr + 7, 140, .95))
        self.clusters = self.clusters[:1]
        for i in range(1, 4):
            self.alimentar(i, cajas)
        self.assertEqual(self.obs.color, 'ROJO')
        self.assertEqual(self.obs.confirmaciones, 3)
        self.assertEqual(self.obs.fuente, 'visual_relativa')
        self.assertGreater(self.obs.y, self.actual.y + 150)

    def test_dos_clusters_en_mismo_rumbo_se_rechazan(self):
        r = math.radians(optica.rumbo_camara_de_cluster(150, 300))
        lejano = (optica.CAM_X + 900 * math.sin(r), optica.CAM_Y + 900 * math.cos(r), 50)
        self.assertFalse(dp.asociar((self.cajas[0],), [self.clusters[0], lejano]))

    def test_repetir_cuadro_no_confirma(self):
        cuadro, ahora = self.alimentar()
        for _ in range(10):
            self.obs.actualizar(cuadro, self.clusters, self.actual, 0, 0, ahora + .01, lambda x, y: True)
        self.assertEqual(self.obs.confirmaciones, 1)

    def test_vacio_rompe_racha(self):
        self.alimentar(1)
        self.alimentar(2, cajas=())
        self.assertIsNone(self.obs.color)
        self.alimentar(3)
        self.assertEqual(self.obs.confirmaciones, 1)

    def test_cuadro_viejo_no_rejuvenece(self):
        self.confirmar()
        t = self.obs.t
        self.alimentar(4, edad=.5)
        self.assertEqual(self.obs.t, t)
        self.assertEqual(self.obs.fuente, 'prediccion')
        self.assertFalse(self.obs.autoriza_control())

    def test_occlusion_guarda_identidad_sin_inventar_observaciones(self):
        self.confirmar()
        t = self.obs.t
        self.alimentar(4, cajas=())
        self.assertEqual(self.obs.color, 'ROJO')
        self.assertEqual(self.obs.t, t)
        self.assertFalse(self.obs.autoriza_control())
        self.alimentar(50, cajas=())
        self.assertIsNone(self.obs.color)

    def test_incertidumbre_excesiva_descarta_memoria(self):
        self.confirmar()
        self.obs.sigma = 301
        self.alimentar(4, cajas=())
        self.assertIsNone(self.obs.color)

    def test_memoria_reciente_solo_con_lidar_y_limites(self):
        self.confirmar()
        self.alimentar(4, cajas=())
        self.assertFalse(self.obs.autoriza_control())
        self.assertTrue(self.obs.autoriza_control(10.5, memoria=True))
        self.assertFalse(self.obs.autoriza_control(12., memoria=True))
        self.obs.sigma = 151
        self.assertFalse(self.obs.autoriza_control(10.5, memoria=True))
        self.obs.sigma = 40
        self.obs._origen_memoria = 'visual_relativa'
        self.assertFalse(self.obs.autoriza_control(10.5, memoria=True))
        self.obs._origen_memoria = 'lidar'
        self.obs._seccion_medida = False
        self.assertFalse(self.obs.autoriza_control(10.5, memoria=True))

    def test_cambio_actual_borra_confirmacion(self):
        self.confirmar()
        self.actual.id = 2
        self.alimentar(4)
        self.assertEqual(self.obs.confirmaciones, 1)

    def test_actual_incierto_no_autoriza(self):
        self.actual.sigma = 200
        self.confirmar()
        self.assertIsNone(self.obs.color)

    def test_actual_incierto_no_borra_siguiente_medido(self):
        self.confirmar()
        self.actual.sigma = 200
        self.alimentar(4)
        self.assertEqual(self.obs.color, 'ROJO')
        self.assertEqual(self.obs.motivo, 'confirmado_independiente')

    def test_actual_soltado_no_borra_siguiente_visible(self):
        self.confirmar()
        self.actual.activo = False
        self.alimentar(4)
        self.assertEqual(self.obs.color, 'ROJO')

    def test_siguiente_visual_sobrevive_sin_ver_al_actual(self):
        # Cajas proporcionales a la profundidad, para comprobar continuidad
        # con la misma geometria antes y despues de perder al cercano.
        cx = self.cajas[1].cx
        h = self.cajas[0].alto * (300 - optica.CAM_Y) / (1050 - optica.CAM_Y)
        lejos = dp.Caja('ROJO', cx - 10, 100, cx + 10, 100 + h, .9)
        self.clusters = self.clusters[:1]
        for i in range(1, 4):
            self.alimentar(i, (self.cajas[0], lejos))
        self.actual.sigma = 200
        self.alimentar(4, (lejos,))
        self.assertEqual(self.obs.color, 'ROJO')
        self.assertEqual(self.obs.fuente, 'visual_memoria')
        self.assertFalse(self.obs.autoriza_control())
        self.obs._t_escala -= 10
        self.alimentar(5, (lejos,))
        self.assertEqual(self.obs.fuente, 'prediccion')
        self.assertFalse(self.obs.autoriza_control())

    def test_siguiente_detras_de_pared_no_autoriza(self):
        for i in range(1, 4):
            self.alimentar(i, en_seccion=lambda x, y: False)
        self.assertEqual(self.obs.color, 'ROJO')
        self.assertFalse(self.obs.autoriza_control())

    def test_distancia_visual_solo_en_sombra(self):
        cx = self.cajas[1].cx
        lejano = dp.Caja('ROJO', cx - 10, 100, cx + 10, 140, .9)
        self.clusters = self.clusters[:1]
        for i in range(1, 4):
            self.alimentar(i, (self.cajas[0], lejano))
        self.assertEqual(self.obs.color, 'ROJO')
        self.assertEqual(self.obs.confirmaciones, 3)
        self.assertEqual(self.obs.fuente, 'visual_relativa')
        self.assertGreater(self.obs.y, self.actual.y)
        self.assertFalse(self.obs.autoriza_control())

    def test_prediccion_incluye_rotacion_y_traslacion(self):
        cuadro, ahora = self.alimentar()
        x, y = dp.geo.lidar_a_eje_trasero(self.obs.x, self.obs.y)
        xp, yp = dp.geo.eje_trasero_a_lidar(*gev.predecir_pilar(x, y, 20, 8))
        self.obs.actualizar(cuadro, self.clusters, self.actual, 8, 20, ahora + .1, lambda x, y: True)
        self.assertAlmostEqual(self.obs.x, xp)
        self.assertAlmostEqual(self.obs.y, yp)

    def test_propuesta_pertenece_al_conjunto_y_limita_cambio(self):
        self.confirmar()
        for base in range(-20, 26):
            candidatos = list(range(-20, 26, 2))
            cmd = self.obs.proponer(candidatos, base, 200, 70, 10.4, -1)
            self.assertTrue(cmd == base or cmd in candidatos)
            self.assertLessEqual(abs(cmd - base), dp.SESGO_MAX)

    def test_sin_salida_o_en_reversa_conserva_base(self):
        self.confirmar()
        self.assertEqual(self.obs.proponer([], 3, 200, 70, 10.4, -1), 3)
        self.assertEqual(self.obs.proponer([-4, 0, 4], 0, -200, 70, 10.4, -1), 0)

    def test_reproduccion_conserva_prediccion_y_relojes(self):
        from reproduccion_dos import RegistroReproduccion, reproducir
        esperados = []
        with tempfile.TemporaryDirectory() as tmp:
            ruta = str(Path(tmp) / 'entrada.jsonl')
            registro = RegistroReproduccion(ruta)
            for n in range(1, 7):
                t = 10 + n * .125
                cuadro = dp.Cuadro(n, t, 1000 + t, self.cajas if n < 4 else ())
                giro, avance = (0., 0.) if n < 4 else (5., 20.)
                registro.guardar(cuadro, self.clusters, self.actual, giro, avance, t,
                                 lambda x, y: True, 'PASO_LATERAL')
                self.obs.actualizar(cuadro, self.clusters, self.actual, giro, avance, t,
                                    lambda x, y: True)
                esperados.append((self.obs.color, self.obs.x, self.obs.y, self.obs.sigma, self.obs.fuente))
            registro.f.close()
            obtenidos = [(r['color'], r['x'], r['y'], r['sigma'], r['fuente']) for r in reproducir(ruta)]
            self.assertEqual(obtenidos, esperados)


class PrepararPasoTests(unittest.TestCase):
    """El rojo de la corrida 004228 al capturarlo: 380mm a la derecha y
    543mm delante, en marco del eje trasero, teniendo que quedar a la
    izquierda. Girando a la derecha se cruza; a la izquierda, no."""

    ROJO = (380.0, 543.0)
    REJILLA = [c / 2.0 for c in range(-40, 51)]

    def test_solo_cruzan_los_giros_hacia_el_pilar(self):
        cruzan = [c for c in self.REJILLA
                  if gev.cruza_al_lado(*self.ROJO, -1, c, 40.)]
        self.assertTrue(cruzan)
        self.assertTrue(all(c < 0 for c in cruzan))
        self.assertIn(-20.0, cruzan)
        self.assertNotIn(0.0, cruzan)

    def test_un_pilar_ya_de_su_lado_no_se_cruza(self):
        self.assertFalse(gev.cruza_al_lado(-380., 543., -1, -20., 40.))
        self.assertFalse(gev.cruza_al_lado(380., -200., -1, -20., 40.))

    def test_elige_el_de_mayor_margen_y_respeta_el_conjunto(self):
        candidatos = [-20., -15., -5., 0., 10.]
        cmd = dp.preparar_paso(candidatos, -10., *self.ROJO, -1, 40.)
        self.assertEqual(cmd, -20.)
        self.assertGreater(gev.margen_cruce(*self.ROJO, -1, -20.),
                           gev.margen_cruce(*self.ROJO, -1, -15.))

    def test_conserva_la_eleccion_anterior_mientras_siga_cruzando(self):
        candidatos = [-20., -15., -5., 0.]
        self.assertEqual(dp.preparar_paso(candidatos, -10., *self.ROJO, -1, 40., -15.), -15.)
        # Si la anterior ya no esta admitida, se vuelve a elegir.
        self.assertEqual(dp.preparar_paso([-20., -5.], -10., *self.ROJO, -1, 40., -15.), -20.)

    def test_sin_comando_que_cruce_devuelve_la_base(self):
        self.assertEqual(dp.preparar_paso([0., 5., 10.], 5., *self.ROJO, -1, 40.), 5.)

    def test_nunca_pide_mas_de_lo_que_el_limitador_da_en_un_ciclo(self):
        # Si la propuesta excede SESGO_MAX, el servo aplicaria un valor
        # intermedio que no es el elegido y la garantia geometrica se
        # evalua sobre un arco que no se esta recorriendo.
        self.assertLessEqual(dp.SESGO_MAX, 12.0)
        cmd = dp.preparar_paso([-20., -15.], -5., *self.ROJO, -1, 40.)
        self.assertEqual(cmd, -15.)


class CruceDelActualTests(unittest.TestCase):
    """El relevo de la corrida 015816: el rojo acaba de pasar a ser el
    pilar fichado, esta 318mm a la derecha y 615mm delante en marco del
    eje trasero, y tiene que quedar a la izquierda. La base viene del
    seguimiento de pared, que no sabe que el rojo existe, y vale +7,5."""

    ROJO = (318.5, 614.9)
    BASE = 7.5
    ALCANZABLES = [-15.0, -12.5, 7.5]

    def test_el_tope_respecto_a_base_deja_el_cruce_fuera_de_alcance(self):
        # Por que existe `sesgo=None`. El arco que cruza esta a 22,5
        # grados de una base que mira al lado contrario, no a 12.
        cmd = dp.preparar_paso(self.ALCANZABLES, self.BASE, *self.ROJO, -1, 40.)
        self.assertEqual(cmd, self.BASE)

    def test_sin_tope_se_elige_el_arco_que_cruza(self):
        cmd = dp.preparar_paso(self.ALCANZABLES, self.BASE, *self.ROJO, -1, 40.,
                               sesgo=None)
        self.assertEqual(cmd, -15.0)
        self.assertGreater(gev.margen_cruce(*self.ROJO, -1, cmd), 100.)

    def test_sin_tope_no_se_sale_del_conjunto_admitido(self):
        # Las paredes y el limitador del servo siguen mandando: lo que no
        # esta en `candidatos` no se puede elegir por mucho que cruce.
        cmd = dp.preparar_paso([-12.5, 7.5], self.BASE, *self.ROJO, -1, 40.,
                               sesgo=None)
        self.assertEqual(cmd, -12.5)

    def test_sin_tope_no_inventa_cruce_con_el_pilar_ya_de_su_lado(self):
        izquierda = (-318.5, 614.9)
        cmd = dp.preparar_paso(self.ALCANZABLES, self.BASE, *izquierda, -1, 40.,
                               sesgo=None)
        self.assertEqual(cmd, self.BASE)

    def test_sin_tope_conserva_la_eleccion_anterior(self):
        cmd = dp.preparar_paso(self.ALCANZABLES, self.BASE, *self.ROJO, -1, 40.,
                               anterior=-12.5, sesgo=None)
        self.assertEqual(cmd, -12.5)


class ApretarElGiroTests(unittest.TestCase):
    """Corrida 025702, t=4,72. El cruce arranco en -15 porque con el servo
    en -5 el limitador no daba mas; un ciclo despues -20 ya esta al
    alcance y gana 74mm de margen. Sostener -15 tres ciclos mas dejo el
    margen en 67mm y obligo a retroceder."""

    ROJO = (265.4, 580.2)
    ALCANZABLES = [-20., -17.5, -15., -12.5]

    def margen(self, cmd):
        return gev.margen_cruce(*self.ROJO, -1, cmd)

    def test_el_mismo_giro_mas_apretado_gana_a_la_histeresis(self):
        cmd = dp.preparar_paso(self.ALCANZABLES, 20., *self.ROJO, -1, 40.,
                               anterior=-15., sesgo=None, mejora=20.)
        self.assertEqual(cmd, -20.)
        self.assertGreater(self.margen(-20.) - self.margen(-15.), 20.)

    def test_sin_mejora_la_histeresis_sigue_mandando(self):
        # El pilar SIGUIENTE no pasa `mejora`: su comportamiento no cambia.
        cmd = dp.preparar_paso(self.ALCANZABLES, 20., *self.ROJO, -1, 40.,
                               anterior=-15., sesgo=None)
        self.assertEqual(cmd, -15.)

    def test_una_ganancia_por_debajo_del_umbral_no_cambia_la_eleccion(self):
        cmd = dp.preparar_paso(self.ALCANZABLES, 20., *self.ROJO, -1, 40.,
                               anterior=-15., sesgo=None, mejora=100.)
        self.assertEqual(cmd, -15.)

    def test_no_se_conserva_una_anterior_que_ya_no_cruza(self):
        cmd = dp.preparar_paso(self.ALCANZABLES, 20., *self.ROJO, -1, 40.,
                               anterior=12.5, sesgo=None, mejora=20.)
        self.assertEqual(cmd, -20.)


class IntegracionArbitrajeTests(unittest.TestCase):
    def navegador(self, modo, fuente='lidar', seguimiento=False):
        import navegacion
        n = navegacion.Navegador.__new__(navegacion.Navegador)
        n._comandos_seguros = lambda *a, **kw: ([-4., 0., 4., 8.], None)
        n._compatibles = lambda seguros, *a: seguros
        n.tracker = SimpleNamespace(activo=True, id=1, color='VERDE', x=200, y=-40,
                                    s_lado=1, xy_eje=lambda: (196, 88))
        n._dos_modo = modo
        n._dos_memoria = False
        n._dos = SimpleNamespace(proponer=lambda *a: 4., autoriza_control=lambda **kw: fuente == 'lidar',
                                 secuencia=1, color='ROJO', x=-200, y=1000,
                                 confirmaciones=3, motivo='confirmado', fuente=fuente, en_seccion=True,
                                 sigma=30., t=None)
        n._dos_seg = SimpleNamespace(proponer=lambda *a: -4., autoriza_control=lambda *a: seguimiento,
                                     color='ROJO', x=-200, y=1000, sigma=30., fuente=None,
                                     motivo='sin_identidad', racha=0, t_lidar=None)
        n._dos_preparacion = None
        n._dos_cruce = None
        n._dos_fichero = io.StringIO()
        n._dos_csv = csv.writer(n._dos_fichero)
        n._dos_filas = 0
        n.estado = 'PASO_LATERAL'
        n._ultimo_angulo = 0
        n._solo_obstaculo = False
        return n

    def test_sombra_calcula_pero_no_cambia_consigna(self):
        apagado, sombra = self.navegador('0'), self.navegador('sombra')
        self.assertEqual(apagado._arbitrar(0, None, 55), sombra._arbitrar(0, None, 55))
        fila = next(csv.reader(io.StringIO(sombra._dos_fichero.getvalue())))
        self.assertEqual(float(fila[14]), 4.)
        self.assertEqual(fila[15], '0')

    def test_activo_solo_acepta_posicion_habilitada(self):
        lidar, visual = self.navegador('activo'), self.navegador('activo', 'visual_relativa')
        self.assertEqual(lidar._arbitrar(0, None, 55), (4., True))
        self.assertEqual(visual._arbitrar(0, None, 55), (0., True))

    def test_apertura_no_anticipa_la_salida(self):
        # No es una preferencia: durante la apertura, los comandos que
        # garantizan el lado del pilar ACTUAL y los que cruzan el
        # SIGUIENTE son conjuntos disjuntos. Orbitar el verde para dejarlo
        # a la derecha obliga a girar a la izquierda; cruzar el rojo a su
        # izquierda obliga a girar a la derecha. Medido sobre 025702 y
        # 034105: cero comandos en comun en todos los ciclos de APERTURA.
        n = self.navegador('activo')
        n.estado = 'APERTURA'
        self.assertEqual(n._arbitrar(0, None, 55), (0., True))

    def test_recuperacion_si_prepara_el_siguiente(self):
        n = self.navegador('activo')
        n.estado = 'RECUPERACION'
        self.assertEqual(n._arbitrar(0, None, 55), (4., True))

    def test_el_seguimiento_manda_sobre_el_observador(self):
        n = self.navegador('activo', fuente='prediccion', seguimiento=True)
        self.assertEqual(n._arbitrar(0, None, 55), (-4., True))
        self.assertEqual(n._dos_preparacion, -4.)

    def relevo(self, modo='activo', xy=(318.5, 614.9)):
        """El rojo recien fichado, del lado prohibido, con la base del
        seguimiento de pared mirando al lado contrario (015816, t=6,34)."""
        n = self.navegador(modo)
        n._comandos_seguros = lambda *a, **kw: ([-15., -12.5, 7.5], None)
        n.tracker = SimpleNamespace(activo=True, id=2, color='ROJO', x=320, y=620,
                                    s_lado=-1, xy_eje=lambda: xy)
        n.estado = 'CONFIRMACION'
        n._ultimo_angulo = -5
        return n

    def test_confirmacion_cruza_el_pilar_recien_fichado(self):
        n = self.relevo()
        self.assertEqual(n._arbitrar(0, None, 55), (-15., True))
        self.assertEqual(n._dos_cruce, -15.)

    def test_el_cruce_no_actua_con_el_pilar_ya_de_su_lado(self):
        n = self.relevo(xy=(-318.5, 614.9))
        self.assertEqual(n._arbitrar(0, None, 55), (7.5, True))
        self.assertIsNone(n._dos_cruce)

    def test_el_cruce_no_actua_fuera_de_sus_estados(self):
        n = self.relevo()
        n.estado = 'CRUCERO'
        self.assertEqual(n._arbitrar(0, None, 55), (7.5, True))

    def test_sombra_registra_el_cruce_pero_no_lo_aplica(self):
        n = self.relevo(modo='sombra')
        self.assertEqual(n._arbitrar(0, None, 55), (7.5, True))
        fila = next(csv.reader(io.StringIO(n._dos_fichero.getvalue())))
        self.assertEqual(float(fila[40]), -15.)
        self.assertEqual(fila[41], '0')


if __name__ == '__main__':
    unittest.main()
