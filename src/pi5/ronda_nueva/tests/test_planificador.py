"""El planificador de carril, contra las disposiciones oficiales del sorteo.

Este es el banco de pruebas que decide si el recorrido vale.  No comprueba
umbrales internos: monta el modelo de bicicleta del chasis real, lo hace
recorrer una recta con el planificador al mando, y mide con geometria de
rectangulo si el robot rebaso cada poste POR EL LADO CORRECTO y sin tocarlo.
"""

import math
import unittest

from ..modelos import PoseCarril
from ..planificador import ConversorDireccion, PlanificadorCarril
from .apoyo import (
    FILA_EXTERIOR_MM,
    FILA_INTERIOR_MM,
    POS_P1,
    POS_P3,
    disposiciones_oficiales,
    evaluar_paso,
    simular_recta,
)


def config():
    return {
        "track": {"lane_width_mm": 1000.0, "pillar_width_mm": 100.0},
        "chassis": {
            "wheelbase_mm": 136.0,
            "width_mm": 125.0,
            "length_mm": 222.0,
            "turn_radius_left_mm": 228.0,
            "turn_radius_right_mm": 260.0,
        },
        "control": {"speed_cruise_pwm": 55, "mm_s_per_pwm": 4.0},
    }


class PruebaConversorDireccion(unittest.TestCase):
    def test_el_mando_no_es_el_angulo_de_rueda(self):
        """Con batalla 136 y radio 228, el tope real son 30,8 grados de rueda.

        El mando maximo es 25.  Tratarlos como lo mismo metia un 23 % de error
        a la izquierda y un 38 % a la derecha.
        """

        conversor = ConversorDireccion(config())
        self.assertAlmostEqual(conversor.rueda_max_izq, 30.8, delta=0.2)
        self.assertAlmostEqual(conversor.rueda_max_der, 27.6, delta=0.2)
        self.assertAlmostEqual(conversor.a_mando(conversor.rueda_max_izq), 25.0, places=4)
        self.assertAlmostEqual(conversor.a_mando(-conversor.rueda_max_der), -20.0, places=4)

    def test_satura_en_los_topes(self):
        conversor = ConversorDireccion(config())
        self.assertAlmostEqual(conversor.a_mando(90.0), 25.0, places=4)
        self.assertAlmostEqual(conversor.a_mando(-90.0), -20.0, places=4)


class PruebaLadoDePaso(unittest.TestCase):
    """Rojo por su derecha, verde por su izquierda; el offset invierte el signo."""

    def setUp(self):
        self.plan = PlanificadorCarril(config())

    def test_horario_el_exterior_esta_a_la_izquierda(self):
        # sentido +1: el offset crece hacia la DERECHA del robot.
        self.assertEqual(self.plan.lado_de_paso("ROJO", 400.0, 1), 1)
        self.assertEqual(self.plan.lado_de_paso("VERDE", 400.0, 1), -1)

    def test_antihorario_se_invierte(self):
        self.assertEqual(self.plan.lado_de_paso("ROJO", 400.0, -1), -1)
        self.assertEqual(self.plan.lado_de_paso("VERDE", 400.0, -1), 1)

    def test_el_objetivo_deja_holgura_y_no_pisa_la_pared(self):
        objetivo = self.plan.offset_de_paso("ROJO", FILA_INTERIOR_MM, 1)
        self.assertGreater(objetivo, FILA_INTERIOR_MM)
        margen = self.plan.medio_robot_mm + self.plan.holgura_pared_mm
        self.assertLessEqual(objetivo, 1000.0 - margen + 1e-6)


class PruebaGeometriaDelPlan(unittest.TestCase):
    def setUp(self):
        self.plan = PlanificadorCarril(config())

    def test_el_desplazamiento_maximo_respeta_la_curva_en_s(self):
        """En 500 mm no se pueden ganar mas de ~240 mm de lado con radio 260."""

        self.assertAlmostEqual(
            self.plan.desplazamiento_maximo(500.0), 500.0 ** 2 / (4 * 260.0), delta=1.0
        )
        # En tramos largos manda el cruce sostenido, no el radio.
        self.assertAlmostEqual(
            self.plan.desplazamiento_maximo(3000.0),
            3000.0 * math.tan(math.radians(self.plan.rumbo_max_ruta_deg)),
            delta=1.0,
        )

    def test_la_anchura_barrida_crece_al_cruzar(self):
        recto = self.plan.semiancho_barrido_mm(0.0)
        cruzado = self.plan.semiancho_barrido_mm(math.tan(math.radians(30.0)))
        self.assertAlmostEqual(recto, 62.5, places=3)
        self.assertGreater(cruzado, 100.0)

    def test_la_ruta_esta_ordenada_por_avance_decreciente(self):
        pose = PoseCarril(0.0, 0, 2900.0, 500.0, 0.0, True, True)
        ruta = self.plan.construir_ruta(
            pose, 1, [(POS_P1, FILA_EXTERIOR_MM, "ROJO"), (POS_P3, FILA_INTERIOR_MM, "VERDE")]
        )
        avances = [nodo.avance_mm for nodo in ruta]
        self.assertEqual(avances, sorted(avances, reverse=True))

    def test_los_pilares_de_detras_no_entran_en_el_plan(self):
        """El avance BAJA hacia adelante; la primera version tenia el filtro
        invertido y los postes solo entraban 700 mm antes de llegar."""

        pose = PoseCarril(0.0, 0, 2900.0, 500.0, 0.0, True, True)
        ruta = self.plan.construir_ruta(pose, 1, [(POS_P3, FILA_EXTERIOR_MM, "ROJO")])
        self.assertGreater(len(ruta), 1, "el pilar de delante tiene que estar en la ruta")
        atras = self.plan.construir_ruta(
            pose, 1, [(2900.0 + 900.0, FILA_EXTERIOR_MM, "ROJO")]
        )
        self.assertEqual(len(atras), 1, "lo que quedo muy atras no se planifica")

    def test_interpolacion_entre_nodos(self):
        from ..modelos import NodoRuta

        ruta = [NodoRuta(2000.0, 200.0), NodoRuta(1000.0, 800.0)]
        self.assertAlmostEqual(self.plan.offset_en(ruta, 2500.0), 200.0)
        self.assertAlmostEqual(self.plan.offset_en(ruta, 1500.0), 500.0)
        self.assertAlmostEqual(self.plan.offset_en(ruta, 500.0), 800.0)


class PruebaRecorridoCompleto(unittest.TestCase):
    """Las 28 disposiciones del sorteo, en los dos sentidos."""

    HOLGURA_MINIMA_MM = 15.0

    def test_todas_las_disposiciones_sin_ruido(self):
        plan = PlanificadorCarril(config())
        peor_global = float("inf")
        for sentido in (1, -1):
            for pilares, nombre in disposiciones_oficiales():
                traza = simular_recta(plan, sentido, pilares)
                holgura, fallos = evaluar_paso(traza, sentido, pilares)
                peor_global = min(peor_global, holgura)
                with self.subTest(sentido=sentido, disposicion=nombre):
                    self.assertEqual(fallos, [], f"holgura {holgura:.0f} mm")
        self.assertGreater(peor_global, self.HOLGURA_MINIMA_MM)

    def test_todas_las_disposiciones_con_ruido_de_medida(self):
        """25 mm de ruido en el pilar y 12 en la pose: mas de lo que se espera.

        La camara con homografia da unos 10-20 mm a un metro; se prueba con el
        doble para que el margen no dependa de que la calibracion salga fina.
        """

        plan = PlanificadorCarril(config())
        for semilla in (1, 2, 3):
            for sentido in (1, -1):
                for pilares, nombre in disposiciones_oficiales():
                    traza = simular_recta(
                        plan,
                        sentido,
                        pilares,
                        ruido_pilar_mm=25.0,
                        ruido_pose_mm=12.0,
                        semilla=semilla,
                    )
                    _holgura, fallos = evaluar_paso(traza, sentido, pilares)
                    with self.subTest(semilla=semilla, sentido=sentido, disposicion=nombre):
                        self.assertEqual(fallos, [])

    def test_sin_pilares_el_robot_se_queda_en_el_centro(self):
        plan = PlanificadorCarril(config())
        traza = simular_recta(plan, 1, [], offset_inicial=700.0)
        final = traza[-1]
        self.assertAlmostEqual(final[1], plan.offset_crucero_mm, delta=40.0)
        self.assertLess(abs(final[2]), 6.0)

    def test_la_direccion_no_da_saltos_bruscos_en_recta(self):
        plan = PlanificadorCarril(config())
        traza = simular_recta(plan, 1, [(POS_P2 := 1500.0, FILA_INTERIOR_MM, "VERDE")])
        saltos = [abs(b[3] - a[3]) for a, b in zip(traza, traza[1:])]
        self.assertLess(max(saltos), 12.0)


class PruebaEntradaDeEsquina(unittest.TestCase):
    """Abrirse hacia el muro exterior antes de tirar el volante a tope.

    Sin esto, en la corrida 2 del 04-09 el robot raspo el bloque interior en
    las cuatro esquinas: a radio 260 y entrando desde el centro del carril, la
    esquina delantera interior barre 117 mm dentro del bloque.
    """

    def _plan(self, offset=300.0, avance=900.0):
        cfg = config()
        cfg["control"]["corner_entry_offset_mm"] = offset
        cfg["control"]["corner_entry_avance_mm"] = avance
        return PlanificadorCarril(cfg)

    def _pose(self, avance):
        return PoseCarril(0.0, 0, avance, 500.0, 0.0, True, True)

    def test_apagada_la_ruta_termina_en_crucero(self):
        plan = self._plan(offset=0.0)
        ruta = plan.construir_ruta(self._pose(2600.0), 1, [])
        self.assertAlmostEqual(plan.offset_en(ruta, 300.0), plan.offset_crucero_mm)

    def test_encendida_se_abre_hacia_el_muro_exterior(self):
        plan = self._plan()
        ruta = plan.construir_ruta(self._pose(2600.0), 1, [])
        self.assertAlmostEqual(plan.offset_en(ruta, 900.0), 300.0, delta=1.0)
        # Lejos de la esquina no cambia nada: sigue siendo crucero.
        self.assertAlmostEqual(plan.offset_en(ruta, 2400.0), plan.offset_crucero_mm)

    def test_la_apertura_llega_por_una_rampa_no_de_golpe(self):
        plan = self._plan()
        ruta = plan.construir_ruta(self._pose(2600.0), 1, [])
        avances = [2000.0, 1700.0, 1400.0, 1100.0, 900.0]
        offsets = [plan.offset_en(ruta, a) for a in avances]
        self.assertTrue(
            all(b <= a + 1e-6 for a, b in zip(offsets, offsets[1:])),
            f"la apertura tiene que ser monotona: {offsets}",
        )
        saltos = [abs(b - a) for a, b in zip(offsets, offsets[1:])]
        self.assertLess(max(saltos), 160.0, "sin escalones de golpe")

    def test_un_pilar_cercano_sigue_mandando(self):
        """Sobre el poste manda el paso por su lado, no la apertura de esquina."""

        poste = [(1000.0, FILA_INTERIOR_MM, "VERDE")]
        base = self._plan(offset=0.0).construir_ruta(self._pose(2600.0), 1, poste)
        plan = self._plan(avance=700.0)
        ruta = plan.construir_ruta(self._pose(2600.0), 1, poste)
        self.assertAlmostEqual(
            plan.offset_en(ruta, 1000.0),
            plan.offset_en(base, 1000.0),
            delta=1.0,
        )

    def test_la_entrada_no_puede_comerse_la_meseta_del_poste(self):
        """Si se activa, ``corner_entry_avance_mm`` tiene que caber DEBAJO del
        poste mas cercano.  El sorteo pone el ultimo a 1000 mm del muro de
        enfrente y su meseta llega a 850: una entrada a 900 lo pisa.
        """

        poste = [(1000.0, FILA_INTERIOR_MM, "VERDE")]
        base = self._plan(offset=0.0).construir_ruta(self._pose(2600.0), 1, poste)
        esperado = self._plan(offset=0.0).offset_en(base, 900.0)
        pisa = self._plan(avance=900.0)
        respeta = self._plan(avance=700.0)
        self.assertNotAlmostEqual(
            pisa.offset_en(pisa.construir_ruta(self._pose(2600.0), 1, poste), 900.0),
            esperado,
            delta=1.0,
        )
        self.assertAlmostEqual(
            respeta.offset_en(
                respeta.construir_ruta(self._pose(2600.0), 1, poste), 900.0
            ),
            esperado,
            delta=1.0,
        )


class PruebaVelocidad(unittest.TestCase):
    def setUp(self):
        self.plan = PlanificadorCarril(config())
        self.pose = PoseCarril(0.0, 0, 2000.0, 500.0, 0.0, True, True)

    def test_frena_con_la_pared_de_enfrente(self):
        libre = self.plan.velocidad(self.pose, 0.0, 3000.0)
        cerca = self.plan.velocidad(self.pose, 0.0, 400.0)
        self.assertLess(cerca, libre)

    def test_frena_con_el_error_lateral(self):
        centrado = self.plan.velocidad(self.pose, 0.0, 3000.0)
        desviado = self.plan.velocidad(self.pose, 300.0, 3000.0)
        self.assertLess(desviado, centrado)

    def test_nunca_baja_del_minimo(self):
        cruzado = PoseCarril(0.0, 0, 2000.0, 500.0, 45.0, True, True)
        self.assertGreaterEqual(
            self.plan.velocidad(cruzado, 900.0, 200.0), self.plan.velocidad_min
        )


if __name__ == "__main__":
    unittest.main()
