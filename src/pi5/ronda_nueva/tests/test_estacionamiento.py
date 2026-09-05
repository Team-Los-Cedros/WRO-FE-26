"""Maniobra de parqueo: cierra en varios tiempos y verifica antes de decir listo."""

import unittest

from ..estacionamiento import (
    ALINEAR,
    ARCO_ENDEREZA,
    ARCO_ENTRADA,
    BUSCAR,
    CENTRAR,
    FALLO,
    LISTO,
    VAIVEN_ADELANTE,
    VERIFICAR,
    ControlEstacionamiento,
)
from ..modelos import HuecoParqueo, MapaParedes, Recta


def config():
    return {
        "parking": {},
        "control": {"steering_max_left_deg": 25.0, "steering_max_right_deg": -20.0},
        "chassis": {"length_mm": 222.0, "width_mm": 125.0, "rear_overhang_mm": 60.0},
    }


def paredes(frontal=800.0, izquierda=270.0, trasera=400.0, angulo_izq=-90.0):
    return MapaParedes(
        timestamp=1.0,
        frontal=Recta(frontal, 0.0, 1.0, 20, 1.0),
        trasera=Recta(trasera, 180.0, 1.0, 20, 1.0),
        izquierda=Recta(izquierda, angulo_izq, 1.0, 30, 1.0),
        derecha=Recta(900.0, 90.0, 1.0, 30, 1.0),
        frontal_min_mm=frontal,
        trasera_min_mm=trasera,
        izquierda_min_mm=izquierda,
        derecha_min_mm=900.0,
        corredor_mm=frontal,
    )


def hueco(delantero=40.0, lateral=270.0):
    return HuecoParqueo(
        timestamp=1.0,
        lado=-1,
        borde_trasero_y_mm=delantero - 390.0,
        borde_delantero_y_mm=delantero,
        centro_y_mm=delantero - 195.0,
        separacion_mm=390.0,
        distancia_lateral_mm=lateral,
        confianza=0.78,
    )


class PruebaSecuencia(unittest.TestCase):
    def setUp(self):
        self.parqueo = ControlEstacionamiento(config())

    def test_sin_hueco_no_se_mueve_a_ciegas_y_acaba_por_timeout(self):
        """Es exactamente lo que paso el 03-09: 287 ciclos sin un candidato."""

        consigna = self.parqueo.procesar(paredes(), None, None, -1, 0.0)
        self.assertEqual(consigna.estado, BUSCAR)
        consigna = self.parqueo.procesar(paredes(), None, None, -1, 20.0)
        self.assertEqual(consigna.estado, FALLO)
        self.assertIn("hueco", consigna.razon)

    def test_con_hueco_pasa_a_alinear_y_luego_al_arco(self):
        self.parqueo.procesar(paredes(), hueco(), None, -1, 0.0)
        self.assertEqual(self.parqueo.estado, ALINEAR)
        # Con el borde delantero ya en el objetivo, se pasa al arco.
        self.parqueo.procesar(paredes(), hueco(delantero=45.0), None, -1, 0.3)
        self.assertEqual(self.parqueo.estado, ARCO_ENTRADA)

    def test_el_arco_de_entrada_termina_por_angulo(self):
        self.parqueo.procesar(paredes(), hueco(), None, -1, 0.0)
        self.parqueo.procesar(paredes(), hueco(delantero=45.0), None, -1, 0.3)
        consigna = self.parqueo.procesar(paredes(), hueco(), None, -1, 0.6)
        self.assertLess(consigna.velocidad, 0, "el arco de entrada va en reversa")
        # 45 grados de desviacion contra el muro: se pasa a enderezar.
        self.parqueo.procesar(
            paredes(angulo_izq=-45.0), hueco(), None, -1, 1.0
        )
        self.assertEqual(self.parqueo.estado, ARCO_ENDEREZA)

    def test_la_culata_cerca_corta_el_arco_y_abre_un_vaiven(self):
        """Los 9 mm que la simulacion daba de choque dejan de ser terminales."""

        self.parqueo.procesar(paredes(), hueco(), None, -1, 0.0)
        self.parqueo.procesar(paredes(), hueco(delantero=45.0), None, -1, 0.3)
        self.parqueo.procesar(paredes(trasera=90.0), hueco(), None, -1, 0.6)
        self.assertEqual(self.parqueo.estado, VAIVEN_ADELANTE)

    def test_el_ultrasonido_manda_si_ve_menos_que_el_lidar(self):
        """Se toma la MENOR de las dos: un fallo de cualquiera frena."""

        medida = self.parqueo._trasera_mm(paredes(trasera=400.0), 80.0)
        self.assertAlmostEqual(medida, 80.0, delta=1.0)
        medida = self.parqueo._trasera_mm(paredes(trasera=400.0), 900.0)
        self.assertLess(medida, 400.0)

    def test_un_ultrasonido_fuera_de_rango_se_ignora(self):
        medida = self.parqueo._trasera_mm(paredes(trasera=400.0), 5.0)
        self.assertGreater(medida, 100.0)

    def test_termina_verificado_solo_tras_varios_barridos_buenos(self):
        self.parqueo._entrar(VERIFICAR, 0.0)
        self.parqueo._lado = -1
        dentro = paredes(izquierda=130.0, trasera=160.0, frontal=200.0)
        for paso in range(2):
            consigna = self.parqueo.procesar(dentro, hueco(), None, -1, paso * 0.1)
            self.assertFalse(consigna.terminado)
        consigna = self.parqueo.procesar(dentro, hueco(), None, -1, 0.3)
        self.assertEqual(consigna.estado, LISTO)
        self.assertTrue(consigna.terminado)
        self.assertTrue(consigna.verificado)

    def test_no_verifica_si_no_esta_paralelo(self):
        self.parqueo._entrar(VERIFICAR, 0.0)
        self.parqueo._lado = -1
        torcido = paredes(izquierda=130.0, trasera=160.0, angulo_izq=-70.0)
        for paso in range(4):
            consigna = self.parqueo.procesar(torcido, hueco(), None, -1, paso * 0.1)
        self.assertNotEqual(consigna.estado, LISTO)

    def test_el_timeout_total_cierra_la_maniobra(self):
        self.parqueo.procesar(paredes(), hueco(), None, -1, 0.0)
        consigna = self.parqueo.procesar(paredes(), hueco(), None, -1, 120.0)
        self.assertEqual(consigna.estado, FALLO)
        self.assertTrue(consigna.terminado)
        self.assertFalse(consigna.verificado)

    def test_los_vaivenes_estan_acotados(self):
        """Sin tope, un robot que no entra se queda meciendose hasta el final."""

        self.parqueo._entrar(VAIVEN_ADELANTE, 0.0)
        self.parqueo._lado = -1
        self.parqueo._vaivenes = self.parqueo.max_vaivenes
        self.parqueo.procesar(paredes(), hueco(), None, -1, 0.1)
        self.assertEqual(self.parqueo.estado, VERIFICAR)


class PruebaGeometria(unittest.TestCase):
    def setUp(self):
        self.parqueo = ControlEstacionamiento(config())
        self.parqueo._lado = -1

    def test_el_volante_se_invierte_en_reversa(self):
        """Para meter la culata a la izquierda hay que girar a la derecha."""

        adelante = self.parqueo._mando(hacia_bahia=True, reversa=False)
        atras = self.parqueo._mando(hacia_bahia=True, reversa=True)
        self.assertLess(adelante * atras, 0.0)

    def test_el_paralelismo_se_mide_contra_el_muro_del_lado_de_la_bahia(self):
        self.assertAlmostEqual(self.parqueo._paralelo(paredes(angulo_izq=-90.0)), 0.0)
        self.assertAlmostEqual(self.parqueo._paralelo(paredes(angulo_izq=-78.0)), 12.0)

    def test_el_centrado_reparte_lo_que_sobra_de_bahia(self):
        """Bahia 390, robot 222: sobran 168, o sea 84 por lado."""

        self.parqueo._entrar(CENTRAR, 0.0)
        consigna = self.parqueo.procesar(paredes(trasera=144.0), hueco(), None, -1, 0.1)
        self.assertEqual(self.parqueo.estado, VERIFICAR)


if __name__ == "__main__":
    unittest.main()
