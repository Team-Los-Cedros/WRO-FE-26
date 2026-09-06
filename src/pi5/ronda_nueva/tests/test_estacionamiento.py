"""Maniobra de parqueo: cierra en varios tiempos y verifica antes de decir listo.

Las medidas que aparecen aqui salen de la sesion de banco del 06-09-2026
(``src/pi5/MEDICIONES_20260906.md``).  Los tres numeros que mandan sobre casi
todo el fichero son estos: la bahia util mide 330 mm, el ultrasonido esta 34 mm
por delante de la culata, y en la pose aparcada el LiDAR reportaba una pared
trasera a 1777 mm mientras el ultrasonido leia 44.
"""

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
    VAIVEN_ATRAS,
    VERIFICAR,
    ControlEstacionamiento,
)
from ..modelos import HuecoParqueo, MapaParedes, Recta


# Lectura de ultrasonido que deja la culata holgada: 200 - 34 de sensor a
# culata son 166 mm, muy por encima de min_rear_clearance_mm.
US_HOLGADO = 200.0


def config():
    return {
        "parking": {},
        "control": {"steering_max_left_deg": 25.0, "steering_max_right_deg": -20.0},
        "chassis": {
            "length_mm": 210.0,
            "width_mm": 130.0,
            "rear_overhang_mm": 55.0,
            "ultrasound_rear_to_tail_mm": 34.0,
            "turn_radius_left_mm": 228.0,
            "turn_radius_right_mm": 260.0,
        },
    }


def paredes(
    frontal=800.0,
    izquierda=270.0,
    trasera=400.0,
    angulo_izq=-90.0,
    timestamp=1.0,
):
    """Mapa de paredes de juguete.  ``izquierda=None`` = el LiDAR no la ve."""

    recta_izq = (
        None if izquierda is None else Recta(izquierda, angulo_izq, 1.0, 30, 1.0)
    )
    return MapaParedes(
        timestamp=timestamp,
        frontal=Recta(frontal, 0.0, 1.0, 20, 1.0),
        trasera=Recta(trasera, 180.0, 1.0, 20, 1.0),
        izquierda=recta_izq,
        derecha=Recta(900.0, 90.0, 1.0, 30, 1.0),
        frontal_min_mm=frontal,
        trasera_min_mm=trasera,
        izquierda_min_mm=float("inf") if izquierda is None else izquierda,
        derecha_min_mm=900.0,
        corredor_mm=frontal,
    )


def hueco(delantero=40.0, lateral=270.0, timestamp=1.0):
    return HuecoParqueo(
        timestamp=timestamp,
        lado=-1,
        borde_trasero_y_mm=delantero - 330.0,
        borde_delantero_y_mm=delantero,
        centro_y_mm=delantero - 165.0,
        separacion_mm=390.0,
        distancia_lateral_mm=lateral,
        confianza=0.78,
    )


class PruebaSecuencia(unittest.TestCase):
    def setUp(self):
        self.parqueo = ControlEstacionamiento(config())

    def _hasta_el_arco(self, us=US_HOLGADO):
        """Deja la FSM en ARCO_ENTRADA por el camino bueno, con rumbo 0."""

        self.parqueo.procesar(paredes(), hueco(), us, -1, 0.0, 0.0)
        self.parqueo.procesar(paredes(), hueco(delantero=45.0), us, -1, 0.1, 0.0)
        assert self.parqueo.estado == ARCO_ENTRADA, self.parqueo.estado

    def test_sin_hueco_no_se_mueve_a_ciegas_y_acaba_por_timeout(self):
        """Es exactamente lo que paso el 03-09: 287 ciclos sin un candidato."""

        consigna = self.parqueo.procesar(paredes(), None, None, -1, 0.0)
        self.assertEqual(consigna.estado, BUSCAR)
        consigna = self.parqueo.procesar(paredes(), None, None, -1, 20.0)
        self.assertEqual(consigna.estado, FALLO)
        self.assertIn("hueco", consigna.razon)

    def test_con_hueco_pasa_a_alinear_y_luego_al_arco(self):
        self.parqueo.procesar(paredes(), hueco(), US_HOLGADO, -1, 0.0)
        self.assertEqual(self.parqueo.estado, ALINEAR)
        # Con el borde delantero ya en el objetivo, se pasa al arco.
        self.parqueo.procesar(paredes(), hueco(delantero=45.0), US_HOLGADO, -1, 0.3)
        self.assertEqual(self.parqueo.estado, ARCO_ENTRADA)

    def test_un_hueco_viejo_no_alinea_contra_un_recuerdo(self):
        """``confirmar_hueco`` devuelve el ultimo confirmado indefinidamente.

        Si nadie mira la edad, ALINEAR persigue un borde que ya no esta
        delante y lo hace con cara de medida.
        """

        self.parqueo.procesar(paredes(), hueco(delantero=300.0), US_HOLGADO, -1, 0.0)
        self.assertEqual(self.parqueo.estado, ALINEAR)
        # Mismo hueco, pero el barrido de ahora es dos segundos posterior.
        consigna = self.parqueo.procesar(
            paredes(timestamp=3.0), None, US_HOLGADO, -1, 2.0
        )
        self.assertEqual(consigna.estado, BUSCAR)
        self.assertIn("perdido", consigna.razon)

    def test_alinear_se_aparta_del_muro_cuando_esta_demasiado_cerca(self):
        """El signo de la correccion lateral estaba invertido.

        Con la bahia a la IZQUIERDA y el robot mas cerca del muro de lo que
        pide approach_lateral_mm hay que girar a la DERECHA (angulo negativo).
        Antes se giraba a la izquierda, o sea contra el muro: realimentacion
        positiva justo en el tramo que prepara la entrada.
        """

        self.parqueo.procesar(
            paredes(izquierda=200.0), hueco(delantero=300.0), US_HOLGADO, -1, 0.0
        )
        consigna = self.parqueo.procesar(
            paredes(izquierda=200.0), hueco(delantero=300.0), US_HOLGADO, -1, 0.1
        )
        self.assertEqual(consigna.estado, ALINEAR)
        self.assertLess(consigna.angulo, 0.0)

    def test_el_arco_de_entrada_termina_por_angulo(self):
        self._hasta_el_arco()
        consigna = self.parqueo.procesar(paredes(), hueco(), US_HOLGADO, -1, 0.2, 0.0)
        self.assertLess(consigna.velocidad, 0, "el arco de entrada va en reversa")
        # 45 grados de desviacion contra el muro: se pasa a enderezar.
        self.parqueo.procesar(
            paredes(angulo_izq=-45.0), hueco(), US_HOLGADO, -1, 1.0, 0.0
        )
        self.assertEqual(self.parqueo.estado, ARCO_ENDEREZA)

    def test_el_arco_corta_por_la_imu_cuando_el_lidar_pierde_la_pared(self):
        """Cruzado dentro de un hueco de 330 mm no hay pared lateral que ver.

        Sin la IMU el tramo solo podia cortar por reloj, que es lo que hacia
        la maniobra irrepetible.  Rumbo positivo es giro a la izquierda, y eso
        lleva la normal del muro izquierdo de -90 a -90-delta: el paralelismo
        es MENOS el rumbo ganado.
        """

        self._hasta_el_arco()
        ciego = paredes(izquierda=None)
        self.parqueo.procesar(ciego, None, US_HOLGADO, -1, 0.3, 20.0)
        self.assertEqual(self.parqueo.estado, ARCO_ENTRADA, "20 grados no bastan")
        self.parqueo.procesar(ciego, None, US_HOLGADO, -1, 0.6, 45.0)
        self.assertEqual(self.parqueo.estado, ARCO_ENDEREZA)

    def test_perder_el_hueco_a_mitad_del_arco_no_rompe_la_maniobra(self):
        """Al cruzarse, los delimitadores dejan de emparejarse: es lo normal.

        Si el arco dependiera del detector, la maniobra se caeria justo en el
        unico sitio donde no se puede parar.
        """

        self._hasta_el_arco()
        for paso, instante in enumerate((0.3, 0.6, 0.9)):
            consigna = self.parqueo.procesar(
                paredes(izquierda=None, timestamp=10.0 + paso),
                None,
                US_HOLGADO,
                -1,
                instante,
                5.0 * paso,
            )
            self.assertEqual(consigna.estado, ARCO_ENTRADA)
            self.assertLess(consigna.velocidad, 0)

    def test_la_culata_cerca_corta_el_arco_y_abre_un_vaiven(self):
        """Los milimetros que la simulacion daba de choque dejan de ser terminales."""

        self._hasta_el_arco()
        # 90 de lectura son 56 mm de culata: por debajo de los 70 exigidos.
        self.parqueo.procesar(paredes(), hueco(), 90.0, -1, 0.3, 0.0)
        self.assertEqual(self.parqueo.estado, VAIVEN_ADELANTE)

    def test_termina_verificado_solo_tras_varios_barridos_buenos(self):
        self.parqueo._entrar(VERIFICAR, 0.0)
        self.parqueo._lado = -1
        dentro = paredes(izquierda=130.0, frontal=200.0)
        for paso in range(2):
            consigna = self.parqueo.procesar(
                dentro, hueco(), 160.0, -1, paso * 0.1
            )
            self.assertFalse(consigna.terminado)
        consigna = self.parqueo.procesar(dentro, hueco(), 160.0, -1, 0.3)
        self.assertEqual(consigna.estado, LISTO)
        self.assertTrue(consigna.terminado)
        self.assertTrue(consigna.verificado)

    def test_la_pose_medida_el_0609_ya_da_dentro_y_manda_centrar(self):
        """Regresion del hallazgo principal de la sesion de banco.

        Con el robot colocado a mano en la bahia el LiDAR daba 77,9 mm de
        lateral y normal -93,4, o sea 3,4 grados de paralelo: las dos medidas
        que la FSM no tenia (salian None) y sin las cuales VERIFICAR no podia
        cerrar nunca.  Con ellas ``_dentro()`` da True.

        Lo que esa pose NO estaba era centrada: el ultrasonido leia 44 mm, que
        descontados los 34 del sensor a la culata son 10 mm de sitio real
        contra los 60 que tocan.  La respuesta correcta no es fallar ni
        mecerse, es CENTRAR.
        """

        self.parqueo._entrar(VERIFICAR, 0.0)
        self.parqueo._lado = -1
        pose = paredes(izquierda=77.9, angulo_izq=-93.4, frontal=110.0)
        self.assertTrue(
            self.parqueo._dentro(77.9, self.parqueo._paralelo(pose)),
            "lateral y paralelo de la pose medida tienen que dar dentro",
        )
        for paso in range(3):
            self.parqueo.procesar(pose, None, 44.0, -1, paso * 0.1)
        self.assertEqual(self.parqueo.estado, VERIFICAR)
        consigna = self.parqueo.procesar(pose, None, 44.0, -1, 9.0)
        self.assertEqual(consigna.estado, CENTRAR)
        self.assertIn("corrido", consigna.razon)

    def test_la_pose_medida_el_0609_ya_centrada_cierra_la_verificacion(self):
        """La misma pose con la culata donde la deja CENTRAR: 60 mm.

        94 de lectura menos los 34 del sensor son los 60 mm que reparte
        ``_centrar`` con la bahia de 330 y el robot de 210.
        """

        self.parqueo._entrar(VERIFICAR, 0.0)
        self.parqueo._lado = -1
        pose = paredes(izquierda=77.9, angulo_izq=-93.4, frontal=110.0)
        for paso in range(3):
            consigna = self.parqueo.procesar(pose, None, 94.0, -1, paso * 0.1)
        self.assertEqual(consigna.estado, LISTO)
        self.assertTrue(consigna.verificado)

    def test_no_verifica_si_no_esta_paralelo(self):
        self.parqueo._entrar(VERIFICAR, 0.0)
        self.parqueo._lado = -1
        torcido = paredes(izquierda=130.0, angulo_izq=-70.0)
        for paso in range(4):
            consigna = self.parqueo.procesar(torcido, hueco(), 160.0, -1, paso * 0.1)
        self.assertNotEqual(consigna.estado, LISTO)

    def test_el_timeout_total_cierra_la_maniobra(self):
        self.parqueo.procesar(paredes(), hueco(), US_HOLGADO, -1, 0.0)
        consigna = self.parqueo.procesar(paredes(), hueco(), US_HOLGADO, -1, 120.0)
        self.assertEqual(consigna.estado, FALLO)
        self.assertTrue(consigna.terminado)
        self.assertFalse(consigna.verificado)

    def test_los_vaivenes_estan_acotados(self):
        """Sin tope, un robot que no entra se queda meciendose hasta el final."""

        self.parqueo._entrar(VAIVEN_ADELANTE, 0.0)
        self.parqueo._lado = -1
        self.parqueo._vaivenes = self.parqueo.max_vaivenes
        self.parqueo.procesar(paredes(), hueco(), US_HOLGADO, -1, 0.1)
        self.assertEqual(self.parqueo.estado, VERIFICAR)

    def test_agotar_los_vaivenes_acaba_parado_y_con_las_ruedas_rectas(self):
        """Un FALLO tiene que dejar el robot inocuo, no a medio vaiven."""

        self.parqueo._entrar(VAIVEN_ADELANTE, 0.0)
        self.parqueo._lado = -1
        self.parqueo._vaivenes = self.parqueo.max_vaivenes
        self.parqueo.procesar(paredes(izquierda=None), None, US_HOLGADO, -1, 0.1)
        consigna = self.parqueo.procesar(
            paredes(izquierda=None), None, US_HOLGADO, -1, 9.0
        )
        self.assertEqual(consigna.estado, FALLO)
        self.assertEqual(consigna.velocidad, 0)
        self.assertEqual(consigna.angulo, 0.0)
        self.assertTrue(consigna.terminado)
        self.assertFalse(consigna.verificado)

    def test_el_vaiven_se_corta_por_rumbo_y_no_por_reloj(self):
        """12 grados a los 15,7 deg/s medidos son unos 0,8 s.

        Lo que importa no es que coincida con el reloj viejo, sino que si el
        robot va mas lento el tramo dura mas en vez de quedarse corto.
        """

        self.parqueo._entrar(VAIVEN_ADELANTE, 0.0)
        self.parqueo._lado = -1
        self.parqueo.procesar(paredes(izquierda=None), None, US_HOLGADO, -1, 0.0, 0.0)
        consigna = self.parqueo.procesar(
            paredes(izquierda=None), None, US_HOLGADO, -1, 2.0, 5.0
        )
        self.assertEqual(consigna.estado, VAIVEN_ADELANTE, "5 grados no bastan")
        self.parqueo.procesar(paredes(izquierda=None), None, US_HOLGADO, -1, 2.2, 13.0)
        self.assertEqual(self.parqueo.estado, VAIVEN_ATRAS)
        self.assertEqual(self.parqueo._vaivenes, 1)


class PruebaTrasera(unittest.TestCase):
    """La holgura trasera sale del ultrasonido y de nada mas."""

    def setUp(self):
        self.parqueo = ControlEstacionamiento(config())
        self.parqueo._lado = -1

    def test_el_lidar_no_puede_sustituir_al_ultrasonido(self):
        """06-09, misma pose: LiDAR 1777 mm con calidad 0,95, ultrasonido 44.

        El LiDAR esta ciego justo hacia atras y esa trasera se reconstruye de
        los hombros en oblicuo: es una extrapolacion, y se equivoca en el
        sentido peligroso.
        """

        medida = self.parqueo._trasera_mm(paredes(trasera=1777.0), 44.0)
        self.assertAlmostEqual(medida, 10.0, delta=0.1)

    def test_se_descuenta_lo_que_el_sensor_esta_por_delante_de_la_culata(self):
        """34 mm, medidos con regla el 06-09.  44 de lectura son 10 de sitio."""

        self.assertAlmostEqual(
            self.parqueo._trasera_mm(paredes(), 104.0), 70.0, delta=0.1
        )

    def test_sin_ultrasonido_la_respuesta_es_sin_evidencia_y_no_libre(self):
        self.assertIsNone(self.parqueo._trasera_mm(paredes(trasera=400.0), None))

    def test_un_ultrasonido_fuera_de_rango_tampoco_es_libre(self):
        self.assertIsNone(self.parqueo._trasera_mm(paredes(trasera=400.0), 5.0))
        self.assertIsNone(self.parqueo._trasera_mm(paredes(trasera=400.0), 9000.0))

    def test_retroceder_sin_evidencia_trasera_acaba_en_fallo_limpio(self):
        """Un barrido sin eco es ruido; tres seguidos son retroceder a ciegas."""

        self.parqueo.procesar(paredes(), hueco(), US_HOLGADO, -1, 0.0, 0.0)
        self.parqueo.procesar(paredes(), hueco(delantero=45.0), US_HOLGADO, -1, 0.1, 0.0)
        self.assertEqual(self.parqueo.estado, ARCO_ENTRADA)
        for paso in range(2):
            consigna = self.parqueo.procesar(
                paredes(), None, None, -1, 0.2 + 0.1 * paso, 0.0
            )
            self.assertEqual(consigna.estado, ARCO_ENTRADA)
        consigna = self.parqueo.procesar(paredes(), None, None, -1, 0.5, 0.0)
        self.assertEqual(consigna.estado, FALLO)
        self.assertIn("evidencia trasera", consigna.razon)
        self.assertEqual(consigna.velocidad, 0)
        self.assertEqual(consigna.angulo, 0.0)


class PruebaGeometria(unittest.TestCase):
    def setUp(self):
        self.parqueo = ControlEstacionamiento(config())
        self.parqueo._lado = -1

    def test_el_volante_se_invierte_en_reversa(self):
        """Para meter la culata a la izquierda hay que girar a la derecha."""

        adelante = self.parqueo._mando(hacia_bahia=True, reversa=False)
        atras = self.parqueo._mando(hacia_bahia=True, reversa=True)
        self.assertLess(adelante * atras, 0.0)

    def test_el_arco_de_entrada_traza_el_radio_grande_por_los_dos_lados(self):
        """348 mm de hueco con el radio grande contra 363 con el chico (06-09).

        Con la bahia a la izquierda el tope ya da el radio grande y no hay que
        tocar nada.  Con la bahia a la derecha el tope daria 228, asi que se
        rebaja el volante en proporcion inversa al radio.
        """

        self.parqueo._lado = -1
        self.assertAlmostEqual(self.parqueo._mando_arco_entrada(), -20.0, delta=0.01)

        self.parqueo._lado = 1
        esperado = 25.0 * 228.0 / 260.0
        self.assertAlmostEqual(self.parqueo._mando_arco_entrada(), esperado, delta=0.01)
        self.assertLess(self.parqueo._mando_arco_entrada(), 25.0)

    def test_el_paralelismo_se_mide_contra_el_muro_del_lado_de_la_bahia(self):
        self.assertAlmostEqual(self.parqueo._paralelo(paredes(angulo_izq=-90.0)), 0.0)
        self.assertAlmostEqual(self.parqueo._paralelo(paredes(angulo_izq=-78.0)), 12.0)

    def test_la_imu_solo_habla_cuando_el_lidar_calla(self):
        self.parqueo._rumbo_paralelo = 0.0
        self.parqueo._rumbo = 30.0
        valor, fuente = self.parqueo._paralelo_medido(paredes(angulo_izq=-78.0))
        self.assertEqual(fuente, "lidar")
        self.assertAlmostEqual(valor, 12.0)

        valor, fuente = self.parqueo._paralelo_medido(paredes(izquierda=None))
        self.assertEqual(fuente, "imu")
        self.assertAlmostEqual(valor, -30.0)

    def test_sin_referencia_de_rumbo_no_se_inventa_un_paralelismo(self):
        valor, fuente = self.parqueo._paralelo_medido(paredes(izquierda=None))
        self.assertIsNone(valor)
        self.assertEqual(fuente, "")

    def test_el_centrado_reparte_lo_que_sobra_de_bahia(self):
        """Bahia 330, robot 210: sobran 120, o sea 60 por lado (06-09).

        Con el 390 viejo el objetivo eran 84 y el robot se quedaba 24 mm
        corrido hacia el fondo.
        """

        self.parqueo._entrar(CENTRAR, 0.0)
        # 94 de lectura son justo los 60 mm de objetivo: se da por centrado.
        consigna = self.parqueo.procesar(paredes(), hueco(), 94.0, -1, 0.1)
        self.assertEqual(self.parqueo.estado, VERIFICAR)
        self.assertEqual(consigna.velocidad, 0)

    def test_centrar_sin_trasera_no_mueve_el_robot(self):
        self.parqueo._entrar(CENTRAR, 0.0)
        consigna = self.parqueo.procesar(paredes(), hueco(), None, -1, 0.1)
        self.assertEqual(self.parqueo.estado, VERIFICAR)
        self.assertEqual(consigna.velocidad, 0)
        self.assertIn("sin medida trasera", consigna.razon)


if __name__ == "__main__":
    unittest.main()
