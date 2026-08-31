import copy
import unittest

from src.pi3B.ronda_nueva.config import cargar_configuracion
from src.pi3B.ronda_nueva.estacionamiento import ControlEstacionamiento
from src.pi3B.ronda_nueva.modelos import HuecoParqueo


def crear_hueco(borde_delantero_mm, timestamp=1.0, lado=1):
    return HuecoParqueo(
        timestamp=float(timestamp),
        lado=lado,
        borde_trasero_y_mm=float(borde_delantero_mm) - 353.0,
        borde_delantero_y_mm=float(borde_delantero_mm),
        centro_y_mm=float(borde_delantero_mm) - 176.5,
        separacion_mm=353.0,
        distancia_lateral_mm=360.0,
        confianza=0.95,
    )


def sensores_nominales(**cambios):
    sensores = {
        "trasera_valida": True,
        "cobertura_trasera": 1.0,
        "lateral_mm": 100.0,
        "lateral_valida": True,
        "trasera_izquierda_mm": 650.0,
        "trasera_derecha_mm": 650.0,
        "trasera_izquierda_valida": True,
        "trasera_derecha_valida": True,
        "cobertura_trasera_izquierda": 1.0,
        "cobertura_trasera_derecha": 1.0,
    }
    sensores.update(cambios)
    return sensores


class EstacionamientoTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()

    def _nuevo(self):
        controlador = ControlEstacionamiento(copy.deepcopy(self.config))
        hueco = crear_hueco(controlador.objetivo_alineacion_mm)
        return controlador, hueco

    def _hasta_arc_in(self):
        controlador, hueco = self._nuevo()
        resultado = controlador.procesar(
            hueco, 0.0, 500.0, 500.0, ahora=0.0, **sensores_nominales()
        )
        self.assertEqual(resultado.estado, "ALIGN")
        # Una deteccion basta: el hueco ya fue persistido por PercepcionLidar.
        resultado = controlador.procesar(
            hueco, 0.0, 500.0, 500.0, ahora=0.1, **sensores_nominales()
        )
        self.assertEqual(resultado.estado, "ARC_IN")
        return controlador, hueco

    def _hasta_center(self):
        controlador, hueco = self._hasta_arc_in()
        resultado = controlador.procesar(
            hueco, 42.0, 500.0, 500.0, ahora=0.2, **sensores_nominales()
        )
        self.assertEqual(resultado.estado, "ARC_OUT")
        self.assertLess(resultado.velocidad, 0)
        resultado = controlador.procesar(
            hueco, 0.0, 500.0, 500.0, ahora=0.3, **sensores_nominales()
        )
        self.assertEqual(resultado.estado, "CENTER")
        return controlador, hueco

    def _distancias_centradas(self, controlador):
        largo = float(self.config["parking"]["bay_length_mm"])
        delta = controlador.delta_centrado_objetivo_mm
        return (largo - delta) / 2.0, (largo + delta) / 2.0

    def _hasta_verify(self):
        controlador, hueco = self._hasta_center()
        frontal, trasera = self._distancias_centradas(controlador)
        resultado = controlador.procesar(
            hueco,
            0.0,
            frontal,
            trasera,
            ahora=0.4,
            **sensores_nominales(),
        )
        self.assertEqual(resultado.estado, "VERIFY")
        self.assertFalse(resultado.terminado)
        return controlador, hueco, frontal, trasera

    def test_secuencia_completa_exige_tres_barridos_y_termina_verificada(self):
        controlador, hueco, frontal, trasera = self._hasta_verify()

        for indice, ahora in enumerate((0.5, 0.6), start=1):
            resultado = controlador.procesar(
                hueco,
                0.0,
                frontal,
                trasera,
                ahora=ahora,
                **sensores_nominales(),
            )
            self.assertEqual(resultado.estado, "VERIFY")
            self.assertFalse(resultado.terminado)
            self.assertIn("{}/3".format(indice), resultado.razon)

        resultado = controlador.procesar(
            hueco,
            0.0,
            frontal,
            trasera,
            ahora=0.7,
            **sensores_nominales(),
        )
        self.assertEqual(resultado.estado, "DONE")
        self.assertTrue(resultado.terminado)
        self.assertTrue(resultado.verificado)
        self.assertEqual(resultado.velocidad, 0)

    def test_centro_corrige_offset_del_lidar(self):
        """El objetivo sale de la geometria, no de un numero fijo.

        Con el LiDAR medido al ras del morro el delta es 222 mm. Antes se
        creia 154 porque `lidar_forward_from_rear_axle_mm` valia 128, un
        valor de fotogrametria que el propio geometria_robot.py pedia
        sustituir por una medicion con regla; la real dio 162."""

        controlador, hueco = self._hasta_center()
        parking = self.config["parking"]
        esperado = 2.0 * (
            float(parking["rear_overhang_mm"])
            + float(parking["lidar_forward_from_rear_axle_mm"])
            - float(parking["robot_length_mm"]) / 2.0
        )
        self.assertAlmostEqual(esperado, 222.0)
        self.assertAlmostEqual(
            controlador.delta_centrado_objetivo_mm, esperado
        )

        # Igualar ambas holguras centraria el LiDAR, pero dejaria adelantado el
        # centro del robot. Debe ordenar avance para corregirlo.
        resultado = controlador.procesar(
            hueco,
            0.0,
            166.5,
            166.5,
            ahora=0.4,
            **sensores_nominales(),
        )
        self.assertEqual(resultado.estado, "CENTER")
        self.assertGreater(resultado.velocidad, 0)

        frontal, trasera = self._distancias_centradas(controlador)
        self.assertAlmostEqual(trasera - frontal, esperado)
        resultado = controlador.procesar(
            hueco,
            0.0,
            frontal,
            trasera,
            ahora=0.5,
            **sensores_nominales(),
        )
        self.assertEqual(resultado.estado, "VERIFY")

    def test_no_acepta_suma_incompatible_con_largo_del_hueco(self):
        controlador, hueco = self._hasta_center()
        resultado = controlador.procesar(
            hueco,
            0.0,
            200.0,
            354.0,
            ahora=0.4,
            **sensores_nominales(),
        )
        self.assertEqual(resultado.estado, "CENTER")
        self.assertEqual(resultado.velocidad, 0)
        self.assertFalse(resultado.terminado)

    def test_trasera_invalida_detiene_y_el_timeout_falla(self):
        controlador, hueco = self._hasta_arc_in()
        resultado = controlador.procesar(
            hueco,
            0.0,
            500.0,
            8000.0,
            ahora=0.2,
            **sensores_nominales(trasera_valida=False),
        )
        self.assertEqual(resultado.estado, "ARC_IN")
        self.assertEqual(resultado.velocidad, 0)
        self.assertFalse(resultado.terminado)
        self.assertFalse(resultado.verificado)

        resultado = controlador.procesar(
            hueco,
            0.0,
            500.0,
            8000.0,
            ahora=3.4,
            **sensores_nominales(trasera_valida=False),
        )
        self.assertEqual(resultado.estado, "FAILED")
        self.assertTrue(resultado.terminado)
        self.assertFalse(resultado.verificado)
        self.assertIn("timeout", resultado.razon)

    def test_busqueda_hacia_delante_no_confunde_sin_eco_con_obstaculo(self):
        controlador = ControlEstacionamiento(copy.deepcopy(self.config))
        resultado = controlador.procesar(
            None,
            0.0,
            800.0,
            8000.0,
            ahora=0.0,
            trasera_valida=False,
        )
        self.assertEqual(resultado.estado, "SEARCH_GAP")
        self.assertGreater(resultado.velocidad, 0)

    def test_eco_trasero_92_mm_no_autoriza_reversa_umbral_230(self):
        controlador, hueco = self._hasta_arc_in()
        resultado = controlador.procesar(
            hueco,
            0.0,
            500.0,
            92.0,
            ahora=0.2,
            **sensores_nominales(),
        )
        self.assertEqual(resultado.estado, "FAILED")
        self.assertEqual(resultado.velocidad, 0)
        self.assertFalse(resultado.verificado)
        self.assertIn("holgura trasera", resultado.razon)

    def test_arco_frena_con_diagonal_sin_dato_o_cobertura_insuficiente(self):
        controlador, hueco = self._hasta_arc_in()
        axial_sin_cobertura = controlador.procesar(
            hueco,
            0.0,
            500.0,
            600.0,
            ahora=0.2,
            **sensores_nominales(cobertura_trasera=0.0),
        )
        self.assertEqual(axial_sin_cobertura.estado, "ARC_IN")
        self.assertEqual(axial_sin_cobertura.velocidad, 0)
        self.assertIn("cobertura trasera no valida", axial_sin_cobertura.razon)

        sin_dato = controlador.procesar(
            hueco,
            0.0,
            500.0,
            600.0,
            ahora=0.3,
            **sensores_nominales(
                trasera_izquierda_mm=8000.0,
                trasera_izquierda_valida=True,
            ),
        )
        self.assertEqual(sin_dato.estado, "ARC_IN")
        self.assertEqual(sin_dato.velocidad, 0)
        self.assertFalse(sin_dato.terminado)
        self.assertIn("diagonal trasera izquierda no valida", sin_dato.razon)

        baja_cobertura = controlador.procesar(
            hueco,
            0.0,
            500.0,
            600.0,
            ahora=0.4,
            **sensores_nominales(cobertura_trasera_derecha=0.0),
        )
        self.assertEqual(baja_cobertura.estado, "ARC_IN")
        self.assertEqual(baja_cobertura.velocidad, 0)
        self.assertIn("diagonal trasera derecha no valida", baja_cobertura.razon)

    def test_arco_falla_sin_ordenar_reversa_ante_holgura_critica(self):
        controlador, hueco = self._hasta_arc_in()
        diagonal = controlador.procesar(
            hueco,
            0.0,
            500.0,
            600.0,
            ahora=0.2,
            **sensores_nominales(trasera_derecha_mm=180.0),
        )
        self.assertEqual(diagonal.estado, "FAILED")
        self.assertEqual(diagonal.velocidad, 0)
        self.assertTrue(diagonal.terminado)
        self.assertIn("diagonal trasera derecha critica", diagonal.razon)

        controlador, hueco = self._hasta_arc_in()
        lateral = controlador.procesar(
            hueco,
            0.0,
            500.0,
            600.0,
            ahora=0.2,
            **sensores_nominales(lateral_mm=70.0),
        )
        self.assertEqual(lateral.estado, "FAILED")
        self.assertEqual(lateral.velocidad, 0)
        self.assertTrue(lateral.terminado)
        self.assertIn("holgura lateral critica", lateral.razon)

    def test_arco_pausa_con_lateral_invalida_y_timeout_falla(self):
        controlador, hueco = self._hasta_arc_in()
        sensores = sensores_nominales(lateral_valida=False)
        detenido = controlador.procesar(
            hueco, 0.0, 500.0, 600.0, ahora=0.2, **sensores
        )
        self.assertEqual(detenido.estado, "ARC_IN")
        self.assertEqual(detenido.velocidad, 0)
        self.assertFalse(detenido.terminado)
        self.assertIn("distancia lateral no valida", detenido.razon)

        timeout = controlador.procesar(
            hueco, 0.0, 500.0, 600.0, ahora=3.4, **sensores
        )
        self.assertEqual(timeout.estado, "FAILED")
        self.assertEqual(timeout.velocidad, 0)
        self.assertTrue(timeout.terminado)
        self.assertIn("timeout", timeout.razon)

    def test_lateral_ausente_o_fuera_no_suma_verificaciones(self):
        controlador, hueco, frontal, trasera = self._hasta_verify()

        sin_dato = controlador.procesar(
            hueco,
            0.0,
            frontal,
            trasera,
            ahora=0.5,
            **sensores_nominales(lateral_mm=None, lateral_valida=False),
        )
        self.assertEqual(sin_dato.estado, "VERIFY")
        self.assertFalse(sin_dato.terminado)
        self.assertIn("medida lateral valida", sin_dato.razon)

        fuera = controlador.procesar(
            hueco,
            0.0,
            frontal,
            trasera,
            ahora=0.6,
            **sensores_nominales(lateral_mm=180.0),
        )
        self.assertEqual(fuera.estado, "VERIFY")
        self.assertFalse(fuera.terminado)
        self.assertIn("fuera del objetivo", fuera.razon)

        # Tras ambos rechazos el primer barrido bueno debe seguir siendo 1/3.
        bueno = controlador.procesar(
            hueco,
            0.0,
            frontal,
            trasera,
            ahora=0.7,
            **sensores_nominales(),
        )
        self.assertIn("1/3", bueno.razon)

    def test_timeout_no_se_convierte_en_exito_con_geometria_perfecta(self):
        controlador, hueco, frontal, trasera = self._hasta_verify()
        resultado = controlador.procesar(
            hueco,
            0.0,
            frontal,
            trasera,
            ahora=25.0,
            **sensores_nominales(),
        )
        self.assertEqual(resultado.estado, "FAILED")
        self.assertTrue(resultado.terminado)
        self.assertFalse(resultado.verificado)
        self.assertIn("timeout", resultado.razon)

    def test_align_deriva_objetivo_de_geometria_y_tolera_dos_perdidas(self):
        config = copy.deepcopy(self.config)
        config["parking"]["align_edge_trim_mm"] = 25.0
        config["parking"]["align_edge_y_mm"] = 999.0  # no debe dominar al nuevo
        controlador = ControlEstacionamiento(config)
        # Derivado de la geometria: -(lidar_desde_eje - medio_separador -
        # recorte). Con el LiDAR medido en el morro (162 mm) son -127; con
        # el valor viejo de fotogrametria (128) salian -93.
        esperado = -(
            float(config["parking"]["lidar_forward_from_rear_axle_mm"])
            - float(config["parking"]["separator_thickness_mm"]) / 2.0
            - float(config["parking"]["align_edge_trim_mm"])
        )
        self.assertAlmostEqual(esperado, -127.0)
        self.assertAlmostEqual(controlador.objetivo_alineacion_mm, esperado)

        hueco = crear_hueco(controlador.objetivo_alineacion_mm)
        controlador.procesar(hueco, 0.0, 500.0, 500.0, ahora=0.0)
        perdida_1 = controlador.procesar(None, 0.0, 500.0, 500.0, ahora=0.1)
        perdida_2 = controlador.procesar(None, 0.0, 500.0, 500.0, ahora=0.2)
        self.assertEqual(perdida_1.estado, "ALIGN")
        self.assertEqual(perdida_2.estado, "ALIGN")
        self.assertEqual(perdida_1.velocidad, 0)
        self.assertEqual(perdida_2.velocidad, 0)

        recuperado = controlador.procesar(
            hueco, 0.0, 500.0, 500.0, ahora=0.3
        )
        self.assertEqual(recuperado.estado, "ARC_IN")


if __name__ == "__main__":
    unittest.main()
