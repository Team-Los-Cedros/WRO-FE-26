import copy
import inspect
import math
import sys
import unittest
from pathlib import Path


RAIZ_REPO = Path(__file__).resolve().parents[4]
if str(RAIZ_REPO) not in sys.path:
    sys.path.insert(0, str(RAIZ_REPO))

from src.pi3B.ronda_nueva.config import cargar_configuracion
from src.pi3B.ronda_nueva.control_ruta import ControlRuta
from src.pi3B.ronda_nueva.modelos import (
    Corredor,
    ResultadoParqueo,
    TrackObstaculo,
)


def corredor(
    frontal=1400.0,
    frontal_muro=1500.0,
    izquierda=500.0,
    derecha=500.0,
    error_lateral=None,
    error_rumbo=0.0,
    calidad=1.0,
    trasera=850.0,
    trasera_valida=True,
    diagonal_izquierda=900.0,
    diagonal_derecha=900.0,
    diagonales_validas=True,
    cobertura_trasera=1.0,
    cobertura_diagonal_izquierda=1.0,
    cobertura_diagonal_derecha=1.0,
    laterales_validas=True,
    timestamp=0.0,
):
    if error_lateral is None:
        error_lateral = izquierda - derecha
    return Corredor(
        timestamp=float(timestamp),
        frontal_mm=float(frontal),
        frontal_muro_mm=float(frontal_muro),
        trasera_mm=float(trasera),
        izquierda_mm=float(izquierda),
        derecha_mm=float(derecha),
        trasera_izquierda_mm=float(diagonal_izquierda),
        trasera_derecha_mm=float(diagonal_derecha),
        error_lateral_mm=float(error_lateral),
        error_rumbo_muro_deg=float(error_rumbo),
        calidad_pared=float(calidad),
        izquierda_valida=bool(laterales_validas),
        derecha_valida=bool(laterales_validas),
        trasera_valida=bool(trasera_valida),
        trasera_izquierda_valida=bool(diagonales_validas),
        trasera_derecha_valida=bool(diagonales_validas),
        cobertura_trasera=float(cobertura_trasera),
        cobertura_trasera_izquierda=float(cobertura_diagonal_izquierda),
        cobertura_trasera_derecha=float(cobertura_diagonal_derecha),
    )


def track(track_id, color, x=0.0, y=800.0, confirmado=True, timestamp=0.0):
    return TrackObstaculo(
        track_id=track_id,
        timestamp=float(timestamp),
        x_mm=float(x),
        y_mm=float(y),
        distancia_mm=math.hypot(x, y),
        bearing_deg=math.degrees(math.atan2(x, y)),
        color=color,
        confianza_color=0.95,
        impactos_lidar=3,
        impactos_color=3,
        edad_s=0.0,
        confirmado=confirmado,
    )


class ParkingFalso:
    def __init__(self, resultado):
        self.resultado = resultado
        self.llamada = None

    def reiniciar(self):
        pass

    def procesar(
        self,
        hueco,
        heading_deg,
        frontal_mm,
        trasera_mm,
        ahora=None,
        trasera_valida=True,
        cobertura_trasera=0.0,
        lateral_mm=None,
        lateral_valida=True,
        trasera_izquierda_mm=None,
        trasera_derecha_mm=None,
        trasera_izquierda_valida=False,
        trasera_derecha_valida=False,
        cobertura_trasera_izquierda=0.0,
        cobertura_trasera_derecha=0.0,
    ):
        self.llamada = {
            "hueco": hueco,
            "heading": heading_deg,
            "frontal": frontal_mm,
            "trasera": trasera_mm,
            "trasera_valida": trasera_valida,
            "cobertura_trasera": cobertura_trasera,
            "lateral": lateral_mm,
            "lateral_valida": lateral_valida,
            "trasera_izquierda": trasera_izquierda_mm,
            "trasera_derecha": trasera_derecha_mm,
            "trasera_izquierda_valida": trasera_izquierda_valida,
            "trasera_derecha_valida": trasera_derecha_valida,
            "cobertura_trasera_izquierda": cobertura_trasera_izquierda,
            "cobertura_trasera_derecha": cobertura_trasera_derecha,
            "ahora": ahora,
        }
        return self.resultado


class ControlRutaTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(cargar_configuracion())

    def _sin_slew(self):
        self.config["control"]["steering_slew_deg_per_scan"] = 100.0
        self.config["control"]["speed_slew_pwm_per_scan"] = 100

    def test_auto_no_mueve_hasta_resolver_sentido_y_timeout_falla(self):
        control = ControlRuta(self.config)
        orden = control.procesar(corredor(), (), 0.0, "PISTA", ahora=10.0)
        self.assertEqual(control.estado, "WAIT_DIRECTION")
        self.assertEqual((orden.velocidad, orden.angulo), (0, 0.0))
        self.assertEqual(control.sentido, 0)

        orden = control.procesar(corredor(), (), 0.0, "AZUL", ahora=10.1)
        self.assertEqual(control.estado, "CRUISE")
        self.assertEqual(control.sentido, 1)
        self.assertGreater(orden.velocidad, 0)

        horario = ControlRuta(self.config)
        horario.procesar(corredor(), (), 0.0, "NARANJA", ahora=20.0)
        self.assertEqual(horario.sentido, -1)

        espera = ControlRuta(self.config)
        espera.procesar(corredor(), (), 0.0, None, ahora=30.0)
        orden = espera.procesar(
            corredor(),
            (),
            0.0,
            "DESCONOCIDO",
            ahora=30.0 + self.config["control"]["direction_timeout_s"] + 0.01,
        )
        self.assertEqual(espera.estado, "FAILED")
        self.assertTrue(orden.terminado)
        self.assertFalse(orden.verificado)
        self.assertEqual(orden.velocidad, 0)

    def test_esquina_usa_delta_firmado_reapertura_confirmacion_y_conteo(self):
        self._sin_slew()
        self.config["control"]["corners_before_parking"] = 1
        control = ControlRuta(self.config)
        abierta = corredor(frontal=1200.0, frontal_muro=1200.0)
        cerrada = corredor(frontal=1000.0, frontal_muro=500.0)

        control.procesar(abierta, (), 170.0, "AZUL", ahora=0.0)
        control.procesar(cerrada, (), 170.0, "PISTA", ahora=0.1)
        orden = control.procesar(cerrada, (), 170.0, "PISTA", ahora=0.2)
        self.assertEqual(orden.estado, "TURN")
        self.assertGreater(orden.angulo, 0.0)

        # Cruce +180 -> -180: el delta antihorario sigue siendo +65 grados.
        control.procesar(abierta, (), -125.0, "PISTA", ahora=0.3)
        orden = control.procesar(abierta, (), -124.0, "PISTA", ahora=0.4)
        self.assertEqual(control.esquinas, 1)
        self.assertEqual(control.estado, "PARKING")
        self.assertEqual(control.lado_parqueo_solicitado, 1)
        self.assertEqual(orden.velocidad, 0)

    def test_esquina_tiene_prioridad_sobre_pilar_confirmado(self):
        """Regresion de pista: el pilar no debe desviar hacia la isla."""

        self._sin_slew()
        self.config["control"]["turn_direction"] = "RIGHT"
        control = ControlRuta(self.config)
        entrada = corredor(frontal=611.8, frontal_muro=612.0)
        pilar = track(2, "ROJO", x=120.0, y=520.0)

        primera = control.procesar(
            entrada, (pilar,), 0.0, "PISTA", ahora=0.0
        )
        self.assertEqual(control.estado, "CRUISE")
        self.assertEqual(primera.estado, "CRUISE")
        self.assertIsNone(control.track_activo_id)

        segunda = control.procesar(
            entrada, (pilar,), 0.0, "PISTA", ahora=0.1
        )
        self.assertEqual(control.estado, "TURN")
        self.assertEqual(segunda.estado, "TURN")
        self.assertLess(segunda.angulo, 0.0)
        self.assertIsNone(control.track_activo_id)

    def test_verde_pasa_izquierda_rojo_derecha_y_bloquea_id(self):
        self._sin_slew()
        pista = corredor()

        verde = ControlRuta(self.config)
        orden = verde.procesar(
            pista, (track(7, "VERDE"),), 0.0, "AZUL", ahora=0.0
        )
        self.assertEqual(verde.estado, "AVOID_APPROACH")
        self.assertGreater(orden.angulo, 0.0)

        # Un rojo nuevo, aunque este mas cerca, no reemplaza el ID ya unido.
        orden = verde.procesar(
            pista,
            (track(8, "ROJO", y=300.0), track(7, "VERDE", y=600.0)),
            0.0,
            "PISTA",
            ahora=0.1,
        )
        self.assertGreater(orden.angulo, 0.0)
        self.assertIn("7", orden.razon)

        rojo = ControlRuta(self.config)
        orden = rojo.procesar(
            pista, (track(2, "ROJO"),), 0.0, "NARANJA", ahora=1.0
        )
        self.assertEqual(rojo.estado, "AVOID_APPROACH")
        self.assertLess(orden.angulo, 0.0)

    def test_reasocia_id_del_mismo_pilar_y_no_persigue_posicion_congelada(self):
        self._sin_slew()
        pista = corredor()
        control = ControlRuta(self.config)

        control.procesar(
            pista, (track(2, "ROJO", x=-130.0, y=490.0),),
            0.0, "NARANJA", ahora=0.0,
        )
        self.assertEqual(control.estado, "AVOID_APPROACH")
        self.assertEqual(control.track_activo_id, 2)

        # La fusion regenero el ID durante la rotacion, pero la geometria y
        # el color permiten demostrar que sigue siendo el mismo pilar.
        orden = control.procesar(
            pista, (track(9, "ROJO", x=-155.0, y=430.0),),
            -10.0, "PISTA", ahora=0.1,
        )
        self.assertEqual(control.track_activo_id, 9)
        self.assertTrue(control.track_activo_observado)
        self.assertGreater(orden.velocidad, 0)

        # Sin una coincidencia segura, el mando antiguo nunca se prolonga.
        orden = control.procesar(
            pista, (), -12.0, "PISTA", ahora=0.2
        )
        self.assertEqual((orden.velocidad, orden.angulo), (0, 0.0))
        self.assertFalse(control.track_activo_observado)
        self.assertIn("perdido", orden.razon)

    def test_timeout_evasion_se_escala_con_pwm_reducido(self):
        self._sin_slew()
        self.config["control"]["speed_avoid_pwm"] = 25
        self.config["control"]["obstacle_timeout_s"] = 6.0
        self.config["control"]["obstacle_timeout_reference_pwm"] = 40.0
        pista = corredor()
        pilar = track(2, "ROJO", x=-130.0, y=490.0)
        control = ControlRuta(self.config)

        control.procesar(
            pista, (pilar,), 0.0, "NARANJA", ahora=0.0
        )
        orden = control.procesar(
            pista, (pilar,), 0.0, "PISTA", ahora=6.1
        )
        self.assertEqual(control.estado, "AVOID_APPROACH")
        self.assertFalse(orden.terminado)

        orden = control.procesar(
            pista, (pilar,), 0.0, "PISTA", ahora=9.61
        )
        self.assertEqual(control.estado, "FAILED")
        self.assertTrue(orden.terminado)

    def test_sobrepaso_reincorpora_por_distancia_si_pilar_sale_del_fov(self):
        self._sin_slew()
        self.config["control"]["speed_avoid_pwm"] = 25
        self.config["control"]["obstacle_pass_distance_mm"] = 200.0
        pista = corredor()
        control = ControlRuta(self.config)

        orden = control.procesar(
            pista, (track(2, "ROJO", x=-270.0, y=150.0),),
            -30.0, "NARANJA", ahora=0.0,
        )
        self.assertEqual(control.estado, "AVOID_PASS")
        self.assertGreater(orden.velocidad, 0)

        control.procesar(pista, (), -30.0, "PISTA", ahora=1.0)
        orden = control.procesar(
            pista, (), -30.0, "PISTA", ahora=2.0
        )
        self.assertEqual(control.estado, "RECENTER")
        self.assertAlmostEqual(control.distancia_sobrepaso_mm, 200.0)
        self.assertIn("200 mm", orden.razon)

    def test_timeout_recentrado_se_escala_con_pwm_reducido(self):
        self._sin_slew()
        self.config["control"]["speed_avoid_pwm"] = 25
        self.config["control"]["recenter_timeout_s"] = 3.0
        self.config["control"]["obstacle_timeout_reference_pwm"] = 40.0
        self.config["control"]["obstacle_pass_distance_mm"] = 200.0
        pista = corredor()
        descentrado = corredor(izquierda=900.0, derecha=200.0)
        control = ControlRuta(self.config)

        control.procesar(
            pista, (track(2, "ROJO", x=-270.0, y=150.0),),
            -30.0, "NARANJA", ahora=0.0,
        )
        control.procesar(pista, (), -30.0, "PISTA", ahora=1.0)
        control.procesar(pista, (), -30.0, "PISTA", ahora=2.0)
        self.assertEqual(control.estado, "RECENTER")

        orden = control.procesar(
            descentrado, (), -20.0, "PISTA", ahora=5.1
        )
        self.assertEqual(control.estado, "RECENTER")
        self.assertFalse(orden.terminado)

        orden = control.procesar(
            descentrado, (), -10.0, "PISTA", ahora=6.81
        )
        self.assertEqual(control.estado, "FAILED")
        self.assertTrue(orden.terminado)

    def test_recentrado_libera_con_dos_laterales_despejados_de_la_pista_real(self):
        """Regresion del recorrido 20260831_114618 antes de la esquina."""

        self._sin_slew()
        self.config["control"]["speed_avoid_pwm"] = 25
        self.config["control"]["obstacle_pass_distance_mm"] = 200.0
        control = ControlRuta(self.config)
        pista = corredor()

        control.procesar(
            pista, (track(2, "ROJO", x=-270.0, y=150.0),),
            -30.0, "NARANJA", ahora=0.0,
        )
        control.procesar(pista, (), -30.0, "PISTA", ahora=1.0)
        control.procesar(
            corredor(izquierda=882.2, derecha=138.8, calidad=0.813),
            (), -30.0, "PISTA", ahora=2.0,
        )
        self.assertEqual(control.estado, "RECENTER")

        primera = control.procesar(
            corredor(izquierda=724.4, derecha=593.7, calidad=0.799),
            (), 22.14, "PISTA", ahora=2.1,
        )
        self.assertEqual(control.estado, "RECENTER")
        self.assertFalse(primera.terminado)

        segunda = control.procesar(
            corredor(izquierda=722.8, derecha=605.5, calidad=0.701),
            (), 23.86, "PISTA", ahora=2.2,
        )
        self.assertEqual(control.estado, "CRUISE")
        self.assertEqual(segunda.razon, "reincorporacion verificada")

    def test_recentrado_entrega_esquina_confirmada_antes_del_timeout(self):
        """Regresion del recorrido 20260831_120547 junto al pilar rojo."""

        self._sin_slew()
        self.config["control"]["speed_avoid_pwm"] = 25
        self.config["control"]["obstacle_pass_distance_mm"] = 200.0
        control = ControlRuta(self.config)
        pista = corredor()

        control.procesar(
            pista, (track(2, "ROJO", x=-270.0, y=150.0),),
            -30.0, "NARANJA", ahora=0.0,
        )
        control.procesar(pista, (), -30.0, "PISTA", ahora=1.0)
        control.procesar(
            corredor(izquierda=873.8, derecha=138.5, calidad=0.798),
            (), -15.06, "PISTA", ahora=2.0,
        )
        self.assertEqual(control.estado, "RECENTER")

        primera = control.procesar(
            corredor(
                frontal=810.5, frontal_muro=810.5,
                izquierda=732.2, derecha=447.0, calidad=0.816,
            ),
            (), 20.63, "PISTA", ahora=5.69,
        )
        self.assertEqual(control.estado, "RECENTER")
        self.assertFalse(primera.terminado)

        segunda = control.procesar(
            corredor(
                frontal=804.5, frontal_muro=804.5,
                izquierda=729.3, derecha=443.5, calidad=0.634,
            ),
            (), 21.18, "PISTA", ahora=5.79,
        )
        self.assertEqual(control.estado, "TURN")
        self.assertFalse(segunda.terminado)
        self.assertIn("giro de esquina", segunda.razon)
        self.assertEqual(
            segunda.angulo,
            self.config["control"]["steering_max_right_deg"],
        )

    def test_emergencia_no_retrocede_con_trasera_ciega_y_diagonal_invalida_neutra(self):
        self._sin_slew()
        control = ControlRuta(self.config)
        control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)

        peligro_ciego = corredor(
            frontal=100.0, trasera=8000.0, trasera_valida=False
        )
        orden = control.procesar(
            peligro_ciego, (), 0.0, "PISTA", ahora=0.1
        )
        self.assertEqual(control.estado, "RECOVERY")
        self.assertEqual((orden.velocidad, orden.angulo), (0, 0.0))

        control = ControlRuta(self.config)
        control.procesar(corredor(), (), 0.0, "AZUL", ahora=1.0)
        peligro = corredor(
            frontal=100.0,
            trasera=700.0,
            trasera_valida=True,
            diagonal_izquierda=float("nan"),
            diagonal_derecha=600.0,
            diagonales_validas=False,
        )
        # Primer ciclo intercala cero antes de invertir el sentido de marcha.
        control.procesar(peligro, (), 0.0, "PISTA", ahora=1.1)
        orden = control.procesar(peligro, (), 0.0, "PISTA", ahora=1.2)
        self.assertLess(orden.velocidad, 0)
        self.assertEqual(orden.angulo, 0.0)

    def test_limites_asimetricos_y_slew_unico_de_salida(self):
        self.config["control"]["turn_direction"] = "LEFT"
        self.config["control"]["speed_slew_pwm_per_scan"] = 10
        self.config["control"]["steering_slew_deg_per_scan"] = 6.0
        control = ControlRuta(self.config)
        descentrado = corredor(error_lateral=1000.0, calidad=1.0)

        anterior_v = 0
        anterior_a = 0.0
        for indice in range(6):
            orden = control.procesar(
                descentrado, (), 0.0, None, ahora=float(indice) * 0.1
            )
            self.assertLessEqual(abs(orden.velocidad - anterior_v), 10)
            self.assertLessEqual(abs(orden.angulo - anterior_a), 6.0001)
            self.assertLessEqual(orden.angulo, 25.0)
            self.assertGreaterEqual(orden.angulo, -20.0)
            anterior_v, anterior_a = orden.velocidad, orden.angulo

        self.assertEqual(anterior_v, self.config["control"]["speed_cruise_pwm"])
        self.assertEqual(anterior_a, 25.0)

    def test_recuperaciones_en_ventana_comprometen_giro_al_sentido(self):
        self._sin_slew()
        self.config["control"]["forced_turn_after_recoveries"] = 2
        self.config["control"]["recovery_min_s"] = 0.05
        self.config["control"]["turn_direction"] = "RIGHT"
        control = ControlRuta(self.config)
        libre = corredor(frontal=900.0, frontal_muro=900.0)
        peligro = corredor(frontal=100.0, trasera=700.0)

        control.procesar(libre, (), 0.0, None, ahora=0.0)
        control.procesar(peligro, (), 0.0, None, ahora=0.1)
        control.procesar(libre, (), 0.0, None, ahora=0.2)
        self.assertEqual(control.estado, "CRUISE")

        control.procesar(peligro, (), 0.0, None, ahora=0.3)
        orden = control.procesar(libre, (), 0.0, None, ahora=0.4)
        self.assertEqual(control.estado, "FORCED_TURN")
        self.assertLess(orden.angulo, 0.0)
        self.assertIn("sentido de pista", orden.razon)

    def test_parqueo_recibe_validez_y_lateral_y_propaga_done_failed(self):
        self._sin_slew()
        resultado_ok = ResultadoParqueo(
            velocidad=0,
            angulo=0.0,
            estado="DONE",
            razon="geometria estable",
            terminado=True,
            verificado=True,
        )
        falso = ParkingFalso(resultado_ok)
        control = ControlRuta(self.config)
        control._estacionamiento = falso
        control._parametros_estacionamiento = set(
            inspect.signature(falso.procesar).parameters
        )
        control._sentido = 1
        control._lado_parqueo_solicitado = 1
        control._estado = "PARKING"
        control._t_estado = 5.0
        pista = corredor(derecha=100.0, izquierda=650.0, trasera_valida=True)

        orden = control.procesar(pista, (), 12.0, "PISTA", ahora=5.1)
        self.assertEqual(control.estado, "DONE")
        self.assertTrue(orden.terminado)
        self.assertTrue(orden.verificado)
        self.assertTrue(falso.llamada["trasera_valida"])
        self.assertEqual(falso.llamada["cobertura_trasera"], 1.0)
        self.assertEqual(falso.llamada["lateral"], 100.0)
        self.assertTrue(falso.llamada["lateral_valida"])
        self.assertEqual(falso.llamada["trasera_izquierda"], 900.0)
        self.assertEqual(falso.llamada["trasera_derecha"], 900.0)
        self.assertTrue(falso.llamada["trasera_izquierda_valida"])
        self.assertTrue(falso.llamada["trasera_derecha_valida"])

        resultado_fallo = ResultadoParqueo(
            velocidad=0,
            angulo=0.0,
            estado="FAILED",
            razon="hueco perdido",
            terminado=True,
            verificado=False,
        )
        falso = ParkingFalso(resultado_fallo)
        control = ControlRuta(self.config)
        control._estacionamiento = falso
        control._parametros_estacionamiento = set(
            inspect.signature(falso.procesar).parameters
        )
        control._sentido = -1
        control._lado_parqueo_solicitado = -1
        control._estado = "PARKING"
        control._t_estado = 8.0
        orden = control.procesar(corredor(), (), 0.0, "PISTA", ahora=8.1)
        self.assertEqual(control.estado, "FAILED")
        self.assertEqual(orden.velocidad, 0)
        self.assertTrue(orden.terminado)
        self.assertFalse(orden.verificado)

    def test_parqueo_inseguro_falla_sin_entrar_a_recuperacion(self):
        self._sin_slew()
        reversa = ResultadoParqueo(
            velocidad=-22,
            angulo=-20.0,
            estado="ARC_IN",
            razon="arco de entrada",
        )
        falso = ParkingFalso(reversa)
        control = ControlRuta(self.config)
        control._estacionamiento = falso
        control._parametros_estacionamiento = set(
            inspect.signature(falso.procesar).parameters
        )
        control._sentido = 1
        control._lado_parqueo_solicitado = 1
        control._estado = "PARKING"
        control._t_estado = 0.0

        diagonal_invalida = corredor(
            derecha=110.0,
            diagonales_validas=False,
            diagonal_izquierda=8000.0,
            diagonal_derecha=700.0,
        )
        orden = control.procesar(
            diagonal_invalida, (), 0.0, "PISTA", ahora=0.1
        )
        self.assertEqual(control.estado, "FAILED")
        self.assertNotEqual(control.estado, "RECOVERY")
        self.assertEqual(orden.velocidad, 0)
        self.assertTrue(orden.terminado)
        self.assertIn("diagonal izquierda segura", orden.razon)


if __name__ == "__main__":
    unittest.main()
