"""Ultrasonido trasero: firmware, telemetria y uso en el estacionamiento.

Cubre la cadena entera sin hardware: la logica de la Pico, el parseo de la
trama en la Pi, la fusion con la trasera del LiDAR dentro de la FSM de
parqueo y la guardia independiente de ``control_ruta``.
"""

import copy
import threading
import unittest

from src.pi3B.ronda_nueva.config import cargar_configuracion
from src.pi3B.ronda_nueva.control_ruta import ControlRuta
from src.pi3B.ronda_nueva.estacionamiento import (
    ControlEstacionamiento,
    ejecutar_estacionamiento,
)
from src.pi3B.ronda_nueva.hardware import EnlacePicoNuevo
from src.pi3B.ronda_nueva.modelos import MedidasParqueo, ResultadoParqueo
from src.pi3B.ronda_nueva.tests.test_control_ruta import corredor
from src.pi3B.ronda_nueva.tests.test_estacionamiento import (
    crear_hueco,
    sensores_nominales,
)
from src.pico.ultrasonido import (
    SIN_MEDIDA,
    FiltroUltrasonido,
    distancia_mm,
    mediana,
)


class LogicaPicoTests(unittest.TestCase):
    def test_convierte_el_ancho_del_eco_a_la_mitad_del_recorrido(self):
        # 2000 us de ida y vuelta a 343 m/s son 343 mm de recorrido total y
        # por tanto 343 mm... no: 2000*0.343 = 686 mm de recorrido, 343 de
        # distancia. Comprobado explicitamente porque olvidar el factor 2 es
        # el error clasico de este sensor.
        self.assertEqual(distancia_mm(2000), 343)
        self.assertEqual(distancia_mm(1000), 171)

    def test_un_eco_ausente_o_imposible_no_es_una_distancia(self):
        for entrada in (0, -5, None, "", "texto"):
            with self.subTest(entrada=entrada):
                self.assertEqual(distancia_mm(entrada), SIN_MEDIDA)

    def test_la_mediana_descarta_el_pico_aislado(self):
        self.assertEqual(mediana([300, 1900, 305]), 305)
        self.assertEqual(mediana([]), SIN_MEDIDA)

    def test_el_filtro_absorbe_un_rebote_suelto(self):
        filtro = FiltroUltrasonido(ventana=3, minima_mm=20, maxima_mm=4000)
        self.assertEqual(filtro.actualizar(1750), 300)      # 1750 us -> 300 mm
        self.assertEqual(filtro.actualizar(1800), 308)      # 1800 us -> 308 mm
        # Rebote lejano intercalado: sigue siendo una medida "posible" para el
        # sensor, asi que el rango no la descarta; la mediana si.
        self.assertEqual(filtro.actualizar(11000), 308)
        # Y si el eco largo se repite, deja de ser un pico y pasa a mandar.
        filtro.actualizar(11000)
        self.assertGreater(filtro.valor(), 1000)

    def test_sin_eco_repetido_la_medida_caduca_en_vez_de_congelarse(self):
        filtro = FiltroUltrasonido(ventana=3, fallos_max=3)
        self.assertEqual(filtro.actualizar(1750), 300)
        # Los dos primeros fallos conservan la ultima buena: un solo hueco no
        # puede paralizar la maniobra.
        self.assertEqual(filtro.actualizar(0), 300)
        self.assertEqual(filtro.actualizar(0), 300)
        # El tercero ya es una perdida sostenida y se declara sin medida.
        self.assertEqual(filtro.actualizar(0), SIN_MEDIDA)

    def test_una_lectura_buena_reanuda_el_filtro(self):
        filtro = FiltroUltrasonido(ventana=3, fallos_max=2)
        filtro.actualizar(0)
        filtro.actualizar(0)
        self.assertEqual(filtro.valor(), SIN_MEDIDA)
        self.assertEqual(filtro.actualizar(1750), 300)
        self.assertEqual(filtro.fallos_seguidos, 0)

    def test_rechaza_medidas_fuera_del_rango_configurado(self):
        filtro = FiltroUltrasonido(ventana=3, minima_mm=100, maxima_mm=500)
        self.assertEqual(filtro.actualizar(100), SIN_MEDIDA)      # ~17 mm
        self.assertEqual(filtro.actualizar(100000), SIN_MEDIDA)   # muy lejos


class TelemetriaUltrasonidoTests(unittest.TestCase):
    def test_lee_el_campo_us_sin_romper_el_contrato_historico(self):
        linea = "IMU:-12.5,COLOR:azul,US:185,WD:OK"
        trama = EnlacePicoNuevo.parsear_telemetria_completa(linea)
        self.assertEqual(trama.yaw, -12.5)
        self.assertEqual(trama.color, "AZUL")
        self.assertEqual(trama.watchdog, "OK")
        self.assertEqual(trama.ultrasonido_mm, 185.0)

        # Los parsers historicos siguen devolviendo lo mismo que antes.
        self.assertEqual(
            EnlacePicoNuevo.parsear_telemetria(linea), (-12.5, "AZUL")
        )
        self.assertEqual(
            EnlacePicoNuevo.parsear_telemetria_extendida(linea),
            (-12.5, "AZUL", "OK"),
        )

    def test_us_negativo_ausente_o_corrupto_significa_sin_evidencia(self):
        for linea in (
            "IMU:1.0,COLOR:PISTA,US:-1,WD:OK",       # el sensor no vio eco
            "IMU:1.0,COLOR:PISTA,WD:OK",             # firmware anterior
            "IMU:1.0,COLOR:PISTA,US:,WD:OK",         # trama truncada
            "IMU:1.0,COLOR:PISTA,US:nan,WD:OK",
            "IMU:1.0,COLOR:PISTA,US:abc,WD:OK",
        ):
            with self.subTest(linea=linea):
                trama = EnlacePicoNuevo.parsear_telemetria_completa(linea)
                self.assertIsNotNone(trama, "la IMU no puede perderse por el US")
                self.assertEqual(trama.yaw, 1.0)
                self.assertIsNone(trama.ultrasonido_mm)

    def _enlace_suelto(self, timeout_s=0.5):
        """Instancia sin abrir puerto: solo interesa el acceso a la medida."""

        enlace = EnlacePicoNuevo.__new__(EnlacePicoNuevo)
        enlace._lock = threading.Lock()
        enlace._timeout_s = float(timeout_s)
        enlace._t_telemetria = 0.0
        enlace._ultrasonido_mm = None
        return enlace

    def test_la_medida_caduca_con_la_trama_que_la_trajo(self):
        enlace = self._enlace_suelto(timeout_s=0.5)
        self.assertIsNone(enlace.distancia_ultrasonido_mm(ahora=10.0))

        enlace._t_telemetria = 10.0
        enlace._ultrasonido_mm = 240.0
        self.assertEqual(enlace.distancia_ultrasonido_mm(ahora=10.2), 240.0)
        # Pasado el watchdog de la Pico la medida deja de valer, aunque el
        # numero siga guardado.
        self.assertIsNone(enlace.distancia_ultrasonido_mm(ahora=11.0))


class FusionTraseraTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()

    def _controlador(self, **cambios_parking):
        config = copy.deepcopy(self.config)
        config["parking"].update(cambios_parking)
        return ControlEstacionamiento(config)

    def test_con_las_dos_fuentes_manda_la_mas_conservadora(self):
        controlador = self._controlador()
        distancia, disponible, por_us = controlador._fusionar_trasera(
            500.0, True, 320.0
        )
        self.assertTrue(disponible)
        self.assertTrue(por_us)
        self.assertEqual(distancia, 320.0)
        self.assertEqual(controlador.fuente_trasera, "LIDAR+US")

        # Si el LiDAR ve mas cerca, gana el LiDAR: nunca se usa el
        # ultrasonido para autorizar algo que el LiDAR desaconseja.
        distancia, _disponible, por_us = controlador._fusionar_trasera(
            280.0, True, 600.0
        )
        self.assertEqual(distancia, 280.0)
        self.assertFalse(por_us)

    def test_el_ultrasonido_cubre_la_trasera_ciega_del_lidar(self):
        controlador = self._controlador()
        distancia, disponible, por_us = controlador._fusionar_trasera(
            0.0, False, 300.0
        )
        self.assertTrue(disponible)
        self.assertTrue(por_us)
        self.assertEqual(distancia, 300.0)
        self.assertEqual(controlador.fuente_trasera, "US")

    def test_sin_ninguna_fuente_no_hay_trasera(self):
        controlador = self._controlador()
        _distancia, disponible, _por_us = controlador._fusionar_trasera(
            0.0, False, None
        )
        self.assertFalse(disponible)
        self.assertEqual(controlador.fuente_trasera, "NINGUNA")

    def test_un_eco_fuera_de_rango_no_entra_en_la_fusion(self):
        controlador = self._controlador(
            ultrasound_rear_min_mm=20.0, ultrasound_rear_max_mm=1200.0
        )
        for medida in (5.0, 3000.0, float("nan"), None):
            with self.subTest(medida=medida):
                self.assertFalse(controlador.ultrasonido_utilizable(medida))

    def test_se_puede_desactivar_por_configuracion(self):
        controlador = self._controlador(ultrasound_rear_enabled=False)
        self.assertFalse(controlador.ultrasonido_utilizable(300.0))
        _distancia, disponible, _por_us = controlador._fusionar_trasera(
            0.0, False, 300.0
        )
        self.assertFalse(disponible)


class ArcosConUltrasonidoTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()

    def _hasta_arc_in(self, **cambios_parking):
        config = copy.deepcopy(self.config)
        config["parking"].update(cambios_parking)
        controlador = ControlEstacionamiento(config)
        hueco = crear_hueco(controlador.objetivo_alineacion_mm)
        controlador.procesar(
            hueco, 0.0, 500.0, 500.0, ahora=0.0, **sensores_nominales()
        )
        resultado = controlador.procesar(
            hueco, 0.0, 500.0, 500.0, ahora=0.1, **sensores_nominales()
        )
        self.assertEqual(resultado.estado, "ARC_IN")
        return controlador, hueco

    def test_el_arco_sigue_retrocediendo_con_la_trasera_lidar_ciega(self):
        controlador, hueco = self._hasta_arc_in()
        # Sin ultrasonido, una trasera sin cobertura frena el arco.
        ciego = sensores_nominales(trasera_valida=False, cobertura_trasera=0.0)
        parado = controlador.procesar(
            hueco, 10.0, 500.0, 8000.0, ahora=0.2, **ciego
        )
        self.assertEqual(parado.velocidad, 0)
        self.assertIn("cobertura trasera no valida", parado.razon)

        # Con el sensor viendo 400 mm libres, la misma situacion permite
        # continuar: es el punto ciego del mastil lo que se estaba cubriendo.
        con_sensor = controlador.procesar(
            hueco, 10.0, 500.0, 8000.0, ahora=0.3,
            ultrasonido_trasero_mm=400.0, **ciego
        )
        self.assertLess(con_sensor.velocidad, 0)
        self.assertEqual(con_sensor.estado, "ARC_IN")
        self.assertEqual(controlador.fuente_trasera, "US")

    def test_el_ultrasonido_no_puede_saltarse_la_holgura_minima(self):
        controlador, hueco = self._hasta_arc_in()
        ciego = sensores_nominales(trasera_valida=False, cobertura_trasera=0.0)
        resultado = controlador.procesar(
            hueco, 10.0, 500.0, 8000.0, ahora=0.2,
            ultrasonido_trasero_mm=92.0, **ciego
        )
        self.assertEqual(resultado.estado, "FAILED")
        self.assertIn("holgura trasera", resultado.razon)

    def test_el_ultrasonido_no_sustituye_a_las_diagonales(self):
        # El sensor mira recto hacia atras; las esquinas traseras las sigue
        # midiendo el LiDAR y su ausencia tiene que seguir frenando el arco.
        controlador, hueco = self._hasta_arc_in()
        sin_diagonales = sensores_nominales(
            trasera_izquierda_valida=False,
            cobertura_trasera_izquierda=0.0,
        )
        resultado = controlador.procesar(
            hueco, 10.0, 500.0, 500.0, ahora=0.2,
            ultrasonido_trasero_mm=400.0, **sin_diagonales
        )
        self.assertEqual(resultado.velocidad, 0)
        self.assertIn("diagonal trasera izquierda", resultado.razon)


class EjecutarEstacionamientoTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()

    def test_la_entrada_de_alto_nivel_equivale_a_llamar_a_la_fsm(self):
        hueco = None
        medidas = MedidasParqueo(
            frontal_mm=800.0,
            trasera_mm=500.0,
            trasera_valida=True,
            cobertura_trasera=1.0,
            lateral_mm=100.0,
            lateral_valida=True,
            trasera_izquierda_mm=650.0,
            trasera_derecha_mm=650.0,
            trasera_izquierda_valida=True,
            trasera_derecha_valida=True,
            cobertura_trasera_izquierda=1.0,
            cobertura_trasera_derecha=1.0,
            ultrasonido_trasero_mm=430.0,
        )
        por_alto_nivel = ejecutar_estacionamiento(
            ControlEstacionamiento(copy.deepcopy(self.config)),
            hueco,
            0.0,
            medidas,
            0.0,
        )
        directa = ControlEstacionamiento(copy.deepcopy(self.config)).procesar(
            hueco,
            0.0,
            800.0,
            500.0,
            ahora=0.0,
            trasera_valida=True,
            cobertura_trasera=1.0,
            lateral_mm=100.0,
            lateral_valida=True,
            trasera_izquierda_mm=650.0,
            trasera_derecha_mm=650.0,
            trasera_izquierda_valida=True,
            trasera_derecha_valida=True,
            cobertura_trasera_izquierda=1.0,
            cobertura_trasera_derecha=1.0,
            ultrasonido_trasero_mm=430.0,
        )
        self.assertEqual(por_alto_nivel, directa)

    def test_no_pasa_argumentos_que_la_fsm_montada_no_declara(self):
        # Una FSM anterior (o un doble de prueba) no conoce el ultrasonido:
        # la llamada tiene que seguir funcionando, no reventar por un kwarg.
        class FsmAntigua:
            def __init__(self):
                self.recibido = None

            def procesar(self, hueco, heading_deg, frontal_mm, trasera_mm,
                         ahora=None, trasera_valida=False):
                self.recibido = {
                    "frontal_mm": frontal_mm,
                    "trasera_valida": trasera_valida,
                }
                return ResultadoParqueo(0, 0.0, "SEARCH_GAP")

        fsm = FsmAntigua()
        resultado = ejecutar_estacionamiento(
            fsm,
            None,
            0.0,
            MedidasParqueo(
                frontal_mm=700.0,
                trasera_mm=400.0,
                trasera_valida=True,
                ultrasonido_trasero_mm=350.0,
            ),
            1.0,
        )
        self.assertEqual(resultado.estado, "SEARCH_GAP")
        self.assertEqual(fsm.recibido["frontal_mm"], 700.0)
        self.assertTrue(fsm.recibido["trasera_valida"])


class _ParkingConUltrasonido:
    """Doble que siempre pide reversa, con los limites reales del sensor."""

    def __init__(self, config, resultado):
        self._real = ControlEstacionamiento(config)
        self.resultado = resultado
        self.recibido = None

    @property
    def parametros_procesar(self):
        return self._real.parametros_procesar

    def ultrasonido_utilizable(self, medida):
        return self._real.ultrasonido_utilizable(medida)

    def reiniciar(self):
        pass

    def procesar(self, hueco, heading_deg, frontal_mm, trasera_mm, **kwargs):
        self.recibido = kwargs
        return self.resultado


class GuardiaDeReversaTests(unittest.TestCase):
    def setUp(self):
        self.config = cargar_configuracion()

    def _control_en_parqueo(self, ultrasonido_mm):
        reversa = ResultadoParqueo(
            velocidad=-22, angulo=-20.0, estado="ARC_IN", razon="arco"
        )
        falso = _ParkingConUltrasonido(copy.deepcopy(self.config), reversa)
        control = ControlRuta(self.config)
        control._estacionamiento = falso
        control._parametros_estacionamiento = set(falso.parametros_procesar)
        control._sentido = 1
        control._lado_parqueo_solicitado = 1
        control._estado = "PARKING"
        control._t_estado = 0.0
        # Trasera axial ciega, diagonales y lateral sanas.
        pista = corredor(
            derecha=100.0,
            izquierda=650.0,
            trasera=8000.0,
            trasera_valida=False,
            cobertura_trasera=0.0,
        )
        orden = control.procesar(
            pista, (), 0.0, "PISTA", ahora=0.1, ultrasonido_mm=ultrasonido_mm
        )
        return control, falso, orden

    def test_sin_ultrasonido_la_reversa_a_ciegas_sigue_fallando(self):
        control, _falso, _orden = self._control_en_parqueo(None)
        self.assertEqual(control.estado, "FAILED")

    def test_el_ultrasonido_autoriza_la_reversa_que_el_lidar_no_ve(self):
        control, falso, orden = self._control_en_parqueo(420.0)
        self.assertEqual(control.estado, "PARKING")
        self.assertLess(orden.velocidad, 0)
        # Y la medida llego efectivamente hasta la FSM.
        self.assertEqual(falso.recibido["ultrasonido_trasero_mm"], 420.0)

    def test_un_eco_por_debajo_del_margen_de_emergencia_no_autoriza(self):
        margen = float(self.config["control"]["emergency_rear_mm"])
        control, _falso, _orden = self._control_en_parqueo(margen - 10.0)
        self.assertEqual(control.estado, "FAILED")


if __name__ == "__main__":
    unittest.main()
