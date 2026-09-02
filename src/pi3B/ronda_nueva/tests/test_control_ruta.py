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

    def test_auto_no_fija_el_sentido_sin_linea_y_agota_el_timeout(self):
        control = ControlRuta(self.config)
        orden = control.procesar(corredor(), (), 0.0, "PISTA", ahora=10.0)
        # Sobre pista blanca busca la linea avanzando, pero lo que importa
        # es que NO se inventa un sentido sin haber cruzado ninguna.
        self.assertEqual(control.estado, "WAIT_DIRECTION")
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

    def test_sin_pared_confiable_endereza_por_rumbo_no_congela_el_timon(self):
        """Regresion de la corrida 20260831_120547: con el robot cruzado a
        +33 grados y calidad de pared bajo el minimo, el timon quedaba en
        0.0 fijo y el robot derivaba en diagonal hasta el timeout."""

        self._sin_slew()
        control = ControlRuta(self.config)
        control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)
        self.assertEqual(control.estado, "CRUISE")

        borrosa = corredor(calidad=0.10)
        orden = control.procesar(borrosa, (), 33.0, "PISTA", ahora=0.1)
        self.assertEqual(control.estado, "CRUISE")
        self.assertLess(orden.angulo, 0.0)

        orden = control.procesar(borrosa, (), -33.0, "PISTA", ahora=0.2)
        self.assertGreater(orden.angulo, 0.0)

        alineada = control.procesar(borrosa, (), 0.0, "PISTA", ahora=0.3)
        self.assertAlmostEqual(alineada.angulo, 0.0, places=3)

    def test_la_transicion_pared_rumbo_es_continua(self):
        """Conmutar de golpe entre los dos controles hace oscilar el timon.

        En el episodio final de la corrida 154617 la calidad del ajuste
        alternaba entre 0,21 y 0,83 barrido a barrido, y con ella el mando
        saltaba entre el centrado de pared y el rumbo del carril. El robot
        se paso el centro dos veces sin confirmarlo: 1 de 65 ciclos bajo de
        la tolerancia y el recentrado murio por timeout, oscilando
        alrededor del sitio correcto."""

        self._sin_slew()
        umbral = float(self.config["control"].get("wall_min_quality", 0.30))
        descentrado = dict(izquierda=200.0, derecha=800.0, error_rumbo=0.0)

        angulos = []
        for calidad in (umbral * 0.6, umbral * 0.8, umbral, umbral * 1.4):
            control = ControlRuta(self.config)
            control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)
            orden = control.procesar(
                corredor(calidad=calidad, **descentrado), (), 0.0, "PISTA",
                ahora=0.1,
            )
            angulos.append(orden.angulo)

        # Descentrado a la izquierda, la pared manda girar a la derecha, asi
        # que el angulo se vuelve mas negativo conforme crece la calidad.
        # Lo que se comprueba es que esa autoridad aparece de forma gradual
        # y no de golpe al cruzar el umbral.
        for previo, siguiente in zip(angulos, angulos[1:]):
            self.assertGreaterEqual(previo, siguiente - 1e-6)
        recorrido = abs(angulos[-1] - angulos[0])
        self.assertGreater(recorrido, 1.0, "la calidad deberia mover el mando")
        saltos = [abs(b - a) for a, b in zip(angulos, angulos[1:])]
        self.assertLess(
            max(saltos), recorrido * 0.9,
            "ningun paso de calidad debe concentrar casi todo el cambio",
        )

    def test_rumbo_de_carril_avanza_90_grados_por_esquina(self):
        self._sin_slew()
        control = ControlRuta(self.config)
        abierta = corredor(frontal=1200.0, frontal_muro=1200.0)
        cerrada = corredor(frontal=1000.0, frontal_muro=500.0)

        control.procesar(abierta, (), 0.0, "AZUL", ahora=0.0)
        control.procesar(cerrada, (), 0.0, "PISTA", ahora=0.1)
        control.procesar(cerrada, (), 0.0, "PISTA", ahora=0.2)
        self.assertEqual(control.estado, "TURN")
        control.procesar(abierta, (), 88.0, "PISTA", ahora=0.3)
        control.procesar(abierta, (), 89.0, "PISTA", ahora=0.4)
        self.assertEqual(control.esquinas, 1)
        self.assertEqual(control.estado, "CRUISE")

        # Tras una esquina antihoraria el rumbo del tramo es +90: sin pared
        # confiable, a +60 corrige a la izquierda y a +120 a la derecha.
        borrosa = corredor(calidad=0.10)
        orden = control.procesar(borrosa, (), 60.0, "PISTA", ahora=2.0)
        self.assertEqual(control.estado, "CRUISE")
        self.assertGreater(orden.angulo, 0.0)
        orden = control.procesar(borrosa, (), 120.0, "PISTA", ahora=2.1)
        self.assertLess(orden.angulo, 0.0)

    def _hasta_seguir_pilar(self, control, color="VERDE", y=600.0):
        control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)
        control.procesar(
            corredor(), (track(70, color, x=-260.0, y=y),),
            0.0, "PISTA", ahora=0.1,
        )
        self.assertEqual(control.estado, "AVOID_APPROACH")
        self.assertEqual(control.track_activo_color, color)

    def test_dentro_del_ciego_hereda_el_color_del_track_anterior(self):
        """La camara no puede confirmar por debajo de 251 mm.

        La base del pilar sale del encuadre y `min_ground_support` lo
        rechaza, asi que exigir un track `confirmado` para reasociar pide
        algo que ningun barrido puede dar. El LiDAR si lo sigue viendo: el
        color se hereda del track anterior en vez de reabrirse."""

        self._sin_slew()
        control = ControlRuta(self.config)
        self._hasta_seguir_pilar(control, "VERDE", y=600.0)

        # Se acerca hasta el punto ciego, todavia con color.
        control.procesar(
            corredor(), (track(70, "VERDE", x=-250.0, y=300.0),),
            0.0, "PISTA", ahora=0.2,
        )
        # Ahora el LiDAR lo ve pero sin color y con id nuevo.
        ciego = track(88, None, x=-248.0, y=250.0, confirmado=False)
        orden = control.procesar(
            corredor(), (ciego,), 0.0, "PISTA", ahora=0.3
        )
        self.assertEqual(control.track_activo_id, 88)
        self.assertEqual(control.track_activo_color, "VERDE")
        self.assertNotIn("perdido", orden.razon)

    def test_no_hereda_el_color_lejos_ni_sobre_otro_color(self):
        """Las dos guardas que impiden que la herencia salte de pilar."""

        self._sin_slew()
        # Lejos del ciego no se hereda: ahi la camara si deberia ver.
        lejos = ControlRuta(self.config)
        self._hasta_seguir_pilar(lejos, "VERDE", y=900.0)
        suelto = track(91, None, x=-260.0, y=880.0, confirmado=False)
        lejos.procesar(corredor(), (suelto,), 0.0, "PISTA", ahora=0.2)
        self.assertNotEqual(lejos.track_activo_id, 91)

        # Y un candidato con color propio distinto nunca se adopta.
        otro = ControlRuta(self.config)
        self._hasta_seguir_pilar(otro, "VERDE", y=600.0)
        otro.procesar(
            corredor(), (track(70, "VERDE", x=-250.0, y=280.0),),
            0.0, "PISTA", ahora=0.2,
        )
        rojo = track(92, "ROJO", x=-248.0, y=250.0, confirmado=False)
        otro.procesar(corredor(), (rojo,), 0.0, "PISTA", ahora=0.3)
        self.assertNotEqual(otro.track_activo_id, 92)

    def test_pilar_no_reasociado_se_suelta_en_vez_de_esperar_parado(self):
        """Regresion de las corridas 181323 y 181547.

        Pararse a reasociar es un punto muerto cuando el pilar se perdio
        por acercarse demasiado: la reasociacion exige un track confirmado,
        la confirmacion exige color, y el color no vuelve mientras el pilar
        siga dentro del punto ciego. Quieto no se sale de ahi nunca, y el
        robot agotaba parado el timeout de evasion (9,7 s en ambas). Tras
        un margen corto debe soltar el pilar y seguir el carril."""

        self._sin_slew()
        control = ControlRuta(self.config)
        control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)
        control.procesar(
            corredor(), (track(9, "VERDE", x=-260.0, y=600.0),),
            0.0, "PISTA", ahora=0.1,
        )
        self.assertEqual(control.estado, "AVOID_APPROACH")

        # Sin tracks: al principio espera, quieto.
        orden = control.procesar(corredor(), (), 0.0, "PISTA", ahora=0.2)
        self.assertEqual(orden.velocidad, 0)
        self.assertIn("parada para reasociar", orden.razon)

        margen = float(self.config["control"]["obstacle_relock_timeout_s"])
        orden = control.procesar(
            corredor(), (), 0.0, "PISTA", ahora=0.2 + margen + 0.05
        )
        self.assertEqual(control.estado, "CRUISE")
        self.assertGreater(orden.velocidad, 0)
        self.assertIn("se suelta", orden.razon)
        self.assertIsNone(control.track_activo_id)

        # Y el margen no debe agotar el timeout de evasion, que es lo que
        # convertia la espera en un fallo terminal.
        self.assertLess(margen, float(self.config["control"]["obstacle_timeout_s"]))

    def test_el_sobrepaso_arranca_antes_del_punto_ciego(self):
        """La camara pierde el pilar antes que el LiDAR.

        Medido sobre un frame de a bordo con la optica calibrada: la base
        del pilar sale del recorte inferior de la ROI a unos 251 mm del
        LiDAR, y sin suelo debajo `min_ground_support` lo rechaza. Con
        `obstacle_pass_y_mm` en 160 el robot creia estar rebasando el pilar
        cuando llevaba 9 cm sin poder verlo, y la maniobra dependia de un
        color que ya no se medía. El umbral debe quedar por encima del
        punto ciego, con margen para varios ciclos de vision."""

        control = self.config["control"]
        ciego = float(control["near_blind_spot_mm"])
        paso = float(control["obstacle_pass_y_mm"])
        self.assertGreater(paso, ciego)
        # A 150 mm/s y ~70 ms de edad de vision, cada ciclo son ~10 mm.
        self.assertGreaterEqual(paso - ciego, 20.0)

        # Los dos umbrales estan acoplados: el sobrepaso termina tras
        # recorrer obstacle_pass_distance_mm desde que empezo, asi que
        # subir uno sin el otro deja al robot recentrando con el pilar
        # todavia por delante. Se comprobo en pista (corrida 160916:
        # 280 con 200 dejaba el pilar 80 mm delante y fallo a los 11,7 s).
        recorrido = float(control["obstacle_pass_distance_mm"])
        self.assertLess(
            paso - recorrido, 0.0,
            "al terminar el sobrepaso el pilar debe quedar detras",
        )

    def test_el_punto_de_paso_cabe_en_el_hueco_mas_estrecho(self):
        """El paso junto al pilar verde del tramo inferior mide 333 mm de la
        pared al centro del pilar (medido por el LiDAR en las corridas
        152614 y 153132). Con `obstacle_lateral_clearance_mm` en 255 el
        punto de paso caia a 78 mm de la pared y el borde del robot a 16:
        no cabe. El robot nunca llegaba alli porque la guardia de pared lo
        frenaba antes, y ese rescate era justo lo que lo empujaba hacia el
        pilar. La restriccion es geometrica, asi que se comprueba aqui."""

        control = self.config["control"]
        parking = self.config["parking"]
        hueco = float(control["narrowest_gap_wall_to_pillar_mm"])
        semi_pilar = 25.0
        separacion = float(control["obstacle_lateral_clearance_mm"])
        # El LiDAR no esta centrado: 61 mm al perimetro izquierdo y 45 al
        # derecho, medidos con regla el 2026-08-31. Rebasando un pilar verde
        # la pared queda a la izquierda, que es el borde mas lejano.
        borde_pared = float(parking["lidar_to_left_edge_mm"])
        borde_pilar = float(parking["lidar_to_right_edge_mm"])

        # Distancia de la pared al eje del LiDAR en el punto de paso.
        eje = hueco - separacion
        holgura_pared = eje - borde_pared
        holgura_pilar = (hueco - semi_pilar) - eje - borde_pilar

        self.assertGreater(
            holgura_pared, 50.0,
            "el punto de paso deja %.0f mm entre el robot y la pared" % holgura_pared,
        )
        self.assertGreater(
            holgura_pilar, 50.0,
            "el punto de paso deja %.0f mm entre el robot y el pilar" % holgura_pilar,
        )
        # Y debe quedar razonablemente centrado, no rozando un lado.
        self.assertLess(abs(holgura_pared - holgura_pilar), 40.0)

    def test_el_recentrado_es_alcanzable_con_el_radio_real(self):
        """La tolerancia y el timeout tienen que caber en la geometria.

        Rebasar el pilar deja el eje a unos 154 mm de la pared -el hueco
        mide 333 de pared a centro de pilar y la separacion pedida es 179-,
        y desde ahi hay que volver al centro de un carril de ~1015 mm. Con
        el radio medido de 559 mm, cerrar hasta una tolerancia de 150
        exigia 5,74 s de arco puro cuando el timeout daba 4,8: no cabia ni
        en el caso ideal. Las corridas 134517 y 144526 murieron ahi las
        dos, con el mismo error de 689 mm.
        """

        import math

        control = self.config["control"]
        radio_mm = 559.0
        velocidad_mm_s = float(control["speed_cruise_pwm"]) * 4.0
        carril_mm = 1015.0
        eje_tras_rebasar = (
            float(control["narrowest_gap_wall_to_pillar_mm"])
            - float(control["obstacle_lateral_clearance_mm"])
        )
        objetivo = (carril_mm - float(control["recenter_tolerance_mm"])) / 2.0
        desplazamiento = objetivo - eje_tras_rebasar
        coseno = 1.0 - desplazamiento / radio_mm
        arco_s = radio_mm * math.acos(max(-1.0, coseno)) / velocidad_mm_s

        referencia = float(control["obstacle_timeout_reference_pwm"])
        disponible = float(control["recenter_timeout_s"]) * referencia / float(
            control["speed_cruise_pwm"]
        )
        self.assertLess(
            arco_s, disponible,
            "recentrar pide %.2f s de arco y el timeout da %.2f" % (arco_s, disponible),
        )

    def test_recentrado_confirma_con_laterales_aunque_baje_la_calidad(self):
        """Regresion de las corridas 152111 y 152413.

        El robot llego a 108 mm de error lateral, dentro de la tolerancia
        de 150, y aun asi agoto el timeout de reincorporacion porque la
        calidad del ajuste de pared habia caido a 0,21 y el criterio de
        centrado la exigia por encima de 0,30. Estar centrado es una
        afirmacion sobre distancias medidas, no sobre el ajuste."""

        self._sin_slew()
        control = ControlRuta(self.config)
        control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)
        control.procesar(
            corredor(), (track(4, "ROJO", x=-260.0, y=150.0),),
            0.0, "PISTA", ahora=0.1,
        )
        # El sobrepaso dura lo que tarde en recorrer
        # obstacle_pass_distance_mm, asi que se avanza hasta que termine en
        # vez de fijar un instante que dependa de ese parametro.
        t = 1.0
        for _ in range(40):
            control.procesar(
                corredor(izquierda=860.0, derecha=140.0, calidad=0.8),
                (), 0.0, "PISTA", ahora=t,
            )
            if control.estado == "RECENTER":
                break
            t += 0.2
        self.assertEqual(control.estado, "RECENTER")

        # Centrado de sobra (108 mm), pero con el ajuste degradado.
        centrado = corredor(
            izquierda=922.0, derecha=814.0, calidad=0.21,
            frontal=1400.0, frontal_muro=1400.0,
        )
        control.procesar(centrado, (), 0.0, "PISTA", ahora=2.1)
        orden = control.procesar(centrado, (), 0.0, "PISTA", ahora=2.2)
        self.assertEqual(control.estado, "CRUISE")
        self.assertIn("reincorporacion verificada", orden.razon)

        # Una lateral invalida si debe impedir la confirmacion: ahi no hay
        # geometria que respalde la afirmacion de estar centrado.
        otro = ControlRuta(self.config)
        otro.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)
        otro.procesar(
            corredor(), (track(5, "ROJO", x=-260.0, y=150.0),),
            0.0, "PISTA", ahora=0.1,
        )
        otro.procesar(corredor(), (), 0.0, "PISTA", ahora=1.0)
        otro.procesar(
            corredor(izquierda=860.0, derecha=140.0, calidad=0.8),
            (), 0.0, "PISTA", ahora=2.0,
        )
        ciego = corredor(
            izquierda=922.0, derecha=814.0, calidad=0.21,
            laterales_validas=False,
        )
        otro.procesar(ciego, (), 0.0, "PISTA", ahora=2.1)
        otro.procesar(ciego, (), 0.0, "PISTA", ahora=2.2)
        self.assertEqual(otro.estado, "RECENTER")

    def test_la_pared_conserva_su_termino_lateral_con_pilar_activo(self):
        """La guardia empuja hacia el pilar, y aun asi debe conservarse.

        Rebasar un pilar centrado deja unos 175 mm a la pared en un carril
        de 1000, asi que la guardia interviene en toda evasion, no como
        excepcion: en la corrida 145857 llego a pesar 0,94 del mando y el
        robot giraba hacia el pilar que esquivaba. Se probo dejar solo el
        termino de rumbo mientras hay un pilar activo y en pista salio
        peor -- corrida 152111, cero esquinas frente a las dos de la 151037
        desde la misma salida-- porque ese termino lateral es tambien lo
        que impide pegarse a la pared: sin el, el robot llega descentrado
        al recentrado y agota su timeout antes de la primera esquina.

        Esta prueba fija que se conserva, para que el intento no se repita
        sin leer antes la bitacora del README."""

        self._sin_slew()
        control = ControlRuta(self.config)
        control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.0)

        pegado = corredor(
            izquierda=150.0, derecha=850.0, error_rumbo=0.0, calidad=0.9
        )
        verde = control.procesar(
            pegado, (track(3, "VERDE", x=260.0, y=300.0),),
            0.0, "PISTA", ahora=0.1,
        )
        self.assertEqual(control.track_activo_color, "VERDE")
        # Descentrado y paralelo, el mando lo domina la pared alejandolo de
        # ella, no el pure-pursuit hacia el punto de paso.
        self.assertLess(verde.angulo, -10.0)

    def _entrar_en_giro(self, control, sentido_color="AZUL"):
        """Deja la FSM en TURN, que es donde vive la maniobra de esquina."""

        abierta = corredor(frontal=1200.0, frontal_muro=1200.0)
        cerrada = corredor(frontal=1000.0, frontal_muro=500.0)
        control.procesar(abierta, (), 0.0, sentido_color, ahora=0.0)
        control.procesar(cerrada, (), 0.0, "PISTA", ahora=0.1)
        control.procesar(cerrada, (), 0.0, "PISTA", ahora=0.2)
        self.assertEqual(control.estado, "TURN")

    def test_kturn_invierte_el_volante_al_retroceder(self):
        """El radio de ~600 mm medido en pista no cierra la esquina sola.

        Con Ackermann la rotacion es omega = v*tan(delta)/L: al retroceder
        hace falta el volante al lado contrario para que el morro siga
        rotando hacia el mismo lado."""

        self._sin_slew()
        for direccion, signo in (("LEFT", +1), ("RIGHT", -1)):
            with self.subTest(direccion=direccion):
                self.config["control"]["turn_direction"] = direccion
                control = ControlRuta(self.config)
                self._entrar_en_giro(control)

                avance = control.procesar(
                    corredor(frontal=1000.0, frontal_muro=500.0),
                    (), 5.0 * signo, "PISTA", ahora=0.3,
                )
                self.assertGreater(avance.velocidad, 0)
                self.assertEqual(avance.angulo > 0, signo > 0)

                # Sin frente y con trasera medida: entra en reversa. El
                # primer ciclo sale a cero porque _emitir nunca invierte el
                # signo de la velocidad de golpe.
                atascado = corredor(
                    frontal=200.0, frontal_muro=200.0, trasera=800.0
                )
                reversa = control.procesar(
                    atascado, (), 10.0 * signo, "PISTA", ahora=0.4
                )
                self.assertIn("reversa", reversa.razon)
                self.assertEqual(reversa.velocidad, 0)
                # El volante va al lado contrario del sentido de giro.
                self.assertEqual(reversa.angulo > 0, signo < 0)

                reversa = control.procesar(
                    atascado, (), 12.0 * signo, "PISTA", ahora=0.5
                )
                self.assertLess(reversa.velocidad, 0)
                self.assertEqual(reversa.angulo > 0, signo < 0)

    def test_kturn_no_retrocede_sin_trasera_fiable(self):
        """Un SIN_DATO trasero detiene la maniobra; no la ejecuta a ciegas."""

        self._sin_slew()
        control = ControlRuta(self.config)
        self._entrar_en_giro(control)

        ciego = corredor(
            frontal=200.0,
            frontal_muro=200.0,
            trasera=800.0,
            trasera_valida=False,
        )
        orden = control.procesar(ciego, (), 5.0, "PISTA", ahora=0.3)
        self.assertEqual(orden.velocidad, 0)
        self.assertIn("sin trasera fiable", orden.razon)

        pegado = corredor(frontal=200.0, frontal_muro=200.0, trasera=120.0)
        orden = control.procesar(pegado, (), 5.0, "PISTA", ahora=0.4)
        self.assertEqual(orden.velocidad, 0)

    def test_kturn_resuelve_frontal_critico_sin_bucle_de_recuperacion(self):
        """Regresion de la corrida 20260901_111701, barrido 164."""

        self._sin_slew()
        self.config["control"]["turn_direction"] = "RIGHT"
        control = ControlRuta(self.config)
        self._entrar_en_giro(control, sentido_color="NARANJA")
        critico = corredor(
            frontal=66.0,
            frontal_muro=592.5,
            izquierda=521.6,
            derecha=712.6,
            trasera=543.3,
            trasera_valida=True,
            cobertura_trasera=0.917,
        )

        orden = control.procesar(critico, (), -43.13, "PISTA", ahora=0.3)
        self.assertEqual(control.estado, "TURN")
        self.assertIn("reversa", orden.razon)
        self.assertEqual(orden.velocidad, 0)
        self.assertGreater(orden.angulo, 0.0)

        orden = control.procesar(critico, (), -43.0, "PISTA", ahora=0.4)
        self.assertEqual(control.estado, "TURN")
        self.assertLess(orden.velocidad, 0)
        self.assertGreater(orden.angulo, 0.0)

    def test_kturn_alterna_tramos_y_respeta_el_maximo(self):
        self._sin_slew()
        self.config["control"]["corner_kturn_max_tramos"] = 2
        control = ControlRuta(self.config)
        self._entrar_en_giro(control)

        sin_frente = corredor(
            frontal=200.0, frontal_muro=200.0, trasera=900.0
        )
        con_frente = corredor(
            frontal=600.0, frontal_muro=600.0, trasera=500.0
        )
        # Cada fase dura al menos corner_kturn_min_tramo_s, asi que los
        # ciclos se espacian como lo haria un tramo real en pista.
        t = 0.3
        for tramo in range(2):
            # Un ciclo a cero al invertir el signo, y ya retrocede.
            orden = control.procesar(sin_frente, (), 5.0, "PISTA", ahora=t)
            self.assertIn("reversa", orden.razon)
            t += 0.1
            orden = control.procesar(sin_frente, (), 5.0, "PISTA", ahora=t)
            self.assertLess(orden.velocidad, 0)
            t += 0.9
            # Recuperado el frente, vuelve a avanzar sin esperar timeout.
            orden = control.procesar(con_frente, (), 5.0, "PISTA", ahora=t)
            self.assertNotIn("reversa", orden.razon)
            t += 0.1
            orden = control.procesar(con_frente, (), 5.0, "PISTA", ahora=t)
            self.assertGreater(orden.velocidad, 0)
            t += 0.9

        # Agotados los tramos, insiste avanzando en vez de seguir oscilando.
        orden = control.procesar(sin_frente, (), 5.0, "PISTA", ahora=t)
        self.assertNotIn("reversa", orden.razon)
        self.assertGreaterEqual(orden.velocidad, 0)

    def test_kturn_no_oscila_con_la_trasera_bimodal_de_la_esquina(self):
        """Regresion de la corrida 145409.

        En la esquina el sector trasero cruza la arista entre dos paredes y
        la lectura alterna entre dos valores reales (258 y 690 mm medidos,
        ambos validos y con cobertura plena). Comparar cada ciclo contra la
        holgura de confort hacia conmutar la fase a 5 Hz: el robot alternaba
        avance y reversa sin llegar a moverse porque el slew de velocidad
        nunca alcanzaba el PWM pedido."""

        self._sin_slew()
        control = ControlRuta(self.config)
        self._entrar_en_giro(control)

        lejos = corredor(frontal=230.0, frontal_muro=230.0, trasera=690.0)
        cerca = corredor(frontal=230.0, frontal_muro=230.0, trasera=258.0)

        t = 0.3
        control.procesar(lejos, (), 5.0, "PISTA", ahora=t)
        self.assertEqual(control.estado, "TURN")

        # Doce ciclos alternando las dos lecturas, como en el CSV.
        fases = []
        for i in range(12):
            t += 0.1
            orden = control.procesar(
                cerca if i % 2 else lejos, (), 5.0 + i, "PISTA", ahora=t
            )
            fases.append("reversa" in orden.razon)

        # 258 mm sigue por encima de emergency_rear_mm, asi que el tramo
        # empezado se sostiene durante su duracion minima en vez de rebotar
        # en cada barrido. Antes conmutaba en practicamente todos.
        self.assertTrue(all(fases[:6]), "el tramo debe sostenerse 0,8 s")
        conmutaciones = sum(
            1 for a, b in zip(fases, fases[1:]) if a != b
        )
        self.assertLessEqual(
            conmutaciones, 2, "la maniobra no debe oscilar por barrido"
        )

        # Y el limite duro si lo corta: por debajo de la emergencia trasera.
        t += 0.1
        pegado = corredor(frontal=230.0, frontal_muro=230.0, trasera=100.0)
        orden = control.procesar(pegado, (), 20.0, "PISTA", ahora=t)
        self.assertNotIn("reversa", orden.razon)

    def test_kturn_desactivado_conserva_el_giro_de_una_sola_pasada(self):
        self._sin_slew()
        self.config["control"]["corner_kturn_enabled"] = False
        control = ControlRuta(self.config)
        self._entrar_en_giro(control)

        orden = control.procesar(
            corredor(frontal=200.0, frontal_muro=200.0, trasera=900.0),
            (), 5.0, "PISTA", ahora=0.3,
        )
        self.assertGreaterEqual(orden.velocidad, 0)
        self.assertNotIn("reversa", orden.razon)

    def test_auto_avanza_a_buscar_la_linea_de_sentido(self):
        """El modo AUTO era inservible esperando quieto.

        Las lineas de sentido estan en las esquinas y el robot arranca
        sobre pista blanca: parado no las alcanza nunca y solo llegaba al
        timeout. Ahora avanza centrado hasta cruzar la primera."""

        self._sin_slew()
        self.config["control"]["turn_direction"] = "AUTO"
        control = ControlRuta(self.config)

        # Sobre blanco: avanza en vez de quedarse quieto.
        orden = control.procesar(
            corredor(izquierda=300.0, derecha=700.0, calidad=0.9),
            (), 0.0, "PISTA", ahora=0.0,
        )
        self.assertEqual(control.estado, "WAIT_DIRECTION")
        self.assertGreater(orden.velocidad, 0)
        self.assertIn("buscando la linea", orden.razon)
        # Y se centra mientras busca: descentrado a la izquierda, corrige.
        self.assertLess(orden.angulo, 0.0)
        self.assertEqual(control.sentido, 0)

        # Al cruzar la linea queda fijado el sentido y arranca la ronda.
        orden = control.procesar(corredor(), (), 0.0, "AZUL", ahora=0.5)
        self.assertEqual(control.estado, "CRUISE")
        self.assertEqual(control.sentido, 1)

    def test_con_sentido_escrito_no_se_mueve_a_buscar_nada(self):
        """Con LEFT/RIGHT explicito no hay busqueda: se resuelve al vuelo."""

        self._sin_slew()
        self.config["control"]["turn_direction"] = "RIGHT"
        control = ControlRuta(self.config)
        orden = control.procesar(corredor(), (), 0.0, "PISTA", ahora=0.0)
        self.assertEqual(control.estado, "CRUISE")
        self.assertEqual(control.sentido, -1)

        # Y si se desactiva la busqueda, AUTO vuelve a esperar quieto.
        self.config["control"]["turn_direction"] = "AUTO"
        self.config["control"]["direction_search_moving"] = False
        quieto = ControlRuta(self.config)
        orden = quieto.procesar(corredor(), (), 0.0, "PISTA", ahora=0.0)
        self.assertEqual(orden.velocidad, 0)
        self.assertIn("esperando", orden.razon)

    def test_sale_despacio_de_la_esquina_y_luego_recupera_el_crucero(self):
        """Girando 90 grados el robot no ve hacia donde va a salir.

        Un pilar situado tras la esquina queda fuera del campo durante todo
        el giro y aparece de golpe: en la corrida 130028 el sector frontal
        paso de 760 a 30 mm en un solo barrido de 0,1 s, cuando a esa
        velocidad el robot solo avanza 12 mm. A 25 PWM habia margen de
        reaccion; a 35 la emergencia llego con el pilar tocando el morro."""

        self._sin_slew()
        self.config["control"]["speed_cruise_pwm"] = 35
        self.config["control"]["speed_avoid_pwm"] = 25
        control = ControlRuta(self.config)
        abierta = corredor(frontal=1200.0, frontal_muro=1200.0)
        cerrada = corredor(frontal=1000.0, frontal_muro=500.0)

        control.procesar(abierta, (), 0.0, "AZUL", ahora=0.0)
        control.procesar(cerrada, (), 0.0, "PISTA", ahora=0.1)
        control.procesar(cerrada, (), 0.0, "PISTA", ahora=0.2)
        self.assertEqual(control.estado, "TURN")
        control.procesar(abierta, (), 88.0, "PISTA", ahora=0.3)
        salida = control.procesar(abierta, (), 89.0, "PISTA", ahora=0.4)
        self.assertEqual(control.esquinas, 1)

        # Recien salido de la esquina va moderado, no a crucero.
        self.assertLessEqual(salida.velocidad, 25)
        ventana = float(self.config["control"]["post_corner_slow_s"])
        dentro = control.procesar(abierta, (), 89.0, "PISTA", ahora=0.4 + ventana / 2.0)
        self.assertLessEqual(dentro.velocidad, 25)

        # Pasada la ventana recupera el crucero completo.
        fuera = control.procesar(abierta, (), 89.0, "PISTA", ahora=0.4 + ventana + 0.2)
        self.assertGreater(fuera.velocidad, 25)

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
        # Se llega aqui con sentido +1 (AZUL = antihorario), asi que el lado
        # pedido tiene que ser el que la configuracion asocie al giro a la
        # izquierda. Se lee de la config a proposito: cual de los dos lados
        # es el bueno es una calibracion de pista que el equipo ha invertido
        # ya una vez, y lo que esta prueba fija es el MAPEO, no el numero.
        self.assertEqual(
            control.lado_parqueo_solicitado,
            self.config["parking"]["parking_side_for_left_turn"],
        )
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

        # Las distancias frontales van por debajo de
        # recenter_corner_handoff_mm, que sigue al radio de giro: bajo de
        # 900 a 700 al medirlo, y de 700 a 620 al comprobar que girando a
        # la izquierda el radio es menor (559 mm contra 617).
        handoff = float(self.config["control"]["recenter_corner_handoff_mm"])
        primera = control.procesar(
            corredor(
                frontal=handoff - 10.0, frontal_muro=handoff - 10.0,
                izquierda=732.2, derecha=447.0, calidad=0.816,
            ),
            (), 20.63, "PISTA", ahora=5.69,
        )
        self.assertEqual(control.estado, "RECENTER")
        self.assertFalse(primera.terminado)

        segunda = control.procesar(
            corredor(
                frontal=handoff - 16.0, frontal_muro=handoff - 16.0,
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
        self.config["control"]["recovery_exit_confirm_scans"] = 1
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

    def test_recuperacion_lateral_confirma_salida_y_gira_lejos_del_pilar(self):
        """Reproduce el bucle observado junto al pilar verde derecho."""

        self._sin_slew()
        self.config["control"]["turn_direction"] = "RIGHT"
        self.config["control"]["forced_turn_after_recoveries"] = 1
        self.config["control"]["recovery_min_s"] = 0.05
        self.config["control"]["recovery_exit_confirm_scans"] = 3
        control = ControlRuta(self.config)
        libre = corredor(frontal=490.0, frontal_muro=490.0, derecha=890.0)
        pilar_derecho = corredor(
            frontal=490.0,
            frontal_muro=490.0,
            izquierda=472.0,
            derecha=56.0,
            trasera=610.0,
        )

        control.procesar(libre, (), 0.0, "AZUL", ahora=0.0)
        control.procesar(pilar_derecho, (), 0.0, "PISTA", ahora=0.1)
        self.assertEqual(control.estado, "RECOVERY")

        # Un unico barrido libre no debe invertir inmediatamente la marcha.
        control.procesar(libre, (), 0.0, "PISTA", ahora=0.2)
        self.assertEqual(control.estado, "RECOVERY")
        control.procesar(libre, (), 0.0, "PISTA", ahora=0.3)
        self.assertEqual(control.estado, "RECOVERY")

        orden = control.procesar(libre, (), 0.0, "PISTA", ahora=0.4)
        self.assertEqual(control.estado, "FORCED_TURN")
        self.assertGreater(orden.angulo, 0.0)
        self.assertIn("lateral derecho", orden.razon)

    def test_recuperacion_acepta_salida_confirmada_en_borde_del_timeout(self):
        """Regresion de la corrida 20260901_121058."""

        self._sin_slew()
        self.config["control"]["recovery_min_s"] = 0.0
        self.config["control"]["recovery_timeout_s"] = 0.25
        self.config["control"]["recovery_exit_confirm_scans"] = 3
        control = ControlRuta(self.config)
        libre = corredor(frontal=1300.0, izquierda=480.0, derecha=500.0)
        peligro = corredor(frontal=43.0, trasera=600.0)

        control.procesar(libre, (), 0.0, "AZUL", ahora=0.0)
        control.procesar(peligro, (), 0.0, "PISTA", ahora=0.1)
        control.procesar(libre, (), 0.0, "PISTA", ahora=0.2)
        control.procesar(libre, (), 0.0, "PISTA", ahora=0.3)
        orden = control.procesar(libre, (), 0.0, "PISTA", ahora=0.4)

        self.assertEqual(control.estado, "CRUISE")
        self.assertFalse(orden.terminado)
        self.assertIn("recuperacion despejada", orden.razon)

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
