"""El piloto completo, conduciendo una pista sintetica de principio a fin.

Es el test que mas cosas rompe cuando algo esta mal, porque encadena
percepcion, localizacion, mapa, planificador y maquina de estados con el
mismo modelo de bicicleta del chasis real.  Si el robot da doce esquinas aqui,
lo que quede por fallar en pista sera de sensores, no de logica.
"""

import math
import unittest

from ..estacionamiento import ControlEstacionamiento
from ..modelos import DeteccionPilar, MapaParedes, Recta
from ..percepcion_lidar import PercepcionLidar
from ..piloto import FIN, GIRO, ORIENTACION, RECTA, RETROCESO, Piloto
from .apoyo import barrido_sintetico, config_minima, pose_en_carril


def config():
    base = config_minima()
    base["chassis"] = {
        "wheelbase_mm": 136.0,
        "width_mm": 125.0,
        "length_mm": 222.0,
        "turn_radius_left_mm": 228.0,
        "turn_radius_right_mm": 260.0,
    }
    base["control"] = {
        "turn_direction": "RIGHT",
        "speed_cruise_pwm": 55,
        "speed_turn_pwm": 45,
        "mm_s_per_pwm": 4.0,
        "parking_enabled": False,
        "corners_before_parking": 12,
    }
    base["fusion"] = {}
    return base


class Mundo:
    """Robot + pista, con el piloto al mando y barridos sinteticos.

    El rumbo del MUNDO crece en sentido horario (es la convencion del barrido
    sintetico); la IMU del robot es positiva a la izquierda, de ahi el cambio
    de signo al pasarselo al piloto.  Confundir esos dos signos hace que el
    robot gire perfectamente hacia el lado contrario, asi que conviene tenerlo
    escrito.
    """

    def __init__(self, cfg, pilares_mundo=(), piloto=None, dt=0.05):
        self.cfg = cfg
        self.percepcion = PercepcionLidar(cfg)
        self.piloto = piloto or Piloto(cfg)
        self.pilares = list(pilares_mundo)
        self.dt = dt
        self.mm_por_pwm = float(cfg["control"].get("mm_s_per_pwm", 4.0))
        self.x, self.y, self.theta = pose_en_carril(0, 2600.0, 500.0)
        self.t = 0.0
        self.angulo = 0.0
        self.historia = []

    def _detecciones(self, paredes):
        """Lo que la camara "veria": color verdadero de los postes cercanos."""

        salida = []
        for px, py, _lado, color in self.pilares:
            dx, dy = px - self.x, py - self.y
            rad = math.radians(self.theta)
            xr = dx * math.cos(rad) - dy * math.sin(rad)
            yr = dx * math.sin(rad) + dy * math.cos(rad)
            if 100.0 < yr < 2200.0 and abs(xr) < 1400.0:
                salida.append(
                    DeteccionPilar(self.t, color, xr, yr, "CAMARA", 0.9)
                )
        return salida

    def paso(self):
        scan = barrido_sintetico(
            self.x,
            self.y,
            self.theta,
            pilares=[(px, py, lado) for px, py, lado, _c in self.pilares],
        )
        paredes, _objetos, hueco = self.percepcion.procesar(
            scan, self.t, angulo_servo_deg=self.angulo
        )
        consigna = self.piloto.procesar(
            paredes=paredes,
            pilares=self._detecciones(paredes),
            hueco=hueco,
            rumbo_deg=-self.theta,
            ahora=self.t,
        )
        self.angulo = consigna.angulo
        rueda = math.radians(self.piloto.plan.conversor.a_rueda(consigna.angulo))
        velocidad = consigna.velocidad * self.mm_por_pwm

        rad = math.radians(self.theta)
        self.x += velocidad * math.sin(rad) * self.dt
        self.y += velocidad * math.cos(rad) * self.dt
        self.theta += math.degrees(-velocidad / 136.0 * math.tan(rueda) * self.dt)
        self.t += self.dt
        self.historia.append((self.t, self.x, self.y, self.theta, consigna))
        return consigna

    def correr(self, segundos=180.0):
        pasos = int(segundos / self.dt)
        for _ in range(pasos):
            consigna = self.paso()
            if consigna.terminado:
                return consigna
        return None


class PruebaVueltaCompleta(unittest.TestCase):
    def test_doce_esquinas_sin_pilares(self):
        """Tres vueltas limpias: el conteo y el reanclaje de rumbo funcionan."""

        mundo = Mundo(config())
        final = mundo.correr(200.0)
        self.assertIsNotNone(final, "la ronda deberia terminar sola")
        self.assertEqual(mundo.piloto.esquinas, 12)
        self.assertEqual(mundo.piloto.estado, FIN)
        self.assertLess(mundo.t, 175.0)

    def test_doce_esquinas_con_ocho_pilares(self):
        """La disposicion mas cargada que el sorteo permite: dos por recta.

        Es la prueba de que la evasion ya no cuesta tiempo.  Con la maquina de
        estados anterior, evadir se llevaba el 45-52 % de la ronda; aqui el
        pilar solo deforma el carril y la ronda con ocho postes tarda un 16 %
        mas que la ronda vacia, no el doble.
        """

        pilares = []
        for segmento in range(4):
            for avance, offset, color in (
                (2000.0, 380.0, "ROJO"),
                (1000.0, 574.0, "VERDE"),
            ):
                x, y, _rumbo = pose_en_carril(segmento, avance, offset)
                pilares.append((x, y, 100.0, color))

        mundo = Mundo(config(), pilares_mundo=pilares)
        final = mundo.correr(220.0)
        self.assertIsNotNone(final)
        self.assertEqual(mundo.piloto.esquinas, 12)
        self.assertEqual(mundo.piloto._retrocesos, 0, "no deberia hacer falta retroceder")
        self.assertLess(mundo.t, 175.0)
        # Las doce casillas con poste tienen que estar en el mapa, y con el
        # color correcto: es lo que hace rapidas las vueltas 2 y 3.
        self.assertEqual(mundo.piloto.mapa.resumen(), "R0:R.V R1:R.V R2:R.V R3:R.V")

    def test_un_pilar_de_la_recta_siguiente_no_deforma_la_ruta(self):
        """Con la camara alta se ven por encima del bloque interior.

        En el marco de la recta actual caen en sitios creibles (offset 1000,
        justo en el borde del carril) y colarlos hacia que el robot maniobrara
        contra un obstaculo que no tenia delante: 10 esquinas de 12 y la
        velocidad media clavada en 29 PWM.
        """

        piloto = Piloto(config())
        self.assertFalse(piloto.mapa.pertenece_a_esta_recta(380.0, 1000.0))
        self.assertTrue(piloto.mapa.pertenece_a_esta_recta(1000.0, 574.0))
        self.assertTrue(piloto.mapa.pertenece_a_esta_recta(2000.0, 380.0))

    def test_no_choca_con_ninguna_pared(self):
        mundo = Mundo(config())
        mundo.correr(200.0)
        for _t, x, y, _theta, _c in mundo.historia:
            with self.subTest(x=round(x), y=round(y)):
                # Fuera del bloque interior (1000 mm) y dentro del muro (3000).
                self.assertLess(max(abs(x), abs(y)), 1500.0 - 62.0)
                self.assertGreater(max(abs(x), abs(y)), 500.0 + 62.0)

    def test_con_pilares_rebasa_por_el_lado_correcto(self):
        """Un rojo y un verde en la primera recta, en filas opuestas."""

        cfg = config()
        rojo = pose_en_carril(0, 2000.0, 380.0)
        verde = pose_en_carril(0, 1000.0, 574.0)
        mundo = Mundo(
            cfg,
            pilares_mundo=[
                (rojo[0], rojo[1], 100.0, "ROJO"),
                (verde[0], verde[1], 100.0, "VERDE"),
            ],
        )
        mundo.correr(60.0)

        # sentido RIGHT (+1): el offset crece a la derecha del robot, asi que
        # el rojo se rebasa con MAS offset y el verde con menos.
        for (px, py, _lado, color), esperado_mayor in (
            ((rojo[0], rojo[1], 100.0, "ROJO"), True),
            ((verde[0], verde[1], 100.0, "VERDE"), False),
        ):
            paso = min(
                mundo.historia, key=lambda h: math.hypot(h[1] - px, h[2] - py)
            )
            # offset del robot y del poste respecto al muro exterior (x=-1500)
            offset_robot = paso[1] + 1500.0
            offset_pilar = px + 1500.0
            with self.subTest(color=color):
                if esperado_mayor:
                    self.assertGreater(offset_robot, offset_pilar)
                else:
                    self.assertLess(offset_robot, offset_pilar)

    def test_el_mapa_recuerda_lo_que_vio(self):
        cfg = config()
        rojo = pose_en_carril(0, 1500.0, 380.0)
        mundo = Mundo(cfg, pilares_mundo=[(rojo[0], rojo[1], 100.0, "ROJO")])
        mundo.correr(60.0)
        self.assertGreaterEqual(mundo.piloto.mapa.casillas_conocidas(), 1)
        self.assertIn("R", mundo.piloto.mapa.resumen())


class MundoSinColor(Mundo):
    """La camara ve los postes pero NO acierta el color.

    Es el caso peor real: en pista la camara aporta color en el 67 % de los
    ciclos, asi que hay postes que llegan cerca sin identificar.
    """

    def _detecciones(self, paredes):
        return [
            DeteccionPilar(d.timestamp, "", d.x_mm, d.y_mm, "CAMARA", 0.4)
            for d in super()._detecciones(paredes)
        ]


class PruebaSinColor(unittest.TestCase):
    """Un bulto sin color cerca tiene que mover el carril igual.

    "Frenar y seguir recto" sonaba prudente y en pista era chocar: con el
    umbral en 0 esta misma vuelta da 0 esquinas y 56 retrocesos, que es
    exactamente lo que se vio en pista los dias 04 y 05-09.
    """

    def _pilares(self):
        pilares = []
        for segmento in range(4):
            for avance, offset, color in (
                (2000.0, 380.0, "ROJO"),
                (1000.0, 574.0, "VERDE"),
            ):
                x, y, _r = pose_en_carril(segmento, avance, offset)
                pilares.append((x, y, 100.0, color))
        return pilares

    def _correr(self, umbral, segundos=220.0):
        cfg = config()
        cfg["control"]["plan_colorless_below_mm"] = umbral
        mundo = MundoSinColor(cfg, pilares_mundo=self._pilares())
        mundo.correr(segundos)
        return mundo

    def test_sin_umbral_la_vuelta_se_atasca(self):
        # 60 s de simulacion bastan y ahorran dos minutos de suite: para
        # entonces una vuelta sana lleva 5 esquinas y esta lleva cero.
        mundo = self._correr(0.0, segundos=60.0)
        self.assertLess(mundo.piloto.esquinas, 3)
        self.assertGreater(mundo.piloto._retrocesos, 10)

    def test_con_umbral_la_vuelta_se_completa(self):
        mundo = self._correr(900.0)
        self.assertEqual(mundo.piloto.esquinas, 12)
        self.assertEqual(mundo.piloto._retrocesos, 0)


class PruebaVigilanteDeAtascos(unittest.TestCase):
    """El robot manda avanzar y el mundo no cambia.

    Caso real: corrida del 05-09 14:18.  21 de sus 68 s inmovil con el motor
    en 25 PWM, paredes clavadas a +-1 mm y rumbo a 0,1 grados, con el LiDAR
    fresco, el watchdog de la Pico en OK y 495 mm libres por delante.  No hay
    emergencia que lo dispare (no hay nada cerca) y el estado seguia siendo
    RECTA: nadie se enteraba.
    """

    def _paredes(self, frontal, izq=224.0, der=793.0):
        return MapaParedes(
            timestamp=1.0,
            frontal=Recta(frontal, 0.0, 1.0, 30, 1.0),
            izquierda=Recta(izq, -90.0, 1.0, 40, 1.0),
            derecha=Recta(der, 90.0, 1.0, 40, 1.0),
            frontal_min_mm=frontal,
            izquierda_min_mm=izq,
            derecha_min_mm=der,
            trasera_min_mm=1800.0,
            corredor_mm=frontal,
        )

    def _rodar(self, piloto, paredes_por_ciclo, rumbo=0.0, t0=0.0, dt=0.1):
        consignas = []
        for i, frontal in enumerate(paredes_por_ciclo):
            consignas.append(
                piloto.procesar(
                    paredes=self._paredes(frontal),
                    rumbo_deg=rumbo,
                    ahora=t0 + i * dt,
                )
            )
        return consignas

    def _arrancado(self):
        piloto = Piloto(config())
        # Un par de ciclos para salir de ESPERA y quedarse conduciendo.
        self._rodar(piloto, [1200.0, 1195.0])
        return piloto

    def test_el_mundo_quieto_dispara_un_empujon(self):
        piloto = self._arrancado()
        # Clavado en mitad de una recta.  En la corrida la frontal era 495 mm,
        # pero a esa distancia el disparo de esquina salta antes y enturbia la
        # prueba; lo que se comprueba aqui es el vigilante, no la esquina.
        consignas = self._rodar(piloto, [1200.0] * 30, t0=1.0)
        razones = [c.razon for c in consignas]
        self.assertIn("empujon de atasco", razones, "nadie detecto el atasco")
        empujones = [c for c in consignas if c.razon == "empujon de atasco"]
        self.assertEqual(
            empujones[0].velocidad,
            piloto.atasco_empujon_pwm,
            "el empujon tiene que saltarse la rampa: subir de 25 a 55 en tres "
            "ciclos es lo que NO rompe un rozamiento estatico",
        )
        self.assertGreaterEqual(piloto._atascos, 1)

    def test_si_el_robot_avanza_no_hay_falso_positivo(self):
        """Una recta normal: los laterales y el rumbo SI son constantes."""

        piloto = self._arrancado()
        # 100 mm/s a 10 Hz: la pared frontal se acerca 10 mm por ciclo.
        frontales = [1200.0 - 10.0 * i for i in range(40)]
        consignas = self._rodar(piloto, frontales, t0=1.0)
        self.assertNotIn("empujon de atasco", [c.razon for c in consignas])
        self.assertEqual(piloto._atascos, 0)

    def test_tras_los_empujones_escala_a_retroceso(self):
        piloto = self._arrancado()
        # 12 s clavado: dos empujones (2 s de ventana + 0,8 de empujon cada
        # uno) y el escalado.
        consignas = self._rodar(piloto, [1200.0] * 120, t0=1.0)
        estados = [c.estado for c in consignas]
        self.assertIn(
            RETROCESO, estados, "si el empujon no lo saca, no es par: hay algo trabado"
        )
        empujones = sum(1 for c in consignas if c.razon == "empujon de atasco")
        self.assertGreater(empujones, 0, "el retroceso llega DESPUES del empujon")
        self.assertGreater(piloto._retrocesos, 0)

    def test_con_ciclos_irregulares_la_ventana_se_completa(self):
        """El reloj real no da 0,1 s clavados, y eso destapo un fallo.

        Podando el historial por su PRIMER elemento se tira justo la muestra
        que completa la ventana: la edad se queda pegada por debajo del umbral
        y la deteccion no salta jamas.  Con pasos de 0,1 exactos colaba por
        redondeo; en el robot (pasos de 0,098-0,101) la edad se quedo clavada
        en 1,90 s con la ventana en 2,0 y **cero** detecciones en 18 s de robot
        inmovil.  Este test usa esos mismos pasos irregulares.
        """

        import random

        rnd = random.Random(11)
        piloto = self._arrancado()
        t = 1.0
        for _ in range(60):
            t += 0.098 + rnd.random() * 0.005
            piloto.procesar(
                paredes=self._paredes(1200.0 + rnd.gauss(0, 8.4)),
                rumbo_deg=rnd.gauss(0, 0.02),
                ahora=t,
            )
        self.assertGreaterEqual(
            piloto._atascos, 1, "la ventana nunca llego a completarse"
        )

    def test_aguanta_el_ruido_real_del_lidar(self):
        """El ruido de la pared frontal es mayor que la tolerancia.

        Medido con el robot INMOVIL en el banco el 05-09: ``frontal_min``
        oscila 37 mm de amplitud (desviacion 8,4) mientras los dos laterales
        se mueven 2 mm.  La primera version comparaba las dos muestras de los
        extremos de la ventana y un solo barrido ruidoso bastaba para
        declarar que el robot se movia: en 18 s de robot parado en el banco
        NO disparo ni una vez.  Por eso se comparan medianas.
        """

        import random

        rnd = random.Random(3)
        piloto = self._arrancado()
        for i in range(40):
            piloto.procesar(
                paredes=self._paredes(
                    1200.0 + rnd.gauss(0, 8.4),
                    540.0 + rnd.gauss(0, 0.4),
                    417.0 + rnd.gauss(0, 0.6),
                ),
                rumbo_deg=rnd.gauss(0, 0.02),
                ahora=1.0 + i * 0.1,
            )
        self.assertGreaterEqual(piloto._atascos, 1, "el ruido tapo el atasco")

        # Y con el mismo ruido, avanzando de verdad a 100 mm/s: ni un aviso.
        movil = self._arrancado()
        for i in range(40):
            movil.procesar(
                paredes=self._paredes(
                    1200.0 - 10.0 * i + rnd.gauss(0, 8.4),
                    540.0 + rnd.gauss(0, 0.4),
                    417.0 + rnd.gauss(0, 0.6),
                ),
                rumbo_deg=rnd.gauss(0, 0.02),
                ahora=1.0 + i * 0.1,
            )
        self.assertEqual(movil._atascos, 0, "falso positivo con el robot rodando")

    def test_con_el_corredor_justo_no_empuja_hacia_delante(self):
        """Quieto con algo encima: la salida es retroceder, no acelerar."""

        piloto = self._arrancado()
        # 260 mm de corredor: por encima de la emergencia (145) pero muy por
        # debajo del sitio que exige un empujon.
        consignas = self._rodar(piloto, [260.0] * 40, t0=1.0)
        self.assertNotIn("empujon de atasco", [c.razon for c in consignas])
        self.assertIn(RETROCESO, [c.estado for c in consignas])

    def test_sin_medida_longitudinal_no_se_declara_atasco(self):
        """Sin pared frontal ni corredor no hay forma de saberlo, y callarse
        es mejor que empujar a ciegas."""

        piloto = self._arrancado()
        infinito = float("inf")
        for i in range(40):
            piloto.procesar(
                paredes=MapaParedes(
                    timestamp=1.0,
                    izquierda=Recta(400.0, -90.0, 1.0, 40, 1.0),
                    derecha=Recta(600.0, 90.0, 1.0, 40, 1.0),
                    frontal_min_mm=infinito,
                    izquierda_min_mm=400.0,
                    derecha_min_mm=600.0,
                    corredor_mm=infinito,
                ),
                rumbo_deg=0.0,
                ahora=1.0 + i * 0.1,
            )
        self.assertEqual(piloto._atascos, 0)


class PruebaEstados(unittest.TestCase):
    def _paredes(self, frontal=1200.0, izq=400.0, der=600.0):
        return MapaParedes(
            timestamp=1.0,
            frontal=Recta(frontal, 0.0, 1.0, 30, 1.0),
            izquierda=Recta(izq, -90.0, 1.0, 40, 1.0),
            derecha=Recta(der, 90.0, 1.0, 40, 1.0),
            frontal_min_mm=frontal,
            izquierda_min_mm=izq,
            derecha_min_mm=der,
            trasera_min_mm=1800.0,
            # El corredor es lo que decide emergencias y frenado; sin ponerlo
            # queda en infinito y el robot no ve la pared que tiene encima.
            corredor_mm=frontal,
        )

    def test_con_sentido_fijo_no_pasa_por_orientacion(self):
        piloto = Piloto(config())
        piloto.procesar(paredes=self._paredes(), ahora=0.0)
        self.assertEqual(piloto.estado, RECTA)
        self.assertEqual(piloto.sentido, 1)

    def test_auto_resuelve_el_sentido_por_la_asimetria(self):
        cfg = config()
        cfg["control"]["turn_direction"] = "AUTO"
        piloto = Piloto(cfg)
        # Interior a la derecha (mas cerca) -> se gira a la derecha.
        piloto.procesar(paredes=self._paredes(izq=800.0, der=250.0), ahora=0.0)
        self.assertEqual(piloto.sentido, 1)

    def test_auto_prefiere_la_linea_de_piso(self):
        from ..modelos import LineaPiso

        cfg = config()
        cfg["control"]["turn_direction"] = "AUTO"
        piloto = Piloto(cfg)
        piloto.procesar(
            paredes=self._paredes(izq=800.0, der=250.0),
            lineas=[LineaPiso(0.0, "AZUL", 500.0, 0.0, 900)],
            ahora=0.0,
        )
        self.assertEqual(piloto.sentido, -1, "AZUL manda sobre la asimetria")

    def test_una_pared_encima_dispara_el_retroceso(self):
        piloto = Piloto(config())
        piloto.procesar(paredes=self._paredes(), ahora=0.0)
        consigna = piloto.procesar(paredes=self._paredes(frontal=90.0), ahora=0.1)
        self.assertEqual(piloto.estado, RETROCESO)
        self.assertLess(consigna.velocidad, 0)

    def test_un_retroceso_en_mitad_del_giro_no_pierde_la_esquina(self):
        """Reproduce el fallo de la primera corrida con motores (04-09).

        El robot giro los 90 grados, se le echo encima el rincon, retrocedio
        y volvio a RECTA sin contar la esquina.  Con el contador congelado el
        rumbo de referencia queda 90 grados equivocado toda la recta
        siguiente.  Ahora el retroceso devuelve el control al GIRO y la
        esquina la acredita su propia condicion de salida.
        """

        piloto = Piloto(config())
        piloto.procesar(paredes=self._paredes(), ahora=0.0)
        # Frente cerrandose: entra en giro.
        piloto.procesar(paredes=self._paredes(frontal=600.0), rumbo_deg=0.0, ahora=0.5)
        self.assertEqual(piloto.estado, GIRO)
        # Gira de verdad y entonces se encaja contra el rincon.
        piloto.procesar(paredes=self._paredes(frontal=400.0), rumbo_deg=-45.0, ahora=1.0)
        piloto.procesar(paredes=self._paredes(frontal=90.0), rumbo_deg=-85.0, ahora=1.5)
        self.assertEqual(piloto.estado, RETROCESO)
        self.assertEqual(piloto.esquinas, 0, "no se acredita a mitad de maniobra")
        # Sale del rincon: el frente vuelve a abrirse.  Hacen falta dos ciclos
        # despues del retroceso -- uno reanuda el GIRO y el siguiente lo cierra.
        for paso in range(1, 12):
            piloto.procesar(
                paredes=self._paredes(frontal=1400.0), rumbo_deg=-85.0, ahora=2.0 + 0.5 * paso
            )
            if piloto.esquinas:
                break
        self.assertEqual(piloto.esquinas, 1, "la esquina girada se cuenta una vez")
        self.assertEqual(piloto.estado, RECTA)

    def test_un_frente_cerrado_no_deja_el_giro_pasarse_de_rumbo(self):
        """Tope de rumbo en el giro, de la corrida 2 con motores (04-09).

        Un eco espurio a 150 mm mantenia ``frente_abierto`` en falso y el
        robot encadenaba 155 grados en un solo giro, con el rumbo de
        referencia cada vez mas desfasado.  Una esquina son 90: pasado el
        tope se cierra y que la recta corrija.
        """

        piloto = Piloto(config())
        piloto.procesar(paredes=self._paredes(), ahora=0.0)
        piloto.procesar(paredes=self._paredes(frontal=600.0), rumbo_deg=0.0, ahora=0.5)
        self.assertEqual(piloto.estado, GIRO)
        # El frente NUNCA se abre (200 mm), pero se mantiene por encima de la
        # emergencia para que lo que se pruebe sea el tope, no el retroceso.
        rumbo = 0.0
        for paso in range(1, 40):
            rumbo -= 8.0
            piloto.procesar(
                paredes=self._paredes(frontal=200.0), rumbo_deg=rumbo, ahora=0.5 + 0.1 * paso
            )
            if piloto.esquinas:
                break
        self.assertEqual(piloto.esquinas, 1)
        self.assertLessEqual(abs(rumbo), 120.0, "no puede encadenar 155 grados")

    def test_el_disparo_del_giro_apunta_al_offset_de_salida(self):
        """Girar 90 grados a radio R convierte el disparo D en offset D - R.

        La interpolacion anterior se quedaba a medias: para un verde en la fila
        interior pedia salir a offset 731 y disparaba a 905, o sea salida en
        645, con 86 mm todavia por corregir nada mas empezar la recta.
        """

        from ..modelos import PoseCarril

        piloto = Piloto(config())
        piloto.procesar(paredes=self._paredes(), ahora=0.0)
        piloto.pose = PoseCarril(0.0, 0, 2600.0, 500.0, 0.0, True, True)
        # Un rojo en la fila interior pide salir por dentro: 756 mm, que
        # queda dentro de los topes y deja ver la relacion sin recortes.
        pilar = DeteccionPilar(0.0, "ROJO", 0.0, 0.0, "CAMARA", 0.9)
        piloto.mapa.observar(pilar, 1, 1000.0, 574.0, vuelta=0)

        objetivo = piloto.plan.offset_de_paso("ROJO", 574.0, piloto.sentido)
        disparo = piloto._disparo_de_giro_mm()
        radio = piloto.radio_der_mm if piloto.sentido > 0 else piloto.radio_izq_mm
        self.assertAlmostEqual(disparo - radio, objetivo, delta=1.0)

    def test_el_disparo_del_giro_no_baja_del_suelo_de_la_esquina(self):
        """Apurar mas mete el propio giro contra el muro de enfrente.

        Con el suelo en 520 la vuelta cargada pasaba de 0 a 3 retrocesos: no es
        una preferencia, es lo que impone la geometria del giro.
        """

        from ..modelos import PoseCarril

        piloto = Piloto(config())
        piloto.procesar(paredes=self._paredes(), ahora=0.0)
        piloto.pose = PoseCarril(0.0, 0, 2600.0, 500.0, 0.0, True, True)
        # Un verde en la fila interior pide salir pegado al muro exterior.
        pilar = DeteccionPilar(0.0, "VERDE", 0.0, 0.0, "CAMARA", 0.9)
        piloto.mapa.observar(pilar, 1, 1000.0, 574.0, vuelta=0)

        objetivo = piloto.plan.offset_de_paso("VERDE", 574.0, piloto.sentido)
        radio = piloto.radio_der_mm if piloto.sentido > 0 else piloto.radio_izq_mm
        self.assertLess(objetivo + radio, piloto.giro_disparo_min_mm)
        self.assertEqual(piloto._disparo_de_giro_mm(), piloto.giro_disparo_min_mm)

    def test_el_timeout_de_ronda_para_el_robot(self):
        piloto = Piloto(config())
        piloto.procesar(paredes=self._paredes(), ahora=0.0)
        consigna = piloto.procesar(paredes=self._paredes(), ahora=400.0)
        self.assertEqual(consigna.estado, FIN)
        self.assertEqual(consigna.velocidad, 0)

    def test_la_direccion_no_salta_de_golpe(self):
        """El limitador protege la mecanica y evita que el LiDAR se vea la rueda."""

        piloto = Piloto(config())
        piloto.procesar(paredes=self._paredes(), ahora=0.0)
        anterior = 0.0
        for paso in range(1, 12):
            consigna = piloto.procesar(
                paredes=self._paredes(izq=120.0), ahora=paso * 0.1
            )
            if piloto.estado == RECTA:
                self.assertLessEqual(
                    abs(consigna.angulo - anterior), piloto.slew_direccion + 1e-6
                )
            anterior = consigna.angulo


if __name__ == "__main__":
    unittest.main()
