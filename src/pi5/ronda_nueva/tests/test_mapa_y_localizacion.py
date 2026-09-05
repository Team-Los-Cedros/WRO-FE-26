"""Localizacion relativa a paredes, mapa de casillas y fusion camara-LiDAR."""

import unittest

from ..fusion import FusionPilares
from ..localizacion import Localizador, diferencia_angular
from ..mapa_pista import MapaPista
from ..modelos import DeteccionPilar, MapaParedes, ObjetoLidar, Recta
from ..percepcion_lidar import PercepcionLidar
from .apoyo import barrido_sintetico, config_minima, pose_en_carril


def _paredes(frontal=1200.0, izquierda=400.0, derecha=600.0, trasera=1800.0, angulo=0.0):
    return MapaParedes(
        timestamp=1.0,
        frontal=Recta(frontal, angulo, 1.0, 30, 1.0),
        trasera=Recta(trasera, 180.0 + angulo, 1.0, 30, 1.0),
        izquierda=Recta(izquierda, -90.0 + angulo, 1.0, 40, 1.0),
        derecha=Recta(derecha, 90.0 + angulo, 1.0, 40, 1.0),
        frontal_min_mm=frontal,
        trasera_min_mm=trasera,
        izquierda_min_mm=izquierda,
        derecha_min_mm=derecha,
    )


class PruebaAvanceSinRectaFrontal(unittest.TestCase):
    """Sin recta frontal ajustada, el avance se PREDICE; no se inventa.

    Se probo lo contrario -- usar la minima del sector frontal como medida de
    ultimo recurso -- y en pista fue peor: corrida 8 del 05-09, 17 retrocesos
    frente a 7 y la velocidad media de 24,7 a 10,6 PWM.  Justo despues de una
    esquina el robot todavia va cruzado y lo que tiene a 300-800 mm no es el
    muro de la recta nueva.
    """

    def _sin_frontal(self, frontal_min):
        return MapaParedes(
            timestamp=1.0,
            frontal=None,
            trasera=None,
            izquierda=Recta(400.0, -90.0, 1.0, 40, 1.0),
            derecha=Recta(600.0, 90.0, 1.0, 40, 1.0),
            frontal_min_mm=frontal_min,
            trasera_min_mm=float("inf"),
            izquierda_min_mm=400.0,
            derecha_min_mm=600.0,
            corredor_mm=frontal_min,
        )

    def test_la_minima_frontal_no_se_usa_como_medida(self):
        loc = Localizador(config_minima())
        loc.actualizar(_paredes(frontal=2900.0), 0.0, sentido=1, timestamp=1.0)
        pose = None
        for paso in range(1, 8):
            pose = loc.actualizar(
                self._sin_frontal(321.0), 0.0, sentido=1, timestamp=1.0 + 0.1 * paso
            )
        self.assertGreater(pose.avance_mm, 2000.0)
        self.assertFalse(pose.avance_valido)

    def test_sin_ninguna_medida_sigue_prediciendo(self):
        loc = Localizador(config_minima())
        loc.actualizar(_paredes(frontal=2900.0), 0.0, sentido=1, timestamp=1.0)
        pose = loc.actualizar(
            self._sin_frontal(float("inf")), 0.0, sentido=1, timestamp=1.1
        )
        self.assertFalse(pose.avance_valido)


class PruebaLocalizador(unittest.TestCase):
    def setUp(self):
        self.loc = Localizador(config_minima())

    def test_horario_toma_el_muro_izquierdo_como_exterior(self):
        pose = self.loc.actualizar(_paredes(), 0.0, sentido=1, timestamp=1.0)
        self.assertTrue(pose.offset_valido)
        self.assertAlmostEqual(pose.offset_mm, 400.0, delta=60.0)
        self.assertEqual(pose.fuente_offset, "EXTERIOR")

    def test_antihorario_toma_el_derecho(self):
        pose = self.loc.actualizar(_paredes(), 0.0, sentido=-1, timestamp=1.0)
        self.assertAlmostEqual(pose.offset_mm, 600.0, delta=60.0)

    def test_sin_muro_exterior_usa_el_interior_y_el_ancho_del_carril(self):
        paredes = _paredes()
        paredes = MapaParedes(
            timestamp=1.0,
            frontal=paredes.frontal,
            trasera=paredes.trasera,
            izquierda=None,
            derecha=paredes.derecha,
            frontal_min_mm=1200.0,
            izquierda_min_mm=float("inf"),
            derecha_min_mm=600.0,
        )
        pose = self.loc.actualizar(paredes, 0.0, sentido=1, timestamp=1.0)
        self.assertEqual(pose.fuente_offset, "INTERIOR")
        self.assertAlmostEqual(pose.offset_mm, 400.0, delta=60.0)

    def test_sin_muro_frontal_usa_el_trasero_y_el_largo_de_la_recta(self):
        base = _paredes()
        paredes = MapaParedes(
            timestamp=1.0,
            frontal=None,
            trasera=base.trasera,
            izquierda=base.izquierda,
            derecha=base.derecha,
            frontal_min_mm=float("inf"),
            trasera_min_mm=1800.0,
            izquierda_min_mm=400.0,
            derecha_min_mm=600.0,
        )
        pose = self.loc.actualizar(paredes, 0.0, sentido=1, timestamp=1.0)
        self.assertEqual(pose.fuente_avance, "TRASERA")
        self.assertAlmostEqual(pose.avance_mm, 1200.0, delta=80.0)

    def test_una_esquina_avanza_el_segmento_y_reancla_el_rumbo(self):
        self.loc.actualizar(_paredes(), 0.0, sentido=1, timestamp=1.0)
        self.loc.anotar_esquina(sentido=1)
        self.assertEqual(self.loc.segmento, 1)
        self.assertAlmostEqual(self.loc.rumbo_cardinal_deg, -90.0)
        # El rumbo de la IMU sigue creciendo en negativo al girar a la derecha.
        pose = self.loc.actualizar(_paredes(), -88.0, sentido=1, timestamp=2.0)
        self.assertAlmostEqual(pose.rumbo_error_deg, 2.0, delta=0.01)

    def test_un_salto_imposible_se_rechaza_pero_no_para_siempre(self):
        """Un rechazo aislado es ruido; tres seguidos son la prediccion mal."""

        for t in range(1, 4):
            self.loc.actualizar(_paredes(), 0.0, sentido=1, timestamp=float(t))
        salto = self.loc.actualizar(
            _paredes(izquierda=1500.0), 0.0, sentido=1, timestamp=4.0
        )
        self.assertFalse(salto.offset_valido)
        for t in range(5, 9):
            pose = self.loc.actualizar(
                _paredes(izquierda=1500.0), 0.0, sentido=1, timestamp=float(t)
            )
        self.assertTrue(pose.offset_valido)

    def test_diferencia_angular_envuelve(self):
        self.assertAlmostEqual(diferencia_angular(179.0, -179.0), -2.0)
        self.assertAlmostEqual(diferencia_angular(-179.0, 179.0), 2.0)


class PruebaPuntosSobreLaPista(unittest.TestCase):
    """El paso de (x, y) del robot a (avance, offset) de la recta."""

    def test_contra_un_barrido_sintetico_real(self):
        percepcion = PercepcionLidar(config_minima())
        loc = Localizador(config_minima())
        x, y, rumbo = pose_en_carril(0, 1500.0, 400.0)
        px, py, _ = pose_en_carril(0, 900.0, 650.0)
        scan = barrido_sintetico(x, y, rumbo, pilares=[(px, py, 100.0)])
        paredes, objetos, _hueco = percepcion.procesar(scan, 1.0)
        self.assertTrue(objetos)
        pilar = objetos[0]
        avance = loc.avance_de_punto(pilar.x_mm, pilar.y_mm, paredes)
        offset = loc.offset_de_punto(pilar.x_mm, pilar.y_mm, paredes, sentido=1)
        self.assertAlmostEqual(avance, 900.0, delta=70.0)
        self.assertAlmostEqual(offset, 650.0, delta=70.0)


class PruebaAvanceTrasEsquina(unittest.TestCase):
    """El salto de avance que aparece 0,3-0,4 s DESPUES de contar la esquina.

    Medido en las corridas 7, 8 y 9 del 05-09: seis o siete por corrida, todos
    en esa ventana.  El mecanismo, leido en el codigo:

    1. `anotar_esquina` pone `_avance_mm = 3000` (el marco de la recta nueva).
    2. El robot sigue CRUZADO, asi que la pared que el LiDAR ajusta por delante
       no es la que cierra la recta nueva: esta a 800 mm.
    3. El filtro rechaza esa medida por salto imposible... tres veces, que a
       10 Hz son 0,3 s exactos.
    4. Y entonces salta la resincronizacion de `_mezclar`, que ADOPTA la medida
       entera. El avance se va a 800 mm en medio de una recta de 3000.

    Esa pose mala es la que alimenta el disparo de esquina, y por eso el giro
    se dispara a 448 mm de mediana contra un diseño de 680-1050.
    """

    def _paredes(self, frontal_mm, t):
        return MapaParedes(
            timestamp=t,
            frontal=Recta(frontal_mm, 0.0, 1.0, 30, 1.0),
            izquierda=Recta(400.0, -90.0, 1.0, 40, 1.0),
            derecha=Recta(600.0, 90.0, 1.0, 40, 1.0),
            frontal_min_mm=frontal_mm,
            izquierda_min_mm=400.0,
            derecha_min_mm=600.0,
            corredor_mm=frontal_mm,
        )

    def _tras_la_esquina(self, rumbo_error_deg, ciclos=8):
        """Recien contada la esquina, con el robot cruzado `rumbo_error` grados
        y una pared espuria a 800 mm por delante."""

        loc = Localizador(config_minima())
        loc.reiniciar()
        # OJO con el reloj: `primera` se detecta con `_ultimo_t <= 0`, asi que
        # sembrar en t=0 deja el filtro en "primer ciclo" para siempre y adopta
        # cualquier medida.  En el robot `time.monotonic()` es grande y no
        # pasa, pero aqui hay que arrancar por encima de cero.
        loc.actualizar(self._paredes(2900.0, 10.0), 0.0, 1, 25.0, 10.0)
        loc.anotar_esquina(1)
        avances = []
        for i in range(ciclos):
            t = 10.0 + 0.1 * (i + 1)
            pose = loc.actualizar(
                self._paredes(800.0, t),
                # El rumbo de la IMU no ha alcanzado aun el cardinal nuevo.
                loc.rumbo_cardinal_deg + rumbo_error_deg,
                1,
                25.0,
                t,
            )
            avances.append(pose.avance_mm)
        return avances

    def test_cruzado_no_adopta_la_pared_espuria(self):
        avances = self._tras_la_esquina(rumbo_error_deg=40.0)
        self.assertGreater(
            min(avances),
            2000.0,
            "con el robot cruzado 40 grados, la pared de delante no es la que "
            "cierra la recta: adoptarla manda el avance de 3000 a 800",
        )

    def test_ya_encarado_si_se_resincroniza(self):
        """La resincronizacion tiene que seguir existiendo.

        Es lo que arregla un choque o que alguien mueva el robot; solo se le
        exige estar encarado a la recta que dice medir.
        """

        avances = self._tras_la_esquina(rumbo_error_deg=3.0)
        self.assertLess(
            min(avances),
            1200.0,
            "encarado a la recta, la medida manda y el filtro debe adoptarla",
        )


class PruebaMapaPista(unittest.TestCase):
    def setUp(self):
        self.mapa = MapaPista(config_minima())

    def _pilar(self, color="ROJO", confianza=0.9):
        return DeteccionPilar(1.0, color, 0.0, 0.0, "FUSION", confianza)

    def test_el_indice_es_el_orden_de_encuentro(self):
        casilla = self.mapa.observar(self._pilar(), 0, 2000.0, 380.0)
        self.assertEqual((casilla.segmento, casilla.indice), (0, 0))
        casilla = self.mapa.observar(self._pilar(), 0, 1000.0, 380.0)
        self.assertEqual(casilla.indice, 2)

    def test_el_mapa_presta_el_color_a_una_deteccion_que_lo_perdio(self):
        """De cerca la camara deja de clasificar, y ahi es donde se decide.

        Medido el 05-09 sobre tres corridas: la tasa de color del poste que usa
        el planificador es 73-75 % entre 0,75 y 1,25 m y cae al **13 %** por
        debajo de 250 mm.  Sin color no se aplica la regla de WRO y el lado se
        elige por el hueco mayor: 4 de 12 rebases salieron por el lado
        equivocado, uno con el poste a 6 mm del eje.
        """

        self.mapa.observar(self._pilar("VERDE"), 0, 1500.0, 574.0)
        self.assertEqual(
            self.mapa.color_recordado(0, 1500.0, 574.0),
            "VERDE",
            "la casilla observada tiene que prestar su color",
        )

    def test_no_presta_el_color_de_la_otra_fila(self):
        """Las dos filas del sorteo estan a 380 y 574 mm del muro exterior.

        Prestar el color entre filas mandaria el robot por el lado contrario,
        que es peor que no saber el color.
        """

        self.mapa.observar(self._pilar("ROJO"), 0, 1500.0, 380.0)
        self.assertIsNone(
            self.mapa.color_recordado(0, 1500.0, 574.0 + 250.0),
            "un poste lejos de la casilla no puede heredar su color",
        )

    def test_una_casilla_nunca_vista_no_presta_nada(self):
        self.assertIsNone(self.mapa.color_recordado(0, 1500.0, 380.0))

    def test_fuera_de_banda_no_se_memoriza(self):
        """1250 mm no es ninguna posicion del sorteo: no ensucia el mapa.

        Ojo: eso NO significa que no se esquive.  El planificador usa ademas
        las detecciones vivas, asi que un poste mal colocado se rodea igual.
        """

        self.assertIsNone(self.mapa.observar(self._pilar(), 0, 1250.0, 380.0))

    def test_un_pilar_de_la_recta_siguiente_cambia_de_marco(self):
        """avance_siguiente = largo - offset_actual; offset_siguiente = avance."""

        # Poste al otro lado del bloque interior: offset 2000, avance 500.
        # En la recta siguiente eso son avance 1000 (la casilla p3) y offset
        # 500, o sea el centro del carril.
        casilla = self.mapa.observar(self._pilar("VERDE"), 0, 500.0, 2000.0)
        self.assertIsNotNone(casilla)
        self.assertEqual((casilla.segmento, casilla.indice), (1, 2))
        entrada = self.mapa.entrada(1, 2)
        self.assertAlmostEqual(entrada.avance_mm, 1000.0, delta=1.0)
        self.assertAlmostEqual(entrada.offset_mm, 500.0, delta=1.0)

    def test_los_votos_mandan_sobre_una_deteccion_suelta(self):
        for _ in range(6):
            self.mapa.observar(self._pilar("ROJO", 0.9), 0, 1500.0, 380.0)
        self.mapa.observar(self._pilar("VERDE", 0.4), 0, 1500.0, 380.0)
        self.assertEqual(self.mapa.entrada(0, 1).color, "ROJO")
        self.assertGreater(self.mapa.entrada(0, 1).confianza, 0.85)

    def test_baja_confianza_no_vota(self):
        self.assertIsNone(self.mapa.observar(self._pilar("ROJO", 0.05), 0, 1500.0, 380.0))

    def test_la_fila_se_deduce_del_offset(self):
        self.mapa.observar(self._pilar(), 0, 1500.0, 380.0)
        self.assertEqual(self.mapa.entrada(0, 1).lado, "EXTERIOR")
        self.mapa.observar(self._pilar(), 1, 1500.0, 574.0)
        self.assertEqual(self.mapa.entrada(1, 1).lado, "INTERIOR")

    def test_el_resumen_cabe_en_una_celda_del_csv(self):
        self.mapa.observar(self._pilar("ROJO"), 0, 2000.0, 380.0)
        self.mapa.observar(self._pilar("VERDE"), 2, 1000.0, 574.0)
        self.assertEqual(self.mapa.resumen(), "R0:R.. R1:... R2:..V R3:...")
        self.assertEqual(self.mapa.casillas_conocidas(), 2)


class PruebaFusion(unittest.TestCase):
    def setUp(self):
        self.fusion = FusionPilares(
            {"fusion": {}, "track": {"pillar_width_mm": 100.0}}
        )

    def test_empareja_por_cercania_en_milimetros(self):
        visual = DeteccionPilar(1.0, "VERDE", 200.0, 800.0, "CAMARA", 0.9)
        objeto = ObjetoLidar(1.0, 190.0, 760.0, 782.0, 14.0, 90.0, 8)
        salida = self.fusion.asociar([visual], [objeto], 1.0)
        self.assertEqual(len(salida), 1)
        self.assertEqual(salida[0].fuente, "FUSION")
        self.assertEqual(salida[0].color, "VERDE")

    def test_el_centroide_lidar_se_corrige_hacia_el_centro_del_poste(self):
        """El LiDAR solo ve la cara frontal: su centroide queda medio poste corto."""

        objeto = ObjetoLidar(1.0, 0.0, 800.0, 800.0, 0.0, 90.0, 8)
        corregido = self.fusion._centro_corregido(objeto)
        self.assertAlmostEqual(corregido[1], 850.0, delta=1.0)

    def test_un_blob_sin_lidar_sobrevive(self):
        """El plano del LiDAR pasa por encima de los postes lejanos."""

        visual = DeteccionPilar(1.0, "ROJO", -300.0, 2100.0, "CAMARA", 0.8)
        salida = self.fusion.asociar([visual], [], 1.0)
        self.assertEqual(len(salida), 1)
        self.assertEqual(salida[0].fuente, "CAMARA")

    def test_un_objeto_sin_color_se_emite_como_estorbo(self):
        objeto = ObjetoLidar(1.0, 100.0, 500.0, 510.0, 11.0, 90.0, 6)
        salida = self.fusion.asociar([], [objeto], 1.0)
        self.assertEqual(len(salida), 1)
        self.assertEqual(salida[0].color, "")
        self.assertLess(salida[0].confianza, 0.5)

    def test_no_se_empareja_dos_veces_el_mismo_objeto(self):
        a = DeteccionPilar(1.0, "ROJO", 200.0, 800.0, "CAMARA", 0.9)
        b = DeteccionPilar(1.0, "VERDE", 210.0, 810.0, "CAMARA", 0.9)
        objeto = ObjetoLidar(1.0, 205.0, 760.0, 787.0, 15.0, 90.0, 8)
        salida = self.fusion.asociar([a, b], [objeto], 1.0)
        fuentes = sorted(pilar.fuente for pilar in salida)
        self.assertEqual(fuentes, ["CAMARA", "FUSION"])


if __name__ == "__main__":
    unittest.main()
