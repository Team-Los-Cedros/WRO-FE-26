"""Vision de pista sobre imagenes sinteticas, y validacion de configuracion."""

import json
import math
import unittest
from pathlib import Path

import cv2
import numpy as np

from ..config import (
    ErrorConfiguracion,
    calibraciones_pendientes,
    cargar_configuracion,
    exigir_listo_para_mover,
    validar_configuracion,
)
from ..geometria_suelo import ProyectorSuelo, desde_montaje
from ..vision_pista import VisionPista


RUTA_CONFIG = Path(__file__).resolve().parents[1] / "configuracion.json"
ANCHO, ALTO = 640, 360


def config_vision(con_suelo=True):
    base = json.loads(RUTA_CONFIG.read_text(encoding="utf-8"))
    base["camera"]["width"] = ANCHO
    base["camera"]["height"] = ALTO
    base["camera"]["principal_x_px"] = ANCHO / 2.0
    base["camera"]["principal_y_px"] = ALTO / 2.0
    base["camera"]["array_color_order"] = "BGR"
    if con_suelo:
        # Montaje de referencia del mastil nuevo: 400 mm de altura y 28
        # grados de cabeceo.  Con esa optica el suelo entra desde 265 mm por
        # delante hasta unos 3,1 m, que es el alcance que el mapa necesita
        # para anticipar los tres postes de la recta.
        H = desde_montaje(
            ANCHO, ALTO, base["camera"]["hfov_deg"], 400.0, 28.0, adelante_mm=-80.0
        )
        base["camera"]["ground_homography"] = {
            "ready": True,
            "matrix": H.tolist(),
            "max_range_mm": 3200.0,
            "min_forward_mm": 60.0,
        }
    else:
        base["camera"]["ground_homography"] = {"ready": False}
    base["vision"]["min_area_px"] = 60
    base["vision"]["morph_open_px"] = 3
    base["vision"]["morph_close_px"] = 3
    return base


def _proyector(cfg):
    H = np.asarray(cfg["camera"]["ground_homography"]["matrix"], dtype=float)
    focal = (ANCHO / 2.0) / math.tan(math.radians(cfg["camera"]["hfov_deg"]) / 2.0)
    return ProyectorSuelo(H, ANCHO, ALTO, focal_px=focal)


def escena(cfg, postes=(), magenta=(), lineas=()):
    """Cuadro sintetico: lona gris clara, muros negros, objetos de colores.

    Los objetos se dibujan EN SU SITIO, proyectando su base y su altura con la
    misma homografia que la vision va a usar para deshacer la operacion.  Asi
    el test comprueba la cadena entera y no solo que encuentra manchas.
    """

    proyector = _proyector(cfg)
    imagen = np.zeros((ALTO, ANCHO, 3), dtype=np.uint8)
    imagen[:, :] = (150, 150, 150)          # lona: gris, baja saturacion

    # Todo lo que queda por encima del horizonte es muro: oscuro.  Si NINGUNA
    # fila falla es que el cuadro entero ve suelo y no hay que pintar muro
    # -- pintarlo por defecto dejaba la escena entera negra y la vision sin
    # nada que segmentar.
    fallidas = [
        v for v in range(ALTO) if proyector.punto_suelo(ANCHO / 2.0, v) is None
    ]
    if fallidas:
        imagen[: max(0, min(fallidas) + 4), :] = (28, 28, 28)

    def _dibujar(x_mm, y_mm, alto_mm, ancho_mm, color_bgr):
        pie = proyector.punto_imagen(x_mm, y_mm)
        if pie is None:
            return
        distancia = math.hypot(x_mm, y_mm)
        escala = proyector.focal_px / max(1.0, distancia)
        alto_px = int(alto_mm * escala)
        ancho_px = max(2, int(ancho_mm * escala))
        u, v = int(pie[0]), int(pie[1])
        cv2.rectangle(
            imagen,
            (u - ancho_px // 2, v - alto_px),
            (u + ancho_px // 2, v),
            color_bgr,
            -1,
        )

    for x_mm, y_mm, color in postes:
        _dibujar(x_mm, y_mm, 100.0, 100.0, (0, 0, 220) if color == "ROJO" else (0, 190, 0))
    for x_mm, y_mm in magenta:
        _dibujar(x_mm, y_mm, 60.0, 200.0, (200, 0, 200))
    for x_mm, y_mm, color in lineas:
        pie = proyector.punto_imagen(x_mm, y_mm)
        if pie is None:
            continue
        escala = proyector.focal_px / max(1.0, math.hypot(x_mm, y_mm))
        cv2.rectangle(
            imagen,
            (int(pie[0] - 400 * escala), int(pie[1] - 25 * escala)),
            (int(pie[0] + 400 * escala), int(pie[1] + 25 * escala)),
            (200, 60, 0) if color == "AZUL" else (0, 110, 230),
            -1,
        )
    return imagen


class PruebaVision(unittest.TestCase):
    def test_un_poste_se_mide_en_milimetros(self):
        cfg = config_vision()
        vision = VisionPista(cfg)
        paquete = vision.procesar(escena(cfg, postes=[(200.0, 900.0, "ROJO")]), 1.0)
        self.assertEqual(len(paquete.pilares), 1)
        pilar = paquete.pilares[0]
        self.assertEqual(pilar.color, "ROJO")
        self.assertEqual(pilar.fuente, "CAMARA")
        self.assertAlmostEqual(pilar.x_mm, 200.0, delta=60.0)
        self.assertAlmostEqual(pilar.y_mm, 900.0, delta=90.0)

    def test_la_medida_aguanta_de_cerca_y_de_lejos(self):
        """El LiDAR pierde los postes lejanos y la camara los cercanos.

        Que la camara acierte a 400 y a 1800 mm es justo lo que tapa el agujero
        del plano del LiDAR, que a partir de cierta distancia pasa por encima
        de un poste de 100 mm.
        """

        cfg = config_vision()
        vision = VisionPista(cfg)
        for distancia in (400.0, 900.0, 1800.0):
            paquete = vision.procesar(
                escena(cfg, postes=[(0.0, distancia, "VERDE")]), 1.0
            )
            with self.subTest(distancia=distancia):
                self.assertEqual(len(paquete.pilares), 1)
                self.assertAlmostEqual(
                    paquete.pilares[0].y_mm, distancia, delta=0.12 * distancia
                )

    def test_dos_postes_de_colores_distintos(self):
        cfg = config_vision()
        vision = VisionPista(cfg)
        paquete = vision.procesar(
            escena(cfg, postes=[(-250.0, 800.0, "ROJO"), (250.0, 1300.0, "VERDE")]), 1.0
        )
        colores = sorted(p.color for p in paquete.pilares)
        self.assertEqual(colores, ["ROJO", "VERDE"])
        rojo = next(p for p in paquete.pilares if p.color == "ROJO")
        self.assertLess(rojo.x_mm, 0.0)

    def test_los_muros_del_cajon_se_ven_como_magenta(self):
        cfg = config_vision()
        vision = VisionPista(cfg)
        paquete = vision.procesar(escena(cfg, magenta=[(-300.0, 1000.0)]), 1.0)
        self.assertTrue(paquete.magenta)
        self.assertLess(paquete.magenta[0].x_mm, 0.0)

    def test_las_lineas_del_piso_dan_el_sentido(self):
        """Sustituyen al sensor de color, que en la Pi 5 da SIN_SENSOR."""

        cfg = config_vision()
        vision = VisionPista(cfg)
        paquete = vision.procesar(escena(cfg, lineas=[(0.0, 700.0, "AZUL")]), 1.0)
        colores = [linea.color for linea in paquete.lineas]
        self.assertIn("AZUL", colores)

    def test_sin_homografia_sigue_dando_una_distancia_por_la_altura(self):
        """Es el modo del primer dia de montaje: peor, pero utilizable."""

        cfg_con = config_vision()
        imagen = escena(cfg_con, postes=[(0.0, 900.0, "ROJO")])
        vision = VisionPista(config_vision(con_suelo=False))
        paquete = vision.procesar(imagen, 1.0)
        self.assertEqual(len(paquete.pilares), 1)
        self.assertEqual(paquete.pilares[0].fuente, "ALTURA")
        self.assertAlmostEqual(paquete.pilares[0].y_mm, 900.0, delta=200.0)

    def test_un_poste_pegado_al_muro_del_fondo_si_se_detecta(self):
        """El caso que el AND con el poligono de espacio libre perdia.

        Un poste en medio de la lona es un AGUJERO del componente de suelo y al
        rellenar el contorno queda dentro del poligono.  Uno pegado al muro del
        fondo no: es una MUESCA del borde superior, y el AND lo dejaba fuera.
        Medido en el robot el 04-09, un rojo perfectamente visible a 1,4 m
        pasaba de 725 pixeles de mascara a 7.

        Preguntar por la lona que hay DEBAJO de la base cubre los dos casos.
        """

        cfg = config_vision()
        vision = VisionPista(cfg)
        proyector = _proyector(cfg)

        imagen = escena(cfg, postes=[(150.0, 1400.0, "ROJO")])
        # Muro del fondo justo detras del poste: se pinta desde la fila donde
        # el poste apoya hacia arriba, que es lo que hace la muesca.
        pie = proyector.punto_imagen(150.0, 1400.0)
        cv2.rectangle(imagen, (0, 0), (ANCHO, int(pie[1])), (26, 26, 26), -1)
        # Y se vuelve a dibujar el poste encima, apoyado en el borde del muro.
        escala = proyector.focal_px / 1400.0
        alto_px, ancho_px = int(100.0 * escala), int(100.0 * escala)
        cv2.rectangle(
            imagen,
            (int(pie[0] - ancho_px // 2), int(pie[1] - alto_px)),
            (int(pie[0] + ancho_px // 2), int(pie[1])),
            (0, 0, 220),
            -1,
        )

        paquete = vision.procesar(imagen, 1.0)
        self.assertEqual(len(paquete.pilares), 1, "el poste pegado al muro tiene que verse")
        self.assertAlmostEqual(paquete.pilares[0].y_mm, 1400.0, delta=180.0)

    def test_una_mancha_sobre_el_muro_no_es_un_poste(self):
        """Prueba del poligono de espacio libre: sin lona debajo, no cuenta.

        Es el filtro que descarta un reflejo, una camiseta roja del publico o
        cualquier cosa que no este apoyada en la pista.  La version anterior lo
        hacia contando pixeles blancos en una banda bajo la caja; aqui es un
        AND contra el poligono de suelo relleno, de una sola pasada.
        """

        cfg = config_vision()
        imagen = escena(cfg, postes=[(0.0, 900.0, "VERDE")])
        # Muro negro arriba y una mancha roja dentro, con 22 px de muro entre
        # su base y la lona.  Antes se dejaban 8 y la morfologia los cerraba:
        # la mancha acababa tocando el suelo y el test no probaba lo que decia
        # probar.  El muro no puede bajar mas o taparia al poste verde, que a
        # 900 mm apoya sobre la fila 132.
        cv2.rectangle(imagen, (0, 0), (ANCHO, 60), (25, 25, 25), -1)
        cv2.rectangle(imagen, (300, 5), (345, 38), (0, 0, 220), -1)
        vision = VisionPista(cfg)
        paquete = vision.procesar(imagen, 1.0)
        colores = [pilar.color for pilar in paquete.pilares]
        self.assertNotIn("ROJO", colores, "la mancha del muro no esta apoyada")
        self.assertIn("VERDE", colores, "el poste de verdad si tiene que salir")

    def test_el_cuadro_vacio_no_inventa_nada(self):
        cfg = config_vision()
        vision = VisionPista(cfg)
        paquete = vision.procesar(escena(cfg), 1.0)
        self.assertEqual(paquete.pilares, ())
        self.assertGreater(paquete.suelo_px, 1000)

    def test_procesar_a_media_resolucion_da_los_mismos_milimetros(self):
        """El color se procesa reducido para ahorrar CPU; la medida no cambia.

        En la Pi 5 el cuadro completo a 1280x720 costaba 49 ms por las seis
        mascaras HSV y sus doce pasadas de morfologia.  Reducir a la mitad de
        lado deja la cuarta parte de pixeles, pero la homografia sigue
        calibrada en el cuadro entero: si el reescalado de coordenadas tuviera
        un fallo, la distancia saldria justo al doble o a la mitad.
        """

        cfg_completo = config_vision()
        cfg_completo["vision"]["process_scale"] = 1.0
        cfg_medio = config_vision()
        cfg_medio["vision"]["process_scale"] = 0.5

        imagen = escena(cfg_completo, postes=[(180.0, 1100.0, "ROJO")])
        completo = VisionPista(cfg_completo).procesar(imagen, 1.0)
        medio = VisionPista(cfg_medio).procesar(imagen, 1.0)

        self.assertEqual(len(completo.pilares), 1)
        self.assertEqual(len(medio.pilares), 1)
        self.assertAlmostEqual(
            medio.pilares[0].x_mm, completo.pilares[0].x_mm, delta=45.0
        )
        self.assertAlmostEqual(
            medio.pilares[0].y_mm, completo.pilares[0].y_mm, delta=90.0
        )

    def test_un_tablero_de_pie_no_se_confunde_con_una_linea(self):
        """El delimitador del cajón del banco es violeta, no magenta.

        Medido el 04-09: H 127-131, S 85-140, V 34-77.  El magenta oficial de
        WRO (#F702F9) daria H 150, S 253, V 249, asi que ese delimitador cae
        dentro del rango de la LINEA AZUL del piso (H 125-132) y ningun umbral
        de color los separa.  Se distinguen por geometria: proyectando al suelo
        el borde de arriba y el de abajo, una linea pintada es plana y un
        tablero de pie manda su borde superior al infinito.
        """

        cfg = config_vision()
        vision = VisionPista(cfg)
        proyector = _proyector(cfg)

        # Un tablero vertical: base a 700 mm, 250 mm de alto.
        imagen = escena(cfg)
        pie = proyector.punto_imagen(0.0, 700.0)
        escala = proyector.focal_px / 700.0
        alto_px = int(250.0 * escala)
        ancho_px = int(600.0 * escala)
        cv2.rectangle(
            imagen,
            (int(pie[0] - ancho_px // 2), int(pie[1] - alto_px)),
            (int(pie[0] + ancho_px // 2), int(pie[1])),
            (200, 60, 0),          # el mismo BGR que usa la linea azul
            -1,
        )
        paquete = vision.procesar(imagen, 1.0)
        self.assertEqual(
            [linea.color for linea in paquete.lineas],
            [],
            "un tablero de pie no puede contarse como linea de sentido",
        )

        # Y la linea de verdad, tumbada, si tiene que salir.
        plana = escena(cfg, lineas=[(0.0, 700.0, "AZUL")])
        self.assertIn("AZUL", [l.color for l in vision.procesar(plana, 1.0).lineas])

    def test_sin_homografia_no_hay_lineas_de_piso(self):
        """Limitacion conocida, no un fallo: una linea sin milimetros no sirve.

        A un pilar se le puede estimar la distancia por su altura aparente,
        que es conocida (100 mm).  Una linea pintada no tiene altura, asi que
        sin homografia no hay forma de decir a que distancia esta, y un valor
        inventado enganaria al filtro que decide cual es la linea que el robot
        va a cruzar.

        La consecuencia practica: sin calibrar, el sentido de giro sale de la
        asimetria de las paredes (ver Piloto._resolver_sentido), no del color.
        """

        cfg_con = config_vision()
        imagen = escena(cfg_con, lineas=[(0.0, 700.0, "NARANJA")])
        vision = VisionPista(config_vision(con_suelo=False))
        self.assertEqual(vision.procesar(imagen, 1.0).lineas, ())

    def test_bearing_para_x_es_cero_en_el_centro_optico(self):
        vision = VisionPista(config_vision())
        self.assertAlmostEqual(vision.bearing_para_x(ANCHO / 2.0), 0.0, places=6)
        self.assertGreater(vision.bearing_para_x(ANCHO - 1.0), 20.0)


class PruebaMontajeMedido(unittest.TestCase):
    """Numeros que salieron de una medida y que nadie debe ajustar a ojo."""

    def test_la_mascara_del_mastil_es_la_medida_en_la_pi5(self):
        # Medido el 2026-09-05 con herramientas/diag_mastil.py, tres veces
        # seguidas: el mastil ocupa 141..212 grados a 35-68 mm.  A esa
        # distancia no cabe pista: el LiDAR va al ras del parachoques y el
        # robot se extiende 222 mm hacia atras.
        #
        # La mascara que acababa en 195 dejaba los grados 196..210 de la
        # estructura dentro de rear_sector_deg: trasera_min se clavaba entre
        # 44 y 58 mm y bloqueaba el parqueo.  El soporte de camara, 28..54
        # grados, no se anade como segunda cuna: se saca estrechando el
        # comienzo del sector derecho a 56 grados.
        config = cargar_configuracion(str(RUTA_CONFIG))
        self.assertEqual(config["lidar"]["blind_sectors_deg"], [[140.0, 213.0]])
        self.assertEqual(config["lidar"]["right_sector_deg"], [56.0, 135.0])
        self.assertEqual(config["lidar"]["rear_shoulder_offset_deg"], [40.0, 60.0])
        self.assertEqual(config["lidar"]["rear_shoulder_wall_fraction"], 0.8)
        self.assertFalse(config["control"]["line_avance_ceiling_enabled"])
        self.assertEqual(config["control"]["line_avance_ceiling_margin_mm"], 1200.0)

    def test_el_modo_del_sensor_no_se_toca_al_cambiar_la_resolucion(self):
        # El CAMPO lo fija el modo raw, no la resolucion de salida: 2304x1296
        # usa el area completa del IMX708, mientras el modo nativo 1536x864
        # recorta a 3072x1728 y tira un tercio del angulo.  Verificado en la
        # Pi 5 el 05-09: pidiendo main 1536x864 con raw 2304x1296, el
        # ScalerCrop sigue siendo (0, 0, 4608, 2592), igual que a 1280x720.
        config = cargar_configuracion(str(RUTA_CONFIG))
        camara = config["camera"]
        self.assertEqual(camara["raw_sensor_size"], [2304, 1296])
        self.assertEqual(camara["width"] * 9, camara["height"] * 16)
        self.assertAlmostEqual(camara["hfov_deg"], 68.16865, places=4)
        # El centro optico se midio en pixeles sobre 640 de ancho.  Lo que
        # describe a la lente es la FRACCION, no el pixel.
        self.assertAlmostEqual(
            camara["principal_x_px"] / camara["width"], 352.074 / 640.0, places=5
        )
        self.assertAlmostEqual(
            camara["principal_y_px"] / camara["height"], 0.5, places=5
        )

    def test_reescalar_la_camara_no_mueve_un_solo_milimetro_del_suelo(self):
        # La homografia mapea PIXEL a milimetros.  Si se sube la resolucion y
        # no se compone con la escala, cada pixel se proyecta a 1,2 veces su
        # distancia real y el mapa entero se estira sin que salte ninguna
        # validacion.  Este test es la red: el mismo punto del mundo, mirado a
        # dos resoluciones, tiene que dar el mismo milimetro.
        import importlib.util

        ruta = (
            Path(__file__).resolve().parents[2] / "herramientas" / "reescalar_camara.py"
        )
        spec = importlib.util.spec_from_file_location("reescalar_camara", ruta)
        modulo = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(modulo)

        base = json.loads(RUTA_CONFIG.read_text(encoding="utf-8"))
        ancho, alto = base["camera"]["width"], base["camera"]["height"]
        for factor in (0.5, 1.5):
            otra = modulo.reescalar(
                base, int(round(ancho * factor)), int(round(alto * factor))
            )
            uno = ProyectorSuelo.desde_config(base["camera"])
            dos = ProyectorSuelo.desde_config(otra["camera"])
            self.assertIsNotNone(uno)
            self.assertIsNotNone(dos)
            for u, v in ((ancho / 2, alto * 0.95), (ancho * 0.1, alto * 0.8),
                         (ancho * 0.9, alto * 0.7)):
                a = uno.punto_suelo(u, v)
                b = dos.punto_suelo(u * factor, v * factor)
                self.assertIsNotNone(a)
                self.assertIsNotNone(b)
                self.assertAlmostEqual(a[0], b[0], places=6)
                self.assertAlmostEqual(a[1], b[1], places=6)

    def test_una_escala_no_uniforme_se_rechaza(self):
        import importlib.util

        ruta = (
            Path(__file__).resolve().parents[2] / "herramientas" / "reescalar_camara.py"
        )
        spec = importlib.util.spec_from_file_location("reescalar_camara", ruta)
        modulo = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(modulo)

        base = json.loads(RUTA_CONFIG.read_text(encoding="utf-8"))
        with self.assertRaises(modulo.ErrorReescalado):
            modulo.reescalar(base, 1280, 864)


class PruebaConfig(unittest.TestCase):
    def test_la_configuracion_del_repo_es_valida(self):
        config = cargar_configuracion(str(RUTA_CONFIG))
        self.assertIn("track", config)

    def test_falta_un_bloque(self):
        config = json.loads(RUTA_CONFIG.read_text(encoding="utf-8"))
        del config["track"]
        with self.assertRaises(ErrorConfiguracion):
            validar_configuracion(config)

    def test_la_derecha_del_servo_tiene_que_ser_negativa(self):
        config = json.loads(RUTA_CONFIG.read_text(encoding="utf-8"))
        config["control"]["steering_max_right_deg"] = 20.0
        with self.assertRaises(ErrorConfiguracion):
            validar_configuracion(config)

    def test_un_carril_que_no_cabe_se_detecta_aqui(self):
        config = json.loads(RUTA_CONFIG.read_text(encoding="utf-8"))
        config["track"]["lane_width_mm"] = 200.0
        with self.assertRaises(ErrorConfiguracion):
            validar_configuracion(config)

    def test_homografia_lista_sin_datos_de_montaje(self):
        config = json.loads(RUTA_CONFIG.read_text(encoding="utf-8"))
        # Sin matriz explicita la homografia se reconstruye del montaje, y
        # entonces esas medidas tienen que ser creibles.  Con matriz, manda la
        # matriz y el montaje solo queda como documentacion.
        config["camera"]["ground_homography"]["matrix"] = None
        config["camera"]["ground_homography"]["ready"] = True
        config["camera"]["ground_homography"]["pitch_deg"] = 0.0
        with self.assertRaises(ErrorConfiguracion):
            validar_configuracion(config)

    def test_con_matriz_explicita_el_montaje_es_solo_documentacion(self):
        """La ronda usa la matriz ajustada, no la reconstruye del montaje.

        Reconstruirla abria la puerta a que la herramienta de calibracion y la
        ronda usaran opticas distintas: paso de verdad el 04-09, con 64 px de
        diferencia en el centro optico, que son casi cuatro grados de guiñada.
        """

        config = json.loads(RUTA_CONFIG.read_text(encoding="utf-8"))
        self.assertIsNotNone(config["camera"]["ground_homography"]["matrix"])
        config["camera"]["ground_homography"]["pitch_deg"] = 0.0
        validar_configuracion(config)

    def test_la_traccion_armada_no_desarma_la_puerta_de_calibracion(self):
        """``motion_enabled`` se armo a mano el 04-09; la puerta sigue cerrada.

        Antes esta prueba exigia ``motion_enabled == false`` de fabrica. Ya no
        sirve: Daniel armo la traccion para la primera corrida con motores. Lo
        que si tiene que seguir siendo cierto -- y es lo que de verdad protege
        al robot -- es que una calibracion pendiente siga impidiendo rodar
        aunque la traccion este armada.
        """

        config = cargar_configuracion(str(RUTA_CONFIG))
        self.assertTrue(config["runtime"]["motion_enabled"])
        self.assertIn("parking_ready", calibraciones_pendientes(config))
        with self.assertRaises(ErrorConfiguracion):
            exigir_listo_para_mover(config)

    def test_se_puede_omitir_una_calibracion_a_proposito(self):
        config = cargar_configuracion(str(RUTA_CONFIG))
        pendientes = calibraciones_pendientes(config, omitir=("parking_ready",))
        self.assertNotIn("parking_ready", pendientes)


if __name__ == "__main__":
    unittest.main()
