"""Pruebas offline de la Ronda Abierta portada a la Pi 5.

Lo que se fija aqui son las tres cosas que cambiaron al portar: el conteo de
vueltas sin sensor de color, el enmascarado de la propia estructura y el
parser de la trama de la Pico.
"""

import math
import unittest

from src.pi5.ronda_abierta.ronda_abierta import (
    SECTORES_CIEGOS,
    ContadorEsquinas,
    ContadorLineas,
    EnlacePico,
    _sectores,
    angulo_centrado,
    coincide_firma,
    en_sector,
    esta_ciego,
    minimos_laterales,
)


def barrido(*muestras):
    return list(muestras)


class SectoresTests(unittest.TestCase):
    def test_sector_normal_y_sector_que_cruza_el_cero(self):
        self.assertTrue(en_sector(60.0, (30.0, 90.0)))
        self.assertFalse(en_sector(120.0, (30.0, 90.0)))
        # El sector frontal de este robot cruza el 0.
        self.assertTrue(en_sector(350.0, (330.0, 30.0)))
        self.assertTrue(en_sector(10.0, (330.0, 30.0)))
        self.assertFalse(en_sector(180.0, (330.0, 30.0)))

    def test_los_arcos_medidos_del_robot_se_reconocen_como_ciegos(self):
        # Mastil (141..212) y su soporte (28..54), medidos con diag_mastil.
        for angulo in (30.0, 45.0, 54.0, 150.0, 195.0, 212.0):
            self.assertTrue(esta_ciego(angulo, SECTORES_CIEGOS), angulo)
        for angulo in (60.0, 90.0, 120.0, 250.0, 300.0):
            self.assertFalse(esta_ciego(angulo, SECTORES_CIEGOS), angulo)


class MinimosLateralesTests(unittest.TestCase):
    def test_toma_el_minimo_de_cada_lado(self):
        izq, der = minimos_laterales(
            barrido((70.0, 500.0), (80.0, 430.0), (300.0, 610.0), (320.0, 590.0))
        )
        self.assertEqual(der, 430.0)
        self.assertEqual(izq, 590.0)

    def test_la_estructura_propia_no_contamina_el_lado_derecho(self):
        # Esta es LA razon del port: el soporte devuelve 150 mm en el grado 45,
        # que cae dentro del sector derecho (30-90). Sin enmascararlo, la
        # derecha se queda clavada ahi y el centrado deja de funcionar.
        muestras = barrido((45.0, 150.0), (70.0, 480.0), (300.0, 520.0))
        _izq, der_sin_mascara = minimos_laterales(muestras, ciegos=())
        self.assertEqual(der_sin_mascara, 150.0)

        _izq, der = minimos_laterales(muestras)
        self.assertEqual(der, 480.0)

    def test_el_mastil_tampoco_entra_por_el_lado_izquierdo(self):
        # 195 grados esta dentro del mastil; no pertenece a ningun sector
        # lateral, pero se comprueba que ni siquiera se considera.
        izq, der = minimos_laterales(barrido((195.0, 48.0), (300.0, 700.0)))
        self.assertEqual(izq, 700.0)
        self.assertFalse(math.isfinite(der))

    def test_un_lado_sin_ecos_devuelve_infinito_no_cero(self):
        izq, der = minimos_laterales(barrido((70.0, 400.0)))
        self.assertEqual(der, 400.0)
        self.assertTrue(math.isinf(izq))

    def test_descarta_distancias_imposibles(self):
        izq, der = minimos_laterales(
            barrido((70.0, 0.0), (75.0, 9000.0), (80.0, 450.0), (300.0, 500.0))
        )
        self.assertEqual(der, 450.0)
        self.assertEqual(izq, 500.0)


class ContadorEsquinasTests(unittest.TestCase):
    def test_cuenta_una_esquina_por_cada_90_grados(self):
        c = ContadorEsquinas()
        c.fijar_referencia(0.0)
        self.assertEqual(c.actualizar(0.0), 0)
        self.assertEqual(c.actualizar(80.0), 0)      # aun no llega
        self.assertEqual(c.actualizar(103.0), 1)     # ya paso los 90
        self.assertEqual(c.actualizar(195.0), 2)
        self.assertEqual(c.actualizar(1085.0), 12)   # tres vueltas

    def test_funciona_igual_en_sentido_horario(self):
        c = ContadorEsquinas()
        c.fijar_referencia(0.0)
        self.assertEqual(c.actualizar(-103.0), 1)
        self.assertEqual(c.actualizar(-1085.0), 12)
        self.assertEqual(c.sentido, -1)

    def test_la_referencia_no_tiene_por_que_ser_cero(self):
        # El rumbo de la Pico viene acumulado de arranques anteriores.
        c = ContadorEsquinas()
        c.fijar_referencia(-306.3)
        self.assertEqual(c.actualizar(-306.3), 0)
        self.assertEqual(c.actualizar(-409.3), 1)

    def test_el_bamboleo_del_centrado_no_inventa_esquinas(self):
        c = ContadorEsquinas()
        c.fijar_referencia(0.0)
        for rumbo in (5.0, -6.0, 8.0, -4.0, 7.0, 0.0):
            self.assertEqual(c.actualizar(rumbo), 0)

    def test_no_descuenta_si_el_robot_deshace_parte_del_giro(self):
        # Una maniobra de rescate puede devolver rumbo; la esquina ya se hizo.
        c = ContadorEsquinas()
        c.fijar_referencia(0.0)
        self.assertEqual(c.actualizar(200.0), 2)
        self.assertEqual(c.actualizar(140.0), 2)
        self.assertEqual(c.actualizar(95.0), 2)

    def test_sentido_desconocido_hasta_que_hay_giro_suficiente(self):
        c = ContadorEsquinas()
        c.fijar_referencia(0.0)
        c.actualizar(10.0)
        self.assertEqual(c.sentido, 0)
        c.actualizar(120.0)
        self.assertEqual(c.sentido, 1)


class ContadorLineasTests(unittest.TestCase):
    def test_cuenta_una_vez_por_cruce_con_refractario(self):
        c = ContadorLineas()
        self.assertEqual(c.actualizar("NARANJA", 10.0), 1)
        self.assertEqual(c.actualizar("NARANJA", 10.1), 1)   # misma linea
        self.assertEqual(c.actualizar("PISTA", 10.5), 1)
        self.assertEqual(c.actualizar("PISTA", 11.0), 1)     # ya fuera
        self.assertEqual(c.actualizar("NARANJA", 12.0), 2)

    def test_el_azul_no_cuenta(self):
        c = ContadorLineas()
        self.assertEqual(c.actualizar("AZUL", 10.0), 0)
        self.assertEqual(c.actualizar("AZUL", 11.0), 0)


class CentradoTests(unittest.TestCase):
    def test_gira_hacia_el_lado_con_mas_hueco(self):
        # Mas espacio a la izquierda -> angulo positivo (izquierda).
        self.assertGreater(angulo_centrado(700.0, 300.0), 0.0)
        self.assertLess(angulo_centrado(300.0, 700.0), 0.0)
        self.assertEqual(angulo_centrado(500.0, 500.0), 0.0)

    def test_no_pide_mas_angulo_del_que_el_servo_puede_dar(self):
        self.assertLessEqual(abs(angulo_centrado(4000.0, 100.0)), 25.0)
        self.assertLessEqual(abs(angulo_centrado(100.0, 4000.0)), 25.0)


class FirmaTests(unittest.TestCase):
    def test_reconoce_el_corredor_de_la_salida(self):
        self.assertTrue(coincide_firma(520.0, 470.0, 500.0, 500.0))
        self.assertFalse(coincide_firma(900.0, 470.0, 500.0, 500.0))


class ParserPicoTests(unittest.TestCase):
    def test_lee_la_trama_actual_con_ultrasonido(self):
        t = EnlacePico.parsear("IMU:-306.30,COLOR:SIN_SENSOR,US:1688,WD:STOP")
        self.assertAlmostEqual(t["rumbo"], -306.30)
        self.assertEqual(t["color"], "SIN_SENSOR")
        self.assertEqual(t["ultrasonido"], 1688.0)
        self.assertEqual(t["watchdog"], "STOP")

    def test_lee_la_trama_historica_sin_ultrasonido(self):
        t = EnlacePico.parsear("IMU:12.5,COLOR:NARANJA")
        self.assertAlmostEqual(t["rumbo"], 12.5)
        self.assertEqual(t["color"], "NARANJA")
        self.assertIsNone(t["ultrasonido"])

    def test_el_orden_de_los_campos_no_es_contrato(self):
        t = EnlacePico.parsear("IMU:1.0,WD:OK,US:200,COLOR:PISTA")
        self.assertEqual(t["color"], "PISTA")
        self.assertEqual(t["ultrasonido"], 200.0)
        self.assertEqual(t["watchdog"], "OK")

    def test_sin_eco_el_ultrasonido_es_none_no_cero(self):
        t = EnlacePico.parsear("IMU:1.0,COLOR:PISTA,US:-1,WD:OK")
        self.assertIsNone(t["ultrasonido"])

    def test_rechaza_lo_que_no_es_trama(self):
        for linea in ("", "hola", "COLOR:PISTA", "IMU:texto"):
            self.assertIsNone(EnlacePico.parsear(linea), linea)


class ArgumentosTests(unittest.TestCase):
    def test_parsea_los_sectores_ciegos_de_la_linea_de_comandos(self):
        self.assertEqual(_sectores("27-55,140-213"), ((27.0, 55.0), (140.0, 213.0)))
        self.assertEqual(_sectores(""), ())


if __name__ == "__main__":
    unittest.main()
