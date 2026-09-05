# Driver del RPLIDAR C1: conexion serial, protocolo binario y deteccion
# de barrido completo (wrap-around del angulo). No interpreta geometria
# -- entrega el barrido crudo (lista de (angulo, distancia_mm)) a quien
# lo pida; la interpretacion (paredes, clustering) vive en lidar_geometria.py.
#
# POR QUE SE LEE EN BLOQUES
# La version anterior hacia read(1) + read(4) por muestra. A 460800 bps el C1
# entrega ~9200 paquetes/s, o sea ~18400 llamadas de sistema por segundo solo
# para recibir. Mientras el consumidor procesaba un barrido (fusion, vision y
# control tardan 15-30 ms) el kernel seguia acumulando bytes en el buffer del
# puerto, y al volver a leer el driver decodificaba datos viejos: el control
# reaccionaba a una foto atrasada de la pista y el retraso crecia solo.
#
# Ahora cada vuelta del hilo drena con UNA lectura todo lo que hay pendiente y
# decodifica el lote completo de una vez (vectorizado con numpy si esta
# disponible). El instante de esa lectura viaja junto al barrido, de modo que
# quien controla puede medir la edad real del dato y descartar lo obsoleto.
import inspect
import time

import serial

try:
    import numpy as _np
except ImportError:      # el parser puro es igual de correcto, solo mas lento
    _np = None

PUERTO_LIDAR   = '/dev/ttyUSB0'
BAUDRATE_LIDAR = 460800

START_MOTOR_CMD = b'\xa5\xf0\x02\x94\x02\xc1\x02'
START_SCAN_CMD  = b'\xa5\x20'
STOP_CMD        = b'\xa5\x25'

TAM_PAQUETE        = 5          # modo de barrido estandar del C1
DISTANCIA_MAX_MM   = 6000.0     # mismo recorte que usaba la lectura byte a byte
SALTO_BARRIDO_DEG  = 300.0      # caida de angulo que delata el fin de un barrido
PAQUETES_SINCRONIA = 12         # paquetes seguidos coherentes para fiarse de una alineacion
TAM_BLOQUE_DEFECTO = 4096
_LIMITE_RESIDUO    = 1 << 16    # freno ante un flujo corrupto persistente


def _paquete_valido(datos, inicio):
    # En un paquete valido el bit de start y su inverso difieren, y el campo
    # de angulo trae su propio check bit en el bit 0 del segundo byte.
    byte0 = datos[inicio]
    if (byte0 & 0x01) == ((byte0 >> 1) & 0x01):
        return False
    return (datos[inicio + 1] & 0x01) == 1


def buscar_sincronia(datos, desde=0):
    # Primer desplazamiento donde encajan varios paquetes seguidos. Exigir una
    # racha (y no un solo paquete) evita engancharse a una coincidencia: los
    # dos check bits dejan pasar 1 de cada 4 posiciones al azar.
    necesarios = TAM_PAQUETE * PAQUETES_SINCRONIA
    inicio = desde
    while inicio + necesarios <= len(datos):
        if all(
            _paquete_valido(datos, inicio + TAM_PAQUETE * k)
            for k in range(PAQUETES_SINCRONIA)
        ):
            return inicio
        inicio += 1
    return None


def decodificar_lote(datos, inicio, cantidad):
    # Decodifica `cantidad` paquetes ya alineados en `inicio` y devuelve
    # (angulos, distancias, indice_incoherente). El indice vale -1 si el lote
    # entero encajo; si no, apunta al primer paquete que rompio la alineacion.
    #
    # El filtro de distancia es el mismo de la version byte a byte. Se anade el
    # tope de 360 grados: el campo trae 15 bits (hasta 511.98 grados) y un
    # paquete corrupto que superara los dos check bits terminaba aliasado
    # dentro de construir_perfil_360 (indice % 360), inventando un obstaculo
    # fantasma en un sector que nadie estaba mirando.
    if _np is not None:
        bloque = _np.frombuffer(
            datos, dtype=_np.uint8, offset=inicio, count=cantidad * TAM_PAQUETE
        ).reshape(cantidad, TAM_PAQUETE)
        byte0 = bloque[:, 0]
        coherentes = (((byte0 ^ (byte0 >> 1)) & 1) == 1) & ((bloque[:, 1] & 1) == 1)
        fallo = -1
        if not bool(coherentes.all()):
            fallo = int(_np.argmin(coherentes))
            bloque = bloque[:fallo]
        if bloque.shape[0] == 0:
            return _np.empty(0), _np.empty(0), fallo
        angulos = (
            (bloque[:, 2].astype(_np.uint16) << 7) | (bloque[:, 1] >> 1)
        ).astype(_np.float64) / 64.0
        distancias = (
            (bloque[:, 4].astype(_np.uint16) << 8) | bloque[:, 3]
        ).astype(_np.float64) / 4.0
        utiles = (
            (distancias > 0.0)
            & (distancias < DISTANCIA_MAX_MM)
            & (angulos < 360.0)
        )
        return angulos[utiles], distancias[utiles], fallo

    angulos = []
    distancias = []
    for k in range(cantidad):
        base = inicio + k * TAM_PAQUETE
        if not _paquete_valido(datos, base):
            return angulos, distancias, k
        angulo = ((datos[base + 2] << 7) | (datos[base + 1] >> 1)) / 64.0
        distancia = ((datos[base + 4] << 8) | datos[base + 3]) / 4.0
        if 0.0 < distancia < DISTANCIA_MAX_MM and angulo < 360.0:
            angulos.append(angulo)
            distancias.append(distancia)
    return angulos, distancias, -1


class LidarDriver:
    def __init__(self, puerto=PUERTO_LIDAR, baudrate=BAUDRATE_LIDAR,
                 tam_bloque=TAM_BLOQUE_DEFECTO):
        self._puerto        = puerto
        self._baudrate      = baudrate
        self._tam_bloque    = max(TAM_PAQUETE, int(tam_bloque))
        self._ser           = None
        self._residuo       = b""
        self._sincronizado  = False
        self._angulo_previo = 0.0
        self._parcial       = []
        # Diagnostico: paquetes que obligaron a resincronizar y barridos
        # entregados. Permite distinguir "el LiDAR calla" de "el flujo llega
        # corrupto" sin tener que instrumentar el driver en pista.
        self.resincronizaciones  = 0
        self.barridos_entregados = 0

    @staticmethod
    def _acepta_timestamp(al_barrido):
        # Los scripts historicos declaran al_barrido(scan). Los nuevos pueden
        # declarar al_barrido(scan, timestamp) y reciben ademas el instante de
        # captura, sin tener que tocar las herramientas de calibracion.
        try:
            firma = inspect.signature(al_barrido)
        except (TypeError, ValueError):
            return False
        posicionales = 0
        for parametro in firma.parameters.values():
            if parametro.kind == parametro.VAR_POSITIONAL:
                return True
            if parametro.kind in (
                parametro.POSITIONAL_ONLY, parametro.POSITIONAL_OR_KEYWORD
            ):
                posicionales += 1
        return posicionales >= 2

    def _abrir(self):
        self._ser = serial.Serial(self._puerto, baudrate=self._baudrate, timeout=1)
        time.sleep(0.5)
        self._ser.write(START_MOTOR_CMD)
        time.sleep(1.5)
        self._ser.reset_input_buffer()
        self._ser.write(START_SCAN_CMD)
        time.sleep(0.5)
        if self._ser.in_waiting >= 7:          # descartar cabecera de respuesta
            self._ser.read(7)

    def _leer_bloque(self):
        # read(1) con timeout hace de espera pasiva cuando el LiDAR calla: el
        # hilo no gira en vacio quemando CPU de la Pi. En cuanto llega el
        # primer byte se drena de un tiron todo lo que el kernel tenga listo.
        pendientes = self._ser.in_waiting
        if pendientes > 0:
            return self._ser.read(min(pendientes, self._tam_bloque))
        primero = self._ser.read(1)
        if not primero:
            return b""
        pendientes = self._ser.in_waiting
        if pendientes <= 0:
            return primero
        return primero + self._ser.read(min(pendientes, self._tam_bloque))

    def _cortes_de_barrido(self, angulos):
        # Indices donde el angulo retrocede: ahi termina un barrido completo.
        previo = self._angulo_previo
        if _np is not None and isinstance(angulos, _np.ndarray):
            anteriores = _np.empty_like(angulos)
            anteriores[0] = previo
            anteriores[1:] = angulos[:-1]
            saltos = (angulos < anteriores) & (
                (anteriores - angulos) > SALTO_BARRIDO_DEG
            )
            return _np.nonzero(saltos)[0].tolist()
        cortes = []
        for indice, angulo in enumerate(angulos):
            if angulo < previo and (previo - angulo) > SALTO_BARRIDO_DEG:
                cortes.append(indice)
            previo = angulo
        return cortes

    def _acumular(self, angulos, distancias):
        # Corta el lote en barridos completos y arrastra la cola incompleta al
        # siguiente bloque. Devuelve la lista de barridos cerrados.
        if len(angulos) == 0:
            return []
        cortes = self._cortes_de_barrido(angulos)
        lista_ang = angulos.tolist() if hasattr(angulos, "tolist") else angulos
        lista_dist = (
            distancias.tolist() if hasattr(distancias, "tolist") else distancias
        )
        barridos = []
        inicio = 0
        for corte in cortes:
            tramo = list(zip(lista_ang[inicio:corte], lista_dist[inicio:corte]))
            if self._parcial or tramo:
                barridos.append(self._parcial + tramo)
            self._parcial = []
            inicio = corte
        self._parcial = self._parcial + list(
            zip(lista_ang[inicio:], lista_dist[inicio:])
        )
        self._angulo_previo = float(lista_ang[-1])
        return barridos

    def _consumir(self, bloque):
        datos = self._residuo + bloque
        if len(datos) > _LIMITE_RESIDUO:
            # Solo se llega aqui con el flujo corrupto de forma persistente: se
            # conserva la cola reciente y se vuelve a buscar alineacion.
            datos = datos[-_LIMITE_RESIDUO:]
            self._sincronizado = False
        barridos = []
        indice = 0
        total = len(datos)
        while True:
            if not self._sincronizado:
                inicio = buscar_sincronia(datos, indice)
                if inicio is None:
                    # Lo anterior quedo descartado como ruido; se guarda solo
                    # la cola que todavia podria contener el patron.
                    indice = max(
                        indice, total - TAM_PAQUETE * PAQUETES_SINCRONIA + 1
                    )
                    break
                indice = inicio
                self._sincronizado = True
            cantidad = (total - indice) // TAM_PAQUETE
            if cantidad <= 0:
                break
            angulos, distancias, fallo = decodificar_lote(datos, indice, cantidad)
            barridos.extend(self._acumular(angulos, distancias))
            if fallo < 0:
                indice += cantidad * TAM_PAQUETE
                break
            self.resincronizaciones += 1
            self._sincronizado = False
            indice += fallo * TAM_PAQUETE + 1
        self._residuo = datos[indice:]
        return barridos

    def hilo_lectura(self, obtener_corriendo, al_barrido):
        # al_barrido(scan) se llama una vez por barrido completo, con
        # scan = lista de (angulo_deg, distancia_mm). Si el destinatario
        # declara un segundo parametro recibe tambien el instante de captura
        # (time.monotonic() de la lectura que cerro el barrido).
        con_timestamp = self._acepta_timestamp(al_barrido)
        try:
            self._abrir()
            print("[+] Telemetria LiDAR activa.")

            while obtener_corriendo():
                bloque = self._leer_bloque()
                if not bloque:
                    continue
                instante = time.monotonic()
                for barrido in self._consumir(bloque):
                    self.barridos_entregados += 1
                    if con_timestamp:
                        al_barrido(barrido, instante)
                    else:
                        al_barrido(barrido)

        except Exception as e:
            if obtener_corriendo():
                print(f"[-] Falla en hilo LiDAR: {e}")

    def cerrar(self):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(STOP_CMD)
                self._ser.close()
            except Exception:
                pass
