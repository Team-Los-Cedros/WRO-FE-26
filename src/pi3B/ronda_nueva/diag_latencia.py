# -*- coding: utf-8 -*-
"""Mide latencia de LiDAR y vision con la CPU cargada como en carrera.

Los numeros de la bitacora -vision a 67-72 ms, barrido a 16,0 ms de edad
media, 10,2 Hz- son medidas de la Raspberry 3B. Al pasar a la Pi 5 dejan de
valer todos: describen la MAQUINA, no el robot. Esto los vuelve a tomar.

Monta la misma tuberia que la ronda -hilo de LiDAR publicando en el buzon de
un hueco, hilo de camara procesando vision, bucle de decision consumiendo-
pero NO instancia EnlacePico: el robot no se mueve.

Uso (en la Pi, dentro del despliegue):
    python3 -m ronda_nueva.diag_latencia --segundos 20
    python3 -m ronda_nueva.diag_latencia --ancho 1280 --alto 720
"""
import argparse
import copy
import threading
import time

from .config import cargar_configuracion
from .hardware import FuenteCamara
from .percepcion_lidar import PercepcionLidar
from .servidor_web import EstadoRobot, ServidorPanel
from .ronda_nueva import _drivers_comunes
from .sincronizacion import BuzonBarridosLidar, BuzonVision
from .vision_ligera import VisionLigera


def _percentil(valores, fraccion):
    if not valores:
        return float("nan")
    ordenados = sorted(valores)
    indice = min(len(ordenados) - 1, int(fraccion * len(ordenados)))
    return ordenados[indice]


def _resumen(nombre, valores, unidad="ms"):
    if not valores:
        print("  %-22s sin muestras" % nombre)
        return
    print("  %-22s media %6.1f | p95 %6.1f | max %6.1f | n %d  (%s)"
          % (nombre, sum(valores) / len(valores), _percentil(valores, 0.95),
             max(valores), len(valores), unidad))


def main():
    analizador = argparse.ArgumentParser(description=__doc__)
    analizador.add_argument("--config", default=None)
    analizador.add_argument("--segundos", type=float, default=20.0)
    analizador.add_argument("--ancho", type=int, default=None,
                            help="ancho de captura; por defecto el de la config")
    analizador.add_argument("--alto", type=int, default=None)
    analizador.add_argument("--sin-camara", action="store_true",
                            help="mide el LiDAR sin la carga de vision")
    analizador.add_argument("--panel-web", nargs="?", type=int, const=8080,
                            default=None, metavar="PUERTO",
                            help="sirve el panel en vivo mientras mide; el "
                                 "robot sigue sin moverse")
    args = analizador.parse_args()

    config = cargar_configuracion(args.config) if args.config else cargar_configuracion()
    config = copy.deepcopy(config)

    if args.ancho and args.alto:
        base_ancho = float(config["camera"]["width"])
        config["camera"]["width"] = args.ancho
        config["camera"]["height"] = args.alto
        # El HFOV es optica y no cambia; el centro optico esta en pixeles y si.
        config["camera"]["principal_x_px"] = (
            float(config["camera"]["principal_x_px"]) * args.ancho / base_ancho
        )

    hardware = config["hardware"]
    ancho = config["camera"]["width"]
    alto = config["camera"]["height"]
    print("[*] captura %dx%d sobre modo de sensor %s | vision %s"
          % (ancho, alto, config["camera"].get("raw_sensor_size"),
             "DESACTIVADA" if args.sin_camara else "activa"))
    print("[*] midiendo %.0f s; el robot no se mueve" % args.segundos)
    print()

    estado_panel = None
    panel = None
    if args.panel_web:
        estado_panel = EstadoRobot()
        panel = ServidorPanel(estado_panel, args.panel_web)
        if panel.arrancar():
            print("[i] panel web en http://<ip-de-la-pi>:%d/  (el robot NO se mueve)"
                  % panel.puerto)
        else:
            print("[!] no se pudo abrir el panel: %s" % panel.error)
            panel = None

    LidarDriver, ProcesadorLidar = _drivers_comunes()
    buzon_lidar = BuzonBarridosLidar()
    buzon_vision = BuzonVision(4)
    seguir = threading.Event()
    seguir.set()

    ms_vision = []

    camara = None
    hilo_camara = None
    if not args.sin_camara:
        vision = VisionLigera(config)
        camara = FuenteCamara(config["camera"])

        def al_frame(frame, timestamp):
            inicio = time.monotonic()
            buzon_vision.publicar(vision.procesar(frame, timestamp))
            ms_vision.append((time.monotonic() - inicio) * 1000.0)
            if estado_panel is not None:
                estado_panel.publicar_frame(frame)

        hilo_camara = threading.Thread(
            target=camara.bucle,
            args=(seguir.is_set, al_frame),
            name="camara-diag",
            daemon=True,
        )
        hilo_camara.start()

    driver = LidarDriver(hardware["lidar_port"], int(hardware["lidar_baudrate"]))

    def al_barrido(scan):
        buzon_lidar.publicar(scan, time.monotonic())

    hilo_lidar = threading.Thread(
        target=driver.hilo_lectura,
        args=(seguir.is_set, al_barrido),
        name="lidar-diag",
        daemon=True,
    )
    hilo_lidar.start()

    geo = ProcesadorLidar()
    percepcion = PercepcionLidar(config)

    edades = []
    periodos = []
    ms_ciclo = []
    anterior_ts = None
    barridos = 0
    limite = time.monotonic() + args.segundos

    while time.monotonic() < limite:
        barrido = buzon_lidar.tomar(timeout_s=0.5)
        if barrido is None:
            continue
        ahora = time.monotonic()
        # La edad es lo que separa el instante en que el LiDAR vio la pista del
        # instante en que el control lo va a usar. Es la magnitud que hacia
        # esquivar donde el pilar estaba, no donde esta.
        edades.append((ahora - barrido.timestamp) * 1000.0)
        if anterior_ts is not None:
            periodos.append((barrido.timestamp - anterior_ts) * 1000.0)
        anterior_ts = barrido.timestamp

        inicio = time.monotonic()
        medicion = geo.procesar(barrido.muestras)
        percepcion.procesar(
            barrido.muestras, medicion, timestamp=barrido.timestamp, lado_parqueo=0
        )
        buzon_vision.mas_cercano(barrido.timestamp, 0.25)
        ms_ciclo.append((time.monotonic() - inicio) * 1000.0)
        barridos += 1

        if estado_panel is not None:
            corredor = medicion
            estado_panel.publicar_barrido(barrido.muestras)
            estado_panel.publicar(
                estado="DIAGNOSTICO",
                razon="midiendo latencia; el robot no se mueve",
                t=barridos * 0.1,
                frontal=round(float(getattr(corredor, "frontal", 0.0) or 0.0), 1),
                izquierda=round(float(getattr(corredor, "izquierda", 0.0) or 0.0), 1),
                derecha=round(float(getattr(corredor, "derecha", 0.0) or 0.0), 1),
                trasera=round(float(getattr(corredor, "trasera", 0.0) or 0.0), 1),
                lidar_edad_ms=round(edades[-1], 2),
                vision_edad_ms=round(ms_vision[-1], 1) if ms_vision else None,
                barridos_descartados=buzon_lidar.descartados,
            )

    # FuenteCamara no expone cierre: su bucle suelta la camara al ver que
    # `seguir` ya no esta puesto.
    seguir.clear()
    if hilo_camara is not None:
        hilo_camara.join(timeout=3.0)
    hilo_lidar.join(timeout=2.0)
    if panel is not None:
        panel.detener()

    duracion = args.segundos
    print("== LiDAR ==")
    print("  barridos consumidos: %d en %.1f s -> %.1f Hz"
          % (barridos, duracion, barridos / duracion if duracion else 0.0))
    print("  recibidos %d | descartados %d (%.1f%%)"
          % (buzon_lidar.recibidos, buzon_lidar.descartados,
             100.0 * buzon_lidar.descartados / max(1, buzon_lidar.recibidos)))
    _resumen("edad del barrido", edades)
    _resumen("periodo entre barridos", periodos)
    _resumen("proceso por barrido", ms_ciclo)
    print()
    print("== VISION ==")
    if args.sin_camara:
        print("  desactivada")
    else:
        _resumen("proceso por frame", ms_vision)
        print("  frames: %d en %.1f s -> %.1f fps"
              % (len(ms_vision), duracion,
                 len(ms_vision) / duracion if duracion else 0.0))
        if camara is not None and camara.ultima_advertencia:
            print("  aviso: %s" % camara.ultima_advertencia)
        if camara is not None and camara.ultimo_error:
            print("  ERROR: %s" % camara.ultimo_error)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
