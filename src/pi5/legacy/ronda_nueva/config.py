"""Carga y validacion de la configuracion.

DOS COSAS QUE HACE ESTE MODULO Y QUE NO SON OBVIAS

1. **Falla en el escritorio, no en la pista.**  Una clave que falta o un rango
   HSV mal escrito tienen que saltar al validar, no cinco segundos despues de
   pulsar el boton con el robot ya rodando.
2. **Guarda las calibraciones pendientes.**  ``motion_enabled`` no basta: hay
   partes del sistema (la homografia del suelo, los HSV, las medidas del
   chasis, el parqueo) cuyo estado de calibracion es independiente, y arrancar
   con una sin hacer produce un fallo que parece de software y no lo es.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


class ErrorConfiguracion(ValueError):
    """La configuracion no describe un robot con el que se pueda rodar."""


CALIBRACIONES = {
    "camera_ground_ready": (
        "homografia del suelo -- herramientas/calibrar_suelo.py"
    ),
    "hsv_ready": "umbrales HSV de rojo, verde, magenta y lineas",
    "steering_ready": "limites y radios de giro del servo",
    "chassis_measured": "medidas del chasis comprobadas con regla",
    "parking_ready": "maniobra de parqueo probada en la bahia",
}


def _exigir(config: Dict[str, Any], ruta: Iterable[str]) -> Any:
    actual: Any = config
    recorrido: List[str] = []
    for clave in ruta:
        recorrido.append(str(clave))
        if not isinstance(actual, dict) or clave not in actual:
            raise ErrorConfiguracion(f"Falta la clave {'.'.join(recorrido)}")
        actual = actual[clave]
    return actual


def _positivo(valor: Any, nombre: str) -> float:
    try:
        numero = float(valor)
    except (TypeError, ValueError):
        raise ErrorConfiguracion(f"{nombre} debe ser numerico") from None
    if not numero > 0.0:
        raise ErrorConfiguracion(f"{nombre} debe ser positivo (es {numero})")
    return numero


def _validar_rangos_hsv(rangos: Any, nombre: str) -> None:
    if not isinstance(rangos, list) or not rangos:
        raise ErrorConfiguracion(f"{nombre} debe ser una lista no vacia")
    for indice, par in enumerate(rangos):
        if not isinstance(par, list) or len(par) != 2:
            raise ErrorConfiguracion(f"{nombre}[{indice}] debe ser [bajo, alto]")
        for extremo in par:
            if not isinstance(extremo, list) or len(extremo) != 3:
                raise ErrorConfiguracion(
                    f"{nombre}[{indice}] debe tener tripletas H,S,V"
                )
            if not all(0 <= int(v) <= 255 for v in extremo):
                raise ErrorConfiguracion(f"{nombre}[{indice}] fuera de 0..255")
        if int(par[0][0]) > int(par[1][0]) and nombre != "red_ranges":
            raise ErrorConfiguracion(
                f"{nombre}[{indice}] tiene el matiz invertido; para el rojo se "
                "usan dos rangos, no uno que cruce el 179"
            )


def validar_configuracion(config: Dict[str, Any]) -> None:
    """Levanta ``ErrorConfiguracion`` con un mensaje accionable, o no hace nada."""

    for bloque in ("camera", "vision", "track", "chassis", "lidar", "control", "parking", "hardware", "runtime"):
        _exigir(config, (bloque,))

    camara = config["camera"]
    for clave in ("width", "height", "fps", "hfov_deg"):
        _positivo(_exigir(camara, (clave,)), f"camera.{clave}")
    if not 1.0 < float(camara["hfov_deg"]) < 179.0:
        raise ErrorConfiguracion("camera.hfov_deg fuera de rango")

    suelo = camara.get("ground_homography")
    if not isinstance(suelo, dict):
        raise ErrorConfiguracion("Falta el bloque camera.ground_homography")
    if suelo.get("ready"):
        if suelo.get("matrix") is None:
            # Sin matriz se construye desde el montaje: entonces esas medidas
            # tienen que existir y ser creibles.
            _positivo(suelo.get("height_mm"), "ground_homography.height_mm")
            cabeceo = float(suelo.get("pitch_deg", 0.0))
            if not 0.0 < cabeceo < 89.0:
                raise ErrorConfiguracion(
                    "ground_homography.pitch_deg debe mirar al suelo (0 < c < 89)"
                )
        else:
            matriz = suelo["matrix"]
            plana = [v for fila in matriz for v in fila] if matriz and isinstance(matriz[0], list) else matriz
            if len(plana) != 9:
                raise ErrorConfiguracion("ground_homography.matrix debe tener 9 valores")

    vision = config["vision"]
    for nombre in ("red_ranges", "green_ranges", "floor_ranges"):
        _validar_rangos_hsv(_exigir(vision, (nombre,)), nombre)

    pista = config["track"]
    _positivo(pista.get("lane_width_mm"), "track.lane_width_mm")
    _positivo(pista.get("segment_length_mm"), "track.segment_length_mm")
    posiciones = pista.get("pillar_positions_mm")
    if not isinstance(posiciones, list) or len(posiciones) < 1:
        raise ErrorConfiguracion("track.pillar_positions_mm debe ser una lista")

    chasis = config["chassis"]
    for clave in ("wheelbase_mm", "width_mm", "length_mm", "turn_radius_left_mm", "turn_radius_right_mm"):
        _positivo(chasis.get(clave), f"chassis.{clave}")

    control = config["control"]
    sentido = str(control.get("turn_direction", "AUTO")).upper()
    if sentido not in ("AUTO", "LEFT", "RIGHT"):
        raise ErrorConfiguracion("control.turn_direction debe ser AUTO, LEFT o RIGHT")
    if int(control.get("speed_cruise_pwm", 0)) <= 0:
        raise ErrorConfiguracion("control.speed_cruise_pwm debe ser positivo")
    if float(control.get("steering_max_left_deg", 0)) <= 0:
        raise ErrorConfiguracion("control.steering_max_left_deg debe ser positivo")
    if float(control.get("steering_max_right_deg", 0)) >= 0:
        raise ErrorConfiguracion(
            "control.steering_max_right_deg debe ser negativo (la derecha resta)"
        )

    # El carril tiene que dar para el robot mas dos margenes; si no, cualquier
    # plan sale imposible y conviene enterarse aqui.
    ancho_util = float(pista["lane_width_mm"]) - float(chasis["width_mm"])
    if ancho_util <= 2.0 * float(control.get("wall_clearance_mm", 55.0)):
        raise ErrorConfiguracion(
            "El carril no deja sitio para el robot con los margenes pedidos"
        )

    hardware = config["hardware"]
    for clave in ("pico_port", "lidar_port"):
        if not str(hardware.get(clave, "")).strip():
            raise ErrorConfiguracion(f"hardware.{clave} vacio")


def cargar_configuracion(ruta: Optional[str] = None) -> Dict[str, Any]:
    destino = Path(ruta) if ruta else Path(__file__).with_name("configuracion.json")
    if not destino.is_file():
        raise ErrorConfiguracion(f"No existe la configuracion: {destino}")
    with destino.open("r", encoding="utf-8") as archivo:
        config = json.load(archivo)
    validar_configuracion(config)
    return config


def calibraciones_pendientes(config: Dict[str, Any], omitir: Iterable[str] = ()) -> List[str]:
    """Nombres de las calibraciones que siguen sin aprobar."""

    bloque = config.get("calibration", {})
    saltar = set(omitir)
    return [
        nombre
        for nombre in CALIBRACIONES
        if nombre not in saltar and not bool(bloque.get(nombre))
    ]


def exigir_listo_para_mover(config: Dict[str, Any], omitir: Iterable[str] = ()) -> None:
    """Puerta unica antes de armar la traccion.

    Se comprueba aqui y no en el arranque para que ninguna herramienta pueda
    saltarsela por descuido: mover el robot con la homografia sin calibrar
    significa esquivar donde el pilar NO esta.
    """

    if not bool(config.get("runtime", {}).get("motion_enabled")):
        raise ErrorConfiguracion(
            "runtime.motion_enabled es false: la traccion esta desarmada a proposito"
        )
    pendientes = calibraciones_pendientes(config, omitir)
    if pendientes:
        detalle = "; ".join(f"{nombre} ({CALIBRACIONES[nombre]})" for nombre in pendientes)
        raise ErrorConfiguracion(f"Calibraciones pendientes: {detalle}")
