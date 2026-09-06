"""Carga y validacion de la configuracion unica de ``ronda_nueva``."""

import json
import math
import os
from pathlib import Path
from typing import Any, Dict, Iterable, Optional


RUTA_CONFIG_DEFECTO = Path(__file__).with_name("configuracion.json")


class ErrorConfiguracion(ValueError):
    pass


def _exigir(config: Dict[str, Any], ruta: Iterable[str]) -> Any:
    actual: Any = config
    partes = list(ruta)
    for parte in partes:
        if not isinstance(actual, dict) or parte not in actual:
            raise ErrorConfiguracion("Falta el parametro: " + ".".join(partes))
        actual = actual[parte]
    return actual


def validar_configuracion(config: Dict[str, Any]) -> None:
    if config.get("schema_version") != 1:
        raise ErrorConfiguracion("schema_version debe ser 1")

    ancho = int(_exigir(config, ("camera", "width")))
    alto = int(_exigir(config, ("camera", "height")))
    fps = float(_exigir(config, ("camera", "fps")))
    if ancho < 160 or alto < 120 or fps <= 0:
        raise ErrorConfiguracion("Resolucion/FPS de camara invalidos")

    top = float(_exigir(config, ("camera", "roi_top_ratio")))
    bottom = float(_exigir(config, ("camera", "roi_bottom_ratio")))
    if not 0.0 <= top < bottom <= 1.0:
        raise ErrorConfiguracion("ROI de camara invalida")

    rotacion = int(_exigir(config, ("camera", "rotation_deg")))
    if rotacion not in (0, 90, 180, 270):
        raise ErrorConfiguracion("camera.rotation_deg debe ser 0, 90, 180 o 270")

    c0 = float(_exigir(config, ("camera", "principal_x_px")))
    hfov = float(_exigir(config, ("camera", "hfov_deg")))
    if not 0 <= c0 <= ancho or not 20.0 <= hfov <= 170.0:
        raise ErrorConfiguracion("Centro optico/HFOV invalidos")

    formato_picamera = str(
        _exigir(config, ("camera", "picamera_format"))
    ).upper()
    orden_array = str(
        _exigir(config, ("camera", "array_color_order"))
    ).upper()
    if formato_picamera not in ("RGB888", "BGR888"):
        raise ErrorConfiguracion("camera.picamera_format no soportado")
    if orden_array not in ("RGB", "BGR"):
        raise ErrorConfiguracion("camera.array_color_order debe ser RGB o BGR")

    modo_raw = _exigir(config, ("camera", "raw_sensor_size"))
    if (
        not isinstance(modo_raw, list)
        or len(modo_raw) != 2
        or any(isinstance(valor, bool) for valor in modo_raw)
        or any(int(valor) < 1 for valor in modo_raw)
    ):
        raise ErrorConfiguracion(
            "camera.raw_sensor_size debe ser [ancho, alto] positivo"
        )

    for nombre in (
        "yaw_from_lidar_deg",
        "forward_from_lidar_mm",
        "right_from_lidar_mm",
        "latency_s",
    ):
        if not math.isfinite(float(_exigir(config, ("camera", nombre)))):
            raise ErrorConfiguracion("Parametro de camara no finito: " + nombre)

    sectores_ciegos = _exigir(config, ("lidar", "blind_sectors_deg"))
    if not isinstance(sectores_ciegos, list) or not sectores_ciegos:
        raise ErrorConfiguracion("lidar.blind_sectors_deg debe contener sectores")
    for sector in sectores_ciegos:
        if not isinstance(sector, list) or len(sector) != 2:
            raise ErrorConfiguracion("Cada sector ciego debe tener inicio y fin")
        if not all(math.isfinite(float(valor)) for valor in sector):
            raise ErrorConfiguracion("Sector ciego no finito")

    hombros = _exigir(config, ("lidar", "rear_shoulder_offset_deg"))
    if len(hombros) != 2 or not 0.0 <= float(hombros[0]) < float(hombros[1]) < 90.0:
        raise ErrorConfiguracion("Ventana de hombros traseros invalida")
    if int(_exigir(config, ("lidar", "rear_min_valid_points"))) < 1:
        raise ErrorConfiguracion("rear_min_valid_points debe ser positivo")

    max_izq = float(_exigir(config, ("control", "steering_max_left_deg")))
    max_der = float(_exigir(config, ("control", "steering_max_right_deg")))
    if max_izq <= 0 or max_der >= 0:
        raise ErrorConfiguracion("Los limites deben ser izquierda>0 y derecha<0")

    if int(_exigir(config, ("control", "corners_before_parking"))) < 1:
        raise ErrorConfiguracion("corners_before_parking invalido")

    exigir_watchdog_pico = _exigir(
        config, ("hardware", "require_pico_command_watchdog")
    )
    if not isinstance(exigir_watchdog_pico, bool):
        raise ErrorConfiguracion(
            "hardware.require_pico_command_watchdog debe ser booleano"
        )

    direccion = str(_exigir(config, ("control", "turn_direction"))).upper()
    if direccion not in ("AUTO", "LEFT", "RIGHT"):
        raise ErrorConfiguracion("control.turn_direction debe ser AUTO, LEFT o RIGHT")

    lado_izq = int(_exigir(config, ("parking", "parking_side_for_left_turn")))
    lado_der = int(_exigir(config, ("parking", "parking_side_for_right_turn")))
    if lado_izq not in (-1, 1) or lado_der not in (-1, 1) or lado_izq == lado_der:
        raise ErrorConfiguracion("Los lados de parqueo deben ser -1/+1 y opuestos")

    largo_hueco = float(_exigir(config, ("parking", "bay_length_mm")))
    largo_robot = float(_exigir(config, ("parking", "robot_length_mm")))
    profundidad = float(_exigir(config, ("parking", "bay_depth_mm")))
    ancho_robot = float(_exigir(config, ("parking", "robot_width_mm")))
    if largo_hueco <= largo_robot or profundidad <= ancho_robot:
        raise ErrorConfiguracion("El robot no cabe en la geometria de parqueo configurada")

    cobertura_trasera = float(
        _exigir(config, ("parking", "minimum_rear_coverage"))
    )
    cobertura_diagonal = float(
        _exigir(config, ("parking", "minimum_rear_diagonal_coverage"))
    )
    if not 0.0 < cobertura_trasera <= 1.0:
        raise ErrorConfiguracion("minimum_rear_coverage debe estar en (0, 1]")
    if not 0.0 < cobertura_diagonal <= 1.0:
        raise ErrorConfiguracion(
            "minimum_rear_diagonal_coverage debe estar en (0, 1]"
        )

    despeje_diagonal = float(
        _exigir(config, ("parking", "minimum_rear_diagonal_clearance_mm"))
    )
    despeje_lateral = float(
        _exigir(config, ("parking", "minimum_lateral_clearance_mm"))
    )
    sin_dato = float(_exigir(config, ("lidar", "rear_no_data_mm")))
    if not 0.0 < despeje_diagonal < sin_dato:
        raise ErrorConfiguracion(
            "minimum_rear_diagonal_clearance_mm fuera del rango LiDAR"
        )
    if not ancho_robot / 2.0 < despeje_lateral < sin_dato:
        raise ErrorConfiguracion(
            "minimum_lateral_clearance_mm no protege medio ancho del robot"
        )


def cargar_configuracion(ruta: Optional[str] = None) -> Dict[str, Any]:
    elegida = ruta or os.environ.get("WRO_NUEVA_CONFIG") or str(RUTA_CONFIG_DEFECTO)
    with open(elegida, "r", encoding="utf-8") as archivo:
        config = json.load(archivo)
    validar_configuracion(config)
    config["_ruta"] = str(Path(elegida).resolve())
    return config


def calibraciones_pendientes(
    config: Dict[str, Any], incluir_estacionamiento: bool = True
) -> list:
    calibracion = config.get("calibration", {})
    return sorted(
        nombre
        for nombre, listo in calibracion.items()
        if not bool(listo)
        and (incluir_estacionamiento or nombre != "parking_ready")
    )


def exigir_listo_para_mover(
    config: Dict[str, Any], incluir_estacionamiento: bool = True
) -> None:
    pendientes = calibraciones_pendientes(config, incluir_estacionamiento)
    if pendientes:
        raise ErrorConfiguracion(
            "Movimiento bloqueado; faltan calibraciones: " + ", ".join(pendientes)
        )
    if not bool(config.get("runtime", {}).get("motion_enabled", False)):
        raise ErrorConfiguracion(
            "Movimiento bloqueado por runtime.motion_enabled=false"
        )
