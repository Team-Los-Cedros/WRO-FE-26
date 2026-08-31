"""Validacion de consignas y watchdog para el firmware de la Pico 2.

Este modulo evita dependencias de ``machine`` para que el protocolo se pueda
probar tambien en CPython. En la Pico, ``main.py`` suministra
``time.ticks_diff`` para conservar el manejo correcto del desbordamiento del
contador de milisegundos de MicroPython.
"""


def parsear_consigna(linea):
    """Devuelve ``(velocidad, angulo, kd)`` o ``None`` si la trama es insegura.

    Se aceptan las dos formas historicas ``velocidad,angulo`` y
    ``velocidad,angulo,kd``. Los limites de protocolo son algo mas amplios que
    los limites mecanicos calibrados; el firmware sigue acotando el servo.
    """

    try:
        partes = [parte.strip() for parte in str(linea).strip().split(",")]
        if len(partes) not in (2, 3) or any(not parte for parte in partes):
            return None
        velocidad = int(partes[0])
        angulo = float(partes[1])
        kd = 1.0 if len(partes) == 2 else float(partes[2])
    except (TypeError, ValueError):
        return None

    # Las comparaciones tambien rechazan NaN e infinitos sin depender de que
    # una version concreta de MicroPython exponga math.isfinite().
    if not -100 <= velocidad <= 100:
        return None
    if not -45.0 <= angulo <= 45.0:
        return None
    if not 0.0 <= kd <= 2.0:
        return None
    return velocidad, angulo, kd


def watchdog_vencido(ahora_ms, ultimo_comando_ms, timeout_ms, ticks_diff):
    """Indica si no ha llegado una consigna valida dentro del plazo."""

    if ultimo_comando_ms is None:
        return True
    return ticks_diff(ahora_ms, ultimo_comando_ms) > int(timeout_ms)


__all__ = ["parsear_consigna", "watchdog_vencido"]
