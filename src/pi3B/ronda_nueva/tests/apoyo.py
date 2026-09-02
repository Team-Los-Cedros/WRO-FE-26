"""Utilidades compartidas por las pruebas offline.

En la Pi estan instalados pyserial y el resto de drivers. En la laptop donde
se escriben las pruebas no, y ``comun/lidar_driver.py`` importa ``serial`` al
cargarse porque en la Pi ese import siempre existe. Para poder probar el
parser sin hardware se registra un modulo minimo, y solo si pyserial falta de
verdad: en la Pi se sigue usando el real.
"""

import sys
import types


def asegurar_pyserial() -> bool:
    """Deja ``serial`` importable. Devuelve True si tuvo que simularlo."""

    try:
        import serial  # noqa: F401
    except ImportError:
        modulo = types.ModuleType("serial")

        class SerialException(Exception):
            pass

        modulo.SerialException = SerialException
        modulo.Serial = object
        sys.modules["serial"] = modulo
        return True
    return False


__all__ = ["asegurar_pyserial"]
