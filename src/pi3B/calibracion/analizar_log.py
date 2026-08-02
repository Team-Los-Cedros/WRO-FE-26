# Herramienta offline (no corre en carrera): resume un CSV generado por
# comun/registro_metricas.py en metricas agregadas, para validar el ajuste
# de KP_LATERAL/KD_ESTABILIDAD (README seccion 5.4) con datos en vez de
# observacion cualitativa en pista.
#
# Uso: python3 analizar_log.py <ruta_al_csv>
import csv
import statistics
import sys

LIMITE_SERVO_IZQ = 25.0  # grados, tope fisico util (ver README seccion 7.3)
LIMITE_SERVO_DER = -25.0


def _flotante(valor):
    try:
        return float(valor)
    except (TypeError, ValueError):
        return None


def analizar(ruta):
    filas = []
    with open(ruta, newline="") as f:
        for fila in csv.DictReader(f):
            filas.append(fila)

    if not filas:
        print("[!] Log vacio, nada que analizar.")
        return

    duracion = _flotante(filas[-1]["t"]) or 0.0
    errores = [abs(v) for v in (_flotante(f["error_lateral"]) for f in filas) if v is not None]
    angulos = [_flotante(f["angulo"]) for f in filas]
    angulos = [a for a in angulos if a is not None]

    saturado = sum(1 for a in angulos if a >= LIMITE_SERVO_IZQ or a <= LIMITE_SERVO_DER)

    conteo_fase = {}
    for f in filas:
        conteo_fase[f["fase"]] = conteo_fase.get(f["fase"], 0) + 1

    transiciones_retroceso = 0
    estado_anterior = None
    for f in filas:
        estado = f.get("estado") or None
        if estado == "RETROCESO" and estado_anterior != "RETROCESO":
            transiciones_retroceso += 1
        estado_anterior = estado

    print(f"=== {ruta} ===")
    print(f"Filas (ciclos de barrido): {len(filas)}")
    print(f"Duracion: {duracion:.1f} s")
    if errores:
        print(f"Error lateral |e|: promedio {statistics.mean(errores):.1f} mm, "
              f"maximo {max(errores):.1f} mm, mediana {statistics.median(errores):.1f} mm")
    if angulos:
        pct_saturado = 100.0 * saturado / len(angulos)
        print(f"Ciclos con angulo saturado en el limite fisico del servo: "
              f"{saturado}/{len(angulos)} ({pct_saturado:.1f}%)")
    print(f"Ciclos por fase: {conteo_fase}")
    if transiciones_retroceso:
        print(f"Eventos de emergencia (entradas a RETROCESO): {transiciones_retroceso}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Uso: python3 analizar_log.py <ruta_al_csv>")
        sys.exit(1)
    analizar(sys.argv[1])
