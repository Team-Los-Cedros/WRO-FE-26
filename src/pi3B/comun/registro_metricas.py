# Logger de telemetria por ciclo, compartido por ambas rondas. Hasta ahora
# el ajuste de KP_LATERAL/KD_ESTABILIDAD (ver README seccion 5.4) se
# validaba solo de forma observacional en pista, sin datos numericos. Este
# modulo escribe un CSV por corrida en logs/ (una fila por barrido de
# LiDAR procesado) para poder medir error lateral, saturacion del servo y
# eventos de emergencia entre corridas, no solo "se vio mejor o peor".
#
# Uso: registro = RegistroMetricas("ronda_abierta"); ... ;
# registro.registrar(fase=fase_actual, heading=h, error_lateral=e, angulo=a,
# velocidad=v); ... ; registro.cerrar()
#
# analizar_log.py (en calibracion/) resume estos CSV en metricas agregadas.
import csv
import os
import time

CARPETA_LOGS = "logs"

CAMPOS = ["t", "fase", "estado", "heading", "error_lateral", "angulo", "velocidad"]


class RegistroMetricas:
    def __init__(self, nombre_ronda, carpeta=CARPETA_LOGS):
        os.makedirs(carpeta, exist_ok=True)
        marca = time.strftime("%Y%m%d_%H%M%S")
        self.ruta = os.path.join(carpeta, f"{nombre_ronda}_{marca}.csv")
        self._archivo = open(self.ruta, "w", newline="")
        self._escritor = csv.DictWriter(self._archivo, fieldnames=CAMPOS)
        self._escritor.writeheader()
        self._t0 = time.time()
        print(f"[+] Registro de metricas: {self.ruta}")

    def registrar(self, **campos):
        fila = {c: "" for c in CAMPOS}
        fila["t"] = round(time.time() - self._t0, 3)
        fila.update(campos)
        try:
            self._escritor.writerow(fila)
            self._archivo.flush()
        except Exception:
            pass  # el logging nunca debe tumbar el bucle de control

    def cerrar(self):
        try:
            self._archivo.close()
        except Exception:
            pass
