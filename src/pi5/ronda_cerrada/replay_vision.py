"""Replay offline de imagenes o video, sin abrir ningun hardware.

OpenCV decodifica archivos como BGR, independientemente del formato pedido a
Picamera2 durante la carrera. Por eso el reproductor fuerza solamente el orden
de entrada a BGR y conserva el resto de la calibracion seleccionada.
"""

import argparse
import copy
import csv
from pathlib import Path
from typing import Iterator, Optional, Tuple

import cv2

from .config import cargar_configuracion
from .vision_ligera import VisionLigera


EXTENSIONES_IMAGEN = {".jpg", ".jpeg", ".png", ".bmp"}


def _clave_imagen(ruta: Path):
    try:
        return (0, float(ruta.stem))
    except ValueError:
        return (1, ruta.name.lower())


def iterar_frames(ruta: Path, cada: int = 1) -> Iterator[Tuple[float, object]]:
    """Entrega ``(timestamp_relativo, frame_bgr)`` desde directorio o video."""

    cada = max(1, int(cada))
    if ruta.is_dir():
        imagenes = sorted(
            (p for p in ruta.rglob("*") if p.suffix.lower() in EXTENSIONES_IMAGEN),
            key=_clave_imagen,
        )
        for indice, imagen in enumerate(imagenes):
            if indice % cada:
                continue
            frame = cv2.imread(str(imagen), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            try:
                timestamp = float(imagen.stem)
            except ValueError:
                timestamp = indice / 15.0
            yield timestamp, frame
        return

    captura = cv2.VideoCapture(str(ruta))
    if not captura.isOpened():
        raise OSError("no se pudo abrir {}".format(ruta))
    fps = float(captura.get(cv2.CAP_PROP_FPS))
    if fps <= 0.0:
        fps = 30.0
    indice = 0
    try:
        while True:
            ok, frame = captura.read()
            if not ok:
                break
            if indice % cada == 0:
                timestamp_ms = float(captura.get(cv2.CAP_PROP_POS_MSEC))
                timestamp = timestamp_ms / 1000.0 if timestamp_ms > 0 else indice / fps
                yield timestamp, frame
            indice += 1
    finally:
        captura.release()


def ejecutar_replay(
    entrada: Path,
    config,
    cada: int = 1,
    salida_csv: Optional[Path] = None,
):
    config_replay = copy.deepcopy(config)
    config_replay["camera"]["array_color_order"] = "BGR"
    vision = VisionLigera(config_replay)

    filas = []
    duraciones = []
    conteos = {"ROJO": 0, "VERDE": 0}
    cuadros = 0
    for timestamp, frame in iterar_frames(entrada, cada):
        paquete = vision.procesar(frame, timestamp)
        cuadros += 1
        duraciones.append(paquete.duracion_ms)
        for deteccion in paquete.detecciones:
            conteos[deteccion.color] = conteos.get(deteccion.color, 0) + 1
            filas.append(
                {
                    "timestamp": "{:.6f}".format(timestamp),
                    "color": deteccion.color,
                    "bearing_deg": "{:.3f}".format(deteccion.bearing_deg),
                    "cx": "{:.2f}".format(deteccion.centro_px[0]),
                    "cy": "{:.2f}".format(deteccion.centro_px[1]),
                    "area_ratio": "{:.6f}".format(deteccion.area_ratio),
                    "confianza": "{:.4f}".format(deteccion.confianza),
                    "soporte_suelo": "{:.4f}".format(deteccion.soporte_suelo),
                }
            )

    if salida_csv is not None:
        salida_csv.parent.mkdir(parents=True, exist_ok=True)
        with salida_csv.open("w", newline="", encoding="utf-8") as archivo:
            campos = (
                "timestamp",
                "color",
                "bearing_deg",
                "cx",
                "cy",
                "area_ratio",
                "confianza",
                "soporte_suelo",
            )
            writer = csv.DictWriter(archivo, fieldnames=campos)
            writer.writeheader()
            writer.writerows(filas)

    ordenadas = sorted(duraciones)
    media = sum(ordenadas) / len(ordenadas) if ordenadas else 0.0
    indice_p95 = max(0, int(round(0.95 * len(ordenadas))) - 1)
    p95 = ordenadas[indice_p95] if ordenadas else 0.0
    return {
        "cuadros": cuadros,
        "detecciones": len(filas),
        "rojo": conteos.get("ROJO", 0),
        "verde": conteos.get("VERDE", 0),
        "media_ms": media,
        "p95_ms": p95,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Replay offline de VisionLigera")
    parser.add_argument("entrada", type=Path)
    parser.add_argument("--config")
    parser.add_argument("--cada", type=int, default=1)
    parser.add_argument("--salida-csv", type=Path)
    args = parser.parse_args(argv)

    if not args.entrada.exists():
        parser.error("la entrada no existe: {}".format(args.entrada))
    entrada_normalizada = str(args.entrada).replace("\\", "/").lower()
    if "ronda_camara/webcam" in entrada_normalizada:
        print("[AVISO] El material webcam es externo, no el feed de a bordo.")

    resumen = ejecutar_replay(
        args.entrada,
        cargar_configuracion(args.config),
        cada=args.cada,
        salida_csv=args.salida_csv,
    )
    print(
        "Cuadros={cuadros} detecciones={detecciones} (R={rojo}, V={verde}) "
        "vision media={media_ms:.2f}ms p95={p95_ms:.2f}ms".format(**resumen)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
