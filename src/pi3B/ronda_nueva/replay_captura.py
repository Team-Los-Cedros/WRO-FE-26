"""Replay sincronizado y sin hardware del formato ``captura_*/``.

El formato esperado es el que ya produce el robot::

    captura_YYYYMMDD_HHMMSS/
      meta.json
      imu.csv                 # t,valor_crudo_imu
      lidar.jsonl             # {"t": ..., "scan": [[angulo, mm], ...]}
      frames/2.619.jpg         # el nombre es el timestamp de captura

Los eventos se consumen en orden temporal. Antes de cada barrido LiDAR se
procesan solamente los cuadros cuyo timestamp ya ocurrio; de este modo el
replay no usa informacion futura. El paquete visual se selecciona con el
mismo :class:`BuzonVision` y la misma ventana temporal que la aplicacion de la
Pi. La salida de :class:`ControlRuta` es solo una *propuesta*: este modulo no
importa drivers de hardware ni transmite consignas.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

import cv2

from .config import cargar_configuracion
from .control_ruta import ControlRuta
from .fusion import FusionLigera
from .percepcion_lidar import PercepcionLidar
from .sincronizacion import BuzonVision
from .vision_ligera import VisionLigera

try:  # Paquete normal desde la raiz del repositorio.
    from ..comun.lidar_geometria import ProcesadorLidar
except (ImportError, ValueError):  # pragma: no cover - despliegue plano
    from comun.lidar_geometria import ProcesadorLidar


EXTENSIONES_IMAGEN = {".jpg", ".jpeg", ".png", ".bmp"}


class ErrorFormatoCaptura(ValueError):
    """La carpeta no cumple el contrato de una captura sincronizada."""


@dataclass(frozen=True)
class MuestraImu:
    timestamp: float
    yaw_crudo_deg: float


@dataclass(frozen=True)
class FrameCaptura:
    timestamp: float
    ruta: Path


def _numero_finito(valor: Any, contexto: str) -> float:
    try:
        numero = float(valor)
    except (TypeError, ValueError) as exc:
        raise ErrorFormatoCaptura("{} no es numerico".format(contexto)) from exc
    if not math.isfinite(numero):
        raise ErrorFormatoCaptura("{} no es finito".format(contexto))
    return numero


def _comprobar_archivos(entrada: Path) -> Tuple[Path, Path, Path, Path]:
    if not entrada.is_dir():
        raise ErrorFormatoCaptura("la captura no es un directorio: {}".format(entrada))
    rutas = (
        entrada / "meta.json",
        entrada / "imu.csv",
        entrada / "lidar.jsonl",
        entrada / "frames",
    )
    faltantes = [str(ruta.name) for ruta in rutas if not ruta.exists()]
    if faltantes:
        raise ErrorFormatoCaptura(
            "faltan componentes de la captura: {}".format(", ".join(faltantes))
        )
    if not rutas[3].is_dir():
        raise ErrorFormatoCaptura("frames debe ser un directorio")
    return rutas


def leer_meta(ruta: Path) -> Dict[str, Any]:
    try:
        with ruta.open("r", encoding="utf-8") as archivo:
            meta = json.load(archivo)
    except (OSError, json.JSONDecodeError) as exc:
        raise ErrorFormatoCaptura("meta.json invalido: {}".format(exc)) from exc
    if not isinstance(meta, dict):
        raise ErrorFormatoCaptura("meta.json debe contener un objeto JSON")
    return meta


def leer_imu(ruta: Path) -> Tuple[MuestraImu, ...]:
    muestras: List[MuestraImu] = []
    try:
        with ruta.open("r", newline="", encoding="utf-8-sig") as archivo:
            lector = csv.DictReader(archivo)
            requeridas = {"t", "valor_crudo_imu"}
            if not lector.fieldnames or not requeridas.issubset(lector.fieldnames):
                raise ErrorFormatoCaptura(
                    "imu.csv debe tener columnas t,valor_crudo_imu"
                )
            anterior = -math.inf
            for linea, fila in enumerate(lector, start=2):
                timestamp = _numero_finito(fila.get("t"), "imu.csv:{} t".format(linea))
                yaw = _numero_finito(
                    fila.get("valor_crudo_imu"),
                    "imu.csv:{} valor_crudo_imu".format(linea),
                )
                if timestamp < anterior:
                    raise ErrorFormatoCaptura(
                        "imu.csv no esta ordenado en la linea {}".format(linea)
                    )
                muestras.append(MuestraImu(timestamp, yaw))
                anterior = timestamp
    except OSError as exc:
        raise ErrorFormatoCaptura("no se pudo leer imu.csv: {}".format(exc)) from exc
    if not muestras:
        raise ErrorFormatoCaptura("imu.csv no contiene muestras")
    return tuple(muestras)


def listar_frames(ruta: Path) -> Tuple[FrameCaptura, ...]:
    frames: List[FrameCaptura] = []
    for imagen in ruta.iterdir():
        if not imagen.is_file() or imagen.suffix.lower() not in EXTENSIONES_IMAGEN:
            continue
        timestamp = _numero_finito(
            imagen.stem, "timestamp del frame {}".format(imagen.name)
        )
        frames.append(FrameCaptura(timestamp, imagen))
    frames.sort(key=lambda frame: (frame.timestamp, frame.ruta.name.lower()))
    if not frames:
        raise ErrorFormatoCaptura("frames/ no contiene imagenes con timestamp")
    return tuple(frames)


def _contar_lineas_no_vacias(ruta: Path) -> int:
    try:
        with ruta.open("r", encoding="utf-8") as archivo:
            return sum(1 for linea in archivo if linea.strip())
    except OSError as exc:
        raise ErrorFormatoCaptura("no se pudo leer lidar.jsonl: {}".format(exc)) from exc


def iterar_barridos(ruta: Path) -> Iterator[Tuple[float, Tuple[Tuple[float, float], ...]]]:
    """Entrega barridos validados y ordenados sin cargar el JSONL completo."""

    anterior = -math.inf
    encontrados = 0
    try:
        with ruta.open("r", encoding="utf-8") as archivo:
            for numero_linea, texto in enumerate(archivo, start=1):
                if not texto.strip():
                    continue
                try:
                    registro = json.loads(texto)
                except json.JSONDecodeError as exc:
                    raise ErrorFormatoCaptura(
                        "lidar.jsonl:{} JSON invalido: {}".format(numero_linea, exc)
                    ) from exc
                if not isinstance(registro, dict) or "t" not in registro or "scan" not in registro:
                    raise ErrorFormatoCaptura(
                        "lidar.jsonl:{} requiere t y scan".format(numero_linea)
                    )
                timestamp = _numero_finito(
                    registro["t"], "lidar.jsonl:{} t".format(numero_linea)
                )
                if timestamp < anterior:
                    raise ErrorFormatoCaptura(
                        "lidar.jsonl no esta ordenado en la linea {}".format(numero_linea)
                    )
                scan_crudo = registro["scan"]
                if not isinstance(scan_crudo, list):
                    raise ErrorFormatoCaptura(
                        "lidar.jsonl:{} scan debe ser una lista".format(numero_linea)
                    )
                scan: List[Tuple[float, float]] = []
                for indice, punto in enumerate(scan_crudo):
                    if not isinstance(punto, (list, tuple)) or len(punto) < 2:
                        raise ErrorFormatoCaptura(
                            "lidar.jsonl:{} scan[{}] invalido".format(
                                numero_linea, indice
                            )
                        )
                    angulo = _numero_finito(
                        punto[0],
                        "lidar.jsonl:{} scan[{}].angulo".format(numero_linea, indice),
                    )
                    distancia = _numero_finito(
                        punto[1],
                        "lidar.jsonl:{} scan[{}].distancia".format(numero_linea, indice),
                    )
                    scan.append((angulo, distancia))
                if not scan:
                    raise ErrorFormatoCaptura(
                        "lidar.jsonl:{} contiene un barrido vacio".format(numero_linea)
                    )
                encontrados += 1
                anterior = timestamp
                yield timestamp, tuple(scan)
    except OSError as exc:
        raise ErrorFormatoCaptura("no se pudo leer lidar.jsonl: {}".format(exc)) from exc
    if not encontrados:
        raise ErrorFormatoCaptura("lidar.jsonl no contiene barridos")


class _ImuSincronizado:
    """Seleccion causal (ultima muestra no posterior) con puntero O(n)."""

    def __init__(self, muestras: Sequence[MuestraImu]):
        self._muestras = muestras
        self._indice = -1
        self._cero = float(muestras[0].yaw_crudo_deg)

    def para(self, timestamp: float) -> Tuple[float, float]:
        while (
            self._indice + 1 < len(self._muestras)
            and self._muestras[self._indice + 1].timestamp <= timestamp + 1e-12
        ):
            self._indice += 1
        if self._indice < 0:
            raise ErrorFormatoCaptura(
                "no hay IMU anterior al primer barrido t={:.6f}".format(timestamp)
            )
        muestra = self._muestras[self._indice]
        return float(muestra.yaw_crudo_deg) - self._cero, timestamp - muestra.timestamp


def _alertas_meta(
    meta: Dict[str, Any], frames: int, imu: int, lidar: int
) -> List[str]:
    alertas: List[str] = []
    comparaciones = (
        ("frames_camara", frames),
        ("lecturas_imu", imu),
        ("barridos_lidar", lidar),
    )
    for clave, real in comparaciones:
        if clave not in meta:
            alertas.append("meta.json no declara {}".format(clave))
            continue
        try:
            declarado = int(meta[clave])
        except (TypeError, ValueError):
            alertas.append("meta.json tiene {} invalido".format(clave))
            continue
        if declarado != real:
            alertas.append("{} declara {}, archivo tiene {}".format(clave, declarado, real))
    return alertas


CAMPOS_CSV = (
    "t",
    "heading",
    "imu_edad_ms",
    "vision_t",
    "vision_edad_ms",
    "detecciones_vision",
    "objetos_lidar",
    "tracks",
    "tracks_confirmados",
    "estado",
    "razon",
    "velocidad_propuesta",
    "angulo_propuesto",
    "esquinas",
    "frontal",
    "frontal_muro",
    "izquierda",
    "derecha",
    "trasera",
    "trasera_valida",
    "cobertura_trasera",
    "mascara_oclusion_confirmada",
    "hueco_confianza",
)


def _escribir_csv(ruta: Path, filas: Sequence[Dict[str, Any]]) -> None:
    ruta.parent.mkdir(parents=True, exist_ok=True)
    with ruta.open("w", newline="", encoding="utf-8") as archivo:
        writer = csv.DictWriter(archivo, fieldnames=CAMPOS_CSV)
        writer.writeheader()
        writer.writerows(filas)


def ejecutar_replay(
    entrada: Path,
    config: Dict[str, Any],
    sentido: Optional[str] = None,
    color_piso: Optional[str] = None,
    cada_lidar: int = 1,
    salida_csv: Optional[Path] = None,
    max_barridos: Optional[int] = None,
) -> Dict[str, Any]:
    """Ejecuta la cadena completa como simulacion de decisiones.

    ``velocidad`` se realimenta a la prediccion de tracks tal como ocurriria en
    la Pi, pero nunca sale de memoria. Por diseño este modulo no contiene una
    ruta de codigo capaz de abrir serial, GPIO, Picamera2 o el LiDAR.
    """

    entrada = Path(entrada)
    ruta_meta, ruta_imu, ruta_lidar, ruta_frames = _comprobar_archivos(entrada)
    meta = leer_meta(ruta_meta)
    muestras_imu = leer_imu(ruta_imu)
    frames = listar_frames(ruta_frames)
    total_lidar_archivo = _contar_lineas_no_vacias(ruta_lidar)
    if total_lidar_archivo <= 0:
        raise ErrorFormatoCaptura("lidar.jsonl no contiene barridos")

    paso_lidar = max(1, int(cada_lidar))
    if max_barridos is not None and int(max_barridos) < 1:
        raise ValueError("max_barridos debe ser positivo")

    config_replay = copy.deepcopy(config)
    config_replay["camera"]["array_color_order"] = "BGR"
    if sentido is not None:
        sentido_normalizado = str(sentido).strip().upper()
        if sentido_normalizado not in ("AUTO", "LEFT", "RIGHT"):
            raise ValueError("sentido debe ser AUTO, LEFT o RIGHT")
        config_replay["control"]["turn_direction"] = sentido_normalizado
    if color_piso is not None:
        color_piso = str(color_piso).strip().upper()

    vision = VisionLigera(config_replay)
    buzon_vision = BuzonVision(4)
    lidar_geometria = ProcesadorLidar()
    percepcion = PercepcionLidar(config_replay)
    fusion = FusionLigera(config_replay)
    control = ControlRuta(config_replay)
    imu = _ImuSincronizado(muestras_imu)

    indice_frame = 0
    frames_decodificados = 0
    frames_invalidos = 0
    detecciones_visuales = 0
    barridos_leidos = 0
    barridos_procesados = 0
    objetos_lidar = 0
    ciclos_con_vision = 0
    ciclos_tracks_confirmados = 0
    max_tracks = 0
    propuestas_movimiento = 0
    ultima_velocidad = 0
    ultimo_estado = control.estado
    ultima_razon = ""
    estados: Counter = Counter()
    filas: List[Dict[str, Any]] = []
    edades_imu_ms: List[float] = []
    edades_vision_ms: List[float] = []

    oclusion_racha = 0
    oclusion_mejor_racha = 0
    oclusion_validada = False
    diagnostico_oclusion = "sin barridos procesados"
    requeridos_oclusion = max(
        1, int(config_replay["hardware"].get("occlusion_validation_scans", 3))
    )

    primer_t: Optional[float] = None
    ultimo_t: Optional[float] = None
    for indice_lidar, (timestamp, scan) in enumerate(iterar_barridos(ruta_lidar)):
        barridos_leidos += 1
        if indice_lidar % paso_lidar:
            continue
        if max_barridos is not None and barridos_procesados >= int(max_barridos):
            break

        # Publicar unicamente cuadros ya capturados: nunca se filtra el futuro.
        while indice_frame < len(frames) and frames[indice_frame].timestamp <= timestamp + 1e-12:
            entrada_frame = frames[indice_frame]
            indice_frame += 1
            imagen = cv2.imread(str(entrada_frame.ruta), cv2.IMREAD_COLOR)
            if imagen is None:
                frames_invalidos += 1
                continue
            try:
                paquete_nuevo = vision.procesar(imagen, entrada_frame.timestamp)
            except Exception as exc:
                raise ErrorFormatoCaptura(
                    "no se pudo procesar {}: {}".format(entrada_frame.ruta.name, exc)
                ) from exc
            buzon_vision.publicar(paquete_nuevo)
            frames_decodificados += 1
            detecciones_visuales += len(paquete_nuevo.detecciones)

        heading, edad_imu_s = imu.para(timestamp)
        edades_imu_ms.append(1000.0 * edad_imu_s)
        medicion = lidar_geometria.procesar(scan)
        medicion.timestamp = timestamp
        lado_parqueo = control.lado_parqueo_solicitado
        resultado = percepcion.procesar(
            scan,
            medicion,
            timestamp=timestamp,
            lado_parqueo=lado_parqueo,
        )
        corredor = resultado.corredor
        diagnostico_oclusion = corredor.diagnostico_oclusion
        if corredor.estructura_fuera_mascara_deg:
            oclusion_racha = 0
        elif corredor.mascara_oclusion_confirmada:
            oclusion_racha += 1
            oclusion_mejor_racha = max(oclusion_mejor_racha, oclusion_racha)
            if oclusion_racha >= requeridos_oclusion:
                oclusion_validada = True
        else:
            oclusion_racha = 0

        paquete = buzon_vision.mas_cercano(
            timestamp, float(config_replay["fusion"]["max_camera_lidar_age_s"])
        )
        if paquete is not None:
            ciclos_con_vision += 1
            edad_vision_ms = 1000.0 * (timestamp - paquete.timestamp)
            edades_vision_ms.append(edad_vision_ms)
        else:
            edad_vision_ms = None

        tracks = fusion.actualizar(
            resultado.objetos,
            paquete,
            heading,
            ultima_velocidad,
            timestamp=timestamp,
        )
        consigna = control.procesar(
            corredor,
            tracks,
            heading,
            color_piso,
            hueco=resultado.hueco,
            ahora=timestamp,
        )
        # Realimentacion de la simulacion; no existe ningun envio a hardware.
        ultima_velocidad = int(consigna.velocidad)

        barridos_procesados += 1
        objetos_lidar += len(resultado.objetos)
        confirmados = sum(1 for track in tracks if track.confirmado)
        ciclos_tracks_confirmados += int(confirmados > 0)
        max_tracks = max(max_tracks, len(tracks))
        propuestas_movimiento += int(
            consigna.velocidad != 0 or abs(float(consigna.angulo)) > 1e-9
        )
        ultimo_estado = consigna.estado
        ultima_razon = consigna.razon
        estados[consigna.estado] += 1
        primer_t = timestamp if primer_t is None else primer_t
        ultimo_t = timestamp

        filas.append(
            {
                "t": "{:.6f}".format(timestamp),
                "heading": "{:.3f}".format(heading),
                "imu_edad_ms": "{:.3f}".format(1000.0 * edad_imu_s),
                "vision_t": "" if paquete is None else "{:.6f}".format(paquete.timestamp),
                "vision_edad_ms": "" if edad_vision_ms is None else "{:.3f}".format(edad_vision_ms),
                "detecciones_vision": 0 if paquete is None else len(paquete.detecciones),
                "objetos_lidar": len(resultado.objetos),
                "tracks": len(tracks),
                "tracks_confirmados": confirmados,
                "estado": consigna.estado,
                "razon": consigna.razon,
                "velocidad_propuesta": consigna.velocidad,
                "angulo_propuesto": "{:.3f}".format(consigna.angulo),
                "esquinas": control.esquinas,
                "frontal": "{:.3f}".format(corredor.frontal_mm),
                "frontal_muro": "{:.3f}".format(corredor.frontal_muro_mm),
                "izquierda": "{:.3f}".format(corredor.izquierda_mm),
                "derecha": "{:.3f}".format(corredor.derecha_mm),
                "trasera": "{:.3f}".format(corredor.trasera_mm),
                "trasera_valida": int(corredor.trasera_valida),
                "cobertura_trasera": "{:.4f}".format(corredor.cobertura_trasera),
                "mascara_oclusion_confirmada": int(corredor.mascara_oclusion_confirmada),
                "hueco_confianza": "" if resultado.hueco is None else "{:.4f}".format(resultado.hueco.confianza),
            }
        )

        # Una aplicacion real se detendria aqui; continuar alteraria la FSM y
        # ya no representaria la ejecucion desplegada.
        if consigna.terminado:
            break

    if barridos_procesados == 0:
        raise ErrorFormatoCaptura("no se proceso ningun barrido LiDAR")
    if salida_csv is not None:
        _escribir_csv(Path(salida_csv), filas)

    alertas = _alertas_meta(
        meta, len(frames), len(muestras_imu), total_lidar_archivo
    )
    media_imu = sum(edades_imu_ms) / len(edades_imu_ms)
    media_vision = (
        sum(edades_vision_ms) / len(edades_vision_ms)
        if edades_vision_ms
        else None
    )
    return {
        "modo": "offline_sombra_sin_hardware",
        "captura": str(entrada.resolve()),
        "meta": meta,
        "meta_consistente": not alertas,
        "alertas_meta": tuple(alertas),
        "frames_archivo": len(frames),
        "frames_decodificados": frames_decodificados,
        "frames_invalidos": frames_invalidos,
        "imu_muestras": len(muestras_imu),
        "lidar_barridos_archivo": total_lidar_archivo,
        "lidar_barridos_leidos": barridos_leidos,
        "lidar_barridos_procesados": barridos_procesados,
        "t_inicio": primer_t,
        "t_fin": ultimo_t,
        "duracion_procesada_s": 0.0 if primer_t is None or ultimo_t is None else ultimo_t - primer_t,
        "detecciones_visuales": detecciones_visuales,
        "objetos_lidar": objetos_lidar,
        "ciclos_con_vision": ciclos_con_vision,
        "vision_edad_media_ms": media_vision,
        "vision_edad_max_ms": max(edades_vision_ms) if edades_vision_ms else None,
        "imu_edad_media_ms": media_imu,
        "imu_edad_max_ms": max(edades_imu_ms),
        "max_tracks": max_tracks,
        "ciclos_tracks_confirmados": ciclos_tracks_confirmados,
        "estados": dict(sorted(estados.items())),
        "estado_final": ultimo_estado,
        "razon_final": ultima_razon,
        "esquinas": control.esquinas,
        "propuestas_movimiento": propuestas_movimiento,
        "comandos_hardware_enviados": 0,
        "oclusion_validada": oclusion_validada,
        "oclusion_racha_requerida": requeridos_oclusion,
        "oclusion_mejor_racha": oclusion_mejor_racha,
        "diagnostico_oclusion": diagnostico_oclusion,
        "arranque_hardware_autorizado": False,
    }


def _media_texto(valor: Optional[float]) -> str:
    return "sin coincidencias" if valor is None else "{:.2f} ms".format(valor)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Replay sincronizado LiDAR/camara/IMU, siempre sin hardware"
    )
    parser.add_argument("entrada", type=Path, help="directorio captura_*/")
    parser.add_argument("--config", help="JSON alternativo")
    parser.add_argument(
        "--sentido",
        choices=("AUTO", "LEFT", "RIGHT"),
        help="sobrescribe control.turn_direction solo durante el replay",
    )
    parser.add_argument(
        "--color-piso",
        choices=("AZUL", "NARANJA", "DESCONOCIDO"),
        help="color constante si se desea probar turn_direction=AUTO",
    )
    parser.add_argument("--cada-lidar", type=int, default=1)
    parser.add_argument("--max-barridos", type=int)
    parser.add_argument("--salida-csv", type=Path)
    args = parser.parse_args(argv)

    try:
        resumen = ejecutar_replay(
            args.entrada,
            cargar_configuracion(args.config),
            sentido=args.sentido,
            color_piso=args.color_piso,
            cada_lidar=args.cada_lidar,
            salida_csv=args.salida_csv,
            max_barridos=args.max_barridos,
        )
    except (ErrorFormatoCaptura, OSError, ValueError) as exc:
        parser.error(str(exc))

    print("Replay offline completado; comandos enviados a hardware: 0")
    print(
        "Datos: LiDAR={lidar_barridos_procesados}/{lidar_barridos_archivo} "
        "IMU={imu_muestras} frames={frames_decodificados}/{frames_archivo}".format(
            **resumen
        )
    )
    print(
        "Sincronizacion: IMU media={:.2f} ms, vision media={}".format(
            resumen["imu_edad_media_ms"],
            _media_texto(resumen["vision_edad_media_ms"]),
        )
    )
    estado_oclusion = "VALIDADA" if resumen["oclusion_validada"] else "BLOQUEADA"
    print(
        "Seguridad de oclusion: {} ({}/{}) - {}".format(
            estado_oclusion,
            resumen["oclusion_mejor_racha"],
            resumen["oclusion_racha_requerida"],
            resumen["diagnostico_oclusion"],
        )
    )
    print(
        "FSM sombra: estado={} esquinas={} propuestas={} razon={}".format(
            resumen["estado_final"],
            resumen["esquinas"],
            resumen["propuestas_movimiento"],
            resumen["razon_final"] or "sin razon terminal",
        )
    )
    for alerta in resumen["alertas_meta"]:
        print("[AVISO META] " + alerta)
    if args.salida_csv is not None:
        print("CSV: {}".format(args.salida_csv.resolve()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = (
    "ErrorFormatoCaptura",
    "ejecutar_replay",
    "iterar_barridos",
    "leer_imu",
    "leer_meta",
    "listar_frames",
    "main",
)
