# Analisis OFFLINE de lo que grabo capturar_pista.py. Corre en la laptop,
# sin robot: solo necesita numpy y opencv. Es la mitad que convierte la
# captura en respuestas.
#
# Por cada pose responde tres cosas:
#
#  1. GEOMETRIA. Promedia todos los barridos (el robot estaba quieto) y
#     ajusta una RECTA a los puntos de cada muro lateral por minimos
#     cuadrados totales con poda de outliers. De ahi salen la distancia
#     perpendicular real y el angulo del muro, y se comparan contra lo
#     que devuelve el estimador de dos haces de lidar_geometria.py. Si
#     los dos no coinciden, el estimador de dos haces es el que hay que
#     cambiar -- ese es el punto de "validar la extraccion de rectas".
#
#  2. CLUSTERING. Corre el ABD real del repo sobre el barrido promedio y
#     lista cada cluster con su clasificacion. Aqui se ve si una esquina
#     se cuela como poste o si un poste a 900mm se pierde.
#
#  3. HSV. Reprocesa los frames crudos con los umbrales vigentes de
#     vision.py y, sobre el contorno mas grande, saca los percentiles de
#     H/S/V. Con eso propone rangos en vez de adivinarlos con sliders.
#
# Uso (laptop):
#   python3 revisar_captura.py capturas_pista/                # todas
#   python3 revisar_captura.py capturas_pista/2026...poste_rojo_600/
import json
import math
import os
import sys

import numpy as np
import cv2

# lidar_geometria.py vive en comun/ en el repo y plano en /home/pi.
# Se prueban las dos formas para poder correr esto sin desplegar.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "comun"))
import lidar_geometria as lg          # noqa: E402

# Umbrales vigentes en ronda_cerrada/vision.py. Si se cambian alla, hay
# que cambiarlos aqui (o mejor: mover los dos a un solo modulo).
ROJO_BAJO_1 = np.array([0,   151,  99]);  ROJO_ALTO_1 = np.array([15,  255, 255])
ROJO_BAJO_2 = np.array([158, 160,  82]);  ROJO_ALTO_2 = np.array([179, 255, 255])
VERDE_BAJO  = np.array([43,   68,  50]);  VERDE_ALTO  = np.array([85,  255, 255])
AREA_MIN_DETECCION = 350
_KERNEL = np.ones((5, 5), np.uint8)

# Sectores donde se busca cada muro para el ajuste de recta. Mas anchos
# que los sectores de navegacion a proposito: se quiere ver la recta
# completa, no la distancia minima.
SECTOR_MURO_DER = (25, 155)
SECTOR_MURO_IZQ = (205, 335)
MAX_DIST_MURO   = 2500.0
ITER_PODA       = 3
UMBRAL_PODA     = 2.0      # veces el RMS


# ==========================================
# CARGA
# ==========================================
def cargar_escaneos(carpeta):
    ruta = os.path.join(carpeta, "escaneos.jsonl")
    if not os.path.exists(ruta):
        return []
    escaneos = []
    with open(ruta) as f:
        for linea in f:
            linea = linea.strip()
            if linea:
                escaneos.append(json.loads(linea))
    return escaneos


def perfil_promedio(escaneos):
    # El robot estaba quieto: promediar los perfiles de 360 bins baja el
    # ruido del C1 (~15mm) por raiz de N. Solo se promedian los bins con
    # dato en ese barrido.
    acum = np.zeros(lg.NUM_BINS)
    cuenta = np.zeros(lg.NUM_BINS)
    for e in escaneos:
        perfil = lg.construir_perfil_360([(a, d) for a, d in e["puntos"]])
        for i, v in enumerate(perfil):
            if v < 7999.0:
                acum[i] += v
                cuenta[i] += 1
    perfil = np.full(lg.NUM_BINS, 8000.0)
    validos = cuenta > 0
    perfil[validos] = acum[validos] / cuenta[validos]
    return perfil, cuenta


def puntos_xy(perfil, ang_min, ang_max, dist_max):
    # x+ = derecha, y+ = frente (misma convencion que lidar_geometria)
    pts = []
    for grado in range(ang_min, ang_max + 1):
        d = perfil[grado % lg.NUM_BINS]
        if d < dist_max:
            r = math.radians(grado)
            pts.append((d * math.sin(r), d * math.cos(r)))
    return np.array(pts) if pts else np.empty((0, 2))


# ==========================================
# 1. AJUSTE DE RECTA A UN MURO
# ==========================================
def ajustar_recta(pts):
    # Minimos cuadrados TOTALES (ortogonales): un muro casi paralelo al
    # eje y hace explotar el ajuste clasico y = mx + b, por eso se usa la
    # forma normal n.p = c con n unitario, que no tiene direccion mala.
    if len(pts) < 8:
        return None

    activos = pts
    for _ in range(ITER_PODA):
        centro = activos.mean(axis=0)
        u, s, vt = np.linalg.svd(activos - centro, full_matrices=False)
        direccion = vt[0]                       # eje principal = la recta
        normal = np.array([-direccion[1], direccion[0]])
        c = float(normal @ centro)
        residuos = np.abs(pts @ normal - c)
        rms = float(np.sqrt((residuos ** 2).mean()))
        nuevos = pts[residuos < max(UMBRAL_PODA * rms, 8.0)]
        if len(nuevos) < 8 or len(nuevos) == len(activos):
            activos = nuevos if len(nuevos) >= 8 else activos
            break
        activos = nuevos

    centro = activos.mean(axis=0)
    u, s, vt = np.linalg.svd(activos - centro, full_matrices=False)
    direccion = vt[0]
    normal = np.array([-direccion[1], direccion[0]])
    c = float(normal @ centro)
    residuos_act = np.abs(activos @ normal - c)
    rms = float(np.sqrt((residuos_act ** 2).mean()))

    # Distancia perpendicular del robot (origen) a la recta
    distancia = abs(c)

    # Angulo del muro respecto al eje y (el frente). 0 = muro paralelo al
    # robot, positivo = el muro se abre hacia la derecha por delante.
    ang = math.degrees(math.atan2(direccion[0], direccion[1]))
    while ang > 90:
        ang -= 180
    while ang < -90:
        ang += 180

    return {
        "distancia_perp_mm": round(distancia, 1),
        "angulo_grados": round(ang, 2),
        "rms_mm": round(rms, 2),
        "n_puntos": int(len(activos)),
        "n_descartados": int(len(pts) - len(activos)),
        "extension_mm": round(float(np.linalg.norm(
            activos[np.argmax(activos @ direccion)] -
            activos[np.argmin(activos @ direccion)])), 1),
    }


def analizar_geometria(escaneos):
    perfil, cuenta = perfil_promedio(escaneos)
    perfil_lista = perfil.tolist()

    der = ajustar_recta(puntos_xy(perfil_lista, *SECTOR_MURO_DER, MAX_DIST_MURO))
    izq = ajustar_recta(puntos_xy(perfil_lista, *SECTOR_MURO_IZQ, MAX_DIST_MURO))

    # El mismo estimador de dos haces que corre en carrera
    d_perp_der = lg.distancia_en_rango(perfil_lista, lg.ANGULO_MIN_PERP_DER, lg.ANGULO_MAX_PERP_DER)
    d_perp_izq = lg.distancia_en_rango(perfil_lista, lg.ANGULO_MIN_PERP_IZQ, lg.ANGULO_MAX_PERP_IZQ)
    d_diag_der = lg.distancia_en_rango(perfil_lista, lg.ANGULO_MIN_DIAG_DER, lg.ANGULO_MAX_DIAG_DER)
    d_diag_izq = lg.distancia_en_rango(perfil_lista, lg.ANGULO_MIN_DIAG_IZQ, lg.ANGULO_MAX_DIAG_IZQ)

    ang_2haces_der = 0.0
    if d_perp_der < lg.DIST_PARED_VALIDA_MAX and d_diag_der < lg.DIST_PARED_VALIDA_MAX:
        dx = d_diag_der * 0.7071 - d_perp_der
        dy = d_diag_der * 0.7071
        if dy > 1.0:
            ang_2haces_der = math.degrees(math.atan2(dx, dy))

    ancho_carril = None
    if der and izq:
        ancho_carril = der["distancia_perp_mm"] + izq["distancia_perp_mm"]

    return {
        "n_barridos": len(escaneos),
        "bins_con_dato": int((cuenta > 0).sum()),
        "muro_derecho": der,
        "muro_izquierdo": izq,
        "ancho_carril_mm": round(ancho_carril, 1) if ancho_carril else None,
        "dos_haces": {
            "d_perp_der": round(d_perp_der, 1), "d_diag_der": round(d_diag_der, 1),
            "d_perp_izq": round(d_perp_izq, 1), "d_diag_izq": round(d_diag_izq, 1),
            "angulo_der": round(ang_2haces_der, 2),
        },
        "frontal": round(lg.distancia_en_rango(perfil_lista, 350.0, 10.0), 1),
        "perfil": [round(v, 1) for v in perfil_lista],
    }


# ==========================================
# 2. CLUSTERING
# ==========================================
def analizar_clusters(escaneos):
    # Se usa un barrido real, no el perfil promedio: el ABD depende del
    # orden y de la densidad de puntos crudos.
    if not escaneos:
        return []
    medio = escaneos[len(escaneos) // 2]
    scan = [(a, d) for a, d in medio["puntos"]]
    relevante = [p for p in scan if not (120.0 < p[0] < 240.0)]

    salida = []
    for c in lg.segmentar_clusters_abd(relevante):
        cx, cy = lg.centroide_xy_cluster(c)
        ext = c[-1][0] - c[0][0]
        if ext < 0:
            ext += 360.0
        d_min = min(p[1] for p in c)
        es_poste = lg.es_cluster_obstaculo(c)
        salida.append({
            "clasificacion": "POSTE" if es_poste else "muro/otro",
            "n_puntos": len(c),
            "extension_ang": round(ext, 2),
            "dist_min_mm": round(d_min, 1),
            "centroide_xy": [round(cx, 1), round(cy, 1)],
            "bearing_grados": round(math.degrees(math.atan2(cx, cy)), 2),
        })
    salida.sort(key=lambda s: s["dist_min_mm"])
    return salida


# ==========================================
# 3. HSV
# ==========================================
def _mascaras(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    m_rojo = cv2.inRange(hsv, ROJO_BAJO_1, ROJO_ALTO_1) | cv2.inRange(hsv, ROJO_BAJO_2, ROJO_ALTO_2)
    m_verde = cv2.inRange(hsv, VERDE_BAJO, VERDE_ALTO)
    m_rojo = cv2.morphologyEx(m_rojo, cv2.MORPH_OPEN, _KERNEL)
    m_verde = cv2.morphologyEx(m_verde, cv2.MORPH_OPEN, _KERNEL)
    return hsv, m_rojo, m_verde


def _stats_contorno(hsv, mascara):
    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contornos:
        return None
    c = max(contornos, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < AREA_MIN_DETECCION:
        return None
    x, y, w, h = cv2.boundingRect(c)
    relleno = np.zeros(mascara.shape, np.uint8)
    cv2.drawContours(relleno, [c], -1, 255, -1)
    pix = hsv[relleno > 0]
    if pix.size == 0:
        return None
    return {
        "area": float(area),
        "bbox": [int(x), int(y), int(w), int(h)],
        "cx": int(x + w / 2),
        "aspecto_h_sobre_w": round(h / max(1, w), 2),
        "h": [int(np.percentile(pix[:, 0], p)) for p in (2, 50, 98)],
        "s": [int(np.percentile(pix[:, 1], p)) for p in (2, 50, 98)],
        "v": [int(np.percentile(pix[:, 2], p)) for p in (2, 50, 98)],
    }


def analizar_hsv(carpeta):
    ruta = os.path.join(carpeta, "frames.jsonl")
    if not os.path.exists(ruta):
        return None

    detecciones = {"ROJO": [], "VERDE": []}
    n_frames = 0
    with open(ruta) as f:
        for linea in f:
            linea = linea.strip()
            if not linea:
                continue
            meta = json.loads(linea)
            frame = cv2.imread(os.path.join(carpeta, meta["archivo"]))
            if frame is None:
                continue
            n_frames += 1
            hsv, m_rojo, m_verde = _mascaras(frame)
            for color, mascara in (("ROJO", m_rojo), ("VERDE", m_verde)):
                st = _stats_contorno(hsv, mascara)
                if st:
                    detecciones[color].append(st)

    resumen = {"n_frames": n_frames}
    for color, lista in detecciones.items():
        if not lista:
            resumen[color] = {"detectado_en": 0}
            continue
        resumen[color] = {
            "detectado_en": len(lista),
            "tasa": round(len(lista) / max(1, n_frames), 2),
            "area_mediana": round(float(np.median([d["area"] for d in lista])), 0),
            "cx_mediana": int(np.median([d["cx"] for d in lista])),
            "aspecto_mediano": round(float(np.median([d["aspecto_h_sobre_w"] for d in lista])), 2),
            # Rango sugerido: p2 del percentil bajo y p98 del alto sobre
            # todos los frames, con un margen del 10% en S y V.
            "h_rango_observado": [min(d["h"][0] for d in lista), max(d["h"][2] for d in lista)],
            "s_rango_observado": [min(d["s"][0] for d in lista), max(d["s"][2] for d in lista)],
            "v_rango_observado": [min(d["v"][0] for d in lista), max(d["v"][2] for d in lista)],
        }
    return resumen


# ==========================================
# INFORME
# ==========================================
def revisar_pose(carpeta):
    nombre = os.path.basename(carpeta.rstrip(os.sep))
    print(f"\n{'=' * 70}\n{nombre}\n{'=' * 70}")

    pose = {}
    ruta_pose = os.path.join(carpeta, "pose.json")
    if os.path.exists(ruta_pose):
        with open(ruta_pose) as f:
            pose = json.load(f)
        print(f"  {pose.get('descripcion', '')}")

    escaneos = cargar_escaneos(carpeta)
    if not escaneos:
        print("  [-] Sin barridos de LiDAR en esta captura.")
        geo, clusters = None, []
    else:
        geo = analizar_geometria(escaneos)
        clusters = analizar_clusters(escaneos)

        print(f"\n  -- Geometria ({geo['n_barridos']} barridos promediados, "
              f"{geo['bins_con_dato']}/360 bins con dato) --")
        print(f"  frontal: {geo['frontal']:.0f}mm")
        for lado, clave in (("derecho", "muro_derecho"), ("izquierdo", "muro_izquierdo")):
            r = geo[clave]
            if r is None:
                print(f"  muro {lado}: sin recta ajustable (menos de 8 puntos utiles)")
                continue
            print(f"  muro {lado}: d_perp={r['distancia_perp_mm']:.0f}mm  "
                  f"angulo={r['angulo_grados']:+.2f} grados  RMS={r['rms_mm']:.1f}mm  "
                  f"({r['n_puntos']} pts, {r['n_descartados']} podados, "
                  f"largo visto {r['extension_mm']:.0f}mm)")
        if geo["ancho_carril_mm"]:
            print(f"  ancho de carril medido: {geo['ancho_carril_mm']:.0f}mm "
                  f"(reglamento: 1000mm)")
        d2 = geo["dos_haces"]
        print(f"  estimador de 2 haces (el que corre en carrera): "
              f"angulo_der={d2['angulo_der']:+.2f} grados")
        if geo["muro_derecho"]:
            delta = d2["angulo_der"] - geo["muro_derecho"]["angulo_grados"]
            print(f"    -> discrepancia contra la recta ajustada: {delta:+.2f} grados")
        print(f"    perp_der={d2['d_perp_der']:.0f} diag_der={d2['d_diag_der']:.0f} "
              f"perp_izq={d2['d_perp_izq']:.0f} diag_izq={d2['d_diag_izq']:.0f}")

        print(f"\n  -- Clustering ABD (barrido central) --")
        if not clusters:
            print("  (ningun cluster)")
        for c in clusters[:8]:
            print(f"  {c['clasificacion']:>10}  d={c['dist_min_mm']:6.0f}mm  "
                  f"bearing={c['bearing_grados']:+6.1f}  ext={c['extension_ang']:5.1f} grados  "
                  f"n={c['n_puntos']:3d}  xy=({c['centroide_xy'][0]:+.0f},"
                  f"{c['centroide_xy'][1]:+.0f})")

    hsv = analizar_hsv(carpeta)
    if hsv:
        print(f"\n  -- HSV ({hsv['n_frames']} frames) --")
        for color in ("ROJO", "VERDE"):
            r = hsv.get(color, {})
            if not r.get("detectado_en"):
                print(f"  {color}: NO detectado en ningun frame")
                continue
            print(f"  {color}: detectado en {r['detectado_en']}/{hsv['n_frames']} "
                  f"({r['tasa']:.0%})  area_med={r['area_mediana']:.0f}  "
                  f"cx_med={r['cx_mediana']}  h/w={r['aspecto_mediano']}")
            print(f"     H observado {r['h_rango_observado']}  "
                  f"S {r['s_rango_observado']}  V {r['v_rango_observado']}")

    return {"pose": nombre, "meta": pose, "geometria": geo,
            "clusters": clusters, "hsv": hsv}


def main():
    if len(sys.argv) < 2:
        print("Uso: python3 revisar_captura.py <carpeta_captura_o_carpeta_base>")
        sys.exit(1)

    base = sys.argv[1]
    if os.path.exists(os.path.join(base, "escaneos.jsonl")):
        carpetas = [base]
    else:
        carpetas = sorted(os.path.join(base, d) for d in os.listdir(base)
                          if os.path.isdir(os.path.join(base, d)))
    if not carpetas:
        print(f"[-] No hay capturas en {base}")
        sys.exit(1)

    informes = [revisar_pose(c) for c in carpetas]

    salida = os.path.join(base if len(carpetas) > 1 else os.path.dirname(base.rstrip(os.sep)),
                          "informe_captura.json")
    with open(salida, "w") as f:
        json.dump(informes, f, indent=2)
    print(f"\n[+] Informe completo (incluye los perfiles de 360 bins): {salida}")


if __name__ == "__main__":
    main()
