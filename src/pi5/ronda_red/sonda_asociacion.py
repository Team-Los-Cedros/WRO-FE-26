"""Camara, red y LiDAR con el coche parado. No importa el enlace de la Pico."""
import json
from pathlib import Path
import threading
import time

from camara_driver import CamaraDriver
from lidar_driver import LidarDriver
from lidar_geometria import ProcesadorLidar, centroide_xy_cluster, ancho_cluster, bins_de_clusters
import lidar_mascara
from dos_pilares import asociar
from navegacion import Navegador
import vision_red


def main():
    corriendo = threading.Event()
    corriendo.set()
    camara, lidar = CamaraDriver(), LidarDriver()
    proc = ProcesadorLidar()
    nav = Navegador(proc)
    lock = threading.Lock()
    ultimo = [None]
    registros = []

    def recibir(scan):
        with lock:
            ultimo[0] = (time.monotonic(), scan)

    def frame(imagen):
        vision_red.publicar(imagen, None, None, 0)

    hilos = [threading.Thread(target=camara.hilo_captura, args=(corriendo.is_set, frame), daemon=True),
             threading.Thread(target=lidar.hilo_lectura, args=(corriendo.is_set, recibir), daemon=True)]
    try:
        if not vision_red.arrancar():
            raise RuntimeError('No se pudo iniciar la red')
        for hilo in hilos:
            hilo.start()
        fin = time.monotonic() + 10
        secuencia = -1
        while time.monotonic() < fin:
            time.sleep(.1)
            cuadro = vision_red.ultimo_cuadro()
            with lock:
                barrido = ultimo[0]
            if cuadro is None or barrido is None or cuadro.secuencia == secuencia:
                continue
            if time.monotonic() - min(cuadro.captura_mono, barrido[0]) > .3:
                continue
            secuencia = cuadro.secuencia
            med = lidar_mascara.aplicar(proc.procesar(barrido[1]))
            clusters = [(*centroide_xy_cluster(c), ancho_cluster(c)) for c in med.clusters_estrechos]
            pares = asociar(cuadro.cajas, clusters)
            bins_ = bins_de_clusters(med.clusters_estrechos)
            import math
            datos = [{'color': c.color, 'x': x, 'y': y, 'alto_px': c.alto,
                      'en_seccion': nav._en_mi_seccion(med, x, y, math.degrees(math.atan2(x, y)), bins_)}
                     for c, x, y in pares]
            registros.append({'t_unix': time.time(), 'secuencia': secuencia,
                              'cajas': [c.__dict__ for c in cuadro.cajas],
                              'clusters': clusters, 'asociaciones': datos,
                              'scan': barrido[1]})
            print(json.dumps(datos), flush=True)
    finally:
        corriendo.clear()
        for hilo in hilos:
            if hilo.ident is not None:
                hilo.join(timeout=2)
        camara.cerrar()
        lidar.cerrar()
        vision_red.parar()
        Path('asociacion_estatica.json').write_text(json.dumps(registros), encoding='utf-8')


if __name__ == '__main__':
    main()
