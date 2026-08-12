# Captura sincronizada en la PISTA REAL, con los postes y los muros de
# parqueo puestos. Es la unica medicion que no se puede sustituir por
# nada hecho en casa: sin carriles de 1000mm ni muros de 100mm no se
# puede calibrar HSV con la iluminacion real ni validar que el LiDAR
# separa poste de pared a las distancias que de verdad ocurren.
#
# Graba a disco, con un reloj comun para las tres fuentes:
#   escaneos.jsonl   un barrido crudo completo del LiDAR por linea
#                    {"t":..., "heading":..., "puntos":[[ang,dist],...]}
#   frames/*.png     frame BGR 320x240 tal cual lo ve vision.py
#   frames.jsonl     {"t":..., "archivo":..., "heading":...}
#   pose.json        metadatos de la pose (lo que escribiste al lanzarlo)
#
# NO mueve el robot: el robot se coloca a mano en cada pose y se captura
# quieto. Una captura en movimiento no sirve para calibrar porque no se
# sabe donde estaba el robot en cada barrido.
#
# Uso (en la Pi, con el robot ya colocado en la pose):
#   python3 capturar_pista.py <nombre_pose> [segundos]
#   python3 capturar_pista.py --checklist     # imprime las poses a tomar
#
# Ejemplo:
#   python3 capturar_pista.py poste_rojo_600mm 6
import json
import os
import sys
import time
import threading
from datetime import datetime

import cv2

from lidar_driver import LidarDriver
from enlace_pico import EnlacePico
from camara_driver import CamaraDriver

SEGUNDOS_POR_DEFECTO = 6.0
PERIODO_FRAME        = 0.4       # s entre frames guardados (~2.5 fps)
CARPETA_BASE         = "capturas_pista"

# Poses minimas para desbloquear la calibracion. El nombre importa: el
# analisis offline (revisar_captura.py) agrupa por prefijo.
CHECKLIST = [
    ("carril_centrado",
     "Robot centrado en un tramo recto, mirando a lo largo del carril, sin postes "
     "a la vista. Valida: extraccion de rectas de pared, ancho de carril medido, "
     "cero del angulo de muro."),
    ("carril_sesgado_15",
     "Mismo tramo, pero girado ~15 grados respecto al muro. Valida que angulo_muro "
     "de lidar_geometria.py devuelve algo cercano a 15 y no a otra cosa."),
    ("carril_pegado_izq",
     "Pegado al muro izquierdo (~150mm), recto. Valida el modo Inercial y "
     "EMERGENCIA_LATERAL."),
    ("esquina_interior",
     "Entrando a una esquina, con el muro interior en el sector frontal. Valida que "
     "la esquina NO se clasifica como poste."),
    ("poste_rojo_900", "Poste ROJO centrado al frente, a 900mm medidos con cinta."),
    ("poste_rojo_600", "Poste ROJO centrado al frente, a 600mm."),
    ("poste_rojo_300", "Poste ROJO centrado al frente, a 300mm."),
    ("poste_verde_900", "Poste VERDE centrado al frente, a 900mm."),
    ("poste_verde_600", "Poste VERDE centrado al frente, a 600mm."),
    ("poste_verde_300", "Poste VERDE centrado al frente, a 300mm."),
    # Las tres poses laterales son las que permiten despejar el offset de
    # la camara SIN regla: cada una da un par (cx de la camara, xy del
    # cluster del LiDAR) y con varias se ajusta focal + offset por
    # minimos cuadrados. Ver MEDICIONES.md seccion 2, metodo B. Hacen
    # falta las tres, con desplazamientos laterales bien distintos.
    ("poste_rojo_600_der250",
     "Poste ROJO a 600mm de frente, desplazado ~250mm a la DERECHA del eje del robot."),
    ("poste_rojo_450_der120",
     "Poste ROJO a 450mm de frente, desplazado ~120mm a la DERECHA."),
    ("poste_verde_450_izq200",
     "Poste VERDE a 450mm de frente, desplazado ~200mm a la IZQUIERDA."),
    ("dos_postes",
     "Un ROJO y un VERDE en el campo a la vez, uno detras del otro. Valida que el "
     "clustering entrega dos clusters separados y que la histeresis de vision.py "
     "no oscila entre colores."),
    ("parqueo_lejos",
     "Robot en el carril a ~800mm de los dos muros magenta, mirando hacia ellos de "
     "frente. Valida la deteccion de la firma de parqueo a distancia."),
    ("parqueo_al_lado",
     "Robot en el carril justo al lado del parqueo, paralelo al muro exterior. "
     "Valida el escalon lateral de 200mm y la separacion de 333mm."),
    ("parqueo_dentro",
     "Robot YA estacionado a mano, bien centrado y paralelo. Es la pose objetivo: "
     "de aqui sale la firma LiDAR exacta que la maniobra tiene que alcanzar."),
    ("contraluz",
     "Cualquier pose con postes, pero orientado hacia la peor fuente de luz del "
     "salon (ventana, reflector). Es el caso que rompe los umbrales HSV."),
]


class CapturaPista:
    def __init__(self, nombre_pose, segundos):
        self.nombre_pose = nombre_pose
        self.segundos    = segundos
        self.corriendo   = True
        self.t0          = None

        marca = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.carpeta = os.path.join(CARPETA_BASE, f"{marca}_{nombre_pose}")
        os.makedirs(os.path.join(self.carpeta, "frames"), exist_ok=True)

        self.enlace = EnlacePico()
        self.enlace.enviar(0, 0.0)          # traccion apagada durante toda la captura

        self._lock          = threading.Lock()
        self._f_escaneos    = open(os.path.join(self.carpeta, "escaneos.jsonl"), "w")
        self._f_frames      = open(os.path.join(self.carpeta, "frames.jsonl"), "w")
        self._n_escaneos    = 0
        self._n_frames      = 0
        self._t_ultimo_frame = 0.0

    def _ahora(self):
        return round(time.time() - self.t0, 4)

    def al_barrido(self, scan):
        if not self.corriendo:
            return
        fila = {
            "t": self._ahora(),
            "heading": round(self.enlace.heading(), 2),
            "puntos": [[round(a, 2), round(d, 1)] for a, d in scan],
        }
        with self._lock:
            self._f_escaneos.write(json.dumps(fila) + "\n")
            self._n_escaneos += 1

    def al_frame(self, frame):
        if not self.corriendo:
            return
        ahora = time.time()
        if ahora - self._t_ultimo_frame < PERIODO_FRAME:
            return
        self._t_ultimo_frame = ahora

        t = self._ahora()
        nombre = f"{t:08.3f}.png".replace(".", "_", 1)
        ruta = os.path.join(self.carpeta, "frames", nombre)
        # Se guarda el frame CRUDO, sin mascaras: los umbrales HSV se
        # reajustan offline y hay que poder reprocesar el mismo pixel.
        cv2.imwrite(ruta, frame)
        with self._lock:
            self._f_frames.write(json.dumps({
                "t": t, "archivo": os.path.join("frames", nombre),
                "heading": round(self.enlace.heading(), 2),
            }) + "\n")
            self._n_frames += 1

    def ejecutar(self, descripcion):
        camara = CamaraDriver()
        threading.Thread(target=camara.hilo_captura,
                         args=(lambda: self.corriendo, self.al_frame),
                         daemon=True).start()

        lidar = LidarDriver()
        self.t0 = time.time()
        threading.Thread(target=lidar.hilo_lectura,
                         args=(lambda: self.corriendo, self.al_barrido),
                         daemon=True).start()

        print(f"\n[REC] Capturando '{self.nombre_pose}' durante {self.segundos:.0f}s. "
              f"NO muevas el robot.")
        t_fin = time.time() + self.segundos + 2.5   # +2.5s: arranque del LiDAR
        try:
            while time.time() < t_fin:
                time.sleep(0.25)
                print(f"    barridos={self._n_escaneos}  frames={self._n_frames}  "
                      f"heading={self.enlace.heading():+.1f}", end="\r")
        except KeyboardInterrupt:
            print("\n[!] Interrumpido.")

        self.corriendo = False
        time.sleep(0.3)
        lidar.cerrar()
        self.enlace.cerrar()

        with self._lock:
            self._f_escaneos.close()
            self._f_frames.close()

        with open(os.path.join(self.carpeta, "pose.json"), "w") as f:
            json.dump({
                "pose": self.nombre_pose,
                "descripcion": descripcion,
                "fecha": datetime.now().isoformat(timespec="seconds"),
                "segundos": self.segundos,
                "barridos": self._n_escaneos,
                "frames": self._n_frames,
            }, f, indent=2, ensure_ascii=False)

        print(f"\n[OK] {self._n_escaneos} barridos y {self._n_frames} frames en "
              f"{self.carpeta}")
        if self._n_escaneos == 0:
            print("[-] CERO barridos: revisa que /dev/ttyUSB0 exista y que el "
                  "LiDAR este girando.")
        if self._n_frames == 0:
            print("[-] CERO frames: revisa picamera2 (ver INSTALACION.md paso 3).")


def imprimir_checklist():
    print("Poses a capturar en la pista real (una corrida del script por pose).")
    print("Coloca el robot, mide con cinta lo que diga la pose, y lanza:\n")
    for nombre, desc in CHECKLIST:
        print(f"  python3 capturar_pista.py {nombre}")
        print(f"      {desc}\n")
    print("Al terminar todas, comprime y baja la carpeta completa:")
    print("  tar czf capturas_pista.tar.gz capturas_pista/")
    print("  # y desde la laptop:")
    print("  scp pi@192.168.0.107:~/capturas_pista.tar.gz .")


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("--checklist", "-c", "--help", "-h"):
        imprimir_checklist()
        return

    nombre = sys.argv[1]
    segundos = float(sys.argv[2]) if len(sys.argv) > 2 else SEGUNDOS_POR_DEFECTO
    descripcion = dict(CHECKLIST).get(nombre, "(pose libre, no esta en el checklist)")

    print(f"[POSE] {nombre}")
    print(f"       {descripcion}")

    try:
        captura = CapturaPista(nombre, segundos)
    except Exception as e:
        print(f"[-] No se pudo abrir el enlace con la Pico: {e}")
        sys.exit(1)
    captura.ejecutar(descripcion)


if __name__ == "__main__":
    main()
