# Ronda Cerrada con la CAMARA EN MASTIL TRASERO - punto de entrada.
# Mismo cableado que ronda_cerrada/ronda_cerrada.py; las diferencias
# estan todas en los modulos que importa y en una linea de al_barrido:
#
#   camara_driver.py  el de ronda_camara/, sin rotar 180 (la camara
#                     quedo derecha en el mastil)
#   lidar_mascara.py  tapa los 65 grados que el mastil y el soporte del
#                     ultrasonido le roban al LiDAR (135-199, remedido el
#                     10-09). La distancia trasera NO se reconstruye del
#                     LiDAR -- ahi no queda nada que leer -- sino del
#                     HC-SR04 que la Pico publica en el campo US.
#   navegacion.py     maquina de estados con la maniobra geometrica de
#                     pilar (ver DISENO_CURVAS.md)
#   geometria_evasion.py  geometria pura de la envolvente
#   sentido_vuelta.py     horario / antihorario a partir del yaw
#
# DESPLIEGUE: ademas de los modulos de siempre hay que copiar
# comun/geometria_robot.py, que es de donde salen la batalla, el ancho y
# los offsets de sensores. Sin el, el arranque muere en el import.
#
# Este script solo conecta las piezas:
#   camara_driver.py  hilo de adquisicion de frames (Pi Camera Module 3)
#   vision.py         procesa cada frame: HSV + histeresis
#   lidar_driver.py   hilo del RPLIDAR C1, entrega el barrido crudo
#   lidar_geometria.py  interpreta el barrido: paredes + clustering ABD
#   tracker.py         posicion del poste activo (lo usa navegacion)
#   navegacion.py      maquina de estados, decide (velocidad, angulo)
#   enlace_pico.py      serial con la Pico 2 (consignas + IMU)
#
# Secuencia: armar hilos -> esperar boton GP21 -> fijar cero IMU ->
# arrancar LiDAR -> por cada barrido navegacion decide y se manda a la
# Pico. El bucle principal solo vigila que el LiDAR siga vivo y apaga.
import os
import sys
import time
import signal
import threading

import RPi.GPIO as GPIO

import vision
import navegacion
import lidar_mascara
import panel_web
from camara_driver import CamaraDriver
from lidar_driver import LidarDriver
from lidar_geometria import ProcesadorLidar
from enlace_pico import EnlacePico
from registro_metricas import RegistroMetricas

PIN_BOTON = 21

# Si el LiDAR no entrega barridos en este tiempo con el robot en marcha
# se corta la traccion: sin percepcion no se navega
WATCHDOG_LIDAR = 0.8

corriendo = True
enlace       = None
lidar_driver = None
lidar_geo    = None
navegador    = None
registro     = None

_t_ultimo_barrido = 0.0
_apagando = False

# Una deteccion de camara mas vieja que esto no se usa. La camara entrega
# a ~23 fps (43 ms), asi que 0.4 s son diez frames perdidos seguidos: no
# es un hipo, es que el hilo se cayo.
EDAD_MAX_CAMARA = 0.4

# Panel de telemetria en vivo (LiDAR + camara + estado) en el puerto 8080.
# Apagado por defecto: es CPU y un puerto abierto que solo hacen falta
# depurando. Se enciende con WRO_PANEL=1.
PANEL = os.environ.get("WRO_PANEL") == "1"

# PRUEBA ABIERTA: 3 vueltas y parar, sin pilares y sin estacionamiento.
#
# En la abierta NO hay pilares en la pista, asi que cualquier deteccion de
# color es un falso positivo -- y un falso positivo ahi no es inofensivo:
# la FSM se compromete, abre, se desvia y puede terminar rozando un muro
# por esquivar algo que no existe. Con esto la vision sigue corriendo
# (hace falta para las LINEAS del suelo, que son las que cuentan las
# vueltas) pero su color no llega a la maquina de estados.
#
# Es un interruptor, no codigo nuevo: la ronda abierta usa exactamente el
# mismo control de carril que la de obstaculos, que es el que esta
# probado.
SIN_PILARES = os.environ.get("WRO_SIN_PILARES") == "1"

_avisos = set()


def _avisar_una_vez(clave, mensaje):
    # El callback corre 10 veces por segundo: un aviso por ciclo llenaria
    # la consola y taparia justo lo que hay que leer.
    if clave not in _avisos:
        _avisos.add(clave)
        print(mensaje)


def apagar_sistema(sig=None, frame=None):
    global corriendo, _apagando
    if _apagando:                 # doble Ctrl+C no debe reentrar aca
        return
    _apagando = True
    print("\n[!] Deteniendo sistema de forma segura...")
    corriendo = False
    time.sleep(0.2)
    if enlace:
        enlace.cerrar()
    if lidar_driver:
        lidar_driver.cerrar()
    if registro:
        registro.cerrar()
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f"[-] GPIO.cleanup() fallo (ignorado): {e}")
    sys.exit(0)


def al_barrido(scan):
    # Callback del hilo LiDAR: un ciclo de decision por barrido completo
    global _t_ultimo_barrido
    medicion = lidar_geo.procesar(scan)
    # Corrige los campos traseros antes de que los vea nadie: el mastil y
    # el soporte del ultrasonido ciegan los grados 135-199 (remedido el
    # 10-09) y, como los sectores toman el MINIMO del rango, dejaban
    # `trasera` clavada en ~47mm y las dos diagonales traseras tambien.
    # Con eso, RETROCESO salia en su primer ciclo por "obstaculo trasero"
    # pase lo que pase -- 447 episodios de 1 ciclo en las corridas del
    # 09-09 -- y el giro en reversa apuntaba siempre al mismo lado.
    # Autochequeo de la mascara con el primer barrido de la corrida.
    # `diagnosticar()` existia desde el principio y no lo llamaba nadie:
    # son constantes calibradas contra una pieza fisica y si el mastil se
    # mueve, la mascara tapa pista buena y deja pasar estructura sin que
    # nada avise. Fue lo que dejo el arco ciego real (135-199) descrito
    # como 163-189 durante toda la sesion del 09-09.
    _autochequeo_mascara(medicion.perfil)
    lidar_mascara.aplicar(medicion)

    # La trasera del LiDAR ya no existe (ver lidar_mascara.distancia_trasera):
    # con este montaje no queda ni un grado util a menos de 46 grados del
    # eje trasero. La medida buena es el HC-SR04 que la Pico publica en el
    # campo US, ya descontado el trecho del sensor a la culata.
    # Sin ultrasonido valido se queda el SIN_DATO del LiDAR, que la FSM
    # lee como "nada confirmado detras" -- que es la verdad, no un permiso.
    us_mm = enlace.ultrasonido_mm()
    if us_mm is not None:
        medicion.trasera = us_mm
    _t_ultimo_barrido = medicion.timestamp

    heading = enlace.heading()
    # Una sola lectura por ciclo: el hilo de camara la actualiza por su
    # cuenta, y si se consulta otra vez para el log el CSV podria guardar
    # un color distinto del que realmente uso la FSM en esta decision.
    # cx_cam es la posicion horizontal del poste en el frame; la usa el
    # apareo por rumbo de navegacion para decidir CUAL de los clusters
    # del LiDAR es el que tiene ese color (ver APAREO COLOR <-> CLUSTER).
    color_cam, cx_cam = vision.get_deteccion()
    if SIN_PILARES:
        color_cam, cx_cam = None, None
    # Watchdog de camara y de IMU, los dos que faltaban. El hilo de
    # camara puede morir en silencio dejando un color enganchado, y
    # `enlace.heading()` devuelve el ultimo yaw para siempre si la Pico
    # deja de reportar. Una deteccion vieja es peor que ninguna: la FSM
    # se compromete con un pilar que ya nadie ve.
    edad_cam = vision.edad_deteccion()
    if edad_cam is None or edad_cam > EDAD_MAX_CAMARA:
        color_cam, cx_cam = None, None
        _avisar_una_vez("camara", "[-] Camara sin frames (%s). Se navega solo con LiDAR."
                        % ("nunca arranco" if edad_cam is None else "%.1fs" % edad_cam))
    if not enlace.heading_valido():
        _avisar_una_vez("imu", "[-] IMU sin telemetria: el conteo de vueltas y el "
                               "sentido de giro dejan de ser fiables.")
    # Color del piso desde la Pico (TCS3472). Es None mientras no haya
    # sensor conectado o el firmware reporte SIN_SENSOR/PISTA.
    color_piso = enlace.color_piso()
    linea_vista = vision.linea_a_la_vista()
    consigna = navegador.procesar(medicion, color_cam, heading, cx_cam=cx_cam,
                                  linea_cam=linea_vista,
                                  color_piso=color_piso)
    if consigna is None:          # carrera terminada (parqueo o timeout)
        apagar_sistema()
        return
    velocidad, angulo = consigna
    if PANEL:
        panel_web.publicar(medicion, velocidad, angulo, navegador,
                           {"us_mm": us_mm, "color_piso": color_piso,
                            "edad_cam": None if edad_cam is None else round(edad_cam, 2)})
    if registro:
        error_lateral = medicion.izquierda - medicion.derecha
        trk = navegador.tracker
        registro.registrar(fase=navegador.fase, estado=navegador.estado,
                            heading=f"{heading:.2f}", error_lateral=f"{error_lateral:.1f}",
                            angulo=f"{angulo:.2f}", velocidad=velocidad,
                            frontal=f"{medicion.frontal:.0f}",
                            frontal_muro=f"{medicion.frontal_muro:.0f}",
                            izquierda=f"{medicion.izquierda:.0f}",
                            derecha=f"{medicion.derecha:.0f}",
                            trasera=f"{medicion.trasera:.0f}",
                            color_cam=color_cam or "",
                            trk_activo=int(trk.activo),
                            trk_color=trk.color or "",
                            trk_x=f"{trk.x:.0f}", trk_y=f"{trk.y:.0f}",
                            angulo_muro=f"{medicion.angulo_muro:.2f}",
                            muro_ok=int(medicion.muro_valido),
                            ang_muro_viejo=f"{medicion.angulo_muro_viejo:.2f}",
                            trk_cands=trk.n_candidatos,
                            trk_puerta=trk.n_en_puerta,
                            trk_no_asoc=trk.motivo_no_asoc,
                            fuente_lado=navegador.fuente_lado,
                            color_piso=color_piso or "",
                            paso_ok=int(navegador._paso_validado),
                            lado_esquina=f"{navegador._preferencia_esquina(medicion) or 0:+.0f}",
                            us_mm=("" if us_mm is None else f"{us_mm:.0f}"),
                            rama_evasion=navegador.rama_evasion,
                            rumbo_poste=f"{navegador.rumbo_poste_cam:.1f}",
                            cmd_deseado=f"{navegador.cmd_deseado:.2f}",
                            seguridad=navegador.seguridad,
                            n_seguros=navegador.n_seguros,
                            n_compatibles=navegador.n_compatibles,
                            obj_id=trk.id if trk.activo else 0,
                            obj_lado=trk.s_lado if trk.activo else 0,
                            obj_sigma=f"{trk.sigma:.0f}",
                            obj_progreso=f"{trk.progreso:.0f}",
                            separacion_lat=f"{navegador.separacion_lat:.0f}",
                            holgura_pilar=f"{navegador.holgura_pilar:.0f}",
                            radio_maniobra=f"{navegador.radio_maniobra:.0f}",
                            sentido=navegador.sentido.sentido)
    enlace.enviar(*consigna)


_barridos_vistos = [0]

# El PRIMER barrido tras arrancar el C1 sale incompleto: el driver corta
# en el wrap-around del angulo, asi que hasta que no da una vuelta entera
# faltan grados. Comprobar la mascara ahi da una falsa alarma
# ("ningun grado ve estructura") y ademas deja la firma de parqueo con
# un sector sin datos -- las dos cosas pasaron en la corrida de las
# 10:01. Se espera a tener un barrido completo de verdad.
BARRIDO_PARA_CHEQUEO = 5


def _autochequeo_mascara(perfil):
    if "mascara" in _avisos:
        return
    _barridos_vistos[0] += 1
    if _barridos_vistos[0] < BARRIDO_PARA_CHEQUEO:
        return
    _avisos.add("mascara")
    ok, msj = lidar_mascara.diagnosticar(perfil)
    print(("[+] " if ok else "[!] MASCARA DEL LIDAR: ") + msj)


def _comprobar_ultrasonido():
    """Grita si la unica medida trasera que queda no esta viva.

    Con este montaje el LiDAR no ve hacia atras (arco ciego 135-199, ver
    lidar_mascara), asi que el HC-SR04 de la Pico es la UNICA fuente de
    la distancia trasera. Si no responde, el estado RETROCESO se queda
    sin su condicion de parada y hay que saberlo antes de arrancar, no
    despues de una corrida perdida.
    """
    t0 = time.time()
    while time.time() - t0 < 2.0:
        us = enlace.ultrasonido_mm()
        if us is not None:
            print(f"[+] Ultrasonido trasero vivo: {us:.0f} mm de hueco.")
            return True
        time.sleep(0.1)
    print("[!] ULTRASONIDO TRASERO MUDO. El LiDAR no ve hacia atras con este\n"
          "    mastil, asi que el retroceso se guiara solo por reloj. Revisa el\n"
          "    cableado del HC-SR04 y el firmware de la Pico antes de correr.")
    return False


def preparar_gpio():
    GPIO.setmode(GPIO.BCM)
    try:
        GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    except Exception as e:
        print(f"[!] GPIO ocupado, liberando y reintentando... ({e})")
        try:
            GPIO.cleanup()
        except Exception:
            pass
        time.sleep(0.3)
        # GPIO.cleanup() borra tambien el modo de numeracion, asi que hay
        # que volver a fijarlo ANTES de reintentar el setup. Sin esto el
        # reintento muere con "Please set pin numbering mode" y tapa el
        # error de verdad, que era el GPIO ocupado por otro proceso
        # (normalmente wro.service, que relanza ronda_camara.py solo).
        GPIO.setmode(GPIO.BCM)
        try:
            GPIO.setup(PIN_BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        except Exception:
            # Si sigue ocupado es OTRO proceso, y limpiar no lo arregla.
            # Casi siempre es `wro.service`, que relanza ronda_camara.py
            # al encender la Pi y se queda con el GPIO. El traceback de
            # lgpio no lo dice; esto si.
            print("")
            print("[-] EL GPIO ESTA OCUPADO POR OTRO PROCESO.")
            print("    Casi seguro es el servicio de arranque automatico:")
            print("")
            print("      sudo systemctl stop wro.service")
            print("")
            print("    (y `sudo systemctl start wro.service` al terminar).")
            sys.exit(1)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, apagar_sistema)
    preparar_gpio()

    # 1. Camara primero, necesita ~1s para estabilizar la exposicion
    camara = CamaraDriver()
    threading.Thread(target=camara.hilo_captura,
                     args=(lambda: corriendo, vision.procesar_frame), daemon=True).start()

    # 2. Enlace con la Pico, direccion centrada mientras se espera
    try:
        enlace = EnlacePico()
        print("[+] Conexion serial establecida con Raspberry Pi Pico 2.")
    except Exception as e:
        print(f"[-] Error conectando a la Pi Pico 2: {e}")
        sys.exit(1)
    enlace.enviar(0, 0.0)

    # Arranque sin boton, SOLO para depuracion remota por SSH
    # (WRO_ARRANQUE_AUTO=1). En competencia no se usa: el reglamento pide
    # que la carrera la inicie una accion fisica sobre el robot. La
    # cuenta atras existe para que quede una ventana visible en la que
    # apartar la mano o cortar el proceso antes de que el motor arranque.
    if os.environ.get("WRO_ARRANQUE_AUTO") == "1":
        print("\n[LISTO] ARRANQUE AUTOMATICO (depuracion, sin boton).")
        for queda in (3, 2, 1):
            print(f"[LISTO] arrancando en {queda}...")
            enlace.enviar(0, 0.0)
            time.sleep(1.0)
        print("\n[START] Arranque automatico! Iniciando carrera con obstaculos...")
    else:
        print("\n[LISTO] SISTEMA LISTO (RONDA CON OBSTACULOS). "
              "Coloca el robot y presiona el Boton (GP21)...")
        while GPIO.input(PIN_BOTON) == GPIO.HIGH:
            enlace.enviar(0, 0.0)
            time.sleep(0.05)
        print("\n[START] Boton detectado! Iniciando carrera con obstaculos...")
    enlace.fijar_cero()           # el yaw de este instante es el 0 de carrera
    registro = RegistroMetricas("ronda_camara")

    # 3. LiDAR y navegacion. El LiDAR arranca despues del boton para que
    #    su primer barrido capture la firma de pared del punto de partida
    lidar_driver = LidarDriver()
    lidar_geo    = ProcesadorLidar()
    _comprobar_ultrasonido()
    if PANEL:
        p = panel_web.arrancar()
        if p:
            print("[+] Panel de telemetria en http://<ip-de-la-pi>:%d" % p)
    navegador    = navegacion.Navegador(control_sector=lidar_geo)
    _t_ultimo_barrido = time.time()
    threading.Thread(target=lidar_driver.hilo_lectura,
                     args=(lambda: corriendo, al_barrido), daemon=True).start()

    # 4. Vigilancia: si la percepcion muere el robot se detiene
    while corriendo:
        sin_barridos = time.time() - _t_ultimo_barrido
        if sin_barridos > WATCHDOG_LIDAR and navegador.fase in ("CARRERA", "PARQUEO"):
            enlace.enviar(0, 0.0)
            if sin_barridos > 5.0:
                print("[-] LiDAR sin datos por 5s. Abortando carrera.")
                apagar_sistema()
        time.sleep(0.1)
