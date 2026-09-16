"""Corrida por boton con limite temporal y parada exclusivamente por SIGINT."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--segundos', type=float, default=20)
    parser.add_argument('--espera-boton', type=float, default=120)
    parser.add_argument('--log', required=True)
    parser.add_argument('--modo', choices=('sombra', 'activo'), default='sombra')
    parser.add_argument('--memoria', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.segundos <= 60 or not 1 <= args.espera_boton <= 300:
        parser.error('Limites fuera del intervalo permitido')
    ruta = Path(args.log).resolve()
    if ruta.exists():
        parser.error('El registro ya existe')
    entorno = dict(os.environ)
    entorno.pop('WRO_ARRANQUE_AUTO', None)
    entorno.update(WRO_RED='1', WRO_RED_HZ='8', WRO_DOS_PILARES=args.modo,
                   WRO_DOS_MEMORIA='1' if args.memoria else '0')
    interrumpido = [False]
    signal.signal(signal.SIGINT, lambda *_: interrumpido.__setitem__(0, True))
    proceso = None
    inicio = None
    espera = time.monotonic()
    try:
        with ruta.open('x', buffering=1) as salida:
            proceso = subprocess.Popen([sys.executable, '-u', 'ronda_camara.py'],
                env=entorno, stdout=salida, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True)
            Path(str(ruta) + '.pid').write_text(str(proceso.pid))
            print('PID ronda:', proceso.pid, flush=True)
            with ruta.open() as entrada:
                while proceso.poll() is None and not interrumpido[0]:
                    for linea in entrada.readlines():
                        if '[START] Boton detectado!' in linea:
                            inicio = time.monotonic()
                            print('Boton detectado; limite %.1f s' % args.segundos, flush=True)
                    ahora = time.monotonic()
                    if inicio is not None and ahora - inicio >= args.segundos:
                        print('Limite de corrida: SIGINT', flush=True)
                        break
                    if inicio is None and ahora - espera >= args.espera_boton:
                        print('Sin pulsacion: SIGINT', flush=True)
                        break
                    time.sleep(.05)
    finally:
        if proceso is not None and proceso.poll() is None:
            proceso.send_signal(signal.SIGINT)
            try:
                proceso.wait(timeout=8)
            except subprocess.TimeoutExpired:
                print('ATENCION: el proceso no termino con SIGINT. Verificar detencion fisica.', flush=True)
                # No se usa terminate(), kill() ni SIGTERM como sustituto.
                return 2
    if proceso is None:
        return 1
    print('Fin de proceso:', proceso.returncode, flush=True)
    return proceso.returncode


if __name__ == '__main__':
    sys.exit(main())
