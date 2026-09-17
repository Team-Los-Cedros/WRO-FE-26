#!/bin/bash
# Lo que lanza el servicio: mira que ronda esta elegida y corre esa.
#
# La ronda se cambia con ./ronda.sh, que escribe una palabra en
# /home/pi/ronda_activa. Por defecto, la de obstaculos.
#
# Los dos programas esperan el boton de GP21 por su cuenta antes de mover
# nada, asi que este script no toca el GPIO: solo elige cual lanzar.
set -uo pipefail

RONDA=$(cat /home/pi/ronda_activa 2>/dev/null || echo obstaculos)
LOGDIR=/home/pi/logs
mkdir -p "$LOGDIR"

echo "[servicio] $(date '+%Y-%m-%d %H:%M:%S')  ronda elegida: $RONDA"

case "$RONDA" in
    abierta)
        echo "[servicio] ABIERTA -> /home/pi/prueba_abierta.py"
        cd /home/pi || exit 1
        exec python3 -u prueba_abierta.py
        ;;
    obstaculos)
        echo "[servicio] OBSTACULOS -> ~/ronda_unificada_20260916/ronda_unificada.py"
        cd /home/pi/ronda_unificada_20260916 || exit 1
        exec python3 -u ronda_unificada.py
        ;;
    *)
        echo "[-] '$RONDA' no es una ronda valida."
        echo "    Corre ./ronda.sh abierta   u   ./ronda.sh obstaculos"
        exit 1
        ;;
esac
