#!/bin/bash
# Lo que lanza el servicio: mira que ronda esta elegida y corre esa.
# Ver ronda.sh. Por defecto, la de obstaculos.
RONDA=$(cat /home/pi/ronda_activa 2>/dev/null || echo obstaculos)
echo "[servicio] $(date +%Y-%m-%d\ %H:%M:%S)  ronda elegida: $RONDA"
case "$RONDA" in
    abierta)    exec /bin/bash /home/pi/correr_abierta.sh  "$@" ;;
    obstaculos) exec /bin/bash /home/pi/correr_completa.sh "$@" ;;
    *) echo "[-] '$RONDA' no es una ronda valida. Corre ./ronda.sh"; exit 1 ;;
esac
