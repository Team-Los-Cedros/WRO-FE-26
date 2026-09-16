#!/bin/bash
# PRUEBA ABIERTA (Open Challenge): 3 vueltas y parar.
#
#   correr_abierta.sh [panel]
#
# Solo se diferencia de la ronda de obstaculos en DOS cosas, y ninguna es
# codigo de control nuevo:
#
#   1. No corre parqueo.py. En la abierta el robot arranca en un tramo
#      cualquiera de la pista, no dentro del estacionamiento.
#   2. WRO_SIN_PILARES=1. La camara sigue funcionando -- hace falta para
#      las LINEAS del suelo, que son las que cuentan las vueltas -- pero
#      su color no llega a la maquina de estados. En la abierta no hay
#      pilares, asi que cualquier deteccion de color es un falso
#      positivo, y ahi un falso positivo no es inofensivo: la FSM se
#      compromete, abre, se desvia, y puede acabar rozando un muro por
#      esquivar algo que no existe.
#
# El control de carril es EXACTAMENTE el mismo que ya corre en la ronda
# de obstaculos. No hay ninguna ruta de codigo nueva.
#
# NO TERMINA POR RELOJ. Acaba cuando el contador de lineas llega a 12
# (3 vueltas) y la FSM para en el tramo del que salio. El `timeout` es
# una red por si algo se cuelga.
#
# OJO: aqui NO se pone WRO_LIDAR_CALIENTE. Esa variable acorta la espera
# de arranque del LiDAR y solo vale cuando otro proceso acaba de dejarlo
# girando, que es el caso de la ronda de obstaculos (viene del parqueo).
# Aqui el LiDAR arranca de cero y necesita su tiempo entero.
cd /home/pi/ronda_curvas || exit 1
LIMITE="${WRO_LIMITE:-600}"
[ "$1" = "panel" ] && export WRO_PANEL=1

# El boton abre la ronda, y mientras espera el LED de la Pico parpadea.
python3 -u esperar_boton.py abierta || { echo "[-] arranque cancelado"; exit 1; }
echo
echo "[ABIERTA] $(date +%H:%M:%S)  3 vueltas, sin pilares"
WRO_SIN_PILARES=1 WRO_ARRANQUE_AUTO=1 timeout -s INT "$LIMITE" python3 -u ronda_camara.py
echo "[fin] $(date +%H:%M:%S)  total ${SECONDS}s  |  $(ls -t logs/*.csv 2>/dev/null | head -1)"
