#!/bin/bash
# Ronda de obstaculos COMPLETA: salir del estacionamiento, correr, y
# volver a parar en el cuadrante de salida.
#
#   correr_completa.sh [panel]
#
# Son DOS PROCESOS encadenados a proposito, no un programa fusionado.
# Cada uno abre y cierra su propio hardware, asi que la carrera arranca
# con el LiDAR, la Pico y la camara limpios, y sobre todo: el codigo de
# carrera -- que va 12/12 verificados -- no se toca ni una linea para
# que esto funcione. Cuesta unos 4 s de reapertura entre las dos fases.
#
# NO TERMINA POR RELOJ. La carrera acaba cuando la maquina de estados lo
# decide: UMBRAL_VUELTAS (1010 grados de yaw neto, ~3 vueltas) la pasa a
# fase PARQUEO, y ahi para al reconocer la firma de paredes que capturo
# al arrancar -- o sea, en el mismo sitio del que salio. El `timeout` de
# abajo es solo el limite del reglamento, una red, no el criterio.
cd /home/pi/ronda_curvas || exit 1
LIMITE_REGLAMENTO=180
[ "$1" = "panel" ] && export WRO_PANEL=1

echo "[1/2] $(date +%H:%M:%S)  saliendo del estacionamiento"
INICIO=$SECONDS
# La salida se guarda ENTERA en su propio log. Se perdio una vez por
# truncarla al leerla en remoto, y sin ella no hay diagnostico posible:
# el CSV de la carrera solo empieza cuando el parqueo ya termino.
PLOG="logs/parqueo_$(date +%Y%m%d_%H%M%S).log"
python3 -u parqueo.py salida 2>&1 | tee "$PLOG"
# Con la tuberia, $? es el de tee. El que importa es el de python.
ESTADO=${PIPESTATUS[0]}
echo "[parqueo] diario en $PLOG"
if [ "$ESTADO" -ne 0 ]; then
    echo "[-] no consiguio salir del estacionamiento: no se arranca la carrera"
    exit 1
fi
SALIDA=$((SECONDS - INICIO))
QUEDA=$((LIMITE_REGLAMENTO - SALIDA - 3))
echo "[1/2] salida completada en ${SALIDA}s.  Quedan ${QUEDA}s de ronda."
echo

echo "[2/2] $(date +%H:%M:%S)  carrera: 3 vueltas y vuelta al cuadrante de salida"
WRO_ARRANQUE_AUTO=1 WRO_SIN_CUENTA=1 WRO_LIDAR_CALIENTE=1 timeout -s INT "${QUEDA}" python3 -u ronda_camara.py
echo "[fin] $(date +%H:%M:%S)  total ${SECONDS}s  |  $(ls -t logs/*.csv 2>/dev/null | head -1)"
