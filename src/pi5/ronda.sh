#!/bin/bash
# Elige QUE RONDA corre el robot al encenderse.
#
#   ./ronda.sh                 dice cual esta puesta
#   ./ronda.sh abierta         prueba abierta: 3 vueltas, sin pilares
#   ./ronda.sh obstaculos      estacionamiento + 3 vueltas con pilares
#
# Existe porque en competencia no vas a tener la laptop delante para
# editar el servicio. Esto es un archivo con una palabra dentro, y el
# servicio lo lee al arrancar.
set -euo pipefail
ARCHIVO=/home/pi/ronda_activa

actual() { cat "$ARCHIVO" 2>/dev/null || echo obstaculos; }

if [ $# -eq 0 ]; then
    echo "Ronda puesta: $(actual)"
    echo
    echo "  abierta      -> correr_abierta.sh   (3 vueltas, sin pilares,"
    echo "                                       sin estacionamiento)"
    echo "  obstaculos   -> correr_completa.sh  (sale del estacionamiento,"
    echo "                                       3 vueltas con pilares)"
    echo
    echo "Cambiar:  ./ronda.sh abierta"
    exit 0
fi

case "$1" in
    abierta|obstaculos) ;;
    *) echo "[-] solo vale 'abierta' u 'obstaculos'"; exit 1 ;;
esac

if systemctl is-active --quiet wro.service; then
    echo "[-] wro.service esta corriendo. Parala antes de cambiar de ronda:"
    echo "      sudo systemctl stop wro.service"
    exit 1
fi

echo "$1" > "$ARCHIVO"
echo "[+] ronda puesta: $1"
echo "    Arranca con:  sudo systemctl start wro.service"
echo "    O a mano:     bash ~/correr_$([ "$1" = abierta ] && echo abierta || echo completa).sh"
