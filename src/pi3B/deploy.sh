#!/bin/bash
# Corre esto EN LA PI, desde dentro del repo clonado:
#   bash src/pi3B/deploy.sh
#
# Copia los .py de carrera planos a /home/pi/ (o al destino que le pases
# como argumento) -- controlador_inicio.py y los imports entre modulos
# dependen de que todos queden juntos, sin subcarpetas. No toca
# ronda_cerrada/legacy/ (no se despliega, ver su README).
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DESTINO="${1:-/home/pi}"

cp "$DIR"/controlador_inicio.py \
   "$DIR"/comun/*.py \
   "$DIR"/ronda_abierta/ronda_abierta.py \
   "$DIR"/ronda_cerrada/ronda_cerrada.py \
   "$DIR"/ronda_cerrada/navegacion.py \
   "$DIR"/ronda_cerrada/camara_driver.py \
   "$DIR"/ronda_cerrada/vision.py \
   "$DIR"/ronda_cerrada/tracker.py \
   "$DIR"/calibracion/calibrar_hsv.py \
   "$DESTINO"/

echo "[+] Copiado a $DESTINO"
