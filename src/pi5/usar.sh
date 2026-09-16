#!/bin/bash
# Cambia la version de codigo que corre el robot, entre rondas.
#
#   ./usar.sh                 lista lo que hay y dice cual esta puesta
#   ./usar.sh 01              pone esa version
#   ./usar.sh 01_ULTIMO_PROBADO
#
# Que hace exactamente: copia los .py de ~/respaldos/<version> encima de
# ~/ronda_curvas. NO toca la carpeta logs/ ni corridas/, asi que no se
# pierde ninguna medida al cambiar de version.
#
# Antes de copiar guarda lo que habia en ~/respaldos/99_ANTES_DEL_CAMBIO,
# para que un cambio equivocado a media competencia se deshaga con
# ./usar.sh 99.
set -euo pipefail
ORIGEN=~/respaldos
DESTINO=~/ronda_curvas

if [ $# -eq 0 ]; then
    echo "Versiones disponibles en $ORIGEN:"
    for d in "$ORIGEN"/*/; do
        n=$(basename "$d")
        echo "  $n"
        # || true: con set -e y pipefail, una version sin LEEME.md
        # tumbaba el listado entero.
        (head -3 "$d/LEEME.md" 2>/dev/null | tail -1 | sed "s/^/      /") || true
    done
    echo
    echo "Puesta ahora: $(cat $DESTINO/.version 2>/dev/null || echo desconocida)"
    exit 0
fi

# Acepta el prefijo numerico o el nombre entero
CARPETA=$(ls -d "$ORIGEN"/"$1"* 2>/dev/null | head -1 || true)
if [ -z "$CARPETA" ] || [ ! -d "$CARPETA" ]; then
    echo "[-] no encuentro ninguna version que empiece por \"$1\""
    echo "    Corre ./usar.sh sin argumentos para ver la lista."
    exit 1
fi
NOMBRE=$(basename "$CARPETA")

if systemctl is-active --quiet wro.service; then
    echo "[-] wro.service esta CORRIENDO. Parala antes de cambiar el codigo:"
    echo "      sudo systemctl stop wro.service"
    exit 1
fi

# Respaldo de lo que habia, para poder deshacer
rm -rf "$ORIGEN/99_ANTES_DEL_CAMBIO"
mkdir -p "$ORIGEN/99_ANTES_DEL_CAMBIO/tests"
cp "$DESTINO"/*.py "$ORIGEN/99_ANTES_DEL_CAMBIO/" 2>/dev/null || true
cp "$DESTINO"/tests/*.py "$ORIGEN/99_ANTES_DEL_CAMBIO/tests/" 2>/dev/null || true

cp "$CARPETA"/*.py "$DESTINO"/
mkdir -p "$DESTINO/tests"
cp "$CARPETA"/tests/*.py "$DESTINO/tests/" 2>/dev/null || true
# Los .pyc viejos pueden ser de la version anterior
rm -rf "$DESTINO"/__pycache__ "$DESTINO"/tests/__pycache__
echo "$NOMBRE" > "$DESTINO/.version"

echo "[+] puesta la version: $NOMBRE"
echo "    Comprobando que compila..."
cd "$DESTINO"
for f in navegacion.py parqueo.py vision.py ronda_camara.py enlace_pico.py esperar_boton.py; do
    [ -f "$f" ] && python3 -m py_compile "$f"
done
echo "[+] todo compila. Listo para lanzar."
