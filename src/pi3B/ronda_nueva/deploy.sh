#!/bin/bash
# Ejecutar EN LA PI, desde cualquier directorio dentro del clon:
#   bash src/pi3B/ronda_nueva/deploy.sh --dry-run /home/pi/wro_nueva
#   bash src/pi3B/ronda_nueva/deploy.sh /home/pi/wro_nueva
#
# Este despliegue es deliberadamente "create only": prepara una copia en un
# directorio temporal vecino y la promueve de forma atomica solo si DESTINO no
# existe. Nunca mezcla ni actualiza una instalacion anterior, y no toca
# ronda_cerrada.py, ronda_camara/ ni controlador_inicio.py.
set -euo pipefail

uso() {
    cat <<'EOF'
Uso: deploy.sh [--dry-run|--preflight] [DESTINO]

  --dry-run, --preflight  valida origen y destino sin crear ningun archivo
  DESTINO                 por defecto: /home/pi/wro_nueva

El destino debe no existir. Para conservar una instalacion previa, elige otro
nombre (por ejemplo, /home/pi/wro_nueva_20260829).
EOF
}

MODO="desplegar"
if [[ "${1:-}" == "--dry-run" || "${1:-}" == "--preflight" ]]; then
    MODO="preflight"
    shift
elif [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    uso
    exit 0
elif [[ "${1:-}" == -* ]]; then
    echo "[-] Opcion desconocida: $1" >&2
    uso >&2
    exit 2
fi

if (( $# > 1 )); then
    uso >&2
    exit 2
fi

ORIGEN_PI3B="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
DESTINO_ENTRADA="${1:-/home/pi/wro_nueva}"

if [[ -z "$DESTINO_ENTRADA" || "$DESTINO_ENTRADA" == "/" ]]; then
    echo "[-] Destino inseguro o vacio: '$DESTINO_ENTRADA'" >&2
    exit 2
fi

DEST_PADRE_ENTRADA="$(dirname -- "$DESTINO_ENTRADA")"
NOMBRE_DESTINO="$(basename -- "$DESTINO_ENTRADA")"
if [[ "$NOMBRE_DESTINO" == "." || "$NOMBRE_DESTINO" == ".." ]]; then
    echo "[-] El destino debe nombrar un directorio nuevo." >&2
    exit 2
fi
if [[ ! -d "$DEST_PADRE_ENTRADA" ]]; then
    echo "[-] El directorio padre no existe: $DEST_PADRE_ENTRADA" >&2
    exit 2
fi

DEST_PADRE="$(cd "$DEST_PADRE_ENTRADA" && pwd -P)"
DESTINO="$DEST_PADRE/$NOMBRE_DESTINO"
if [[ -e "$DESTINO" || -L "$DESTINO" ]]; then
    echo "[-] Destino existente; no se modifico: $DESTINO" >&2
    exit 3
fi
if [[ ! -w "$DEST_PADRE" ]]; then
    echo "[-] Sin permiso de escritura en: $DEST_PADRE" >&2
    exit 2
fi

shopt -s nullglob
FUENTES_RONDA=("$ORIGEN_PI3B"/ronda_nueva/*.py)
FUENTES_COMUN=("$ORIGEN_PI3B"/comun/*.py)
if (( ${#FUENTES_RONDA[@]} == 0 || ${#FUENTES_COMUN[@]} == 0 )); then
    echo "[-] Faltan modulos Python en ronda_nueva/ o comun/." >&2
    exit 2
fi
for archivo in \
    "$ORIGEN_PI3B/ronda_nueva/configuracion.json" \
    "$ORIGEN_PI3B/requirements.txt"; do
    if [[ ! -f "$archivo" ]]; then
        echo "[-] Falta archivo requerido: $archivo" >&2
        exit 2
    fi
done

for herramienta in basename cp dirname mkdir mktemp mv python3 rm; do
    if ! command -v "$herramienta" >/dev/null 2>&1; then
        echo "[-] Falta herramienta requerida: $herramienta" >&2
        exit 2
    fi
done

echo "[OK] Preflight: origen completo y destino libre."
echo "[i] Origen:  $ORIGEN_PI3B"
echo "[i] Destino: $DESTINO"
if [[ "$MODO" == "preflight" ]]; then
    echo "[OK] Modo preflight: no se creo ni modifico ningun archivo."
    exit 0
fi

STAGING=""
PREFIJO_STAGING="$DEST_PADRE/.$NOMBRE_DESTINO.staging."
limpiar_staging() {
    if [[ -n "$STAGING" && -e "$STAGING" ]]; then
        # Solo se permite borrar el directorio unico creado por este proceso.
        if [[ "$STAGING" == "$PREFIJO_STAGING"* ]]; then
            rm -rf -- "$STAGING"
        else
            echo "[-] Se rechazo limpiar una ruta inesperada: $STAGING" >&2
        fi
    fi
}
trap limpiar_staging EXIT HUP INT TERM

STAGING="$(mktemp -d "$PREFIJO_STAGING"XXXXXX)"
mkdir "$STAGING/ronda_nueva" "$STAGING/comun"
cp -- "${FUENTES_RONDA[@]}" "$STAGING/ronda_nueva/"
cp -- "$ORIGEN_PI3B/ronda_nueva/configuracion.json" "$STAGING/ronda_nueva/"
cp -- "${FUENTES_COMUN[@]}" "$STAGING/comun/"
cp -- "$ORIGEN_PI3B/requirements.txt" "$STAGING/"

# ``comun`` no posee __init__.py en el repositorio historico; se crea solo en
# el staging nuevo para conservar imports de paquete, nunca sobre el destino.
if [[ ! -e "$STAGING/comun/__init__.py" ]]; then
    : > "$STAGING/comun/__init__.py"
fi

# Comprobacion de sintaxis sin importar drivers Raspberry ni crear pyc.
python3 - "$STAGING" <<'PY'
from pathlib import Path
import sys

raiz = Path(sys.argv[1])
archivos = sorted(raiz.rglob("*.py"))
if not archivos:
    raise SystemExit("staging sin modulos Python")
for archivo in archivos:
    compile(archivo.read_bytes(), str(archivo), "exec")
print("[OK] Staging: {} modulos Python validos.".format(len(archivos)))
PY

# GNU mv con ambas opciones evita tanto reemplazar como anidar el staging si
# otro proceso crea DESTINO entre el preflight y la promocion.
if ! mv --no-clobber --no-target-directory -- "$STAGING" "$DESTINO"; then
    echo "[-] No se pudo promover el staging; destino sin modificar." >&2
    exit 4
fi
if [[ -e "$STAGING" || -L "$STAGING" ]]; then
    echo "[-] El destino aparecio durante el despliegue; no se sobrescribio." >&2
    exit 4
fi

STAGING=""
trap - EXIT HUP INT TERM
echo "[+] ronda_nueva desplegada como paquete nuevo en $DESTINO"
echo "[i] Validar sin hardware: cd '$DESTINO' && python3 -m ronda_nueva.ronda_nueva --validar-config"
