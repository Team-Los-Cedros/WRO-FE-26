#!/bin/bash
# Flashea el firmware de la Pico 2 DESDE LA PROPIA RASPBERRY, sin Thonny ni
# laptop. Ejecutar EN LA PI:
#
#   bash src/pico/deploy_pico.sh --dry-run        # valida sin escribir nada
#   bash src/pico/deploy_pico.sh                  # respalda, copia y reinicia
#
# Por que existe: INSTALACION.md manda subir los archivos con Thonny desde el
# PC, lo que obliga a desconectar la Pico del robot o a arrastrar el cable
# hasta la laptop. La Pi ya tiene la Pico en /dev/ttyACM0 y ya trae el modulo
# mpremote, asi que puede hacerlo ella misma por SSH.
#
# Tres cosas que este script cuida y que a mano se olvidan:
#   1. Respalda lo que hay en la Pico ANTES de sobrescribirlo. El firmware
#      vivo se ha desincronizado del repo mas de una vez.
#   2. Comprueba la sintaxis de cada archivo antes de subirlo. Un error de
#      sintaxis en main.py deja el robot inerte y solo se ve al reiniciar.
#   3. Saca la Pico del raw REPL al terminar y verifica que vuelve la
#      telemetria. Cualquier comando de mpremote interrumpe main.py y deja
#      la placa en raw REPL, donde MicroPython NO relanza main.py: queda
#      viva, muda y recibiendo consignas que nadie ejecuta. Para arreglar
#      eso sin reflashear: bash deploy_pico.sh --reiniciar
set -euo pipefail

PUERTO="${PUERTO_PICO:-/dev/ttyACM0}"
ARCHIVOS=(main.py protocolo_seguro.py ultrasonido.py Mpu6050.py)

uso() {
    cat <<'EOF'
Uso: deploy_pico.sh [--dry-run] [DIRECTORIO_ORIGEN]

  --dry-run   valida origen, puerto y sintaxis; no escribe en la Pico
  --reiniciar NO copia nada: solo saca la Pico del raw REPL y relanza
              main.py. Usalo tras cualquier comando de mpremote.
  DIRECTORIO  por defecto: el directorio donde vive este script

Variables: PUERTO_PICO (por defecto /dev/ttyACM0)
EOF
}

MODO="desplegar"
case "${1:-}" in
    --dry-run|--preflight) MODO="preflight"; shift ;;
    --reiniciar) MODO="reiniciar"; shift ;;
    --help|-h) uso; exit 0 ;;
    -*) echo "[-] Opcion desconocida: $1" >&2; uso >&2; exit 2 ;;
esac

ORIGEN="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)}"
if [[ ! -d "$ORIGEN" ]]; then
    echo "[-] No existe el directorio de origen: $ORIGEN" >&2
    exit 2
fi

# El robot es un recurso exclusivo: si hay una ronda corriendo, esta usando la
# Pico y reiniciarla a media carrera deja el motor a la ultima consigna.
if pgrep -f "ronda_nueva|ronda_cerrada|ronda_abierta" >/dev/null 2>&1; then
    echo "[-] Hay una ronda en ejecucion. Detenla antes de flashear:" >&2
    pgrep -af "ronda_nueva|ronda_cerrada|ronda_abierta" >&2
    exit 3
fi

if ! python3 -c "import mpremote" >/dev/null 2>&1; then
    echo "[-] Falta el modulo mpremote: pip3 install --user mpremote" >&2
    exit 2
fi
if [[ ! -e "$PUERTO" ]]; then
    echo "[-] No existe el puerto $PUERTO (¿Pico desconectada?)" >&2
    exit 2
fi

MP=(python3 -m mpremote connect "$PUERTO")

echo "[i] Origen: $ORIGEN"
echo "[i] Puerto: $PUERTO"

reiniciar_pico() {
# NO se usa machine.reset() a secas. mpremote deja la placa en RAW REPL, y
# en ese modo MicroPython NO ejecuta main.py al arrancar: la Pico queda
# viva, muda y sin control, que es justo el fallo que este script evita.
# La secuencia correcta es Ctrl-B (salir del raw REPL al REPL normal) y
# Ctrl-D (soft reboot, que ya si ejecuta main.py). Ademas no corta el USB,
# asi que no hay que esperar a que el puerto vuelva a enumerarse.
echo "[i] Saliendo del raw REPL y reiniciando main.py..."
python3 - "$PUERTO" <<'PY'
import sys
import time

import serial

puerto = sys.argv[1]
con = serial.Serial(puerto, 115200, timeout=2)
time.sleep(0.3)
con.write(b"\x03")        # Ctrl-C: interrumpe lo que estuviera corriendo
time.sleep(0.3)
con.write(b"\x02")        # Ctrl-B: raw REPL -> REPL normal
time.sleep(0.5)
con.reset_input_buffer()
con.write(b"\x04")        # Ctrl-D: soft reboot; ahora arranca main.py

# El arranque calibra el giroscopio (100 muestras) y el suelo (25), asi que
# la primera trama tarda un par de segundos en aparecer.
lineas = []
limite = time.monotonic() + 20.0
while time.monotonic() < limite:
    linea = con.readline().decode("utf-8", "ignore").strip()
    if linea.startswith("IMU:"):
        lineas.append(linea)
        if len(lineas) >= 20:
            break
con.close()

if not lineas:
    print("[-] La Pico no volvio a emitir telemetria tras el reinicio.")
    print("    Diagnostico: python3 -m mpremote connect {} exec "
          "'exec(open(\"main.py\").read())'".format(puerto))
    raise SystemExit(5)

print("[OK] Telemetria viva. Ultima trama:")
print("     " + lineas[-1])
campos = {p.split(":", 1)[0] for p in lineas[-1].split(",") if ":" in p}
for esperado in ("IMU", "COLOR", "US", "WD"):
    if esperado not in campos:
        print("[!] La trama no trae el campo {}".format(esperado))
PY
}

if [[ "$MODO" == "reiniciar" ]]; then
    reiniciar_pico
    exit 0
fi


# --- Validacion de sintaxis antes de tocar nada -------------------------
# py_compile solo analiza, no importa: que 'machine' no exista en la Pi da
# igual. Un SyntaxError aqui es un robot muerto alli.
faltan=0
for archivo in "${ARCHIVOS[@]}"; do
    ruta="$ORIGEN/$archivo"
    if [[ ! -f "$ruta" ]]; then
        echo "[-] Falta el archivo: $ruta" >&2
        faltan=1
        continue
    fi
    if ! python3 -m py_compile "$ruta" 2>/dev/null; then
        echo "[-] Error de sintaxis en $archivo; no se sube nada." >&2
        python3 -m py_compile "$ruta" || true
        exit 4
    fi
    printf '[OK] sintaxis %-22s %6d bytes\n' "$archivo" "$(wc -c < "$ruta")"
done
(( faltan == 0 )) || exit 2
rm -rf "$ORIGEN/__pycache__" 2>/dev/null || true

if [[ "$MODO" == "preflight" ]]; then
    echo "[OK] Preflight correcto: no se escribio nada en la Pico."
    exit 0
fi

# --- Respaldo del firmware vivo -----------------------------------------
RESPALDO="$HOME/pico_respaldo_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESPALDO"
echo "[i] Respaldando firmware vivo en $RESPALDO"
# Un respaldo que falla en silencio es peor que no tenerlo: si no se puede
# listar la Pico, tampoco se va a poder escribir en ella, asi que se aborta
# aqui en vez de descubrirlo a mitad de la copia.
if ! VIVOS=$("${MP[@]}" fs ls 2>/dev/null | awk 'NR>1 {print $2}'); then
    echo "[-] No se pudo listar la Pico. ¿Otro proceso tiene $PUERTO?" >&2
    echo "    Comprueba con: sudo fuser -v $PUERTO" >&2
    exit 6
fi
respaldados=0
for archivo in $VIVOS; do
    if "${MP[@]}" fs cp ":$archivo" "$RESPALDO/$archivo" >/dev/null 2>&1; then
        echo "     respaldado $archivo"
        respaldados=$((respaldados + 1))
    else
        echo "[!] No se pudo respaldar $archivo (se continua)" >&2
    fi
done
if (( respaldados == 0 )); then
    echo "[-] La Pico no tenia archivos o no se pudo copiar ninguno." >&2
    echo "    Si es una Pico virgen, relanza con FORZAR_SIN_RESPALDO=1" >&2
    [[ "${FORZAR_SIN_RESPALDO:-0}" == "1" ]] || exit 6
fi

# --- Copia ---------------------------------------------------------------
for archivo in "${ARCHIVOS[@]}"; do
    # Se normaliza a LF: el repo se edita en Windows y un CR suelto en la
    # primera linea de un modulo MicroPython es un error de sintaxis.
    tmp="$(mktemp)"
    tr -d '\r' < "$ORIGEN/$archivo" > "$tmp"
    "${MP[@]}" fs cp "$tmp" ":$archivo" >/dev/null
    rm -f "$tmp"
    echo "[+] subido $archivo"
done


reiniciar_pico

echo "[+] Firmware desplegado. Respaldo previo en: $RESPALDO"
