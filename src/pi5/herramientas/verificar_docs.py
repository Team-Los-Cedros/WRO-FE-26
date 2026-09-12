#!/usr/bin/env python3
"""Comprueba que la documentacion del repositorio no esta rota.

    python3 src/pi5/herramientas/verificar_docs.py

Existe porque tres de los fallos que aparecieron en este repositorio eran
INVISIBLES al leer el archivo:

  1. Un caracter de avance de pagina (0x0C) incrustado donde debia ir el
     comando LaTeX de fraccion. En pantalla se veia "rac{5.0}" y la
     formula de autonomia no renderizaba. Lo produce interpretar la barra
     invertida como escape al generar el texto, y pasa igual con el
     tabulador, el retroceso, la campana y el tabulador vertical.
  2. Enlaces internos a secciones cuyo titulo habia cambiado. GitHub no
     avisa: el enlace simplemente no lleva a ninguna parte.
  3. Finales de linea CRLF, que ensucian cada diff.

Ninguno de los tres lo detecta un corrector de ortografia ni se ve al
revisar el documento a ojo.

DOS DECISIONES DEL VERIFICADOR, aprendidas de sus propios falsos
positivos:

  - Los finales de linea se comprueban sobre lo que GUARDA GIT, no sobre
    el archivo del disco. En Windows git convierte a CRLF al sacar los
    archivos, asi que mirar el disco acusa a todo el repositorio de algo
    que no pasa.
  - El recuento de delimitadores de formula salta los bloques de codigo.
    Una linea como `deploy.sh "$DESTINO"` lleva un solo simbolo de dolar
    y no es una formula a medio cerrar.

Devuelve 0 si todo esta bien y 1 si hay algun fallo, para poder
encadenarlo en un gancho de pre-commit.
"""
import io
import os
import re
import subprocess
import sys
import urllib.parse

# Caracteres de control que no deben existir en texto. Son justo los que
# produce interpretar una secuencia de escape sin querer.
CONTROL = {0x00: "nulo", 0x07: "campana", 0x08: "retroceso",
           0x09: "tabulador", 0x0b: "tabulador vertical",
           0x0c: "avance de pagina", 0x1b: "escape"}

# Como genera GitHub el ancla de un titulo (github-slugger): minusculas,
# fuera la puntuacion y los simbolos -- incluido el rango A0-BF, donde
# cae el superindice dos de "I2C" -- y cada espacio pasa a guion, sin
# colapsar los repetidos.
RANGOS_FUERA = [(0x00, 0x1f), (0x21, 0x2c), (0x2e, 0x2f), (0x3a, 0x40),
                (0x5b, 0x5e), (0x60, 0x60), (0x7b, 0x7e), (0xa0, 0xbf),
                (0xd7, 0xd7), (0xf7, 0xf7), (0x2000, 0x206f)]


def ancla(titulo):
    def fuera(c):
        return any(a <= ord(c) <= b for a, b in RANGOS_FUERA)
    limpio = "".join("" if fuera(c) else c for c in titulo.strip().lower())
    return limpio.replace(" ", "-")


def markdowns():
    for dp, dn, fn in os.walk("."):
        if ".git" in dp.split(os.sep):
            continue
        for f in sorted(fn):
            if f.endswith(".md"):
                yield os.path.join(dp, f).replace(os.sep, "/").lstrip("./")


def finales_crlf_en_git():
    """Archivos que git GUARDA con CRLF. Vacio es lo correcto."""
    try:
        salida = subprocess.run(["git", "ls-files", "--eol"],
                                capture_output=True, text=True,
                                encoding="utf-8", errors="replace").stdout
    except (OSError, subprocess.SubprocessError):
        return None                      # sin git: no se puede comprobar
    malos = []
    for linea in salida.split("\n"):
        # Formato: "i/lf    w/crlf  attr/text=auto eol=lf   ruta"
        if linea.startswith("i/crlf") and linea.rstrip().endswith(".md"):
            malos.append(linea.split("\t")[-1].strip())
    return malos


def fuera_de_bloques(texto):
    """El texto sin los bloques cercados con tres acentos graves."""
    lineas, dentro, salida = texto.split("\n"), False, []
    for ln in lineas:
        if ln.lstrip().startswith("```"):
            dentro = not dentro
            salida.append("")
            continue
        salida.append("" if dentro else ln)
    return salida


def main():
    if len(sys.argv) > 1:
        os.chdir(sys.argv[1])
    fallos = []

    crlf = finales_crlf_en_git()
    if crlf is None:
        print("[i] no se pudo consultar a git; se omite la comprobacion de "
              "finales de linea")
    else:
        fallos += ["%s  git lo guarda con finales CRLF" % f for f in crlf]

    for ruta in markdowns():
        texto = io.open(ruta, encoding="utf-8").read()
        base = os.path.dirname(ruta)

        # 1. Caracteres de control invisibles
        for n, linea in enumerate(texto.split("\n"), 1):
            for c in linea:
                if ord(c) in CONTROL:
                    fallos.append("%s:%d  caracter de control: %s"
                                  % (ruta, n, CONTROL[ord(c)]))
                    break

        # 2. Imagenes y enlaces relativos
        destinos = re.findall('<img[^>]*src="([^"]+)"', texto)
        destinos += [d for _, d in re.findall(r"\[([^\]]*)\]\(([^)\s]+)\)", texto)]
        for d in destinos:
            if d.startswith(("http://", "https://", "#", "mailto:", "data:")):
                continue
            p = urllib.parse.unquote(d.split("#")[0])
            if p and not os.path.exists(os.path.normpath(os.path.join(base, p))):
                fallos.append("%s  apunta a algo que no existe: %s" % (ruta, d))

        # 3. Anclas internas
        titulos = {ancla(m.group(1))
                   for m in re.finditer(r"^#{1,6}\s+(.*)$", texto, re.M)}
        for d in re.findall(r"\]\(#([^)]+)\)", texto):
            if d not in titulos:
                fallos.append("%s  ancla que no lleva a ningun titulo: #%s"
                              % (ruta, d))

        # 4. Formulas con los delimitadores desparejados, fuera de codigo
        for n, linea in enumerate(fuera_de_bloques(texto), 1):
            if linea.count("$") % 2:
                fallos.append("%s:%d  numero impar de delimitadores de formula"
                              % (ruta, n))

    if fallos:
        print("DOCUMENTACION CON FALLOS (%d):" % len(fallos))
        for f in fallos:
            print("  " + f)
        return 1
    print("Documentacion verificada: sin caracteres de control, sin enlaces\n"
          "rotos, sin anclas muertas y con finales de linea LF.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
