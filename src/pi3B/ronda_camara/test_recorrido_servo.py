#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Prueba de recorrido de la direccion, con el MOTOR PARADO.

Se corre despues de tocar LIMITE_DER/LIMITE_IZQ en el firmware de la
Pico, para comprobar a ojo que el servo llega al nuevo tope sin topar
contra la varilla. Un servo forzado contra un tope mecanico zumba y
consume corriente sin llegar a la posicion: si se oye eso, hay que
devolver el limite a donde estaba.

Manda velocidad 0 en todas las consignas, asi que el robot no avanza:
solo giran las ruedas delanteras.

La secuencia sube el angulo por pasos y aguanta poco en cada uno, para
no dejar el servo forzando si el tope llega antes de lo esperado.
"""
import time

from enlace_pico import EnlacePico

# Angulo que la Pi manda; el firmware hace servo = CENTRO + angulo, y
# recorta a [LIMITE_DER, LIMITE_IZQ]. Negativo = derecha.
PASOS = [
    ("centro",                0.0, 1.5),
    ("derecha 20 (tope viejo)", -20.0, 1.5),
    ("derecha 22",           -22.0, 1.2),
    ("derecha 24",           -24.0, 1.2),
    ("derecha 25 (tope nuevo)", -25.0, 1.5),
    ("centro",                 0.0, 1.5),
    ("izquierda 25 (referencia)", 25.0, 1.5),
    ("centro",                 0.0, 1.5),
]

print("[*] Conectando con la Pico...")
enlace = EnlacePico()
time.sleep(0.5)
print("[*] MOTOR PARADO en toda la prueba. Mira las ruedas delanteras.\n")

try:
    for etiqueta, ang, espera in PASOS:
        print("  -> %-28s (angulo %+6.1f)" % (etiqueta, ang))
        t0 = time.time()
        while time.time() - t0 < espera:
            enlace.enviar(0, ang)     # velocidad 0: no avanza
            time.sleep(0.05)
    print("\n[+] Secuencia terminada, direccion centrada.")
    print("    Si en 'derecha 24' o 'derecha 25' el servo zumbo o la rueda")
    print("    dejo de girar mas, el tope mecanico esta antes de 65 y hay")
    print("    que volver a LIMITE_DER = 70.")
finally:
    for _ in range(10):
        enlace.enviar(0, 0.0)
        time.sleep(0.05)
    enlace.cerrar()
