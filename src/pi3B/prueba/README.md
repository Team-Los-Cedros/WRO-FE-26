# Borradores de Desarrollo

A diferencia de [`ronda_cerrada/legacy/`](../ronda_cerrada/legacy/README.md) (versiones que **estuvieron desplegadas** y luego se superaron), esta carpeta guarda scripts que se prototiparon y **nunca llegaron a desplegarse** en la Raspberry — ninguno de los dos tipos debe copiarse a `/home/pi/` ni referenciarse desde `controlador_inicio.py` o `deploy.sh`.

| Archivo | Qué es |
| :--- | :--- |
| `prueba_abierta.py` | Borrador donde se prototipó el conteo de líneas naranjas por sensor de color (TCS3472) y los estados `BUSCANDO_PARQUEO` → `AVANZANDO_AL_PARQUEO` → `PARANDO`, antes de portarlos a la versión validada en `../ronda_abierta/ronda_abierta.py`. Útil como referencia de cómo se llegó a esa lógica. |
| `ronda_abierta_v2.py` | Reescritura alternativa de la Ronda Abierta con arquitectura dependiente del sentido de carrera (`BUSCANDO_SENTIDO`, `ESQUINA_ANTICIPACION`) — el mismo patrón que `ronda_cerrada/legacy/reto_obstaculos_v2.py` (ver sección 8.3 del [README principal](../../../README.md)) usaba y que la Ronda Cerrada modular evita a propósito (sección 5.3-C: centrado simétrico, sin necesidad de conocer el sentido). No se adoptó por esa razón — el diseño elegido para `ronda_abierta.py` es el centrado proporcional simétrico existente, que no depende de saber si la pista gira en horario o antihorario. |

Si algún día se retoma la idea de `ronda_abierta_v2.py`, vale la pena releer primero por qué se evitó esa dependencia en la Ronda Cerrada antes de reintroducirla aquí.
