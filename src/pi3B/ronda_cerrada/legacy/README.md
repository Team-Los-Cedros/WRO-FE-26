# Scripts Archivados (Ronda Cerrada)

Estos archivos quedaron **superados** por `../Close2_round.py` y no deben usarse en pista. Se conservan aquí (en vez de borrarse) porque documentan el proceso real de iteración del equipo, y el historial completo de cada uno sigue siendo auditable con `git log --follow -p -- <archivo>`.

| Archivo | Por qué está archivado |
| :--- | :--- |
| `Close_round.py` | Versión original de la Ronda Cerrada. Tenía la regla de color invertida (`EVADIR_POR_IZQUIERDA = (color == "ROJO")`, cuando la regla WRO es Rojo→derecha) y ninguno de los fixes documentados en la sección 8.2 del [README principal](../../../README.md). **`controlador_inicio.py` apuntaba aquí por error** hasta que se corrigió para usar `Close2_round.py`. |
| `Close2_round_Prueba1.py` | Iteración experimental intermedia durante el desarrollo de `Close2_round.py`, con la misma regla de color invertida sin corregir. |

Si necesitas comparar el "antes y después" de los fixes de la Ronda Cerrada, es más confiable revisar el historial de commits de `Close2_round.py` que leer estos archivos, ya que ellos nunca recibieron las correcciones posteriores.
