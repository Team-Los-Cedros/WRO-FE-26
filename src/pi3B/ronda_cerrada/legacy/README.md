# Scripts Archivados (Ronda Cerrada)

Estos archivos quedaron **superados** por `../ronda_cerrada.py` y no deben usarse en pista. Se conservan aquí (en vez de borrarse) porque documentan el proceso real de iteración del equipo, y el historial completo de cada uno sigue siendo auditable con `git log --follow -p -- <archivo>`.

| Archivo | Por qué está archivado |
| :--- | :--- |
| `Close_round.py` | Versión original de la Ronda Cerrada. Tenía la regla de color invertida (`EVADIR_POR_IZQUIERDA = (color == "ROJO")`, cuando la regla WRO es Rojo→derecha) y ninguno de los fixes documentados en la sección 8.2 del [README principal](../../../../README.md). **`controlador_inicio.py` apuntaba aquí por error** hasta que se corrigió para usar `ronda_cerrada.py`. |
| `Close2_round_Prueba1.py` | Iteración experimental intermedia durante el desarrollo de lo que hoy es `ronda_cerrada.py`, con la misma regla de color invertida sin corregir. |
| `reto_obstaculos.py` | Borrador anterior a `reto_obstaculos_v2.py` (estados propios: `NAVEGACION_CARRIL`, `ESQUIVANDO`, `ESQUINA_APROXIMACION`, `ESQUINA_MANIOBRA`), nunca desplegado ni trackeado en ninguna rama hasta archivarse aquí. |
| `reto_obstaculos_v2.py` | Reescritura monolítica de ~1000 líneas que reemplazó a `ronda_cerrada.py` en la Raspberry durante un tiempo sin que el repositorio se enterara — perdió el clustering LiDAR, la fusión sensorial por posición y el parqueo que ya existían en la versión modular. Diagnosticado en detalle en la sección 8.3 del [README principal](../../../../README.md#83-caso-de-estudio-reactivación-de-la-ronda-cerrada-modular-con-evidencia-cuantitativa-2026-08-27): la corrección de fondo no fue arreglar este archivo, fue descubrir que `ronda_cerrada.py` ya tenía todo lo que este intentaba reconstruir peor. |

Si necesitas comparar el "antes y después" de los fixes de la Ronda Cerrada, es más confiable revisar el historial de commits de `ronda_cerrada.py` (`git log --follow`, que sigue el archivo a través del rename desde `Close2_round.py`) que leer estos archivos, ya que ellos nunca recibieron las correcciones posteriores.
