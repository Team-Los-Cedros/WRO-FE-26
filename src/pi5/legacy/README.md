# Código legado de Pi 5

Esta carpeta agrupa las implementaciones anteriores de la ronda de obstáculos:

- `ronda_nueva/`: versión modular con cámara, LiDAR, planificación y pruebas.
- `ronda_cerrada/`: versión anterior conservada para referencia.

No son los programas de trabajo actuales. Se mantienen sin reescritura para
consulta y reproducción. Sus imports usan sus nombres históricos; desde un
checkout del repositorio, ejecuta los comandos antiguos desde `src/pi5/legacy/`.

`src/pi5/deploy.sh` conserva la capacidad de desplegarlas: toma sus archivos
desde esta carpeta, pero crea en el destino los directorios históricos
`ronda_nueva/` y `ronda_cerrada/` para no alterar los imports de ejecución.
