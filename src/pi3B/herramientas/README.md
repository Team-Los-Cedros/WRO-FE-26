# Herramientas de diagnóstico y calibración (Pi 3B)

Scripts que **no corren en carrera**: se lanzan a mano durante una jornada de
pruebas para medir algo concreto del robot o para aislar un síntoma antes de
tocar la lógica de control. Están aquí porque varios de ellos son citados por
los comentarios del código de carrera como la forma correcta de reajustar una
constante (por ejemplo `calib_fov.py` para `FOCAL_PX` en `navegacion.py`), y
sin ellos esas instrucciones apuntan a nada.

> **Nota de historial:** hasta el commit `fb22298b` existió una carpeta
> `src/pi3B/calibracion/` con un propósito parecido, que se eliminó. Estas
> herramientas son posteriores y nacieron durante las sesiones de pista del
> 27 y 28 de agosto; se agrupan bajo un nombre nuevo para no resucitar una
> carpeta que el equipo retiró a propósito. Lo que aquella carpeta contenía
> sigue recuperable de la historia de git (`git show fb22298b^:<ruta>`).

## Cómo se ejecutan

Igual que los scripts de carrera, importan los módulos de `comun/` y de
`ronda_cerrada/` **por nombre suelto**, así que necesitan estar en el mismo
directorio plano que ellos. `deploy.sh` no las copia (no forman parte del
despliegue de competencia); para una jornada de pruebas se copian a mano
junto al resto:

```bash
cp src/pi3B/herramientas/*.py <directorio_de_pruebas>/
```

## Qué mide cada una

| Script | Mueve motores | Para qué sirve |
| :--- | :---: | :--- |
| `test_percepcion.py` | no | Valida el clustering del LiDAR y la visión en estático: qué ve el robot sin conducir. |
| `diag_angulo_muro.py` | no | Imprime `frontal`/`izquierda`/`derecha` y el `angulo_muro` triangulado, un valor por barrido. Se usó para verificar el signo de la asistencia de esquina apuntando a una esquina real. |
| `diag_naranja.py` | no | Aísla la detección HSV de la línea naranja del piso. |
| `diag_magenta.py` | no | Aísla el rango HSV del magenta, usado al depurar falsos positivos de color. |
| `diag_falso_rojo.py` | no | Instrumenta el falso positivo de «ROJO» sin pilares: guarda frame y máscara en cada transición de color. |
| `calib_fov.py` | no | Ajusta el modelo estenopeico de la cámara (centro óptico `c0` y focal `f`) contra postes reales. **Nunca se ha corrido**: `navegacion.py` usa mientras tanto el FOV de catálogo (~102°). |
| `medir_velocidad.py` | **sí** | Mide la curva PWM → mm/s por odometría LiDAR. De aquí salió `MM_POR_SEG_A_PWM100 = 400`, que corrigió una suposición previa de 900. |
| `correr_velocidad.py` | **sí** | Lanza una corrida a velocidad fija para las mediciones de arriba. |

> Las dos últimas mueven el robot. Como cualquier corrida, se terminan con
> `timeout -s INT`, nunca con SIGTERM: solo SIGINT está manejado, y un
> SIGTERM deja el motor girando con la última consigna.
