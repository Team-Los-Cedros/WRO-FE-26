# Manual de Ensamblaje — Team Los Cedros (WRO-FE 2026)

Cómo montar el vehículo desde las piezas sueltas hasta la primera corrida, en el orden en que conviene hacerlo y con la comprobación que cierra cada etapa.

Este manual cubre el **montaje físico**. Lo que hay que comprar está en [`BOM.md`](BOM.md); la instalación del software, en [`INSTALACION.md`](INSTALACION.md); el flujo de señales completo, en el [diagrama de bloques](schemes/Diagrama_Bloques_Senales.svg) de la sección 4 del [README](README.md#4-arquitectura-eléctrica-y-distribución-de-señales).

> **El orden importa.** Está pensado para que cada etapa se pueda verificar antes de que la siguiente la tape: la placa perforada se prueba antes de atornillarla al chasis, y el LiDAR se mide antes de montar la cámara debajo. Saltarse el orden no rompe nada, pero obliga a desmontar para diagnosticar.

---

## Etapa 0 — Antes de tocar una pieza

Tres decisiones que condicionan todo lo demás y que cuesta caro cambiar a mitad de montaje:

* **El chasis es LEGO Technic, no impreso.** La versión impresa (V1) transmitía la vibración de los motores directa a la cámara y descalibraba la visión. Las vigas de fricción absorben ese ruido por flexión y permiten recolocar un sensor en boxes sin reimprimir nada.
* **La electrónica va soldada a una placa perforada, no con cables de puente.** Los *jumpers* sueltos daban falsos contactos por vibración. Es la diferencia entre un robot que falla una vez cada diez corridas sin motivo aparente y uno que no.
* **La alimentación va en tres etapas separadas.** Una sola línea para todo provoca reinicios de la Raspberry cuando el motor pide par. No es una mejora opcional: es lo que hace que el robot termine la ronda.

---

## Etapa 1 — Chasis

El chasis son **83 piezas de LEGO Technic**. El archivo CAD reproducible está en [`3d-Models/Chasis-LEGO-V2/Chasis-V2.io`](3d-Models/Chasis-LEGO-V2/), y el listado completo con el *Design ID* de BrickLink de cada pieza, en el [README de esa carpeta](3d-Models/Chasis-LEGO-V2/README.md).

Para seguir el montaje pieza a pieza, abre el `.io` con [BrickLink Studio](https://www.bricklink.com/v3/studio/download.page) (gratuito) y usa su generador de instrucciones: produce la vista explosionada y los pasos numerados a partir del propio modelo, así que las instrucciones nunca se desincronizan del diseño.

**Geometría que hay que verificar con regla al terminar,** porque de ella dependen la evasión y el estacionamiento:

| Medida | Valor | Por qué importa |
| :--- | :---: | :--- |
| Largo total del vehículo | **242 mm** | Fija el tamaño de plaza que el robot cree tener. |
| Ancho total | **138 mm** | Con él se calcula la holgura contra cada muro. |
| Eje del LiDAR a cada costado | **68 mm** | Es simétrico. Suponerlo asimétrico bloqueó la maniobra de estacionamiento durante toda una jornada. |
| Eje del LiDAR al morro | **54 mm** | Margen frontal de las maniobras. |
| Eje del LiDAR a la cola | **188 mm** | Margen en marcha atrás. |

## Etapa 2 — Dirección y tracción

1. **Servo de dirección.** Acopla el Geekservo directo al `base_servo` del eje delantero. No uses adaptadores impresos entre servo y viga: cada adaptador añade holgura, y la holgura en la dirección se convierte en error de trayectoria que ningún control puede corregir.
2. **Motor de tracción.** El Geekservo DC va al eje trasero. Los neumáticos son de caucho, no de plástico: los rígidos patinan al acelerar y disipan la potencia en calor.
3. **Ruedas.** Cuatro, una por esquina. Las dos delanteras dirigen, las dos traseras traccionan.

**Comprobación de esta etapa** (con el vehículo levantado, las ruedas al aire):

```bash
cd ~/ronda_curvas
python3 test_recorrido_servo.py     # barre el servo de tope a tope
python3 medir_radio.py              # mide el radio de giro real
```

El radio real debe rondar los **242 mm a la izquierda y 244 a la derecha**. Que salgan casi iguales es lo que se espera de una Ackermann bien montada; una diferencia grande delata holgura o un tope mal puesto.

## Etapa 3 — Placa perforada

Suelda sobre la placa perforada, **antes de montarla en el chasis**, estos tres componentes:

* **Raspberry Pi Pico 2**
* **Driver TB6612FNG**
* **MPU6050**

| Capa superior — Pico 2 + MPU6050 | Capa inferior — soldadura y buses |
| :---: | :---: |
| <img src="schemes/Placa_Perforada/Top_Layer_Placa.jpeg" alt="Capa superior de la placa perforada" width="300px"/> | <img src="schemes/Placa_Perforada/Bottom_Layer_Placa.jpeg" alt="Capa inferior de la placa perforada" width="300px"/> |

La IMU tiene una condición de montaje que no es negociable: **alineada con el eje longitudinal del chasis**. Así la lectura del eje Z corresponde exactamente al giro del vehículo y no hay que compensar ninguna desalineación por software. Móntala torcida y todo el conteo de vueltas hereda ese error.

Sigue el pinout de la [sección 4.3 del README](README.md#43-mapa-de-conexiones-calibrado-pinout). Resumen de lo que sale de la Pico:

| Destino | Pines |
| :--- | :--- |
| Servo de dirección | `GP12` (PWM 50 Hz) |
| TB6612FNG | `GP22` PWM · `GP26`/`GP27` dirección · `GP28` habilitación |
| MPU6050 | `GP16` SDA / `GP17` SCL (I²C0, 400 kHz) |
| TCS3472 | `GP18` SDA / `GP19` SCL (I²C1, 100 kHz) |
| HC-SR04 | `GP14` disparo / `GP15` eco |

> El bus del sensor de color va a **100 kHz y no a 400** como el de la IMU. El TCS3472 no sostiene 400 kHz de forma fiable, y a esa velocidad falla de forma intermitente, que es la peor manera de fallar.

## Etapa 4 — Alimentación

Monta las tres etapas **antes** de conectar nada de lógica, y comprueba cada salida con el multímetro con la carga desconectada:

| Etapa | Entrada | Salida esperada | Alimenta |
| :--- | :---: | :---: | :--- |
| Directo de batería | 2S 21700, 7,4-8,4 V | — | TB6612FNG (potencia del motor) |
| **XL4016** | 7,4-8,4 V | **5,1 V** | Raspberry Pi 5, cámara, RPLiDAR C1 |
| **XL1509** | 7,4-8,4 V | **6,0 V** | Servo de dirección |

Los reguladores son ajustables: **ponlos a tensión antes de conectarles la carga.** Un XL4016 que salga de fábrica a 12 V destruye la Raspberry en el primer arranque.

**Todas las tierras van a un único punto central, en estrella.** No encadenes masas de un módulo a otro: eso crea diferencias de potencial entre etapas y el ruido de conmutación del motor aparece como falsos flancos en las líneas digitales.

**Comprobación de esta etapa** — con el vehículo montado y el multímetro en serie con la batería, el consumo debe parecerse a esto:

| Estado | Corriente |
| :--- | :---: |
| Con la Pi apagada (solo Pico, sensores y potencia) | ~0,21 A |
| Sistema encendido, sin tracción | ~0,61 A |
| En marcha | ~1,39 A |

## Etapa 5 — Sensores

**Mástil del LiDAR.** El RPLiDAR C1 va elevado sobre un mástil, por encima de la cámara. La altura no es libre: el plano de barrido tiene que quedar a **69 mm del piso**, que es donde el haz corta tanto los pilares como las paredes (ambos de 100 mm de alto según el reglamento).

El mástil tiene un coste conocido: **el LiDAR se ve a sí mismo**. Mídelo y confírmalo:

```bash
python3 ~/WRO-FE-26/src/pi5/herramientas/diag_mastil.py
```

Debe encontrar el mástil en torno a **141-212° a 35-68 mm**, presente en casi todos los barridos. Ese sector se enmascara por software; si tu montaje lo sitúa en otro rango, hay que actualizar la máscara o el robot creerá tener una pared detrás.

**Cámara.** Al frente, debajo del LiDAR y **retrasada respecto al parachoques**, para que un golpe contra el perímetro no la reciba directa — en la V1 eso destruyó el módulo. Montada a **0° de inclinación**, mirando derecho al frente, sin inclinarla hacia el piso.

**Ultrasonido trasero.** Va en la cola, y apunta su posición exacta: está **34 mm por delante del punto más atrasado del robot**, así que la holgura real de la culata es su lectura menos esos 34 mm. Es la única medida real hacia atrás, porque el mástil ciega al LiDAR justo en ese sector.

**Sensor de color.** Bajo el chasis, mirando la lona.

**Comprobación de esta etapa.** Las pruebas de banco viven en dos sitios: las de `ronda_curvas/` se despliegan con el resto del código, y las de `src/pi5/herramientas/` se corren desde el clon del repositorio.

```bash
cd ~/ronda_curvas
python3 diag_lidar_360.py           # el barrido completo, sector a sector
python3 test_sectores_trasera.py    # coherencia entre el ultrasonido y el LiDAR

# este vive en el clon del repositorio, no en la carpeta de trabajo:
python3 ~/WRO-FE-26/src/pi5/herramientas/diag_eco_volante.py
```

`diag_eco_volante.py` no debería encontrar **ningún punto por debajo de 500 mm** en los sectores laterales con el volante recto ni a los topes. Si aparecen, el LiDAR está viendo su propia rueda y la maniobra de estacionamiento se bloqueará contra un obstáculo que no existe.

## Etapa 6 — Calibración antes de la primera corrida

Tres calibraciones que el robot no puede deducir solo:

```bash
cd ~/ronda_curvas
python3 calib_hsv.py            # umbrales de rojo y verde bajo la luz real del recinto
python3 calibrar_lineas.py      # orden de las líneas de esquina, empujando el robot una vuelta a mano
python3 medir_fov.py            # campo de visión efectivo de la cámara
```

* **Los umbrales HSV se recalibran en cada sede.** La luz de los boxes no es la de la pista oficial, y un umbral que funcionaba ayer deja de ver el pilar rojo hoy.
* **`calibrar_lineas.py` se corre una sola vez** y fija en qué orden se cruzan las líneas naranja y azul. Sin esa calibración el sistema **ignora** esa fuente por completo en vez de inventarse un sentido.
* **El campo de visión se mide, no se copia del catálogo.** El de esta cámara son **53,8° efectivos**, no los 102° que anuncia la hoja de datos de la versión *Wide*: el sensor recorta antes de escalar. Dar por bueno el catálogo infla el rumbo calculado de cada pilar 2,3 veces.

## Etapa 7 — Primera corrida

Con el software ya instalado ([`INSTALACION.md`](INSTALACION.md)):

```bash
bash ~/correr_completa.sh
```

Lo que tiene que pasar, en orden:

1. **El LED de la Pico parpadea.** Significa que todo cargó y espera.
2. **Pulsas el botón** de `GPIO 21`: el LED se apaga y arranca la secuencia.
3. El robot informa de lo que ve antes de moverse. Si está mal colocado en la plaza, **aborta y dice cuántos milímetros moverlo y hacia dónde**.
4. Sale del estacionamiento, corre las tres vueltas contando las líneas de pista y vuelve a parar en el cuadrante de salida.

> **Para detener una corrida a mitad, usa siempre SIGINT y nunca SIGTERM.** Solo SIGINT está manejado; un SIGTERM mata el proceso dejando el motor girando con la última consigna. Desde otra máquina: `ssh pi@<ip> 'pkill -INT -f ronda_camara'`.

---

## Checklist final

Antes de dar el vehículo por montado:

- [ ] El plano del LiDAR está a 69 mm del piso, medido con regla.
- [ ] `diag_mastil.py` sitúa el mástil dentro del sector enmascarado.
- [ ] `diag_eco_volante.py` no ve la rueda propia a ningún ángulo de volante.
- [ ] Las tres salidas de alimentación dan 5,1 V, 6,0 V y tensión de batería.
- [ ] Todas las masas confluyen en un solo punto.
- [ ] La IMU está alineada con el eje longitudinal.
- [ ] El radio de giro real ronda los 242/244 mm a cada lado.
- [ ] Los umbrales HSV están recalibrados bajo la luz del recinto.
- [ ] `calibrar_lineas.py` se corrió al menos una vez.
- [ ] El LED de la Pico parpadea al lanzar y el botón arranca la secuencia.
