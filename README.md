# ftc-biobuzz

Código del robot FTC para la temporada 2026-2027, **BIOBUZZ**, hecho para el CAD
"Robot BIOBUZZ 2027 torreta centrada" **v99** (17-nov-2026) y puesto al día con la
**v138-v145** (documento de instrucciones del CAD del 23-nov-2026). El robot recoge POLLEN y
NECTAR, las sube por el centro de la torreta y las lanza a la CELL de arriba de su HIVE.
La torreta apunta sola con un Limelight 3A; el gap del hood, su inclinación y las rpm del
flywheel se ajustan solos según la pelota y la distancia.

**Nada de esto se ha probado en el robot físico.** Todo sale del CAD, de las fichas de
goBILDA y REV, del manual TU09 y de una simulación. Los valores marcados MEDIR o CALIBRAR
en `Config.java` son de relleno.

Base: FtcRobotController SDK **12.0**, Java. Todo el código propio vive en
`robot/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

## Cómo está armado

| Archivo | Qué hace |
|---|---|
| `Config.java` | **Todos** los números ajustables, con su origen: CAD, CALC o CALIBRAR. |
| `Robot.java` | Junta el hardware, prepara el robot en INIT (R103.B) y decide "lleno" (G407). |
| `vision/CellVision.java` | Lee el Limelight y entrega la CELL de **arriba** vista desde el eje de la torreta. |
| `vision/HiveGeometry.java`, `HiveBelief.java` | Geometría del HIVE y cuál CELL está arriba. |
| `vision/ClusterSolver.java` | Centro de la fila de tags aunque haya tags tapados. |
| `vision/RivalNectarDetector.java` | Cámara C270 de la esquina +X del intake: NECTAR rival enfrente (G408). |
| `subsystems/Turret.java` | Servo de 5 vueltas, 7.78:1, tope ±105°, compensación de retraso y de giro. |
| `subsystems/Hood.java` | Dos servos con la ley ángulo = -gap + δ, y el seguro de sus signos. |
| `subsystems/Flywheel.java`, `Shooter.java` | Flywheel por velocidad y tablas de tiro por tipo de pelota. |
| `feeder/Feeder.java` | Fila, copa y elevador: una pelota por etapa. |
| `feeder/BallInventory.java` | Cuenta de pelotas y "lleno" para G407 (máximo 4). |
| `intake/Intake.java` | Interfaz del intake: cambiar de mecanismo no rompe lo demás. |
| `intake/IntakeLengua.java` | El intake: lengua de bandas con dedos colgantes, desplegada por un servo. µs de cálculo del CAD v138; no se mueve hasta calibrarla. Probado solo en simulación. |
| `intake/TongueServo.java` | Servo de la lengua: plegar directo; desplegar rápido hasta el empuje y luego en rampa. |
| `intake/ThroatCounter.java` | La garganta: cuenta para G407 y rivales para G408. |
| `sensors/` | Sensores de color REV V3 y break-beams. |
| `opmodes/` | TeleOp rojo y azul, y los OpModes de calibración y prueba. |

## El camino de la pelota

intake → garganta (cuenta) → **fila** (rodillo, BB3) → **copa** (aro en cremallera, V3
de z 52) → **etapa de arriba** (Gecko de z 128 y una loca con resorte, V3 de z 128) → flywheel.

- Disparar = prender el elevador hasta que la pelota de arriba se vaya. El tiro **solo se
  cuenta** si el flywheel lo sintió (caída de rpm) y el V3 de arriba ya no la ve.
- Rellenar = la copa sube la de abajo hasta las Gecko de z 95 y el elevador la sube a su
  etapa. El V3 de arriba la ve **antes** de que llegue (su distancia baja a un mínimo con
  el centro de la pelota a la altura del lente y vuelve a subir): ya vista sube despacio,
  hasta que la distancia (promediada) pasa su mínimo un rato seguido y un tramo más. Tiene
  que quedar con el centro entre z 144.8 (no toca una NECTAR en la copa) y el flywheel
  (NECTAR 156.5): 11.7 mm. Un parpadeo del V3 no cuenta; si la pierde de vista un rato o el
  recorrido estimado llega a su tope (`LIFT_MAX_*`), se detiene y se revisa con la copa abajo.
- La copa **nunca** sube si hay pelota esperando arriba: las dos se tocarían. Eso se decide
  con la copa abajo, con arriba vacío `UPPER_EMPTY_CONFIRM_S` seguidos (un parpadeo del V3
  no basta); ya subiendo, el V3 de arriba puede ver la pelota de la propia copa, así que
  solo aborta si ve algo en los primeros `COPA_UPPER_BLIND_MM` (MEDIR).
- Con la copa ocupada, la 3a se para en el primer corte de BB3 y sigue `FILA_SEAT_S` para
  asentarse en su lugar del CAD.
- El gap del hood se elige por el color de la pelota que va a salir. Una pelota de color
  desconocido va con el gap de NECTAR: una NECTAR por el gap de POLLEN se atora.

**Lleno (G407)**: por la cuenta, o con arriba, copa y fila ocupadas y otra pelota en la
entrada del intake. Con la lengua la 4a se queda en la boca sin cortar el haz de la
garganta: corta BB1, y con BB1 cortado `FULL_MOUTH_S` seguidos el robot está lleno. Solo
BB1: el haz de la garganta lo puede estar cortando la 3a misma (una NECTAR sin asentar).
Esa pelota **no** se suma a la cuenta hasta que cruza la garganta con M_F jalando (si se
sumara en la boca, al cruzar después de un tiro se contaría dos veces y el robot se
quedaría "lleno" con 3). La telemetría dice "+1 en la entrada".

Cuando algo falla, el robot se detiene en vez de adivinar (la telemetría dice qué pasó):

- **V3 sin lectura** (cable o I2C): el SDK no avisa, regresa ceros y 152.4 mm, lo mismo
  que "no hay nada". El código lo detecta y lo toma como **ocupado**: la copa no sube.
- **Tiro sin confirmar** (el V3 la dejó de ver y el flywheel no sintió nada): la pelota
  puede ir atorada en el hood. No se rellena y el gap no se mueve hasta que el artillero
  la empuje (gatillo izquierdo) o diga que el camino está libre (B). Ojo: con
  `SHOT_DIP_RPM` 250 puede que **ningún** tiro se confirme (CALC: las caídas son de 60 a
  190 rpm); hay que medirlo en "Tune: Tiro".
- **Rellenar que falla** dos veces seguidas se detiene; si la copa regresa vacía, la
  pelota se quedó en el tubo y se detiene a la primera. Si la pelota se ve arriba pero
  nunca pasa el V3, también cuenta como falla. B lo reanuda.
- **Fila atorada**: después de sus intentos se apaga y da un solo empujón cada 3 s.
- **Flywheel apagado** a medio tiro: el elevador se detiene.
- **NECTAR rival** que se queda en la entrada: reversa hasta 1.5 s y luego todo quieto
  (intake y fila) hasta que el piloto la saque en reversa. Una rival solo se resta de la
  cuenta si ya se había sumado.
- **M_F con sobrecorriente** (más de `MF_CURRENT_LIMIT_A`, ~5 A, `MF_OVERCURRENT_S`
  seguidos): se detiene en BRAKE y avisa; vuelve a intentar cuando el piloto cambia lo que
  pide. Sacando una rival, la rival queda "atorada" (todo quieto).
- **Pelota esperando** en la entrada mientras la fila no puede recibir (fila ocupada, o la
  copa ocupada o moviéndose): M_F en BRAKE; no cuenta como atasco ni como llegada.
- **Lengua sin calibrar** (`IntakeLenguaCfg.CALIBRATED = false`, como viene): ni la lengua
  ni M_F se mueven en el TeleOp; en INIT avisa que se pliegue a mano.
- **Hood con los signos sin cuadrar** (ver abajo): se queda en POLLEN y no se da por listo
  para una pelota que necesita el gap de NECTAR. Si además quedó guardado fuera de POLLEN,
  al arrancar se queda **sin PWM**: se arregla Config, o se pone a mano en POLLEN y se
  confirma con back en "Tune: Hood".

## Lo que no es obvio

**Las dos CELLS de la alianza se ven al mismo tiempo.** Los tags van en la cara de abajo
de cada CELL y, con el HIVE inclinado 30°, las dos filas miran al mismo lado. La de arriba
está a unos 49.9 in del piso y la de abajo a 35.6 in (calculado del manual). La altura de
cada fila se mide con la **separación entre sus tags** (2 o más tags; los tags miden
3.25 in y su separación es conocida) o, con un solo tag, con la pose 3D ("Full 3D").
Arranca creyendo lo del manual (fig. 10-2: arriba la roja del lado de la audiencia y la
azul del lado contrario). Si el artillero dice con X que se volteó, manda él hasta que 10
fotos seguidas digan lo contrario.

**Se apunta a la abertura, no a los tags.** El centro de la abertura está 3.41 in más
afuera que el centro de la fila de tags. Con 2 o más tags se conoce la dirección de la
fila y se apunta ahí; con uno solo se apunta aproximado y la torreta nunca dice "fija".

**Los tiempos del Limelight.** `getControlHubTimeStampNanos()` del SDK es hora de reloj,
no la base de `System.nanoTime()`, y el SDK crea un resultado nuevo cada 10 ms aunque la
cámara no haya tomado otra foto. La foto se reconoce por `getTimestamp()` (reloj del
Limelight) y su instante se calcula con `getStaleness()` más las latencias.

**La torreta sin feedback.** El servo no dice dónde está: se estima con su velocidad.
Cada foto se suma al rumbo que había **cuando se tomó**, y el objetivo se guarda en el
marco del campo con el IMU, así que si el chasis gira la torreta contragira sola. Para
decir "fijo", además, una foto tomada después de que la torreta llegó tiene que confirmarlo.

**El hood no puede mover un servo solo.** Si el gap cambia y el servo de ángulo no
compensa, la lámina se mete en la Limelight o toca las FlatTop. Por eso los dos servos se
escriben juntos en cada ciclo, el gap cambia en rampa y, al arrancar, se manda primero la
posición en la que quedó la última vez (se guarda en el Control Hub).

**Los dos servos del hood están montados al revés uno del otro** (CAD, inventario v136:
estriados hacia -X y +X, las dos juntas giran sobre +X). El mismo cambio de PWM los gira
en sentidos opuestos, así que sus µs por grado llevan **signo contrario**
(`ANGLE_US_PER_DEG` = -6.667). Con el mismo signo, al ir a NECTAR la lámina gira ~69° y
choca con la carcasa de la Limelight. Un seguro lo revisa (`HoodCfg.SERVOS_FACE_OPPOSITE`)
y "Tune: Hood" lo confirma: con δ +10 la **punta** de la lámina **baja** hacia el flywheel
(δ positivo cierra).

**Los topes del gap están 0.91° más allá** de POLLEN y de NECTAR (el hombro del perno en
su ranura). Como los µs se toman en los topes, el servo de ángulo tiene que compensar
36.1°, no 34.3: `GAP_NECTAR_DEG` arranca en -36.1 y se calibra con la luz (punta = labio).

**Apagar el PWM de un servo.** En el SDK, `setPwmDisable()` borra la caché del servo y
el siguiente `setPower` o `setPosition`, aunque sea igual, lo vuelve a prender. La fila
solo manda comandos cuando cambia de modo; si no, se quedaría en 1500 µs arrastrándose.

**Cambiar el modo de un motor reenvía la última potencia.** Hay que mandar 0 antes de
cada `setMode` (con el intake F2, un `RUN_TO_POSITION` arrancaba al 100 %).

**La lengua se despliega en dos tramos** (plan de la lengua del CAD). De plegada al
"empuje" la manivela gira 77° sin mover la lengua (el perno corre en la ranura de la
biela), así que va rápido. De ahí a abajo va en rampa de 120°/s como máximo: al pasar la
vertical la lengua cae por su peso, y si la manivela se quedara quieta serían ~35° de caída
libre; con la rampa la alcanza sin golpe. Plegar es directo: la manivela la sube con la
biela tensa y, pasada la vertical, la lengua se recarga sola en su tope y el servo se suelta.

**Los µs de la lengua no se usan sin calibrar.** Son 1002 / 1516 / 1998 (plegada / empuje /
abajo) con la ficha de goBILDA vista desde el estriado, pero el estriado tiene 25 dientes y
la manivela puede quedar calada hasta ±48 µs fuera: más que los 36 µs que hay entre la
plegada y el punto en que el servo jala la lengua contra su tope cerca del punto muerto de
la biela (~400-500 N). "Tune: Lengua" los saca con criterios que no dependen del calado.

**La lengua del intake no se pliega sobre una pelota.** Con una pelota en la boca (BB1) o
en la garganta, los dedos centrales se montan en ella y, al subir, chocan con la flecha
del pivote de la fila. Por eso con 4 pelotas la lengua se queda abajo con M_F en BRAKE, y
en INIT solo se pliega con la boca y la garganta libres.

**Lecturas en bloque en MANUAL.** `Robot` pone los hubs en `MANUAL`: cada ciclo tiene que
empezar con `robot.beginLoop()` (`initStep` ya lo hace). Sin eso, motores y sensores
digitales se quedan con la lectura vieja. La sobrecorriente de M_F sale de esa misma
lectura: no cuesta nada.

**Los V3 miden cada 25 ms** (de fábrica, cada 100 ms; se cambia en cada INIT); el de
arriba mide distancia cada 12.5 ms y se lee en cada ciclo mientras sube una pelota. Su
`setGain` es de software y recorta en 1: con mucha ganancia los canales se saturan y el
color sale mal. "Test: Sensores" marca "SATURADO".

## Configuración del robot en la Driver Station

Qué va en cada hub lo propone el CAD (v90-v94); los números de puerto los propone este
código. **Todo a confirmar.** Los nombres distinguen mayúsculas. "Opcional" = el código
usa `tryGet` y sigue sin él.

| Hub | Puerto | Nombre | Tipo |
|---|---|---|---|
| Control Hub | Motores 0-3 | `frontLeft`, `frontRight`, `backLeft`, `backRight` | modelo del motor del chasis |
| Control Hub | Servo 0 | `turret` | Servo (2000-0025-0504, 5 vueltas) |
| Control Hub | Servo 2 | `copa` | Servo (2000-0025-0004) |
| Control Hub | Servo 4 | `fila` | Continuous Rotation Servo (2000-0025-0004 en continuo) |
| Control Hub | USB-A 3.0, el azul | `limelight` | "Ethernet Device" al dar Scan, renombrarlo |
| Control Hub | USB-A 2.0 | `c270` | Webcam (opcional) |
| Control Hub | I2C bus 0 | `imu` | viene configurada sola (opcional) |
| Control Hub | I2C bus 1 | `colorCopa` | REV Color Sensor V3 (z 52) |
| Control Hub | I2C bus 2 | `colorArriba` | REV Color Sensor V3 (z 128) |
| Control Hub | I2C bus 3 | `colorGarganta` | REV Color Sensor V3 (entrada de la fila) |
| Control Hub | Digital 3 | `bbBoca` | Digital Device (BB1) |
| Control Hub | Digital 5 | `bbGarganta` | Digital Device |
| Control Hub | Digital 7 | `bbFila` | Digital Device (BB3) |
| Expansion Hub | Motor 0 + encoder 0 | `flywheel` | goBILDA 5203-2402-0001 (1:1) |
| Expansion Hub | Motor 1 (encoder opcional: no se usa) | `intake` | goBILDA 5203-2402-0005 (5.2:1), M_F |
| Expansion Hub | Motor 2 | `elevador` | goBILDA 5203-2402-0005 (5.2:1) |
| Expansion Hub | Servo 0 | `hoodGap` | Servo (2000-0025-0002) |
| Expansion Hub | Servo 2 | `hoodAngulo` | Servo (2000-0025-0002) |
| Expansion Hub | Servo 4 | `lengua` | Servo (2000-0025-0002), despliega la lengua del intake |

- Los 3 V3 tienen la misma dirección I2C (0x52): cada uno en su propio bus.
- Los servos van en puertos de pares distintos (0-1, 2-3, 4-5): cada par comparte 2 A y
  así un servo trabado no tumba al otro.
- 7 de 8 motores y 6 de 8 servos (R503).
- El cable del servo de la lengua mide 300 mm y sale en (-186, -196, 110): en línea recta
  hay ~424 mm al header de servos de la Expansion Hub (~268 a la Control Hub). **Lleva
  extensión** (CALC del CAD).
- El Limelight pide un USB-C en ángulo recto: entre la pared -X de su carcasa y la placa
  lateral izquierda quedan ~12.6 mm (CAD v100).

La [hoja de cableado](docs/cableado.html) es del diseño de **septiembre** (antes del CAD
v99): sirve para lo eléctrico general (XT30, RS485, pares de servos, USB 3.0), pero sus
puertos ya no son los de esta tabla (habla de 5 servos, un sensor magnético y un segundo
flywheel que ya no existen).

## Pipeline del Limelight

En la interfaz web del Limelight, pipeline **0**:

- Fiducial Markers / AprilTag, familia **36h11**, tamaño **82.55 mm** (3.25 in)
- Sin filtro de IDs: el código filtra por alianza
- "Full 3D" recomendado: con un solo tag a la vista es lo único que dice si es la fila de
  arriba o la de abajo
- Crosshair en el centro: el código usa `tx_nocross` y la corrección va en `AIM_TRIM_DEG`
- Si de lejos no detecta los tags, subir la resolución (1280x960, a costa de fps)
- Si la imagen sale de cabeza, tx y ty se voltean juntos: arreglarlo con la orientación de
  la imagen en la interfaz del Limelight (`TY_SIGN` solo no basta)
- El código **no** cambia de pipeline durante el partido: `pipelineSwitch` es un POST HTTP
  que puede congelar el ciclo hasta 15 s. Solo avisa si el pipeline no es el 0.

## Pruebas en el robot, en este orden

Del documento del CAD v138 (6.2). Cada paso dice qué valor de `Config` sale de ahí.

1. **Banco de un servo 2000 suelto**, antes de armar la copa: mandar 1200 y luego 1800 µs y
   ver hacia dónde gira **visto desde el estriado**. Si es horario, todo el CALC vale. Si
   no, voltear juntos los signos (comentario al principio de `Config.java`) y calar la copa
   con el aro en el piso a ~640 µs. Mejor probar uno de cada modelo (0002, 0004 y 0504).
2. **Test: Sensores**: los 3 haces sin y con pelota (`BEAM_ACTIVE_HIGH`; si el BB1 no lee
   estable, 10 kΩ de la señal a 3.3 V) y los 3 V3 (`SensorCfg`). Con el tubo vacío ningún
   V3 debe decir "SIN LECTURA" ni "SATURADO". La C270 usa rangos de color genéricos del
   SDK: revisar que vea una NECTAR y no las cintas del piso.
3. **Tune: Torreta** (arranca bloqueada a ±35°): **antes de engranar** el piñón, dar Play
   (el servo va a 1500 µs) y engranar con la torreta al frente. Luego `SERVO_CENTER`,
   `SERVO_INVERTED` y `DEG_PER_SERVO_UNIT` con escuadra. Suelta (bumpers + X) solo llega a
   ±105° de Config: **no ampliarlo** (el servo llega a ±115.7°, la guía de cables es para
   ±110°). Con el Limelight sin montar o desconectado.
4. **Tune: Hood**, con un calibrador de luz entre la lámina y la llanta FlatTop: tope de
   POLLEN, ángulo cero (punta = labio, ~62.6-63.0 mm), signo (con δ +10 la punta cierra a
   ~46 mm), sentido del gap, tope de NECTAR y la compensación de NECTAR (`GAP_NECTAR_DEG`,
   punta = labio ≈ 80 mm). La primera vez, hood a mano en POLLEN antes de INIT.
5. **Tune: Copa**: encontrar el piso con la corriente y marcarlo con X (a ±60 µs de
   `COPA_DOWN_US`); `COPA_UP_US` a 54 mm con vernier. Se limita a 54 mm de cálculo sobre
   el piso marcado.
6. **Tune: Alimentador**: signo de `FILA_POWER_IN`, sentido del elevador, y **medir antes
   de dejar que rellene solo** lo que lee el V3 de arriba: con NECTAR en la copa abajo
   (tiene que leer más que 50 mm), con la pelota en la copa a 0, 27 y 54 mm
   (`COPA_UPPER_BLIND_MM`), subiendo a la etapa y ya en su etapa (**la tiene que seguir
   viendo**: si no, la copa subiría contra ella). Luego el ciclo completo: "Última subida"
   y dónde se queda la de arriba (`LIFT_*`: centro entre z 145 y 156 con NECTAR).
7. **Tune: Lengua** con el procedimiento del documento del CAD (calado, sentido, empuje,
   abajo por despegue, plegada desde la vertical, signo de M_F y velocidad). Criterios:
   `PUSH_US - FOLDED_US` ≈ 514 µs y `DOWN_US - PUSH_US` ≈ 482 µs; `PUSH_US - 1516` es el
   error de calado (±48 µs como máximo, ±8 con el REX); con `FOLDED_US` la punta avanza
   ~4 mm a mano y la corriente no sube. Al final, `CALIBRATED = true`.
8. **Banco de llenado**: meter 4 pelotas (POLLEN-POLLEN y NECTAR-NECTAR). Ver dónde se
   quedan la 3a y la 4a, que el robot entre a LLENO y que no haya reversas de atasco.
9. **Atasco de M_F**: detener una pelota con la mano y ver la corriente en "Tune: Lengua":
   el corte a ~5 A tiene que actuar (`MF_CURRENT_LIMIT_A`, `MF_OVERCURRENT_S`).
10. IMU: con el logo del hub hacia arriba y el USB hacia atrás (`ImuCfg`). En el TeleOp,
    girar a la izquierda tiene que subir el yaw.
11. Medir `CAMERA_HEIGHT_IN`, `CAMERA_PITCH_DEG` y las alturas de las filas de tags.
12. **Tune: Tiro**, con la aleta de entrada de la v145 armada (la de la v133-v139 apretaba
    más que el hood): primero revisar que las alturas medidas de las filas den ~49.9 y
    ~35.6 in. Si salen debajo de la cámara (13.4 in) o negativas, `TY_SIGN = -1`. Luego
    llenar `POLLEN_TABLE` y `NECTAR_TABLE` en cinco o seis distancias y anotar la caída
    de rpm de cada tiro para `SHOT_DIP_RPM`. CALC: con la salida a ~35° (δ 0) no se llega
    a la abertura a menos de ~70 in del eje, y con ~45° (δ -10) a menos de ~50 in: la fila
    de 30 in de las tablas probablemente no sirve.
13. Medir la velocidad real de la torreta (`MAX_SPEED_DEG_S`) y ajustar el PIDF del
    flywheel.

## Controles del TeleOp

En INIT el robot se prepara solo (torreta al frente, hood a su casa, copa abajo, lengua
plegada y suelta). Con una pelota en la boca o en la garganta la lengua no se pliega y lo
avisa: hay que sacarla. Sin calibrar, la lengua no se mueve: se pliega a mano. El
artillero indica con su dpad cuántas pelotas van precargadas.

| Gamepad 1, piloto | |
|---|---|
| Sticks | Chasis; gatillo derecho = lento |
| Bumper derecho | Intake sí/no (también reintenta después de una sobrecorriente) |
| Bumper izquierdo (sostener) | Intake en reversa |
| B (sostener) | Jalar aunque la cámara vea NECTAR rival |

| Gamepad 2, artillero | |
|---|---|
| Bumper derecho | Flywheel sí/no |
| Gatillo derecho (sostener) | Dispara cada vez que todo está listo |
| Gatillo izquierdo (sostener) | Empuja con el elevador aunque arriba no se vea pelota (sacar una atorada; flywheel en velocidad, sin mira) |
| B | "El camino está libre": quita los bloqueos del alimentador |
| A | Auto-apuntado sí/no (arranca en sí) |
| Y | Torreta al frente (apaga el auto-apuntado) |
| X | "El HIVE se volteó": cambia cuál CELL está arriba |
| Stick derecho X, dpad izq/der | Torreta a mano |
| Dpad arriba/abajo | Corregir la cuenta de pelotas |

Los dos controles vibran cuando la torreta está fija, el flywheel en velocidad, el hood en
su lugar y hay pelota arriba.

## Probar sin robot

```bash
./sim/run.sh
```

Compila TeamCode contra los JAR reales del SDK 12.0 y corre una simulación con tiempo
virtual y las clases reales del robot:

- El Limelight sale de proyectar el campo 3D del manual (HIVE inclinado, dos filas de
  tags) y pasa por el `LLResult` real del SDK.
- CELL de arriba en una rejilla de 405 posiciones, con y sin Full 3D.
- Torreta de punta a punta: retraso de cámara, servo más lento o más rápido que el modelo,
  chasis girando, vueltas completas y arranque con la torreta girada.
- Hood con servos de distinta velocidad: δ nunca sale de la envolvente. Con los servos
  girando como en el CAD (el de ángulo al revés): con el signo de Config va a NECTAR sin
  salirse; con el signo equivocado el seguro no lo deja, y sin el seguro el modelo choca
  con la Limelight. Con los topes reales del gap: -34.3 deja δ en -1.8° y -36.1 en 0.
- El piloto contra la visión: su X manda hasta 10 fotos seguidas en contra.
- Flywheel: bajar la consigna no es tiro; una caída a velocidad sí, una vez, aunque el
  umbral sea menor que la tolerancia de velocidad. Las caídas de cálculo (60-190 rpm) con
  `SHOT_DIP_RPM` 250 dejan el tiro "sin confirmar".
- Alimentador con un modelo de pelotas que suben de forma continua y un V3 de arriba que
  lee la distancia geométrica con conos de 30° y 45°: 4 tiros, sin subir la copa con
  pelota arriba, la de arriba entre la de la copa y el flywheel (las 4 combinaciones, con
  ruido de 0.3 mm en 30 intentos, con un parpadeo al empezar y con una curva más plana que
  termina en el tope de recorrido), ver la pelota de la propia copa no aborta, un parpadeo
  de 100 ms arriba no sube la copa, la 3a se asienta, y con fallas a propósito: V3 sin
  lectura, parpadeo del V3, pelota atorada antes del flywheel, flywheel apagado a medio
  tiro, rellenado que falla (también una NECTAR que se ve arriba pero no sale de la copa)
  y fila atorada.
- Intake: rechazo de NECTAR rival (contada, sin contar y que se queda), rebotes en la
  garganta, una pelota parada sobre el haz y atascos.
- Lengua (con los µs de cálculo): plegada y suelta en INIT (no se pliega con pelota en la
  boca o en la garganta), se queda plegada al dar Play con 4 pelotas, salta al empuje y
  baja en rampa de 800 µs/s como máximo, solo entonces jala, con 4 se queda abajo en
  BRAKE, y con una rival primero baja y luego hace reversa. La 4a en la boca sin cortar la
  garganta pone LLENO sin reversas (aunque la fila pareciera jalar), no se cuenta dos veces
  aunque corte la garganta a ratos (lleno o esperando), espera mientras la fila no jala y
  se cuenta una vez al entrar; la 3a sin asentar en el haz de la garganta no es la 4a. M_F
  con sobrecorriente se detiene y espera al piloto. Sin calibrar no se mueve nada.
- Hood guardado en NECTAR con los signos sin cuadrar: arranca sin PWM hasta confirmarlo.

Lo que la simulación **no** prueba: fricción, rebotes, el vuelo de la pelota, la luz del
campo, lo que lee de verdad un V3 fuera de su eje, ni que el robot real tenga las medidas
del CAD. Comparte supuestos con el código (la geometría del manual, las convenciones del
Limelight), así que no atrapa errores en esos supuestos.

## Compilar e instalar

Abrir la carpeta `robot/` en Android Studio y correr `TeamCode` sobre el Control Hub.

## Pendiente

- Todos los valores MEDIR y CALIBRAR de `Config.java`, en el orden de arriba.
- Confirmar el reparto de puertos y rehacer la hoja de cableado con él.
- Lengua: calibrarla en "Tune: Lengua" y poner `CALIBRATED = true`; la velocidad real del
  servo; ver en el robot si, plegada y sin PWM, la manivela se regresa al empujar la
  lengua (si se regresa, dejar el PWM prendido plegada).
- C270: medir la cámara real (lado del lente, cuña, FOV) y, después, usar el cuadro
  completo y quedarse con la zona de la boca (el respaldo es el V3 de la garganta).
- Opcional del documento del CAD (no está en su plan): en INIT, revisar la corriente de
  los servos de la Expansion Hub al plegar para detectar la lengua en paro.
- Decidido por Adrián (19-nov): con 4 pelotas la lengua se queda abajo con M_F en BRAKE, y
  la precarga es de 3 POLLEN adentro (2 en el tubo y 1 en la fila) con la 4a en la loseta
  tocando el frente; el autónomo la mete al desplegar (con la misma regla de "lleno"). En
  el TeleOp la lengua baja con el intake prendido y sin estar lleno, y no se vuelve a
  plegar en el partido (el plan lo permitiría con 3 o menos y la boca y la garganta
  libres, p. ej. para defensa).
- Autónomo.

## Otros

El intake F2 (rotor de aletas, estacionado con imán) salió del CAD en la v106; su código
quedó en el historial de git (último commit con F2: `08390d5`).

La aleta de entrada al pie del tubo (rehecha en las v140-v145) es pasiva: no lleva nada
que programar.

`rampa/render_vtk.py` es una función para renderizar piezas de build123d o STL con VTK.
No es parte del código del robot.
