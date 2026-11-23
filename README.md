# ftc-biobuzz

Código del robot FTC para la temporada 2026-2027, **BIOBUZZ**, hecho para el CAD
"Robot BIOBUZZ 2027 torreta centrada" **v99** (17-nov-2026). El robot recoge POLLEN y
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
| `Robot.java` | Junta el hardware y prepara el robot en INIT (R103.B). |
| `vision/CellVision.java` | Lee el Limelight y entrega la CELL de **arriba** vista desde el eje de la torreta. |
| `vision/HiveGeometry.java`, `HiveBelief.java` | Geometría del HIVE y cuál CELL está arriba. |
| `vision/ClusterSolver.java` | Centro de la fila de tags aunque haya tags tapados. |
| `vision/RivalNectarDetector.java` | Cámara C270 de la visera: NECTAR rival enfrente (G408). |
| `subsystems/Turret.java` | Servo de 5 vueltas, 7.78:1, tope ±105°, compensación de retraso y de giro. |
| `subsystems/Hood.java` | Dos servos con la ley ángulo = -gap + δ. |
| `subsystems/Flywheel.java`, `Shooter.java` | Flywheel por velocidad y tablas de tiro por tipo de pelota. |
| `feeder/Feeder.java` | Fila, copa y elevador: una pelota por etapa. |
| `feeder/BallInventory.java` | Cuenta de pelotas para G407 (máximo 4). |
| `intake/Intake.java` | Interfaz del intake: cambiar de mecanismo no rompe lo demás. |
| `intake/IntakeLengua.java` | El intake (18-nov): lengua de bandas con dedos colgantes, desplegada por un servo. Probado solo en simulación; faltan los µs del servo (CAD). |
| `intake/TongueServo.java` | Servo de la lengua: plegar directo; desplegar rápido hasta el empuje y luego en rampa. |
| `intake/ThroatCounter.java` | La garganta: cuenta para G407 y rivales para G408. |
| `sensors/` | Sensores de color REV V3 y break-beams. |
| `opmodes/` | TeleOp rojo y azul, y los OpModes de calibración y prueba. |

## El camino de la pelota

intake → garganta (cuenta) → **fila** (rodillo, BB3) → **copa** (aro en cremallera, V3
de z 50) → **etapa de arriba** (Gecko de z 128 y una loca con resorte, V3 de z 126) → flywheel.

- Disparar = prender el elevador hasta que la pelota de arriba se vaya. El tiro **solo se
  cuenta** si el flywheel lo sintió (caída de rpm) y el V3 de arriba ya no la ve.
- Rellenar = la copa sube la de abajo hasta las Gecko de z 95, el elevador a baja
  potencia la sube hasta que el V3 de arriba la vea, y la copa baja.
- La copa **nunca** sube si hay pelota esperando arriba: las dos se tocarían.
- El gap del hood se elige por el color de la pelota que va a salir. Una pelota de color
  desconocido va con el gap de NECTAR: una NECTAR por el gap de POLLEN se atora.

Cuando algo falla, el robot se detiene en vez de adivinar (la telemetría dice qué pasó):

- **V3 sin lectura** (cable o I2C): el SDK no avisa, regresa ceros y 152.4 mm, lo mismo
  que "no hay nada". El código lo detecta y lo toma como **ocupado**: la copa no sube.
- **Tiro sin confirmar** (el V3 la dejó de ver y el flywheel no sintió nada): la pelota
  puede ir atorada en el hood. No se rellena y el gap no se mueve hasta que el artillero
  la empuje (gatillo izquierdo) o diga que el camino está libre (B).
- **Rellenar que falla** dos veces seguidas se detiene; si la copa regresa vacía, la
  pelota se quedó en el tubo y se detiene a la primera. B lo reanuda.
- **Fila atorada**: después de sus intentos se apaga y da un solo empujón cada 3 s.
- **Flywheel apagado** a medio tiro: el elevador se detiene.
- **NECTAR rival** que se queda en la entrada: reversa hasta 1.5 s y luego todo quieto
  (intake y fila) hasta que el piloto la saque en reversa. Una rival solo se resta de la
  cuenta si ya se había sumado.

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

**Apagar el PWM de un servo.** En el SDK, `setPwmDisable()` borra la caché del servo y
el siguiente `setPower` o `setPosition`, aunque sea igual, lo vuelve a prender. La fila
solo manda comandos cuando cambia de modo; si no, se quedaría en 1500 µs arrastrándose.

**Cambiar el modo de un motor reenvía la última potencia.** Hay que mandar 0 antes de
cada `setMode` (con el intake F2, un `RUN_TO_POSITION` arrancaba al 100 %).

**La lengua se despliega en dos tramos** (CAD v139). De plegada al "empuje" la manivela
gira 77° sin mover la lengua (el perno corre en la ranura de la biela), así que va rápido.
De ahí a abajo va en rampa de 120°/s como máximo: al pasar la vertical la lengua cae por
su peso, y si la manivela se quedara quieta serían ~35° de caída libre; con la rampa la
alcanza sin golpe. Plegar es directo: la manivela la sube con la biela tensa y, pasada la
vertical, la lengua se recarga sola en su tope y el servo se suelta.

**La lengua del intake no se pliega sobre una pelota.** Con una pelota en la garganta,
los dedos centrales se montan en ella y, al subir, la empujarían contra la flecha del
pivote de la fila (CAD v106). Por eso con 4 pelotas la lengua se queda abajo con M_F en
BRAKE, y en INIT solo se pliega con la garganta libre.

**Lecturas en bloque en MANUAL.** `Robot` pone los hubs en `MANUAL`: cada ciclo tiene que
empezar con `robot.beginLoop()` (`initStep` ya lo hace). Sin eso, motores y sensores
digitales se quedan con la lectura vieja.

**Los V3 miden cada 25 ms** (de fábrica, cada 100 ms; se cambia en cada INIT). Su
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
| Control Hub | I2C bus 1 | `colorCopa` | REV Color Sensor V3 (z 50) |
| Control Hub | I2C bus 2 | `colorArriba` | REV Color Sensor V3 (z 126) |
| Control Hub | I2C bus 3 | `colorGarganta` | REV Color Sensor V3 (entrada de la fila) |
| Control Hub | Digital 3 | `bbBoca` | Digital Device (BB1) |
| Control Hub | Digital 5 | `bbGarganta` | Digital Device |
| Control Hub | Digital 7 | `bbFila` | Digital Device (BB3) |
| Expansion Hub | Motor 0 + encoder 0 | `flywheel` | goBILDA 5203-2402-0001 (1:1) |
| Expansion Hub | Motor 1 + encoder 1 | `intake` | goBILDA 5203-2402-0005 (5.2:1), M_F |
| Expansion Hub | Motor 2 | `elevador` | goBILDA 5203-2402-0005 (5.2:1) |
| Expansion Hub | Servo 0 | `hoodGap` | Servo (2000-0025-0002) |
| Expansion Hub | Servo 2 | `hoodAngulo` | Servo (2000-0025-0002) |
| Expansion Hub | Servo 4 | `lengua` | Servo (2000-0025-0002), despliega la lengua del intake |

- Los 3 V3 tienen la misma dirección I2C (0x52): cada uno en su propio bus.
- Los servos van en puertos de pares distintos (0-1, 2-3, 4-5): cada par comparte 2 A y
  así un servo trabado no tumba al otro.
- 7 de 8 motores y 5 de 8 servos (R503).

La [hoja de cableado](docs/cableado.html) es del diseño de **septiembre** (antes del CAD
v99): sirve para lo eléctrico general (XT30, RS485, pares de servos, USB 3.0), pero sus
puertos ya no son los de esta tabla.

## Pipeline del Limelight

En la interfaz web del Limelight, pipeline **0**:

- Fiducial Markers / AprilTag, familia **36h11**, tamaño **82.55 mm** (3.25 in)
- Sin filtro de IDs: el código filtra por alianza
- "Full 3D" recomendado: con un solo tag a la vista es lo único que dice si es la fila de
  arriba o la de abajo
- Crosshair en el centro: el código usa `tx_nocross` y la corrección va en `AIM_TRIM_DEG`
- Si de lejos no detecta los tags, subir la resolución (1280x960, a costa de fps)
- El código **no** cambia de pipeline durante el partido: `pipelineSwitch` es un POST HTTP
  que puede congelar el ciclo hasta 15 s. Solo avisa si el pipeline no es el 0.

## Orden de calibración

1. **Test: Sensores**: cada sensor en vivo. Ajustar `SensorCfg` (colores, distancias,
   sentido de los break-beams). Con el tubo vacío ningún V3 debe decir "SIN LECTURA" ni
   "SATURADO". La C270 usa rangos de color genéricos del SDK: revisar que vea una NECTAR
   y no las cintas del piso.
2. **Tune: Torreta** (arranca bloqueada a ±35°): **antes de engranar** el piñón, dar Play
   (el servo va a 1500 µs) y engranar con la torreta al frente. Luego `SERVO_CENTER`,
   `SERVO_INVERTED` y `DEG_PER_SERVO_UNIT` con escuadra. Suelta (bumpers + X) solo llega a
   ±105° de Config. Con el Limelight sin montar o desconectado.
3. **Tune: Hood**: ángulo cero, topes del gap. Ir a NECTAR está bloqueado hasta confirmar
   el signo del ángulo (gatillos, dpad der.) y el sentido del gap con un movimiento de
   2.5° (dpad izq., botón del stick der.). La primera vez, hood a mano en POLLEN antes de INIT.
4. **Tune: Copa**: encontrar el piso con la corriente y marcarlo con X (a ±60 µs de
   `COPA_DOWN_US`); `COPA_UP_US` a 54 mm con vernier. Se limita a 54 mm de cálculo sobre
   el piso marcado.
5. **Tune: Alimentador**: signo de `FILA_POWER_IN`, sentido del elevador, ciclo completo
   (A solo dispara con el flywheel en velocidad y el hood en su lugar).
6. **Tune: Lengua**: µs de plegada, empuje y abajo sin empujar los topes, con la misma
   secuencia del robot (el sentido del PWM no está verificado: pasos cortos), sentido de
   M_F y la velocidad real del servo (`SERVO_DEG_S`).
7. IMU: con el logo del hub hacia arriba y el USB hacia atrás (`ImuCfg`). En el TeleOp,
   girar a la izquierda tiene que subir el yaw.
8. Medir `CAMERA_HEIGHT_IN`, `CAMERA_PITCH_DEG` y las alturas de las filas de tags.
9. **Tune: Tiro**: primero revisar que las alturas medidas de las filas den ~49.9 y
   ~35.6 in. Si salen debajo de la cámara (13.4 in) o negativas, `TY_SIGN = -1`. Luego
   llenar `POLLEN_TABLE` y `NECTAR_TABLE` en cinco o seis distancias.
10. Medir la velocidad real de la torreta (`MAX_SPEED_DEG_S`) y ajustar el PIDF del
   flywheel y `SHOT_DIP_RPM`.

## Controles del TeleOp

En INIT el robot se prepara solo (torreta al frente, hood a su casa, copa abajo, lengua
plegada y suelta). Con una pelota en la garganta la lengua no se pliega y lo avisa: hay
que sacarla. El artillero indica con su dpad cuántas pelotas van precargadas.

| Gamepad 1, piloto | |
|---|---|
| Sticks | Chasis; gatillo derecho = lento |
| Bumper derecho | Intake sí/no |
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
- Hood con servos de distinta velocidad: δ nunca sale de la envolvente.
- El piloto contra la visión: su X manda hasta 10 fotos seguidas en contra.
- Flywheel: bajar la consigna no es tiro; una caída a velocidad sí, una vez.
- Alimentador con un modelo de pelotas: 4 tiros, sin subir la copa con pelota arriba, y
  con fallas a propósito: V3 sin lectura, parpadeo del V3, pelota atorada antes del
  flywheel, flywheel apagado a medio tiro, rellenado que falla y fila atorada.
- Intake: rechazo de NECTAR rival (contada, sin contar y que se queda), rebotes en la
  garganta, la 4a pelota sobre el haz y atascos.
- Lengua: plegada y suelta en INIT (y no se pliega con pelota en la garganta), se queda
  plegada al dar Play con 4 pelotas, baja y solo entonces jala, con 4 se queda abajo en
  BRAKE, y con una rival primero baja y luego hace reversa.

Lo que la simulación **no** prueba: fricción, rebotes, el vuelo de la pelota, la luz del
campo, ni que el robot real tenga las medidas del CAD. Comparte supuestos con el código
(la geometría del manual, las convenciones del Limelight), así que no atrapa errores en
esos supuestos.

## Compilar e instalar

Abrir la carpeta `robot/` en Android Studio y correr `TeamCode` sobre el Control Hub.

## Pendiente

- Todos los valores MEDIR y CALIBRAR de `Config.java`.
- Confirmar el reparto de puertos y rehacer la hoja de cableado con él.
- Intake de la lengua: faltan los µs de plegada, empuje y abajo (llegan con el documento
  del CAD v138; el CAD propone ≈1998 / 1484 / 1002, sin verificar el sentido del PWM), la
  velocidad real del servo y calibrar la C270 en su lugar nuevo.
- Decidido por Adrián (19-nov): con 4 pelotas la lengua se queda abajo con M_F en BRAKE, y
  la precarga es de 3 POLLEN adentro (2 en el tubo y 1 en la fila) con la 4a en la loseta
  tocando el frente; el autónomo la mete al desplegar. En el TeleOp la lengua baja con el
  intake prendido y menos de 4 pelotas (propuesta del CAD), y no se vuelve a plegar en el
  partido (el plan lo permitiría con 3 o menos y la garganta libre, p. ej. para defensa).
- Autónomo.

## Otros

El intake F2 (rotor de aletas, estacionado con imán) salió del CAD en la v106; su código
quedó en el historial de git (último commit con F2: `08390d5`).

`rampa/render_vtk.py` es una función para renderizar piezas de build123d o STL con VTK.
No es parte del código del robot.
