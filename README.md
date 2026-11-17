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
| `intake/IntakeF2.java` | El intake F2 actual: estacionado con imán, atascos, modo lleno, G408. |
| `sensors/` | Sensores de color REV V3, break-beams y switch magnético. |
| `opmodes/` | TeleOp rojo y azul, y los OpModes de calibración y prueba. |

## El camino de la pelota

intake F2 → garganta (cuenta) → **fila** (rodillo, BB3) → **copa** (aro en cremallera, V3
de z 50) → **etapa de arriba** (Gecko de z 128 y una loca con resorte, V3 de z 126) → flywheel.

- Disparar = prender el elevador hasta que la pelota de arriba se vaya. Se confirma por
  la caída de rpm del flywheel y, de apoyo, porque el V3 de arriba la deja de ver.
- Rellenar = la copa sube la de abajo hasta las Gecko de z 95, el elevador a baja
  potencia la sube hasta que el V3 de arriba la vea, y la copa baja.
- La copa **nunca** sube si hay pelota esperando arriba: las dos se tocarían.
- El gap del hood se elige por el color de la pelota que va a salir. Una pelota de color
  desconocido va con el gap de NECTAR: una NECTAR por el gap de POLLEN se atora.

## Lo que no es obvio

**Las dos CELLS de la alianza se ven al mismo tiempo.** Los tags van en la cara de abajo
de cada CELL y, con el HIVE inclinado 30°, las dos filas miran al mismo lado. La de arriba
está a unos 49.9 in del piso y la de abajo a 35.6 in (calculado del manual). Para saber
cuál es cuál: si se ven las dos, la de arriba se ve más alta; si se ve una, por la altura
de la pose 3D o por tamaño (a la misma elevación, la de arriba está más lejos y se ve casi
3 veces más chica). El HIVE solo se voltea cuando le meten pelotas, así que la decisión
es pegajosa; el artillero la cambia con X si hace falta.

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
| Control Hub | Digital 1 | `magF2` | Digital Device (REV-31-1462, activo bajo) |
| Control Hub | Digital 3 | `bbBoca` | Digital Device (BB1) |
| Control Hub | Digital 5 | `bbGarganta` | Digital Device |
| Control Hub | Digital 7 | `bbFila` | Digital Device (BB3) |
| Expansion Hub | Motor 0 + encoder 0 | `flywheel` | goBILDA 5203-2402-0001 (1:1) |
| Expansion Hub | Motor 1 + encoder 1 | `intakeF2` | goBILDA 5203-2402-0005 (5.2:1) |
| Expansion Hub | Motor 2 | `elevador` | goBILDA 5203-2402-0005 (5.2:1) |
| Expansion Hub | Servo 0 | `hoodGap` | Servo (2000-0025-0002) |
| Expansion Hub | Servo 2 | `hoodAngulo` | Servo (2000-0025-0002) |

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
- "Full 3D" opcional: si está activo, se usa la altura del tag para decidir arriba o abajo
- Crosshair en el centro: el código usa `tx_nocross` y la corrección va en `AIM_TRIM_DEG`

## Orden de calibración

1. **Test: Sensores**: cada sensor en vivo. Ajustar `SensorCfg` (colores, distancias,
   sentido de los break-beams).
2. **Tune: Torreta** (arranca bloqueada a ±35°): `SERVO_CENTER`, `SERVO_INVERTED`, revisar
   `DEG_PER_SERVO_UNIT` con escuadra. Con el Limelight sin montar o desconectado.
3. **Tune: Hood**: µs de los dos topes del gap y del ángulo cero. La primera vez, dejar el
   hood a mano en POLLEN antes de INIT.
4. **Tune: Copa**: `COPA_DOWN_US` sin empujar el piso y `COPA_UP_US` a 54 mm con vernier.
5. **Tune: Alimentador**: signo de `FILA_POWER_IN`, sentido del elevador, ciclo completo.
6. **Tune: Intake F2**: estacionado (meta ±2°, máximo ±5°), sentido, sensores.
7. Medir `CAMERA_HEIGHT_IN`, `CAMERA_PITCH_DEG` y las alturas de las filas de tags.
8. **Tune: Tiro**: llenar `POLLEN_TABLE` y `NECTAR_TABLE` en cinco o seis distancias.
9. Medir la velocidad real de la torreta (`MAX_SPEED_DEG_S`) y ajustar el PIDF del
   flywheel y `SHOT_DIP_RPM`.

## Controles del TeleOp

En INIT el robot se prepara solo (torreta al frente, hood a su casa, copa abajo, F2
estacionado). El artillero indica con su dpad cuántas pelotas van precargadas.

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
- Alimentador con un modelo de pelotas: 4 tiros, sin subir la copa con pelota arriba.
- Intake F2: estacionado, modo lleno, rechazo de NECTAR rival y atascos.

Lo que la simulación **no** prueba: fricción, rebotes, el vuelo de la pelota, la luz del
campo, ni que el robot real tenga las medidas del CAD. Comparte supuestos con el código
(la geometría del manual, las convenciones del Limelight), así que no atrapa errores en
esos supuestos.

## Compilar e instalar

Abrir la carpeta `robot/` en Android Studio y correr `TeamCode` sobre el Control Hub.

## Pendiente

- Todos los valores MEDIR y CALIBRAR de `Config.java`.
- Confirmar el reparto de puertos y rehacer la hoja de cableado con él.
- El intake F2 se va a cambiar: la nueva clase solo tiene que implementar `Intake`.
- Autónomo.

## Otros

`rampa/render_vtk.py` es una función para renderizar piezas de build123d o STL con VTK.
No es parte del código del robot.
