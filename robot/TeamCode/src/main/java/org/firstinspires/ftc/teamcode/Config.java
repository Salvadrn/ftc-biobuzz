package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.util.ColorClassifier;

/**
 * TODOS los números que se ajustan en el robot viven aquí.
 *
 * Etiquetas:
 *  - CAD: sale del CAD "Robot BIOBUZZ 2027 torreta centrada" (v99 a v145) o de su registro.
 *  - CALC: calculado (fichas de goBILDA/REV, manual TU09). No medido.
 *  - MEDIR / CALIBRAR: de relleno. El código corre con ellos, pero el robot no va a
 *    funcionar bien hasta poner el valor real. Los OpModes "Tune: ..." sirven para eso.
 *
 * Sentido de los servos goBILDA serie 2000: las fichas dicen "Clockwise" al subir el PWM
 * sin decir desde qué lado. Todo el CALC de aquí lo supone VISTO DESDE EL ESTRIADO (como la
 * copa y el gap). Se confirma en el banco con un servo suelto (README, pruebas en el robot,
 * paso 1); si sale al revés se voltean JUNTOS: SERVO_INVERTED de la torreta, GAP_NECTAR_US
 * (1500 + 36.1/0.15), ANGLE_US_PER_DEG (+6.667), los µs de la lengua (1998/1484/1002),
 * FILA_POWER_IN (-1) y el armado de la copa (calar con el aro en el piso a ~640 µs).
 *
 * Nada de este archivo se ha medido en el robot físico.
 */
public final class Config {
    private Config() {}

    /** Nombres tal cual van en la configuración del robot en la Driver Station. */
    public static final class Names {
        // Control Hub
        public static final String FRONT_LEFT = "frontLeft";
        public static final String FRONT_RIGHT = "frontRight";
        public static final String BACK_LEFT = "backLeft";
        public static final String BACK_RIGHT = "backRight";
        public static final String TURRET_SERVO = "turret";
        public static final String COPA_SERVO = "copa";
        public static final String FILA_SERVO = "fila";              // servo en giro continuo
        public static final String LIMELIGHT = "limelight";
        public static final String INTAKE_CAMERA = "c270";           // opcional
        public static final String IMU = "imu";
        public static final String COLOR_UPPER = "colorArriba";       // V3 de z 128
        public static final String COLOR_COPA = "colorCopa";          // V3 de z 52
        public static final String COLOR_THROAT = "colorGarganta";    // V3 de la entrada de la fila
        public static final String BEAM_MOUTH = "bbBoca";             // BB1
        public static final String BEAM_THROAT = "bbGarganta";
        public static final String BEAM_FILA = "bbFila";              // BB3
        // Expansion Hub
        public static final String FLYWHEEL = "flywheel";
        public static final String INTAKE_MOTOR = "intake";           // M_F
        public static final String ELEVATOR = "elevador";
        public static final String HOOD_GAP_SERVO = "hoodGap";
        public static final String HOOD_ANGLE_SERVO = "hoodAngulo";
        public static final String TONGUE_SERVO = "lengua";           // 2000-0025-0002, intake de la lengua
    }

    public static final class TurretCfg {
        /** CAD: el servo 2000-0025-0504 da 5 vueltas (1800°) de 500 a 2500 µs. */
        public static double PWM_MIN_US = 500;
        public static double PWM_MAX_US = 2500;

        /** CALC: 1800° del servo / (350T / 45T) = 231.4° de torreta en todo el rango PWM. */
        public static double DEG_PER_SERVO_UNIT = 1800.0 * 45.0 / 350.0;

        /** CALIBRAR con "Tune: Torreta": posición (0 a 1) en la que la torreta mira al frente. */
        public static double SERVO_CENTER = 0.50;

        /** CALIBRAR: true si al subir la posición del servo la torreta gira a la DERECHA. */
        public static boolean SERVO_INVERTED = false;

        /** CAD: tope por software ±105°. La guía de cables es para ±110°; más allá se jalan. */
        public static double MIN_ANGLE_DEG = -105;
        public static double MAX_ANGLE_DEG = 105;

        /**
         * MEDIR. Velocidad real de la torreta. CALC sin carga a 4.8 V: 0.055 s/60° del servo
         * = 140°/s de torreta. Si se pone muy alto, la torreta se pasa y oscila.
         */
        public static double MAX_SPEED_DEG_S = 130;

        /** Cuánto caso se le hace a cada foto nueva: 1 = todo, 0.2 = muy suave. */
        public static double AIM_SMOOTHING = 0.45;

        /** Cambios más chicos que esto no se mandan al servo (0.116° de torreta por µs). */
        public static double DEADBAND_DEG = 0.4;

        /** Diferencia entre lo ordenado y lo estimado para decir que la torreta ya llegó. */
        public static double LOCK_TOLERANCE_DEG = 2.5;

        /**
         * Error de puntería permitido en la abertura, en pulgadas. La ventana útil para una
         * NECTAR es ~5 in de cada lado de frente y ~2 in a 50° de lado. Se convierte a grados
         * con la distancia y se recorta a LOCK_TOL_MIN_DEG..LOCK_TOL_MAX_DEG.
         */
        public static double LOCK_TOLERANCE_IN = 2.5;
        public static double LOCK_TOL_MIN_DEG = 1.0;
        public static double LOCK_TOL_MAX_DEG = 4.0;

        /** La foto más vieja que todavía cuenta para decir que apunta. */
        public static double LOCK_FRAME_MAX_AGE_S = 0.3;

        /**
         * Para decir "fijo", la foto tiene que haberse tomado al menos esto DESPUÉS de que el
         * modelo dice que la torreta llegó. Si el servo real es más lento, en giros grandes
         * la foto todavía lo ve moviéndose y "fijo" espera a una foto posterior.
         */
        public static double LOCK_PHOTO_MARGIN_S = 0.1;

        /** Segundos que se sigue apuntando "de memoria" después de perder el objetivo. */
        public static double TARGET_MEMORY_S = 1.0;

        /** Velocidad del control manual. */
        public static double MANUAL_SPEED_DEG_S = 90;

        /**
         * Con el objetivo detrás (fuera de ±105°), la torreta se queda en el tope del lado en
         * que está mientras el objetivo no pase más de esto al otro lado de los 180°. Sin esto
         * brinca de tope a tope con el ruido.
         */
        public static double BEHIND_HYSTERESIS_DEG = 30;
    }

    public static final class VisionCfg {
        /** Pipeline del Limelight: AprilTag, familia 36h11, tamaño 82.55 mm (3.25 in). */
        public static int APRILTAG_PIPELINE = 0;
        public static int POLL_RATE_HZ = 100;

        /** Fotos con más retraso que esto se descartan, en milisegundos. */
        public static double MAX_LATENCY_MS = 250;

        /** CAD v99: centro del vidrio a 341.6 mm del piso con la torreta en 0. */
        public static double CAMERA_HEIGHT_IN = 341.6 / 25.4;

        /** CAD: inclinada 30° hacia arriba. MEDIR en el robot armado. */
        public static double CAMERA_PITCH_DEG = 30.0;

        /** CAD v99: el vidrio queda 76 mm DETRÁS del eje de la torreta, sobre la línea del lanzador. */
        public static double CAMERA_BEHIND_AXIS_IN = 76.0 / 25.4;

        /**
         * CALC con las cotas del manual TU09 (fig. 9-9, 9-10): altura de la fila de tags de
         * la CELL de arriba y de la de abajo. MEDIR EN CAMPO (±1.5 in).
         */
        public static double TAG_ROW_HEIGHT_UP_IN = 49.9;
        public static double TAG_ROW_HEIGHT_DOWN_IN = 35.6;

        /** Corrección fija de puntería, en grados (+ = izquierda). Se ajusta tirando. */
        public static double AIM_TRIM_DEG = 0.0;

        /**
         * CALC (manual TU09 + librería del SDK): el centro de la abertura está 3.41 in más
         * afuera que el centro de la fila de tags, perpendicular a la fila. Se apunta ahí.
         */
        public static double APERTURE_OUT_IN = 3.41;

        /**
         * CONFIRMAR en el robot: un tag ARRIBA de la mira debe dar ty positivo. La documentación
         * del Limelight se contradice. Si "Tune: Tiro" mide las filas por debajo de la cámara o
         * al revés, poner -1.
         */
        public static double TY_SIGN = 1.0;

        /**
         * Altura medida de la fila contra la mitad entre las dos filas (42.75 in): con esta
         * diferencia mínima ya se decide arriba o abajo. Se mide con la separación entre tags
         * (2 o más tags de la misma fila) o con la pose 3D.
         */
        public static double UPDOWN_HEIGHT_MARGIN_IN = 3.0;

        /** Fotos seguidas que tienen que contradecir lo que dijo el piloto para cambiarlo. */
        public static int UPDOWN_OVERRIDE_FRAMES = 10;

        /** Si pasa más que esto sin fotos que decidan, la racha se reinicia. */
        public static double UPDOWN_STREAK_GAP_S = 0.3;

        /**
         * Manual TU09, fig. 10-2: al empezar el partido están arriba la CELL roja del lado de la
         * audiencia y la azul del lado opuesto. El autónomo puede voltearlas.
         */
        public static String START_UP_RED = "RED_AUDIENCE";
        public static String START_UP_BLUE = "BLUE_SCORING";

        /**
         * Si la pose 3D está disponible ("Full 3D" en el pipeline), con un solo tag se usa la
         * altura del tag para decidir arriba/abajo. Con 2 o más tags se usa su separación.
         */
        public static boolean USE_3D_FOR_UPDOWN = true;

        /** Fotos seguidas que tienen que coincidir para cambiar cuál CELL está arriba. */
        public static int UPDOWN_CONFIRM_FRAMES = 3;

        /** Segundos que se recuerda la geometría de la fila para corregir con un solo tag. */
        public static double SLOPE_MEMORY_S = 1.0;
    }

    public static final class HoodCfg {
        /**
         * CAD: gap del eslabón para cada pelota (POLLEN 63.0 mm de luz, NECTAR 79.7 mm). El gap
         * tiene topes: el hombro Ø8 del perno corre en una ranura de 9.0 mm que lo para 0.91°
         * más allá de cada extremo, en +0.91° y -35.18° (CAD v77). Los µs se toman EN LOS
         * TOPES, así que entre ellos el eslabón gira 34.27 + 2 x 0.91 = 36.1°, y esa es la
         * compensación que tiene que hacer el servo de ángulo.
         * GAP_NECTAR_DEG: CALC de arranque -36.1; se CALIBRA en "Tune: Hood" (paso 6) hasta que
         * en NECTAR con δ 0 la luz en la punta sea igual a la del labio (~80 mm).
         */
        public static double GAP_POLLEN_DEG = 0.0;
        public static double GAP_NECTAR_DEG = -36.1;

        /**
         * CALIBRAR con "Tune: Hood": µs del servo de gap en cada tope, sin forzar. CALC de
         * relleno con 0.15°/µs (servo 2000-0025-0002: 300° de 500 a 2500 µs).
         */
        public static double GAP_POLLEN_US = 1500;
        public static double GAP_NECTAR_US = 1500 - 36.1 / 0.15;

        /**
         * CALIBRAR: µs del servo de ángulo con el hood en δ = 0 y el eslabón en su tope de
         * POLLEN, y µs por grado de δ. CALC: 1 / 0.15 = 6.67 µs por grado, con el signo
         * CONTRARIO al del gap: en el CAD el servo de gap tiene el estriado hacia -X y el de
         * ángulo hacia +X, y las dos juntas giran sobre +X (inventario v136). Con el mismo
         * signo, al ir a NECTAR la lámina gira ~69° y choca con la carcasa de la Limelight.
         * "Tune: Hood" lo confirma (paso 3): con δ +10 la PUNTA de la lámina BAJA hacia el
         * flywheel.
         */
        public static double ANGLE_ZERO_US = 1500;
        public static double ANGLE_US_PER_DEG = -1.0 / 0.15;

        /**
         * CAD: true = los dos servos montados al revés uno del otro, como en el CAD: sus µs por
         * grado tienen que ir con signo contrario, y si no, el hood no va a NECTAR (seguro).
         * Solo se cambia si el armado real difiere del CAD (por ejemplo, el de ángulo
         * volteado), y se deja escrito aquí: entonces van del mismo signo.
         */
        public static boolean SERVOS_FACE_OPPOSITE = true;

        /**
         * CAD v104: δ NO tiene topes físicos (los ±10.5° son solo límites de la junta).
         * δ positivo CIERRA: la punta de la lámina baja hacia el flywheel.
         * La lámina toca la Limelight en δ -30.8° (POLLEN), -24.9° (gap a la mitad) o
         * -18.3° (NECTAR), y las FlatTop en +35.4°, +40.1° o +44.5°. Envolvente segura y
         * rango de uso (el recorrido del servo de ángulo sale de aquí y de los gaps: de
         * -GAP_POLLEN_DEG + DELTA_SAFE_MIN_DEG a -GAP_NECTAR_DEG + DELTA_SAFE_MAX_DEG, hoy
         * -10° a +46.1°; ver Hood.angleMinDeg()):
         */
        public static double DELTA_SAFE_MIN_DEG = -10.0;
        public static double DELTA_SAFE_MAX_DEG = 10.0;
        public static double DELTA_USE_MIN_DEG = -10.0;
        public static double DELTA_USE_MAX_DEG = 0.0;
        public static double DELTA_HOME_DEG = 0.0;

        /**
         * δ mientras el gap cambia. Primero δ va aquí, luego se mueve el gap, luego δ va a lo
         * pedido. Si un servo se atrasa respecto al otro, el error se va hacia el flywheel:
         *  - con -5°, el servo de gap se puede adelantar hasta 13° antes de tocar la Limelight
         *    (NECTAR, -18.3°);
         *  - si el servo de gap no se mueve (cable suelto), al ir a NECTAR δ queda en
         *    36.1 - 5 = 31.1°: 4.3° antes de las FlatTop (+35.4° en POLLEN).
         */
        public static double TRANSIT_DELTA_DEG = -5.0;

        /**
         * Rampas. Los servos sin carga van a ~240°/s; con 60°/s los dos siguen la rampa casi
         * igual y δ no se sale de la envolvente al cambiar de gap. MEDIR que así sea.
         */
        public static double GAP_RATE_DEG_S = 60;
        public static double DELTA_RATE_DEG_S = 60;

        /**
         * Cambios de δ más chicos que esto no se mandan: cada foto trae una distancia un poco
         * distinta y, sin esto, el hood nunca terminaría de asentarse.
         */
        public static double DELTA_DEADBAND_DEG = 0.3;

        /** Tiempo para asentarse después de moverse, y al arrancar. */
        public static double SETTLE_S = 0.15;
        public static double START_SETTLE_S = 0.4;
    }

    public static final class FlywheelCfg {
        /** CAD: Yellow Jacket 5203-2402-0001, 1:1, 28 cuentas por vuelta. */
        public static double TICKS_PER_REV = 28.0;
        public static double MAX_RPM = 5800;
        public static boolean REVERSED = false;   // CALIBRAR

        /** PIDF de velocidad. F = 32767 / (6000 rpm * 28 / 60 = 2800 cuentas/s). CALIBRAR P. */
        public static double VEL_P = 10.0;
        public static double VEL_I = 0.0;
        public static double VEL_D = 0.0;
        public static double VEL_F = 11.7;

        /** Diferencia de rpm permitida para decir que ya está en velocidad. */
        public static double RPM_TOLERANCE = 75;

        /**
         * MEDIR antes de confiar en él: caída de rpm que confirma que una pelota pasó. CALC
         * (inercia del flywheel y energía de la pelota, sin pérdidas por apriete): como mínimo
         * 60-100 rpm con POLLEN y 120-190 con NECTAR. Con 250 puede que ningún tiro se confirme
         * y el alimentador se quede en "tiro sin confirmar" (no rellena hasta B del gamepad 2).
         * "Tune: Tiro" muestra la caída de cada tiro: poner esto abajo de la más chica medida.
         */
        public static double SHOT_DIP_RPM = 250;

        /**
         * Si la consigna cambió más que esto desde que estuvo en velocidad (la tabla pidió otras
         * rpm), una caída no se cuenta como tiro. Antes era SHOT_DIP_RPM / 2: así bajar el
         * umbral no achica esta tolerancia.
         */
        public static double DIP_TARGET_TOL_RPM = 125;
    }

    public static final class ShotCfg {
        /** RPM con el flywheel prendido y sin distancia todavía. */
        public static double IDLE_RPM = 2500;

        /** Qué tanto fuera de la tabla todavía se acepta tirar, en pulgadas. */
        public static double TABLE_MARGIN_IN = 6;

        /**
         * CALIBRAR con "Tune: Tiro". Filas {distancia en in desde el eje de la torreta a la
         * fila de tags, rpm, δ del hood en grados (-10 a 0)}, de menor a mayor distancia.
         * La POLLEN y la NECTAR tienen tablas separadas.
         */
        public static double[][] POLLEN_TABLE = {
                {30, 2400, 0},
                {50, 2700, -2},
                {70, 3000, -4},
                {90, 3350, -6},
                {110, 3700, -8},
        };
        public static double[][] NECTAR_TABLE = {
                {30, 2600, 0},
                {50, 2950, -2},
                {70, 3300, -4},
                {90, 3650, -6},
                {110, 4000, -8},
        };
    }

    public static final class FeederCfg {
        /** CAD: servo de la fila en giro continuo (programador 3102-0001-0001), 1000 a 2000 µs. */
        public static double FILA_PWM_MIN_US = 1000;
        public static double FILA_PWM_MAX_US = 2000;
        /** CALIBRAR el SIGNO en el banco: el par 24T:24T invierte el giro. */
        public static double FILA_POWER_IN = 1.0;
        public static double FILA_JAM_S = 1.0;
        public static double FILA_UNJAM_S = 0.25;
        public static int FILA_UNJAM_MAX_TRIES = 2;

        /**
         * CALIBRAR con vernier en "Tune: Copa". Subir = bajar PWM. CAD: abajo ~2360 µs (aro en
         * el piso); arriba = 2360 - 258.4° / 0.15°/µs ≈ 637 µs. Posición 0 = arriba, 1 = abajo.
         */
        public static double COPA_UP_US = 637;
        public static double COPA_DOWN_US = 2360;

        /** CAD: carrera de uso 54 mm (tope físico en 57: nunca mandar más de 54). */
        public static double COPA_TRAVEL_MM = 54;
        /** CAD: rampa de ≤ 50 mm/s en los últimos 5 mm de subida. */
        public static double COPA_SLOW_ZONE_MM = 5;
        public static double COPA_SLOW_MM_S = 50;
        /** Más lento que el servo real, para que la rampa lenta de arriba sí se cumpla. */
        public static double COPA_FAST_MM_S = 120;
        /**
         * Velocidad que se SUPONE para decir que la copa ya llegó. Pesimista a propósito: CALC
         * sin carga a 5 V da ~236 mm/s; con pelota y fricción será menos. MEDIR.
         */
        public static double COPA_SERVO_MM_S = 100;
        public static double COPA_SETTLE_S = 0.1;
        /** Tiempo abajo antes de soltar el servo (setPwmDisable, el piso carga el peso). */
        public static double COPA_DOWN_RELAX_S = 0.3;
        /** La pelota tiene que estar quieta en la copa este tiempo antes de subirla. */
        public static double COPA_SEATED_S = 0.15;
        public static double COPA_MOVE_TIMEOUT_S = 1.5;

        /** CAD: motor 5203-2402-0005 (1150 rpm), 1:1 a las dos flechas de la columna. */
        public static boolean ELEVATOR_REVERSED = false;   // CALIBRAR: + tiene que subir
        public static double ELEVATOR_FIRE_POWER = 0.8;
        /** Las Gecko de 32 mm van a ~1.9 m/s a toda potencia: baja para acomodar arriba. */
        public static double ELEVATOR_STAGE_POWER = 0.25;
        /** Solo en "Tune: Alimentador", con la copa abajo y VACÍA. */
        public static double ELEVATOR_REVERSE_POWER = 0.2;

        /** Disparo: tiempo después de la caída de rpm / de que el V3 de arriba la pierde. */
        public static double FIRE_AFTER_DIP_S = 0.08;
        public static double FIRE_AFTER_LOST_S = 0.25;
        public static double FIRE_TIMEOUT_S = 1.0;
        public static double LIFT_TIMEOUT_S = 1.5;

        /*
         * Fin de LIFT (documento del CAD v138, 4.7). El V3 de arriba (lente en z 128) ve la
         * pelota ANTES de que llegue a su etapa: la distancia baja hasta un mínimo con el centro
         * de la pelota a la altura del lente y vuelve a subir. La pelota tiene que quedar con el
         * centro entre z 144.8 (para no tocar una NECTAR en la copa) y el contacto con el
         * flywheel (NECTAR 156.5, POLLEN 170.8): 11.7 mm con NECTAR sobre NECTAR. Por eso, ya
         * vista, sube despacio, se espera a que la distancia suba LIFT_PASS_MM sobre su mínimo
         * (el centro ya pasó el lente) y se sigue LIFT_EXTRA_MM más. Mientras tanto el V3 de
         * arriba se lee en cada ciclo. Todo CALC de relleno: se mide en "Tune: Alimentador". Si
         * en el robot no se puede dejar dentro de esa ventana, avisar al CAD (límite mecánico).
         */
        /** CALIBRAR: potencia del elevador ya que el V3 de arriba vio la pelota. */
        public static double LIFT_SEAT_POWER = 0.10;
        /** CALC sin carga: 1150 rpm x 0.10 en Gecko de 32 mm ≈ 190 mm/s. MEDIR con pelota. */
        public static double LIFT_SEAT_MM_S = 190;
        /** CALIBRAR: subida sobre el mínimo que dice que el centro ya pasó el lente (CALC: con 1 mm, ~10 mm arriba). */
        public static double LIFT_PASS_MM = 1.0;
        /** CALIBRAR: lecturas nuevas seguidas, viéndola, arriba del mínimo + LIFT_PASS_MM (contra el ruido). */
        public static int LIFT_PASS_SAMPLES = 2;
        /**
         * Ya vista, lecturas nuevas seguidas SIN verla para detenerse: o fue un parpadeo largo o
         * ya salió del cono del V3 por arriba (con 30°, eso es arriba de z 159: ya en el flywheel).
         */
        public static int LIFT_LOST_SAMPLES = 3;
        /**
         * CALC → CALIBRAR. Se cuenta desde que se nota la subida, no desde z 128: el documento
         * del CAD pide ~17 mm desde el lente (hasta z ~145) y la subida se nota ~10 mm arriba (y
         * se confirma con LIFT_PASS_SAMPLES lecturas).
         */
        public static double LIFT_EXTRA_MM = 6;
        /**
         * CALC: tope de recorrido de la subida, desde la copa arriba, por si la distancia nunca
         * marca bien su mínimo. Del centro con la copa arriba (z 106.9 NECTAR, 94.9 POLLEN) al
         * contacto con el flywheel (156.5 y 170.8), menos ~3 mm. El recorrido se estima con la
         * potencia y LIFT_SEAT_MM_S (sin carga): si el elevador real es más lento, para más abajo.
         */
        public static double LIFT_MAX_NECTAR_MM = 47;
        public static double LIFT_MAX_POLLEN_MM = 72;
        /** Cada cuánto se lee el V3 de arriba mientras sube la pelota (0 = cada ciclo). */
        public static double LIFT_READ_INTERVAL_S = 0.0;
        /**
         * CALIBRAR: arriba tiene que verse vacío este tiempo seguido antes de subir la copa. Un
         * parpadeo del V3 (en un tiro se ven de 60 ms) no basta para subirla contra la de arriba.
         */
        public static double UPPER_EMPTY_CONFIRM_S = 0.2;

        /**
         * MEDIR en "Tune: Alimentador": altura de la copa hasta la que el V3 de arriba todavía
         * no ve la pelota de la copa. Al subir la copa solo se aborta si el V3 de arriba ve algo
         * con la copa debajo de esto: más arriba lo que ve puede ser la pelota que va subiendo
         * (NECTAR a 12.5 mm con la copa arriba). "Hay pelota arriba" se decide antes de subir,
         * con la copa abajo. Arranque: 10 mm (CALC: con la copa abajo la NECTAR queda a 54° del
         * eje del V3; con la copa a 27 mm, a 41°).
         */
        public static double COPA_UPPER_BLIND_MM = 10;

        /**
         * CALC → CALIBRAR: con la copa ocupada la fila se detiene en el primer corte de BB3, con
         * la 3a a 28-33 mm de su lugar del CAD (y -112). Sigue esto más para asentarla
         * (~0.23 m/s). Sin asentar, una NECTAR se queda cortando el haz de la garganta (centro
         * en y -145; el haz la ve hasta -139), y la 4a no tendría dónde quedarse.
         */
        public static double FILA_SEAT_S = 0.12;

        /**
         * Una caída de rpm cuando la pelota todavía se ve arriba más de esto después no es el
         * tiro (golpe, cambio de consigna): se descarta.
         */
        public static double DIP_EARLY_S = 0.15;

        /** Intentos de rellenar que pueden fallar seguidos antes de esperar al piloto. */
        public static int REFILL_MAX_FAILS = 2;
        /**
         * Tras un rellenado fallido, con la copa abajo se espera esto para ver si la pelota
         * regresó a la copa. Si no regresó, se quedó en el tubo: se detiene a la primera.
         */
        public static double LIFT_CHECK_S = 0.3;

        /** Después de "fila atorada" se apaga; vuelve a intentar al cambiar algo o tras esto. */
        public static double FILA_JAM_COOLDOWN_S = 3.0;
    }

    /** Garganta del intake: break-beam que cuenta y V3 que dice el tipo. */
    public static final class ThroatCfg {
        /** Una pelota tiene que cortar el haz al menos esto para contarse. */
        public static double MIN_S = 0.03;
        /** Y el haz tiene que quedar libre al menos esto antes de contar la siguiente (rebotes). */
        public static double CLEAR_S = 0.06;
    }

    /**
     * Intake de la lengua de bandas con dedos colgantes (CAD v107-v118, revisada en la
     * v138-v139; sustituye a F2, que salió en la v106). M_F mueve las bandas y los 4 dedos; un
     * servo 2000-0025-0002 con manivela de r 16 y biela con ranura despliega la lengua.
     */
    public static final class IntakeLenguaCfg {
        /**
         * + = jalar, como SUPUESTO: el CAD no da el signo del motor (depende de su polaridad y
         * del par de ingletes). CONFIRMAR en "Tune: Lengua": el tramo de abajo de las bandas
         * tiene que jalar hacia adentro (+Y).
         */
        public static boolean REVERSED = false;
        /** CALC: al 100 % la cadena de M_F lleva los dedos a ~1.53 m/s sin carga. */
        public static double IN_POWER = 1.0;

        /**
         * SEGURO (documento del CAD v138, 4.3): mientras sea false, el TeleOp NO mueve el servo
         * de la lengua ni M_F (en INIT avisa que se pliegue a mano). Solo "Tune: Lengua" los
         * mueve. Se pone en true después de calibrar con "Tune: Lengua" y copiar los µs aquí.
         * Con los µs de cálculo no basta: el calado del estriado puede quedar hasta ±48 µs
         * (medio diente) fuera, más que los 36 µs que hay entre plegada y el punto en que el
         * servo jala la lengua contra su tope cerca del punto muerto de la biela.
         */
        public static boolean CALIBRATED = false;

        /**
         * CALC → CALIBRAR con "Tune: Lengua" (documento del CAD v138, 4.3). µs del servo, con
         * µs = 1500 - (ψ + 48.15) / 0.15: ψ es el ángulo de la manivela desde +Y (atrás) hacia
         * +Z (arriba), visto desde +X, y el estriado mira a +X; con la ficha vista desde el
         * estriado, más PWM = horario = ψ baja (la convención de la copa y del gap).
         *  - Plegada (inicio), ψ +26.5°: recargada por su peso en el tope de atrás; PWM apagado.
         *  - Empuje, ψ -50.5°: de plegada a aquí la manivela gira 77° y la lengua no se mueve
         *    (el perno corre en la ranura de la biela); desde aquí la biela la empuja.
         *  - Abajo, ψ -122.8°: la lengua se apoya en su tope por su peso (la ranura deja 1.11
         *    mm: el servo no la carga). PWM prendido.
         * Si el banco da el sentido al revés, se reflejan alrededor de 1500: 1998 / 1484 / 1002.
         * Con 4 pelotas (G407) se queda ABAJO con M_F en BRAKE: subirla empujaría la 4a contra
         * la flecha del pivote de la fila (Adrián lo decidió el 19-nov).
         */
        public static double FOLDED_US = 1002;
        public static double PUSH_US = 1516;
        public static double DOWN_US = 1998;
        /** CAD: el estriado se cala con la manivela a medio camino, ψ -48.15°, recibiendo esto. */
        public static double SPLINE_SET_US = 1500;
        /**
         * Velocidad que se SUPONE del servo para decir "ya llegó" (no da su posición). Ficha:
         * 240°/s sin carga a 4.8 V; aquí, pesimista. MEDIR cronometrando en "Tune: Lengua".
         */
        public static double SERVO_DEG_S = 200;
        /**
         * Plan de la lengua del CAD (plan_lengua_final.json, A/decisiones[11]): de empuje a
         * abajo, rampa de 120°/s como máximo. Al pasar la vertical la lengua cae por su peso;
         * con la rampa, la manivela va bajando y la lengua la alcanza sin golpe (quieta, serían
         * ~35° de caída libre).
         */
        public static double DEPLOY_RAMP_DEG_S = 120;
        /** Ya en su lugar, este tiempo más antes de darla por quieta. */
        public static double SETTLE_S = 0.1;

        /**
         * CALC (plan de la lengua, E/problemas[10]): límite de corriente de M_F, ≈0.42 N·m con
         * el 5203-2402-0005 (9.2 A y 0.775 N·m en paro). Con M_F en paro, el balero del cubo de
         * Q llega a su capacidad estática y el servo de la lengua ve hasta 64 % de su paro.
         */
        public static double MF_CURRENT_LIMIT_A = 5.0;
        /** CALIBRAR: arriba del límite este tiempo seguido y M_F se detiene (BRAKE) y avisa. */
        public static double MF_OVERCURRENT_S = 0.2;

        /** Atasco (la misma regla que tenía F2, CAD): BB1 más de esto sin que la garganta cuente. */
        public static double UNJAM_MOUTH_S = 0.25;
        public static int UNJAM_MAX_TRIES = 3;
        public static double UNJAM_MIN_MS = 60;
        public static double UNJAM_MAX_MS = 100;

        /**
         * CALIBRAR (documento del CAD v138, 4.5). Con arriba, copa y fila ocupadas, la 4a se
         * queda en la boca sin cortar el haz de la garganta (le faltan 7-24 mm) y corta BB1 en
         * todos los casos (11-19 mm): una pelota en BB1 de forma continua esto es la 4a y el
         * robot está LLENO. Solo BB1: el haz de la garganta lo puede estar cortando la 3a misma
         * (una NECTAR sin asentar). Menor que UNJAM_MOUTH_S, y además, con la fila sin jalar,
         * una pelota en BB1 espera: no es atasco.
         */
        public static double FULL_MOUTH_S = 0.2;

        /**
         * G408: reversa de M_F y de la fila con la lengua ABAJO (plegada, los dedos no tocan la
         * pelota). Las reglas de F2 (el plan de la lengua no las redefine). CALC: una rival
         * regresa 135-180 mm, del V3 de la garganta a pasar el primer toque de los dedos: ~0.1-
         * 0.14 s a la velocidad de las bandas, ~0.18-0.27 s si rueda. CALIBRAR.
         */
        public static double REJECT_S = 0.4;
        public static double REJECT_MAX_S = 1.5;
    }

    /**
     * Cámara C270 que busca NECTAR rival frente al intake (CAD v116-v127, esquina +X). Su
     * pose del CAD, (x 178, y -220, z 160) mm con 15° hacia abajo y 10° hacia -X, es el
     * centro de la CARA de una caja provisional: el lente real queda a ~17 mm de lado y la
     * cámara real apuntaría 2-3.8° más arriba. Falta medirla (fondo, lado del lente, cuña del
     * asiento y FOV de 55 o 60°). NO calibrar con la pose vieja de la visera de F2.
     */
    public static final class IntakeCameraCfg {
        public static int WIDTH_PX = 320;
        public static int HEIGHT_PX = 240;
        /**
         * Solo la mitad de abajo de la imagen (el piso cerca del intake). CALC: ve centros de
         * NECTAR de y ≈ -380 a -670 y, cerca del robot, solo la mitad +X de la boca. Después de
         * medir la cámara conviene usar el cuadro completo y quedarse con la zona de la boca.
         */
        public static double ROI_TOP = 0.0;
        /**
         * CALC → CALIBRAR: a 320x240 una NECTAR a 60 cm mide ~2,200-2,700 px² (FOV de 60° a
         * 55°); 400 px² sería una a ~1.5 m.
         */
        public static double MIN_AREA_PX = 2500;
        public static double MAX_AREA_PX = 40000;
        public static double MIN_CIRCULARITY = 0.6;
        /** Vista en vivo en la Driver Station: útil para calibrar, apagada en competencia. */
        public static boolean LIVE_VIEW = false;
    }

    public static final class SensorCfg {
        /** CALIBRAR: con pull-up, haz cortado = true. Si sale al revés, cambiar a false. */
        public static boolean BEAM_ACTIVE_HIGH = true;

        /**
         * Ganancia de los V3 (es de software: el SDK multiplica y recorta a 1, así que con
         * mucha ganancia los canales se saturan y el color sale mal) y cada cuánto se lee
         * cada uno. El sensor se pone a medir cada 25 ms (de fábrica, cada 100 ms).
         */
        public static float COLOR_GAIN = 2f;
        public static double COLOR_MIN_INTERVAL_S = 0.04;

        /**
         * El V3 de arriba mide distancia cada 12.5 ms (los otros cada 25): el fin de LIFT depende
         * de notar pronto que la pelota ya pasó el lente (FeederCfg.LIFT_*).
         */
        public static boolean UPPER_FAST_PROXIMITY = true;

        /**
         * CALIBRAR con "Test: Sensores". {presente a menos de mm, NECTAR a menos de mm (NaN =
         * no usar el tamaño), saturación mínima, brillo mínimo}.
         * CALC (CAD v131): en la copa, NECTAR a 4.5 mm y POLLEN a 16.2 mm en el aro. En la
         * garganta el tamaño no separa una de otra (paredes de la fila en x ±49): NECTAR de 8.3
         * a 14.1 mm y POLLEN de 6.3 a 33.1 mm según de qué lado vaya; ahí G408 depende solo del
         * color. Arriba: lo que lea fuera del eje (la pelota de la copa subiendo) hay que MEDIRLO.
         */
        public static ColorClassifier.Thresholds UPPER = new ColorClassifier.Thresholds(50, Double.NaN, 0.25, 0.02);
        public static ColorClassifier.Thresholds COPA = new ColorClassifier.Thresholds(35, 10, 0.25, 0.02);
        public static ColorClassifier.Thresholds THROAT = new ColorClassifier.Thresholds(45, Double.NaN, 0.25, 0.02);
    }

    public static final class DriveCfg {
        /** Multiplicador de velocidad con el gatillo de precisión. */
        public static double SLOW_FACTOR = 0.4;
    }

    public static final class ImuCfg {
        public static boolean USE_IMU = true;
        /**
         * CONFIRMAR en el robot. Las hubs van planas, giradas 90° con los puertos de motor
         * hacia afuera (CAD v90). Como solo se usan cambios de yaw, lo que importa es si el
         * logo va arriba o abajo; hacia dónde apuntan los USB solo suma un ángulo fijo.
         */
        public static RevHubOrientationOnRobot.LogoFacingDirection LOGO =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static RevHubOrientationOnRobot.UsbFacingDirection USB =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;   // CAD v104: USB hacia atrás
    }
}
