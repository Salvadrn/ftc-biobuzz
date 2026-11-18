package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.util.ColorClassifier;

/**
 * TODOS los números que se ajustan en el robot viven aquí.
 *
 * Etiquetas:
 *  - CAD: sale del CAD "Robot BIOBUZZ 2027 torreta centrada" v99 o de su registro.
 *  - CALC: calculado (fichas de goBILDA/REV, manual TU09). No medido.
 *  - MEDIR / CALIBRAR: de relleno. El código corre con ellos, pero el robot no va a
 *    funcionar bien hasta poner el valor real. Los OpModes "Tune: ..." sirven para eso.
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
        public static final String COLOR_UPPER = "colorArriba";       // V3 de z 126
        public static final String COLOR_COPA = "colorCopa";          // V3 de z 50
        public static final String COLOR_THROAT = "colorGarganta";    // V3 de la entrada de la fila
        public static final String BEAM_MOUTH = "bbBoca";             // BB1
        public static final String BEAM_THROAT = "bbGarganta";
        public static final String BEAM_FILA = "bbFila";              // BB3
        public static final String MAGNET_F2 = "magF2";               // REV-31-1462
        // Expansion Hub
        public static final String FLYWHEEL = "flywheel";
        public static final String INTAKE_MOTOR = "intake";           // M_F (mueve F2 o la lengua)
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
         * sí tiene tope: el perno corre en una ranura que lo para ~0.9° después de 0 y de -34.3.
         */
        public static double GAP_POLLEN_DEG = 0.0;
        public static double GAP_NECTAR_DEG = -34.3;

        /**
         * CALIBRAR con "Tune: Hood": µs del servo de gap en cada tope. CALC de relleno con
         * 0.15°/µs (servo 2000-0025-0002: 300° de 500 a 2500 µs). En NECTAR lo toma el tope
         * duro: ponerlo justo donde llega sin forzar.
         */
        public static double GAP_POLLEN_US = 1500;
        public static double GAP_NECTAR_US = 1500 - 34.3 / 0.15;

        /**
         * CALIBRAR: µs del servo de ángulo con el hood en δ = 0 y gap de POLLEN, y µs por
         * grado (signo según cómo quedó montado). CALC: 1 / 0.15 = 6.67 µs por grado.
         */
        public static double ANGLE_ZERO_US = 1500;
        public static double ANGLE_US_PER_DEG = 1.0 / 0.15;

        /** CAD: el servo de ángulo, relativo al eslabón, va de -10° a +44.3° (setPwmRange = último tope). */
        public static double ANGLE_MIN_DEG = -10.0;
        public static double ANGLE_MAX_DEG = 44.3;

        /**
         * CAD v104: δ NO tiene topes físicos (los ±10.5° son solo límites de la junta).
         * La lámina toca la Limelight en δ -30.8° (POLLEN), -24.9° (gap a la mitad) o
         * -18.3° (NECTAR), y las FlatTop en +35.4°, +40.1° o +44.5°. Envolvente segura y
         * rango de uso:
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
         *    34.3 - 5 = 29.3°: 6° antes de las FlatTop.
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

        /** CALIBRAR: caída de rpm que confirma que una pelota pasó. */
        public static double SHOT_DIP_RPM = 250;
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

    public static final class IntakeF2Cfg {
        public static boolean REVERSED = false;   // CALIBRAR: + tiene que jalar hacia adentro
        public static double IN_POWER = 1.0;

        /** CALC: 145.1 cuentas por vuelta de M_F x 30/20 de la banda = 217.65 por vuelta de F2. */
        public static double TICKS_PER_ROTOR_REV = 145.1 * 30.0 / 20.0;

        /** Estacionado (spec F2 §2.10): despacio, primero en un sentido y luego en el otro. */
        public static double PARK_SEARCH_TICKS_PER_S = 40;   // ≈ 66°/s del rotor
        public static double PARK_CLEAR_DEG = 25;
        /** El primer flanco solo cuenta si antes se vio el imán ausente al menos esto. */
        public static double PARK_ARM_DEG = 4;
        /** CALIBRAR: dónde queda 0° respecto al centro del imán (marca grabada en el disco). */
        public static double INDEX_OFFSET_DEG = 0;
        /** Tolerancia del hub en RUN_TO_POSITION: 1 cuenta = 1.65° (R102 deja ±5°). */
        public static int PARK_TOLERANCE_TICKS = 1;
        /** Se da por llegado tras estar dentro de 2 cuentas este tiempo (el hub termina de acercarse). */
        public static double GOTO_SETTLE_S = 0.06;
        public static int GOTO_ACCEPT_TICKS = 2;
        public static double PARK_STEP_TIMEOUT_S = 6;
        public static double GOTO_POWER = 0.3;

        /** Modo lleno (G407): F2 se queda en +25° en BRAKE. */
        public static double FULL_HOLD_DEG = 25;
        public static double FULL_GOTO_TIMEOUT_S = 1.0;

        /** Una pelota tiene que cortar el haz de la garganta al menos esto para contarse. */
        public static double THROAT_MIN_S = 0.03;
        /** Y el haz tiene que quedar libre al menos esto antes de contar la siguiente (rebotes). */
        public static double THROAT_CLEAR_S = 0.06;

        /** Atasco (spec F2 §2.8). */
        public static double UNJAM_MOUTH_S = 0.25;
        public static int UNJAM_MAX_TRIES = 3;
        public static double UNJAM_MIN_MS = 60;
        public static double UNJAM_MAX_MS = 100;

        /** G408: reversa de F2 y de la fila al ver una NECTAR rival adentro. */
        public static double REJECT_S = 0.4;
        /** Si la sigue viendo, la reversa sigue hasta esto; luego se detiene y avisa. */
        public static double REJECT_MAX_S = 1.5;

        /** Antes de ir a +25° en modo lleno, frena este tiempo (viene girando al 100 %). */
        public static double FULL_BRAKE_S = 0.15;
    }

    /**
     * Qué intake usa el robot. El 18-nov Adrián cambió F2 por la lengua de bandas con dedos
     * colgantes. Se queda F2 hasta que el CAD dé los ángulos del servo de la lengua; al
     * cambiar, también cambia la configuración de la Driver Station (README).
     */
    public static final class IntakeChoice {
        public static boolean USE_LENGUA = false;
    }

    /** Garganta (break-beam que cuenta y V3 que dice el tipo), para el intake de la lengua. */
    public static final class ThroatCfg {
        /** Una pelota tiene que cortar el haz al menos esto para contarse. */
        public static double MIN_S = 0.03;
        /** Y el haz tiene que quedar libre al menos esto antes de contar la siguiente (rebotes). */
        public static double CLEAR_S = 0.06;
    }

    /**
     * Intake de la lengua de bandas con dedos colgantes (CAD, 18-nov; el diseño final todavía
     * está en plan). M_F mueve las bandas y los 4 dedos con la misma cadena que tenía F2; un
     * servo 2000-0025-0002 despliega la lengua. No hay estacionado ni sensor magnético.
     */
    public static final class IntakeLenguaCfg {
        /** CAD: + = jalar (las bandas no invierten el giro, igual que F2). CONFIRMAR en el robot. */
        public static boolean REVERSED = false;
        /** CAD: al 100 % las bandas van a ~1.5 m/s sin carga. */
        public static double IN_POWER = 1.0;

        /**
         * PENDIENTE del CAD (sale del diseño de la palanca y la biela). Mientras, las tres
         * posiciones valen lo mismo y el servo no se mueve. 0.15°/µs; 1500 µs = centro.
         *  - Plegada: recargada por su peso en el tope de atrás; PWM apagado.
         *  - Abajo: en el tope duro de abajo; PWM prendido (por ahora, CAD).
         *  - Arriba (G407, 4 pelotas): PWM prendido, la gravedad la baja.
         */
        public static double FOLDED_US = 1500;
        public static double DOWN_US = 1500;
        public static double UP_US = 1500;
        /**
         * El servo no dice dónde está: "ya llegó" = este tiempo después de mandarla. CAD: 0.25 s
         * por 60° sin carga y la carga es chica; ~0.6 s un viaje completo. CONFIRMAR con el ángulo.
         */
        public static double TRAVEL_S = 0.6;

        /** Atasco, igual que F2 (CAD): BB1 más de esto sin que la garganta cuente. */
        public static double UNJAM_MOUTH_S = 0.25;
        public static int UNJAM_MAX_TRIES = 3;
        public static double UNJAM_MIN_MS = 60;
        public static double UNJAM_MAX_MS = 100;

        /**
         * G408: reversa de M_F y de la fila con la lengua ABAJO (arriba, los dedos no tocan la
         * pelota). CAD: de la garganta a pasar F son ~0.1-0.2 s; se empieza con los de F2.
         */
        public static double REJECT_S = 0.4;
        public static double REJECT_MAX_S = 1.5;
    }

    public static final class IntakeCameraCfg {
        public static int WIDTH_PX = 320;
        public static int HEIGHT_PX = 240;
        /** Solo la mitad de abajo de la imagen (el piso cerca del intake). */
        public static double ROI_TOP = 0.0;
        /** CALIBRAR: a 320x240, una NECTAR a ~60 cm mide ~400 px de área. */
        public static double MIN_AREA_PX = 400;
        public static double MAX_AREA_PX = 40000;
        public static double MIN_CIRCULARITY = 0.6;
        /** Vista en vivo en la Driver Station: útil para calibrar, apagada en competencia. */
        public static boolean LIVE_VIEW = false;
    }

    public static final class SensorCfg {
        /** CALIBRAR: con pull-up, haz cortado = true. Si sale al revés, cambiar a false. */
        public static boolean BEAM_ACTIVE_HIGH = true;
        /** REV-31-1462: activo bajo (imán presente = false). */
        public static boolean MAGNET_ACTIVE_HIGH = false;

        /**
         * Ganancia de los V3 (es de software: el SDK multiplica y recorta a 1, así que con
         * mucha ganancia los canales se saturan y el color sale mal) y cada cuánto se lee
         * cada uno. El sensor se pone a medir cada 25 ms (de fábrica, cada 100 ms).
         */
        public static float COLOR_GAIN = 2f;
        public static double COLOR_MIN_INTERVAL_S = 0.04;

        /**
         * CALIBRAR con "Test: Sensores". {presente a menos de mm, NECTAR a menos de mm (NaN =
         * no usar el tamaño), saturación mínima, brillo mínimo}.
         * CAD: en la copa, NECTAR a 4.6 mm y POLLEN a 16.1 mm; en la garganta, NECTAR a ~9 mm
         * y POLLEN a 19-33 mm.
         */
        public static ColorClassifier.Thresholds UPPER = new ColorClassifier.Thresholds(50, Double.NaN, 0.25, 0.02);
        public static ColorClassifier.Thresholds COPA = new ColorClassifier.Thresholds(35, 10, 0.25, 0.02);
        public static ColorClassifier.Thresholds THROAT = new ColorClassifier.Thresholds(45, 14, 0.25, 0.02);
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
