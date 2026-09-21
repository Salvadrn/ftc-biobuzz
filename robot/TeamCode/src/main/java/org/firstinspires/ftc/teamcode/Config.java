package org.firstinspires.ftc.teamcode;

/**
 * TODOS los números que se ajustan en el robot viven aquí.
 *
 * Los que dicen "MEDIR" o "CALIBRAR" son de relleno: el código compila y corre con
 * ellos, pero el robot no va a atinar hasta que se pongan los valores reales.
 * Los OpModes "Tune: Torreta" y "Tune: Tiro" sirven para sacarlos.
 */
public final class Config {
    private Config() {}

    /** Nombres tal cual están en la configuración del robot en la Driver Station. */
    public static final class Names {
        public static final String LIMELIGHT = "limelight";
        public static final String TURRET_SERVO = "turret";
        public static final String HOOD_SERVO = "hood";
        public static final String FLYWHEEL = "flywheel";
        public static final String FLYWHEEL_2 = "flywheel2";   // opcional, si no existe se ignora
        public static final String IMU = "imu";
        public static final String FRONT_LEFT = "frontLeft";
        public static final String FRONT_RIGHT = "frontRight";
        public static final String BACK_LEFT = "backLeft";
        public static final String BACK_RIGHT = "backRight";
    }

    public static final class TurretCfg {
        /** MEDIR. Posición del servo (0 a 1) en la que la torreta mira al frente del robot. */
        public static double SERVO_CENTER = 0.50;

        /**
         * MEDIR. Grados que gira la TORRETA cuando el servo va de 0 a 1.
         * Ya incluye la relación de engranes o banda entre servo y torreta.
         * Ejemplo: servo de 300 grados con reducción 2:1 da 150.
         */
        public static double DEG_PER_SERVO_UNIT = 300.0;

        /** true si al subir la posición del servo la torreta gira a la DERECHA. */
        public static boolean SERVO_INVERTED = false;

        /** MEDIR. Límites de giro en grados. 0 = frente, positivo = izquierda. Dejar margen para los cables. */
        public static double MIN_ANGLE_DEG = -110;
        public static double MAX_ANGLE_DEG = 110;

        /**
         * MEDIR. Velocidad real de la torreta con carga, en grados por segundo.
         * El servo no avisa dónde está, así que el código lo estima con este número.
         * Si se pone muy alto la torreta se pasa y oscila; muy bajo y responde lento.
         */
        public static double MAX_SPEED_DEG_S = 250;

        /** Cuánto caso se le hace a cada foto nueva: 1 = todo, 0.2 = muy suave. */
        public static double AIM_SMOOTHING = 0.45;

        /** Cambios de ángulo más chicos que esto no se mandan al servo, para que no tiemble. */
        public static double DEADBAND_DEG = 0.4;

        /** Error máximo para considerar que ya está apuntando. El CELL mide 20 in de ancho. */
        public static double LOCK_TOLERANCE_DEG = 2.5;

        /** Segundos que se sigue apuntando "de memoria" después de perder el objetivo. */
        public static double TARGET_MEMORY_S = 1.0;

        /** Velocidad del control manual con el stick, en grados por segundo. */
        public static double MANUAL_SPEED_DEG_S = 140;
    }

    public static final class VisionCfg {
        /** Pipeline del Limelight configurado como AprilTag, familia 36h11, tamaño 82.5 mm. */
        public static int APRILTAG_PIPELINE = 0;

        public static int POLL_RATE_HZ = 100;

        /** Fotos con más retraso que esto se descartan, en milisegundos. */
        public static double MAX_LATENCY_MS = 250;

        /**
         * Cómo se calcula la distancia:
         * false = trigonometría con ty. Necesita las tres medidas de abajo.
         * true  = pose 3D del Limelight. Necesita "Full 3D" activado en el pipeline.
         * "Tune: Tiro" muestra las dos para comparar contra una cinta métrica.
         */
        public static boolean USE_3D_DISTANCE = false;

        /** MEDIR. Altura del lente sobre el piso, en pulgadas. */
        public static double CAMERA_HEIGHT_IN = 12.0;

        /** MEDIR. Inclinación de la cámara hacia arriba, en grados. */
        public static double CAMERA_PITCH_DEG = 30.0;

        /**
         * MEDIR EN CAMPO. Altura de la fila de tags del CELL que mira hacia arriba.
         * El HIVE es simétrico, así que vale lo mismo para los dos CELLS de la alianza.
         */
        public static double TAG_HEIGHT_IN = 40.0;

        /** Segundos que se recuerda la geometría de la fila para corregir cuando se ve un solo tag. */
        public static double SLOPE_MEMORY_S = 1.0;
    }

    public static final class ShooterCfg {
        /** Ticks del encoder por vuelta del FLYWHEEL. Motor sin caja a 1:1 = 28. */
        public static double TICKS_PER_FLYWHEEL_REV = 28.0;

        public static boolean FLYWHEEL_REVERSED = false;
        public static boolean FLYWHEEL_2_REVERSED = true;

        /** PIDF de velocidad. F sale de 32767 / ticks por segundo a máxima velocidad. */
        public static double VEL_P = 10.0;
        public static double VEL_I = 0.0;
        public static double VEL_D = 0.0;
        public static double VEL_F = 11.7;

        /** Diferencia de RPM permitida para decir que el flywheel está listo. */
        public static double RPM_TOLERANCE = 75;

        /** RPM cuando está prendido pero no hay objetivo a la vista. */
        public static double IDLE_RPM = 2500;

        /** Límites del servo del hood para no forzar el mecanismo. MEDIR. */
        public static double HOOD_MIN = 0.10;
        public static double HOOD_MAX = 0.90;

        /**
         * CALIBRAR con "Tune: Tiro". Filas {distancia en pulgadas, RPM, posición del hood},
         * de menor a mayor distancia. Entre filas se interpola en línea recta.
         * Con cinco o seis distancias bien medidas es suficiente.
         */
        public static double[][] SHOT_TABLE = {
                // dist,  rpm, hood
                {  30,   2400, 0.20},
                {  50,   2700, 0.32},
                {  70,   3000, 0.45},
                {  90,   3350, 0.55},
                { 110,   3700, 0.62},
        };
    }

    public static final class DriveCfg {
        /** Multiplicador de velocidad mientras se mantiene el gatillo de precisión. */
        public static double SLOW_FACTOR = 0.4;
    }

    public static final class ImuCfg {
        /**
         * Con IMU la torreta compensa los giros del chasis y no pierde el objetivo
         * cuando el driver rota. Si el hub está montado distinto, cambiar la orientación
         * en Robot.java.
         */
        public static boolean USE_IMU = true;
    }
}
