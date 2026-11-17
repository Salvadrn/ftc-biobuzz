package org.firstinspires.ftc.teamcode;

/**
 * Qué pelota hay en un lugar del robot.
 *
 * POLLEN es amarilla y mide unas 2.8 in. NECTAR mide unas 3.6 in y es roja o azul
 * según la alianza (manual BIOBUZZ TU09, sección 9.8). El tamaño decide el gap del
 * hood; el color decide si es nuestra o del rival (G408).
 */
public enum BallType {
    NONE,
    POLLEN,
    NECTAR_RED,
    NECTAR_BLUE,
    /** Hay algo, pero el color no alcanza para decir qué es. */
    UNKNOWN;

    public boolean isBall() {
        return this != NONE;
    }

    public boolean isNectar() {
        return this == NECTAR_RED || this == NECTAR_BLUE;
    }

    /** NECTAR del otro equipo: G408 prohíbe controlarla. */
    public boolean isRivalFor(Alliance alliance) {
        return (alliance == Alliance.RED && this == NECTAR_BLUE)
                || (alliance == Alliance.BLUE && this == NECTAR_RED);
    }

    /**
     * Gap del hood que le toca. Una pelota desconocida va con el gap de NECTAR:
     * una NECTAR por el gap de POLLEN se atora (28 mm de apriete), una POLLEN por el
     * gap de NECTAR solo sale floja.
     */
    public boolean needsNectarGap() {
        return this != POLLEN;
    }

    public static BallType nectarOf(Alliance alliance) {
        return alliance == Alliance.RED ? NECTAR_RED : NECTAR_BLUE;
    }
}
