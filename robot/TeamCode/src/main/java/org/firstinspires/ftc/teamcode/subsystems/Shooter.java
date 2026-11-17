package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config.ShotCfg;
import org.firstinspires.ftc.teamcode.util.InterpTable;

/**
 * Junta flywheel y hood: con la distancia y el tipo de la próxima pelota decide rpm,
 * gap e inclinación del hood.
 *
 * La POLLEN (2.8 in) y la NECTAR (3.6 in) no vuelan igual, así que cada una tiene su tabla.
 */
public class Shooter {
    public final Flywheel flywheel;
    public final Hood hood;

    private final InterpTable pollenRpm, pollenDelta, nectarRpm, nectarDelta;

    private BallType nextBall = BallType.NONE;
    private double distanceIn = Double.NaN;
    private boolean manual = false;

    public Shooter(Flywheel flywheel, Hood hood) {
        this.flywheel = flywheel;
        this.hood = hood;
        pollenRpm = column(ShotCfg.POLLEN_TABLE, 1);
        pollenDelta = column(ShotCfg.POLLEN_TABLE, 2);
        nectarRpm = column(ShotCfg.NECTAR_TABLE, 1);
        nectarDelta = column(ShotCfg.NECTAR_TABLE, 2);
    }

    private static InterpTable column(double[][] table, int col) {
        double[][] pts = new double[table.length][];
        for (int i = 0; i < table.length; i++) pts[i] = new double[]{table[i][0], table[i][col]};
        return new InterpTable(pts);
    }

    /**
     * Llamar una vez por ciclo.
     * @param distanceIn distancia a la CELL desde el eje de la torreta, NaN si no se sabe
     * @param next       pelota que sigue en salir (decide gap y tabla); NONE = no cambiar el gap
     */
    public void update(long nowNs, double distanceIn, BallType next) {
        this.distanceIn = distanceIn;
        if (next != null && next != BallType.NONE) nextBall = next;

        if (!manual) {
            hood.setGapFor(nextBall);
            boolean nectar = nextBall.needsNectarGap() && nextBall != BallType.NONE;
            if (!Double.isNaN(distanceIn)) {
                flywheel.setTargetRpm((nectar ? nectarRpm : pollenRpm).get(distanceIn));
                hood.setDelta((nectar ? nectarDelta : pollenDelta).get(distanceIn));
            } else if (flywheel.getTargetRpm() == 0) {
                flywheel.setTargetRpm(ShotCfg.IDLE_RPM);
            }
        }
        flywheel.update();
        hood.update(nowNs);
    }

    /** Para "Tune: Tiro": rpm y δ a mano, ignorando las tablas. */
    public void setManual(boolean on, double rpm, double deltaDeg, BallType ball) {
        manual = on;
        if (on) {
            nextBall = ball;
            hood.setGapFor(ball);
            flywheel.setTargetRpm(rpm);
            hood.setDelta(deltaDeg);
        }
    }

    /** ¿La distancia está dentro de lo que la tabla de esa pelota cubre? */
    public boolean distanceInTable(double d) {
        if (Double.isNaN(d)) return false;
        InterpTable t = nextBall.needsNectarGap() ? nectarRpm : pollenRpm;
        return d >= t.minX() - ShotCfg.TABLE_MARGIN_IN && d <= t.maxX() + ShotCfg.TABLE_MARGIN_IN;
    }

    /** Flywheel en velocidad, hood en su lugar y una distancia que la tabla cubre. */
    public boolean isReady(long nowNs) {
        return flywheel.isAtSpeed() && hood.isReady(nowNs)
                && (manual || distanceInTable(distanceIn));
    }

    public BallType getNextBall() { return nextBall; }

    public void addTelemetry(Telemetry telemetry) {
        flywheel.addTelemetry(telemetry);
        hood.addTelemetry(telemetry);
        if (!manual && !Double.isNaN(distanceIn) && !distanceInTable(distanceIn)) {
            telemetry.addLine("!! Distancia fuera de la tabla de tiro");
        }
    }

    public void stop() {
        flywheel.stop();
    }
}
