package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config.HoodCfg;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.util.StateStore;

/**
 * Hood de gap variable del CAD v99, con DOS servos 2000-0025-0002 en modo posición.
 *
 *  - Servo de GAP: gira el eslabón alrededor de Q. Gap 0° = POLLEN (63.0 mm de luz),
 *    gap -34.3° = NECTAR (79.7 mm). El CAD permite -34.5 a +0.5.
 *  - Servo de ÁNGULO: va montado EN el eslabón y gira el hood alrededor de P.
 *
 * LEY OBLIGATORIA (guía del CAD, 17-nov): comando del servo de ángulo = -gap + δ, donde δ
 * es la inclinación real del hood. Así, al abrir el gap el hood se traslada sin girar.
 * Si el servo de ángulo no compensa, el hood gira con el eslabón:
 *  - gap -34.3 con ángulo 0 (δ = -34.3): la lámina se mete en la Limelight; ya con δ ≤ -20 le pega.
 *  - gap 0 con ángulo ≥ 35.4: la lámina toca las FlatTop.
 * Envolvente segura: δ entre -10° y +10° con cualquier gap. En uso, δ de -10° a 0°.
 *
 * Por eso aquí:
 *  1. Los dos servos se escriben SIEMPRE juntos, en cada ciclo, con la ley.
 *  2. El gap no salta: primero δ va a TRANSIT_DELTA_DEG, luego el gap se mueve en rampa
 *     (GAP_RATE_DEG_S, mucho más lento que los servos) y al final δ va a lo pedido. Si un
 *     servo se atrasa, el error se va hacia el flywheel, no hacia la Limelight (CAD v104:
 *     δ no tiene topes físicos; la Limelight está a -18.3° en NECTAR).
 *  3. Cada servo tiene setPwmRange: aunque haya un error, no puede salir de su recorrido.
 *  4. Al arrancar, los servos reciben primero la posición en la que quedaron la última
 *     vez (se guarda en el Control Hub). Así no hay un salto a ciegas.
 */
public class Hood {
    private static final String KEY_GAP = "hood.gap";
    private static final String KEY_DELTA = "hood.delta";

    private final Servo gapServo;
    private final Servo angleServo;
    private final StateStore store;

    private double gapCmdDeg, deltaCmdDeg;
    private double gapTargetDeg = HoodCfg.GAP_POLLEN_DEG;
    private double deltaTargetDeg = HoodCfg.DELTA_HOME_DEG;
    private double deltaMinDeg = HoodCfg.DELTA_USE_MIN_DEG;
    private double deltaMaxDeg = HoodCfg.DELTA_USE_MAX_DEG;

    private boolean started = false;
    private long settleUntilNs = 0;
    private long lastUpdateNs = 0;
    private long lastPersistNs = 0;
    private double persistedGap = Double.NaN, persistedDelta = Double.NaN;

    public Hood(HardwareMap hardwareMap, StateStore store) {
        this(hardwareMap.get(Servo.class, Names.HOOD_GAP_SERVO),
                hardwareMap.get(Servo.class, Names.HOOD_ANGLE_SERVO), store);
    }

    public Hood(Servo gapServo, Servo angleServo, StateStore store) {
        this.gapServo = gapServo;
        this.angleServo = angleServo;
        this.store = store;
        if (gapServo instanceof PwmControl) {
            ((PwmControl) gapServo).setPwmRange(new PwmControl.PwmRange(gapUsLow(), gapUsHigh()));
        }
        if (angleServo instanceof PwmControl) {
            ((PwmControl) angleServo).setPwmRange(new PwmControl.PwmRange(angleUsLow(), angleUsHigh()));
        }
    }

    /** Vuelve a poner los rangos PWM (después de cambiar Config desde "Tune: Hood"). */
    public void reapplyPwmRanges() {
        if (gapServo instanceof PwmControl) {
            ((PwmControl) gapServo).setPwmRange(new PwmControl.PwmRange(gapUsLow(), gapUsHigh()));
        }
        if (angleServo instanceof PwmControl) {
            ((PwmControl) angleServo).setPwmRange(new PwmControl.PwmRange(angleUsLow(), angleUsHigh()));
        }
        write();
    }

    // ---------- la ley ----------

    /** Comando del servo de ángulo, en grados relativos al eslabón. */
    public static double angleServoDeg(double gapDeg, double deltaDeg) {
        return -gapDeg + deltaDeg;
    }

    // ---------- arranque ----------

    /**
     * Primer comando: exactamente donde quedó el hood la última vez (sin movimiento).
     * Después update() lo lleva en rampa a POLLEN y δ de casa.
     */
    public void start(long nowNs) {
        gapCmdDeg = clampGap(store.getDouble(KEY_GAP, HoodCfg.GAP_POLLEN_DEG));
        deltaCmdDeg = Range.clip(store.getDouble(KEY_DELTA, HoodCfg.DELTA_HOME_DEG),
                HoodCfg.DELTA_SAFE_MIN_DEG, HoodCfg.DELTA_SAFE_MAX_DEG);
        started = true;
        lastUpdateNs = nowNs;
        settleUntilNs = nowNs + (long) (HoodCfg.START_SETTLE_S * 1e9);
        write();
    }

    // ---------- órdenes ----------

    /** Gap para la pelota que va a salir. NONE no cambia nada. */
    public void setGapFor(BallType ball) {
        if (ball == null || ball == BallType.NONE) return;
        gapTargetDeg = ball.needsNectarGap() ? HoodCfg.GAP_NECTAR_DEG : HoodCfg.GAP_POLLEN_DEG;
    }

    /** Inclinación del hood δ. Se recorta al rango de uso (o al de calibración). */
    public void setDelta(double deltaDeg) {
        deltaTargetDeg = Range.clip(deltaDeg, deltaMinDeg, deltaMaxDeg);
    }

    /** Solo para "Tune: Hood": permite toda la envolvente segura (δ de -10 a +10). */
    public void allowFullSafeRange(boolean allow) {
        deltaMinDeg = allow ? HoodCfg.DELTA_SAFE_MIN_DEG : HoodCfg.DELTA_USE_MIN_DEG;
        deltaMaxDeg = allow ? HoodCfg.DELTA_SAFE_MAX_DEG : HoodCfg.DELTA_USE_MAX_DEG;
        deltaTargetDeg = Range.clip(deltaTargetDeg, deltaMinDeg, deltaMaxDeg);
    }

    /** Llamar una vez por ciclo. Mueve las dos rampas y escribe los dos servos. */
    public void update(long nowNs) {
        if (!started) start(nowNs);
        double dt = Range.clip((nowNs - lastUpdateNs) / 1e9, 0, 0.1);
        lastUpdateNs = nowNs;

        double oldGap = gapCmdDeg, oldDelta = deltaCmdDeg;
        // Cambio de gap en tres tiempos: δ al de tránsito, el gap en rampa, δ a lo pedido.
        boolean gapMoving = gapCmdDeg != gapTargetDeg;
        double deltaGoal = gapMoving ? HoodCfg.TRANSIT_DELTA_DEG : deltaTargetDeg;
        deltaCmdDeg = approach(deltaCmdDeg, deltaGoal, HoodCfg.DELTA_RATE_DEG_S * dt);
        if (gapMoving && deltaCmdDeg == HoodCfg.TRANSIT_DELTA_DEG) {
            gapCmdDeg = approach(gapCmdDeg, gapTargetDeg, HoodCfg.GAP_RATE_DEG_S * dt);
        }
        if (gapCmdDeg != oldGap || deltaCmdDeg != oldDelta) {
            settleUntilNs = nowNs + (long) (HoodCfg.SETTLE_S * 1e9);
        }
        write();
        persist(nowNs);
    }

    /** true cuando el gap y δ ya llegaron a lo pedido y pasó el tiempo de asentarse. */
    public boolean isReady(long nowNs) {
        return started && gapCmdDeg == gapTargetDeg && deltaCmdDeg == deltaTargetDeg
                && nowNs >= settleUntilNs;
    }

    public boolean isNectarGap() {
        return gapTargetDeg == HoodCfg.GAP_NECTAR_DEG;
    }

    public double getGapDeg() { return gapCmdDeg; }
    public double getDeltaDeg() { return deltaCmdDeg; }

    // ---------- servos ----------

    private void write() {
        gapServo.setPosition(gapToPosition(gapCmdDeg));
        angleServo.setPosition(angleToPosition(angleServoDeg(gapCmdDeg, deltaCmdDeg)));
    }

    private void persist(long nowNs) {
        boolean moved = gapCmdDeg != persistedGap || deltaCmdDeg != persistedDelta;
        boolean arrived = gapCmdDeg == gapTargetDeg && deltaCmdDeg == deltaTargetDeg;
        if (moved && (arrived || nowNs - lastPersistNs > 100_000_000L)) {
            store.putDouble(KEY_GAP, gapCmdDeg);
            store.putDouble(KEY_DELTA, deltaCmdDeg);
            persistedGap = gapCmdDeg;
            persistedDelta = deltaCmdDeg;
            lastPersistNs = nowNs;
        }
    }

    // ---------- grados <-> microsegundos <-> posición ----------

    /** µs del servo de gap, interpolando entre los dos topes calibrados. */
    public static double gapToUs(double gapDeg) {
        double t = gapDeg / HoodCfg.GAP_NECTAR_DEG;   // 0 en POLLEN, 1 en NECTAR
        return HoodCfg.GAP_POLLEN_US + t * (HoodCfg.GAP_NECTAR_US - HoodCfg.GAP_POLLEN_US);
    }

    /** µs del servo de ángulo para un ángulo relativo al eslabón. */
    public static double angleToUs(double angleDeg) {
        return HoodCfg.ANGLE_ZERO_US + angleDeg * HoodCfg.ANGLE_US_PER_DEG;
    }

    private static double gapUsLow() {
        return Math.min(HoodCfg.GAP_POLLEN_US, HoodCfg.GAP_NECTAR_US);
    }

    private static double gapUsHigh() {
        return Math.max(HoodCfg.GAP_POLLEN_US, HoodCfg.GAP_NECTAR_US);
    }

    private static double angleUsLow() {
        return Math.min(angleToUs(HoodCfg.ANGLE_MIN_DEG), angleToUs(HoodCfg.ANGLE_MAX_DEG));
    }

    private static double angleUsHigh() {
        return Math.max(angleToUs(HoodCfg.ANGLE_MIN_DEG), angleToUs(HoodCfg.ANGLE_MAX_DEG));
    }

    public static double gapToPosition(double gapDeg) {
        double us = gapToUs(clampGap(gapDeg));
        return Range.clip((us - gapUsLow()) / (gapUsHigh() - gapUsLow()), 0, 1);
    }

    public static double angleToPosition(double angleDeg) {
        double a = Range.clip(angleDeg, HoodCfg.ANGLE_MIN_DEG, HoodCfg.ANGLE_MAX_DEG);
        return Range.clip((angleToUs(a) - angleUsLow()) / (angleUsHigh() - angleUsLow()), 0, 1);
    }

    private static double clampGap(double gapDeg) {
        return Range.clip(gapDeg, HoodCfg.GAP_NECTAR_DEG, HoodCfg.GAP_POLLEN_DEG);
    }

    private static double approach(double value, double target, double maxStep) {
        if (Math.abs(target - value) <= maxStep) return target;
        return value + Math.signum(target - value) * maxStep;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Hood", "gap %.1f° (%s)  δ %.1f°  servo ángulo %.1f°",
                gapCmdDeg, isNectarGap() ? "NECTAR" : "POLLEN", deltaCmdDeg,
                angleServoDeg(gapCmdDeg, deltaCmdDeg));
    }
}
