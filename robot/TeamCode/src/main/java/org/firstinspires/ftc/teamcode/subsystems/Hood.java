package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxServoController;
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
 * Hood de gap variable del CAD (v99 a v145, sin cambios en lo que usa esto), con DOS servos
 * 2000-0025-0002 en modo posición.
 *
 *  - Servo de GAP: gira el eslabón alrededor de Q. Gap 0° = tope de POLLEN (63.0 mm de luz),
 *    GAP_NECTAR_DEG = tope de NECTAR (~80 mm). Entre los dos topes el eslabón gira 36.1°
 *    (CALC; se calibra en "Tune: Hood").
 *  - Servo de ÁNGULO: va montado EN el eslabón y gira el hood alrededor de P. Está montado al
 *    revés que el de gap (estriados hacia +X y -X): sus µs por grado llevan signo contrario.
 *
 * LEY OBLIGATORIA (guía del CAD, 17-nov): comando del servo de ángulo = -gap + δ, donde δ
 * es la inclinación real del hood (positivo cierra: la punta baja hacia el flywheel). Así, al
 * abrir el gap el hood se traslada sin girar. Si el servo de ángulo no compensa (o gira al
 * revés), el hood gira con el eslabón:
 *  - gap de NECTAR con ángulo 0 (δ ≈ -36): la lámina se mete en la Limelight; ya con
 *    δ ≤ -18.3 le pega. Con el signo del ángulo al revés, δ ≈ -69°.
 *  - gap 0 con ángulo ≥ 35.4: la lámina toca las FlatTop.
 * Envolvente segura: δ entre -10° y +10° con cualquier gap. En uso, δ de -10° a 0°.
 *
 * SEGURO (documento del CAD v138, 4.1): si los signos de los dos servos no cuadran con cómo
 * están montados (HoodCfg.SERVOS_FACE_OPPOSITE), el hood no va a NECTAR: se queda en POLLEN,
 * avisa y no se da por listo para una pelota que necesita el gap de NECTAR.
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

    /** Con los signos sin cuadrar, "Tune: Hood" solo mueve el gap hasta aquí (la prueba de 2.5°). */
    public static final double UNCHECKED_GAP_LIMIT_DEG = 5.0;

    private final Servo gapServo;
    private final Servo angleServo;
    private final StateStore store;

    private double gapCmdDeg, deltaCmdDeg;
    private double gapTargetDeg = HoodCfg.GAP_POLLEN_DEG;
    private double deltaTargetDeg = HoodCfg.DELTA_HOME_DEG;
    private double deltaMinDeg = HoodCfg.DELTA_USE_MIN_DEG;
    private double deltaMaxDeg = HoodCfg.DELTA_USE_MAX_DEG;

    private boolean started = false;
    private boolean nectarRefused = false;      // pidieron NECTAR con los signos sin cuadrar
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

    /**
     * Vuelve a poner los rangos PWM (después de cambiar Config desde "Tune: Hood"). El SDK no
     * reenvía el pulso si la posición 0..1 no cambió: se le hace olvidar la última posición.
     */
    public void reapplyPwmRanges() {
        if (gapServo instanceof PwmControl) {
            ((PwmControl) gapServo).setPwmRange(new PwmControl.PwmRange(gapUsLow(), gapUsHigh()));
        }
        if (angleServo instanceof PwmControl) {
            ((PwmControl) angleServo).setPwmRange(new PwmControl.PwmRange(angleUsLow(), angleUsHigh()));
        }
        forgetLastPosition(gapServo);
        forgetLastPosition(angleServo);
        write();
    }

    private static void forgetLastPosition(Servo servo) {
        if (servo.getController() instanceof LynxServoController) {
            ((LynxServoController) servo.getController()).forgetLastKnownPosition(servo.getPortNumber());
        }
    }

    // ---------- la ley ----------

    /** Comando del servo de ángulo, en grados relativos al eslabón. */
    public static double angleServoDeg(double gapDeg, double deltaDeg) {
        return -gapDeg + deltaDeg;
    }

    /** Recorrido del servo de ángulo: la ley con la envolvente segura en los dos topes del gap. */
    public static double angleMinDeg() {
        return Math.min(angleServoDeg(HoodCfg.GAP_POLLEN_DEG, HoodCfg.DELTA_SAFE_MIN_DEG),
                angleServoDeg(HoodCfg.GAP_NECTAR_DEG, HoodCfg.DELTA_SAFE_MIN_DEG));
    }

    public static double angleMaxDeg() {
        return Math.max(angleServoDeg(HoodCfg.GAP_POLLEN_DEG, HoodCfg.DELTA_SAFE_MAX_DEG),
                angleServoDeg(HoodCfg.GAP_NECTAR_DEG, HoodCfg.DELTA_SAFE_MAX_DEG));
    }

    /**
     * Los signos de los dos servos cuadran con cómo están montados (documento del CAD v138,
     * 4.1): con SERVOS_FACE_OPPOSITE, los µs por grado del gap y del ángulo tienen signo
     * contrario. Si no, ir a NECTAR giraría la lámina contra la Limelight.
     */
    public static boolean signsConsistent() {
        double gapUsPerDeg = (HoodCfg.GAP_NECTAR_US - HoodCfg.GAP_POLLEN_US)
                / (HoodCfg.GAP_NECTAR_DEG - HoodCfg.GAP_POLLEN_DEG);
        if (gapUsPerDeg == 0 || Double.isNaN(gapUsPerDeg) || HoodCfg.ANGLE_US_PER_DEG == 0) return false;
        double expected = HoodCfg.SERVOS_FACE_OPPOSITE ? -Math.signum(gapUsPerDeg) : Math.signum(gapUsPerDeg);
        return Math.signum(HoodCfg.ANGLE_US_PER_DEG) == expected;
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

    /**
     * Gap para la pelota que va a salir. NONE no cambia nada. Con los signos sin cuadrar, una
     * que necesita el gap de NECTAR deja el hood en POLLEN y isReady() da false.
     */
    public void setGapFor(BallType ball) {
        if (ball == null || ball == BallType.NONE) return;
        boolean nectar = ball.needsNectarGap();
        nectarRefused = nectar && !signsConsistent();
        gapTargetDeg = nectar && !nectarRefused ? HoodCfg.GAP_NECTAR_DEG : HoodCfg.GAP_POLLEN_DEG;
    }

    /**
     * Inclinación del hood δ. Se recorta al rango de uso (o al de calibración). Cambios más
     * chicos que DELTA_DEADBAND_DEG se ignoran: la distancia cambia un poco en cada foto y,
     * sin esto, el hood nunca terminaría de asentarse.
     */
    public void setDelta(double deltaDeg) {
        double d = Range.clip(deltaDeg, deltaMinDeg, deltaMaxDeg);
        if (Math.abs(d - deltaTargetDeg) < HoodCfg.DELTA_DEADBAND_DEG) return;
        deltaTargetDeg = d;
    }

    /**
     * Solo para "Tune: Hood": un gap intermedio (recortado al recorrido del eslabón), para
     * probar el sentido con un movimiento corto antes de ir hasta NECTAR. Con los signos sin
     * cuadrar, no más de UNCHECKED_GAP_LIMIT_DEG desde POLLEN.
     */
    public void setGapDegForTuning(double gapDeg) {
        double g = clampGap(gapDeg);
        if (!signsConsistent()) {
            g = Range.clip(g, HoodCfg.GAP_POLLEN_DEG - UNCHECKED_GAP_LIMIT_DEG,
                    HoodCfg.GAP_POLLEN_DEG + UNCHECKED_GAP_LIMIT_DEG);
        }
        nectarRefused = false;
        gapTargetDeg = g;
    }

    /**
     * Solo para "Tune: Hood" (paso 6): cambió HoodCfg.GAP_NECTAR_DEG. El servo de gap no se
     * mueve (en NECTAR sigue en GAP_NECTAR_US); solo cambia cuánto compensa el de ángulo.
     */
    public void nectarGapChanged(double oldNectarDeg) {
        if (gapCmdDeg == oldNectarDeg) gapCmdDeg = HoodCfg.GAP_NECTAR_DEG;
        if (gapTargetDeg == oldNectarDeg) gapTargetDeg = HoodCfg.GAP_NECTAR_DEG;
        reapplyPwmRanges();
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

    /**
     * true cuando el gap y δ ya llegaron a lo pedido y pasó el tiempo de asentarse. false si
     * la pelota pedida necesita el gap de NECTAR y el seguro de los signos lo impide.
     */
    public boolean isReady(long nowNs) {
        return started && !nectarRefused && gapCmdDeg == gapTargetDeg && deltaCmdDeg == deltaTargetDeg
                && nowNs >= settleUntilNs;
    }

    public boolean isNectarGap() {
        return gapTargetDeg == HoodCfg.GAP_NECTAR_DEG;
    }

    /** Pidieron el gap de NECTAR y el seguro de los signos no lo deja (se quedó en POLLEN). */
    public boolean isNectarRefused() { return nectarRefused; }

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
        return Math.min(angleToUs(angleMinDeg()), angleToUs(angleMaxDeg()));
    }

    private static double angleUsHigh() {
        return Math.max(angleToUs(angleMinDeg()), angleToUs(angleMaxDeg()));
    }

    public static double gapToPosition(double gapDeg) {
        double us = gapToUs(clampGap(gapDeg));
        return Range.clip((us - gapUsLow()) / (gapUsHigh() - gapUsLow()), 0, 1);
    }

    public static double angleToPosition(double angleDeg) {
        double a = Range.clip(angleDeg, angleMinDeg(), angleMaxDeg());
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
        if (!signsConsistent()) {
            telemetry.addLine("!! Hood: el signo del servo de ángulo no cuadra con el del gap "
                    + "(HoodCfg): no va a NECTAR. Revisa con \"Tune: Hood\"");
        }
    }
}
