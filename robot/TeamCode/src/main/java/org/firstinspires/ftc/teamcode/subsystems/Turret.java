package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.TurretCfg;
import org.firstinspires.ftc.teamcode.util.AngleHistory;
import org.firstinspires.ftc.teamcode.vision.CellTarget;

/**
 * Torreta del CAD v99: corona de 350T movida por un piñón de 45T (7.78:1) con un servo
 * goBILDA 2000-0025-0504 de 5 vueltas en modo posición. El Limelight va encima.
 *
 * Convención: 0 = frente del robot, positivo = izquierda (antihorario visto desde arriba),
 * igual que el yaw del IMU y que la junta "Giro torreta" del CAD.
 *
 * Rango: con el PWM por defecto del SDK (600 a 2400 µs) el servo solo lleva la torreta a
 * ±104°. Aquí se usa 500 a 2500 µs (±115.7°) y el tope por software es ±105°: la guía de
 * cables se diseñó para ±110° y más allá se jalan los cables.
 *
 * Cómo apunta:
 *  1. El servo no dice dónde está. Se estima suponiendo que avanza hacia lo ordenado
 *     a MAX_SPEED_DEG_S.
 *  2. Cada foto nueva se convierte en "rumbo del objetivo en el campo": el rumbo que
 *     tenía la torreta cuando se tomó la foto, más el rumbo medido. Usar el rumbo del
 *     instante de la foto y no el actual es lo que evita que oscile por el retraso.
 *  3. Ese rumbo se suaviza y en cada ciclo se ordena: rumbo del objetivo menos el yaw
 *     actual del chasis. Si el piloto gira el robot, la torreta contragira sola.
 */
public class Turret {
    private final Servo servo;
    private final AngleHistory headingHistory = new AngleHistory(0.6);

    private double commandedDeg = 0;
    private double estimatedDeg = 0;
    private boolean positionKnown = false;
    private long settleUntilNs = 0;

    private boolean hasTarget = false;
    private double targetFieldDeg = 0;
    private long lastSeenNs = 0;
    private double lastFrameBearingDeg = Double.NaN;
    private long lastFrameCaptureNs = Long.MIN_VALUE;
    private long settledSinceNs = -1;   // desde cuándo el estimado llegó a lo ordenado

    private boolean atLimit = false;
    private long lastUpdateNs = 0;
    private double lastYawDeg = 0;

    public Turret(HardwareMap hardwareMap) {
        this(hardwareMap.get(Servo.class, Names.TURRET_SERVO));
    }

    /** También para pruebas en la computadora con un servo simulado. */
    public Turret(Servo servo) {
        this.servo = servo;
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(
                    new PwmControl.PwmRange(TurretCfg.PWM_MIN_US, TurretCfg.PWM_MAX_US));
        }
    }

    /**
     * Manda la torreta al frente y no confía en su posición hasta que alcance a llegar
     * desde el peor lugar posible. Se llama en INIT (R103.B permite prepararla ahí).
     */
    public void startCentering(long nowNs) {
        commandedDeg = 0;
        estimatedDeg = 0;
        positionKnown = false;
        hasTarget = false;
        double worstTravel = Math.max(Math.abs(TurretCfg.MIN_ANGLE_DEG), Math.abs(TurretCfg.MAX_ANGLE_DEG));
        settleUntilNs = nowNs + (long) ((worstTravel / TurretCfg.MAX_SPEED_DEG_S + 0.2) * 1e9);
        servo.setPosition(angleToServo(0));
    }

    /**
     * Llamar una vez por ciclo.
     *
     * @param nowNs       System.nanoTime() de este ciclo
     * @param yawDeg      rumbo continuo del chasis en grados, o 0 si no hay IMU
     * @param newFrame    objetivo de una foto NUEVA, o null si este ciclo no llegó ninguna
     * @param autoAim     true = seguir el objetivo solo
     * @param manualInput -1 a 1, positivo = izquierda. Se usa cuando no hay objetivo que seguir
     */
    public void update(long nowNs, double yawDeg, CellTarget newFrame, boolean autoAim, double manualInput) {
        double dt = lastUpdateNs == 0 ? 0 : Range.clip((nowNs - lastUpdateNs) / 1e9, 0, 0.1);
        lastUpdateNs = nowNs;
        lastYawDeg = yawDeg;

        // 1. Dónde creemos que está la torreta de verdad
        if (!positionKnown && nowNs >= settleUntilNs) {
            positionKnown = true;
            estimatedDeg = commandedDeg;
        }
        if (positionKnown) {
            double maxStep = TurretCfg.MAX_SPEED_DEG_S * dt;
            estimatedDeg += Range.clip(commandedDeg - estimatedDeg, -maxStep, maxStep);
            headingHistory.add(nowNs, yawDeg + estimatedDeg);
            if (Math.abs(commandedDeg - estimatedDeg) < TurretCfg.LOCK_TOLERANCE_DEG) {
                if (settledSinceNs < 0) settledSinceNs = nowNs;
            } else {
                settledSinceNs = -1;
            }
        }

        // 2. Foto nueva: actualizar el rumbo del objetivo en el campo.
        //    Mientras no se sabe dónde está la torreta, las fotos no sirven.
        if (newFrame != null && positionKnown) {
            Double headingAtCapture = headingHistory.at(newFrame.captureTimeNs);
            if (headingAtCapture == null) headingAtCapture = yawDeg + estimatedDeg;
            double measuredFieldDeg = headingAtCapture + newFrame.bearingDeg;

            if (hasTarget) {
                targetFieldDeg += TurretCfg.AIM_SMOOTHING * wrap180(measuredFieldDeg - targetFieldDeg);
            } else {
                targetFieldDeg = measuredFieldDeg;
                hasTarget = true;
            }
            lastSeenNs = nowNs;
            lastFrameBearingDeg = newFrame.bearingDeg;
            lastFrameCaptureNs = newFrame.captureTimeNs;
        }
        if (hasTarget && (nowNs - lastSeenNs) / 1e9 > TurretCfg.TARGET_MEMORY_S) {
            hasTarget = false;
        }

        // 3. Decidir el ángulo a ordenar
        boolean tracking = autoAim && hasTarget && positionKnown;
        double desiredDeg;
        if (tracking) {
            desiredDeg = wrap180(targetFieldDeg - yawDeg);
        } else {
            desiredDeg = commandedDeg + manualInput * TurretCfg.MANUAL_SPEED_DEG_S * dt;
        }

        double limitedDeg = Range.clip(desiredDeg, minAngleDeg(), maxAngleDeg());
        atLimit = tracking && limitedDeg != desiredDeg;

        boolean bigEnough = Math.abs(limitedDeg - commandedDeg) > TurretCfg.DEADBAND_DEG;
        if (!tracking || bigEnough) {
            commandedDeg = limitedDeg;
        }
        servo.setPosition(angleToServo(commandedDeg));
    }

    /** Manda la torreta al frente durante el partido (botón de pánico). */
    public void center() {
        commandedDeg = 0;
        hasTarget = false;
        servo.setPosition(angleToServo(0));
    }

    /** Error actual entre hacia dónde apunta la torreta (estimado) y el objetivo, en grados. */
    public double aimErrorDeg() {
        if (!hasTarget) return Double.NaN;
        return wrap180(targetFieldDeg - lastYawDeg - estimatedDeg);
    }

    /**
     * true cuando hay objetivo reciente, la torreta (estimada) ya llegó y apunta a él, y
     * además la última foto lo confirma. El servo no dice dónde está: si es más lento que
     * MAX_SPEED_DEG_S, el estimado se adelanta y solo la foto lo delata.
     */
    public boolean isLocked(long nowNs) {
        if (!positionKnown || !hasTarget || atLimit) return false;
        boolean seenRecently = (nowNs - lastSeenNs) / 1e9 < TurretCfg.LOCK_FRAME_MAX_AGE_S;
        boolean settled = Math.abs(commandedDeg - estimatedDeg) < TurretCfg.LOCK_TOLERANCE_DEG;
        // La foto tiene que ser de DESPUÉS de que la torreta (según el modelo) llegó, con
        // margen por si el servo real es más lento, y tiene que decir que el error es chico.
        boolean photoAfterSettle = settledSinceNs >= 0
                && lastFrameCaptureNs >= settledSinceNs + (long) (TurretCfg.LOCK_PHOTO_MARGIN_S * 1e9);
        boolean photoAgrees = photoAfterSettle && Math.abs(lastFrameBearingDeg) < TurretCfg.LOCK_TOLERANCE_DEG;
        return seenRecently && settled && photoAgrees && Math.abs(aimErrorDeg()) < TurretCfg.LOCK_TOLERANCE_DEG;
    }

    /** true si el objetivo quedó fuera del giro de la torreta: hay que rotar el chasis. */
    public boolean isAtLimit() { return atLimit; }

    public boolean hasTarget() { return hasTarget; }
    public boolean isPositionKnown() { return positionKnown; }
    public double getCommandedDeg() { return commandedDeg; }
    public double getEstimatedDeg() { return estimatedDeg; }

    // ---------- conversión ángulo <-> servo ----------

    private static double direction() {
        return TurretCfg.SERVO_INVERTED ? -1.0 : 1.0;
    }

    /** Posición del servo (0 a 1, dentro del rango PWM de Config) para un ángulo de torreta. */
    public static double angleToServo(double angleDeg) {
        double pos = TurretCfg.SERVO_CENTER + direction() * angleDeg / TurretCfg.DEG_PER_SERVO_UNIT;
        return Range.clip(pos, 0.0, 1.0);
    }

    public static double servoToAngle(double position) {
        return direction() * (position - TurretCfg.SERVO_CENTER) * TurretCfg.DEG_PER_SERVO_UNIT;
    }

    /** El límite real es el más estricto entre el de Config y el recorrido del servo. */
    public static double minAngleDeg() {
        return Math.max(TurretCfg.MIN_ANGLE_DEG, Math.min(servoToAngle(0), servoToAngle(1)));
    }

    public static double maxAngleDeg() {
        return Math.min(TurretCfg.MAX_ANGLE_DEG, Math.max(servoToAngle(0), servoToAngle(1)));
    }

    public static double wrap180(double deg) {
        deg = deg % 360.0;
        if (deg > 180) deg -= 360;
        if (deg <= -180) deg += 360;
        return deg;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (!positionKnown) {
            telemetry.addLine("Torreta: centrando, todavía no se confía en su posición");
            return;
        }
        telemetry.addData("Torreta", "orden %.1f  estimado %.1f  error %.1f grados",
                commandedDeg, estimatedDeg, hasTarget ? aimErrorDeg() : Double.NaN);
        if (atLimit) telemetry.addLine("!! TORRETA AL LIMITE: girar el chasis");
    }
}
