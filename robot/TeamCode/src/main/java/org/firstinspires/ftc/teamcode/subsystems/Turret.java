package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.TurretCfg;
import org.firstinspires.ftc.teamcode.util.AngleHistory;
import org.firstinspires.ftc.teamcode.vision.CellTarget;

/**
 * Torreta movida por un servo posicional, con el Limelight montado encima.
 *
 * Convención de ángulos: 0 = frente del robot, positivo = izquierda (antihorario visto
 * desde arriba), igual que el yaw del IMU. El tx del Limelight va al revés (positivo =
 * derecha), por eso se resta.
 *
 * Cómo apunta:
 *  1. Un servo posicional no dice dónde está. Se estima suponiendo que se mueve hacia
 *     lo ordenado a MAX_SPEED_DEG_S.
 *  2. Cada foto nueva se convierte en "rumbo del objetivo en el campo": el rumbo que
 *     tenía la torreta cuando se tomó la foto, menos tx. Usar el rumbo del momento de la
 *     foto y no el actual es lo que evita que oscile por el retraso de la cámara.
 *  3. Ese rumbo se suaviza y en cada ciclo se ordena: rumbo del objetivo menos el yaw
 *     actual del chasis. Así, si el driver gira el robot, la torreta contragira sola
 *     aunque todavía no llegue otra foto.
 *
 * Sin IMU el yaw vale siempre 0 y todo funciona igual, solo que sin la compensación.
 */
public class Turret {
    private final Servo servo;
    private final AngleHistory headingHistory = new AngleHistory(0.6);

    private double commandedDeg = 0;
    private double estimatedDeg = 0;

    private boolean hasTarget = false;
    private double targetFieldDeg = 0;
    private double lastTxDeg = 0;
    private long lastSeenNs = 0;

    private boolean atLimit = false;
    private long lastUpdateNs = 0;

    public Turret(HardwareMap hardwareMap) {
        this(hardwareMap.get(Servo.class, Names.TURRET_SERVO));
    }

    /** Para pruebas en la computadora con un servo simulado. */
    public Turret(Servo servo) {
        this.servo = servo;
    }

    /**
     * Llamar una vez por ciclo.
     *
     * @param yawDeg      rumbo continuo del chasis en grados, o 0 si no hay IMU
     * @param newFrame    objetivo de una foto NUEVA, o null si este ciclo no llegó ninguna
     * @param autoAim     true = seguir el objetivo solo
     * @param manualInput -1 a 1, positivo = izquierda. Se usa cuando no hay objetivo que seguir
     */
    public void update(double yawDeg, CellTarget newFrame, boolean autoAim, double manualInput) {
        long nowNs = System.nanoTime();
        double dt = lastUpdateNs == 0 ? 0 : Range.clip((nowNs - lastUpdateNs) / 1e9, 0, 0.1);
        lastUpdateNs = nowNs;

        // 1. Dónde creemos que está la torreta de verdad
        double maxStep = TurretCfg.MAX_SPEED_DEG_S * dt;
        estimatedDeg += Range.clip(commandedDeg - estimatedDeg, -maxStep, maxStep);
        headingHistory.add(nowNs, yawDeg + estimatedDeg);

        // 2. Foto nueva: actualizar el rumbo del objetivo en el campo
        if (newFrame != null) {
            Double headingAtCapture = headingHistory.at(newFrame.captureTimeNs);
            if (headingAtCapture == null) headingAtCapture = yawDeg + estimatedDeg;
            double measuredFieldDeg = headingAtCapture - newFrame.txDeg;

            if (hasTarget) {
                targetFieldDeg += TurretCfg.AIM_SMOOTHING * (measuredFieldDeg - targetFieldDeg);
            } else {
                targetFieldDeg = measuredFieldDeg;
                hasTarget = true;
            }
            lastTxDeg = newFrame.txDeg;
            lastSeenNs = nowNs;
        }
        if (hasTarget && (nowNs - lastSeenNs) / 1e9 > TurretCfg.TARGET_MEMORY_S) {
            hasTarget = false;
        }

        // 3. Decidir el ángulo a ordenar
        boolean tracking = autoAim && hasTarget;
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

    /** Manda la torreta al frente. Útil al arrancar y como botón de pánico. */
    public void center() {
        commandedDeg = 0;
        hasTarget = false;
        servo.setPosition(angleToServo(0));
    }

    /** true cuando hay objetivo reciente, el error es chico y el servo ya llegó. */
    public boolean isLocked() {
        boolean seenRecently = hasTarget && (System.nanoTime() - lastSeenNs) / 1e9 < 0.3;
        boolean settled = Math.abs(commandedDeg - estimatedDeg) < TurretCfg.LOCK_TOLERANCE_DEG;
        return seenRecently && settled && !atLimit
                && Math.abs(lastTxDeg) < TurretCfg.LOCK_TOLERANCE_DEG;
    }

    /** true si el objetivo quedó fuera del giro de la torreta: hay que rotar el chasis. */
    public boolean isAtLimit() { return atLimit; }

    public boolean hasTarget() { return hasTarget; }
    public double getCommandedDeg() { return commandedDeg; }
    public double getEstimatedDeg() { return estimatedDeg; }

    // ---------- conversión ángulo <-> servo ----------

    private static double direction() {
        return TurretCfg.SERVO_INVERTED ? -1.0 : 1.0;
    }

    public static double angleToServo(double angleDeg) {
        double pos = TurretCfg.SERVO_CENTER + direction() * angleDeg / TurretCfg.DEG_PER_SERVO_UNIT;
        return Range.clip(pos, 0.0, 1.0);
    }

    public static double servoToAngle(double position) {
        return direction() * (position - TurretCfg.SERVO_CENTER) * TurretCfg.DEG_PER_SERVO_UNIT;
    }

    /** El límite real es el más estricto entre el de Config y el recorrido físico del servo. */
    private static double minAngleDeg() {
        return Math.max(TurretCfg.MIN_ANGLE_DEG, Math.min(servoToAngle(0), servoToAngle(1)));
    }

    private static double maxAngleDeg() {
        return Math.min(TurretCfg.MAX_ANGLE_DEG, Math.max(servoToAngle(0), servoToAngle(1)));
    }

    private static double wrap180(double deg) {
        deg = deg % 360.0;
        if (deg > 180) deg -= 360;
        if (deg <= -180) deg += 360;
        return deg;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Torreta", "orden %.1f  estimado %.1f grados", commandedDeg, estimatedDeg);
        if (atLimit) telemetry.addLine("!! TORRETA AL LIMITE: girar el chasis");
    }
}
