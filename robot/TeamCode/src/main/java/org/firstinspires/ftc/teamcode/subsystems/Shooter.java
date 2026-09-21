package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.ShooterCfg;
import org.firstinspires.ftc.teamcode.util.InterpTable;

/**
 * Flywheel de velocidad variable y hood de ángulo variable.
 * Los dos se ajustan solos con la distancia que da el Limelight, usando SHOT_TABLE.
 *
 * El flywheel usa el control de velocidad del hub (RUN_USING_ENCODER + setVelocity),
 * así que cada motor del flywheel necesita su cable de encoder conectado.
 */
public class Shooter {
    private final DcMotorEx flywheel;
    private final DcMotorEx flywheel2;   // null si el robot solo tiene uno
    private final Servo hood;

    private final InterpTable rpmTable;
    private final InterpTable hoodTable;

    private boolean enabled = false;
    private double targetRpm = 0;
    private double hoodPosition;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, Names.FLYWHEEL);
        flywheel2 = hardwareMap.tryGet(DcMotorEx.class, Names.FLYWHEEL_2);
        hood = hardwareMap.get(Servo.class, Names.HOOD_SERVO);

        setupMotor(flywheel, ShooterCfg.FLYWHEEL_REVERSED);
        if (flywheel2 != null) setupMotor(flywheel2, ShooterCfg.FLYWHEEL_2_REVERSED);

        double[][] table = ShooterCfg.SHOT_TABLE;
        double[][] rpmPoints = new double[table.length][];
        double[][] hoodPoints = new double[table.length][];
        for (int i = 0; i < table.length; i++) {
            rpmPoints[i] = new double[]{table[i][0], table[i][1]};
            hoodPoints[i] = new double[]{table[i][0], table[i][2]};
        }
        rpmTable = new InterpTable(rpmPoints);
        hoodTable = new InterpTable(hoodPoints);
        hoodPosition = hoodTable.get(rpmTable.minX());
    }

    private static void setupMotor(DcMotorEx motor, boolean reversed) {
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // que frene solo, sin forzar
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(ShooterCfg.VEL_P, ShooterCfg.VEL_I, ShooterCfg.VEL_D, ShooterCfg.VEL_F);
    }

    public void setEnabled(boolean enabled) { this.enabled = enabled; }
    public boolean isEnabled() { return enabled; }

    /**
     * Llamar una vez por ciclo.
     * @param distanceIn distancia al CELL en pulgadas, o NaN si ahorita no se conoce.
     *                   Sin distancia se conserva el último ajuste.
     */
    public void update(double distanceIn) {
        if (!Double.isNaN(distanceIn)) {
            targetRpm = rpmTable.get(distanceIn);
            hoodPosition = hoodTable.get(distanceIn);
        } else if (targetRpm == 0) {
            targetRpm = ShooterCfg.IDLE_RPM;
        }
        apply();
    }

    /** Para "Tune: Tiro": fija RPM y hood a mano, ignorando la tabla. */
    public void setManual(double rpm, double hoodPos) {
        targetRpm = Math.max(0, rpm);
        hoodPosition = hoodPos;
        apply();
    }

    private void apply() {
        hoodPosition = Range.clip(hoodPosition, ShooterCfg.HOOD_MIN, ShooterCfg.HOOD_MAX);
        hood.setPosition(hoodPosition);

        if (enabled) {
            double ticksPerSecond = targetRpm / 60.0 * ShooterCfg.TICKS_PER_FLYWHEEL_REV;
            flywheel.setVelocity(ticksPerSecond);
            if (flywheel2 != null) flywheel2.setVelocity(ticksPerSecond);
        } else {
            flywheel.setPower(0);
            if (flywheel2 != null) flywheel2.setPower(0);
        }
    }

    public double getRpm() {
        return flywheel.getVelocity() / ShooterCfg.TICKS_PER_FLYWHEEL_REV * 60.0;
    }

    public double getTargetRpm() { return targetRpm; }
    public double getHoodPosition() { return hoodPosition; }

    /** true cuando el flywheel está prendido y ya alcanzó las RPM pedidas. */
    public boolean isReady() {
        return enabled && targetRpm > 0 && Math.abs(getRpm() - targetRpm) < ShooterCfg.RPM_TOLERANCE;
    }

    public void stop() {
        enabled = false;
        apply();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flywheel", "%s  %.0f / %.0f rpm", enabled ? "ON" : "off", getRpm(), targetRpm);
        telemetry.addData("Hood", "%.3f", hoodPosition);
    }
}
