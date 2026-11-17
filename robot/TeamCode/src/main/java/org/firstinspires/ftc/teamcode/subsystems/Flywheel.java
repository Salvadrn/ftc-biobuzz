package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.FlywheelCfg;
import org.firstinspires.ftc.teamcode.Config.Names;

/**
 * Flywheel del CAD v99: rueda de acero de 82 mm más 2 FlatTop de 96 mm en flecha REX,
 * movida directo (Hyper Coupler) por un Yellow Jacket 5203-2402-0001 (1:1, ~6000 rpm).
 * Velocidad con el PIDF del hub y el encoder del motor (28 cuentas por vuelta).
 *
 * Apagar: el SDK, en RUN_USING_ENCODER, convierte setPower(0) en "velocidad objetivo 0"
 * y el hub frena la rueda activamente (auditoría del 27-oct, por bytecode). Aquí, al
 * apagar se pasa a RUN_WITHOUT_ENCODER con FLOAT para que gire libre hasta parar.
 */
public class Flywheel {
    private final DcMotorEx motor;

    private boolean enabled = false;
    private double targetRpm = 0;
    private double lastSentTicksPerS = Double.NaN;

    private double rpm = 0;            // leído una vez por ciclo
    private double peakRpmSinceReady = 0;
    private boolean dipLatched = false;

    public Flywheel(HardwareMap hardwareMap) {
        this(hardwareMap.get(DcMotorEx.class, Names.FLYWHEEL));
    }

    public Flywheel(DcMotorEx motor) {
        this.motor = motor;
        motor.setDirection(FlywheelCfg.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setVelocityPIDFCoefficients(FlywheelCfg.VEL_P, FlywheelCfg.VEL_I, FlywheelCfg.VEL_D, FlywheelCfg.VEL_F);
        coast();
    }

    public void setEnabled(boolean on) {
        if (on == enabled) return;
        enabled = on;
        if (on) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastSentTicksPerS = Double.NaN;
        } else {
            coast();
        }
    }

    public boolean isEnabled() { return enabled; }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0, Math.min(rpm, FlywheelCfg.MAX_RPM));
    }

    public double getTargetRpm() { return targetRpm; }

    /** Llamar una vez por ciclo: lee la velocidad UNA vez y manda la consigna si cambió. */
    public void update() {
        rpm = motor.getVelocity() / FlywheelCfg.TICKS_PER_REV * 60.0;
        if (enabled) {
            double ticksPerS = targetRpm / 60.0 * FlywheelCfg.TICKS_PER_REV;
            if (ticksPerS != lastSentTicksPerS) {   // setVelocity no tiene caché en el SDK
                motor.setVelocity(ticksPerS);
                lastSentTicksPerS = ticksPerS;
            }
            trackDip();
        }
    }

    private void coast() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        lastSentTicksPerS = Double.NaN;
        peakRpmSinceReady = 0;
        dipLatched = false;
    }

    /**
     * Cuando una pelota pasa, la rueda pierde vueltas de golpe. Eso confirma el tiro.
     * Se marca si, ya en velocidad, cae más de SHOT_DIP_RPM.
     */
    private void trackDip() {
        if (isAtSpeed()) {
            peakRpmSinceReady = Math.max(peakRpmSinceReady, rpm);
        }
        if (peakRpmSinceReady > 0 && rpm < peakRpmSinceReady - FlywheelCfg.SHOT_DIP_RPM) {
            dipLatched = true;
            peakRpmSinceReady = 0;
        }
    }

    /** true una sola vez por cada caída de rpm detectada desde la última consulta. */
    public boolean consumeShotDip() {
        boolean d = dipLatched;
        dipLatched = false;
        return d;
    }

    public double getRpm() { return rpm; }

    public boolean isAtSpeed() {
        return enabled && targetRpm > 0 && Math.abs(rpm - targetRpm) < FlywheelCfg.RPM_TOLERANCE;
    }

    public void stop() {
        setEnabled(false);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flywheel", "%s  %.0f / %.0f rpm", enabled ? "ON" : "off", rpm, targetRpm);
    }
}
