package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.FeederCfg;
import org.firstinspires.ftc.teamcode.Config.Names;

import java.util.List;

/**
 * Calibra la copa (cremallera + servo 2000-0025-0004). Solo necesita "copa".
 *
 * Guía del CAD (diseno_copa_cremallera.md): 0.209 mm por grado de servo, carrera de uso
 * 54 mm (≈258° de servo), tope físico en 57 mm. Subir = BAJAR el PWM.
 *
 * PROCEDIMIENTO
 *  1. Arranca en COPA_DOWN_US. Con el bumper derecho (+1 µs, baja) ve hasta que la
 *     corriente de la Control Hub suba (el aro empuja el piso) y con el izquierdo (-1 µs,
 *     sube) regresa al último valor con corriente de vacío: ese es el piso. Márcalo con X
 *     y cópialo a COPA_DOWN_US.
 *  2. Sube con dpad arriba (10 µs) hasta que el vernier marque 54 mm de carrera: ese es
 *     COPA_UP_US.
 *  3. Con los dos valores en Config, A sube y B baja con la rampa del robot (≤ 50 mm/s en
 *     los últimos 5 mm). Revisa con y sin pelota.
 *
 * El recorrido se limita a 54 mm de cálculo (1722.5 µs con 0.209 mm/° y 0.15°/µs) sobre
 * el piso marcado, para no llegar al tope de 57 mm. El piso solo se puede marcar a ±60 µs
 * de COPA_DOWN_US: marcarlo arriba del piso real subiría el límite. Si en el límite el
 * vernier marca menos de 54 mm, la relación real no es la de cálculo: anota cuánto marca
 * y avísale al CAD antes de seguir.
 */
@TeleOp(name = "Tune: Copa", group = "2 Calibracion")
public class TuneCopa extends LinearOpMode {
    /** Guía del CAD: 0.209 mm por grado de servo y 0.15° por µs. */
    private static final double CALC_MM_PER_US = 0.209 * 0.15;
    private static final double MARK_WINDOW_US = 60;

    @Override
    public void runOpMode() {
        Servo copa = hardwareMap.get(Servo.class, Names.COPA_SERVO);
        PwmControl pwm = copa instanceof PwmControl ? (PwmControl) copa : null;
        if (pwm != null) pwm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        double us = FeederCfg.COPA_DOWN_US;
        double floorUs = FeederCfg.COPA_DOWN_US;
        boolean floorMarked = false;
        double rampTargetUs = Double.NaN;
        long lastNs = System.nanoTime();

        telemetry.addLine("Al dar Play la copa va a COPA_DOWN_US (" + us + " µs).");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = Math.min(0.1, (now - lastNs) / 1e9);
            lastNs = now;

            if (gamepad1.dpadUpWasPressed()) { us -= 10; rampTargetUs = Double.NaN; }
            if (gamepad1.dpadDownWasPressed()) { us += 10; rampTargetUs = Double.NaN; }
            if (gamepad1.leftBumperWasPressed()) { us -= 1; rampTargetUs = Double.NaN; }
            if (gamepad1.rightBumperWasPressed()) { us += 1; rampTargetUs = Double.NaN; }
            if (gamepad1.aWasPressed()) rampTargetUs = FeederCfg.COPA_UP_US;
            if (gamepad1.bWasPressed()) rampTargetUs = floorUs;
            if (gamepad1.xWasPressed() && Math.abs(us - FeederCfg.COPA_DOWN_US) <= MARK_WINDOW_US) {
                floorUs = us;
                floorMarked = true;
            }

            // Rampa como la del robot: rápida y, en los últimos 5 mm de subida, lenta
            if (!Double.isNaN(rampTargetUs)) {
                double usPerMm = (FeederCfg.COPA_DOWN_US - FeederCfg.COPA_UP_US) / FeederCfg.COPA_TRAVEL_MM;
                double mm = (FeederCfg.COPA_DOWN_US - us) / usPerMm;
                boolean slow = rampTargetUs < us && mm >= FeederCfg.COPA_TRAVEL_MM - FeederCfg.COPA_SLOW_ZONE_MM;
                double rate = (slow ? FeederCfg.COPA_SLOW_MM_S : FeederCfg.COPA_FAST_MM_S) * usPerMm * dt;
                us += Range.clip(rampTargetUs - us, -rate, rate);
            }

            double topLimitUs = floorUs - FeederCfg.COPA_TRAVEL_MM / CALC_MM_PER_US;
            us = Range.clip(us, topLimitUs, FeederCfg.COPA_DOWN_US + MARK_WINDOW_US);
            copa.setPosition((us - 500) / 2000.0);

            double usPerMm = (FeederCfg.COPA_DOWN_US - FeederCfg.COPA_UP_US) / FeederCfg.COPA_TRAVEL_MM;
            telemetry.addData("Pulso", "%.0f µs%s", us, us <= topLimitUs ? "  (LÍMITE de 54 mm)" : "");
            telemetry.addData("Piso", "%.0f µs  %s", floorUs, floorMarked ? "(marcado)" : "(COPA_DOWN_US; X lo marca)");
            telemetry.addData("Carrera según Config", "%.1f mm", (FeederCfg.COPA_DOWN_US - us) / usPerMm);
            telemetry.addData("Carrera por cálculo", "%.1f mm desde el piso  (0.209 mm/° y 0.15°/µs)",
                    (floorUs - us) * CALC_MM_PER_US);
            for (LynxModule hub : hubs) {
                telemetry.addData("Corriente " + hub.getDeviceName(), "%.2f A", hub.getCurrent(CurrentUnit.AMPS));
            }
            telemetry.addLine();
            telemetry.addLine("dpad arriba/abajo = 10 µs (arriba = sube)   bumpers = 1 µs");
            telemetry.addLine("A = subir con rampa   B = bajar con rampa   X = el piso es aquí");
            telemetry.update();
        }
    }
}
