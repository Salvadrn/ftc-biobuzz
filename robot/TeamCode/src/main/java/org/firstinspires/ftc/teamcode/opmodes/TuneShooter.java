package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Config.ShooterCfg;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.vision.CellTarget;
import org.firstinspires.ftc.teamcode.vision.CellVision;

/**
 * Para llenar Config.ShooterCfg.SHOT_TABLE. No usa el chasis: el robot se coloca a mano.
 *
 * PROCEDIMIENTO
 *  1. Poner el robot a una distancia conocida del CELL. Comparar la cinta métrica contra
 *     las dos distancias de la telemetría. Si "trig" sale mal, corregir CAMERA_HEIGHT_IN,
 *     CAMERA_PITCH_DEG y TAG_HEIGHT_IN en Config.
 *  2. Ajustar RPM y hood hasta que el POLLEN entre limpio varias veces seguidas.
 *  3. Anotar la fila {distancia, rpm, hood}. Usar la MISMA distancia que usará el robot
 *     en partido: la que marca USE_3D_DISTANCE.
 *  4. Repetir en cinco o seis distancias, de la más cercana a la más lejana.
 *
 * CONTROLES
 *   En INIT:  X = alianza azul    B = alianza roja
 *   dpad arriba/abajo = RPM ±50      dpad der/izq = hood ±0.01
 *   bumper derecho = flywheel sí/no  A = auto-apuntado sí/no
 */
@TeleOp(name = "Tune: Tiro", group = "2 Calibracion")
public class TuneShooter extends LinearOpMode {
    @Override
    public void runOpMode() {
        Alliance alliance = Alliance.RED;
        while (opModeInInit()) {
            if (gamepad1.x) alliance = Alliance.BLUE;
            if (gamepad1.b) alliance = Alliance.RED;
            telemetry.addData("Alianza", "%s   (X = azul, B = roja)", alliance);
            telemetry.update();
        }
        if (isStopRequested()) return;

        Turret turret = new Turret(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        CellVision vision = new CellVision(hardwareMap, alliance);

        double rpm = ShooterCfg.IDLE_RPM;
        double hood = (ShooterCfg.HOOD_MIN + ShooterCfg.HOOD_MAX) / 2;
        boolean autoAim = true;
        turret.center();

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) rpm += 50;
            if (gamepad1.dpadDownWasPressed()) rpm -= 50;
            if (gamepad1.dpadRightWasPressed()) hood += 0.01;
            if (gamepad1.dpadLeftWasPressed()) hood -= 0.01;
            if (gamepad1.rightBumperWasPressed()) shooter.setEnabled(!shooter.isEnabled());
            if (gamepad1.aWasPressed()) autoAim = !autoAim;

            rpm = Math.max(0, rpm);
            hood = Math.max(ShooterCfg.HOOD_MIN, Math.min(ShooterCfg.HOOD_MAX, hood));

            CellTarget frame = vision.update();
            turret.update(0, frame, autoAim, -gamepad1.right_stick_x);
            shooter.setManual(rpm, hood);

            telemetry.addData("Distancia trig", "%.1f in", vision.getTrigDistanceIn());
            telemetry.addData("Distancia 3D", "%.1f in", vision.get3dDistanceIn());
            telemetry.addLine();
            telemetry.addData("FILA", "{ %.0f, %.0f, %.2f },",
                    frame != null ? frame.distanceIn
                            : (vision.getLastTarget() != null ? vision.getLastTarget().distanceIn : Double.NaN),
                    rpm, hood);
            telemetry.addLine();
            vision.addTelemetry(telemetry);
            turret.addTelemetry(telemetry);
            shooter.addTelemetry(telemetry);
            telemetry.addData("Torreta fija", turret.isLocked() ? "SI" : "no");
            telemetry.update();
        }

        shooter.stop();
        vision.stop();
    }
}
