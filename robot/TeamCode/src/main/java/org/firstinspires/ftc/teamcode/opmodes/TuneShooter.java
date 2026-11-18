package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config.HoodCfg;
import org.firstinspires.ftc.teamcode.Config.ShotCfg;
import org.firstinspires.ftc.teamcode.Config.VisionCfg;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.vision.Cell;
import org.firstinspires.ftc.teamcode.vision.CellTarget;

/**
 * Para llenar Config.ShotCfg.POLLEN_TABLE y NECTAR_TABLE. Usa el robot completo con las
 * pelotas metidas a mano en la fila o en la copa. El chasis no se mueve: el robot se
 * coloca a mano.
 *
 * PROCEDIMIENTO
 *  1. Antes, "Tune: Torreta", "Tune: Hood", "Tune: Copa" y "Test: Sensores" tienen que
 *     estar hechos. Con cinta métrica, compara la distancia de la telemetría con la real
 *     (del eje de la torreta a la fila de tags). Si no cuadra, revisa CAMERA_HEIGHT_IN,
 *     CAMERA_PITCH_DEG y las alturas de las filas en Config.
 *  2. Revisa que "HIVE arriba" diga la CELL correcta. Si no, X la cambia y anota en qué
 *     distancia se equivocó. Las alturas medidas de las filas tienen que salir cerca de
 *     49.9 in (arriba) y 35.6 in (abajo). Si salen debajo de la cámara (13.4 in) o
 *     negativas, el signo de ty está al revés: VisionCfg.TY_SIGN = -1.
 *  3. Ajusta rpm y δ hasta que la pelota entre limpia varias veces seguidas. Anota la FILA
 *     en la tabla de esa pelota (el gap lo decide el V3 de arriba, no tú).
 *  4. Cinco o seis distancias por tipo de pelota, de la más cercana a la más lejana.
 *
 * CONTROLES
 *   En INIT: X = azul, B = roja, A = arrancar con esa alianza
 *   dpad arriba/abajo = rpm ±50     dpad der/izq = δ ±0.5°
 *   bumper derecho = flywheel sí/no    Y = auto-apuntado sí/no
 *   gatillo derecho = un tiro (cuando todo está listo)    X = cambiar CELL de arriba
 *   B = "el camino está libre" (quita los bloqueos del alimentador)
 *   El intake se queda quieto (las pelotas se meten a mano).
 */
@TeleOp(name = "Tune: Tiro", group = "2 Calibracion")
public class TuneShooter extends LinearOpMode {
    @Override
    public void runOpMode() {
        Alliance alliance = Alliance.RED;
        Robot robot = null;
        while (opModeInInit()) {
            long now = System.nanoTime();
            if (robot == null) {
                if (gamepad1.x) alliance = Alliance.BLUE;
                if (gamepad1.b) alliance = Alliance.RED;
                if (gamepad1.aWasPressed()) robot = new Robot(hardwareMap, alliance);
                telemetry.addData("Alianza", "%s   (X = azul, B = roja, A = arrancar)", alliance);
            } else {
                boolean ready = robot.initStep(now);
                telemetry.addData("Alianza", alliance);
                telemetry.addData("Inicio", ready ? "listo" : "preparando...");
            }
            telemetry.update();
        }
        if (robot == null || isStopRequested()) {
            if (robot != null) robot.stop();
            return;
        }

        try {
            double rpm = ShotCfg.IDLE_RPM;
            double delta = 0;
            boolean autoAim = true;
            boolean fireWasDown = false;

            while (opModeIsActive()) {
                long now = System.nanoTime();
                robot.beginLoop();
                if (gamepad1.dpadUpWasPressed()) rpm += 50;
                if (gamepad1.dpadDownWasPressed()) rpm -= 50;
                if (gamepad1.dpadRightWasPressed()) delta += 0.5;
                if (gamepad1.dpadLeftWasPressed()) delta -= 0.5;
                if (gamepad1.rightBumperWasPressed()) robot.flywheel.setEnabled(!robot.flywheel.isEnabled());
                if (gamepad1.yWasPressed()) autoAim = !autoAim;
                if (gamepad1.xWasPressed()) robot.vision.flipHive();
                if (gamepad1.bWasPressed()) robot.feeder.release();
                rpm = Math.max(0, rpm);
                delta = Math.max(HoodCfg.DELTA_USE_MIN_DEG, Math.min(HoodCfg.DELTA_USE_MAX_DEG, delta));

                CellTarget frame = robot.vision.update(now);
                robot.turret.update(now, robot.updateYaw(), frame, autoAim, -gamepad1.right_stick_x);

                BallType next = robot.feeder.nextBall();
                BallType forGap = next.isBall() ? next : BallType.POLLEN;
                robot.shooter.setManual(true, rpm, delta, forGap);
                robot.shooter.update(now, Double.NaN, BallType.NONE);

                boolean fireDown = gamepad1.right_trigger > 0.5;
                boolean ready = robot.flywheel.isAtSpeed() && robot.hood.isReady(now)
                        && robot.feeder.hasUpperBall() && !robot.feeder.isBusy()
                        && (!autoAim || robot.turret.isLocked(now));
                if (fireDown && !fireWasDown && ready) robot.feeder.requestShot();
                fireWasDown = fireDown;
                robot.feeder.update(now, false, false, false, robot.flywheel.consumeShotDip(),
                        robot.flywheel.isEnabled());

                // Intake quieto: si dieron Play a medio estacionado, esto apaga M_F
                robot.intake.setCommand(Intake.Command.STOP);
                robot.intake.update(now, robot.inventory.isFull());

                CellTarget last = robot.vision.getLastTarget();
                double age = robot.vision.secondsSinceTarget(now);
                double dist = last == null || age > 0.5 ? Double.NaN : last.distanceIn;
                String table = forGap.needsNectarGap() ? "NECTAR_TABLE" : "POLLEN_TABLE";

                telemetry.addData("Distancia (eje de la torreta)", "%.1f in", dist);
                telemetry.addData("Distancia 3D (cámara)", "%.1f in", robot.vision.get3dDistanceIn());
                Cell up = robot.vision.upCell();
                telemetry.addData("Altura de las filas", "%s %.1f in · %s %.1f in  (esperado %.1f / %.1f)",
                        up, robot.vision.getRowHeightIn(up), up.partner(), robot.vision.getRowHeightIn(up.partner()),
                        VisionCfg.TAG_ROW_HEIGHT_UP_IN, VisionCfg.TAG_ROW_HEIGHT_DOWN_IN);
                telemetry.addLine();
                if (Double.isNaN(dist)) {
                    telemetry.addData("FILA para " + table, "sin distancia reciente: no anotar");
                } else {
                    telemetry.addData("FILA para " + table, "{ %.0f, %.0f, %.1f },", dist, rpm, delta);
                }
                telemetry.addData("Listo", ready ? "SÍ (gatillo derecho)" : "no");
                telemetry.addLine();
                robot.vision.addTelemetry(telemetry, now);
                robot.turret.addTelemetry(telemetry);
                robot.shooter.addTelemetry(telemetry);
                robot.feeder.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            robot.stop();
        }
    }
}
