package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Config.DriveCfg;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.CellTarget;

/**
 * TeleOp base. La alianza decide qué tags se siguen, por eso hay una versión roja y una azul.
 *
 * CONTROLES
 *   Gamepad 1, chasis:
 *     stick izquierdo      avanzar y moverse de lado
 *     stick derecho X      girar
 *     gatillo derecho      modo lento
 *   Gamepad 1 o 2, tiro:
 *     A                    auto-apuntado sí/no   (arranca en sí)
 *     bumper derecho       flywheel sí/no
 *     Y                    torreta al frente
 *     dpad izq/der         mover torreta a mano  (gamepad 2 también con stick derecho X)
 *   El control vibra cuando la torreta está fija en el CELL y el flywheel ya tiene sus RPM.
 */
public abstract class BiobuzzTeleOp extends LinearOpMode {

    protected abstract Alliance alliance();

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, alliance());

        telemetry.addData("Alianza", alliance());
        telemetry.addData("IMU", robot.hasImu() ? "ok" : "NO ENCONTRADO: la torreta no compensa giros");
        telemetry.addLine("Listo. La torreta se centra al dar Play.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.turret.center();
        boolean autoAim = true;
        boolean wasReadyToFire = false;
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            // ----- chasis -----
            double speed = gamepad1.right_trigger > 0.3 ? DriveCfg.SLOW_FACTOR : 1.0;
            robot.drive.drive(
                    -gamepad1.left_stick_y * speed,
                    gamepad1.left_stick_x * speed,
                    gamepad1.right_stick_x * speed);

            // ----- botones de tiro -----
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) autoAim = !autoAim;
            if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
                robot.shooter.setEnabled(!robot.shooter.isEnabled());
            }
            if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                autoAim = false;
                robot.turret.center();
            }

            double manualTurret = -gamepad2.right_stick_x;   // stick a la izquierda = torreta a la izquierda
            if (gamepad1.dpad_left || gamepad2.dpad_left) manualTurret = 1;
            if (gamepad1.dpad_right || gamepad2.dpad_right) manualTurret = -1;

            // ----- visión, torreta y tiro -----
            double yaw = robot.updateYaw();
            CellTarget frame = robot.vision.update();
            robot.turret.update(yaw, frame, autoAim, manualTurret);

            CellTarget last = robot.vision.getLastTarget();
            boolean distanceIsFresh = last != null && robot.vision.secondsSinceTarget() < 0.5;
            robot.shooter.update(distanceIsFresh ? last.distanceIn : Double.NaN);

            boolean readyToFire = robot.turret.isLocked() && robot.shooter.isReady();
            if (readyToFire && !wasReadyToFire) {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            }
            wasReadyToFire = readyToFire;

            // ----- telemetría -----
            telemetry.addData("Alianza", "%s   apuntado %s", alliance(), autoAim ? "AUTO" : "manual");
            telemetry.addData("TIRO", readyToFire ? ">>> LISTO <<<" : "esperando");
            robot.vision.addTelemetry(telemetry);
            robot.turret.addTelemetry(telemetry);
            robot.shooter.addTelemetry(telemetry);
            telemetry.addData("Ciclo", "%.0f ms", loopTimer.milliseconds());
            loopTimer.reset();
            telemetry.update();
        }

        robot.stop();
    }
}
