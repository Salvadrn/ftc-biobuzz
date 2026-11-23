package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Config.DriveCfg;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.vision.CellTarget;

/**
 * TeleOp de competencia. La alianza decide qué tags se siguen y qué NECTAR se rechaza.
 *
 * INIT (R103.B): torreta al frente, hood a su casa, copa abajo y la lengua del intake
 *   plegada y suelta (no se pliega con una pelota en la boca o en la garganta: hay que
 *   sacarla). Sin calibrar (IntakeLenguaCfg.CALIBRATED) la lengua y M_F no se mueven: se
 *   pliega a mano.
 *   Gamepad 2 dpad arriba/abajo: cuántas pelotas van precargadas (para la cuenta de G407).
 *
 * LLENO (G407): por la cuenta, o con arriba, copa y fila ocupadas y otra pelota en la
 *   entrada del intake (la 4a se queda en la boca). Lleno, la lengua se queda abajo y M_F
 *   para en BRAKE. Mientras la fila no puede recibir, una pelota en la entrada espera con
 *   M_F en BRAKE (no es atasco).
 *
 * GAMEPAD 1 (piloto)
 *   stick izquierdo        avanzar y moverse de lado
 *   stick derecho X        girar
 *   gatillo derecho        modo lento
 *   bumper derecho         intake sí/no
 *   bumper izquierdo       sostener: intake en reversa
 *   B                      sostener: jalar aunque la cámara vea NECTAR rival
 *
 * GAMEPAD 2 (artillero)
 *   bumper derecho         flywheel sí/no
 *   gatillo derecho        sostener: dispara cada vez que todo está listo
 *   gatillo izquierdo      sostener: empujar con el elevador aunque arriba no se vea pelota
 *                          (sacar una atorada en el hood; flywheel en velocidad, sin mira)
 *   B                      "el camino está libre": quita los bloqueos del alimentador
 *   A                      auto-apuntado sí/no (arranca en sí)
 *   Y                      torreta al frente (apaga el auto-apuntado)
 *   X                      "el HIVE se volteó": cambia cuál CELL está arriba
 *   stick derecho X, dpad izq/der   torreta a mano (sin auto-apuntado o sin objetivo)
 *   dpad arriba/abajo      corregir la cuenta de pelotas
 *
 * Los dos controles vibran cuando la torreta está fija, el flywheel en velocidad, el hood
 * en su lugar y hay pelota arriba.
 */
public abstract class BiobuzzTeleOp extends LinearOpMode {

    protected abstract Alliance alliance();

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        Robot robot = new Robot(hardwareMap, alliance());
        try {
            int preload = 0;
            while (opModeInInit()) {
                long now = System.nanoTime();
                boolean ready = robot.initStep(now);
                if (gamepad2.dpadUpWasPressed()) preload = Math.min(4, preload + 1);
                if (gamepad2.dpadDownWasPressed()) preload = Math.max(0, preload - 1);

                telemetry.addData("Alianza", alliance());
                telemetry.addData("Inicio", ready ? "LISTO (tamaño de inicio)" : "preparando...");
                telemetry.addData("Precargadas", "%d  (gamepad 2 dpad arriba/abajo)", preload);
                telemetry.addData("IMU", !robot.hasImu() ? "NO ENCONTRADO: la torreta no compensa giros"
                        : robot.imuInitOk() ? "ok" : "!! no inicializó");
                if (!robot.hoodStateFound()) {
                    telemetry.addLine("!! Sin archivo del hood: se supuso gap POLLEN y δ de casa. "
                            + "Si no está así, ponlo a mano antes de INIT.");
                }
                robot.intake.addTelemetry(telemetry);
                robot.hood.addTelemetry(telemetry);
                telemetry.update();
            }
            if (isStopRequested()) return;
            robot.inventory.set(preload);
            runMatch(robot);
        } finally {
            robot.stop();
        }
    }

    private void runMatch(Robot robot) {
        boolean autoAim = true;
        boolean intakeOn = false;
        boolean wasReady = false;
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            robot.beginLoop();

            // ----- chasis -----
            double speed = gamepad1.right_trigger > 0.3 ? DriveCfg.SLOW_FACTOR : 1.0;
            robot.drive.drive(-gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed,
                    gamepad1.right_stick_x * speed);

            // ----- botones (cada acción en un solo control: *WasPressed se consume al leerse) -----
            if (gamepad1.rightBumperWasPressed()) intakeOn = !intakeOn;
            if (gamepad2.rightBumperWasPressed()) robot.flywheel.setEnabled(!robot.flywheel.isEnabled());
            if (gamepad2.aWasPressed()) autoAim = !autoAim;
            if (gamepad2.yWasPressed()) {
                autoAim = false;
                robot.turret.center();
            }
            if (gamepad2.xWasPressed()) robot.vision.flipHive();
            if (gamepad2.bWasPressed()) robot.feeder.release();
            if (gamepad2.dpadUpWasPressed()) robot.inventory.set(robot.inventory.count() + 1);
            if (gamepad2.dpadDownWasPressed()) robot.inventory.set(robot.inventory.count() - 1);

            double manualTurret = -gamepad2.right_stick_x;
            if (Math.abs(manualTurret) < 0.08) manualTurret = 0;
            if (gamepad2.dpad_left) manualTurret = 1;
            if (gamepad2.dpad_right) manualTurret = -1;

            // ----- visión y torreta -----
            double yaw = robot.updateYaw();
            CellTarget frame = robot.vision.update(now);
            robot.turret.update(now, yaw, frame, autoAim, manualTurret);

            CellTarget last = robot.vision.getLastTarget();
            boolean distanceFresh = last != null && robot.vision.secondsSinceTarget(now) < 0.5;

            // ----- intake -----
            Intake.Command cmd = gamepad1.left_bumper ? Intake.Command.OUT
                    : intakeOn ? Intake.Command.IN : Intake.Command.STOP;
            robot.intake.setCommand(cmd);
            robot.intake.setRivalOverride(gamepad1.b);
            robot.updateIntake(now);

            // ----- tiro -----
            robot.shooter.update(now, distanceFresh ? last.distanceIn : Double.NaN, robot.feeder.nextBall());
            boolean readyToFire = robot.flywheel.isEnabled()
                    && robot.turret.isLocked(now)
                    && robot.shooter.isReady(now)
                    && robot.feeder.hasUpperBall()
                    && !robot.feeder.isBusy();
            // Empujón: sin mira ni distancia, pero con el flywheel en velocidad y el hood quieto
            boolean forceReady = robot.flywheel.isEnabled()
                    && robot.flywheel.isAtSpeed()
                    && robot.hood.isReady(now)
                    && !robot.feeder.isBusy();
            if (gamepad2.right_trigger > 0.5) {
                if (readyToFire) robot.feeder.requestShot();
            } else if (gamepad2.left_trigger > 0.5) {
                if (forceReady) robot.feeder.forceShot();
            } else {
                robot.feeder.cancelShot();
            }
            robot.feeder.update(now, robot.intake.isFeeding(), robot.intake.wantsFilaReverse(),
                    robot.intake.holdsFila(), robot.flywheel.consumeShotDip(), robot.flywheel.isEnabled());
            robot.updateInventory();

            if (readyToFire && !wasReady) {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            }
            wasReady = readyToFire;

            // ----- telemetría -----
            telemetry.addData("Alianza", "%s   apuntado %s   pelotas %d/4%s%s", alliance(),
                    autoAim ? "AUTO" : "manual", robot.inventory.count(),
                    robot.fourthAtEntry() ? " +1 en la entrada" : "", robot.isFull() ? " LLENO" : "");
            telemetry.addData("TIRO", readyToFire ? ">>> LISTO <<<" : "esperando");
            robot.vision.addTelemetry(telemetry, now);
            robot.turret.addTelemetry(telemetry);
            robot.shooter.addTelemetry(telemetry);
            robot.feeder.addTelemetry(telemetry);
            robot.intake.addTelemetry(telemetry);
            telemetry.addData("Yaw", "%.1f°  (girar a la izquierda debe subirlo)%s", yaw,
                    robot.imuFailedReads() > 0 ? String.format("  · %d lecturas fallidas", robot.imuFailedReads()) : "");
            telemetry.addData("Ciclo", "%.0f ms", loopTimer.milliseconds());
            loopTimer.reset();
            telemetry.update();
        }
    }
}
