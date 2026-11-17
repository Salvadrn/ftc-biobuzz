package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config.HoodCfg;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.util.FileStateStore;

/**
 * Calibra los dos servos del hood sin romper la ley (servo de ángulo = -gap + δ).
 * Solo necesita "hoodGap" y "hoodAngulo".
 *
 * ANTES DE EMPEZAR: si es la primera vez, con el robot apagado deja el hood a mano en
 * POLLEN (eslabón en su tope de POLLEN, hood en su posición normal). El OpMode arranca
 * mandando la posición guardada; la primera vez supone POLLEN.
 *
 * PROCEDIMIENTO (los µs cambian en vivo; al final se copian a Config)
 *  1. Con el gap en POLLEN (botón X), elige "ángulo cero" (Y) y ajusta con dpad (±5 µs)
 *     o bumpers (±1 µs) hasta que el hood quede en δ = 0: la lámina a 63 mm del flywheel.
 *  2. Gatillo derecho: δ = +10; izquierdo: δ = -10. Si se mueve al revés, A cambia el
 *     signo de ANGLE_US_PER_DEG.
 *  3. Elige "gap POLLEN" (dpad con X sostenido) y ajusta hasta que el eslabón llegue a su
 *     tope de POLLEN sin forzar.
 *  4. Botón B: el gap va a NECTAR en rampa y el servo de ángulo compensa solo. Elige
 *     "gap NECTAR" y ajusta hasta que el eslabón llegue al tope de NECTAR sin forzar
 *     (escucha el servo: si zumba, está empujando el tope).
 *  5. En NECTAR, el hood tiene que haberse TRASLADADO sin girar. Si giró, el signo o la
 *     escala del ángulo están mal: no tires así.
 */
@TeleOp(name = "Tune: Hood", group = "2 Calibracion")
public class TuneHood extends LinearOpMode {
    private enum Field { ANGLE_ZERO, GAP_POLLEN, GAP_NECTAR }

    @Override
    public void runOpMode() {
        Hood hood = new Hood(hardwareMap, new FileStateStore());
        hood.allowFullSafeRange(true);
        Field field = Field.ANGLE_ZERO;
        BallType gapFor = BallType.POLLEN;

        telemetry.addLine("Primera vez: hood a mano en POLLEN antes de dar INIT.");
        telemetry.update();
        waitForStart();
        hood.start(System.nanoTime());

        while (opModeIsActive()) {
            long now = System.nanoTime();

            if (gamepad1.yWasPressed()) field = Field.ANGLE_ZERO;
            if (gamepad1.x && gamepad1.dpad_up || gamepad1.x && gamepad1.dpad_down) field = Field.GAP_POLLEN;
            if (gamepad1.b && gamepad1.dpad_up || gamepad1.b && gamepad1.dpad_down) field = Field.GAP_NECTAR;
            if (gamepad1.xWasPressed()) gapFor = BallType.POLLEN;
            if (gamepad1.bWasPressed()) gapFor = BallType.NECTAR_RED;
            // Cambiar el signo con el servo lejos de 0° lo haría girar al revés el doble: solo en casa
            boolean atHome = Math.abs(Hood.angleServoDeg(hood.getGapDeg(), hood.getDeltaDeg())) < 1.0;
            if (gamepad1.aWasPressed() && atHome) {
                HoodCfg.ANGLE_US_PER_DEG = -HoodCfg.ANGLE_US_PER_DEG;
                hood.reapplyPwmRanges();
            }

            double step = 0;
            if (gamepad1.dpadUpWasPressed()) step = 5;
            if (gamepad1.dpadDownWasPressed()) step = -5;
            if (gamepad1.rightBumperWasPressed()) step = 1;
            if (gamepad1.leftBumperWasPressed()) step = -1;
            if (step != 0) {
                switch (field) {
                    case ANGLE_ZERO: HoodCfg.ANGLE_ZERO_US += step; break;
                    case GAP_POLLEN: HoodCfg.GAP_POLLEN_US += step; break;
                    case GAP_NECTAR: HoodCfg.GAP_NECTAR_US += step; break;
                }
                hood.reapplyPwmRanges();
            }

            double delta = 0;
            if (gamepad1.right_trigger > 0.5) delta = HoodCfg.DELTA_SAFE_MAX_DEG;
            if (gamepad1.left_trigger > 0.5) delta = HoodCfg.DELTA_SAFE_MIN_DEG;
            hood.setGapFor(gapFor);
            hood.setDelta(delta);
            hood.update(now);

            telemetry.addData("Ajustando", field);
            telemetry.addData("ANGLE_ZERO_US", "%.0f", HoodCfg.ANGLE_ZERO_US);
            telemetry.addData("ANGLE_US_PER_DEG", "%.3f", HoodCfg.ANGLE_US_PER_DEG);
            telemetry.addData("GAP_POLLEN_US", "%.0f", HoodCfg.GAP_POLLEN_US);
            telemetry.addData("GAP_NECTAR_US", "%.0f", HoodCfg.GAP_NECTAR_US);
            telemetry.addLine();
            hood.addTelemetry(telemetry);
            telemetry.addData("µs gap / ángulo", "%.0f / %.0f", Hood.gapToUs(hood.getGapDeg()),
                    Hood.angleToUs(Hood.angleServoDeg(hood.getGapDeg(), hood.getDeltaDeg())));
            telemetry.addLine();
            telemetry.addLine("Y = ángulo cero   X+dpad = gap POLLEN   B+dpad = gap NECTAR");
            telemetry.addLine("X / B = ir a POLLEN / NECTAR   gatillos = δ ±10");
            telemetry.addLine(atHome ? "A = cambiar signo del ángulo" : "(el signo solo se cambia en POLLEN con δ 0)");
            telemetry.update();
        }
    }
}
