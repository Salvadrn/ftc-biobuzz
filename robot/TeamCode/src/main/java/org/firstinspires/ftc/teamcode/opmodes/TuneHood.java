package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
 * Ir a NECTAR está BLOQUEADO hasta confirmar el signo del ángulo y el sentido del gap: con
 * cualquiera de los dos al revés, la lámina llega a la Limelight en un tercio de segundo
 * (revisión del 17-nov, límites del CAD v104).
 *
 * PROCEDIMIENTO (los µs cambian en vivo; al final se copian a Config)
 *  1. Gap en POLLEN (X). Con "ángulo cero" elegido (Y cambia el campo), ajusta con dpad
 *     arriba/abajo (±5 µs) o bumpers (±1 µs) hasta δ = 0: la lámina a 63 mm del flywheel.
 *  2. Signo: sostén el gatillo derecho (δ = +10) y el izquierdo (δ = -10). Con el derecho
 *     el hood tiene que abrir hacia arriba. Si va al revés, A cambia el signo (solo en
 *     casa). Cuando esté bien, dpad derecho lo confirma.
 *  3. Elige "gap POLLEN" y ajusta hasta que el eslabón llegue a su tope sin forzar.
 *  4. Sentido del gap: dpad izquierdo mueve el gap 2.5° hacia NECTAR. El eslabón tiene que
 *     separarse de su tope de POLLEN sin forzar y el hood trasladarse sin girar. Si es así,
 *     botón del stick derecho lo confirma. Si empuja contra el tope, GAP_NECTAR_US está del
 *     lado equivocado: corrígelo con "gap NECTAR" antes de seguir.
 *  5. B: el gap va a NECTAR en rampa y el servo de ángulo compensa solo. Elige "gap NECTAR"
 *     y ajusta hasta que el eslabón llegue al tope de NECTAR sin forzar (si el servo zumba,
 *     está empujando el tope).
 *  6. En NECTAR, el hood tiene que haberse TRASLADADO sin girar. Si giró, el signo o la
 *     escala del ángulo están mal: no tires así.
 */
@TeleOp(name = "Tune: Hood", group = "2 Calibracion")
public class TuneHood extends LinearOpMode {
    private enum Field { ANGLE_ZERO, GAP_POLLEN, GAP_NECTAR }

    /** Movimiento corto del gap para probar el sentido, en grados hacia NECTAR. */
    private static final double GAP_JOG_DEG = 2.5;

    @Override
    public void runOpMode() {
        FileStateStore store = new FileStateStore();
        Hood hood = new Hood(hardwareMap, store);
        hood.allowFullSafeRange(true);
        Field field = Field.ANGLE_ZERO;
        double gapGoal = HoodCfg.GAP_POLLEN_DEG;
        double jogGap = HoodCfg.GAP_POLLEN_DEG + Math.signum(HoodCfg.GAP_NECTAR_DEG - HoodCfg.GAP_POLLEN_DEG) * GAP_JOG_DEG;
        boolean angleSignOk = false, gapDirOk = false;

        telemetry.addLine("Primera vez: hood a mano en POLLEN antes de dar INIT.");
        if (!store.existed()) telemetry.addLine("!! No hay archivo del hood: se supone POLLEN y δ 0.");
        telemetry.update();
        waitForStart();
        hood.start(System.nanoTime());

        while (opModeIsActive()) {
            long now = System.nanoTime();

            // Elegir campo no mueve nada
            if (gamepad1.yWasPressed()) field = Field.values()[(field.ordinal() + 1) % Field.values().length];

            boolean gapAtPollen = hood.getGapDeg() == HoodCfg.GAP_POLLEN_DEG && gapGoal == HoodCfg.GAP_POLLEN_DEG;
            // Cambiar el signo con el servo lejos de 0° lo haría girar al revés el doble: solo en casa
            boolean atHome = gapAtPollen && Math.abs(Hood.angleServoDeg(hood.getGapDeg(), hood.getDeltaDeg())) < 1.0;
            if (gamepad1.aWasPressed() && atHome) {
                HoodCfg.ANGLE_US_PER_DEG = -HoodCfg.ANGLE_US_PER_DEG;
                hood.reapplyPwmRanges();
                angleSignOk = false;
                gapDirOk = false;
            }
            if (gamepad1.dpadRightWasPressed()) angleSignOk = true;

            if (gamepad1.xWasPressed()) gapGoal = HoodCfg.GAP_POLLEN_DEG;
            if (gamepad1.dpadLeftWasPressed() && angleSignOk) gapGoal = jogGap;
            boolean atJog = gapGoal == jogGap && hood.getGapDeg() == jogGap;
            if (gamepad1.rightStickButtonWasPressed() && atJog) gapDirOk = true;
            boolean bPressed = gamepad1.bWasPressed();
            if (bPressed && angleSignOk && gapDirOk) gapGoal = HoodCfg.GAP_NECTAR_DEG;

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
            hood.setGapDegForTuning(gapGoal);
            hood.setDelta(delta);
            hood.update(now);

            telemetry.addData("Ajustando (Y cambia)", field);
            telemetry.addData("ANGLE_ZERO_US", "%.0f", HoodCfg.ANGLE_ZERO_US);
            telemetry.addData("ANGLE_US_PER_DEG", "%.3f", HoodCfg.ANGLE_US_PER_DEG);
            telemetry.addData("GAP_POLLEN_US", "%.0f", HoodCfg.GAP_POLLEN_US);
            telemetry.addData("GAP_NECTAR_US", "%.0f", HoodCfg.GAP_NECTAR_US);
            telemetry.addLine();
            hood.addTelemetry(telemetry);
            telemetry.addData("µs gap / ángulo", "%.0f / %.0f", Hood.gapToUs(hood.getGapDeg()),
                    Hood.angleToUs(Hood.angleServoDeg(hood.getGapDeg(), hood.getDeltaDeg())));
            telemetry.addData("Signo del ángulo", angleSignOk ? "confirmado" : "SIN CONFIRMAR (paso 2)");
            telemetry.addData("Sentido del gap", gapDirOk ? "confirmado" : "SIN CONFIRMAR (paso 4)");
            if (bPressed && !(angleSignOk && gapDirOk)) telemetry.addLine("!! B bloqueado: confirma signo y sentido");
            telemetry.addLine();
            telemetry.addLine("dpad arriba/abajo ±5 µs   bumpers ±1 µs   gatillos = δ ±10");
            telemetry.addLine("X = POLLEN   dpad izq = gap 2.5° (prueba)   B = NECTAR");
            telemetry.addLine(atHome ? "A = cambiar signo   dpad der = signo bien"
                    : "dpad der = signo bien   (el signo solo se cambia en POLLEN con δ 0)");
            if (atJog) telemetry.addLine("Botón del stick derecho = el gap se movió hacia NECTAR sin forzar");
            telemetry.update();
        }
    }
}
