package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.HoodCfg;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.util.FileStateStore;

import java.util.List;

/**
 * Calibra los dos servos del hood sin romper la ley (servo de ángulo = -gap + δ).
 * Solo necesita "hoodGap" y "hoodAngulo". Hace falta un calibrador de luz entre la cara
 * interior de la lámina y la llanta FlatTop.
 *
 * ANTES DE EMPEZAR: si es la primera vez, con el robot apagado deja el hood a mano en
 * POLLEN (eslabón en su tope de POLLEN, hood en su posición normal). El OpMode arranca
 * mandando la posición guardada; la primera vez supone POLLEN.
 *
 * DÓNDE MEDIR LA LUZ (documento del CAD v138, 4.1). La PUNTA es el canto de arriba de la
 * lámina (θ 55°, por donde sale la pelota); el LABIO es su canto de abajo (θ 0°, junto al
 * perno P): mídelo en la cara de la lámina unos milímetros arriba del chaflán de 0.6 mm, NO
 * en el labio de entrada de PA-CF (va 0.1 mm atrás de la lámina) ni en el labio guía del
 * tope (no se mueve con el hood). El labio casi no cambia con δ (63.1 a 63.2 mm de δ 0 a
 * ±10.5); la punta cambia ~1.7 mm por grado. Por eso δ = 0 es "punta = labio".
 *
 * Ir a NECTAR está BLOQUEADO hasta confirmar el signo del ángulo y el sentido del gap, y
 * mientras los signos de los dos servos no cuadren con cómo están montados (HoodCfg.
 * SERVOS_FACE_OPPOSITE: en el CAD, µs por grado de signo contrario). Con cualquiera al revés,
 * la lámina llega a la carcasa de la Limelight en un tercio de segundo.
 *
 * PROCEDIMIENTO (los valores cambian en vivo; al final se copian a Config). Y cambia el
 * campo que se ajusta: dpad arriba/abajo ±5 µs (±0.5° en el paso 6), bumpers ±1 µs (±0.1°).
 *  1. "gap POLLEN" (X manda el gap a POLLEN): ajusta GAP_POLLEN_US hasta que el eslabón
 *     llegue a su tope de POLLEN sin forzar (si el servo zumba o la corriente sube, está
 *     empujando el tope).
 *  2. "ángulo cero": ajusta ANGLE_ZERO_US hasta que la luz en la punta sea igual a la del
 *     labio: ~62.6 mm con el eslabón en su tope de POLLEN (63.0 en el 0 del CAD).
 *  3. Signo: sostén el gatillo derecho (δ = +10): la PUNTA de la lámina BAJA hacia el
 *     flywheel (la luz en la punta pasa de ~63 a ~46 mm) y el labio casi no se mueve. Gatillo
 *     izquierdo (δ = -10): la punta SUBE (~79 mm). Si va al revés, A cambia el signo (solo en
 *     casa). Cuando esté bien, dpad derecho lo confirma.
 *  4. Sentido del gap: dpad izquierdo mueve el gap 2.5° hacia NECTAR. El eslabón tiene que
 *     separarse de su tope de POLLEN sin forzar y el hood trasladarse sin girar. Si es así,
 *     botón del stick derecho lo confirma. Si empuja contra el tope, GAP_NECTAR_US está del
 *     lado equivocado: elige "gap NECTAR" y, ya en POLLEN, el botón del stick izquierdo lo
 *     refleja al otro lado de GAP_POLLEN_US.
 *  5. B: el gap va a NECTAR en rampa y el servo de ángulo compensa solo. Elige "gap NECTAR"
 *     y ajusta GAP_NECTAR_US hasta que el eslabón llegue al tope de NECTAR sin forzar.
 *  6. Ya en NECTAR con δ 0, elige "compensación NECTAR" y ajusta GAP_NECTAR_DEG en pasos de
 *     0.1° hasta que la luz en la punta sea igual a la del labio, ≈80 mm (CALC: 80.2 en el
 *     labio y 80.1 en la punta). Si la punta da más que el labio, el hood quedó abierto
 *     (δ < 0): GAP_NECTAR_DEG más negativo (dpad arriba / bumper derecho). Arranca en -36.1
 *     (los topes están en +0.91° y -35.18°: entre ellos el eslabón gira 36.1°, no los 34.3
 *     del CAD entre POLLEN y NECTAR). El servo de gap no se mueve: solo el de ángulo.
 *  7. En NECTAR el hood tiene que haberse TRASLADADO sin girar. Si giró, el signo o la escala
 *     del ángulo están mal: no tires así.
 */
@TeleOp(name = "Tune: Hood", group = "2 Calibracion")
public class TuneHood extends LinearOpMode {
    private enum Field { GAP_POLLEN, ANGLE_ZERO, GAP_NECTAR, NECTAR_DEG }

    /** Movimiento corto del gap para probar el sentido, en grados hacia NECTAR. */
    private static final double GAP_JOG_DEG = 2.5;

    @Override
    public void runOpMode() {
        FileStateStore store = new FileStateStore();
        Hood hood = new Hood(hardwareMap, store);
        hood.allowFullSafeRange(true);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        Field field = Field.GAP_POLLEN;
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
            String note = "";

            // Elegir campo no mueve nada
            if (gamepad1.yWasPressed()) field = Field.values()[(field.ordinal() + 1) % Field.values().length];

            boolean gapAtPollen = hood.getGapDeg() == HoodCfg.GAP_POLLEN_DEG && gapGoal == HoodCfg.GAP_POLLEN_DEG;
            boolean atNectar = hood.getGapDeg() == HoodCfg.GAP_NECTAR_DEG && gapGoal == HoodCfg.GAP_NECTAR_DEG;
            // Cambiar un signo con el servo lejos de 0° lo haría girar al revés el doble: solo en casa
            boolean atHome = gapAtPollen && Math.abs(Hood.angleServoDeg(hood.getGapDeg(), hood.getDeltaDeg())) < 1.0;
            if (gamepad1.aWasPressed() && atHome) {
                HoodCfg.ANGLE_US_PER_DEG = -HoodCfg.ANGLE_US_PER_DEG;
                hood.reapplyPwmRanges();
                angleSignOk = false;
                gapDirOk = false;
            }
            if (gamepad1.leftStickButtonWasPressed() && field == Field.GAP_NECTAR) {
                if (atHome) {                  // reflejar GAP_NECTAR_US al otro lado de POLLEN
                    HoodCfg.GAP_NECTAR_US = 2 * HoodCfg.GAP_POLLEN_US - HoodCfg.GAP_NECTAR_US;
                    hood.reapplyPwmRanges();
                    gapDirOk = false;
                } else {
                    note = "reflejar GAP_NECTAR_US solo en POLLEN con δ 0 (X)";
                }
            }
            if (gamepad1.dpadRightWasPressed()) angleSignOk = true;

            if (gamepad1.xWasPressed()) gapGoal = HoodCfg.GAP_POLLEN_DEG;
            if (gamepad1.dpadLeftWasPressed() && angleSignOk) gapGoal = jogGap;
            boolean atJog = gapGoal == jogGap && hood.getGapDeg() == jogGap;
            if (gamepad1.rightStickButtonWasPressed() && atJog) gapDirOk = true;
            boolean signs = Hood.signsConsistent();
            if (gamepad1.bWasPressed()) {
                if (!angleSignOk || !gapDirOk) {
                    note = "B bloqueado: confirma el signo (paso 3) y el sentido del gap (paso 4)";
                } else if (!signs) {
                    note = "B bloqueado: el signo del ángulo no cuadra con el del gap (HoodCfg.SERVOS_FACE_OPPOSITE)";
                } else {
                    gapGoal = HoodCfg.GAP_NECTAR_DEG;
                }
            }
            if (!signs && gapGoal == HoodCfg.GAP_NECTAR_DEG) gapGoal = HoodCfg.GAP_POLLEN_DEG;   // A en NECTAR no se puede, pero por si acaso

            double step = 0;
            if (gamepad1.dpadUpWasPressed()) step = 5;
            if (gamepad1.dpadDownWasPressed()) step = -5;
            if (gamepad1.rightBumperWasPressed()) step = 1;
            if (gamepad1.leftBumperWasPressed()) step = -1;
            if (step != 0) {
                switch (field) {
                    case GAP_POLLEN:
                        HoodCfg.GAP_POLLEN_US += step;
                        hood.reapplyPwmRanges();
                        break;
                    case ANGLE_ZERO:
                        HoodCfg.ANGLE_ZERO_US += step;
                        hood.reapplyPwmRanges();
                        break;
                    case GAP_NECTAR:
                        HoodCfg.GAP_NECTAR_US += step;
                        hood.reapplyPwmRanges();
                        break;
                    case NECTAR_DEG:
                        if (atNectar && hood.getDeltaDeg() == 0) {
                            double old = HoodCfg.GAP_NECTAR_DEG;
                            HoodCfg.GAP_NECTAR_DEG = Math.round((old - step / 10.0) * 10) / 10.0;   // + = más negativo = cierra
                            hood.nectarGapChanged(old);
                            gapGoal = HoodCfg.GAP_NECTAR_DEG;
                            jogGap = HoodCfg.GAP_POLLEN_DEG + Math.signum(HoodCfg.GAP_NECTAR_DEG - HoodCfg.GAP_POLLEN_DEG) * GAP_JOG_DEG;
                        } else {
                            note = "la compensación de NECTAR solo se ajusta en NECTAR con δ 0 (suelta los gatillos)";
                        }
                        break;
                }
            }

            double delta = 0;
            if (gamepad1.right_trigger > 0.5) delta = HoodCfg.DELTA_SAFE_MAX_DEG;
            if (gamepad1.left_trigger > 0.5) delta = HoodCfg.DELTA_SAFE_MIN_DEG;
            hood.setGapDegForTuning(gapGoal);
            hood.setDelta(delta);
            hood.update(now);

            telemetry.addData("Ajustando (Y cambia)", field);
            telemetry.addData("GAP_POLLEN_US", "%.0f", HoodCfg.GAP_POLLEN_US);
            telemetry.addData("ANGLE_ZERO_US", "%.0f", HoodCfg.ANGLE_ZERO_US);
            telemetry.addData("ANGLE_US_PER_DEG", "%.3f", HoodCfg.ANGLE_US_PER_DEG);
            telemetry.addData("GAP_NECTAR_US", "%.0f", HoodCfg.GAP_NECTAR_US);
            telemetry.addData("GAP_NECTAR_DEG", "%.1f°  (servo de ángulo hasta %.1f°)", HoodCfg.GAP_NECTAR_DEG, Hood.angleMaxDeg());
            telemetry.addLine();
            hood.addTelemetry(telemetry);
            telemetry.addData("µs gap / ángulo", "%.0f / %.0f", Hood.gapToUs(hood.getGapDeg()),
                    Hood.angleToUs(Hood.angleServoDeg(hood.getGapDeg(), hood.getDeltaDeg())));
            telemetry.addData("Signo del ángulo", angleSignOk ? "confirmado" : "SIN CONFIRMAR (paso 3)");
            telemetry.addData("Sentido del gap", gapDirOk ? "confirmado" : "SIN CONFIRMAR (paso 4)");
            telemetry.addData("Signos gap/ángulo", signs ? "cuadran con el armado" : "NO CUADRAN: NECTAR bloqueado");
            for (LynxModule hub : hubs) {
                telemetry.addData("Corriente " + hub.getDeviceName(), "%.2f A", hub.getCurrent(CurrentUnit.AMPS));
            }
            if (!note.isEmpty()) telemetry.addLine("!! " + note);
            telemetry.addLine();
            telemetry.addLine("dpad arriba/abajo ±5 µs   bumpers ±1 µs   gatillos = δ ±10");
            telemetry.addLine("X = POLLEN   dpad izq = gap 2.5° (prueba)   B = NECTAR");
            telemetry.addLine(atHome ? "A = cambiar signo   dpad der = signo bien"
                    : "dpad der = signo bien   (el signo solo se cambia en POLLEN con δ 0)");
            if (atJog) telemetry.addLine("Botón del stick derecho = el gap se movió hacia NECTAR sin forzar");
            if (field == Field.NECTAR_DEG) {
                telemetry.addLine("Compensación NECTAR: dpad ±0.5°, bumpers ±0.1°. Arriba/der. = más negativo = la punta CIERRA");
            }
            telemetry.update();
        }
    }
}
