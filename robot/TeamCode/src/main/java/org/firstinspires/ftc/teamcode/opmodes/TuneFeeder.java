package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config.FeederCfg;
import org.firstinspires.ftc.teamcode.Config.HoodCfg;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.SensorCfg;
import org.firstinspires.ftc.teamcode.Config.ShotCfg;
import org.firstinspires.ftc.teamcode.feeder.Feeder;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;
import org.firstinspires.ftc.teamcode.sensors.RevColorReader;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.util.FileStateStore;

/**
 * Prueba la fila, la copa y el elevador con pelotas metidas a mano.
 * Necesita fila, copa, elevador, flywheel, hoodGap, hoodAngulo, colorArriba, colorCopa y
 * bbFila. El hood se controla aquí también: su gap sigue a la pelota que va a salir (una
 * NECTAR por el gap de POLLEN se atora).
 *
 * MODO MANUAL (arranca así), para confirmar sentidos y medir:
 *   gatillo derecho   fila hacia adentro (FILA_POWER_IN). Si saca la pelota, cambia el signo.
 *   gatillo izquierdo fila hacia afuera
 *   dpad arriba       elevador sube (ELEVATOR_STAGE_POWER). Si baja, ELEVATOR_REVERSED.
 *   dpad abajo        elevador baja, SOLO con la copa abajo y vacía (la guía del CAD lo pide)
 *   dpad der / izq    copa a la siguiente altura de 0, 27 y 54 mm (no sube si arriba hay pelota)
 *
 * MODO CICLO (botón Y): corre la máquina de estados del robot.
 *   bumper derecho    flywheel sí/no (a ShotCfg.IDLE_RPM)
 *   A                 un tiro: solo con el flywheel en velocidad y el hood en su lugar
 *   B                 "el camino está libre" (quita los bloqueos del alimentador)
 *   La copa y el elevador rellenan solos; la fila carga la copa.
 *
 * MEDIR ANTES DE DEJAR QUE RELLENE SOLO (documento del CAD v138, 4.7), con POLLEN y NECTAR.
 * El V3 de arriba (z 128) ve la pelota de la copa antes de que llegue y la de arriba antes
 * de que llegue a su etapa; lo que lee fuera de su eje no está publicado:
 *  1. Con NECTAR en la copa abajo y nada arriba, el V3 de arriba tiene que leer MÁS que
 *     SensorCfg.UPPER (presente a menos de 50 mm). Si no, la copa nunca sube: bajar ese umbral.
 *  2. Pelota en la copa a 0, 27 y 54 mm (dpad der/izq): anota lo que lee arriba. La altura más
 *     alta en la que todavía NO la ve es FeederCfg.COPA_UPPER_BLIND_MM.
 *  3. Pelota subiendo hasta la etapa (dpad arriba): la telemetría guarda la distancia mínima.
 *  4. Encoder del elevador (encoder 2): en modo manual, dpad arriba tiene que SUBIR las
 *     cuentas. Mide con regla cuánto sube la superficie de las Gecko por cuenta
 *     (ELEVATOR_MM_PER_TICK, CALC 0.69 mm).
 *  5. PRUEBA DE BANCO DEL CAD (v146), con el flywheel APAGADO, en modo ciclo: deja que suba
 *     una pelota y mira dónde se queda la de arriba y si se queda, BAJA o SE TREPA (la cuña de
 *     las locas la empuja hacia arriba arriba de z 128). Con NECTAR y con POLLEN, con y sin
 *     pelota en la copa. "Última subida" dice cómo terminó y a qué altura la deja el encoder.
 *     Las Gecko de z 128 la sostienen hasta z 147.0 (NECTAR) y 145.4 (POLLEN); abajo no tiene
 *     que tocar la de la copa (144.8 NECTAR sobre NECTAR, 134.4 POLLEN sobre NECTAR). Ajusta
 *     LIFT_TARGET_Z_* (y, sin encoder, LIFT_PASS_* y LIFT_EXTRA_MM). Si se trepa o no cabe en
 *     esa ventana, avisar al CAD.
 */
@TeleOp(name = "Tune: Alimentador", group = "2 Calibracion")
public class TuneFeeder extends LinearOpMode {
    @Override
    public void runOpMode() {
        CRServo fila = hardwareMap.get(CRServo.class, Names.FILA_SERVO);
        Servo copa = hardwareMap.get(Servo.class, Names.COPA_SERVO);
        DcMotor elevator = hardwareMap.get(DcMotor.class, Names.ELEVATOR);
        BallSensor upper = new BallSensor(new RevColorReader(hardwareMap, Names.COLOR_UPPER, SensorCfg.COLOR_GAIN,
                SensorCfg.UPPER_FAST_PROXIMITY), SensorCfg.UPPER, SensorCfg.COLOR_MIN_INTERVAL_S);
        BallSensor inCopa = new BallSensor(new RevColorReader(hardwareMap, Names.COLOR_COPA, SensorCfg.COLOR_GAIN),
                SensorCfg.COPA, SensorCfg.COLOR_MIN_INTERVAL_S);
        BeamSensor filaBeam = new BeamSensor(hardwareMap, Names.BEAM_FILA, SensorCfg.BEAM_ACTIVE_HIGH);
        Feeder feeder = new Feeder(fila, copa, elevator, upper, inCopa, filaBeam);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Hood hood = new Hood(hardwareMap, new FileStateStore());
        hood.start(System.nanoTime());

        waitForStart();
        long start = System.nanoTime();
        feeder.initHardware(start);
        boolean cycleMode = false;
        double lastFilaPower = Double.NaN;
        final double[] copaLevels = {0, 27, FeederCfg.COPA_TRAVEL_MM};   // alturas del documento del CAD
        int copaLevel = 0;
        double minUpperMm = Double.NaN;                                  // mínimo mientras sube el elevador

        while (opModeIsActive()) {
            long now = System.nanoTime();
            String note = "";
            // Se leen en cada ciclo: *WasPressed se queda guardado hasta leerse, y uno apretado en
            // modo ciclo subiría la copa al regresar a manual
            boolean copaUpPressed = gamepad1.dpadRightWasPressed();
            boolean copaDownPressed = gamepad1.dpadLeftWasPressed();
            boolean liftPressed = gamepad1.dpadUpWasPressed();
            if (gamepad1.yWasPressed()) {
                cycleMode = !cycleMode;
                feeder.stop();
                if (fila instanceof PwmControl) ((PwmControl) fila).setPwmDisable();
                lastFilaPower = Double.NaN;
                copaLevel = 0;
                feeder.setCopaTargetForTuning(0);
            }

            hood.setGapFor(feeder.nextBall());
            hood.setDelta(HoodCfg.DELTA_HOME_DEG);
            hood.update(now);

            if (cycleMode) {
                if (gamepad1.rightBumperWasPressed()) flywheel.setEnabled(!flywheel.isEnabled());
                flywheel.setTargetRpm(ShotCfg.IDLE_RPM);
                flywheel.update(now);
                boolean canFire = flywheel.isEnabled() && flywheel.isAtSpeed() && hood.isReady(now);
                if (gamepad1.aWasPressed()) {
                    if (canFire) feeder.requestShot();
                    else telemetry.addLine("!! A: primero flywheel en velocidad y hood en su lugar");
                }
                if (gamepad1.bWasPressed()) feeder.release();
                feeder.update(now, false, false, false, flywheel.consumeShotDip(), flywheel.isEnabled());
            } else {
                feeder.initUpdate(now);
                double filaPower = 0;
                if (gamepad1.right_trigger > 0.3) filaPower = FeederCfg.FILA_POWER_IN;
                if (gamepad1.left_trigger > 0.3) filaPower = -FeederCfg.FILA_POWER_IN;
                if (filaPower != lastFilaPower) {      // solo al cambiar: apagar borra la caché del SDK
                    if (filaPower != 0) {
                        if (fila instanceof PwmControl) ((PwmControl) fila).setPwmEnable();
                        fila.setPower(filaPower);
                    } else {
                        fila.setPower(0);
                        if (fila instanceof PwmControl) ((PwmControl) fila).setPwmDisable();   // en 0 se arrastra
                    }
                    lastFilaPower = filaPower;
                }

                // Copa a 0, 27 o 54 mm para ver qué lee el V3 de arriba (nunca sube con pelota arriba)
                if (copaUpPressed && copaLevel < copaLevels.length - 1) {
                    if (feeder.getCopaMm() == 0 && upper.hasBall()) {
                        note = "arriba hay pelota (o el V3 no lee): la copa no sube";
                    } else {
                        copaLevel++;
                    }
                }
                if (copaDownPressed && copaLevel > 0) copaLevel--;
                feeder.setCopaTargetForTuning(copaLevels[copaLevel]);

                double elev = 0;
                if (liftPressed) minUpperMm = Double.NaN;   // nueva subida
                if (gamepad1.dpad_up) elev = FeederCfg.ELEVATOR_STAGE_POWER;
                boolean canReverse = feeder.getCopaMm() == 0 && !inCopa.hasBall();
                if (gamepad1.dpad_down && canReverse) elev = -FeederCfg.ELEVATOR_REVERSE_POWER;
                elevator.setPower(elev);
                if (gamepad1.dpad_down && !canReverse) note = "reversa solo con la copa abajo y vacía";
                upper.read(now);
                inCopa.read(now);
                if (elev > 0 && upper.hasBall() && !upper.isFault()) {
                    minUpperMm = Double.isNaN(minUpperMm) ? upper.distanceMm() : Math.min(minUpperMm, upper.distanceMm());
                }
            }

            telemetry.addData("Modo", cycleMode ? "CICLO (Y = manual)" : "MANUAL (Y = ciclo)");
            feeder.addTelemetry(telemetry);
            hood.addTelemetry(telemetry);
            telemetry.addData("V3 arriba", upper.describe());
            telemetry.addData("V3 copa", inCopa.describe());
            telemetry.addData("BB3 fila", filaBeam.isActive() ? "cortado (hay pelota)" : "libre");
            telemetry.addData("Encoder elevador", "%d cuentas (%.1f mm de Gecko)", elevator.getCurrentPosition(),
                    elevator.getCurrentPosition() * FeederCfg.ELEVATOR_MM_PER_TICK);
            if (cycleMode) {
                telemetry.addData("Última subida", feeder.liftTrace().isEmpty() ? "todavía no" : feeder.liftTrace());
            } else {
                telemetry.addData("Copa", "%.0f mm (pedida %.0f; dpad der/izq)", feeder.getCopaMm(), copaLevels[copaLevel]);
                telemetry.addData("Mínimo arriba al subir", Double.isNaN(minUpperMm) ? "-" : String.format("%.1f mm", minUpperMm));
            }
            flywheel.addTelemetry(telemetry);
            if (!note.isEmpty()) telemetry.addLine("!! " + note);
            telemetry.update();
        }
        feeder.stop();
        flywheel.stop();
    }
}
