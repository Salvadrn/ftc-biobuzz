package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config.FeederCfg;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.SensorCfg;
import org.firstinspires.ftc.teamcode.feeder.Feeder;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;
import org.firstinspires.ftc.teamcode.sensors.RevColorReader;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

/**
 * Prueba la fila, la copa y el elevador con pelotas metidas a mano.
 * Necesita fila, copa, elevador, flywheel, colorArriba, colorCopa y bbFila.
 *
 * MODO MANUAL (arranca así), para confirmar sentidos:
 *   gatillo derecho   fila hacia adentro (FILA_POWER_IN). Si saca la pelota, cambia el signo.
 *   gatillo izquierdo fila hacia afuera
 *   dpad arriba       elevador sube (ELEVATOR_STAGE_POWER). Si baja, ELEVATOR_REVERSED.
 *   dpad abajo        elevador baja, SOLO con la copa abajo y vacía (la guía del CAD lo pide)
 *
 * MODO CICLO (botón Y): corre la máquina de estados del robot.
 *   bumper derecho    flywheel sí/no (a ShotCfg.IDLE_RPM)
 *   A                 un tiro
 *   La copa y el elevador rellenan solos; la fila carga la copa.
 */
@TeleOp(name = "Tune: Alimentador", group = "2 Calibracion")
public class TuneFeeder extends LinearOpMode {
    @Override
    public void runOpMode() {
        CRServo fila = hardwareMap.get(CRServo.class, Names.FILA_SERVO);
        Servo copa = hardwareMap.get(Servo.class, Names.COPA_SERVO);
        DcMotor elevator = hardwareMap.get(DcMotor.class, Names.ELEVATOR);
        BallSensor upper = new BallSensor(new RevColorReader(hardwareMap, Names.COLOR_UPPER, SensorCfg.COLOR_GAIN),
                SensorCfg.UPPER, SensorCfg.COLOR_MIN_INTERVAL_S);
        BallSensor inCopa = new BallSensor(new RevColorReader(hardwareMap, Names.COLOR_COPA, SensorCfg.COLOR_GAIN),
                SensorCfg.COPA, SensorCfg.COLOR_MIN_INTERVAL_S);
        BeamSensor filaBeam = new BeamSensor(hardwareMap, Names.BEAM_FILA, SensorCfg.BEAM_ACTIVE_HIGH);
        Feeder feeder = new Feeder(fila, copa, elevator, upper, inCopa, filaBeam);
        Flywheel flywheel = new Flywheel(hardwareMap);

        waitForStart();
        long start = System.nanoTime();
        feeder.initHardware(start);
        boolean cycleMode = false;
        double lastFilaPower = Double.NaN;

        while (opModeIsActive()) {
            long now = System.nanoTime();
            if (gamepad1.yWasPressed()) {
                cycleMode = !cycleMode;
                feeder.stop();
                if (fila instanceof PwmControl) ((PwmControl) fila).setPwmDisable();
                lastFilaPower = Double.NaN;
            }

            if (cycleMode) {
                if (gamepad1.rightBumperWasPressed()) flywheel.setEnabled(!flywheel.isEnabled());
                flywheel.setTargetRpm(org.firstinspires.ftc.teamcode.Config.ShotCfg.IDLE_RPM);
                flywheel.update();
                if (gamepad1.aWasPressed()) feeder.requestShot();
                feeder.update(now, false, false, flywheel.consumeShotDip());
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

                double elev = 0;
                if (gamepad1.dpad_up) elev = FeederCfg.ELEVATOR_STAGE_POWER;
                boolean canReverse = feeder.getCopaMm() == 0 && !inCopa.hasBall();
                if (gamepad1.dpad_down && canReverse) elev = -FeederCfg.ELEVATOR_REVERSE_POWER;
                elevator.setPower(elev);
                if (gamepad1.dpad_down && !canReverse) telemetry.addLine("!! Reversa solo con la copa abajo y vacía");
                upper.read(now);
                inCopa.read(now);
            }

            telemetry.addData("Modo", cycleMode ? "CICLO (Y = manual)" : "MANUAL (Y = ciclo)");
            feeder.addTelemetry(telemetry);
            telemetry.addData("V3 arriba", upper.describe());
            telemetry.addData("V3 copa", inCopa.describe());
            telemetry.addData("BB3 fila", filaBeam.isActive() ? "cortado (hay pelota)" : "libre");
            flywheel.addTelemetry(telemetry);
            telemetry.update();
        }
        feeder.stop();
        flywheel.stop();
    }
}
