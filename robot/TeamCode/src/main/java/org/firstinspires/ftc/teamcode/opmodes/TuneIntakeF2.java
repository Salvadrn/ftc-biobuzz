package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Config.IntakeF2Cfg;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.SensorCfg;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeF2;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;
import org.firstinspires.ftc.teamcode.sensors.RevColorReader;

/**
 * Prueba el intake F2: estacionado con el sensor magnético, sentidos y sensores.
 * Necesita intakeF2, magF2, bbBoca, bbGarganta y colorGarganta (la cámara no).
 *
 * En INIT corre el estacionado de verdad (el mismo del TeleOp) y muestra el error.
 * Meta de la spec: ±2°; máximo ±5° para caber en R102. Compara la marca grabada del
 * disco con la de la placa +X; si no coinciden, ajusta INDEX_OFFSET_DEG.
 *
 * Después de Play:
 *   gatillo derecho   jalar (si saca las pelotas, cambia IntakeF2Cfg.REVERSED)
 *   gatillo izquierdo reversa
 *   A                 simular "lleno": va a +25° y frena
 */
@TeleOp(name = "Tune: Intake F2", group = "2 Calibracion")
public class TuneIntakeF2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, Names.INTAKE_MOTOR);
        BeamSensor magnet = new BeamSensor(hardwareMap, Names.MAGNET_F2, SensorCfg.MAGNET_ACTIVE_HIGH);
        BeamSensor mouth = new BeamSensor(hardwareMap, Names.BEAM_MOUTH, SensorCfg.BEAM_ACTIVE_HIGH);
        BeamSensor throat = new BeamSensor(hardwareMap, Names.BEAM_THROAT, SensorCfg.BEAM_ACTIVE_HIGH);
        BallSensor color = new BallSensor(new RevColorReader(hardwareMap, Names.COLOR_THROAT, SensorCfg.COLOR_GAIN),
                SensorCfg.THROAT, SensorCfg.COLOR_MIN_INTERVAL_S);
        IntakeF2 intake = new IntakeF2(motor, magnet, mouth, throat, color, null, Alliance.RED);

        while (opModeInInit()) {
            boolean done = intake.initStep(System.nanoTime());
            telemetry.addData("Estacionado", done ? (intake.isParked() ? "listo" : "FALLÓ") : "buscando el imán...");
            telemetry.addData("Error", "%.1f°  (meta ±2°, máximo ±5°)", intake.parkErrorDeg());
            telemetry.addData("Imán", magnet.isActive() ? "presente" : "no");
            telemetry.addData("Encoder", motor.getCurrentPosition());
            intake.addTelemetry(telemetry);
            telemetry.update();
        }

        boolean full = false;
        int totalIn = 0, totalOut = 0;
        while (opModeIsActive()) {
            long now = System.nanoTime();
            if (gamepad1.aWasPressed()) full = !full;
            Intake.Command cmd = Intake.Command.STOP;
            if (gamepad1.right_trigger > 0.3) cmd = Intake.Command.IN;
            if (gamepad1.left_trigger > 0.3) cmd = Intake.Command.OUT;
            intake.setCommand(cmd);
            intake.update(now, full);
            totalIn += intake.consumeEntered();
            totalOut += intake.consumeRejected();

            telemetry.addData("Lleno (simulado)", full ? "sí" : "no");
            telemetry.addData("Ángulo desde el índice", "%.1f°", intake.parkErrorDeg());
            telemetry.addData("Entraron / rechazadas", "%d / %d", totalIn, totalOut);
            telemetry.addData("V3 garganta", color.describe());
            telemetry.addData("Imán", magnet.isActive() ? "presente" : "no");
            intake.addTelemetry(telemetry);
            telemetry.addData("Cuentas por vuelta del rotor", "%.2f", IntakeF2Cfg.TICKS_PER_ROTOR_REV);
            telemetry.update();
        }
        intake.stop();
    }
}
